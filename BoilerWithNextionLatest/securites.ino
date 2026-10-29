// ============================================================================
// SÉCURITÉS : capteurs, MAX6675, failsafe vis, alarmes
// ============================================================================

// Début des défauts continus (0 = pas de défaut en cours)
static unsigned long tcNanSinceMs = 0;
static unsigned long noDepSinceMs = 0;

// Diagnostic anti-spam
static unsigned long lastDiagMs = 0;

// --------------------------------------------------------------------
// CONVERSIONS capteurs
// --------------------------------------------------------------------
float adcRawToVoltage10V(int raw) { return (raw * 10.0f) / 1023.0f; }

float waterPressureBarFromRaw(int raw) {
  float p = 0.01055f * (float)raw - 0.655f;
  if (p < 0.0f) p = 0.0f;
  return p;
}

static constexpr int   NTC_N = 8;
static const int   ntcRaw[NTC_N] = {272, 215, 190, 169, 143, 113,  87,  54};
static const float ntcDeg[NTC_N] = { 18,  32,  38,  44,  51,  61,  72,  89};

float waterTempC_fromRaw(int raw) {
  if (raw >= ntcRaw[0]) return ntcDeg[0];
  if (raw <= ntcRaw[NTC_N - 1]) return ntcDeg[NTC_N - 1];
  for (int i = 0; i < NTC_N - 1; i++) {
    if (raw <= ntcRaw[i] && raw >= ntcRaw[i + 1]) {
      float x0 = (float)ntcRaw[i], x1 = (float)ntcRaw[i + 1];
      float y0 = ntcDeg[i],      y1 = ntcDeg[i + 1];
      return y0 + ((float)raw - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  return NAN;
}

static constexpr int DEP_RAW0 = 272;
int depressionDeltaRaw(int raw) { return raw - DEP_RAW0; }
float depressionDeltaVolts(int raw) { return adcRawToVoltage10V(raw) - adcRawToVoltage10V(DEP_RAW0); }

// --------------------------------------------------------------------
// MAX6675
// --------------------------------------------------------------------
uint16_t max6675_readRaw() {
  uint16_t v = 0;
  digitalWrite(PIN_MAX6675_CS, LOW);
  delayMicroseconds(1);
  for (int i = 15; i >= 0; i--) {
    digitalWrite(PIN_MAX6675_SCK, HIGH);
    delayMicroseconds(1);
    v <<= 1;
    if (digitalRead(PIN_MAX6675_SO)) v |= 1;
    digitalWrite(PIN_MAX6675_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_MAX6675_CS, HIGH);
  return v;
}

float max6675_readCelsius() {
  uint16_t raw = max6675_readRaw();
  if (raw & 0x0004) return NAN;
  raw >>= 3;
  return raw * 0.25f;
}

// --------------------------------------------------------------------
// Lectures capteurs
// --------------------------------------------------------------------
inline bool acPresentFromRaw(int raw) { return (raw < AC_RAW_THRESHOLD); }
inline bool acAbsentFromRaw(int raw)  { return (raw >= AC_RAW_THRESHOLD); }

void readAllSensorsOnce() {
  rawNoDep     = analogRead(PIN_AI_NO_DEPRESSION);
  rawPelletsOH = analogRead(PIN_AI_PELLETS_OVERHEAT);
  rawWaterOH   = analogRead(PIN_AI_WATER_OVERHEAT);
  rawNTC       = analogRead(PIN_AI_WATER_NTC);
  rawPressure  = analogRead(PIN_AI_WATER_PRESSURE);
  rawDep       = analogRead(PIN_AI_DEPRESSION);
  lastTempFumeesC = max6675_readCelsius();
}

// --------------------------------------------------------------------
// Patch: log + failsafe vis sans fin
// --------------------------------------------------------------------
void pelletDebugAndFailsafe(unsigned long nowMs) {
  if (statePellet != lastPelletLogged) {
    lastPelletLogged = statePellet;

    Serial.print(F("[EVT] pellet -> "));
    Serial.print(statePellet ? F("ON") : F("OFF"));
    Serial.print(F("  mode="));
    Serial.print(modeToStr(controlMode));
    Serial.print(F(" demand="));
    Serial.print(programDemandOn ? F("ON") : F("OFF"));
    Serial.print(F(" state="));
    if (boilerOn) Serial.print(F("RUN"));
    else if (stopCooldownActive) Serial.print(F("COOLDOWN"));
    else Serial.print(F("IDLE"));
    Serial.print(F("  ms="));
    Serial.println(nowMs);

    if (statePellet) pelletOnSinceMs = nowMs;
  }

  if (statePellet) {
    unsigned long onMs = nowMs - pelletOnSinceMs;
    if (onMs > pelletLongestOnMs) pelletLongestOnMs = onMs;

    // En RUN la vis ne doit jamais dépasser PON : au-delà c'est un bug => ALARM (cooldown),
    // sinon boilerBasicService() la rallumerait aussitôt (relais qui claque).
    unsigned long maxOnMs = boilerOn ? activePelletOnMs() + PELLET_RUN_ON_MARGIN_MS
                                     : PELLET_MAX_CONTINUOUS_ON_MS;
    if (onMs > maxOnMs) {
      Serial.print(F("!!! FAILSAFE: pellet ON too long ("));
      Serial.print(onMs);
      Serial.println(F(" ms) -> FORCE OFF"));

      statePellet = false;
      applyOutputs();
      if (boilerOn) {
        triggerAlarmAndCooldown(ALM_PELLET_FAILSAFE, nowMs);
        if (boilerOn) boilerEnterCooldown(nowMs, F("FAILSAFE")); // alarme déjà active
      }
    }
  }
}

// Candelette allumée à la main (c1) : coupée au bout de MANUAL_CANDLE_MAX_MS
void manualCandleGuard(unsigned long now) {
  if (boilerOn || stopCooldownActive || !stateCandle) return;
  if (now - candleManualSinceMs < MANUAL_CANDLE_MAX_MS) return;
  stateCandle = false;
  applyOutputs();
  Serial.println(F("Candelette manuelle: coupee apres 10 min (securite)."));
}

// --------------------------------------------------------------------
// Alarme / diagnostic
// --------------------------------------------------------------------
void clearAlarm() {
  alarmActive = false;
  alarmCode = ALM_NONE;
}

void triggerAlarmAndCooldown(AlarmCode code, unsigned long now) {
  if (alarmActive) return;
  alarmActive = true;
  alarmCode = code;

  Serial.print(F("!!! ALARM: "));
  Serial.println(alarmToStr(code));

  if (boilerOn) boilerEnterCooldown(now, F("ALARM"));
}

void diagPrint(unsigned long now, const __FlashStringHelper* label, const char* detail) {
  if (now - lastDiagMs < DIAG_PERIOD_MS) return;
  lastDiagMs = now;
  Serial.print(F("[DIAG] "));
  Serial.print(label);
  if (detail && detail[0]) { Serial.print(F(" : ")); Serial.print(detail); }
  Serial.println();
}

// Défauts instantanés : thermostats de sécurité (AC absent), sonde eau, pression
AlarmCode currentInputFault() {
  bool pelletsFault = acAbsentFromRaw(rawPelletsOH);
  bool waterFault   = acAbsentFromRaw(rawWaterOH);
  if (pelletsFault && waterFault) return ALM_PELLETS_WATER_OH;
  if (pelletsFault)               return ALM_PELLETS_OH;
  if (waterFault)                 return ALM_WATER_OH;
  if (rawNTC > 0 && rawNTC < RAW_WATER_SENSOR_OVERHEAT) return ALM_WATER_SENSOR_OH;
  if (rawPressure > RAW_PRESSURE_TOO_HIGH) return ALM_PRESSURE_HIGH;
  if (rawPressure < RAW_PRESSURE_TOO_LOW)  return ALM_PRESSURE_LOW;
  return ALM_NONE;
}

void checkSafeties(unsigned long now) {
  bool tcNan = isnan(lastTempFumeesC);
  if (!tcNan) tcNanSinceMs = 0;
  else if (tcNanSinceMs == 0) tcNanSinceMs = now;

  bool noDep = acAbsentFromRaw(rawNoDep);
  if (!noDep) noDepSinceMs = 0;
  else if (noDepSinceMs == 0) noDepSinceMs = now;

  if (!boilerOn && !stopCooldownActive) {
    AlarmCode fault = currentInputFault();
    if (fault != ALM_NONE) diagPrint(now, alarmToStr(fault), "");
    else if (tcNan) diagPrint(now, F("THERMOCOUPLE"), "NaN (circuit ouvert ?)");
    else if (noDep && fanPwm > 0) diagPrint(now, F("NO DEPRESSION"), "ventilateur ON");
    return;
  }

  if (alarmActive) return;

  AlarmCode fault = currentInputFault();
  if (fault != ALM_NONE) {
    triggerAlarmAndCooldown(fault, now);
    return;
  }

  if (!boilerOn || lastStartMs == 0) return;
  unsigned long sinceStart = now - lastStartMs;

  // Thermocouple illisible : sans lui, la détection NO FIRE ne fonctionne plus
  if (tcNan && min(now - tcNanSinceMs, sinceStart) >= TC_FAULT_ALARM_MS) {
    triggerAlarmAndCooldown(ALM_THERMOCOUPLE, now);
    return;
  }

  if (noDep && sinceStart >= NO_DEP_GRACE_MS) {
#if ENABLE_NO_DEP_ALARM
    if (min(now - noDepSinceMs, sinceStart - NO_DEP_GRACE_MS) >= NO_DEP_ALARM_MS) {
      triggerAlarmAndCooldown(ALM_NO_DEPRESSION, now);
      return;
    }
#else
    diagPrint(now, F("NO DEPRESSION (IGNORED)"), "running");
#endif
  }

  if (sinceStart >= NO_FIRE_DELAY_MS && !tcNan && lastTempFumeesC < FLUE_NO_FIRE_MIN_C) {
    triggerAlarmAndCooldown(ALM_NO_FIRE, now);
    return;
  }
}
