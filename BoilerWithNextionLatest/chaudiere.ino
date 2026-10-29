// ============================================================================
// CHAUDIÈRE : puissance, sorties, transitions IDLE/RUN/COOLDOWN, AUTO, START/STOP, forçages
// ============================================================================

// --------------------------------------------------------------------
// PUISSANCE
// --------------------------------------------------------------------
// Sélectionner un profil : charge ses valeurs dans les paramètres réglés
void setSelectedPowerLevel(PowerLevel lvl) {
  if ((uint8_t)lvl > 2) lvl = PWR_MED;
  selectedPowerLevel  = lvl;
  pelletCyclePeriodMs = kProfiles[(uint8_t)lvl].ppMs;
  pelletOnDurationMs  = kProfiles[(uint8_t)lvl].ponMs;
  runFanVoltageV      = kProfiles[(uint8_t)lvl].fvV;
}

// Paramètres réellement appliqués en RUN (profil MED pendant la phase de démarrage)
unsigned long activePelletPeriodMs() { return startupLowPowerActive ? kProfiles[PWR_MED].ppMs  : pelletCyclePeriodMs; }
unsigned long activePelletOnMs()     { return startupLowPowerActive ? kProfiles[PWR_MED].ponMs : pelletOnDurationMs; }
float         activeFanVoltageV()    { return startupLowPowerActive ? kProfiles[PWR_MED].fvV   : runFanVoltageV; }

// Bornes PP/PON (testées AVANT d'appliquer une nouvelle valeur)
bool powerParamsValid(long pp, long pon, bool printErr) {
  if (pp < 1000L || pp > 60000L) {
    if (printErr) Serial.println(F("ERR: PP range 1000..60000"));
    return false;
  }
  if (pon < 50L || pon > 5000L) {
    if (printErr) Serial.println(F("ERR: PON range 50..5000"));
    return false;
  }
  if (pon + (long)PELLET_MIN_OFF_MS > pp) {
    if (printErr) Serial.println(F("ERR: need PON <= PP - 500"));
    return false;
  }
  return true;
}

bool validatePowerParams(bool printErr) {
  if (!(runFanVoltageV >= 0.0f)) runFanVoltageV = 0.0f; // aussi NaN
  if (runFanVoltageV > 10.0f) runFanVoltageV = 10.0f;
  return powerParamsValid((long)pelletCyclePeriodMs, (long)pelletOnDurationMs, printErr);
}

// --------------------------------------------------------------------
// Sorties
// --------------------------------------------------------------------
uint8_t voltsToPwm(float volts) {
  if (volts < 0.0f)  volts = 0.0f;
  if (volts > 10.0f) volts = 10.0f;
  return (uint8_t)roundf((volts * 255.0f) / 10.0f);
}

void applyOutputs() {
  digitalWrite(PIN_R_PELLET, statePellet ? HIGH : LOW);
  digitalWrite(PIN_R_PUMP,   statePump   ? HIGH : LOW);
  digitalWrite(PIN_R_SSR,    stateCandle ? HIGH : LOW);
  analogWrite(PIN_AO_FAN_SPEED, fanPwm);
}

// --------------------------------------------------------------------
// Transitions d'état (IDLE / RUN / COOLDOWN)
// --------------------------------------------------------------------
// START : depuis IDLE, ou reprise depuis COOLDOWN (foyer chaud : la pompe reste ON)
void boilerEnterRun(unsigned long now, const __FlashStringHelper* origin) {
  bool resume = stopCooldownActive;

  boilerOn = true;
  stopCooldownActive = false;
  boilerStartMs = now;
  lastStartMs = now;
  startupLowPowerActive = true;
  startupLowPowerUntilMs = now + STARTUP_LOW_POWER_MS;
  pumpKeepOn = resume;

  stateCandle = true;
  fanVoltageV = activeFanVoltageV();
  fanPwm      = voltsToPwm(fanVoltageV);
  statePellet = false;
  statePump   = resume;

  applyOutputs();
  Serial.print(resume ? F("REPRISE depuis COOLDOWN (") : F("START chaudiere basique ("));
  Serial.print(origin);
  Serial.println(F(")."));
}

// Arrêt propre : COOLDOWN 10 min (vis + candelette OFF, pompe + ventilateur ON)
void boilerEnterCooldown(unsigned long now, const __FlashStringHelper* origin) {
  boilerOn = false;
  stopCooldownActive = true;
  stopStartMs = now;
  startupLowPowerActive = false;
  pumpKeepOn = false;

  statePellet = false;
  stateCandle = false;
  statePump   = true;
  fanVoltageV = runFanVoltageV;
  fanPwm      = voltsToPwm(fanVoltageV);

  applyOutputs();
  Serial.print(F("COOLDOWN 10 min ("));
  Serial.print(origin);
  Serial.println(F(")."));
}

// --------------------------------------------------------------------
// AUTO control (utilise cache RTC)
// --------------------------------------------------------------------
void autoControlService(unsigned long nowMs) {
  if (controlMode != MODE_AUTO) return;
  if (!g_rtcCachedOk) return;

  uint8_t dow = dowFromRTC();
  uint16_t mod = minuteOfDayFromRTC();
  bool demand = scheduleDemand(dow, mod);

  if (demand != lastDemandSeen) {
    lastDemandSeen = demand;
    lastDemandChangeMs = nowMs;
  }
  programDemandOn = demand;
  if (nowMs - lastDemandChangeMs < AUTO_DEMAND_STABLE_MS) return;

  // Fin de la plage : le maintien "Forcer Cooldown" est relâché
  if (autoHoldActive && !demand) {
    autoHoldActive = false;
    Serial.println(F("FORCE COOLDOWN: fin de plage -> maintien AUTO relache."));
  }

  if (alarmActive || autoHoldActive) return;
  if (nowMs - lastAutoCommandMs < AUTO_MIN_COMMAND_GAP_MS) return;

  if (demand && !boilerOn && !stopCooldownActive) {
    boilerEnterRun(nowMs, F("AUTO"));
    lastAutoCommandMs = nowMs;
  } else if (!demand && boilerOn) {
    boilerEnterCooldown(nowMs, F("AUTO"));
    lastAutoCommandMs = nowMs;
  }
}

// --------------------------------------------------------------------
// Chaudière basique MANUAL (START/STOP/SERVICE)
// --------------------------------------------------------------------
void boilerBasicStart(unsigned long now) {
  if (alarmActive) {
    Serial.print(F("START refuse: ALARM active (a0 ou i0). Cause="));
    Serial.println(alarmToStr(alarmCode));
    return;
  }
  if (controlMode == MODE_OFF) {
    Serial.println(F("START refuse: MODE OFF."));
    return;
  }
  if (boilerOn) {
    Serial.println(F("START refuse: deja RUN."));
    return;
  }
  if (autoHoldActive) {
    autoHoldActive = false;
    Serial.println(F("FORCE COOLDOWN: maintien AUTO relache (START)."));
  }
  boilerEnterRun(now, F("MANUAL"));
}

void boilerBasicStop(unsigned long now) {
  if (stopCooldownActive) {
    Serial.println(F("STOP: cooldown deja en cours."));
    return;
  }
  if (!boilerOn) {
    Serial.println(F("STOP: deja arretee."));
    return;
  }
  boilerEnterCooldown(now, F("STOP"));
}

void boilerBasicService(unsigned long now) {
  if (boilerOn) {
    if (startupLowPowerActive && (long)(now - startupLowPowerUntilMs) >= 0) {
      startupLowPowerActive = false;
      Serial.println(F("Startup: fin puissance MED -> retour profil choisi."));
    }

    unsigned long elapsed = now - boilerStartMs;

    fanVoltageV = activeFanVoltageV();
    fanPwm      = voltsToPwm(fanVoltageV);

    stateCandle = (elapsed < IGNITION_DURATION_MS);

    unsigned long period = activePelletPeriodMs();
    if (period < 1UL) period = 1UL;
    unsigned long phase  = elapsed % period;
    statePellet = (phase < activePelletOnMs());

    statePump = pumpKeepOn || (elapsed >= PUMP_START_DELAY_MS);

    applyOutputs();
  } else if (stopCooldownActive) {
    unsigned long sinceStop = now - stopStartMs;

    statePellet = false;
    stateCandle = false;
    statePump   = true;
    fanVoltageV = runFanVoltageV;
    fanPwm      = voltsToPwm(fanVoltageV);

    if (sinceStop >= STOP_COOLDOWN_MS) {
      stopCooldownActive = false;

      statePump   = false;
      fanVoltageV = 0.0f;
      fanPwm      = voltsToPwm(fanVoltageV);

      lastStartMs = 0;

      Serial.println(F("Cooldown termine: IDLE, sorties OFF."));
    }

    applyOutputs();
  }
}

// --------------------------------------------------------------------
// FORCE IDLE
// --------------------------------------------------------------------
void forceIdle(unsigned long now) {
  (void)now;
  boilerOn = false;
  stopCooldownActive = false;
  boilerStartMs = 0;
  stopStartMs = 0;
  lastStartMs = 0;

  clearAlarm();

  autoHoldActive = false;
  startupLowPowerActive = false;
  startupLowPowerUntilMs = 0;
  pumpKeepOn = false;

  statePellet = false;
  statePump   = false;
  stateCandle = false;
  fanVoltageV = 0.0f;
  fanPwm      = voltsToPwm(fanVoltageV);
  applyOutputs();

  lastPelletLogged = statePellet;
  pelletOnSinceMs = 0;
  pelletLongestOnMs = 0;

  Serial.println(F("FORCE IDLE: sorties OFF, cooldown annule, alarmes effacees."));
}

// --------------------------------------------------------------------
// FORCE COOLDOWN (manuel + auto)
// --------------------------------------------------------------------
void forceCooldownOn(unsigned long now) {
  autoHoldActive = true;
  if (stopCooldownActive) Serial.println(F("FORCE COOLDOWN: cooldown deja en cours."));
  else boilerEnterCooldown(now, F("FORCE"));
  Serial.println(F("FORCE COOLDOWN: puis IDLE ; AUTO bloque jusqu'a la fin de la plage (k0 pour relacher)."));
}

void forceCooldownOff() {
  autoHoldActive = false;
  Serial.println(F("FORCE COOLDOWN: maintien AUTO relache. (cooldown normal peut continuer si en cours)"));
}
