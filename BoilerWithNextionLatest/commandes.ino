// ============================================================================
// COMMANDES : journal d'événements, affichages série, aide, commandes série
// ============================================================================

// --------------------------------------------------------------------
// Etat du journal d'événements
// --------------------------------------------------------------------
bool lastDemandLogged = false;
bool lastBoilerOnLogged = false;
bool lastCooldownLogged = false;
bool lastRtcOkLogged = true;

// --------------------------------------------------------------------
// DEBUG AUTO event logging (1 Hz) - utilise cache RTC
// --------------------------------------------------------------------
void logAutoEventIfChanged(unsigned long nowMs) {
  (void)nowMs;

  bool rtcOk = g_rtcCachedOk;

  uint8_t dow = 0;
  uint16_t mod = 0;
  if (rtcOk) {
    dow = dowFromRTC();
    mod = minuteOfDayFromRTC();
  }

  if (rtcOk != lastRtcOkLogged) {
    lastRtcOkLogged = rtcOk;
    Serial.print(F("[EVT] rtcOk -> "));
    Serial.println(rtcOk ? F("1") : F("0"));
    if (rtcOk) {
      Serial.print(F("[EVT] DOW=")); Serial.print(dow);
      Serial.print(F(" MOD=")); Serial.println(mod);
    }
  }

  if (programDemandOn != lastDemandLogged) {
    lastDemandLogged = programDemandOn;
    Serial.print(F("[EVT] demand -> "));
    Serial.print(programDemandOn ? F("ON") : F("OFF"));
    Serial.print(F("  rtcOk=")); Serial.print(rtcOk ? 1 : 0);
    if (rtcOk) {
      Serial.print(F(" DOW=")); Serial.print(dow);
      Serial.print(F(" MOD=")); Serial.print(mod);
    }
    if (autoHoldActive) Serial.print(F("  AUTO_HOLD=1"));
    Serial.println();
  }

  if (boilerOn != lastBoilerOnLogged || stopCooldownActive != lastCooldownLogged) {
    lastBoilerOnLogged = boilerOn;
    lastCooldownLogged = stopCooldownActive;

    Serial.print(F("[EVT] state -> "));
    if (boilerOn) Serial.print(F("RUN"));
    else if (stopCooldownActive) Serial.print(F("COOLDOWN"));
    else Serial.print(F("IDLE"));

    Serial.print(F("  rtcOk=")); Serial.print(rtcOk ? 1 : 0);
    if (rtcOk) {
      Serial.print(F(" DOW=")); Serial.print(dow);
      Serial.print(F(" MOD=")); Serial.print(mod);
    }
    if (autoHoldActive) Serial.print(F("  AUTO_HOLD=1"));
    Serial.println();

    Serial.print(F("[EVT] since lastAutoCommandMs = "));
    Serial.print((unsigned long)(millis() - lastAutoCommandMs));
    Serial.println(F(" ms"));
  }
}

// --------------------------------------------------------------------
// Affichages / help
// --------------------------------------------------------------------
static void print2d(int v) { if (v < 10) Serial.print('0'); Serial.print(v); }

void printTimeLine() {
  if (!rtcReadOk()) {
    Serial.println(F("RTC: read error (check switches RTC SCL/SDA + SETTIME)."));
    return;
  }
  Serial.print(F("RTC: "));
  Serial.print(RTC.getYear()); Serial.print('-');
  print2d(RTC.getMonth()); Serial.print('-');
  print2d(RTC.getMonthDay()); Serial.print(' ');
  print2d(RTC.getHour()); Serial.print(':');
  print2d(RTC.getMinute()); Serial.print(':');
  print2d(RTC.getSecond());
  Serial.print(F("  DOW=")); Serial.println(dowFromRTC());
}

const __FlashStringHelper* modeToStr(ControlMode m) {
  switch (m) {
    case MODE_OFF:    return F("OFF");
    case MODE_MANUAL: return F("MANUAL");
    case MODE_AUTO:   return F("AUTO");
    default:          return F("?");
  }
}

const __FlashStringHelper* pwrToStr(PowerLevel p) {
  switch (p) {
    case PWR_LOW:  return F("LOW");
    case PWR_MED:  return F("MED");
    case PWR_HIGH: return F("HIGH");
    default:       return F("?");
  }
}

// Libellés de l'écran (mêmes textes que select0)
const __FlashStringHelper* pwrToStrFr(PowerLevel p) {
  switch (p) {
    case PWR_LOW:  return F("MIN.");
    case PWR_MED:  return F("MOY.");
    case PWR_HIGH: return F("MAX.");
    default:       return F("?");
  }
}

const __FlashStringHelper* alarmToStr(AlarmCode a) {
  switch (a) {
    case ALM_PELLETS_WATER_OH: return F("PELLETS + WATER OVERHEAT");
    case ALM_PELLETS_OH:       return F("PELLETS OVERHEAT");
    case ALM_WATER_OH:         return F("WATER OVERHEAT");
    case ALM_WATER_SENSOR_OH:  return F("WATER SENSOR OVERHEAT");
    case ALM_PRESSURE_HIGH:    return F("WATER PRESSURE TOO HIGH");
    case ALM_PRESSURE_LOW:     return F("WATER PRESSURE TOO LOW");
    case ALM_NO_FIRE:          return F("ALARM NO FIRE");
    case ALM_NO_DEPRESSION:    return F("NO DEPRESSION");
    case ALM_THERMOCOUPLE:     return F("THERMOCOUPLE FAULT");
    case ALM_PELLET_FAILSAFE:  return F("PELLET FAILSAFE (ON too long)");
    default:                   return F("-");
  }
}

// Libellés de l'écran (police iso-8859-1 : \xE9 = é, \xEA = ê)
const __FlashStringHelper* alarmToStrFr(AlarmCode a) {
  switch (a) {
    case ALM_PELLETS_WATER_OH: return F("Surchauffe pellets + eau");
    case ALM_PELLETS_OH:       return F("Surchauffe pellets");
    case ALM_WATER_OH:         return F("Surchauffe eau (thermostat)");
    case ALM_WATER_SENSOR_OH:  return F("Surchauffe eau (sonde)");
    case ALM_PRESSURE_HIGH:    return F("Pression eau trop haute");
    case ALM_PRESSURE_LOW:     return F("Pression eau trop basse");
    case ALM_NO_FIRE:          return F("Pas de flamme (pellets ?)");
    case ALM_NO_DEPRESSION:    return F("Pas de d\xE9" "pression (tirage)");
    case ALM_THERMOCOUPLE:     return F("Sonde fum\xE9" "es illisible");
    case ALM_PELLET_FAILSAFE:  return F("Vis ON trop longtemps");
    default:                   return F("-");
  }
}

void printParams() {
  Serial.println(F("=== PARAMS (power) ==="));
  Serial.print(F("LEVEL = ")); Serial.print(pwrToStr(selectedPowerLevel));
  if (boilerOn && startupLowPowerActive) Serial.print(F("  (demarrage en cours: MED actif)"));
  Serial.println();
  Serial.print(F("PP  = ")); Serial.print(pelletCyclePeriodMs); Serial.println(F(" ms"));
  Serial.print(F("PON = ")); Serial.print(pelletOnDurationMs);  Serial.println(F(" ms"));
  Serial.print(F("FV  = ")); Serial.print(runFanVoltageV, 1);   Serial.println(F(" V"));
  Serial.println(F("======================"));
}

void printSchedule() {
  static const char* dn[7] = {"Lun","Mar","Mer","Jeu","Ven","Sam","Dim"};
  for (int d = 0; d < 7; d++) {
    DaySchedule &ds = weekSched[d];
    Serial.print(dn[d]); Serial.print(F(" : "));
    Serial.print(ds.start1/60); Serial.print(':'); print2d(ds.start1%60);
    Serial.print(F("-"));
    Serial.print(ds.end1/60); Serial.print(':'); print2d(ds.end1%60);
    Serial.print(F("   "));
    Serial.print(ds.start2/60); Serial.print(':'); print2d(ds.start2%60);
    Serial.print(F("-"));
    Serial.print(ds.end2/60); Serial.print(':'); print2d(ds.end2%60);
    Serial.println();
  }
}

void printSnapshot() {
  bool acNoDepPresent     = acPresentFromRaw(rawNoDep);
  bool acPelletsOHPresent = acPresentFromRaw(rawPelletsOH);
  bool acWaterOHPresent   = acPresentFromRaw(rawWaterOH);

  float vNTC      = adcRawToVoltage10V(rawNTC);
  float vPressure = adcRawToVoltage10V(rawPressure);
  float vDep      = adcRawToVoltage10V(rawDep);

  float tWaterC   = waterTempC_fromRaw(rawNTC);
  float pWaterBar = waterPressureBarFromRaw(rawPressure);

  int   dep_dRaw  = depressionDeltaRaw(rawDep);
  float dep_dV    = depressionDeltaVolts(rawDep);
  float dep_dMbar = 10.0f * dep_dV;

  Serial.println(F("===== SNAPSHOT M-Duino ====="));
  printTimeLine();

  Serial.print(F("MODE: "));
  Serial.print(modeToStr(controlMode));
  Serial.print(F("  programDemand="));
  Serial.print(programDemandOn ? F("ON") : F("OFF"));
  Serial.print(F("  autoHold="));
  Serial.println(autoHoldActive ? F("ON") : F("OFF"));

  Serial.print(F("Power: LEVEL=")); Serial.print(pwrToStr(selectedPowerLevel));
  if (boilerOn && startupLowPowerActive) Serial.print(F(" (startup MED)"));
  Serial.print(F(" PP=")); Serial.print(pelletCyclePeriodMs);
  Serial.print(F(" PON="));      Serial.print(pelletOnDurationMs);
  Serial.print(F(" FV="));       Serial.print(runFanVoltageV, 1);
  Serial.println();

  Serial.print(F("Pellet ON longest   : "));
  Serial.print(pelletLongestOnMs);
  Serial.println(F(" ms (since boot/forceIdle)"));

  Serial.print(F("I0.7  No depression : RAW=")); Serial.print(rawNoDep);
  Serial.print(F("  -> ")); Serial.println(acNoDepPresent ? F("AC PRESENT") : F("AC ABSENT"));

  Serial.print(F("I0.8  Pellets OH    : RAW=")); Serial.print(rawPelletsOH);
  Serial.print(F("  -> ")); Serial.println(acPelletsOHPresent ? F("AC PRESENT") : F("AC ABSENT"));

  Serial.print(F("I0.9  Water OH      : RAW=")); Serial.print(rawWaterOH);
  Serial.print(F("  -> ")); Serial.println(acWaterOHPresent ? F("AC PRESENT") : F("AC ABSENT"));

  Serial.print(F("I0.10 NTC eau       : RAW=")); Serial.print(rawNTC);
  Serial.print(F("  V=")); Serial.print(vNTC, 3);
  Serial.print(F("  T="));
  if (isnan(tWaterC)) Serial.println(F("NaN"));
  else { Serial.print(tWaterC, 1); Serial.println(F(" °C")); }

  Serial.print(F("I0.11 Pression eau  : RAW=")); Serial.print(rawPressure);
  Serial.print(F("  V=")); Serial.print(vPressure, 3);
  Serial.print(F("  P=")); Serial.print(pWaterBar, 2);
  Serial.println(F(" bar"));

  Serial.print(F("I0.12 Depression    : RAW=")); Serial.print(rawDep);
  Serial.print(F("  V=")); Serial.print(vDep, 3);
  Serial.print(F("  dRAW=")); Serial.print(dep_dRaw);
  Serial.print(F("  dV=")); Serial.print(dep_dV, 3);
  Serial.print(F("  ~dP=")); Serial.print(dep_dMbar, 1);
  Serial.println(F(" mbar (indicatif)"));

  Serial.print(F("MAX6675 fumées      : "));
  if (isnan(lastTempFumeesC)) Serial.println(F("NaN"));
  else { Serial.print(lastTempFumeesC, 2); Serial.println(F(" °C")); }

  Serial.print(F("R1.1 Vis sans fin   : ")); Serial.println(statePellet ? F("ON") : F("OFF"));
  Serial.print(F("R1.2 Pompe          : ")); Serial.println(statePump ? F("ON") : F("OFF"));
  Serial.print(F("R1.4 Candelette     : ")); Serial.println(stateCandle ? F("ON") : F("OFF"));
  Serial.print(F("A1.0 Ventilateur    : ")); Serial.print(fanVoltageV, 1);
  Serial.print(F(" V (PWM=")); Serial.print(fanPwm); Serial.println(F(")"));

  Serial.print(F("Etat chaudiere      : "));
  if (boilerOn) Serial.println(F("RUN"));
  else if (stopCooldownActive) Serial.println(F("COOLDOWN"));
  else Serial.println(F("IDLE"));

  if (alarmActive) {
    Serial.print(F("ALARM              : "));
    Serial.println(alarmToStr(alarmCode));
  }

  Serial.println(F("============================"));
}

void printHelp() {
  Serial.println(F("=== Commandes serie M-Duino ==="));
  Serial.println(F("h         : aide"));
  Serial.println(F("r         : snapshot immediat"));
  Serial.println(F("s0 / s1   : auto snapshot OFF/ON"));
  Serial.println(F("w0 / w1   : vis sans fin OFF/ON [manuel IDLE, 3 s max]"));
  Serial.println(F("p0 / p1   : pompe OFF/ON [manuel IDLE]"));
  Serial.println(F("c0 / c1   : candelette OFF/ON [manuel IDLE, 10 min max]"));
  Serial.println(F("fN        : ventilateur N volts (0..10) [manuel IDLE] ex f5.5"));
  Serial.println(F("b1        : START (MANUAL ; pendant un COOLDOWN = reprise, pompe maintenue)"));
  Serial.println(F("b0        : STOP -> cooldown 10 min (MANUAL)"));
  Serial.println(F("a0        : acquitter/effacer ALARM"));
  Serial.println(F("i0        : FORCER IDLE (sorties OFF, annule cooldown, efface ALARM)"));
  Serial.println(F("k1 / k0   : FORCER COOLDOWN 10 min + bloquer l'AUTO jusqu'a fin de plage / relacher"));
  Serial.println(F("COOLDOWN ON|OFF : idem k1/k0"));
  Serial.println(F("--- RTC / Planning ---"));
  Serial.println(F("TIME"));
  Serial.println(F("SETTIME YYYY-MM-DD HH:MM:SS"));
  Serial.println(F("MODE OFF|MANUAL|AUTO"));
  Serial.println(F("LISTSCHED"));
  Serial.println(F("SETDAY D HH:MM HH:MM HH:MM HH:MM"));
  Serial.println(F("DEFAULTSCHED"));
  Serial.println(F("SAVE"));
  Serial.println(F("--- Puissance ---"));
  Serial.println(F("POWER LOW|MED|HIGH   (charge le profil + sauvegarde)"));
  Serial.println(F("PARAMS"));
  Serial.println(F("PP <ms>   (1000..60000)"));
  Serial.println(F("PON <ms>  (50..5000, PON <= PP-500)"));
  Serial.println(F("FV <V>    (0..10)"));
  Serial.println(F("DEFAULTPARAMS"));
  Serial.println(F("================================"));
}

// --------------------------------------------------------------------
// Commandes série (tampons char : pas de String, donc pas de fragmentation du tas)
// --------------------------------------------------------------------
bool manualAllowed() { return (!boilerOn && !stopCooldownActive && !alarmActive); }

// Préfixe / égalité sans tenir compte de la casse (mots-clés en flash)
bool startsWithNoCase(const char* s, const __FlashStringHelper* prefix) {
  PGM_P p = reinterpret_cast<PGM_P>(prefix);
  return strncasecmp_P(s, p, strlen_P(p)) == 0;
}

bool equalsNoCase(const char* s, const __FlashStringHelper* word) {
  return strcasecmp_P(s, reinterpret_cast<PGM_P>(word)) == 0;
}

// Retire les blancs en début et fin de chaîne (en place)
char* trimInPlace(char* s) {
  while (isspace((unsigned char)*s)) s++;
  char* e = s + strlen(s);
  while (e > s && isspace((unsigned char)e[-1])) *--e = '\0';
  return s;
}

// Argument après le premier espace, ou nullptr
static char* commandArg(char* line) {
  char* p = strchr(line, ' ');
  if (!p) return nullptr;
  while (*p == ' ') p++;
  return *p ? p : nullptr;
}

// Lit un entier positif et avance le curseur
static bool readUInt(const char*& p, int& v) {
  if (!isdigit((unsigned char)*p)) return false;
  long acc = 0;
  while (isdigit((unsigned char)*p)) {
    acc = acc * 10 + (*p++ - '0');
    if (acc > 9999) return false;
  }
  v = (int)acc;
  return true;
}

static bool readChar(const char*& p, char c) {
  if (*p != c) return false;
  p++;
  return true;
}

static int parseHHMM_toMin(const char* hhmm) {
  const char* p = hhmm;
  int hh, mm;
  if (!readUInt(p, hh) || !readChar(p, ':') || !readUInt(p, mm) || *p) return -1;
  if (hh > 23 || mm > 59) return -1;
  return hh * 60 + mm;
}

// Commande interne écrite en flash, ex. handleCommand(F("MODE MANUAL"))
void handleCommand(const __FlashStringHelper* cmd) {
  char buf[SERIAL_LINE_MAX + 1];
  strncpy_P(buf, reinterpret_cast<PGM_P>(cmd), SERIAL_LINE_MAX);
  buf[SERIAL_LINE_MAX] = '\0';
  handleCommand(buf);
}

void handleCommand(const char* lineIn) {
  char buf[SERIAL_LINE_MAX + 1];
  strncpy(buf, lineIn, SERIAL_LINE_MAX);
  buf[SERIAL_LINE_MAX] = '\0';
  char* line = trimInPlace(buf);
  if (!*line) return;

  unsigned long now = millis();

  if (startsWithNoCase(line, F("TIME"))) { printTimeLine(); return; }

  if (startsWithNoCase(line, F("COOLDOWN"))) {
    char* v = commandArg(line);
    if (v && (equalsNoCase(v, F("ON")) || equalsNoCase(v, F("1")))) forceCooldownOn(now);
    else if (v && (equalsNoCase(v, F("OFF")) || equalsNoCase(v, F("0")))) forceCooldownOff();
    else Serial.println(F("ERR: COOLDOWN ON|OFF"));
    return;
  }

  if (startsWithNoCase(line, F("POWER"))) {
    char* v = commandArg(line);
    PowerLevel lvl;
    if (v && equalsNoCase(v, F("LOW"))) lvl = PWR_LOW;
    else if (v && (equalsNoCase(v, F("MED")) || equalsNoCase(v, F("MEDIUM")))) lvl = PWR_MED;
    else if (v && equalsNoCase(v, F("HIGH"))) lvl = PWR_HIGH;
    else { Serial.println(F("ERR: POWER LOW|MED|HIGH")); return; }

    setSelectedPowerLevel(lvl);
    eepromSaveAll();

    Serial.print(F("OK: POWER -> "));
    Serial.print(pwrToStr(selectedPowerLevel));
    if (boilerOn && startupLowPowerActive) Serial.print(F(" (applique a la fin du demarrage MED)"));
    Serial.println();
    printParams();
    return;
  }

  if (startsWithNoCase(line, F("SETTIME"))) {
    // SETTIME YYYY-MM-DD HH:MM:SS
    const char* p = commandArg(line);
    int Y, M, DD, hh, mm, ss;
    bool ok = p && readUInt(p, Y) && readChar(p, '-') && readUInt(p, M) && readChar(p, '-') && readUInt(p, DD);
    if (ok) {
      while (*p == ' ') p++;
      ok = readUInt(p, hh) && readChar(p, ':') && readUInt(p, mm) && readChar(p, ':') && readUInt(p, ss) && !*p;
    }
    if (!ok) { Serial.println(F("ERR: SETTIME format.")); return; }

    if (Y < 2000 || Y > 2099 || M < 1 || M > 12 || DD < 1 || DD > daysInMonth(Y, M) ||
        hh > 23 || mm > 59 || ss > 59) {
      Serial.println(F("ERR: SETTIME date/heure invalide."));
      return;
    }

    RTC.setYear(Y);
    RTC.setMonth(M);
    RTC.setMonthDay(DD);
    RTC.setHour(hh);
    RTC.setMinute(mm);
    RTC.setSecond(ss);

    if (!RTC.write()) Serial.println(F("ERR: RTC.write failed (check switches RTC SCL/SDA)."));
    else { Serial.println(F("OK: RTC updated.")); printTimeLine(); }
    return;
  }

  if (startsWithNoCase(line, F("MODE"))) {
    char* m = commandArg(line);
    ControlMode newMode;
    if (m && equalsNoCase(m, F("OFF"))) newMode = MODE_OFF;
    else if (m && equalsNoCase(m, F("MANUAL"))) newMode = MODE_MANUAL;
    else if (m && equalsNoCase(m, F("AUTO"))) newMode = MODE_AUTO;
    else { Serial.println(F("ERR: MODE OFF|MANUAL|AUTO")); return; }

    if (newMode != controlMode && autoHoldActive) {
      autoHoldActive = false;
      Serial.println(F("FORCE COOLDOWN: maintien AUTO relache (MODE)."));
    }
    controlMode = newMode;
    eepromSaveAll();
    Serial.print(F("MODE -> ")); Serial.println(modeToStr(controlMode));

    if (controlMode == MODE_OFF && boilerOn) boilerEnterCooldown(now, F("MODE OFF"));
    return;
  }

  if (startsWithNoCase(line, F("LISTSCHED"))) { printSchedule(); return; }

  if (startsWithNoCase(line, F("DEFAULTSCHED"))) {
    setDefaultSchedule();
    eepromSaveAll();
    Serial.println(F("OK: default schedule applied + saved."));
    printSchedule();
    return;
  }

  if (startsWithNoCase(line, F("SAVE"))) {
    if (!validatePowerParams(true)) return;
    eepromSaveAll();
    Serial.println(F("OK: EEPROM saved (mode+sched+params)."));
    return;
  }

  if (startsWithNoCase(line, F("SETDAY"))) {
    // SETDAY D HH:MM HH:MM HH:MM HH:MM
    char* rest = commandArg(line);
    char* tok[5];
    int ti = 0;
    for (char* t = rest ? strtok(rest, " ") : nullptr; t && ti < 5; t = strtok(nullptr, " ")) tok[ti++] = t;
    if (ti != 5) { Serial.println(F("ERR: SETDAY D HH:MM HH:MM HH:MM HH:MM")); return; }

    int D = atoi(tok[0]);
    if (D < 1 || D > 7) { Serial.println(F("ERR: D must be 1..7 (Mon..Sun).")); return; }

    int s1 = parseHHMM_toMin(tok[1]);
    int e1 = parseHHMM_toMin(tok[2]);
    int s2 = parseHHMM_toMin(tok[3]);
    int e2 = parseHHMM_toMin(tok[4]);
    if (s1 < 0 || e1 < 0 || s2 < 0 || e2 < 0) { Serial.println(F("ERR: bad HH:MM.")); return; }

    weekSched[D - 1].start1 = (uint16_t)s1;
    weekSched[D - 1].end1   = (uint16_t)e1;
    weekSched[D - 1].start2 = (uint16_t)s2;
    weekSched[D - 1].end2   = (uint16_t)e2;

    eepromSaveAll();
    Serial.println(F("OK: day schedule updated + saved."));
    return;
  }

  if (startsWithNoCase(line, F("PARAMS"))) { printParams(); return; }

  if (startsWithNoCase(line, F("DEFAULTPARAMS"))) {
    setDefaultParams();
    eepromSaveAll();
    Serial.println(F("OK: default power params applied + saved."));
    printParams();
    return;
  }

  if (startsWithNoCase(line, F("PP"))) {
    char* a = commandArg(line);
    if (!a) { Serial.println(F("ERR: PP <ms>")); return; }
    long v = atol(a);
    if (!powerParamsValid(v, (long)pelletOnDurationMs, true)) return;
    pelletCyclePeriodMs = (unsigned long)v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  if (startsWithNoCase(line, F("PON"))) {
    char* a = commandArg(line);
    if (!a) { Serial.println(F("ERR: PON <ms>")); return; }
    long v = atol(a);
    if (!powerParamsValid((long)pelletCyclePeriodMs, v, true)) return;
    pelletOnDurationMs = (unsigned long)v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  if (startsWithNoCase(line, F("FV"))) {
    char* a = commandArg(line);
    if (!a) { Serial.println(F("ERR: FV <volts>")); return; }
    float v = (float)atof(a);
    if (v < 0.0f) v = 0.0f;
    if (v > 10.0f) v = 10.0f;
    runFanVoltageV = v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  char cmd = line[0];
  switch (cmd) {
    case 'h': case 'H': printHelp(); break;

    case 'r': case 'R':
      readAllSensorsOnce();
      checkSafeties(now);
      printSnapshot();
      break;

    case 's': case 'S':
      if (line[1]) {
        char v = line[1];
        if (v == '0') autoSnapshotEnabled = false;
        else if (v == '1') autoSnapshotEnabled = true;
        Serial.print(F("Auto snapshot -> "));
        Serial.println(autoSnapshotEnabled ? F("ON") : F("OFF"));
      }
      break;

    case 'k': case 'K':
      if (line[1] == '1') forceCooldownOn(now);
      else if (line[1] == '0') forceCooldownOff();
      break;

    case 'a': case 'A':
      if (line[1] == '0') {
        clearAlarm();
        Serial.println(F("ALARM acquittee."));
      }
      break;

    case 'i': case 'I':
      if (line[1] == '0') forceIdle(now);
      break;

    case 'w': case 'W':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line[1]) {
        statePellet = (line[1] == '1');
        applyOutputs();
        Serial.print(F("Vis sans fin -> ")); Serial.println(statePellet ? F("ON") : F("OFF"));
      }
      break;

    case 'p': case 'P':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line[1]) {
        statePump = (line[1] == '1');
        applyOutputs();
        Serial.print(F("Pompe -> ")); Serial.println(statePump ? F("ON") : F("OFF"));
      }
      break;

    case 'c': case 'C':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line[1]) {
        stateCandle = (line[1] == '1');
        if (stateCandle) candleManualSinceMs = now;
        applyOutputs();
        Serial.print(F("Candelette -> ")); Serial.println(stateCandle ? F("ON (10 min max)") : F("OFF"));
      }
      break;

    case 'f': case 'F':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line[1]) {
        float v = (float)atof(line + 1);
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        fanVoltageV = v;
        fanPwm      = voltsToPwm(fanVoltageV);
        applyOutputs();
        Serial.print(F("Ventilateur(manuel) -> ")); Serial.print(fanVoltageV, 1); Serial.println(F(" V"));
      }
      break;

    case 'b': case 'B':
      if (controlMode == MODE_AUTO) {
        Serial.println(F("b0/b1 refuses (MODE=AUTO). Utiliser MODE MANUAL."));
        break;
      }
      if (line[1] == '1') boilerBasicStart(now);
      else if (line[1] == '0') boilerBasicStop(now);
      break;

    default:
      Serial.print(F("Commande inconnue: "));
      Serial.println(line);
      printHelp();
      break;
  }
}

void serviceSerial() {
  static char buf[SERIAL_LINE_MAX + 1];
  static uint8_t len = 0;
  static bool overflow = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      if (overflow) Serial.println(F("ERR: ligne trop longue, ignoree."));
      else handleCommand(buf);
      len = 0;
      overflow = false;
    } else if (len < SERIAL_LINE_MAX) {
      buf[len++] = c;
    } else {
      overflow = true;
    }
  }
}
