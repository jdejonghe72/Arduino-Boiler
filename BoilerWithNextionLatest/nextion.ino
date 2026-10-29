// ============================================================================
// NEXTION (Serial1) : commandes de l'écran, synchro, bloc d'état page0.t3
// ============================================================================

enum FireState : uint8_t { FIRE_IDLE = 0, FIRE_RUN = 1, FIRE_COOLDOWN = 2 };
static FireState lastNextionFireState = FIRE_IDLE;
static bool nextionFireStateInit = false;
static bool lastNextionCandleOn = false;
static bool nextionCandleInit = false;

const uint16_t NX_COLOR_TEXT  = 54938; // gris clair (couleur des libellés de page0)
const uint16_t NX_COLOR_WARN  = 64512; // orange : défaut détecté à l'arrêt
const uint16_t NX_COLOR_ALARM = 63488; // rouge : ALARM
static unsigned long nextionStatusLastMs = 0;
static unsigned long nextionResyncLastMs = 0;
static uint16_t nextionStatusCrc = 0;
static uint16_t nextionStatusColor = 0;
static bool nextionSyncPending = false;

// START/STOP depuis l'écran : on passe d'abord en MANUAL (comme le bouton le fait déjà)
static void nextionEnsureManual() {
  if (controlMode == MODE_AUTO) handleCommand(F("MODE MANUAL"));
}

// Commandes texte de l'écran (print "..." + printh 0A)
void handleNextionCommand(const char* lineIn) {
  char buf[SERIAL_LINE_MAX + 1];
  strncpy(buf, lineIn, SERIAL_LINE_MAX);
  buf[SERIAL_LINE_MAX] = '\0';
  char* line = trimInPlace(buf);
  if (!*line) return;

  if (equalsNoCase(line, F("START"))) { nextionEnsureManual(); handleCommand(F("b1")); return; }
  if (equalsNoCase(line, F("STOP")))  { nextionEnsureManual(); handleCommand(F("b0")); return; }

  if (equalsNoCase(line, F("MODE_AUTO")) || equalsNoCase(line, F("AUTO")))     { handleCommand(F("MODE AUTO")); return; }
  if (equalsNoCase(line, F("MODE_MANUAL")) || equalsNoCase(line, F("MANUAL"))) { handleCommand(F("MODE MANUAL")); return; }
  if (equalsNoCase(line, F("MODE_OFF")) || equalsNoCase(line, F("OFF")))       { handleCommand(F("MODE OFF")); return; }

  if (equalsNoCase(line, F("PWR_LOW")) || equalsNoCase(line, F("POWER_LOW")))   { handleCommand(F("POWER LOW")); return; }
  if (equalsNoCase(line, F("PWR_MED")) || equalsNoCase(line, F("POWER_MED")))   { handleCommand(F("POWER MED")); return; }
  if (equalsNoCase(line, F("PWR_HIGH")) || equalsNoCase(line, F("POWER_HIGH"))) { handleCommand(F("POWER HIGH")); return; }
  if (equalsNoCase(line, F("FORCE_IDLE")) || equalsNoCase(line, F("IDLE")))     { handleCommand(F("i0")); return; }
  if (equalsNoCase(line, F("COOLDOWN_ON")))  { handleCommand(F("k1")); return; }
  if (equalsNoCase(line, F("COOLDOWN_OFF"))) { handleCommand(F("k0")); return; }

  if (startsWithNoCase(line, F("SETDAY"))) { handleCommand(line); return; }
  if (startsWithNoCase(line, F("LOAD_"))) {
    int day = atoi(line + 5);
    if (day < 1 || day > 7) {
      Serial.println(F("ERR: LOAD_<1..7>"));
      return;
    }
    nextionSendDaySchedule((uint8_t)day);
    return;
  }

  Serial.print(F("Nextion cmd inconnu: "));
  Serial.println(line);
}

// Trames binaires du Nextion (codes retour terminés par FF FF FF)
static void handleNextionReturnCode(uint8_t code) {
  if (code == 0x88) { // "ready" envoyé par Program.s à chaque démarrage de l'écran
    Serial.println(F("[NEXTION] ecran pret -> resynchro"));
    nextionSyncAll();
    return;
  }
  if (code == 0x00) return; // "startup" (00 00 00 FF FF FF)

  static unsigned long lastLogMs = 0;
  unsigned long now = millis();
  if (lastLogMs != 0 && now - lastLogMs < 10000UL) return;
  lastLogMs = now;
  Serial.print(F("[NEXTION] code retour 0x"));
  Serial.println(code, HEX);
}

// File d'envoi vers le Nextion : vidée sans attendre par nextionService()
#if ENABLE_NEXTION
static uint8_t  nxTx[NEXTION_TX_BUF];
static uint16_t nxTxHead = 0;
static uint16_t nxTxTail = 0;

static bool nxTxEmpty() { return nxTxHead == nxTxTail; }

static void nxTxService() {
  while (nxTxTail != nxTxHead && NEXTION_SERIAL.availableForWrite() > 0) {
    NEXTION_SERIAL.write(nxTx[nxTxTail]);
    nxTxTail = (nxTxTail + 1) % NEXTION_TX_BUF;
  }
}

static void nxPutByte(uint8_t b) {
  uint16_t next = (nxTxHead + 1) % NEXTION_TX_BUF;
  while (next == nxTxTail) { // file pleine (rare) : on attend l'UART
    NEXTION_SERIAL.write(nxTx[nxTxTail]);
    nxTxTail = (nxTxTail + 1) % NEXTION_TX_BUF;
  }
  nxTx[nxTxHead] = b;
  nxTxHead = next;
}
#else
static bool nxTxEmpty() { return true; }
static void nxTxService() {}
static void nxPutByte(uint8_t b) { (void)b; }
#endif

static void nxPut(const char* s) { while (*s) nxPutByte((uint8_t)*s++); }

static void nxPutP(const __FlashStringHelper* fs) {
  PGM_P p = reinterpret_cast<PGM_P>(fs);
  char c;
  while ((c = (char)pgm_read_byte(p++)) != 0) nxPutByte((uint8_t)c);
}

static void nxPutNum(long v) { char tmp[12]; ltoa(v, tmp, 10); nxPut(tmp); }

static void nxEnd() { nxPutByte(0xFF); nxPutByte(0xFF); nxPutByte(0xFF); }

void nextionSendCommand(const __FlashStringHelper* cmd) {
  nxPutP(cmd);
  nxEnd();
}

void nextionSendValue(const char* id, uint8_t val) {
  nxPut(id);
  nxPutP(F(".val="));
  nxPutNum(val);
  nxEnd();
}

static uint8_t minuteToHourClamped(uint16_t m) {
  if (m > 1439) m = 1439;
  return (uint8_t)(m / 60);
}

static uint8_t minuteToQuarter(uint16_t m) {
  if (m > 1439) m = 1439;
  uint8_t mm = (uint8_t)(m % 60);
  uint8_t q = (uint8_t)((mm + 7) / 15);
  if (q > 3) q = 3;
  return q;
}

void nextionSendDaySchedule(uint8_t day1to7) {
  if (day1to7 < 1 || day1to7 > 7) return;
  DaySchedule &ds = weekSched[day1to7 - 1];

  uint8_t h1 = minuteToHourClamped(ds.start1);
  uint8_t m1 = minuteToQuarter(ds.start1);
  uint8_t h2 = minuteToHourClamped(ds.end1);
  uint8_t m2 = minuteToQuarter(ds.end1);
  uint8_t h3 = minuteToHourClamped(ds.start2);
  uint8_t m3 = minuteToQuarter(ds.start2);
  uint8_t h4 = minuteToHourClamped(ds.end2);
  uint8_t m4 = minuteToQuarter(ds.end2);

  nextionSendValue("H1", h1);
  nextionSendValue("M1", m1);
  nextionSendValue("H2", h2);
  nextionSendValue("M2", m2);
  nextionSendValue("H3", h3);
  nextionSendValue("M3", m3);
  nextionSendValue("H4", h4);
  nextionSendValue("M4", m4);
  nextionSendCommand(F("loadDONE.en=1"));
}

void updateNextionFireState(bool force) {
#if ENABLE_NEXTION
  FireState state = FIRE_IDLE;
  if (boilerOn) state = FIRE_RUN;
  else if (stopCooldownActive) state = FIRE_COOLDOWN;

  if (!force && nextionFireStateInit && state == lastNextionFireState) return;

  lastNextionFireState = state;
  nextionFireStateInit = true;

  switch (state) {
    case FIRE_RUN:
      nextionSendCommand(F("redfire.en=1"));
      nextionSendCommand(F("page0.nofire.aph=0"));
      nextionSendCommand(F("page1.nofire.aph=0"));
      break;
    case FIRE_COOLDOWN:
      nextionSendCommand(F("flashfire.en=1"));
      break;
    case FIRE_IDLE:
    default:
      nextionSendCommand(F("grayfire.en=1"));
      nextionSendCommand(F("page0.nofire.aph=127"));
      nextionSendCommand(F("page1.nofire.aph=127"));
      break;
  }
#else
  (void)force;
#endif
}

void updateNextionControlMode(bool force) {
#if ENABLE_NEXTION
  static bool lastAuto = false;
  static bool init = false;

  bool isAuto = (controlMode == MODE_AUTO);
  if (!force && init && isAuto == lastAuto) return;

  lastAuto = isAuto;
  init = true;

  if (isAuto) nextionSendCommand(F("page0.sw0.val=1"));
  else nextionSendCommand(F("page0.sw0.val=0"));
#else
  (void)force;
#endif
}

void updateNextionCandleState(bool force) {
#if ENABLE_NEXTION
  bool candleOn = stateCandle;
  if (!force && nextionCandleInit && candleOn == lastNextionCandleOn) return;

  lastNextionCandleOn = candleOn;
  nextionCandleInit = true;

  if (candleOn) {
    nextionSendCommand(F("page0.bougie.aph=127"));
    nextionSendCommand(F("page1.bougie.aph=127"));
  } else {
    nextionSendCommand(F("page0.bougie.aph=0"));
    nextionSendCommand(F("page1.bougie.aph=0"));
  }
#else
  (void)force;
#endif
}

// select0 (MIN./MOY./MAX.) : pas de renvoi périodique pour ne pas écraser
// un choix en cours sur l'écran (tm0 l'envoie 1,3 s après la sélection)
void updateNextionPowerLevel(bool force) {
#if ENABLE_NEXTION
  static int8_t lastLevel = -1;
  if (!force && lastLevel == (int8_t)selectedPowerLevel) return;
  lastLevel = (int8_t)selectedPowerLevel;
  nextionSendValue("page0.select0", (uint8_t)selectedPowerLevel);
#else
  (void)force;
#endif
}

static void statusAddTemp(TextBuf& t, float v) {
  if (isnan(v)) t.addP(F("--"));
  else t.addInt((long)roundf(v));
  t.addP(F("\xB0" "C"));
}

static void statusAddDuration(TextBuf& t, unsigned long ms) {
  unsigned long m = ms / 60000UL;
  if (m < 60UL) { t.addInt((long)m); t.addP(F(" min")); return; }
  t.addInt((long)(m / 60UL)); t.addP(F(" h "));
  if (m % 60UL < 10UL) t.addChar('0');
  t.addInt((long)(m % 60UL));
}

// Bloc d'état page0.t3 (lignes séparées par \r) ; retourne la couleur du texte
static uint16_t buildNextionStatus(TextBuf& t, unsigned long now) {
  // Températures / pression
  t.addP(F("Eau ")); statusAddTemp(t, waterTempC_fromRaw(rawNTC));
  t.addP(F("   ")); t.addFloat(waterPressureBarFromRaw(rawPressure), 1);
  t.addP(F(" bar   Fum\xE9" "es ")); statusAddTemp(t, lastTempFumeesC);
  t.addP(F("\\r"));

  // Sorties
  t.addP(F("Ventilo "));
  if (fanPwm == 0) t.addP(F("OFF"));
  else { t.addFloat(fanVoltageV, 1); t.addP(F("V")); }
  t.addP(F("   Pompe ")); t.addP(statePump ? F("ON") : F("OFF"));
  t.addP(F("   Vis "));
  if (boilerOn) {
    t.addFloat(activePelletOnMs() / 1000.0f, 1); t.addChar('/');
    t.addInt((long)((activePelletPeriodMs() + 500UL) / 1000UL)); t.addChar('s');
  } else {
    t.addP(statePellet ? F("ON") : F("OFF"));
  }
  t.addP(F("\\r"));

  // Dépression (pressostat significatif seulement ventilateur en marche)
  t.addP(F("D\xE9" "pression ")); t.addFloat(10.0f * depressionDeltaVolts(rawDep), 1);
  t.addP(F(" mbar"));
  if (fanPwm > 0) t.addP(acPresentFromRaw(rawNoDep) ? F("   Tirage OK") : F("   Pas de tirage"));
  t.addP(F("\\r"));

  // Etat chaudière
  if (boilerOn) {
    if (startupLowPowerActive) {
      t.addP(F("D\xE9" "marrage ")); t.addInt((long)((now - boilerStartMs) / 60000UL));
      t.addChar('/'); t.addInt((long)(STARTUP_LOW_POWER_MS / 60000UL)); t.addP(F(" min (MOY.)"));
    } else {
      t.addP(F("RUN ")); statusAddDuration(t, now - boilerStartMs);
      t.addP(F("   Puissance ")); t.addP(pwrToStrFr(selectedPowerLevel));
    }
  } else if (stopCooldownActive) {
    unsigned long since = now - stopStartMs;
    unsigned long left = (since < STOP_COOLDOWN_MS) ? STOP_COOLDOWN_MS - since : 0UL;
    t.addP(F("Cooldown : encore ")); t.addInt((long)((left + 59999UL) / 60000UL)); t.addP(F(" min"));
  } else {
    t.addP(F("Arr\xEA" "t   Puissance ")); t.addP(pwrToStrFr(selectedPowerLevel));
  }
  t.addP(F("\\r"));

  if (alarmActive) {
    t.addP(F("ALARME : ")); t.addP(alarmToStrFr(alarmCode));
    t.addP(F("\\rAcquitter : Avanc\xE9" "s > Forcer l'arr\xEA" "t"));
    return NX_COLOR_ALARM;
  }

  // Mode
  switch (controlMode) {
    case MODE_AUTO:
      if (!g_rtcCachedOk) t.addP(F("Mode AUTO : horloge HS !"));
      else if (autoHoldActive) t.addP(F("AUTO suspendu jusqu'\xE0" " fin de plage"));
      else t.addP(programDemandOn ? F("Mode AUTO : plage en cours") : F("Mode AUTO : hors plage"));
      break;
    case MODE_MANUAL: t.addP(F("Mode MANUEL")); break;
    default:          t.addP(F("Mode OFF (commande s\xE9" "rie)")); break;
  }
  t.addP(F("\\r"));

  // Défaut visible à l'arrêt (en RUN/COOLDOWN il déclenche une ALARM)
  AlarmCode fault = ALM_NONE;
  if (!boilerOn && !stopCooldownActive) {
    fault = currentInputFault();
    if (fault == ALM_NONE && isnan(lastTempFumeesC)) fault = ALM_THERMOCOUPLE;
  }
  if (fault == ALM_NONE) {
    t.addP(F("Aucune alarme"));
    return NX_COLOR_TEXT;
  }
  t.addP(F("D\xE9" "faut : ")); t.addP(alarmToStrFr(fault));
  return NX_COLOR_WARN;
}

void updateNextionStatus(bool force) {
#if ENABLE_NEXTION
  static char text[256];
  TextBuf t = { text, sizeof(text), 0 };
  text[0] = '\0';
  uint16_t color = buildNextionStatus(t, millis());

  uint16_t crc = crc16_ccitt((const uint8_t*)text, t.len);
  if (force || crc != nextionStatusCrc) {
    nextionStatusCrc = crc;
    nxPutP(F("page0.t3.txt=\""));
    nxPut(text);
    nxPutByte('"');
    nxEnd();
  }
  if (force || color != nextionStatusColor) {
    nextionStatusColor = color;
    nxPutP(F("page0.t3.pco="));
    nxPutNum(color);
    nxEnd();
  }
#else
  (void)force;
#endif
}

// Resynchro complète, faite dès que la file d'envoi est vide (demandes regroupées :
// au démarrage, setup() et le 0x88 de l'écran arrivent presque en même temps)
void nextionSyncAll() {
  nextionSyncPending = true;
}

static void nextionSendAll() {
  updateNextionFireState(true);
  updateNextionCandleState(true);
  updateNextionControlMode(true);
  updateNextionPowerLevel(true);
  updateNextionStatus(true);
}

// Appelé à chaque loop() : changements d'état, bloc d'état, resynchro, envoi de la file
void nextionService(unsigned long now) {
#if ENABLE_NEXTION
  if (nextionSyncPending && nxTxEmpty()) {
    nextionSyncPending = false;
    nextionSendAll();
  }

  updateNextionFireState(false);
  updateNextionCandleState(false);
  updateNextionControlMode(false);
  updateNextionPowerLevel(false);

  if (now - nextionResyncLastMs >= NEXTION_RESYNC_PERIOD_MS) {
    nextionResyncLastMs = now;
    updateNextionFireState(true);
    updateNextionCandleState(true);
    updateNextionControlMode(true);
  }
  if (now - nextionStatusLastMs >= NEXTION_STATUS_PERIOD_MS) {
    nextionStatusLastMs = now;
    updateNextionStatus(false);
  }
  nxTxService();
#else
  (void)now;
#endif
}

void serviceNextion() {
#if ENABLE_NEXTION
  static char buf[SERIAL_LINE_MAX + 1];
  static uint8_t len = 0;
  static uint8_t ffCount = 0;

  while (NEXTION_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)NEXTION_SERIAL.read();

    if (b == 0xFF) {
      ffCount++;
      if (ffCount >= 3) {
        if (len > 0) {
          uint8_t first = (uint8_t)buf[0];
          buf[len] = '\0';
          if (first < 0x20 || first >= 0x80) handleNextionReturnCode(first);
          else handleNextionCommand(buf);
          len = 0;
        }
        ffCount = 0;
      }
      continue;
    }

    if (ffCount > 0) ffCount = 0;

    if (b == '\r' || b == '\n') {
      if (len > 0) {
        buf[len] = '\0';
        handleNextionCommand(buf);
        len = 0;
      }
      continue;
    }

    if (len < SERIAL_LINE_MAX) buf[len++] = (char)b;
  }
#endif
}
