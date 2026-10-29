// ============================================================================
// COMMENT FLASHER :
//   - Tools > Board > industrialshield-avr > M-DUINO Family
//   - Tools > Port > puis sélectionner le bon
//   - Tools > Model > M-DUINO 38AR+
//   Voilà !
// ============================================================================
// ============================================================================
// M-Duino 38AR+ : Chaudière basique + MAX6675 (sans bibliothèque)
// + RTC + EEPROM + Planning hebdo (2 plages/jour)
// + Sécurités : diagnostic-only en IDLE, latch + cooldown en RUN/COOLDOWN
// + Paramètres de puissance modifiables par commandes série + persistants EEPROM
// + Debug AUTO "event logging" (peu verbeux) : log uniquement sur changements
//
// PATCH 2025-12-19:
// + EVT pellets ON/OFF (log sur changement)
// + Mesure durée ON continue
// + FAILSAFE temporaire : coupe vis si ON continu trop longtemps
//
// PATCH 2025-12-19 (TEST RTC BOMBARDMENT):
// + Compteur appels rtcReadOk() (calls/sec) + ok/fail + burst_max + loop_hz
// + Impression 1 fois/seconde : ligne [RTCSTATS]
//
// PATCH 2025-12-19 (FIX RTC BOMBARDMENT):
// + Throttle + cache RTC : une seule lecture RTC toutes les RTC_POLL_MS
// + autoControlService() et logAutoEventIfChanged() réutilisent le cache
//
// PATCH 2025-12-19 (NEW):
// 1) Force COOLDOWN (manuel + auto) via commande "K1/K0" ou "COOLDOWN ON/OFF"
// 2) 3 niveaux de puissance: LOW/MED/HIGH via "POWER LOW|MED|HIGH"
//    - par défaut: LOW=PP10000, MED=8000, HIGH=6000 ; PON=1000 ; FV=6.0
//    - PP/PON/FV restent possibles (override) + persistants EEPROM
// 3) Watchdog AVR (WDT) + log cause reset
//    + Ajout option compile-time pour désactiver facilement le WDT:
//      #define ENABLE_WDT 0
//
// PATCH 2026-10-29:
// 1) FORCE COOLDOWN (k1 / COOLDOWN_ON) : 10 min de cooldown puis IDLE (avant : cooldown
//    sans fin). En AUTO, la chaudière reste arrêtée jusqu'à la fin de la plage en cours
//    (START, changement de MODE, k0 ou i0 relâchent ce maintien).
// 2) START pendant un COOLDOWN : reprise en RUN sans couper la pompe (refusé si ALARM ou
//    MODE OFF). START/STOP du Nextion passent d'abord en MANUAL si besoin.
// 3) Sécurités en RUN : thermocouple illisible (NaN > 30 s), pas de dépression (> 15 s après
//    60 s de RUN, voir ENABLE_NO_DEP_ALARM), vis ON trop longtemps (ALARM au lieu de faire
//    claquer le relais). Candelette manuelle (c1) coupée après 10 min.
// 4) PP/PON/FV : une valeur refusée n'est plus appliquée ; PON <= PP - 500 ms ; ces réglages
//    ne sont plus écrasés à chaque démarrage (le MED de démarrage est appliqué à part).
// 5) Nextion : bloc d'état page0.t3 (T°, pression, dépression, sorties, état, mode, alarme),
//    resynchro complète quand l'écran (re)démarre, sw0/select0 suivis à chaque changement,
//    envois non bloquants (file d'attente).
// 6) Robustesse : ligne série bornée, SETTIME et EEPROM validés, [RTCSTATS] -> DEBUG_RTCSTATS.
//
// NO DEPRESSION : ALARM en RUN (ENABLE_NO_DEP_ALARM 1) ; diagnostic seulement si 0.
//
// PATCH 2026-10-29 (2):
// + Sketch découpé en onglets (voir ORGANISATION ci-dessous) + config.h pour les réglages.
// + Commandes série et Nextion sans String (tampons char, mots-clés en flash).
// + Banc de simulation tests/ + compilation GitHub Actions à chaque push.
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#include <RTC.h>

#include "config.h"

#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  #include <avr/wdt.h>
  #include <avr/io.h>
#endif

// ============================================================================
// ORGANISATION (onglets de l'IDE Arduino, assemblés dans cet ordre : ce fichier,
// puis les autres .ino par ordre alphabétique)
//   config.h       : switches, câblage, temporisations, seuils, profils de puissance
//   (ce fichier)   : types, état partagé, prototypes, watchdog, setup() / loop()
//   chaudiere.ino  : puissance, sorties, transitions IDLE/RUN/COOLDOWN, AUTO, START/STOP
//   commandes.ino  : commandes série, aide, snapshot, journal d'événements
//   nextion.ino    : écran Nextion (commandes, synchro, bloc d'état)
//   planning.ino   : EEPROM, RTC, planning hebdo
//   securites.ino  : capteurs, MAX6675, failsafe vis, alarmes
// Une variable utilisée dans plusieurs onglets est déclarée ici ; une variable
// propre à un onglet est déclarée en tête de cet onglet.
// ============================================================================

// --------------------------------------------------------------------
// TYPES
// --------------------------------------------------------------------
enum PowerLevel : uint8_t { PWR_LOW = 0, PWR_MED = 1, PWR_HIGH = 2 };

enum ControlMode : uint8_t { MODE_OFF = 0, MODE_MANUAL = 1, MODE_AUTO = 2 };

enum AlarmCode : uint8_t {
  ALM_NONE = 0,
  ALM_PELLETS_WATER_OH, ALM_PELLETS_OH, ALM_WATER_OH, ALM_WATER_SENSOR_OH,
  ALM_PRESSURE_HIGH, ALM_PRESSURE_LOW,
  ALM_NO_FIRE, ALM_NO_DEPRESSION, ALM_THERMOCOUPLE, ALM_PELLET_FAILSAFE
};

struct DaySchedule {
  uint16_t start1;
  uint16_t end1;
  uint16_t start2;
  uint16_t end2;
};

// Petit tampon texte (bloc d'état Nextion, sans String)
struct TextBuf {
  char*    buf;
  uint16_t cap;
  uint16_t len;
  void addChar(char c) { if (len + 1 < cap) { buf[len++] = c; buf[len] = '\0'; } }
  void add(const char* s) { while (*s) addChar(*s++); }
  void addP(const __FlashStringHelper* fs) {
    PGM_P p = reinterpret_cast<PGM_P>(fs);
    char c;
    while ((c = (char)pgm_read_byte(p++)) != 0) addChar(c);
  }
  void addInt(long v) { char tmp[12]; ltoa(v, tmp, 10); add(tmp); }
  void addFloat(float v, uint8_t dec) { char tmp[16]; dtostrf(v, 0, dec, tmp); add(tmp); }
};

// --------------------------------------------------------------------
// ÉTAT PARTAGÉ
// --------------------------------------------------------------------
unsigned long lastTickMs = 0;
bool autoSnapshotEnabled = true;

// Sorties
bool    statePellet   = false;
bool    statePump     = false;
bool    stateCandle   = false;
float   fanVoltageV   = 0.0f;  // valeur appliquée
uint8_t fanPwm        = 0;

// Dernière température fumées
float lastTempFumeesC = NAN;

// Chaudière
bool boilerOn = false;
bool stopCooldownActive = false;
unsigned long boilerStartMs = 0;
unsigned long stopStartMs   = 0;
bool startupLowPowerActive = false;
unsigned long startupLowPowerUntilMs = 0;
bool pumpKeepOn = false; // reprise depuis COOLDOWN : foyer chaud => pompe ON dès le START

// Repère de start (NO FIRE + NO DEP)
unsigned long lastStartMs = 0;

// Puissance : paramètres réglés (profil choisi, éventuellement modifiés via PP/PON/FV),
// persistants EEPROM. Pendant le démarrage la chaudière tourne en MED sans les écraser.
PowerLevel selectedPowerLevel = PWR_MED;
unsigned long pelletCyclePeriodMs = 8000UL;
unsigned long pelletOnDurationMs  = 1000UL;
float         runFanVoltageV      = 6.0f;

// FORCE COOLDOWN (k1) : cooldown 10 min puis IDLE. Le maintien bloque les démarrages AUTO
// jusqu'à la fin de la plage en cours ; START, changement de MODE, k0 et i0 le relâchent.
bool autoHoldActive = false;

// Debug + failsafe vis sans fin
bool lastPelletLogged = false;
unsigned long pelletOnSinceMs = 0;
unsigned long pelletLongestOnMs = 0;

// Alarme
bool alarmActive = false;
AlarmCode alarmCode = ALM_NONE;

// Derniers RAW
int rawNoDep = 0, rawPelletsOH = 0, rawWaterOH = 0, rawNTC = 0, rawPressure = 0, rawDep = 0;

// Candelette allumée à la main (c1)
unsigned long candleManualSinceMs = 0;

// Mode + planning
ControlMode controlMode = MODE_MANUAL;
DaySchedule weekSched[7]; // 0..6 => Lun..Dim
bool programDemandOn = false;

// AUTO : anti-oscillation (demande stable avant action) + délai entre commandes
unsigned long lastAutoCommandMs = 0;
bool lastDemandSeen = false;
unsigned long lastDemandChangeMs = 0;

// Cache RTC (dernière lecture OK ?)
bool g_rtcCachedOk = false;

// --------------------------------------------------------------------
// PROTOTYPES (par onglet)
// --------------------------------------------------------------------
// chaudiere.ino
void setSelectedPowerLevel(PowerLevel lvl);
unsigned long activePelletPeriodMs();
unsigned long activePelletOnMs();
float activeFanVoltageV();
bool powerParamsValid(long pp, long pon, bool printErr);
bool validatePowerParams(bool printErr);
uint8_t voltsToPwm(float volts);
void applyOutputs();
void boilerEnterRun(unsigned long now, const __FlashStringHelper* origin);
void boilerEnterCooldown(unsigned long now, const __FlashStringHelper* origin);
void autoControlService(unsigned long nowMs);
void boilerBasicStart(unsigned long now);
void boilerBasicStop(unsigned long now);
void boilerBasicService(unsigned long now);
void forceIdle(unsigned long now);
void forceCooldownOn(unsigned long now);
void forceCooldownOff();

// commandes.ino
void logAutoEventIfChanged(unsigned long nowMs);
void printTimeLine();
const __FlashStringHelper* modeToStr(ControlMode m);
const __FlashStringHelper* pwrToStr(PowerLevel p);
const __FlashStringHelper* pwrToStrFr(PowerLevel p);
const __FlashStringHelper* alarmToStr(AlarmCode a);
const __FlashStringHelper* alarmToStrFr(AlarmCode a);
void printParams();
void printSchedule();
void printSnapshot();
void printHelp();
bool manualAllowed();
bool startsWithNoCase(const char* s, const __FlashStringHelper* prefix);
bool equalsNoCase(const char* s, const __FlashStringHelper* word);
char* trimInPlace(char* s);
void handleCommand(const char* lineIn);
void handleCommand(const __FlashStringHelper* cmd);
void serviceSerial();

// nextion.ino
void handleNextionCommand(const char* lineIn);
void nextionSendCommand(const __FlashStringHelper* cmd);
void nextionSendValue(const char* id, uint8_t val);
void nextionSendDaySchedule(uint8_t day1to7);
void updateNextionFireState(bool force);
void updateNextionControlMode(bool force);
void updateNextionCandleState(bool force);
void updateNextionPowerLevel(bool force);
void updateNextionStatus(bool force);
void nextionSyncAll();
void nextionService(unsigned long now);
void serviceNextion();

// planning.ino
uint16_t crc16_ccitt(const uint8_t* data, size_t len);
void setDefaultSchedule();
void setDefaultParams();
void eepromSaveAll();
bool eepromLoadAny();
bool sanitizeLoadedConfig();
bool rtcReadOk();
void rtcPollService(unsigned long nowMs);
uint8_t dowFromDate(int y, int m, int d);
uint8_t dowFromRTC();
uint16_t minuteOfDayFromRTC();
uint8_t daysInMonth(int y, int m);
#if DEBUG_RTCSTATS
void rtcStatsOnLoop();
void rtcStatsPrint1Hz(unsigned long nowMs);
#endif
bool inRange(uint16_t nowMin, uint16_t startMin, uint16_t endMin);
bool scheduleDemand(uint8_t dow_1to7, uint16_t nowMin);

// securites.ino
float adcRawToVoltage10V(int raw);
float waterPressureBarFromRaw(int raw);
float waterTempC_fromRaw(int raw);
int depressionDeltaRaw(int raw);
float depressionDeltaVolts(int raw);
uint16_t max6675_readRaw();
float max6675_readCelsius();
inline bool acPresentFromRaw(int raw);
inline bool acAbsentFromRaw(int raw);
void readAllSensorsOnce();
void pelletDebugAndFailsafe(unsigned long nowMs);
void manualCandleGuard(unsigned long now);
void clearAlarm();
void triggerAlarmAndCooldown(AlarmCode code, unsigned long now);
void diagPrint(unsigned long now, const __FlashStringHelper* label, const char* detail);
AlarmCode currentInputFault();
void checkSafeties(unsigned long now);

// --------------------------------------------------------------------
// WATCHDOG helpers (AVR) - compilable disable via ENABLE_WDT
// --------------------------------------------------------------------
static uint8_t g_resetCause = 0;

static void captureResetCauseEarly() {
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  g_resetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
#else
  g_resetCause = 0;
#endif
}

static void printResetCause() {
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  Serial.print(F("ResetCause(MCUSR)=0x"));
  Serial.println(g_resetCause, HEX);
  if (g_resetCause & _BV(WDRF)) Serial.println(F(" - WDT reset"));
  if (g_resetCause & _BV(BORF)) Serial.println(F(" - Brown-out reset"));
  if (g_resetCause & _BV(EXTRF)) Serial.println(F(" - External reset"));
  if (g_resetCause & _BV(PORF)) Serial.println(F(" - Power-on reset"));
#else
  (void)g_resetCause;
#endif
}

static void watchdogEnable() {
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  wdt_enable(WDTO_4S); // 4 secondes
#endif
}

static inline void watchdogKick() {
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  wdt_reset();
#endif
}

// --------------------------------------------------------------------
// SETUP / LOOP
// --------------------------------------------------------------------
void setup() {
  captureResetCauseEarly();

  Serial.begin(115200);
#if ENABLE_NEXTION
  NEXTION_SERIAL.begin(NEXTION_BAUD);
#endif

  // Évite blocage éternel si pas d'USB: max ~2s
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000UL)) {}

  Serial.println(F("=== M-Duino 38AR+ Boiler basic + RTC cache + POWER + FORCE_COOLDOWN + WDT + Nextion status ==="));
  printResetCause();

  // Entrées analogiques
  pinMode(PIN_AI_WATER_NTC,      INPUT);
  pinMode(PIN_AI_WATER_PRESSURE, INPUT);
  pinMode(PIN_AI_DEPRESSION,     INPUT);
  pinMode(PIN_AI_NO_DEPRESSION,    INPUT);
  pinMode(PIN_AI_PELLETS_OVERHEAT, INPUT);
  pinMode(PIN_AI_WATER_OVERHEAT,   INPUT);

  // Sorties
  pinMode(PIN_AO_FAN_SPEED, OUTPUT);
  pinMode(PIN_R_PELLET,     OUTPUT);
  pinMode(PIN_R_PUMP,       OUTPUT);
  pinMode(PIN_R_SSR,        OUTPUT);

  // MAX6675
  pinMode(PIN_MAX6675_SO,  INPUT);
  pinMode(PIN_MAX6675_SCK, OUTPUT);
  pinMode(PIN_MAX6675_CS,  OUTPUT);
  digitalWrite(PIN_MAX6675_SCK, LOW);
  digitalWrite(PIN_MAX6675_CS,  HIGH);

  // Init sorties OFF
  statePellet = false;
  statePump   = false;
  stateCandle = false;
  fanVoltageV = 0.0f;
  fanPwm      = voltsToPwm(fanVoltageV);
  applyOutputs();

  // Patch debug init
  lastPelletLogged = statePellet;
  pelletOnSinceMs = 0;
  pelletLongestOnMs = 0;

  boilerOn = false;
  stopCooldownActive = false;
  boilerStartMs = 0;
  stopStartMs = 0;
  lastStartMs = 0;

  clearAlarm();

  autoHoldActive = false;

  // Charger EEPROM
  if (!eepromLoadAny()) {
    controlMode = MODE_MANUAL;
    setDefaultSchedule();
    setDefaultParams();
    eepromSaveAll();
    Serial.println(F("EEPROM: no valid data -> defaults created."));
  } else {
    if (sanitizeLoadedConfig()) {
      eepromSaveAll();
      Serial.println(F("EEPROM: mode/planning hors bornes -> corriges."));
    }
    if (!validatePowerParams(false)) {
      setDefaultParams();
      eepromSaveAll();
      Serial.println(F("EEPROM: invalid power params -> reset to defaults."));
    } else {
      Serial.println(F("EEPROM: loaded OK."));
    }
  }

  // Init anti-oscillation
  lastDemandSeen = programDemandOn;
  lastDemandChangeMs = millis();

  // Prime lecture capteurs
  readAllSensorsOnce();

  printHelp();
  printSchedule();
  printParams();
  printTimeLine();

  lastTickMs = millis();
  // Si l'écran démarre après l'Arduino, il renverra 0x88 => nouvelle resynchro
  nextionSyncAll();

  // Activer WDT en fin de setup (après initialisations)
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  watchdogEnable();
  watchdogKick();
#endif
}

void loop() {
  unsigned long now = millis();
#if DEBUG_RTCSTATS
  rtcStatsOnLoop();
#endif

  serviceSerial();
  serviceNextion();

  // Mise à jour RTC (throttled) - UNE seule lecture périodique
  rtcPollService(now);

  // MODE OFF => arrêt propre si RUN
  if (controlMode == MODE_OFF && boilerOn) boilerEnterCooldown(now, F("MODE OFF"));

  // Pilotage AUTO (utilise cache)
  autoControlService(now);

  // Service chaudière
  boilerBasicService(now);
  manualCandleGuard(now);

  // Patch: debug + failsafe vis
  pelletDebugAndFailsafe(now);

  // Tick 1s : capteurs + sécurités + debug events + snapshot
  if (now - lastTickMs >= SNAPSHOT_PERIOD_MS) {
    lastTickMs = now;

    readAllSensorsOnce();
    checkSafeties(now);

    logAutoEventIfChanged(now);

#if DEBUG_RTCSTATS
    rtcStatsPrint1Hz(now);
#endif

    if (autoSnapshotEnabled) printSnapshot();
  }

  // Écran : état feu/candelette/mode/puissance + bloc d'état + envoi non bloquant
  nextionService(now);

  // Watchdog feed: à la fin de loop quand tout s'est bien déroulé
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  watchdogKick();
#endif
}
