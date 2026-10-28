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
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#include <RTC.h>

// --------------------------------------------------------------------
// NEXTION (utilise Serial1 pour ne pas bloquer le debug sur Serial)
// --------------------------------------------------------------------
#define ENABLE_NEXTION 1
#define NEXTION_BAUD 9600
#define NEXTION_SERIAL Serial1
#define NEXTION_TX_BUF 512  // file d'envoi : loop() n'attend pas l'UART à 9600 bauds

// --------------------------------------------------------------------
// WATCHDOG enable switch
// --------------------------------------------------------------------
#define ENABLE_WDT 1  // 0 = watchdog désactivé, 1 = watchdog activé

// --------------------------------------------------------------------
// SECURITES / DEBUG (switches compile-time)
// --------------------------------------------------------------------
#define ENABLE_NO_DEP_ALARM 1  // 1 = pas de dépression en RUN => ALARM ; 0 = diagnostic seulement
#define DEBUG_RTCSTATS      0  // 1 = ligne [RTCSTATS] chaque seconde (test RTC du 2025-12-19)

#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  #include <avr/wdt.h>
  #include <avr/io.h>
#endif

// --------------------------------------------------------------------
// MAPPING INDUSTRIAL SHIELDS (Zone B analog inputs)
// --------------------------------------------------------------------
const int PIN_AI_NO_DEPRESSION    = 54; // I0.7
const int PIN_AI_PELLETS_OVERHEAT = 55; // I0.8
const int PIN_AI_WATER_OVERHEAT   = 56; // I0.9
const int PIN_AI_WATER_NTC        = 57; // I0.10
const int PIN_AI_WATER_PRESSURE   = 58; // I0.11
const int PIN_AI_DEPRESSION       = 59; // I0.12

// Sorties
const int PIN_AO_FAN_SPEED = A1_0;  // 0-10V ventilateur
const int PIN_R_PELLET     = R1_1;  // vis sans fin
const int PIN_R_PUMP       = R1_2;  // pompe
const int PIN_R_SSR        = R1_4;  // SSR candelette

// MAX6675 (bit-banging)
const int PIN_MAX6675_SO  = 50;  // MISO
const int PIN_MAX6675_SCK = 52;  // SCK
const int PIN_MAX6675_CS  = 2;   // CS

// --------------------------------------------------------------------
// PARAMÈTRES GÉNÉRAUX
// --------------------------------------------------------------------
const unsigned long SNAPSHOT_PERIOD_MS = 1000;
unsigned long lastTickMs = 0;
bool autoSnapshotEnabled = true;
const unsigned int SERIAL_LINE_MAX = 64; // longueur max d'une commande (série et Nextion)

// Sorties
bool    statePellet   = false;
bool    statePump     = false;
bool    stateCandle   = false;
float   fanVoltageV   = 0.0f;  // valeur appliquée
uint8_t fanPwm        = 0;

// Dernière température fumées
float lastTempFumeesC = NAN;

// --------------------------------------------------------------------
// MODE CHAUDIÈRE BASIQUE
// --------------------------------------------------------------------
const unsigned long IGNITION_DURATION_MS   = 6UL * 60UL * 1000UL;
const unsigned long PUMP_START_DELAY_MS    = 1UL  * 60UL * 1000UL;
const unsigned long STOP_COOLDOWN_MS       = 10UL * 60UL * 1000UL;
const unsigned long STARTUP_LOW_POWER_MS   = 6UL * 60UL * 1000UL;

bool boilerOn = false;
bool stopCooldownActive = false;
unsigned long boilerStartMs = 0;
unsigned long stopStartMs   = 0;
bool startupLowPowerActive = false;
unsigned long startupLowPowerUntilMs = 0;
bool pumpKeepOn = false; // reprise depuis COOLDOWN : foyer chaud => pompe ON dès le START

// Repère de start (NO FIRE + NO DEP)
unsigned long lastStartMs = 0;

// --------------------------------------------------------------------
// PUISSANCE : 3 niveaux + paramètres réglés (override possible)
// --------------------------------------------------------------------
enum PowerLevel : uint8_t { PWR_LOW = 0, PWR_MED = 1, PWR_HIGH = 2 };
PowerLevel selectedPowerLevel = PWR_MED;

struct PowerProfile {
  unsigned long ppMs;
  unsigned long ponMs;
  float fvV;
};

// Valeurs demandées (modifiable plus tard dans le code)
static const PowerProfile kProfiles[3] = {
  {10000UL, 1000UL, 6.0f}, // LOW
  { 8000UL, 1000UL, 6.0f}, // MED
  { 6000UL, 1000UL, 6.0f}, // HIGH
};

// Paramètres réglés (profil choisi, éventuellement modifiés via PP/PON/FV) : persistants EEPROM.
// Pendant la phase de démarrage, la chaudière tourne en MED sans les écraser (voir active*()).
unsigned long pelletCyclePeriodMs = 8000UL;
unsigned long pelletOnDurationMs  = 1000UL;
float         runFanVoltageV      = 6.0f;

// --------------------------------------------------------------------
// FORCE COOLDOWN (manuel + auto)
// --------------------------------------------------------------------
// k1 : cooldown 10 min puis IDLE. Le maintien bloque les démarrages AUTO jusqu'à la fin
// de la plage en cours ; START, changement de MODE, k0 et i0 le relâchent.
bool autoHoldActive = false;

// --------------------------------------------------------------------
// PATCH DEBUG + FAILSAFE VIS SANS FIN
// --------------------------------------------------------------------
const unsigned long PELLET_MAX_CONTINUOUS_ON_MS = 3000UL; // 3 s en manuel (w1)
const unsigned long PELLET_RUN_ON_MARGIN_MS     = 1000UL; // en RUN : PON actif + 1 s
const unsigned long PELLET_MIN_OFF_MS           = 500UL;  // PON <= PP - 500 ms

bool lastPelletLogged = false;
unsigned long pelletOnSinceMs = 0;
unsigned long pelletLongestOnMs = 0;

// --------------------------------------------------------------------
// SECURITES
// --------------------------------------------------------------------
const int AC_RAW_THRESHOLD = 300; // AC présent => RAW < 300 ; AC absent => RAW >= 300

const unsigned long NO_DEP_GRACE_MS      = 60UL * 1000UL;        // délai après START
const unsigned long NO_DEP_ALARM_MS      = 15UL * 1000UL;        // absence continue de dépression
const unsigned long NO_FIRE_DELAY_MS     = 10UL * 60UL * 1000UL; // actif en RUN
const unsigned long TC_FAULT_ALARM_MS    = 30UL * 1000UL;        // thermocouple illisible (NaN)
const unsigned long MANUAL_CANDLE_MAX_MS = 10UL * 60UL * 1000UL; // c1 manuel en IDLE

const int RAW_WATER_SENSOR_OVERHEAT = 55;   // I0.10 : RAW < 55
const int RAW_PRESSURE_TOO_HIGH     = 200;  // I0.11 : RAW > 200
const int RAW_PRESSURE_TOO_LOW      = 80;   // I0.11 : RAW < 80

const float FLUE_NO_FIRE_MIN_C      = 60.0f;

enum AlarmCode : uint8_t {
  ALM_NONE = 0,
  ALM_PELLETS_WATER_OH, ALM_PELLETS_OH, ALM_WATER_OH, ALM_WATER_SENSOR_OH,
  ALM_PRESSURE_HIGH, ALM_PRESSURE_LOW,
  ALM_NO_FIRE, ALM_NO_DEPRESSION, ALM_THERMOCOUPLE, ALM_PELLET_FAILSAFE
};

bool alarmActive = false;
AlarmCode alarmCode = ALM_NONE;

// Derniers RAW
int rawNoDep = 0, rawPelletsOH = 0, rawWaterOH = 0, rawNTC = 0, rawPressure = 0, rawDep = 0;

// Début des défauts continus (0 = pas de défaut en cours)
unsigned long tcNanSinceMs = 0;
unsigned long noDepSinceMs = 0;
unsigned long candleManualSinceMs = 0;

// Diagnostic anti-spam
unsigned long lastDiagMs = 0;
const unsigned long DIAG_PERIOD_MS = 5000;

// --------------------------------------------------------------------
// RTC + PLANNING + EEPROM
// --------------------------------------------------------------------
enum ControlMode : uint8_t { MODE_OFF = 0, MODE_MANUAL = 1, MODE_AUTO = 2 };
ControlMode controlMode = MODE_MANUAL;

struct DaySchedule {
  uint16_t start1;
  uint16_t end1;
  uint16_t start2;
  uint16_t end2;
};

DaySchedule weekSched[7]; // 0..6 => Lun..Dim

bool programDemandOn = false;

const unsigned long AUTO_MIN_COMMAND_GAP_MS = 30UL * 1000UL;
unsigned long lastAutoCommandMs = 0;

// Option anti-oscillation : demande stable avant action
const unsigned long AUTO_DEMAND_STABLE_MS = 60UL * 1000UL;
bool lastDemandSeen = false;
unsigned long lastDemandChangeMs = 0;

// EEPROM layout
static const uint16_t EEPROM_MAGIC = 0xB017;
static const uint8_t  EEPROM_VER   = 3;

struct EepromDataV3 {
  uint16_t magic;
  uint8_t  ver;
  uint8_t  mode;
  uint8_t  pwrLevel;
  uint8_t  reserved;
  DaySchedule sched[7];
  unsigned long pelletCyclePeriodMs;
  unsigned long pelletOnDurationMs;
  float         runFanVoltageV;
  uint16_t crc;
};

struct EepromDataV2 {
  uint16_t magic;
  uint8_t  ver;
  uint8_t  mode;
  DaySchedule sched[7];
  unsigned long pelletCyclePeriodMs;
  unsigned long pelletOnDurationMs;
  float         runFanVoltageV;
  uint16_t crc;
};

struct EepromDataV1 {
  uint16_t magic;
  uint8_t  ver;
  uint8_t  mode;
  DaySchedule sched[7];
  uint16_t crc;
};

// --------------------------------------------------------------------
// DEBUG AUTO (event logging)
// --------------------------------------------------------------------
bool lastDemandLogged = false;
bool lastBoilerOnLogged = false;
bool lastCooldownLogged = false;
bool lastRtcOkLogged = true;

// --------------------------------------------------------------------
// NEXTION: état feu (IDLE/RUN/COOLDOWN) + bloc d'état page0.t3
// --------------------------------------------------------------------
enum FireState : uint8_t { FIRE_IDLE = 0, FIRE_RUN = 1, FIRE_COOLDOWN = 2 };
static FireState lastNextionFireState = FIRE_IDLE;
static bool nextionFireStateInit = false;
static bool lastNextionCandleOn = false;
static bool nextionCandleInit = false;

const unsigned long NEXTION_STATUS_PERIOD_MS = 2000;  // rafraîchissement du bloc d'état
const unsigned long NEXTION_RESYNC_PERIOD_MS = 60000; // renvoi périodique état feu/candelette/mode
const uint16_t NX_COLOR_TEXT  = 54938; // gris clair (couleur des libellés de page0)
const uint16_t NX_COLOR_WARN  = 64512; // orange : défaut détecté à l'arrêt
const uint16_t NX_COLOR_ALARM = 63488; // rouge : ALARM
static unsigned long nextionStatusLastMs = 0;
static unsigned long nextionResyncLastMs = 0;
static uint16_t nextionStatusCrc = 0;
static uint16_t nextionStatusColor = 0;
static bool nextionSyncPending = false;

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

#if DEBUG_RTCSTATS
// --------------------------------------------------------------------
// PATCH TEST RTC: instrumentation rtcReadOk() / I2C load
// --------------------------------------------------------------------
static volatile uint32_t g_rtc_calls = 0;
static volatile uint32_t g_rtc_ok    = 0;
static volatile uint32_t g_rtc_fail  = 0;

// "Burst" = appels rapprochés (symptôme d'un appel en boucle très rapide)
static uint32_t g_rtc_last_us   = 0;
static uint32_t g_rtc_burst_cur = 0;
static uint32_t g_rtc_burst_max = 0;
static const uint32_t RTC_BURST_US = 2000; // 2 ms

// Stats 1 Hz
static uint32_t g_rtc_calls_last = 0;
static uint32_t g_rtc_ok_last    = 0;
static uint32_t g_rtc_fail_last  = 0;

static uint32_t g_loop_count = 0;
static uint32_t g_loop_last  = 0;
#endif

// --------------------------------------------------------------------
// PATCH FIX RTC: cache + throttle (ONE read each RTC_POLL_MS)
// --------------------------------------------------------------------
static unsigned long g_rtcPollLastMs = 0;
static bool          g_rtcCachedOk   = false;
static const unsigned long RTC_POLL_MS = 1000; // 1 Hz recommandé

// --------------------------------------------------------------------
// PROTOS
// --------------------------------------------------------------------
const __FlashStringHelper* modeToStr(ControlMode m);
const __FlashStringHelper* pwrToStr(PowerLevel p);
const __FlashStringHelper* alarmToStr(AlarmCode a);
void readAllSensorsOnce();
void checkSafeties(unsigned long now);
void logAutoEventIfChanged(unsigned long nowMs);
void printSnapshot();
void applyOutputs();
void boilerEnterRun(unsigned long now, const __FlashStringHelper* origin);
void boilerEnterCooldown(unsigned long now, const __FlashStringHelper* origin);
void triggerAlarmAndCooldown(AlarmCode code, unsigned long now);
void handleCommand(const String &lineIn);
void serviceNextion();
void nextionService(unsigned long now);
void nextionSyncAll();
void updateNextionFireState(bool force);
void updateNextionControlMode(bool force);
void updateNextionCandleState(bool force);
void updateNextionPowerLevel(bool force);
void updateNextionStatus(bool force);
void nextionSendDaySchedule(uint8_t day1to7);
void nextionSendValue(const char* id, uint8_t val);

// --------------------------------------------------------------------
// OUTILS CRC / EEPROM
// --------------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

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

void setDefaultSchedule() {
  for (int d = 0; d < 7; d++) {
    weekSched[d].start1 = 6 * 60 + 30;
    weekSched[d].end1   = 9 * 60;
    weekSched[d].start2 = 17 * 60 + 30;
    weekSched[d].end2   = 22 * 60 + 30;
  }
}

void setDefaultParams() {
  setSelectedPowerLevel(PWR_MED);
}

void eepromSaveAll() {
  EepromDataV3 ed;
  ed.magic = EEPROM_MAGIC;
  ed.ver   = EEPROM_VER;
  ed.mode  = (uint8_t)controlMode;
  ed.pwrLevel = (uint8_t)selectedPowerLevel;
  ed.reserved = 0;
  for (int i = 0; i < 7; i++) ed.sched[i] = weekSched[i];
  ed.pelletCyclePeriodMs = pelletCyclePeriodMs;
  ed.pelletOnDurationMs  = pelletOnDurationMs;
  ed.runFanVoltageV      = runFanVoltageV;

  ed.crc = 0;
  ed.crc = crc16_ccitt((const uint8_t*)&ed, sizeof(EepromDataV3) - sizeof(ed.crc));
  EEPROM.put(0, ed);
}

bool eepromLoadAny() {
  uint16_t magic = 0;
  uint8_t ver = 0;
  EEPROM.get(0, magic);
  EEPROM.get(sizeof(uint16_t), ver);
  if (magic != EEPROM_MAGIC) return false;

  if (ver == 3) {
    EepromDataV3 ed;
    EEPROM.get(0, ed);
    uint16_t crc = ed.crc;
    ed.crc = 0;
    uint16_t calc = crc16_ccitt((const uint8_t*)&ed, sizeof(EepromDataV3) - sizeof(ed.crc));
    if (crc != calc) return false;

    controlMode = (ControlMode)ed.mode;
    setSelectedPowerLevel((PowerLevel)ed.pwrLevel);

    for (int i = 0; i < 7; i++) weekSched[i] = ed.sched[i];
    pelletCyclePeriodMs = ed.pelletCyclePeriodMs;
    pelletOnDurationMs  = ed.pelletOnDurationMs;
    runFanVoltageV      = ed.runFanVoltageV;
    return true;
  }

  if (ver == 2) {
    EepromDataV2 ed;
    EEPROM.get(0, ed);
    uint16_t crc = ed.crc;
    ed.crc = 0;
    uint16_t calc = crc16_ccitt((const uint8_t*)&ed, sizeof(EepromDataV2) - sizeof(ed.crc));
    if (crc != calc) return false;

    controlMode = (ControlMode)ed.mode;
    setSelectedPowerLevel(PWR_MED); // défaut en migration
    for (int i = 0; i < 7; i++) weekSched[i] = ed.sched[i];
    pelletCyclePeriodMs = ed.pelletCyclePeriodMs;
    pelletOnDurationMs  = ed.pelletOnDurationMs;
    runFanVoltageV      = ed.runFanVoltageV;

    eepromSaveAll(); // upgrade -> V3
    return true;
  }

  if (ver == 1) {
    EepromDataV1 ed;
    EEPROM.get(0, ed);
    uint16_t crc = ed.crc;
    ed.crc = 0;
    uint16_t calc = crc16_ccitt((const uint8_t*)&ed, sizeof(EepromDataV1) - sizeof(ed.crc));
    if (crc != calc) return false;

    controlMode = (ControlMode)ed.mode;
    for (int i = 0; i < 7; i++) weekSched[i] = ed.sched[i];
    setDefaultParams();
    eepromSaveAll();
    return true;
  }

  return false;
}

// Valeurs EEPROM hors bornes (CRC OK mais ancienne version / bug) => corrigées
bool sanitizeLoadedConfig() {
  bool fixed = false;
  if ((uint8_t)controlMode > MODE_AUTO) { controlMode = MODE_MANUAL; fixed = true; }
  for (int d = 0; d < 7; d++) {
    DaySchedule &ds = weekSched[d];
    if (ds.start1 > 1440 || ds.end1 > 1440 || ds.start2 > 1440 || ds.end2 > 1440) {
      setDefaultSchedule();
      fixed = true;
      break;
    }
  }
  return fixed;
}

// --------------------------------------------------------------------
// RTC helpers
// --------------------------------------------------------------------
#if DEBUG_RTCSTATS
static void rtcStatsOnCall() {
  g_rtc_calls++;

  const uint32_t now_us = micros();
  const uint32_t dt = now_us - g_rtc_last_us;
  g_rtc_last_us = now_us;

  if (dt <= RTC_BURST_US) {
    g_rtc_burst_cur++;
  } else {
    if (g_rtc_burst_cur > g_rtc_burst_max) g_rtc_burst_max = g_rtc_burst_cur;
    g_rtc_burst_cur = 0;
  }
}
#endif

bool rtcReadOk() {
#if DEBUG_RTCSTATS
  rtcStatsOnCall();
  bool ok = RTC.read();
  if (ok) g_rtc_ok++; else g_rtc_fail++;
  return ok;
#else
  return RTC.read();
#endif
}

// Throttled RTC poll (cache)
static void rtcPollService(unsigned long nowMs) {
  if (nowMs - g_rtcPollLastMs < RTC_POLL_MS) return;
  g_rtcPollLastMs = nowMs;
  g_rtcCachedOk = rtcReadOk();
}

// Sakamoto: retourne 1..7 = Lun..Dim
uint8_t dowFromDate(int y, int m, int d) {
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y -= 1;
  int w = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7; // 0=Dim..6=Sam
  if (w == 0) return 7;   // Dim
  return (uint8_t)w;      // Lun=1..Sam=6
}

uint8_t dowFromRTC() {
  return dowFromDate(RTC.getYear(), RTC.getMonth(), RTC.getMonthDay());
}

uint16_t minuteOfDayFromRTC() {
  return (uint16_t)(RTC.getHour() * 60 + RTC.getMinute());
}

static uint8_t daysInMonth(int y, int m) {
  static const uint8_t dm[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
  return dm[m - 1];
}

#if DEBUG_RTCSTATS
// Impression 1 fois/seconde (appelée depuis tick 1s)
static void rtcStatsPrint1Hz(unsigned long nowMs) {
  (void)nowMs;

  const uint32_t calls = g_rtc_calls;
  const uint32_t ok    = g_rtc_ok;
  const uint32_t fail  = g_rtc_fail;

  const uint32_t rtc_cps = calls - g_rtc_calls_last;
  const uint32_t ok_d    = ok    - g_rtc_ok_last;
  const uint32_t fail_d  = fail  - g_rtc_fail_last;

  g_rtc_calls_last = calls;
  g_rtc_ok_last    = ok;
  g_rtc_fail_last  = fail;

  const uint32_t loop_hz = g_loop_count - g_loop_last;
  g_loop_last = g_loop_count;

  const uint32_t burst_max = g_rtc_burst_max;
  g_rtc_burst_max = 0;

  Serial.print(F("[RTCSTATS] mode="));
  Serial.print(modeToStr(controlMode));
  Serial.print(F(" rtc_cps="));
  Serial.print(rtc_cps);
  Serial.print(F(" ok/s="));
  Serial.print(ok_d);
  Serial.print(F(" fail/s="));
  Serial.print(fail_d);
  Serial.print(F(" burst_max="));
  Serial.print(burst_max);
  Serial.print(F(" loop_hz="));
  Serial.println(loop_hz);
}
#endif

// --------------------------------------------------------------------
// PLANNING
// --------------------------------------------------------------------
bool inRange(uint16_t nowMin, uint16_t startMin, uint16_t endMin) {
  if (startMin == endMin) return false;
  if (startMin > 1440) startMin = 1440;
  if (endMin > 1440) endMin = 1440;
  if (startMin < endMin) return (nowMin >= startMin && nowMin < endMin);
  return (nowMin >= startMin || nowMin < endMin); // traverse minuit
}

bool scheduleDemand(uint8_t dow_1to7, uint16_t nowMin) {
  if (dow_1to7 < 1 || dow_1to7 > 7) return false;
  DaySchedule &ds = weekSched[dow_1to7 - 1];
  return inRange(nowMin, ds.start1, ds.end1) || inRange(nowMin, ds.start2, ds.end2);
}

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
// Sorties & MAX6675
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
// Commandes série
// --------------------------------------------------------------------
bool manualAllowed() { return (!boilerOn && !stopCooldownActive && !alarmActive); }

static bool startsWithNoCase(const String& s, const char* prefix) {
  unsigned int n = strlen(prefix);
  if (s.length() < n) return false;
  for (unsigned int i = 0; i < n; i++) {
    char a = s.charAt(i), b = prefix[i];
    if (a >= 'a' && a <= 'z') a -= 32;
    if (b >= 'a' && b <= 'z') b -= 32;
    if (a != b) return false;
  }
  return true;
}

static int parseHHMM_toMin(const String& hhmm) {
  int colon = hhmm.indexOf(':');
  if (colon < 0) return -1;
  int hh = hhmm.substring(0, colon).toInt();
  int mm = hhmm.substring(colon + 1).toInt();
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return -1;
  return hh * 60 + mm;
}

void handleCommand(const String &lineIn) {
  String line = lineIn;
  line.trim();
  if (line.length() == 0) return;

  unsigned long now = millis();

  if (startsWithNoCase(line, "TIME")) { printTimeLine(); return; }

  if (startsWithNoCase(line, "COOLDOWN")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: COOLDOWN ON|OFF")); return; }
    String v = line.substring(sp + 1); v.trim(); v.toUpperCase();
    if (v == "ON" || v == "1") forceCooldownOn(now);
    else if (v == "OFF" || v == "0") forceCooldownOff();
    else Serial.println(F("ERR: COOLDOWN ON|OFF"));
    return;
  }

  if (startsWithNoCase(line, "POWER")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: POWER LOW|MED|HIGH")); return; }
    String v = line.substring(sp + 1); v.trim(); v.toUpperCase();
    PowerLevel lvl;
    if (v == "LOW") lvl = PWR_LOW;
    else if (v == "MED" || v == "MEDIUM") lvl = PWR_MED;
    else if (v == "HIGH") lvl = PWR_HIGH;
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

  if (startsWithNoCase(line, "SETTIME")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: SETTIME format.")); return; }
    String rest = line.substring(sp + 1); rest.trim();
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) { Serial.println(F("ERR: SETTIME format.")); return; }

    String d = rest.substring(0, sp2);
    String t = rest.substring(sp2 + 1);

    int d1 = d.indexOf('-'), d2 = d.lastIndexOf('-');
    int t1 = t.indexOf(':'), t2 = t.lastIndexOf(':');
    if (d1 < 0 || d2 <= d1 || t1 < 0 || t2 <= t1) { Serial.println(F("ERR: SETTIME format.")); return; }

    int Y  = d.substring(0, d1).toInt();
    int M  = d.substring(d1 + 1, d2).toInt();
    int DD = d.substring(d2 + 1).toInt();

    int hh = t.substring(0, t1).toInt();
    int mm = t.substring(t1 + 1, t2).toInt();
    int ss = t.substring(t2 + 1).toInt();

    if (Y < 2000 || Y > 2099 || M < 1 || M > 12 || DD < 1 || DD > daysInMonth(Y, M) ||
        hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) {
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

  if (startsWithNoCase(line, "MODE")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: MODE OFF|MANUAL|AUTO")); return; }
    String m = line.substring(sp + 1); m.trim(); m.toUpperCase();

    ControlMode newMode;
    if (m == "OFF") newMode = MODE_OFF;
    else if (m == "MANUAL") newMode = MODE_MANUAL;
    else if (m == "AUTO") newMode = MODE_AUTO;
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

  if (startsWithNoCase(line, "LISTSCHED")) { printSchedule(); return; }

  if (startsWithNoCase(line, "DEFAULTSCHED")) {
    setDefaultSchedule();
    eepromSaveAll();
    Serial.println(F("OK: default schedule applied + saved."));
    printSchedule();
    return;
  }

  if (startsWithNoCase(line, "SAVE")) {
    if (!validatePowerParams(true)) return;
    eepromSaveAll();
    Serial.println(F("OK: EEPROM saved (mode+sched+params)."));
    return;
  }

  if (startsWithNoCase(line, "SETDAY")) {
    int p1 = line.indexOf(' ');
    if (p1 < 0) { Serial.println(F("ERR: SETDAY D HH:MM HH:MM HH:MM HH:MM")); return; }
    String rest = line.substring(p1 + 1); rest.trim();

    String tok[5];
    int ti = 0;
    while (rest.length() && ti < 5) {
      int sp = rest.indexOf(' ');
      if (sp < 0) { tok[ti++] = rest; rest = ""; break; }
      tok[ti++] = rest.substring(0, sp);
      rest = rest.substring(sp + 1);
      rest.trim();
    }
    if (ti != 5) { Serial.println(F("ERR: SETDAY D HH:MM HH:MM HH:MM HH:MM")); return; }

    int D = tok[0].toInt();
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

  if (startsWithNoCase(line, "PARAMS")) { printParams(); return; }

  if (startsWithNoCase(line, "DEFAULTPARAMS")) {
    setDefaultParams();
    eepromSaveAll();
    Serial.println(F("OK: default power params applied + saved."));
    printParams();
    return;
  }

  if (startsWithNoCase(line, "PP")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: PP <ms>")); return; }
    long v = line.substring(sp + 1).toInt();
    if (!powerParamsValid(v, (long)pelletOnDurationMs, true)) return;
    pelletCyclePeriodMs = (unsigned long)v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  if (startsWithNoCase(line, "PON")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: PON <ms>")); return; }
    long v = line.substring(sp + 1).toInt();
    if (!powerParamsValid((long)pelletCyclePeriodMs, v, true)) return;
    pelletOnDurationMs = (unsigned long)v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  if (startsWithNoCase(line, "FV")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: FV <volts>")); return; }
    float v = line.substring(sp + 1).toFloat();
    if (v < 0.0f) v = 0.0f;
    if (v > 10.0f) v = 10.0f;
    runFanVoltageV = v;
    Serial.println(F("OK (SAVE pour garder apres reboot)"));
    return;
  }

  char cmd = line.charAt(0);
  switch (cmd) {
    case 'h': case 'H': printHelp(); break;

    case 'r': case 'R':
      readAllSensorsOnce();
      checkSafeties(now);
      printSnapshot();
      break;

    case 's': case 'S':
      if (line.length() >= 2) {
        char v = line.charAt(1);
        if (v == '0') autoSnapshotEnabled = false;
        else if (v == '1') autoSnapshotEnabled = true;
        Serial.print(F("Auto snapshot -> "));
        Serial.println(autoSnapshotEnabled ? F("ON") : F("OFF"));
      }
      break;

    case 'k': case 'K':
      if (line.length() >= 2) {
        char v = line.charAt(1);
        if (v == '1') forceCooldownOn(now);
        else if (v == '0') forceCooldownOff();
      }
      break;

    case 'a': case 'A':
      if (line.length() >= 2 && line.charAt(1) == '0') {
        clearAlarm();
        Serial.println(F("ALARM acquittee."));
      }
      break;

    case 'i': case 'I':
      if (line.length() >= 2 && line.charAt(1) == '0') forceIdle(now);
      break;

    case 'w': case 'W':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line.length() >= 2) {
        statePellet = (line.charAt(1) == '1');
        applyOutputs();
        Serial.print(F("Vis sans fin -> ")); Serial.println(statePellet ? F("ON") : F("OFF"));
      }
      break;

    case 'p': case 'P':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line.length() >= 2) {
        statePump = (line.charAt(1) == '1');
        applyOutputs();
        Serial.print(F("Pompe -> ")); Serial.println(statePump ? F("ON") : F("OFF"));
      }
      break;

    case 'c': case 'C':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line.length() >= 2) {
        stateCandle = (line.charAt(1) == '1');
        if (stateCandle) candleManualSinceMs = now;
        applyOutputs();
        Serial.print(F("Candelette -> ")); Serial.println(stateCandle ? F("ON (10 min max)") : F("OFF"));
      }
      break;

    case 'f': case 'F':
      if (!manualAllowed()) { Serial.println(F("Manuel refuse (RUN/COOLDOWN/ALARM).")); break; }
      if (controlMode == MODE_AUTO) { Serial.println(F("Manuel refuse (MODE=AUTO).")); break; }
      if (line.length() >= 2) {
        float v = line.substring(1).toFloat();
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
      if (line.length() >= 2) {
        char v = line.charAt(1);
        if (v == '1') boilerBasicStart(now);
        else if (v == '0') boilerBasicStop(now);
      }
      break;

    default:
      Serial.print(F("Commande inconnue: "));
      Serial.println(line);
      printHelp();
      break;
  }
}

void serviceSerial() {
  static String buffer;
  static bool overflow = false;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (overflow) Serial.println(F("ERR: ligne trop longue, ignoree."));
      else handleCommand(buffer);
      buffer = "";
      overflow = false;
    } else if (buffer.length() < SERIAL_LINE_MAX) {
      buffer += c;
    } else {
      overflow = true;
    }
  }
}

// --------------------------------------------------------------------
// NEXTION (Serial1) : commandes simples START/STOP/AUTO/MANUAL/POWER
// --------------------------------------------------------------------
// START/STOP depuis l'écran : on passe d'abord en MANUAL (comme le bouton le fait déjà)
static void nextionEnsureManual() {
  if (controlMode == MODE_AUTO) handleCommand("MODE MANUAL");
}

void handleNextionCommand(const String &lineIn) {
  String line = lineIn;
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (line == "START") { nextionEnsureManual(); handleCommand("b1"); return; }
  if (line == "STOP")  { nextionEnsureManual(); handleCommand("b0"); return; }

  if (line == "MODE_AUTO" || line == "AUTO")   { handleCommand("MODE AUTO"); return; }
  if (line == "MODE_MANUAL" || line == "MANUAL") { handleCommand("MODE MANUAL"); return; }
  if (line == "MODE_OFF" || line == "OFF") { handleCommand("MODE OFF"); return; }

  if (line == "PWR_LOW" || line == "POWER_LOW")   { handleCommand("POWER LOW"); return; }
  if (line == "PWR_MED" || line == "POWER_MED")   { handleCommand("POWER MED"); return; }
  if (line == "PWR_HIGH" || line == "POWER_HIGH") { handleCommand("POWER HIGH"); return; }
  if (line == "FORCE_IDLE" || line == "IDLE") { handleCommand("i0"); return; }
  if (line == "COOLDOWN_ON")  { handleCommand("k1"); return; }
  if (line == "COOLDOWN_OFF") { handleCommand("k0"); return; }

  if (startsWithNoCase(line, "SETDAY")) { handleCommand(line); return; }
  if (startsWithNoCase(line, "LOAD_")) {
    int day = line.substring(5).toInt();
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
  static String buffer;
  static uint8_t ffCount = 0;

  while (NEXTION_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)NEXTION_SERIAL.read();

    if (b == 0xFF) {
      ffCount++;
      if (ffCount >= 3) {
        if (buffer.length() > 0) {
          uint8_t first = (uint8_t)buffer.charAt(0);
          if (first < 0x20 || first >= 0x80) handleNextionReturnCode(first);
          else handleNextionCommand(buffer);
          buffer = "";
        }
        ffCount = 0;
      }
      continue;
    }

    if (ffCount > 0) ffCount = 0;

    if (b == '\r' || b == '\n') {
      if (buffer.length() > 0) {
        handleNextionCommand(buffer);
        buffer = "";
      }
      continue;
    }

    if (buffer.length() < SERIAL_LINE_MAX) buffer += (char)b;
  }
#endif
}

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

  // Init logger EVT
  lastDemandLogged = programDemandOn;
  lastBoilerOnLogged = boilerOn;
  lastCooldownLogged = stopCooldownActive;
  lastRtcOkLogged = true;

#if DEBUG_RTCSTATS
  // Init RTCSTATS
  g_rtc_calls = g_rtc_ok = g_rtc_fail = 0;
  g_rtc_calls_last = g_rtc_ok_last = g_rtc_fail_last = 0;
  g_rtc_last_us = micros();
  g_rtc_burst_cur = g_rtc_burst_max = 0;
  g_loop_count = g_loop_last = 0;
#endif

  // Init RTC cache
  g_rtcPollLastMs = 0;
  g_rtcCachedOk = false;

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
  g_loop_count++;
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
