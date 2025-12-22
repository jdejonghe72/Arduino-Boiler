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
// NO DEPRESSION : ignoré pour les sécurités (diagnostic seulement)
//
// PATCH 2025-12-19 (NEW):
// 1) Force COOLDOWN (manuel + auto) via commande "K1/K0" ou "COOLDOWN ON/OFF"
// 2) 3 niveaux de puissance: LOW/MED/HIGH via "POWER LOW|MED|HIGH"
//    - par défaut: LOW=PP10000, MED=8000, HIGH=6000 ; PON=1000 ; FV=6.0
//    - PP/PON/FV restent possibles (override) + persistants EEPROM
// 3) Watchdog AVR (WDT) + log cause reset
//    + Ajout option compile-time pour désactiver facilement le WDT:
//      #define ENABLE_WDT 0
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <EEPROM.h>
#include <RTC.h>

// --------------------------------------------------------------------
// Dual-serial helper: mirrors all output to both PC Serial and Serial1
// and merges input streams so commands can arrive from either side.
// --------------------------------------------------------------------
HardwareSerial &pcSerial = Serial;
HardwareSerial &hmiSerial = Serial1;

class DualSerial : public Stream {
public:
  DualSerial(HardwareSerial &primary, HardwareSerial &secondary)
  : _pc(primary), _hmi(secondary) {}

  void begin(unsigned long baud) {
    _pc.begin(baud);
    _hmi.begin(baud);
  }

  virtual int available() override {
    return _pc.available() + _hmi.available();
  }

  virtual int read() override {
    if (_pc.available()) return _pc.read();
    return _hmi.available() ? _hmi.read() : -1;
  }

  virtual int peek() override {
    if (_pc.available()) return _pc.peek();
    return _hmi.available() ? _hmi.peek() : -1;
  }

  virtual void flush() override {
    _pc.flush();
    _hmi.flush();
  }

  operator bool() { return (bool)_pc; }

  virtual size_t write(uint8_t b) override {
    size_t n1 = _pc.write(b);
    size_t n2 = _hmi.write(b);
    return (n1 == 0) ? n2 : n1;  // best-effort
  }

  using Print::write;

private:
  HardwareSerial &_pc;
  HardwareSerial &_hmi;
};

DualSerial SerialDual(pcSerial, hmiSerial);
#define Serial SerialDual

// --------------------------------------------------------------------
// WATCHDOG enable switch
// --------------------------------------------------------------------
#define ENABLE_WDT 1  // 0 = watchdog désactivé, 1 = watchdog activé

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
const unsigned long IGNITION_DURATION_MS   = 10UL * 60UL * 1000UL;
const unsigned long PUMP_START_DELAY_MS    = 1UL  * 60UL * 1000UL;
const unsigned long STOP_COOLDOWN_MS       = 10UL * 60UL * 1000UL;

bool boilerOn = false;
bool stopCooldownActive = false;
unsigned long boilerStartMs = 0;
unsigned long stopStartMs   = 0;

// Repère de start (NO FIRE + diag NO DEP)
unsigned long lastStartMs = 0;

// --------------------------------------------------------------------
// PUISSANCE : 3 niveaux + paramètres actifs (override possible)
// --------------------------------------------------------------------
enum PowerLevel : uint8_t { PWR_LOW = 0, PWR_MED = 1, PWR_HIGH = 2 };
PowerLevel powerLevel = PWR_MED;

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

// Paramètres "actifs" utilisés par la chaudière (peuvent être modifiés via PP/PON/FV)
unsigned long pelletCyclePeriodMs = 8000UL;
unsigned long pelletOnDurationMs  = 1000UL;
float         runFanVoltageV      = 6.0f;

// Appliquer les valeurs du profil sélectionné aux paramètres actifs
void applyPowerProfile(PowerLevel lvl) {
  if ((uint8_t)lvl > 2) lvl = PWR_MED;
  powerLevel = lvl;
  pelletCyclePeriodMs = kProfiles[(uint8_t)lvl].ppMs;
  pelletOnDurationMs  = kProfiles[(uint8_t)lvl].ponMs;
  runFanVoltageV      = kProfiles[(uint8_t)lvl].fvV;
}

// --------------------------------------------------------------------
// FORCE COOLDOWN (manuel + auto)
// --------------------------------------------------------------------
bool forceCooldownActive = false; // si true => on force l'état COOLDOWN, même en AUTO

// --------------------------------------------------------------------
// PATCH DEBUG + FAILSAFE VIS SANS FIN
// --------------------------------------------------------------------
const unsigned long PELLET_MAX_CONTINUOUS_ON_MS = 3000UL; // 3 s (sécurité debug)

bool lastPelletLogged = false;
unsigned long pelletOnSinceMs = 0;
unsigned long pelletLongestOnMs = 0;

// --------------------------------------------------------------------
// SECURITES
// --------------------------------------------------------------------
const int AC_RAW_THRESHOLD = 300; // AC présent => RAW < 300 ; AC absent => RAW >= 300

const unsigned long NO_DEP_GRACE_MS  = 60UL * 1000UL;        // diag seulement
const unsigned long NO_FIRE_DELAY_MS = 10UL * 60UL * 1000UL; // actif en RUN

const int RAW_WATER_SENSOR_OVERHEAT = 55;   // I0.10 : RAW < 55
const int RAW_PRESSURE_TOO_HIGH     = 200;  // I0.11 : RAW > 200
const int RAW_PRESSURE_TOO_LOW      = 80;   // I0.11 : RAW < 80

const float FLUE_NO_FIRE_MIN_C      = 60.0f;

bool alarmActive = false;
char alarmText[48] = {0};

// Derniers RAW
int rawNoDep = 0, rawPelletsOH = 0, rawWaterOH = 0, rawNTC = 0, rawPressure = 0, rawDep = 0;

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
void readAllSensorsOnce();
void checkSafeties(unsigned long now);
void logAutoEventIfChanged(unsigned long nowMs);
void printSnapshot();
void applyOutputs();
void forceCooldown(unsigned long now);

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

void setDefaultSchedule() {
  for (int d = 0; d < 7; d++) {
    weekSched[d].start1 = 6 * 60 + 30;
    weekSched[d].end1   = 9 * 60;
    weekSched[d].start2 = 17 * 60 + 30;
    weekSched[d].end2   = 22 * 60 + 30;
  }
}

void setDefaultParams() {
  applyPowerProfile(PWR_MED);
}

void eepromSaveAll() {
  EepromDataV3 ed;
  ed.magic = EEPROM_MAGIC;
  ed.ver   = EEPROM_VER;
  ed.mode  = (uint8_t)controlMode;
  ed.pwrLevel = (uint8_t)powerLevel;
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
    powerLevel  = (PowerLevel)ed.pwrLevel;
    if ((uint8_t)powerLevel > 2) powerLevel = PWR_MED;

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
    powerLevel  = PWR_MED; // défaut en migration
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

// --------------------------------------------------------------------
// RTC helpers
// --------------------------------------------------------------------
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

bool rtcReadOk() {
  rtcStatsOnCall();
  bool ok = RTC.read();
  if (ok) g_rtc_ok++; else g_rtc_fail++;
  return ok;
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

    if (PELLET_MAX_CONTINUOUS_ON_MS > 0 && onMs > PELLET_MAX_CONTINUOUS_ON_MS) {
      Serial.print(F("!!! FAILSAFE: pellet ON too long ("));
      Serial.print(onMs);
      Serial.println(F(" ms) -> FORCE OFF"));

      statePellet = false;
      applyOutputs();
    }
  }
}

// --------------------------------------------------------------------
// Alarme / diagnostic
// --------------------------------------------------------------------
void forceCooldown(unsigned long now) {
  boilerOn = false;
  stopCooldownActive = true;
  stopStartMs = now;
  statePellet = false;
  stateCandle = false;
  applyOutputs();
}

void triggerAlarmAndCooldown(const char* msg, unsigned long now) {
  if (alarmActive) return;
  alarmActive = true;
  strncpy(alarmText, msg, sizeof(alarmText) - 1);
  alarmText[sizeof(alarmText) - 1] = '\0';

  Serial.print(F("!!! ALARM: "));
  Serial.println(alarmText);

  if (!stopCooldownActive) forceCooldown(now);
}

void diagPrint(unsigned long now, const __FlashStringHelper* label, const char* detail) {
  if (now - lastDiagMs < DIAG_PERIOD_MS) return;
  lastDiagMs = now;
  Serial.print(F("[DIAG] "));
  Serial.print(label);
  if (detail && detail[0]) { Serial.print(F(" : ")); Serial.print(detail); }
  Serial.println();
}

void checkSafeties(unsigned long now) {
  if (!boilerOn && !stopCooldownActive) {
    if (acAbsentFromRaw(rawPelletsOH)) diagPrint(now, F("PELLETS OVERHEAT input"), "AC ABSENT");
    if (acAbsentFromRaw(rawWaterOH))   diagPrint(now, F("WATER OVERHEAT input"),   "AC ABSENT");

    if (rawNTC > 0 && rawNTC < RAW_WATER_SENSOR_OVERHEAT) diagPrint(now, F("WATER SENSOR OVERHEAT"), "NTC RAW low");
    if (rawPressure > RAW_PRESSURE_TOO_HIGH)             diagPrint(now, F("WATER PRESSURE HIGH"), "RAW high");
    if (rawPressure < RAW_PRESSURE_TOO_LOW)              diagPrint(now, F("WATER PRESSURE LOW"),  "RAW low");

    if (acAbsentFromRaw(rawNoDep)) diagPrint(now, F("NO DEPRESSION"), "input open");
    return;
  }

  if (alarmActive) return;

  bool pelletsFault = acAbsentFromRaw(rawPelletsOH);
  bool waterFault   = acAbsentFromRaw(rawWaterOH);
  if (pelletsFault || waterFault) {
    if (pelletsFault && waterFault) triggerAlarmAndCooldown("PELLETS + WATER OVERHEAT", now);
    else if (pelletsFault)          triggerAlarmAndCooldown("PELLETS OVERHEAT", now);
    else                            triggerAlarmAndCooldown("WATER OVERHEAT", now);
    return;
  }

  if (rawNTC > 0 && rawNTC < RAW_WATER_SENSOR_OVERHEAT) {
    triggerAlarmAndCooldown("WATER SENSOR OVERHEAT", now);
    return;
  }

  if (rawPressure > RAW_PRESSURE_TOO_HIGH) { triggerAlarmAndCooldown("WATER PRESSURE TOO HIGH", now); return; }
  if (rawPressure < RAW_PRESSURE_TOO_LOW)  { triggerAlarmAndCooldown("WATER PRESSURE TOO LOW",  now); return; }

  if (boilerOn && (lastStartMs > 0) && (now - lastStartMs >= NO_DEP_GRACE_MS)) {
    if (acAbsentFromRaw(rawNoDep)) diagPrint(now, F("NO DEPRESSION (IGNORED)"), "running");
  }

  if (boilerOn && (lastStartMs > 0) && (now - lastStartMs >= NO_FIRE_DELAY_MS)) {
    if (!isnan(lastTempFumeesC) && lastTempFumeesC < FLUE_NO_FIRE_MIN_C) {
      triggerAlarmAndCooldown("ALARM NO FIRE", now);
      return;
    }
  }
}

// --------------------------------------------------------------------
// AUTO control (utilise cache RTC)
// --------------------------------------------------------------------
void autoControlService(unsigned long nowMs) {
  if (controlMode != MODE_AUTO) return;
  if (alarmActive) return;

  // Force COOLDOWN bloque toute tentative AUTO de démarrer
  if (forceCooldownActive) return;

  if (!g_rtcCachedOk) return;

  uint8_t dow = dowFromRTC();
  uint16_t mod = minuteOfDayFromRTC();
  bool demand = scheduleDemand(dow, mod);

  if (demand != lastDemandSeen) {
    lastDemandSeen = demand;
    lastDemandChangeMs = nowMs;
  }
  if (nowMs - lastDemandChangeMs < AUTO_DEMAND_STABLE_MS) {
    programDemandOn = demand;
    return;
  }

  programDemandOn = demand;

  if (nowMs - lastAutoCommandMs < AUTO_MIN_COMMAND_GAP_MS) return;

  if (demand) {
    if (!boilerOn && !stopCooldownActive) {
      boilerOn = true;
      boilerStartMs = nowMs;
      lastStartMs = nowMs;

      stateCandle = true;
      fanVoltageV = runFanVoltageV;
      fanPwm      = voltsToPwm(fanVoltageV);
      statePellet = false;
      statePump   = false;

      applyOutputs();
      Serial.println(F("START chaudiere basique (AUTO)."));

      lastAutoCommandMs = nowMs;
    }
  } else {
    if (boilerOn && !stopCooldownActive) {
      boilerOn = false;
      stopCooldownActive = true;
      stopStartMs = nowMs;

      statePellet = false;
      stateCandle = false;
      applyOutputs();
      Serial.println(F("STOP chaudiere basique (AUTO): cooldown."));

      lastAutoCommandMs = nowMs;
    }
  }
}

// --------------------------------------------------------------------
// Chaudière basique MANUAL (START/STOP/SERVICE)
// --------------------------------------------------------------------
void boilerBasicStart(unsigned long now) {
  if (alarmActive) {
    Serial.print(F("START refuse: ALARM active (a0 ou i0). Cause="));
    Serial.println(alarmText);
    return;
  }
  if (forceCooldownActive) {
    Serial.println(F("START refuse: FORCE COOLDOWN actif (K0 pour relacher)."));
    return;
  }
  if (boilerOn || stopCooldownActive) {
    Serial.println(F("START refuse: deja RUN ou COOLDOWN."));
    return;
  }

  boilerOn = true;
  boilerStartMs = now;
  lastStartMs = now;

  stateCandle = true;
  fanVoltageV = runFanVoltageV;
  fanPwm      = voltsToPwm(fanVoltageV);
  statePellet = false;
  statePump   = false;

  applyOutputs();
  Serial.println(F("START chaudiere basique."));
}

void boilerBasicStop(unsigned long now) {
  if (!boilerOn && !stopCooldownActive) {
    Serial.println(F("STOP: deja arretee."));
    return;
  }
  if (stopCooldownActive) {
    Serial.println(F("STOP: cooldown deja en cours."));
    return;
  }

  boilerOn = false;
  stopCooldownActive = true;
  stopStartMs = now;

  statePellet = false;
  stateCandle = false;

  applyOutputs();
  Serial.println(F("STOP: debut cooldown 10 min."));
}

void boilerBasicService(unsigned long now) {
  if (boilerOn) {
    unsigned long elapsed = now - boilerStartMs;

    fanVoltageV = runFanVoltageV;
    fanPwm      = voltsToPwm(fanVoltageV);

    stateCandle = (elapsed < IGNITION_DURATION_MS);

    unsigned long period = (pelletCyclePeriodMs < 1UL) ? 1UL : pelletCyclePeriodMs;
    unsigned long phase  = elapsed % period;
    statePellet = (phase < pelletOnDurationMs);

    statePump = (elapsed >= PUMP_START_DELAY_MS);

    applyOutputs();
  }

  if (stopCooldownActive) {
    unsigned long sinceStop = now - stopStartMs;

    statePellet = false;
    stateCandle = false;
    statePump   = true;
    fanVoltageV = runFanVoltageV;
    fanPwm      = voltsToPwm(fanVoltageV);

    if (sinceStop >= STOP_COOLDOWN_MS) {
      stopCooldownActive = false;
      boilerOn = false;

      statePellet = false;
      statePump   = false;
      stateCandle = false;
      fanVoltageV = 0.0f;
      fanPwm      = voltsToPwm(fanVoltageV);

      lastStartMs = 0;

      Serial.println(F("Cooldown termine: IDLE, sorties OFF."));
    }

    applyOutputs();
  }

  // Si FORCE COOLDOWN est actif, on garantit qu'on est au moins en COOLDOWN.
  if (forceCooldownActive && !stopCooldownActive) {
    forceCooldown(now);
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

  alarmActive = false;
  alarmText[0] = '\0';

  forceCooldownActive = false;

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
  forceCooldownActive = true;
  forceCooldown(now);
  Serial.println(F("FORCE COOLDOWN: actif (bloque AUTO start)."));
}

void forceCooldownOff() {
  forceCooldownActive = false;
  Serial.println(F("FORCE COOLDOWN: relache. (cooldown normal peut continuer si en cours)"));
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
    if (forceCooldownActive) Serial.print(F("  FORCE_COOLDOWN=1"));
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
    if (forceCooldownActive) Serial.print(F("  FORCE_COOLDOWN=1"));
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

void printParams() {
  Serial.println(F("=== PARAMS (power) ==="));
  Serial.print(F("LEVEL = ")); Serial.println(pwrToStr(powerLevel));
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
  Serial.print(F("  forceCooldown="));
  Serial.println(forceCooldownActive ? F("ON") : F("OFF"));

  Serial.print(F("Power: LEVEL=")); Serial.print(pwrToStr(powerLevel));
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
    Serial.println(alarmText);
  }

  Serial.println(F("============================"));
}

bool validatePowerParams(bool printErr) {
  if (pelletCyclePeriodMs < 1000UL || pelletCyclePeriodMs > 60000UL) {
    if (printErr) Serial.println(F("ERR: PP range 1000..60000"));
    return false;
  }
  if (pelletOnDurationMs < 50UL || pelletOnDurationMs > 5000UL) {
    if (printErr) Serial.println(F("ERR: PON range 50..5000"));
    return false;
  }
  if (pelletOnDurationMs >= pelletCyclePeriodMs) {
    if (printErr) Serial.println(F("ERR: need PON < PP"));
    return false;
  }
  if (runFanVoltageV < 0.0f) runFanVoltageV = 0.0f;
  if (runFanVoltageV > 10.0f) runFanVoltageV = 10.0f;
  return true;
}

void printHelp() {
  Serial.println(F("=== Commandes serie M-Duino ==="));
  Serial.println(F("h         : aide"));
  Serial.println(F("r         : snapshot immediat"));
  Serial.println(F("s0 / s1   : auto snapshot OFF/ON"));
  Serial.println(F("w0 / w1   : vis sans fin OFF/ON [manuel IDLE]"));
  Serial.println(F("p0 / p1   : pompe OFF/ON [manuel IDLE]"));
  Serial.println(F("c0 / c1   : candelette OFF/ON [manuel IDLE]"));
  Serial.println(F("fN        : ventilateur N volts (0..10) [manuel IDLE] ex f5.5"));
  Serial.println(F("b1        : START chaudiere basique (MANUAL uniquement)"));
  Serial.println(F("b0        : STOP chaudiere basique (MANUAL uniquement)"));
  Serial.println(F("a0        : acquitter/effacer ALARM"));
  Serial.println(F("i0        : FORCER IDLE"));
  Serial.println(F("k1 / k0   : FORCER COOLDOWN ON/OFF (manuel + auto)"));
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
  Serial.println(F("PON <ms>  (50..5000, PON<PP)"));
  Serial.println(F("FV <V>    (0..10)"));
  Serial.println(F("DEFAULTPARAMS"));
  Serial.println(F("================================"));
}

// --------------------------------------------------------------------
// Commandes série
// --------------------------------------------------------------------
bool manualAllowed() { return (!boilerOn && !stopCooldownActive && !alarmActive); }

static bool startsWithNoCase(const String& s, const char* prefix) {
  int n = strlen(prefix);
  if (s.length() < n) return false;
  for (int i = 0; i < n; i++) {
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
    PowerLevel lvl = powerLevel;
    if (v == "LOW") lvl = PWR_LOW;
    else if (v == "MED" || v == "MEDIUM") lvl = PWR_MED;
    else if (v == "HIGH") lvl = PWR_HIGH;
    else { Serial.println(F("ERR: POWER LOW|MED|HIGH")); return; }

    applyPowerProfile(lvl);
    if (!validatePowerParams(true)) return;
    eepromSaveAll();

    Serial.print(F("OK: POWER -> "));
    Serial.println(pwrToStr(powerLevel));
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

    if (m == "OFF") controlMode = MODE_OFF;
    else if (m == "MANUAL") controlMode = MODE_MANUAL;
    else if (m == "AUTO") controlMode = MODE_AUTO;
    else { Serial.println(F("ERR: MODE OFF|MANUAL|AUTO")); return; }

    eepromSaveAll();
    Serial.print(F("MODE -> ")); Serial.println(modeToStr(controlMode));

    if (controlMode == MODE_OFF) {
      if (boilerOn && !stopCooldownActive) boilerBasicStop(now);
    }
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
    if (v < 1000 || v > 60000) { Serial.println(F("ERR: PP range 1000..60000")); return; }
    pelletCyclePeriodMs = (unsigned long)v;
    if (!validatePowerParams(true)) return;
    Serial.println(F("OK"));
    return;
  }

  if (startsWithNoCase(line, "PON")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: PON <ms>")); return; }
    long v = line.substring(sp + 1).toInt();
    if (v < 50 || v > 5000) { Serial.println(F("ERR: PON range 50..5000")); return; }
    pelletOnDurationMs = (unsigned long)v;
    if (!validatePowerParams(true)) return;
    Serial.println(F("OK"));
    return;
  }

  if (startsWithNoCase(line, "FV")) {
    int sp = line.indexOf(' ');
    if (sp < 0) { Serial.println(F("ERR: FV <volts>")); return; }
    float v = line.substring(sp + 1).toFloat();
    if (v < 0.0f) v = 0.0f;
    if (v > 10.0f) v = 10.0f;
    runFanVoltageV = v;
    if (!validatePowerParams(true)) return;
    Serial.println(F("OK"));
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
        alarmActive = false;
        alarmText[0] = '\0';
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
        applyOutputs();
        Serial.print(F("Candelette -> ")); Serial.println(stateCandle ? F("ON") : F("OFF"));
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
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { handleCommand(buffer); buffer = ""; }
    else buffer += c;
  }
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

  // Évite blocage éternel si pas d'USB: max ~2s
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000UL)) {}

  Serial.println(F("=== M-Duino 38AR+ Boiler basic + RTC cache/throttle + RTCSTATS + POWER + FORCE_COOLDOWN + WDT ==="));
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

  alarmActive = false;
  alarmText[0] = '\0';

  forceCooldownActive = false;

  // Charger EEPROM
  if (!eepromLoadAny()) {
    controlMode = MODE_MANUAL;
    setDefaultSchedule();
    setDefaultParams();
    eepromSaveAll();
    Serial.println(F("EEPROM: no valid data -> defaults created."));
  } else {
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

  // Init RTCSTATS
  g_rtc_calls = g_rtc_ok = g_rtc_fail = 0;
  g_rtc_calls_last = g_rtc_ok_last = g_rtc_fail_last = 0;
  g_rtc_last_us = micros();
  g_rtc_burst_cur = g_rtc_burst_max = 0;
  g_loop_count = g_loop_last = 0;

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

  // Activer WDT en fin de setup (après initialisations)
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  watchdogEnable();
  watchdogKick();
#endif
}

void loop() {
  unsigned long now = millis();
  g_loop_count++;

  serviceSerial();

  // Mise à jour RTC (throttled) - UNE seule lecture périodique
  rtcPollService(now);

  // MODE OFF => arrêt propre si RUN
  if (controlMode == MODE_OFF) {
    if (boilerOn && !stopCooldownActive) boilerBasicStop(now);
  }

  // Si FORCE COOLDOWN actif, on force l'état (et donc stoppe RUN si besoin)
  if (forceCooldownActive && (boilerOn || !stopCooldownActive)) {
    forceCooldown(now);
  }

  // Pilotage AUTO (utilise cache)
  autoControlService(now);

  // Service chaudière
  boilerBasicService(now);

  // Patch: debug + failsafe vis
  pelletDebugAndFailsafe(now);

  // Tick 1s : capteurs + sécurités + debug events + snapshot + RTCSTATS
  if (now - lastTickMs >= SNAPSHOT_PERIOD_MS) {
    lastTickMs = now;

    readAllSensorsOnce();
    checkSafeties(now);

    logAutoEventIfChanged(now);

    rtcStatsPrint1Hz(now);

    if (autoSnapshotEnabled) printSnapshot();
  }

  // Watchdog feed: à la fin de loop quand tout s'est bien déroulé
#if defined(ARDUINO_ARCH_AVR) && ENABLE_WDT
  watchdogKick();
#endif
}
