// ============================================================================
// PLANNING : EEPROM, RTC, planning hebdo
// ============================================================================

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

void rtcStatsOnLoop() { g_loop_count++; }
#endif

// Cache RTC (une seule lecture toutes les RTC_POLL_MS)
static unsigned long g_rtcPollLastMs = 0;

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
void rtcPollService(unsigned long nowMs) {
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

uint8_t daysInMonth(int y, int m) {
  static const uint8_t dm[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
  return dm[m - 1];
}

#if DEBUG_RTCSTATS
// Impression 1 fois/seconde (appelée depuis tick 1s)
void rtcStatsPrint1Hz(unsigned long nowMs) {
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
