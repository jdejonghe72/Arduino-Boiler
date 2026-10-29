// Banc de simulation du sketch (temps simulé, entrées/sorties modélisées).
// Chaque scénario part d'un Arduino fraîchement démarré (processus séparé).
#include "Arduino.h"
#include "EEPROM.h"
#include "RTC.h"
#include <sys/wait.h>
#include <unistd.h>

unsigned long g_ms = 0;
Port Serial;
Port Serial1;
EEPROMClass EEPROM;
long g_rtcBaseSec = 6L * 3600; // 06:00:00
bool g_rtcOk = true;
RTCClass RTC;

// ---- modèle matériel ----
float g_flue = 25.0f;        // NAN = thermocouple ouvert
bool g_forceNoDep = false;   // pressostat ouvert même ventilateur ON
bool g_waterOH = false, g_pelletsOH = false;
int g_rawNTC = 216, g_rawPressure = 125;
int g_fanPwm = 0;
int g_pin[80] = {0};
long g_pelletToggles = 0;
static uint16_t g_sr = 0;
static int g_bit = 15;

void pinMode(int, int) {}
void digitalWrite(int pin, int v) {
  if (pin == R1_1 && g_pin[pin] != v) g_pelletToggles++;
  if (pin == 2 && v == LOW) { // MAX6675 : CS bas = chargement du registre
    if (std::isnan(g_flue)) g_sr = 0x0004; else g_sr = (uint16_t)((uint16_t)(g_flue / 0.25f) << 3);
    g_bit = 15;
  }
  if (pin == 52 && v == LOW && g_pin[52] == HIGH) g_bit--; // front descendant SCK
  g_pin[pin] = v;
}
int digitalRead(int pin) { if (pin == 50) return g_bit >= 0 ? (g_sr >> g_bit) & 1 : 0; return g_pin[pin]; }
int analogRead(int pin) {
  switch (pin) {
    case 54: return (g_fanPwm > 0 && !g_forceNoDep) ? 100 : 489; // pressostat dépression
    case 55: return g_pelletsOH ? 500 : 0;
    case 56: return g_waterOH ? 500 : 0;
    case 57: return g_rawNTC;
    case 58: return g_rawPressure;
    case 59: return 263;
  }
  return 0;
}
void analogWrite(int pin, int v) { if (pin == A1_0) g_fanPwm = v; }

#include "build/sketch_all.cpp"

// ---- outils de test ----
static int g_fail = 0, g_pass = 0;
#define CHECK(c, msg) do { if (c) { g_pass++; printf("  PASS %s\n", msg); } else { g_fail++; printf("  FAIL %s (ligne %d)\n", msg, __LINE__); } } while (0)
static void run(unsigned long ms, unsigned long step = 1) { for (unsigned long t = 0; t < ms; t += step) { g_ms += step; loop(); } }
static void runUntil(long hh, long mm, long ss = 0) { long target = hh * 3600 + mm * 60 + ss; while (RTC.sec() < target) { g_ms += 2; loop(); } }
static void usb(const char* c) { while (*c) Serial.rx.push_back((uint8_t)*c++); Serial.rx.push_back('\n'); run(5); }
static void nx(const char* c) { while (*c) Serial1.rx.push_back((uint8_t)*c++); Serial1.rx.push_back('\n'); run(5); }
static void nxCode(uint8_t c) { Serial1.rx.push_back(c); for (int i = 0; i < 3; i++) Serial1.rx.push_back(0xFF); run(5); }
static size_t g_logMark = 0, g_nxMark = 0;
static std::string usbSince() { return std::string(Serial.tx.begin() + g_logMark, Serial.tx.end()); }
static std::string nxSince() { return std::string(Serial1.tx.begin() + g_nxMark, Serial1.tx.end()); }
static void mark() { g_logMark = Serial.tx.size(); g_nxMark = Serial1.tx.size(); }
static bool logHas(const char* s) { return usbSince().find(s) != std::string::npos; }
static bool nxHas(const char* s) { return nxSince().find(s) != std::string::npos; }
static std::string lastStatus() {
  std::string all(Serial1.tx.begin(), Serial1.tx.end());
  size_t p = all.rfind("page0.t3.txt=\"");
  if (p == std::string::npos) return "(aucun)";
  size_t e = all.find('\xFF', p);
  std::string s = all.substr(p + 14, e - p - 15), out;
  for (size_t i = 0; i < s.size(); i++) {
    unsigned char c = s[i];
    if (c == '\\' && i + 1 < s.size() && s[i + 1] == 'r') { out += "\n      "; i++; }
    else if (c == 0xE9) out += "é"; else if (c == 0xEA) out += "ê"; else if (c == 0xE0) out += "à"; else if (c == 0xB0) out += "°";
    else if (c >= 0x80) { char b[8]; snprintf(b, sizeof b, "<%02X>", c); out += b; } else out += (char)c;
  }
  return out;
}

// ---- scénarios ----
static void forcecooldown_auto() {
  usb("MODE AUTO");
  g_flue = 150.0f;
  runUntil(6, 33);
  CHECK(boilerOn, "AUTO demarre dans la plage 06:30-09:00");
  mark(); nx("COOLDOWN_ON");
  CHECK(stopCooldownActive && autoHoldActive, "Forcer Cooldown -> COOLDOWN + maintien AUTO");
  run(9UL * 60000UL, 2);
  CHECK(stopCooldownActive && statePump, "toujours en cooldown a 9 min (pompe ON)");
  run(61000UL, 2);
  CHECK(!boilerOn && !stopCooldownActive, "IDLE apres 10 min (plus de cooldown sans fin)");
  CHECK(!statePump && g_fanPwm == 0, "pompe et ventilateur coupes");
  runUntil(8, 59);
  CHECK(!boilerOn && !stopCooldownActive && autoHoldActive, "reste arrete jusqu'a la fin de la plage");
  mark(); runUntil(9, 2);
  CHECK(!autoHoldActive && logHas("fin de plage"), "maintien relache a la fin de la plage");
  runUntil(17, 33);
  CHECK(boilerOn, "AUTO redemarre a la plage suivante (17:30)");
}

static void start_in_cooldown() {
  g_flue = 150.0f;
  nx("START"); run(5UL * 60000UL, 2);
  CHECK(boilerOn && statePump, "RUN depuis 5 min, pompe ON");
  nx("STOP"); run(60000UL, 2);
  CHECK(stopCooldownActive, "STOP -> cooldown");
  mark(); nx("START");
  CHECK(boilerOn && statePump && stateCandle, "START en cooldown -> reprise RUN, pompe toujours ON");
  CHECK(logHas("REPRISE depuis COOLDOWN"), "log REPRISE");
  run(30000UL, 2);
  CHECK(statePump, "pompe toujours ON 30 s apres la reprise");
  g_waterOH = true; run(2000);
  CHECK(alarmActive && alarmCode == ALM_WATER_OH && stopCooldownActive, "surchauffe eau -> ALARM + cooldown");
  g_waterOH = false; mark(); nx("START"); run(2500);
  CHECK(stopCooldownActive && !boilerOn && logHas("START refuse: ALARM"), "START refuse pendant ALARM");
  printf("  bloc d'etat :\n      %s\n", lastStatus().c_str());
  nx("FORCE_IDLE"); nx("START");
  CHECK(boilerOn && !alarmActive, "Forcer l'arret acquitte, puis START OK");
}

static void nextion_start_in_auto() {
  usb("MODE AUTO"); g_rtcBaseSec = 12L * 3600; // hors plage
  run(2000); mark(); nx("START"); run(1500);
  CHECK(controlMode == MODE_MANUAL && boilerOn, "START ecran en AUTO -> MANUAL + START");
  CHECK(nxHas("page0.sw0.val=0"), "sw0 resynchronise a 0");
  mark(); usb("MODE AUTO"); run(1500);
  CHECK(nxHas("page0.sw0.val=1"), "MODE AUTO via USB -> sw0 mis a jour");
  usb("MODE MANUAL"); usb("MODE OFF"); run(10);
  CHECK(stopCooldownActive, "MODE OFF pendant RUN -> cooldown");
  run(11UL * 60000UL, 2); mark(); usb("b1");
  CHECK(!boilerOn && logHas("MODE OFF"), "START refuse en MODE OFF");
}

static void thermocouple() {
  g_flue = NAN; nx("START");
  run(29000UL);
  CHECK(boilerOn && !alarmActive, "pas d'alarme avant 30 s");
  run(3000UL);
  CHECK(alarmActive && alarmCode == ALM_THERMOCOUPLE && stopCooldownActive, "thermocouple NaN 30 s -> ALARM + cooldown");
}

static void nodep() {
  g_flue = 150.0f; nx("START"); run(5UL * 60000UL, 2);
  CHECK(boilerOn && !alarmActive, "depression OK : pas d'alarme en 5 min");
  nx("STOP"); run(11UL * 60000UL, 2); g_forceNoDep = true; nx("START");
  run(74000UL);
  CHECK(boilerOn && !alarmActive, "pas d'alarme avant 60 s + 15 s");
  run(2000UL);
  CHECK(alarmActive && alarmCode == ALM_NO_DEPRESSION, "pas de depression -> ALARM a ~75 s");
}

static void nofire() {
  g_flue = 25.0f; nx("START");
  run(10UL * 60000UL - 2000UL, 2);
  CHECK(boilerOn && !alarmActive, "pas d'alarme NO FIRE avant 10 min");
  run(3000UL);
  CHECK(alarmActive && alarmCode == ALM_NO_FIRE, "NO FIRE a 10 min");
}

static void params() {
  mark(); usb("PP 1000");
  CHECK(pelletCyclePeriodMs == 8000UL && logHas("ERR"), "PP 1000 refuse et non applique");
  usb("PP 1400"); CHECK(pelletCyclePeriodMs == 8000UL, "PP 1400 refuse (PON 1000 + 500)");
  usb("PP 9000"); CHECK(pelletCyclePeriodMs == 9000UL, "PP 9000 accepte");
  usb("PON 8600"); CHECK(pelletOnDurationMs == 1000UL, "PON 8600 refuse (> 5000)");
  g_flue = 150.0f; nx("START"); run(1000);
  CHECK(activePelletPeriodMs() == 8000UL, "demarrage : MED (8000) actif");
  run(6UL * 60000UL + 1000UL, 2);
  CHECK(!startupLowPowerActive && activePelletPeriodMs() == 9000UL, "apres demarrage : PP 9000 conserve");
  mark(); nx("PWR_HIGH"); run(1500);
  CHECK(pelletCyclePeriodMs == 6000UL && selectedPowerLevel == PWR_HIGH && nxHas("page0.select0.val=2"), "PWR_HIGH -> profil HIGH + select0=2");
}

static void failsafe() {
  mark(); usb("w1"); run(5000UL);
  CHECK(!statePellet && g_pelletToggles == 2 && logHas("FAILSAFE"), "w1 manuel coupe apres 3 s, sans claquement");
  g_flue = 150.0f; nx("START"); run(6UL * 60000UL + 2000UL, 2);
  pelletCyclePeriodMs = 5000UL; pelletOnDurationMs = 5000UL; // bug simulé : vis ON en continu
  long t0 = g_pelletToggles; run(10000UL);
  CHECK(alarmActive && alarmCode == ALM_PELLET_FAILSAFE && !statePellet, "vis ON continue en RUN -> ALARM failsafe");
  CHECK(g_pelletToggles - t0 <= 3, "pas de claquement du relais");
}

static void manual_candle() {
  usb("c1"); run(9UL * 60000UL, 2);
  CHECK(stateCandle, "candelette manuelle ON a 9 min");
  run(61000UL, 2);
  CHECK(!stateCandle, "candelette manuelle coupee a 10 min");
}

static void nextion_ready() {
  mark(); nxCode(0x00); nxCode(0x88); run(1500);
  CHECK(logHas("ecran pret") && !logHas("inconnu"), "trames 00 / 88 reconnues");
  CHECK(nxHas("page0.select0.val=1") && nxHas("page0.sw0.val=0") && nxHas("grayfire.en=1") && nxHas("page0.t3.txt=") && nxHas("page0.t3.pco="), "resynchro complete");
  mark(); nxCode(0x1A); CHECK(logHas("0x1A"), "code d'erreur Nextion journalise");
  printf("  bloc d'etat (IDLE) :\n      %s\n", lastStatus().c_str());
  g_flue = 150.0f; nx("START"); run(2UL * 60000UL + 3000UL, 2);
  printf("  bloc d'etat (DEMARRAGE) :\n      %s\n", lastStatus().c_str());
  run(6UL * 60000UL, 2);
  printf("  bloc d'etat (RUN) :\n      %s\n", lastStatus().c_str());
  g_rawPressure = 60; run(4500);
  printf("  bloc d'etat (ALARM) :\n      %s\n", lastStatus().c_str());
  size_t mx = 0, p = 0;
  std::string all(Serial1.tx.begin(), Serial1.tx.end());
  while ((p = all.find("page0.t3.txt=\"", p)) != std::string::npos) { size_t e = all.find('\xFF', p); mx = std::max(mx, e - p - 15); p = e; }
  printf("  texte le plus long : %zu octets ; ecritures bloquantes UART Nextion : %ld\n", mx, Serial1.blockedWrites);
  CHECK(mx < 250, "bloc d'etat < 250 caracteres");
  CHECK(Serial1.blockedWrites == 0, "aucune ecriture bloquante vers l'ecran");
}

static void misc() {
  mark(); usb("SETTIME 2026-02-30 10:00:00"); CHECK(logHas("invalide"), "SETTIME 30 fevrier refuse");
  mark(); usb("SETTIME 2026-02-28 10:00:00"); CHECK(logHas("OK: RTC updated"), "SETTIME valide accepte");
  mark(); std::string big(200, 'x'); usb(big.c_str()); CHECK(logHas("trop longue"), "ligne de 200 caracteres rejetee");
  mark(); nx("SETDAY 3 06:15 08:00 18:00 22:45");
  CHECK(weekSched[2].start1 == 375 && weekSched[2].end2 == 1365, "SETDAY depuis l'ecran");
  mark(); nx("LOAD_3"); run(1500);
  CHECK(nxHas("H1.val=6") && nxHas("M1.val=1") && nxHas("M4.val=3") && nxHas("loadDONE.en=1"), "LOAD_3 renvoie le planning");
  mark(); usb("k1"); CHECK(stopCooldownActive && autoHoldActive, "k1 en IDLE -> cooldown 10 min");
  usb("k0"); CHECK(!autoHoldActive && stopCooldownActive, "k0 relache le maintien, le cooldown continue");
}

// Analyseur de commandes sans String : casse, espaces, formats invalides
static void commandes() {
  usb("mode auto"); CHECK(controlMode == MODE_AUTO, "'mode auto' en minuscules");
  usb("  Mode Manual  "); CHECK(controlMode == MODE_MANUAL, "espaces + casse mixte");
  usb("power   high"); CHECK(selectedPowerLevel == PWR_HIGH, "plusieurs espaces avant l'argument");
  mark(); usb("POWER"); CHECK(logHas("ERR: POWER") && selectedPowerLevel == PWR_HIGH, "POWER sans argument refuse");
  mark(); usb("MODE PAUSE"); CHECK(logHas("ERR: MODE") && controlMode == MODE_MANUAL, "MODE inconnu refuse");
  mark(); usb("SETTIME 2026-2-8 1:2:3"); CHECK(logHas("OK: RTC updated"), "SETTIME sans zeros de tete");
  mark(); usb("SETTIME 2026-02-28 10:00"); CHECK(logHas("ERR: SETTIME format"), "SETTIME sans secondes refuse");
  mark(); usb("SETTIME 2026/02/28 10:00:00"); CHECK(logHas("ERR: SETTIME format"), "SETTIME avec / refuse");
  mark(); usb("SETDAY 8 06:00 07:00 08:00 09:00"); CHECK(logHas("ERR: D must be"), "SETDAY jour 8 refuse");
  mark(); usb("SETDAY 1 06:00 07:00 08:00"); CHECK(logHas("ERR: SETDAY"), "SETDAY incomplet refuse");
  mark(); usb("SETDAY 1 24:00 07:00 08:00 09:00"); CHECK(logHas("ERR: bad HH:MM"), "SETDAY 24:00 refuse");
  usb("setday 1 6:00 7:30 18:00 22:00");
  CHECK(weekSched[0].start1 == 360 && weekSched[0].end1 == 450 && weekSched[0].end2 == 1320, "setday minuscules + heures sur 1 chiffre");
  mark(); usb("PP"); CHECK(logHas("ERR: PP <ms>"), "PP sans valeur refuse");
  usb("f5.5"); CHECK(fabsf(fanVoltageV - 5.5f) < 0.01f, "f5.5 ventilateur manuel");
  usb("f0");
  mark(); usb("xyz"); CHECK(logHas("Commande inconnue: xyz"), "commande inconnue");
  mark(); nx("  start  "); CHECK(boilerOn, "ecran : '  start  ' accepte");
  mark(); nx("LOAD_9"); CHECK(logHas("ERR: LOAD_"), "ecran : LOAD_9 refuse");
  mark(); nx("BONJOUR"); CHECK(logHas("Nextion cmd inconnu: BONJOUR"), "ecran : commande inconnue journalisee");
}

struct Scenario { const char* name; void (*fn)(); };
static const Scenario kScenarios[] = {
  {"forcecooldown_auto", forcecooldown_auto}, {"start_in_cooldown", start_in_cooldown},
  {"nextion_start_in_auto", nextion_start_in_auto}, {"thermocouple", thermocouple}, {"nodep", nodep},
  {"nofire", nofire}, {"params", params}, {"failsafe", failsafe}, {"manual_candle", manual_candle},
  {"nextion_ready", nextion_ready}, {"misc", misc}, {"commandes", commandes},
};

static int runOne(const Scenario& s) {
  Serial.echo = getenv("ECHO") != nullptr;
  g_ms = 100;
  setup();
  usb("s0"); // pas de snapshot chaque seconde
  printf("== %s\n", s.name);
  s.fn();
  printf("== %s : %d PASS, %d FAIL\n", s.name, g_pass, g_fail);
  return g_fail ? 1 : 0;
}

int main(int argc, char** argv) {
  int failed = 0, count = 0;
  for (const Scenario& s : kScenarios) {
    if (argc > 1 && strcmp(argv[1], s.name) != 0) continue;
    count++;
    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) { int r = runOne(s); fflush(stdout); _exit(r); }
    int st = 0;
    waitpid(pid, &st, 0);
    if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) failed++;
  }
  if (count == 0) { printf("scenario inconnu : %s\n", argv[1]); return 2; }
  printf("\n%d scenario(s), %d en echec\n", count, failed);
  return failed ? 1 : 0;
}
