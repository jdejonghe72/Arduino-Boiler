// ============================================================================
// CONFIGURATION : switches de compilation, câblage, temporisations, seuils
// (inclus une seule fois, en tête de BoilerWithNextionLatest.ino)
// ============================================================================
#pragma once

#include <Arduino.h>

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
const unsigned int SERIAL_LINE_MAX = 64; // longueur max d'une commande (série et Nextion)

// --------------------------------------------------------------------
// MODE CHAUDIÈRE BASIQUE
// --------------------------------------------------------------------
const unsigned long IGNITION_DURATION_MS   = 6UL * 60UL * 1000UL;
const unsigned long PUMP_START_DELAY_MS    = 1UL  * 60UL * 1000UL;
const unsigned long STOP_COOLDOWN_MS       = 10UL * 60UL * 1000UL;
const unsigned long STARTUP_LOW_POWER_MS   = 6UL * 60UL * 1000UL;

// --------------------------------------------------------------------
// PUISSANCE : profils LOW / MED / HIGH (MED aussi utilisé pendant le démarrage)
// --------------------------------------------------------------------
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

// --------------------------------------------------------------------
// FAILSAFE VIS SANS FIN
// --------------------------------------------------------------------
const unsigned long PELLET_MAX_CONTINUOUS_ON_MS = 3000UL; // 3 s en manuel (w1)
const unsigned long PELLET_RUN_ON_MARGIN_MS     = 1000UL; // en RUN : PON actif + 1 s
const unsigned long PELLET_MIN_OFF_MS           = 500UL;  // PON <= PP - 500 ms

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

// Diagnostic anti-spam
const unsigned long DIAG_PERIOD_MS = 5000;

// --------------------------------------------------------------------
// AUTO (planning)
// --------------------------------------------------------------------
const unsigned long AUTO_MIN_COMMAND_GAP_MS = 30UL * 1000UL;
const unsigned long AUTO_DEMAND_STABLE_MS   = 60UL * 1000UL; // demande stable avant action
const unsigned long RTC_POLL_MS             = 1000;          // 1 lecture RTC par seconde

// --------------------------------------------------------------------
// NEXTION : bloc d'état page0.t3
// --------------------------------------------------------------------
const unsigned long NEXTION_STATUS_PERIOD_MS = 2000;  // rafraîchissement du bloc d'état
const unsigned long NEXTION_RESYNC_PERIOD_MS = 60000; // renvoi périodique état feu/candelette/mode
