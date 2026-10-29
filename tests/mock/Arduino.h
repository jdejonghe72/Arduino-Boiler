// Mini environnement Arduino pour simuler le sketch sur PC (temps simulé)
#pragma once
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <math.h>
#include <ctype.h>
#include <strings.h>

typedef uint8_t byte;
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#define PSTR(s) (s)
#define PGM_P const char*
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
#define strcasecmp_P strcasecmp
#define strncasecmp_P strncasecmp
#define strlen_P strlen
#define strncpy_P strncpy
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define DEC 10
#define HEX 16
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
// Broches Industrial Shields (valeurs arbitraires)
#define A1_0 4
#define R1_1 22
#define R1_2 23
#define R1_4 25

extern unsigned long g_ms;
inline unsigned long millis() { return g_ms; }
inline unsigned long micros() { return g_ms * 1000UL; }
inline void delayMicroseconds(unsigned int) {}
void pinMode(int, int);
void digitalWrite(int pin, int v);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int v);

inline char* ltoa(long v, char* buf, int) { snprintf(buf, 12, "%ld", v); return buf; }
inline char* dtostrf(double v, signed char w, unsigned char p, char* buf) { snprintf(buf, 16, "%*.*f", w, p, v); return buf; }

// Port série simulé : garde tout ce qui est écrit, modélise la FIFO matérielle de 63 octets
class Port {
 public:
  std::string line;        // ligne en cours (sortie texte)
  std::vector<uint8_t> tx; // tout ce qui a été écrit
  std::deque<uint8_t> rx;
  bool echo = false;
  int fifo = 0; unsigned long fifoMs = 0; long blockedWrites = 0;
  void begin(long) {}
  explicit operator bool() const { return true; }
  int available() { return (int)rx.size(); }
  int read() { if (rx.empty()) return -1; int c = rx.front(); rx.pop_front(); return c; }
  void drain() { unsigned long d = g_ms - fifoMs; fifoMs = g_ms; fifo -= (int)d; if (fifo < 0) fifo = 0; } // ~1 octet/ms à 9600 bauds
  int availableForWrite() { drain(); return 63 - fifo; }
  size_t write(uint8_t b) {
    drain(); if (fifo >= 63) blockedWrites++; else fifo++;
    tx.push_back(b);
    if (b == '\n') { if (echo) printf("[%7.1fs] %s\n", g_ms / 1000.0, line.c_str()); line.clear(); }
    else line.push_back((char)b);
    return 1;
  }
  void out(const char* s) { while (*s) write((uint8_t)*s++); }
  void print(const char* s) { out(s); }
  void print(const __FlashStringHelper* f) { out(reinterpret_cast<const char*>(f)); }
  void print(char c) { write((uint8_t)c); }
  void print(int v, int base = DEC) { print((long)v, base); }
  void print(unsigned int v, int base = DEC) { print((unsigned long)v, base); }
  void print(long v, int base = DEC) { char b[24]; snprintf(b, sizeof b, base == HEX ? "%lX" : "%ld", v); out(b); }
  void print(unsigned long v, int base = DEC) { char b[24]; snprintf(b, sizeof b, base == HEX ? "%lX" : "%lu", v); out(b); }
  void print(uint8_t v, int base = DEC) { print((unsigned long)v, base); }
  void print(double v, int d = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", d, v); out(b); }
  template <typename T> void println(T v) { print(v); write('\n'); }
  template <typename T> void println(T v, int x) { print(v, x); write('\n'); }
  void println() { write('\n'); }
};
extern Port Serial;
extern Port Serial1;
