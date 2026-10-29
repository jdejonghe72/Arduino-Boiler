#pragma once
#include <cstring>
struct EEPROMClass {
  uint8_t mem[4096];
  EEPROMClass() { memset(mem, 0xFF, sizeof(mem)); }
  template <typename T> T& get(int a, T& t) { memcpy(&t, mem + a, sizeof(T)); return t; }
  template <typename T> const T& put(int a, const T& t) { memcpy(mem + a, &t, sizeof(T)); return t; }
};
extern EEPROMClass EEPROM;
