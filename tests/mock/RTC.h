#pragma once
// Horloge simulée (bibliothèque RTC Industrial Shields) : heure de base + temps simulé, jeudi 2026-10-29
extern long g_rtcBaseSec;
extern bool g_rtcOk;
struct RTCClass {
  long sec() const { return g_rtcBaseSec + (long)(g_ms / 1000UL); }
  bool read() { return g_rtcOk; }
  bool write() { return true; }
  int getYear() { return 2026; }
  int getMonth() { return 10; }
  int getMonthDay() { return 29; }
  int getHour() { return (int)((sec() / 3600) % 24); }
  int getMinute() { return (int)((sec() / 60) % 60); }
  int getSecond() { return (int)(sec() % 60); }
  void setYear(int) {} void setMonth(int) {} void setMonthDay(int) {}
  void setHour(int) {} void setMinute(int) {} void setSecond(int) {}
};
extern RTCClass RTC;
