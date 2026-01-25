## Arduino-based pellet boiler with Nextion display 🔥
### M-Duino 38AR+: Basic boiler + MAX6675 (without library)
+ RTC + EEPROM + Weekly planning (2 beaches/day)
+ Safety: diagnosis-only in IDLE, latch + cooldown in RUN/COOLDOWN
+ Power parameters modifiable by serial controls + EEPROM persistent
+ Debug AUTO "event logging" (little verbal): log only on changes

### Avis rapide sur le code
- Points forts : logique claire par états, sécurités multiples (surchauffe, pression, watchdog), gestion EEPROM/RTC robuste et logs d'événements utiles pour diagnostiquer.
- Points à surveiller : seuils capteurs à recalibrer selon le matériel réel, sécurité "failsafe vis sans fin" réglée très courte (3 s) à adapter aux cycles pellet, et bien valider le comportement en COOLDOWN/STARTUP sur le terrain.
- Pour aller plus loin : isoler la configuration (seuils/temporisations) dans un bloc dédié ou un fichier, et prévoir des tests de simulation pour les scénarios critiques.

### Quick take on the code
- Strengths: clear state-based logic, multiple safety checks (overheat, pressure, watchdog), solid EEPROM/RTC handling, and event logs that help debugging.
- Watch-outs: sensor thresholds must be calibrated for the real hardware, the "pellet screw failsafe" window is very short (3 s) and may need tuning, and COOLDOWN/STARTUP behavior should be validated on the actual boiler.
- Next steps: centralize configuration (thresholds/timers) in a dedicated block or file, and add simulation tests for critical scenarios.
### How to flash:
   - Tools > Board > industrialshield-avr > M-DUINO Family
   - Tools > Port > then select the good one
   - Tools > Model > M-DUINO 38AR+
