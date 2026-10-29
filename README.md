## Arduino-based pellet boiler with Nextion display 🔥

[![CI](https://github.com/jdejonghe72/Arduino-Boiler/actions/workflows/ci.yml/badge.svg)](https://github.com/jdejonghe72/Arduino-Boiler/actions/workflows/ci.yml)

>#### Here is my project for a DIY-Retrofited pellet boiler
> + You can find the Arduino Code in the `BoilerWithNextionLatest` directory.
> + The Nextion HMI file is in the `Nextion HMI` directory.
> + Other ressources are in the `Documetation` directory.

> [!Note]
> Most of the ressources are in French for the moment.

### M-Duino 38AR+: Basic boiler + MAX6675 (without library)
+ RTC + EEPROM + Weekly planning (2 slots/day)
+ Safety: diagnosis-only in IDLE, latch + cooldown in RUN/COOLDOWN (overheat, water pressure, no fire, no depression, thermocouple fault, auger failsafe)
+ Power parameters modifiable by serial controls + EEPROM persistent
+ Debug AUTO "event logging" (little verbal): log only on changes
+ Nextion: status block on page 0 (temperatures, pressure, outputs, state, alarm), resync when the screen restarts
+ Settings (pins, delays, thresholds, power profiles) are in `config.h`; the code is split into IDE tabs by topic

### Tests
`tests/run.sh` runs the sketch on a PC with simulated time and I/O (force cooldown, alarms, Nextion, serial commands).
GitHub Actions runs it on every push and compiles the sketch for the M-Duino 38AR+ with the Industrial Shields core.

### Nextion HMI file
`Nextion HMI/hmi_tool.py` reads the `.HMI` without Nextion Editor (`dump`, or `export`/`import` as JSON).
Imported pages keep their exact size (no new components), so always reopen the file in Nextion Editor before compiling.

### How to flash:
   - Tools > Board > industrialshield-avr > M-DUINO Family
   - Tools > Port > then select the good one
   - Tools > Model > M-DUINO 38AR+


<img src="https://github.com/user-attachments/assets/bc547c7a-eea5-4014-8e09-e56e7d0db5f2" alt="Projet photo" width="400">
