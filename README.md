## Arduino-based pellet boiler with Nextion display 🔥

>#### Here is my project for a DIY-Retrofited pellet boiler
> + You can find the Arduino Code in the `BoilerWithNextionLatest` directory.
> + The Nextion HMI file is in the `Nextion HMI` directory.
> + Other ressources are in the `Documetation` directory.

> [!Note]
> Most of the ressources are in French for the moment.

### M-Duino 38AR+: Basic boiler + MAX6675 (without library)
+ RTC + EEPROM + Weekly planning (2 slots/day)
+ Safety: diagnosis-only in IDLE, latch + cooldown in RUN/COOLDOWN
+ Power parameters modifiable by serial controls + EEPROM persistent
+ Debug AUTO "event logging" (little verbal): log only on changes

### How to flash:
   - Tools > Board > industrialshield-avr > M-DUINO Family
   - Tools > Port > then select the good one
   - Tools > Model > M-DUINO 38AR+


<div align="center">
<svg width="400" height="300" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink">
  <defs>
    <clipPath id="rounded">
      <rect width="400" height="300" rx="15" ry="15"/>
    </clipPath>
  </defs>
  <image width="400" height="300" xlink:href="https://github.com/user-attachments/assets/bc547c7a-eea5-4014-8e09-e56e7d0db5f2" clip-path="url(#rounded)"/>
</svg>
</div>
