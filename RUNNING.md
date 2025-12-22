# Running and Testing

This repository targets an Industrial Shields M-Duino 38AR+ PLC with Arduino IDE tooling. The current container lacks the Arduino toolchain and attached hardware, so firmware compilation or runtime validation cannot be performed here. To build and run the sketch locally:

1. Install the Arduino IDE (or `arduino-cli`) with the required board support package for Industrial Shields hardware.
2. Open `BoilerProgrammableSafetyDebug7Watchdog.ino` in the IDE.
3. Select the correct board profile and serial port for the M-Duino 38AR+ PLC.
4. Compile and upload the sketch to the device.
5. Monitor serial output on both the USB Serial and TTL `Serial1` ports to verify communication with the Nextion panel.

If `arduino-cli` is available, you can run `arduino-cli compile --fqbn <fqbn> BoilerProgrammableSafetyDebug7Watchdog.ino` to verify compilation from the command line.
