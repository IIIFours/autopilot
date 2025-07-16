# 3Fours Autopilot

This project contains firmware for a Teensy 4.0 based marine autopilot. It reads NMEA 2000 navigation data and drives a stepper motor to control the rudder using PID control.

## Main Components

- **NMEA 2000 message handling** – functions like `handleHeading`, `handleXTE` and `handleNavigationInfo` parse incoming CAN bus messages.
- **PID control** – `calculateRudderAngle` computes the rudder angle using cross‑track error with heading correction and rate limiting.
- **Stepper motor control** – `performHoming` and `updateRudderPosition` manage homing via a limit switch and drive the rudder stepper.
- **Serial telemetry and tuning** – a background `serialThread` sends telemetry using `sendAutopilotData` and receives PID updates through `processSerialData`.
- **EEPROM storage** – PID parameters persist across reboots using `loadPIDParameters` and `savePIDParameters`.
- **Main loop** – the `loop` function orchestrates parsing, timing, homing and rudder control.

## Running the Code

1. Install [PlatformIO](https://platformio.org/) and its dependencies.
2. Build the firmware:
   ```bash
   pio run
   ```
3. Upload to a Teensy 4.0:
   ```bash
   pio run --target upload
   ```
4. Optionally monitor serial output:
   ```bash
   pio device monitor
   ```
