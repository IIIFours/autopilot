# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a marine autopilot system for Teensy 4.0 that reads NMEA 2000 navigation data and controls a rudder stepper motor using PID control to maintain course to waypoint.

## Development Commands

### Build and Upload
```bash
# Build the project
pio run

# Upload to Teensy 4.0
pio run --target upload

# Clean build
pio run --target clean

# Monitor serial output
pio device monitor --baud 115200
```

### Project Management
```bash
# Check project configuration
pio project config

# List available boards
pio boards teensy

# Install dependencies
pio pkg install
```

## Code Architecture

### Core Data Structures

- **`PIDParameters`** (`autopilot.cpp:98`): Stores PID control gains (kp, ki, kd) and heading correction gain
- **`NavigationData`** (`autopilot.cpp:119`): Contains NMEA 2000 navigation values (heading, XTE, bearing to waypoint)
- **`AutopilotState`** (`autopilot.cpp:132`): Main system state including PID parameters, navigation data, filtered values, and rudder control state

### Key Functional Areas

#### NMEA 2000 Communication (`autopilot.cpp:334-432`)
- Handles incoming navigation messages (PGN 129284, 129283, 127250)
- Parses heading, cross-track error (XTE), and navigation info
- Message handlers: `handleHeading()`, `handleXTE()`, `handleNavigationInfo()`

#### PID Control System (`autopilot.cpp:516-613`)
- **`calculateRudderAngle()`**: Core PID algorithm with filtering and anti-windup protection
- Uses filtered XTE as primary error signal
- Includes heading correction to prevent oscillation
- Implements rate limiting and saturation protection

#### Motor Control (`autopilot.cpp:438-510`)
- **`performHoming()`**: Auto-homes rudder using limit switch
- **`updateRudderPosition()`**: Non-blocking stepper motor control
- Maps rudder angles (-45° to +45°) to stepper positions

#### Serial Communication (`autopilot.cpp:238-329`)
- Bidirectional telemetry on Serial2 (9600 baud)
- Sends comprehensive autopilot state data
- Receives PID parameter updates
- Multi-threaded operation using TeensyThreads

#### EEPROM Storage (`autopilot.cpp:212-232`)
- Persistent storage of PID parameters
- Automatic validation and fallback to defaults

### Hardware Configuration

- **Target Board**: Teensy 4.0 (Arduino framework)
- **Stepper Motor**: Controls rudder position via AccelStepper library
- **NMEA 2000**: CAN bus communication using NMEA2000_Teensyx library
- **Display**: ST7789 TFT (configured but not actively used)
- **Serial**: Debug on Serial (115200), telemetry on Serial2 (9600)

### Control Flow

1. **Initialization**: Load PID parameters from EEPROM, initialize hardware, start telemetry thread
2. **Main Loop**: Parse NMEA messages → Update timing → Process serial data → Validate navigation → Home if needed → Calculate PID → Update motor
3. **Safety Features**: Input validation, motor disable on invalid data, homing failure protection

### Key Constants

- PID tuning starts at `autopilot.cpp:52-68`
- Motor limits and positions at `autopilot.cpp:74-76`
- NMEA 2000 PGNs at `autopilot.cpp:89-92`

### Threading Model

Uses TeensyThreads for concurrent operations with thread safety:
- Main thread: Control loop and NMEA parsing
- Serial thread: Continuous telemetry transmission

**Thread Safety Mechanisms:**
- `g_stateMutex`: Protects main autopilot state during PID calculations and updates
- `g_serialMutex`: Protects serial communication buffers
- Thread-safe accessor functions: `updateNavigationData()`, `updatePIDParameters()`, `getAutopilotStateSnapshot()`
- Atomic variables for simple shared state: `g_bufferIndex`, `g_dataStarted`
- Critical sections protected with `Threads::Scope` RAII locks

### Dependencies

- **NMEA2000-library**: Core NMEA 2000 protocol support
- **TeensyThreads**: Multi-threading capabilities
- **AccelStepper**: Stepper motor control
- **ST7735_t3/ST7789_t3**: Display support (currently unused)