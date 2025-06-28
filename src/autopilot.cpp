/*
 * 3Fours Autopilot System
 * 
 * This system reads NMEA 2000 messages for navigation data and controls
 * a rudder stepper motor using PID control to maintain course to waypoint.
 * 
 * Features:
 * - NMEA 2000 message parsing (Heading, XTE, Navigation Info)
 * - PID-based rudder control
 * - Stepper motor control with homing
 * - Serial communication for PID parameter updates
 * - EEPROM storage for PID parameters
 * - Multi-threaded operation using TeensyThreads
 */

#include <Arduino.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>
#include <AccelStepper.h>
#include <SPI.h>
#include <ST7735_t3.h>
#include <ST7789_t3.h>
#include <TeensyThreads.h>
#include <EEPROM.h>

// =============================================================================
// CONSTANTS AND CONFIGURATION
// =============================================================================

// Pin Definitions
constexpr uint8_t N2K_CAN_INT_PIN = 21;
constexpr uint8_t TFT_RST = 14;
constexpr uint8_t TFT_DC = 9;
constexpr uint8_t TFT_MOSI = 11;
constexpr uint8_t TFT_SCLK = 13;
constexpr uint8_t TFT_CS = 10;
constexpr uint8_t LCD_BL = 15;
constexpr uint8_t STEPPER_STEP_PIN = 1;
constexpr uint8_t STEPPER_DIR_PIN = 2;
constexpr uint8_t MOTOR_ENABLE_PIN = 4;
constexpr uint8_t LIMIT_SWITCH_PIN = 3;

// Communication Constants
constexpr uint32_t SERIAL_BAUD_RATE = 9600;
constexpr uint32_t SERIAL_DEBUG_BAUD_RATE = 115200;
constexpr uint8_t START_MARKER = 0x02;
constexpr uint8_t END_MARKER = 0x03;
constexpr size_t PID_BUFFER_SIZE = 16;

// Control Constants
constexpr float DEFAULT_PID_VALUE = 1.0f;
constexpr float DEFAULT_KP_VALUE = 1.5f;    // Reduced from 2.0 for stability
constexpr float DEFAULT_KI_VALUE = 0.02f;   // Much lower to prevent windup
constexpr float DEFAULT_KD_VALUE = 0.3f;    // Reduced for marine environment
constexpr float DEFAULT_HEADING_GAIN = 0.2f; // More conservative
constexpr float MIN_PID_VALUE = 0.0f;
constexpr float MAX_PID_VALUE = 10.0f;
constexpr float MAX_RUDDER_ANGLE = 45.0f;
constexpr float MIN_RUDDER_ANGLE = -45.0f;
constexpr float MAX_INTEGRAL_WINDUP = 50.0f;  // Much lower limit
constexpr float MAX_VALID_XTE = 100.0f;
constexpr float MIN_VALID_XTE = -100.0f;
constexpr float MAX_VALID_HEADING = 360.0f;
constexpr float MIN_VALID_HEADING = 0.0f;
constexpr float MAX_RUDDER_RATE = 10.0f; // degrees per second
constexpr float XTE_FILTER_ALPHA = 0.8f; // Low-pass filter coefficient (0-1, higher = less filtering)
constexpr float HEADING_FILTER_ALPHA = 0.7f;

// Motor Constants
constexpr float STEPPER_MAX_SPEED = 1000.0f;
constexpr float STEPPER_ACCELERATION = 50000.0f;
constexpr uint16_t STEPPER_MIN_PULSE_WIDTH = 3;
constexpr int32_t RUDDER_POSITION_MIN = -1050;
constexpr int32_t RUDDER_POSITION_MAX = -70;
constexpr int32_t RUDDER_POSITION_CENTER = -450;

// Timing Constants
constexpr uint32_t MAIN_LOOP_DELAY_MS = 100;
constexpr uint32_t SERIAL_THREAD_DELAY_MS = 1000;
constexpr float MAX_TIME_DELTA = 1.0f; // Maximum allowed time delta in seconds

// EEPROM Addresses
constexpr int EEPROM_KP_ADDR = 0;
constexpr int EEPROM_KI_ADDR = 4;
constexpr int EEPROM_KD_ADDR = 8;
constexpr int EEPROM_HEADING_GAIN_ADDR = 12;

// NMEA 2000 PGN Constants
constexpr unsigned long PGN_NAVIGATION_INFO = 129284L;
constexpr unsigned long PGN_XTE = 129283L;
constexpr unsigned long PGN_HEADING = 127250L;

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct PIDParameters {
    float kp = DEFAULT_KP_VALUE;
    float ki = DEFAULT_KI_VALUE;
    float kd = DEFAULT_KD_VALUE;
    float heading_gain = DEFAULT_HEADING_GAIN;
    
    bool isValid() const {
        return (kp > MIN_PID_VALUE && kp <= MAX_PID_VALUE &&
                ki > MIN_PID_VALUE && ki <= MAX_PID_VALUE &&
                kd > MIN_PID_VALUE && kd <= MAX_PID_VALUE &&
                heading_gain >= 0.0f && heading_gain <= 2.0f);
    }
    
    void setDefaults() {
        kp = DEFAULT_KP_VALUE;
        ki = DEFAULT_KI_VALUE;
        kd = DEFAULT_KD_VALUE;
        heading_gain = DEFAULT_HEADING_GAIN;
    }
};

struct NavigationData {
    float heading = -1.0f;
    float xte = -1.0f;
    float bearingToWaypoint = -1.0f;
    double destinationLatitude = 0.0;
    double destinationLongitude = 0.0;
    
    bool hasValidHeading() const { return heading >= MIN_VALID_HEADING && heading <= MAX_VALID_HEADING; }
    bool hasValidXTE() const { return xte > MIN_VALID_XTE && xte < MAX_VALID_XTE; }
    bool hasValidBearing() const { return bearingToWaypoint >= MIN_VALID_HEADING && bearingToWaypoint <= MAX_VALID_HEADING; }
    bool isNavigationDataValid() const { return hasValidHeading() && hasValidXTE() && hasValidBearing(); }
};

struct AutopilotState {
    PIDParameters pid;
    NavigationData navigation;
    
    // Filtered navigation data for smoother control
    float filteredXTE = 0.0f;
    float filteredHeading = 0.0f;
    bool filtersInitialized = false;
    
    // Previous values for derivative calculations
    float previousXte = -1.0f;
    float previousBearing = 0.0f;
    double previousDestinationLat = 0.0;
    double previousDestinationLon = 0.0;
    uint32_t previousTime = 0;
    
    // PID control variables
    float integralXTE = 0.0f;
    float derivativeXTE = 0.0f;
    float timeDelta = 0.0f;
    float headingCorrection = 0.0f;
    
    // Rudder control
    float rudderAngle = 0.0f;
    float previousRudderAngle = 0.0f; // For rate limiting
    int32_t rudderPosition = 0;
    int32_t targetMotorPosition = 0;
    
    // System state
    bool homingComplete = false;
};

struct NMEA2000Handler {
    unsigned long PGN;
    void (*Handler)(const tN2kMsg &N2kMsg);
};

// =============================================================================
// GLOBAL OBJECTS AND VARIABLES
// =============================================================================

// Hardware objects
AccelStepper rudderStepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// System state
AutopilotState g_autopilotState;
Stream* g_outputStream = nullptr;

// Serial communication buffer
uint8_t g_serialBuffer[PID_BUFFER_SIZE];
int g_bufferIndex = 0;
bool g_dataStarted = false;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

constexpr double radiansToDegrees(double radians) {
    return radians * (180.0 / PI);
}

double normalizeHeading(double heading) {
    while (heading > 180.0) {
        heading -= 360.0;
    }
    while (heading < -180.0) {
        heading += 360.0;
    }
    return heading;
}

bool isDestinationChanged(const AutopilotState& state, double newLat, double newLon) {
    return (newLat != state.previousDestinationLat) || (newLon != state.previousDestinationLon);
}

// =============================================================================
// EEPROM OPERATIONS
// =============================================================================

void savePIDParameters(const PIDParameters& pid) {
    EEPROM.put(EEPROM_KP_ADDR, pid.kp);
    EEPROM.put(EEPROM_KI_ADDR, pid.ki);
    EEPROM.put(EEPROM_KD_ADDR, pid.kd);
    EEPROM.put(EEPROM_HEADING_GAIN_ADDR, pid.heading_gain);
}

PIDParameters loadPIDParameters() {
    PIDParameters pid;
    EEPROM.get(EEPROM_KP_ADDR, pid.kp);
    EEPROM.get(EEPROM_KI_ADDR, pid.ki);
    EEPROM.get(EEPROM_KD_ADDR, pid.kd);
    EEPROM.get(EEPROM_HEADING_GAIN_ADDR, pid.heading_gain);
    
    if (!pid.isValid()) {
        pid.setDefaults();
        savePIDParameters(pid);
    }
    
    return pid;
}

// =============================================================================
// SERIAL COMMUNICATION
// =============================================================================

void sendAutopilotData(const AutopilotState& state) {
    // Create a packed structure for transmission
    struct __attribute__((packed)) AutopilotTelemetry {
        float kp, ki, kd, heading_gain;
        float bearingToWaypoint;
        float destinationLatitude;
        float destinationLongitude;
        float previousDestinationLatitude;
        float previousDestinationLongitude;
        float heading;
        float xte;
        float filteredHeading;    // New: filtered heading
        float filteredXTE;        // New: filtered XTE
        float previousXte;
        float previousTime;
        float previousBearing;
        float integralXTE;
        float derivativeXTE;
        float timeDelta;
        float rudderAngle;
        float rudderPosition;
        float targetMotorPosition;
        bool homingComplete;
        float headingCorrection;
    } telemetry;
    
    // Populate telemetry data
    telemetry.kp = state.pid.kp;
    telemetry.ki = state.pid.ki;
    telemetry.kd = state.pid.kd;
    telemetry.heading_gain = state.pid.heading_gain;
    telemetry.bearingToWaypoint = state.navigation.bearingToWaypoint;
    telemetry.destinationLatitude = static_cast<float>(state.navigation.destinationLatitude);
    telemetry.destinationLongitude = static_cast<float>(state.navigation.destinationLongitude);
    telemetry.previousDestinationLatitude = static_cast<float>(state.previousDestinationLat);
    telemetry.previousDestinationLongitude = static_cast<float>(state.previousDestinationLon);
    telemetry.heading = state.navigation.heading;
    telemetry.xte = state.navigation.xte;
    telemetry.filteredHeading = state.filteredHeading;
    telemetry.filteredXTE = state.filteredXTE;
    telemetry.previousXte = state.previousXte;
    telemetry.previousTime = static_cast<float>(state.previousTime);
    telemetry.previousBearing = state.previousBearing;
    telemetry.integralXTE = state.integralXTE;
    telemetry.derivativeXTE = state.derivativeXTE;
    telemetry.timeDelta = state.timeDelta;
    telemetry.rudderAngle = state.rudderAngle;
    telemetry.rudderPosition = static_cast<float>(state.rudderPosition);
    telemetry.targetMotorPosition = static_cast<float>(state.targetMotorPosition);
    telemetry.homingComplete = state.homingComplete;
    telemetry.headingCorrection = state.headingCorrection;
    
    // Send telemetry
    Serial2.write(START_MARKER);
    g_outputStream->printf("Sending telemetry: %ld\n", sizeof(telemetry));
    Serial2.write(reinterpret_cast<const uint8_t*>(&telemetry), sizeof(telemetry));
    Serial2.write(END_MARKER);
}

void processSerialData(AutopilotState& state) {
    while (Serial2.available()) {
        uint8_t incomingByte = Serial2.read();
        
        if (incomingByte == START_MARKER) {
            g_bufferIndex = 0;
            g_dataStarted = true;
        } else if (g_dataStarted && g_bufferIndex < PID_BUFFER_SIZE) {
            g_serialBuffer[g_bufferIndex++] = incomingByte;
        } else if (incomingByte == END_MARKER && g_bufferIndex == PID_BUFFER_SIZE) {
            // Parse received PID parameters
            const PIDParameters* receivedPID = reinterpret_cast<const PIDParameters*>(g_serialBuffer);
            
            if (receivedPID->isValid()) {
                state.pid = *receivedPID;
                savePIDParameters(state.pid);
                if (g_outputStream) {
                    g_outputStream->println("PID parameters updated");
                }
            }
            
            g_bufferIndex = 0;
            g_dataStarted = false;
        }
    }
}

void serialThread() {
    while (true) {
        sendAutopilotData(g_autopilotState);
        threads.delay(SERIAL_THREAD_DELAY_MS);
    }
}

// =============================================================================
// NMEA 2000 MESSAGE HANDLERS
// =============================================================================

void handleHeading(const tN2kMsg& N2kMsg) {
    unsigned char SID;
    double heading, deviation, variation;
    tN2kHeadingReference headingReference;

    if (ParseN2kHeading(N2kMsg, SID, heading, deviation, variation, headingReference)) {
        double headingDegrees = radiansToDegrees(heading);
        if (headingDegrees >= MIN_VALID_HEADING && headingDegrees <= MAX_VALID_HEADING) {
            g_autopilotState.navigation.heading = static_cast<float>(headingDegrees);
        } else {
            g_autopilotState.navigation.heading = -1.0f;
        }
    } else if (g_outputStream) {
        g_outputStream->print("Failed to parse Heading PGN: ");
        g_outputStream->println(N2kMsg.PGN);
    }
}

void handleXTE(const tN2kMsg& N2kMsg) {
    unsigned char SID;
    tN2kXTEMode XTEMode;
    bool navigationTerminated;
    double XTE;

    if (ParseN2kXTE(N2kMsg, SID, XTEMode, navigationTerminated, XTE)) {
        if (XTE > MIN_VALID_XTE && XTE < MAX_VALID_XTE) {
            g_autopilotState.navigation.xte = static_cast<float>(XTE);
        } else {
            g_autopilotState.navigation.xte = -1.0f;
        }
    } else if (g_outputStream) {
        g_outputStream->print("Failed to parse XTE PGN: ");
        g_outputStream->println(N2kMsg.PGN);
    }
}

void handleCOG(const tN2kMsg& N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference headingReference;
    double COG, SOG;
    
    if (ParseN2kPGN129026(N2kMsg, SID, headingReference, COG, SOG)) {
        double cogDegrees = radiansToDegrees(COG);
        if (cogDegrees >= MIN_VALID_HEADING && cogDegrees <= MAX_VALID_HEADING) {
            g_autopilotState.navigation.heading = static_cast<float>(cogDegrees);
        } else {
            g_autopilotState.navigation.heading = -1.0f;
        }
    }
}

void handleNavigationInfo(const tN2kMsg& N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference bearingReference;
    double bearingOriginToDestination, bearingPositionToDestination;
    double distanceToWaypoint, waypointClosingVelocity;
    uint32_t originWaypointNumber, destinationWaypointNumber;
    double destinationLatitude, destinationLongitude;
    bool perpendicularCrossed, arrivalCircleEntered;
    tN2kDistanceCalculationType calculationType;
    double ETATime;
    int16_t ETADate;

    if (ParseN2kNavigationInfo(N2kMsg, SID, distanceToWaypoint, bearingReference, 
                              perpendicularCrossed, arrivalCircleEntered, calculationType, 
                              ETATime, ETADate, bearingOriginToDestination, bearingPositionToDestination, 
                              originWaypointNumber, destinationWaypointNumber, destinationLatitude, 
                              destinationLongitude, waypointClosingVelocity)) {
        
        double bearingDegrees = radiansToDegrees(bearingPositionToDestination);
        if (bearingDegrees >= MIN_VALID_HEADING && bearingDegrees <= MAX_VALID_HEADING) {
            g_autopilotState.navigation.bearingToWaypoint = static_cast<float>(bearingDegrees);
            g_autopilotState.navigation.destinationLatitude = destinationLatitude;
            g_autopilotState.navigation.destinationLongitude = destinationLongitude;
        } else {
            g_autopilotState.navigation.bearingToWaypoint = -1.0f;
        }
    } else if (g_outputStream) {
        g_outputStream->print("Failed to parse Navigation Info PGN: ");
        g_outputStream->println(N2kMsg.PGN);
    }
}

void handleNMEA2000Message(const tN2kMsg& N2kMsg) {
    static const NMEA2000Handler handlers[] = {
        {PGN_NAVIGATION_INFO, &handleNavigationInfo},
        {PGN_XTE, &handleXTE},
        {PGN_HEADING, &handleHeading},
        {0, nullptr}
    };
    
    for (int i = 0; handlers[i].PGN != 0; i++) {
        if (N2kMsg.PGN == handlers[i].PGN) {
            handlers[i].Handler(N2kMsg);
            break;
        }
    }
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================

void initializeStepperMotor() {
    rudderStepper.setMaxSpeed(STEPPER_MAX_SPEED);
    rudderStepper.setAcceleration(STEPPER_ACCELERATION);
    rudderStepper.setSpeed(STEPPER_MAX_SPEED);
    rudderStepper.setMinPulseWidth(STEPPER_MIN_PULSE_WIDTH);
}

bool performHoming() {
    if (g_autopilotState.homingComplete) {
        return true;
    }
    
    rudderStepper.setCurrentPosition(0);
    int32_t currentPosition = 0;
    
    // Move to limit switch (farthest right position)
    while (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
        if (g_outputStream) {
            g_outputStream->println("Homing...");
        }
        rudderStepper.move(++currentPosition);
        rudderStepper.runToPosition();
        
        // Add safety limit to prevent infinite loop
        if (currentPosition > 10000) {
            if (g_outputStream) {
                g_outputStream->println("Homing failed: limit switch not found");
            }
            return false;
        }
    }
    
    // Stop and set this position as the reference (farthest right)
    rudderStepper.stop();
    rudderStepper.setCurrentPosition(0);
    
    if (g_outputStream) {
        g_outputStream->println("Limit switch reached, moving to center...");
    }
    
    // Calculate center position: halfway between min and max positions
    // RUDDER_POSITION_MIN = -1050, RUDDER_POSITION_MAX = -70
    // Center = (-1050 + -70) / 2 = -560
    // Since we're at -70 (position 0), center is at -560 - (-70) = -490 steps
    int32_t centerPosition = RUDDER_POSITION_CENTER;
    
    // Move to center position
    rudderStepper.moveTo(centerPosition);
    while (rudderStepper.distanceToGo() != 0) {
        rudderStepper.run();
    }
    
    g_autopilotState.homingComplete = true;
    
    if (g_outputStream) {
        g_outputStream->printf("Homing complete - rudder centered at position %ld\n", 
                              rudderStepper.currentPosition());
    }
    return true;
}

void updateRudderPosition(AutopilotState& state) {
    state.targetMotorPosition = map(static_cast<long>(state.rudderAngle), 
                                   static_cast<long>(MIN_RUDDER_ANGLE), 
                                   static_cast<long>(MAX_RUDDER_ANGLE), 
                                   RUDDER_POSITION_MIN, RUDDER_POSITION_MAX);
    
    // Non-blocking motor control
    rudderStepper.moveTo(state.targetMotorPosition);
    rudderStepper.run();
    
    state.rudderPosition = rudderStepper.currentPosition();
}

// =============================================================================
// PID CONTROL
// =============================================================================

void calculateRudderAngle(AutopilotState& state) {
    // Reset integral when destination changes (but not on zero crossing)
    bool shouldResetIntegral = 
        isDestinationChanged(state, state.navigation.destinationLatitude, state.navigation.destinationLongitude);
    
    if (shouldResetIntegral) {
        state.integralXTE = 0.0f;
    }

    // Initialize filters on first run or reset filters if navigation data was invalid
    if (!state.filtersInitialized || state.previousXte == -1.0f) {
        state.filteredXTE = state.navigation.xte;
        state.filteredHeading = state.navigation.heading;
        state.filtersInitialized = true;
    } else {
        // Apply low-pass filtering to reduce noise
        state.filteredXTE = (XTE_FILTER_ALPHA * state.navigation.xte) + 
                           ((1.0f - XTE_FILTER_ALPHA) * state.filteredXTE);
        
        // Handle heading wraparound for filtering
        float headingDiff = normalizeHeading(state.navigation.heading - state.filteredHeading);
        state.filteredHeading = state.filteredHeading + (HEADING_FILTER_ALPHA * headingDiff);
        if (state.filteredHeading >= 360.0f) state.filteredHeading -= 360.0f;
        if (state.filteredHeading < 0.0f) state.filteredHeading += 360.0f;
    }

    // Use filtered cross-track error as the primary error signal for all PID terms
    float xteError = -state.filteredXTE; // Negative because positive XTE means turn left (negative rudder)

    // PID calculations - all based on filtered XTE
    state.integralXTE += xteError * state.timeDelta;
    state.integralXTE = constrain(state.integralXTE, -MAX_INTEGRAL_WINDUP, MAX_INTEGRAL_WINDUP);
    
    // Reset integral if it becomes disproportionately large
    float proportionalTerm = state.pid.kp * xteError;
    float integralTerm = state.pid.ki * state.integralXTE;
    if (abs(integralTerm) > 3.0f * abs(proportionalTerm) && abs(integralTerm) > 10.0f) {
        if (g_outputStream) {
            g_outputStream->printf("INTEGRAL RESET: I-term %.2f too large vs P-term %.2f\n", 
                                  integralTerm, proportionalTerm);
        }
        state.integralXTE *= 0.5f; // Reduce by half instead of complete reset
    }

    if (state.timeDelta > 0) {
        // Correct derivative calculation: rate of change of error
        float previousXteError = -state.previousXte;
        state.derivativeXTE = (xteError - previousXteError) / state.timeDelta;
    } else {
        state.derivativeXTE = 0.0f;
    }

    // Calculate rudder angle using consistent PID control
    double rudderAngle = (state.pid.kp * xteError) + 
                        (state.pid.ki * state.integralXTE) + 
                        (state.pid.kd * state.derivativeXTE);

    // Add heading correction to prevent oscillation (using filtered heading)
    double headingError = normalizeHeading(state.navigation.bearingToWaypoint - state.filteredHeading);
    state.headingCorrection = state.pid.heading_gain * headingError;
    
    rudderAngle += state.headingCorrection;

    // Anti-windup protection: Don't accumulate integral if output is saturated
    bool rudderSaturated = (rudderAngle >= MAX_RUDDER_ANGLE || rudderAngle <= MIN_RUDDER_ANGLE);
    if (rudderSaturated) {
        // If saturated and error would make it worse, don't accumulate integral
        if ((rudderAngle >= MAX_RUDDER_ANGLE && xteError > 0) || 
            (rudderAngle <= MIN_RUDDER_ANGLE && xteError < 0)) {
            // Back out the integral update we just added
            state.integralXTE -= xteError * state.timeDelta;
            if (g_outputStream) {
                g_outputStream->printf("ANTI-WINDUP: Rudder saturated at %.1f, integral held at %.2f\n", 
                                      rudderAngle, state.integralXTE);
            }
        }
    }

    // Apply rate limiting for safety
    if (state.timeDelta > 0) {
        float maxRudderChange = MAX_RUDDER_RATE * state.timeDelta;
        float rudderChange = static_cast<float>(rudderAngle) - state.previousRudderAngle;
        rudderChange = constrain(rudderChange, -maxRudderChange, maxRudderChange);
        rudderAngle = state.previousRudderAngle + rudderChange;
    }

    // Clamp rudder angle to valid range
    state.rudderAngle = constrain(static_cast<float>(rudderAngle), MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE);
    
    // Store for next iteration
    state.previousRudderAngle = state.rudderAngle;

    // Update previous values
    state.previousXte = state.navigation.xte;
    state.previousBearing = state.navigation.bearingToWaypoint;
    state.previousDestinationLat = state.navigation.destinationLatitude;
    state.previousDestinationLon = state.navigation.destinationLongitude;
}

// =============================================================================
// SYSTEM INITIALIZATION
// =============================================================================

void initializeNMEA2000() {
    NMEA2000.SetDeviceCount(1);
    NMEA2000.SetProductInformation(
        "123434",                    // Model serial code
        444,                         // Product code
        "3Fours Autopilot",         // Model ID
        "1.0.0",                    // Software version
        "1.0.0",                    // Model version
        0xff,                       // Load equivalency (default)
        0xffff,                     // NMEA 2000 version (default)
        0xff,                       // Certification level (default)
        0
    );

    NMEA2000.SetDeviceInformation(
        123434,                     // Unique device ID
        150,                        // Device class
        40,                         // Device function
        444,                        // Industry group
        4,                          // Marine
        0
    );

    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
    NMEA2000.SetForwardStream(g_outputStream);
    NMEA2000.EnableForward(false);
    NMEA2000.SetMsgHandler(handleNMEA2000Message);
    NMEA2000.Open();
}

void initializeSystem() {
    // Initialize serial communication
    Serial.begin(SERIAL_DEBUG_BAUD_RATE);
    Serial2.begin(SERIAL_BAUD_RATE, SERIAL_8N1);
    g_outputStream = &Serial;

    // Load PID parameters from EEPROM
    g_autopilotState.pid = loadPIDParameters();

    // Initialize hardware
    initializeStepperMotor();
    
    // Initialize GPIO pins
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH); // Disable motor initially

    // Initialize NMEA 2000
    initializeNMEA2000();

    // Start serial communication thread
    threads.addThread(serialThread);

    if (g_outputStream) {
        g_outputStream->println("3Fours Autopilot System Initialized");
    }
}

// =============================================================================
// DEBUG AND MONITORING
// =============================================================================

void printSystemStatus(const AutopilotState& state) {
    if (!g_outputStream) return;
    
    g_outputStream->printf("PID: kp=%.2f ki=%.3f kd=%.2f heading_gain=%.2f\n", 
                          state.pid.kp, state.pid.ki, state.pid.kd, state.pid.heading_gain);
    g_outputStream->printf("Navigation: heading=%.1f(%.1f) xte=%.2f(%.2f) bearing=%.1f\n",
                          state.navigation.heading, state.filteredHeading, 
                          state.navigation.xte, state.filteredXTE, state.navigation.bearingToWaypoint);
    
    // Calculate individual PID terms for display
    float xteError = -state.filteredXTE;
    float pTerm = state.pid.kp * xteError;
    float iTerm = state.pid.ki * state.integralXTE;
    float dTerm = state.pid.kd * state.derivativeXTE;
    
    g_outputStream->printf("PID Terms: P=%.2f I=%.2f(%.1f) D=%.2f HeadingCorr=%.2f\n",
                          pTerm, iTerm, state.integralXTE, dTerm, state.headingCorrection);
    
    // Check for saturation warning
    bool saturated = (state.rudderAngle >= MAX_RUDDER_ANGLE - 0.1f || 
                     state.rudderAngle <= MIN_RUDDER_ANGLE + 0.1f);
    
    g_outputStream->printf("Rudder: angle=%.1f%s pos=%ld target=%ld\n",
                          state.rudderAngle, 
                          saturated ? " [SATURATED]" : "",
                          state.rudderPosition, state.targetMotorPosition);
    g_outputStream->printf("Timing: delta=%.3f integral=%.2f derivative=%.2f\n", 
                          state.timeDelta, state.integralXTE, state.derivativeXTE);
    g_outputStream->println("==================================");
}

// =============================================================================
// MAIN PROGRAM
// =============================================================================

void setup() {
    initializeSystem();
}

void loop() {
    // Parse NMEA 2000 messages
    NMEA2000.ParseMessages();

    // Update timing with protection against large jumps
    uint32_t currentTime = millis();
    if (g_autopilotState.previousTime == 0) {
        // First run - initialize
        g_autopilotState.previousTime = currentTime;
        g_autopilotState.timeDelta = 0.0f;
    } else {
        float deltaMs = static_cast<float>(currentTime - g_autopilotState.previousTime);
        g_autopilotState.timeDelta = deltaMs / 1000.0f;
        
        // Limit maximum time delta to prevent issues after pauses
        if (g_autopilotState.timeDelta > MAX_TIME_DELTA) {
            g_autopilotState.timeDelta = MAX_TIME_DELTA;
        }
        
        g_autopilotState.previousTime = currentTime;
    }

    // Process incoming serial data for PID parameter updates
    processSerialData(g_autopilotState);

    // Check if we have valid navigation data
    if (!g_autopilotState.navigation.isNavigationDataValid()) {
        if (g_outputStream) {
            g_outputStream->println("Autopilot off: invalid navigation data");
        }
        // Disable motor power and reset homing
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
        g_autopilotState.homingComplete = false;
        delay(MAIN_LOOP_DELAY_MS);
        return;
    }

    // Enable motor power
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    // Perform homing if not complete
    if (!performHoming()) {
        digitalWrite(MOTOR_ENABLE_PIN, HIGH);
        delay(MAIN_LOOP_DELAY_MS);
        return;
    }

    // Calculate and apply rudder control
    calculateRudderAngle(g_autopilotState);
    updateRudderPosition(g_autopilotState);

    // Print system status for debugging
    // printSystemStatus(g_autopilotState);

    // Main loop delay
    delay(MAIN_LOOP_DELAY_MS);
}