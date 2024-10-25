// Demo: NMEA2000 library. 
// This demo reads messages from NMEA 2000 bus and
// sends them translated to clear text to Serial.

// Note! If you use this on Arduino Mega, I prefer to also connect interrupt line
// for MCP2515 and define N2k_CAN_INT_PIN to related line. E.g. MessageSender
// sends so many messages that Mega can not handle them. If you only use
// received messages internally without slow operations, then youmay survive
// without interrupt.

#include <Arduino.h>
//#include <Time.h>  // 
#define N2k_CAN_INT_PIN 21
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

#include <AccelStepper.h>

#include <SPI.h>
#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library

#include <TeensyThreads.h>
#include <EEPROM.h>

#define TFT_RST    14   // chip reset
#define TFT_DC     9   // tells the display if you're sending data (D) or commands (C)   --> WR pin on TFT
#define TFT_MOSI   11  // Data out    (SPI standard)
#define TFT_SCLK   13  // Clock out   (SPI standard)
#define TFT_CS     10  // chip select (SPI standard)
#define PID_SIZE  12

AccelStepper rudderStepper(AccelStepper::DRIVER, 1, 2); // (DRIVER mode, STEP_PIN, DIR_PIN)

ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
int LCD_BL = 15;       // LCD back light control

typedef struct __attribute__((packed)) {
  float kp;
  float ki;
  float kd;
  float bearingPositionToDestinationWaypoint;
  float destinationLatitude;
  float destinationLongitude;
  float previousDestinationLatitude;
  float previousDestinationLongitude;
  float heading;
  float xte;
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
} Autopilot;

typedef struct {
  float kp;
  float ki;
  float kd;
} PIDs;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void NavigationInfo(const tN2kMsg &N2kMsg);
void XTE(const tN2kMsg &N2kMsg);
void COG(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);

Autopilot* autopilot = new Autopilot;
PIDs* pids = new PIDs;

byte startMarker = 0x02;
byte endMarker = 0x03;

tNMEA2000Handler NMEA2000Handlers[]={
  {129284L,&NavigationInfo},
  {129283L,&XTE},
  {127250L,&Heading},
  {0,0}
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void sendAutopilotData(Autopilot* autopilot) {
  byte buffer[sizeof(Autopilot)];
  memcpy(buffer, autopilot, sizeof(Autopilot));
  Serial2.write(startMarker);
  Serial2.write(buffer, sizeof(Autopilot));
  Serial2.write(endMarker);
}

void SerialThread() {
  while (1) {
    sendAutopilotData(autopilot);
    threads.delay(1000);
  }
}

void savePIDValues() {
  EEPROM.put(0, pids->kp);
  EEPROM.put(4, pids->ki);
  EEPROM.put(8, pids->kd);
}

void loadPIDValues() {
  EEPROM.get(0, pids->kp);
  EEPROM.get(4, pids->ki);
  EEPROM.get(8, pids->kd);
}

void setup() {
  loadPIDValues();
  if (pids->kp <= 0 || pids->kp > 10) {
    pids->kp = 1;
  }
  if (pids->ki <= 0 || pids->ki > 10) {
    pids->kp = 1;
  }
  if (pids->kd <= 0 || pids->kd > 10) {
    pids->kd = 1;
  }

  autopilot->kp = pids->kp;
  autopilot->ki = pids->ki;
  autopilot->kd = pids->kd;
  autopilot->bearingPositionToDestinationWaypoint = -1;
  autopilot->destinationLatitude = 0;
  autopilot->destinationLongitude = 0;
  autopilot->previousDestinationLatitude = 0;
  autopilot->previousDestinationLongitude = 0;
  autopilot->heading = -1;
  autopilot->xte = -1;
  autopilot->previousXte = -1;
  autopilot->previousTime = 0;
  autopilot->previousBearing = 0;
  autopilot->integralXTE = 0;
  autopilot->derivativeXTE = 0;
  autopilot->timeDelta = 0;
  autopilot->rudderAngle = 0;
  autopilot->rudderPosition = 0;
  autopilot->targetMotorPosition = 0;
  autopilot->homingComplete = false;

  rudderStepper.setMaxSpeed(1000); // Adjust based on your motor
  rudderStepper.setAcceleration(50000); // Adjust based on your motor
  rudderStepper.setSpeed(1000); // Steps per second
  rudderStepper.setMinPulseWidth(3);

  pinMode(4, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  digitalWrite(4, HIGH);
          
  OutputStream=&Serial;

  Serial2.begin(9600, SERIAL_8N1);

  NMEA2000.SetDeviceCount(1);
  NMEA2000.SetProductInformation("123434", // Manufacturer's Model serial code. 
                                 444, // Manufacturer's product code
                                 "3Fours Autopilot",  // Manufacturer's Model ID
                                 "1.0.0",  // Manufacturer's Software version code
                                 "1.0.0", // Manufacturer's Model version
                                 0xff, // load equivalency - use default
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 0
                                 );
  NMEA2000.SetDeviceInformation(123434, // Unique number. Use e.g. Serial number.
                                150, // See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                40, // See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                444, // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4, // Marine
                                0
                               );
   
//  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
//  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();

  threads.addThread(SerialThread);
}

double radiansToDegrees(double radians) {
  return radians * (180.0 / PI);
}

void Heading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double Heading;
  double Deviation;
  double Variation;
  tN2kHeadingReference HeadingReference;

   if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, HeadingReference) ) 
    {
      if (Heading <= 360 && Heading >= 0) {
        autopilot->heading = radiansToDegrees(Heading);
      } else {
        autopilot-> heading = -1;
      }
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

void XTE(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kXTEMode XTEMode;
  bool NavigationTerminated;
  double XTE;

   if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE) ) 
    {
      if (XTE > -100 && XTE < 100) {
        autopilot->xte = XTE;
      } else {
        autopilot-> xte = -1;
      }
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

void COG(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double COG;
  double SOG;
  
  if (ParseN2kPGN129026(N2kMsg, SID, HeadingReference, COG, SOG)) {
    if (COG <= 360 && COG >= 0) {
        autopilot->heading = radiansToDegrees(COG);
      } else {
        autopilot->heading = -1;
      }
  }
}

void NavigationInfo(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double DistanceToWaypoint;
    tN2kHeadingReference BearingReference;
    bool PerpendicularCrossed;
    bool ArrivalCircleEntered;
    tN2kDistanceCalculationType CalculationType;
    double ETATime;
    int16_t ETADate;
    double BearingOriginToDestinationWaypoint;
    double BearingPositionToDestinationWaypoint;
    uint32_t OriginWaypointNumber;
    uint32_t DestinationWaypointNumber;
    double DestinationLatitude;
    double DestinationLongitude;
    double WaypointClosingVelocity;

    if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered,
         CalculationType, ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint, OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity) ) 
    {
      if (BearingOriginToDestinationWaypoint <= 360 && BearingPositionToDestinationWaypoint >= 0) {
        autopilot->bearingPositionToDestinationWaypoint = radiansToDegrees(BearingPositionToDestinationWaypoint);
        autopilot->destinationLatitude = DestinationLatitude;
        autopilot->destinationLongitude = DestinationLongitude;
      } else {
        autopilot->bearingPositionToDestinationWaypoint = -1;
      }

    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
  //OutputStream->print("In Main Handler: "); OutputStream->println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

void calculateRudderAngle() {
  // Reset integral when course is reached or the destination changes
  if ((autopilot->xte > 0 && autopilot->previousXte <= 0) || (autopilot->xte < 0 && autopilot->previousXte >= 0) || 
      ((autopilot->destinationLatitude != autopilot->previousDestinationLatitude) && 
      (autopilot->destinationLongitude != autopilot->previousDestinationLongitude))) {
        autopilot->integralXTE = 0;
  }

  // Calculate heading error
  double headingError = autopilot->bearingPositionToDestinationWaypoint - autopilot->heading;
  // Normalize heading error to be within [-180, 180] degrees
  if (headingError > 180) {
    headingError -= 360;
  } else if (headingError < -180) {
    headingError += 360;
  }

  double integralMax = 300;

  // PID calculations
  autopilot->integralXTE += -(autopilot->xte * autopilot->timeDelta);
  if (autopilot->integralXTE > integralMax) {
    autopilot->integralXTE = integralMax;
  } else if (autopilot->integralXTE < -integralMax) {
    autopilot->integralXTE = -integralMax;
  }

  autopilot->derivativeXTE = (autopilot->xte - autopilot->previousXte) / autopilot->timeDelta;
  autopilot->previousXte = autopilot->xte;

  // Calculate rudder angle using PID control
  double rudderAngle = (autopilot->kp * headingError) + (autopilot->ki * autopilot->integralXTE) + (autopilot->kd * autopilot->derivativeXTE);

  // Clamp the rudder angle to a realistic range, e.g., -45 to 45 degrees
  autopilot->rudderAngle = constrain(rudderAngle, -45, 45);
  autopilot->previousBearing = autopilot->bearingPositionToDestinationWaypoint;
  autopilot->previousDestinationLatitude = autopilot->destinationLatitude;
  autopilot->previousDestinationLongitude = autopilot->destinationLongitude;
}

static byte buffer[PID_SIZE];
static int bufferIndex = 0;
bool dataStarted = false;

//*****************************************************************************
void loop() 
{ 
  NMEA2000.ParseMessages();

  double currentTime = millis();
  autopilot->timeDelta = (currentTime - autopilot->previousTime) / 1000.0;
  autopilot->previousTime = currentTime;

  if (Serial2.available()) {
    while (Serial2.available()) {
      byte incomingByte = Serial2.read();
      if (incomingByte == startMarker) {
        bufferIndex = 0;
        dataStarted = true;
      } else if (dataStarted && bufferIndex < PID_SIZE) {
        buffer[bufferIndex++] = incomingByte;
      } else if (incomingByte == endMarker && bufferIndex == PID_SIZE) {
        pids = (PIDs*)buffer;
        autopilot->kp = pids->kp;
        autopilot->ki = pids->ki;
        autopilot->kd = pids->kd;
        savePIDValues();
        bufferIndex = 0;
        dataStarted = false;
      }
    }
  }

  if (autopilot->bearingPositionToDestinationWaypoint == -1 || autopilot->xte == -1) {
    Serial.println("Autopilot off: no bearing or xte");
    // Disable motor power
    digitalWrite(4, HIGH);
    // Reset homing
    autopilot->homingComplete = false;
    return;
  }

  // Enable motor power
  digitalWrite(4, LOW);

  if (!autopilot->homingComplete) {
    rudderStepper.setCurrentPosition(0);
    int currentPosition = 0;
    while(digitalRead(3) == LOW) {
      Serial.println("Homing...");
      rudderStepper.move(currentPosition += 1);
      rudderStepper.runToPosition();
      Serial.println(currentPosition);
    }
    rudderStepper.stop();
    autopilot->homingComplete = true;
    rudderStepper.setCurrentPosition(0);
    Serial.println("Homing complete");
  }

  calculateRudderAngle();

  autopilot->targetMotorPosition = map(autopilot->rudderAngle, -45, 45, -1050, -70);
  rudderStepper.moveTo(autopilot->targetMotorPosition);
  while (rudderStepper.distanceToGo() != 0) {
    rudderStepper.run();
  }

  // Update the current rudder position
  autopilot->rudderPosition = autopilot->targetMotorPosition;

  Serial.print("kp: ");
  Serial.println(autopilot->kp);
  Serial.print("ki: ");
  Serial.println(autopilot->ki);
  Serial.print("kd: ");
  Serial.println(autopilot->kd);
  Serial.print("destinationLatitude: ");
  Serial.println(autopilot->destinationLatitude);
  Serial.print("destinationLongitude: ");
  Serial.println(autopilot->destinationLongitude);
  Serial.print("bearingPositionToDestinationWaypoint: ");
  Serial.println(autopilot->bearingPositionToDestinationWaypoint);
  Serial.print("heading: ");
  Serial.println(autopilot->heading);
  Serial.print("xte: ");
  Serial.println(autopilot->xte);
  Serial.print("previousXte: ");
  Serial.println(autopilot->previousXte);
  Serial.print("previousTime: ");
  Serial.println(autopilot->previousTime);
  Serial.print("previousBearing: ");
  Serial.println(autopilot->previousBearing);
  Serial.print("integralXTE: ");
  Serial.println(autopilot->integralXTE);
  Serial.print("derivativeXTE: ");
  Serial.println(autopilot->derivativeXTE);
  Serial.print("timeDelta: ");
  Serial.println(autopilot->timeDelta);
  Serial.print("rudderAngle: ");
  Serial.println(autopilot->rudderAngle);
  Serial.print("rudderPosition: ");
  Serial.println(autopilot->rudderPosition);
  Serial.print("targetMotorPosition: ");
  Serial.println(autopilot->targetMotorPosition);
  Serial.println("==================================");

  // Delay for stability (adjust as needed)
  delay(100);  
}