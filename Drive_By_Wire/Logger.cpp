
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <stdio.h>
#include <Arduino.h>
#include "DBW_Pins.h"
#include "Vehicle.h"
#include "Can_Protocol.h"

tmElements_t tm;
extern Vehicle *myTrike;

/****************************************************************************
 Data Logger
 Record all stste data on each time slice.
 This is the priomary method for debugging. Add new parameters as needed.
 The data log provides information useful for crafting control strategies and simulators.

 The original method was to log to an SD card.
 Options have been added to log to CAN or a serial connection.
 The CAN device handling logging will respond to the appropriate messages and write the log.

 The serial device could be any of the four lines on the Due
 Serial goes to the serial monitor on the Due, but is confounded by other messages
 On the Bridge circuit board, DBW Serial1 is connected to router serial 2.
 On the DBW v5 board, Serial2 goes to connector J9
 Serial3 could also be used.  
 The receiving serial device could display data on its serial monitor or write it to a file.

 In DBW_Pins.h, set serialLOG to logfile (for SD) or to one of the four serial ports.
 In Logger::Initialize, set logMethod to 0 for SD, 1 for Serial or 2 for CAN.

   Constructor
 ****************************************************************************/
Logger::Logger()
{
  initialize();
#ifndef USE_NATIVE_USB
  Serial.println("Logger constructor finished.");
#endif
}

/*****************************************************************************
   Destructor
 ****************************************************************************/
Logger::~Logger() {
}

/*******************************
Initialize RTC and SD Card
********************************/ 
void Logger::initialize() {
#ifdef HAS_RTC
  initRTC();
#endif
  // Decide whether an SD or serial sink is available. CAN log emit is unconditional below.
  // serialLOG (in Settings.h) is either `logfile` (SD) or a Serial port.
  logMethod = (serialLOG == logfile) ? 0 : 1;
  if (logMethod == 0) {
    if (!initSD() || !openSD()) {
      logMethod = 2;  // SD failed — no local sink, CAN emit only
    }
  } else {
#if (serialLOG != logfile)
    if (serialLOG != Serial) {  // Serial monitor was begun in setup()
      serialLOG.begin(115200);
    }
#endif
  }

  // CAN: always emit session header so any listener (Sensor Hub Due, debug laptop, etc.)
  // sees a new session on the bus regardless of local SD/serial availability.
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = Header_CANID;
  outCAN->length = 0;
  myTrike->sendCan();
#ifndef USE_NATIVE_USB
  Serial.println("CAN log session opened (0x700).");
#endif

  // SD/serial: write CSV header only if that sink is available.
  if (logMethod != 2) {
    serialLOG.print(VEHICLE_NAME);
    serialLOG.print(",");
    serialLOG.print(timeString);
    serialLOG.print(",");
    serialLOG.println(dateString);
    HdrTime();
    HdrRC();
    HdrDesired();
    HdrThrottle();
    HdrBrakes();
    HdrSteer();
    HdrEndLine();
#ifndef USE_NATIVE_USB
    Serial.println(logMethod == 0 ? "SD log initialized." : "Serial log initialized.");
#endif
  } else {
#ifndef USE_NATIVE_USB
    Serial.println("No SD/serial sink — CAN log emit only.");
#endif
  }
}
/*******************************
Initialize real time clock
********************************/ 
bool Logger::initRTC()
{
/*
typedef struct  { 
  uint8_t Second; 
  uint8_t Minute; 
  uint8_t Hour; 
  uint8_t Wday;   // day of week, sunday is day 1
  uint8_t Day;
  uint8_t Month; 
  uint8_t Year;   // offset from 1970; 
} 	tmElements_t
*/
  tm.Hour = (uint8_t)18;
  tm.Minute = (uint8_t)8;
  tm.Second = (uint8_t)0;
  tm.Wday = (uint8_t)5;
  tm.Day = (uint8_t)23;
  tm.Month = (uint8_t)10;
  tm.Year = (uint8_t)50;
  // set needs an arg of time_t  RTC.set(tm);
  // time_t is an unsigned long of seconds since 1/1/1970
  time_t  epochSeconds= 0;
  // epochSeconds += tm.Minute *60;
  // epochSeconds += tm.Hour*60*60;
  // epochSeconds += tm.Day*24*60*60;
  epochSeconds += tm.Year*365*24*60*60;
  epochSeconds += tm.Month *30*24*60*60;
  // RTC.set(epochSeconds);
  // setTime(epochSeconds);

  strncpy(timeString, CompileTime,12);
  strncpy(dateString, CompileDate,12); 
  if (RTC.read(tm)) {
    Serial.print("Ok, Time = ");
    sprintf(timeString, "%2.2u:%2.2u:%2.2u\0",tm.Hour,tm.Minute,tm.Second);
    Serial.print(timeString);
    Serial.print(", Date (D/M/Y) = ");
    sprintf(dateString, "%2.2u/%2.2u/%4.4u\0",tm.Day, tm.Month, tm.Year+1970);
    Serial.println(dateString);
    return (true);
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped. Please run SetTime.");
    } else {
      Serial.println("DS1307 read error!");
    }
  }
  return (false);
}
/*******************************
Initialize SD Card
********************************/ 
bool Logger::initSD()
{
  // Initialize SD Card
  Serial.print("Initializing SD card...");
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(10,OUTPUT);   // Shield may use pin 10

  if (!SD.begin(SD_CS_PIN)) {
    
    logfile = File();  // mark logfile as invalid
    return (false);
  }
 
  return (true);
}
/*******************************
Open the SD Card
********************************/ 
bool Logger::openSD()
{
  //Generate Unique Filename
  char filename[13];
  for (int i = 0; i < 100; i++) {
    sprintf(filename, "LOG%02d.CSV", i);
    if (!SD.exists(filename)) {
      break;
    }
  }
  logfile = SD.open(filename, FILE_WRITE);
  if (!logfile) {
    Serial.print("FAILED to open file: ");
    Serial.println(filename);
       return (false);
  }
  Serial.print("Logging to: ");
  Serial.println(filename);
  return (true);
}
/****************************************
Write a new set of data to all available sinks.
  - CAN log emit is unconditional (every loop tick puts 0x701-0x70A on the bus).
  - SD/serial write only if logMethod != 2 (i.e. that sink was successfully initialized).
****************************************/
void Logger::update() {
  // CAN log emit — always on. Sensor Hub Due (and any other listener) picks up from the bus.
  CANLogTime();      // 0x701
  CANLogRC();        // 0x702
  CANLogOp();        // 0x703
  CANLogAuto();      // 0x704
  CANLogDesired();   // 0x705
  CANLogThrottle();  // 0x706
  CANLogBrakes();    // 0x707
  CANLogSteer();     // 0x708
  // 0x709 LogPosition is emitted by Nav (not DBW) — left for the Sensor Hub log row.

  // SD/serial: write a CSV row if that sink is available.
  if (logMethod != 2) {
    TxTime();
    TxLogRC();
    TxDesired();
    TxThrottle();
    TxBrakes();
    TxSteer();
  }
  // Caller must finish the row with EndLine() (emits 0x70A and terminates CSV line).
}
/******************************************************
Line starts with a relative time stamp in milliseconds
*******************************************************/ 
void Logger::HdrTime() {
    serialLOG.print("time_ms");
}
void Logger::TxTime()
{
   serialLOG.print(millis());
}
// 0x701 LogTime — 4 bytes uint32 LE: relative time in milliseconds.
void Logger::CANLogTime() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogTime_CANID;
  outCAN->length = 4;
  outCAN->data.uint32[0] = millis();
  myTrike->sendCan();
}
/******************************************************
Include what has been commanded by Radio Control
*******************************************************/ 
void Logger::HdrRC() {
  // expected order
  // serialLOG.print(",Ch1,Ch2,Ch3,Ch4,Ch5,Ch6");
  // serialLOG.print(",Map1,Map2,DriveMode,AutoMode,Map5,Map6");
  // What we see

    serialLOG.print(",Ch1Str,Ch2ThB,Ch3,Ch4,Ch5,Ch6");
    serialLOG.print(",MapStr,MapThB,Map3,AutoMode,Map5,EStopBtn");
}

void Logger::TxLogRC() {
  int i;
  unsigned long data;
  long mappd;
  static bool first_time = true; 
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    data = getRCtime(*myTrike, i);
    serialLOG.print(",");
    serialLOG.print(data);
  }
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    mappd = getRCmapped(*myTrike, i);
    serialLOG.print(",");
    serialLOG.print(mappd);
  }
  if (first_time)
  {  // Show on serial monitor. Skipped on Bridge board: Serial (pins 0/1) connects to Router Arduino and blocks if not reading
    #ifndef USE_NATIVE_USB
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      data = getRCtime(*myTrike, i);
      Serial.print(data);
      Serial.print(", ");
    }
    Serial.println(" ");
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      mappd = getRCmapped(*myTrike, i);
      Serial.print(mappd);
      Serial.print(", ");
    }
    Serial.println(" ");
    #endif
    first_time = false;
  }
}
// 0x702 LogRC — 8 bytes: 4 RC channel pulse widths as int16 LE microseconds.
// RC has 6 channels; emit the 4 most useful (CH1 steer, CH2 throttle/brake, CH3 fwd/rev, CH4 mode).
void Logger::CANLogRC() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogRC_CANID;
  outCAN->length = 8;
  outCAN->data.int16[0] = (int16_t)getRCtime(*myTrike, CH1);
  outCAN->data.int16[1] = (int16_t)getRCtime(*myTrike, CH2);
  outCAN->data.int16[2] = (int16_t)getRCtime(*myTrike, CH3);
  outCAN->data.int16[3] = (int16_t)getRCtime(*myTrike, CH4);
  myTrike->sendCan();
}
/******************************************************
Include wthe goals, either from RC or CAN
*******************************************************/ 
void Logger::HdrDesired() {
  serialLOG.print(",desired_speed_ms,desired_brake,desired_angle_DegX10");
}
void Logger::TxDesired() {
  serialLOG.print(",");
  serialLOG.print( getD_speed_cmPs(*myTrike));
  serialLOG.print(",");
  serialLOG.print( getD_brakes(*myTrike));
  serialLOG.print(",");
  serialLOG.print( getD_Angle(*myTrike));
}
// 0x705 LogDesired — 6 bytes: the active source's commands (same shape as 0x350).
//   bytes 0-1: speed int16 LE (cm/s)
//   byte  2:   brake uint8 (wiki: 0=off, 1=hold, 2=on)
//   byte  3:   mode  uint8 (DBW's current AutoMode, 0-7 — matches wiki mode table)
//   bytes 4-5: steer angle int16 LE (deg x 10)
void Logger::CANLogDesired() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogDesired_CANID;
  outCAN->length = 6;
  outCAN->data.int16[0] = getD_speed_cmPs(*myTrike);
  outCAN->data.uint8[2] = (uint8_t)getD_brakes(*myTrike);
  outCAN->data.uint8[3] = (uint8_t)getAutoMode(*myTrike);
  outCAN->data.int16[2] = getD_Angle(*myTrike);
  myTrike->sendCan();
}
/******************************************************
Report sensors and actuators for steering
*******************************************************/ 
void Logger::HdrSteer() {
  serialLOG.print(",current_angle,Rturn,Lturn");
}

void Logger::TxSteer() {
  serialLOG.print(",");
  serialLOG.print(getAngle(*myTrike));
  serialLOG.print(",");
  serialLOG.print(digitalRead(RIGHT_TURN_PIN));
  serialLOG.print(",");
  serialLOG.print(digitalRead(LEFT_TURN_PIN));
} 
// 0x708 LogSteer — 6 bytes: actual steer angle + L/R column sensor analog readings.
//   bytes 0-1: actual_angle int16 LE (deg x 10)
//   bytes 2-3: R column sensor int16 (raw analogRead, 0-1023)
//   bytes 4-5: L column sensor int16 (raw analogRead, 0-1023)
void Logger::CANLogSteer() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogSteer_CANID;
  outCAN->length = 6;
  outCAN->data.int16[0] = getAngle(*myTrike);
  outCAN->data.int16[1] = (int16_t)analogRead(R_SENSE_PIN);
  outCAN->data.int16[2] = (int16_t)analogRead(L_SENSE_PIN);
  myTrike->sendCan();
}
/******************************************************
Report sensors and actuators for propulsion
*******************************************************/ 
void Logger::HdrThrottle() {
  serialLOG.print(",current_speed,driveMode");
}
void Logger::TxThrottle() {
  serialLOG.print(",");
  serialLOG.print(getSpeed(*myTrike));
  // serialLOG.print(",");
  // serialLOG.print(throttlePulse_ms);
  serialLOG.print(",");
  serialLOG.print(getDriveMode(*myTrike));
}
// 0x706 LogThrottle — 4 bytes: actual_speed + throttle_pwm + drive_mode.
//   bytes 0-1: actual_speed int16 LE (cm/s)
//   byte  2:   throttle_pwm uint8 (0-255; not yet surfaced from SpeedController, emits 0)
//   byte  3:   drive_mode uint8 (FORWARD_MODE/REVERSE_MODE)
void Logger::CANLogThrottle() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogThrottle_CANID;
  outCAN->length = 4;
  outCAN->data.int16[0] = getSpeed(*myTrike);
  outCAN->data.uint8[2] = 0;  // TODO: expose throttle PWM from SpeedController
  outCAN->data.uint8[3] = (uint8_t)getDriveMode(*myTrike);
  myTrike->sendCan();
}
/******************************************************
Report state of the brakes
*******************************************************/ 
void Logger::HdrBrakes() {
  serialLOG.print(",BrakeOn,BrakeVolt");
}
void Logger::TxBrakes()  {
  serialLOG.print(",");
  serialLOG.print(digitalRead(BRAKE_ON_PIN));
  serialLOG.print(",");
  serialLOG.print(digitalRead(BRAKE_VOLT_PIN));
}
// 0x707 LogBrakes — 2 bytes: brake_on (uint8) + brake_voltage (uint8).
void Logger::CANLogBrakes() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogBrakes_CANID;
  outCAN->length = 2;
  outCAN->data.uint8[0] = (uint8_t)digitalRead(BRAKE_ON_PIN);
  outCAN->data.uint8[1] = (uint8_t)digitalRead(BRAKE_VOLT_PIN);
  myTrike->sendCan();
}
/******************************************************
Terminate the line and indicate the part of loop time used
*******************************************************/ 
void Logger::HdrEndLine() {
  serialLOG.println(",Utilization");
}
void Logger::EndLine(uint32_t delayTime) {
  int PerCentBusy = ((LOOP_TIME_MS - delayTime) * 100) / LOOP_TIME_MS;
  CANLogFinalize(PerCentBusy);   // 0x70A — always emit so the receiver knows the row is complete
  if (logMethod != 2) {
    serialLOG.print(",");
    serialLOG.println(PerCentBusy);
    serialLOG.flush();   // ensure data is written immediately
  }
}

// 0x703 LogOp — 8 bytes: operator panel state.
//   bytes 0-1: joystick X (int16, raw analogRead of OP_STEER, 0-1023)
//   bytes 2-3: joystick Y (int16, raw analogRead of OP_THROTTLE, 0-1023)
//   byte  4:   switch bitfield (0x10 fwd/rev, 0x20 auto/manual, 0x40 disconnect, 0x80 estop)
//   byte  5:   commanded brake (uint8)
//   bytes 6-7: commanded steer angle (int16 LE, deg x 10)
void Logger::CANLogOp() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogOp_CANID;
  outCAN->length = 8;
  outCAN->data.int16[0] = (int16_t)analogRead(OP_STEER);
  outCAN->data.int16[1] = (int16_t)analogRead(OP_THROTTLE);
  uint8_t sw = 0;
  if (digitalRead(OP_FWD_PIN))    sw |= 0x10;
  if (digitalRead(OP_MODE_PIN))   sw |= 0x20;
  if (digitalRead(OP_DISCNT_PIN)) sw |= 0x40;
  if (digitalRead(OP_ESTOP))      sw |= 0x80;
  outCAN->data.uint8[4] = sw;
  outCAN->data.uint8[5] = (uint8_t)getD_brakes(*myTrike);
  outCAN->data.int16[3] = getD_Angle(*myTrike);
  myTrike->sendCan();
}

// 0x704 LogAuto — 7 bytes: what DBW received from Nav (mirrors 0x350 + 0x100 status byte).
//   bytes 0-1: nav_speed int16 LE (cm/s)
//   byte  2:   nav_brake uint8 (0/1/2)
//   byte  3:   nav_mode  uint8 (0-7, Nav-requested state — DBW arbitrates)
//   bytes 4-5: nav_angle int16 LE (deg x 10)
//   byte  6:   nav_status uint8 (0x100 byte 0: 0x80 estop, 0x40 auto, 0x04 reverse, ...)
void Logger::CANLogAuto() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogAuto_CANID;
  outCAN->length = 7;
  outCAN->data.int16[0] = getNavSpeed(*myTrike);
  outCAN->data.uint8[2] = getNavBrake(*myTrike);
  outCAN->data.uint8[3] = getNavMode(*myTrike);
  outCAN->data.int16[2] = getNavAngle(*myTrike);
  outCAN->data.uint8[6] = getNavStatus(*myTrike);
  myTrike->sendCan();
}

// 0x70A LogFinalize — 1 byte: CPU utilization (%) for the loop tick.
void Logger::CANLogFinalize(int PerCentBusy) {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogFinalize_CANID;
  outCAN->length = 1;
  outCAN->data.uint8[0] = (uint8_t)PerCentBusy;
  myTrike->sendCan();
}