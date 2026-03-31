
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
extern Vehicle myTrike;

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
  Serial.println("Logger constructor finished.");
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
  initRTC();

  logMethod = serialLOG==logfile? 0:1;  // set 0 for SD, 1 for serial, 2 for CAN
  // logMethod = 2;  // CAN
  if (logMethod == 0 && !initSD())
  {   // SD Card failed");
    logMethod = 2;   // USE CAN
  }
 
 #if (serialLOG != logfile)
  if (logMethod == 1 && serialLOG != Serial)
  {  // if using serial monitor, it has already been started
     serialLOG.begin(115200);
  }
#endif
  if (logMethod == 0 && !openSD())
    logMethod == 2;  // use CAN if SD doesn't work
 
  if (logMethod == 2)
  {  // CAN will open a file and write header information
    CAN_FRAME* outCAN = getCAN(myTrike);
    outCAN->length = 0;
    outCAN->id = Header_CANID;
    myTrike.sendCan();
    CANlogID = Log_CANID;
    Serial.println("CAN log initialized");
  }
  else  // SD or serial
  {
    //Write CSV Header 
    serialLOG.print(VEHICLE_NAME);
    serialLOG.print(",");
    serialLOG.print(timeString);
    serialLOG.print(",");
    serialLOG.println(dateString);   
    // Write snd line of header
    HdrTime();
    HdrRC();
    HdrDesired();
    HdrThrottle();
    HdrBrakes();
    HdrSteer();
    HdrEndLine();
    if (logMethod == 0)
      Serial.println("SD log initialized.");
    else
      Serial.println("Serial Log initialized.");
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
Write a new set of data to the SD card or serial or CAN
****************************************/ 
void Logger::update(){
 
  if (logMethod != 2)
  {
    TxTime();
    TxLogRC();
    TxDesired();
    TxThrottle();
    TxBrakes();
    TxSteer();  
  }
  else
  {
    CANTime();
    CANLogRC();
    CANDesired();
    CANThrottle();
    CANBrakes();
    CANSteer();  
 }
  // Need to finish with EndLine();
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
void Logger::CANTime()
{
  unsigned long time = millis();
  CAN_FRAME* outCAN = getCAN(myTrike);
  outCAN->length = 4;
  outCAN->id = CANlogID++;
  outCAN->data.uint32[0] = time;
  myTrike.sendCan();
}
/******************************************************
Include what has been commanded by Radio Control
*******************************************************/ 
void Logger::HdrRC() {
  // expected order
  // serialLOG.print(",Ch1,Ch2,Ch3,Ch4,Ch5,Ch6");
  // serialLOG.print(",Map1,Map2,DriveMode,AutoMode,Map5,Map6");
  // What we see

    serialLOG.print(",Ch4,Ch3,Ch6,Ch1Str,Ch2ThB,Ch5");
    serialLOG.print(",Map5,Map6,MapStr,MapThB,Map3,AutoMode");
}

void Logger::TxLogRC() {
  int i;
  unsigned long data;
  long mappd;
  static bool first_time = true; 
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    data = getRCtime(myTrike, i);
    serialLOG.print(",");
    serialLOG.print(data);
  }
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    mappd = getRCmapped(myTrike, i);
    serialLOG.print(",");
    serialLOG.print(mappd);
  }
  if (first_time)
  {  // Show on serial monitor
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      data = getRCtime(myTrike, i);
      Serial.print(data);
      Serial.print(", ");
   }
   Serial.println(" ");
   for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    mappd = getRCmapped(myTrike, i);
    Serial.print(mappd);
    Serial.print(", ");
   }
    Serial.println(" ");
    first_time = false;
  }
}
void Logger::CANLogRC() {
  int i;
  unsigned long data;
  long mappd;
  CAN_FRAME* outCAN = getCAN(myTrike);

  outCAN->length = 2;  // assumes RC_NUM_SIGNALS is even
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    data = getRCtime(myTrike, i++);
    outCAN->data.uint32[0] = data;
    data = getRCtime(myTrike, i);
    outCAN->data.uint32[1] = data;
    outCAN->id = CANlogID++;
    myTrike.sendCan();
    }
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    mappd = getRCmapped(myTrike, i++);
    outCAN->data.int32[0] = mappd;
    mappd = getRCmapped(myTrike, i);
    outCAN->data.int32[1] = data;
    outCAN->id = CANlogID++;
    myTrike.sendCan();
  }
}
/******************************************************
Include wthe goals, either from RC or CAN
*******************************************************/ 
void Logger::HdrDesired() {
  serialLOG.print(",desired_speed_ms,desired_brake,desired_angle_DegX10");
}
void Logger::TxDesired() {
  serialLOG.print(",");
  serialLOG.print( getD_speed_cmPs(myTrike));
  serialLOG.print(",");
  serialLOG.print( getD_brakes(myTrike));
  serialLOG.print(",");
  serialLOG.print( getD_Angle(myTrike));
}
void Logger::CANDesired() {
  unsigned long data;
  CAN_FRAME* outCAN = getCAN(myTrike);
  outCAN->length = 8;
  outCAN->id = CANlogID++;
  data = getD_speed_cmPs(myTrike);
  outCAN->data.uint32[0] = data;
  data = getD_brakes(myTrike);
  outCAN->data.uint32[1] = data;
  myTrike.sendCan();

  outCAN->length = 4;
  outCAN->id = CANlogID++;
  data = getD_Angle(myTrike);
  outCAN->data.uint32[0] = data;
  myTrike.sendCan();
}
/******************************************************
Report sensors and actuators for steering
*******************************************************/ 
void Logger::HdrSteer() {
  serialLOG.print(",current_angle,Rturn,Lturn");
}

void Logger::TxSteer() {
  serialLOG.print(",");
  serialLOG.print(getAngle(myTrike));
  serialLOG.print(",");
  serialLOG.print(digitalRead(RIGHT_TURN_PIN));
  serialLOG.print(",");
  serialLOG.print(digitalRead(LEFT_TURN_PIN));
} 
void Logger::CANSteer() {
  
  unsigned long data;
  int turn;
  CAN_FRAME* outCAN = getCAN(myTrike);
  outCAN->length = 8;
  outCAN->id = CANlogID++;
   data = getAngle(myTrike);
  outCAN->data.uint32[0] = data;
  turn = digitalRead(RIGHT_TURN_PIN);
  outCAN->data.int16[2] = turn;
  turn = digitalRead(LEFT_TURN_PIN);
  outCAN->data.int16[3] = turn;
  myTrike.sendCan();
} 
/******************************************************
Report sensors and actuators for propulsion
*******************************************************/ 
void Logger::HdrThrottle() {
  serialLOG.print(",current_speed,driveMode");
}
void Logger::TxThrottle() {
  serialLOG.print(",");
  serialLOG.print(getSpeed(myTrike));
  // serialLOG.print(",");
  // serialLOG.print(throttlePulse_ms);
  serialLOG.print(",");
  serialLOG.print(getDriveMode(myTrike));
}
void Logger::CANThrottle() {
  unsigned long data;
  int mode;
  CAN_FRAME* outCAN = getCAN(myTrike);
  outCAN->length = 8;
  outCAN->id = CANlogID++;
  data = getSpeed(myTrike);
  outCAN->data.uint32[0] = data;
  mode = getDriveMode(myTrike);
  outCAN->data.int16[3] = mode;
  myTrike.sendCan();
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
void Logger::CANBrakes()  {
  char brake;
  CAN_FRAME* outCAN = getCAN(myTrike);
  outCAN->length = 2;
  outCAN->id = CANlogID++;
  brake = digitalRead(BRAKE_ON_PIN);
  outCAN->data.int8[0] = brake;
  brake = digitalRead(BRAKE_VOLT_PIN);
  outCAN->data.int8[1] = brake;
  myTrike.sendCan();
}
/******************************************************
Terminate the line and indicate the part of loop time used
*******************************************************/ 
void Logger::HdrEndLine() {
  serialLOG.println(",Utilization");
}
void Logger::EndLine(uint32_t delayTime) {
  int PerCentBusy = ((LOOP_TIME_MS - delayTime)*100)/LOOP_TIME_MS;
  if (logMethod == 3)
  {
    CAN_FRAME* outCAN = getCAN(myTrike);
    outCAN->length = 2;
    outCAN->id = CANlogID;
    outCAN->data.int16[0] = PerCentBusy;
    myTrike.sendCan();   
    CANlogID = Log_CANID;
  }
  else
  {
    serialLOG.print(",");
    serialLOG.println(PerCentBusy);
    serialLOG.flush();   // Flush the file to make sure data is written immediately
  }
}