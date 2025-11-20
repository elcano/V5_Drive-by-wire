
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <stdio.h>
#include <Arduino.h>
#include "DBW_Pins.h"
#include "Vehicle.h"

tmElements_t tm;
extern Vehicle myTrike;

/****************************************************************************
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
  Serial.println("initialize() starting");
  if (!initRTC())
  {
  }
  if (!initSD())
  {
    Serial.println("Card failed, or not present");
    return;
  }
  else
    Serial.println("Card initialized.");
  if (openSD())
  {
    //Write CSV Header 
    logfile.print(VEHICLE_NAME);
    logfile.print(",");
    logfile.print(timeString);
    logfile.print(",");
    logfile.println(dateString);
    
    HdrTime();
    HdrRC();
    HdrDesired();
    HdrThrottle();
    HdrBrakes();
    HdrSteer();
    HdrEndLine();
  }
  Serial.println("initialize() finished");
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
Write a new set of data to the SD card
****************************************/ 
void Logger::update(){
 // Log data to the file
 if (!logfile) {
  return;
}
  Time();
  LogRC();
  Desired();
  Throttle();
  Brakes();
  Steer();  
  // Need to finish with EndLine();
}
/******************************************************
Line starts with a relative time stamp in milliseconds
*******************************************************/ 
void Logger::HdrTime() {
  logfile.print("time_ms");
}
void Logger::Time()
{
   logfile.print(millis());
}
/******************************************************
Include what has been commanded by Radio Control
*******************************************************/ 
void Logger::HdrRC() {
  // expected order
  // logfile.print(",Ch1,Ch2,Ch3,Ch4,Ch5,Ch6");
  // logfile.print(",Map1,Map2,DriveMode,AutoMode,Map5,Map6");
  // What we see
  logfile.print(",Ch4,Ch3,Ch6,Ch1Str,Ch2ThB,Ch5");
  logfile.print(",Map5,Map6,MapStr,MapThB,Map3,AutoMode");
}
void Logger::LogRC() {
  int i;
  long mappd;
  static bool first_time = true; 
  logfile.print(",");
  logfile.print( getRCtime1(myTrike));
  for (i = 1; i < RC_NUM_SIGNALS; i++) {
    logfile.print(",");
    logfile.print( getRCtime(myTrike, i));
    }
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    logfile.print(",");
    mappd = getRCmapped(myTrike, i);
    logfile.print(mappd);
  }
   if (first_time)
  {
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    Serial.print(getRCtime(myTrike, i));
    Serial.print(", ");
   }
    Serial.println(" ");
   for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    Serial.print(getRCmapped(myTrike, i));
    Serial.print(", ");
   }
    Serial.println(" ");
    first_time = false;
  }
}
/******************************************************
Include wthe goals, either from RC or CAN
*******************************************************/ 
void Logger::HdrDesired() {
  logfile.print(",desired_speed_ms,desired_brake,desired_angle_DegX10");
}
void Logger::Desired() {
  logfile.print(",");
  logfile.print( getD_speed_mmPs(myTrike));
  logfile.print(",");
  logfile.print( getD_brakes(myTrike));
  logfile.print(",");
  logfile.print( getD_Angle(myTrike));
}
/******************************************************
Report sensors and actuators for steering
*******************************************************/ 
void Logger::HdrSteer() {
  logfile.print(",current_angle,Rturn,Lturn");
}
void Logger::Steer() {
  logfile.print(",");
  logfile.print(getAngle(myTrike));
  logfile.print(",");
  logfile.print(digitalRead(RIGHT_TURN_PIN));
  logfile.print(",");
  logfile.print(digitalRead(LEFT_TURN_PIN));
} 
/******************************************************
Report sensors and actuators for propulsion
*******************************************************/ 
void Logger::HdrThrottle() {
  logfile.print(",current_speed,driveMode");
}
void Logger::Throttle() {
  logfile.print(",");
  logfile.print(getSpeed(myTrike));
  // logfile.print(",");
  // logfile.print(throttlePulse_ms);
  logfile.print(",");
  logfile.print(getDriveMode(myTrike));
}
/******************************************************
Report state of the brakes
*******************************************************/ 
void Logger::HdrBrakes() {
  logfile.print(",BrakeOn,BrakeVolt");
}
void Logger::Brakes()  {
  logfile.print(",");
  logfile.print(digitalRead(BRAKE_ON_PIN));
  logfile.print(",");
  logfile.print(digitalRead(BRAKE_VOLT_PIN));
}
/******************************************************
Terminate the line and indicate the part of loop time used
*******************************************************/ 
void Logger::HdrEndLine() {
  logfile.println(",Utilization");
}
void Logger::EndLine(uint32_t delayTime) {
  int PerCentBusy = ((LOOP_TIME_MS - delayTime)*100)/LOOP_TIME_MS;
  logfile.print(",");
  logfile.println(PerCentBusy);
  logfile.flush();   // Flush the file to make sure data is written immediately
}

