#pragma once

#include "SpeedController.h"
#include "SteeringController.h"
#include "RC_Controller.h"
#include <TimeLib.h> 
#include <SD.h>
#include <stdio.h>
#include <due_can.h>    // from due-can library
//Timing stuff
#define LOOP_TIME_MS 100

extern tmElements_t tm;

class Vehicle {
public:
  friend class Logger;
  Vehicle();
  ~Vehicle();
  void update();
  void updateRC();

  void sendCan();
  void receiveCan();

private:
  static RC_Controller* RC;
  static SpeedController* throttle;
  static SteeringController* steer;
  // CAN_FRAME is defined in can_common.h
  CAN_FRAME incoming;
  CAN_FRAME outgoing;
  unsigned long RCtime[RC_NUM_SIGNALS];
  unsigned long RCtime1;
  long RCMapped[RC_NUM_SIGNALS];
  static int16_t desired_speed_mmPs;
  static int16_t desired_brake;
  static int16_t desired_angle_DegX10; 
  int16_t currentSpeed;
  int16_t currentBrake;
  int16_t currentAngle_DegX10;
  DriveMode currentDriveMode;
  AutoMode currentAutoMode;
  AutoMode  int2Auto(int);
  int brakeHold; // Hold brakes with 12V 
  long throttle_mmPs;
  long steer_DegX10;
} ;

class Logger {
public:
 void initialize(); 
  Logger();
  ~Logger();
  void update();
  void EndLine(uint32_t delayTime);

private:
  void Time();
  void LogRC();
  void Desired();
  void Throttle();
  void Brakes();
  void Steer();  
  void HdrTime();
  void HdrRC();
  void HdrDesired();
  void HdrThrottle();
  void HdrBrakes();
  void HdrSteer();
  void HdrEndLine();
  File logfile;
  bool initRTC();
  char timeString[12];
  char dateString[12];
  bool initSD();
  bool openSD();
 
  int16_t getD_speed_mmPs(Vehicle& v) {
    return(v.desired_speed_mmPs);
  }
  int16_t getD_brakes(Vehicle& v) {
    return(v.desired_brake);
  }
  int16_t getD_Angle(Vehicle& v) {
    return(v.desired_angle_DegX10);
  }
  int16_t getSpeed(Vehicle& v) {
    return(v.currentSpeed);
  }
  int16_t getAngle(Vehicle& v) {
    return(v.currentAngle_DegX10);
  }  
  int16_t getDriveMode(Vehicle& v) {
    return(v.currentDriveMode);
  }
  int16_t getAutoMode(Vehicle& v) {
    return(v.currentAutoMode);
  }
   long getRCmapped(Vehicle& v, int channel) {
    return(v.RCMapped[channel]);
  }
   unsigned long getRCtime(Vehicle& v, int channel) {
    return(v.RCtime[channel]);
  }
  unsigned long getRCtime1(Vehicle& v) {
    return(v.RCtime1);
  }
} ;

  