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

  bool sendCan();
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
  static int16_t desired_speed_cmPs;
  static int16_t desired_brake;
  static int16_t desired_angle_DegX10; 
  int16_t currentSpeed_cmPs;
  int16_t currentBrake;
  int16_t currentAngle_DegX10;
  DriveMode currentDriveMode;
  AutoMode currentAutoMode;
  AutoMode  int2Auto(int);
  bool canActive;  // true when CAN commands are being received
  // Buffered last values received from Nav (for 0x704 Log_auto emit).
  // The 0x350 mode byte is informational; DBW arbitrates its own state.
  int16_t last_nav_speed_cmPs;
  uint8_t last_nav_brake;
  uint8_t last_nav_mode;
  int16_t last_nav_angle_DegX10;
  uint8_t last_nav_status;
  int brakeHold; // Hold brakes with 12V
  long throttle_cmPs;
  long steer_DegX10;
} ;

class Logger {
public:
 void initialize(); 
  Logger();
  ~Logger();
  void update();
  void EndLine(uint32_t delayTime);
  // Sink state. CAN log emit is always on; SD and serial are optional additions.
  // logMethod retained for back-compat with Tx* path: 0 = SD, 1 = serial, 2 = neither.
  int logMethod;

private:
  void TxTime();
  void TxLogRC();
  void TxDesired();
  void TxThrottle();
  void TxBrakes();
  void TxSteer();

  // CAN log emit — one frame per method, fixed wiki IDs (0x701-0x70A).
  void CANLogTime();      // 0x701
  void CANLogRC();        // 0x702
  void CANLogOp();        // 0x703
  void CANLogAuto();      // 0x704
  void CANLogDesired();   // 0x705
  void CANLogThrottle();  // 0x706
  void CANLogBrakes();    // 0x707
  void CANLogSteer();     // 0x708
  // 0x709 LogPosition is Nav-sourced; not emitted by DBW.
  void CANLogFinalize(int PerCentBusy);  // 0x70A — called from EndLine

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
 
  int16_t getD_speed_cmPs(Vehicle& v) {
    return(v.desired_speed_cmPs);
  }
  int16_t getD_brakes(Vehicle& v) {
    return(v.desired_brake);
  }
  int16_t getD_Angle(Vehicle& v) {
    return(v.desired_angle_DegX10);
  }
  int16_t getSpeed(Vehicle& v) {
    return(v.currentSpeed_cmPs);
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
  CAN_FRAME* getCAN(Vehicle& v) {
    return (&v.outgoing);
  }
  // Accessors for 0x704 Log_auto: last values received from Nav over CAN.
  int16_t getNavSpeed(Vehicle& v)  { return v.last_nav_speed_cmPs; }
  uint8_t getNavBrake(Vehicle& v)  { return v.last_nav_brake; }
  uint8_t getNavMode(Vehicle& v)   { return v.last_nav_mode; }
  int16_t getNavAngle(Vehicle& v)  { return v.last_nav_angle_DegX10; }
  uint8_t getNavStatus(Vehicle& v) { return v.last_nav_status; }
  bool    getCanActive(Vehicle& v) { return v.canActive; }
} ;

  
