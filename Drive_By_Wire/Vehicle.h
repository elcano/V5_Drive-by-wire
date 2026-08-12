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
  // What each source WOULD command if it were the selected one. Filled every
  // loop by updateRC() whatever the mode, using the same "-1 means brake" rule
  // that update() applies. Logged only - the vehicle acts on desired_* above,
  // which still change only when that source is actually in control.
  int16_t rc_speed_cmPs = 0, rc_brake = 0, rc_angle_DegX10 = 0;
  int16_t op_speed_cmPs = 0, op_brake = 0, op_angle_DegX10 = 0;
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
  // Measured actual wheel angle, supplied by simulator over CAN (0x430).
  // Replaces the analog L_SENSE/R_SENSE read in the closed steering loop
  // when running on the Bridge (where those analog wires are broken).
  // 0 until the first 0x430 frame arrives.
  int16_t measured_wheel_angle_DegX10;
public:
  int16_t getMeasuredWheelAngle_DegX10() const { return measured_wheel_angle_DegX10; }
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
  // Where CSV rows actually go. File, UARTClass and Serial_ all derive from
  // Print, so one pointer covers every sink and can be re-pointed at runtime.
  // initialize() falls back from SD to Serial if the card fails, so a log is
  // still produced instead of logMethod going to 2 and writing nothing.
  Print *out;

private:
  // The CSV header is written on the first update() rather than in initialize().
  // On the native USB port initialize() runs before the host has opened the
  // port, and Serial_::write() discards bytes with no host attached — so a
  // header written at boot is silently lost and you get rows with no names.
  void writeHeader();
  bool headerSent;

  void TxTime();
  void TxLogRC();
  void TxDesired();
  void TxRequests();
  void TxThrottle();
  void TxBrakes();
  void TxSteer();
  void TxOp();       // operator joystick + switches
  void TxCAN();      // what Nav sent us over CAN

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
  void HdrRequests();
  void HdrThrottle();
  void HdrBrakes();
  void HdrSteer();
  void HdrOp();
  void HdrCAN();
  void HdrEndLine();
  File logfile;
  bool initRTC();
  char timeString[12];
  char dateString[12];
  char logName[16];   // actual log file name, e.g. 07031815.CSV
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
  // What the RC and the operator joystick are each asking for, whether or not
  // they are the selected source. See the note beside these in Vehicle.
  int16_t getRC_speed(Vehicle& v)  { return(v.rc_speed_cmPs); }
  int16_t getRC_brake(Vehicle& v)  { return(v.rc_brake); }
  int16_t getRC_angle(Vehicle& v)  { return(v.rc_angle_DegX10); }
  int16_t getOP_speed(Vehicle& v)  { return(v.op_speed_cmPs); }
  int16_t getOP_brake(Vehicle& v)  { return(v.op_brake); }
  int16_t getOP_angle(Vehicle& v)  { return(v.op_angle_DegX10); }
  // Measured wheel speed from the cyclometer, cm/s. getSpeed() below returns
  // currentSpeed_cmPs, which actually holds the throttle DAC value that
  // SpeedController::update() returns — not a speed.
  int16_t getMeasuredSpeed(Vehicle& v) {
    return(v.throttle->getMeasuredSpeed_cmPs());
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

  
