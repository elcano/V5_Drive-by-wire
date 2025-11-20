#pragma once
#include "DBW_Pins.h"


// Ignore any RC pulses outside these limits
#define MIN_RC_PULSE 970
#define MAX_RC_PULSE 2030

class RC_Controller {

private:
  static volatile unsigned long RC_Elapsed[RC_NUM_SIGNALS];
  static volatile unsigned long riseTime[RC_NUM_SIGNALS];
  static long ValuesMapped[RC_NUM_SIGNALS];  
  static unsigned long elapsedTime[RC_NUM_SIGNALS]; 

  DriveMode driveMode = FORWARD_MODE;
  AutoMode  autoMode  = MANUAL_MODE;
  bool rc_data = false;
  bool op_estop = false;
  bool op_enabled = false;

public:
  RC_Controller();
  ~RC_Controller();
  AutoMode updateMode(AutoMode);
  void mapValues();
  void opUpdate();
  DriveMode getDriveMode() const;
  long getMappedValue(int channel);
  unsigned long getEtime(int channel);
  void autoModeLED(AutoMode);

  static void ISR_STEERING_RISE();
  static void ISR_STEERING_FALL();
  static void ISR_THROTTLE_RISE();
  static void ISR_THROTTLE_FALL();
  static void ISR_CH3_Rise();
  static void ISR_CH3_Fall();
  static void ISR_CH4_Rise();
  static void ISR_CH4_Fall();
  static void ISR_CH5_Rise();
  static void ISR_CH5_Fall();
  static void ISR_CH6_Rise();
  static void ISR_CH6_Fall();
 
};
