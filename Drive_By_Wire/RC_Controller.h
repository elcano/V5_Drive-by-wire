#pragma once
#include "DBW_Pins.h"


// Ignore any RC pulses outside these limits.
// Widened to bracket every channel actually produced by this transmitter
// (measured 869..2120 us across CH1-CH6). The old 970..2030 window sat inside
// the sticks' real travel: CH1 reaches 2035 at full right, so holding a full
// right turn made updateMode() decide the receiver had failed and walk the
// vehicle into ESTOP_RC, capping right steering partway through its travel.
// A genuinely dead channel reads 0 us and is still rejected.
#define MIN_RC_PULSE 850
#define MAX_RC_PULSE 2200

class RC_Controller {

private:
  static volatile unsigned long RC_Elapsed[RC_NUM_SIGNALS];
  static volatile unsigned long riseTime[RC_NUM_SIGNALS];
  static long ValuesMapped[RC_NUM_SIGNALS];
  static unsigned long elapsedTime[RC_NUM_SIGNALS];

  // What each control source is asking for, refreshed every loop regardless of
  // mode. Only the selected source is copied into ValuesMapped[], which is what
  // actually reaches the vehicle - so these let the log show what the RC and the
  // joystick each want even while they are not driving.
  long rcRequest[RC_NUM_SIGNALS] = {0};
  long opRequest[RC_NUM_SIGNALS] = {0};
  DriveMode rcDriveMode = FORWARD_MODE;
  DriveMode opDriveMode = FORWARD_MODE;

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
  // Unfiltered by mode - see rcRequest/opRequest above.
  long getRCRequest(int channel) { return rcRequest[channel]; }
  long getOPRequest(int channel) { return opRequest[channel]; }
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
