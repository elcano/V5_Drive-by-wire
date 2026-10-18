#pragma once

#include "FirmwareMode.h"

#if DBW_SERIAL_TEST_MODE

#include <Arduino.h>

class SpeedController;
class SteeringController;

class VehicleDiagnostics {
public:
  VehicleDiagnostics(SpeedController &speedController, SteeringController &steeringController);
  void begin();
  void update();
  void testCommand(char command, long value, bool hasValue);

private:
  SpeedController &throttle;
  SteeringController &steer;
  struct DiagnosticState {
    char mode = 0;
    bool estopped = true;
    bool physicalStop = false;
    bool steeringActive = false;
    bool throttleActive = false;
    bool ledActive = false;
    bool rcActive = false;
    uint8_t steeringPwm = 50;
    uint8_t ledColor = 0;
    uint32_t steerStarted = 0, steerDuration = 0;
    uint32_t outputStarted = 0, ledStarted = 0, rcReported = 0;
  } diagnostic;
  void setTestColor(uint8_t color);
  void stopTestMotion();
  void applyTestBrakes();
  void emergencyTestStop();
  void applyTestValue(char command, long value);
  void serviceTestOutputs();
};

#endif