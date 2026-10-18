#include "FirmwareMode.h"

#if DBW_SERIAL_TEST_MODE

#include <Arduino.h>
#include "../../DBW_Pins.h"
#include "../../Vehicle.h"
#include "SerialTestMode.h"
#include "VehicleDiagnostics.h"

Vehicle::Vehicle(Startup) : diagnosticMode(true) {
  currentSpeed_cmPs = 0;
  currentAngle_DegX10 = 0;
  currentBrake = 0;
  desired_speed_cmPs = 0;
  desired_brake = 0;
  desired_angle_DegX10 = 0;
  currentDriveMode = FORWARD_MODE;
  currentAutoMode = INITIALIZING;
  FirstTime = false;
  canActive = false;
  last_nav_speed_cmPs = 0;
  last_nav_brake = 0;
  last_nav_mode = 0;
  last_nav_angle_DegX10 = 0;
  last_nav_status = 0;
  measured_wheel_angle_DegX10 = 0;
  SerialTestMode::begin();
  analogWriteResolution(8);
  analogReadResolution(10);
  steer = new SteeringController(SteeringController::Startup::Diagnostic);
  throttle = new SpeedController(SpeedController::Startup::Diagnostic);
  diagnostic = new VehicleDiagnostics(*throttle, *steer);
  diagnostic->begin();
}

void Vehicle::test() {
  if (diagnosticMode) diagnostic->update();
}

void Vehicle::testCommand(char command, long value, bool hasValue) {
  if (diagnosticMode) diagnostic->testCommand(command, value, hasValue);
}

#endif