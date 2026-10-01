#include "FirmwareMode.h"

#if DBW_SERIAL_TEST_MODE

#include <Arduino.h>
#include "../../DBW_Pins.h"
#include "../../SpeedController.h"

SpeedController::SpeedController(Startup)
  : PIDThrottle(0),
    speedPID(&speedCyclometer_cmPs, &PIDThrottle, &desiredSpeed_cmPs,
             proportional_throttle, integral_throttle, derivative_throttle, DIRECT) {
  digitalWrite(BRAKE_ON_PIN, ON_BR);
  digitalWrite(BRAKE_VOLT_PIN, ON_BR);
  pinMode(BRAKE_ON_PIN, OUTPUT);
  pinMode(BRAKE_VOLT_PIN, OUTPUT);
  state = BR_OFF;
  brake_change_ms = 0;
  calcTime_ms[0] = calcTime_ms[1] = 0;
  prevSpeed_cmPs = 0;
  testStop();
}

bool SpeedController::testThrottle(uint8_t value) {
  if (value > 0 && brakesApplied()) return false;
  currentThrottle = value;
  analogWrite(DAC0, currentThrottle);
  return true;
}

void SpeedController::testStop() {
  if (state == BR_OFF) Stop();
  else testThrottle(0);
  serviceBrakes();
}

void SpeedController::serviceBrakes() {
  uint32_t started = brake_change_ms - MAXHI_MS;
  if (state == BR_HI_VOLTS && uint32_t(millis() - started) >= MAXHI_MS) {
    digitalWrite(BRAKE_VOLT_PIN, OFF_BR);
    state = BR_LO_VOLTS;
  }
}

#endif