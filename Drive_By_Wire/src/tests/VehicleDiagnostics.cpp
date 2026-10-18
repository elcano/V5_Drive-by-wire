#include "VehicleDiagnostics.h"

#if DBW_SERIAL_TEST_MODE

#include "../../DBW_Pins.h"
#include "../../Settings.h"
#include "../../SpeedController.h"
#include "../../SteeringController.h"
#include "SerialTestMode.h"
#include <string.h>

namespace {
const uint32_t MAX_STEER_MS = 1000;
const uint32_t OUTPUT_TIMEOUT_MS = 5000;
const uint8_t testRcPins[6] = {STEERING_CH1_PIN, THROTTLE_BR_CH2_PIN, CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN};
volatile uint32_t rcRise[6] = {}, rcWidth[6] = {}, rcSeen[6] = {};
volatile bool rcHigh[6] = {};

template <uint8_t channel> void testRcEdge() {
  uint32_t now = micros();
  if (digitalRead(testRcPins[channel])) {
    rcRise[channel] = now;
    rcHigh[channel] = true;
  } else if (rcHigh[channel]) {
    rcWidth[channel] = now - rcRise[channel];
    rcSeen[channel] = now;
    rcHigh[channel] = false;
  }
}

bool validTestValue(char command, long value) {
  switch (command) {
    case 'V': return value >= 5 && value <= 255;
    case 'S': return value >= -(long)MAX_STEER_MS && value <= (long)MAX_STEER_MS;
    case 'T': return value >= 0 && value <= 255;
    case 'P': return value >= MIN_LEFT_US && value <= MAX_RIGHT_US;
    default: return false;
  }
}
}
using SerialTestMode::report;

VehicleDiagnostics::VehicleDiagnostics(SpeedController &speedController, SteeringController &steeringController)
  : throttle(speedController), steer(steeringController) {
}

void VehicleDiagnostics::begin() {
  pinMode(OP_ESTOP, INPUT_PULLUP);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  for (uint8_t channel = 0; channel < 6; ++channel) {
    pinMode(testRcPins[channel], INPUT);
    rcRise[channel] = rcWidth[channel] = rcSeen[channel] = 0;
    rcHigh[channel] = false;
  }
  attachInterrupt(digitalPinToInterrupt(testRcPins[0]), testRcEdge<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(testRcPins[1]), testRcEdge<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(testRcPins[2]), testRcEdge<2>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(testRcPins[3]), testRcEdge<3>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(testRcPins[4]), testRcEdge<4>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(testRcPins[5]), testRcEdge<5>, CHANGE);
  emergencyTestStop();
  report("DBW SERIAL TEST MODE: Programming USB, 115200, newline or CRLF");
  report("L colors | B brakes | R release | V 5..255 PWM | S signed ms | T 0..255 DAC");
  report("E immediate stop | C RC inputs | P 1000..1850 us servo; physical stop LOW inhibits commands");
  report("BENCH ONLY until pin polarity verified. DAC0=0 is NOT zero volts. No actuators attached.");
}

void VehicleDiagnostics::update() {
  serviceTestOutputs();
  SerialTestMode::readCommands(*this);
  serviceTestOutputs();
  SerialTestMode::flushReports();
}

void VehicleDiagnostics::setTestColor(uint8_t color) {
  digitalWrite(RED_LED_PIN, (color & 1) ? HIGH : LOW);
  digitalWrite(GREEN_LED_PIN, (color & 2) ? HIGH : LOW);
  digitalWrite(BLUE_LED_PIN, (color & 4) ? HIGH : LOW);
}

void VehicleDiagnostics::stopTestMotion() {
  steer.stopTest();
  throttle.testThrottle(0);
  diagnostic.steeringActive = false;
  diagnostic.throttleActive = false;
}

void VehicleDiagnostics::applyTestBrakes() {
  bool alreadyApplied = throttle.brakesApplied();
  throttle.testStop();
  diagnostic.throttleActive = false;
  if (!alreadyApplied) {
    report("BRAKE ON: D%u=%u D%u=%u; 800 ms boost", BRAKE_ON_PIN, ON_BR, BRAKE_VOLT_PIN, ON_BR);
  }
}

void VehicleDiagnostics::emergencyTestStop() {
  stopTestMotion();
  applyTestBrakes();
  diagnostic.estopped = true;
  diagnostic.mode = 0;
  diagnostic.ledActive = false;
  diagnostic.rcActive = false;
  setTestColor(1);
  report("ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked");
}

void VehicleDiagnostics::applyTestValue(char command, long value) {
  if (command == 'V') {
    diagnostic.steeringPwm = (uint8_t)value;
    report("STEER PWM setting=%ld (output remains 0 until S)", value);
  } else if (command == 'S') {
    stopTestMotion();
    if (value != 0) {
      steer.testMotor(value < 0, diagnostic.steeringPwm);
      diagnostic.steerDuration = (uint32_t)(value < 0 ? -value : value);
      diagnostic.steerStarted = millis();
      diagnostic.steeringActive = true;
    }
    report("STEER ms=%ld PWM=%u; initial angle=%d", value, diagnostic.steeringPwm, steer.readLeftSensorRaw());
  } else if (command == 'T') {
    stopTestMotion();
    if (!throttle.testThrottle((uint8_t)value)) {
      report("ERR brakes applied; use R then T before nonzero throttle");
      return;
    }
    diagnostic.throttleActive = value > 0;
    diagnostic.outputStarted = millis();
    report("THROTTLE DAC0=%ld; nonzero output expires in 5000 ms", value);
  } else if (command == 'P') {
    stopTestMotion();
    if (!steer.testPulse((uint16_t)value)) {
      report("ERR pulse outside configured steering limits");
      return;
    }
    diagnostic.outputStarted = millis();
    report("SERVO D%u=%ld us; angle=%d; detach in 5000 ms", STEER_PULSE_PIN, value, steer.readLeftSensorRaw());
  }
}

void VehicleDiagnostics::testCommand(char command, long value, bool hasValue) {
  if (command == 'E') {
    emergencyTestStop();
    return;
  }
  bool letter = command != 0;
  if (!letter) command = diagnostic.mode;
  if (!command || !strchr("LBRVSTCP", command)) {
    report("ERR expected L B R V S T E C P");
    return;
  }
  if (hasValue && !validTestValue(command, value)) {
    report("ERR range: V 5..255; S -1000..1000 ms; T 0..255; P 1000..1850 us");
    return;
  }
  if (diagnostic.physicalStop || (!letter && diagnostic.estopped)) {
    report("ERR ESTOP latched or physical stop grounded");
    return;
  }
  if (letter) {
    stopTestMotion();
    diagnostic.ledActive = false;
    diagnostic.rcActive = false;
    diagnostic.estopped = false;
    setTestColor(0);
    diagnostic.mode = strchr("VSTP", command) ? command : 0;
    report("COMMAND %c; previous motion cancelled; ESTOP latch cleared", command);
    if (command == 'B') applyTestBrakes();
    if (command == 'R') {
      throttle.ReleaseBrakes();
      report("BRAKE OFF: D%u=%u D%u=%u", BRAKE_ON_PIN, OFF_BR, BRAKE_VOLT_PIN, OFF_BR);
    }
    if (command == 'L') {
      diagnostic.ledColor = 0;
      diagnostic.ledStarted = millis();
      diagnostic.ledActive = true;
      setTestColor(0);
      report("LED RGB mask=0 (off)");
    }
    if (command == 'C') {
      diagnostic.rcActive = true;
      diagnostic.rcReported = millis() - 250;
    }
  }
  if (hasValue) applyTestValue(command, value);
}

void VehicleDiagnostics::serviceTestOutputs() {
  uint32_t now = millis();
  bool pressed = digitalRead(OP_ESTOP) == LOW;
  if (pressed && !diagnostic.physicalStop) emergencyTestStop();
  diagnostic.physicalStop = pressed;
  bool wasBoosting = throttle.brakeBoostActive();
  throttle.serviceBrakes();
  if (wasBoosting && !throttle.brakeBoostActive()) {
    report("BRAKE HOLD: D%u=%u D%u=%u", BRAKE_ON_PIN, ON_BR, BRAKE_VOLT_PIN, OFF_BR);
  }
  if (diagnostic.steeringActive && now - diagnostic.steerStarted >= diagnostic.steerDuration) {
    stopTestMotion();
    report("STEER STOP; angle=%d (10-bit raw); calibrated=%d degX10",
           steer.readLeftSensorRaw(), steer.testAngleLeft());
  }
  if ((diagnostic.throttleActive || steer.pulseTestActive()) && now - diagnostic.outputStarted >= OUTPUT_TIMEOUT_MS) {
    emergencyTestStop();
    report("OUTPUT TIMEOUT; angle=%d", steer.readLeftSensorRaw());
  }
  if (diagnostic.ledActive && now - diagnostic.ledStarted >= 1000) {
    diagnostic.ledStarted = now;
    if (++diagnostic.ledColor == 8) {
      diagnostic.ledActive = false;
      setTestColor(0);
      report("LED cycle complete");
    } else {
      setTestColor(diagnostic.ledColor);
      report("LED RGB mask=%u (R=1 G=2 B=4)", diagnostic.ledColor);
    }
  }
  if (diagnostic.rcActive && now - diagnostic.rcReported >= 250) {
    diagnostic.rcReported = now;
    uint32_t widths[6], seen[6];
    noInterrupts();
    for (uint8_t channel = 0; channel < 6; ++channel) {
      widths[channel] = rcWidth[channel];
      seen[channel] = rcSeen[channel];
    }
    interrupts();
    uint32_t nowUs = micros();
    for (uint8_t channel = 0; channel < 6; ++channel) {
      if (nowUs - seen[channel] > 100000) widths[channel] = 0;
    }
    report("RC us: %lu %lu %lu %lu %lu %lu (0=missing/stale)",
           (unsigned long)widths[0], (unsigned long)widths[1], (unsigned long)widths[2],
           (unsigned long)widths[3], (unsigned long)widths[4], (unsigned long)widths[5]);
  }
}

#endif