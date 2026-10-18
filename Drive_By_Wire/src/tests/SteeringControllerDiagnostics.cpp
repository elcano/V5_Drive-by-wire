#include "FirmwareMode.h"

#if DBW_SERIAL_TEST_MODE

#include <Arduino.h>
#include "../../DBW_Pins.h"
#include "../../Settings.h"
#include "../../SteeringController.h"

SteeringController::SteeringController(Startup)
  : steerAngle_DegX10(0), SteerControl(0), desiredTurn_DegX10(0),
    steerPID(&steerAngle_DegX10, &SteerControl, &desiredTurn_DegX10,
             proportional_steering, integral_steering, derivative_steering, DIRECT) {
  analogWrite(STEER_SPEED_PIN, 0);
  digitalWrite(STEER_ON_PIN, ST_OFF);
  digitalWrite(STEER_PULSE_PIN, LOW);
  digitalWrite(LEFT_TURN_PIN, LOW);
  pinMode(STEER_SPEED_PIN, OUTPUT);
  pinMode(STEER_ON_PIN, OUTPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);
  pinMode(STEER_PULSE_PIN, OUTPUT);
  pinMode(LEFT_TURN_PIN, OUTPUT);
  stopTest();
}

int SteeringController::testAngleLeft() {
  return computeAngleLeft();
}

int SteeringController::readLeftSensorRaw() const {
  return analogRead(L_SENSE_PIN);
}

void SteeringController::testMotor(bool turnLeft, uint8_t pwm) {
  stopTest();
  analogWrite(STEER_SPEED_PIN, pwm);
  digitalWrite(STEER_DIR_PIN, turnLeft ? ST_LEFT : ST_RIGHT);
  digitalWrite(STEER_ON_PIN, ST_ON);
}

bool SteeringController::testPulse(uint16_t width_us) {
  if (width_us < MIN_LEFT_US || width_us > MAX_RIGHT_US) return false;
  stopTest();
  Steer_Servo.attach(STEER_PULSE_PIN, MIN_LEFT_US, MAX_RIGHT_US);
  Steer_Servo.writeMicroseconds(width_us);
  currentSteering_us = width_us;
  return true;
}

void SteeringController::stopTest() {
  analogWrite(STEER_SPEED_PIN, 0);
  digitalWrite(STEER_ON_PIN, ST_OFF);
  if (Steer_Servo.attached()) Steer_Servo.detach();
  digitalWrite(STEER_PULSE_PIN, LOW);
  digitalWrite(LEFT_TURN_PIN, LOW);
  currentSteering_us = 0;
}

#endif