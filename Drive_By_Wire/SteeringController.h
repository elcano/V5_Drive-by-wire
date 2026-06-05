#pragma once

#include <Servo.h>
#include "PID.h"

// STEER_SERVO true -> use pulse on linear servo STEER_PULSE_PIN; 
// STEER_SERVO false -> use motor controller board, motor A
#define STEER_SERVO true


class SteeringController {
private:
  Servo Steer_Servo;
  PID steerPID;

  // PID control variables
  float steerAngle_DegX10;
  float SteerControl;
  float desiredTurn_DegX10;

  // State tracking
  int currentSteering_us = 0;
  int currentAngle_DegX10 = 0;
  int steeringMode = 0;

  // Steering-feedback state. Preferred source is the measured angle passed
  // into update() (from simulator 0x430 CAN, or on real hardware from analog
  // L_SENSE/R_SENSE reads in a future revision). If no feedback has ever
  // arrived (haveMeasured == false), update() falls back to an internal
  // open-loop model that integrates the assertions from SteeringPID.
  uint32_t lastUpdate_ms = 0;
  bool haveMeasured  = false;
  bool drovLeftLast  = false;
  bool drovRightLast = false;

  // Private methods
  void SteeringPID(int input);
   int computeAngleLeft();

public:
  SteeringController();
  ~SteeringController();

  // Main update method.
  // measured_angle_DegX10 is the actual wheel angle reported back from the
  // simulator (or, on real hardware, the analog sensor read). Pass 0 if no
  // feedback is available; the open-loop model fallback will be used.
  int update(int desiredangle_DegX10, int measured_angle_DegX10 = 0);

  // Optional getter for debugging
  int getSteeringMode();

  // Optional use of right sensor (if supported)
  int computeAngleRight();
};

