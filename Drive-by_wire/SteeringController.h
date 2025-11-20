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

  // Private methods
  void SteeringPID(int input);
   int computeAngleLeft();

public:
  SteeringController();
  ~SteeringController();

  // Main update method
  int update(int desiredangle_DegX10);

  // Optional getter for debugging
  int getSteeringMode();

  // Optional use of right sensor (if supported)
  int computeAngleRight();
};

