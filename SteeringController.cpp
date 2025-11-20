#include <Arduino.h>
#include "PID.h"
#include "DBW_Pins.h"
#include "Settings.h"
#include "SteeringController.h"

/*-----------------------------------------------------------------------------------*/
SteeringController::SteeringController()
  : steerPID(&steerAngle_DegX10, &SteerControl, &desiredTurn_DegX10,
             proportional_steering, integral_steering, derivative_steering, DIRECT) {
  // for steer servo
  pinMode(STEER_PULSE_PIN, OUTPUT);
  steerPID.SetSampleTime(PID_SAMPLE_TIME);
  steerPID.SetMode(AUTOMATIC);
  if (STEER_SERVO)
  {  // width range for servo pulse in microseconds
    steerPID.SetControlLimits(MIN_LEFT_US, MAX_RIGHT_US);
    Steer_Servo.attach(STEER_PULSE_PIN);
    delay(1);
  }
  else
  { // time to move motor in milliseconds
    steerPID.SetControlLimits(LEFT_ST_MS, RIGHT_ST_MS);
  }
  // for motor control board
  pinMode(STEER_SPEED_PIN,OUTPUT); // PWM A  
  pinMode(STEER_ON_PIN,OUTPUT);    // Brake A
  pinMode(STEER_DIR_PIN,OUTPUT);   // Dir A
  digitalWrite(STEER_ON_PIN, ST_OFF);
  analogWrite(STEER_SPEED_PIN, 100);   // slowr speed for initial centering
   
  update(0);    // point wheels straight ahead
  analogWrite(STEER_SPEED_PIN, 255);     // fast steer control
   if (DEBUG) {
    Serial.println("Steering Setup Complete");
  }
}
/*-----------------------------------------------------------------------------------*/
SteeringController::~SteeringController() {}
/*-----------------------------------------------------------------------------------*/
int SteeringController::update(int desiredangle_DegX10) 
{
  int steerAngle_DegX10;
  if (ANGLE_CONFIG & L_ANALOG)
  {
    steerAngle_DegX10 = computeAngleLeft();  // Use left sensor
    if (ANGLE_CONFIG & R_ANALOG)
    {
      steerAngle_DegX10 += computeAngleRight();  // Use both sensors
      // TODO: Compensate for L,R steer difference due to Ackerman steering
      steerAngle_DegX10 /= 2;                    // take average
    }
  }
  else if (ANGLE_CONFIG & R_ANALOG)
  {
    steerAngle_DegX10 = computeAngleRight();  // Use right sensor
  }

  SteeringPID(desiredangle_DegX10);
  delay(1);
  return steerAngle_DegX10;
}
/*-----------------------------------------------------------------------------------*/
void SteeringController::SteeringPID(int input_DegX10) {
  desiredTurn_DegX10 = input_DegX10;
  // produce new SteerControl if steerAngle_DegX10 and desiredTurn_DegX10 differ
  steerPID.Compute(); 
  if (STEER_SERVO)
  { 
    if (SteerControl != currentSteering_us) {
      Steer_Servo.writeMicroseconds(SteerControl);
      currentSteering_us = SteerControl;
    }
  }
  else
  {
    unsigned long motorTime_ms = SteerControl;
    if (SteerControl < 0)
    {
      digitalWrite(STEER_DIR_PIN,ST_LEFT);
      motorTime_ms = -motorTime_ms;
    }
    else
      digitalWrite(STEER_DIR_PIN,ST_RIGHT);
    digitalWrite(STEER_ON_PIN, ST_ON);
    // TODO: set a timer to turn off after motorTime_ms
  }
}
/*-----------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------*/
int SteeringController::computeAngleLeft() {
  int val = analogRead(L_SENSE_PIN);
  int degreeX10;
  if (val == Left_Straight_Read) 
     {  currentAngle_DegX10 = 0; return 0;}
  if (val > Left_Straight_Read)
  {  // right turn
     degreeX10 = (val - Left_Straight_Read) * (MIN_LEFT_DEGx10) / (Left_Straight_Read-Left_Read_at_MIN_TURN);
  }
  else
  {  // left turn
     degreeX10 = -(val - Left_Straight_Read) * (MAX_RIGHT_DEGx10) / (Left_Straight_Read-Left_Read_at_MAX_TURN);
  }
  if (DEBUG) {
    Serial.print("Left sensor: ");
    Serial.print(degreeX10);
    Serial.print(", ");
    Serial.println(val);
  }
  currentAngle_DegX10 = degreeX10;
  return degreeX10;
}
/*-----------------------------------------------------------------------------------*/
int SteeringController::computeAngleRight() {
  int val = analogRead(R_SENSE_PIN);
  int degreeX10;  
  if (val == Right_Straight_Read) 
     return 0;
  if (val > Right_Straight_Read)
  {  // right turn
     degreeX10 = (val - Right_Straight_Read) * (MIN_LEFT_DEGx10) / (Right_Straight_Read-Right_Read_at_MIN_TURN);
  }
  else
  {  // left turn
     degreeX10 = -(val - Right_Straight_Read) * (MAX_RIGHT_DEGx10) / (Right_Straight_Read-Right_Read_at_MAX_TURN);
  }
  if (DEBUG) {
    Serial.print("Right sensor: ");
    Serial.print(degreeX10);
    Serial.print(", ");
    Serial.println(val);
  }
  return degreeX10;  
}
/*-----------------------------------------------------------------------------------*/
int SteeringController::getSteeringMode() {
  return steeringMode;
}
