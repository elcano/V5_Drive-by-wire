#include <Arduino.h>
#include "PID.h"
#include "DBW_Pins.h"
#include "Settings.h"
#include "SteeringController.h"

/*-----------------------------------------------------------------------------------*/
SteeringController::SteeringController()
  : steerAngle_DegX10(0), SteerControl(0), desiredTurn_DegX10(0),
    steerPID(&steerAngle_DegX10, &SteerControl, &desiredTurn_DegX10,
             proportional_steering, integral_steering, derivative_steering, DIRECT) {
  // TWO-WIRE DIGITAL STEERING:
  //   DBW D26 LEFT_TURN_PIN  → Router D4 : HIGH = drive steer motor LEFT
  //   DBW D28 RIGHT_TURN_PIN → Router D2 : HIGH = drive steer motor RIGHT
  //   both LOW = HOLD. Router uses digitalRead (single PIO register access,
  //   robust against CAN-interrupt contention) and dispatches into
  //   updateAngle(lTurn,rTurn). DBW maintains its own open-loop model of
  //   Router's angle_tenths (see update()) since the L_SENSE/R_SENSE wires
  //   for analog feedback don't carry signal on this bridge.
  pinMode(LEFT_TURN_PIN,  OUTPUT);
  pinMode(RIGHT_TURN_PIN, OUTPUT);
  digitalWrite(LEFT_TURN_PIN,  LOW);
  digitalWrite(RIGHT_TURN_PIN, LOW);
  steerPID.SetSampleTime(PID_SAMPLE_TIME);
  steerPID.SetMode(AUTOMATIC);
  steerPID.SetControlLimits(MIN_LEFT_US, MAX_RIGHT_US);
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
int SteeringController::update(int desiredangle_DegX10, int measured_angle_DegX10)
{
  // Closed-loop steering. Preferred source for the actual wheel angle is
  // measured_angle_DegX10 passed in by Vehicle — fed by the simulator's
  // 0x430 CAN frame (or, on real hardware, by an L_SENSE/R_SENSE analog
  // read done in Vehicle::update before the call).
  //
  // If no measured value has ever arrived (haveMeasured stays false), fall
  // back to an open-loop integrator that integrates whatever we drove last
  // loop. This keeps DBW operational in degraded environments — sim with
  // no 0x430 publisher, or real trike whose sensors are absent.
  if (measured_angle_DegX10 != 0) haveMeasured = true;

  if (haveMeasured) {
    steerAngle_DegX10 = (float)measured_angle_DegX10;
  } else {
    // Open-loop fallback. Slew 200 degX10/sec matches Router's default
    // ANGLE_CHANGE_TENTHS = 20 per 100 ms.
    const int FALLBACK_SLEW_DegX10_PER_SEC = 200;
    const int MAX_ANGLE_DegX10             = 250;
    uint32_t now = millis();
    if (lastUpdate_ms != 0) {
      uint32_t dt_ms = now - lastUpdate_ms;
      int      slew  = (int)(((uint32_t)FALLBACK_SLEW_DegX10_PER_SEC * dt_ms) / 1000U);
      if (drovLeftLast)  steerAngle_DegX10 -= slew;
      if (drovRightLast) steerAngle_DegX10 += slew;
      if (steerAngle_DegX10 >  MAX_ANGLE_DegX10) steerAngle_DegX10 =  MAX_ANGLE_DegX10;
      if (steerAngle_DegX10 < -MAX_ANGLE_DegX10) steerAngle_DegX10 = -MAX_ANGLE_DegX10;
    }
    lastUpdate_ms = now;
  }
  currentAngle_DegX10 = (int)steerAngle_DegX10;

  SteeringPID(desiredangle_DegX10);
  delay(1);
  return (int)steerAngle_DegX10;
}
/*-----------------------------------------------------------------------------------*/
void SteeringController::SteeringPID(int input_DegX10) {
  desiredTurn_DegX10 = input_DegX10;
  // Bang-bang on two digital wires with deadband.
  //   err > +5  -> RIGHT_TURN HIGH, LEFT_TURN LOW
  //   err < -5  -> LEFT_TURN HIGH,  RIGHT_TURN LOW
  //   |err| <=5 -> both LOW (hold)
  // update() advances the modeled angle next loop based on which direction
  // we drove this loop.
  const int DEADBAND_DegX10 = 5;
  int err = input_DegX10 - (int)steerAngle_DegX10;
  bool turnLeft  = (err < -DEADBAND_DegX10);
  bool turnRight = (err >  DEADBAND_DegX10);
  digitalWrite(LEFT_TURN_PIN,  turnLeft  ? HIGH : LOW);
  digitalWrite(RIGHT_TURN_PIN, turnRight ? HIGH : LOW);
  drovLeftLast  = turnLeft;
  drovRightLast = turnRight;

  static uint32_t lastDbg_ms = 0;
  if (millis() - lastDbg_ms > 1000) {
    lastDbg_ms = millis();
    SerialUSB.print("# DBW steer cmd="); SerialUSB.print(input_DegX10);
    SerialUSB.print(" modeled=");        SerialUSB.print((int)steerAngle_DegX10);
    SerialUSB.print(" err=");            SerialUSB.print(err);
    SerialUSB.print(" L=");              SerialUSB.print(turnLeft  ? 1 : 0);
    SerialUSB.print(" R=");              SerialUSB.println(turnRight ? 1 : 0);
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
