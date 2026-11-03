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
switch (STEER_METHOD)
{
case STR_HBRIDGE:
  // TWO-WIRE DIGITAL STEERING:
  //   LEFT_TURN_PIN  : HIGH = drive steer motor LEFT
  //   RIGHT_TURN_PIN : HIGH = drive steer motor RIGHT
  //   (see DBW_Pins.h for which physical pins those are on this board - they
  //   are not D26/D28 here, as an earlier version of this comment claimed)
  //   both LOW = HOLD. Router uses digitalRead (single PIO register access,
  //   robust against CAN-interrupt contention) and dispatches into
  //   updateAngle(lTurn,rTurn). DBW maintains its own open-loop model of
  //   Router's angle_tenths (see update()) since the L_SENSE/R_SENSE wires
  //   for analog feedback don't carry signal on this bridge.
    pinMode(LEFT_TURN_PIN,  OUTPUT);
    pinMode(RIGHT_TURN_PIN, OUTPUT);
    digitalWrite(LEFT_TURN_PIN,  LOW);
    digitalWrite(RIGHT_TURN_PIN, LOW);
    break;

  case STR_PWM:
  // Servo actuator on STEER_PULSE_PIN. The Servo library repeats the pulse
  // every 20 ms on a hardware timer; we only set its width. The servo does its
  // own position control, so the PID controller is not used.
  // attach() with our limits makes writeMicroseconds() clamp to them.
    steerPID.SetControlLimits(MIN_LEFT_US, MAX_RIGHT_US);
    Steer_Servo.attach(STEER_PULSE_PIN, MIN_LEFT_US, MAX_RIGHT_US);
    Steer_Servo.writeMicroseconds(STRAIGHT_US);  // center the steering
    currentSteering_us = STRAIGHT_US;
    break;

  case STR_MOTOR_CONTROL:
    // for motor control board
    pinMode(STEER_SPEED_PIN,OUTPUT); // PWM A  
    pinMode(STEER_ON_PIN,OUTPUT);    // Brake A
    pinMode(STEER_DIR_PIN,OUTPUT);   // Dir A
    digitalWrite(STEER_ON_PIN, ST_OFF);
    analogWrite(STEER_SPEED_PIN, 100);   // slowr speed for initial centering
    // analogWrite(STEER_SPEED_PIN, 255);     // fast steer control
  }

  steerPID.SetSampleTime(PID_SAMPLE_TIME);
  steerPID.SetMode(AUTOMATIC);
   
  update(0);    // point wheels straight ahead
 
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

#if (STEER_METHOD == STR_PWM)
  // The servo goes where the pulse tells it, so without feedback the best
  // estimate of the wheel angle is the last commanded angle. The open-loop
  // step model below is for the two-wire H-bridge and does not apply here.
  SteeringPID(desiredangle_DegX10);
  steerAngle_DegX10 = haveMeasured ? (float)measured_angle_DegX10
                                   : (float)pulseToAngle(currentSteering_us);
  currentAngle_DegX10 = (int)steerAngle_DegX10;
  return currentAngle_DegX10;
#endif

  if (haveMeasured) {
    steerAngle_DegX10 = (float)measured_angle_DegX10;
  } else {
    // Open-loop fallback: mirror Router's fixed 20 degX10 / loop step
    // (ANGLE_CHANGE_TENTHS in simulator_stage1.ino). One step per DBW
    // loop while a direction wire is asserted.
    const int ANGLE_STEP_DegX10 = 20;
    const int MAX_ANGLE_DegX10  = 250;
    if (drovLeftLast)  steerAngle_DegX10 -= ANGLE_STEP_DegX10;
    if (drovRightLast) steerAngle_DegX10 += ANGLE_STEP_DegX10;
    if (steerAngle_DegX10 >  MAX_ANGLE_DegX10) steerAngle_DegX10 =  MAX_ANGLE_DegX10;
    if (steerAngle_DegX10 < -MAX_ANGLE_DegX10) steerAngle_DegX10 = -MAX_ANGLE_DegX10;
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
  //   err > +10  -> RIGHT_TURN HIGH, LEFT_TURN LOW
  //   err < -10  -> LEFT_TURN HIGH,  RIGHT_TURN LOW
  //   |err| <=10 -> both LOW (hold)
  // update() advances the modeled angle next loop based on which direction
  // we drove this loop.
  // Deadband is 10 (= half the 20-tenth/loop slew step). The wheel can only
  // rest on a 20-spaced grid, so the nearest holdable point is at most 10 away.
  // A smaller deadband (e.g. 5) leaves the wheel unable to land inside the band
  // and it hunts +/-10 forever; 10 lets it settle.
  const int DEADBAND_DegX10 = 10;
  int err = input_DegX10 - (int)steerAngle_DegX10;
  bool turnLeft  = (err < -DEADBAND_DegX10);
  bool turnRight = (err >  DEADBAND_DegX10);
#if (STEER_METHOD == STR_MOTOR_CONTROL)
  if (err >= -DEADBAND_DegX10 && err <=  DEADBAND_DegX10)
  {
    digitalWrite(STEER_ON_PIN, ST_OFF);  // no movement
  }
  else {
   // analogWrite, not digitalWrite: STEER_SPEED_PIN is the shield's PWM A pin,
   // and digitalWrite(pin, 0xFF) is just HIGH - full speed with no way to do
   // anything else, which made the "slow down for small err" note below
   // impossible to act on. 255 is the same full speed, but now scalable.
   analogWrite(STEER_SPEED_PIN, 255);  // max speed. May want to slow down for small err
   // To do: read motor current on A0 and redeuce speed if too much power.
   // NOTE: this writes the raw bool, so turning right drives the pin HIGH -
   // which contradicts ST_RIGHT (= !ST_LEFT = LOW) in DBW_Pins.h. One of the
   // two is wrong. Left as-is deliberately: picking a side without a meter on
   // the motor controller could send the steering the wrong way. Verify on
   // hardware before running STR_MOTOR_CONTROL, then use ST_LEFT/ST_RIGHT here.
   digitalWrite(STEER_DIR_PIN, turnRight);
   digitalWrite(STEER_ON_PIN, ST_ON);  // move
  }
#elif (STEER_METHOD == STR_HBRIDGE)
  digitalWrite(LEFT_TURN_PIN,  turnLeft  ? HIGH : LOW);
  digitalWrite(RIGHT_TURN_PIN, turnRight ? HIGH : LOW);
#else  // pulse width
  // The servo holds position on its own, so no deadband or direction wires:
  // just send the pulse width for the desired angle.
  turnLeft = turnRight = false;
  currentSteering_us = angleToPulse(input_DegX10);
  Steer_Servo.writeMicroseconds(currentSteering_us);
#endif
  drovLeftLast  = turnLeft;
  drovRightLast = turnRight;

  static uint32_t lastDbg_ms = 0;
  // Only print when the USB TX buffer has room — otherwise SerialUSB.print()
  // blocks on a full buffer when no host is reading and stalls the control
  // loop. availableForWrite() gating makes the print non-blocking: it is simply
  // skipped when there is no room.
  if (SerialUSB && SerialUSB.availableForWrite() >= 96 && millis() - lastDbg_ms > 1000) {
    lastDbg_ms = millis();
    SerialUSB.print("# DBW steer cmd="); SerialUSB.print(input_DegX10);
    SerialUSB.print(" modeled=");        SerialUSB.print((int)steerAngle_DegX10);
    SerialUSB.print(" err=");            SerialUSB.print(err);
    SerialUSB.print(" L=");              SerialUSB.print(turnLeft  ? 1 : 0);
    SerialUSB.print(" R=");              SerialUSB.print(turnRight ? 1 : 0);
    SerialUSB.print(" us=");             SerialUSB.println(currentSteering_us);
  }
}
/*-----------------------------------------------------------------------------------*/
// Pulse width for a wheel angle. Left and right are interpolated separately
// because the pulse limits and the angle limits are not symmetric about
// straight (e.g. 1000/1500/1850 us for -240/0/+250 degX10). All five numbers
// come from Settings.h and are found experimentally for each vehicle.
int SteeringController::angleToPulse(int angle_DegX10) {
  if (angle_DegX10 < MIN_LEFT_DEGx10)  angle_DegX10 = MIN_LEFT_DEGx10;
  if (angle_DegX10 > MAX_RIGHT_DEGx10) angle_DegX10 = MAX_RIGHT_DEGx10;
  if (angle_DegX10 < 0)
    return STRAIGHT_US + (long)angle_DegX10 * (STRAIGHT_US - MIN_LEFT_US) / (-MIN_LEFT_DEGx10);
  return STRAIGHT_US + (long)angle_DegX10 * (MAX_RIGHT_US - STRAIGHT_US) / MAX_RIGHT_DEGx10;
}
/*-----------------------------------------------------------------------------------*/
// Inverse of angleToPulse.
int SteeringController::pulseToAngle(int pulse_us) {
  if (pulse_us < MIN_LEFT_US)  pulse_us = MIN_LEFT_US;
  if (pulse_us > MAX_RIGHT_US) pulse_us = MAX_RIGHT_US;
  if (pulse_us < STRAIGHT_US)
    return (long)(pulse_us - STRAIGHT_US) * (-MIN_LEFT_DEGx10) / (STRAIGHT_US - MIN_LEFT_US);
  return (long)(pulse_us - STRAIGHT_US) * MAX_RIGHT_DEGx10 / (MAX_RIGHT_US - STRAIGHT_US);
}
/*-----------------------------------------------------------------------------------*/
// Get data from sensor on left steering column.
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
// Get data from sensor on right steering column.
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
/*-----------------------------------------------------------------------------------*/
// Move the wheels to center, then left, right, center.
// Use left sensor for feedback.
void SteeringController::test() {
  computeAngleLeft();  // left turn if currentAngle_DegX10 < 0
  switch (STEER_METHOD)
  {
  case STR_HBRIDGE:
  // TWO-WIRE DIGITAL STEERING:
    digitalWrite(LEFT_TURN_PIN,  LOW);
    digitalWrite(RIGHT_TURN_PIN, LOW);
   if (currentAngle_DegX10 < 0)
      while (computeAngleLeft() < 0)
        digitalWrite(RIGHT_TURN_PIN, HIGH);  // move to center
    else  
     while (computeAngleLeft() > 0)
        digitalWrite(LEFT_TURN_PIN, HIGH);  // move to center

    // continue to move to sides and back. No one is currently using this steering modde.
    break;

  case STR_PWM:
  // Center, full left, full right, center. Watch where the wheels land to
  // check the MIN_LEFT_US / MAX_RIGHT_US / STRAIGHT_US values in Settings.h.
    Steer_Servo.writeMicroseconds(STRAIGHT_US);
    delay(1000);
    Steer_Servo.writeMicroseconds(MIN_LEFT_US);
    delay(1000);
    Steer_Servo.writeMicroseconds(MAX_RIGHT_US);
    delay(1000);
    Steer_Servo.writeMicroseconds(STRAIGHT_US);
    currentSteering_us = STRAIGHT_US;
    break;

  case STR_MOTOR_CONTROL:
   // for motor control board
    analogWrite(STEER_SPEED_PIN, 100);   // slowr speed for initial centering
    digitalWrite (STEER_DIR_PIN, computeAngleLeft() < 0 ? ST_LEFT: ST_RIGHT);
    if (currentAngle_DegX10 < 0)
    {
      digitalWrite (STEER_ON_PIN, ST_ON);
      while (computeAngleLeft() < 0) ;
      digitalWrite (STEER_ON_PIN, ST_OFF);
    }
    digitalWrite (STEER_DIR_PIN, ST_RIGHT);
    if (currentAngle_DegX10 > 0)
    {
      digitalWrite (STEER_ON_PIN, ST_ON);
      while (computeAngleLeft() > 0) ;
      digitalWrite (STEER_ON_PIN, ST_OFF);
    }
    // centered. Now pause and move right.
    digitalWrite (STEER_ON_PIN, ST_OFF);
    delay(1000);
    analogWrite(STEER_SPEED_PIN, 170);     // faster steer control
    digitalWrite (STEER_ON_PIN, ST_ON);
    while (computeAngleLeft() <  MAX_RIGHT_DEGx10) ;
    digitalWrite (STEER_ON_PIN, ST_OFF);
    delay(1000);

    // move left
    digitalWrite (STEER_DIR_PIN, ST_LEFT);
    digitalWrite (STEER_ON_PIN, ST_ON);
    while (computeAngleLeft() > MIN_LEFT_DEGx10) ;
    digitalWrite (STEER_ON_PIN, ST_OFF);
    delay(1000);
    // back to center
    digitalWrite (STEER_DIR_PIN, ST_RIGHT);
    digitalWrite (STEER_ON_PIN, ST_ON);
    while (computeAngleLeft() < 0) ;
    digitalWrite (STEER_ON_PIN, ST_OFF);
    }
  }


