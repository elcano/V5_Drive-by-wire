#include <Arduino.h>
#include "RC_Controller.h"
#include "DBW_Pins.h"
#include "Settings.h"

volatile unsigned long RC_Controller::riseTime[RC_NUM_SIGNALS];
volatile unsigned long RC_Controller::RC_Elapsed[RC_NUM_SIGNALS];
unsigned long RC_Controller::elapsedTime[RC_NUM_SIGNALS]; 
long RC_Controller::ValuesMapped[RC_NUM_SIGNALS]; 

RC_Controller::RC_Controller() {
  pinMode(STEERING_CH1_PIN, INPUT);
  pinMode(THROTTLE_BR_CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);
   for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    RC_Elapsed[i] = 0;
    riseTime[i] = 0;
    elapsedTime[i] = 0;
    ValuesMapped[i] = 0;
   }
   
  attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING); 
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), RC_Controller::ISR_CH3_Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(CH4_PIN), RC_Controller::ISR_CH4_Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(CH5_PIN), RC_Controller::ISR_CH5_Rise, RISING);
  attachInterrupt(digitalPinToInterrupt(CH6_PIN), RC_Controller::ISR_CH6_Rise, RISING);
}

RC_Controller::~RC_Controller() {}
//_______________________________________________________________________________
AutoMode RC_Controller::updateMode(AutoMode oldAutoMode) {
  AutoMode  RC_switchMode;
  AutoMode  newAutoMode = oldAutoMode;
  noInterrupts();
  for (int i = 0; i < RC_NUM_SIGNALS; i++) {
    elapsedTime[i] = RC_Elapsed[i];
  }
  interrupts();
  /*
  Vehicle comes up in INITIALIZING. To leave that state, 
  it must receive valid data from RC or Operator showing that EStop is available.
  On valid data, transition to MANUAL_MODE, where all RC channels are active.
  Channel 4 can switch to AUTO_RC where control comes from CAN bus.
  In AUTO_RC, the only active RC controls are CH4 and brakes.
  Applying the brake in AUTO_RC is an emergency stop; The vehicle stops and will not respond
  until the e-stop is removed.
  The e-stop is removed by switching CH4 out of AUTO_RC.
  */
  // Is there valid RC data?
  if (elapsedTime[CH1] > MIN_RC_PULSE && elapsedTime[CH1] < MAX_RC_PULSE &&
      elapsedTime[CH2] > MIN_RC_PULSE && elapsedTime[CH2] < MAX_RC_PULSE)
  {
    rc_data = true;
    RC_switchMode = MANUAL_MODE; 
    if (elapsedTime[CH4] < CH4_MIDLO )
      RC_switchMode = OPERATOR_MODE;  
    if (elapsedTime[CH4] > CH4_MIDHI)
      RC_switchMode = AUTO_RC;
    }
  else
  {
    rc_data = false;
    RC_switchMode = INITIALIZING;  // unknown
  }
  op_estop =   digitalRead(OP_ESTOP)? false: true;
  op_enabled = digitalRead(OP_MODE_PIN) ? false: true;

// Assume operator has not pressed e-stop button
  switch (oldAutoMode) {
    case INITIALIZING:  
      if (rc_data && RC_switchMode==MANUAL_MODE)  newAutoMode = MANUAL_MODE;
      else if (op_enabled)                        newAutoMode = OPERATOR_MODE;
      break;
    case MANUAL_MODE:
      if (!rc_data)                               newAutoMode = INITIALIZING;
      else if (rc_data && RC_switchMode==AUTO_RC) newAutoMode = AUTO_RC;
      else if (RC_switchMode==OPERATOR_MODE)      newAutoMode = OPERATOR_MODE;
      break;
    case OPERATOR_MODE:
      if (!op_enabled)                            newAutoMode = AUTO_OP;
      if (RC_switchMode==MANUAL_MODE)             newAutoMode = MANUAL_MODE;
      // if (operator time out)                   NewAutoMode = INITIALIZING;
      break;
    case AUTO_RC:
      if (RC_switchMode==MANUAL_MODE)             newAutoMode = MANUAL_MODE;
      // if (CAN_MAN || NO_CAN)                   newAutoMode = MANUAL_MODE;
      if (!rc_data ||                              
         (rc_data && elapsedTime[CH2]<CH2_MIDLO) ||
         analogRead(OP_THROTTLE)<OP_MIDLO)     newAutoMode = ESTOP_RC;
      break;
    case AUTO_OP:
      if (op_enabled)                             newAutoMode = OPERATOR_MODE;
      // if (CAN_MAN || NO_CAN)                   newAutoMode = OPERATOR_MODE; 
      if (rc_data && elapsedTime[CH2]<CH2_MIDLO
      || analogRead(OP_THROTTLE)<OP_MIDLO)      newAutoMode = ESTOP_OP;
      break;
    case ESTOP_RC:
      if (RC_switchMode==MANUAL_MODE)               newAutoMode = INITIALIZING;
      break;
    case ESTOP_OP:
      if (RC_switchMode==MANUAL_MODE || op_enabled) newAutoMode = INITIALIZING;
      break;
    case ESTOP_BTN:
      if (!op_estop)                                 newAutoMode = INITIALIZING;
      break;  
  }
  // if they did press estop, forget about the stuff above.
  if (op_estop)                                     newAutoMode = ESTOP_BTN;

switch (newAutoMode) {
  case MANUAL_MODE:
    ValuesMapped[CH4] = newAutoMode; 
    mapValues();
    break;
  case OPERATOR_MODE:
    ValuesMapped[CH4] = newAutoMode; 
    opUpdate();
    break;
// AUTO modes will receiveCAN; INITIALIZING does nothing.
// if (ESTOP) Stop() is called on return
  default:
    break;
  }
  autoModeLED(newAutoMode);  // display the mode
  return (newAutoMode);
}
//_______________________________________________________________________________
void RC_Controller::mapValues() {
  static bool first_time = true;
  long leftTurn;  // goes negative

  if (first_time)
  {
   for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      Serial.print(elapsedTime[i]);
      Serial.print(", ");
    }
    Serial.println(" ");
  }
  leftTurn = elapsedTime[CH1];  // convert from unsigned to signed.
  ValuesMapped[CH1] = 0;  // steer straight
  if (elapsedTime[CH1] <  CH1_MIDLO)  // turn left
      ValuesMapped[CH1] = MAP(leftTurn, CH1_MIN, CH1_MIDLO, MIN_LEFT_DEGx10, 0);
  if (elapsedTime[CH1] >  CH1_MIDHI)  // turn right
      ValuesMapped[CH1] = MAP(leftTurn, CH1_MIDHI,CH1_MAX, 0, MAX_RIGHT_DEGx10);

  ValuesMapped[CH2] = 0;  // coast
  if (elapsedTime[CH2] <  CH2_MIDLO) 
      ValuesMapped[CH2] = -1; // brake
  if (elapsedTime[CH2] >  CH2_MIDHI)
      ValuesMapped[CH2] = MAP(elapsedTime[CH2],CH2_MIDHI,CH2_MAX, 0,MAX_SPEED_cmPs);
  
  ValuesMapped[CH3] = HIGH;
  if (elapsedTime[CH3] <  CH3_MIDLO)
    ValuesMapped[CH3] = LOW;
  driveMode = (ValuesMapped[CH3])? REVERSE_MODE: FORWARD_MODE;
  ValuesMapped[CH5] = MAP(elapsedTime[CH5],CH5_MIN,CH5_MAX,0,100);
  ValuesMapped[CH6] = MAP(elapsedTime[CH6],CH6_MIN,CH6_MAX,0,100);

  if (first_time)
  {
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      Serial.print(ValuesMapped[i]);
      Serial.print(", ");
    }
    Serial.println(" ");
    first_time = false;
  }
}
//_______________________________________________________________________________
void RC_Controller::opUpdate() {
//since we are in OPERATOR MODE, fill in mapped values
  long turn = analogRead(OP_STEER); 
  long throttle = analogRead(OP_THROTTLE);

  ValuesMapped[CH1] = 0;  // steer straight
  if (turn <  OP_MIDLO)  // turn left
      ValuesMapped[CH1] = MAP(turn, OP_MIN, OP_MIDLO, MIN_LEFT_DEGx10, 0);
  if (turn >  OP_MIDHI)  // turn right
      ValuesMapped[CH1] = MAP(turn, OP_MIDHI,OP_MAX, 0, MAX_RIGHT_DEGx10);

  ValuesMapped[CH2] = 0;  // coast
  if (throttle <  OP_MIDLO) 
      ValuesMapped[CH2] = -1; // brake
  if (throttle >  OP_MIDHI)
      ValuesMapped[CH2] = MAP(throttle,OP_MIDHI,OP_MAX, 0,MAX_SPEED_cmPs);
  
  ValuesMapped[CH3] = digitalRead(OP_FWD_PIN)? HIGH: LOW;
  ValuesMapped[CH5] = digitalRead(OP_DISCNT_PIN)? HIGH: LOW;
}
//_______________________________________________________________________________
long RC_Controller::getMappedValue(int channel) {
  // for some unknown reason, this returns chaneels 4,5,6,1,2,3
  return (ValuesMapped[channel]);
  // if (channel > 2)
  //   return (ValuesMapped[channel-3]);
  // else
  //   return (ValuesMapped[channel+3]);
}
unsigned long RC_Controller::getEtime(int channel) {
  return (elapsedTime[channel]);
  //  if (channel > 2)
  //   return (elapsedTime[channel-3]);
  // else
  //   return (elapsedTime[channel+3]);
}
DriveMode RC_Controller::getDriveMode() const {
    return driveMode;
}
//_______________________________________________________________________________
void RC_Controller::autoModeLED(AutoMode mode){
  switch(mode) {
    case INITIALIZING:  // white
      digitalWrite(RED_LED_PIN,HIGH);
      digitalWrite(GREEN_LED_PIN,HIGH);
      digitalWrite(BLUE_LED_PIN,HIGH);
      break;
    case MANUAL_MODE:   // Green
      digitalWrite(RED_LED_PIN,LOW);
      digitalWrite(GREEN_LED_PIN,HIGH);
      digitalWrite(BLUE_LED_PIN,LOW);
      break;
    case OPERATOR_MODE:  // Blue
      digitalWrite(RED_LED_PIN,LOW);
      digitalWrite(GREEN_LED_PIN,LOW);
      digitalWrite(BLUE_LED_PIN,HIGH);
      break;
    case AUTO_RC:      // Yellow
      digitalWrite(RED_LED_PIN,HIGH);
      digitalWrite(GREEN_LED_PIN,HIGH);
      digitalWrite(BLUE_LED_PIN,LOW);
    case AUTO_OP:      // Violet
      digitalWrite(RED_LED_PIN,HIGH);
      digitalWrite(GREEN_LED_PIN,LOW);
      digitalWrite(BLUE_LED_PIN,HIGH);
      break;
    case ESTOP_OP:
    case ESTOP_BTN:
    case ESTOP_RC:  // Red
      digitalWrite(RED_LED_PIN,HIGH);
      digitalWrite(GREEN_LED_PIN,LOW);
      digitalWrite(BLUE_LED_PIN,LOW);
      break;
  }
}
void RC_Controller::ISR_STEERING_RISE() {
  noInterrupts();
  if (digitalRead(STEERING_CH1_PIN) == HIGH) {
    riseTime[CH1] = micros();
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_FALL, FALLING);
  }
   interrupts();
}

void RC_Controller::ISR_STEERING_FALL() {
  noInterrupts();
  if (digitalRead(STEERING_CH1_PIN) == LOW) {
    RC_Elapsed[CH1] = micros() - riseTime[CH1];
    attachInterrupt(digitalPinToInterrupt(STEERING_CH1_PIN), ISR_STEERING_RISE, RISING);
  }
  interrupts();
}

void RC_Controller::ISR_THROTTLE_RISE() {
  noInterrupts();
  if (digitalRead(THROTTLE_BR_CH2_PIN) == HIGH) {
    riseTime[CH2] = micros();
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_FALL, FALLING);
  }
  interrupts();
}

void RC_Controller::ISR_THROTTLE_FALL() {
  noInterrupts();
  if (digitalRead(THROTTLE_BR_CH2_PIN) == LOW) {
    RC_Elapsed[CH2] = micros() - riseTime[CH2];
    attachInterrupt(digitalPinToInterrupt(THROTTLE_BR_CH2_PIN), ISR_THROTTLE_RISE, RISING);
  }
  interrupts();
}

void RC_Controller::ISR_CH3_Rise() {
  noInterrupts();
  if (digitalRead(CH3_PIN) == HIGH) {
    riseTime[CH3] = micros();
    attachInterrupt(digitalPinToInterrupt(CH3_PIN), ISR_CH3_Fall, FALLING);
  }
  interrupts();
}
void RC_Controller::ISR_CH3_Fall() {
  noInterrupts();
  if (digitalRead(CH3_PIN) == LOW) {
    RC_Elapsed[CH3] = micros() - riseTime[CH3];
    attachInterrupt(digitalPinToInterrupt(CH3_PIN), ISR_CH3_Rise, RISING);
  }
  interrupts();
}
void RC_Controller::ISR_CH4_Rise() {
  noInterrupts();
    if (digitalRead(CH4_PIN) == HIGH) {
      riseTime[CH4] = micros();
      attachInterrupt(digitalPinToInterrupt(CH4_PIN), ISR_CH4_Fall, FALLING);
    }
  interrupts();
}
void RC_Controller::ISR_CH4_Fall() {
    noInterrupts();
    if (digitalRead(CH4_PIN) == LOW) {
      RC_Elapsed[CH4] = micros() - riseTime[CH4];
      attachInterrupt(digitalPinToInterrupt(CH4_PIN), ISR_CH4_Rise, RISING);
    }
    interrupts();
}
void RC_Controller::ISR_CH5_Rise() {
  noInterrupts();
  if (digitalRead(CH5_PIN) == HIGH) {
    riseTime[CH5] = micros();
    attachInterrupt(digitalPinToInterrupt(CH5_PIN), ISR_CH5_Fall, FALLING);
  }
  interrupts();
}
void RC_Controller::ISR_CH5_Fall() {
  noInterrupts();
  if (digitalRead(CH5_PIN) == LOW) {
    RC_Elapsed[CH5] = micros() - riseTime[CH5];
    attachInterrupt(digitalPinToInterrupt(CH5_PIN), ISR_CH5_Rise, RISING);
  }
  interrupts();
}
void RC_Controller::ISR_CH6_Rise() {
  noInterrupts();
  if (digitalRead(CH6_PIN) == HIGH) {
    riseTime[CH6] = micros();
    attachInterrupt(digitalPinToInterrupt(CH6_PIN), ISR_CH6_Fall, FALLING);
  }
  interrupts();
}
void RC_Controller::ISR_CH6_Fall() {
  noInterrupts();
  if (digitalRead(CH6_PIN) == LOW) {
    RC_Elapsed[CH6] = micros() - riseTime[CH6];
    attachInterrupt(digitalPinToInterrupt(CH6_PIN), ISR_CH6_Rise, RISING);
  }
  interrupts();
}
