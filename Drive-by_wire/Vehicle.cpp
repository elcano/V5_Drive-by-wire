#include <Arduino.h>
#include "DBW_Pins.h"
#include "Vehicle.h"
#include "Can_Protocol.h"
#include "Settings.h"

RC_Controller* Vehicle::RC;
SpeedController* Vehicle::throttle;
SteeringController* Vehicle::steer;

int16_t Vehicle::desired_speed_cmPs;
int16_t Vehicle::desired_brake;
int16_t Vehicle::desired_angle_DegX10; 

/****************************************************************************
   Constructor
 ****************************************************************************/
Vehicle::Vehicle() 
{
  RC = new RC_Controller();
  throttle = new SpeedController();
  steer = new SteeringController();
 
  // Intialize default values
  currentSpeed_cmPs = 0;
  currentAngle_DegX10 = 0;
  currentBrake = 0;
  desired_speed_cmPs = 0;
  desired_brake = 0;
  desired_angle_DegX10 = 0;
  currentDriveMode = FORWARD_MODE;
  currentAutoMode = INITIALIZING;

   if (!Can0.begin(CAN_BPS_500K))  // initialize CAN with 500kbps baud rate
  {
    Serial.println("Can0 init success");
  } else {
     Serial.println("Can0 init failed");
  }
 
  Serial.println("Vehicle constructor finished.");
}

/*****************************************************************************
   Destructor
 ****************************************************************************/
Vehicle::~Vehicle() {
}


/*******************************************************************************************************
   Called after updated desired settings are received from RC and CAN.
   Change Vehicle speed and steering angle settings
 *******************************************************************************************************/
void Vehicle::update() {
  // ____First, get desired values___________________________________________________
  int16_t tempDspeed;
  int16_t newAutoMode;
   int16_t newDriveMode;
  newAutoMode = RC->getMappedValue(CH4);
  if (currentAutoMode == ESTOP_RC)
  {  //E-stop happens when RC brake is applied in AUTO_RC
    if (newAutoMode == OPERATOR_MODE)
      currentAutoMode = OPERATOR_MODE;   // remove E-stop
    // if current mode is e-stopped, stay there until switch to operator mode
  }
  else
    currentAutoMode = int2Auto(newAutoMode);
  if (currentAutoMode == MANUAL_MODE)
  {
    tempDspeed = RC->getMappedValue(CH2);
    if (tempDspeed == -1)
    {
      desired_brake = 100;  // brake on
      desired_speed_cmPs = 0;  // stop
    }
    else
    {
      desired_brake = 0;  // brake off
      desired_speed_cmPs = tempDspeed;
    }
    desired_angle_DegX10 =  RC->getMappedValue(CH1);
   }
   if (currentAutoMode == AUTO_RC) 
   {
    tempDspeed = RC->getMappedValue(CH2);
    if (tempDspeed = -1)
    {  // E-Stop!
      desired_brake = 100;  // brake on
      desired_speed_cmPs = 0;  // stop
      currentAutoMode  = ESTOP_RC;
    }
   //recieveCan();  //check for new message  
   }
  //_____________Implement desired values__________________________________________________

  currentSpeed_cmPs = throttle->update(desired_speed_cmPs, currentDriveMode);  
  currentAngle_DegX10 = steer->update(desired_angle_DegX10);  
  outgoing.id= Actual_CANID;  
  outgoing.length = 6;
  outgoing.data.int16[0] =  currentSpeed_cmPs;
  outgoing.data.int16[2] = currentAngle_DegX10;
  sendCan();   // send current velocity
}
//*************************************************************************************
void Vehicle::updateRC() {
  currentAutoMode = RC->updateMode(currentAutoMode);
  if (currentAutoMode == ESTOP_RC || currentAutoMode == ESTOP_OP || currentAutoMode == ESTOP_BTN)
    throttle->Stop();
  currentDriveMode = RC->getDriveMode();
  for (int i = 0; i < RC_NUM_SIGNALS; i++)
  {
    RCtime[i] = RC->getEtime(i);
    RCMapped[i] = RC->getMappedValue(i);
  }
}
//*************************************************************************************
AutoMode Vehicle::int2Auto( int amode)
{
  if (amode == 0) return MANUAL_MODE;
  if (amode == 1) return OPERATOR_MODE;
  if (amode == 2) return AUTO_RC;
  return  ESTOP_RC;
}
/************************************************************************************
*  Send current vehicle velocity over CAN
*************************************************************************************/
bool Vehicle::sendCan() {
  if (Can0.sendFrame(outgoing))
    return (true);
  else 
    return (false);
}
/*************************************************************************************
   Checks for receipt of a message from CAN bus for new
   desired speed/angle/brake instructions from high-level board
 ************************************************************************************/
void Vehicle::receiveCan() {  
  //need to ADD ALL the other CAN IDs possible (RC instructions etc. 4-23-19)
  noInterrupts();  // why?
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned int canId = 0;

// CAN message receipt for system using Arduino Due
  Can0.watchForRange(Actual_CANID, HiStatus_CANID);  //filter for high level communication
  while (Can0.available() > 0) {                     // check if CAN message available
    Can0.read(incoming);                             // reading data from CAN message
    canId = incoming.id;
  }
  interrupts();
  if (canId == HiDrive_CANID) {  // the drive ID receive from high level
    if (DEBUG) {
      Serial.println("RECEIVED CAN MESSAGE FROM HIGH LEVEL WITH ID: " + String(canId, HEX));
    }
    // SPEED IN cm/s
    int16_t low_result = incoming.data.int16[0];
    desired_speed_cmPs = low_result;

    // BRAKE ON/OFF
    int16_t mid_result = incoming.data.int16[1];
    desired_brake = mid_result;
  
    // WHEEL ANGLE
    int16_t high_result = incoming.data.int16[2];
    desired_angle_DegX10 = high_result;

    if (DEBUG) {
      Serial.print("CAN Speed: " + String(low_result, DEC));
      Serial.print(", CAN Brake: " + String(mid_result, DEC));
      Serial.print(",  CAN Angle: ");
      Serial.println(high_result, DEC);
      Serial.println("mapped angle: " + String(desired_angle_DegX10));
    }
  } else if (canId == HiStatus_CANID) {  //High-level Status change (just e-stop for now 4/23/19)
    desired_speed_cmPs = 0;
   // eStop();
  }
}
