#include <Arduino.h>
#include "DBW_Pins.h"
#include "Vehicle.h"
#include "Can_Protocol.h"
#include "Settings.h"

RC_Controller* Vehicle::RC;
SpeedController* Vehicle::throttle;
SteeringController* Vehicle::steer;

int16_t Vehicle::desired_speed_mmPs;
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
  currentSpeed = 0;
  currentAngle_DegX10 = 0;
  currentBrake = 0;
  desired_speed_mmPs = 0;
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
/*****************************************************************************
   Struct for sending current speed and angle to high-level board through CAN
 ****************************************************************************/
typedef union {
  struct {
    uint16_t sspeed;
    uint16_t brake;
    uint16_t angle;
    uint16_t reserved;
  };
} speedAngleMessage;

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
      desired_speed_mmPs = 0;  // stop
    }
    else
    {
      desired_brake = 0;  // brake off
      desired_speed_mmPs = tempDspeed;
    }
    desired_angle_DegX10 =  RC->getMappedValue(CH1);
   }
   if (currentAutoMode == AUTO_RC) 
   {
    tempDspeed = RC->getMappedValue(CH2);
    if (tempDspeed = -1)
    {  // E-Stop!
      desired_brake = 100;  // brake on
      desired_speed_mmPs = 0;  // stop
      currentAutoMode  = ESTOP_RC;
    }
   //recieveCan();  //check for new message  
   }
  //_____________Implement desired values__________________________________________________

  currentSpeed = throttle->update(desired_speed_mmPs, currentDriveMode);   // <- use '->'
  currentAngle_DegX10 = steer->update(desired_angle_DegX10);      // <- use '->'
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
void Vehicle::sendCan() {
  //build struct to send data to Highlevel through CAN
  speedAngleMessage MSG;
  MSG.sspeed = currentSpeed;
  MSG.brake = currentBrake;
  MSG.angle = currentAngle_DegX10;
  MSG.reserved = 0;

  // unknown status (120 - 145)
 
//   outgoing.data.int16[0] = MSG.sspeed;
//   outgoing.data.int16[1] = MSG.brake;
//   outgoing.data.int16[2] = MSG.angle;
//   outgoing.data.int16[3] = MSG.reserved;
//   Can0.sendFrame(outgoing);

  if (DEBUG) {
    if (Can0.sendFrame(outgoing)) {
      Serial.println("Sending Message to DUE");
    } else {
      Serial.println("Message Failed");
    }
  }
  // Update every LOOP_TIME_MS = 100 ms
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
    // SPEED IN mm/s
    int16_t low_result = incoming.data.int16[0];
    desired_speed_mmPs = low_result;

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
    desired_speed_mmPs = 0;
   // eStop();
  }
}