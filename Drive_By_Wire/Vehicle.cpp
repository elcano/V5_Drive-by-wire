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
  canActive = false;
  last_nav_speed_cmPs = 0;
  last_nav_brake = 0;
  last_nav_mode = 0;
  last_nav_angle_DegX10 = 0;
  last_nav_status = 0;

   if (!Can0.begin(CAN_BPS_500K))  // initialize CAN with 500kbps baud rate
  {
    Serial.println("Can0 init success");
  } else {
     Serial.println("Can0 init failed");
  }
  Can0.watchFor(HiStatus_CANID);   // listen for 0x100 (desired speed)
  Can0.watchFor(HiDrive_CANID);    // listen for 0x350 (desired angle)
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
  // currentAutoMode is set by updateRC() before update() is called.
  // Do not re-derive it here; int2Auto() expects switch positions, not AutoMode enum values.
  int16_t tempDspeed;
  if (currentAutoMode == MANUAL_MODE && !canActive)
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
  if (currentAutoMode == OPERATOR_MODE)
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
    desired_angle_DegX10 = RC->getMappedValue(CH1);
  }
   if (currentAutoMode == AUTO_RC)
   {
    tempDspeed = RC->getMappedValue(CH2);
    if (tempDspeed == -1)
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
  outgoing.id = Actual_CANID;
  outgoing.length = 6;
  outgoing.data.int16[0] = currentSpeed_cmPs;
  outgoing.data.int16[2] = currentAngle_DegX10;
  sendCan();   // send current velocity

  // 0x200 LowStatus - status-bit mirror so Nav can see what mode DBW is in
  outgoing.id = LowStatus_CANID;
  outgoing.length = 1;
  uint8_t status = 0;
  if (currentAutoMode == ESTOP_RC || currentAutoMode == ESTOP_OP || currentAutoMode == ESTOP_BTN) status |= 0x80;
  if (currentAutoMode == AUTO_RC || currentAutoMode == AUTO_OP) status |= 0x40;
  if (currentDriveMode == REVERSE_MODE) status |= 0x04;
  outgoing.data.uint8[0] = status;
  sendCan();
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
   desired speed/angle/brake instructions from high-level board.
   Protocol per https://www.elcanoproject.org/wiki/Communication :
     0x350 HiDrive (6 bytes used):
       bytes 0-1: CommandedSpeed int16 LE (cm/s)
       byte  2:   Brake uint8 (0=off, 1=hold/12V, 2=on/24V)
       byte  3:   Mode  uint8 (0-7 — Nav-requested state; DBW arbitrates)
       bytes 4-5: CommandedSteerAngle int16 LE (deg x 10)
     0x100 HiStatus (1 byte used):
       byte 0 bits: 0x80=E-stop, 0x40=autonomous, 0x04=reverse,
                    0x02=reverse pending, 0x01=reverse unavailable
 ************************************************************************************/
void Vehicle::receiveCan() {
  while (Can0.available() > 0) {
    Can0.read(incoming);
    if (incoming.id == HiDrive_CANID) {
      desired_speed_cmPs   = incoming.data.int16[0];   // bytes 0-1
      desired_brake        = incoming.data.uint8[2];   // byte 2 (was int16 — wiki spec is uint8)
      desired_angle_DegX10 = incoming.data.int16[2];   // bytes 4-5
      // Buffer raw Nav-sent values for 0x704 Log_auto emit.
      last_nav_speed_cmPs   = desired_speed_cmPs;
      last_nav_brake        = incoming.data.uint8[2];
      last_nav_mode         = incoming.data.uint8[3];  // byte 3 mode (informational)
      last_nav_angle_DegX10 = desired_angle_DegX10;
      canActive = true;
    } else if (incoming.id == HiStatus_CANID) {
      uint8_t status = incoming.data.uint8[0];
      last_nav_status = status;
      if (status & 0x80) {
        currentAutoMode = ESTOP_RC;                    // 0x80 E-stop
      } else {
        currentAutoMode = (status & 0x40) ? AUTO_RC    // 0x40 set  -> autonomous
                                          : MANUAL_MODE;  // 0x40 clear -> manual
      }
      currentDriveMode = (status & 0x04) ? REVERSE_MODE : FORWARD_MODE;
      canActive = true;
    }
  }
}
