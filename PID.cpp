/**********************************************************************************************
 * Arduino PID Library - Version 1.2.2
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * Modifications by Tyler Folsom: Change doubles to floats; change names
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include <Arduino.h>
#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(float* Feedback, float* Control, float* Setpoint,
        float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
    myControl = Control;
    myFeedback = Feedback;
    mySetpoint = Setpoint;
    fbAuto = false;

    PID::SetControlLimits(0, 255);				//default control limit corresponds to
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(float* Feedback, float* Control, float* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection)
    :PID::PID(Feedback, Control, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Control needs to be computed.  returns true when the control is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!fbAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      float feedback = *myFeedback;
      float error = *mySetpoint - feedback;
      float dFeedback = (feedback - lastFeedback);
      controlSum+= (ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pOnE) controlSum-= kp * dFeedback;

      if(controlSum > controlMax) controlSum= controlMax;
      else if(controlSum < controlMin) controlSum= controlMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   float control;
      if(pOnE) control = kp * error;
      else control = 0;

      /*Compute Rest of PID Control*/
      control += controlSum - kd * dFeedback;

	    if(control > controlMax) control = controlMax;
      else if(control < controlMin) control = controlMin;
	    *myControl = control;

      /*Remember some variables for next time*/
      lastFeedback = feedback;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   pOn = POn;
   pOnE = POn == P_ON_E;

   dispKp = Kp; dispKi = Ki; dispKd = Kd;

   float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetControlLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the control will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetControlLimits(float min, float max)
{
   if(min >= max) return;
   controlMin = min;
   controlMax = max;

   if(fbAuto)
   {
	   if(*myControl > controlMax) *myControl = controlMax;
	   else if(*myControl < controlMin) *myControl = controlMin;

	   if(controlSum > controlMax) controlSum= controlMax;
	   else if(controlSum < controlMin) controlSum= controlMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !fbAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    fbAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
   controlSum = *myControl;
   lastFeedback = *myFeedback;
   if(controlSum > controlMax) controlSum = controlMax;
   else if(controlSum < controlMin) controlSum = controlMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Control leads
 * to +Feedback) or a REVERSE acting process(+Control leads to -Feedback.)  we need to
 * know which one, because otherwise we may increase the control when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(fbAuto && Direction !=controllerDirection)
   {
	    kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::GetKp(){ return  dispKp; }
float PID::GetKi(){ return  dispKi;}
float PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  fbAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

