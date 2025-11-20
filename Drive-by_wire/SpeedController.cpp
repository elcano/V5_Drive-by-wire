#include <Arduino.h>
#include "DBW_Pins.h"
#include "SpeedController.h"

volatile uint32_t SpeedController::tickTime_ms[2];

SpeedController::SpeedController()
  : speedPID(&speedCyclometer_mmPs, &PIDThrottle, &desiredSpeed_mmPs, proportional_throttle, integral_throttle, derivative_throttle, DIRECT) 
{
   // Initialize Pins
  pinMode(BRAKE_ON_PIN, OUTPUT);
  pinMode(BRAKE_VOLT_PIN, OUTPUT);
  // Brakes are released as a default setting
  state = BR_OFF;
  ReleaseBrakes(); 
  brake_change_ms = 0;  // only used when brakes on
  
  currentThrottle = 0;
  speedPID.SetControlLimits(MIN_PID_TH, MAX_PID_TH);
  speedPID.SetSampleTime(PID_SAMPLE_TIME);
  speedPID.SetMode(AUTOMATIC);
  calcTime_ms[0] = 0;
  calcTime_ms[1] = 0;
  prevSpeed_mmPs = 0;

  attachInterrupt(IRPT_WHEEL, tick, RISING);
  if (DEBUG)
    Serial.println("Speed Setup Complete");
}

SpeedController::~SpeedController() {
}

// Interrupt service routine
void SpeedController::tick() {
  uint32_t tick = millis();
  noInterrupts();
  if ((tick - tickTime_ms[0]) > MIN_TICK_TIME_ms) {
    tickTime_ms[1] = tickTime_ms[0];
    tickTime_ms[0] = tick;
  }
  interrupts();
}

/**
 * receives requested speed from Vehicle requested from High Level board
 * attempts to adjust speed either through PIDS or standard engageThrottle() 
 * based on the PIDs being on or off in Settings
 * param dSpeed desired speed in mm/s
 */
int32_t SpeedController::update(int32_t dSpeed, DriveMode mode) {
  if (state == BR_HI_VOLTS && millis() > brake_change_ms)
  {
    digitalWrite(BRAKE_VOLT_PIN, OFF_BR);   // Use 12V activation  
    state = BR_LO_VOLTS;
  }
 
  //Reverse doesn't seem to work
  // if (mode == REVERSE_MODE) {
  //   dSpeed = constrain(dSpeed, 0, 150);  // prevent negatives
  //   dSpeed = map(dSpeed, 0, 150, 150, 0);  // invert DAC for reverse, or just limit
  // }

  ThrottlePID(dSpeed);
  computeSpeed();

  if (DEBUG) {
    Serial.println("mm Speed: " + String(speedCyclometer_mmPs));
    Serial.print("PWM speed: ");
    Serial.println(currentThrottle);
  }
  return currentThrottle;
}

void SpeedController::ThrottlePID(int32_t desiredValue) {
  // speedPID(&speedCyclometer_mmPs, &PIDThrottle, &desiredSpeed_mmPs
    speedPID.Compute();
    if (PIDThrottle < PID_BRAKE)
    {  // Apply brakes
      Stop();
    }
    else if (PIDThrottle < PID_COAST)
    {  // coast
      currentThrottle = 0;
      ReleaseBrakes();
    }
    else
    {  // accelerate
      currentThrottle = MAP(PIDThrottle,MIN_THROTTLE,MAX_THROTTLE,PID_COAST,MAX_PID_TH);
      ReleaseBrakes();
    }
   analogWrite(DAC0,currentThrottle);
}

int32_t SpeedController::extrapolateSpeed() {
  int32_t y;
  int32_t t = millis();
  //slope calculation
  y = (speedCyclometer_mmPs - prevSpeed_mmPs) / (calcTime_ms[0] - calcTime_ms[1]);
  // * change in time
  y *= (t - calcTime_ms[0]);
  // + current speed
  y += speedCyclometer_mmPs;

  if (y < 0)
    y = 0;
  return y;
}
/*
Uses previous two speeds to extrapolate the current speed
Used to determine when we have stopped
*/
void SpeedController::computeSpeed() {
  uint32_t tempTick[2];
  noInterrupts();
  tempTick[0] = tickTime_ms[0];
  tempTick[1] = tickTime_ms[1];
  interrupts();
  if (tempTick[1] == 0)
    speedCyclometer_mmPs = 0;
  else if (calcTime_ms[0] == 0) {
    speedCyclometer_mmPs = WHEEL_CIRCUM_MM * (1000.0 / (tempTick[0] - tempTick[1]));
    prevSpeed_mmPs = speedCyclometer_mmPs;
    calcTime_ms[1] = tempTick[1];
    calcTime_ms[0] = tempTick[0];
  } else {
    if (calcTime_ms[1] == tempTick[1]) {
      uint32_t timeDiff = millis() - calcTime_ms[0];
      if (timeDiff > MAX_TICK_TIME_ms) {
        speedCyclometer_mmPs = 0;
        if (timeDiff > (2 * MAX_TICK_TIME_ms)) {
          prevSpeed_mmPs = 0;
          noInterrupts();
          tickTime_ms[1] = 0;
          interrupts();
        }
      } else if (prevSpeed_mmPs > speedCyclometer_mmPs) {
        speedCyclometer_mmPs = extrapolateSpeed();
      }
    } else {
      calcTime_ms[1] = calcTime_ms[0];
      calcTime_ms[0] = tempTick[0];
      prevSpeed_mmPs = speedCyclometer_mmPs;
      speedCyclometer_mmPs = WHEEL_CIRCUM_MM * (1000.0 / (tempTick[0] - tempTick[1]));
    }
  }
}
void SpeedController::ReleaseBrakes() {
  // Release the brakes, state becomes BR_OFF
  // physically DISENGAGE the brakes
  digitalWrite(BRAKE_ON_PIN, OFF_BR); 
  digitalWrite(BRAKE_VOLT_PIN, OFF_BR); 
  state = BR_OFF;
}

/* Expected behavior:
    * This function should physically ENGAGE the brakes (apply 24V initially).
    * Based on observation, sending LOW to the pins physically ENGAGES.
    * Both LEDs come on for Relays 2 and 3
    * Relay 2 connects NO (solenoids) to COM (ground)
    * Relay 3 connects COM (other end of solenoids) to NC (36V)
    */
void SpeedController::Stop() {
 // set the throttle signal to zero
  analogWrite(DAC0, 0);
  currentThrottle = 0;
  if (state == BR_OFF)
  {  // first time to apply brakes
    digitalWrite(BRAKE_VOLT_PIN, ON_BR);   // Use 24V activation
    digitalWrite(BRAKE_ON_PIN, ON_BR);
    brake_change_ms = millis() + MAXHI_MS; 
    state = BR_HI_VOLTS;  
  }
  else if (state == BR_HI_VOLTS && millis() > brake_change_ms)
  {
    digitalWrite(BRAKE_VOLT_PIN, OFF_BR);   // Use 12V activation  
    state = BR_LO_VOLTS;
  }
}
