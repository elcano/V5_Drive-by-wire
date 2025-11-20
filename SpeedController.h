#pragma once
#include "Settings.h"
#include <SPI.h>
#include "PID.h"
//min/max throttle from the PID in arbitrary Units
// if PID_Thottle < 0, apply brakes
#define MIN_PID_TH -255
#define PID_BRAKE  -100
#define PID_COAST   10
#define MAX_PID_TH  255
/* Current Calibration: (Last Update: May 14, 2023)
1.187 V: nothing 75
1.20 V: Just starting 82
1.345 V: Slow, steady 85
1.50 V: Brisker, 100
1.85 V: 120 - Maximum Count (for current testing purposes, can extend maximum count)
*/
#define MIN_THROTTLE 75
// Max throttle can go higher. Being cautious for now
#define MAX_THROTTLE 175
/* Time that the brakes can be high */
#define MAXHI_MS 800

class SpeedController {
private:
  int32_t currentThrottle = 0;
  float speedCyclometer_mmPs = 0;
  float PIDThrottle;
  float desiredSpeed_mmPs = 0;
  PID speedPID;

  static const uint32_t MIN_TICK_TIME_ms = (WHEEL_CIRCUM_MM * 1000) / MAX_SPEED_mmPs;
  static const uint32_t MAX_TICK_TIME_ms = (WHEEL_CIRCUM_MM * 1000) / MIN_SPEED_mmPs;

  static volatile uint32_t tickTime_ms[2];
  uint32_t calcTime_ms[2];
  int32_t prevSpeed_mmPs;

  void ThrottlePID(int32_t desiredValue);
  int32_t extrapolateSpeed();
  void computeSpeed();
  static void tick();
  enum brake_state { BR_OFF,
                    BR_HI_VOLTS,
                    BR_LO_VOLTS } state;
  volatile uint32_t brake_change_ms;
public:
  SpeedController();
  ~SpeedController();
  void Stop();
  // returns currentThrottle
  int32_t update(int32_t dSpeed, DriveMode mode);
  void ReleaseBrakes();
 };
//_______________Brakes___________________________
/* Solenoid controlled Brakes.
 * * Solenoid (Johnson Electic model 150 174432-024) 
 * can be kept at lower voltage (12V) indefinitely. 
 * It has a holding force of 14.5 lb.
 * At the higher voltage, data sheet expects it to be high (24V) for
 * 25% of the time and low for 75%. However, it can stay high for 100 sec.
 * The solenoid typically reacts in less than a second.
 * For 0.25 inch throw and 24V it can pull 7 lb.
 * We are using the part with 0.3 inch throw, 12V in low state and 24V in high
 * state, but keeping voltage high for only a second or two.
 *   
 * The solenoids are controlled by relays on two digital lines.
 * * Tyler Folsom   April 2018   
 * * * Expected behavior
 * * When Green LED is on, NO is connected to COM; NC is not
 * * Writing HIGH to a relay will turn LED on, and connect NO to COM
 * * When green LED is off, NC is connected to COM; NO is not.
 * * Writing LOW to a relay will turn LED off, and connect NC to COM.
 * * You shoud hear a click when the relay operates.
 * * If there is a change in LED, but no click, the relay does not have enough power.
 */
// Invert the RELAYInversion flag to correct brake logic
#if RELAYInversion
#define ON_BR LOW
#define OFF_BR HIGH
#else
#define ON_BR HIGH
#define OFF_BR LOW
#endif

