#ifndef _DBW_PINS_
#define _DBW_PINS_

#define DEBUG false

#include <Arduino.h>

// eg  Nov 12 2025       20:50:01
#define CompileDate __DATE__ 
#define CompileTime __TIME__

//Version can be 1,3, 4, 5; LLB = Low Level Board = Drive By Wire; 1,2,3 no longer supported.
#define DBWversion 5

#define MAP(val,dlo,dhi,nlo,nhi) ((nlo) + (((val-(dlo))*((nhi)-(nlo)))/((dhi)-(dlo))))

// serial goes to serial monitor; other settings can have another microprocessor do the logging
// serial1 connects to router on Bridge; 
// serial2 goes to J9
// define serialLOG  must be logFile for logMethod 0 or a serial class for logMethod 1
// #define serialLOG Serial1
// for SD card
// #define serialLOG logfile
// serialLOG is defined in Settings.h — use SerialUSB for Bridge board, Serial for standalone Due


// Serial Monitor uses pins 0,1 for RX0,TX0
// Pins D0-D13 and A0-A5 are reserved for the Motor Shield

/*
 *  --------Motor shield pins --------------
 *  Channel A Current sensing   A0
 *  Channel B Current sensing   A1
 *  In2 Connector               A2
 *  In3 Connector               A3
 *  Channel A Speed (PWM)       3
 *  Out5 connector              5
 *  Out6 connector              6
 *  Channel B Brake             8
 *  Channel A Brake             9
 *  Channel B Speed (PWM)       11
 *  Channel A Direection        12
 *  Channel B Direection        13
 *  
 *  Version 4 connectors:
 *  RJ45 Steering: 
 *        L_RTN, L_ANG, R_RTN, R_ANG, NC (GND), STEER_PULSE/R_TURN, L_TURN, NC (5V)
 *        A chip converts (L_ANG-L_RTN) into L_SENSE and (R_ANG-R_RTN) onto R_SENSE
 *        Alternatively, first four signals can be MISO, ANG_CS, SCK, MOSI. jumpers select proper connection
 *  Two 4 wire screw terminals for CAN bus
 *        12V in, GND, CAN LO, CAN HI
 *  One 4 wire screw terminal for brake solenoid
 *        12V in, GND, 24V in, Brake solenoid out (12 or 24V)
 *        The 12V in on this connector is tied to the 12V in on the CAN bus connector,
 *        There should be only one connector to 12V power in, 
 *        A spare 12V connector could be wired to the motor shield, which
 *        already has a common ground.`
 *  One 4 wire screw terminal:
 *        12V out (controlled by STEER_ON), GND, IRPT_WHEEL, GND
 *  RJ45 to e-bike Motor Controller:
 *        FWDSW, EBIKE_POWER_IN, GND, DAC0, DAC1, SPEEDOMETER, WATCHDOG, 
 *        BRAKE_PULSE, REVERSE, REGEN, 
 */
// Wheel angles sensors mounted on left and right steering columns
#define L_SENSE_PIN        A10
#define R_SENSE_PIN        A11
 // relay had turned on 12V power for steering. Now it controls 12V power to disconnect.
// GATE_RAISE_PIN was on 28; pin 28 is now claimed by RIGHT_TURN_PIN per bridge
// wiring. If gate-raise is reintroduced, move it to a different pin.
// #define GATE_RAISE_PIN      28
#if (DBWversion <=4) 
  #define STEERING_CH1_PIN  23
  // Command to e-bike controller to drive in reverse
  #define REVERSE_PIN       24
#else
 #define STEERING_CH1_PIN   42
 #define RTS2               23
 #define CTS2               24  
 #define REVERSE_PIN        36
#endif
#define THROTTLE_BR_CH2_PIN 46
#define CH3_PIN             32
#define CH4_PIN             25
#define CH5_PIN             31
#define CH6_PIN             26
// Chip select pin for the SD card Reader
#define SD_CS_PIN           33
// Chip select if using an SPI angle sensor
#define ANG_CS_PIN          39
// Closed-loop steering: two-wire digital direction signals (per Minhee's bridge wiring).
//   DBW pin 26 (L_TURN) → Router D4  (digitalWrite HIGH while turning left)
//   DBW pin 28 (R_TURN) → Router D2  (digitalWrite HIGH while turning right)
// Pin 26 collides with CH6_PIN — RC_Controller's CH6 init must stay disabled.
#define LEFT_TURN_PIN       26
#define RIGHT_TURN_PIN      28
// STEER_PULSE_PIN was the old servo-pulse output; bridge uses the two-wire
// L_TURN/R_TURN scheme above instead. Aliased to RIGHT_TURN_PIN so any
// legacy reference still resolves to a valid pin.
#define STEER_PULSE_PIN     RIGHT_TURN_PIN
// steer pins on motor control board
#define STEER_SPEED_PIN     3
#define STEER_ON_PIN        9
#define STEER_DIR_PIN       12
#define ST_ON  HIGH
#define ST_OFF (!ST_ON)
#define ST_LEFT HIGH
#define ST_RIGHT (!ST_LEFT)

// Not used: reserved to turn on a light
#define DBW_LED             30

// Pins 34 and 35 could conflict with the Router board on the Bridge.
// Switch to drive in forward or reverse; not used
#define FWDSW_PIN           36
// Not used. Buzzer can make warning sound when vehicle moves autonomously
#define BUZZER_PIN          38 
// Reserved for regenerative braking
#define REGEN_PIN            43 
// Restored to pin 44 now that steering uses the proper L_TURN/R_TURN wires
// (DBW D26/D28). The Router brake wire (DBW D44 → Router D48) is the live path.
#define BRAKE_ON_PIN         44
// Brakes, have relays for both on/off as well as selecting 12/24v power.
#define BRAKE_VOLT_PIN       40

/* Any Arduino Due pin can be an interrupt. */

// Multiple clicks per wheel revolution (not implemented)
#define SPEEDOMETER_PIN    22
// When the E-bike controller has power, this line is 5V.
// If it falls to zero, ebike has lost power.
#define EBIKE_POWER_IN_PIN 45

// Wheel click interrupt (digitally high or low, referred to as an "odometer"). Once per wheel revolution.
// Odometer reed switch is pulled up to high voltage.
// IRPT_WHEEL = WHEELROTATION
#define IRPT_WHEEL         47
// operator controls
#define OP_CENTER     A6
#define OP_STEER      A7
#define OP_THROTTLE   A8
#define OP_MODE_PIN   35
#define OP_FWD_PIN    36
#define OP_DISCNT_PIN 37
#define OP_ESTOP      49
// Number of RC channels
#define RC_NUM_SIGNALS 6
// RC Channels
// Steering
#define CH1 0  
// Thottle and brake
#define CH2 1 
// two-position with light: Forward / Reverse
#define CH3 2 
// three-position: Control from RC / Operator / CAN
#define CH4 3
// Analog: Disconnect coupler
#define CH5 4
// Analog; reerved
#define CH6 5

enum DriveMode {
    REVERSE_MODE,
    FORWARD_MODE
};
#define RED_LED_PIN 51
#define GREEN_LED_PIN 52
#define BLUE_LED_PIN 53
enum AutoMode {
    INITIALIZING,  // No action
    MANUAL_MODE,   // Under RC control
    OPERATOR_MODE, // controlled by on-board operator
    AUTO_RC,    // Automatic with Estop from RC
    AUTO_OP,    // Automatic with Estop from operator
    ESTOP_RC,   // Estoped with RC in control
    ESTOP_OP,   // Estoped with operator in control
    ESTOP_BTN   // operator pushed E-Stop button
};

#endif   //_DBW_PINS_
