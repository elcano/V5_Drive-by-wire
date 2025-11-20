/*
 * Settings.h is meant to capture characteristics of a particular vehicle.
 * There shoould be no Settings.h file in the GitHub repository.
 * The file in the repository should be SettingsTemplate.h
 * Modify this example file to match your individual vehicle.
 * Do not check your local Settings.h into the repository.
 */

#pragma once
//#define VEHICLE_NAME "01 Orange Trike"
#define VEHICLE_NAME "02 Yellow Trike"
#define RELAYInversion true
/*=======================================================================
Minimum and maximum values to send to the motor
Minimum and maximum speed allowed
*/

// max speed
#define MAX_SPEED_KmPh 20
#define KmPh_mmPs  (1000000 / 3600) 
//at 20KMPH this is roughly 5,555 mm/s
#define MAX_SPEED_mmPs (MAX_SPEED_KmPh * KmPh_mmPs) 

// small min speed is interpreted as stopped
//set as 1% of the maximum speed -> 0.2 KmPh
#define MIN_SPEED_mmPs  (0.01 * MAX_SPEED_mmPs)

/*=========================================================================
Settings for the Steering 
Minimum/Maximum and center turning signals
*/
// Which sensors are enabled: left or right steering column; analog, SPI or CAN
#define R_ANALOG 0x01
#define R_SPI    0x02
#define R_CAN    0x04
#define L_ANALOG 0x10
#define L_SPI    0x20
#define L_CAN    0x40
// combine all present senor bits with |
#define ANGLE_CONFIG L_ANALOG 
// Minimum turn in Degrees*10;  -240 = Left 24 degrees
#define MIN_LEFT_DEGx10 -240
// Maximum turn in Degrees*10; 250 = Right 25 degrees
#define MAX_RIGHT_DEGx10 250
// if using pulse width control for steering, the minimum pulse in microseconds
#define MIN_LEFT_US 1000
// maximum steering servo pulse width in us
#define MAX_RIGHT_US 1850
// Steering pulse widths in us for straight
#define STRAIGHT_US 1500
// For motor controller board, time in ms to move motor
// negative = left; positive right
#define LEFT_ST_MS    -900
#define RIGHT_ST_MS     900


/* There are wheel angle sensors on the left and right steering columns
   Each returns an analog voltage between 0 and 5 volts.
   After the signal passes through ADC it is a value from 0 to 1023.
   Idealy we would line up the sensors so that 511 is straight ahead.
   It is not practical to get ideal alignment.
   Move the wheels to extreme left to get the read value at MIN_TURN.
   Move the wheels to extreme right to get the read value at MAX_TURN.
   These settings will vary significantly from vehicle to vehicle.
*/
/*
// These values have worked for the Carla simulator
#define Left_Read_at_MIN_TURN 532
#define Left_Read_at_MAX_TURN 206
#define Right_Read_at_MIN_TURN 106
#define Right_Read_at_MAX_TURN 532
// These are some typical values for orange trike
#define Left_Read_at_MIN_TURN 485
#define Left_Read_at_MAX_TURN 313
#define Right_Read_at_MIN_TURN 725
#define Right_Read_at_MAX_TURN 785
*/
 // Min_turn =Hard right
#define Right_Read_at_MIN_TURN 673
// Max_turn= Hard left
#define Right_Read_at_MAX_TURN 786 
// right side of drive
#define Right_Straight_Read    731 
 // Yellow Trike wheel angle sensor in ms
// wheel angle sensor had issues only using right sensor values
// Min_turn =Hard right
#define Left_Read_at_MIN_TURN 779 
// Max_turn= Hard left
#define Left_Read_at_MAX_TURN 639 
// left side of drive
#define Left_Straight_Read    722 

/*======================================================================
Vehicle Data
Wheel Diameter, Turn Radius
*/

#define WHEEL_DIAMETER_MM 495.3
//derived settings
#define WHEEL_CIRCUM_MM (WHEEL_DIAMETER_MM * PI)

/*=======================================================================
PID tuning for steering and throttle
*/

#define PID_SAMPLE_TIME 100

// Motor PID
const float proportional_throttle = .0175;
const float integral_throttle = .2;
const float derivative_throttle = .00001;

// Steering PID
const float proportional_steering = .0175;
const float integral_steering = .5;
const float derivative_steering = .00001;

/*=======================================================================
RC Controller: hardware pulse limits in microseconds
*/
#define CH1_MIN 1255
#define CH2_MIN 1264
#define CH3_MIN 1004
#define CH4_MIN 1003
#define CH5_MIN 1000
#define CH6_MIN 1000
#define CH1_MAX 1773
#define CH2_MAX 1776
#define CH3_MAX 1994
#define CH4_MAX 1997
#define CH5_MAX 2000
#define CH6_MAX 2000
#define CH1_MIDLO 1505
#define CH2_MIDLO 1505
#define CH3_MIDLO 1500
#define CH4_MIDLO 1400
#define CH5_MIDLO 1495
#define CH6_MIDLO 1495
#define CH1_MIDHI 1515
#define CH2_MIDHI 1515
#define CH3_MIDHI 1500
#define CH4_MIDHI 1600
#define CH5_MIDHI 1505
#define CH6_MIDHI 1505
/*=======================================================================
Operator: analog in on 0 -1023 scale
*/
#define OP_MIN    0
#define OP_MIDLO  500
#define OP_MIDHI  523
#define OP_MAX    1023

