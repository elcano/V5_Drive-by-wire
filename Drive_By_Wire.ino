/* Drive_By_Wire
 *  Implement vehicle throttle, brakes and steering as commanded
 *  Report status over CAN bus
 *  Accept commands over CAN bus
 */
#include "DBW_Pins.h"
#include "Vehicle.h"
#include <Arduino.h>

#define baud 115200  // baudrate for debugging with a host PC over USB serial
Vehicle *myTrike;
Logger *Log;

void setup() {
  pinMode (RED_LED_PIN, OUTPUT);
  pinMode (GREEN_LED_PIN, OUTPUT);
  pinMode (BLUE_LED_PIN, OUTPUT);
  // Set status LED to white to show initializing
  digitalWrite(RED_LED_PIN,HIGH);
  digitalWrite(GREEN_LED_PIN,HIGH);
  digitalWrite(BLUE_LED_PIN,HIGH);
  Serial.begin(baud);
  myTrike = new Vehicle();
  Log = new Logger();
}

void loop() {
   //Timing code
  const uint32_t offsetTime = 7;  // logged delays are 107 ms or more.
  uint32_t endTime;
  uint32_t delayTime;
  uint32_t timeStart_ms = millis();
  endTime = timeStart_ms + LOOP_TIME_MS;
  // Clock overflow will not happen unless if the vehicle is running for more than 49 days.
  // The batteries won't last that long without recharging.
  // Don't worry about clock rollover.
 // if (end_time < timeStart_ms) {  // clock rollover}
 
  myTrike->updateRC();   // get new desired settings from RC
  //myTrike.receiveCan(); // override desired settings if CAN active
  myTrike->update();   // set throttle, steering and brakes to implement desired settings
  //myTrike.sendCan();   // send current velocity
  Log->update();         // write key data to SD
 
  //Timing code
   delayTime = endTime - millis() - offsetTime;
   Log->EndLine(delayTime);     // Finalize SD line by giving utilization
   if (delayTime <= 0) delayTime = 0;
   if (delayTime >= LOOP_TIME_MS) delayTime = LOOP_TIME_MS;
   delay(delayTime);

}