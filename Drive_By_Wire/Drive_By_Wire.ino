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
  // Always begin Serial, even on the Bridge where it is not cabled to a PC.
  // Logger's initRTC()/initSD()/openSD() print to Serial unconditionally, and
  // UARTClass::write() spin-locks forever once its 128-byte buffer fills if
  // begin() was never called — the TX interrupt that drains it is enabled by
  // begin(). That would hang setup() and loop() would never run.
  Serial.begin(baud);
#ifdef USE_NATIVE_USB
  SerialUSB.begin(baud);
  // Wait up to 500 ms for a serial monitor; proceed regardless so automated
  // tests are not blocked when no monitor is connected. 500 ms is enough
  // for a monitor that was open before reset; cold-boot without monitor
  // starts DBW in < 500 ms so it's ready before the first test assertion.
  { uint32_t t = millis(); while (!SerialUSB && (millis() - t) < 500); }
#endif
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

  myTrike->updateRC();      // get new desired settings from RC
  myTrike->receiveCan();    // override desired settings if CAN active
  myTrike->update();        // set throttle, steering and brakes to implement desired settings
                        // update sends a CAN message with actual velocity
  Log->update();         // write key data to SD
  //Timing code
  delayTime = endTime - millis() - offsetTime;
   Log->EndLine(delayTime);     // Finalize SD line by giving utilization
  if (delayTime <= 0) delayTime = 0;
  if (delayTime >= LOOP_TIME_MS) delayTime = LOOP_TIME_MS;
  // Instead of plain delay(), spin-drain CAN so we don't miss Sensor Hub's
  // 0x350/0x100 frames while DBW's own Logger floods the bus with 0x701-0x70A.
  uint32_t spinUntil = millis() + delayTime;
  while (millis() < spinUntil) {
    myTrike->receiveCan();
  }

}
