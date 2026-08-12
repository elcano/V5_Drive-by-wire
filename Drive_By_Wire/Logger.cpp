
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <stdio.h>
#include <Arduino.h>
#include "DBW_Pins.h"
#include "Vehicle.h"
#include "Can_Protocol.h"

tmElements_t tm;
extern Vehicle *myTrike;

/* Commit a finished CSV row to the log sink.

   serialLOG is a macro naming the sink object, so a bare `serialLOG.flush()`
   means two different things: on the SD `logfile` it commits the 512-byte cache
   and updates the directory entry (without it LOG00.CSV reads as 0 bytes), but
   on SerialUSB it blocks until a USB host drains the TX buffer, which freezes
   the 100 ms control loop. Commit e2e168f deleted the shared call to stop the
   freeze and silently took SD's commit with it.

   Overload resolution keeps both: File matches the non-template exactly and
   gets the real flush(); every serial port falls to the template, whose empty
   body compiles to nothing, so the blocking flush is not in the binary at all. */
/* Name of the steering method compiled in, so a log says which mode produced it. */
#if   (STEER_METHOD == STR_MOTOR_CONTROL)
  #define STEER_METHOD_NAME "motor_control"
#elif (STEER_METHOD == SRT_HBRIDGE)
  #define STEER_METHOD_NAME "hbridge_2wire"
#else
  #define STEER_METHOD_NAME "servo_pwm"
#endif

static inline void flushSink(File &f) { f.flush(); }
template <typename T> static inline void flushSink(T &) {}

/* Is serialLOG the SD File, or a serial port? This has to be decided by TYPE at
   compile time. Testing the objects (`serialLOG == logfile`) instead calls
   operator bool() on both sides, and for SerialUSB that reports "is a USB host
   attached" — false at boot — which wrongly selects the SD branch. */
template <typename A, typename B> struct SinkIsFile      { static const bool value = false; };
template <typename A>             struct SinkIsFile<A,A> { static const bool value = true;  };

/* begin() a serial sink; no-op for the SD File, which has no begin(baud). */
static inline void beginSink(File &, uint32_t) {}
template <typename T> static inline void beginSink(T &port, uint32_t baud) { port.begin(baud); }

/* May we write a CSV row right now?
   Only the native USB port can stall: Serial_::write() discards bytes when no
   host is attached, and a full TX buffer blocks the 100 ms control loop. So the
   host check must apply ONLY when the sink really is SerialUSB. The old
   `#ifdef USE_NATIVE_USB` guard applied it to EVERY sink, which meant that on a
   Bridge board with USE_NATIVE_USB set, SD rows were written only while a PC had
   the USB port open — the card got nothing when running standalone. */
static inline bool sinkReady(File &, int)          { return true; }
static inline bool sinkReady(Serial_ &s, int room) { return s && s.availableForWrite() >= room; }
template <typename T> static inline bool sinkReady(T &, int) { return true; }

/* Boot diagnostics for the Bridge board, where the programming port is not
   cabled to a PC and Serial.println() goes nowhere anyone can see. Queue the
   text here; sendBootReport() emits it on the native USB port from loop() once
   a host is attached. It cannot be printed when it happens — operator bool() is
   false while millis() < 500 and write() drops bytes with no host, so a direct
   print at boot is lost. '#'-prefixed so a CSV parser treats them as comments. */
static char     bootReport[220];
static uint16_t bootLen  = 0;
static bool     bootSent = false;

static void bootMsg(const char *tag, const char *val) {
  if (bootLen >= sizeof(bootReport) - 1) return;
  int n = snprintf(bootReport + bootLen, sizeof(bootReport) - bootLen, "# %s%s\n", tag, val);
  if (n > 0) {
    uint16_t room = (uint16_t)(sizeof(bootReport) - bootLen - 1);
    bootLen += ((uint16_t)n < room) ? (uint16_t)n : room;
  }
}
static void bootMsg(const char *tag, int val) {
  char b[12];
  snprintf(b, sizeof(b), "%d", val);
  bootMsg(tag, b);
}
static void sendBootReport() {
  if (bootSent || bootLen == 0) return;
  if (!SerialUSB || SerialUSB.availableForWrite() < 64) return;
  SerialUSB.write((const uint8_t *)bootReport, bootLen);
  bootSent = true;
}

/****************************************************************************
 Data Logger
 Record all stste data on each time slice.
 This is the priomary method for debugging. Add new parameters as needed.
 The data log provides information useful for crafting control strategies and simulators.

 The original method was to log to an SD card.
 Options have been added to log to CAN or a serial connection.
 The CAN device handling logging will respond to the appropriate messages and write the log.

 The serial device could be any of the four lines on the Due
 Serial goes to the serial monitor on the Due, but is confounded by other messages
 On the Bridge circuit board, DBW Serial1 is connected to router serial 2.
 On the DBW v5 board, Serial2 goes to connector J9
 Serial3 could also be used.  
 The receiving serial device could display data on its serial monitor or write it to a file.

 In DBW_Pins.h, set serialLOG to logfile (for SD) or to one of the four serial ports.
 In Logger::Initialize, set logMethod to 0 for SD, 1 for Serial or 2 for CAN.

   Constructor
 ****************************************************************************/
Logger::Logger()
{
  initialize();
#ifndef USE_NATIVE_USB
  Serial.println("Logger constructor finished.");
#endif
}

/*****************************************************************************
   Destructor
 ****************************************************************************/
Logger::~Logger() {
}

/*******************************
Initialize RTC and SD Card
********************************/ 
void Logger::initialize() {
#ifdef HAS_RTC
  initRTC();
#endif
  // Pick the CSV sink. serialLOG (Settings.h) is either `logfile` (SD) or a
  // serial port; which one is a compile-time fact, decided by type.
  out = &serialLOG;
  logMethod = SinkIsFile<decltype(serialLOG), decltype(logfile)>::value ? 0 : 1;
  if (logMethod == 0) {
    if (!initSD() || !openSD()) {
      // SD unavailable. Rather than go silent (logMethod 2 writes nothing
      // anywhere), fall back to a serial port so a log still exists.
      // On the Bridge board the programming port is not cabled to a PC, so
      // fall back to the native USB port there instead — otherwise the
      // "always written" guarantee would write somewhere nobody can read.
#ifdef USE_NATIVE_USB
      out = &SerialUSB;
      bootMsg("SD unavailable; CSV falls back to SerialUSB", "");
#else
      out = &Serial;
      Serial.println("SD unavailable — falling back to serial log.");
      bootMsg("SD unavailable; CSV falls back to Serial", "");
#endif
      logMethod = 1;
    }
  } else {
    // Serial and SerialUSB are begun in setup(); Serial1/2/3 need it here.
    beginSink(serialLOG, 115200);
  }

  bootMsg("logMethod=", logMethod);   // 0=SD 1=serial 2=none
  bootMsg("steer=", STEER_METHOD_NAME);

  // CAN: always emit session header so any listener (Sensor Hub Due, debug laptop, etc.)
  // sees a new session on the bus regardless of local SD/serial availability.
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = Header_CANID;
  outCAN->length = 0;
  myTrike->sendCan();
#ifndef USE_NATIVE_USB
  Serial.println("CAN log session opened (0x700).");
#endif

  // Header is deferred to the first update() — see writeHeader().
  headerSent = false;
  if (logMethod != 2) {
#ifndef USE_NATIVE_USB
    Serial.println(logMethod == 0 ? "SD log initialized." : "Serial log initialized.");
#endif
  } else {
#ifndef USE_NATIVE_USB
    Serial.println("No SD/serial sink — CAN log emit only.");
#endif
  }
}
/*******************************
Initialize real time clock
********************************/ 
// Convert the compiler's __DATE__/__TIME__ into tmElements_t. Build time is at
// worst a few hours stale, which is far closer to real time than an unset DS1307.
static bool buildTimeToTm(tmElements_t &t) {
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {0};
  int day = 0, year = 0, hh = 0, mi = 0, ss = 0;
  if (sscanf(CompileDate, "%3s %d %d", mon, &day, &year) != 3) return false;
  if (sscanf(CompileTime, "%d:%d:%d", &hh, &mi, &ss) != 3)     return false;
  const char *p = strstr(months, mon);
  if (!p) return false;
  t.Month  = ((p - months) / 3) + 1;
  t.Day    = (uint8_t)day;
  t.Year   = (uint8_t)(year - 1970);   // tmElements_t Year is an offset from 1970
  t.Hour   = (uint8_t)hh;
  t.Minute = (uint8_t)mi;
  t.Second = (uint8_t)ss;
  return true;
}

bool Logger::initRTC()
{
  // Always have a usable timestamp, even with no RTC at all.
  strncpy(timeString, CompileTime, sizeof(timeString) - 1);
  timeString[sizeof(timeString) - 1] = '\0';
  strncpy(dateString, CompileDate, sizeof(dateString) - 1);
  dateString[sizeof(dateString) - 1] = '\0';

  bool ok = RTC.read(tm);

  if (!ok) {
    // The chip is stopped or unreadable. Previously this branch only printed
    // "Please run SetTime" and the clock was never corrected, so every log
    // carried a meaningless timestamp. Set it from build time instead.
    tmElements_t bt;
    if (buildTimeToTm(bt) && RTC.set(makeTime(bt))) {
      Serial.println("RTC was not running — set from build time.");
      bootMsg("RTC was not running; set from build time", "");
      ok = RTC.read(tm);
    } else if (RTC.chipPresent()) {
      Serial.println("DS1307 present but could not be set.");
      bootMsg("DS1307 present but could not be set", "");
    } else {
      Serial.println("DS1307 not found — using build time for log timestamps.");
      bootMsg("DS1307 not found; using build time", "");
    }
  }

  if (ok) {
    snprintf(timeString, sizeof(timeString), "%02u:%02u:%02u",
             tm.Hour, tm.Minute, tm.Second);
    snprintf(dateString, sizeof(dateString), "%02u/%02u/%04u",
             tm.Day, tm.Month, (unsigned)(tm.Year + 1970));
    Serial.print("RTC time = ");            Serial.print(timeString);
    Serial.print("  date (D/M/Y) = ");      Serial.println(dateString);
    bootMsg("RTC ", dateString);
    bootMsg("RTC ", timeString);
  }
  return ok;
}
/*******************************
Initialize SD Card
********************************/ 
bool Logger::initSD()
{
  // Initialize SD Card
  Serial.print("Initializing SD card...");
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(10,OUTPUT);   // Shield may use pin 10

  if (!SD.begin(SD_CS_PIN)) {
    bootMsg("SD.begin FAILED on CS pin ", (int)SD_CS_PIN);
    logfile = File();  // mark logfile as invalid
    return (false);
  }
  bootMsg("SD.begin OK on CS pin ", (int)SD_CS_PIN);
  return (true);
}
/*******************************
Open the SD Card
********************************/ 
bool Logger::openSD()
{
  // Name the log from the RTC so a file is identifiable by when it was recorded:
  // DDHHMMSS.CSV, which is exactly the 8 characters FAT 8.3 allows.
  // initRTC() runs before this in initialize(), so tm is already populated.
  //
  // Seconds rather than the month, because uploading then opening the serial
  // monitor resets the Due twice about a second apart (the programming port
  // toggles DTR on connect). At minute resolution the first boot took the dated
  // name and the second — the run actually being recorded — fell back to LOGnn.
  // The month is recoverable from the file's own timestamp.
  bool haveDate = (tm.Day >= 1 && tm.Day <= 31 && tm.Hour <= 23 && tm.Minute <= 59);
  if (haveDate) {
    snprintf(logName, sizeof(logName), "%02u%02u%02u%02u.CSV",
             tm.Day, tm.Hour, tm.Minute, tm.Second);
  }
  // No usable clock, or a run already started in this same minute:
  // fall back to the original sequential name so we never fail to open.
  if (!haveDate || SD.exists(logName)) {
    for (int i = 0; i < 100; i++) {
      snprintf(logName, sizeof(logName), "LOG%02d.CSV", i);
      if (!SD.exists(logName)) break;
    }
  }
  logfile = SD.open(logName, FILE_WRITE);
  if (!logfile) {
    Serial.print("FAILED to open file: ");
    Serial.println(logName);
       return (false);
  }
  Serial.print("Logging to: ");
  Serial.println(logName);
  bootMsg("logging to ", logName);
  return (true);
}
/****************************************
Write a new set of data to all available sinks.
  - CAN log emit is unconditional (every loop tick puts 0x701-0x70A on the bus).
  - SD/serial write only if logMethod != 2 (i.e. that sink was successfully initialized).
****************************************/
/****************************************
Write the CSV header. Called from update(), not initialize(): on the native USB
port initialize() runs before the host opens the port and the bytes would be
discarded, leaving you with data rows and no column names.
****************************************/
void Logger::writeHeader() {
  out->print(VEHICLE_NAME);
  out->print(",");
  out->print(timeString);
  out->print(",");
  out->println(dateString);
  HdrTime();
  HdrRC();
  HdrOp();
  HdrCAN();
  HdrRequests();
  HdrDesired();
  HdrThrottle();
  HdrBrakes();
  HdrSteer();
  HdrEndLine();
}

void Logger::update() {
  sendBootReport();  // boot diagnostics -> native USB, once a host is listening
  // CAN log emit — always on. Sensor Hub Due (and any other listener) picks up from the bus.
  CANLogTime();      // 0x701
  CANLogRC();        // 0x702
  CANLogOp();        // 0x703
  CANLogAuto();      // 0x704
  CANLogDesired();   // 0x705
  CANLogThrottle();  // 0x706
  CANLogBrakes();    // 0x707
  CANLogSteer();     // 0x708
  // 0x709 LogPosition is emitted by Nav (not DBW) — left for the Sensor Hub log row.

  // SD/serial: write a CSV row if that sink is available.
  // On the Due's native USB, SerialUSB.print()/flush() BLOCK on a full TX
  // buffer when no host is draining it, which freezes the control loop
  // (steering/throttle stop updating — works only with a monitor open).
  // Only write when the USB TX buffer has room for a whole row, so a host that
  // isn't reading can never make print() block. If there's no room, skip the
  // row entirely — the control loop always keeps running.
  if (logMethod != 2 && sinkReady(serialLOG, 200)) {
    if (!headerSent) { writeHeader(); headerSent = true; }
    TxTime();
    TxLogRC();
    TxOp();
    TxCAN();
    TxRequests();
    TxDesired();
    TxThrottle();
    TxBrakes();
    TxSteer();
  }
  // Caller must finish the row with EndLine() (emits 0x70A and terminates CSV line).
}
/******************************************************
Line starts with a relative time stamp in milliseconds
*******************************************************/ 
void Logger::HdrTime() {
    out->print("time_ms");
}
void Logger::TxTime()
{
   out->print(millis());
}
// 0x701 LogTime — 4 bytes uint32 LE: relative time in milliseconds.
void Logger::CANLogTime() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogTime_CANID;
  outCAN->length = 4;
  outCAN->data.uint32[0] = millis();
  myTrike->sendCan();
}
/******************************************************
Include what has been commanded by Radio Control
*******************************************************/ 
void Logger::HdrRC() {
  // expected order
  // out->print(",Ch1,Ch2,Ch3,Ch4,Ch5,Ch6");
  // out->print(",Map1,Map2,DriveMode,AutoMode,Map5,Map6");
  // What we see

    out->print(",Ch1Str,Ch2ThB,Ch3,Ch4,Ch5,Ch6");
    // Sixth column is ValuesMapped[CH6], the reserved analog channel scaled
    // 0..100 - not an e-stop button. It was labelled EStopBtn, which made a
    // routine CH6 sweep look like someone hammering the e-stop.
    out->print(",MapStr,MapThB,Map3,AutoMode,Map5,Map6");
}

void Logger::TxLogRC() {
  int i;
  unsigned long data;
  long mappd;
  static bool first_time = true; 
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    data = getRCtime(*myTrike, i);
    out->print(",");
    out->print(data);
  }
  for (i = 0; i < RC_NUM_SIGNALS; i++) {
    // The AutoMode column is ValuesMapped[CH4], which RC_Controller only assigns
    // in its MANUAL_MODE and OPERATOR_MODE branches - modes 3..7 (AUTO_RC,
    // AUTO_OP and the e-stop states) hit `default:` and leave the previous value
    // in place. So the column could only ever show 0, 1 or 2 and silently went
    // stale everywhere else. Log Vehicle's actual mode for that position.
    mappd = (i == CH4) ? getAutoMode(*myTrike) : getRCmapped(*myTrike, i);
    out->print(",");
    out->print(mappd);
  }
  if (first_time)
  {  // Show on serial monitor. Skipped on Bridge board: Serial (pins 0/1) connects to Router Arduino and blocks if not reading
    #ifndef USE_NATIVE_USB
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      data = getRCtime(*myTrike, i);
      Serial.print(data);
      Serial.print(", ");
    }
    Serial.println(" ");
    for (int i = 0; i < RC_NUM_SIGNALS; i++) {
      mappd = getRCmapped(*myTrike, i);
      Serial.print(mappd);
      Serial.print(", ");
    }
    Serial.println(" ");
    #endif
    first_time = false;
  }
}
// 0x702 LogRC — 8 bytes: 4 RC channel pulse widths as int16 LE microseconds.
// RC has 6 channels; emit the 4 most useful (CH1 steer, CH2 throttle/brake, CH3 fwd/rev, CH4 mode).
void Logger::CANLogRC() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogRC_CANID;
  outCAN->length = 8;
  outCAN->data.int16[0] = (int16_t)getRCtime(*myTrike, CH1);
  outCAN->data.int16[1] = (int16_t)getRCtime(*myTrike, CH2);
  outCAN->data.int16[2] = (int16_t)getRCtime(*myTrike, CH3);
  outCAN->data.int16[3] = (int16_t)getRCtime(*myTrike, CH4);
  myTrike->sendCan();
}
/******************************************************
Include wthe goals, either from RC or CAN
*******************************************************/ 
/******************************************************
What each control source is ASKING for, whatever the mode.

The desired_* group that follows is what the vehicle actually acts on, and it
only changes while that source is selected - so in AUTO_OP, ESTOP_* or
INITIALIZING it freezes and tells you nothing. These six columns are computed
every loop from both sources, so you can see what the RC stick and the operator
joystick each want even when neither is in control. Compare them against
desired_* to see which source won, and against AutoMode to see why.
*******************************************************/
void Logger::HdrRequests() {
  out->print(",rc_speed_cmPs,rc_brake,rc_angle_DegX10");
  out->print(",op_speed_cmPs,op_brake,op_angle_DegX10");
}
void Logger::TxRequests() {
  out->print(",");
  out->print(getRC_speed(*myTrike));
  out->print(",");
  out->print(getRC_brake(*myTrike));
  out->print(",");
  out->print(getRC_angle(*myTrike));
  out->print(",");
  out->print(getOP_speed(*myTrike));
  out->print(",");
  out->print(getOP_brake(*myTrike));
  out->print(",");
  out->print(getOP_angle(*myTrike));
}
void Logger::HdrDesired() {
  // cmPs, not ms: the value is getD_speed_cmPs(), and MAX_SPEED_cmPs is the
  // scale RC_Controller maps CH2 onto.
  out->print(",desired_speed_cmPs,desired_brake,desired_angle_DegX10");
}
void Logger::TxDesired() {
  out->print(",");
  out->print( getD_speed_cmPs(*myTrike));
  out->print(",");
  out->print( getD_brakes(*myTrike));
  out->print(",");
  out->print( getD_Angle(*myTrike));
}
// 0x705 LogDesired — 6 bytes: the active source's commands (same shape as 0x350).
//   bytes 0-1: speed int16 LE (cm/s)
//   byte  2:   brake uint8 (wiki: 0=off, 1=hold, 2=on)
//   byte  3:   mode  uint8 (DBW's current AutoMode, 0-7 — matches wiki mode table)
//   bytes 4-5: steer angle int16 LE (deg x 10)
void Logger::CANLogDesired() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogDesired_CANID;
  outCAN->length = 6;
  outCAN->data.int16[0] = getD_speed_cmPs(*myTrike);
  outCAN->data.uint8[2] = (uint8_t)getD_brakes(*myTrike);
  outCAN->data.uint8[3] = (uint8_t)getAutoMode(*myTrike);
  outCAN->data.int16[2] = getD_Angle(*myTrike);
  myTrike->sendCan();
}
/******************************************************
Report sensors and actuators for steering
*******************************************************/ 
// Log the steering method actually compiled in, and the pins THAT method drives.
// Rturn/Lturn are only meaningful for the two-wire scheme; logging them under
// motor-control or servo just records two pins nothing ever writes.
void Logger::HdrSteer() {
#if   (STEER_METHOD == STR_MOTOR_CONTROL)
  out->print(",steer_method,current_angle,steer_on,steer_dir,steer_speed");
#elif (STEER_METHOD == SRT_HBRIDGE)
  out->print(",steer_method,current_angle,Rturn,Lturn");
#else
  out->print(",steer_method,current_angle,steer_pulse_us");
#endif
}

void Logger::TxSteer() {
  out->print(",");
  out->print(STEER_METHOD_NAME);
  out->print(",");
  out->print(getAngle(*myTrike));
#if   (STEER_METHOD == STR_MOTOR_CONTROL)
  out->print(",");
  out->print(digitalRead(STEER_ON_PIN));
  out->print(",");
  out->print(digitalRead(STEER_DIR_PIN));
  out->print(",");
  out->print(digitalRead(STEER_SPEED_PIN));
#elif (STEER_METHOD == SRT_HBRIDGE)
  out->print(",");
  out->print(digitalRead(RIGHT_TURN_PIN));
  out->print(",");
  out->print(digitalRead(LEFT_TURN_PIN));
#else
  out->print(",");
  out->print(0);   // servo pulse width — STR_PWM output not implemented yet
#endif
}

/******************************************************
Operator joystick and switches (alternative to the RC controller)
*******************************************************/
void Logger::HdrOp() {
  out->print(",op_steer,op_throttle,op_fwd,op_mode,op_discnt,op_estop");
}

void Logger::TxOp() {
  out->print(",");
  out->print(analogRead(OP_STEER));      // A7 joystick X
  out->print(",");
  out->print(analogRead(OP_THROTTLE));   // A8 joystick Y
  out->print(",");
  out->print(digitalRead(OP_FWD_PIN));
  out->print(",");
  out->print(digitalRead(OP_MODE_PIN));
  out->print(",");
  out->print(digitalRead(OP_DISCNT_PIN));
  out->print(",");
  out->print(digitalRead(OP_ESTOP));
}

/******************************************************
What Nav sent us over CAN (0x350 commands + 0x100 status).
Already emitted as CAN frame 0x704; this puts it in the CSV too, so a log
shows what was commanded even when no CAN listener was recording.
*******************************************************/
void Logger::HdrCAN() {
  out->print(",nav_speed,nav_brake,nav_mode,nav_angle,nav_status");
}

void Logger::TxCAN() {
  out->print(",");
  out->print(getNavSpeed(*myTrike));
  out->print(",");
  out->print(getNavBrake(*myTrike));
  out->print(",");
  out->print(getNavMode(*myTrike));
  out->print(",");
  out->print(getNavAngle(*myTrike));
  out->print(",");
  out->print(getNavStatus(*myTrike));
}
// 0x708 LogSteer — 6 bytes: actual steer angle + L/R column sensor analog readings.
//   bytes 0-1: actual_angle int16 LE (deg x 10)
//   bytes 2-3: R column sensor int16 (raw analogRead, 0-1023)
//   bytes 4-5: L column sensor int16 (raw analogRead, 0-1023)
void Logger::CANLogSteer() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogSteer_CANID;
  outCAN->length = 6;
  outCAN->data.int16[0] = getAngle(*myTrike);
  outCAN->data.int16[1] = (int16_t)analogRead(R_SENSE_PIN);
  outCAN->data.int16[2] = (int16_t)analogRead(L_SENSE_PIN);
  myTrike->sendCan();
}
/******************************************************
Report sensors and actuators for propulsion
*******************************************************/ 
void Logger::HdrThrottle() {
  // throttle_pwm, not "current_speed": SpeedController::update() returns
  // currentThrottle (the value written to DAC0), and Vehicle assigns it into
  // currentSpeed_cmPs. measured_speed is the real wheel speed from the
  // odometer, which was previously not logged at all - so there was no way to
  // see what the throttle PID was actually reacting to.
  out->print(",throttle_pwm,measured_speed,driveMode");
}
void Logger::TxThrottle() {
  out->print(",");
  out->print(getSpeed(*myTrike));            // throttle_pwm (DAC value)
  out->print(",");
  out->print(getMeasuredSpeed(*myTrike));    // measured_speed, cm/s from odometer
  out->print(",");
  out->print(getDriveMode(*myTrike));
}
// 0x706 LogThrottle — 4 bytes: actual_speed + throttle_pwm + drive_mode.
//   bytes 0-1: actual_speed int16 LE (cm/s)
//   byte  2:   throttle_pwm uint8 (0-255; not yet surfaced from SpeedController, emits 0)
//   byte  3:   drive_mode uint8 (FORWARD_MODE/REVERSE_MODE)
void Logger::CANLogThrottle() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogThrottle_CANID;
  outCAN->length = 4;
  outCAN->data.int16[0] = getSpeed(*myTrike);
  outCAN->data.uint8[2] = 0;  // TODO: expose throttle PWM from SpeedController
  outCAN->data.uint8[3] = (uint8_t)getDriveMode(*myTrike);
  myTrike->sendCan();
}
/******************************************************
Report state of the brakes
*******************************************************/ 
void Logger::HdrBrakes() {
  out->print(",BrakeOn,BrakeVolt");
}
void Logger::TxBrakes()  {
  // Report the logical state, not the raw pin. RELAYInversion makes ON_BR LOW,
  // so a bare digitalRead printed 1 with the brakes released and 0 with them
  // engaged - the columns read backwards from their names. BrakeVolt is 1 when
  // the 24V pull-in relay is active, 0 while holding on 12V.
  out->print(",");
  out->print(digitalRead(BRAKE_ON_PIN)   == ON_BR);
  out->print(",");
  out->print(digitalRead(BRAKE_VOLT_PIN) == ON_BR);
}
// 0x707 LogBrakes — 2 bytes: brake_on (uint8) + brake_voltage (uint8).
void Logger::CANLogBrakes() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogBrakes_CANID;
  outCAN->length = 2;
  // Same inversion as TxBrakes(): send the logical state, not the raw pin.
  outCAN->data.uint8[0] = (uint8_t)(digitalRead(BRAKE_ON_PIN)   == ON_BR);
  outCAN->data.uint8[1] = (uint8_t)(digitalRead(BRAKE_VOLT_PIN) == ON_BR);
  myTrike->sendCan();
}
/******************************************************
Terminate the line and indicate the part of loop time used
*******************************************************/ 
void Logger::HdrEndLine() {
  out->println(",Utilization");
}
void Logger::EndLine(uint32_t delayTime) {
  int PerCentBusy = ((LOOP_TIME_MS - delayTime) * 100) / LOOP_TIME_MS;
  CANLogFinalize(PerCentBusy);   // 0x70A — always emit so the receiver knows the row is complete
  // Match the guard in update(): only write when the USB TX buffer has room,
  // and never flush() — flush() blocks until a host drains the buffer, which
  // freezes the loop when nothing is reading.
  if (logMethod != 2 && sinkReady(serialLOG, 16)) {
    out->print(",");
    out->println(PerCentBusy);
    // Commit the row to the card. Guarded on logMethod 0 so we never flush an
    // invalid File after the SD-to-serial fallback in initialize().
    if (logMethod == 0) flushSink(serialLOG);
  }
}

// 0x703 LogOp — 8 bytes: operator panel state.
//   bytes 0-1: joystick X (int16, raw analogRead of OP_STEER, 0-1023)
//   bytes 2-3: joystick Y (int16, raw analogRead of OP_THROTTLE, 0-1023)
//   byte  4:   switch bitfield (0x10 fwd/rev, 0x20 auto/manual, 0x40 disconnect, 0x80 estop)
//   byte  5:   commanded brake (uint8)
//   bytes 6-7: commanded steer angle (int16 LE, deg x 10)
void Logger::CANLogOp() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogOp_CANID;
  outCAN->length = 8;
  outCAN->data.int16[0] = (int16_t)analogRead(OP_STEER);
  outCAN->data.int16[1] = (int16_t)analogRead(OP_THROTTLE);
  uint8_t sw = 0;
  if (digitalRead(OP_FWD_PIN))    sw |= 0x10;
  if (digitalRead(OP_MODE_PIN))   sw |= 0x20;
  if (digitalRead(OP_DISCNT_PIN)) sw |= 0x40;
  if (digitalRead(OP_ESTOP))      sw |= 0x80;
  outCAN->data.uint8[4] = sw;
  outCAN->data.uint8[5] = (uint8_t)getD_brakes(*myTrike);
  outCAN->data.int16[3] = getD_Angle(*myTrike);
  myTrike->sendCan();
}

// 0x704 LogAuto — 7 bytes: what DBW received from Nav (mirrors 0x350 + 0x100 status byte).
//   bytes 0-1: nav_speed int16 LE (cm/s)
//   byte  2:   nav_brake uint8 (0/1/2)
//   byte  3:   nav_mode  uint8 (0-7, Nav-requested state — DBW arbitrates)
//   bytes 4-5: nav_angle int16 LE (deg x 10)
//   byte  6:   nav_status uint8 (0x100 byte 0: 0x80 estop, 0x40 auto, 0x04 reverse, ...)
void Logger::CANLogAuto() {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogAuto_CANID;
  outCAN->length = 7;
  outCAN->data.int16[0] = getNavSpeed(*myTrike);
  outCAN->data.uint8[2] = getNavBrake(*myTrike);
  outCAN->data.uint8[3] = getNavMode(*myTrike);
  outCAN->data.int16[2] = getNavAngle(*myTrike);
  outCAN->data.uint8[6] = getNavStatus(*myTrike);
  myTrike->sendCan();
}

// 0x70A LogFinalize — 1 byte: CPU utilization (%) for the loop tick.
void Logger::CANLogFinalize(int PerCentBusy) {
  CAN_FRAME* outCAN = getCAN(*myTrike);
  outCAN->id = LogFinalize_CANID;
  outCAN->length = 1;
  outCAN->data.uint8[0] = (uint8_t)PerCentBusy;
  myTrike->sendCan();
}