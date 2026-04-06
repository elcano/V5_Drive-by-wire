/*
 * SetTime.ino
 *
 * Utility sketch for setting the DS1307 real-time clock (RTC) on the
 * Drive-By-Wire board. Upload this sketch, open Serial Monitor at 9600
 * baud, and type the current time in the format shown below. The RTC
 * will retain the time after the sketch is replaced with the main firmware.
 *
 * Usage:
 *   Enter time as: YYYY-MM-DD HH:MM:SS
 *   Example:       2026-04-02 18:45:30
 *
 * Hardware:
 *   - DS1307 RTC connected via I2C (SDA/SCL)
 *   - Arduino Due (or compatible board with Wire support)
 *
 * Dependencies:
 *   - DS1307RTC  (https://github.com/PaulStoffregen/DS1307RTC)
 *   - TimeLib    (https://github.com/PaulStoffregen/Time)
 *   - Wire       (built-in)
 */

#include <Wire.h>
#include <DS1307RTC.h>
#include <TimeLib.h>

String inputLine = "";

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for Serial on boards that need it
  }

  Wire.begin();

  Serial.println("SetTime sketch started.");
  Serial.println("Enter time in this format:");
  Serial.println("YYYY-MM-DD HH:MM:SS");
  Serial.println("Example: 2026-04-02 18:45:30");
  Serial.println();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') {
      // ignore carriage return
      continue;
    }

    if (c == '\n') {
      if (inputLine.length() > 0) {
        processInput(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }

  printRTCTime();
  delay(1000);
}

void processInput(String s) {
  int year, month, day, hour, minute, second;

  int matched = sscanf(
    s.c_str(),
    "%d-%d-%d %d:%d:%d",
    &year, &month, &day,
    &hour, &minute, &second
  );

  if (matched != 6) {
    Serial.println("Invalid format.");
    Serial.println("Use: YYYY-MM-DD HH:MM:SS");
    return;
  }

  tmElements_t tm;
  tm.Year = CalendarYrToTm(year);
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = minute;
  tm.Second = second;

  bool ok = RTC.write(tm);

  if (ok) {
    Serial.println("RTC updated successfully.");
  } else {
    Serial.println("RTC write failed.");
  }
}

void printRTCTime() {
  tmElements_t tm;

  if (RTC.read(tm)) {
    char buf[25];
    sprintf(
      buf,
      "%02d:%02d:%02d  %02d/%02d/%04d",
      tm.Hour,
      tm.Minute,
      tm.Second,
      tm.Month,
      tm.Day,
      tmYearToCalendar(tm.Year)
    );
    Serial.println(buf);
  } else {
    Serial.println("RTC read error!");
  }
}
