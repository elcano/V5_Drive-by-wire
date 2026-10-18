#pragma once

extern bool servoAttached;
extern int servoWidth;

class Servo {
public:
  void attach(int, int, int) { servoAttached = true; }
  void detach() { servoAttached = false; }
  bool attached() { return servoAttached; }
  void writeMicroseconds(int value) { servoWidth = value; }
};