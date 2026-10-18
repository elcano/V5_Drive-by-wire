#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <deque>
#include <string>
#include <sstream>
#include <vector>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define RISING 4
#define FALLING 5
#define A6 60
#define A7 61
#define A8 62
#define A10 64
#define A11 65
#define DAC0 66
#define PI 3.14159265358979323846

extern uint32_t clockMs;
extern uint32_t clockUs;
extern int digitalPins[128];
extern int analogPins[128];
extern int sensorValue;
extern unsigned int delayCalls;
extern void (*interruptsByPin[128])();
extern int interruptModes[128];

struct PinWrite {
  bool analog;
  int pin;
  int value;
};
extern std::vector<PinWrite> pinWrites;

class String : public std::string {
public:
  explicit String(float value) : std::string(std::to_string(value)) {}
};

struct MockSerial {
  std::deque<char> input;
  std::string output;
  void begin(unsigned long) {}
  int available() { return (int)input.size(); }
  int availableForWrite() { return 256; }
  int read() {
    char value = input.front();
    input.pop_front();
    return value;
  }
  void write(char value) { output += value; }
  explicit operator bool() const { return false; }
  template <typename Value> void print(const Value &value) {
    std::ostringstream stream;
    stream << value;
    output += stream.str();
  }
  template <typename Value> void println(const Value &value) {
    print(value);
    output += '\n';
  }
};

extern MockSerial Serial;
extern MockSerial SerialUSB;
inline uint32_t millis() { return clockMs; }
inline uint32_t micros() { return clockUs; }
inline void pinMode(int pin, int mode) { if (mode == INPUT_PULLUP) digitalPins[pin] = HIGH; }
inline void digitalWrite(int pin, int value) {
  pinWrites.push_back({false, pin, value});
  digitalPins[pin] = value;
}
inline int digitalRead(int pin) { return digitalPins[pin]; }
inline void analogWrite(int pin, int value) {
  pinWrites.push_back({true, pin, value});
  analogPins[pin] = value;
}
inline int analogRead(int) { return sensorValue; }
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int pin, void (*handler)(), int mode) {
  interruptsByPin[pin] = handler;
  interruptModes[pin] = mode;
}
inline void noInterrupts() {}
inline void interrupts() {}
inline void delay(uint32_t duration) {
  ++delayCalls;
  clockMs += duration;
  clockUs += duration * 1000;
}