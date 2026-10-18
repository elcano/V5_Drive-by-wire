#pragma once

#include "Arduino.h"

#define CAN_BPS_500K 500000

struct CAN_FRAME {
  uint32_t id = 0;
  uint8_t length = 0;
  union {
    int16_t int16[4];
    uint8_t uint8[8];
  } data = {};
};

class MockCan {
public:
  unsigned int beginCalls = 0, watchCalls = 0, availableCalls = 0, readCalls = 0;
  std::deque<CAN_FRAME> incoming;
  std::vector<CAN_FRAME> sent;
  uint32_t begin(uint32_t baud) { ++beginCalls; return baud; }
  void watchFor() { ++watchCalls; }
  bool sendFrame(const CAN_FRAME &frame) { sent.push_back(frame); return true; }
  int available() { ++availableCalls; return (int)incoming.size(); }
  void read(CAN_FRAME &frame) {
    ++readCalls;
    frame = incoming.front();
    incoming.pop_front();
  }
};

extern MockCan Can0;