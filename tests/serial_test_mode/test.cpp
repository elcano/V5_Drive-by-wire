#include "Arduino.h"
#include <assert.h>

uint32_t clockMs = 0, clockUs = 0;
int digitalPins[128] = {}, analogPins[128] = {};
int sensorValue = 512;
unsigned int delayCalls = 0;
std::vector<PinWrite> pinWrites;
void (*interruptsByPin[128])() = {};
int interruptModes[128] = {};
MockSerial Serial;
MockSerial SerialUSB;
bool servoAttached = false;
int servoWidth = 0;

#include "../../Drive_By_Wire/Drive_By_Wire.ino"
#include "../../Drive_By_Wire/SpeedController.h"
#include "../../Drive_By_Wire/SteeringController.h"
#include "../../Drive_By_Wire/Can_Protocol.h"

MockCan Can0;

void expectDiagnosticIsolation() {
  size_t previousWrites = pinWrites.size();
  myTrike->receiveCan();
  myTrike->updateRC();
  assert(!myTrike->sendCan());
  assert(pinWrites.size() == previousWrites);
  assert(Can0.beginCalls == 0 && Can0.watchCalls == 0);
  assert(Can0.availableCalls == 0 && Can0.readCalls == 0 && Can0.sent.empty());
  assert(Can0.incoming.size() == 1);
}

void send(const std::string &text) {
  Serial.output.clear();
  for (char value : text) Serial.input.push_back(value);
  while (Serial.available()) loop();
  for (int count = 0; count < 16; ++count) loop();
}

void advance(uint32_t duration) {
  clockMs += duration;
  clockUs += duration * 1000;
  loop();
}

void expectReport(const char *text) {
  assert(Serial.output.find(text) != std::string::npos);
}

void expectStopped() {
  assert(analogPins[STEER_SPEED_PIN] == 0);
  assert(digitalPins[STEER_ON_PIN] == ST_OFF);
  assert(analogPins[DAC0] == 0);
  assert(!servoAttached);
}

int main() {
  setup();
  CAN_FRAME navRequest;
  navRequest.id = HiDrive_CANID;
  navRequest.length = 6;
  navRequest.data.int16[0] = 500;
  navRequest.data.int16[2] = 200;
  Can0.incoming.push_back(navRequest);
  expectDiagnosticIsolation();
  assert(myTrike->getMeasuredWheelAngle_DegX10() == 0);
  assert(digitalPins[OP_MODE_PIN] == LOW);
  assert(interruptModes[STEERING_CH1_PIN] == CHANGE);
  for (const auto &write : pinWrites) {
    if (write.analog) assert(write.value == 0);
    if (write.pin == STEER_ON_PIN) assert(write.value == ST_OFF);
    if (write.pin == BRAKE_ON_PIN || write.pin == BRAKE_VOLT_PIN) assert(write.value == ON_BR);
  }
  assert(delayCalls == 0 && !interruptsByPin[IRPT_WHEEL]);
  send("");
  expectReport("ESTOP:");
  expectStopped();
  assert(digitalPins[BRAKE_ON_PIN] == LOW);
  advance(799);
  assert(digitalPins[BRAKE_VOLT_PIN] == LOW);
  advance(1);
  assert(digitalPins[BRAKE_VOLT_PIN] == HIGH);

  send("v 100\r\ns\n-200\n");
  assert(analogPins[STEER_SPEED_PIN] == 100);
  assert(digitalPins[STEER_ON_PIN] == ST_ON);
  assert(digitalPins[STEER_DIR_PIN] == ST_LEFT);
  advance(199);
  assert(analogPins[STEER_SPEED_PIN] == 100);
  advance(1);
  expectStopped();
  expectReport("STEER STOP; angle=512");
  expectReport("calibrated=");
  send("S 200\n");
  expectDiagnosticIsolation();
  assert(digitalPins[STEER_DIR_PIN] == ST_RIGHT);
  send("e");
  expectReport("ESTOP:");
  expectStopped();
  send("200\n");
  expectReport("ERR expected");
  expectStopped();
  send("Z\n");
  expectReport("ERR expected");
  send("S 1001\nV 4\nT -1\nP 999\nS 999999999999999999999999999999\n");
  expectReport("ERR range:");
  expectReport("ERR invalid integer");
  expectStopped();
  send("S 10garbage\n");
  expectStopped();
  send("S " + std::string(60, '1') + "\n");
  expectReport("ERR line too long");
  send("200\n");
  expectReport("ERR expected");
  expectStopped();

  send("T 75\n");
  expectReport("ERR brakes applied");
  expectStopped();
  send("r\nt\n75\n");
  expectDiagnosticIsolation();
  assert(analogPins[DAC0] == 75 && digitalPins[BRAKE_ON_PIN] == HIGH);
  advance(4999);
  assert(analogPins[DAC0] == 75);
  advance(1);
  expectStopped();
  expectReport("OUTPUT TIMEOUT");
  assert(digitalPins[BRAKE_ON_PIN] == LOW && digitalPins[BRAKE_VOLT_PIN] == LOW);
  advance(400);
  send("B\n");
  advance(400);
  assert(digitalPins[BRAKE_ON_PIN] == LOW && digitalPins[BRAKE_VOLT_PIN] == HIGH);

  send("S 500\n");
  digitalPins[OP_ESTOP] = LOW;
  loop();
  expectStopped();
  send("R\nS 100\n");
  expectReport("ERR ESTOP latched");
  assert(digitalPins[BRAKE_ON_PIN] == LOW);
  expectStopped();
  digitalPins[OP_ESTOP] = HIGH;
  loop();
  send("100\n");
  expectReport("ERR expected");
  expectStopped();

  send("P 1500\n");
  assert(servoAttached && servoWidth == 1500);
  send("E");
  expectStopped();
  send("P 1500\n");
  advance(5000);
  expectStopped();
  expectReport("OUTPUT TIMEOUT");

  send("L\n");
  for (uint8_t color = 0; color < 8; ++color) {
    assert(digitalPins[RED_LED_PIN] == ((color & 1) ? HIGH : LOW));
    assert(digitalPins[GREEN_LED_PIN] == ((color & 2) ? HIGH : LOW));
    assert(digitalPins[BLUE_LED_PIN] == ((color & 4) ? HIGH : LOW));
    advance(1000);
  }
  expectReport("LED cycle complete");

  const uint8_t rcPins[] = {STEERING_CH1_PIN, THROTTLE_BR_CH2_PIN, CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN};
  for (uint8_t channel = 0; channel < 6; ++channel) {
    int pin = rcPins[channel];
    digitalPins[pin] = HIGH;
    interruptsByPin[pin]();
    clockUs += 1000 + channel * 100;
    digitalPins[pin] = LOW;
    interruptsByPin[pin]();
  }
  send("C\n");
  expectReport("RC us: 1000 1100 1200 1300 1400 1500");
  advance(250);
  expectReport("RC us: 0 0 0 0 0 0");

  clockMs = UINT32_MAX - 100;
  send("S 200\n");
  advance(199);
  assert(analogPins[STEER_SPEED_PIN] == 100);
  advance(1);
  expectStopped();

  clockMs = UINT32_MAX - 400;
  send("R\nB\n");
  advance(399);
  send("E");
  assert(digitalPins[BRAKE_VOLT_PIN] == ON_BR);
  advance(400);
  assert(digitalPins[BRAKE_VOLT_PIN] == ON_BR);
  advance(1);
  assert(digitalPins[BRAKE_ON_PIN] == ON_BR && digitalPins[BRAKE_VOLT_PIN] == OFF_BR);
  assert(delayCalls == 0);
  expectDiagnosticIsolation();

  static SpeedController normalSpeed;
  assert(digitalPins[BRAKE_ON_PIN] == OFF_BR && interruptsByPin[IRPT_WHEEL]);
  normalSpeed.Stop();
  assert(!normalSpeed.testThrottle(75));
  clockMs += 801;
  normalSpeed.Stop();
  assert(normalSpeed.brakesApplied() && !normalSpeed.brakeBoostActive());
  normalSpeed.ReleaseBrakes();
  assert(normalSpeed.testThrottle(75) && analogPins[DAC0] == 75);
  normalSpeed.update(0, FORWARD_MODE);
  assert(analogPins[DAC0] == 0 && normalSpeed.brakesApplied());
  clockMs += 100;
  normalSpeed.update(MAX_SPEED_cmPs, FORWARD_MODE);
  assert(!normalSpeed.brakesApplied());
  assert(analogPins[DAC0] >= MIN_THROTTLE && analogPins[DAC0] <= MAX_THROTTLE);
  normalSpeed.Stop();

  SteeringController normalSteering;
  sensorValue = Left_Straight_Read;
  assert(normalSteering.readLeftSensorRaw() == sensorValue);
  assert(normalSteering.testAngleLeft() == 0);
  assert(!normalSteering.testPulse(MIN_LEFT_US - 1));
  normalSteering.testMotor(true, 50);
  assert(digitalPins[STEER_DIR_PIN] == ST_LEFT && analogPins[STEER_SPEED_PIN] == 50);
  normalSteering.stopTest();
  assert(analogPins[STEER_SPEED_PIN] == 0);
  normalSteering.update(200, 100);
  assert(analogPins[STEER_SPEED_PIN] == 255 && digitalPins[STEER_DIR_PIN] == HIGH);
  normalSteering.update(100, 100);
  assert(digitalPins[STEER_ON_PIN] == ST_OFF);
  Vehicle normalVehicle;
  assert(Can0.beginCalls == 1 && Can0.watchCalls == 1);
  assert(interruptModes[STEERING_CH1_PIN] == RISING);
  assert(digitalPins[OP_MODE_PIN] == HIGH);
  assert(digitalPins[BRAKE_ON_PIN] == OFF_BR);
  size_t previousWrites = pinWrites.size();
  normalVehicle.test();
  normalVehicle.testCommand('S', 200, true);
  assert(pinWrites.size() == previousWrites);
  normalVehicle.updateRC();
  normalVehicle.receiveCan();
  assert(Can0.readCalls == 1 && Can0.incoming.empty());
  normalVehicle.update();
  assert(Can0.sent.size() == 2);
  assert(Can0.sent[0].id == Actual_CANID && Can0.sent[1].id == LowStatus_CANID);
  puts("PASS: Vehicle-owned diagnostics, CAN/RC isolation, safe startup, serial commands, sensor conversion, estops, timers/rollover, normal Vehicle startup/control");
}