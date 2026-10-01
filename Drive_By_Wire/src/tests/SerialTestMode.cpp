#include "SerialTestMode.h"
#include "FirmwareMode.h"

#if DBW_SERIAL_TEST_MODE

#include <Arduino.h>
#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "VehicleDiagnostics.h"

namespace SerialTestMode {
namespace {

char inputLine[48];
size_t inputLength = 0;
bool inputOverflow = false;
char outputQueue[2048];
size_t outputHead = 0, outputTail = 0;
}

void report(const char *format, ...) {
  char message[160];
  va_list arguments;
  va_start(arguments, format);
  vsnprintf(message, sizeof(message), format, arguments);
  va_end(arguments);
  size_t length = strlen(message);
  size_t freeSpace = (outputTail + sizeof(outputQueue) - outputHead - 1) % sizeof(outputQueue);
  if (freeSpace < length + 2) return;
  for (size_t index = 0; index < length; ++index) {
    outputQueue[outputHead] = message[index];
    outputHead = (outputHead + 1) % sizeof(outputQueue);
  }
  outputQueue[outputHead] = '\r';
  outputHead = (outputHead + 1) % sizeof(outputQueue);
  outputQueue[outputHead] = '\n';
  outputHead = (outputHead + 1) % sizeof(outputQueue);
}

void flushReports() {
  int capacity = Serial.availableForWrite();
  while (capacity-- > 0 && outputTail != outputHead) {
    Serial.write(outputQueue[outputTail]);
    outputTail = (outputTail + 1) % sizeof(outputQueue);
  }
}

namespace {
void executeLine(VehicleDiagnostics &diagnostics, char *line) {
  while (isspace((unsigned char)*line)) ++line;
  if (!*line) return;
  bool letter = isalpha((unsigned char)*line);
  char command = letter ? (char)toupper((unsigned char)*line++) : 0;
  if (letter && !strchr("LBRVSTCP", command)) {
    report("ERR expected L B R V S T E C P");
    return;
  }
  while (isspace((unsigned char)*line)) ++line;
  bool hasValue = *line != 0;
  long value = 0;
  if (hasValue) {
    char *end;
    errno = 0;
    value = strtol(line, &end, 10);
    if (end == line || errno == ERANGE) {
      report("ERR invalid integer");
      return;
    }
    while (isspace((unsigned char)*end)) ++end;
    if (*end) {
      report("ERR range: V 5..255; S -1000..1000 ms; T 0..255; P 1000..1850 us");
      return;
    }
  } else if (!letter) {
    return;
  }
  diagnostics.testCommand(command, value, hasValue);
}
}

void readCommands(VehicleDiagnostics &diagnostics) {
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    char incoming = (char)Serial.read();
    if (incoming == 'E' || incoming == 'e') {
      inputLength = 0;
      inputOverflow = false;
      diagnostics.testCommand('E', 0, false);
    } else if (incoming == '\r' || incoming == '\n') {
      if (inputOverflow) report("ERR line too long; discarded");
      else {
        inputLine[inputLength] = 0;
        executeLine(diagnostics, inputLine);
      }
      inputLength = 0;
      inputOverflow = false;
    } else if (!inputOverflow) {
      if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = incoming;
      else inputOverflow = true;
    }
  }
}

void begin() {
  Serial.begin(115200);
  inputLength = 0;
  inputOverflow = false;
  outputHead = outputTail = 0;
}

}

#endif