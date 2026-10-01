#pragma once

class VehicleDiagnostics;

namespace SerialTestMode {

void begin();
void readCommands(VehicleDiagnostics &diagnostics);
void report(const char *format, ...);
void flushReports();

}