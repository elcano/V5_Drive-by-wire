@echo off
setlocal
if not defined VSCMD_VER (
  call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
  if errorlevel 1 exit /b 1
)
set "TEST_OUT=%TEMP%\dbw-serial-tests-%RANDOM%"
mkdir "%TEST_OUT%"
cl /nologo /EHsc /std:c++14 /DDBW_SERIAL_TEST_MODE=1 /DSTR_PWM=0 /DSTR_HBRIDGE=1 /DSTR_MOTOR_CONTROL=2 /DSTEER_METHOD=2 /I"%~dp0." "%~dp0test.cpp" "%~dp0..\..\Drive_By_Wire\src\tests\SerialTestMode.cpp" "%~dp0..\..\Drive_By_Wire\src\tests\VehicleDiagnostics.cpp" "%~dp0..\..\Drive_By_Wire\src\tests\VehicleTestMode.cpp" "%~dp0..\..\Drive_By_Wire\Vehicle.cpp" "%~dp0..\..\Drive_By_Wire\RC_Controller.cpp" "%~dp0..\..\Drive_By_Wire\SpeedController.cpp" "%~dp0..\..\Drive_By_Wire\src\tests\SpeedControllerDiagnostics.cpp" "%~dp0..\..\Drive_By_Wire\SteeringController.cpp" "%~dp0..\..\Drive_By_Wire\src\tests\SteeringControllerDiagnostics.cpp" "%~dp0..\..\Drive_By_Wire\PID.cpp" /Fo"%TEST_OUT%\\" /Fe"%TEST_OUT%\test.exe"
if errorlevel 1 exit /b 1
"%TEST_OUT%\test.exe"
exit /b %errorlevel%