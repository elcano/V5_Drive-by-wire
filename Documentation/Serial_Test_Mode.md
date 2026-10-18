# DBW Serial Test Mode

Serial diagnostics are part of the original Drive_By_Wire sketch. The submitted
default is normal DBW operation (`DBW_SERIAL_TEST_MODE=0`); serial diagnostics
must be enabled explicitly for bench testing. The original steering and speed test routines
remain available; serial mode uses interactive methods on those same controller
classes instead of calling the complete blocking test sequences. No separate
Arduino sketch is needed.

`Drive_By_Wire/Vehicle.cpp` retains its original constructor and driving methods,
with only test-mode-guarded dispatch/isolation checks added. Diagnostic constructors
and helpers live in `Drive_By_Wire/src/tests/`, compiled only when
`DBW_SERIAL_TEST_MODE=1`; the setting must apply to all translation units.

Arduino compiles the sketch's `src` directory recursively, so these files are
included automatically when building the original sketch in the IDE. Moving them
to the repository-level `tests/` directory would require extra build integration.
The host mocks and PowerShell runners remain in `tests/serial_test_mode/`; they
must not be compiled into the Due firmware.

| File | Responsibility |
| --- | --- |
| Vehicle.cpp / Vehicle.h | Original Vehicle implementation plus guarded test declarations and RC/CAN isolation checks |
| src/tests/FirmwareMode.h | Normal versus diagnostic build selection, default 0 |
| src/tests/VehicleTestMode.cpp | Diagnostic Vehicle constructor and forwarding test methods |
| src/tests/VehicleDiagnostics.cpp / src/tests/VehicleDiagnostics.h | Command actions, limits, timers, E-stop state, LEDs, and raw RC capture |
| src/tests/SerialTestMode.cpp / src/tests/SerialTestMode.h | Parse serial input and queue responses; no controller construction or direct actuator operations |
| SpeedController.cpp / SteeringController.cpp | Unchanged upstream implementations, including constructors, normal control, and original test() sequences |
| src/tests/SpeedControllerDiagnostics.cpp / src/tests/SteeringControllerDiagnostics.cpp | Additional test-only constructor overloads and manual actuator helpers, declared behind the mode flag in controller headers |
| DBW_Pins.h / Settings.h | Shared hardware pin definitions, polarity, limits, and calibration |

## Vehicle-owned tests

The original sketch constructs `Vehicle(Vehicle::Startup::Diagnostic)` in test
mode and calls `myTrike->update()` from loop(). Vehicle routes that update to
`Vehicle::test()` instead of normal driving control. Test mode constructs actual
SteeringController and SpeedController instances with `Startup::Diagnostic`,
owned by Vehicle just like its normal controllers. VehicleDiagnostics receives
references to those same objects; it does not construct duplicate controllers.
Steering starts with PWM zero and skips automatic
centering. Speed starts with throttle code zero and brakes applied; it does not
briefly release brakes or attach the wheel-speed interrupt. Normal construction
keeps its previous behavior, including the existing automatic startup tests.
Diagnostic Vehicle startup skips the normal RC_Controller, CAN initialization,
and Logger. Its updateRC(), receiveCan(), and sendCan() entry points are guarded
so they cannot override tests even if called accidentally. Raw RC pulse capture
is initialized separately for C; it cannot command motion.

```text
setup() -> Vehicle(Startup::Diagnostic) -> controllers -> VehicleDiagnostics::begin()
loop() -> Vehicle::update() -> Vehicle::test() -> VehicleDiagnostics::update()
    -> serviceTestOutputs()
    -> SerialTestMode::readCommands(diagnostics) -> VehicleDiagnostics::testCommand(...)
    -> serviceTestOutputs()
    -> SerialTestMode::flushReports()
```

For example, S -200 is parsed by the serial module, validated and dispatched by
VehicleDiagnostics::testCommand(), and implemented through the existing steering
controller's testMotor(). The diagnostic module times movement, stops it, and
reports feedback. B uses the diagnostic testStop() wrapper to call the existing
Stop() when initially engaging brakes; R calls the unchanged ReleaseBrakes().
Vehicle::testCommand() remains a guarded forwarding entry point;
normal Vehicle instances reject diagnostic commands and updates.

| Serial action | DBW controller method |
| --- | --- |
| B or E braking | Test-only SpeedController::testStop(), calling the original Stop() on initial engagement |
| R | SpeedController::ReleaseBrakes() |
| Brake holding-voltage timer | Test-only SpeedController::serviceBrakes(); normal update()/Stop() timing remains unchanged |
| T or throttle zero on stop | Test-only SpeedController::testThrottle(), writing the same DAC0 output without refactoring the normal throttle path |
| S movement | Test-only SteeringController::testMotor(), using the existing pin definitions and original test's direction convention |
| Steering stop | SteeringController::stopTest() |
| P | SteeringController::testPulse(), using the controller's Servo instance |
| Steering feedback | readLeftSensorRaw() and testAngleLeft(), which calls the original private computeAngleLeft() |

The serial module retains only parsing and queued reporting. VehicleDiagnostics
owns duration limits and E-stop handling; it does not call normal controller update/PID
paths during diagnostics.
Unlike the old steering test, S stops by elapsed time, not a sensor threshold.

## Minimal production impact

The earlier shared-helper refactoring has been removed. SpeedController.cpp and
SteeringController.cpp match upstream: no changes to normal throttle writes,
steering direction output, sensor conversion, constructor bodies, or original
automatic test sequences. Vehicle's original constructor body is also retained.

The original no-argument constructors remain the default. Test-only constructor
overloads are declared and defined only with diagnostics enabled. They initialize
the controllers without invoking the normal constructors, avoiding initial brake
release and automatic centering. Diagnostic Vehicle state and the update/RC/CAN
guards also compile out entirely in normal mode; diagnostic constructors cannot
accidentally be selected in a mode-0 build.

The diagnostic brake wrapper preserves the existing Stop() method but calls it
only when brakes are released. Repeated stops zero throttle and service a separate
rollover-safe test timer, without restarting the boost. This avoids using the
original absolute-deadline check across timer rollover during a diagnostic test.
Normal-mode brake timing and the original uninitialized PIDThrottle field are not
fixed in this feature. Diagnostic startup initializes its own PIDThrottle to zero;
the original normal-mode issue is deliberately deferred to a separate review.

For code review, start with the guarded additions in Vehicle.cpp and the three
headers, then inspect src/tests/VehicleTestMode.cpp and the two controller diagnostic files.
The sketch retains only compile-time selection of normal versus diagnostic startup
and loop. This limits production changes but does not replace electrical or trike
validation.

## Select a mode

Set `DBW_SERIAL_TEST_MODE` in `Drive_By_Wire/src/tests/FirmwareMode.h`:

- `0` (default): normal DBW firmware, including its existing startup tests.
- `1`: diagnostic Vehicle construction. No Logger, CAN arbitration, controller PID
    updates, or normal RC control. RC inputs are only measured.

Rebuild and upload the original `Drive_By_Wire/Drive_By_Wire.ino`. Switching
modes requires a rebuild and upload; no serial command starts normal driving.
Keep the submitted header set to 0. Changing the source setting does not change
the firmware already running on a Due until another build is uploaded.
The CLI can instead override the setting with
`--build-property compiler.cpp.extra_flags=-DDBW_SERIAL_TEST_MODE=1`.

The local Settings.h must contain the steering definitions from SettingsTemplate.h
for the original controller sources to compile. Do not replace vehicle calibration
with the entire template. This machine's local Settings.h now supplies those
definitions, selecting STR_MOTOR_CONTROL as in the previously validated builds.
Existing calibration values were not changed. Other checkouts may still need
these definitions; this CLI command supplies them explicitly:

```powershell
arduino-cli compile --fqbn arduino:sam:arduino_due_x --libraries . --build-property "compiler.cpp.extra_flags=-DDBW_SERIAL_TEST_MODE=1 -DSTR_PWM=0 -DSTR_HBRIDGE=1 -DSTR_MOTOR_CONTROL=2 -DSTEER_METHOD=2" ./Drive_By_Wire
```

Use `DBW_SERIAL_TEST_MODE=0` for the corresponding normal-mode build. This
steering selection is a compile check, not confirmation of the trike's hardware.

### Arduino IDE upload

On this machine, no extra compiler flags are needed for the diagnostic build:

1. Open `Drive_By_Wire/Drive_By_Wire.ino` in Arduino IDE. In VS Code or another
    text editor, open `Drive_By_Wire/src/tests/FirmwareMode.h`, temporarily change
    `#define DBW_SERIAL_TEST_MODE 0` to `#define DBW_SERIAL_TEST_MODE 1`, and save.
    Files under `src` are compiled automatically but do not appear as IDE sketch tabs.
2. Select Arduino Due (Programming Port) and the current port (COM5 for this bench).
3. Keep the board powered by USB only, without a motor shield or actuators attached.
    Close Serial Monitor, then click Upload; the IDE compiles before uploading.
4. After upload, use Serial Monitor at 115200/Newline for manual commands, or leave
    it closed for the automated runner below.
5. Restore the header to 0 before committing or submitting code. This source-only
    change leaves the uploaded diagnostic firmware running. Do not upload again
    with 0 selected if you intend to continue bench diagnostics.

Before uploading mode 0 for driving, verify the actual steering method, wiring,
polarity, and calibration. Mode changes on the board require a new upload.

## Bench use

Start with a bare Due powered by USB, with no actuators or motor shield attached.
Use the Programming USB port, Serial Monitor at 115200 baud, and newline or CRLF.
Inputs are case-insensitive. Commands and numbers may share a line (`S -200`)
or use separate lines (`S`, then `-200`). `E` is processed without waiting for newline.

| Input | Action |
| --- | --- |
| L | Eight RGB combinations, one second each, then off |
| B | Brake boost for 800 ms, then holding voltage; throttle zero |
| R | Release brakes; previous motion is cancelled |
| V 5..255 | Store motor PWM for the next S test |
| S -1000..1000 | Timed steering; negative left, positive right; report raw and configured calibrated angle after stopping |
| T 0..255 | Raw 8-bit DAC command; nonzero requires brakes released |
| E | Stop steering, zero throttle code, apply brakes, latch stop |
| C | Report six RC pulse widths every 250 ms; stale inputs report zero |
| P 1000..1850 | Servo pulse train; report raw angle at command time |

These are deliberate limits beyond the email: V stores PWM without energizing the
motor; S is limited to one second; throttle and servo outputs expire after five
seconds and apply E-stop. A new valid letter command cancels previous motion and
clears the software stop latch; bare numbers do not. Clearing the latch does not
release brakes: use R explicitly. Invalid commands do not clear the latch.
Startup applies brakes and latches E-stop. Brake boost timing continues during all
tests and is not restarted by repeated B commands while brakes remain engaged.

D49/OP_ESTOP is pulled up; grounding it stops motion and inhibits commands while
held. This software input is not an independent hardware power cutoff.

Pin names come from DBW_Pins.h; brake polarity comes from RELAYInversion in
Settings.h. The Due drives relay logic, not 12/24 V directly. Verify brake supply,
relay polarity, steering enable/direction polarity, pulse limits, and sensor wiring
before connecting actuators. The existing ST_ON/ST_OFF convention is not bench
verified. PWM is also set to zero on every stop.
Normal motor-shield steering currently uses a direction boolean opposite to the
ST_LEFT/ST_RIGHT naming used by the old test() and interactive S test. This
pre-existing discrepancy is preserved rather than guessing physical polarity;
verify direction on hardware before either mode drives the trike.

DAC code zero is not zero physical volts. Due inputs must not receive 5 V.
Raw angle reporting uses 10-bit ADC counts. S also reports the existing
computeAngleLeft() conversion in tenths of a degree, using Settings.h calibration;
that calibration has not been verified for the connected trike. A timer-based
steering test cannot detect mechanical end stops or certify safe travel.

## Automatic serial PASS/FAIL tests

The Windows PowerShell runner tests the actual uploaded firmware through the
Programming USB port. It checks serial responses, not electrical pin levels or
physical actuator movement. Use only a bare Due with USB power: disconnect the
motor shield, actuators, vehicle power, and any external output drivers.

1. Upload a diagnostic build (`DBW_SERIAL_TEST_MODE=1`) to the Due first. The
    runner neither compiles nor uploads firmware. Check the header setting before uploading.
2. Close Arduino Serial Monitor and other applications using the serial port.
3. From the repository root, run the command below. COM5 is an example; confirm
    the current Programming Port in Arduino IDE or Device Manager.
4. No RESET is needed. After a short optional boot-banner check, the runner sends
    E and requires the exact diagnostic E-stop reply before starting any tests.
    It clears old received data before each handshake attempt and allows at most
    three attempts, using only E until the connection is verified.
5. The suite takes about 10 seconds after handshake. It prints each PASS/FAIL and
    its satisfied conditions as that test finishes, before starting the next test.
    It exits with code 0 for success, 1 for failure.

```powershell
.\tests\serial_test_mode\run-serial.ps1 -Port COM5 -BareDueConfirmed
```

`-BareDueConfirmed` is your explicit acknowledgement of the disconnected bench
setup. Without it, the runner refuses to open the port. It cannot detect physical
wiring. Opening/resetting the board executes whichever firmware was previously
uploaded, so verify that diagnostic firmware is installed first.

| Test | Commands | Serial PASS requirements |
| --- | --- | --- |
| Diagnostic handshake | Optional boot banner, then E | Exact diagnostic stop acknowledgement; banner not required for an already-running Due |
| LED sequence | L | Masks 0..7 in order, completion, each reported interval 650..1500 ms |
| Brakes | R, B | D44/D40 reports consistent with release/boost/hold; boost reported for 600..1200 ms |
| Steering | V 100, S -200 | Correct PWM/duration echo, raw readings in 0..1023, stop reported after 100..600 ms |

The runner prints TEST before attempting each command, including handshake
retries and cleanup stops, followed by PASS COMMAND or FAIL COMMAND after its
checks finish. Example excerpt from a successful brake test:

```text
TEST 04 [Brake sequence]: R -- wait for BRAKE OFF
PASS COMMAND 04 [Brake sequence]: R -- BRAKE OFF received: D44=1, D40=1; equal released relay levels.
TEST 05 [Brake sequence]: B -- wait for BRAKE HOLD (about 800 ms)
PASS COMMAND 05 [Brake sequence]: B -- BRAKE ON D44=0, D40=0 -> BRAKE HOLD D44=0, D40=1; expected relay transitions; 794.5 ms within 600..1200 ms (host timing).
TEST 06 [Brake sequence]: E -- wait for ESTOP acknowledgement
PASS COMMAND 06 [Brake sequence]: E -- Exact ESTOP reply received: PWM=0, DAC0=0, brakes applied, numeric commands blocked.
PASS: Brake sequence
```

The example intervals above are illustrative; each run prints its own measured
values rounded to one decimal place. Acceptance checks use the unrounded values.
Brake results show the actual reported relay levels and boost interval against
600..1200 ms. Steering results show the PWM/duration acknowledgement, initial and
final raw sensor values against 0..1023, and stop interval against 100..600 ms.
These are serial-response checks, not measurements of physical outputs.

Each command result appears before the next TEST line. L passes only after the
complete LED cycle; R after release levels are checked; B after boost/hold levels
and timing; V after the setting acknowledgement; S after the stop report and
timing/sensor range checks; E after its exact acknowledgement. A command failure
prints its cause before cleanup. Reconnect attempts have separate command results.

The overall sequence PASS is withheld until the cleanup E acknowledgement
succeeds. A cleanup failure makes the sequence FAIL even when an earlier command
passed. Remaining sequences print SKIP. Best-effort handshake-failure cleanup is
labelled UNVERIFIED when its reply is not checked, never PASS. The final three-line
verification summary does not repeat the individual results.

A TEST line shows a command attempt, not proof of receipt or actuator operation;
the subsequent responses and PASS/FAIL results provide protocol confirmation.
When using `-ReportPath`, the TX entries also include `Step` and `Test` alongside
their existing timestamp and exact command text. Each result also contains a
`Conditions` array with the same evidence shown in the console. `CommandResults`
records each command's step, test name, command text, status, evidence, and
completion timestamp, including failed retries and unverified cleanup requests.

### Repeat the sequence manually

Let the runner finish and release the port before opening Serial Monitor. Use
the same bare-Due setup, Programming USB, 115200 baud, and Newline. Type only the
command column below and send each separately. Do not paste the whole sequence
at once: a new letter command cancels the preceding action. Stop if an expected
response does not appear; send E instead of continuing the sequence.

| Step | Command | Wait before sending the next command |
| --- | --- | --- |
| 1 | E | ESTOP acknowledgement |
| 2 | L | LED cycle complete, about 8 seconds |
| 3 | E | ESTOP acknowledgement |
| 4 | R | BRAKE OFF |
| 5 | B | BRAKE HOLD, about 800 ms after BRAKE ON |
| 6 | E | ESTOP acknowledgement |
| 7 | V 100 | STEER PWM setting=100 |
| 8 | S -200 | STEER STOP and sensor report, about 200 ms |
| 9 | E | ESTOP acknowledgement |

The normal run sends these nine commands. Handshake retries can add E commands,
and failures skip later tests; the console log records what was actually attempted.

### Timing and reports

These timing windows use monotonic host receipt timestamps and deliberately allow
USB and operating-system jitter. They do not measure pin timing. Buffered serial
traffic can also cause a timing failure: inspect the transcript and repeat on a
quiet connection rather than assuming hardware malfunction. Floating sensor inputs
can pass the numeric range check; sensor accuracy and movement are not assessed.

The runner requests E and checks its acknowledgement before the suite and after
each case, including failures. It aborts remaining cases on failure. A missing
acknowledgement fails the test and leaves stop state UNKNOWN; sending E cannot
guarantee stopping a disconnected or frozen controller. After a completed suite,
closing does not send another unacknowledged E: the suite already performed stop
cleanup. An interrupted runner still attempts E as best-effort cleanup before
closing. No throttle or servo-motion commands are
sent. The exact reply `ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked`
is required before L/R/B/V/S commands are sent. An arbitrary message mentioning
E-stop is not accepted, nor is an otherwise correct reply joined to a truncated
older line. Such a handshake failure triggers a bounded receive-buffer reset and
a fresh stop request, not a relaxed match. The banner is only emitted at boot, so requiring it on
every connection would incorrectly reject a running diagnostic board. A timeout
includes recent received lines (or states that no lines were received) to aid
diagnosis; close other serial applications and check the port and uploaded mode.

To save results and the timestamped TX/RX transcript outside the repository:

```powershell
.\tests\serial_test_mode\run-serial.ps1 -Port COM5 -BareDueConfirmed -ReportPath "$env:TEMP\dbw-serial-result.json"
```

Even on success, the summary is deliberately limited:

```text
Serial behavior: PASS
Electrical outputs: NOT TESTED
Physical actuators: NOT TESTED
```

Exercise the runner without any Due or serial connection:

```powershell
.\tests\serial_test_mode\test-serial-runner.ps1
```

That script uses a deterministic fake serial transport and clock. It covers a
successful run, reconnect without a boot banner, stale partial replies, transient
and persistent corrupted acknowledgements, wrong firmware, missing/wrong colors, slow brake timing, incorrect
relay levels, wrong PWM, missing/early steering stop, invalid sensor values,
physical-stop rejection, missing stop acknowledgement, and disconnection. It also
checks result ordering, measured evidence, and that cleanup failures never print a
provisional PASS. A PASS
here verifies the runner itself, not the connected board. Electrical loopback
measurement and physical actuator checks remain separate future work.

## Controller host validation

Run `tests\serial_test_mode\run.cmd` on Windows with MSVC available (the script
defaults to VS 2019 Build Tools, or accepts an existing developer environment).
It compiles the original sketch, Vehicle.cpp, VehicleTestMode.cpp,
VehicleDiagnostics.cpp, RC_Controller.cpp, SerialTestMode.cpp,
SpeedController.cpp, SpeedControllerDiagnostics.cpp, SteeringController.cpp,
SteeringControllerDiagnostics.cpp, and PID.cpp as separate translation
units against mock Arduino/Servo/CAN APIs and minimal SD/time type declarations.
Tests inspect serial messages and mocked outputs, not private
module state. They check that diagnostic construction never commands nonzero
throttle/steering PWM or brake release, plus command timing, parsing, E-stop,
outputs, RC capture, steering and brake timer rollover (including repeated E), sensor conversion, and
normal controller startup/control. They also verify that diagnostic Vehicle
startup never initializes CAN or normal RC control, guarded RC/CAN calls do not
write outputs or consume navigation frames, and normal Vehicle startup still
initializes RC/CAN and sends status frames. The host build supplies the same steering
definitions shown above.
The normal SpeedController fixture has static storage so its memory starts at
zero; this is not evidence that upstream's uninitialized PIDThrottle is safe for
all allocations. The normal brake check uses Stop() after its original deadline,
not the diagnostic timer. The tests do not claim to fix upstream behavior.
These tests and Due compilation do not verify real electrical or mechanical behavior.