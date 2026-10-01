$ErrorActionPreference = 'Stop'
Import-Module (Join-Path $PSScriptRoot 'SerialRunner.psm1') -Force

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (!$Condition) { throw $Message }
}

function New-FakeTransport {
    param([string]$Fault = '')
    $fake = [pscustomobject]@{
        Time = 0; Fault = $Fault; Stops = 0
        Partial = $(if ($Fault -eq 'StalePartial') { 'OP: PWM=0 DAC0=0; brakes applied; numeric comman' } else { '' })
        Discards = 0
        Queue = [System.Collections.Generic.List[object]]::new()
        Commands = [System.Collections.Generic.List[string]]::new()
    }
    $fake | Add-Member ScriptMethod NowMs { return $this.Time }
    $fake | Add-Member ScriptMethod DiscardInput {
        $this.Discards++
        $this.Partial = ''
        $this.Queue.Clear()
    }
    $fake | Add-Member ScriptMethod Enqueue {
        param($offset, $text)
        $this.Queue.Add([pscustomobject]@{ Time = $this.Time + $offset; Text = $text })
    }
    $fake | Add-Member ScriptMethod ReadLine {
        param($timeout)
        if ($this.Queue.Count -gt 0 -and $this.Queue[0].Time -le $this.Time + $timeout) {
            $line = $this.Queue[0]
            $this.Queue.RemoveAt(0)
            $this.Time = [Math]::Max($this.Time, $line.Time)
            $text = $this.Partial + $line.Text
            $this.Partial = ''
            return $text
        }
        $this.Time += $timeout
        return $null
    }
    $fake | Add-Member ScriptMethod WriteLine {
        param($command)
        $this.Commands.Add($command)
        if ($command -eq 'E') {
            $this.Stops++
            $this.Queue.Clear()
            if ($this.Fault -eq 'AlwaysGarbled' -or ($this.Fault -eq 'GarbledOnce' -and $this.Stops -eq 1)) {
                $this.Enqueue(0, 'OP: PWM=0 DAC0=0; brakes applied; numeric commanESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked')
            } elseif ($this.Fault -eq 'WrongFirmware') {
                $this.Enqueue(0, 'Unknown command')
            } elseif ($this.Fault -ne 'NoStop' -or $this.Stops -eq 1) {
                $this.Enqueue(0, 'ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked')
            }
            return
        }
        if ($this.Fault -eq 'Disconnected') { throw 'Cable disconnected' }
        if ($this.Fault -eq 'PhysicalStop') {
            $this.Enqueue(0, 'ERR ESTOP latched or physical stop grounded')
            return
        }
        $this.Enqueue(0, "COMMAND $($command[0]); previous motion cancelled; ESTOP latch cleared")
        switch ($command) {
            'L' {
                $this.Enqueue(0, 'LED RGB mask=0 (off)')
                for ($color = 1; $color -lt 8; $color++) {
                    if ($this.Fault -eq 'MissingColor' -and $color -eq 3) { continue }
                    $mask = if ($this.Fault -eq 'WrongColor' -and $color -eq 3) { 2 } else { $color }
                    $this.Enqueue($color * 1000, "LED RGB mask=$mask (R=1 G=2 B=4)")
                }
                $this.Enqueue(8000, 'LED cycle complete')
            }
            'R' { $this.Enqueue(0, 'BRAKE OFF: D44=1 D40=1') }
            'B' {
                $this.Enqueue(0, 'BRAKE ON: D44=0 D40=0; 800 ms boost')
                $duration = if ($this.Fault -eq 'SlowBrake') { 1400 } else { 800 }
                $level = if ($this.Fault -eq 'WrongRelay') { 1 } else { 0 }
                $this.Enqueue($duration, "BRAKE HOLD: D44=$level D40=1")
            }
            'V 100' { $this.Enqueue(0, 'STEER PWM setting=100 (output remains 0 until S)') }
            'S -200' {
                $pwm = if ($this.Fault -eq 'WrongPwm') { 255 } else { 100 }
                $this.Enqueue(0, "STEER ms=-200 PWM=$pwm; initial angle=512")
                if ($this.Fault -ne 'MissingStop') {
                    $duration = if ($this.Fault -eq 'EarlyStop') { 0 } else { 200 }
                    $raw = if ($this.Fault -eq 'BadSensor') { 2000 } else { 512 }
                    $this.Enqueue($duration, "STEER STOP; angle=$raw (10-bit raw); calibrated=0 degX10")
                }
            }
            default { throw "Unexpected test command: $command" }
        }
    }
    if ($Fault -notin @('WrongFirmware', 'AlreadyRunning', 'StalePartial')) {
        $fake.Enqueue(0, 'ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked')
        $fake.Enqueue(0, 'DBW SERIAL TEST MODE: Programming USB, 115200, newline or CRLF')
    }
    return $fake
}

$refused = $false
try {
    & (Join-Path $PSScriptRoot 'run-serial.ps1') -Port 'DO_NOT_OPEN'
} catch {
    $refused = $_.Exception.Message -match 'BareDueConfirmed'
}
Assert-True $refused 'Runner must refuse before opening a port without bare-Due confirmation'
Write-Host 'PASS: refuses hardware access without explicit bench confirmation'

$fake = New-FakeTransport
$result = Invoke-DbwSerialSuite $fake
Assert-True ($result.SerialBehavior -eq 'PASS') 'Expected successful serial run'
Assert-True ($fake.Stops -eq 4) 'Expected initial stop plus cleanup after every test'
Assert-True (($result.Results | Where-Object Status -eq PASS).Count -eq 4) 'Expected all checks to pass'
Assert-True ($result.ElectricalOutputs -eq 'NOT TESTED' -and $result.PhysicalActuators -eq 'NOT TESTED') 'Must not claim physical verification'
Write-Host 'PASS: successful run and per-test E-stop cleanup'

$fake = New-FakeTransport
$captured = @(Invoke-DbwSerialSuite $fake -ShowCommands 6>&1)
$logged = @($captured | Where-Object { $_ -is [System.Management.Automation.InformationRecord] } | ForEach-Object { $_.ToString() })
$result = $captured | Where-Object { $_ -isnot [System.Management.Automation.InformationRecord] }
Assert-True ($result.SerialBehavior -eq 'PASS') 'Command logging must not change test results'
$expected = @('E', 'L', 'E', 'R', 'B', 'E', 'V 100', 'S -200', 'E')
Assert-True ($logged.Count -eq $expected.Count) 'Expected one log entry per command, including cleanup stops'
$transmitted = @($result.Transcript | Where-Object Direction -eq TX)
for ($index = 0; $index -lt $expected.Count; $index++) {
    $step = $index + 1
    Assert-True ($logged[$index].StartsWith(('TEST {0:D2} [' -f $step))) 'Log steps must be numbered in order and use TEST'
    Assert-True ($logged[$index].Contains("]: $($expected[$index]) -- wait for ")) 'Log must show the exact command and wait instruction'
    Assert-True ($transmitted[$index].Text -eq $expected[$index] -and $transmitted[$index].Step -eq $step) 'JSON transcript and console command order must match'
}
Assert-True ($logged[1].Contains('[LED sequence]') -and $logged[1].Contains('about 8 seconds')) 'LED replay must explain the wait for completion'
Assert-True ($logged[4].Contains('[Brake sequence]') -and $logged[4].Contains('BRAKE HOLD')) 'Brake replay must wait for holding state'
Assert-True ($logged[7].Contains('[Timed steering]') -and $logged[7].Contains('STEER STOP')) 'Steering replay must wait for stop'
Write-Host 'PASS: numbered command log, replay waits, and JSON transcript match the actual sequence'

$fake = New-FakeTransport
$captured = @(Invoke-DbwSerialSuite $fake -ShowCommands -ShowResults 6>&1)
$events = [System.Collections.Generic.List[string]]::new()
foreach ($entry in $captured) {
    if ($entry -is [System.Management.Automation.InformationRecord]) { $events.Add($entry.ToString()) }
}
$result = $captured | Where-Object { $_ -isnot [System.Management.Automation.InformationRecord] }
Assert-True ($result.SerialBehavior -eq 'PASS') 'Live result display must preserve suite result'
Assert-True (@($events | Where-Object { $_ -like 'PASS:*' }).Count -eq 4) 'Expected exactly one result per completed check'
Assert-True ($events.IndexOf('PASS: Diagnostic handshake') -lt $events.FindIndex([Predicate[string]]{ param($line) $line -like 'TEST 02 *' })) 'Handshake result must precede LED commands'
foreach ($caseOrder in @(
    @{ Name = 'LED sequence'; Cleanup = 3; Next = 4 }
    @{ Name = 'Brake sequence'; Cleanup = 6; Next = 7 }
    @{ Name = 'Timed steering'; Cleanup = 9; Next = 0 }
)) {
    $passIndex = $events.IndexOf("PASS: $($caseOrder.Name)")
    $cleanupPrefix = 'TEST {0:D2} ' -f $caseOrder.Cleanup
    $cleanupIndex = $events.FindIndex([Predicate[string]]{ param($line) $line.StartsWith($cleanupPrefix) })
    Assert-True ($passIndex -gt $cleanupIndex) 'A PASS must be printed only after its cleanup stop'
    if ($caseOrder.Next -gt 0) {
        $nextPrefix = 'TEST {0:D2} ' -f $caseOrder.Next
        $nextIndex = $events.FindIndex([Predicate[string]]{ param($line) $line.StartsWith($nextPrefix) })
        Assert-True ($passIndex -lt $nextIndex) 'Result must be printed before starting the next test'
    }
}
foreach ($test in $result.Results) {
    foreach ($condition in $test.Conditions) {
        Assert-True ($events.Contains("  SATISFIED: $condition")) 'Console evidence must match JSON result conditions'
    }
}
$evidence = $events -join "`n"
Assert-True ($evidence.Contains('Reported LED masks: 0, 1, 2, 3, 4, 5, 6, 7')) 'LED pass must explain order verification'
Assert-True ($evidence.Contains('1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0 ms')) 'LED pass must show all measured intervals'
Assert-True ($evidence.Contains('Reported hold: D44=0, D40=1')) 'Brake pass must show actual reported levels'
Assert-True ($evidence.Contains('800.0 ms; allowed 600..1200 ms')) 'Brake pass must show measured versus allowed timing'
Assert-True ($evidence.Contains('200.0 ms; allowed 100..600 ms')) 'Steering pass must show measured versus allowed timing'
Assert-True ($evidence.Contains('512 -> 512; both within 0..1023. Accuracy NOT TESTED.')) 'Sensor check must state values, range, and limitation'
Write-Host 'PASS: results are interleaved after cleanup with measured evidence and acceptance conditions'

Assert-True ($result.CommandResults.Count -eq $expected.Count) 'Every sent command must have its own verification result'
for ($index = 0; $index -lt $expected.Count; $index++) {
    $step = $index + 1
    $commandResult = $result.CommandResults[$index]
    Assert-True ($commandResult.Step -eq $step -and $commandResult.Command -eq $expected[$index] -and $commandResult.Status -eq 'PASS') 'Command result must match its sent command'
    $commandPrefix = 'TEST {0:D2} ' -f $step
    $passPrefix = 'PASS COMMAND {0:D2} ' -f $step
    $commandIndex = $events.FindIndex([Predicate[string]]{ param($line) $line.StartsWith($commandPrefix) })
    $passIndex = $events.FindIndex([Predicate[string]]{ param($line) $line.StartsWith($passPrefix) })
    Assert-True ($passIndex -gt $commandIndex) 'Each command PASS must follow its TEST line'
    Assert-True ($events[$passIndex].EndsWith($commandResult.Detail)) 'Console and JSON must share command evidence'
    if ($step -lt $expected.Count) {
        $nextPrefix = 'TEST {0:D2} ' -f ($step + 1)
        $nextIndex = $events.FindIndex([Predicate[string]]{ param($line) $line.StartsWith($nextPrefix) })
        Assert-True ($passIndex -lt $nextIndex) 'Each command must print its result before the next TEST'
    }
}
Assert-True ($result.CommandResults[1].TimeMs -eq 8000) 'L must not pass before the complete eight-color cycle'
Assert-True ($result.CommandResults[4].TimeMs - $result.CommandResults[3].TimeMs -eq 800) 'B must not pass before the holding-voltage transition'
Assert-True ($result.CommandResults[7].TimeMs - $result.CommandResults[6].TimeMs -eq 200) 'S must not pass before the steering stop report'
Assert-True (@($events | Where-Object { $_ -like 'SEND *' }).Count -eq 0) 'Console command label must be TEST, not SEND'
Write-Host 'PASS: every command prints verified evidence before the next TEST, after its full action completes'

$fake = New-FakeTransport 'SlowBrake'
$captured = @(Invoke-DbwSerialSuite $fake -ShowCommands -ShowResults 6>&1)
$failedEvents = [System.Collections.Generic.List[string]]::new()
foreach ($entry in $captured) {
    if ($entry -is [System.Management.Automation.InformationRecord]) { $failedEvents.Add($entry.ToString()) }
}
$failedResult = $captured | Where-Object { $_ -isnot [System.Management.Automation.InformationRecord] }
Assert-True ($failedResult.CommandResults[4].Status -eq 'FAIL') 'B must fail when hold timing is outside tolerance'
$failureIndex = $failedEvents.FindIndex([Predicate[string]]{ param($line) $line.StartsWith('FAIL COMMAND 05 ') })
$cleanupIndex = $failedEvents.FindIndex([Predicate[string]]{ param($line) $line.StartsWith('TEST 06 ') })
Assert-True ($failureIndex -ge 0 -and $failureIndex -lt $cleanupIndex) 'Command failure must print before cleanup E is sent'
Assert-True ($failedEvents.FindIndex([Predicate[string]]{ param($line) $line.StartsWith('PASS COMMAND 05 ') }) -eq -1) 'No early PASS on B just because BRAKE ON was acknowledged'
Assert-True ($failedResult.CommandResults[5].Status -eq 'PASS' -and $failedResult.Results[2].Status -eq 'FAIL') 'Passing cleanup does not turn a failed action into a test PASS'
Write-Host 'PASS: a failed command reports its cause before cleanup; test remains failed even if cleanup passes'

$fake = New-FakeTransport 'NoStop'
$captured = @(Invoke-DbwSerialSuite $fake -ShowCommands -ShowResults 6>&1)
$events = @($captured | Where-Object { $_ -is [System.Management.Automation.InformationRecord] } | ForEach-Object { $_.ToString() })
Assert-True ($events -contains 'FAIL: LED sequence') 'Missing cleanup acknowledgement must print FAIL'
Assert-True ($events -notcontains 'PASS: LED sequence') 'Do not print provisional PASS before cleanup succeeds'
Assert-True ($events -contains 'SKIP: Brake sequence') 'Failure must print skipped following tests'
Assert-True (($events -join "`n") -match 'Stop state is UNKNOWN') 'Cleanup failure must retain its specific cause'
Assert-True (@($events | Where-Object { $_ -like 'FAIL COMMAND 03 *' }).Count -eq 1) 'Cleanup E must have its own FAIL result'
Write-Host 'PASS: cleanup failure prints FAIL and SKIP, never a provisional PASS'

$fake = New-FakeTransport 'AlreadyRunning'
$result = Invoke-DbwSerialSuite $fake
Assert-True ($result.SerialBehavior -eq 'PASS') 'Must connect to diagnostic firmware already running without a new boot banner'
Assert-True ($fake.Commands[0] -eq 'E') 'Must verify the diagnostic stop reply before starting tests'
Write-Host 'PASS: reconnects to already-running diagnostic firmware without RESET'

$fake = New-FakeTransport 'StalePartial'
$result = Invoke-DbwSerialSuite $fake
Assert-True ($result.SerialBehavior -eq 'PASS') 'Must discard a stale partial line before requesting a fresh E-stop reply'
Assert-True ($fake.Discards -ge 1) 'Must clear the receive buffer before the handshake'
Write-Host 'PASS: clears stale partial reply before reconnect handshake'

$fake = New-FakeTransport 'GarbledOnce'
$result = Invoke-DbwSerialSuite $fake
Assert-True ($result.SerialBehavior -eq 'PASS') 'Must request a fresh reply after an initial garbled acknowledgement'
Assert-True ($fake.Commands[0] -eq 'E' -and $fake.Commands[1] -eq 'E') 'Only stop commands may be used for resynchronization'
Assert-True ($fake.Discards -eq 2) 'Expected one bounded reconnect retry'
Assert-True ($result.CommandResults[0].Status -eq 'FAIL' -and $result.CommandResults[1].Status -eq 'PASS') 'Retries must keep distinct failure and success command results'
Write-Host 'PASS: retries one corrupted handshake without relaxing the acknowledgement pattern'

foreach ($fault in @('WrongFirmware', 'AlwaysGarbled', 'MissingColor', 'WrongColor', 'SlowBrake', 'WrongRelay', 'WrongPwm', 'MissingStop', 'EarlyStop', 'BadSensor', 'PhysicalStop', 'NoStop', 'Disconnected')) {
    $fake = New-FakeTransport $fault
    $result = Invoke-DbwSerialSuite $fake
    Assert-True ($result.SerialBehavior -eq 'FAIL') "Expected failure for $fault"
    Assert-True ($fake.Commands[$fake.Commands.Count - 1] -eq 'E') "Expected cleanup attempt for $fault"
    Assert-True ($fake.Time -le 20000) "Unbounded wait for $fault"
    if ($fault -in @('WrongFirmware', 'AlwaysGarbled')) {
        Assert-True (@($fake.Commands | Where-Object { $_ -ne 'E' }).Count -eq 0) 'Must not start tests without exact diagnostic acknowledgement'
        Assert-True ($fake.Discards -eq 3) 'Handshake retries must be bounded'
    }
    if ($fault -eq 'NoStop') {
        Assert-True ($result.Results[1].Detail -match 'UNKNOWN') 'Missing stop acknowledgement must not pass'
    }
    Write-Host "PASS: detects $fault and attempts stop"
}