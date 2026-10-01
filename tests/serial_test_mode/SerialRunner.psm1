Set-StrictMode -Version Latest

function Wait-DbwLine {
    param($Context, [string]$Pattern, [int]$TimeoutMs = 2000, [switch]$Startup)
    $deadline = $Context.Transport.NowMs() + $TimeoutMs
    while ($Context.Transport.NowMs() -lt $deadline) {
        $remaining = [int][Math]::Ceiling($deadline - $Context.Transport.NowMs())
        $line = $Context.Transport.ReadLine($remaining)
        if ($null -eq $line) { continue }
        $line = $line.TrimEnd("`r", "`n")
        $received = $Context.Transport.NowMs()
        $Context.Transcript.Add([pscustomobject]@{ TimeMs = $received; Direction = 'RX'; Text = $line })
        if ($received -gt $deadline) { break }
        if ($line -cmatch $Pattern) {
            return [pscustomobject]@{ TimeMs = $received; Text = $line; Groups = $Matches.Clone() }
        }
        if ($Startup) { continue }
        if ($line -match '^(ERR|ESTOP:|OUTPUT TIMEOUT|DBW SERIAL TEST MODE:)') {
            throw "Unexpected firmware response: $line"
        }
        if ($line -match '^(COMMAND |LED |STEER |BRAKE OFF:|BRAKE ON:|THROTTLE |SERVO |RC us:)') {
            throw "Expected /$Pattern/ but received: $line"
        }
    }
    $receivedLines = @($Context.Transcript | Where-Object Direction -eq 'RX')
    $lastLines = ($receivedLines | Select-Object -Last 3 | ForEach-Object { $_.Text }) -join ' | '
    if (!$lastLines) { $lastLines = '(no serial lines received)' }
    throw [System.TimeoutException]::new("Timed out after $TimeoutMs ms waiting for /$Pattern/. Recent RX: $lastLines")
}

function Send-DbwCommand {
    param($Context, [string]$Command)
    $Context.CommandNumber++
    $Context.PendingCommand = [pscustomobject]@{
        Step = $Context.CommandNumber; Test = $Context.TestName; Command = $Command
    }
    $Context.Transcript.Add([pscustomobject]@{
        TimeMs = $Context.Transport.NowMs(); Direction = 'TX'; Text = $Command
        Step = $Context.CommandNumber; Test = $Context.TestName
    })
    if ($Context.ShowCommands) {
        $waitFor = switch ($Command) {
            'E' { 'wait for ESTOP acknowledgement' }
            'L' { 'wait for LED cycle complete (about 8 seconds)' }
            'R' { 'wait for BRAKE OFF' }
            'B' { 'wait for BRAKE HOLD (about 800 ms)' }
            'V 100' { 'wait for STEER PWM setting=100' }
            'S -200' { 'wait for STEER STOP (about 200 ms)' }
            default { 'wait for acknowledgement' }
        }
        Write-Host ('TEST {0:D2} [{1}]: {2} -- {3}' -f $Context.CommandNumber, $Context.TestName, $Command, $waitFor)
    }
    $Context.Transport.WriteLine($Command)
}

function Complete-DbwCommand {
    param($Context, [string]$Status, [string]$Detail)
    $pending = $Context.PendingCommand
    if ($null -eq $pending) { return }
    $Context.CommandResults.Add([pscustomobject]@{
        Step = $pending.Step; Test = $pending.Test; Command = $pending.Command
        Status = $Status; Detail = $Detail; TimeMs = $Context.Transport.NowMs()
    })
    $Context.PendingCommand = $null
    if ($Context.ShowResults) {
        Write-Host ('{0} COMMAND {1:D2} [{2}]: {3} -- {4}' -f $Status, $pending.Step, $pending.Test, $pending.Command, $Detail)
    }
}

function Start-DbwCommand {
    param($Context, [string]$Command)
    Send-DbwCommand $Context $Command
    $letter = [regex]::Escape($Command.Substring(0, 1))
    $null = Wait-DbwLine $Context "^COMMAND $letter; previous motion cancelled; ESTOP latch cleared$"
}

function Stop-DbwTest {
    param($Context)
    try {
        Send-DbwCommand $Context 'E'
        $null = Wait-DbwLine $Context '^ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked$'
        Complete-DbwCommand $Context 'PASS' 'Exact ESTOP reply received: PWM=0, DAC0=0, brakes applied, numeric commands blocked.'
    } catch {
        Complete-DbwCommand $Context 'FAIL' $_.Exception.Message
        throw
    }
}

function Assert-DbwInterval {
    param($Earlier, $Later, [int]$MinimumMs, [int]$MaximumMs, [string]$Name)
    $elapsed = $Later.TimeMs - $Earlier.TimeMs
    if ($elapsed -lt $MinimumMs -or $elapsed -gt $MaximumMs) {
        throw "$Name reported interval ${elapsed}ms outside ${MinimumMs}..${MaximumMs}ms (host receipt timing)"
    }
    return $elapsed
}

function Add-DbwResult {
    param($Context, $Results, [string]$Name, [string]$Status, [string]$Detail, [string[]]$Conditions = @())
    $Results.Add([pscustomobject]@{ Name = $Name; Status = $Status; Detail = $Detail; Conditions = $Conditions })
    if ($Context.ShowResults) {
        Write-Host "${Status}: $Name"
        if ($Status -ne 'PASS' -or $Conditions.Count -eq 0) { Write-Host "  $Detail" }
        foreach ($condition in $Conditions) { Write-Host "  SATISFIED: $condition" }
    }
}

function Test-DbwLed {
    param($Context)
    Start-DbwCommand $Context 'L'
    $first = Wait-DbwLine $Context '^LED RGB mask=0 \(off\)$'
    $previous = $first
    $intervals = [System.Collections.Generic.List[double]]::new()
    for ($color = 1; $color -lt 8; $color++) {
        $current = Wait-DbwLine $Context "^LED RGB mask=$color \(R=1 G=2 B=4\)$" 1800
        $intervals.Add((Assert-DbwInterval $previous $current 650 1500 'LED step'))
        $previous = $current
    }
    $complete = Wait-DbwLine $Context '^LED cycle complete$' 1800
    $intervals.Add((Assert-DbwInterval $previous $complete 650 1500 'Final LED step'))
    $measured = ($intervals | ForEach-Object { '{0:F1}' -f $_ }) -join ', '
    Complete-DbwCommand $Context 'PASS' "Masks 0..7 in order, LED cycle complete; host intervals $measured ms, all within 650..1500 ms."
    return [pscustomobject]@{
        Detail = 'Eight colors in order, completion, and 650..1500 ms reported intervals'
        Conditions = @(
            'Reported LED masks: 0, 1, 2, 3, 4, 5, 6, 7, in the required order.'
            'Received LED cycle complete after mask 7.'
            "Host intervals: $measured ms; all 8 within 650..1500 ms."
        )
    }
}

function Test-DbwBrakes {
    param($Context)
    Start-DbwCommand $Context 'R'
    $released = Wait-DbwLine $Context '^BRAKE OFF: D44=(?<on>[01]) D40=(?<voltage>[01])$'
    if ($released.Groups.on -ne $released.Groups.voltage) { throw 'Brake release levels disagree' }
    Complete-DbwCommand $Context 'PASS' "BRAKE OFF received: D44=$($released.Groups.on), D40=$($released.Groups.voltage); equal released relay levels."
    Start-DbwCommand $Context 'B'
    $applied = Wait-DbwLine $Context '^BRAKE ON: D44=(?<on>[01]) D40=(?<voltage>[01]); 800 ms boost$'
    if ($applied.Groups.on -eq $released.Groups.on -or $applied.Groups.voltage -eq $released.Groups.voltage) {
        throw 'Brake boost must invert both released relay levels'
    }
    $hold = Wait-DbwLine $Context '^BRAKE HOLD: D44=(?<on>[01]) D40=(?<voltage>[01])$' 1500
    if ($hold.Groups.on -ne $applied.Groups.on -or $hold.Groups.voltage -ne $released.Groups.voltage) {
        throw 'Brake hold must keep brakes applied and return voltage select to its release level'
    }
    $elapsed = Assert-DbwInterval $applied $hold 600 1200 'Brake boost'
    Complete-DbwCommand $Context 'PASS' ("BRAKE ON D44=$($applied.Groups.on), D40=$($applied.Groups.voltage) -> BRAKE HOLD D44=$($hold.Groups.on), D40=$($hold.Groups.voltage); expected relay transitions; {0:F1} ms within 600..1200 ms (host timing)." -f $elapsed)
    return [pscustomobject]@{
        Detail = 'Release/boost/hold levels and boost interval matched expectations'
        Conditions = @(
            "Reported release: D44=$($released.Groups.on), D40=$($released.Groups.voltage); both at the same released level."
            "Reported boost: D44=$($applied.Groups.on), D40=$($applied.Groups.voltage); both inverted from release."
            "Reported hold: D44=$($hold.Groups.on), D40=$($hold.Groups.voltage); brake stayed applied, voltage select returned to release level."
            ('Boost interval: {0:F1} ms; allowed 600..1200 ms (host receipt timing).' -f $elapsed)
        )
    }
}

function Test-DbwSteering {
    param($Context)
    Start-DbwCommand $Context 'V 100'
    $null = Wait-DbwLine $Context '^STEER PWM setting=100 \(output remains 0 until S\)$'
    Complete-DbwCommand $Context 'PASS' 'STEER PWM setting=100 received, matching the requested setting; firmware reports output remains zero until S.'
    Start-DbwCommand $Context 'S -200'
    $started = Wait-DbwLine $Context '^STEER ms=-200 PWM=100; initial angle=(?<raw>\d+)$'
    if ([int]$started.Groups.raw -gt 1023) { throw 'Initial ADC reading outside 10-bit range' }
    $stopped = Wait-DbwLine $Context '^STEER STOP; angle=(?<raw>\d+) \(10-bit raw\); calibrated=-?\d+ degX10$' 1000
    if ([int]$stopped.Groups.raw -gt 1023) { throw 'Final ADC reading outside 10-bit range' }
    $elapsed = Assert-DbwInterval $started $stopped 100 600 'Steering duration'
    Complete-DbwCommand $Context 'PASS' ("Reported ms=-200 and PWM=100; raw sensor $($started.Groups.raw) -> $($stopped.Groups.raw) within 0..1023; STEER STOP after {0:F1} ms within 100..600 ms (host timing). Sensor accuracy NOT TESTED." -f $elapsed)
    return [pscustomobject]@{
        Detail = 'PWM, signed duration, sensor report, and stop timing matched expectations; sensor accuracy NOT TESTED'
        Conditions = @(
            'Received STEER PWM setting=100 acknowledgement.'
            'Received STEER ms=-200 PWM=100, matching the requested direction, duration, and drive setting.'
            "Raw sensor reports: $($started.Groups.raw) -> $($stopped.Groups.raw); both within 0..1023. Accuracy NOT TESTED."
            ('Received STEER STOP after {0:F1} ms; allowed 100..600 ms (host receipt timing).' -f $elapsed)
        )
    }
}

function Invoke-DbwSerialSuite {
    param([Parameter(Mandatory)]$Transport, [switch]$ShowCommands, [switch]$ShowResults)
    $context = @{
        Transport = $Transport
        Transcript = [System.Collections.Generic.List[object]]::new()
        ShowCommands = $ShowCommands.IsPresent
        ShowResults = $ShowResults.IsPresent
        CommandNumber = 0
        PendingCommand = $null
        CommandResults = [System.Collections.Generic.List[object]]::new()
        TestName = 'Diagnostic handshake'
    }
    $results = [System.Collections.Generic.List[object]]::new()
    $ready = $false
    try {
        $sawBanner = $false
        try {
            $null = Wait-DbwLine $context '^DBW SERIAL TEST MODE: Programming USB, 115200, newline or CRLF$' 1000 -Startup
            $sawBanner = $true
        } catch [System.TimeoutException] {
        }
        for ($attempt = 1; $attempt -le 3; $attempt++) {
            try {
                $Transport.DiscardInput()
                Stop-DbwTest $context
                break
            } catch {
                if ($attempt -eq 3) { throw }
                $context.Transcript.Add([pscustomobject]@{
                    TimeMs = $Transport.NowMs(); Direction = 'SYNC'
                    Text = "Handshake attempt $attempt failed: $($_.Exception.Message)"
                })
            }
        }
        $ready = $true
        $detail = if ($sawBanner) { 'Boot banner and exact diagnostic E-stop reply received' } else { 'Connected to running firmware; exact diagnostic E-stop reply received' }
        Add-DbwResult $context $results 'Diagnostic handshake' 'PASS' $detail @(
            'E returned the exact required reply: ESTOP: PWM=0 DAC0=0; brakes applied; numeric commands blocked'
        )
    } catch {
        Add-DbwResult $context $results 'Diagnostic handshake' 'FAIL' $_.Exception.Message
        try {
            Send-DbwCommand $context 'E'
            Complete-DbwCommand $context 'UNVERIFIED' 'Best-effort stop after handshake failure; acknowledgement not checked.'
        } catch {
            Complete-DbwCommand $context 'FAIL' $_.Exception.Message
        }
    }

    $cases = @(
        @{ Name = 'LED sequence'; Action = 'Test-DbwLed' }
        @{ Name = 'Brake sequence'; Action = 'Test-DbwBrakes' }
        @{ Name = 'Timed steering'; Action = 'Test-DbwSteering' }
    )
    foreach ($case in $cases) {
        if (!$ready) {
            Add-DbwResult $context $results $case.Name 'SKIP' 'Aborted after prior failure'
            continue
        }
        $status = 'PASS'
        $detail = ''
        $conditions = @()
        $context.TestName = $case.Name
        try {
            $outcome = & $case.Action $context
            $detail = $outcome.Detail
            $conditions = $outcome.Conditions
        } catch {
            $status = 'FAIL'
            $detail = $_.Exception.Message
            Complete-DbwCommand $context 'FAIL' $detail
        } finally {
            try {
                Stop-DbwTest $context
                $conditions += 'Cleanup E returned the exact diagnostic E-stop acknowledgement.'
            } catch {
                $status = 'FAIL'
                $detail += "; E-stop acknowledgement failed: $($_.Exception.Message). Stop state is UNKNOWN."
            }
        }
        Add-DbwResult $context $results $case.Name $status $detail $conditions
        if ($status -eq 'FAIL') { $ready = $false }
    }
    return [pscustomobject]@{
        SerialBehavior = $(if ($ready) { 'PASS' } else { 'FAIL' })
        ElectricalOutputs = 'NOT TESTED'
        PhysicalActuators = 'NOT TESTED'
        Results = $results.ToArray()
        CommandResults = $context.CommandResults.ToArray()
        Transcript = $context.Transcript.ToArray()
    }
}

Export-ModuleMember -Function Invoke-DbwSerialSuite