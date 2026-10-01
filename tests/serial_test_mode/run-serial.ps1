[CmdletBinding()]
param(
    [string]$Port = 'COM5',
    [switch]$BareDueConfirmed,
    [string]$ReportPath
)

$ErrorActionPreference = 'Stop'
if (!$BareDueConfirmed) {
    throw 'Disconnect motor shield, actuators, and vehicle power. Re-run with -BareDueConfirmed only for a bare USB-powered Due.'
}
Import-Module (Join-Path $PSScriptRoot 'SerialRunner.psm1') -Force
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200)
$serial.NewLine = "`n"
$serial.Encoding = [System.Text.Encoding]::ASCII
$serial.ReadTimeout = 250
$serial.WriteTimeout = 1000
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$transport = [pscustomobject]@{ Serial = $serial; Clock = [System.Diagnostics.Stopwatch]::StartNew() }
$transport | Add-Member -MemberType ScriptMethod -Name NowMs -Value { return $this.Clock.Elapsed.TotalMilliseconds }
$transport | Add-Member -MemberType ScriptMethod -Name WriteLine -Value { param($text) $this.Serial.WriteLine($text) }
$transport | Add-Member -MemberType ScriptMethod -Name DiscardInput -Value { $this.Serial.DiscardInBuffer() }
$transport | Add-Member -MemberType ScriptMethod -Name ReadLine -Value {
    param($remaining)
    $this.Serial.ReadTimeout = [Math]::Max(1, [Math]::Min(250, $remaining))
    try { return $this.Serial.ReadLine() } catch [System.TimeoutException] { return $null }
}
$exitCode = 1
$suiteCompleted = $false
try {
    $serial.Open()
    Write-Host "Connected to $Port at 115200. Verifying diagnostic firmware with E-stop; no RESET needed."
    $result = Invoke-DbwSerialSuite -Transport $transport -ShowCommands -ShowResults
    $suiteCompleted = $true
    Write-Host "Serial behavior: $($result.SerialBehavior)"
    Write-Host "Electrical outputs: $($result.ElectricalOutputs)"
    Write-Host "Physical actuators: $($result.PhysicalActuators)"
    if ($ReportPath) { $result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $ReportPath -Encoding UTF8 }
    if ($result.SerialBehavior -eq 'PASS') { $exitCode = 0 }
} catch {
    Write-Host "FAIL: serial runner - $($_.Exception.Message)"
    Write-Host 'Electrical outputs: NOT TESTED; physical actuators: NOT TESTED; stop state may be UNKNOWN.'
} finally {
    if ($serial.IsOpen) {
        if (!$suiteCompleted) {
            Write-Host 'TEST [Interrupted-run cleanup]: E -- best effort; acknowledgement not checked'
            try { $serial.WriteLine('E') } catch { Write-Warning 'Could not send final E-stop; do not assume outputs stopped.' }
        }
        $serial.Close()
    }
    $serial.Dispose()
}
exit $exitCode