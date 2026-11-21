[CmdletBinding()]
param(
    [string]$Port = 'COM25',
    [ValidateSet('R', 'F')][string]$Direction = 'R',
    [ValidateSet('M1', 'M2')][string[]]$Motors = @('M1', 'M2'),
    [ValidateRange(20, 255)][int]$Duty = 255,
    [ValidateRange(500, 6000)][int]$DurationMs = 6000,
    [ValidateRange(1, 5)][int]$Repeats = 2,
    [ValidateRange(1, 100000)][double]$TicksPerRevM1 = 1560.0,
    [ValidateRange(1, 100000)][double]$TicksPerRevM2 = 1320.0,
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'pid_data')
)

$ErrorActionPreference = 'Stop'
if ($DurationMs % 500 -ne 0) { throw 'DurationMs must be a multiple of 500.' }
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$outputPath = Join-Path $OutputDirectory ("car1_l298n_profile_{0:yyyyMMdd_HHmmss}.csv" -f (Get-Date))
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = 300
$serial.WriteTimeout = 1000
$rows = [System.Collections.Generic.List[object]]::new()

try {
    $serial.Open()
    Start-Sleep -Milliseconds 250
    for ($repeat = 1; $repeat -le $Repeats; $repeat++) {
        foreach ($motor in $Motors) {
            $motorTicksPerRev = if ($motor -eq 'M1') { $TicksPerRevM1 } else { $TicksPerRevM2 }
            $serial.DiscardInBuffer()
            $serial.WriteLine("profile $motor $Direction $Duty $DurationMs")
            $deadline = [Diagnostics.Stopwatch]::StartNew()
            $seenStart = $false
            $completed = $false
            while ($deadline.Elapsed.TotalSeconds -lt (($DurationMs / 1000.0) + 5.0)) {
                try { $line = $serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
                if ($line -match '^PROFILE,start,motor=(M[12]),dir=([FR]),duty=(\d+),requested_ms=(\d+)$') {
                    if ($Matches[1] -ne $motor -or $Matches[2] -ne $Direction -or [int]$Matches[3] -ne $Duty) {
                        throw "Unexpected profile header: $line"
                    }
                    $seenStart = $true
                    continue
                }
                if ($line -match '^PROFILE,bin=(\d+),dt_us=(\d+),ticks=(-?\d+),invalid=(\d+),a_rise=(\d+)$') {
                    if (!$seenStart) { throw "Profile bin without header: $line" }
                    $index = [int]$Matches[1]
                    $dtUs = [int64]$Matches[2]
                    $ticks = [int64]$Matches[3]
                    $invalid = [int64]$Matches[4]
                    $aRise = [int64]$Matches[5]
                    $countsPerSec = $ticks * 1000000.0 / $dtUs
                    $rows.Add([pscustomobject]@{
                        repeat = $repeat
                        motor = $motor
                        direction = $Direction
                        duty = $Duty
                        bin = $index
                        dt_us = $dtUs
                        ticks = $ticks
                        counts_per_sec = [math]::Round($countsPerSec, 2)
                        ticks_per_rev_assumed = $motorTicksPerRev
                        rpm_estimate = [math]::Round([math]::Abs($countsPerSec) * 60.0 / $motorTicksPerRev, 2)
                        invalid = $invalid
                        a_rise = $aRise
                    })
                    continue
                }
                if ($line -match '^PROFILE,done,bins=(\d+)$') {
                    if ([int]$Matches[1] -ne ($DurationMs / 500)) { throw "Incomplete profile: $line" }
                    $completed = $true
                    break
                }
                if ($line -match '^ERROR|^PROFILE,aborted') { throw "BMduino profile failed: $line" }
            }
            if (!$completed) { throw "Timed out waiting for $motor profile repeat $repeat." }
            Start-Sleep -Milliseconds 500
        }
    }
    $rows | Export-Csv -Path $outputPath -NoTypeInformation -Encoding utf8
    $rows | Format-Table repeat,motor,bin,dt_us,ticks,counts_per_sec,rpm_estimate,invalid,a_rise -AutoSize | Out-Host
    Write-Host "Saved $($rows.Count) bins to $outputPath"
}
finally {
    if ($serial.IsOpen) {
        try { $serial.WriteLine('stop') } catch {}
        $serial.Close()
    }
}
