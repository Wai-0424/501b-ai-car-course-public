[CmdletBinding()]
param(
    [string]$Port = 'COM25',
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'pid_data')
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = 20
$serial.WriteTimeout = 1000
$seq = [int](Get-Random -Minimum 1000 -Maximum 60000)
$rows = [System.Collections.Generic.List[object]]::new()

function Send-Frame {
    param([string]$Payload)
    $checksum = 0
    foreach ($byte in [Text.Encoding]::ASCII.GetBytes($Payload)) { $checksum = $checksum -bxor $byte }
    $serial.WriteLine('<' + $Payload + (',cs {0:X2}>' -f $checksum))
}

try {
    $serial.Open()
    Start-Sleep -Milliseconds 200
    $serial.DiscardInBuffer()
    Send-Frame "CMD,seq $seq,fwa 0,bwl 0,bwr 0"
    Start-Sleep -Milliseconds 350
    $serial.DiscardInBuffer()

    $timer = [Diagnostics.Stopwatch]::StartNew()
    $lastValidMs = 0
    $nextSendMs = 0
    while ($timer.ElapsedMilliseconds -lt 3200) {
        $elapsedMs = [int]$timer.ElapsedMilliseconds
        if ($elapsedMs -ge $nextSendMs) {
            if ($elapsedMs -lt 2000) {
                Send-Frame "CMD,seq $seq,fwa 0,bwl 30,bwr 30"
                $lastValidMs = $elapsedMs
            } else {
                # Valid checksum, but no seq: must neither move motors nor refresh watchdog.
                Send-Frame 'CMD,fwa 0,bwl 30,bwr 30'
            }
            $nextSendMs += 100
        }
        try { $line = $serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        if ($line -match '^<STA,ackseq (\d+),f (\d+),dutyL (\d+),dutyR (\d+),rpmL (-?\d+),rpmR (-?\d+),') {
            $rows.Add([pscustomobject]@{
                elapsed_ms = $elapsedMs
                ms_since_last_valid = $elapsedMs - $lastValidMs
                phase = if ($elapsedMs -lt 2000) { 'driving' } else { 'no_seq_frames' }
                ackseq = [int]$Matches[1]
                fault = [int]$Matches[2]
                duty_m1 = [int]$Matches[3]
                duty_m2 = [int]$Matches[4]
                rpm_m1 = [int]$Matches[5]
                rpm_m2 = [int]$Matches[6]
            })
        }
    }
    if ($rows.Count -lt 25) { throw "Only $($rows.Count) status samples." }
    $driving = @($rows | Where-Object { $_.elapsed_ms -ge 1500 -and $_.elapsed_ms -lt 2000 })
    if (@($driving | Where-Object { $_.duty_m1 -gt 0 -and $_.duty_m2 -gt 0 }).Count -lt 3) {
        throw 'Both motors were not driven before the dropout.'
    }
    $stopped = @($rows | Where-Object { $_.phase -eq 'no_seq_frames' -and $_.duty_m1 -eq 0 -and $_.duty_m2 -eq 0 })
    if ($stopped.Count -eq 0) { throw 'Watchdog did not stop both motors.' }
    $firstStop = $stopped[0]
    if ($firstStop.ms_since_last_valid -gt 650) { throw "Watchdog stop took $($firstStop.ms_since_last_valid) ms." }
    if (@($rows | Where-Object { $_.ackseq -ne $seq -or $_.fault -ne 0 }).Count -gt 0) {
        throw 'Unexpected ackseq change or fault.'
    }
    if (@($rows | Where-Object { $_.elapsed_ms -ge ($firstStop.elapsed_ms + 100) -and ($_.duty_m1 -ne 0 -or $_.duty_m2 -ne 0) }).Count -gt 0) {
        throw 'Motors resumed after watchdog despite no-seq frames.'
    }
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    $outputPath = Join-Path $OutputDirectory ("car1_pid_watchdog_{0:yyyyMMdd_HHmmss}.csv" -f (Get-Date))
    $rows | Export-Csv -LiteralPath $outputPath -NoTypeInformation -Encoding utf8
    [pscustomobject]@{
        seq = $seq
        samples = $rows.Count
        stop_after_last_valid_ms = $firstStop.ms_since_last_valid
        duty_m1_at_stop = $firstStop.duty_m1
        duty_m2_at_stop = $firstStop.duty_m2
        csv = $outputPath
    }
}
finally {
    if ($serial.IsOpen) {
        for ($i = 0; $i -lt 6; $i++) {
            try { Send-Frame "CMD,seq $($seq + 1),fwa 0,bwl 0,bwr 0" } catch {}
            Start-Sleep -Milliseconds 80
        }
        $serial.Close()
    }
}
