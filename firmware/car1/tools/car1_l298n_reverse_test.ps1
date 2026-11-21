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

function Send-CommandFrame {
    param([int]$Sequence, [int]$Percent)
    $payload = "CMD,seq $Sequence,fwa 0,bwl $Percent,bwr $Percent"
    $checksum = 0
    foreach ($byte in [Text.Encoding]::ASCII.GetBytes($payload)) { $checksum = $checksum -bxor $byte }
    $serial.WriteLine('<' + $payload + (',cs {0:X2}>' -f $checksum))
}

try {
    $serial.Open()
    Start-Sleep -Milliseconds 200
    $serial.DiscardInBuffer()
    Send-CommandFrame -Sequence $seq -Percent 0
    Start-Sleep -Milliseconds 350
    $serial.DiscardInBuffer()

    $timer = [Diagnostics.Stopwatch]::StartNew()
    $nextSendMs = 0
    $reverseStartMs = -1
    while ($timer.ElapsedMilliseconds -lt 4500) {
        $elapsedMs = [int]$timer.ElapsedMilliseconds
        if ($elapsedMs -ge $nextSendMs) {
            $percent = if ($elapsedMs -lt 2000) { 30 } else { -30 }
            if ($percent -lt 0 -and $reverseStartMs -lt 0) { $reverseStartMs = $elapsedMs }
            Send-CommandFrame -Sequence $seq -Percent $percent
            $nextSendMs += 100
        }
        try { $line = $serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        if ($line -match '^<STA,ackseq (\d+),f (\d+),dutyL (\d+),dutyR (\d+),rpmL (-?\d+),rpmR (-?\d+),') {
            $rows.Add([pscustomobject]@{
                elapsed_ms = $elapsedMs
                since_reverse_ms = if ($reverseStartMs -lt 0) { -1 } else { $elapsedMs - $reverseStartMs }
                phase = if ($elapsedMs -lt 2000) { 'forward' } else { 'reverse' }
                ackseq = [int]$Matches[1]
                fault = [int]$Matches[2]
                duty_m1 = [int]$Matches[3]
                duty_m2 = [int]$Matches[4]
                rpm_m1 = [int]$Matches[5]
                rpm_m2 = [int]$Matches[6]
            })
        }
    }
    if ($rows.Count -lt 35) { throw "Only $($rows.Count) status samples." }
    if (@($rows | Where-Object { $_.ackseq -ne $seq -or $_.fault -ne 0 }).Count -gt 0) {
        throw 'Unexpected ackseq change or fault.'
    }
    $coast = @($rows | Where-Object { $_.phase -eq 'reverse' -and $_.since_reverse_ms -ge 100 -and $_.since_reverse_ms -lt 280 })
    if ($coast.Count -lt 1 -or @($coast | Where-Object { $_.duty_m1 -ne 0 -or $_.duty_m2 -ne 0 }).Count -gt 0) {
        throw 'Enable did not remain zero during reversal coast interval.'
    }
    $firstRestart = @($rows | Where-Object { $_.phase -eq 'reverse' -and $_.duty_m1 -gt 0 -and $_.duty_m2 -gt 0 } | Select-Object -First 1)
    if ($firstRestart.Count -ne 1 -or $firstRestart[0].since_reverse_ms -lt 300) {
        throw 'Enable resumed before 300 ms reversal wait.'
    }
    if ([math]::Abs($firstRestart[0].rpm_m1) -gt 30 -or [math]::Abs($firstRestart[0].rpm_m2) -gt 30) {
        throw 'Enable resumed while encoder status still showed substantial motion.'
    }
    $late = @($rows | Where-Object { $_.elapsed_ms -ge 4000 })
    if ((($late | Measure-Object rpm_m1 -Average).Average) -gt -70 -or
        (($late | Measure-Object rpm_m2 -Average).Average) -gt -70) {
        throw 'Both wheels did not settle to reverse speed.'
    }
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    $outputPath = Join-Path $OutputDirectory ("car1_pid_reverse_{0:yyyyMMdd_HHmmss}.csv" -f (Get-Date))
    $rows | Export-Csv -LiteralPath $outputPath -NoTypeInformation -Encoding utf8
    [pscustomobject]@{
        samples = $rows.Count
        first_reverse_duty_ms = $firstRestart[0].since_reverse_ms
        rpm_m1_at_restart = $firstRestart[0].rpm_m1
        rpm_m2_at_restart = $firstRestart[0].rpm_m2
        steady_reverse_rpm_m1 = [math]::Round((($late | Measure-Object rpm_m1 -Average).Average), 1)
        steady_reverse_rpm_m2 = [math]::Round((($late | Measure-Object rpm_m2 -Average).Average), 1)
        csv = $outputPath
    }
}
finally {
    if ($serial.IsOpen) {
        for ($i = 0; $i -lt 6; $i++) {
            try { Send-CommandFrame -Sequence ($seq + 1) -Percent 0 } catch {}
            Start-Sleep -Milliseconds 80
        }
        $serial.Close()
    }
}
