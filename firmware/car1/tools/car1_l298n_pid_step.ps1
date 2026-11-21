[CmdletBinding()]
param(
    [string]$Port = 'COM25',
    [ValidateSet('M1', 'M2', 'Both')][string]$Motor = 'M1',
    [ValidateSet('Forward', 'Reverse')][string]$Direction = 'Forward',
    [ValidateRange(1, 100)][int]$Percent = 30,
    [ValidateRange(500, 10000)][int]$DurationMs = 5000,
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'pid_data')
)

$ErrorActionPreference = 'Stop'
$sign = if ($Direction -eq 'Forward') { 1 } else { -1 }
$left = if ($Motor -eq 'M1' -or $Motor -eq 'Both') { $Percent * $sign } else { 0 }
$right = if ($Motor -eq 'M2' -or $Motor -eq 'Both') { $Percent * $sign } else { 0 }
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = 20
$serial.WriteTimeout = 1000
$seq = [int](Get-Random -Minimum 1000 -Maximum 60000)
$rows = [System.Collections.Generic.List[object]]::new()

function Send-CommandFrame {
    param([int]$Sequence, [int]$Left, [int]$Right)
    $payload = "CMD,seq $Sequence,fwa 0,bwl $Left,bwr $Right"
    $checksum = 0
    foreach ($byte in [Text.Encoding]::ASCII.GetBytes($payload)) { $checksum = $checksum -bxor $byte }
    $serial.WriteLine('<' + $payload + (',cs {0:X2}>' -f $checksum))
}

try {
    $serial.Open()
    Start-Sleep -Milliseconds 200
    $serial.DiscardInBuffer()
    Send-CommandFrame -Sequence $seq -Left 0 -Right 0
    Start-Sleep -Milliseconds 350
    $serial.DiscardInBuffer()
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $nextSendMs = 0
    $lastAckMs = 0
    while ($timer.ElapsedMilliseconds -lt $DurationMs) {
        $elapsedMs = [int]$timer.ElapsedMilliseconds
        if ($elapsedMs -ge $nextSendMs) {
            Send-CommandFrame -Sequence $seq -Left $left -Right $right
            $nextSendMs += 100
        }
        try { $line = $serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        if ($line -match '^<STA,ackseq (\d+),f (\d+),dutyL (\d+),dutyR (\d+),rpmL (-?\d+),rpmR (-?\d+),posL (-?\d+),posR (-?\d+),') {
            $ack = [int]$Matches[1]
            $fault = [int]$Matches[2]
            $rows.Add([pscustomobject]@{
                elapsed_ms = $elapsedMs
                motor = $Motor
                direction = $Direction
                command_percent = $Percent
                seq = $seq
                ackseq = $ack
                fault = $fault
                duty_m1 = [int]$Matches[3]
                duty_m2 = [int]$Matches[4]
                rpm_m1 = [int]$Matches[5]
                rpm_m2 = [int]$Matches[6]
                pos_m1 = [int64]$Matches[7]
                pos_m2 = [int64]$Matches[8]
            })
            if ($ack -eq $seq) { $lastAckMs = $elapsedMs }
            if ($fault -ne 0) { throw "BMduino fault=$fault at ${elapsedMs}ms; $line" }
        }
        if ($elapsedMs -gt 700 -and $elapsedMs - $lastAckMs -gt 500) {
            throw "No matching ackseq for 500ms; competing UART command or link failure."
        }
    }
    if ($rows.Count -lt [math]::Floor($DurationMs / 250)) {
        throw "Only $($rows.Count) STA rows during ${DurationMs}ms test."
    }
    New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
    $outputPath = Join-Path $OutputDirectory ("car1_pid_step_{0:yyyyMMdd_HHmmss}_{1}_{2}_{3}.csv" -f (Get-Date), $Motor, $Direction, $Percent)
    $rows | Export-Csv -LiteralPath $outputPath -NoTypeInformation -Encoding utf8
    $steady = @($rows | Where-Object { $_.elapsed_ms -ge ($DurationMs - 1500) -and $_.ackseq -eq $seq })
    [pscustomobject]@{
        test = "$Motor $Direction $Percent%"
        samples = $rows.Count
        steady_duty_m1 = [math]::Round((($steady | Measure-Object duty_m1 -Average).Average), 1)
        steady_duty_m2 = [math]::Round((($steady | Measure-Object duty_m2 -Average).Average), 1)
        steady_rpm_m1 = [math]::Round((($steady | Measure-Object rpm_m1 -Average).Average), 1)
        steady_rpm_m2 = [math]::Round((($steady | Measure-Object rpm_m2 -Average).Average), 1)
        csv = $outputPath
    }
}
finally {
    if ($serial.IsOpen) {
        for ($i = 0; $i -lt 6; $i++) {
            try { Send-CommandFrame -Sequence ($seq + 1) -Left 0 -Right 0 } catch {}
            Start-Sleep -Milliseconds 80
        }
        $serial.Close()
    }
}
