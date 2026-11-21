[CmdletBinding()]
param(
    [ValidateSet('F', 'R')]
    [string]$Direction = 'F',
    [int[]]$Duties = @(20, 40, 60, 80, 100, 120, 140),
    [ValidateRange(50, 3000)]
    [int]$DurationMs = 500,
    [string]$Port = 'COM25',
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'pid_data')
)

$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$outputPath = Join-Path $OutputDirectory ("car1_l298n_open_loop_{0:yyyyMMdd_HHmmss}_{1}.csv" -f (Get-Date), $Direction)

$serial = [System.IO.Ports.SerialPort]::new($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = 200
$serial.WriteTimeout = 1000

function Invoke-Pulse {
    param([string]$Motor, [int]$Duty)

    $serial.DiscardInBuffer()
    $serial.WriteLine("pulse $Motor $Direction $Duty $DurationMs")
    $deadline = [Diagnostics.Stopwatch]::StartNew()
    $responseTimeoutSec = ($DurationMs / 1000.0) + 2.0
    while ($deadline.Elapsed.TotalSeconds -lt $responseTimeoutSec) {
        try { $line = $serial.ReadLine().Trim() } catch [System.TimeoutException] { continue }
        if ($line -match '^PULSE,done,motor=(M[12]),encoder_delta=(-?\d+)$') {
            return [pscustomobject]@{
                timestamp_utc = (Get-Date).ToUniversalTime().ToString('o')
                motor = $Matches[1]
                direction = $Direction
                duty = $Duty
                duration_ms = $DurationMs
                encoder_delta = [int64]$Matches[2]
                counts_per_sec = [math]::Round(([int64]$Matches[2] * 1000.0 / $DurationMs), 2)
            }
        }
        if ($line -match '^ABORT|^ERROR') { throw "BMduino rejected $Motor duty ${Duty}: $line" }
    }
    throw "No completed pulse response for $Motor duty $Duty within $responseTimeoutSec seconds."
}

try {
    $serial.Open()
    Start-Sleep -Milliseconds 250
    $serial.DiscardInBuffer()
    $rows = foreach ($motor in 'M1', 'M2') {
        foreach ($duty in $Duties) {
            Invoke-Pulse -Motor $motor -Duty $duty
        }
    }
    $rows | Export-Csv -NoTypeInformation -Encoding utf8 -Path $outputPath
    $rows | Format-Table -AutoSize | Out-Host
    Write-Host "Saved $($rows.Count) samples to $outputPath"
}
finally {
    if ($serial.IsOpen) {
        try { $serial.WriteLine('stop') } catch {}
        $serial.Close()
    }
}
