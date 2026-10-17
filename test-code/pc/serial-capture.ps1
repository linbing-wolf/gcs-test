# Bounded serial capture: read a COM port for N seconds and append everything
# to a log file, so the output can be read back later from a script/agent.
#
# Usage (from the project folder):
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture.ps1 -Port COM10 -Seconds 300 -Out .\_check\com10.log -PulseDtr
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture.ps1 -Port COM7 -Keys "s" -Seconds 20 -Out .\_check\scan.log
#
# Notes:
#   - Only ONE program can hold a COM port at a time. Close the serial assistant
#     (or stop this script) before the other one runs.
#   - All log lines are ASCII on purpose: the STM32 side prints ASCII, and ASCII
#     avoids any codepage surprises in the log file.
#   - -Keys sends characters to the MCU while capturing (one every -KeyGapMs ms),
#     so a scripted key sequence can be replayed without a human at the keyboard.
param(
    [string]$Port    = 'COM10',
    [int]   $Baud    = 115200,
    [int]   $Seconds = 300,
    [string]$Out     = '',
    [switch]$PulseDtr,
    [string]$Keys    = '',
    [int]   $KeyGapMs = 3000,
    [int]   $KeyStartMs = 500
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($Out)) {
    $Out = Join-Path (Split-Path -Parent $PSScriptRoot) ("_check\capture-" + $Port + ".log")
}
$outDir = Split-Path -Parent $Out
if ($outDir -and -not (Test-Path -LiteralPath $outDir)) {
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
}

function Log([string]$text) {
    [System.IO.File]::AppendAllText($Out, $text, [System.Text.Encoding]::ASCII)
}

Log ("`r`n===== capture start " + (Get-Date).ToString('yyyy-MM-dd HH:mm:ss') + "  port=" + $Port + " baud=" + $Baud + " seconds=" + $Seconds + " pulseDtr=" + [bool]$PulseDtr + " =====`r`n")

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200

try {
    $sp.Open()
}
catch {
    Log ("OPEN FAILED: " + $_.Exception.Message + "`r`n")
    exit 1
}

if ($PulseDtr) {
    # Idle-high, pulse low, back high. On boards that wire DTR to NRST (the
    # classic auto-reset circuit) this reboots the MCU so its boot banner shows.
    Log "pulsing DTR (low) to try to reset the board...`r`n"
    $sp.DtrEnable = $true
    Start-Sleep -Milliseconds 80
    $sp.DtrEnable = $false
    Start-Sleep -Milliseconds 120
    $sp.DtrEnable = $true
    Start-Sleep -Milliseconds 80
}

$sw     = [System.Diagnostics.Stopwatch]::StartNew()
$total  = 0
$buffer = New-Object System.Text.StringBuilder
$keyIdx    = 0
$nextKeyMs = $KeyStartMs

if ($Keys.Length -gt 0) {
    Log ("scripted keys: '" + $Keys + "'  gap=" + $KeyGapMs + "ms  start=" + $KeyStartMs + "ms`r`n")
}

while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    $chunk = ''
    try { $chunk = $sp.ReadExisting() } catch { }

    if ($chunk -and $chunk.Length -gt 0) {
        $total += $chunk.Length
        [void]$buffer.Append($chunk)
        # Flush to disk as we go, so a partially filled log is already useful.
        if ($buffer.Length -gt 0) {
            Log $buffer.ToString()
            [void]$buffer.Clear()
        }
    }

    # Send the next scripted key when its turn comes.
    while ($keyIdx -lt $Keys.Length -and $sw.Elapsed.TotalMilliseconds -ge $nextKeyMs) {
        $k = $Keys[$keyIdx]
        $sp.Write([string]$k)
        Log ("`r`n--- [sent key '" + $k + "' at " + [int]$sw.Elapsed.TotalMilliseconds + "ms] ---`r`n")
        $keyIdx++
        $nextKeyMs += $KeyGapMs
    }

    Start-Sleep -Milliseconds 100
}

$sp.Close()
Log ("`r`n===== capture end   " + (Get-Date).ToString('yyyy-MM-dd HH:mm:ss') + "  totalChars=" + $total + " =====`r`n")
