# Catch the MCU boot banner on a COM port.
#
# The firmware only prints its banner once, right after reset, so a plain
# capture that starts later never sees it. This script waits for the port to
# disappear and come back (USB unplug/replug reboots the board), then opens the
# port immediately and records the first seconds of output.
#
# Usage (from the project folder):
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture-boot.ps1 -Port COM7
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture-boot.ps1 -Port COM7 -CaptureSeconds 15
#
# Notes:
#   - Only ONE program can hold a COM port at a time. Close the serial assistant
#     (or stop this script) before the other one runs.
#   - Keep the log ASCII on purpose: the STM32 side prints ASCII, and ASCII
#     avoids codepage surprises in the log file.
param(
    [string]$Port           = 'COM7',
    [int]   $Baud           = 115200,
    [int]   $WaitSeconds    = 90,
    [int]   $CaptureSeconds = 12,
    [string]$Out            = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($Out)) {
    $Out = Join-Path (Split-Path -Parent $PSScriptRoot) ("_check\boot-" + $Port + ".log")
}
$outDir = Split-Path -Parent $Out
if ($outDir -and -not (Test-Path -LiteralPath $outDir)) {
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
}

function Log([string]$text) {
    [System.IO.File]::AppendAllText($Out, $text, [System.Text.Encoding]::ASCII)
}

function PortPresent([string]$name) {
    return ([System.IO.Ports.SerialPort]::GetPortNames() -contains $name)
}

Log ("`r`n===== boot capture " + (Get-Date).ToString('yyyy-MM-dd HH:mm:ss') + "  port=" + $Port + " baud=" + $Baud + " =====`r`n")

$sw = [System.Diagnostics.Stopwatch]::StartNew()

if (PortPresent $Port) {
    Log "waiting for the USB cable to be UNPLUGGED (port disappears)...`r`n"
    while ((PortPresent $Port) -and ($sw.Elapsed.TotalSeconds -lt $WaitSeconds)) {
        Start-Sleep -Milliseconds 200
    }
}

if (-not (PortPresent $Port)) {
    Log "port gone. waiting for it to come BACK (plug the USB cable in again)...`r`n"
    while ((-not (PortPresent $Port)) -and ($sw.Elapsed.TotalSeconds -lt $WaitSeconds)) {
        Start-Sleep -Milliseconds 200
    }
}

if (-not (PortPresent $Port)) {
    Log "TIMEOUT: port never came back within $WaitSeconds s.`r`n"
    exit 1
}

Log "port is back. opening it now...`r`n"

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
try {
    $sp.Open()
}
catch {
    # The board needs a moment after enumeration before the port is usable.
    Start-Sleep -Milliseconds 700
    try {
        $sp.Open()
    }
    catch {
        Log ("OPEN FAILED: " + $_.Exception.Message + "`r`n")
        exit 1
    }
}

$total = 0
$cap   = [System.Diagnostics.Stopwatch]::StartNew()
while ($cap.Elapsed.TotalSeconds -lt $CaptureSeconds) {
    $chunk = ''
    try { $chunk = $sp.ReadExisting() } catch { }
    if ($chunk -and $chunk.Length -gt 0) {
        $total += $chunk.Length
        [System.IO.File]::AppendAllText($Out, $chunk, [System.Text.Encoding]::ASCII)
    }
    Start-Sleep -Milliseconds 50
}

try { $sp.Close() } catch { }
Log ("`r`n===== done, totalChars=" + $total + " =====`r`n")
