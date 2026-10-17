# Push the text written in screen-text.txt to the TJC screen.
# You normally never run this by hand - just double-click 改屏上的字.bat
#
# How to edit what the screen shows:
#   1) open screen-text.txt with Notepad
#   2) change the line under the "#" comments, save
#   3) double-click 改屏上的字.bat
#
# NOTE: this file is intentionally ASCII-only so PowerShell can never mis-read it as GBK.
param(
    [string]$Port = 'COM12',
    [int]$Baud = 9600,
    [string]$Obj = 't0',
    [string]$Enc = 'gb2312',
    [string]$File = ''
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($File)) {
    $File = Join-Path -Path $PSScriptRoot -ChildPath 'screen-text.txt'
}

if (-not (Test-Path -LiteralPath $File)) {
    Write-Host "Text file not found: $File" -ForegroundColor Red
    exit 1
}

# Read the file whatever way Notepad saved it (UTF-8 with/without BOM, or ANSI/GBK).
function Read-TextFileSmart([string]$path) {
    $bytes = [System.IO.File]::ReadAllBytes($path)
    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        return [System.Text.Encoding]::UTF8.GetString($bytes, 3, $bytes.Length - 3)
    }
    if ($bytes.Length -ge 2 -and $bytes[0] -eq 0xFF -and $bytes[1] -eq 0xFE) {
        return [System.Text.Encoding]::Unicode.GetString($bytes, 2, $bytes.Length - 2)
    }
    try {
        $strict = New-Object System.Text.UTF8Encoding $false, $true
        return $strict.GetString($bytes)
    } catch {
        return ([System.Text.Encoding]::GetEncoding(936)).GetString($bytes)   # ANSI / GB2312
    }
}

$raw = Read-TextFileSmart $File
$raw = $raw.TrimStart([char]0xFEFF)          # drop a stray BOM character

# keep real text lines, throw away blank lines and "#" comment lines
$lines = $raw -split '\r?\n' | Where-Object {
    $_.Trim() -ne '' -and -not $_.TrimStart().StartsWith('#')
}
$text = ($lines -join ' ').Trim()

Write-Host "Text file : $File"
Write-Host "Will show : [$text]" -ForegroundColor Cyan

& (Join-Path -Path $PSScriptRoot -ChildPath 'tjc-text.ps1') -Text $text -Port $Port -Baud $Baud -Obj $Obj -Enc $Enc
