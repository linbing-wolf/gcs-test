# Send text to a TJC (Taobao/Jingcai) serial screen - shortest possible usage.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tjc-text.ps1 -Text "hello"
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tjc-text.ps1 -Text "123.4"
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tjc-text.ps1 -Text "abc" -Obj t1
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tjc-text.ps1 -Text ""      (clears the text)
#
# This is just a thin wrapper: it builds  <Obj>.txt="<Text>"  and calls tjc-send.ps1.
# Port default COM12, baud default 9600 - that is the verified working setting for our screen.
#
# NOTE: this file is intentionally ASCII-only so PowerShell can never mis-read it as GBK.
param(
    [Parameter(Mandatory = $true)][AllowEmptyString()][string]$Text,
    [string]$Port = 'COM12',
    [int]$Baud = 9600,
    [string]$Obj = 't0',
    [string]$Enc = 'gb2312'
)

$ErrorActionPreference = 'Stop'

$sibling = Join-Path -Path $PSScriptRoot -ChildPath 'tjc-send.ps1'
if (-not (Test-Path -LiteralPath $sibling)) {
    Write-Host "Cannot find tjc-send.ps1 next to this script: $sibling" -ForegroundColor Red
    exit 1
}

# Guard against input that would break the command syntax.
if ($Text -match '[\r\n]') {
    Write-Host 'Text must be a single line (no line breaks).' -ForegroundColor Red
    exit 1
}
if ($Text.Contains('"')) {
    Write-Host 'Text must not contain a double quote character.' -ForegroundColor Red
    exit 1
}

$cmd = '{0}.txt="{1}"' -f $Obj, $Text
Write-Host "Building command: $cmd" -ForegroundColor Cyan

& $sibling -Port $Port -Baud $Baud -Cmd $cmd -Enc $Enc
