# STM32 串口监视器（不用装串口助手）
#
# 用法：
#   1. 先查设备管理器里板子占的是哪个 COM 口
#   2. 在本文件夹打开 PowerShell，运行：
#        powershell -ExecutionPolicy Bypass -File .\tools\serial-monitor.ps1 -Port COM9
#   3. 屏幕上会显示单片机打印的内容；直接在键盘打字就会发给单片机
#   4. 按 Ctrl+C 退出
#
# 注意：同一个串口同一时刻只能被一个程序打开。
#       串口助手还开着的话，这里会打不开。
param(
    [string]$Port = 'COM9',
    [int]$Baud = 115200
)

$ErrorActionPreference = 'Stop'

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 20

try {
    $sp.Open()
}
catch {
    Write-Host "打不开 $Port ：$($_.Exception.Message)" -ForegroundColor Red
    Write-Host "常见原因：口选错了 / 被别的程序占着 / 板子没插" -ForegroundColor Yellow
    exit 1
}

Write-Host ''
Write-Host "  已打开 $Port @ $Baud" -ForegroundColor Green
Write-Host "  单片机打印的内容会显示在下面" -ForegroundColor Green
Write-Host "  直接打字就会发给单片机，按 Ctrl+C 退出" -ForegroundColor Green
Write-Host ''

try {
    while ($true) {
        $incoming = $sp.ReadExisting()
        if ($incoming) { Write-Host -NoNewline $incoming }

        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            $ch = $key.KeyChar
            if ($ch -ne [char]0) {
                $sp.Write([string]$ch)
                Write-Host $ch -NoNewline -ForegroundColor Cyan
            }
        }

        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    Write-Host ''
    Write-Host '串口已关闭' -ForegroundColor Yellow
}
