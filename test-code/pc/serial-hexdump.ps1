<#
  把串口收到的原始字节按 十六进制 + ASCII 打出来 —— 专治"这东西到底有没有在说话"。
  典型用途：验证陀螺仪（应该能数出 55 53 开头的 11 字节包）。

  用法：
      .\tools\serial-hexdump.ps1 -Port COM9 -Seconds 5              # 默认 115200
      .\tools\serial-hexdump.ps1 -Port COM9 -Baud 9600 -Seconds 5   # 换波特率

  只想看前几字节够判断了，不用抓很久。
#>
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$Baud = 115200,
    [int]$Seconds = 5
)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
$sp.ReadTimeout = 200
try { $sp.Open() } catch {
    Write-Host ("打不开 " + $Port + " : " + $_.Exception.Message) -ForegroundColor Red
    exit 1
}

Write-Host ("已打开 " + $Port + " @ " + $Baud + "，抓 " + $Seconds + " 秒...") -ForegroundColor Cyan

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$bytes = New-Object System.Collections.Generic.List[byte]
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        while ($sp.BytesToRead -gt 0) { $bytes.Add([byte]$sp.ReadByte()) }
    } catch { }
    Start-Sleep -Milliseconds 50
}
$sp.Close()

if ($bytes.Count -eq 0) {
    Write-Host "一个字节都没收到。" -ForegroundColor Yellow
    Write-Host "依次查：TX/RX 有没有交叉、VCC 有没有接 3V3、GND 有没有接、波特率对不对。" -ForegroundColor Yellow
    exit
}

$hit = 0
for ($i = 0; $i -lt $bytes.Count - 1; $i++) {
    if ($bytes[$i] -eq 0x55 -and $bytes[$i + 1] -eq 0x53) { $hit++ }
}
Write-Host ("共收到 " + $bytes.Count + " 字节，其中 55 53 包头出现 " + $hit + " 次。") -ForegroundColor Green

$max = [Math]::Min($bytes.Count, 176)
Write-Host ("`n前 " + $max + " 字节：")
for ($i = 0; $i -lt $max; $i += 16) {
    $hex = ''
    $ascii = ''
    for ($j = 0; $j -lt 16 -and ($i + $j) -lt $max; $j++) {
        $b = $bytes[$i + $j]
        $hex += ('{0:X2} ' -f $b)
        if ($b -ge 32 -and $b -lt 127) { $ascii += [char]$b } else { $ascii += '.' }
    }
    Write-Host (('{0,6}: ' -f $i) + $hex.PadRight(48) + $ascii)
}
