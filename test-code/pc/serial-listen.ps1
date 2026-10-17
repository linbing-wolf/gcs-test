# 监听无线调试器的串口（COMx），同时提示用户按 RST，抓开机打印。
# 用法: powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-listen.ps1 -Port COM8 -Seconds 25
param(
    [string]$Port = 'COM8',
    [int]$Baud = 115200,
    [int]$Seconds = 25
)

$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, 'None', 8, 'One'
try { $p.Open() } catch { Write-Host "打不开 $Port : $($_.Exception.Message)"; exit 1 }
$p.DiscardInBuffer()
Write-Host "== 已开始监听 $Port @$Baud，（现在按一下核心板的 RST）=="

$buf = New-Object System.Text.StringBuilder
$sw = [Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    try {
        $c = $p.ReadExisting()
        if ($c.Length -gt 0) { [void]$buf.Append($c) }
    } catch { }
    Start-Sleep -Milliseconds 100
}
$p.Close()

$t = $buf.ToString()
Write-Host "== 监听结束，收到 $($t.Length) 个字符 =="
if ($t.Length -gt 0) { Write-Host $t } else { Write-Host '(一个字都没收到)' }
