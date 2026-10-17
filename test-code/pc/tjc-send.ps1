# 给淘晶驰（TJC）串口屏发一条指令（不用装 USART HMI 就能先测通不通）
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File .\tools\tjc-send.ps1 -Port COM5
#     -> 自动扫波特率，把 t0.txt 的文字改成 "hi"
#
#   powershell -ExecutionPolicy Bypass -File .\tools\tjc-send.ps1 -Port COM5 -Cmd 'page 0'
#     -> 换成别的指令
#
#   -Enc gb2312（默认） / -Enc utf8 / -Enc ascii
#     -> 指令正文按哪种编码发出去。发中文要用 gb2312（屏里字库就是这个编码）。
#        ASCII 内容三种编码结果一模一样，所以平时不用管这个参数。
#
# 接线（USB 转 TTL）：
#   USB-TTL TXD -> 屏 RX        USB-TTL RXD -> 屏 TX（不接也能改字，但收不到应答）
#   USB-TTL GND -> 屏 GND      屏的 5V 仍走电源模块绿座
#
# 应答码（屏回 1 个字节）：0x01=成功  0x00=指令无效  0x02=控件不存在  0x03=变量名无效
param(
    [string]$Port = 'COM9',
    [int]$Baud = 0,                       # 0 = 自动扫；也可以用 -Baud 115200 只试一个
    [string]$Cmd = 't0.txt="hi"',
    [string]$Enc = 'gb2312',              # gb2312（默认）/ utf8 / ascii
    [int]$WaitMs = 400                    # 每档波特率等应答的时间
)

$ErrorActionPreference = 'Stop'

$candidates = if ($Baud -gt 0) { @($Baud) } else { @(115200, 9600, 19200, 38400, 57600, 4800, 2400, 1200, 230400) }

# 指令 = 正文（按 -Enc 编码）+ 三个 0xFF 结束符
# 屏里字库是 GB2312：中文必须按 GB2312 发，发 UTF-8 会变乱码。
function Get-CmdEncoding([string]$name) {
    switch ($name.ToLower()) {
        'ascii' { return [System.Text.Encoding]::ASCII }
        'utf8' { return (New-Object System.Text.UTF8Encoding $false) }
        default {
            try { return [System.Text.Encoding]::GetEncoding(936) }   # 936 = GBK ⊇ GB2312
            catch { return [System.Text.Encoding]::ASCII }
        }
    }
}
$encoding = Get-CmdEncoding $Enc
$body = $encoding.GetBytes($Cmd)
$frame = New-Object byte[] ($body.Length + 3)
[Array]::Copy($body, 0, $frame, 0, $body.Length)
$frame[$body.Length] = 0xFF
$frame[$body.Length + 1] = 0xFF
$frame[$body.Length + 2] = 0xFF

function Show-Bytes([byte[]]$b) {
    if ($null -eq $b -or $b.Length -eq 0) { return '(空)' }
    return (($b | ForEach-Object { $_.ToString('X2') }) -join ' ')
}

Write-Host ''
Write-Host "  口 = $Port   编码 = $Enc   指令 = $Cmd" -ForegroundColor Cyan
Write-Host '  （发完抬头看屏：t0 那个文字控件的字应该变成上面指令里的内容）' -ForegroundColor Cyan
Write-Host ''

$hit = $false
foreach ($b in $candidates) {
    $sp = New-Object System.IO.Ports.SerialPort $Port, $b, 'None', 8, 'One'
    $sp.ReadTimeout = 100
    $sp.WriteTimeout = 500
    try { $sp.Open() }
    catch {
        Write-Host "[$b] 打不开 $Port：$($_.Exception.Message)" -ForegroundColor Red
        Write-Host "  常见原因：口选错了 / 被别的程序（串口助手、USART HMI）占着" -ForegroundColor Yellow
        exit 1
    }

    $sp.DiscardInBuffer()
    $sp.DiscardOutBuffer()
    Start-Sleep -Milliseconds 50
    try { $sp.Write($frame, 0, $frame.Length) }
    catch { Write-Host "[$b] 写失败：$($_.Exception.Message)" -ForegroundColor Red }

    $got = New-Object System.Collections.Generic.List[byte]
    $deadline = (Get-Date).AddMilliseconds($WaitMs)
    while ((Get-Date) -lt $deadline) {
        $n = $sp.BytesToRead
        if ($n -gt 0) {
            $buf = New-Object byte[] $n
            $read = $sp.Read($buf, 0, $n)
            for ($i = 0; $i -lt $read; $i++) { $got.Add($buf[$i]) }
            break
        }
        Start-Sleep -Milliseconds 20
    }
    $sp.Close()

    if ($got.Count -gt 0) {
        $reply = $got.ToArray()
        $code = $reply[0]
        $meaning = switch ($code) {
            0x00 { '指令无效（语法错/少了三个 FF）' }
            0x01 { '指令成功 ✅' }
            0x02 { '控件不存在（t0 不在当前页面？）' }
            0x03 { '变量名无效' }
            0x11 { '屏的串口缓冲溢出' }
            default { '未知应答' }
        }
        Write-Host "[$b] 屏回：$(Show-Bytes $reply)  -> $meaning" -ForegroundColor Green
        $hit = $true
        break
    }
    else {
        Write-Host "[$b] 没收到应答" -ForegroundColor DarkGray
    }
}

Write-Host ''
if ($hit) {
    Write-Host '  波特率就是这个，指令也发出去了 —— 抬头看屏。' -ForegroundColor Green
}
else {
    Write-Host '  所有波特率都没收到应答。三种可能：' -ForegroundColor Yellow
    Write-Host '    1) 屏的 TX 没接到 USB-TTL 的 RXD（只接 TXD 也能改字，只是看不到应答）'
    Write-Host '    2) 波特率不在上面这些档里（去问队友工程里设的是多少）'
    Write-Host '    3) 屏的 RX 没接上 / 接反了'
    Write-Host '  —— 不管哪种，先看屏：如果屏上的字变了，说明链路其实是通的。' -ForegroundColor Yellow
}
