# STM32 串口助手（图形界面，零安装）
#
# 用法（二选一）：
#   1. 右键这个文件 → "使用 PowerShell 运行"
#   2. 在本文件夹开 PowerShell，粘贴：
#        powershell -ExecutionPolicy Bypass -File .\tools\serial-gui.ps1
#
# 功能：
#   - 自动列出电脑上的串口，不用手打
#   - 波特率可选，默认 115200
#   - 下面是接收区，中间是发送框，回车即发送
#   - 支持十六进制显示（调 CAN/协议的时候有用）
param([switch]$Test)

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

$ErrorActionPreference = 'Stop'

# ==================== 窗口 ====================
$form               = New-Object System.Windows.Forms.Form
$form.Text          = 'STM32 串口助手'
$form.ClientSize    = [System.Drawing.Size]::new(820, 560)
$form.StartPosition = 'CenterScreen'
$form.MinimumSize   = [System.Drawing.Size]::new(660, 430)

# ==================== 顶部工具条 ====================
$lblPort          = New-Object System.Windows.Forms.Label
$lblPort.Text     = '串口'
$lblPort.Location = [System.Drawing.Point]::new(12, 18)
$lblPort.AutoSize = $true
$form.Controls.Add($lblPort)

$cboPort               = New-Object System.Windows.Forms.ComboBox
$cboPort.Location      = [System.Drawing.Point]::new(50, 14)
$cboPort.Size          = [System.Drawing.Size]::new(110, 25)
$cboPort.DropDownStyle = 'DropDown'
$form.Controls.Add($cboPort)

$btnRefresh          = New-Object System.Windows.Forms.Button
$btnRefresh.Text     = '刷新'
$btnRefresh.Location = [System.Drawing.Point]::new(166, 13)
$btnRefresh.Size     = [System.Drawing.Size]::new(56, 26)
$form.Controls.Add($btnRefresh)

$lblBaud          = New-Object System.Windows.Forms.Label
$lblBaud.Text     = '波特率'
$lblBaud.Location = [System.Drawing.Point]::new(238, 18)
$lblBaud.AutoSize = $true
$form.Controls.Add($lblBaud)

$cboBaud               = New-Object System.Windows.Forms.ComboBox
$cboBaud.Location      = [System.Drawing.Point]::new(286, 14)
$cboBaud.Size          = [System.Drawing.Size]::new(96, 25)
$cboBaud.DropDownStyle = 'DropDown'
@('9600', '19200', '38400', '57600', '115200', '230400', '460800') | ForEach-Object { [void]$cboBaud.Items.Add($_) }
$cboBaud.Text = '115200'
$form.Controls.Add($cboBaud)

$btnOpen          = New-Object System.Windows.Forms.Button
$btnOpen.Text     = '打开串口'
$btnOpen.Location = [System.Drawing.Point]::new(394, 13)
$btnOpen.Size     = [System.Drawing.Size]::new(96, 26)
$form.Controls.Add($btnOpen)

$chkHex          = New-Object System.Windows.Forms.CheckBox
$chkHex.Text     = '十六进制显示'
$chkHex.Location = [System.Drawing.Point]::new(504, 17)
$chkHex.AutoSize = $true
$form.Controls.Add($chkHex)

$btnClear          = New-Object System.Windows.Forms.Button
$btnClear.Text     = '清空'
$btnClear.Location = [System.Drawing.Point]::new(630, 13)
$btnClear.Size     = [System.Drawing.Size]::new(60, 26)
$form.Controls.Add($btnClear)

# ==================== 接收区 ====================
$txtRecv           = New-Object System.Windows.Forms.TextBox
$txtRecv.Location  = [System.Drawing.Point]::new(12, 50)
$txtRecv.Size      = [System.Drawing.Size]::new(796, 418)
$txtRecv.Multiline = $true
$txtRecv.ScrollBars = 'Vertical'
$txtRecv.ReadOnly  = $true
$txtRecv.Font      = New-Object System.Drawing.Font('Consolas', 11)
$txtRecv.BackColor = [System.Drawing.Color]::White
$txtRecv.Anchor    = 'Top,Left,Right,Bottom'
$form.Controls.Add($txtRecv)

# ==================== 发送区 ====================
$txtSend          = New-Object System.Windows.Forms.TextBox
$txtSend.Location = [System.Drawing.Point]::new(12, 484)
$txtSend.Size     = [System.Drawing.Size]::new(650, 25)
$txtSend.Font     = New-Object System.Drawing.Font('Consolas', 11)
$txtSend.Anchor   = 'Bottom,Left,Right'
$form.Controls.Add($txtSend)

$btnSend          = New-Object System.Windows.Forms.Button
$btnSend.Text     = '发送'
$btnSend.Location = [System.Drawing.Point]::new(674, 482)
$btnSend.Size     = [System.Drawing.Size]::new(60, 28)
$btnSend.Anchor   = 'Bottom,Right'
$form.Controls.Add($btnSend)

$chkNewline          = New-Object System.Windows.Forms.CheckBox
$chkNewline.Text     = '末尾加回车换行'
$chkNewline.Location = [System.Drawing.Point]::new(744, 487)
$chkNewline.AutoSize = $true
$chkNewline.Anchor   = 'Bottom,Right'
# 默认勾上：STM32 那边的程序是靠"收到回车"才知道这一行打完了。
# 不勾的话板子只是一直攒字符，永远不把这一行发出去 —— 看起来就是"按了发送没反应"。
$chkNewline.Checked  = $true
$form.Controls.Add($chkNewline)

$lblTip          = New-Object System.Windows.Forms.Label
$lblTip.Location = [System.Drawing.Point]::new(12, 524)
$lblTip.AutoSize = $true
$lblTip.ForeColor = [System.Drawing.Color]::Gray
$form.Controls.Add($lblTip)

# ==================== 逻辑 ====================
$script:sp = $null

function Update-Tip {
    if ($script:sp -and $script:sp.IsOpen) {
        $lblTip.Text = "已连接  $($cboPort.Text)  @  $($cboBaud.Text)"
        $lblTip.ForeColor = [System.Drawing.Color]::Green
    }
    else {
        $lblTip.Text = '提示：波特率两端必须一致（本课用 115200）'
        $lblTip.ForeColor = [System.Drawing.Color]::Gray
    }
}

function Update-Controls {
    $opened = ($script:sp -and $script:sp.IsOpen)
    $cboPort.Enabled    = -not $opened
    $cboBaud.Enabled    = -not $opened
    $btnRefresh.Enabled = -not $opened
    if ($opened) { $btnOpen.Text = '关闭串口' } else { $btnOpen.Text = '打开串口' }
    Update-Tip
}

function Update-Ports {
    $keep = $cboPort.Text
    $cboPort.Items.Clear()
    [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object | ForEach-Object { [void]$cboPort.Items.Add($_) }
    if ($keep -and $cboPort.Items.Contains($keep)) { $cboPort.Text = $keep }
    elseif ($cboPort.Items.Count -gt 0) { $cboPort.SelectedIndex = 0 }
}

function Do-Send {
    if (-not ($script:sp -and $script:sp.IsOpen)) { return }
    $text = $txtSend.Text
    if ($chkNewline.Checked) { $text = $text + "`r`n" }
    if ($text.Length -eq 0) { return }
    # 自己动手把字符串转成 GB2312 字节再发。
    # 不依赖串口对象的 Encoding 属性 —— 万一它没生效，汉字就会被换成 '?'（屏上出现问号就是这么来的）。
    try {
        $bytes = [System.Text.Encoding]::GetEncoding(936).GetBytes($text)
        $script:sp.Write($bytes, 0, $bytes.Length)
    }
    catch { }
}

$btnRefresh.Add_Click({ Update-Ports })

$btnClear.Add_Click({ $txtRecv.Clear() })

$btnOpen.Add_Click({
    if ($script:sp -and $script:sp.IsOpen) {
        try { $script:sp.Close() } catch { }
        $script:sp.Dispose()
        $script:sp = $null
        Update-Controls
        return
    }
    if ([string]::IsNullOrWhiteSpace($cboPort.Text)) {
        [void][System.Windows.Forms.MessageBox]::Show('先选一个串口。如果列表是空的，说明板子没插好。', '提示')
        return
    }
    try {
        # 注意：这里必须先把值取出来存进变量，不能写成 $cbo.Text, [int]$cbo2.Text
        # 那样 PowerShell 会把 [int]$cbo2.Text 解析成"把下拉框对象转成整数"，直接报错。
        $portName = ($cboPort.Text).Trim()
        $baudText = ($cboBaud.Text).Trim()
        $baudRate = [int]$baudText

        $new = New-Object System.IO.Ports.SerialPort -ArgumentList @($portName, $baudRate, 'None', 8, 'One')
        $new.ReadTimeout = 20
        # 关键：.NET 串口默认用 ASCII 发，非 ASCII 的字符全会被换成 '?'，
        # 打中文就发不过去。设成 GB2312（代码页 936）才能把汉字原样发出去 ——
        # 屏那边的字库也是 GB2312，两边对得上。
        $new.Encoding = [System.Text.Encoding]::GetEncoding(936)
        $new.Open()
        $script:sp = $new
        Update-Controls
    }
    catch {
        [void][System.Windows.Forms.MessageBox]::Show("打不开 $($cboPort.Text)：`r`n$($_.Exception.Message)`r`n`r`n常见原因：被别的程序占用 / 口号变了 / 板子没插好", '打不开串口')
    }
})

$btnSend.Add_Click({ Do-Send })

$txtSend.Add_KeyDown({
    param($sender, $e)
    if ($e.KeyCode -eq [System.Windows.Forms.Keys]::Enter) {
        Do-Send
        $e.SuppressKeyPress = $true
    }
})

$timer          = New-Object System.Windows.Forms.Timer
$timer.Interval = 50
$timer.Add_Tick({
    if (-not ($script:sp -and $script:sp.IsOpen)) { return }
    $n = 0
    try { $n = $script:sp.BytesToRead } catch { return }
    if ($n -le 0) { return }

    $buf = New-Object byte[] $n
    try { [void]$script:sp.Read($buf, 0, $n) } catch { return }

    if ($chkHex.Checked) {
        $text = ($buf | ForEach-Object { $_.ToString('X2') }) -join ' '
        $text = $text + ' '
    }
    else {
        # 按 GB2312 解码：纯英文/数字/换行的结果和 ASCII 一样，但板子万一回中文也能显示对
        $text = [System.Text.Encoding]::GetEncoding(936).GetString($buf)
        # WinForms 的文本框只认 CRLF 换行。先把各种换行统一成 \n，再统一成 \r\n，
        # 否则单片机的每行输出会头尾相接挤成一大串。
        $text = $text -replace "`r`n", "`n"
        $text = $text -replace "`r", "`n"
        $text = $text -replace "`n", "`r`n"
    }

    $txtRecv.AppendText($text)
    $txtRecv.SelectionStart = $txtRecv.TextLength
    $txtRecv.ScrollToCaret()
})

$form.Add_FormClosing({
    $timer.Stop()
    if ($script:sp) {
        try { if ($script:sp.IsOpen) { $script:sp.Close() } } catch { }
        $script:sp.Dispose()
    }
})

# ==================== 启动 ====================
Update-Ports
Update-Controls

if ($Test) {
    Write-Output '窗口构建成功（测试模式，不显示窗口）'
    Write-Output ('自动找到的串口：' + (($cboPort.Items | ForEach-Object { $_ }) -join ', '))
    $form.Dispose()
    exit 0
}

$timer.Start()
[void]$form.ShowDialog()
$form.Dispose()
