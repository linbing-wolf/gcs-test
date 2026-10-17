# 换 main.c 的一键工具：挑版本 -> 拷进 Keil 工程 -> 刷新时间戳 ->（可选）重编 ->（可选）下载
#
# 用法（在项目根目录下执行）：
#   .\tools\switch-main.ps1                      看现有版本 + 板子上现在是哪版
#   .\tools\switch-main.ps1 -List                只列版本
#   .\tools\switch-main.ps1 timer                只换文件，不编译
#   .\tools\switch-main.ps1 motor_selftest -Build        换 + Keil 重编（Rebuild all）
#   .\tools\switch-main.ps1 timer -Build -Download       换 + 重编 + 下载到板子
#
# 为什么要有这个脚本：
#   Windows 的 Copy-Item 会把源文件的时间戳一起带过去，拷进来的 main.c
#   可能比上次编出来的 main.o 还旧，Keil 的增量编译就会认为"没东西要编"，
#   直接跳过 —— 结果下载的还是上一次的 hex，白忙一场。
#   所以这里拷完会强制把时间戳刷成当前时间，并且默认用 -r（Rebuild all）全编。
param(
    [Parameter(Position = 0)][string]$Name,
    [switch]$List,
    [switch]$Status,
    [switch]$Build,
    [switch]$Download,
    [switch]$Incremental,
    [switch]$Force,
    [string]$Project = 'C:\Users\Kobe\Desktop\stm32-test\USER\Template.uvprojx',
    [string]$Keil    = 'D:\keil5\UV4\UV4.exe'
)

$ErrorActionPreference = 'Stop'

$ToolsDir = $PSScriptRoot
$UserDir  = Split-Path -Parent $Project
$ObjDir   = Join-Path (Split-Path -Parent $UserDir) 'OBJ'
$DestMain = Join-Path $UserDir 'main.c'
$HexFile  = Join-Path $ObjDir 'Template.hex'

function Get-Title([string]$path) {
    try {
        $lines = Get-Content -LiteralPath $path -TotalCount 8 -Encoding UTF8
    } catch {
        return '(读不到标题)'
    }
    foreach ($l in $lines) {
        $t = $l -replace '^\s*/\*+\s*=*\s*', '' -replace '^\s*\*+\s*', '' -replace '\s*=+\s*$', ''
        if ($t -match 'STM32') { return $t.Trim() }
    }
    return '(无标题)'
}

function Get-Versions {
    Get-ChildItem -LiteralPath $ToolsDir -Filter 'main_*.c' -File | Sort-Object Name | ForEach-Object {
        [pscustomobject]@{
            Name  = ($_.Name -replace '^main_', '') -replace '\.c$', ''
            File  = $_.FullName
            Bytes = $_.Length
            Title = Get-Title $_.FullName
        }
    }
}

function Show-Status {
    Write-Host ''
    Write-Host '【板子工程里现在是什么】'
    if (Test-Path -LiteralPath $DestMain) {
        $m = Get-Item -LiteralPath $DestMain
        $h = (Get-FileHash -LiteralPath $DestMain -Algorithm MD5).Hash
        Write-Host ("  main.c        : {0} 字节, 时间 {1}" -f $m.Length, $m.LastWriteTime.ToString('MM-dd HH:mm:ss'))
        Write-Host ("  内容          : {0}" -f (Get-Title $DestMain))
        $match = Get-ChildItem -LiteralPath $ToolsDir -Filter 'main_*.c' -File |
                 Where-Object { (Get-FileHash -LiteralPath $_.FullName -Algorithm MD5).Hash -eq $h }
        if ($match) {
            Write-Host ("  等于 tools 里的: {0}" -f $match[0].Name)
        } else {
            Write-Host '  在 tools 里找不到一模一样的版本（可能是手改过的）'
        }
    } else {
        Write-Host ("  {0} 不存在" -f $DestMain)
    }
    if (Test-Path -LiteralPath $HexFile) {
        $x = Get-Item -LiteralPath $HexFile
        Write-Host ("  已编译的 hex  : 时间 {0}  ({1} 字节)" -f $x.LastWriteTime.ToString('MM-dd HH:mm:ss'), $x.Length)
        Write-Host '  ↑ 板子上跑的很可能就是它。hex 比 main.c 旧 = 下载的还是上一版'
    } else {
        Write-Host '  还没有编译过（OBJ\Template.hex 不存在）'
    }
}

function Show-Versions($versions) {
    Write-Host ''
    Write-Host '【tools 里现成的版本】'
    Write-Host '  名字                大小     说明'
    Write-Host '  ------------------  -------  ------------------------------------------'
    foreach ($v in $versions) {
        Write-Host ("  {0,-18}  {1,7}  {2}" -f $v.Name, $v.Bytes, $v.Title)
    }
    Write-Host ''
    Write-Host '  用法: .\tools\switch-main.ps1 <名字> [-Build] [-Download]'
}

function Assert-KeilUsable([string]$what) {
    if (-not (Test-Path -LiteralPath $Keil)) { throw "找不到 Keil 命令行: $Keil（用 -Keil 指定路径）" }
    if ($Force) { return }
    $running = Get-Process -Name UV4 -ErrorAction SilentlyContinue
    if ($running) {
        throw "Keil (UV4) 正开着，$what 会失败。先把 Keil 界面关掉，或者用界面上那个 Rebuild 按钮手动编。"
    }
}

function Invoke-Keil([string]$switchArgs, [string]$label) {
    $log = Join-Path $ObjDir ("switch-" + $label + ".log")
    if (-not (Test-Path -LiteralPath $ObjDir)) { New-Item -ItemType Directory -Force -Path $ObjDir | Out-Null }
    if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log -Force }

    Write-Host ("  [{0}] 正在跑 Keil: UV4 {1}" -f $label, $switchArgs)
    $p = Start-Process -FilePath $Keil -ArgumentList @($switchArgs, $Project, '-o', $log) `
                       -Wait -PassThru -NoNewWindow

    $logText = ''
    if (Test-Path -LiteralPath $log) { $logText = Get-Content -LiteralPath $log -Raw -Encoding Default }

    $errLine = ($logText -split "`r?`n" | Where-Object { $_ -match 'Error\(s\)' } | Select-Object -First 1)
    $compiled = ($logText -split "`r?`n" | Where-Object { $_ -match 'compiling (main\.c|.*\.c)' } | Measure-Object).Count
    Write-Host ("  [{0}] Keil 返回码 {1}；真正编译了 {2} 个文件" -f $label, $p.ExitCode, $compiled)
    if ($errLine) { Write-Host ("  [{0}] {1}" -f $label, $errLine.Trim()) }
    if ($compiled -eq 0) {
        Write-Host ("  [{0}] ⚠ 一个文件都没编译 —— 说明 Keil 认为不用编，下载的还是老 hex" -f $label)
    }
    # Keil 返回码: 0 = 无错无警告, 1 = 只有警告, >=2 = 有错
    if ($p.ExitCode -ge 2) { throw "Keil $label 失败（返回码 $($p.ExitCode)，完整日志 $log）" }
}

$versions = Get-Versions

if ($List) { Show-Versions $versions; exit 0 }
if (-not $Name) { Show-Versions $versions; Show-Status; exit 0 }

# 允许写成 timer / main_timer / main_timer.c 三种
$want = ($Name -replace '^main_', '') -replace '\.c$', ''
$target = $versions | Where-Object { $_.Name -eq $want } | Select-Object -First 1
if (-not $target) {
    Write-Host "没有叫 '$Name' 的版本。现成的有：" -ForegroundColor Yellow
    $versions | ForEach-Object { Write-Host ("  " + $_.Name) }
    exit 1
}

Show-Status
Write-Host ''
Write-Host ("【要换成】{0}  ({1} 字节)" -f $target.Name, $target.Bytes)
Write-Host ("         {0}" -f $target.Title)

$srcHash = (Get-FileHash -LiteralPath $target.File -Algorithm MD5).Hash
$dstHash = if (Test-Path -LiteralPath $DestMain) { (Get-FileHash -LiteralPath $DestMain -Algorithm MD5).Hash } else { '' }
if ($srcHash -eq $dstHash) {
    Write-Host '  （内容和现在这版一模一样，只是重新刷新时间戳 + 重编）'
}

Copy-Item -LiteralPath $target.File -Destination $DestMain -Force
(Get-Item -LiteralPath $DestMain).LastWriteTime = Get-Date

$afterHash = (Get-FileHash -LiteralPath $DestMain -Algorithm MD5).Hash
if ($afterHash -ne $srcHash) { throw "拷贝后校验不一致，别继续" }
$m = Get-Item -LiteralPath $DestMain
Write-Host ("  已拷贝，校验一致（MD5 {0}）" -f $afterHash)
Write-Host ("  时间戳已刷成 {0}（这一步就是让 Keil 认得出文件变了）" -f $m.LastWriteTime.ToString('MM-dd HH:mm:ss'))

$hexBefore = if (Test-Path -LiteralPath $HexFile) { (Get-Item -LiteralPath $HexFile).LastWriteTime } else { $null }

if ($Build) {
    Assert-KeilUsable '编译'
    $keilSwitch = if ($Incremental) { '-b' } else { '-r' }   # -r = Rebuild all，无视时间戳全编
    Invoke-Keil $keilSwitch 'build'

    if (Test-Path -LiteralPath $HexFile) {
        $hexAfter = (Get-Item -LiteralPath $HexFile).LastWriteTime
        if ($hexBefore -and $hexAfter -le $hexBefore) {
            Write-Host '  ⚠ hex 时间没变，说明其实没重新生成，先别下载' -ForegroundColor Yellow
        } else {
            Write-Host ("  hex 已更新: {0}  ({1} 字节)" -f $hexAfter.ToString('MM-dd HH:mm:ss'), (Get-Item $HexFile).Length)
        }
    }
}

if ($Download) {
    if (-not $Build) {
        Write-Host '  注意: 没有 -Build，下载的是现在这个 hex（可能还是上一版的）' -ForegroundColor Yellow
    }
    Assert-KeilUsable '下载'
    Invoke-Keil '-f' 'download'
    Write-Host '  下载命令已执行。看串口确认板子上的内容换了没有'
}

Write-Host ''
Write-Host '【下一步】抓串口看板子到底在跑什么：'
Write-Host '  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\serial-capture.ps1 -Port COM7 -Seconds 10 -Out .\_check\check.log'
