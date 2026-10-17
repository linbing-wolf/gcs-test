# pack.ps1 —— 打包 MaixCAM 标准 APP（优先 maixtool，兜底按 app.yaml 白名单 zip）
# 用法： pwsh -File pack.ps1
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (Get-Command maixtool -ErrorAction SilentlyContinue) {
    Write-Host "[i] 检测到 maixtool，使用官方打包：maixtool release"
    maixtool release
    exit $LASTEXITCODE
}

Write-Host "[i] 未安装 maixtool，按 app.yaml 的 files 白名单打包（建议: pip install maixtool）"

# 解析 app.yaml：id / version / files 白名单
$lines = Get-Content app.yaml
$id  = ($lines | Select-String -Pattern '^id:\s*(.+)$').Matches[0].Groups[1].Value.Trim()
$ver = ($lines | Select-String -Pattern '^version:\s*(.+)$').Matches[0].Groups[1].Value.Trim()

$files = @()
$inFiles = $false
foreach ($l in $lines) {
    if ($l -match '^files:\s*$') { $inFiles = $true; continue }
    if ($inFiles) {
        if ($l -match '^\s+-\s*(.+)$') { $files += $Matches[1].Trim() }
        elseif ($l -match '^\S') { break }
    }
}
if ($files.Count -eq 0) { throw "app.yaml 里没解析到 files 白名单" }

$missing = $files | Where-Object { -not (Test-Path $_) }
if ($missing) { throw ("app.yaml 列的文件不存在: " + ($missing -join ", ")) }

New-Item -ItemType Directory -Force dist | Out-Null
$zip = Join-Path "dist" ("maix-{0}-v{1}.zip" -f $id, $ver)
Remove-Item $zip -ErrorAction SilentlyContinue
Compress-Archive -Path $files -DestinationPath $zip -Force
Write-Host ("[OK] 已打包: {0}  (含 {1} 项)" -f $zip, $files.Count)
$files | ForEach-Object { Write-Host "     - $_" }