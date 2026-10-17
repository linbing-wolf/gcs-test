# pack.ps1 —— 批量/单个打包 maix_apps 下的标准 APP
# 用法：
#   pwsh -File pack.ps1                 # 打包所有含 app.yaml 的子目录
#   pwsh -File pack.ps1 -App calib_capture
param([string]$App = "")
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

function Pack-One($dir) {
    Push-Location $dir
    try {
        if (Get-Command maixtool -ErrorAction SilentlyContinue) {
            Write-Host "[i] maixtool release ($dir)"
            maixtool release
            return
        }
        $lines = Get-Content app.yaml
        $id  = ($lines | Select-String -Pattern '^id:\s*(.+)$').Matches[0].Groups[1].Value.Trim()
        $ver = ($lines | Select-String -Pattern '^version:\s*(.+)$').Matches[0].Groups[1].Value.Trim()
        $files = @(); $inFiles = $false
        foreach ($l in $lines) {
            if ($l -match '^files:\s*$') { $inFiles = $true; continue }
            if ($inFiles) {
                if ($l -match '^\s+-\s*(.+)$') { $files += $Matches[1].Trim() }
                elseif ($l -match '^\S') { break }
            }
        }
        if ($files.Count -eq 0) { throw "$dir : app.yaml 无 files 白名单" }
        $missing = $files | Where-Object { -not (Test-Path $_) }
        if ($missing) { throw "$dir : 缺文件 $($missing -join ', ')" }
        New-Item -ItemType Directory -Force dist | Out-Null
        $zip = Join-Path "dist" ("maix-{0}-v{1}.zip" -f $id, $ver)
        Remove-Item $zip -ErrorAction SilentlyContinue
        Compress-Archive -Path $files -DestinationPath $zip -Force
        Write-Host ("[OK] {0} -> {1}" -f $dir, $zip)
    } finally { Pop-Location }
}

if ($App) {
    Pack-One (Resolve-Path $App).Path
} else {
    Get-ChildItem -Directory |
        Where-Object { Test-Path (Join-Path $_.FullName "app.yaml") } |
        ForEach-Object { Pack-One $_.FullName }
}