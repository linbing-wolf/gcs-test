param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$qrRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$qrOut=Join-Path $qrRoot ('evidence/scanner-recovery-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($qrOut)
$qrCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$qrOldPath=$env:PATH
Push-Location -LiteralPath $qrRoot
try {
    $env:PATH=(Split-Path -Parent $qrCompiler)+';'+$env:PATH
    $qrExe='evidence/'+(Split-Path -Leaf $qrOut)+'/verify_scanner_recovery.exe'
    & $qrCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_scanner_recovery.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $qrExe
    if($LASTEXITCODE -ne 0){throw 'Scanner recovery compilation failed'}
    $qrText=& (Join-Path $qrRoot $qrExe)
    $qrExit=$LASTEXITCODE
    $qrText | Tee-Object -FilePath (Join-Path $qrOut 'result.log')
    if($qrExit -ne 0){throw 'Scanner recovery behavior failed'}
    $qrHashes=[ordered]@{}
    foreach($qrName in @('src/main_table_nav.c','src/car_config.h','vendor/stm32/SYSTEM/Serial.c','tests/entry_host_io.h','tests/verify_entry.c','tests/verify_scanner_recovery.c','tools/verify-scanner-recovery.ps1')) {
        $qrHashes[$qrName]=(Get-FileHash -LiteralPath $qrName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$qrExit;sourceHashes=$qrHashes;scope='Real scanner/route code with simulated USART1 IRQ/TC and peer ACK; not physical scanner verification';hardwareAccessed=$false} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $qrOut 'result.json') -Encoding UTF8
    Write-Output $qrOut
} finally {$env:PATH=$qrOldPath;Pop-Location}
