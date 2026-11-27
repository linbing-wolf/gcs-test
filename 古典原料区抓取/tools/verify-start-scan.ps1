param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$startRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$startOut=Join-Path $startRoot ('evidence/start-scan-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($startOut)
$startCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$startOldPath=$env:PATH
Push-Location -LiteralPath $startRoot
try {
    $env:PATH=(Split-Path -Parent $startCompiler)+';'+$env:PATH
    foreach($startContinuous in @(0,1)) {
    $startExe='evidence/'+(Split-Path -Leaf $startOut)+'/verify_start_scan_'+$startContinuous+'.exe'
    & $startCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$startContinuous) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_start_scan.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $startExe
    if($LASTEXITCODE -ne 0){throw 'Start scan test compilation failed'}
    $startText=& (Join-Path $startRoot $startExe)
    $startExit=$LASTEXITCODE
    $startText | Tee-Object -FilePath (Join-Path $startOut ('result-'+$startContinuous+'.log'))
    if($startExit -ne 0){throw 'Start scan behavior test failed'}
    }
    $startHashes=[ordered]@{}
    foreach($startName in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_config.h',
        'src/vision_link.c','src/arm_control.c','src/arm_config.h','src/heading_control.c','src/heading_sensor.c',
        'src/road_map.c','src/structure.h','src/full_route.h','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/vision_movexy_cases.h','tests/verify_start_scan.c','tools/verify-start-scan.ps1')) {
        $startHashes[$startName]=(Get-FileHash -LiteralPath $startName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$startExit;variants=@(0,1);sourceHashes=$startHashes;hardwareAccessed=$false;
        scope='Segmented/continuous variants: original A/C with manual-X heading; early/preloaded QR continuously bypasses B, missing QR stops at shifted B; E tangent exit and no initial turn; stop/fault and full A/C jobs; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $startOut 'result.json') -Encoding UTF8
    Write-Output $startOut
} finally {$env:PATH=$startOldPath;Pop-Location}
