param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$nostopRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$nostopRelative='evidence/no-auto-stop-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$nostopOut=Join-Path $nostopRoot $nostopRelative
[void][IO.Directory]::CreateDirectory($nostopOut)
$nostopOldPath=$env:PATH
Push-Location -LiteralPath $nostopRoot
try {
    $nostopCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $nostopCompiler)+';'+$env:PATH
    foreach($nostopVariant in @(0,1)) {
        $nostopExe=$nostopRelative+'/verify_no_auto_stop_'+$nostopVariant+'.exe'
        & $nostopCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$nostopVariant) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_no_auto_stop.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $nostopExe
        if($LASTEXITCODE -ne 0){throw 'No-auto-stop compilation failed'}
        $nostopText=& (Join-Path $nostopRoot $nostopExe)
        $nostopExit=$LASTEXITCODE
        $nostopText | Tee-Object -FilePath (Join-Path $nostopOut ('result-'+$nostopVariant+'.log'))
        if($nostopExit -ne 0){throw 'No-auto-stop behavior failed'}
    }
    $nostopHashes=[ordered]@{}
    foreach($nostopFile in @('src/main_table_nav.c','src/car_config.h','src/road_map.c','src/structure.h','src/full_route.h',
        'src/arm_control.c','src/station_job.c','src/motion.c','src/motion.h','src/navigation.c','src/navigation.h',
        'lookup/table_data.h','lookup/generation.json','tests/verify_no_auto_stop.c','tests/verify_start_scan.c',
        'tests/verify_station_vision.c','tests/verify_heading.c','tools/verify-no-auto-stop.ps1')) {
        $nostopHashes[$nostopFile]=(Get-FileHash -LiteralPath $nostopFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@(0,1);sourceHashes=$nostopHashes;hardwareAccessed=$false;
        scope='No automatic actuator stop/abort on diagnostics; real motion/status/stale/bus/RX/encoder/CAN mechanism injections, manual ! and planned completion retained; invalid input unresolved, no hardware guarantee'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $nostopOut 'result.json') -Encoding UTF8
    Write-Output $nostopOut
} finally {$env:PATH=$nostopOldPath;Pop-Location}
