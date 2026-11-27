param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$tierRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$tierRelative='evidence/lateral-speed-tiers-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$tierOut=Join-Path $tierRoot $tierRelative
[void][IO.Directory]::CreateDirectory($tierOut)
$tierOldPath=$env:PATH
Push-Location -LiteralPath $tierRoot
try {
    $tierCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $tierCompiler)+';'+$env:PATH
    foreach($tierCorner in @(0,1)) {
        $tierExe=$tierRelative+'/verify_lateral_speed_tiers_'+$tierCorner+'.exe'
        & $tierCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$tierCorner) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_lateral_speed_tiers.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $tierExe
        if($LASTEXITCODE -ne 0){throw 'Lateral speed tiers compilation failed'}
        $tierText=& (Join-Path $tierRoot $tierExe);$tierExit=$LASTEXITCODE
        $tierText | Tee-Object -FilePath (Join-Path $tierOut ('result-'+$tierCorner+'.log'))
        if($tierExit -ne 0){throw 'Lateral speed tiers behavior failed'}
    }
    $tierHashes=[ordered]@{}
    foreach($tierFile in @('src/main_table_nav.c','src/car_config.h','src/motion.c','src/navigation.c','src/heading_control.c',
        'src/arm_config.h','src/arm_control.c','src/station_job.c','src/station_config.h','src/road_map.c','src/structure.h','src/full_route.h',
        'lookup/table_data.h','lookup/generation.json','tests/verify_lateral_speed_tiers.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-lateral-speed-tiers.ps1')) {
        $tierHashes[$tierFile]=(Get-FileHash -LiteralPath $tierFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@(0,1);sourceHashes=$tierHashes;hardwareAccessed=$false;
        scope='Lateral1200mm tiers250/200, forward/backward250 independent of distance, manual250 and lower, mixed wheel caps, F-E-D continuity and A/C corner geometry; model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $tierOut 'result.json') -Encoding UTF8
    Write-Output $tierOut
} finally {$env:PATH=$tierOldPath;Pop-Location}
