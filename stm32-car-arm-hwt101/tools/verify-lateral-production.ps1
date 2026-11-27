param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$lateralRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$lateralOut=Join-Path $lateralRoot ('evidence/lateral-production-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($lateralOut)
$lateralCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$lateralOldPath=$env:PATH
Push-Location -LiteralPath $lateralRoot
try {
    $env:PATH=(Split-Path -Parent $lateralCompiler)+';'+$env:PATH
    foreach($lateralCorner in @(0,1)) {
        $lateralExe='evidence/'+(Split-Path -Leaf $lateralOut)+'/'+$lateralCorner+'.exe'
        & $lateralCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$lateralCorner) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_lateral_production.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $lateralExe
        if($LASTEXITCODE -ne 0){throw 'Lateral production compilation failed'}
        $lateralText=& (Join-Path $lateralRoot $lateralExe) 2>&1
        $lateralExit=$LASTEXITCODE
        $lateralText | Tee-Object -FilePath (Join-Path $lateralOut ($lateralCorner.ToString()+'.log'))
        if($lateralExit -ne 0){throw 'Lateral production behavior failed'}
    }
    $lateralHashes=[ordered]@{}
    foreach($lateralName in @('src/main_table_nav.c','src/car_config.h','src/motion.c','src/arm_control.c','src/arm_config.h',
        'src/station_job.c','src/station_config.h','src/vision_link.c','src/heading_control.c','src/heading_sensor.c',
        'src/road_map.c','src/structure.h','src/full_route.h','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/verify_corner_route.c','tests/verify_lateral_production.c','tools/verify-lateral-production.ps1')) {
        $lateralHashes[$lateralName]=(Get-FileHash -LiteralPath $lateralName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$lateralHashes;variants=@(0,1);hardwareAccessed=$false;
        scope='Production lateral calibration/150RPM-per-second acceleration matches test; forward/arc/braking and race250/200RPM tiers retained; feed-forward and CPOS inverse consistent'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $lateralOut 'result.json') -Encoding UTF8
    Write-Output $lateralOut
} finally {$env:PATH=$lateralOldPath;Pop-Location}
