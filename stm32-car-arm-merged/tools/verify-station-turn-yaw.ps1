param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$yawRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$yawRelative='evidence/station-turn-yaw-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$yawOut=Join-Path $yawRoot $yawRelative
[void][IO.Directory]::CreateDirectory($yawOut)
$yawOldPath=$env:PATH
Push-Location -LiteralPath $yawRoot
try {
    $yawCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $yawCompiler)+';'+$env:PATH
    $yawExe=$yawRelative+'/verify_station_turn_yaw.exe'
    & $yawCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_station_turn_yaw.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $yawExe
    if($LASTEXITCODE -ne 0){throw 'Yaw vision compilation failed'}
    $yawText=& (Join-Path $yawRoot $yawExe)
    $yawExit=$LASTEXITCODE
    $yawText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $yawOut 'result.log')
    if($yawExit -ne 0){throw 'Yaw vision behavior failed'}
    $yawHashes=[ordered]@{}
    foreach($yawFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_station_turn_yaw.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-station-turn-yaw.ps1')) {
        $yawHashes[$yawFile]=(Get-FileHash -LiteralPath $yawFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$yawExit;sourceHashes=$yawHashes;hardwareAccessed=$false;
        scope='A/C both rough180 visits: observation PWM starts before90deg while D waits for original chassis arrival; immediate reply executes once; H/F curved entries still overlap turning; full8-job routes and manual !; no hardware acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $yawOut 'result.json')
    $yawText | Select-Object -Last 5
    Write-Output $yawOut
} finally {$env:PATH=$yawOldPath;Pop-Location}
