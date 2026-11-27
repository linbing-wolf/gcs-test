param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$yawRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$yawRelative='evidence/chassis-no-hold-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$yawOut=Join-Path $yawRoot $yawRelative
[void][IO.Directory]::CreateDirectory($yawOut)
$yawOldPath=$env:PATH
Push-Location -LiteralPath $yawRoot
try {
    $yawCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $yawCompiler)+';'+$env:PATH
    $yawExe=$yawRelative+'/verify_chassis_no_hold.exe'
    & $yawCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_chassis_no_hold.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $yawExe
    if($LASTEXITCODE -ne 0){throw 'Chassis hold compilation failed'}
    $yawText=& (Join-Path $yawRoot $yawExe)
    $yawExit=$LASTEXITCODE
    $yawText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $yawOut 'result.log')
    if($yawExit -ne 0){throw 'Chassis hold behavior failed'}
    $yawHashes=[ordered]@{}
    foreach($yawFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_chassis_no_hold.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-chassis-no-hold.ps1')) {
        $yawHashes[$yawFile]=(Get-FileHash -LiteralPath $yawFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$yawExit;sourceHashes=$yawHashes;hardwareAccessed=$false;
        scope='Production chassis/heading/interlock: no extra stable hold, first valid new sample and post-zero feedback completion, old/stale/missing feedback and high rate still block, mechanical200/40/50ms preserved, both rough180 visits on A/C wait for chassis arrival before observation yaw, manual emergency; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $yawOut 'result.json')
    $yawText | Select-Object -Last 5
    Write-Output $yawOut
} finally {$env:PATH=$yawOldPath;Pop-Location}
