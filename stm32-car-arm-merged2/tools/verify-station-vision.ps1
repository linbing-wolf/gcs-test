param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$checkRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$checkOut=Join-Path $checkRoot ('evidence/station-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($checkOut)
$checkCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$checkOldPath=$env:PATH
Push-Location -LiteralPath $checkRoot
try {
    $env:PATH=(Split-Path -Parent $checkCompiler)+';'+$env:PATH
    $checkVariants=@()
    foreach($flowMock in @(0,1,2,3)) {
        $checkDefines=@()
        if($flowMock -eq 1){$checkDefines=@('-DSTN_ROUGH_PLACE_READY=1','-DSTN_ROUGH_SLIDE_MM=40','-DSTN_ROUGH_LIFT_MM=90','-DSTN_ROUGH_YAW_DEG=120.0f','-DSTN_STORAGE_PLACE_READY=1','-DSTN_PLATFORM_TAKE_READY=1')}
        if($flowMock -eq 2){$checkDefines=@('-DSTN_ROUGH_PLACE_READY=1','-DSTN_PLATFORM_TAKE_READY=1')}
        if($flowMock -eq 3){$checkDefines=@('-DSTN_ROUGH_PLACE_READY=1','-DSTN_ROUGH_SLIDE_MM=40','-DSTN_ROUGH_LIFT_MM=90','-DSTN_ROUGH_YAW_DEG=120.0f')}
        $checkExe='evidence/'+(Split-Path -Leaf $checkOut)+'/verify_station_vision_'+$flowMock+'.exe'
        & $checkCompiler -DARM_HOST_TEST @checkDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_station_vision.c src/arm_control.c src/station_job.c src/vision_link.c src/motion.c src/navigation.c src/road_map.c `
            src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $checkExe
        if($LASTEXITCODE -ne 0){throw 'Station/vision test compilation failed'}
        $checkText=& (Join-Path $checkRoot $checkExe)
        $checkExit=$LASTEXITCODE
        $checkText | Tee-Object -FilePath (Join-Path $checkOut ('result-'+$flowMock+'.log'))
        if($checkExit -ne 0){throw 'Station/vision behavior test failed'}
        $checkVariants+=@{testOnlyReadyFlagsEnabled=[bool]$flowMock;configuration=@('production','mock_calibrated_rough_full_flow','ready_without_pose','platform_take_not_ready')[$flowMock];defines=$checkDefines;exitCode=$checkExit;output=$checkText}
    }
    $checkHashes=[ordered]@{}
    foreach($name in @('src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','src/vision_link.h','src/arm_control.c','src/arm_control.h','src/arm_config.h','tests/vision_host_io.h','src/main_table_nav.c','src/car_config.h','src/full_route.h','src/structure.h',
        'src/heading_control.c','src/heading_sensor.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tools/verify-station-vision.ps1')) {
        $checkHashes[$name]=(Get-FileHash -LiteralPath (Join-Path $checkRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$checkExit;scope='actual READY=0 guards plus test-only READY overrides for full mock job flow; placeholder poses are NOT calibration; no hardware accessed';sourceHashes=$checkHashes;variants=$checkVariants} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $checkOut 'result.json') -Encoding UTF8
    Write-Output $checkOut
} finally {$env:PATH=$checkOldPath;Pop-Location}
