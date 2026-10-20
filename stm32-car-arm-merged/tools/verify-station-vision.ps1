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
    foreach($flowMock in @(0,1,2,3,4,5,6,7)) {
        $checkDefines=@()
        if($flowMock -eq 1){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=20')}
        if($flowMock -eq 2){$checkDefines=@('-DSTN_ROUGH_SLIDE_MM=0','-DSTN_ROUGH_LIFT_MM=0','-DSTN_ROUGH_PLACE_LIFT_MM=0','-DSTN_ROUGH_YAW_DEG=0.0f')}
        if($flowMock -eq 3){$checkDefines=@('-DSTN_ROUGH_PLATFORM_TAKE_READY=0')}
        if($flowMock -eq 4){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=0')}
        if($flowMock -eq 5){$checkDefines=@('-DSTN_STORAGE_PLACE_READY=0')}
        if($flowMock -eq 6){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=130')}
        if($flowMock -eq 7){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=131')}
        $checkExe='evidence/'+(Split-Path -Leaf $checkOut)+'/verify_station_vision_'+$flowMock+'.exe'
        & $checkCompiler -DARM_HOST_TEST @checkDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_station_vision.c src/arm_control.c src/station_job.c src/vision_link.c src/motion.c src/navigation.c src/road_map.c `
            src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $checkExe
        if($LASTEXITCODE -ne 0){throw 'Station/vision test compilation failed'}
        $checkText=& (Join-Path $checkRoot $checkExe)
        $checkExit=$LASTEXITCODE
        $checkText | Tee-Object -FilePath (Join-Path $checkOut ('result-'+$flowMock+'.log'))
        if($checkExit -ne 0){throw 'Station/vision behavior test failed'}
        $checkVariants+=@{testOnlyOverrides=[bool]$flowMock;configuration=@('production_first5_second65_full_flow','mock_layer20_full_flow','ready_without_pose','rough_platform_take_not_ready','layer_height_unknown','storage_not_ready','mock_layer130_upper_boundary','mock_layer131_exceeds_travel')[$flowMock];defines=$checkDefines;exitCode=$checkExit;output=$checkText}
    }
    $checkHashes=[ordered]@{}
    foreach($name in @('src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','src/vision_link.h','src/arm_control.c','src/arm_control.h','src/arm_config.h','tests/vision_host_io.h','src/main_table_nav.c','src/car_config.h','src/full_route.h','src/structure.h',
        'src/heading_control.c','src/heading_sensor.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tools/verify-station-vision.ps1')) {
        $checkHashes[$name]=(Get-FileHash -LiteralPath (Join-Path $checkRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$checkExit;scope='simultaneous platform/yaw during rough/storage cup selection and final parking; B turn accepts QR early, first valid code latched, departure waits for correct route heading without redundant scan interlock; rough D once / storage E once per visit at entry ring2 yaw125; no target/return requests; single D/E session, continuous MOVE until final DISC; fake peer streams every300ms without new requests, exact one D per alignment; no R/READY cancellation, in-motion samples dropped, terminal DISC deferred until stopped; independent +/-9mm geometry, fresh3mm corrections, transport and camera/fault limits; rough pickup100/place5/retrieve5; storage first pickup100/place5/yaw125, second pickup110/place65/yaw125; wheel log-only policy retained; persistent signed5/7 axis RPM is logged only across full A/C routes and station grip/departure; late PICK/DISC, UART/FIFO errors and nominal station/vision budgets retain requests/tasks; query gaps retain stage and recover; production layer60 completes both batches; layer20/130 mocks complete routes, layer0 unknown and layer131 overtravel refuse; no hardware accessed';sourceHashes=$checkHashes;variants=$checkVariants} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $checkOut 'result.json') -Encoding UTF8
    Write-Output $checkOut
} finally {$env:PATH=$checkOldPath;Pop-Location}
