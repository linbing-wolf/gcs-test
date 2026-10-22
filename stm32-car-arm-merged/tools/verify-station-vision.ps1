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
    foreach($flowMock in @(0,1,2,3,4,5,6,7,8,9)) {
        $checkDefines=@()
        $checkTest='tests/verify_station_vision.c'
        $stationSource=@('src/station_job.c')
        if($flowMock -eq 1){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=20')}
        if($flowMock -eq 2){$checkDefines=@('-DSTN_ROUGH_SLIDE_MM=0','-DSTN_ROUGH_LIFT_MM=0','-DSTN_ROUGH_PLACE_LIFT_MM=0','-DSTN_ROUGH_YAW_DEG=0.0f')}
        if($flowMock -eq 3){$checkDefines=@('-DSTN_ROUGH_PLATFORM_TAKE_READY=0')}
        if($flowMock -eq 4){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=0')}
        if($flowMock -eq 5){$checkDefines=@('-DSTN_STORAGE_PLACE_READY=0')}
        if($flowMock -eq 6){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=130')}
        if($flowMock -eq 7){$checkDefines=@('-DSTN_STORAGE_LAYER_LIFT_MM=131')}
        if($flowMock -eq 8){$checkDefines=@('-DSTN_VISION_MM_PER_PX_X1000=500');$checkTest='tests/verify_vision_gain.c'}
        if($flowMock -eq 9){$checkTest='tests/verify_station_compaction.c';$stationSource=@()}
        $checkExe='evidence/'+(Split-Path -Leaf $checkOut)+'/verify_station_vision_'+$flowMock+'.exe'
        & $checkCompiler -DARM_HOST_TEST @checkDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            $checkTest src/arm_control.c @stationSource src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c src/road_map.c `
            src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $checkExe
        if($LASTEXITCODE -ne 0){throw 'Station/vision test compilation failed'}
        $checkText=& (Join-Path $checkRoot $checkExe)
        $checkExit=$LASTEXITCODE
        $checkText | Tee-Object -FilePath (Join-Path $checkOut ('result-'+$flowMock+'.log'))
        if($checkExit -ne 0){throw 'Station/vision behavior test failed'}
        $checkVariants+=@{testOnlyOverrides=[bool]$flowMock;configuration=@('production_first5_second65_full_flow','mock_layer20_full_flow','ready_without_pose','rough_platform_take_not_ready','layer_height_unknown','storage_not_ready','mock_layer130_upper_boundary','mock_layer131_exceeds_travel','test_only_half_mm_per_pixel','station_semantic_compaction')[$flowMock];defines=$checkDefines;exitCode=$checkExit;output=$checkText}
    }
    $checkHashes=[ordered]@{}
    foreach($name in @('src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','src/vision_link.h','src/hmi_status.c','src/hmi_status.h','src/arm_control.c','src/arm_control.h','src/arm_config.h','tests/vision_host_io.h','src/main_table_nav.c','src/car_config.h','src/full_route.h','src/structure.h',
        'src/heading_control.c','src/heading_sensor.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/verify_station_compaction.c','tests/verify_vision_gain.c','tools/verify-station-vision.ps1')) {
        $checkHashes[$name]=(Get-FileHash -LiteralPath (Join-Path $checkRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$checkExit;scope='r49 standalone vision debug default/D,E,F1-4; no task required, existing pose/platform declarations retained; observation yaw only then same MOVE/MOVEY FIFO; completion holds pose with no grip/index/return/route; explicit off/! and bus-off cancel; task/auto policy preserved; current user acceleration/claw timing retained; r47 D/E/F upper-commanded signed-pixel MOVE/MOVEY FIFO, full magnitude and optional plus, unchanged1px/mm and half-gain variant, positive MOVE backward and positive MOVEY left; each discrete command retained in receive order, paired/multiple rounds and zero cannot overwrite earlier commands; DISC/STACK report coordinates/residual never generate motion or veto declared completion; new STACK,x,y,dx,ok plus legacy two-field terminal; terminal waits for queue and active encoder/IMU move/relock, postterminal commands ignored; missing launch feedback retains head, explicit cancellation clears queue;32-entry queue overflow logged explicitly; F corrected position retained after placement; no automatic inverse y/x return; nine signed/zero pairs, pickup100/place65 and all ring/color permutations; D once per rough visit, E once first storage, F once per second-storage material, no R/READY/resend; full A/C eight-job routes,13960mm and visited-road lateral strategy; r46 axes4000 and current configured acceleration/claw timing, fresh completed-feedback reuse and departure final-rotation overlap retained; layer/travel/calibration variants, axis status/RPM log-only, real position completion, chassis faults and emergency retained; no hardware accessed';sourceHashes=$checkHashes;variants=$checkVariants} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $checkOut 'result.json') -Encoding UTF8
    Write-Output $checkOut
} finally {$env:PATH=$checkOldPath;Pop-Location}
