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
    [ordered]@{passed=$true;exitCode=$checkExit;scope='station target compaction deletes only redundant completed-target steps for rough/storage; initial targets and all state-changing grip/motion/vision steps retained; raw/manual APIs unchanged,600/200ms actual interpolation and real feedback unchanged; direct ring1-3/ring3-1 single300mm index (neighbors150mm), all ring/color permutations, no intermediate ring2 stop, physical endpoints and high/retracted arm retained; D/E MOVEY positive LEFT/negative RIGHT (operator direction correction), signed/full magnitude/zero, paired11/-87, independent displacement, per-axis latest/zero supersede, no drop while moving, early DISC retains both axes, missing launch feedback keeps pending until recovery, lateral emergency,0.5mm/px; STACK x/y: x drives front/back then y drives lateral, ypositive RIGHT/ynegative LEFT at240; nine signed/zero cases, inverse y/x return, lateral emergency and half gain; MOVE pixels with operator-assumed1px/mm configurable scaling; second storage skips E completely and starts F,color/STACK per material; first storage retains E, no returned height, one center correction before100 pickup/65 place, exact colors/CRC and cancellation/late frames; full signed pixel magnitude and plus supported; DISC coordinate consistency veto removed: D/E accept reported ok1 despite dx!=cx-320, including386/177/71 and420/620 offsets; in-motion terminal reply waits for completion then reaches platform100 grip; simultaneous platform/yaw during rough/storage cup selection and final parking; B turn accepts QR early, first valid code latched, departure waits for correct route heading without redundant scan interlock; rough D once / first storage E once / second storage no E at entry ring2 yaw126.5; no target/return D/E requests; single D/E session, continuous MOVE until final DISC; fake peer streams every1200ms without new requests, exact one D per alignment; no R/READY cancellation, latest per-axis MOVE/MOVEY retained during motion, terminal DISC deferred until both axes complete; independent +/-9mm geometry, full commanded millimeter corrections, transport and camera/fault limits; rough pickup100/place5/retrieve5; storage first pickup100/place5/yaw126.5, second pickup100/place65/yaw126.5; wheel log-only policy retained; persistent signed5/7 axis RPM is logged only across full A/C routes and station grip/departure; late PICK/DISC, UART/FIFO errors and nominal station/vision budgets retain requests/tasks; query gaps retain stage and recover; production layer60 completes both batches; layer20/130 mocks complete routes, layer0 unknown and layer131 overtravel refuse; no hardware accessed';sourceHashes=$checkHashes;variants=$checkVariants} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $checkOut 'result.json') -Encoding UTF8
    Write-Output $checkOut
} finally {$env:PATH=$checkOldPath;Pop-Location}
