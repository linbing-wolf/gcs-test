param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$diagRoot=Split-Path -Parent $PSScriptRoot
$diagOut=Join-Path $diagRoot ('evidence/jy901b-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($diagOut)
$diagOldPath=$env:PATH
Push-Location -LiteralPath $diagRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $diagExe='evidence/'+(Split-Path -Leaf $diagOut)+'/verify.exe'
    & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Isrc -Ilookup `
        tests/verify_jy901b_chassis.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $diagExe
    if($LASTEXITCODE -ne 0){throw 'JY901B diagnostic compilation failed'}
    $diagText=& (Join-Path $diagRoot $diagExe) 2>&1;$diagExit=$LASTEXITCODE
    $diagText | Tee-Object -FilePath (Join-Path $diagOut 'test.log')
    if($diagExit -ne 0){throw 'JY901B diagnostic test failed'}
    foreach($extra in @('verify_async_feedback','verify_lateral_hold','verify_sensor_dynamics','verify_lateral_braking','verify_acceleration_log','verify_accel_extra','verify_nine_axis','verify_manual_stop','verify_lateral_path','verify_forward_d','verify_forward_path','verify_gain_schedule')) {
        $extraExe='evidence/'+(Split-Path -Leaf $diagOut)+'/'+$extra+'.exe'
        & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Isrc -Ilookup `
            ('tests/'+$extra+'.c') src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $extraExe
        if($LASTEXITCODE -ne 0){throw 'Additional feedback test compilation failed'}
        $modes=if($extra -eq 'verify_async_feedback'){@(0,1,2)}elseif($extra -eq 'verify_lateral_braking'){@(0..7)}elseif($extra -eq 'verify_accel_extra'){@(0..6)}elseif($extra -eq 'verify_manual_stop'){@(0..3)}elseif($extra -eq 'verify_lateral_path'){@(0..11)}elseif($extra -eq 'verify_forward_path'){@(0..11)}elseif($extra -eq 'verify_gain_schedule'){@(0..12)}else{@(0)}
        foreach($mode in $modes) {
            $extraText=& (Join-Path $diagRoot $extraExe) $mode 2>&1;$extraExit=$LASTEXITCODE
            $extraText | Tee-Object -FilePath (Join-Path $diagOut ($extra+'-'+$mode+'.log'))
            if($extraExit -ne 0){throw ('Additional feedback test failed: '+$extra)}
        }
    }
    $diagHashes=[ordered]@{}
    foreach($diagPath in @('src/main_table_nav.c','src/car_config.h','src/arm_control.c','src/arm_control.h','src/arm_config.h',
        'src/heading_sensor.c','src/heading_sensor.h','src/heading_control.c','src/heading_control.h',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_jy901b_chassis.c','tests/verify_async_feedback.c','tests/verify_lateral_hold.c','tests/verify_sensor_dynamics.c',
        'tests/verify_lateral_braking.c','tests/verify_acceleration_log.c','tests/verify_accel_extra.c','tests/verify_nine_axis.c','tests/verify_manual_stop.c','tests/verify_lateral_path.c','tests/verify_forward_d.c','tests/verify_forward_path.c','tests/verify_gain_schedule.c','tools/verify-diagnostic.ps1')) {
        $diagHashes[$diagPath]=(Get-FileHash -LiteralPath $diagPath -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    @{passed=$true;sourceHashes=$diagHashes;hardwareAccessed=$false} | ConvertTo-Json -Depth 5 |
        Set-Content -LiteralPath (Join-Path $diagOut 'result.json') -Encoding UTF8
    Write-Output $diagOut
} finally {$env:PATH=$diagOldPath;Pop-Location}
