param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$departureRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$departureRelative='evidence/departure-overlap-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$departureOut=Join-Path $departureRoot $departureRelative
[void][IO.Directory]::CreateDirectory($departureOut)
$departureOldPath=$env:PATH
Push-Location -LiteralPath $departureRoot
try {
    $departureCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $departureCompiler)+';'+$env:PATH
    foreach($departureVariant in @(0,1)) {
        $departureDefines=@('-DARM_HOST_TEST')
        if($departureVariant -eq 1){$departureDefines+=@('-DARM_CUP_EXIT_DESCENT_DELAY_MS=0')}
        $departureExe=$departureRelative+'/verify_departure_overlap_'+$departureVariant+'.exe'
        & $departureCompiler @departureDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_departure_overlap.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $departureExe
        if($LASTEXITCODE -ne 0){throw 'Departure compilation failed'}
        $departureText=& (Join-Path $departureRoot $departureExe)
        $departureExit=$LASTEXITCODE
        $departureText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $departureOut ('result-'+$departureVariant+'.log'))
        if($departureExit -ne 0){throw 'Departure behavior failed'}
        $departureText | Select-Object -Last 5
    }
    $departureHashes=[ordered]@{}
    foreach($departureFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_departure_overlap.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-departure-overlap.ps1')) {
        $departureHashes[$departureFile]=(Get-FileHash -LiteralPath $departureFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$departureExit;sourceHashes=$departureHashes;hardwareAccessed=$false;
        variants=@('current_descent_delay','test_only_zero_descent_delay');
        scope='Both rough batches/all6 ring orders release final block before departure; both raw visits move during released lift; both storage retain actual125mm clearance between rings; all6 departures overlap and station turns precede zero command on A/C; target PWM and axes gate platform release without extra yaw hold; final-storage departure tested separately; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $departureOut 'result.json')
    Write-Output $departureOut
} finally {$env:PATH=$departureOldPath;Pop-Location}
