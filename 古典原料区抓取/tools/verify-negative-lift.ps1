param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$negativeRoot=Split-Path -Parent $PSScriptRoot
$negativeOut='evidence/negative-lift-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$oldNegativePath=$env:PATH
Push-Location $negativeRoot
try {
    [void][IO.Directory]::CreateDirectory((Join-Path $negativeRoot $negativeOut))
    $env:PATH=(Split-Path $Gcc)+';'+$env:PATH
    $modules=@('src/arm_control.c','src/station_job.c','src/vision_link.c','src/hmi_status.c','src/motion.c',
        'src/navigation.c','src/road_map.c','src/heading_sensor.c','src/heading_control.c','lookup/route_table.c')
    foreach($variant in @('manual','station','no_direct','serial','below_minimum','platform')) {
        $test='tests/verify_negative_lift.c';$defines=@()
        if($variant -in @('station','no_direct','serial')) {
            $test='tests/verify_station_fixed_ring.c'
            $defines=@('-DFIXED_RING_TEST_CALIBRATED','-DSTN_ROUGH_PLACE_LIFT_MM=-10',
                '-DSTN_ROUGH_LIFT_MM=-15','-DSTN_STORAGE_LIFT_MM=-10')
            if($variant -eq 'no_direct'){$defines+='-DARM_DIRECT_TRANSFER_ENABLED=0'}
            if($variant -eq 'serial'){$defines+='-DARM_LIFT_PARALLEL_ENABLED=0'}
        }
        if($variant -eq 'below_minimum'){$defines=@('-DNEGATIVE_STATION_REJECT','-DSTN_ROUGH_PLACE_LIFT_MM=-21')}
        if($variant -eq 'platform'){$test='tests/verify_platform_timing.c'}
        & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup @defines $test @modules -lm -o "$negativeOut/$variant.exe"
        if($LASTEXITCODE -ne 0){throw "$variant compile failed"}
        & "./$negativeOut/$variant.exe" | Tee-Object "$negativeOut/$variant.log"
        if($LASTEXITCODE -ne 0){throw "$variant behavior failed"}
    }
    @{passed=$true;hardwareAccessed=$false;variants=@('manual','station','no_direct','serial','below_minimum','platform')} |
        ConvertTo-Json | Set-Content "$negativeOut/result.json"
    Write-Output $negativeOut
} finally {$env:PATH=$oldNegativePath;Pop-Location}
