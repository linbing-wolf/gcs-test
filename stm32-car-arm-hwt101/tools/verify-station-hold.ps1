param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$holdRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$holdOut=Join-Path $holdRoot ('evidence/station-hold-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($holdOut)
$holdOldPath=$env:PATH
Push-Location -LiteralPath $holdRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    foreach($holdDefault in @(0,1)) {
        $holdExe='evidence/'+(Split-Path -Leaf $holdOut)+"/$holdDefault.exe"
        & $Gcc -DARM_HOST_TEST ("-DCAR_STATION_HOLD_DEFAULT_ON=$holdDefault") -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_station_hold.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $holdExe
        if($LASTEXITCODE -ne 0){throw 'Station hold compilation failed'}
        $holdText=& (Join-Path $holdRoot $holdExe) 2>&1;$holdExit=$LASTEXITCODE
        $holdText | Tee-Object -FilePath (Join-Path $holdOut "$holdDefault.log")
        if($holdExit -ne 0){throw 'Station hold verification failed'}
    }
    $holdHashes=[ordered]@{}
    foreach($holdName in @('src/main_table_nav.c','src/car_config.h','src/arm_config.h','src/arm_control.c','src/station_job.c',
        'src/station_config.h','src/heading_control.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/verify_raw_plate.c','tests/verify_station_hold.c','tools/verify-station-hold.ps1')) {
        $holdHashes[$holdName]=(Get-FileHash -LiteralPath $holdName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$holdHashes;defaultVariants=@(0,1);hardwareAccessed=$false;
        scope='Startup map reference, bounded work-station rotation, D/H station shift yaw correction, wheel-stop interlock and departure'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $holdOut 'result.json') -Encoding UTF8
    Write-Output $holdOut
} finally {$env:PATH=$holdOldPath;Pop-Location}
