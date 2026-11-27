param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$stallRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$stallRelative='evidence/wheel-stall-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$stallOut=Join-Path $stallRoot $stallRelative
[void][IO.Directory]::CreateDirectory($stallOut)
$stallOldPath=$env:PATH
Push-Location -LiteralPath $stallRoot
try {
    $stallCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $stallCompiler)+';'+$env:PATH
    $stallExe=$stallRelative+'/verify_wheel_stall_log.exe'
    & $stallCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_wheel_stall_log.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $stallExe
    if($LASTEXITCODE -ne 0){throw 'Wheel stall focused compilation failed'}
    $stallText=& (Join-Path $stallRoot $stallExe);$stallExit=$LASTEXITCODE
    $stallText | Tee-Object -FilePath (Join-Path $stallOut 'result.log')
    if($stallExit -ne 0){throw 'Wheel stall focused behavior failed'}
    $stallHashes=[ordered]@{}
    foreach($stallFile in @('src/main_table_nav.c','src/car_config.h','src/arm_control.c','src/arm_config.h',
        'src/station_job.c','src/station_config.h','src/vision_link.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/verify_wheel_stall_log.c','tools/verify-wheel-stall-log.ps1')) {
        $stallHashes[$stallFile]=(Get-FileHash -LiteralPath $stallFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$stallExit;sourceHashes=$stallHashes;hardwareAccessed=$false;
        scope='All four wheel stall flags preserve raw feedback and log transitions; scalar motion and A/C automatic curves continue; interlock/station launch accept stall-only; disabled/protected, actual frozen encoder and global emergency still stop'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $stallOut 'result.json') -Encoding UTF8
    Write-Output $stallOut
} finally {$env:PATH=$stallOldPath;Pop-Location}
