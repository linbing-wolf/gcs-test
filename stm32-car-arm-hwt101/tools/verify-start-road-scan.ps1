param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$scanRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$scanRelative='evidence/start-road-scan-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$scanOut=Join-Path $scanRoot $scanRelative
[void][IO.Directory]::CreateDirectory($scanOut)
$scanOldPath=$env:PATH
Push-Location -LiteralPath $scanRoot
try {
    $scanCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $scanCompiler)+';'+$env:PATH
    $scanExe=$scanRelative+'/verify_start_road_scan.exe'
    & $scanCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_start_road_scan.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $scanExe
    if($LASTEXITCODE -ne 0){throw 'Start scan compilation failed'}
    $scanText=& (Join-Path $scanRoot $scanExe) 2>&1
    $scanExit=$LASTEXITCODE
    $scanText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $scanOut 'result.log')
    $scanText | Write-Output
    if($scanExit -ne 0){throw 'Start scan behavior failed'}
    $scanHashes=[ordered]@{}
    foreach($scanFile in @('src/start_scan.c','src/start_scan.h','src/main_table_nav.c','src/car_config.h',
        'src/road_map.c','src/structure.h','src/heading_control.c','src/vision_link.c','src/vision_soft_i2c.h',
        'src/arm_control.c','src/arm_config.h','src/station_job.c','src/station_config.h','src/motion.c','src/navigation.c',
        'tests/verify_start_road_scan.c','tests/verify_heading.c','tests/verify_entry.c','tests/entry_host_io.h','tools/verify-start-road-scan.ps1')) {
        $scanHashes[$scanFile]=(Get-FileHash -LiteralPath $scanFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$scanExit;sourceHashes=$scanHashes;hardwareAccessed=$false;
        scope='Real entry/IMU/motor-model scan A right90 and C left90, 31 positions x3 edge-timed samples, 145mm probe offset, chassis-footprint candidates, unknown/suspect only, emergency ownership, no route changes; not physical road classification'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $scanOut 'result.json')
    Write-Output $scanOut
} finally {$env:PATH=$scanOldPath;Pop-Location}
