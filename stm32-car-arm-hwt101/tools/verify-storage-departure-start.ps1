param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$storageRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$storageRelative='evidence/storage-departure-start-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$storageOut=Join-Path $storageRoot $storageRelative
[void][IO.Directory]::CreateDirectory($storageOut)
$storageOldPath=$env:PATH
Push-Location -LiteralPath $storageRoot
try {
    $storageCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $storageCompiler)+';'+$env:PATH
    $storageExe=$storageRelative+'/verify_storage_departure_start.exe'
    & $storageCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_storage_departure_start.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $storageExe
    if($LASTEXITCODE -ne 0){throw 'Storage vision compilation failed'}
    $storageText=& (Join-Path $storageRoot $storageExe)
    $storageExit=$LASTEXITCODE
    $storageText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $storageOut 'result.log')
    if($storageExit -ne 0){throw 'Storage vision behavior failed'}
    $storageHashes=[ordered]@{}
    foreach($storageFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_storage_departure_start.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-storage-departure-start.ps1')) {
        $storageHashes[$storageFile]=(Get-FileHash -LiteralPath $storageFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$storageExit;sourceHashes=$storageHashes;hardwareAccessed=$false;
        scope='Both storage visits: first nonzero chassis batch while yaw is not home; lift held at release height and missing old zero timers; missing wheel feedback still waits and large heading corrections use a new stopped-position baseline; actual pure180 command after10mm entry with nonzero measured wheel speed and no zero batch in between; two full A/C routes; no physical acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $storageOut 'result.json')
    $storageText | Select-Object -Last 5
    Write-Output $storageOut
} finally {$env:PATH=$storageOldPath;Pop-Location}
