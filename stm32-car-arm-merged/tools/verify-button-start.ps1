param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$buttonRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$buttonRelative='evidence/button-start-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$buttonOut=Join-Path $buttonRoot $buttonRelative
[void][IO.Directory]::CreateDirectory($buttonOut)
$buttonOldPath=$env:PATH
Push-Location -LiteralPath $buttonRoot
try {
    $buttonCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $buttonCompiler)+';'+$env:PATH
    $buttonExe=$buttonRelative+'/verify_button_start.exe'
    & $buttonCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_button_start.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $buttonExe
    if($LASTEXITCODE -ne 0){throw 'Button startup compilation failed'}
    $buttonText=& (Join-Path $buttonRoot $buttonExe)
    $buttonExit=$LASTEXITCODE
    $buttonText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $buttonOut 'result.log')
    if($buttonExit -ne 0){throw 'Button startup behavior failed'}
    $buttonHashes=[ordered]@{}
    foreach($buttonFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_button_start.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/entry_host_io.h','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-button-start.ps1')) {
        $buttonHashes[$buttonFile]=(Get-FileHash -LiteralPath $buttonFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$buttonExit;sourceHashes=$buttonHashes;hardwareAccessed=$false;
        scope='Cold PC9 enables arm after real four-wheel status, declares user-confirmed home pose with existing parameters, waits fresh IMU, starts C route once; cancellation, debounce, delayed feedback, timestamp wrap, busy/manual pose preservation; prepared-button regression and cold eight-job C route using scanner/vision peers; no hardware acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $buttonOut 'result.json')
    $buttonText | Select-Object -Last 5
    Write-Output $buttonOut
} finally {$env:PATH=$buttonOldPath;Pop-Location}
