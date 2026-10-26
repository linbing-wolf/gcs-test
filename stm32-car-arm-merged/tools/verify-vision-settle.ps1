param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$visualRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$visualRelative='evidence/vision-settle-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$visualOut=Join-Path $visualRoot $visualRelative
[void][IO.Directory]::CreateDirectory($visualOut)
$visualOldPath=$env:PATH
Push-Location -LiteralPath $visualRoot
try {
    $visualCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $visualCompiler)+';'+$env:PATH
    $visualExe=$visualRelative+'/verify_vision_settle.exe'
    & $visualCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_vision_settle.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $visualExe
    if($LASTEXITCODE -ne 0){throw 'Vision settle compilation failed'}
    $visualText=& (Join-Path $visualRoot $visualExe)
    $visualExit=$LASTEXITCODE
    $visualText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $visualOut 'result.log')
    if($visualExit -ne 0){throw 'Vision settle behavior failed'}
    $visualHashes=[ordered]@{}
    foreach($visualFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/arm_control.c',
        'src/heading_control.c','src/vision_link.c','tests/verify_vision_settle.c','tests/vision_movexy_cases.h',
        'tests/verify_heading.c','tests/verify_entry.c','tests/verify_station_vision.c','tools/verify-vision-settle.ps1')) {
        $visualHashes[$visualFile]=(Get-FileHash -LiteralPath $visualFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$visualExit;sourceHashes=$visualHashes;hardwareAccessed=$false;
        scope='Actual MOVE/MOVEY/MOVEXY UART entry,terminal re-entry and slew,12 wheel-response cases,2/3mm moves,FIFO,manual stop,nonvisual index isolation,protocol,all quadrants/polarities and D/E/F automatic jobs; no physical acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $visualOut 'result.json')
    $visualText | Select-Object -Last 5
    Write-Output $visualOut
} finally {$env:PATH=$visualOldPath;Pop-Location}
