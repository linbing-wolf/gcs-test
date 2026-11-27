param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$rawEntryRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$rawEntryOut='evidence/raw-entry-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $rawEntryRoot $rawEntryOut))
$rawEntryCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$rawEntryOldPath=$env:PATH
Push-Location -LiteralPath $rawEntryRoot
try {
    $env:PATH=(Split-Path -Parent $rawEntryCompiler)+';'+$env:PATH
    & $rawEntryCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_raw_entry.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c `
        -lm -o ($rawEntryOut+'/verify_raw_entry.exe')
    if($LASTEXITCODE -ne 0){throw 'Raw entry compilation failed'}
    $rawEntryText=& ($rawEntryRoot+'/'+$rawEntryOut+'/verify_raw_entry.exe') 2>&1
    $rawEntryExit=$LASTEXITCODE
    $rawEntryText | Tee-Object -FilePath ($rawEntryOut+'/result.log')
    if($rawEntryExit -ne 0){throw 'Raw entry behavior failed'}
    $rawEntryHashes=[ordered]@{}
    foreach($rawEntryFile in @('src/main_table_nav.c','src/car_config.h','src/arm_control.c','src/arm_control.h',
        'src/arm_config.h','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/vision_movexy_cases.h',
        'tests/verify_raw_entry.c','tools/verify-raw-entry.ps1')) {
        $rawEntryHashes[$rawEntryFile]=(Get-FileHash -LiteralPath $rawEntryFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$rawEntryExit;sourceHashes=$rawEntryHashes;hardwareAccessed=$false;
        scope='A/C and both raw entries, approach reference speed, observed20/100mm overshoot, weak wheel response, H after encoder arrival; unobservable ground bias remains';output=$rawEntryText} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath ($rawEntryOut+'/result.json') -Encoding UTF8
    Write-Output $rawEntryOut
} finally {$env:PATH=$rawEntryOldPath;Pop-Location}
