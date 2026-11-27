param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$plateRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$plateOut=Join-Path $plateRoot ('evidence/raw-plate-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($plateOut)
$plateCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$plateOldPath=$env:PATH
Push-Location -LiteralPath $plateRoot
try {
    $env:PATH=(Split-Path -Parent $plateCompiler)+';'+$env:PATH
    $plateExe='evidence/'+(Split-Path -Leaf $plateOut)+'/verify_raw_plate.exe'
    & $plateCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_raw_plate.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $plateExe
    if($LASTEXITCODE -ne 0){throw 'Raw plate test compilation failed'}
    $plateText=& (Join-Path $plateRoot $plateExe)
    $plateExit=$LASTEXITCODE
    $plateText | Tee-Object -FilePath (Join-Path $plateOut 'result.log')
    if($plateExit -ne 0){throw 'Raw plate behavior test failed'}
    $plateHashes=[ordered]@{}
    foreach($plateName in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_config.h',
        'src/vision_link.c','src/vision_link.h','src/arm_control.c','src/arm_config.h','src/heading_control.c','src/heading_sensor.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/vision_movexy_cases.h',
        'tests/verify_raw_plate.c','tools/verify-raw-plate.ps1')) {
        $plateHashes[$plateName]=(Get-FileHash -LiteralPath $plateName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$plateExit;sourceHashes=$plateHashes;hardwareAccessed=$false;
        scope='Each raw visit H once before C,C,C; FIFO/early DISC, signs/displacement, feedback recovery, late terminal, emergency/driver fault and full A/C routes; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $plateOut 'result.json') -Encoding UTF8
    Write-Output $plateOut
} finally {$env:PATH=$plateOldPath;Pop-Location}
