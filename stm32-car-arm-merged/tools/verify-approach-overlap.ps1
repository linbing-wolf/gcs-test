param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$fineRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$fineOut=Join-Path $fineRoot ('evidence/station-terminal-overlap-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($fineOut)
$fineCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$fineOldPath=$env:PATH
Push-Location -LiteralPath $fineRoot
try {
    $env:PATH=(Split-Path -Parent $fineCompiler)+';'+$env:PATH
    $fineExe='evidence/'+(Split-Path -Leaf $fineOut)+'/verify_station_terminal_overlap.exe'
    & $fineCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_station_terminal_overlap.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $fineExe
    if($LASTEXITCODE -ne 0){throw 'Approach overlap test compilation failed'}
    $fineText=& (Join-Path $fineRoot $fineExe)
    $fineExit=$LASTEXITCODE
    $fineText | Tee-Object -FilePath (Join-Path $fineOut 'result.log')
    if($fineExit -ne 0){throw 'Approach overlap behavior test failed'}
    $fineHashes=[ordered]@{}
    foreach($fineName in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/heading_control.h',
        'src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/vision_movexy_cases.h','tests/verify_approach_overlap.c','tests/verify_station_terminal_overlap.c','tools/verify-approach-overlap.ps1')) {
        $fineHashes[$fineName]=(Get-FileHash -LiteralPath $fineName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$fineExit;sourceHashes=$fineHashes;
        scope='All-area station entry PWM and index axis commands with nonzero chassis speed; real grip waits for completion and vision; no hardware';hardwareAccessed=$false} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $fineOut 'result.json') -Encoding UTF8
    Write-Output $fineOut
} finally {$env:PATH=$fineOldPath;Pop-Location}
