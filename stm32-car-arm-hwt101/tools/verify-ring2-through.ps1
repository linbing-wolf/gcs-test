param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$throughRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$throughOut='evidence/ring2-through-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $throughRoot $throughOut))
$throughCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$throughOldPath=$env:PATH
Push-Location -LiteralPath $throughRoot
try {
    $env:PATH=(Split-Path -Parent $throughCompiler)+';'+$env:PATH
    & $throughCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_ring2_through.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c `
        -lm -o ($throughOut+'/verify_ring2_through.exe')
    if($LASTEXITCODE -ne 0){throw 'Ring2 through compilation failed'}
    $throughText=& ($throughRoot+'/'+$throughOut+'/verify_ring2_through.exe') 2>&1
    $throughExit=$LASTEXITCODE
    $throughText | Tee-Object -FilePath ($throughOut+'/result.log')
    if($throughExit -ne 0){throw 'Ring2 through behavior failed'}
    $throughHashes=[ordered]@{}
    foreach($throughFile in @('src/main_table_nav.c','src/car_config.h','src/arm_control.c','src/arm_control.h',
        'src/arm_config.h','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/vision_movexy_cases.h',
        'tests/verify_ring2_through.c','tools/verify-ring2-through.ps1')) {
        $throughHashes[$throughFile]=(Get-FileHash -LiteralPath $throughFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$throughExit;sourceHashes=$throughHashes;hardwareAccessed=$false;
        scope='A/C both rough visits, six ring permutations; actual nonzero wheel commands through ring2 blend; last onboard release during motion; fixed map endpoint, visual counts, feedback pause and cancellation';output=$throughText} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath ($throughOut+'/result.json') -Encoding UTF8
    Write-Output $throughOut
} finally {$env:PATH=$throughOldPath;Pop-Location}
