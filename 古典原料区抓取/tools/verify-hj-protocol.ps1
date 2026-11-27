param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$hjRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$hjOut=Join-Path $hjRoot ('evidence/hj-protocol-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($hjOut)
$hjOldPath=$env:PATH
Push-Location -LiteralPath $hjRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $hjExe='evidence/'+(Split-Path -Leaf $hjOut)+'/verify_hj_protocol.exe'
    & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup tests/verify_hj_protocol.c `
        src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c `
        src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $hjExe
    if($LASTEXITCODE -ne 0){throw 'H/J protocol compilation failed'}
    $hjText=& (Join-Path $hjRoot $hjExe) 2>&1;$hjExit=$LASTEXITCODE
    $hjText | Tee-Object -FilePath (Join-Path $hjOut 'result.log')
    if($hjExit -ne 0){throw 'H/J protocol verification failed'}
    $hjHashes=[ordered]@{}
    foreach($hjName in @('src/main_table_nav.c','src/vision_link.c','src/vision_link.h','src/station_job.c',
        'src/arm_control.c','src/arm_config.h','src/station_config.h','tests/verify_entry.c',
        'tests/verify_heading.c','tests/verify_station_vision.c','tests/verify_hj_protocol.c','tools/verify-hj-protocol.ps1')) {
        $hjHashes[$hjName]=(Get-FileHash -LiteralPath $hjName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$hjHashes;hardwareAccessed=$false;
        scope='J unique unordered colors, long frames, malformed input/CRC, silent wait without retry/timeout; H/J cancel -> one R -> READY -> deferred request, stale frame isolation and delivery gate; H direction integrated in raw-material suite'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $hjOut 'result.json') -Encoding UTF8
    Write-Output $hjOut
} finally {$env:PATH=$hjOldPath;Pop-Location}
