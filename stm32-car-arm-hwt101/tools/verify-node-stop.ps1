param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$nodeRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$nodeOut=Join-Path $nodeRoot ('evidence/node-stop-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($nodeOut)
$nodeCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$nodeOldPath=$env:PATH
Push-Location -LiteralPath $nodeRoot
try {
    $env:PATH=(Split-Path -Parent $nodeCompiler)+';'+$env:PATH
    foreach($nodeSuite in @('node_stop','cross_handoff')) {
        $nodeExe='evidence/'+(Split-Path -Leaf $nodeOut)+'/verify_'+$nodeSuite+'.exe'
        & $nodeCompiler -DARM_HOST_TEST -DCAR_RUN_CORNER_ENABLED=0 -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            "tests/verify_$nodeSuite.c" src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $nodeExe
        if($LASTEXITCODE -ne 0){throw "Host compilation failed: $nodeSuite"}
        $nodeText=& (Join-Path $nodeRoot $nodeExe) 2>&1;$nodeExit=$LASTEXITCODE
        $nodeText | Tee-Object -FilePath (Join-Path $nodeOut "$nodeSuite.log")
        if($nodeExit -ne 0){throw "Host verification failed: $nodeSuite"}
    }
    $nodeHashes=[ordered]@{}
    foreach($nodeName in @('src/main_table_nav.c','src/car_config.h','src/full_route.h','src/structure.h','src/road_map.c',
        'src/arm_control.c','src/arm_config.h','src/station_job.c','src/station_config.h','src/vision_link.c',
        'src/heading_control.c','src/heading_sensor.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/verify_node_stop.c','tests/verify_cross_handoff.c','tools/verify-node-stop.ps1')) {
        $nodeHashes[$nodeName]=(Get-FileHash -LiteralPath $nodeName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$nodeHashes;hardwareAccessed=$false;
        scope='Explicit segmented comparison build: wheel-stop gate without fixed dwell, A/C direction changes and normal station turns, 910mm raw entry and 990mm post-work release, map-based storage departures with batch-specific offsets and avoidance on/off; lateral stop hands residual to work vision or next road including heading projection and physical retreat mileage'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $nodeOut 'result.json') -Encoding UTF8
    Write-Output $nodeOut
} finally {$env:PATH=$nodeOldPath;Pop-Location}
