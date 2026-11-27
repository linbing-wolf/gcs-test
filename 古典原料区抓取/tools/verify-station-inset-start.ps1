param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$insetRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$insetRelative='evidence/station-inset-start-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$insetOut=Join-Path $insetRoot $insetRelative
[void][IO.Directory]::CreateDirectory($insetOut)
$insetOldPath=$env:PATH
Push-Location -LiteralPath $insetRoot
try {
    $insetCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $insetCompiler)+';'+$env:PATH
    foreach($insetVariant in @(0,1)) {
        $insetExe=$insetRelative+'/verify_station_inset_start_'+$insetVariant+'.exe'
        & $insetCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$insetVariant) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_station_inset_start.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $insetExe
        if($LASTEXITCODE -ne 0){throw 'Station inset/start compilation failed'}
        $insetText=& (Join-Path $insetRoot $insetExe)
        $insetExit=$LASTEXITCODE
        $insetText | Tee-Object -FilePath (Join-Path $insetOut ('result-'+$insetVariant+'.log'))
        if($insetExit -ne 0){throw 'Station inset/start behavior failed'}
    }
    $insetHashes=[ordered]@{}
    foreach($insetFile in @('src/main_table_nav.c','src/car_config.h','src/road_map.c','src/structure.h','src/full_route.h',
        'lookup/table_data.h','lookup/generation.json','tests/verify_station_inset_start.c','tests/verify_start_scan.c',
        'tests/verify_station_vision.c','tests/verify_heading.c','tools/verify-station-inset-start.ps1')) {
        $insetHashes[$insetFile]=(Get-FileHash -LiteralPath $insetFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@(0,1);sourceHashes=$insetHashes;hardwareAccessed=$false;
        scope='F/H additional50mm retained, D/I additional80mm compensation removed; retained start60mm blend, early/late QR and B waiting, cancellation and A/C partial-curve obstacle return; only host focused checks'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $insetOut 'result.json') -Encoding UTF8
    Write-Output $insetOut
} finally {$env:PATH=$insetOldPath;Pop-Location}
