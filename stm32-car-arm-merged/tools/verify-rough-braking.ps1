param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$brakeRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$brakeRelative='evidence/rough-braking-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$brakeOut=Join-Path $brakeRoot $brakeRelative
[void][IO.Directory]::CreateDirectory($brakeOut)
$brakeOldPath=$env:PATH
Push-Location -LiteralPath $brakeRoot
try {
    $brakeCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $brakeCompiler)+';'+$env:PATH
    foreach($brakeVariant in @(0,1)) {
        $brakeExe=$brakeRelative+'/verify_rough_braking_'+$brakeVariant+'.exe'
        & $brakeCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$brakeVariant) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_rough_braking.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $brakeExe
        if($LASTEXITCODE -ne 0){throw 'Rough braking compilation failed'}
        $brakeText=& (Join-Path $brakeRoot $brakeExe)
        $brakeExit=$LASTEXITCODE
        $brakeText | Tee-Object -FilePath (Join-Path $brakeOut ('result-'+$brakeVariant+'.log'))
        if($brakeExit -ne 0){throw 'Rough braking behavior failed'}
    }
    $brakeHashes=[ordered]@{}
    foreach($brakeFile in @('src/main_table_nav.c','src/car_config.h','src/road_map.c','src/structure.h','src/full_route.h',
        'lookup/table_data.h','lookup/generation.json','tests/verify_rough_braking.c','tests/verify_start_scan.c',
        'tests/verify_station_vision.c','tests/verify_heading.c','tools/verify-rough-braking.ps1')) {
        $brakeHashes[$brakeFile]=(Get-FileHash -LiteralPath $brakeFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@(0,1);sourceHashes=$brakeHashes;hardwareAccessed=$false;
        scope='D first-stop peak with200ms brake lookahead and240RPM/s; both plans/batches, wheel responses0.18/0.30; slower0.12 and injected encoder overrun are corrected without automatic fault stop but remain no-entry failures; invisible ground displacement remains a limitation'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $brakeOut 'result.json') -Encoding UTF8
    Write-Output $brakeOut
} finally {$env:PATH=$brakeOldPath;Pop-Location}
