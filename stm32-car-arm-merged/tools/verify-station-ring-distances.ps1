param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$indexRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$indexRelative='evidence/station-ring-distances-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$indexOut=Join-Path $indexRoot $indexRelative
[void][IO.Directory]::CreateDirectory($indexOut)
$indexOldPath=$env:PATH
Push-Location -LiteralPath $indexRoot
try {
    $indexCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $indexCompiler)+';'+$env:PATH
    $indexVariants=@()
    foreach($indexVariant in 0..7) {
        $indexDefines=@()
        if($indexVariant -in @(1,3,5,6,7)){$indexDefines+=@('-DSTN_ROUGH_INDEX_12_MM=145','-DSTN_ROUGH_INDEX_23_MM=165')}
        if($indexVariant -in @(2,3,5,6,7)){$indexDefines+=@('-DSTN_STORAGE_INDEX_12_MM=140','-DSTN_STORAGE_INDEX_23_MM=170')}
        if($indexVariant -eq 4){$indexDefines+=@('-DSTN_ROUGH_INDEX_12_MM=165','-DSTN_ROUGH_INDEX_23_MM=145','-DSTN_STORAGE_INDEX_12_MM=170','-DSTN_STORAGE_INDEX_23_MM=140')}
        if($indexVariant -eq 5){$indexDefines+='-DCAR_ARM_TRANSLATION_OVERLAP_ENABLED=0'}
        if($indexVariant -eq 6){$indexDefines+='-DCAR_RUN_CORNER_ENABLED=0'}
        if($indexVariant -eq 7){$indexDefines+='-DCAR_RUN_OMNI_ENABLED=0'}
        $indexExe=$indexRelative+'/verify_station_ring_distances_'+$indexVariant+'.exe'
        & $indexCompiler -DARM_HOST_TEST @indexDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_station_ring_distances.c src/arm_control.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $indexExe
        if($LASTEXITCODE -ne 0){throw 'Ring distance compilation failed'}
        $indexText=& (Join-Path $indexRoot $indexExe) 2>&1
        $indexExit=$LASTEXITCODE
        $indexText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $indexOut ('result-'+$indexVariant+'.log'))
        if($indexExit -ne 0){$indexText | Select-Object -Last 4;throw 'Ring distance behavior failed'}
        $indexVariants+=@{variant=$indexVariant;testOnlyOverrides=($indexVariant -ne 0);defines=$indexDefines;exitCode=$indexExit}
        $indexText | Select-Object -First 1
    }
    $indexHashes=[ordered]@{}
    foreach($indexFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_config.h','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/vision_link.c','src/heading_control.c',
        'tests/verify_station_ring_distances.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tests/vision_movexy_cases.h','tools/verify-station-ring-distances.ps1')) {
        $indexHashes[$indexFile]=(Get-FileHash -LiteralPath $indexFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$indexHashes;variants=$indexVariants;hardwareAccessed=$false;
        scope='Four independent spatial gaps; rough/storage both batches and all six orders; every compiled move and return-to-ring2 prefix; actual modeled ground positions; production defaults and seven test-only asymmetric/switch variants'} |
        ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $indexOut 'result.json')
    Write-Output $indexOut
} finally {$env:PATH=$indexOldPath;Pop-Location}
