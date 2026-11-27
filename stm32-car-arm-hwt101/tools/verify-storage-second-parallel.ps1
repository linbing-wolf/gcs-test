param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$secondRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$secondOut='evidence/storage-second-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $secondRoot $secondOut))
$secondOldPath=$env:PATH
Push-Location -LiteralPath $secondRoot
try {
    $secondCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $secondCompiler)+';'+$env:PATH
    foreach($secondVariant in @(0,1,2,3)) {
        $secondDefines=@()
        if($secondVariant -eq 0){$secondDefines=@('-DCAR_RUN_CORNER_ENABLED=0')}
        if($secondVariant -eq 2){$secondDefines=@('-DCAR_RUN_OMNI_ENABLED=0')}
        if($secondVariant -eq 3){$secondDefines=@('-DCAR_ARM_TRANSLATION_OVERLAP_ENABLED=0')}
        $secondExe=$secondOut+'/second-'+$secondVariant+'.exe'
        & $secondCompiler -DARM_HOST_TEST @secondDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_storage_second_parallel.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $secondExe
        if($LASTEXITCODE -ne 0){throw 'Second storage compilation failed'}
        $secondText=& (Join-Path $secondRoot $secondExe);$secondStatus=$LASTEXITCODE
        $secondText | Set-Content -Encoding UTF8 -LiteralPath ($secondOut+'/result-'+$secondVariant+'.log')
        if($secondStatus -ne 0){throw 'Second storage behavior failed'}
        $secondText | Select-Object -Last 3
    }
    $secondHashes=[ordered]@{}
    foreach($secondFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h','src/station_config.h',
        'src/arm_control.c','src/arm_config.h','src/heading_control.c','src/road_map.c','src/structure.h','src/full_route.h','lookup/table_data.h',
        'tests/verify_storage_second_parallel.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-storage-second-parallel.ps1')) {
        $secondHashes[$secondFile]=(Get-FileHash -LiteralPath $secondFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$secondHashes;variants=@('corner_off','all_on','omni_off','arm_overlap_off');hardwareAccessed=$false;
        scope='Second storage72 ring/color/plan combinations per variant; fixed index/last ring2 through overlaps retraction, actual125mm versus135mm lift, stale feedback and manual!, exactly3F and signed vision corrections; model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath ($secondOut+'/result.json')
    Write-Output $secondOut
} finally {$env:PATH=$secondOldPath;Pop-Location}
