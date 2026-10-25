param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$storageRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$storageRelative='evidence/storage-first-through-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$storageOut=Join-Path $storageRoot $storageRelative
[void][IO.Directory]::CreateDirectory($storageOut)
$storageOldPath=$env:PATH
Push-Location -LiteralPath $storageRoot
try {
    $storageCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $storageCompiler)+';'+$env:PATH
    foreach($storageVariant in @(0,1,2,3)) {
        $storageDefines=@()
        if($storageVariant -eq 0){$storageDefines=@('-DCAR_RUN_CORNER_ENABLED=0')}
        if($storageVariant -eq 1){$storageDefines=@('-DCAR_RUN_CORNER_ENABLED=1')}
        if($storageVariant -eq 2){$storageDefines=@('-DCAR_RUN_OMNI_ENABLED=0')}
        if($storageVariant -eq 3){$storageDefines=@('-DCAR_ARM_TRANSLATION_OVERLAP_ENABLED=0')}
        $storageExe=$storageRelative+'/verify_storage_first_through_'+$storageVariant+'.exe'
        & $storageCompiler -DARM_HOST_TEST @storageDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_storage_first_through.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $storageExe
        if($LASTEXITCODE -ne 0){throw 'First storage through compilation failed'}
        $storageText=& (Join-Path $storageRoot $storageExe)
        $storageExit=$LASTEXITCODE
        $storageText | Tee-Object -FilePath (Join-Path $storageOut ('result-'+$storageVariant+'.log'))
        if($storageExit -ne 0){throw 'First storage through behavior failed'}
    }
    $storageHashes=[ordered]@{}
    foreach($storageFile in @('src/main_table_nav.c','src/car_config.h','src/road_map.c','src/structure.h','src/full_route.h','src/station_job.c','src/station_job.h','src/station_config.h','src/arm_control.c','src/arm_config.h',
        'lookup/table_data.h','lookup/generation.json','tests/verify_storage_first_through.c','tests/verify_start_scan.c',
        'tests/verify_station_vision.c','tests/verify_heading.c','tools/verify-storage-first-through.ps1')) {
        $storageHashes[$storageFile]=(Get-FileHash -LiteralPath $storageFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@('corner_off','all_on','omni_off','arm_overlap_off');sourceHashes=$storageHashes;hardwareAccessed=$false;
        scope='First storage last release then measured-clearance retraction and fixed ring2 prefix joins H-E-F without stop/vision; all6 ring orders, A/C, corner disabled fallback; second storage3F unchanged; host model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $storageOut 'result.json') -Encoding UTF8
    Write-Output $storageOut
} finally {$env:PATH=$storageOldPath;Pop-Location}
