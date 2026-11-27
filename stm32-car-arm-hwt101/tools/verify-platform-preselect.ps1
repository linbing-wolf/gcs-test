param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe',[switch]$RetainSlideOnly)
$ErrorActionPreference='Stop'
$preRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$preRelative='evidence/platform-preselect-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$preOut=Join-Path $preRoot $preRelative
[void][IO.Directory]::CreateDirectory($preOut)
$prePath=$env:PATH
Push-Location -LiteralPath $preRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $variants=[ordered]@{default=@();no_direct=@('-DARM_DIRECT_TRANSFER_ENABLED=0');
        serial=@('-DARM_LIFT_PARALLEL_ENABLED=0');preselect_off=@('-DARM_STATION_PRESELECT_ENABLED=0');
        fixed=@('-DSTN_FIXED_RING_ARM_ENABLED=1')}
    $suiteArgs=@()
    if($RetainSlideOnly) {
        $variants.Remove('fixed')
        $variants['no_index_overlap']=@('-DCAR_ARM_TRANSLATION_OVERLAP_ENABLED=0')
        $variants['unequal_slides']=@('-DSTN_ROUGH_INDEX_RING1_SLIDE_MM=42','-DSTN_STORAGE_INDEX_RING3_SLIDE_MM=43')
        $suiteArgs=@('retain-slide')
    }
    foreach($name in $variants.Keys) {
        $exe=$preRelative+'/'+$name+'.exe'
        & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup @($variants[$name]) `
            tests/verify_platform_preselect.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $exe
        if($LASTEXITCODE -ne 0){throw "Platform preselection $name compilation failed"}
        $result=& (Join-Path $preRoot $exe) @suiteArgs 2>&1;$code=$LASTEXITCODE
        $result | Tee-Object -FilePath (Join-Path $preOut ($name+'.log'))
        if($code -ne 0){throw "Platform preselection $name failed"}
    }
    $hashes=[ordered]@{}
    foreach($file in @('src/arm_control.c','src/arm_config.h','src/station_job.c','src/station_config.h',
        'src/main_table_nav.c','src/vision_link.c','src/heading_control.c','src/heading_sensor.c',
        'tests/verify_platform_preselect.c','tests/verify_station_vision.c','tests/verify_heading.c',
        'tests/verify_entry.c','tools/verify-platform-preselect.ps1')) {
        $hashes[$file]=(Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$hashes;variants=@($variants.Keys);hardwareAccessed=$false;
        scope=if($RetainSlideOnly){'Both rough batches and first storage retain matching cup slide until final parking; monitored lift clearance, wheel-stop release, serial variants and per-ring fallback'}else{'Outbound transfer holds the cup slide through placement yaw and delays the next cup until clearance and yaw hold; both rough batches, both storage batches, concurrent ground slide and descent, delayed slide feedback on stack return, four-wheel stop and heading hold at ground placement, cup release with nonzero wheel feedback, raw loading, retrieval preselection and explicit stop'}} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $preOut 'result.json') -Encoding UTF8
    Write-Output $preOut
} finally {$env:PATH=$prePath;Pop-Location}
