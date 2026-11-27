param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$materialRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$materialOut=Join-Path $materialRoot ('evidence/raw-material-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($materialOut)
$materialCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$materialOldPath=$env:PATH
Push-Location -LiteralPath $materialRoot
try {
    $env:PATH=(Split-Path -Parent $materialCompiler)+';'+$env:PATH
    $materialVariants=[ordered]@{default=@();no_direct=@('-DARM_DIRECT_TRANSFER_ENABLED=0');serial=@('-DARM_LIFT_PARALLEL_ENABLED=0');
        distinct_poses=@('-DARM_RAW_PAST_SLIDE_MM=142','-DARM_RAW_PAST_LIFT_MM=81','-DARM_RAW_PAST_YAW_DEG=106.5f',
            '-DARM_RAW_CURRENT_SLIDE_MM=54','-DARM_RAW_CURRENT_LIFT_MM=84','-DARM_RAW_CURRENT_YAW_DEG=130.5f',
            '-DARM_RAW_NEXT_SLIDE_MM=169','-DARM_RAW_NEXT_LIFT_MM=87','-DARM_RAW_NEXT_YAW_DEG=155.5f')}
    foreach($materialVariant in $materialVariants.Keys) {
        $materialExe='evidence/'+(Split-Path -Leaf $materialOut)+'/'+$materialVariant+'.exe'
        & $materialCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            @($materialVariants[$materialVariant]) tests/verify_raw_material.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $materialExe
        if($LASTEXITCODE -ne 0){throw "Raw material $materialVariant compilation failed"}
        $materialText=& (Join-Path $materialRoot $materialExe)
        $materialExit=$LASTEXITCODE
        $materialText | Tee-Object -FilePath (Join-Path $materialOut ($materialVariant+'.log'))
        if($materialExit -ne 0){throw "Raw material $materialVariant behavior test failed"}
    }
    $materialHashes=[ordered]@{}
    foreach($materialName in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h','src/station_config.h',
        'src/raw_region_config.h','src/vision_link.c','src/vision_link.h','src/vision_soft_i2c.h','src/arm_control.c','src/arm_control.h','src/arm_config.h',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/verify_raw_plate.c',
        'tests/verify_raw_material.c','tools/verify-raw-material.ps1')) {
        $materialHashes[$materialName]=(Get-FileHash -LiteralPath $materialName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$materialExit;variants=@($materialVariants.Keys);sourceHashes=$materialHashes;hardwareAccessed=$false;
        scope='Automatic HDISC2 plus three J at both raw visits, fresh target coordinates per grip, 96 combinations, queued correction, BUSY/CRC holds, ERR and R/READY, legacy manual H; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $materialOut 'result.json') -Encoding UTF8
    Write-Output $materialOut
} finally {$env:PATH=$materialOldPath;Pop-Location}
