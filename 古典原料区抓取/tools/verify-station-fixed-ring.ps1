param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$root=Join-Path $PSScriptRoot '..'
$out='evidence/fixed-ring-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$compiler=(Get-Command $Gcc -ErrorAction Stop).Source
$oldPath=$env:PATH
Push-Location -LiteralPath $root
try {
    [void](New-Item -ItemType Directory -Path $out)
    $env:PATH=(Split-Path -Parent $compiler)+';'+$env:PATH
    $common=@('-DARM_HOST_TEST','-std=c99','-Wall','-Wextra','-Werror','-Isrc','-Ilookup',
        'tests/verify_station_fixed_ring.c','src/arm_control.c','src/station_job.c','src/vision_link.c',
        'src/hmi_status.c','src/motion.c','src/navigation.c','src/road_map.c','src/heading_sensor.c',
        'src/heading_control.c','lookup/route_table.c','-lm')
    $missing=@('-DSTN_ROUGH_RING1_SLIDE_MM=-1')
    # 测试专用，故意给每个环不同的数值以识别姿态映射错误。
    $calibrated=@('-DFIXED_RING_TEST_CALIBRATED',
        '-DSTN_ROUGH_RING1_SLIDE_MM=120','-DSTN_ROUGH_RING1_YAW_DEG=83.0f',
        '-DSTN_ROUGH_RING3_SLIDE_MM=128','-DSTN_ROUGH_RING3_YAW_DEG=178.0f',
        '-DSTN_STORAGE_RING1_SLIDE_MM=110','-DSTN_STORAGE_RING1_YAW_DEG=90.0f',
        '-DSTN_STORAGE_RING3_SLIDE_MM=140','-DSTN_STORAGE_RING3_YAW_DEG=182.0f')
    foreach($variant in @('missing','calibrated')) {
        [string[]]$parameters=if($variant -eq 'missing'){$missing}else{$calibrated}
        $exe="$out/$variant.exe"
        & $compiler @common @parameters -o $exe
        if($LASTEXITCODE -ne 0){throw "Fixed ring $variant compilation failed"}
        & "./$exe" | Tee-Object -FilePath "$out/$variant.log"
        if($LASTEXITCODE -ne 0){throw "Fixed ring $variant verification failed"}
    }
    Write-Output $out
} finally {
    $env:PATH=$oldPath
    Pop-Location
}
