param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$experimentRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$experimentOutput='evidence/imu-experiment-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$experimentCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$experimentPath=$env:PATH
Push-Location -LiteralPath $experimentRoot
try {
    [void][IO.Directory]::CreateDirectory((Join-Path $experimentRoot $experimentOutput))
    $env:PATH=(Split-Path -Parent $experimentCompiler)+';'+$env:PATH
    $experimentExe="$experimentOutput/verify_imu_experiment.exe"
    & $experimentCompiler -DARM_HOST_TEST -DCHASSIS_IMU_EXPERIMENT=1 -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_imu_experiment.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $experimentExe
    if($LASTEXITCODE -ne 0){throw 'IMU experiment host compilation failed'}
    & (Join-Path $experimentRoot $experimentExe)
    if($LASTEXITCODE -ne 0){throw 'IMU experiment host test failed'}
    Write-Output (Join-Path $experimentRoot $experimentOutput)
} finally {
    $env:PATH=$experimentPath
    Pop-Location
}
