param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$limitRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$limitRelative='evidence/chassis-rpm-limit-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$limitOut=Join-Path $limitRoot $limitRelative
[void][IO.Directory]::CreateDirectory($limitOut)
$limitOldPath=$env:PATH
Push-Location -LiteralPath $limitRoot
try {
    $limitCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $limitCompiler)+';'+$env:PATH
    $limitExe=$limitRelative+'/verify_chassis_rpm_limit.exe'
    & $limitCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_chassis_rpm_limit.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $limitExe
    if($LASTEXITCODE -ne 0){throw 'Chassis RPM limit compilation failed'}
    $limitText=& (Join-Path $limitRoot $limitExe)
    $limitExit=$LASTEXITCODE
    $limitText | Tee-Object -FilePath (Join-Path $limitOut 'result.log')
    if($limitExit -ne 0){throw 'Chassis RPM limit behavior failed'}
    $limitHashes=[ordered]@{}
    foreach($limitFile in @('src/main_table_nav.c','src/car_config.h','src/motion.c','src/navigation.c','src/heading_control.c',
        'src/arm_config.h','src/arm_control.c','src/station_job.c','src/station_config.h','lookup/table_data.h',
        'tests/verify_chassis_rpm_limit.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-chassis-rpm-limit.ps1')) {
        $limitHashes[$limitFile]=(Get-FileHash -LiteralPath $limitFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$limitHashes;hardwareAccessed=$false;
        scope='Chassis200RPM input boundaries, mixed wheel saturation, position f/b, closed-loop l/r and180deg turns, A/C automatic launch; no hardware tracking acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $limitOut 'result.json') -Encoding UTF8
    Write-Output $limitOut
} finally {$env:PATH=$limitOldPath;Pop-Location}
