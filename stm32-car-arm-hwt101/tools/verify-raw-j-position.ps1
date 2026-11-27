param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$positionRoot=Split-Path -Parent $PSScriptRoot
$positionOut='evidence/raw-j-position-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$oldPositionPath=$env:PATH
Push-Location $positionRoot
try {
    [void][IO.Directory]::CreateDirectory((Join-Path $positionRoot $positionOut))
    $env:PATH=(Split-Path $Gcc)+';'+$env:PATH
    & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup tests/verify_raw_j_position.c `
        src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c `
        src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o "$positionOut/test.exe"
    if($LASTEXITCODE -ne 0){throw 'Position regression compilation failed'}
    & "./$positionOut/test.exe" | Tee-Object "$positionOut/test.log"
    if($LASTEXITCODE -ne 0){throw 'Position regression failed'}
    @{passed=$true;hardwareAccessed=$false} | ConvertTo-Json | Set-Content "$positionOut/result.json"
    Write-Output $positionOut
} finally {$env:PATH=$oldPositionPath;Pop-Location}
