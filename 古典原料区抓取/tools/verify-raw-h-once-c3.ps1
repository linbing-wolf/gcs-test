param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$relative='evidence/raw-h-once-c3-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$out=Join-Path $root $relative
[void][IO.Directory]::CreateDirectory($out)
$savedPath=$env:PATH
Push-Location -LiteralPath $root
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $exe=$relative+'/verify-raw-h-once-c3.exe'
    & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_raw_h_once_c3.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $exe
    if($LASTEXITCODE -ne 0){throw 'Raw H/C3 host compilation failed'}
    $result=& (Join-Path $root $exe) 2>&1
    $code=$LASTEXITCODE
    $result | Tee-Object -FilePath (Join-Path $out 'test.log')
    if($code -ne 0){throw "Raw H/C3 regression failed with exit code $code"}
    Write-Output $out
} finally {
    $env:PATH=$savedPath
    Pop-Location
}
