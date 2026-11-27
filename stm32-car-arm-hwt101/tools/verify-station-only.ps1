param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe',[switch]$RawC3)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$relative='evidence/station-only-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$out=Join-Path $root $relative
[void][IO.Directory]::CreateDirectory($out)
$savedPath=$env:PATH
$defines=@('-DARM_HOST_TEST')
if($RawC3){$defines+=@('-DEXPECT_RAW_C3')}
Push-Location -LiteralPath $root
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $exe=$relative+'/station-only.exe'
    & $Gcc @defines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_station_only.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $exe
    if($LASTEXITCODE -ne 0){throw 'Station-only host compilation failed'}
    $result=& (Join-Path $root $exe) 2>&1
    $code=$LASTEXITCODE
    $result | Tee-Object -FilePath (Join-Path $out 'test.log')
    if($code -ne 0){throw "Station-only regression failed with exit code $code"}
    Write-Output $out
} finally {
    $env:PATH=$savedPath
    Pop-Location
}
