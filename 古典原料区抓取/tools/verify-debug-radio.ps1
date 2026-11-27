param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$radioRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$radioOut='evidence/debug-radio-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$radioOldPath=$env:PATH
Push-Location -LiteralPath $radioRoot
try {
    [void][IO.Directory]::CreateDirectory($radioOut)
    $radioCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $radioCompiler)+';'+$env:PATH
    & $radioCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_debug_radio.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c `
        -lm -o "$radioOut/verify-debug-radio.exe"
    if($LASTEXITCODE -ne 0){throw 'Debug radio host compilation failed'}
    $radioText=& "./$radioOut/verify-debug-radio.exe"
    $radioExit=$LASTEXITCODE
    $radioText | Tee-Object -FilePath "$radioOut/result.log"
    if($radioExit -ne 0){throw 'Debug radio behavior failed'}
    $radioHashes=[ordered]@{}
    foreach($radioFile in @('src/debug_radio.c','src/debug_radio.h','src/main_table_nav.c','src/car_config.h','tests/verify_debug_radio.c')) {
        $radioHashes[$radioFile]=(Get-FileHash -LiteralPath $radioFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$radioExit;sourceHashes=$radioHashes;scope='Bit-driven UART state machine and production command callback, no hardware waveform verification'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath "$radioOut/result.json" -Encoding UTF8
    Write-Output $radioOut
} finally {$env:PATH=$radioOldPath;Pop-Location}
