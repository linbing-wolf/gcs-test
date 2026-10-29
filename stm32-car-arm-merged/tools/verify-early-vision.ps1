param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$earlyRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$earlyRelative='evidence/early-vision-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$earlyOut=Join-Path $earlyRoot $earlyRelative
[void][IO.Directory]::CreateDirectory($earlyOut)
$earlyOldPath=$env:PATH
Push-Location -LiteralPath $earlyRoot
try {
    $earlyCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $earlyCompiler)+';'+$env:PATH
    $earlyExe=$earlyRelative+'/verify_early_vision.exe'
    & $earlyCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_early_vision.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $earlyExe
    if($LASTEXITCODE -ne 0){throw 'Early vision compilation failed'}
    $earlyText=& (Join-Path $earlyRoot $earlyExe)
    $earlyExit=$LASTEXITCODE
    $earlyText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $earlyOut 'result.log')
    if($earlyExit -ne 0){throw 'Early vision behavior failed'}
    $earlyHashes=[ordered]@{}
    foreach($earlyFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/heading_control.c','src/vision_link.c','tests/verify_early_vision.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-early-vision.ps1')) {
        $earlyHashes[$earlyFile]=(Get-FileHash -LiteralPath $earlyFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$earlyExit;sourceHashes=$earlyHashes;hardwareAccessed=$false;
        scope='D waits for original station arrival; H/E/F and first C retain early requests; immediate replies, H FIFO and C handoff, F index handoff, manual !, exact request counts and full A/C routes; no physical acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $earlyOut 'result.json')
    $earlyText | Select-Object -Last 5
    Write-Output $earlyOut
} finally {$env:PATH=$earlyOldPath;Pop-Location}
