param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$settleRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$settleRelative='evidence/terminal-settle-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$settleOut=Join-Path $settleRoot $settleRelative
[void][IO.Directory]::CreateDirectory($settleOut)
$settleOldPath=$env:PATH
Push-Location -LiteralPath $settleRoot
try {
    $settleCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $settleCompiler)+';'+$env:PATH
    $settleExe=$settleRelative+'/verify_terminal_settle.exe'
    & $settleCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_terminal_settle.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $settleExe
    if($LASTEXITCODE -ne 0){throw 'Terminal settle compilation failed'}
    $settleText=& (Join-Path $settleRoot $settleExe)
    $settleExit=$LASTEXITCODE
    $settleText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $settleOut 'result.log')
    if($settleExit -ne 0){throw 'Terminal settle behavior failed'}
    $settleHashes=[ordered]@{}
    foreach($settleFile in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/arm_control.c',
        'src/station_job.c','src/arm_config.h','src/station_config.h','tests/verify_terminal_settle.c',
        'tests/verify_heading.c','tests/verify_entry.c','tools/verify-terminal-settle.ps1')) {
        $settleHashes[$settleFile]=(Get-FileHash -LiteralPath $settleFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$settleExit;sourceHashes=$settleHashes;hardwareAccessed=$false;
        scope='Recorded terminal command replay,12 inertia cases,late CPOS phase changes,25ms/subcycle slew,nonvisual index/unmarked vector/obstacle exclusions,normal90/180deg turns,drive yaw damping; no physical acceptance';
        lateralDistanceWindowMm=1;headingWindowCdeg=80;output=$settleText} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $settleOut 'result.json')
    $settleText | Select-Object -Last 4
    Write-Output $settleOut
} finally {$env:PATH=$settleOldPath;Pop-Location}
