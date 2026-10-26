param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$recoveryRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$recoveryRelative='evidence/corner-exit-recovery-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$recoveryOut=Join-Path $recoveryRoot $recoveryRelative
[void][IO.Directory]::CreateDirectory($recoveryOut)
$recoveryOldPath=$env:PATH
Push-Location -LiteralPath $recoveryRoot
try {
    $recoveryCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $recoveryCompiler)+';'+$env:PATH
    $recoveryExe=$recoveryRelative+'/verify_corner_exit_recovery.exe'
    & $recoveryCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_corner_exit_recovery.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $recoveryExe
    if($LASTEXITCODE -ne 0){throw 'Recovery vision compilation failed'}
    $recoveryText=& (Join-Path $recoveryRoot $recoveryExe)
    $recoveryExit=$LASTEXITCODE
    $recoveryText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $recoveryOut 'result.log')
    if($recoveryExit -ne 0){throw 'Recovery vision behavior failed'}
    $recoveryHashes=[ordered]@{}
    foreach($recoveryFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_corner_exit_recovery.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-corner-exit-recovery.ps1')) {
        $recoveryHashes[$recoveryFile]=(Get-FileHash -LiteralPath $recoveryFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$recoveryExit;sourceHashes=$recoveryHashes;hardwareAccessed=$false;
        scope='Geometric last5deg enters recovery, aligned exit wheel acceleration uses configured maximum, zero remains200; six actual-wheel post-corner speed profiles compared with prechange snapshot; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $recoveryOut 'result.json')
    $recoveryText | Select-Object -Last 5
    Write-Output $recoveryOut
} finally {$env:PATH=$recoveryOldPath;Pop-Location}
