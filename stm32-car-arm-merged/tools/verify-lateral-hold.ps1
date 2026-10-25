param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$lateralRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$lateralRelative='evidence/lateral-hold-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$lateralOut=Join-Path $lateralRoot $lateralRelative
[void][IO.Directory]::CreateDirectory($lateralOut)
$lateralOldPath=$env:PATH
Push-Location -LiteralPath $lateralRoot
try {
    $lateralCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $lateralCompiler)+';'+$env:PATH
    foreach($lateralEnabled in @(0,1)) {
        $lateralExe=$lateralRelative+'/verify_lateral_hold_'+$lateralEnabled+'.exe'
        & $lateralCompiler -DARM_HOST_TEST ('-DCAR_STRAIGHT_HOLD_ENABLED='+$lateralEnabled) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_lateral_hold.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $lateralExe
        if($LASTEXITCODE -ne 0){throw 'Lateral hold compilation failed'}
        $lateralText=& (Join-Path $lateralRoot $lateralExe)
        $lateralExit=$LASTEXITCODE
        $lateralText | Tee-Object -FilePath (Join-Path $lateralOut ('result-'+$lateralEnabled+'.log'))
        if($lateralExit -ne 0){throw 'Lateral hold behavior failed'}
    }
    $lateralHashes=[ordered]@{}
    foreach($lateralFile in @('src/main_table_nav.c','src/car_config.h','src/motion.c','src/navigation.c','src/heading_control.c',
        'src/arm_config.h','src/arm_control.c','src/station_job.c','src/station_config.h','lookup/table_data.h',
        'tests/verify_lateral_hold.c','tests/verify_straight_hold.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-lateral-hold.ps1')) {
        $lateralHashes[$lateralFile]=(Get-FileHash -LiteralPath $lateralFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;variants=@(0,1);sourceHashes=$lateralHashes;hardwareAccessed=$false;
        scope='Common-time encoder forward-position hold during ordinary left/right motion; enabled/off,64 lateral and64 existing forward cases, visible/invisible drift, polarity, timer wrap, signs,200RPM, scope and E-pass continuity; no ground slip acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $lateralOut 'result.json') -Encoding UTF8
    Write-Output $lateralOut
} finally {$env:PATH=$lateralOldPath;Pop-Location}
