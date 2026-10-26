param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$scanRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$scanRelative='evidence/scan-straight-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$scanOut=Join-Path $scanRoot $scanRelative
[void][IO.Directory]::CreateDirectory($scanOut)
$scanOldPath=$env:PATH
Push-Location -LiteralPath $scanRoot
try {
    $scanCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $scanCompiler)+';'+$env:PATH
    foreach($scanCorners in @(0,1)) {
        $scanExe=$scanRelative+'/verify_scan_straight_'+$scanCorners+'.exe'
        & $scanCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$scanCorners) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_scan_straight.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $scanExe
        if($LASTEXITCODE -ne 0){throw 'Straight scan compilation failed'}
        $scanText=& (Join-Path $scanRoot $scanExe)
        $scanExit=$LASTEXITCODE
        $scanText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $scanOut ('result-'+$scanCorners+'.log'))
        if($scanExit -ne 0){throw 'Straight scan behavior failed'}
        $scanText | Select-Object -Last 3
    }
    $scanHashes=[ordered]@{}
    foreach($scanFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/heading_control.c','tests/verify_scan_straight.c','tests/verify_heading.c',
        'tests/verify_entry.c','tests/verify_station_vision.c','tools/verify-scan-straight.ps1')) {
        $scanHashes[$scanFile]=(Get-FileHash -LiteralPath $scanFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$scanHashes;hardwareAccessed=$false;cornerVariants=@(0,1);
        scope='A/C early/preloaded/B-E/E-F/F-wait/no QR,48 timing cases,two bauds,missing config ACK,split/invalid/duplicate input,one init,manual task,emergency and two full two-batch routes; no board acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $scanOut 'result.json')
    Write-Output $scanOut
} finally {$env:PATH=$scanOldPath;Pop-Location}
