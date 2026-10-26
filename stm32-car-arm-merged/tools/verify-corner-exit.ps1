param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$exitRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$exitOut='evidence/corner-exit-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $exitRoot $exitOut))
$exitOldPath=$env:PATH
Push-Location -LiteralPath $exitRoot
try {
    $exitCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $exitCompiler)+';'+$env:PATH
    foreach($exitEnabled in @(0,1)) {
        $exitExe=$exitOut+'/corner-exit-'+$exitEnabled+'.exe'
        & $exitCompiler -DARM_HOST_TEST ('-DCAR_CORNER_EXIT_BLEND_ENABLED='+$exitEnabled) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_turning_corner_speed.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $exitExe
        if($LASTEXITCODE -ne 0){throw 'Corner exit compilation failed'}
        $exitText=& (Join-Path $exitRoot $exitExe);$exitStatus=$LASTEXITCODE
        $exitText | Set-Content -Encoding UTF8 -LiteralPath ($exitOut+'/focused-'+$exitEnabled+'.log')
        if($exitStatus -ne 0){throw 'Corner exit focused behavior failed'}
        $exitText | Select-Object -Last 2
        $exitText=& (Join-Path $exitRoot $exitExe) full-route;$exitStatus=$LASTEXITCODE
        $exitText | Set-Content -Encoding UTF8 -LiteralPath ($exitOut+'/full-route-'+$exitEnabled+'.log')
        if($exitStatus -ne 0){throw 'Corner exit full route failed'}
        $exitText
    }
    $exitHashes=[ordered]@{}
    foreach($exitFile in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/arm_control.c','src/station_job.c',
        'src/motion.c','src/navigation.c','src/road_map.c','src/structure.h','src/full_route.h','lookup/table_data.h',
        'tests/verify_turning_corner_speed.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-corner-exit.ps1')) {
        $exitHashes[$exitFile]=(Get-FileHash -LiteralPath $exitFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$exitHashes;variants=@(0,1);hardwareAccessed=$false;
        scope='R94 off/current on: progressive exit reference with5..15deg residual;108 independent plant cases and both full eight-job routes per variant; model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath ($exitOut+'/result.json')
    Write-Output $exitOut
} finally {$env:PATH=$exitOldPath;Pop-Location}
