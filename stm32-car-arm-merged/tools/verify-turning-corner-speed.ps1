param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$turnRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$turnOut='evidence/turning-corner-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $turnRoot $turnOut))
$turnOldPath=$env:PATH
Push-Location -LiteralPath $turnRoot
try {
    $turnCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $turnCompiler)+';'+$env:PATH
    foreach($turnVariant in @('baseline','current')) {
        $turnDefines=@('-DARM_HOST_TEST')
        if($turnVariant -eq 'baseline'){$turnDefines+='-DTURNING_SPEED_BASELINE'}
        $turnExe=$turnOut+'/turning-'+$turnVariant+'.exe'
        & $turnCompiler @turnDefines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_turning_corner_speed.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $turnExe
        if($LASTEXITCODE -ne 0){throw 'Turning corner compilation failed'}
        $turnText=& (Join-Path $turnRoot $turnExe);$turnExit=$LASTEXITCODE
        $turnText | Set-Content -Encoding UTF8 -LiteralPath ($turnOut+'/'+$turnVariant+'.log')
        if($turnExit -ne 0){throw ('Turning corner behavior failed: '+$turnVariant)}
        $turnText | Select-Object -Last 4
    }
    $turnHashes=[ordered]@{}
    foreach($turnFile in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/arm_control.c','src/station_job.c',
        'src/motion.c','src/navigation.c','src/road_map.c','src/structure.h','src/full_route.h','lookup/table_data.h',
        'tests/verify_turning_corner_speed.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-turning-corner-speed.ps1')) {
        $turnHashes[$turnFile]=(Get-FileHash -LiteralPath $turnFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$turnHashes;variants=@('baseline','current');hardwareAccessed=$false;
        scope='108 independent plant cases per variant; rotating-only slew, ordinary stopping and tiers retained; full-route mode separate; model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath ($turnOut+'/result.json')
    Write-Output $turnOut
} finally {$env:PATH=$turnOldPath;Pop-Location}
