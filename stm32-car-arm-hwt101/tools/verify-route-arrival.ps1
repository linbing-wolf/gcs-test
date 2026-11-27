param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$arrivalRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$arrivalOut=Join-Path $arrivalRoot ('evidence/route-arrival-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($arrivalOut)
$arrivalCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$arrivalOldPath=$env:PATH
Push-Location -LiteralPath $arrivalRoot
try {
    $env:PATH=(Split-Path -Parent $arrivalCompiler)+';'+$env:PATH
    $arrivalExe='evidence/'+(Split-Path -Leaf $arrivalOut)+'/verify_route_arrival.exe'
    & $arrivalCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_route_arrival.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $arrivalExe
    if($LASTEXITCODE -ne 0){throw 'Route arrival test compilation failed'}
    $arrivalText=& (Join-Path $arrivalRoot $arrivalExe)
    $arrivalExit=$LASTEXITCODE
    $arrivalText | Tee-Object -FilePath (Join-Path $arrivalOut 'result.log')
    if($arrivalExit -ne 0){throw 'Route arrival behavior test failed'}
    $arrivalHashes=[ordered]@{}
    foreach($arrivalName in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_config.h',
        'src/vision_link.c','src/vision_link.h','src/arm_control.c','src/arm_config.h','src/heading_control.c','src/heading_sensor.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/vision_movexy_cases.h',
        'tests/verify_route_arrival.c','tools/verify-route-arrival.ps1')) {
        $arrivalHashes[$arrivalName]=(Get-FileHash -LiteralPath $arrivalName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$arrivalExit;sourceHashes=$arrivalHashes;hardwareAccessed=$false;
        scope='C return blend, independent corner/endpoint, visual-only terminal handoff without reverse, and non-overlapping wall-clock categories; host only'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $arrivalOut 'result.json') -Encoding UTF8
    Write-Output $arrivalOut
} finally {$env:PATH=$arrivalOldPath;Pop-Location}
