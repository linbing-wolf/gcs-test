param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$polarityRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$polarityRelative='evidence/movexy-polarity-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$polarityOut=Join-Path $polarityRoot $polarityRelative
[void][IO.Directory]::CreateDirectory($polarityOut)
$polarityOldPath=$env:PATH
Push-Location -LiteralPath $polarityRoot
try {
    $polarityCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $polarityCompiler)+';'+$env:PATH
    $polarityExe=$polarityRelative+'/verify_movexy_polarity.exe'
    & $polarityCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_movexy_polarity.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $polarityExe
    if($LASTEXITCODE -ne 0){throw 'Polarity vision compilation failed'}
    $polarityText=& (Join-Path $polarityRoot $polarityExe)
    $polarityExit=$LASTEXITCODE
    $polarityText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $polarityOut 'result.log')
    if($polarityExit -ne 0){throw 'Polarity vision behavior failed'}
    $polarityHashes=[ordered]@{}
    foreach($polarityFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h',
        'src/arm_control.c','src/arm_control.h','src/arm_config.h','src/station_config.h','src/heading_control.c','src/vision_link.c','tests/verify_movexy_polarity.c',
        'tests/verify_heading.c','tests/verify_entry.c','tests/vision_movexy_cases.h','tests/verify_station_vision.c','tools/verify-movexy-polarity.ps1')) {
        $polarityHashes[$polarityFile]=(Get-FileHash -LiteralPath $polarityFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$polarityExit;sourceHashes=$polarityHashes;hardwareAccessed=$false;
        scope='Reversal CPOS replay rejects old-direction pairing; learned polarity retained across scalar/vector handoff;36/-72 and reverse with all16 polarities/two responses; cold four quadrants; no physical acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $polarityOut 'result.json')
    $polarityText | Select-Object -Last 5
    Write-Output $polarityOut
} finally {$env:PATH=$polarityOldPath;Pop-Location}
