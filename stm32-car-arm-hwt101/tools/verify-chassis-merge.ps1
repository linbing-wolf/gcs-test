param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$mergeRoot=Split-Path -Parent $PSScriptRoot
$mergeOut=Join-Path $mergeRoot ('evidence/chassis-merge-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($mergeOut)
$mergeOldPath=$env:PATH
Push-Location -LiteralPath $mergeRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $mergeExe='evidence/'+(Split-Path -Leaf $mergeOut)+'/verify.exe'
    & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Isrc -Ilookup `
        tests/verify_chassis_merge.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $mergeExe
    if($LASTEXITCODE -ne 0){throw 'Merged chassis compilation failed'}
    foreach($mergeMode in 0..8) {
        $mergeText=& (Join-Path $mergeRoot $mergeExe) $mergeMode 2>&1
        $mergeExit=$LASTEXITCODE
        $mergeText | Tee-Object -FilePath (Join-Path $mergeOut ("mode-$mergeMode.log"))
        if($mergeExit -ne 0){throw "Merged chassis mode $mergeMode failed"}
    }
    $mergeHashes=[ordered]@{}
    foreach($mergeFile in @('src/main_table_nav.c','src/car_config.h','src/heading_sensor.c','src/heading_sensor.h',
        'src/heading_control.c','src/heading_control.h','src/arm_control.c','src/arm_config.h','src/station_job.c',
        'src/station_config.h','src/vision_link.c','src/vision_link.h','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_chassis_merge.c','tools/verify-chassis-merge.ps1')) {
        $mergeHashes[$mergeFile]=(Get-FileHash -LiteralPath $mergeFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    @{passed=$true;sourceHashes=$mergeHashes;hardwareAccessed=$false} | ConvertTo-Json -Depth 5 |
        Set-Content -LiteralPath (Join-Path $mergeOut 'result.json') -Encoding UTF8
    Write-Output $mergeOut
} finally {$env:PATH=$mergeOldPath;Pop-Location}
