param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$lateralRoot=Split-Path -Parent $PSScriptRoot
$lateralOut='evidence/lateral-direction-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$lateralOldPath=$env:PATH
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    foreach($lateralProject in @($lateralRoot,(Join-Path $lateralRoot 'diagnostics/jy901b-chassis'))) {
        Push-Location -LiteralPath $lateralProject
        try {
            [void][IO.Directory]::CreateDirectory((Join-Path $lateralProject $lateralOut))
            $lateralExe=$lateralOut+'/verify.exe'
            & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Isrc -Ilookup `
                tests/verify_lateral_direction.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
                src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $lateralExe
            if($LASTEXITCODE -ne 0){throw 'Lateral direction compilation failed'}
            & (Join-Path $lateralProject $lateralExe) | Tee-Object -FilePath ($lateralOut+'/test.log')
            if($LASTEXITCODE -ne 0){throw 'Lateral direction regression failed'}
            $lateralHashes=[ordered]@{}
            foreach($lateralFile in @('src/main_table_nav.c','src/car_config.h','tests/verify_lateral_direction.c','tests/verify_heading.c')) {
                $lateralHashes[$lateralFile]=(Get-FileHash -LiteralPath $lateralFile -Algorithm SHA256).Hash.ToLowerInvariant()
            }
            @{passed=$true;sourceHashes=$lateralHashes;hardwareAccessed=$false} | ConvertTo-Json -Depth 5 |
                Set-Content -LiteralPath ($lateralOut+'/result.json') -Encoding UTF8
            Write-Output (Join-Path $lateralProject $lateralOut)
        } finally {Pop-Location}
    }
} finally {$env:PATH=$lateralOldPath}
