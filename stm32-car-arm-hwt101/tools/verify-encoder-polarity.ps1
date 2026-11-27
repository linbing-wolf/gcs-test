param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$polarityRoot=Split-Path -Parent $PSScriptRoot
$polarityOut='evidence/encoder-polarity-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$polarityOldPath=$env:PATH
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    foreach($polarityProject in @($polarityRoot,(Join-Path $polarityRoot 'diagnostics/jy901b-chassis'))) {
        Push-Location -LiteralPath $polarityProject
        try {
            [void][IO.Directory]::CreateDirectory((Join-Path $polarityProject $polarityOut))
            $polarityExe=$polarityOut+'/verify.exe'
            & $Gcc -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Isrc -Ilookup `
                tests/verify_encoder_polarity.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
                src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $polarityExe
            if($LASTEXITCODE -ne 0){throw 'Encoder polarity compilation failed'}
            foreach($polarityMode in 0..4) {
                & (Join-Path $polarityProject $polarityExe) $polarityMode | Tee-Object -FilePath ($polarityOut+'/mode-'+$polarityMode+'.log')
                if($LASTEXITCODE -ne 0){throw "Encoder polarity mode $polarityMode failed"}
            }
            $polarityHashes=[ordered]@{}
            foreach($polarityFile in @('src/main_table_nav.c','src/car_config.h','tests/verify_encoder_polarity.c','tests/verify_heading.c')) {
                $polarityHashes[$polarityFile]=(Get-FileHash -LiteralPath $polarityFile -Algorithm SHA256).Hash.ToLowerInvariant()
            }
            @{passed=$true;modes=5;sourceHashes=$polarityHashes;hardwareAccessed=$false} | ConvertTo-Json -Depth 5 |
                Set-Content -LiteralPath ($polarityOut+'/result.json') -Encoding UTF8
            Write-Output (Join-Path $polarityProject $polarityOut)
        } finally {Pop-Location}
    }
} finally {$env:PATH=$polarityOldPath}
