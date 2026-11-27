param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$loopRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$loopOut=Join-Path $loopRoot ('evidence/drive-loops-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($loopOut)
$loopCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$loopOldPath=$env:PATH
Push-Location -LiteralPath $loopRoot
try {
    $env:PATH=(Split-Path -Parent $loopCompiler)+';'+$env:PATH
    foreach($loopEncoder in @(0,1)) {foreach($loopGyro in @(0,1)) {
        $loopVariant="$loopEncoder-$loopGyro"
        $loopExe='evidence/'+(Split-Path -Leaf $loopOut)+'/'+$loopVariant+'.exe'
        & $loopCompiler -DARM_HOST_TEST ("-DCAR_ENCODER_DRIFT_DEFAULT_ON=$loopEncoder") ("-DCAR_GYRO_HEADING_DEFAULT_ON=$loopGyro") `
            -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup tests/verify_drive_loops.c `
            src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c `
            src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $loopExe
        if($LASTEXITCODE -ne 0){throw 'Drive loops compilation failed'}
        $loopText=& (Join-Path $loopRoot $loopExe) 2>&1;$loopExit=$LASTEXITCODE
        $loopText | Tee-Object -FilePath (Join-Path $loopOut ($loopVariant+'.log'))
        if($loopExit -ne 0){throw "Drive loops verification failed: $loopVariant"}
    }}
    $loopHashes=[ordered]@{}
    foreach($loopName in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/heading_sensor.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/verify_drive_loops.c','tools/verify-drive-loops.ps1')) {
        $loopHashes[$loopName]=(Get-FileHash -LiteralPath $loopName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$loopHashes;variants=@('0-0','0-1','1-0','1-1');hardwareAccessed=$false;
        scope='Runtime independent encoder drift and gyro translation feedback, fixed trims/distance retained, f/b/l/r and stationary turns, four reset-default combinations; segmented production'} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $loopOut 'result.json') -Encoding UTF8
    Write-Output $loopOut
} finally {$env:PATH=$loopOldPath;Pop-Location}
