param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$checkRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$checkOut=Join-Path $checkRoot ('evidence/continuous-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($checkOut)
$checkCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$checkOldPath=$env:PATH
Push-Location -LiteralPath $checkRoot
try {
    $env:PATH=(Split-Path -Parent $checkCompiler)+';'+$env:PATH
    $checkExe='evidence/'+(Split-Path -Leaf $checkOut)+'/verify_continuous.exe'
    & $checkCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_continuous.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c src/road_map.c `
        src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $checkExe
    if($LASTEXITCODE -ne 0){throw 'Continuous route test compilation failed'}
    $checkText=& (Join-Path $checkRoot $checkExe)
    $checkExit=$LASTEXITCODE
    $checkText | Tee-Object -FilePath (Join-Path $checkOut 'result.log')
    if($checkExit -ne 0){throw 'Continuous route/obstacle behavior test failed'}
    $checkHashes=[ordered]@{}
    foreach($name in @('src/main_table_nav.c','src/hmi_status.c','src/hmi_status.h','src/arm_control.c','src/arm_control.h','src/station_job.c','src/station_job.h','src/vision_link.c','src/vision_link.h','src/car_config.h','src/full_route.h','src/structure.h',
        'src/heading_control.c','src/heading_sensor.c','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_continuous.c','tools/verify-continuous.ps1')) {
        $checkHashes[$name]=(Get-FileHash -LiteralPath (Join-Path $checkRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$checkExit;scope='actual configured RPM; simulated nonstop node passing and obstacle return; no hardware accessed';sourceHashes=$checkHashes;output=$checkText} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $checkOut 'result.json') -Encoding UTF8
    Write-Output $checkOut
} finally {$env:PATH=$checkOldPath;Pop-Location}
