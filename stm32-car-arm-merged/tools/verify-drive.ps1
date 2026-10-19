param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$entryRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$entryOut=Join-Path $entryRoot ('evidence/drive-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($entryOut)
$entryExe=Join-Path $entryOut 'verify_drive.exe'
$entryGccPath=(Get-Command $Gcc -ErrorAction Stop).Source
$entryOldPath=$env:PATH
Push-Location -LiteralPath $entryRoot
try {
    $env:PATH=(Split-Path -Parent $entryGccPath)+';'+$env:PATH
    $entryRelativeExe='evidence/'+(Split-Path -Leaf $entryOut)+'/verify_drive.exe'
    & $entryGccPath -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_drive.c src/arm_control.c src/station_job.c src/vision_link.c src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c `
        lookup/route_table.c -lm -o $entryRelativeExe
    if($LASTEXITCODE -ne 0){throw 'Host test compilation failed'}
    $entryText=& $entryExe
    $entryExit=$LASTEXITCODE
    $entryText | Tee-Object -FilePath (Join-Path $entryOut 'result.log')
    if($entryExit -ne 0){throw 'Drive control behavior test failed'}
    $driveHashes=[ordered]@{}
    foreach($driveName in @('src/main_table_nav.c','src/arm_control.c','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','src/vision_link.h','tests/vision_host_io.h','vendor/stm32/lanya/usart5.c','src/arm_control.h','src/arm_config.h',
        'src/car_config.h','src/heading_control.c','src/heading_control.h',
        'tests/verify_entry.c','tests/verify_drive.c','tests/verify_heading.c','tests/entry_host_io.h','tests/arm_host_io.h','tools/verify-drive.ps1')) {
        $driveHashes[$driveName]=(Get-FileHash -LiteralPath (Join-Path $entryRoot $driveName) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{sourceHashes=$driveHashes;passed=$true;exitCode=$entryExit;scope='host drive heading/ramp/stop/log checks and asymmetric-wheel simulation; not physical accuracy';sourceSha256=(Get-FileHash -LiteralPath (Join-Path $entryRoot 'src/main_table_nav.c') -Algorithm SHA256).Hash.ToLowerInvariant();output=$entryText} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $entryOut 'result.json') -Encoding UTF8
} finally {$env:PATH=$entryOldPath;Pop-Location}
