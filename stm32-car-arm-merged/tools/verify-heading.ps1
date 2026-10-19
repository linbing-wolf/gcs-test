param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$headingRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$headingOut=Join-Path $headingRoot ('evidence/heading-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($headingOut)
$headingGccPath=(Get-Command $Gcc -ErrorAction Stop).Source
$headingOldPath=$env:PATH
Push-Location -LiteralPath $headingRoot
try {
    $env:PATH=(Split-Path -Parent $headingGccPath)+';'+$env:PATH
    $headingOutputs=@()
    foreach($headingEnabled in @(0,1)) {
        $headingExe=Join-Path $headingOut ('verify_heading_'+$headingEnabled+'.exe')
        $headingRelativeExe='evidence/'+(Split-Path -Leaf $headingOut)+'/verify_heading_'+$headingEnabled+'.exe'
        & $headingGccPath -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror ('-DCAR_IMU_UART_ENABLED='+$headingEnabled) `
            -Isrc -Ilookup tests/verify_heading.c src/arm_control.c src/station_job.c src/vision_link.c src/motion.c src/navigation.c src/road_map.c `
            src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $headingRelativeExe
        if($LASTEXITCODE -ne 0){throw 'Heading test compilation failed'}
        $headingText=& $headingExe
        $headingExit=$LASTEXITCODE
        $headingText | Tee-Object -FilePath (Join-Path $headingOut ('heading-'+$headingEnabled+'.log'))
        if($headingExit -ne 0){throw 'Heading/feedback behavior test failed'}
        $headingOutputs+=@{imuUartEnabled=$headingEnabled;exitCode=$headingExit;output=$headingText}
    }
    $headingCanExe=Join-Path $headingOut 'verify_can_rx.exe'
    $headingRelativeCanExe='evidence/'+(Split-Path -Leaf $headingOut)+'/verify_can_rx.exe'
    & $headingGccPath -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror tests/verify_can_rx.c -o $headingRelativeCanExe
    if($LASTEXITCODE -ne 0){throw 'CAN IRQ test compilation failed'}
    & $headingCanExe | Tee-Object -FilePath (Join-Path $headingOut 'can-rx.log')
    if($LASTEXITCODE -ne 0){throw 'CAN IRQ test failed'}
    $headingHashes=[ordered]@{}
    foreach($headingName in @('src/arm_control.c','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c','src/vision_link.h','tests/vision_host_io.h','vendor/stm32/lanya/usart5.c','src/arm_control.h','src/arm_config.h','tests/arm_host_io.h','src/main_table_nav.c','src/car_config.h','src/heading_sensor.c','src/heading_sensor.h','src/heading_control.c','src/heading_control.h',
        'src/motion.c','src/navigation.c','src/road_map.c','lookup/route_table.c','lookup/table_data.h','lookup/generation.json',
        'src/full_route.h','src/structure.h','src/road_map.h','src/motion.h','src/navigation.h',
        'tests/verify_entry.c','tests/entry_host_io.h','tests/verify_heading.c','tests/verify_can_rx.c','tools/verify-heading.ps1',
        'vendor/stm32/can_ZDT/can.c','vendor/stm32/imu/imu.c','vendor/stm32/SYSTEM/Serial.c')) {
        $headingHashes[$headingName]=(Get-FileHash -LiteralPath (Join-Path $headingRoot $headingName) -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;scope='host mocks and simulated heading, not physical vehicle';legacyRunRemoved=$true;retainedEntryIncluded=$true;imuUartVariants=@(0,1);sourceHashes=$headingHashes;results=$headingOutputs} |
        ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $headingOut 'result.json') -Encoding UTF8
    Write-Output $headingOut
} finally {$env:PATH=$headingOldPath;Pop-Location}
