param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$holdRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$holdOut=Join-Path $holdRoot ('evidence/straight-hold-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($holdOut)
$holdCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$holdOldPath=$env:PATH
Push-Location -LiteralPath $holdRoot
try {
    $env:PATH=(Split-Path -Parent $holdCompiler)+';'+$env:PATH
    foreach($holdEnabled in @(0,1)) {
        $holdExe='evidence/'+(Split-Path -Leaf $holdOut)+'/verify_straight_hold_'+$holdEnabled+'.exe'
        & $holdCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror ('-DCAR_STRAIGHT_HOLD_ENABLED='+$holdEnabled) `
            -Isrc -Ilookup tests/verify_straight_hold.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $holdExe
        if($LASTEXITCODE -ne 0){throw 'Straight hold test compilation failed'}
        $holdText=& (Join-Path $holdRoot $holdExe)
        $holdExit=$LASTEXITCODE
        $holdText | Tee-Object -FilePath (Join-Path $holdOut ('enabled-'+$holdEnabled+'.log'))
        if($holdExit -ne 0){throw 'Straight hold behavior test failed'}
    }
    $holdHashes=[ordered]@{}
    foreach($holdName in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','tests/verify_heading.c','tests/verify_straight_hold.c','tools/verify-straight-hold.ps1')) {
        $holdHashes[$holdName]=(Get-FileHash -LiteralPath $holdName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;enabledVariants=@(0,1);sourceHashes=$holdHashes;scope='Independent unequal-wheel simulation; not physical side-slip detection or vehicle acceptance';hardwareAccessed=$false} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $holdOut 'result.json') -Encoding UTF8
    Write-Output $holdOut
} finally {$env:PATH=$holdOldPath;Pop-Location}
