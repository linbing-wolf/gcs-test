param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$omniRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$omniOut=Join-Path $omniRoot ('evidence/omni-route-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($omniOut)
$omniCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$omniOldPath=$env:PATH
Push-Location -LiteralPath $omniRoot
try {
    $env:PATH=(Split-Path -Parent $omniCompiler)+';'+$env:PATH
    $omniResults=@()
    foreach($omniMode in @(0,1)) {
        $omniExe='evidence/'+(Split-Path -Leaf $omniOut)+'/verify_omni_route_'+$omniMode+'.exe'
        & $omniCompiler -DARM_HOST_TEST ('-DCAR_RUN_OMNI_ENABLED='+$omniMode) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_omni_route.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
            src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $omniExe
        if($LASTEXITCODE -ne 0){throw 'Omni route compilation failed'}
        $omniText=& (Join-Path $omniRoot $omniExe)
        $omniExit=$LASTEXITCODE
        $omniText | Tee-Object -FilePath (Join-Path $omniOut ('result-'+$omniMode+'.log'))
        if($omniExit -ne 0){throw 'Omni route behavior test failed'}
        $omniResults+=@{omniEnabled=$omniMode;testOnlyLegacyOverride=($omniMode -eq 0);exitCode=$omniExit}
    }
    $omniHashes=[ordered]@{}
    foreach($omniName in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/heading_control.h',
        'src/arm_control.c','src/arm_config.h','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c',
        'src/road_map.c','src/structure.h','src/full_route.h','tests/verify_entry.c','tests/verify_heading.c',
        'tests/verify_station_vision.c','tests/vision_movexy_cases.h','tests/verify_omni_route.c','tools/verify-omni-route.ps1')) {
        $omniHashes[$omniName]=(Get-FileHash -LiteralPath $omniName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$omniHashes;variants=$omniResults;
        scope='All-road heading retention and independent map projection; current and test-only legacy comparison; no hardware';hardwareAccessed=$false} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $omniOut 'result.json') -Encoding UTF8
    Write-Output $omniOut
} finally {$env:PATH=$omniOldPath;Pop-Location}
