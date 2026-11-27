param([string]$Gcc='gcc',[switch]$FinalReturnOnly)
$ErrorActionPreference='Stop'
$cornerRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$cornerOut=Join-Path $cornerRoot ('evidence/corner-route-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($cornerOut)
$cornerCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$cornerOldPath=$env:PATH
Push-Location -LiteralPath $cornerRoot
try {
    $env:PATH=(Split-Path -Parent $cornerCompiler)+';'+$env:PATH
    $cornerSuites=@('final_return')
    if(!$FinalReturnOnly){$cornerSuites+='corner_route'}
    foreach($cornerSuite in $cornerSuites) {
        foreach($cornerEnabled in @(0,1)) {
            $cornerExe='evidence/'+(Split-Path -Leaf $cornerOut)+"/verify_${cornerSuite}_$cornerEnabled.exe"
            & $cornerCompiler -DARM_HOST_TEST ('-DCAR_RUN_CORNER_ENABLED='+$cornerEnabled) -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
                "tests/verify_$cornerSuite.c" src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
                src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $cornerExe
            if($LASTEXITCODE -ne 0){throw 'Corner route host compilation failed'}
            $cornerText=& (Join-Path $cornerRoot $cornerExe) 2>&1
            $cornerExit=$LASTEXITCODE
            $cornerText | Tee-Object -FilePath (Join-Path $cornerOut "$cornerSuite-$cornerEnabled.log")
            if($cornerExit -ne 0){throw "Verification failed: $cornerSuite, corner=$cornerEnabled"}
        }
    }
    $cornerHashes=[ordered]@{}
    foreach($cornerName in @('src/main_table_nav.c','src/car_config.h','src/heading_control.c','src/heading_control.h',
        'src/heading_sensor.c','src/heading_sensor.h','src/road_map.c','src/full_route.h','src/structure.h',
        'src/arm_control.c','src/arm_config.h','src/station_job.c','src/station_config.h','src/vision_link.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c',
        'tests/verify_final_return.c','tests/verify_corner_route.c','tools/verify-corner-route.ps1')) {
        $cornerHashes[$cornerName]=(Get-FileHash -LiteralPath $cornerName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;sourceHashes=$cornerHashes;variants=@(0,1);suites=$cornerSuites;hardwareAccessed=$false;
        scope=if($FinalReturnOnly){'A/C final home offset merged on continuous return; segmented comparison, original endpoints and wheel-feedback stop'}else{'Fast transitions: six corners for A/C, fixed/changing yaw, per-wheel acceleration/braking/reversal, final home offset blended into B-A/B-C and visual terminal handoff, full jobs, coherent feedback, pause/resume and curved return; not vehicle acceptance'}} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $cornerOut 'result.json') -Encoding UTF8
    Write-Output $cornerOut
} finally {$env:PATH=$cornerOldPath;Pop-Location}
