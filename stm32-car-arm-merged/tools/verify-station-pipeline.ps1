param([string]$Gcc='gcc',[string]$BaselineRoot='')
$ErrorActionPreference='Stop'
$pipelineRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$pipelineOut='evidence/station-pipeline-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $pipelineRoot $pipelineOut))
$pipelineCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$pipelineOldPath=$env:PATH
Push-Location -LiteralPath $pipelineRoot
try {
    $env:PATH=(Split-Path -Parent $pipelineCompiler)+';'+$env:PATH
    & $pipelineCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_station_pipeline.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c `
        -lm -o ($pipelineOut+'/verify_station_pipeline.exe')
    if($LASTEXITCODE -ne 0){throw 'Station pipeline compilation failed'}
    $pipelineText=& ($pipelineRoot+'/'+$pipelineOut+'/verify_station_pipeline.exe') 2>&1
    $pipelineExit=$LASTEXITCODE
    $pipelineText | Tee-Object -FilePath ($pipelineOut+'/result.log')
    if($pipelineExit -ne 0){throw 'Station pipeline behavior failed'}
    $pipelineHashes=[ordered]@{}
    foreach($pipelineFile in @('src/main_table_nav.c','src/car_config.h','src/arm_control.c','src/arm_control.h',
        'src/arm_config.h','src/station_job.c','src/station_job.h','src/station_config.h','src/vision_link.c',
        'tests/verify_entry.c','tests/verify_heading.c','tests/verify_station_vision.c','tests/vision_movexy_cases.h',
        'tests/verify_station_pipeline.c','tests/benchmark_station_pipeline.c','tools/verify-station-pipeline.ps1')) {
        $pipelineHashes[$pipelineFile]=(Get-FileHash -LiteralPath $pipelineFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    $pipelineComparison=[ordered]@{}
    if($BaselineRoot) {
        foreach($pipelineLabel in @('before','after')) {
            $pipelineBase=if($pipelineLabel -eq 'before'){$BaselineRoot}else{'.'}
            $pipelineUnits=@('arm_control','station_job','vision_link','hmi_status','motion','navigation','road_map','heading_sensor','heading_control') | ForEach-Object {$pipelineBase+'/src/'+$_+'.c'}
            & $pipelineCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror ('-I'+$pipelineBase+'/tests') `
                ('-I'+$pipelineBase+'/src') ('-I'+$pipelineBase+'/lookup') tests/benchmark_station_pipeline.c `
                @pipelineUnits ($pipelineBase+'/lookup/route_table.c') -lm -o ($pipelineOut+'/benchmark-'+$pipelineLabel+'.exe')
            if($LASTEXITCODE -ne 0){throw 'Pipeline comparison compilation failed'}
            $pipelineBenchText=& ($pipelineRoot+'/'+$pipelineOut+'/benchmark-'+$pipelineLabel+'.exe') 2>&1
            $pipelineBenchExit=$LASTEXITCODE
            $pipelineBenchText | Tee-Object -FilePath ($pipelineOut+'/benchmark-'+$pipelineLabel+'.log')
            if($pipelineBenchExit -ne 0){throw 'Pipeline comparison behavior failed'}
            $pipelineComparison[$pipelineLabel]=@{sourceRoot=$pipelineBase;exitCode=$pipelineBenchExit;output=$pipelineBenchText}
        }
    }
    [ordered]@{passed=$true;exitCode=$pipelineExit;sourceHashes=$pipelineHashes;hardwareAccessed=$false;
        scope='Measured clearance, missing lift/wheel feedback, early observation yaw, onboard work during indexes, ground-action joins, full A/C routes and unchanged visual counts';output=$pipelineText;timingComparison=$pipelineComparison} |
        ConvertTo-Json -Depth 6 | Set-Content -LiteralPath ($pipelineOut+'/result.json') -Encoding UTF8
    Write-Output $pipelineOut
} finally {$env:PATH=$pipelineOldPath;Pop-Location}
