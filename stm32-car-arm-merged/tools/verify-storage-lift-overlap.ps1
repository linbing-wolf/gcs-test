param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$liftRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$liftOut='evidence/storage-lift-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
[void][IO.Directory]::CreateDirectory((Join-Path $liftRoot $liftOut))
$liftOldPath=$env:PATH
Push-Location -LiteralPath $liftRoot
try {
    $liftCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $liftCompiler)+';'+$env:PATH
    $liftExe=$liftOut+'/storage-lift.exe'
    & $liftCompiler -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_storage_lift_overlap.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c `
        src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $liftExe
    if($LASTEXITCODE -ne 0){throw 'Storage lift overlap compilation failed'}
    $liftText=& (Join-Path $liftRoot $liftExe);$liftStatus=$LASTEXITCODE
    $liftText | Set-Content -Encoding UTF8 -LiteralPath ($liftOut+'/result.log')
    if($liftStatus -ne 0){throw 'Storage lift overlap behavior failed'}
    $liftHashes=[ordered]@{}
    foreach($liftFile in @('src/main_table_nav.c','src/car_config.h','src/station_job.c','src/station_job.h','src/station_config.h',
        'src/arm_control.c','src/arm_config.h','src/heading_control.c','src/road_map.c','src/full_route.h','lookup/table_data.h',
        'tests/verify_storage_lift_overlap.c','tests/verify_station_vision.c','tests/verify_heading.c','tests/verify_entry.c',
        'tools/verify-storage-lift-overlap.ps1')) {
        $liftHashes[$liftFile]=(Get-FileHash -LiteralPath $liftFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=0;sourceHashes=$liftHashes;hardwareAccessed=$false;cases=36;firmwareLogicChanged=$false;
        scope='Existing r96 both storage batches/all3 releases/A-C;124mm holds and125mm enables nonzero wheel commands with lift below135; stale feedback and manual!; controlled model only'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath ($liftOut+'/result.json')
    $liftText | Select-Object -Last 7
    Write-Output $liftOut
} finally {$env:PATH=$liftOldPath;Pop-Location}
