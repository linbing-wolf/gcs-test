param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$hmiRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$hmiOut=Join-Path $hmiRoot ('evidence/hmi-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($hmiOut)
$hmiCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
$hmiOldPath=$env:PATH
Push-Location -LiteralPath $hmiRoot
try {
    $env:PATH=(Split-Path -Parent $hmiCompiler)+';'+$env:PATH
    $variants=@()
    foreach($mapped in @(0,1)) {
        $defines=@()
        if($mapped){$defines=@('-DHMI_LF_ADDRESS=1','-DHMI_RF_ADDRESS=2','-DHMI_LR_ADDRESS=3','-DHMI_RR_ADDRESS=4')}
        $exe='evidence/'+(Split-Path -Leaf $hmiOut)+'/verify_hmi_'+$mapped+'.exe'
        & $hmiCompiler -DARM_HOST_TEST @defines -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
            tests/verify_hmi.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c `
            src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c lookup/route_table.c -lm -o $exe
        if($LASTEXITCODE -ne 0){throw 'HMI test compilation failed'}
        $output=& (Join-Path $hmiRoot $exe)
        $result=$LASTEXITCODE
        $output | Tee-Object -FilePath (Join-Path $hmiOut ('result-'+$mapped+'.log'))
        if($result -ne 0){throw 'HMI behavior test failed'}
        $variants+=@{testOnlyWheelMapping=[bool]$mapped;exitCode=$result;output=$output}
    }
    $hashes=[ordered]@{}
    $names=@(Get-ChildItem -LiteralPath 'src' -File | Where-Object {$_.Extension -in '.c','.h'} | ForEach-Object {'src/'+$_.Name})
    $names+=@('tests/verify_hmi.c','tests/verify_heading.c','tests/verify_entry.c','tests/entry_host_io.h',
        'tests/arm_host_io.h','tests/vision_host_io.h','tools/verify-hmi.ps1','keil/TableNavigation.uvprojx')
    foreach($name in $names){$hashes[$name]=(Get-FileHash -LiteralPath $name -Algorithm SHA256).Hash.ToLowerInvariant()}
    [ordered]@{passed=$true;exitCode=0;scope='read-only serial screen host integration; no screen or vehicle accessed; synthetic wheel mapping is test-only';sourceHashes=$hashes;variants=$variants} |
        ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $hmiOut 'result.json') -Encoding UTF8
    Write-Output $hmiOut
} finally {$env:PATH=$hmiOldPath;Pop-Location}
