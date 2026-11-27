param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$entryRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$entryOut=Join-Path $entryRoot ('evidence/native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($entryOut)
$entryExe=Join-Path $entryOut 'verify_entry.exe'
$entryGccPath=(Get-Command $Gcc -ErrorAction Stop).Source
$entryOldPath=$env:PATH
Push-Location -LiteralPath $entryRoot
try {
    $env:PATH=(Split-Path -Parent $entryGccPath)+';'+$env:PATH
    $entryRelativeExe='evidence/'+(Split-Path -Leaf $entryOut)+'/verify_entry.exe'
    & $entryGccPath -DARM_HOST_TEST -std=c99 -Wall -Wextra -Werror -Isrc -Ilookup `
        tests/verify_entry.c src/arm_control.c src/station_job.c src/vision_link.c src/hmi_status.c src/motion.c src/navigation.c src/road_map.c src/heading_sensor.c src/heading_control.c `
        lookup/route_table.c -lm -o $entryRelativeExe
    if($LASTEXITCODE -ne 0){throw 'Host test compilation failed'}
    $entryText=& $entryExe
    $entryExit=$LASTEXITCODE
    $entryText | Tee-Object -FilePath (Join-Path $entryOut 'result.log')
    if($entryExit -ne 0){throw 'Entry behavior test failed'}
    [ordered]@{passed=$true;exitCode=$entryExit;scope='host command/state/motor submission checks; not board runtime';sourceSha256=(Get-FileHash -LiteralPath (Join-Path $entryRoot 'src/main_table_nav.c') -Algorithm SHA256).Hash.ToLowerInvariant();output=$entryText} |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $entryOut 'result.json') -Encoding UTF8
} finally {$env:PATH=$entryOldPath;Pop-Location}
