param([string]$Node='D:/nodejs/node.exe',[string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$mapRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$mapOut=Join-Path $mapRoot ('evidence/map-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
[void][IO.Directory]::CreateDirectory($mapOut)
$mapOldPath=$env:PATH
Push-Location -LiteralPath $mapRoot
try {
    $env:PATH=(Split-Path -Parent $Gcc)+';'+$env:PATH
    $mapRelativeOut='evidence/'+(Split-Path -Leaf $mapOut)
    $mapExporter=Join-Path $mapOut 'export_map.exe'
    & $Gcc -std=c99 -O2 -Wall -Wextra -Werror -Isrc tests/export_map.c src/road_map.c -o ($mapRelativeOut+'/export_map.exe')
    if($LASTEXITCODE -ne 0){throw 'Map export compilation failed'}
    $mapRaw=& $mapExporter
    if($LASTEXITCODE -ne 0){throw 'Map geometry validation failed'}
    $mapInput=$mapRaw|ConvertFrom-Json
    if(!$mapInput.geometryValidationPassed){throw 'Map geometry not validated'}
    $mapHashes=[ordered]@{}
    foreach($mapName in @('src/car_config.h','src/structure.h','src/road_map.c','lookup/route_table.c',
        'tests/export_map.c','tests/reference_route.c','tests/reference_route.h','tests/verify_route_table.c',
        'tools/generate-route-table.cjs','tools/rebuild-route-table.ps1')) {
        $mapHashes[$mapName]=(Get-FileHash -LiteralPath $mapName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    $mapInput|Add-Member -NotePropertyName sourceHashes -NotePropertyValue $mapHashes
    $mapInputPath=Join-Path $mapOut 'map-input.json'
    $mapInput|ConvertTo-Json -Depth 10|Set-Content -LiteralPath $mapInputPath -Encoding UTF8
    & $Node tools/generate-route-table.cjs $mapInputPath
    if($LASTEXITCODE -ne 0){throw 'Offline route table generation failed'}
    $mapVerifier=Join-Path $mapOut 'verify_route_table.exe'
    & $Gcc -std=c99 -O2 -Wall -Wextra -Werror -DROUTE_TABLE_VERIFY_STORAGE -Isrc -Ilookup -Itests `
        tests/verify_route_table.c tests/reference_route.c src/road_map.c lookup/route_table.c -o ($mapRelativeOut+'/verify_route_table.exe')
    if($LASTEXITCODE -ne 0){throw 'Route verifier compilation failed'}
    Push-Location -LiteralPath $mapOut
    try {
        & $mapVerifier | Tee-Object -FilePath 'verification.log'
        if($LASTEXITCODE -ne 0){throw 'Exhaustive route verification failed'}
    } finally {Pop-Location}
    $mapValidation=Get-Content -Raw (Join-Path $mapOut 'validation-result.json')|ConvertFrom-Json
    if($mapValidation.states -ne 531441 -or $mapValidation.queries -ne 43046721 -or !$mapValidation.exactBaselineRoute -or !$mapValidation.independentFloyd){throw 'Incomplete route validation'}
    $mapGeneration=Get-Content -Raw lookup/generation.json|ConvertFrom-Json
    if($mapGeneration.tableDataSha256 -ne (Get-FileHash lookup/table_data.h -Algorithm SHA256).Hash.ToLowerInvariant()){throw 'Generated table hash mismatch'}
    $mapGeneration.validation='passed: all 531441 states / 43046721 exact C-reference queries and independent Floyd'
    $mapGeneration|ConvertTo-Json -Depth 10|Set-Content -LiteralPath lookup/generation.json -Encoding UTF8
    $mapHashes['lookup/table_data.h']=(Get-FileHash lookup/table_data.h -Algorithm SHA256).Hash.ToLowerInvariant()
    $mapHashes['lookup/generation.json']=(Get-FileHash lookup/generation.json -Algorithm SHA256).Hash.ToLowerInvariant()
    [ordered]@{passed=$true;sourceHashes=$mapHashes;map=$mapInput;validation=$mapValidation;hardwareRun=$false}|
        ConvertTo-Json -Depth 12|Set-Content -LiteralPath (Join-Path $mapOut 'result.json') -Encoding UTF8
    Write-Output $mapOut
} finally {$env:PATH=$mapOldPath;Pop-Location}
