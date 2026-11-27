param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$i2cRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$i2cRelative='evidence/vision-soft-i2c-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$i2cOut=Join-Path $i2cRoot $i2cRelative
[void][IO.Directory]::CreateDirectory($i2cOut)
$i2cOldPath=$env:PATH
Push-Location -LiteralPath $i2cRoot
try {
    $i2cCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $i2cCompiler)+';'+$env:PATH
    $i2cExe=$i2cRelative+'/verify_vision_i2c.exe'
    & $i2cCompiler -std=c99 -Wall -Wextra -Werror -Itests/vision_i2c_stubs -Isrc tests/verify_vision_i2c.c -o $i2cExe
    if($LASTEXITCODE -ne 0){throw 'Software I2C edge branch compilation failed'}
    $i2cText=& (Join-Path $i2cRoot $i2cExe) 2>&1;$i2cExit=$LASTEXITCODE
    $i2cText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $i2cOut 'result.log')
    if($i2cExit -ne 0){$i2cText;throw 'Software I2C edge branch behavior failed'}
    $i2cHashes=[ordered]@{}
    foreach($i2cFile in @('src/vision_link.c','src/vision_link.h','src/vision_soft_i2c.h','tests/verify_vision_i2c.c',
        'tests/vision_i2c_stubs/stm32f4xx.h','tools/verify-vision-i2c.ps1','src/main_table_nav.c')) {
        $i2cHashes[$i2cFile]=(Get-FileHash -LiteralPath $i2cFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$i2cExit;sourceHashes=$i2cHashes;hardwareAccessed=$false;
        scope='Production software slave edge model: I2C mailbox32/0x42, interrupt arrival timestamps/error boundaries, delayed foreground vs true201ms gap, lost first/middle fragments, CRC/ring full, duplicate terminal, request delivery and R/READY isolation; no camera ACK/sequence protocol and no physical timing acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $i2cOut 'result.json')
    $i2cText;Write-Output $i2cOut
} finally {$env:PATH=$i2cOldPath;Pop-Location}
