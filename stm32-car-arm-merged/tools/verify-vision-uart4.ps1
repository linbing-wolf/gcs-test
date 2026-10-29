param([string]$Gcc='C:/msys64/ucrt64/bin/gcc.exe')
$ErrorActionPreference='Stop'
$uartRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$uartRelative='evidence/vision-uart4-native-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$uartOut=Join-Path $uartRoot $uartRelative
[void][IO.Directory]::CreateDirectory($uartOut)
$uartOldPath=$env:PATH
Push-Location -LiteralPath $uartRoot
try {
    $uartCompiler=(Get-Command $Gcc -ErrorAction Stop).Source
    $env:PATH=(Split-Path -Parent $uartCompiler)+';'+$env:PATH
    $uartExe=$uartRelative+'/verify_vision_uart4.exe'
    & $uartCompiler -std=c99 -Wall -Wextra -Werror -Itests/vision_uart_stubs -Isrc tests/verify_vision_uart4.c src/vision_link.c -o $uartExe
    if($LASTEXITCODE -ne 0){throw 'UART4 hardware branch compilation failed'}
    $uartText=& (Join-Path $uartRoot $uartExe) 2>&1;$uartExit=$LASTEXITCODE
    $uartText | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $uartOut 'result.log')
    if($uartExit -ne 0){$uartText;throw 'UART4 hardware branch behavior failed'}
    $uartHashes=[ordered]@{}
    foreach($uartFile in @('src/vision_link.c','src/vision_link.h','vendor/stm32/lanya/usart5.c',
        'tests/verify_vision_uart4.c','tests/vision_uart_stubs/stm32f4xx.h','tools/verify-vision-uart4.ps1')) {
        $uartHashes[$uartFile]=(Get-FileHash -LiteralPath $uartFile -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    [ordered]@{passed=$true;exitCode=$uartExit;sourceHashes=$uartHashes;hardwareAccessed=$false;
        scope='Production hardware branch with mocked registers/peripherals: UART4 clocks,PC10/PC11 AF8,115200 8N1,IRQ RX/error and TXE-gated TX; real parser and FIFO; no physical communication acceptance'} |
        ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 -LiteralPath (Join-Path $uartOut 'result.json')
    $uartText;Write-Output $uartOut
} finally {$env:PATH=$uartOldPath;Pop-Location}
