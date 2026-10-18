param([string]$PackageRoot=(Join-Path $PSScriptRoot '..'))
$ErrorActionPreference='Stop'
$packageRootPath=[IO.Path]::GetFullPath($PackageRoot).TrimEnd([IO.Path]::DirectorySeparatorChar)
$packagePrefix=$packageRootPath+[IO.Path]::DirectorySeparatorChar
function Resolve-PackageInput([string]$Base,[string]$Relative){
    if([IO.Path]::IsPathRooted($Relative)){throw "Absolute project path: $Relative"}
    $packageInput=[IO.Path]::GetFullPath((Join-Path $Base $Relative))
    if(!$packageInput.StartsWith($packagePrefix,[StringComparison]::OrdinalIgnoreCase)){throw "Path outside package: $Relative"}
    if(!(Test-Path -LiteralPath $packageInput)){throw "Missing input: $Relative"}
    return $packageInput
}
$packageManifest=Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $packageRootPath 'package-manifest.json')|ConvertFrom-Json
foreach($packageEntry in $packageManifest.files.PSObject.Properties){
    $packageInput=Resolve-PackageInput $packageRootPath $packageEntry.Name
    if((Get-FileHash -LiteralPath $packageInput -Algorithm SHA256).Hash.ToLowerInvariant() -ne $packageEntry.Value){throw "File changed or transfer mismatch: $($packageEntry.Name)"}
}
$packageGeneration=Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $packageRootPath 'lookup/generation.json')|ConvertFrom-Json
if(!$packageGeneration.validation.StartsWith('passed:')){throw 'Offline route table has not passed exhaustive validation'}
if($packageGeneration.tableDataSha256 -ne (Get-FileHash -LiteralPath (Join-Path $packageRootPath 'lookup/table_data.h') -Algorithm SHA256).Hash.ToLowerInvariant()){throw 'Generated route table hash mismatch'}
foreach($packageSource in $packageGeneration.sourceHashes.PSObject.Properties){
    if($packageSource.Name -eq 'src/car_config.h'){continue} # 速度等无关配置可调；内移参数在下面单独核对。
    $packageInput=Resolve-PackageInput $packageRootPath $packageSource.Name
    if((Get-FileHash -LiteralPath $packageInput -Algorithm SHA256).Hash.ToLowerInvariant() -ne $packageSource.Value){throw "Stale route table: rerun tools/rebuild-route-table.ps1 after changing $($packageSource.Name)"}
}
$packageConfig=Get-Content -Raw -LiteralPath (Join-Path $packageRootPath 'src/car_config.h')
foreach($packageParameter in @(@('CAR_MAP_TOP_INSET_MM','topInsetMm'),@('CAR_MAP_BOTTOM_INSET_MM','bottomInsetMm'),@('CAR_MAP_RAW_EXTRA_INSET_MM','rawExtraInsetMm'),@('CAR_MAP_STORAGE_INSET_MM','storageInsetMm'))){
    $packageMatch=[regex]::Match($packageConfig,'(?m)^#define\s+'+$packageParameter[0]+'\s+(\d+)\s*(?:/\*.*?\*/\s*)?$')
    if(!$packageMatch.Success -or [int]$packageMatch.Groups[1].Value -ne $packageGeneration.parameters.($packageParameter[1])){throw 'Map inset differs from generated table; rerun tools/rebuild-route-table.ps1'}
}
$packageProjectDir=Join-Path $packageRootPath 'keil'
$packageProject=[xml](Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $packageProjectDir 'TableNavigation.uvprojx'))
$packageTarget=$packageProject.Project.Targets.Target
$packageUnits=0
foreach($packageFile in $packageTarget.Groups.Group.Files.File){
    [void](Resolve-PackageInput $packageProjectDir $packageFile.FilePath)
    if($packageFile.FileType -in @('1','2')){$packageUnits++}
}
foreach($packageInclude in $packageTarget.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';')){
    if($packageInclude){[void](Resolve-PackageInput $packageProjectDir $packageInclude)}
}
if($packageUnits -ne 67){throw "Unexpected compilation unit count: $packageUnits"}
Write-Output ('PASS package: '+@($packageManifest.files.PSObject.Properties).Count+' files match; generated route table matches map inputs; 67 units and include paths stay inside package. No hardware accessed.')
