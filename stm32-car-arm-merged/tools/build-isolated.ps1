param([string]$CompilerBin='D:\keil5\ARM\ARMCC\bin')
$ErrorActionPreference='Stop'
$armBuildRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$armProjectDir=Join-Path $armBuildRoot 'keil'
$armBuildRelative='evidence/merged-keil-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$armBuildOut=Join-Path $armBuildRoot $armBuildRelative
[void][IO.Directory]::CreateDirectory($armBuildOut)
$armBuildProject=[xml](Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $armProjectDir 'TableNavigation.uvprojx'))
$armTarget=$armBuildProject.Project.Targets.Target
$armBuildLog=Join-Path $armBuildOut 'build.log'
function Invoke-ArmTool([string]$Tool,[string[]]$Arguments){
    Add-Content -LiteralPath $armBuildLog -Value ($Tool+' '+($Arguments -join ' ')) -Encoding UTF8
    $armPreference=$ErrorActionPreference
    $ErrorActionPreference='Continue'
    try {
        $armText=& (Join-Path $CompilerBin $Tool) @Arguments 2>&1
        $armExit=$LASTEXITCODE
    } finally {$ErrorActionPreference=$armPreference}
    $armLines=@($armText | ForEach-Object {$_.ToString()})
    $armLines | Add-Content -LiteralPath $armBuildLog -Encoding UTF8
    $armLines | Write-Output
    Add-Content -LiteralPath $armBuildLog -Value ('exitCode='+$armExit) -Encoding UTF8
    if($armExit -ne 0){throw "$Tool failed: exit $armExit"}
}
Push-Location -LiteralPath $armBuildRoot
try {
    $armIncludes=@($armTarget.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath.Split(';') | ForEach-Object {
        '-I'+[IO.Path]::GetFullPath((Join-Path $armProjectDir $_)).Substring($armBuildRoot.Length+1)
    })
    $armObjects=@();$armUnits=0
    foreach($armFile in $armTarget.Groups.Group.Files.File){
        if($armFile.FileType -notin @('1','2')){continue}
        $armSource=[IO.Path]::GetFullPath((Join-Path $armProjectDir $armFile.FilePath)).Substring($armBuildRoot.Length+1)
        $armObject=$armBuildRelative+'/'+$armUnits+'_'+[IO.Path]::GetFileNameWithoutExtension($armSource)+'.o'
        if($armFile.FileType -eq '1'){
            Invoke-ArmTool 'armcc.exe' (@('--cpu','Cortex-M4.fp.sp','--c99','--apcs=interwork','-g','-O0',
                '-DSTM32F40_41xxx','-DUSE_STDPERIPH_DRIVER')+$armIncludes+@('-c',$armSource,'-o',$armObject))
        }else{
            Invoke-ArmTool 'armasm.exe' @('--cpu','Cortex-M4.fp.sp','--apcs=interwork','-g',$armSource,'-o',$armObject)
        }
        $armObjects+=$armObject;$armUnits++
    }
    $armAxf=$armBuildRelative+'/table_navigation.axf'
    Invoke-ArmTool 'armlink.exe' (@('--cpu','Cortex-M4.fp.sp','--scatter','keil/navigation.sct',
        '--map','--callgraph','--list',($armBuildRelative+'/table_navigation.map'),'--info','sizes','--output',$armAxf)+$armObjects)
    Invoke-ArmTool 'fromelf.exe' @('--i32combined','--output',($armBuildRelative+'/table_navigation.hex'),$armAxf)
    Write-Output ("PASS ARM full rebuild: $armUnits units, all tool exit codes 0; no hardware accessed.")
    Write-Output $armBuildOut
} finally {Pop-Location}
