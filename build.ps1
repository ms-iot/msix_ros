[CmdletBinding()]
param(
    $badParam,
    [Parameter(Mandatory=$False)][string]$certFile = "test.pfx",
    [Parameter(Mandatory=$False)][string]$certPassword = "1234",
    [Parameter(Mandatory=$False)][string]$packagingLayout = "example\PackagingLayout.xml"
)
$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest
# Powershell2-compatible way of forcing named-parameters
if ($badParam)
{
    throw "Only named parameters are allowed"
}

$scriptsDir = split-path -parent $script:MyInvocation.MyCommand.Definition

Push-Location $scriptsDir

Import-Module $scriptsDir\sdktools.psm1 -WarningAction SilentlyContinue -Force

Set-Alias makeappx (Find-Tool "makeappx.exe") -Scope Script
Set-Alias signtool (Find-Tool "signtool.exe") -Scope Script

$msbuildExeWithPlatformToolset = findAnyMSBuildWithCppPlatformToolset ""
$msbuildExe = $msbuildExeWithPlatformToolset[0]
$platformToolset = $msbuildExeWithPlatformToolset[1]
$windowsSDK = getWindowsSDK -withWinSDK ""

Write-Host "$msbuildExeWithPlatformToolset"
Write-Host "$msbuildExe"
Write-Host "$platformToolset"
Write-Host "$windowsSDK"

function invokeCommandClean()
{
    param ( [Parameter(Mandatory=$true)][string]$executable,
                                        [string]$arguments = "")

    Write-Verbose "Clean-Executing: ${executable} ${arguments}"
    $scriptsDir = split-path -parent $script:MyInvocation.MyCommand.Definition
    $cleanEnvScript = "$scriptsDir\cleanEnvironmentHelper.ps1"
    $tripleQuotes = "`"`"`""
    $argumentsWithEscapedQuotes = $arguments -replace "`"", $tripleQuotes
    $command = ". $tripleQuotes$cleanEnvScript$tripleQuotes; & $tripleQuotes$executable$tripleQuotes $argumentsWithEscapedQuotes"
    $arg = "-NoProfile", "-ExecutionPolicy Bypass", "-command $command"

    $process = Start-Process -FilePath powershell.exe -ArgumentList $arg -PassThru -NoNewWindow
    Wait-Process -InputObject $process
    $ec = $process.ExitCode
    Write-Verbose "Execution terminated with exit code $ec."
    return $ec
}

$arguments = (
"/p:Configuration=Release",
"/p:Platform=x64",
"/p:PlatformToolset=$platformToolset",
"/p:TargetPlatformVersion=$windowsSDK",
"/t:Rebuild",
"/verbosity:minimal",
"/m",
"/nologo",
"`"$scriptsDir\Launcher\Launcher.sln`"") -join " "


Write-Host "`nBuilding Launcher.exe ...`n"
$ec = invokeCommandClean $msbuildExe $arguments

if ($ec -ne 0)
{
    Write-Error "Building Launcher.exe failed. Please ensure you have installed Visual Studio with the Desktop C++ workload and the Windows SDK for Desktop C++."
    return
}

Write-Host "`nBuilding Launcher.exe ... done`n"

Remove-Item "$scriptsDir\output" -Force -Recurse -ErrorAction SilentlyContinue
if (Test-Path "$scriptsDir\output" -PathType Container) {
    throw "cannot remove $scriptsDir\output"
}

Write-Host "`nBuilding MSIX bundle ... `n"

makeappx build /f $packagingLayout /op output\ /v
if (-not $?) {
    throw "makeappx step failed"
}

Write-Host "`nBuilding MSIX bundle ... done`n"

Write-Host "`nSigning MSIX bundle ...`n"

Get-ChildItem "$scriptsDir\output\*.msix*" | % {
    signtool sign /fd SHA256 /a /f $certFile /p $certPassword $_.FullName
}

Write-Host "`nSigning MSIX bundle ... done`n"
