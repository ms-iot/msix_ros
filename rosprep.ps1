## make the ROS install relocatable
[CmdletBinding()]
param(
    $badParam,
    [Parameter(Mandatory=$False)][string]$inputRosDir = "c:\opt\"
)
$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest
# Powershell2-compatible way of forcing named-parameters
if ($badParam)
{
    throw "Only named parameters are allowed"
}

$scriptsDir = split-path -parent $script:MyInvocation.MyCommand.Definition

Import-Module $scriptsDir\sdktools.psm1 -WarningAction SilentlyContinue -Force

$msbuildExeWithPlatformToolset = findAnyMSBuildWithCppPlatformToolset ""
$vcFolder = $msbuildExeWithPlatformToolset[2]

Get-ChildItem (Join-Path $vcFolder "*dumpbin.exe") -Recurse | % {
    # TODO: a more deterministic way to look for dumpbin
    $dumpbin = $_.FullName
}

Set-Alias dumpbin $dumpbin -Scope Script

New-Item -ItemType Directory -Path (Join-Path $scriptsDir "log") -Force | Out-Null

Set-Alias ruplacer (Join-Path $scriptsDir "ruplacer\ruplacer.exe") -Scope Script

$outputRosDir = (Join-Path $scriptsDir "working")

Push-Location $scriptsDir

Write-Host "`nRemoving working folder ... `n"

Remove-Item "$outputRosDir" -Force -Recurse -ErrorAction SilentlyContinue

Write-Host "`nRemoving working folder ... done`n"

Write-Host "`nCopying ROS install ... `n"

# exclude development time required files
# * some packages have runtime dependency on pkg-config
#   so we don't remove them for now.
$distro = 'distribution'



$developmentFiles = @(
    '*.lib',
    '*.pdb',
    '*.dll.a',
    '*.cmake'
)


[System.Collections.ArrayList]$runtimeFolders = @(
    "install"
    "ros\$distro\x64\Scripts"
    "ros\$distro\x64\plugins",
    "ros\$distro\x64\bin",
    "ros\$distro\x64\share"
    "ros\$distro\x64\etc"
)

if ($distro -eq 'melodic') {



    $dllFolder = "ros\$distro\x64\DLLs"

    $inputDir = (Join-Path $inputRosDir $dllFolder)
    $outputDir = (Join-Path $outputRosDir "ros\$distro\x64\")
    $dllArgs = @{
        Path = $inputDir
        Recurse = $True
        Destination = $outputDir
        Container = $True
        Exclude = $developmentFiles
        
    }
    Copy-Item @dllArgs 
    Write-Host "`n done DLLs`n"
    

    $libFolder = "ros\$distro\x64\lib"
    $inputDir = (Join-Path $inputRosDir $libFolder)
    $outputDir = (Join-Path $outputRosDir "ros\$distro\x64\Lib")
    $libArgs = @{
        Path = $inputDir
        Recurse = $True
        Destination = $outputDir
        Container = $True
        Exclude = $developmentFiles
    }
    Copy-Item @libArgs

    Write-Host "`n done Lib`n"
    
    $mediaFolder = "ros\$distro\x64\media"
    $inputDir = (Join-Path $inputRosDir $mediaFolder)
    $outputDir = (Join-Path $outputRosDir "ros\$distro\x64\Media")
    $mediaArgs = @{
        Path = $inputDir
        Recurse = $True
        Destination = $outputDir
        Container = $True
        Exclude = $developmentFiles
    }
    Copy-Item @mediaArgs

    Write-Host "`n done Media`n"
}
elseif ($distro -eq 'noetic') {
    $runtimeFolders += "ros\$distro\x64\Media"
    $runtimeFolders += "ros\$distro\x64\Lib"    
}
else {
    $runtimeFolders.Remove("ros\$distro\x64\etc")
    $runtimeFolders += "ros\$distro\x64\Media"
    $runtimeFolders += "ros\$distro\x64\Lib"    
}


foreach ($runtimeFolder in $runtimeFolders) {
    $inputDir = (Join-Path $inputRosDir $runtimeFolder)
    $outputDir = (Join-Path $outputRosDir $runtimeFolder)
    $arguments = @{
        Path = $inputDir
        Recurse = $True
        Destination = $outputDir
        Container = $True
        Exclude = $developmentFiles
    }
    Copy-Item @arguments
}


# Copying the files under the root folder.
Copy-Item -Path (Join-Path $inputRosDir "ros\$distro\x64\*.*") -Destination (Join-Path $outputRosDir "ros\$distro\x64") -Container

Write-Host "`nCopying ROS install ... done`n"

Write-Host "`nFixing up shared file location ...`n"

$fixupFolder = "ros\$distro\x64"


$outputDir = (Join-Path $outputRosDir $fixupFolder)
$sharedLibs = (Join-Path $outputDir "*.dll")
$binDir = (Join-Path $outputDir "bin")
$fixupArgs = @{
    Path = $sharedLibs
    Destination = $binDir
    Force = $True
}
Move-Item @fixupArgs


# $runtimeFolders += $fixupFolder
$searchPattern = @()
foreach ($runtimeFolder in $runtimeFolders) {
    $searchDir = (Join-Path $outputRosDir $runtimeFolder)
    if (-Not (Test-Path -Path $searchDir -PathType Container)) {
        continue
    }   
    $executables = (Join-Path $searchDir "*.exe")
    $searchPattern += $executables
}

$roscppPattern = @(
    'roscpp.dll'
)

$roscppNodes = @()
Get-ChildItem $searchPattern -Recurse | % {
    $output = & $dumpbin /dependents $_.FullName | Select-String -Pattern $roscppPattern
    

    if ($output) {
        $roscppNodes += $_.FullName.Replace($outputRosDir, 'VFS\Common AppData')
        # Write-Host "`n$roscppNodes`n"
    }
}


Write-Host "`nGenerating the baseline firewall rules ... `n"

[xml]$doc = New-Object System.Xml.XmlDocument

$desktop2uri = "http://schemas.microsoft.com/appx/manifest/desktop/windows10/2"
$root = $doc.CreateNode("element","Extensions",$null)
$root.SetAttribute("xmlns:desktop2",$desktop2uri) | Out-Null

$roscppNodes | % {

<#
    Generate the firewall rules like below:
    <desktop2:Extension Category="windows.firewallRules">
      <desktop2:FirewallRules Executable="VFS\Common AppData\ros\abc.exe">
        <desktop2:Rule Direction="in" IPProtocol="TCP" Profile="private" />
        <desktop2:Rule Direction="in" IPProtocol="UDP" Profile="private" />
      </desktop2:FirewallRules>
    </desktop2:Extension>
#>
    $extension = $doc.CreateNode("element","desktop2:Extension",$desktop2uri)
    $extension.SetAttribute("Category","windows.firewallRules") | Out-Null

    $firewallRules = $doc.CreateNode("element","desktop2:FirewallRules",$desktop2uri)
    $firewallRules.SetAttribute("Executable","$_") | Out-Null

    $tcpRule = $doc.CreateNode("element","desktop2:Rule",$desktop2uri)
    $tcpRule.SetAttribute("Direction","in") | Out-Null
    $tcpRule.SetAttribute("IPProtocol","TCP") | Out-Null
    $tcpRule.SetAttribute("Profile","private") | Out-Null

    $udpRule = $doc.CreateNode("element","desktop2:Rule",$desktop2uri)
    $udpRule.SetAttribute("Direction","in") | Out-Null
    $udpRule.SetAttribute("IPProtocol","UDP") | Out-Null
    $udpRule.SetAttribute("Profile","private") | Out-Null

    $firewallRules.AppendChild($tcpRule) | Out-Null
    $firewallRules.AppendChild($udpRule) | Out-Null

    $extension.AppendChild($firewallRules) | Out-Null

    $root.AppendChild($extension) | Out-Null
}

$doc.AppendChild($root) | Out-Null

$doc.Save("$outputRosDir\firewall.xml")

Write-Host "`nGenerating the baseline firewall rules ... done`n"

Write-Host "`nFixing up hard-coded PREFIX ... `n"

ruplacer "C:/opt" "C:/ProgramData" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer1.log")
ruplacer "c:\opt" "c:\ProgramData" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer2.log")
ruplacer "C:\opt" "C:\ProgramData" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer3.log")
if ($distro -ne 'foxy') {
ruplacer "C:\\opt" "C:\\ProgramData" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer4.log")
}
Write-Host "`nFixing up hard-coded PREFIX ... done`n"
