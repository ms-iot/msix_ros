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
$developmentFiles = @(
    '*.lib',
    '*.pdb',
    '*.dll.a',
    '*.cmake',
    '*.pc'
)

$runtimeFolders = @(
    'install',
    'python27amd64',
    'rosdeps\x64\bin',
    'rosdeps\x64\lib',
    'rosdeps\x64\share',
    'rosdeps\x64\etc',
    'rosdeps\x64\media',
    'rosdeps\x64\plugins',
    'ros\melodic\x64\bin',
    'ros\melodic\x64\lib',
    'ros\melodic\x64\share',
    'ros\melodic\x64\etc'
)

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
Copy-Item -Path (Join-Path $inputRosDir 'ros\melodic\x64\*.*') -Destination (Join-Path $outputRosDir 'ros\melodic\x64') -Container

Write-Host "`nCopying ROS install ... done`n"

Write-Host "`nGenerating the baseline firewall rules ... `n"

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
    }
}

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
ruplacer "C:\\opt" "C:\\ProgramData" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer4.log")
ruplacer "_parent_package_path.py" "%LOCALAPPDATA%\_parent_package_path.py" "$outputRosDir" --no-regex --color never --go | Out-File -FilePath (Join-Path $scriptsDir "log\ruplacer5.log")

Write-Host "`nFixing up hard-coded PREFIX ... done`n"
