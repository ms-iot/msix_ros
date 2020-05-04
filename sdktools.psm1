# adopted from 
# https://github.com/python/cpython/blob/master/Tools/msi/sdktools.psm1
# https://github.com/microsoft/vcpkg/blob/master/scripts/bootstrap.ps1

function hasProperty([Parameter(Mandatory=$true)][AllowNull()]$object, [Parameter(Mandatory=$true)]$propertyName)
{
    if ($null -eq $object)
    {
        return $false
    }

    return [bool]($object.psobject.Properties | Where-Object { $_.Name -eq "$propertyName"})
}

function getProgramFiles32bit()
{
    $out = ${env:PROGRAMFILES(X86)}
    if ($null -eq $out)
    {
        $out = ${env:PROGRAMFILES}
    }

    if ($null -eq $out)
    {
        throw "Could not find [Program Files 32-bit]"
    }

    return $out
}

function getVisualStudioInstances()
{
    $programFiles = getProgramFiles32bit
    $results = New-Object System.Collections.ArrayList
    $vswhereExe = "$programFiles\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhereExe)
    {
        $output = & $vswhereExe -prerelease -legacy -products * -format xml
        [xml]$asXml = $output

        foreach ($instance in $asXml.instances.instance)
        {
            $installationPath = $instance.InstallationPath -replace "\\$" # Remove potential trailing backslash
            $installationVersion = $instance.InstallationVersion

            $isPrerelease = -7
            if (hasProperty -object $instance -propertyName "isPrerelease")
            {
                $isPrerelease = $instance.isPrerelease
            }

            if ($isPrerelease -eq 0)
            {
                $releaseType = "PreferenceWeight3::StableRelease"
            }
            elseif ($isPrerelease -eq 1)
            {
                $releaseType = "PreferenceWeight2::PreRelease"
            }
            else
            {
                $releaseType = "PreferenceWeight1::Legacy"
            }

            # Placed like that for easy sorting according to preference
            $results.Add("${releaseType}::${installationVersion}::${installationPath}") > $null
        }
    }
    else
    {
        Write-Verbose "Could not locate vswhere at $vswhereExe"
    }

    if ("$env:vs140comntools" -ne "")
    {
        $installationPath = Split-Path -Parent $(Split-Path -Parent "$env:vs140comntools")
        $clExe = "$installationPath\VC\bin\cl.exe"
        $vcvarsallbat = "$installationPath\VC\vcvarsall.bat"

        if ((Test-Path $clExe) -And (Test-Path $vcvarsallbat))
        {
            $results.Add("PreferenceWeight1::Legacy::14.0::$installationPath") > $null
        }
    }

    $installationPath = "$programFiles\Microsoft Visual Studio 14.0"
    $clExe = "$installationPath\VC\bin\cl.exe"
    $vcvarsallbat = "$installationPath\VC\vcvarsall.bat"

    if ((Test-Path $clExe) -And (Test-Path $vcvarsallbat))
    {
        $results.Add("PreferenceWeight1::Legacy::14.0::$installationPath") > $null
    }

    $results.Sort()
    $results.Reverse()

    return $results
}

function findAnyMSBuildWithCppPlatformToolset([string]$withVSPath)
{
    $VisualStudioInstances = getVisualStudioInstances
    if ($null -eq $VisualStudioInstances)
    {
        throw "Could not find Visual Studio. VS2015 or VS2017 (with C++) needs to be installed."
    }

    Write-Verbose "VS Candidates:`n`r$([system.String]::Join([Environment]::NewLine, $VisualStudioInstances))"
    foreach ($instanceCandidate in $VisualStudioInstances)
    {
        Write-Verbose "Inspecting: $instanceCandidate"
        $split = $instanceCandidate -split "::"
        # $preferenceWeight = $split[0]
        # $releaseType = $split[1]
        $version = $split[2]
        $path = $split[3]

        if ($withVSPath -ne "" -and $withVSPath -ne $path)
        {
            Write-Verbose "Skipping: $instanceCandidate"
            continue
        }

        $majorVersion = $version.Substring(0,2);
        if ($majorVersion -eq "16")
        {
            $VCFolder= "$path\VC\Tools\MSVC\"
            if (Test-Path $VCFolder)
            {
                Write-Verbose "Picking: $instanceCandidate"
                return "$path\MSBuild\Current\Bin\MSBuild.exe", "v142", "$VCFolder"
            }
        }

        if ($majorVersion -eq "15")
        {
            $VCFolder= "$path\VC\Tools\MSVC\"
            if (Test-Path $VCFolder)
            {
                Write-Verbose "Picking: $instanceCandidate"
                return "$path\MSBuild\15.0\Bin\MSBuild.exe", "v141", "$VCFolder"
            }
        }

        if ($majorVersion -eq "14")
        {
            $clExe= "$path\VC\bin\cl.exe"
            if (Test-Path $clExe)
            {
                Write-Verbose "Picking: $instanceCandidate"
                $programFilesPath = getProgramFiles32bit
                return "$programFilesPath\MSBuild\14.0\Bin\MSBuild.exe", "v140", "$path\VC"
            }
        }
    }

    throw "Could not find MSBuild version with C++ support. VS2015, VS2017, or VS2019 (with C++) needs to be installed."
}
function getWindowsSDK( [Parameter(Mandatory=$False)][switch]$DisableWin10SDK = $False,
                        [Parameter(Mandatory=$False)][switch]$DisableWin81SDK = $False,
                        [Parameter(Mandatory=$False)][string]$withWinSDK)
{
    if ($DisableWin10SDK -and $DisableWin81SDK)
    {
        throw "Both Win10SDK and Win81SDK were disabled."
    }

    Write-Verbose "Finding WinSDK"

    $validInstances = New-Object System.Collections.ArrayList

    # Windows 10 SDK
    function CheckWindows10SDK($path)
    {
        if ($null -eq $path)
        {
            return
        }

        $folder = (Join-Path $path "Include")
        if (!(Test-Path $folder))
        {
            Write-Verbose "$folder - Not Found"
            return
        }

        Write-Verbose "$folder - Found"
        $win10sdkVersions = @(Get-ChildItem $folder | Where-Object {$_.Name -match "^10"} | Sort-Object)
        [array]::Reverse($win10sdkVersions) # Newest SDK first

        foreach ($win10sdk in $win10sdkVersions)
        {
            $win10sdkV = $win10sdk.Name
            $windowsheader = "$folder\$win10sdkV\um\windows.h"
            if (!(Test-Path $windowsheader))
            {
                Write-Verbose "$windowsheader - Not Found"
                continue
            }
            Write-Verbose "$windowsheader - Found"

            $ddkheader = "$folder\$win10sdkV\shared\sdkddkver.h"
            if (!(Test-Path $ddkheader))
            {
                Write-Verbose "$ddkheader - Not Found"
                continue
            }

            Write-Verbose "$ddkheader - Found"
            $win10sdkVersionString = $win10sdkV.ToString()
            Write-Verbose "Found $win10sdkVersionString"
            $validInstances.Add($win10sdkVersionString) > $null
        }
    }

    Write-Verbose "`n"
    Write-Verbose "Looking for Windows 10 SDK"
    $regkey10 = Get-ItemProperty -Path 'HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots\' -Name 'KitsRoot10' -ErrorAction SilentlyContinue
    $regkey10Wow6432 = Get-ItemProperty -Path 'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots\' -Name 'KitsRoot10' -ErrorAction SilentlyContinue
    if (hasProperty -object $regkey10 "KitsRoot10") { CheckWindows10SDK($regkey10.KitsRoot10) }
    if (hasProperty -object $regkey10Wow6432 "KitsRoot10") { CheckWindows10SDK($regkey10Wow6432.KitsRoot10) }
    CheckWindows10SDK("$env:ProgramFiles\Windows Kits\10")
    CheckWindows10SDK("${env:ProgramFiles(x86)}\Windows Kits\10")

    # Windows 8.1 SDK
    function CheckWindows81SDK($path)
    {
        if ($null -eq $path)
        {
            return
        }

        $folder = "$path\Include"
        if (!(Test-Path $folder))
        {
            Write-Verbose "$folder - Not Found"
            return
        }

        Write-Verbose "$folder - Found"
        $win81sdkVersionString = "8.1"
        Write-Verbose "Found $win81sdkVersionString"
        $validInstances.Add($win81sdkVersionString) > $null
    }

    Write-Verbose "`n"
    Write-Verbose "Looking for Windows 8.1 SDK"
    $regkey81 = Get-ItemProperty -Path 'HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots\' -Name 'KitsRoot81' -ErrorAction SilentlyContinue
    $regkey81Wow6432 = Get-ItemProperty -Path 'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots\' -Name 'KitsRoot81' -ErrorAction SilentlyContinue
    if (hasProperty -object $regkey81 "KitsRoot81") { CheckWindows81SDK($regkey81.KitsRoot81) }
    if (hasProperty -object $regkey81Wow6432 "KitsRoot81") { CheckWindows81SDK($regkey81Wow6432.KitsRoot81) }
    CheckWindows81SDK("$env:ProgramFiles\Windows Kits\8.1")
    CheckWindows81SDK("${env:ProgramFiles(x86)}\Windows Kits\8.1")

    Write-Verbose "`n`n`n"
    Write-Verbose "The following Windows SDKs were found:"
    foreach ($instance in $validInstances)
    {
        Write-Verbose $instance
    }

    # Selecting
    if ($withWinSDK -ne "")
    {
        foreach ($instance in $validInstances)
        {
            if ($instance -eq $withWinSDK)
            {
                return $instance
            }
        }

        throw "Could not find the requested Windows SDK version: $withWinSDK"
    }

    foreach ($instance in $validInstances)
    {
        if (!$DisableWin10SDK -and $instance -match "10.")
        {
            return $instance
        }

        if (!$DisableWin81SDK -and $instance -match "8.1")
        {
            return $instance
        }
    }

    throw "Could not detect a Windows SDK / TargetPlatformVersion"
}

function Find-Tool {
    param([string]$toolname)

    $kitroot = (gp 'HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots\').KitsRoot10
    $tool = (gci -r "$kitroot\Bin\*\x64\$toolname" | sort FullName -Desc | select -First 1)
    if (-not $tool) {
        throw "$toolname is not available"
    }
    Write-Host "Found $toolname at $($tool.FullName)"
    return $tool.FullName
}
