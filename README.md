# ROS on MSIX Packaging

MSIX is a Windows app package format that provides a modern packaging experience to all Windows apps.

This project contains some useful scripts to build MSIX packages for ROS applications.

## Prerequisites

- [Visual Studio 2019][Visual Studio 2019] with [C++ support][C++ support]
- [Windows 10 SDK][Windows 10 SDK]
- [Create a certificate for package signing][Create a certificate for package signing]
- Clone this project to a local location
- ROS Workspace ready for packaging

> You can install `Windows 10 SDK` by selecting `Windows 10 SDK (10.0.19041.0)` in the optional components of the Visual Studio 2019 Installer.

**Before you proceed, it is highly recommended to read [`What's MSIX`][What's MSIX] and learn it benefits.**

## Task 1: Binary Install ROS on Windows

You can binary install ROS on Windows by following [`ROS on Windows installation`][ROS on Windows installation].
You can skip this step if you did it already.

## Task 2: Install ROS Workspace

Now you will need to install your ROS workspace into `c:\opt\install` and make it live side-by-side with the ROS installation.

You can do it by specifying the install space when running `catkin_make_isolated`:

```Batchfile
:: catkin_make_isolated example

catkin_make_isolated --install --merge --install-space c:\opt\install
```

## Task 3: Prepare ROS Runtime Contents

Check the folder layout of `c:\opt` and it should be like as below:

```
-- opt
   |-- python27amd64
   |-- ros\melodic\x64
   |-- rosdeps\x64
   |-- install
```

Now open the PowerShell terminal and change the working directory to this project.
Run `rosprep.ps1` to prepare the ROS runtime contents, and it will stage the results under `working` folder.

> `rosprep.ps1` copies the contents only required at runtime and fixes up hard-coded prefix in ROS files to accommodate MSIX virtualized file system requirement.

## Task 4: Prepare MSIX Packaging Layout and App Manifest Files

Before building MSIX package, you will need to define some answer files for MSIX to build.

In this project, you can check this [example][example] and modify it per your requirement.

Under the example folder, you can find:

  * [PackagingLayout.xml][packaging-layout]: Define the file contents for MSIX packaging.
  * [AppxManifest.xml][appx-package-manifest]: Define the MSIX app's entry points and additional capability\behavior.
  * Launcher.bat: Define the start-up ROS application by roslaunch convention.

ROS is using a communication model which requires to listen TCP\UDP ports.
It is important to identify and define the proper firewall rules for the best experience.
From the previous step, `rosprep.ps1` generates a `firewall.xml` as a baseline for all the executables potentially required to do ROS topics publish and subscribe.
It is recommended to review it and integrate the rules back to your `AppxManifest.xml`.

## Task 5: Build MSIX Bundles and Packages

Run `build.ps1` to package the example project.
The `-certFile test.pfx -certPassword 1234` can specify the signing certificate.
The `-packagingLayout <path to PackagingLayout.xml>` can specify your customized `PackagingLayout.xml`.

## Task 6: Install MSIX Packages

Now you can find the MSIX packages under `output` folder.
Double click `ros-melodic-desktop.msixbundle` to install.

Or you can install MSIX to device context by [Add-AppProvisionedPackage][Add-AppProvisionedPackage].

On your target machine, the signing certificate needs to be trusted before installing the package.
Open `PowerShell` in administrative privilege, run `Import-Certificate -FilePath "<your certificate file (.cer)>" -CertStoreLocation Cert:\LocalMachine\Root`. Then the certificate will be added to the `Trusted Root Certification Authorities` store.

## Task 7: Run ROS Application

You can find the app entry point registered on the `Start Menu`.
Click it to execute.

> You will need to rename the `c:\opt` to something else (for example, `c:\opt_`) on your target device. Otherwise, the application could be affected by the ROS installation. This is due to ROS build tools currently embed hard-coded paths to PE files and this [fix][catkin-fix] is work in progress now.

## What's Next

Now you know how to package ROS into MSIX package.
You can integrate it with CI\CD pipelines.
And you may want to know more about how to distribute in larger scale.

Learn more about MSIX distribution [here][managing-your-msix-deployment-enterprise].

## Troubleshooting

Sometimes you need to be able to trouble shoot an installation.
Here is a collection of useful resources to help.

  * [MSIX Validation and Troubleshooting][managing-your-msix-deployment-troubleshooting]
  * [Hover][Hover]: Launching apps inside a MSIX/App-V container

[Visual Studio 2019]: https://visualstudio.microsoft.com/vs/
[C++ support]: https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation
[Windows 10 SDK]: https://developer.microsoft.com/en-us/windows/downloads/windows-10-sdk/
[ROS on Windows installation]: http://wiki.ros.org/Installation/Windows
[What's MSIX]: https://docs.microsoft.com/en-us/windows/msix/overview
[Create a certificate for package signing]: https://docs.microsoft.com/en-us/windows/msix/package/create-certificate-package-signing
[example]: ./example
[appx-package-manifest]: https://docs.microsoft.com/en-us/uwp/schemas/appxpackage/appx-package-manifest
[packaging-layout]: https://docs.microsoft.com/en-us/windows/msix/package/packaging-layout
[Add-AppProvisionedPackage]: https://www.advancedinstaller.com/per-machine-msix.html
[catkin-fix]: https://github.com/ros/catkin/pull/1086
[managing-your-msix-deployment-enterprise]: https://docs.microsoft.com/en-us/windows/msix/desktop/managing-your-msix-deployment-enterprise
[Hover]: https://www.advancedinstaller.com/hover.html
[managing-your-msix-deployment-troubleshooting]: https://docs.microsoft.com/en-us/windows/msix/desktop/managing-your-msix-deployment-troubleshooting
