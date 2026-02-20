#### HELPER FUNCTIONS
#### These functions should NOT be called directly.  They are called by the Setup[COMPUTER] functions at the bottom
#### Usage of this file:  Import the file "./NewComputer_3DTM_Environment_Setup.ps1" and then call the Setup[COMPUTER] function
#### you need, e.g. SetupFusion to set up a Fusion computer.
$global:WSUSURL = "http://51.132.63.142/Content/TRANSFER/"
function Create3DTMStandardDirectories
{
    # Create directories
    New-Item -Path "C:\3DTelemedicine" -ItemType Directory -ErrorAction SilentlyContinue
    New-Item -Path "C:\3DTelemedicine\DATA" -ItemType Directory -ErrorAction SilentlyContinue
    New-Item -Path "C:\3DTelemedicine\Tools" -ItemType Directory -ErrorAction SilentlyContinue
    New-Item -Path "C:\3DTelemedicine\Tools\Scripts" -ItemType Directory -ErrorAction SilentlyContinue
}
function Set-AutoLogin
{
    New-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion\Winlogon" -Name "AutoAdminLogon" -PropertyType String -Value "1" -Force
    New-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion\Winlogon" -Name "DefaultUserName" -PropertyType String -Value "3DTM" -Force
    New-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion\Winlogon" -Name "DefaultPassword" -PropertyType String -Value "" -Force
}
function InstallOpenCV
{
    param
    (
        [Parameter(Mandatory=$false)]
        [Switch]
        $CudaRequired = $false
    )
    # Install OpenCV 454
    $dest = "C:\3DTelemedicine\opencv-4.5.4-vc14_vc15.exe"
    # Check if already downloaded
    if (Test-Path -Path $dest -PathType Leaf)
    {  
        write-host -ForegroundColor Green "OpenCV Already downloaded, installing."
    }
    else
    {
        write-host -ForegroundColor Green "Downloading OpenCV 454"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://github.com/opencv/opencv/releases/download/4.5.4/opencv-4.5.4-vc14_vc15.exe" -OutFile $dest
    }
    C:\3DTelemedicine\opencv-4.5.4-vc14_vc15.exe -o"C:\OpenCV454\" -y
    setx -m OPENCV_DIR C:\OpenCV454\opencv\build\x64\vc15\bin 
    $env:Path = $env:Path+";C:\OpenCV454\opencv\build\x64\vc15\bin"
    [Environment]::SetEnvironmentVariable("PATH", "$env:Path", "Machine")
    Read-Host "Press enter to continue once OpenCV install is complete"

    if($CudaRequired)
    {
        # Download the CUDA-enabled openCV_world454.dll from the WSUS
        write-host -ForegroundColor Green "Downloading the CUDA-enabled OpenCV DLL"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "${global:WSUSURL}Tools/opencv_world454.dll" -OutFile "$env:OPENCV_DIR/opencv_world454.dll"
        write-host -ForegroundColor Green "Download Complete"
    }
    write-host -ForegroundColor Yellow "You may need to close any open file explorer windows before trying to run software to update the path."
}
function InstallCUDA
{
    $dest = "C:\3DTelemedicine\cuda_11.5.0_496.13_win10.exe"
    if(Test-Path -Path $dest -PathType Leaf)
    {
        write-host -ForegroundColor Green "CUDA already downloaded.  Installing"
    }
    else
    {
        write-host -ForegroundColor Green "Downloading CUDA 11.5"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://developer.download.nvidia.com/compute/cuda/11.5.0/local_installers/cuda_11.5.0_496.13_win10.exe" -OutFile $dest
        write-host -ForegroundColor Green "Download complete."
    }
    C:\3DTelemedicine\cuda_11.5.0_496.13_win10.exe /y 
    Read-Host "Press enter to continue once CUDA install is complete"
    write-host -ForegroundColor Green "Downloading the cudnn DLL"
    $env:Path = $env:Path+";C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.5\bin"
    $env:CUDA_PATH_V11_5 = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.5"
    [Environment]::SetEnvironmentVariable("PATH", "$env:Path", "Machine")
    [Environment]::SetEnvironmentVariable("CUDA_PATH_V11_5", "$env:CUDA_PATH_V11_5", "Machine")
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}Tools/cudnn64_8.dll" -OutFile "$env:CUDA_PATH_V11_5/bin/cudnn64_8.dll"
    write-host -ForegroundColor "Download Complete"
}
function InstallVCRedist
{
    $dest = "C:\3DTelemedicine\vc_redist.x64.exe"
    if(Test-Path -Path $dest -PathType Leaf)
    {
        write-host -ForegroundColor Green "Already downloaded.  Installing VC Redistributable"        
    }
    else
    {
        write-host -ForegroundColor Green "Downloading VC Redistributable"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://aka.ms/vs/16/release/vc_redist.x64.exe" -OutFile $dest
        write-host -ForegroundColor Green "Download complete"
    }
    C:\3DTelemedicine\vc_redist.x64.exe /install /quiet
}
function InstallDotNetRedist
{
    $dest = "C:\3DTelemedicine\windowsdesktop-runtime-5.0.12-win-x64.exe"
    if(Test-Path -Path $dest -PathType Leaf)
    {
        write-host -ForegroundColor Green "Already downloaded.  Installing VC .NET Desktop Redistributable"
    }
    else
    {
        write-host -ForegroundColor Green "Downloading VC .NET Desktop Redistributable"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://download.visualstudio.microsoft.com/download/pr/1daf85dc-291b-4bb8-812e-a0df5cdb6701/85455a4a851347de26e2901e043b81e1/windowsdesktop-runtime-5.0.12-win-x64.exe
" -OutFile $dest
        write-host -ForegroundColor Green "Download Complete."
    }
    C:\3DTelemedicine\windowsdesktop-runtime-5.0.12-win-x64.exe /install /quiet
}
function InstallVSCode
{
    $dest = "C:\3DTelemedicine\vscodeinstaller.exe"
    if(Test-Path -Path $dest -PathType Leaf)
    {
        write-host -ForegroundColor Green "Already downloaded.  Installing VS Code."
    }
    else
    {
        # Install VS Code
        write-host -ForegroundColor Green "Downloading VS Code Editor"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://code.visualstudio.com/sha/download?build=stable&os=win32-x64" -OutFile $dest
        write-host -ForegroundColor Green "Download complete"
    }
    C:\3DTelemedicine\vscodeinstaller.exe /silent /MERGETASKS=!runcode
}
function InstallNvidiaBroadcast
{
    $broadcastFileName = "nvidia_broadcast_v1.3.5.4.exe"
    $dest = "C:\3DTelemedicine\$broadcastFileName"
    if(Test-Path -Path $dest -PathType Leaf)
    {
        write-host -ForegroundColor Green "Already downloaded.  Installing NVidia Broadcast"
    }
    else
    {
        write-host -ForegroundColor Green "Downloading NVidia broadcast installer"
        $ProgressPreference = 'SilentlyContinue'
        Invoke-WebRequest -Uri "https://international.download.nvidia.com/Windows/broadcast/1.3.5/$broadcastFileName" -OutFile $dest
        write-host -ForegroundColor Green "Download complete"
    }
    C:\3DTelemedicine\nvidia_broadcast_v1.3.5.4.exe /s
    #Download the green screen
    New-Item -type Directory -Path C:\3DTelemedicine\Viewer -ErrorAction SilentlyContinue
    Invoke-WebRequest -Uri "${global:WSUSURL}Tools/chromakeybg.png" -OutFile "C:\3DTelemedicine\Viewer\chromakeybg.png"

    Write-Host -ForegroundColor Yellow "Follow the setup prompts on the Nvidia Broadcast screen when it appears."
    Write-Host -ForegroundColor Yellow "You can ignore the error about not finding a microphone."
    Write-Host -ForegroundColor Yellow "Click the Camera tab, select your webcam, and under Effects choose 'Background Replacement'"
    Write-Host -ForegroundColor Yellow "Select the C:\3DTelemedicine\Viewer\chromakeybg.png and then hit the X in the top right to close the window"
    Read-Host "Hit enter here when complete to continue..."

    # Create the required registry key
    write-host -ForegroundColor Green "Creating necessary registry key"
    New-ItemProperty -Path "HKLM:\SOFTWARE\Classes\CLSID\{860BB310-5D01-11d0-BD3B-00A0C911CE86}\Instance\{7BBFF097-B3FB-4B26-B685-7A998DE7CEAC}\" -Name "DevicePath" -PropertyType String -Value "NEW" -Force
    Remove-Item -Path "C:\3DTelemedicine\$broadcastFileName" -Force -ErrorAction SilentlyContinue

}
function Set-ServiceRecovery
{
    param
    (
        [string] [Parameter(Mandatory=$true)] $ServiceDisplayName,
        [string] [Parameter(Mandatory=$true)] $Server,
        [string] $action1 = "restart",
        [int] $time1 =  30000, # in miliseconds
        [string] $action2 = "restart",
        [int] $time2 =  30000, # in miliseconds
        [string] $actionLast = "restart",
        [int] $timeLast = 30000, # in miliseconds
        [int] $resetCounter = 4000 # in seconds
    )
    $serverPath = "\\" + $Server
    $service = Get-Service -DisplayName $ServiceDisplayName
    $action = $action1+"/"+$time1+"/"+$action2+"/"+$time2+"/"+$actionLast+"/"+$timeLast
    sc.exe $serverPath failure $($service.Name) actions= $action reset= $resetCounter
}
function Install3DTMLauncherService
{
    New-Item -Path "C:\3DTelemedicine\3DTMLauncherService" -ItemType Directory -ErrorAction SilentlyContinue

    write-host -ForegroundColor Green "Downloading 3DTM Launcher Service (latest) from WSUS"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}3DTMLauncherService.zip" -OutFile "C:\3DTelemedicine\3DTMLauncherService.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\3DTMLauncherService.zip" -DestinationPath "C:\3DTelemedicine\3DTMLauncherService" -Force

    write-host -ForegroundColor Green "Moving 3DTM Launcher Service Updater (latest) downloaded from WSUS"
    Move-Item -Path "C:\3DTelemedicine\3DTMLauncherService\3DTM Launcher Service Updater.exe" -Destination "C:\3DTelemedicine\Tools\3DTM Launcher Service Updater.exe"
    Invoke-WebRequest -Uri "${global:WSUSURL}Tools/SharpConfig.dll" -OutFile "C:\3DTelemedicine\Tools\SharpConfig.dll"
    write-host -ForegroundColor Green "Download complete"

    write-host -ForegroundColor Green "Creating 3DTM Launcher Windows Service"
    sc.exe create "3DTMLauncherService" binPath= "C:\3DTelemedicine\3DTMLauncherService\3DTMLauncherService.exe" start= delayed-auto DisplayName= "3D Telemedicine Launcher Service" 

    Set-ServiceRecovery -ServiceDisplayName "3D Telemedicine Launcher Service" -Server $env:Computername
}
function Set3DTMEnv
{
    $env:3DTelemedicine_dir = "C:\3DTelemedicine"
    [Environment]::SetEnvironmentVariable("3DTelemedicine_Dir", "C:\3DTelemedicine", "Machine")
}
function CreateFileRotateTasks
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/FileRotate.ps1" -OutFile "C:\3DTelemedicine\Tools\Scripts\FileRotate.ps1"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/CreateScheduledTasksForUser.ps1" -OutFile "C:\3DTelemedicine\Tools\Scripts\CreateScheduledTasksForUser.ps1"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/File Rotate.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\FileRotate.xml"

    Import-Module -Force "C:\3DTelemedicine\Tools\Scripts\CreateScheduledTasksForUser.ps1" 
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\FileRotate.xml" -userID $userID -taskName "RotateFiles"

    Write-Host -ForegroundColor Yellow "Please remember to edit the FileRotate.ps1 file and uncomment the lines at the bottom for which file rotates you want to perform (and make sure the directories are correct)!"
    Start-Process notepad.exe "C:\3DTelemedicine\Tools\Scripts\FileRotate.ps1"
}
function CreateFusionScheduledTasks
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )

    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Launch Fusion.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Launch Fusion.xml"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Kill Fusion.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Kill Fusion.xml"
    
    Import-Module -Force "C:\3DTelemedicine\Tools\Scripts\CreateScheduledTasksForUser.ps1" 
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Launch Fusion.xml" -userID $userID -taskName "Launch Fusion"
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Kill Fusion.xml" -userID $userID -taskName "Kill Fusion"
}
function CreateCalibrationSoftwareScheduledTasks
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Launch Calibration Software.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Launch Calibration Software.xml"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Kill Calibration Software.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Kill Calibration Software.xml"

    Import-Module -Force "C:\3DTelemedicine\Tools\Scripts\CreateScheduledTasksForUser.ps1" 
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Launch Calibration Software.xml" -userID $userID -taskName "Launch Calibration Software"
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Kill Calibration Software.xml" -userID $userID -taskName "Kill Calibration Software"
}
function CreateRenderScheduledTasks
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )

    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Launch Renderer.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Launch Renderer.xml"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/Scheduled Tasks/Kill Renderer.xml" -OutFile "C:\3DTelemedicine\Tools\Scripts\Kill Renderer.xml"

    Import-Module -Force "C:\3DTelemedicine\Tools\Scripts\CreateScheduledTasksForUser.ps1" 
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Launch Renderer.xml" -userID $userID -taskName "Launch Renderer"
    CreateTaskForUser -exampleXml "C:\3DTelemedicine\Tools\Scripts\Kill Renderer.xml" -userID $userID -taskName "Kill Renderer"
}
function UploadBinariesToPods
{
    write-host -ForegroundColor Green "Downloading pod-side binaries (latest) from WSUS"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}AKLauncherDaemon.bin" -OutFile "$env:USERPROFILE\Downloads\AKLauncherDaemon"
    Invoke-WebRequest -Uri "${global:WSUSURL}AzureKinectNanoToFusion.bin" -OutFile "$env:USERPROFILE\Downloads\AzureKinectNanoToFusion"
    Invoke-WebRequest -Uri "${global:WSUSURL}K4ARecorder.bin" -OutFile "$env:USERPROFILE\Downloads\K4ARecorder"
    
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/UploadAllBinariesInDownloadsToNanos.ps1" -OutFile "C:\3DTelemedicine\Tools\Scripts\UploadAllBinariesInDownloadsToNanos.ps1"
    Invoke-WebRequest -Uri "${global:WSUSURL}Scripts/UploadBinariesToNano.ps1" -OutFile "C:\3DTelemedicine\Tools\Scripts\UploadBinariesToNano.ps1"
    write-host -ForegroundColor Green "Downloads complete"
    Import-Module -Force "C:\3DTelemedicine\Tools\Scripts\UploadAllBinariesInDownloadsToNanos.ps1"
    
    UploadAllBinariesInDownloadsToNanos
}
function CleanUp
{
    Remove-Item -Path "C:\3DTelemedicine\ControlPanel.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\Tools\ConfigFileUpdater.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\vc_redist.x64.exe" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\windowsdesktop-runtime-5.0.12-win-x64.exe" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\opencv-4.5.4-vc14_vc15.exe" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\vscodeinstaller.exe" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\cuda_11.5.0_496.13_win10.exe" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\3DTMLauncherService.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\KinectNanoCommunicatorService.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\Fusion.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\CalibrationSoftware.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\Viewer.zip" -Force -ErrorAction SilentlyContinue
    Remove-Item -Path "C:\3DTelemedicine\Renderer.zip" -Force -ErrorAction SilentlyContinue
}






#### CALLABLE FUNCTIONS
#### You should call the function below based on which computer type you are setting up
#### Functions below use the helper functions above
function SetupControlPanel
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Write-host -ForegroundColor Cyan "Starting to setup Control Panel"

    # Test for internet connection before proceeding.
    $connok = Test-NetConnection onedrive.live.com
    $Internetaccess = (Get-NetConnectionProfile -IPv4Connectivity Internet).ipv4connectivity
    if(!$connok -or !$Internetaccess)
    {
        write-host -ForegroundColor Red "Sorry, I can't reach onedrive.  You need to be connected to the internet to proceed."
        return
    }

    # Create directories
    Create3DTMStandardDirectories

    # Install Pre-requisites
    InstallVCRedist
    InstallDotNetRedist
    InstallVSCode
    InstallOpenCV
    Set3DTMEnv

    write-host -ForegroundColor Green "Downloading Control Panel (latest) from OneDrive"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}ControlPanel.zip" -OutFile "C:\3DTelemedicine\ControlPanel.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\ControlPanel.zip" -DestinationPath "C:\3DTelemedicine\Control Panel" -Force

    CreateFileRotateTasks -userID $userID
    New-netfirewallrule -DisplayName "3DTM Control Panel" -Enabled "True" -Description "Allows the control panel through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\Control Panel\3D Telemedicine Control Panel.exe"

    Invoke-WebRequest -Uri "${global:WSUSURL}3DTelemedicine.cfg" -OutFile "C:\3DTelemedicine\3dTelemedicine.cfg"
    write-host -ForegroundColor Green "Download complete"
    Start-Process notepad.exe "C:\3DTelemedicine\3dTelemedicine.cfg"

    Write-Host -ForegroundColor Yellow "Please edit the opened 3dtelemedicine.cfg file and update the Network and Ports section to reflect your network configuration before starting the control panel"

    Invoke-WebRequest -Uri "${global:WSUSURL}ConfigFileUpdater.zip" -OutFile "C:\3DTelemedicine\Tools\ConfigFileUpdater.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\Tools\ConfigFileUpdater.zip" -DestinationPath "C:\3DTelemedicine\Tools" -Force
    
    CleanUp
}

function SetupNTPServer
{
    Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Services\w32time\TimeProviders\NtpServer" -Name "Enabled" -Value 1
    Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\services\W32Time\Config" -Name "AnnounceFlags" -Value 5
    Set-Service -Name w32Time -StartupType 'Automatic'
    Restart-Service w32Time
    New-NetFirewallRule `
        -Name "NTP Server Port" `
        -DisplayName "NTP Server Port" `
        -Description 'Allow NTP Server Port' `
        -Profile Any `
        -Direction Inbound `
        -Action Allow `
        -Protocol UDP `
        -Program Any `
        -LocalAddress Any `
        -LocalPort 123 
}

function SetupOpenSSHServer
{
    Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0
    Start-Service sshd
    Set-Service -Name sshd -StartupType 'Automatic'
    if(Test-Path C:\Users\3DTM\.ssh\id_rsa )
    {
        write-host -ForegroundColor Yellow "An ssh key already exists at c:\users\3dtm\.ssh\id_rsa, so I won't generate a new one"
    }
    else
    {
        write-host -ForegroundColor Green "Generating an SSH Key.  Press 'Enter' for No Passkey"
        ssh-keygen -t rsa -f C:\Users\3DTM\.ssh\id_rsa
    }
    write-host -ForegroundColor Yellow "Copy the contents of C:\Users\3DTM\.ssh\id_rsa.pub to each pod's ~/.ssh/authorized_keys to allow the UploadToNanos script to work"
    notepad C:\Users\3DTM\.ssh\id_rsa.pub
}

function SetupFusion
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Write-host -ForegroundColor Cyan "Starting to setup Fusion"

    # Test for internet connection before proceeding.
    $connok = Test-NetConnection onedrive.live.com
    $Internetaccess = (Get-NetConnectionProfile -IPv4Connectivity Internet).ipv4connectivity
    if(!$connok -or !$Internetaccess)
    {
        write-host -ForegroundColor Red "Sorry, I can't reach onedrive.  You need to be connected to the internet to proceed."
        return
    }

    Create3DTMStandardDirectories
    New-Item -Path "C:\3DTelemedicine\DATA\Calibration" -ItemType Directory -ErrorAction SilentlyContinue 
    New-Item -Path "C:\3DTelemedicine\DATA\FusionCalibration" -ItemType Directory -ErrorAction SilentlyContinue

    Set-AutoLogin
    InstallVCRedist
    InstallVSCode
    InstallCUDA
    InstallOpenCV -CudaRequired
    Install3DTMLauncherService
    InstallNvidiaBroadcast
    Set3DTMEnv

    #On fusion, we also need to create the KinectNanoCommunicatorService folder and 3DTMLauncherService folder
    New-Item -Path "C:\3DTelemedicine\KinectNanoCommunicatorService" -ItemType Directory -ErrorAction SilentlyContinue
    write-host -ForegroundColor Green "Downloading Kinect Nano Communicator Service (latest) from WSUS"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}KinectNanoCommunicatorService.zip" -OutFile "C:\3DTelemedicine\KinectNanoCommunicatorService.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\KinectNanoCommunicatorService.zip" -DestinationPath "C:\3DTelemedicine\KinectNanoCommunicatorService" -Force

    write-host -ForegroundColor Green "Creating Kinect Nano Communicator Windows Service"
    sc.exe create "3DTMKinectNanoCommunicatorService" binPath= "C:\3DTelemedicine\KinectNanoCommunicatorService\KinectNanoCommunicatorService.exe" start= delayed-auto DisplayName= "3D Telemedicine Kinect Communicator Service" 
    Set-ServiceRecovery -ServiceDisplayName "3D Telemedicine Kinect Communicator Service" -Server $env:Computername

    write-host -ForegroundColor Green "Downloading Fusion (latest) from WSUS"
    Invoke-WebRequest -Uri "${global:WSUSURL}Fusion.zip" -OutFile "C:\3DTelemedicine\Fusion.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\Fusion.zip" -DestinationPath "C:\3DTelemedicine\Fusion" -Force

    CreateFileRotateTasks -userID $userID
    CreateFusionScheduledTasks -userID $userID
    New-netfirewallrule -DisplayName "3DTM Fusion" -Enabled "True" -Description "Allows Fusion through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\Fusion\LiveFusionDemo-MultiView.exe"
    New-netfirewallrule -DisplayName "3DTM Launcher Service" -Enabled "True" -Description "Allows Launcher Service through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\3DTMLauncherService\3DTMLauncherService.exe"
    New-netfirewallrule -DisplayName "3DTM Kinect Nano Communicator Service" -Enabled "True" -Description "Allows Kinect Nano Communicator Service through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\KinectNanoCommunicatorService\KinectNanoCommunicatorService.exe"
    
    write-host -ForegroundColor Green "Setting up NTP Server for pod time synchronization"
    SetupNTPServer
    write-host -ForegroundColor Green "Setting up OpenSSH server for pod calibration file uploads"
    SetupOpenSSHServer

    Invoke-WebRequest -Uri "${global:WSUSURL}3DTelemedicine.cfg" -OutFile "C:\3DTelemedicine\3dTelemedicine.cfg"
    write-host -ForegroundColor Green "Download complete"
    Start-Process notepad.exe "C:\3DTelemedicine\3dTelemedicine.cfg"
    Write-Host -ForegroundColor Yellow "Please edit the opened 3dtelemedicine.cfg file and update the Network and Ports section to reflect your network configuration before starting the control panel"

    $msg = "Do you want to upload binaries to pods?  [Y/N]"
    do {
        $user_input = Read-Host -Prompt $msg
        if (($user_input -eq 'y')) {
            UploadBinariesToPods        
            break
        }
    } until ($user_input -eq 'n')
    CleanUp

    $msg = "Do you want to setup Calibratiom Software in this machine? [Y/N]"
    do {
        $user_input = Read-Host -Prompt $msg
        if (($user_input -eq 'y')) {
            SetupCalibrationSoftware
            break
        }
    } until ($user_input -eq 'n')

    $msg = "Do you want to setup Viewer in this machine? [Y/N]"
    do {
        $user_input = Read-Host -Prompt $msg
        if (($user_input -eq 'y')) {
            SetupViewer
            break
        }
    } until ($user_input -eq 'n')
}

function SetupCalibrationSoftware
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Write-host -ForegroundColor Cyan "Starting to setup Calibration Software"

    # Test for internet connection before proceeding.
    $connok = Test-NetConnection onedrive.live.com
    $Internetaccess = (Get-NetConnectionProfile -IPv4Connectivity Internet).ipv4connectivity
    if(!$connok -or !$Internetaccess)
    {
        write-host -ForegroundColor Red "Sorry, I can't reach onedrive.  You need to be connected to the internet to proceed."
        return
    }
    # Creating standard 3DTM directories if the calibration software runs in a different machine than Fusion
    Create3DTMStandardDirectories

    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}CalibrationSoftware.zip" -OutFile "C:\3DTelemedicine\CalibrationSoftware.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\CalibrationSoftware.zip" -DestinationPath "C:\3DTelemedicine\CalibrationSoftware" -Force

    # Adding firewall rule to allow the calibration software to communicate with Control Panel 
    New-netfirewallrule -DisplayName "3DTM Calibration Software" -Enabled "True" -Description "Allows CalibrationSoftware through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\CalibrationSoftware\CalibrationSoftware.exe"  

    CreateCalibrationSoftwareScheduledTasks -userID $userID

    CleanUp
}

function SetupRender
{
    param(
        [Parameter(Mandatory=$false)]
        [String]
        $userID = "3DTM"
    )
    Write-host -ForegroundColor Cyan "Starting to setup Render"

    Create3DTMStandardDirectories
    New-Item -Path "C:\3DTelemedicine\DATA\RendererCalibration" -ItemType Directory -ErrorAction SilentlyContinue

    Set-AutoLogin
    InstallVCRedist
    InstallVSCode
    InstallCUDA
    InstallOpenCV -CudaRequired
    Install3DTMLauncherService
    Set3DTMEnv

    write-host -ForegroundColor Green "Downloading Render (latest) from WSUS"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}Renderer.zip" -OutFile "C:\3DTelemedicine\Renderer.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\Renderer.zip" -DestinationPath "C:\3DTelemedicine\Renderer" -Force

    CreateFileRotateTasks -userID $userID
    CreateRenderScheduledTasks -userID $userID
    New-netfirewallrule -DisplayName "3DTM Render" -Enabled "True" -Description "Allows renderer through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\Renderer\HoloPortRenderer.exe"
    New-netfirewallrule -DisplayName "3DTM Launcher Service" -Enabled "True" -Description "Allows Launcher Service through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\3DTMLauncherService\3DTMLauncherService.exe"
    
    Invoke-WebRequest -Uri "${global:WSUSURL}3DTelemedicine.cfg" -OutFile "C:\3DTelemedicine\3dTelemedicine.cfg"
    write-host -ForegroundColor Green "Download complete"
    Start-Process notepad.exe "C:\3DTelemedicine\3dTelemedicine.cfg"

    Write-Host -ForegroundColor Yellow "Please edit the opened 3dtelemedicine.cfg file and update the Network and Ports section to reflect your network configuration before starting the control panel"

    CleanUp
}

function SetupViewer
{
    Write-host -ForegroundColor Cyan "Starting to setup Viewer"

    write-host -ForegroundColor Green "Downloading Viewer (latest) from WSUS"
    $ProgressPreference = 'SilentlyContinue'
    Invoke-WebRequest -Uri "${global:WSUSURL}Viewer.zip" -OutFile "C:\3DTelemedicine\Viewer.zip"
    write-host -ForegroundColor Green "Download complete"
    Expand-Archive -Path "C:\3DTelemedicine\Viewer.zip" -DestinationPath "C:\3DTelemedicine\Viewer" -Force

    New-netfirewallrule -DisplayName "3DTM Viewer" -Enabled "True" -Description "Allows viewer through the firewall" -Profile Domain,Private,Public -Direction Inbound -Action Allow -Program "C:\3DTelemedicine\Viewer\3D Telemedicine Viewer.exe"

    CleanUp
}
