function CreateTaskForUser 
{
    Param(
        [Parameter(Mandatory=$true)]
        [String]
        $exampleXML,
        [Parameter(Mandatory=$true)]
        [String]
        $userID,
        [Parameter(Mandatory=$false)]
        [String]
        $computerName = $env:Computername,
        [Parameter(Mandatory=$true)]
        [String]
        $taskName
    )
    #Write-Host "Getting $userID's SID.  $userID should be in format username@domain.com"
    #
    $SID = (New-Object System.Security.Principal.NTAccount($userID)).Translate([System.Security.Principal.SecurityIdentifier]).value
    if($computerName -ne $env:ComputerName)
    {
        $cim = new-cimsession $computerName
    }
    else {
        $cim = new-cimsession
    }
    
    $taskName = "$taskName - $userID"

    ## Delete any existing tasks for the user
    $tasks = Get-scheduledTask -TaskPath "\3DTelemedicine\" -cimsession $cim -ErrorAction SilentlyContinue
    foreach($task in $tasks)
    {
        if($task.TaskName -eq $taskName)
        {
            Write-Host "Unregistering $($task.TaskName)"
            Unregister-ScheduledTask -cimsession $cim -TaskName $task.TaskName -confirm:$false
        }
    }

    #Load the example XML file
    $xml = [System.Xml.XmlDocument](Get-Content $exampleXML)
    #Change the principal to the desired user
    $xml.Task.Principals.Principal.UserId = $SID
    #save to temporarly XML
    $xml.Save("C:\\NewTask.xml")
    if($computerName -ne $env:computername)
    {
        #copy the XML to the desired machine
        copy-item C:\\NewTask.xml "\\$computerName\C$\"
    }
    else
    {
        write-host "Skipping the copy since we are installing locally"
    }
    #run schtasks on the remote machine
    $command = "schtasks.exe /create /xml C:\\NewTask.xml "
    if($computerName -ne $env:ComputerName)
    {
        $command += "/S $computerName "
    }
    $command += "/TN '3DTelemedicine\$taskName' "

    $sb = [ScriptBlock]::Create($command)

    #Create the task
    write-host "Executing: $command"
    if($computerName -ne $env:ComputerName)
    {
        invoke-command -computername $computerName -ScriptBlock $sb
    }
    else {
        invoke-command -ScriptBlock $sb        
    }

    #delete the temp XML file
    remove-item "C:\\NewTask.xml"
}


$launchDepthGenExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Launch Depth Gen.xml"
$killDepthGenExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Kill Depth Gen.xml"
$launchFusionExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Launch Fusion.xml"
$launchCameraRecorderExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Launch Camera Recorder.xml"
$killCameraRecorderExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Kill Camera Recorder.xml"
$killFusionExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Kill Fusion.xml"
$launchRendererXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Launch Renderer.xml"
$killRendererExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Kill Renderer.xml"
$restart3DTMServiceExampleXML = "..\\..\\ConfigFileExamples\\ControlPanel\\Scheduled Tasks\\Restart3DTMLauncherService.xml"
#CreateTaskForUser $launchDepthGenExampleXML $SID $computerName "Launch DepthGen"
#CreateTaskForUser $killDepthGenExampleXML $SID $computerName "Kill DepthGen"
#CreateTaskForUser $launchFusionExampleXML $SID $computerName "Launch Fusion"
#CreateTaskForUser $killFusionExampleXML $SID $computerName "Kill Fusion"
#CreateTaskForUser $launchCameraRecorderExampleXML $SID $computerName "Launch Camera Recorder"
#CreateTaskForUser $killCameraRecorderExampleXML $SID $computerName "Kill Camera Recorder"
#CreateTaskForUser $launchRendererXML $SID $computerName "Launch Renderer"
#CreateTaskForUser $killRendererExampleXML $SID $computerName "Kill Renderer"