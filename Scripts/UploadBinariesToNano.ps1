function UploadBinariesToNano
{
    param(
        [Parameter(Mandatory=$true)]
        [String]
        $Binary,
        # Parameter help description
        [Parameter(Mandatory=$false)]
        [String]
        $DestinationPath = "/usr/local/bin/",
        # Pod list
        [Parameter(Mandatory=$false)]
        [String[]]
        $PODs=@("POD-1", "POD-2", "POD-3", "POD-4", "POD-5", "POD-6", "POD-7", "POD-8", "POD-9", "POD-10"),
        # Transfer to all pods in parallel or serially
        [Parameter(Mandatory=$false)]
        [Switch]
        $serialMode=$false,
        # If set, adds the recursive flag for copying full directories
        [Parameter(Mandatory=$false)]
        [Switch]
        $Directory = $false    
        )

    $filename = [io.path]::GetFileName($Binary)
    $sshkey = ".\private.ppk"
    if(!(test-path $sshkey))
    {
        $sshkey = "$env:USERPROFILE\.ssh\id_rsa"
    }
    if(!(test-path $sshkey))
    {
        $sshkey = ""
    }
    else {
        $sshkey = "-i $sshkey"
    }
    if(!(test-path $Binary))
    {
        write-host "$Binary was not found"
        return
    }
    if(!(test-path $Binary))
    {
        write-host "$Binary was not found"
        return
    }

    if($serialMode)
    {
        foreach($POD in $PODs)
        {
            if($filename -eq "AKLauncherDaemon")
            {
                $DestinationPath = "/home/peabody/staging/"
                write-host -ForegroundColor Red "Copying to the staging folder.  You must restart the aklauncherdaemon systemctl service to install the new launcher."
            }
            if($Directory)
            {
                $cmd = "scp.exe -P 22 ${sshkey} -r '${Binary}' peabody@${POD}:'${DestinationPath}'"
            }
            else
            {        
                $cmd = "scp.exe -P 22 ${sshkey} ${Binary} peabody@${POD}:${DestinationPath}${filename}"
            }
            $cmd
            invoke-expression $cmd
        }
    }
    else
    {
        write-host -ForegroundColor Green "PARALLEL MODE"
        $jobs = @()
        if($filename -eq "AKLauncherDaemon")
        {
            $DestinationPath = "/home/peabody/staging/"
            write-host -ForegroundColor Red "Copying to the staging folder.  You must restart the aklauncherdaemon systemctl service to install the new launcher."
        }
        foreach($POD in $PODs)
        {
            if($Directory)
            {
                $sb = { param($sshkey,$Binary,$POD,$DestinationPath,$filename) $cmd = "scp.exe -P 22 ${sshkey} -r '${Binary}' peabody@${POD}:${DestinationPath}"; $cmd; invoke-expression $cmd}
            }
            else {
                $sb = { param($sshkey,$Binary,$POD,$DestinationPath,$filename) $cmd = "scp.exe -P 22 ${sshkey} ${Binary} peabody@${POD}:${DestinationPath}${filename}"; $cmd; invoke-expression $cmd}
            }
            $job = start-job -scriptblock $sb -argumentList $sshkey,$Binary,$POD,$DestinationPath,$filename
            $jobs += $job
        }
        $jobcount = $jobs.Count
        $running = $true
        while($running)
        {
            $done = 0
            $running = $false
            foreach($job in $jobs)
            {
                receive-job -id $job.id
                if($job.state -ne "Completed")
                {
                    $running = $true
                }
                else
                {
                    $done += 1
                }
            }
            if($running)
            {
                write-host -nonewline -ForegroundColor Green "$done/$jobcount"
                start-sleep -seconds 1
            }
        }
    }
}

function UploadAllNanoBinariesFromOneDrive
{
    $oneDrivePath = "$env:USERPROFILE\OneDrive\FullSystemDeploy\develop"
    $Binary = "$oneDrivePath\K4ARecorder.bin"
    if(test-path $Binary)
    {
        Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
        UploadBinariesToNano -Binary $Binary
    }

    $Binary = "$oneDrivePath\AzureKinectNanoToFusion.bin"
    if(test-path $Binary)
    {
        Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
        UploadBinariesToNano -Binary $Binary
    }

    $Binary = "$oneDrivePath\AKLauncherDaemon.bin"
    if(test-path $Binary)
    {
        Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
        UploadBinariesToNano -Binary $Binary
    }
}


function UploadAllBinariesInDownloadsToNanos
{
    $Binary = "$env:USERPROFILE\Downloads\K4ARecorder"
    Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
    UploadBinariesToNano -Binary $Binary

    $Binary = "$env:USERPROFILE\Downloads\AzureKinectNanoToFusion"
    Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
    UploadBinariesToNano -Binary $Binary

    $Binary = "$env:USERPROFILE\Downloads\AKLauncherDaemon"
    Write-Host -ForegroundColor Yellow "Sending $Binary to the pods via scp"
    UploadBinariesToNano -Binary $Binary
}
