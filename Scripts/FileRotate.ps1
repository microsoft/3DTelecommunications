#### Looks through the calibration folder, and archives old versions, deleting the oldest based on specified rules
function rotateFiles
{
    param(        
        [Parameter(Mandatory=$false)]
        [String]
        $filePath = "C:\3DTelemedicine\DATA\AllCalibrations",
        # Test will output what it would do, but not actually perform any operations
        [Parameter(Mandatory=$false)]
        [Switch]
        $test = $false,
        #Number of days worth of calibrations to keep (uncompressed)
        [Parameter(Mandatory=$false)]
        [Int32]
        $daysToKeep = 7,
        #any calibration directories older than today - $daysToKeep will be compressed into an archive by week
        #number of weekly archives to keep
        [Parameter(Mandatory=$false)]
        [Int32]
        $archiveWeeksToKeep = 4,
        #any weekly archives older than today - $archiveWeeksToKeep will be compressed into a monthly archive
        #number of monthly archives to keep
        [Parameter(Mandatory=$false)]
        [Int32]
        $archiveMonthsToKeep = 2,
        #any monthly archives older than today - $archiveMonthsToKeep will be compressed into a yearly archive
        #number of yearly archives to keep
        [Parameter(Mandatory=$false)]
        [Int32]
        $archiveYearsToKeep = 1,
        #any yearly archives older than today - $archiveYearsToKeep will be deleted
        #maximum size in MB that we'll allow the calibration folder to get before deleting
        #deleting will start with year archives, then month archives, etc... until we are below the threshold
        [Parameter(Mandatory=$false)]
        [Int32]
        $maxFolderSize = 100,
        # If true, will rotate (non-.zip) files in the $filePath, otherwise it will only rotate directories
        [Parameter(Mandatory=$false)]
        [Switch]
        $ProcessFiles = $false
    )
    write-host -ForegroundColor Green "Rotating files in $filePath"
    $culture = get-culture

    $now = Get-Date
    $oldestDayToKeep = $now.AddDays(-1*$daysToKeep)
    $oldestWeekToKeep = $now.AddDays(-7*$archiveWeeksToKeep)
    $oldestMonthToKeep = $now.AddDays(-30*$archiveMonthsToKeep)
    $oldestYearToKeep = $now.AddDays(-365*$archiveYearsToKeep)

    $weeklyArchives = @{};
    $monthlyArchives = @{};
    $yearlyArchives = @{};
    # Get archives
    foreach($item in Get-ChildItem -path $filePath -File)
    {
        if($item.Extension -eq ".zip")
        {
            if($item.Name.StartsWith("weekly_"))
            {                
                $weeklyArchives.Add($item.CreationTime, $item.FullName)
            }
            elseif($item.Name.StartsWith("monthly_"))
            {
                $monthlyArchives.Add($item.CreationTime, $item.FullName)
            }
            elseif($item.Name.StartsWith("yearly_"))
            {
                $yearlyArchives.Add($item.CreationTime, $item.FullName)                
            }            
        }
    }

    if($ProcessFiles)
    {
        write-host -foregroundColor Yellow "Processing all files except *.zip"
        $fileList = Get-ChildItem -path $filePath -File -Exclude "*.zip" -Recurse
    }
    else 
    {
        write-host -foregroundColor Yellow "Processing directories"
        $fileList = Get-ChildItem -path $filePath -Directory   
    }
    #check new directories that should be added to archives (or deleted)
    foreach($item in $fileList)
    {
        #Don't archive the default folder or any folders that don't start with a number
        if($item.Name -eq "Default" -or !($item[0] -match "\d"))
        {
            continue;
        }
        #write-host "Checking $($item.Name).  Created $($item.CreationTime)"
        $itemWeekOfYear = $culture.calendar.GetWeekOfYear($item.CreationTime, $culture.datetimeformat.calendarweekrule, $culture.datetimeformat.firstdayofweek)
        if( ($now - $item.CreationTime).TotalDays -gt $daysToKeep )
        {
            write-host "$($item.Name) is older than $oldestDayToKeep.  Adding it to an archive"
            # Do I already have an archive I should add to?
            $archiveExists = $false
            foreach($key in $weeklyArchives.Keys)
            {
                $archiveWeekOfYear = $culture.calendar.GetWeekOfYear($key, $culture.datetimeformat.calendarweekrule, $culture.datetimeformat.firstdayofweek)
                #write-host "Comparing archive WoY $archiveWeekOfYear ($key) to item WoY $itemWeekOfYear $($item.CreationTime)"
                if( $itemWeekOfYear -eq $archiveWeekOfYear)
                {
                    $archiveExists = $true
                    write-host "Adding $($item.Name) to $($weeklyArchives[$key])"
                    if(!$test)
                    {
                        Compress-Archive -DestinationPath $weeklyArchives[$key] -Path $item.FullName -Update
                    }
                    break;
                }
            }
            if($archiveExists -eq $false)
            {
                # Create a new archive
                $archiveName = "weekly_$($item.CreationTime.Year)-$($item.CreationTime.Month)-$($item.CreationTime.Day).zip"
                write-host "Creating a new archive for the week of $($item.CreationTime): $($filePath)\\$archiveName"
                if(!$test)
                {
                    Compress-Archive -DestinationPath "$($filePath)\\$($archiveName)" -Path $item.FullName
                    $newZip = Get-Item -Path "$($filePath)\\$($archiveName)"
                    $newZip.CreationTime = $item.CreationTime
                    # add this to the list of weekly archives
                    $weeklyArchives.add($item.CreationTime, "$($filePath)\$($archiveName)")
                }
            }
            write-host -ForegroundColor Yellow "$($item.Name) has been archived.  Deleting original"
            if(!$test)
            {
                remove-item -path $item.FullName -Recurse
            }
        }
        else
        {
            write-host "Leaving $($item.Name) alone as it is too new to rotate."
        }
    }
    write-host -ForegroundColor Yellow "Rolling weekly archives"
    RolloverItems $weeklyArchives $oldestWeekToKeep $monthlyArchives "monthly" $filePath -test:$test
    write-host -ForegroundColor Yellow  "Rolling monthly archives"
    RolloverItems $monthlyArchives $oldestMonthToKeep $yearlyArchives "yearly" $filePath -test:$test
    write-host -ForegroundColor Yellow  "Rolling yearly archives"
    foreach($archiveCreationTime in $yearlyArchives.Keys)
    {
        if( ($oldestYearToKeep - $archiveCreationTime).TotalDays -gt 0 )
        {
            write-host -ForegroundColor Yellow "$($yearlyArchives[$archiveCreationTime]) is older that our oldest limit.  Deleting"
            if(!$test)
            {
                Remove-Item -Path $yearlyArchives[$archiveCreationTime]
            }
        }
    }

    #Final size check
    $directoryInfo = Get-childItem -path $filePath -recurse -depth 5 | measure-object -property Length -sum
    $directorySize = $directoryInfo.Sum / 1Mb
    write-host "Directory size is $($directorySize)Mb and Max allowed is $($maxFolderSize)Mb"
    while($directorySize -gt $maxFolderSize)
    {
        if($yearlyArchives.Count -gt 0)
        {
            write-host -ForegroundColor Red "Directory is too large ($directorySize Mb).  Removing files starting at oldest yearly archive."        
            $sortedYearly = $yearlyArchives.GetEnumerator() | Sort Name
            $oldest = $sortedYearly[0]
            write-host -ForegroundColor Red "Deleting $($oldest.Value) to clear up space"
            if(!test)
            {
                remove-item -path $oldest.Value
            }
            $yearlyArchives.Remove($oldest.Name);
        }
        else
        {
            if($monthlyArchives.Count -gt 0)
            {
                write-host -ForegroundColor Red "Directory is still too large ($directorySize Mb).  Removing files starting at oldest monthly archive."        
                $sortedMonthly = $monthlyArchives.GetEnumerator() | Sort Name
                $oldest = $sortedMonthly[0]
                write-host -ForegroundColor Red "Deleting $($oldest.Value) to clear up space"
                if(!$test)
                {
                    remove-item -path $oldest.Value
                }
                $monthlyArchives.Remove($oldest.Name);
            }
            else 
            {
                if($weeklyArchives.Count -gt 0)
                {
                    write-host -ForegroundColor Red "Directory is still too large ($directorySize Mb).  Removing files starting at oldest weekly archive."        
                    $sortedWeekly = $weeklyArchives.GetEnumerator() | Sort Name
                    $oldest = $sortedWeekly[0]
                    write-host -ForegroundColor Red "Deleting $($oldest.Value) to clear up space"
                    if(!$test)
                    {
                        remove-item -path $oldest.Value
                    }
                    $weeklyArchives.Remove($oldest.Name);    
                }    
                else 
                {
                    write-host -ForegroundColor Red "ERROR.  Can't find enough archives to delete!  Increase your max size allowance!"   
                    break;
                }
            }
        }
        $directoryInfo = Get-childItem -path $filePath -recurse -depth 5 | measure-object -property Length -sum
        $directorySize = $directoryInfo.Sum / 1Mb    
    }

}
function RolloverItems {
    param(
        [Parameter(Mandatory=$true)]
        [HashTable] 
        $archiveListToRoll,
        [Parameter(Mandatory=$true)]
        [DateTime] 
        $oldestDateToKeep,
        [Parameter(Mandatory=$true)]
        [HashTable]
        $archiveToRollInto,
        [Parameter(Mandatory=$false)]
        [String]
        $granularity = "Monthly",
        [Parameter(Mandatory=$false)]
        [String]
        $filePath = "C:\3DTelemedicine\DATA\AllCalibrations",
        [Parameter(Mandatory=$false)]
        [Switch]
        $test = $false
    )

    foreach($archiveCreationTime in $archiveListToRoll.Keys)
    {
        $archiveName = $archiveListToRoll[$archiveCreationTime]
        #move any archives older than the oldest to keep into the yearly archive
        if( ($oldestDateToKeep - $archiveCreationTime).TotalDays -gt 0 )
        {
            #write-host "$($archiveCreationTime) is older than our oldest item to keep $($oldestDateToKeep) so I'll move it into the next archive"
            $archiveExists = $false
            foreach($nextArchiveCreationTime in $archiveToRollInto.Keys)
            {
                #write-host "Comparing Archive $($archiveCreationTime.Month)-$($archiveCreationTime.Year) to $($nextArchiveCreationTime.Month)-$($nextArchiveCreationTime.Year)"
                if($granularity -eq "Yearly")
                {
                    if($nextArchiveCreationTime.Year -eq $archiveCreationTime.Year)
                    {
                        $archiveExists = $true
                        write-host "Adding $($archiveName) to $($archiveToRollInto[$nextArchiveCreationTime])"
                        if(!$test)
                        {
                            Compress-Archive -DestinationPath $archiveToRollInto[$nextArchiveCreationTime] -Path $archiveName -Update
                        }
                        break;
                    }
                }
                else 
                {
                    if($nextArchiveCreationTime.Month -eq $archiveCreationTime.Month -and $nextArchiveCreationTime.Year -eq $archiveCreationTime.Year)
                    {
                        $archiveExists = $true
                        write-host "Adding $($archiveName) to $($archiveToRollInto[$nextArchiveCreationTime])"
                        if(!$test)
                        {
                            Compress-Archive -DestinationPath $archiveToRollInto[$nextArchiveCreationTime] -Path $archiveName -Update
                        }
                        break;
                    }
                }
            }
            if($archiveExists -eq $false)
            {
                if($granularity -eq "Yearly")
                {
                    $newRolledArchiveName = "$($granularity)_$($archiveCreationTime.Year).zip"
                }
                else
                {                    
                    $newRolledArchiveName = "$($granularity)_$($archiveCreationTime.Year)-$($archiveCreationTime.Month).zip"
                }
                write-host "Creating a $($granularity) archive for $($archiveCreationTime.Month)-$($archiveCreationTime.Year): $($filePath)\$($newRolledArchiveName)"
                if(!$test)
                {
                    Compress-Archive -DestinationPath "$($filePath)\$($newRolledArchiveName)" -Path $archiveName
                    $newZip = Get-Item -Path "$($filePath)\$($newRolledArchiveName)"
                    $newZip.CreationTime = $archiveCreationTime
                    # Add to the list of monthly archives
                    $archiveToRollInto.add($archiveCreationTime, "$($filePath)\$($newRolledArchiveName)");
                }
            }
            write-host -ForegroundColor Yellow "Archive $($archiveName) was rolled.  Deleting the original archive."
            if(!$test)
            {
                remove-item -path $($archiveName)
            }
        }        
    }
}

#### Uncomment the lines (or add new ones) corresponding to the folders you want to rotate files in
#### You can add -test to test the system without actually rotating or deleting
#### Additional parameters (add as -paramname value to the rotateFiles commands below)
# Number of days worth of calibrations to keep (uncompressed)
# daysToKeep = 7,
#any calibration directories older than today - $daysToKeep will be compressed into an archive by week
#number of weekly archives to keep
# archiveWeeksToKeep = 4,
#any weekly archives older than today - $archiveWeeksToKeep will be compressed into a monthly archive
#number of monthly archives to keep
# archiveMonthsToKeep = 2,
#any monthly archives older than today - $archiveMonthsToKeep will be compressed into a yearly archive
#number of yearly archives to keep
# archiveYearsToKeep = 1,
#any yearly archives older than today - $archiveYearsToKeep will be deleted
#maximum size in MB that we'll allow the calibration folder to get before deleting
#deleting will start with year archives, then month archives, etc... until we are below the threshold
# maxFolderSize = 100

rotateFiles "C:\3DTelemedicine\DATA\AllCalibrations" -test
#rotateFiles "C:\Users\BCUTLER\AppData\LocalLow\Microsoft Research\FusionLogs" -ProcessFiles -maxFolderSize 1000 -daysToKeep 4 =test
rotateFiles "C:\Users\BCUTLER\AppData\LocalLow\Microsoft Research\MultiViewCalibLogs" -test -ProcessFiles
#rotateFiles "C:\Users\BCUTLER\Documents\Telemedicine\snapshots" -maxFolderSize 10000 -test