$arg0 = ($(git describe --tags --abbrev=0 --match "v*"))
Write-Host "Arg0 = $arg0"

$arg1 = ($(git rev-parse HEAD))
write-host "Arg1 = $arg1"

#$arg2 = ($(git log -1 --format=`%ct))
$lastTag = "$(git describe --tags --abbrev=0 --match "v*" `@)..HEAD"
$arg2 = ($(git log --oneline $lastTag | measure-object -line)).Lines
write-host "Arg2 = $arg2"

$arg3 = $args[0]

$success = $arg0 -match "v([0-9]*).([0-9]*).([0-9]*)"
if(!$success)
{
  write-host -ForegroundColor Red "ERROR.  Latest tag is not in SemRev format! ($arg0)"
  $major = 0;
  $minor = 0;
  $patch = 0;
}
else {
  $major = [int]$matches[1]
  $minor = [int]$matches[2]
  $patch = [int]$matches[3]    
}
$build_string = "$major.$minor.$patch"

$branchname = ($(git rev-parse --abbrev-ref HEAD))
$branchname = $branchname.Replace("/", "-")

 $sha1 = $arg1

 $commit = $arg2 

 $destinationDir = $arg3
 $cppfile = "3dtm_version.h"
 $csfile = "3dtm_version.config"
 $jsonfile = "3dtm_version.json"

$build_string = $build_string + '-'+$branchname+'.' + $commit
"Writing version " + $build_string + " to file: " + $destinationDir + "/"+$cppfile+" (C++)"

if( ( Test-Path $destinationDir/$cppfile ) )
{
  Remove-Item $destinationDir/$cppfile
}

"#define VERSION_MAJOR " + $major | Out-File $destinationDir/$cppfile -Append
"#define VERSION_MINOR " + $minor | Out-File $destinationDir/$cppfile -Append
"#define VERSION_PATCH " + $patch | Out-File $destinationDir/$cppfile -Append
'#define VERSION_STRING "' + $major + '.' + $minor + '.' + $patch + '+' + $commit + '"' | Out-File $destinationDir/$cppfile -Append
'#define VERSION_SHA1 "' + $sha1 + '"' | Out-File $destinationDir/$cppfile -Append
'#define VERSION_COMMITS ' + $commit | Out-File $destinationDir/$cppfile -Append
'#define VERSION_BRANCH_NAME "' + $branchname + '"' | Out-File $destinationDir/$cppfile -Append

'#define VERSION_DESCRIPTION "' + $build_string + '"' | Out-File $destinationDir/$cppfile -Append

"version.h written!"

"Writing $destinationDir/$jsonfile"
#Make a powershell object
$version = @{Major=$major;Minor=$minor;Patch=$patch;Description=$build_string;Sha=$sha1;Commits=$commit;BranchName=$branchname}
$version
$json = @{Version=$version}
$text = $json | convertto-json 
$text | out-file $destinationDir/$jsonfile

$savedir = "$destinationdir/$csfile"
"Writing version to $savedir (C#)"
# initialize the xml object
$appConfig = New-Object XML
$configuration = $appConfig.AppendChild($appConfig.CreateElement("configuration"));
$appSettings = $configuration.AppendChild($appConfig.CreateElement("appSettings"));
foreach($v in $version.Keys)
{
  $add = $appConfig.CreateElement("add");
  $add.SetAttribute("key", $v);
  $add.SetAttribute("value", $version[$v]);
  $appSettings.AppendChild($add);
}

# save the updated config file
$appConfig.Save($savedir)

# put just the description in a .version File
$build_string | Out-File $destinationDir/".version" 