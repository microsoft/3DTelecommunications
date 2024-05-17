using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.ServiceProcess;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

namespace _3DTM_Launcher_Service_Updater
{
    class Program
    {
        static string updaterLockFile = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "3DTMUpdater.lock");
        static void Main(string[] args)
        {
            SharpConfig.Configuration Config;
            const string configFilename = "3DTelemedicine.cfg";
            string configPath;
            configPath = Environment.GetEnvironmentVariable("3DTelemedicine_Dir");
            if (!String.IsNullOrEmpty(configPath))
            {
                configPath += Path.DirectorySeparatorChar + configFilename;
            }
            else
            {
                // Otherwise, try and load it from the current directory
                configPath = Environment.CurrentDirectory + Path.DirectorySeparatorChar + configFilename;
            }
            Console.WriteLine("Setting configPath to " + configPath);

            Config = SharpConfig.Configuration.LoadFromFile(configPath);

            // Log to the event log
            System.Diagnostics.EventLog AppLog = 
                new System.Diagnostics.EventLog
                {
                    Source = "Peabody Launcher Service Updater"
                };

            if (File.Exists(updaterLockFile))
            {
                AppLog.WriteEntry($"Lock file {updaterLockFile} already exists.  Cannot continue.  Exiting.");
                return;
            }
            File.Create(updaterLockFile);

            string binaryLocation;
            if (!Config["3DTMLauncherService"].Contains("BinaryLocation"))
                binaryLocation = "C:\\3DTelemedicine\\3DTMLauncherService";
            else
                binaryLocation = Config["3DTMLauncherService"]["BinaryLocation"].StringValue;

            // Step 1 - Check for the "new" binary
            string newBinaryFilename = "C:\\3DTelemedicine\\3DTMLauncherService.new.zip"; // default
            if (Config["3DTMLauncherService"].Contains("UpdatedZipFilename"))
            {
                newBinaryFilename = Config["3DTMLauncherService"]["UpdatedZipFilename"].StringValue;
            }

            AppLog.WriteEntry($"Launcher Service Updater has started.  Looking for new zip {newBinaryFilename}");
            if (!File.Exists(newBinaryFilename))
            {
                AppLog.WriteEntry($"Could not file {newBinaryFilename}.  Cannot update.  Exiting.", EventLogEntryType.Error);
                return;
            }

            // Step 2 - Stop the existing service
            AppLog.WriteEntry($"Stopping the 3DTM Launcher Service");
            ServiceController service = new ServiceController("3DTMLauncherService");
            try
            {
                service.Stop();
                service.WaitForStatus(ServiceControllerStatus.Stopped, TimeSpan.FromSeconds(30));
                if (service.Status != ServiceControllerStatus.Stopped)
                {
                    AppLog.WriteEntry("The service did not stop after 30 seconds.  Could not stop the service.  Aborting update.", EventLogEntryType.Error);
                    return;
                }
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Couldn't gracefully shutdown the service: " + e.Message + " Kiling.", EventLogEntryType.Warning);
                foreach (var proc in Process.GetProcessesByName("3DTMLauncherService"))
                {
                    proc.Kill();
                }
            }
            // Step 3 - Unzip the binary and overwrite the service
            // zip the old verison
            string previousZipFile = binaryLocation + ".old.zip";
            if(File.Exists(previousZipFile))
            {
                AppLog.WriteEntry(previousZipFile + " already exists.  Deleting to make room for new backup.");
                try
                {
                    File.Delete(previousZipFile);
                }
                catch (System.IO.IOException e)
                {
                    AppLog.WriteEntry("Could not delete " + previousZipFile + " Error: " + e.Message, EventLogEntryType.Warning);
                }
            }
            try
            {
                System.IO.Compression.ZipFile.CreateFromDirectory(binaryLocation, previousZipFile);
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Could not create a backup.  Error:" + e.Message, EventLogEntryType.Warning);
            }
            // delete the old folder
            var dir = new DirectoryInfo(binaryLocation);
            try
            {
                dir.Delete(true);
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Could not delete the original directory!  Installation will probably fail. Error: " + e.Message);
            }
            // unzip the new version
            try
            {
                System.IO.Compression.ZipFile.ExtractToDirectory(newBinaryFilename, binaryLocation);
                AppLog.WriteEntry($"{previousZipFile} created from {binaryLocation}.  {newBinaryFilename} extracted to {binaryLocation}");
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Could not unzip the new version. Error: " + e.Message, EventLogEntryType.Error);
            }

            // Step 4 - Restart the service
            finally
            {
                try
                {
                    service.Start();
                    service.WaitForStatus(ServiceControllerStatus.Running, TimeSpan.FromSeconds(30));
                    if (service.Status != ServiceControllerStatus.Running)
                    {
                        AppLog.WriteEntry("The service did not start after 30 seconds.  Please review the service logs and retry.", EventLogEntryType.Error);
                    }
                    AppLog.WriteEntry("Service updated successfully!");
                }
                catch (Exception e)
                {
                    AppLog.WriteEntry("Couldn't start the service automatically.  You will need to manually restart.  Message: " + e.Message, EventLogEntryType.Error);
                }
                File.Delete(updaterLockFile);
            }
        }
    }
}
