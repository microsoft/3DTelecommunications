using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.ServiceProcess;
using System.Text;
using NetMQ.Sockets;
using NetMQ;
using System.Net.Sockets;
using System.Net;
using System.Configuration;
using System.Threading;
using PeabodyNetworkingLibrary;
using System.ComponentModel;
using System.Runtime.InteropServices;
using SharpConfig;
using System.IO;
using System.Globalization;
using System.Linq;
using System.Diagnostics.Eventing.Reader;
using System.IO.Compression;

namespace PeabodyLauncherService
{
    public partial class PeabodyLauncherService : ServiceBase
    {
        System.Diagnostics.EventLog AppLog;
        bool Running = false;
        NetMQPoller controlPanelEventPoller;
        SubscriberSocket eventSubscriberSocket;
        PublisherSocket statusPublisherSocket;

        Thread applicationMonitorThread;    // Monitors application existence and state
        Thread applicationLauncherThread;   // Launches the application
        Thread heartbeatThread;
        const int HeartbeatSleepInMilliseconds = 5000;

        Process RunningApplicationProcess;
        Mutex ProcessMutex;
        Mutex StateMutex;
        PeabodyNetworkingLibrary.SOFTWARE_STATE[] SoftwareStates = new SOFTWARE_STATE[(int)PeabodyNetworkingLibrary.SOFTWARE.COUNT];

        WindowsEventLog EventLogger;
        readonly int TimeoutBeforeKillingApplicationInMilliseconds = 8000;  // how long should the service wait after sending a "Close" event before force-killing the application?
            
        // TODO: TVS, refactor code to fully support SOFTWARE.CALIBRATION. ADO feature #9689
        SOFTWARE ApplicationToRun;// = ConfigurationManager.AppSettings["ApplicationToRun"] == "Fusion" ? SOFTWARE.FUSION : SOFTWARE.RENDER;
        String EventPort;// = ConfigurationManager.AppSettings["EventPort"];
        String StatusPort;// = ConfigurationManager.AppSettings["StatusPort"];
        String ControlPanelIP;// = ConfigurationManager.AppSettings["ControlPanel"];
        String ProcessToMonitor;// = ConfigurationManager.AppSettings["ProcessRegex"];
        String CalibrationFilePath;// = ConfigurationManager.AppSettings["CalibrationFilePath"];
        bool RunCalibrationSoftwareOnThisMachine = false;

        SharpConfig.Configuration Config;
        readonly string configFilename = "3DTelemedicine.cfg";
        string configPath;

        // Can be overridden in config file
        string fusionLogPath = "C:\\users\\BCUTLER\\AppData\\LocalLow\\Microsoft Research\\FusionLogs";
        string renderLogPath = "C:\\users\\BCUTLER\\AppData\\LocalLow\\Microsoft Research\\HoloPortRenderer";

        PeabodyNetworkingLibrary.VersionData versionData;

        public PeabodyLauncherService()
        {
            InitializeComponent();
            StateMutex = new Mutex();
            ProcessMutex = new Mutex();
            AppLog =
                new System.Diagnostics.EventLog
                {
                    Source = "Peabody Launcher Service"
                };
            AppLog.WriteEntry("Service started.");
            EventLogger = new WindowsEventLog();
            EventLogger.CreateLogSource("3DTelemedicineLauncherService", "Application");
            foreach(SOFTWARE software in Enum.GetValues(typeof(SOFTWARE)))
            {
                SetState(software, SOFTWARE_STATE.UNKNOWN);
            }
        }

        private int Verbosity()
        {
            return Config.Contains("3DTMLauncherService", "Verbosity")?Config["3DTMLauncherService"]["Verbosity"].IntValue : 0;
        }
        private static List<String> GetLocalIPAddresses()
        {
            List<String> Addresses = new List<string>();
            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    Addresses.Add(ip.ToString());
                }
            }
            if (Addresses.Count > 0)
            {
                return Addresses;
            }
            throw new Exception("Local IP Address Not Found!");
        }

        private void SetRunningProcess(Process p)
        {
            using (ProcessMutex)
            {
                RunningApplicationProcess = p;
            }
        }
        
        private Process GetRunningProcess()
        {
            using (ProcessMutex)
            {
                return RunningApplicationProcess;
            }
        }
        
        private SOFTWARE_STATE[] GetStates()
        {
            using (StateMutex)
            {
                return SoftwareStates;
            }
        }

        private int[] GetStatesAsIntArray()
        {
            SOFTWARE_STATE[] states = GetStates();
            int[] ints = new int[states.Length];
            using (StateMutex)
            {
                for(int i = 0; i < (int)SOFTWARE.COUNT; i++)
                {
                    ints[i] = (int)states[i];
                }
            }
            return ints;
        }

        private SOFTWARE_STATE GetState(SOFTWARE software)
        {
            if (software == SOFTWARE.COUNT)
                return SOFTWARE_STATE.UNKNOWN;
            using (StateMutex)
            {
                return SoftwareStates[(int)software];
            }
        }
        private void SetState(SOFTWARE software, SOFTWARE_STATE state)
        {
            if (software == SOFTWARE.COUNT)
                return;
            using (StateMutex)
            {
                SoftwareStates[(int)software] = state;
            }
        }
        
        protected override void OnStart(string[] args)
        {
            // Look for an environment variable for the config file location
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
            AppLog.WriteEntry("Setting configPath to " + configPath, EventLogEntryType.Information);

            versionData = PeabodyNetworkingLibrary.Utility.ReadVersionDataFromConfig(Environment.GetEnvironmentVariable("3DTelemedicine_Dir") + "\\3DTMlauncherService\\3dtm_version.config");
            
            AppLog.WriteEntry($"Version {versionData.Major}.{versionData.Minor}.{versionData.Patch}+{versionData.Commits}  {versionData.Description}");

            List<String> ips = GetLocalIPAddresses();
            try
            {
                Config = SharpConfig.Configuration.LoadFromFile(configPath);
                if(Verbosity() > 0)
                {
                    AppLog.WriteEntry("Verbose mode activated.  To reduce logging, set [3DTMLauncherService][Verbose] to 0", EventLogEntryType.Information);
                }
                EventPort = Config["Ports"]["EventPort"].StringValue;
                StatusPort = Config["Ports"]["DaemonPort"].StringValue;
                ControlPanelIP = Config["Network"]["ControlPanelIPAddress"].StringValue;
                // Figure out if we are on fusion or renderer by looking to see which one matches our IP address
                bool isFusion = false;
                foreach (string ip in ips)
                {
                    if (ip.Equals(Config["Network"]["FusionIPAddress"].StringValue))
                    {
                        isFusion = true;
                        AppLog.WriteEntry("Found a local IP matching Fusion's IP address.  Setting application to run as Fusion", EventLogEntryType.Information);
                    }
                    else if(Verbosity() > 0)
                    {                       
                        AppLog.WriteEntry($"Local IP {ip} does not match Fusion's IP {Config["Network"]["FusionIPAddress"]}", EventLogEntryType.Information);
                    }
                    if (ip.Equals(Config["Network"]["CalibrationSoftwareIPAddress"].StringValue))
                    {
                        RunCalibrationSoftwareOnThisMachine = true;
                        AppLog.WriteEntry("Found a local IP matching CalibrationSoftware's IP address. Enabling daemon to run CalibrationSoftware.", EventLogEntryType.Information);
                    }
                    else if(Verbosity() > 0)
                    {                       
                        AppLog.WriteEntry($"Local IP {ip} does not match CalibrationSoftware's IP {Config["Network"]["CalibrationSoftwareIPAddress"]}", EventLogEntryType.Information);
                    }
                }
                ApplicationToRun = isFusion ? SOFTWARE.FUSION : SOFTWARE.RENDER;
                ProcessToMonitor = isFusion? Config["Fusion"]["ProcessRegex"].StringValue : Config["Renderer"]["ProcessRegex"].StringValue;
                CalibrationFilePath = (ApplicationToRun == SOFTWARE.FUSION) ? Config["Fusion"]["CalibrationDirectory"].StringValue : Config["Renderer"]["CalibrationDirectory"].StringValue;

                fusionLogPath = Config["Debug"].Contains("FusionLogDirectory") ? Config["Debug"]["FusionLogDirectory"].StringValue : fusionLogPath;
                renderLogPath = Config["Debug"].Contains("RenderLogDirectory") ? Config["Debug"]["RenderLogDirectory"].StringValue : renderLogPath;

                AppLog.WriteEntry("Loaded the config file from " + configPath, EventLogEntryType.Information);
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Could not load the configuration file from " + configPath + ". Error: .  Attempting to load from the old location.  This is NOT recommended!" + e.Message, EventLogEntryType.Error);
                ApplicationToRun = ConfigurationManager.AppSettings["ApplicationToRun"] == "Fusion" ? SOFTWARE.FUSION : SOFTWARE.RENDER;
                EventPort = ConfigurationManager.AppSettings["EventPort"];
                StatusPort = ConfigurationManager.AppSettings["StatusPort"];
                ControlPanelIP = ConfigurationManager.AppSettings["ControlPanel"];
                ProcessToMonitor = ConfigurationManager.AppSettings["ProcessRegex"];
                CalibrationFilePath = ConfigurationManager.AppSettings["CalibrationFilePath"];
            }

            Running = true;
            controlPanelEventPoller = new NetMQPoller();

            string controlPanelEventPublisherTCPUri = $"tcp://{ControlPanelIP}:{EventPort}";
            eventSubscriberSocket = new SubscriberSocket();
            SetDefaultSocketOptions(eventSubscriberSocket.Options);
            eventSubscriberSocket.Connect(controlPanelEventPublisherTCPUri);
            AppLog.WriteEntry("Creating control panel event subscriber on port " + controlPanelEventPublisherTCPUri, EventLogEntryType.Information);
            eventSubscriberSocket.Subscribe(""); // Subscribe to everything
            eventSubscriberSocket.ReceiveReady += ReceivedControlPanelEvent;
            controlPanelEventPoller.Add(eventSubscriberSocket);

            SetRunningProcess(null);
            applicationLauncherThread = new Thread(ApplicationLauncherThread);
            applicationLauncherThread.Start();

            statusPublisherSocket = new PublisherSocket();
            SetDefaultSocketOptions(statusPublisherSocket.Options);
            AppLog.WriteEntry("Creating staus publisher socket on port " + StatusPort, EventLogEntryType.Information);
            foreach (string ip in ips)
            {
                string statusPublisherSocketUri = $"tcp://{ip}:{StatusPort}";
                try
                {
                    statusPublisherSocket.Bind(statusPublisherSocketUri);
                }
                catch (NetMQ.AddressAlreadyInUseException)
                {
                    AppLog.WriteEntry($"Could not bind to {statusPublisherSocketUri}  -- Address is already in use.  Is the service already running?", EventLogEntryType.Error);
                    if (Environment.UserInteractive)
                    {
                        Console.WriteLine($"Could not bind to {statusPublisherSocketUri}-- Address is already in use.  Is the service already running?");
                    }
                    System.Environment.Exit(-1);
                }
            }

            controlPanelEventPoller.RunAsync();
            applicationMonitorThread = new Thread(MonitorApplicationStates);
            applicationMonitorThread.Start();

            heartbeatThread = new Thread(Heartbeat);
            heartbeatThread.Start();
        }

        private void SetDefaultSocketOptions(SocketOptions options)
        {
            options.ReceiveBuffer = 1000000;
            options.ReceiveHighWatermark = 10000;
            options.SendBuffer = 1000000;
            options.SendHighWatermark = 10000;
        }

        private void ApplicationLauncherThread()
        {
            while(Running)
            {
                // Check for any changes of the state to "STARTING"
                if (GetState(ApplicationToRun) == SOFTWARE_STATE.STARTED && GetRunningProcess() == null)
                {
                    // You can't start a GUI application from inside a service, so we can't use Process.Start() here
                    // instead, we'll fire a windows even that's attached to a task scheduler that will launch it for us
                    if(ApplicationToRun == SOFTWARE.FUSION)
                        EventLogger.WriteLaunchFusionEvent(Environment.MachineName);
                    else if(ApplicationToRun == SOFTWARE.RENDER)
                        EventLogger.WriteLaunchRenderEvent(Environment.MachineName);
                }
                else if(GetState(SOFTWARE.CALIBRATION) == SOFTWARE_STATE.STARTED)
                {
                    EventLogger.WriteLaunchCalibrationSoftwareEvent(Environment.MachineName);
                    // TODO:  Move this down into the MonitorApplicationStates thread.  Requires
                    // refactoring this code to handle ApplicationToRun being multiple software (Fusion + Calibration)
                    // ADO #9689
                    SetState(SOFTWARE.CALIBRATION, SOFTWARE_STATE.RUNNING);
                    PublishApplicationState();
                }
                else if (GetState(ApplicationToRun) == SOFTWARE_STATE.STOPPED)
                {
                    using (ProcessMutex)
                    {
                        if (RunningApplicationProcess != null)
                        {
                            AppLog.WriteEntry("Launcher is stopping " + ApplicationToRun, EventLogEntryType.Information);
                            RunningApplicationProcess.CloseMainWindow();
                            if (!RunningApplicationProcess.WaitForExit(TimeoutBeforeKillingApplicationInMilliseconds))
                            {
                                if (GetState(ApplicationToRun) != SOFTWARE_STATE.NOT_RUNNING)
                                {
                                    AppLog.WriteEntry("Launcher is killing " + ApplicationToRun, EventLogEntryType.Information);
                                    try
                                    {
                                        RunningApplicationProcess.Kill();
                                    }
                                    catch (Win32Exception)
                                    {
                                        AppLog.WriteEntry("Launcher was unable to kill the running process.  Cause could be that it was already in process of closing.", EventLogEntryType.Error);
                                    }
                                }
                            }
                            // Could be nullified already by the other process
                            if (RunningApplicationProcess != null)
                            {
                                RunningApplicationProcess.Dispose();
                            }
                            RunningApplicationProcess = null;
                        }
                        else
                        {
                            AppLog.WriteEntry("Got a stop request but I can't find an active process.", EventLogEntryType.Warning);
                        }
                    }
                }
                System.Threading.Thread.Sleep(100);
            }
        }

        private void Heartbeat()
        {
            if(Config["3DTMLauncherService"].Contains("DisableHeartbeat") && Config["3DTMLauncherService"]["DisableHeartbeat"].BoolValue)
            {
                AppLog.WriteEntry("Disabling heartbeat thread per config DisableHeartbeat value.", EventLogEntryType.Warning);
                return;
            }
            while(Running)
            {
                byte[] data = new byte[1];
                data[0] = (byte)CPC_STATUS.IS_ALIVE;
                statusPublisherSocket.SendFrame(data);

                // This way we'll check for a shutting-down service much faster than
                // waiting the full HeartbeatSleepInMilliseconds would 
                int elapsedMilliseconds = 0;
                while(elapsedMilliseconds < HeartbeatSleepInMilliseconds && Running)
                {
                    System.Threading.Thread.Sleep(100);
                    elapsedMilliseconds += 100;
                }
                if(Verbosity() > 1)
                {
                    AppLog.WriteEntry("3DTM Heartbeat");
                }
            }
        }

        private void MonitorApplicationStates()
        {
            PublishApplicationState();
            if(Verbosity() > 0)
            {
                AppLog.WriteEntry($"I'm monitoring the existence of {ProcessToMonitor} in the process list.");
            }
            while(Running)
            {
                // Get a list of all running processes
                Process[] processList = Process.GetProcesses();
                // Search for ProcessToMonitor in that list
                Process foundProcess = null;
                string verboseOutputString = "Searched Process List: \n";
                foreach(var process in processList)
                {
                    if(Verbosity() > 1)
                    {
                        verboseOutputString += process.ProcessName + "\n";
                    }
                    if(process.ProcessName.Contains(ProcessToMonitor))
                    {
                        if(Verbosity() > 1)
                        {
                            verboseOutputString += "^^^^^^^^^^^ Found Process ^^^^^^^^^^^^\n";
                        }
                        foundProcess = process;
                        break;
                    }
                }
                SetRunningProcess(foundProcess);
                if(Verbosity() > 1)
                {
                    EventLog.WriteEntry(verboseOutputString, EventLogEntryType.Information);
                }
                // If the state != previous state, publish update
                if (GetRunningProcess() != null && GetState(ApplicationToRun) != SOFTWARE_STATE.RUNNING && GetState(ApplicationToRun) != SOFTWARE_STATE.STOPPED)  // don't transition to running if I've gotten a stop request, that means it just hasn't stopped yet
                {
                    if(Verbosity() > 1)
                    {
                        EventLog.WriteEntry($"Running process: {GetRunningProcess()}\nState: {GetState(ApplicationToRun)}.  Setting state to RUNNING.", EventLogEntryType.Information);
                    }
                    SetState(ApplicationToRun, SOFTWARE_STATE.RUNNING);
                    PublishApplicationState();
                }
                else if (GetRunningProcess() == null && GetState(ApplicationToRun) != SOFTWARE_STATE.STARTED && GetState(ApplicationToRun) != SOFTWARE_STATE.NOT_RUNNING) // only ignore not finding the process if we're still starting up, or we're already in not running
                {
                    if (Verbosity() > 1)
                    {
                        EventLog.WriteEntry($"Running process is null.  State: {GetState(ApplicationToRun)}. Setting state to NOT RUNNING.", EventLogEntryType.Information);
                    }
                    SetState(ApplicationToRun, SOFTWARE_STATE.NOT_RUNNING);
                    PublishApplicationState();
                }
                System.Threading.Thread.Sleep(100);
            }
        }

        private void ReceivedControlPanelEvent(object sender, NetMQSocketEventArgs e)
        {
            ////// when a status update is received, handle
            byte[] results;
            while (this.Running && e.Socket.TryReceiveFrameBytes(System.TimeSpan.FromMilliseconds(100), out results))
            {
                if (results != null && results.Length > 0)
                {
                    // Parse results and handle appropriately
                    CONTROL_PANEL_EVENT EventType = (CONTROL_PANEL_EVENT)results[0];
                    switch(EventType)
                    {
                        case CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED:
                            {
                                if (results.Length > 1)
                                {
                                    SOFTWARE startType = (SOFTWARE)results[1];
                                    if (startType == ApplicationToRun)
                                    {
                                        if (startType == SOFTWARE.FUSION)
                                        {
                                            if (Environment.UserInteractive)
                                            {
                                                Console.WriteLine("Got a fusion start request.");
                                            }
                                            else if (Verbosity() > 0)
                                            {
                                                AppLog.WriteEntry("Got a fusion start request.");
                                            }
                                            SetState(ApplicationToRun, SOFTWARE_STATE.STARTED);
                                        }
                                        else if (startType == SOFTWARE.RENDER)
                                        {
                                            if (Environment.UserInteractive)
                                            {
                                                Console.WriteLine("Got a render start request.");
                                            }
                                            else if (Verbosity() > 0)
                                            {
                                                AppLog.WriteEntry("Got a render start request.");
                                            }
                                            SetState(ApplicationToRun, SOFTWARE_STATE.STARTED);
                                        }
                                    }
                                    else if(startType == SOFTWARE.CALIBRATION && RunCalibrationSoftwareOnThisMachine)
                                    {
                                        if(results.Length > 2 && results[2] == (char)PeabodyNetworkingLibrary.CALIBRATION_SOFTWARE_COMPONENT.BACKEND)
                                        {
                                            if (Environment.UserInteractive)
                                            {
                                                Console.WriteLine("Got a start calibration software request.");
                                            }
                                            else if (Verbosity() > 0)
                                            {
                                                AppLog.WriteEntry("Got a start calibration software request.");
                                            }
                                            SetState(SOFTWARE.CALIBRATION, SOFTWARE_STATE.STARTED);
                                        }                                         
                                    }
                                    else
                                    {
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry("Received an unknown software application start request: " + startType, EventLogEntryType.Error);
                                        }
                                    }
                                    PrintState();
                                }
                                else
                                {
                                    // software mode start request, ignore
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED:
                            {
                                if (results.Length > 1)
                                {
                                    SOFTWARE startType = (SOFTWARE)results[1];
                                    if (ApplicationToRun == startType)
                                    {
                                        if (startType == SOFTWARE.FUSION)
                                        {
                                            AppLog.WriteEntry("Received a software application stop request.  Killing FUSION", EventLogEntryType.Warning);
                                            // in the application monitor thread, we will actually get the process
                                            // and send a closeWindow event, rather than doing the scheduled tasks (taskkill)
                                            SetState(ApplicationToRun, SOFTWARE_STATE.STOPPED);
                                        }
                                        else if (startType == SOFTWARE.RENDER)
                                        {
                                            AppLog.WriteEntry("Received a software application stop request.  Killing RENDER", EventLogEntryType.Warning);
                                            // in the application monitor thread, we will actually get the process
                                            // and send a closeWindow event, rather than doing the scheduled tasks (taskkill)
                                            SetState(ApplicationToRun, SOFTWARE_STATE.STOPPED);
                                        }
                                    }
                                    else
                                    {
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry("Received an unknown software application stop request: " + startType, EventLogEntryType.Information);
                                        }
                                    }
                                    PrintState();
                                }
                                else
                                {
                                    // software mode stop request, ignore
                                }
                                PublishApplicationState();
                                break;
                            }
                        case CONTROL_PANEL_EVENT.CONTROL_PANEL_STATE_UPDATE_REQUESTED:
                            {
                                if (Verbosity() > 1)
                                {
                                    AppLog.WriteEntry("Got a state update request.  Publishing state.");
                                }
                                PublishApplicationState();
                                break;
                            }
                        case CONTROL_PANEL_EVENT.CALIBRATION_DATA:
                            {
                                // Deserialize the JSON
                                string jsonString = Encoding.Default.GetString(results, sizeof(int), results.Length - sizeof(int)); //get rid of the first byte which is the header
                                if (Environment.UserInteractive)
                                {
                                    Console.WriteLine("Got calibration data.  Raw JSON: \n" + jsonString + "\n");
                                }
                                else if (Verbosity() > 1)
                                {
                                    AppLog.WriteEntry("Got calibration data.  Raw JSON: \n" + jsonString + "\n");
                                }
                                try
                                {
                                    CalibrationContainer calibrationContainer = CameraCalibrationHelper.Deserialize(jsonString);
                                    if (ApplicationToRun == SOFTWARE.FUSION && calibrationContainer.Name == "FusionCalibrations")
                                    {
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Received new calibration data for fusion.  Saving to disk at {CalibrationFilePath}", EventLogEntryType.Information);
                                        }
                                        if (Environment.UserInteractive)
                                        {
                                            Console.WriteLine($"Received new calibration data for fusion.  Saving to disk at {CalibrationFilePath}");
                                        }
                                        CameraCalibrationHelper.SaveToCalibBACamFiles(calibrationContainer, CalibrationFilePath);
                                        CameraCalibrationHelper.SaveToCalibColorCamsFile(calibrationContainer, CalibrationFilePath);
                                    }
                                    else if (ApplicationToRun == SOFTWARE.RENDER && calibrationContainer.Name == "RenderCalibrations")
                                    {
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Received new calibration data for renderer.  Saving to disk at {CalibrationFilePath}", EventLogEntryType.Information);
                                        }
                                        if (Environment.UserInteractive)
                                        {
                                            Console.WriteLine($"Received new calibration data for renderer.  Saving to disk at {CalibrationFilePath}");
                                        }
                                        CameraCalibrationHelper.SaveToCalibBACamFiles(calibrationContainer, CalibrationFilePath);
                                        CameraCalibrationHelper.SaveToCalibColorCamsFile(calibrationContainer, CalibrationFilePath);
                                    }
                                    else
                                    {
                                        if (Environment.UserInteractive)
                                        {
                                            Console.WriteLine($"Received new calibration data for {calibrationContainer.Name}");
                                            Console.WriteLine(jsonString);
                                        }
                                        else if (Verbosity() > 1)
                                        {
                                            AppLog.WriteEntry($"Received new calibration data for {calibrationContainer.Name}. Data: {jsonString}");
                                        }
                                    }
                                    // Acknowledge that the new calibration data was successfully received
                                    byte[] data = new byte[1];
                                    data[0] = (byte)CPC_STATUS.RECEIVED_NEW_DATA;
                                    statusPublisherSocket.SendFrame(data);
                                }
                                catch (Exception exception)
                                {
                                    // Expected.  If user is loading calibration, camera calibs, which aren't packaged inside CalibrationContainers
                                    // will also be sent and will cause a JSON parsing exception.  We can safely ignore this.
                                    if (Environment.UserInteractive)
                                    {
                                        Console.WriteLine("Error parsing JSON: " + exception.Message + " Raw JSON: " + jsonString);
                                    }
                                    else if (Verbosity() > 1)
                                    {
                                        AppLog.WriteEntry("Error parsing JSON: " + exception.Message + "  Raw JSON: " + jsonString);
                                    }
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.SYSTEM_CONFIGURATION_UPDATE:
                            {
                                if (results.Length < 2)
                                {
                                    AppLog.WriteEntry("Received a system configuration update with no data.", EventLogEntryType.Error);
                                    return;
                                }
                                int configFileSize = BitConverter.ToInt32(results, 1);
                                string configFileContents = Encoding.Default.GetString(results, sizeof(int) + 1, configFileSize);
                                if (Verbosity() > 0)
                                {
                                    AppLog.WriteEntry($"Config size {configFileSize} bytes. Contents: [{configFileContents}]", EventLogEntryType.Information);
                                }
                                SharpConfig.Configuration newConfig = SharpConfig.Configuration.LoadFromString(configFileContents);
                                // When we get a new system config, we ignore the ApplicationToRun and ProcessRegex variables
                                // because we need to keep them specific to our current host machine (Fusion or Render) not
                                // set to the global system.  So we'll keep whatever our local file has before we overwrite
                                newConfig["3DTMLauncherService"]["ApplicationToRun"].StringValue = Config["3DTMLauncherService"]["ApplicationToRun"].StringValue;
                                newConfig["3DTMLauncherService"]["ProcessRegex"].StringValue = Config["3DTMLauncherService"]["ProcessRegex"].StringValue;

                                // Check if any of the changes effect our process
                                bool necessaryToRestart = DoesNewConfigSectionDiffer(Config["Network"], newConfig["Network"]) ||
                                    DoesNewConfigSectionDiffer(Config["Ports"], newConfig["Ports"]) ||
                                    DoesNewConfigSectionDiffer(Config["Fusion"], newConfig["Fusion"]) ||
                                    DoesNewConfigSectionDiffer(Config["Renderer"], newConfig["Renderer"]) ||
                                    DoesNewConfigSectionDiffer(Config["3DTMLauncherService"], newConfig["3DTMLauncherService"]) ||
                                    DoesNewConfigSectionDiffer(Config["Debug"], newConfig["Debug"]);

                                // config destination should be what my system environment variable says first
                                string configFileDestination = Environment.GetEnvironmentVariable("3DTelemedicine_dir");
                                if (String.IsNullOrEmpty(configFileDestination))
                                {
                                    configFileDestination = newConfig["Configuration"]["ConfigurationFile"].StringValue;
                                    if (String.IsNullOrEmpty(configFileDestination))
                                    {
                                        configFileDestination = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "/" + "3DTelemedicine.cfg";
                                        AppLog.WriteEntry("Received a system configuration update, but the 3DTelemedicine_dir environment variable is not set and there is no ConfigurationFile parameter in the new config.", EventLogEntryType.Error);
                                    }
                                }
                                else
                                {
                                    // Environment var is just the path, not the filename, so add it here
                                    configFileDestination += "/3DTelemedicine.cfg";
                                }
                                try
                                {
                                    File.WriteAllText(configFileDestination, configFileContents);
                                    AppLog.WriteEntry("Received a system configuration update.  Saved to " + configFileDestination, EventLogEntryType.Information);
                                    if (necessaryToRestart)
                                    {
                                        AppLog.WriteEntry("Configuration data updated.  Shutting down to reload new configuration.", EventLogEntryType.Warning);
                                        Environment.Exit(-1);
                                    }
                                }
                                catch (Exception e2)
                                {
                                    AppLog.WriteEntry("Could not save new configuration to " + configFileDestination + ". Reason: " + e2.Message, EventLogEntryType.Error);
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED:
                            {
                                try
                                {
                                    // Create the build version struct and send it back
                                    byte[] data = PeabodyNetworkingLibrary.Utility.CreateBuildVersionPacket(
                                        versionData.Major,
                                        versionData.Minor,
                                        versionData.Patch,
                                        versionData.Commits,
                                        versionData.BranchName,
                                        versionData.Description,
                                        versionData.Sha1);

                                    statusPublisherSocket.SendFrame(data);
                                }
                                catch (Exception ex)
                                {
                                    AppLog.WriteEntry("Error converting build version into packet." + ex.Message, EventLogEntryType.Error);
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.LOG_COLLECTION_REQUESTED:
                            {
                                if (Verbosity() > 0)
                                {
                                    AppLog.WriteEntry($"Got a log collection request");
                                }
                                EventLogSession els = new EventLogSession();
                                string queryString = "*";
                                string tempFileName = "";
                                // gather the fusion or render log
                                if (ApplicationToRun == SOFTWARE.FUSION)
                                {
                                    // Get the latest logfile in the fusion log folder
                                    if (!Directory.Exists(fusionLogPath))
                                    {
                                        AppLog.WriteEntry($"Error.  Fusion log path \"{fusionLogPath}\" does not exist.", EventLogEntryType.Error);
                                        return;
                                    }
                                    DirectoryInfo info = new DirectoryInfo(fusionLogPath);
                                    FileInfo[] files = info.GetFiles("Fusion_Output_*.txt").OrderByDescending(p => p.CreationTime).ToArray();
                                    if (files.Length > 0)
                                    {
                                        // send the first (newest)
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Sending log file {files[0].Name}");
                                        }
                                        PublishLogFile(files[0]);
                                    }
                                    files = info.GetFiles("Fusion_Error_*.txt").OrderByDescending(p => p.CreationTime).ToArray();
                                    if (files.Length > 0)
                                    {
                                        // send the first (newest)
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Sending log file {files[0].Name}");
                                        }
                                        PublishLogFile(files[0]);
                                    }
                                    // gather the EventLog for KNCS
                                    queryString = "<QueryList>" +
                                            "<Query Id=\"0\" Path=\"Application\">" +
                                            "<Select Path=\"Application\"> *[System[Provider[@Name = 'Kinect Nano Communcator Service'] and TimeCreated[timediff(@SystemTime) &lt;= 43200000]]] </Select>" +
                                            "</Query>" +
                                            "</QueryList>";
                                    tempFileName = Path.GetTempPath() + "\\KNCSEventLog.evtx";
                                    if (File.Exists(tempFileName)) File.Delete(tempFileName);
                                    try
                                    {
                                        els.ExportLogAndMessages("Application",
                                            PathType.LogName,
                                            queryString,
                                            tempFileName,
                                            true,
                                            CultureInfo.CurrentCulture);
                                        if (File.Exists(tempFileName))
                                        {
                                            FileInfo fileInfo = new FileInfo(tempFileName);
                                            if (Verbosity() > 0)
                                            {
                                                AppLog.WriteEntry($"Sending event log file {fileInfo.Name}");
                                            }
                                            PublishLogFile(fileInfo);
                                        }
                                    }
                                    catch (EventLogException e2)
                                    {
                                        AppLog.WriteEntry($"Error exporting event log: {e2.Message}");
                                    }
                                }
                                else if (ApplicationToRun == SOFTWARE.RENDER)
                                {
                                    if (File.Exists(renderLogPath + "\\Player.log"))
                                    {
                                        FileInfo file = new FileInfo(renderLogPath + "\\Player.log");
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Sending log file {file.Name}");
                                        }
                                        PublishLogFile(file);
                                    }
                                    if (File.Exists(renderLogPath + "\\Player-prev.log"))
                                    {
                                        FileInfo file = new FileInfo(renderLogPath + "\\Player-prev.log");
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Sending log file {file.Name}");
                                        }
                                        PublishLogFile(file);
                                    }
                                }
                                // gather the EventLog for 3DTM
                                queryString = "<QueryList>" +
                                        "<Query Id=\"0\" Path=\"Application\">" +
                                        "<Select Path=\"Application\"> *[System[Provider[@Name = 'Peabody Launcher Service' or @Name = '3DTelemedicineLauncherService'] and TimeCreated[timediff(@SystemTime) &lt;= 43200000]]] </Select>" +
                                        "</Query>" +
                                        "</QueryList>";
                                tempFileName = Path.GetTempPath() + "\\3DTMEventLog.evtx";
                                if (File.Exists(tempFileName)) File.Delete(tempFileName);
                                try
                                {
                                    els.ExportLogAndMessages("Application",
                                        PathType.LogName,
                                        queryString,
                                        tempFileName,
                                        true,
                                        CultureInfo.CurrentCulture);
                                    if (File.Exists(tempFileName))
                                    {
                                        FileInfo fileInfo = new FileInfo(tempFileName);
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Sending event log file {fileInfo.Name}");
                                        }
                                        PublishLogFile(fileInfo);
                                    }
                                }
                                catch (EventLogException e2)
                                {
                                    AppLog.WriteEntry($"Error exporting event log: {e2.Message}");
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.NEW_BINARY_TRANSFER:
                            {
                                // first packet will be which software this pertains to
                                if (results.Length < 2)
                                {
                                    AppLog.WriteEntry($"Received a new binary transfer packet with no data!", EventLogEntryType.Error);
                                }
                                bool fileIsImportantToMe = (ApplicationToRun == SOFTWARE.FUSION && (SOFTWARE)results[1] == SOFTWARE.FUSION) ||
                                    (ApplicationToRun == SOFTWARE.RENDER && (SOFTWARE)results[1] == SOFTWARE.RENDER) ||
                                    (ApplicationToRun == SOFTWARE.FUSION && (SOFTWARE)results[1] == SOFTWARE.KINECT_COMMUNICATOR) ||
                                    ((SOFTWARE)results[1] == SOFTWARE.WINDOWS_SERVICE);

                                if (fileIsImportantToMe)
                                {
                                    if(Verbosity() > 0)
                                    {
                                        AppLog.WriteEntry($"Received a new binary transfer packet for application {(SOFTWARE)results[1]} that I care about.  Processing.");
                                    }
                                    int fileSize = BitConverter.ToInt32(results, 2);
                                    // make sure the software is stopped.  Kill it if not
                                    if (GetState(ApplicationToRun) == SOFTWARE_STATE.STARTED || GetState(ApplicationToRun) == SOFTWARE_STATE.RUNNING)
                                    {
                                        SetState(ApplicationToRun, SOFTWARE_STATE.STOPPED);
                                    }
                                    // save the transfer packet to a temporary file
                                    string tempFilename = Path.GetTempFileName();
                                    FileStream fs = new FileStream(tempFilename, FileMode.Truncate);
                                    if(Verbosity() > 0)
                                    {
                                        AppLog.WriteEntry($"Saving {fileSize} bytes to {tempFilename}");
                                    }
                                    fs.Write(results, sizeof(byte)+sizeof(byte)+sizeof(int), fileSize);
                                    fs.Close();

                                    string previousZipFile = "";
                                    string destinationDir = "";
                                    if (ApplicationToRun == SOFTWARE.FUSION && (SOFTWARE)results[1] == SOFTWARE.FUSION)
                                    {
                                        destinationDir = "C:\\3DTelemedicine\\Fusion";
                                    }
                                    else if (ApplicationToRun == SOFTWARE.RENDER && (SOFTWARE)results[1] == SOFTWARE.RENDER)
                                    {
                                        destinationDir = "C:\\3DTelemedicine\\Renderer";
                                    }
                                    else if (ApplicationToRun == SOFTWARE.FUSION && (SOFTWARE)results[1] == SOFTWARE.KINECT_COMMUNICATOR)
                                    {
                                        // stop the KNCS first
                                        ServiceController sc = new ServiceController("3DTMKinectNanoCommunicatorService");
                                        try
                                        {
                                            sc.Stop();
                                            destinationDir = "C:\\3DTelemedicine\\KinectNanoCommunicatorService";
                                        }
                                        catch (Exception ex)
                                        {
                                            AppLog.WriteEntry($"Couldn't stop the KNCS!", EventLogEntryType.Error);
                                            PublishInstallationResult((SOFTWARE)results[1], false, $"Couldn't stop the KNCS. {ex.Message}");
                                            return;
                                        }
                                    }
                                    else if ((SOFTWARE)results[1] == SOFTWARE.WINDOWS_SERVICE)
                                    {
                                        string destinationFilename = "C:\\3DTelemedicine\\3DTMLauncherService.new.zip"; // default
                                        if (Config["3DTMLauncherService"].Contains("UpdatedZipFilename"))
                                        {
                                            destinationFilename = Config["3DTMLauncherService"]["UpdatedZipFilename"].StringValue;
                                        }
                                        
                                        if (Verbosity() > 0)
                                        {
                                            AppLog.WriteEntry($"Moving temp file {tempFilename} to {destinationFilename}");
                                        }
                                        // this one's tricky.  I can't overwrite my own files while I'm running!
                                        try
                                        {
                                            if(File.Exists(destinationFilename))
                                            {
                                                AppLog.WriteEntry($"{destinationFilename} already exists.  Deleting to make room for new version.");
                                                File.Delete(destinationFilename);
                                            }
                                            File.Move(tempFilename, destinationFilename);
                                            System.Security.AccessControl.FileSecurity fSecurity = File.GetAccessControl(destinationFilename);
                                            fSecurity.AddAccessRule(new System.Security.AccessControl.FileSystemAccessRule("Everyone", System.Security.AccessControl.FileSystemRights.FullControl, System.Security.AccessControl.AccessControlType.Allow));
                                            File.SetAccessControl(destinationFilename, fSecurity);
                                            AppLog.WriteEntry($"New version of 3DTM Launcher Service saved to {destinationFilename}");
                                            if (!Config["3DTMLauncherService"].Contains("UpdaterExecutable") || !File.Exists(Config["3DTMLauncherService"]["UpdaterExecutable"].StringValue))
                                            {
                                                PublishInstallationResult((SOFTWARE)results[1], true, $"New version saved to {destinationFilename}.  Updater service [3DTMLauncherService][UpdaterExecutable] is not set or does not exist.  Please extract the zip and restart the service manually.");
                                            }
                                            else
                                            {
                                                PublishInstallationResult((SOFTWARE)results[1], true, $"New version saved to {destinationFilename}.  Running the updater to update.  This service will be stopped immediately.");
                                                ProcessStartInfo startInfo = new ProcessStartInfo(Config["3DTMLauncherService"]["UpdaterExecutable"].StringValue);
                                                System.Diagnostics.Process.Start(startInfo);
                                            }
                                        }
                                        catch (Exception ex)
                                        {
                                            AppLog.WriteEntry($"Exception trying to move the file: {ex.Message}", EventLogEntryType.Error);
                                            PublishInstallationResult((SOFTWARE)results[1], false, $"Couldn't move the 3DTM zip file: {ex.Message}");
                                            return;
                                        }
                                    }

                                    previousZipFile = destinationDir + ".old.zip";

                                    if (destinationDir != "")
                                    {
                                        try
                                        {
                                            if (File.Exists(previousZipFile))
                                            {
                                                if (Verbosity() > 0)
                                                {
                                                    AppLog.WriteEntry($"{previousZipFile} already exists.  Removing to make room for new backup.");
                                                }
                                                File.Delete(previousZipFile);
                                            }
                                            // zip the old verison
                                            System.IO.Compression.ZipFile.CreateFromDirectory(destinationDir, previousZipFile);
                                            // delete the old folder
                                            var dir = new DirectoryInfo(destinationDir);
                                            dir.Delete(true);
                                            // unzip the new version
                                            System.IO.Compression.ZipFile.ExtractToDirectory(tempFilename, destinationDir);
                                            if (Verbosity() > 0)
                                            {
                                                AppLog.WriteEntry($"{previousZipFile} created from {destinationDir}.  {tempFilename} extracted to {destinationDir}");
                                            }
                                            PublishInstallationResult((SOFTWARE)results[1], true, $"Successfully installed to {destinationDir}");
                                        }
                                        catch (Exception ex)
                                        {
                                            AppLog.WriteEntry($"Error extracting new software version: {ex.Message}", EventLogEntryType.Error);
                                            PublishInstallationResult((SOFTWARE)results[1], false, $"Error extracting new software version: {ex.Message}");
                                            return;
                                        }
                                        if (ApplicationToRun == SOFTWARE.FUSION && (SOFTWARE)results[1] == SOFTWARE.KINECT_COMMUNICATOR)
                                        {
                                            // start the KNCS
                                            ServiceController sc = new ServiceController("3DTMKinectNanoCommunicatorService");
                                            try
                                            {
                                                sc.Start();
                                            }
                                            catch (Exception ex)
                                            {
                                                AppLog.WriteEntry($"Couldn't start the KNCS!", EventLogEntryType.Error);
                                                PublishInstallationResult((SOFTWARE)results[1], false, $"Installation complete, but couldn't restart KNCS: {ex.Message}");
                                            }
                                        }

                                    }
                                }
                                break;
                            }
                        case CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT:
                            {
                                if(Verbosity() > 0)
                                {
                                    AppLog.WriteEntry("Received calibration software result");
                                }

                                if(results.Length > 1 && results[1] > 0)
                                {
                                    string json_str;
                                    CameraCalibrationHelper.BytePacketToJSONString(results, out json_str);
                                    CameraCalibrationHelper.SaveTo3DTMCalibrationFormat(json_str, CalibrationFilePath);
                                } 
                                else
                                {
                                    if (Verbosity() > 0)
                                    {
                                        AppLog.WriteEntry("Error! Calibration software result came back as an invalid file! Answering back to Control Panel with received new data to prevent it from hanging.");
                                    }
                                }

                                byte[] data = new byte[1];
                                data[0] = (byte)CPC_STATUS.RECEIVED_NEW_DATA;
                                statusPublisherSocket.SendFrame(data);

                                break;
                            }
                        default:
                            if(Verbosity() > 0)
                            {
                                AppLog.WriteEntry($"Received an unhandled control panel event ({EventType})", EventLogEntryType.Information);
                            }
                            // ignore other events
                            break;
                    }
                }
            }
        }

        private bool DoesNewConfigSectionDiffer(Section currentConfigSection, Section newConfigSection)
        {
            foreach (var currentConfigItem in currentConfigSection)
            {
                AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}]");
                if (!newConfigSection.Contains(currentConfigItem.Name))
                {
                    if (Verbosity() > 0)
                    {
                        AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}] does not exist in new config.  Restart is required.");
                    }
                    return true;
                }
                if (!currentConfigItem.IsArray)
                {
                    if (newConfigSection[currentConfigItem.Name].IsArray)
                    {
                        if(Verbosity() > 0)
                        {
                            AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}] is now an array.  Restart is required.");
                        }
                        return true;
                    }                        
                    if (currentConfigItem.StringValue != newConfigSection[currentConfigItem.Name].StringValue)
                    {
                        if (Verbosity() > 0)
                        {
                            AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}] has changed.  Restart is required.");
                        }
                        return true;
                    }
                }
                else
                {
                    if(!newConfigSection[currentConfigItem.Name].IsArray)
                    {
                        if(Verbosity() > 0)
                        {
                            AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}] was an array, but no longer is.  Restart is required.");
                        }
                        return true;
                    }
                    if (currentConfigItem.ArraySize != newConfigSection[currentConfigItem.Name].ArraySize)
                    {
                        if (Verbosity() > 0)
                        {
                            AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}] (array) is different length.  Restart required.");
                        }
                        return true;
                    }
                    for(int i = 0; i < currentConfigItem.ArraySize; i++)
                    {
                        AppLog.WriteEntry($"[{currentConfigSection.Name}][{currentConfigItem.Name}][{i}]");

                        if (currentConfigItem.StringValueArray[i] != newConfigSection[currentConfigItem.Name].StringValueArray[i])
                        {
                            if(Verbosity() > 0)
                            {
                                AppLog.WriteEntry($"[{currentConfigSection.Name}[{currentConfigItem.Name}][{i}] does not match.  Restart required.");
                            }
                            return true;
                        }
                    }
                }
            }
            foreach (var newConfigItem in newConfigSection)
            {
                if(!currentConfigSection.Contains(newConfigItem.Name))
                {
                    if(Verbosity() > 0)
                    {
                        AppLog.WriteEntry($"[{currentConfigSection.Name}] does not contain [{newConfigItem.Name}].  Restart is required.");
                    }
                    return true;
                }
            }
            return false;
        }

        private void PrintState()
        {
            var states = Enum.GetNames(typeof(SOFTWARE_STATE));
            if (Environment.UserInteractive)
            {
                Console.WriteLine($"SystemState: [{ApplicationToRun}][{states[Convert.ToInt32(GetState(ApplicationToRun))]}]");
            }
            if(Verbosity() > 1)
                AppLog.WriteEntry($"SystemState: [{ApplicationToRun}][{states[Convert.ToInt32(GetState(ApplicationToRun))]}]", EventLogEntryType.Information);
        }

        private void PublishApplicationState()
        {
            string serialNumber = ApplicationToRun.ToString();
            int size = 1 + sizeof(int) + serialNumber.Length + (GetStates().Length * sizeof(int));
            byte[] data = new byte[size];
            data[0] = (byte)CPC_STATUS.RUNNING;
            int pos = 1;
            Buffer.BlockCopy(BitConverter.GetBytes(serialNumber.Length), 0, data, pos, sizeof(int));
            pos += sizeof(int);
            Buffer.BlockCopy(Encoding.Default.GetBytes(serialNumber), 0, data, pos, serialNumber.Length);
            pos += serialNumber.Length;
            Buffer.BlockCopy(GetStatesAsIntArray(), 0, data, pos, GetStates().Length * sizeof(int));

            statusPublisherSocket.SendFrame(data);
            PrintState();
        }
        private void PublishLogFile(FileInfo fileInfo)
        {
            byte[] fileName = Encoding.Default.GetBytes(fileInfo.Name);
            byte[] fileData = File.ReadAllBytes(fileInfo.FullName);
            byte[] packet = new byte[1 + sizeof(int) + fileName.Length + sizeof(int) + fileData.Length];
            packet[0] = (byte)CPC_STATUS.LOG_DATA;
            int pos = 1;
            byte[] lengthBytes = BitConverter.GetBytes(fileName.Length);
            Buffer.BlockCopy(lengthBytes, 0, packet, pos, lengthBytes.Length);
            pos += lengthBytes.Length;
            Buffer.BlockCopy(fileName, 0, packet, pos, fileName.Length);
            pos += fileName.Length;
            lengthBytes = BitConverter.GetBytes(fileData.Length);
            Buffer.BlockCopy(lengthBytes, 0, packet, pos, lengthBytes.Length);
            pos += lengthBytes.Length;
            Buffer.BlockCopy(fileData, 0, packet, pos, fileData.Length);

            statusPublisherSocket.SendFrame(packet);
        }

        private void PublishInstallationResult(SOFTWARE software, bool success, string errorMessage)
        {
            int index = 0;
            byte[] packet = new byte[sizeof(byte) + sizeof(byte) + sizeof(bool) + sizeof(int) + errorMessage.Length];
            packet[index++] = (byte)CPC_STATUS.SOFTWARE_INSTALLATION_RESULT;
            packet[index++] = (byte)software;
            byte[] successBytes = BitConverter.GetBytes(success);
            successBytes.CopyTo(packet, index);
            index += successBytes.Length;
            byte[] lengthBytes = BitConverter.GetBytes(errorMessage.Length);
            lengthBytes.CopyTo(packet, index);
            index += lengthBytes.Length;
            byte[] message = Encoding.Default.GetBytes(errorMessage);
            message.CopyTo(packet, index);

            statusPublisherSocket.SendFrame(packet);
        }
        protected override void OnStop()
        {
            controlPanelEventPoller.Stop();
            Running = false;
            // Some threads have a 500ms timeout on their poll function, so wait that long to make sure they all see the running flag before trying to close/join
            System.Threading.Thread.Sleep(500); 
            AppLog.WriteEntry("OnStop called.  Service terminating.", EventLogEntryType.Warning);

            eventSubscriberSocket.Close();
            statusPublisherSocket.Close();

            heartbeatThread.Join();
            applicationMonitorThread.Join();
            applicationLauncherThread.Join();
        }

        static void Main(string[] args)

        { 
            PeabodyLauncherService service = new PeabodyLauncherService();

            if (Environment.UserInteractive)
            {
                service.OnStart(args);
                Console.WriteLine("Press Enter to stop program");
                Console.Read();
                service.OnStop();
            }
            else
            {
                ServiceBase.Run(service);
            }

        }
    }
}
