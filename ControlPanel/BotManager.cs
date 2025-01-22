using ControlPanel;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading;
using System.Text;
using PeabodyNetworkingLibrary;
using System.Xml;
using System.Xml.Serialization;
using System.Net;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Collections.Concurrent;
using NetMQ;
using NLog;
using Newtonsoft.Json;

namespace ControlPanel
{
    public delegate void BotManagerCallback();
    public class BotManager
    {
        public const long cMaxCleanupTime = 8; //in seconds max cleanupTime allocated
        const int cUpdateOffset = 1; //first byte of status packet is the status type, so skip it

        //threadsafe SINGLETON pattern
        protected static BotManager instance;
        protected static ControlPanel controlPanel;
        private Thread resetDaemonStatusThread;
        private Thread broadcastThread;
        private Thread cleanupCheckThread;
        private bool runDaemonStatusThread = true;
        private bool runBroadcastThread = true;

        public bool CalibrationComplete { get; private set; } = false;
        public List<VersionContent> versionContentList => instance.versionMap.Values.ToList();
        public static void Init(ControlPanel controlPanel)
        {
            if (BotManager.instance == null)
            {
                BotManager.instance = new BotManager();
            }
            BotManager.controlPanel = controlPanel;
        }
        public static async Task WaitForSecondsAsync(double seconds)
        {
            await Task.Delay((int)(seconds * 1000));
        }
        public static BotManager Instance
        {
            get
            {
                return instance;
            }
        }
        // create a destructor
        ~BotManager()
        {
            runDaemonStatusThread = false;
            runBroadcastThread = false;
            OutputHelper.OutputLog("BotManager destructor called", OutputHelper.Verbosity.Info);

            OnDestroy();
        }
        // Start is called before the first frame update
        public void Start()
        {
            LoadVersionInformationFromDisk();
            instance.InitializePublisherAndStart();
            instance.CreateDaemonMonitorsBasedOnConfig();
            CheckBotManagerReady(SOFTWARE.CAPTURE);
            instance.CreateStatusBotsBasedOnConfig();
            instance.CreateCalibrationSoftwareStatusBot();
            mainThreadCallbacks = new ConcurrentQueue<BotManagerCallback>();
            Tuple<bool, DAEMON_GROUP_STATE> botReadyStatus = CheckBotManagerReady(SOFTWARE.CAPTURE);


        }

        // Update is called once per frame
        void Update()
        {
            if (mainThreadCallbacks.Count > 0)
            {
                BotManagerCallback callback;
                while (mainThreadCallbacks.TryDequeue(out callback))
                {
                    callback.Invoke();
                }

            }
            // Update the UI events
            //foreach (VersionContent vc in versionMap.Values)
            //{
            //    vc.UpdateUIElements();
            //}
        }

        //Implementation

        public EventBot DaemonPublisher;
        private SOFTWARE lastStartType = SOFTWARE.CAPTURE; // START with this so we force cleanup on launch

        public bool IsCleaningUp { get; private set; } = false;
        public BotManagerCallback onCleanedup;

        //daemon related
        public StatusBot[] depthGenDaemonBots { get; private set; }
        public StatusBot fusionDaemonBot;
        public StatusBot renderDaemonBot;
        public bool depthDaemonBotsCreated = false;
        public bool allDaemonBotsReadyToBeLaunched_Calib;
        public bool allDaemonBotsReadyToBeLaunched_Capture;
        public bool allDaemonBotsReadyToBeLaunched_BGCap;
        public bool AllDaemonBotsReadyToBeLaunched_Calib
        {
            get
            {
                return allDaemonBotsReadyToBeLaunched_Calib;
            }
            set
            {
                allDaemonBotsReadyToBeLaunched_Calib = value;
            }
        }
        public bool AllDaemonBotsReadyToBeLaunched_Capture
        {
            get
            {
                return allDaemonBotsReadyToBeLaunched_Capture;
            }
            set
            {
                allDaemonBotsReadyToBeLaunched_Capture = value;
            }
        }
        public bool AllDaemonBotsReadyToBeLaunched_BGCap
        {
            get
            {
                return allDaemonBotsReadyToBeLaunched_BGCap;
            }
            set
            {
                allDaemonBotsReadyToBeLaunched_BGCap = value;
            }
        }
        public long lastDaemonCheckTime = 0;
        const long cDaemonStatusCheckInterval = 5; // in seconds, how often we reissue a status check on daemons for app status
                                                   //daemon status checks suceess and failure callbacks
                                                   // this facilitates many UnityEngine related calls that need to be executed on the main thread. so we queue these up to be run in Update()
        public ConcurrentQueue<BotManagerCallback> mainThreadCallbacks;

        public StatusBot[] depthGenStatusBots { get; private set; }
        public StatusBot fusionStatusBot;
        public StatusBot renderStatusBot;
        public StatusBot calibrationSoftwareStatusBot;

        // We're keeping the versionMap struct instead of just accessing the data from inside the bots that are part
        // of the BotManager class because not all bots are created at any point in time, so if the user wanted to update
        // Fusion, they would have to have fusion RUNNING for that bot to exist so we could get the data from it.
        // That's not convenient, so instead we'll keep this map of basic version data so we can 
        // save the version info to disk, load it when control panel loads, and use that to decide if there are
        // updates.
        public class VersionContent
        {
            public string unitName { get; set; }  //only used in saving and loading XML
            public PeabodyNetworkingLibrary.VersionData versionData { get; set; }

            private string currentButtonText = "";
            private string newButtonText = "";
            private bool buttonEnable = false;

            private string currentVersionText = "";
            private string newVersionText = "";

            // public string UpdateButtonText { get { return currentButtonText; } set { newButtonText = value; } }
            public string? UpdateButtonText { get; set; }
            public void EnableUpdateButton(bool enable)
            {
                buttonEnable = enable;
            }

            //public string VersionStringText { get { return currentVersionText; } set { newVersionText = value; } }
            public string? Version => versionData.Description;

            public string? OnlineVersion { get; set; }

        }
        public ConcurrentDictionary<string, VersionContent> versionMap = new ConcurrentDictionary<string, VersionContent>();

        //public void CreateVersionTextObject(string unitName, PeabodyNetworkingLibrary.VersionData versionData, ComponentStatusContainer container = null)
        //{
        //    Text[] texts;
        //    GameObject versionGameObject;
        //    if (versionMap.ContainsKey(unitName) && versionMap[unitName].gameObject != null)
        //    {
        //        // already created, probably ready from disk
        //        versionGameObject = versionMap[unitName].gameObject;
        //    }
        //    else
        //    {
        //        // Create the version info in the version list
        //        versionGameObject = Instantiate(VersionGameObject);
        //    }
        //    texts = versionGameObject.GetComponentsInChildren<Text>();
        //    texts[0].text = $"{unitName}: {versionData.Description}";
        //    if (container != null)
        //    {
        //        OutputHelper.OutputLog($"CVTO called {unitName}.  Container is valid.  Setting texts.", OutputHelper.Verbosity.Trace);
        //        container.versionText = texts[0];
        //        container.cloudText = texts[1];
        //    }
        //    else
        //    {
        //        OutputHelper.OutputLog($"CVTO called {unitName}.  Container is NULL.", OutputHelper.Verbosity.Trace);
        //    }
        //    Button updateButton = versionGameObject.GetComponentInChildren<Button>();
        //    updateButton.onClick.AddListener(delegate { OnButton_UpdateComponent(versionMap[unitName]); });
        //    updateButton.enabled = false;
        //    Text buttonText = updateButton.GetComponentInChildren<Text>();
        //    buttonText.text = " No Update Available ";
        //    versionGameObject.transform.SetParent(VersionListCanvas.gameObject.transform);
        //    // don't know if I need any of this, just trying to make the text visible
        //    versionGameObject.transform.Translate(new Vector3(0, 0, 0));
        //    foreach (Text t in texts)
        //    {
        //        t.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        //        t.fontStyle = FontStyle.Normal;
        //    }

        //    if (versionMap.ContainsKey(unitName))
        //    {
        //        // game object is null, attach
        //        VersionContent thisVC = versionMap[unitName];
        //        thisVC.gameObject = versionGameObject;
        //        versionMap[unitName] = thisVC;
        //    }
        //    else
        //    {
        //        // creating a new object
        //        VersionContent vc = new VersionContent();
        //        vc.gameObject = versionGameObject;
        //        vc.unitName = unitName;
        //        vc.versionData = versionData;
        //        versionMap.TryAdd(unitName, vc);
        //    }
        //    // Sort alphabetically
        //    var sortedMap = versionMap.OrderBy(x => x.Value.unitName);
        //    int siblingIndex = 0;
        //    foreach (var item in sortedMap)
        //    {
        //        item.Value.gameObject.transform.SetSiblingIndex(siblingIndex++);
        //    }
        //}

        public void OnButton_UpdateComponent(VersionContent versionContent)
        {
            versionContent.EnableUpdateButton(false);
            // Create a new client so that we're not calling old event handlers
            WebClient webDownloaderClient = new WebClient();
            // strip the program name from the front of the unit name
            int pos = versionContent.unitName.IndexOf("_");
            pos = pos > 0 ? pos : versionContent.unitName.Length;
            var programName = versionContent.unitName.Substring(0, pos);
            OutputHelper.OutputLog($"Downloading new version of {programName} for {versionContent.unitName}", OutputHelper.Verbosity.Info);
            // Get the latest version from the web
            var url = SettingsManager.Instance.GetValueWithDefault<string>("Updates", programName + "SoftwareUrl", "");
            if (url == "")
            {
                OutputHelper.OutputLog($"Could not download an update for {programName} because the entry [Updates][{programName + "SoftwareUrl"}] does not exist", OutputHelper.Verbosity.Error);
                versionContent.EnableUpdateButton(false);
                versionContent.UpdateButtonText = $" Error ";
                return;
            }
            var softwareFile = Path.GetTempFileName();
            OutputHelper.OutputLog($"Download software from {url} for {versionContent.unitName} and save to {softwareFile}", OutputHelper.Verbosity.Debug);
            try
            {
                webDownloaderClient.DownloadProgressChanged += (sender, eventArgs) =>
                {
                    Client_DownloadProgressChanged(eventArgs, versionContent);
                };

                webDownloaderClient.DownloadFileCompleted += (sender, eventArgs) =>
                {
                    Client_DownloadFileCompleted(eventArgs, versionContent, softwareFile);
                };

                webDownloaderClient.DownloadFileAsync(new Uri(url), softwareFile);
            }
            catch (System.Net.WebException e)
            {
                OutputHelper.OutputLog($"Couldn't connect to the web.  Check your internet connection! Error: {e.Message}", OutputHelper.Verbosity.Error);
                return;
            }
        }
        private void Client_DownloadProgressChanged(DownloadProgressChangedEventArgs e, VersionContent vc)
        {
            vc.EnableUpdateButton(false);
            vc.UpdateButtonText = $" {e.ProgressPercentage}% ";
        }
        private void Client_DownloadFileCompleted(AsyncCompletedEventArgs e, VersionContent vc, string filename)
        {
            vc.EnableUpdateButton(false);
            vc.UpdateButtonText = $" Installing... ";

            // Read the file into memory
            byte[] binary = File.ReadAllBytes(filename);
            byte[] lengthInBytes = BitConverter.GetBytes(binary.Length);
            byte[] dataPacket = new byte[sizeof(byte) + sizeof(int) + binary.Length];

            dataPacket[0] = (byte)GetSoftwareTypeFromVersionContent(vc);
            Buffer.BlockCopy(lengthInBytes, 0, dataPacket, 1, sizeof(int));
            binary.CopyTo(dataPacket, sizeof(byte) + sizeof(int));
            // Transmit to the component
            OutputHelper.OutputLog($"Read {binary.Length} bytes from {filename}.  Sending a packet of size {dataPacket.Length} + 1", OutputHelper.Verbosity.Trace);
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.NEW_BINARY_TRANSFER, dataPacket);
        }

        private PeabodyNetworkingLibrary.SOFTWARE GetSoftwareTypeFromVersionContent(VersionContent vc)
        {
            int pos = vc.unitName.IndexOf("_");
            pos = pos > 0 ? pos : vc.unitName.Length;
            switch (vc.unitName.Substring(0, pos))
            {
                case "3DTMLauncherService":
                    return SOFTWARE.WINDOWS_SERVICE;
                case "AKLauncherDaemon":
                    return SOFTWARE.LINUX_DAEMON;
                case "AzureKinectNanoToFusion":
                    return SOFTWARE.CAPTURE;
                case "KinectNanoCommunicatorService":
                    return SOFTWARE.KINECT_COMMUNICATOR;
                case "Fusion":
                    return SOFTWARE.FUSION;
                case "Render":
                    return SOFTWARE.RENDER;
                default:
                    OutputHelper.OutputLog($"unit {vc.unitName} is an unknown type, can't get software version!", OutputHelper.Verbosity.Error);
                    return 0;
            }
        }
        #region Daemon
        protected SOFTWARE_STATE[] ProcessDaemonStateUpdate(byte[] update)
        {
            int updateOffset = cUpdateOffset;
            int minPacketLength = 1 + sizeof(int) + sizeof(int);
            SOFTWARE_STATE[] states = new SOFTWARE_STATE[(int)PeabodyNetworkingLibrary.SOFTWARE.COUNT];

            //packet format: [CPC_STATUS::RUNNING][(int)serialLen][(char[serialLen])serialNumber][(int)systemState(unused)][(int)calibrationState][(int)captureState][(int)BGState][(int)fusionState][(int)render state] (see order of SOFTWARE enum)
            OutputHelper.OutputLog($"[ProcessDaemonStateUpdate] UpdateContent ({update.Length} bytes): [{BitConverter.ToString(update)}]", OutputHelper.Verbosity.Trace);

            if (update.Length >= minPacketLength)
            {
                int serialLength = System.BitConverter.ToInt32(update, updateOffset);
                OutputHelper.OutputLog($"Serial length is {serialLength}", OutputHelper.Verbosity.Trace);
                updateOffset += sizeof(int);
                string serial = System.BitConverter.ToString(update, updateOffset, serialLength); // We don't currently use this, but it's passed so forwards of this packet know where it came from
                OutputHelper.OutputLog($"Serial Number: {serial}", OutputHelper.Verbosity.Trace);
                updateOffset += serialLength;
                for (int i = 0; i < (int)SOFTWARE.COUNT; i++)
                {
                    states[i] = (SOFTWARE_STATE)System.BitConverter.ToInt32(update, updateOffset);
                    updateOffset += sizeof(int);
                    OutputHelper.OutputLog($"Setting Software[{Enum.GetNames(typeof(SOFTWARE))[i]}] to state {states[i]}", OutputHelper.Verbosity.Trace);
                }
            }
            return states;
        }
        public bool CreateDaemonMonitorsBasedOnConfig()
        {
            depthDaemonBotsCreated = false;
            int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
            int portBase = SettingsManager.Instance.GetValueWithDefault("Ports", "DepthPodDaemonPortBase", 14530, true);
            depthGenDaemonBots = new StatusBot[depthGenNum];
            OutputHelper.OutputLog("Started creating DaemonBots", OutputHelper.Verbosity.Trace);
            for (int i = 0; i < depthGenNum; ++i)
            {
                int podNumber = i + 1;
                string hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "192.168.102.250", true);
                string hostNameOrIP_PN = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250", true);
                bool UsingKNCS = !hostNameOrIP.Equals(hostNameOrIP_PN);
                string dgName = "AKLauncherDaemon_Pod" + (i + 1).ToString();  //pod names are 1-based, not 0-based
                string tcpPort = SettingsManager.Instance.GetValueWithDefault("Ports", "NanoDaemonStatusPort", "14501");
                if (UsingKNCS)
                {
                    tcpPort = $"{portBase + podNumber}";
                }
                else
                {
                    hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "DepthPodIPBase", "192.168.101.", true) + podNumber.ToString();
                }

                //bot setup start
                depthGenDaemonBots[i] = new StatusBot(hostNameOrIP, dgName, controlPanel, tcpPort);
                depthGenDaemonBots[i].SetDepthGenID(i);
                depthGenDaemonBots[i].DepthGenMachineID = i;
                DefaultStatusForwarderSetup(depthGenDaemonBots[i]);
                depthGenDaemonBots[i].DefaultStatusUpdateSetup(false);
                depthGenDaemonBots[i].RegisterUpdateFunction(CPC_STATUS.FPS, depthGenDaemonBots[i].daemonStatusBotFPSFunction);
                SetupSocket(depthGenDaemonBots[i], hostNameOrIP);

                depthGenDaemonBots[i].componentStatus = new ComponentStatus
                {
                    ID = i,
                    Name = dgName,
                    FPS = 0,
                    FPS_max = 0.0,
                    FPS_min = 100.0,
                    FrameNum = 0,
                    ErrState = CPC_ERROR.NONE,
                    Status = Status.Stopped,
                    IP = hostNameOrIP,
                    Subscribe = true,
                    //   CanLaunch = canLaunch // NOT used in this setup
                };

                StatusBot currBot = depthGenDaemonBots[i];
                int currBotID = i;
                SetVersionDataFromMap(currBot);

                //currBot.componentStatusContainer.onTimedOutCallback_permanent += () =>
                //{
                //    //timed out, so no longer running
                //    currBot.UpdateStateAndUI(SOFTWARE.CAPTURE, SOFTWARE_STATE.TIMEDOUT);
                //    currBot.UpdateStateAndUI(SOFTWARE.CALIBRATION, SOFTWARE_STATE.TIMEDOUT);
                //    currBot.UpdateStateAndUI(SOFTWARE.LINUX_DAEMON, SOFTWARE_STATE.TIMEDOUT);
                //    OutputHelper.OutputLog($"[{DateTime.UtcNow}] Daemon {currBot.UnitName} timed out. calling callback!", OutputHelper.Verbosity.Warning);

                //    //status updated
                //    RespondToDaemonUpdate();
                //};

                currBot.RegisterUpdateFunction(CPC_STATUS.RUNNING, (byte[] update) =>
                {
                    SOFTWARE_STATE[] states = ProcessDaemonStateUpdate(update);
                    //If I got a packet, I know the daemon is alive, so make sure it's state stays RUNNING
                    states[(int)SOFTWARE.LINUX_DAEMON] = SOFTWARE_STATE.RUNNING;
                    //ONLY update and make callbacks if the state changed (so we're not doing redudant work)               
                    if (currBot.UpdateSoftwareStates(states))
                    {
                        OutputHelper.OutputLog($"[ {DateTime.UtcNow} ] {currBot.UnitName} Daemon UpdateContent: [{states[(int)SOFTWARE.CAPTURE]}][{states[(int)SOFTWARE.CALIBRATION]}]", OutputHelper.Verbosity.Trace);
                        //status updated
                        RespondToDaemonUpdate();
                        //There's a condition where we might start up the software, the daemons start up, and find that there's already
                        //a copy of AKNF running.  In this case, they'll update the software state here, but there won't be a status bot connected to the pod
                        //so start up the status bot here for that pod
                        if (states[(int)SOFTWARE.CAPTURE] == SOFTWARE_STATE.RUNNING)
                        {
                            if (depthGenStatusBots[currBotID] != null && depthGenStatusBots[currBotID].componentStatus.Status != Status.Running)
                            {
                                OutputHelper.OutputLog($"Daemon {currBot.UnitName} says the application running, but status bot says it is not running.  Reconnecting status bot.", OutputHelper.Verbosity.Debug);
                                depthGenStatusBots[currBotID].Reconnect();
                            }
                        }
                    }
                    //packet was received, so can update timeout counter
                    currBot.UpdateTimeLastHBReceived();

                });

                currBot.RegisterUpdateFunction(CPC_STATUS.KINECT_FACTORY_CALIBRATION_DATA,
                   delegate (byte[] update)
                   {
                       //[(int)fileLength][(char[fileLength])fileData]...  
                       // Order is color, depth, extrinsics, imusample
                       if (update.Length <= sizeof(int))
                       {
                           OutputHelper.OutputLog("ERROR: not enough data in KINECT_FACTORY_CALIBRATION_DATA", OutputHelper.Verbosity.Error);
                           return;
                       }

                       string folderPath = Path.Combine(SettingsManager.Instance.GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "", true), "K4AFactoryCalibs");
                       folderPath = Path.Combine(folderPath, SettingsManager.Instance.GetValueWithDefault("Calibration", "CameraDirectoryPrefix", "cam") + currBotID);
                       string colorFilePath = Path.Combine(folderPath, "colorConfig00.json");
                       string depthFilePath = Path.Combine(folderPath, "depthConfig00.json");
                       string extrinsicsFilePath = Path.Combine(folderPath, "extrinsics00.json");
                       string imuFilePath = Path.Combine(folderPath, "imuSample.json");

                       System.IO.Directory.CreateDirectory(folderPath);

                       //write color
                       int sizeOffset = cUpdateOffset;
                       int colorFileLength = System.BitConverter.ToInt32(update, sizeOffset);
                       sizeOffset += sizeof(int) + colorFileLength;
                       if (sizeOffset >= update.Length)
                       {
                           OutputHelper.OutputLog($"ERROR.  Not enough data for color ({colorFileLength}) + depth factory data (wanted {sizeOffset})", OutputHelper.Verbosity.Error);
                           return;
                       }
                       int depthFileLength = System.BitConverter.ToInt32(update, sizeOffset);
                       sizeOffset += sizeof(int) + depthFileLength;
                       if (sizeOffset >= update.Length)
                       {
                           OutputHelper.OutputLog($"ERROR.  Not enough data for color ({colorFileLength}), depth  ({depthFileLength}), and extrinsic factory data (wanted {sizeOffset})", OutputHelper.Verbosity.Error);
                           return;
                       }
                       int extrinsicFileLength = System.BitConverter.ToInt32(update, sizeOffset);
                       sizeOffset += sizeof(int) + extrinsicFileLength;
                       int imuFileLength = 0;
                       if (sizeOffset >= update.Length)
                       {
                           OutputHelper.OutputLog($"WARNING.  Not enough data for color ({colorFileLength}), depth  ({depthFileLength}), extrinsic ({extrinsicFileLength}) and IMU sample (wanted {sizeOffset}). Maybe no IMU data has been captured yet?", OutputHelper.Verbosity.Warning);
                       }
                       else
                       {
                           imuFileLength = System.BitConverter.ToInt32(update, sizeOffset);
                       }
                       int expectedPacketSize = sizeof(int) * 3 + colorFileLength + depthFileLength + extrinsicFileLength;
                       expectedPacketSize = (imuFileLength > 0) ? expectedPacketSize + imuFileLength + sizeof(int) : expectedPacketSize;
                       if (update.Length < expectedPacketSize)
                       {
                           OutputHelper.OutputLog($"ERROR.  Calculated color ({colorFileLength}) + depth ({depthFileLength}) + extrinsic ({extrinsicFileLength}) file lengths are > packet size ({update.Length}).  Aborting.", OutputHelper.Verbosity.Error);
                           return;
                       }
                       OutputHelper.OutputLog($"Received calibration data.  Color [{colorFileLength}] Depth [{depthFileLength}] Extrinsics [{extrinsicFileLength}] IMU [{imuFileLength}]", OutputHelper.Verbosity.Debug);
                       int colorDataStartIndex = cUpdateOffset + sizeof(int);
                       int depthDataStartIndex = colorDataStartIndex + colorFileLength + sizeof(int);
                       int extrinsicDataStartIndex = depthDataStartIndex + depthFileLength + sizeof(int);
                       int imuDataStartIndex = extrinsicDataStartIndex + extrinsicFileLength + sizeof(int);
                       if (colorFileLength + colorDataStartIndex <= update.Length)
                       {
                           OutputHelper.OutputLog($"Color file is {colorFileLength} bytes.  Saving to ${colorFilePath}", OutputHelper.Verbosity.Debug);
                           using (BinaryWriter bw = new BinaryWriter(File.Open(colorFilePath, FileMode.Create)))
                               bw.Write(update, colorDataStartIndex, colorFileLength);
                       }
                       if (depthFileLength + depthDataStartIndex <= update.Length)
                       {
                           OutputHelper.OutputLog($"Depth file is {depthFileLength} bytes.  Saving to ${depthFilePath}", OutputHelper.Verbosity.Debug);
                           using (BinaryWriter bw = new BinaryWriter(File.Open(depthFilePath, FileMode.Create)))
                               bw.Write(update, depthDataStartIndex, depthFileLength);
                       }
                       if (extrinsicFileLength + extrinsicDataStartIndex <= update.Length)
                       {
                           OutputHelper.OutputLog($"Extrinsic file is {extrinsicFileLength} bytes.  Saving to ${extrinsicsFilePath}", OutputHelper.Verbosity.Debug);
                           using (BinaryWriter bw = new BinaryWriter(File.Open(extrinsicsFilePath, FileMode.Create)))
                               bw.Write(update, extrinsicDataStartIndex, extrinsicFileLength);
                       }
                       if (imuFileLength > 0 && (imuFileLength + imuDataStartIndex) <= update.Length)
                       {
                           OutputHelper.OutputLog($"IMU file is {imuFileLength} bytes.  Saving to ${imuFilePath}", OutputHelper.Verbosity.Debug);
                           using (BinaryWriter bw = new BinaryWriter(File.Open(imuFilePath, FileMode.Create)))
                               bw.Write(update, imuDataStartIndex, imuFileLength);
                       }
                       else
                       {
                           OutputHelper.OutputLog($"IMU file is {imuFileLength} bytes.  That would make it ({imuFileLength + imuDataStartIndex}) >= packet size ({update.Length})", OutputHelper.Verbosity.Debug);
                       }

                   });

                currBot.RegisterUpdateFunction(CPC_STATUS.LOG_DATA, (byte[] update) =>
                {
                    SaveLogData(currBot, update);
                });
                OutputHelper.OutputLog($"{currBot.UnitName} registering new update function for CPC_STATUS.BUILD_VERSION");
                currBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
                {
                    UpdateVersionData(currBot, update);
                });

                currBot.RegisterUpdateFunction(CPC_STATUS.SOFTWARE_INSTALLATION_RESULT, (byte[] update) =>
                {
                    HandleInstallationResult(currBot, update);
                });
                currBot.RegisterUpdateFunction(CPC_STATUS.IS_ALIVE, (byte[] update) =>
                {
                    OutputHelper.OutputLog($"Daemon {currBot.UnitName} received an is alive update", OutputHelper.Verbosity.Debug);
                    currBot.UpdateSoftwareState(SOFTWARE.LINUX_DAEMON, SOFTWARE_STATE.RUNNING);
                });
                //CreateVersionTextObject(currBot.UnitName, currBot.GetVersionData(), currBot.componentStatusContainer);
                currBot.Start();
            }

            //create fusion/render daemon (3dtm service) monitors
            string daemonPort = SettingsManager.Instance.GetValueWithDefault("Ports/Fusion", "DaemonPort", SettingsManager.Instance.GetValueWithDefault("Ports", "DaemonPort", "14511", true));

            fusionDaemonBot = new StatusBot(null, "3DTMLauncherService_Fusion", controlPanel, daemonPort);
            DefaultStatusForwarderSetup(fusionDaemonBot);
            SetupSocket(fusionDaemonBot, SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true));
            fusionDaemonBot.componentStatus = new ComponentStatus
            {
                ID = 20,
                Name = "3DTMLauncherService_Fusion",
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                Status = Status.Stopped,
                IP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true),
                Subscribe = true,
                NewDataReceived = false,
            };
            SetVersionDataFromMap(fusionDaemonBot);

            //CreateVersionTextObject(fusionDaemonBot.UnitName, fusionDaemonBot.GetVersionData(), fusionDaemonBot.componentStatusContainer);


            //Renderer
            daemonPort = SettingsManager.Instance.GetValueWithDefault("Ports/Renderer", "DaemonPort", SettingsManager.Instance.GetValueWithDefault("Ports", "DaemonPort", "14511", true));
            renderDaemonBot = new StatusBot(null, "3DTMLauncherService_Render", controlPanel, daemonPort);
            DefaultStatusForwarderSetup(renderDaemonBot);
            SetupSocket(renderDaemonBot, SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true));
            renderDaemonBot.componentStatus = new ComponentStatus
            {
                ID = 21,
                Name = "3DTMLauncherService_Render",
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                Status = Status.Stopped,
                IP = SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true),
                Subscribe = true,
                NewDataReceived = false
            };

            SetVersionDataFromMap(renderDaemonBot);

            //CreateVersionTextObject(renderDaemonBot.UnitName, renderDaemonBot.GetVersionData(), renderDaemonBot.componentStatusContainer);

            StatusBot[] windowsDaemonBots = { fusionDaemonBot, renderDaemonBot };

            for (int i = 0; i < windowsDaemonBots.Length; ++i)
            {
                StatusBot currBot = windowsDaemonBots[i];
                currBot.DefaultStatusUpdateSetup(false);
                currBot.RegisterUpdateFunction(CPC_STATUS.IS_ALIVE, (byte[] update) =>
                {
                    currBot.UpdateSoftwareState(SOFTWARE.WINDOWS_SERVICE, SOFTWARE_STATE.RUNNING);
                });
                currBot.RegisterUpdateFunction(CPC_STATUS.RUNNING, (byte[] update) =>
                {
                    SOFTWARE_STATE[] states = ProcessDaemonStateUpdate(update);
                    // If I got a packet from the daemon, I know it's running, so make sure it's state stays running
                    states[(int)SOFTWARE.WINDOWS_SERVICE] = SOFTWARE_STATE.RUNNING;
                    if (currBot.UpdateSoftwareStates(states))
                    {
                        RespondToDaemonUpdate();
                    }
                    for (int j = 0; j < (int)SOFTWARE.COUNT; j++)
                    {
                        if (states[j] != SOFTWARE_STATE.UNKNOWN)
                        {
                            currBot.UpdateSoftwareState((SOFTWARE)j, states[j]);
                        }
                    }
                    //packet was received, so can update timeout counter
                    currBot.UpdateTimeLastHBReceived();
                });

                currBot.RegisterUpdateFunction(CPC_STATUS.LOG_DATA, (byte[] update) =>
                {
                    SaveLogData(currBot, update);
                });
                currBot.RegisterUpdateFunction(CPC_STATUS.RECEIVED_NEW_DATA, (byte[] update) =>
                {
                    OutputHelper.OutputLog($"{currBot.componentStatus.Name} received valid calibration data.", OutputHelper.Verbosity.Trace);
                    currBot.componentStatus.NewDataReceived = true;
                });
                currBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
                {
                    UpdateVersionData(currBot, update);
                });
                currBot.RegisterUpdateFunction(CPC_STATUS.SOFTWARE_INSTALLATION_RESULT, (byte[] update) =>
                {
                    HandleInstallationResult(currBot, update);
                });


                //currBot.componentStatusContainer.onTimedOutCallback_permanent += () =>
                //{
                //    //timed out, so no longer running
                //    currBot.UpdateStateAndUI(SOFTWARE.WINDOWS_SERVICE, SOFTWARE_STATE.TIMEDOUT);
                //    OutputHelper.OutputLog($"[{DateTime.UtcNow}] Daemon {currBot.UnitName} timed out. calling callback!", OutputHelper.Verbosity.Warning);
                //};
                currBot.Start();
            }

            // Create a thread that will reset the daemon states, and request updates
            // pass in cDaemonStatusCheckInterval as the interval
            runDaemonStatusThread = true;
            resetDaemonStatusThread = new Thread(() =>
            {
                while (runDaemonStatusThread)
                {
                    RequestDaemonStateUpdate();
                    Thread.Sleep((int)cDaemonStatusCheckInterval * 1000);
                }
                OutputHelper.OutputLog($"!!!CRITICAL!!! Daemon Status Thread is exiting!", OutputHelper.Verbosity.Fatal);
            });
            resetDaemonStatusThread.Start();
            OutputHelper.OutputLog("Finished creating DaemonBots", OutputHelper.Verbosity.Trace);
            depthDaemonBotsCreated = true;
            // request the bots give their latest version
            BroadcastEventOnce(CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
            return true;
        }
        public void SaveLogData(StatusBot currBot, byte[] update)
        {
            OutputHelper.OutputLog($"Received new log data from {currBot.UnitName}", OutputHelper.Verbosity.Debug);
            int position = cUpdateOffset;
            try
            {
                // Format is [(int)FileNameLength][(char[FileNameLength])FileName][(int)FileDataLength][(char[FileDataLength])FileData]
                int fileNameLength = System.BitConverter.ToInt32(update, position);
                position += sizeof(Int32);
                string fileName = Encoding.Default.GetString(update, position, fileNameLength);
                position += fileNameLength;
                int fileDataLength = System.BitConverter.ToInt32(update, position);
                position += sizeof(Int32);
                byte[] fileData = new byte[fileDataLength];
                Array.Copy(update, position, fileData, 0, fileDataLength);

                OutputHelper.OutputLog($"Saving {fileDataLength} bytes of log data to {fileName}", OutputHelper.Verbosity.Debug);
                currBot.SaveLogFile(fileName, fileData);
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog($"Exception trying to parse received log data: {e.Message}", OutputHelper.Verbosity.Error);
            }
        }
        public void RespondToDaemonUpdate()
        {
            CheckBotManagerReady(lastStartType);
            controlPanel.Refresh();
        }
        // returns empty string if daemons are ready
        // either Check CALIB state or CAPTURE state based on checkCalib
        public string GetDaemonNotReadyText(SOFTWARE requestType)
        {
            if (IsCleaningUp)
            {
                return "Cleaning";
            }
            else if (!BotManager.Instance.depthDaemonBotsCreated)
            {
                return "Creating daemon bots";
            }
            else if (requestType != SOFTWARE.CALIBRATION && requestType != SOFTWARE.CAPTURE)
            {
                return "Invalid RequestType";
            }

            string daemonStatus = "";
            for (int i = 0; i < BotManager.Instance.depthGenDaemonBots.Length; ++i)
            {
                // check which Daemon's not ready
                if (BotManager.Instance.depthGenDaemonBots[i].softwareStates[(int)requestType] != SOFTWARE_STATE.NOT_RUNNING)
                {
                    daemonStatus += $"[{(i + 1)}]:" + BotManager.Instance.depthGenDaemonBots[i].softwareStates[(int)requestType] + " ";
                }
            }
            return "Not all Daemons are ready..." + daemonStatus;
        }

        public delegate SOFTWARE_STATE GetBotStateFunction(StatusBot bot);
        public DAEMON_GROUP_STATE GetDaemonGroupState(GetBotStateFunction GetBotStateFunc, ref bool readyToBeLaunched)
        {
            int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);

            //check for errors: early outs
            for (int daemonIndex = 0; daemonIndex < depthGenNum; ++daemonIndex)
            {
                //check for error
                if (GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.LOCKED ||
                    GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.STOPPED ||
                    GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.TIMEDOUT)
                {
                    readyToBeLaunched = false;
                    return DAEMON_GROUP_STATE.ERROR;
                }
                //check if we haven't heard from this daemon
                if (GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.UNKNOWN)
                {
                    readyToBeLaunched = false;
                    return DAEMON_GROUP_STATE.UNKNOWN;
                }
            }
            //check for ready to launch: all need to be not runnning
            bool hasNotRunning = false;
            bool hasRunning = false;
            for (int daemonIndex = 0; daemonIndex < depthGenNum; ++daemonIndex)
            {
                //At this point in the function, we are either STARTED or NOT_RUNNING or RUNNING
                if (GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.STARTED)
                {
                    return DAEMON_GROUP_STATE.LAUNCHING;
                }
                else if (GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.NOT_RUNNING)
                {
                    hasNotRunning = true;
                }
                else
                {
                    //RUNNING
                    hasRunning = true;
                }
            }
            if (hasNotRunning)
            {
                if (hasRunning)
                {
                    readyToBeLaunched = false;
                    return DAEMON_GROUP_STATE.LAUNCHING;
                }
                else
                {
                    readyToBeLaunched = true;
                    return DAEMON_GROUP_STATE.NOT_RUNNING;
                }
            }
            else
            {
                if (hasRunning)
                {
                    readyToBeLaunched = false;
                    return DAEMON_GROUP_STATE.RUNNING;
                }
                else
                {
                    OutputHelper.OutputLog("VERY unexpected DAEMON group state", OutputHelper.Verbosity.Error);
                    readyToBeLaunched = false;
                    return DAEMON_GROUP_STATE.ERROR;
                }
            }
        }

        public DAEMON_GROUP_STATE GetDaemonGroupState_Calib()
        {
            return GetDaemonGroupState((StatusBot db) => { return db.softwareStates[(int)SOFTWARE.CALIBRATION]; }, ref allDaemonBotsReadyToBeLaunched_Calib);
        }

        public DAEMON_GROUP_STATE GetDaemonGroupState_Capture()
        {
            return GetDaemonGroupState((StatusBot db) => { return db.softwareStates[(int)SOFTWARE.CAPTURE]; }, ref allDaemonBotsReadyToBeLaunched_Capture);
        }

        public DAEMON_GROUP_STATE GetDaemonGroupState_BGCap()
        {
            return GetDaemonGroupState((StatusBot db) => { return db.softwareStates[(int)SOFTWARE.BACKGROUND_CAPTURE]; }, ref allDaemonBotsReadyToBeLaunched_BGCap);
        }
        public void UpdateDaemonBotsReadyStatus(SOFTWARE requestType)
        {
            OutputHelper.OutputLog("Call UpdateDaemonBotsReadyStatus", OutputHelper.Verbosity.Debug);

            DAEMON_GROUP_STATE captureGroupState = GetDaemonGroupState_Capture();
            DAEMON_GROUP_STATE calibGroupState = GetDaemonGroupState_Calib();
            DAEMON_GROUP_STATE bgcapGroupState = GetDaemonGroupState_BGCap();

            bool hasErrors =
                captureGroupState == DAEMON_GROUP_STATE.ERROR || captureGroupState == DAEMON_GROUP_STATE.UNKNOWN ||
                calibGroupState == DAEMON_GROUP_STATE.ERROR || calibGroupState == DAEMON_GROUP_STATE.UNKNOWN ||
                bgcapGroupState == DAEMON_GROUP_STATE.ERROR || bgcapGroupState == DAEMON_GROUP_STATE.UNKNOWN;
            if (hasErrors ||
                captureGroupState == DAEMON_GROUP_STATE.NOT_RUNNING ||
                calibGroupState == DAEMON_GROUP_STATE.NOT_RUNNING ||
                bgcapGroupState == DAEMON_GROUP_STATE.NOT_RUNNING)
            {

                if ((!allDaemonBotsReadyToBeLaunched_Capture || !allDaemonBotsReadyToBeLaunched_Calib || !allDaemonBotsReadyToBeLaunched_BGCap) || hasErrors)
                {
                    //not ready to launch a new session
                    OutputHelper.OutputLog("UpdateDaemonBotsReadyStatus: " + BotManager.Instance.GetDaemonNotReadyText(requestType), OutputHelper.Verbosity.Debug);
                    controlPanel.DisableButtons();
                    //DialogManager.Instance.UpdateMessage(GetDaemonNotReadyText(requestType), DialogManager.Instance.WarningMessage);
                }
                else
                {
                    OutputHelper.OutputLog($"All Daemons ready: [Capture:{captureGroupState}][Calib:{calibGroupState}][BG:{bgcapGroupState}]", OutputHelper.Verbosity.Debug);
                    if (allDaemonBotsReadyToBeLaunched_Capture && allDaemonBotsReadyToBeLaunched_BGCap && allDaemonBotsReadyToBeLaunched_Calib)
                    {
                        controlPanel.EnableButtons();
                    }
                    //DialogManager.Instance.UpdateMessage($"All Daemons ready: [Capture:{captureGroupState}][Calib:{calibGroupState}][BG:{bgcapGroupState}]", DialogManager.Instance.NormalMessage);
                }

            }
            controlPanel.Refresh();
        }

        // parameter-less method so we can InvokeRepeat on it
        public void RequestDaemonStateUpdate()
        {
            //request each daemon to tell us about their running
            BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STATE_UPDATE_REQUESTED);
        }
        #endregion
        private void SetupSocket(SocketBot bot, String IP)
        {
            bot.SetUseMulticast(SettingsManager.Instance.GetValueWithDefault("Network", "UseMultiCast", false, false));
            bot.SetUseTCP(SettingsManager.Instance.GetValueWithDefault("Network", "UseTCP", true, false));
            bot.SetIP(IP);
            //Check for Port override
            string overrideStatusPortName = "Ports/" + bot.UnitName;
            string overrideStatusPort = "";
            if (SettingsManager.Instance.config.Contains(overrideStatusPortName, "StatusPort"))
            {
                overrideStatusPort = SettingsManager.Instance.config[overrideStatusPortName]["StatusPort"].StringValue;
            }
            bot.SetStatusPorts(overrideStatusPort, "");
        }

        private void SetInitialEventPorts(EventBot bot)
        {
            //Check for Port override
            string overrideEventPortName = "Ports/" + bot.UnitName;
            string overrideEventPort = "";
            if (SettingsManager.Instance.config.Contains(overrideEventPortName, "EventPort"))
            {
                overrideEventPort = SettingsManager.Instance.config[overrideEventPortName]["EventPort"].StringValue;
            }
            bot.SetEventPorts(overrideEventPort, "");
        }

        /// <summary>
        /// Checks if the BotManager is ready to startup. 
        /// </summary>
        /// <param name="checkCalib">if true, we check if we're ready for calibration. else we check if we're ready for a session</param>
        /// <returns> 1)true if we're ready to start bots up and 2) what state the requested mode is in </returns>
        public Tuple<bool, DAEMON_GROUP_STATE> CheckBotManagerReady(SOFTWARE requestType)
        {
            OutputHelper.OutputLog("Call CheckBotManagerReady", OutputHelper.Verbosity.Debug);
            //check debug flag whether we're skipping this check
            if (SettingsManager.Instance.GetValueWithDefault("Debug", "SkipControlPanelDaemonStatusCheck", false))
            {
                return new Tuple<bool, DAEMON_GROUP_STATE>(true, DAEMON_GROUP_STATE.NOT_RUNNING);
            }

            if (IsCleaningUp)
            {
                OutputHelper.OutputLog("Still cleaning up previous session.. ", OutputHelper.Verbosity.Debug);
                return new Tuple<bool, DAEMON_GROUP_STATE>(false, DAEMON_GROUP_STATE.UNKNOWN);
            }
            else if (!BotManager.Instance.depthDaemonBotsCreated)
            {
                OutputHelper.OutputLog("waiting to create bots", OutputHelper.Verbosity.Debug);
                return new Tuple<bool, DAEMON_GROUP_STATE>(false, DAEMON_GROUP_STATE.UNKNOWN);
            }

            bool checkTypeReady = false;
            DAEMON_GROUP_STATE state = DAEMON_GROUP_STATE.UNKNOWN;
            switch (requestType)
            {
                case SOFTWARE.BACKGROUND_CAPTURE:
                    state = GetDaemonGroupState_BGCap();
                    checkTypeReady = allDaemonBotsReadyToBeLaunched_BGCap;
                    OutputHelper.OutputLog($"Calling GetDaemonGroupState_BGCap for {requestType}. All bots ready to be launched = {checkTypeReady}  State = {state}", OutputHelper.Verbosity.Debug);
                    break;
                case SOFTWARE.CALIBRATION:
                    state = GetDaemonGroupState_Calib();
                    checkTypeReady = allDaemonBotsReadyToBeLaunched_Calib;
                    OutputHelper.OutputLog($"Calling GetDaemonGroupState_Calib for {requestType}. All bots ready to be launched = {checkTypeReady}  State = {state}", OutputHelper.Verbosity.Debug);
                    break;
                case SOFTWARE.CAPTURE:
                    state = GetDaemonGroupState_Capture();
                    checkTypeReady = allDaemonBotsReadyToBeLaunched_Capture;
                    OutputHelper.OutputLog($"Calling GetDaemonGroupState_Capture for {requestType}. All bots ready to be launched = {checkTypeReady}  State = {state}", OutputHelper.Verbosity.Debug);
                    break;
                default:
                    OutputHelper.OutputLog("WARNING: checking an invalid starttype", OutputHelper.Verbosity.Warning);
                    state = DAEMON_GROUP_STATE.ERROR;
                    checkTypeReady = false;
                    break;
            }

            UpdateDaemonBotsReadyStatus(requestType);

            return new Tuple<bool, DAEMON_GROUP_STATE>(checkTypeReady, state);

        }

        /// <summary>
        /// Sets up the default forwarding functions for status updates
        /// This has to happen at the BotManager level, not the StatusBot level
        /// because it requires access to the DaemonPublisher to re-broadcast
        /// </summary>
        private void DefaultStatusForwarderSetup(StatusBot currBot)
        {
            // forwarding events
            // This even should only come from daemons
            currBot.RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_STARTED,
                delegate (byte[] update)
                {
                    OutputHelper.OutputLog($"Forwarding a calibration software started event.", OutputHelper.Verbosity.Trace);
                    BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_STARTED, update);
                });
        }
        /// <summary>
        /// Creates all the status bots fo thte depth pods.
        /// </summary>
        /// <returns></returns>
        public bool CreateDepthStatusBots()
        {
            int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);
            if (depthGenNum <= 0)
            {
                return false;
            }
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    OutputHelper.OutputLog($"CreateDepthStatusBots called. depthGenStatusBot array is not null.  Stopping existing bots.");
                    depthGenStatusBots[i].Stop();
                }
            }
            depthGenStatusBots = new StatusBot[depthGenNum];
            for (int i = 0; i < depthGenNum; ++i)
            {
                string dgName = "AzureKinectNanoToFusion_Pod" + (i + 1).ToString();
                // if the IP for fusion and fusion pod network are the same, use IP addresses, not ports (no need for KNCS)
                string hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "192.168.102.250", true);
                string hostNameOrIPPods = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250", true);
                bool UsingKNCS = !hostNameOrIP.Equals(hostNameOrIPPods);
                int dgPort = SettingsManager.Instance.GetValueWithDefault("Ports", "NanoApplicationStatusPort", 14502, true);
                if (UsingKNCS)
                {
                    dgPort = SettingsManager.Instance.GetValueWithDefault("Ports", "DepthPodApplicationStatusPortBase", 14570, true) + (i + 1);
                }
                else
                {
                    hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "DepthPodIPBase", "192.168.101.", true) + (i + 1).ToString();
                }

                StatusBot currBot = new StatusBot(hostNameOrIP, dgName, controlPanel, dgPort.ToString());
                currBot.SetDepthGenID(i);
                currBot.DepthGenMachineID = i;
                currBot.DefaultStatusUpdateSetup();
                DefaultStatusForwarderSetup(currBot);
                SetupSocket(currBot, hostNameOrIP);

                currBot.componentStatus = new ComponentStatus
                {
                    ID = i,
                    Name = dgName,
                    FPS = 0,
                    FrameNum = 0,
                    ErrState = CPC_ERROR.NONE,
                    Status = Status.Stopped,
                    IP = hostNameOrIP,
                    Subscribe = true,
                    //   CanLaunch = canLaunch // NOT used in this setup
                };

                OutputHelper.OutputLog($"{currBot.UnitName} registering new update function for CPC_STATUS.BUILD_VERSION");
                currBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
                {
                    UpdateVersionData(currBot, update);
                });
                SetVersionDataFromMap(currBot);

                depthGenStatusBots[i] = currBot;
            }
            return true;
        }

        public bool CreateCalibrationStatusBotsBasedOnConfig(SOFTWARE software)
        {
            Tuple<bool, DAEMON_GROUP_STATE> botReadyStatus = CheckBotManagerReady(software);
            if (!botReadyStatus.Item1 || botReadyStatus.Item2 != DAEMON_GROUP_STATE.NOT_RUNNING)
            {
                return false;
            }
            //DepthGen
            CreateDepthStatusBots();

            //CalibrationSoftware
            CreateCalibrationSoftwareStatusBot();

            return true;
        }

        public bool CreateCalibrationSoftwareStatusBot()
        {
            CalibrationComplete = false;
            if (calibrationSoftwareStatusBot != null)
            {
                calibrationSoftwareStatusBot.Stop();
            }

            //CalibrationSoftware
            calibrationSoftwareStatusBot = new StatusBot(null, "CalibrationSoftware", controlPanel);

            calibrationSoftwareStatusBot.DefaultStatusUpdateSetup();

            //If a specific IP address is not set for the CalibrationSoftware machine, it is usually located in the same machine as Fusion
            String fusionIPaddress = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true);
            SetupSocket(calibrationSoftwareStatusBot, SettingsManager.Instance.GetValueWithDefault("Network", "CalibrationSoftwareIPAddress", fusionIPaddress, true));
            calibrationSoftwareStatusBot.componentStatus = new ComponentStatus
            {
                ID = 12, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
                Name = "CalibrationSoftware",
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                Status = Status.Stopped,
                //If a specific IP address is not set for the CalibrationSoftware machine, it is usually located in the same machine as Fusion
                IP = SettingsManager.Instance.GetValueWithDefault("Network", "CalibrationSoftwareIPAddress", fusionIPaddress, true),
                Subscribe = true,
                //   CanLaunch = canLaunch // NOT used in this setup
            };

            calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
            {
                UpdateVersionData(calibrationSoftwareStatusBot, update);
            });
            calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_PROCESSING,
                delegate (byte[] update)
                {
                    CalibrationComplete = false;
                    // Rebroadcast so every other subscriber knows it is processing
                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_PROCESSING, update);
                    if (update.Length > cUpdateOffset)
                    {
                        String progress_msg = System.Text.Encoding.UTF8.GetString(update, cUpdateOffset, update.Length - cUpdateOffset);
                        controlPanel.UpdateCalibrationStatusTextbox(progress_msg);
                    }
                    OutputHelper.OutputLog($"Forwarding a calibration software processing event.", OutputHelper.Verbosity.Debug);
                });
            calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_RESULT,
                delegate (byte[] update)
                {
                    OutputHelper.OutputLog("CalibrationSoftware provided result");
                    Newtonsoft.Json.Linq.JObject json = new Newtonsoft.Json.Linq.JObject();

                    int calibJsonLength = 0;
                    bool success = false;
                    // Upon success, we expect the size of the calibration Json file to be returned. However, such size may
                    // also be 0 if an error occurred and we need to take care of that case.
                    if (update.Length >= cUpdateOffset + sizeof(int))
                    {
                        calibJsonLength = BitConverter.ToInt32(update, cUpdateOffset);

                        OutputHelper.OutputLog($"CalibrationSoftware Json payload size {calibJsonLength}");

                        if (calibJsonLength > 0)
                        {
                            // Creating payload array for calibration Json
                            byte[] jsonPacketBytes = new byte[calibJsonLength + sizeof(int)];


                            // Copying Json ([length][Json data]) from update into the new payload array. The idea is to remove the
                            // first "cUpdateOffset" bytes from "update", since they represent the CPC_STATUS.CALIBRATION_SOFTWARE_RESULT status
                            Buffer.BlockCopy(BitConverter.GetBytes(calibJsonLength), 0, jsonPacketBytes, 0, sizeof(int));
                            Buffer.BlockCopy(update, cUpdateOffset + sizeof(int), jsonPacketBytes, sizeof(int), calibJsonLength);

                            // Decoding Json
                            CameraCalibrationHelper.BytePacketToJSON(update, out json);

                            //Re-broadcasting the calibration software result to Fusion, Render, and the PODs with sanitized Json payload
                            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT, jsonPacketBytes);
                            success = true;
                        }
                    }

                    // Calibration failed and we should let the user know and stop calibration
                    if (!success)
                    {
                        OutputHelper.OutputLog($"Error: CalibrationSoftware failed! Calibration Json with length 0 returned after processing.", OutputHelper.Verbosity.Debug);
                        controlPanel.UpdateCalibrationStatusTextbox("Calibration Failed.  Please re-try calibration.");

                        // Returning empty Json string to indicate a failed calibration to Fusion, Render, and the PODs
                        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT, new byte[0]);

                        // Shut down the session if we failed.
                        controlPanel.StopCalibration();
                    }
                    else
                    {
                        float ReprojectionError = json["calibrationSession"]["metrics"]["ReprojectionError"].ToObject<float>();
                        float DepthError = json["calibrationSession"]["metrics"]["DepthError"].ToObject<float>();
                        float MultiViewMisalignment = json["calibrationSession"]["metrics"]["MultiViewMisalignment"].ToObject<float>();
                        String metric_msg = $"Successful Calibration. Reprojection error {ReprojectionError:F3} pixels. Depth error {DepthError:F3}mm. MultiViewMisalignment {MultiViewMisalignment:F3}mm";
                        if (MultiViewMisalignment > 2.0)
                        {
                            metric_msg = $"Calibration succeeded, but the MultiViewMisalignment is greater than 2.0.  Quality of the output may be degraded with this calibration.  You may way to recalibrate. Reprojection error {ReprojectionError:F3} pixels. Depth error {DepthError:F3}mm. MultiViewMisalignment {MultiViewMisalignment:F3}mm";
                        }

                        controlPanel.UpdateCalibrationStatusTextbox(metric_msg);
                        OutputHelper.OutputLog(metric_msg);
                        OutputHelper.OutputLog($"Distributing 3DTM calibration files");

                        controlPanel.UpdateCalibrationStatusTextbox("Distributing 3DTM calibration files");
                    }
                    CalibrationComplete = true;  // This will let the control panel thread move to the next stage
                });

            SetVersionDataFromMap(calibrationSoftwareStatusBot);

            calibrationSoftwareStatusBot.Start();
            return true;
        }

        /// <summary>
        /// Call this function to create the bots and connect up. Callbacks and sequenced tasks have to be setup separately by setting the bots' componentStatusContainer.onRunningCallback method.
        /// </summary>
        public bool CreateStatusBotsBasedOnConfig()
        {
            //DepthGen
            CreateDepthStatusBots();

            //Fusion
            if (fusionStatusBot != null)
            {
                fusionStatusBot.Stop();
            }
            fusionStatusBot = new StatusBot(null, "Fusion", controlPanel);
            fusionStatusBot.DefaultStatusUpdateSetup();
            DefaultStatusForwarderSetup(fusionStatusBot);

            SetupSocket(fusionStatusBot, SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true));
            fusionStatusBot.componentStatus = new ComponentStatus
            {
                ID = 10, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
                Name = "Fusion",
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                Status = Status.Stopped,
                IP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true),
                Subscribe = true,
                //   CanLaunch = canLaunch // NOT used in this setup
            };

            fusionStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
            {
                UpdateVersionData(fusionStatusBot, update);
            });
            SetVersionDataFromMap(fusionStatusBot);

            //CreateVersionTextObject(fusionStatusBot.UnitName, fusionStatusBot.GetVersionData(), fusionStatusBot.componentStatusContainer);
            //Renderer
            if (renderStatusBot != null)
            {
                renderStatusBot.Stop();
            }
            renderStatusBot = new StatusBot(null, "Render", controlPanel);
            renderStatusBot.DefaultStatusUpdateSetup();
            DefaultStatusForwarderSetup(renderStatusBot);

            SetupSocket(renderStatusBot, SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true));

            renderStatusBot.RegisterUpdateFunction(CPC_STATUS.TOGGLE_FUSION_HIGH_RESOLUTION,
                delegate (byte[] update)
                {
                    //packet that needs to be forwarded to Fusion
                    if (update.Length <= 0)
                    {
                        OutputHelper.OutputLog("TOGGLE_FUSION_HIGH_RESOLUTION packet missing data!", OutputHelper.Verbosity.Error);
                        return;
                    }
                    byte[] fixedPacket = new byte[update.Length - 1];
                    Buffer.BlockCopy(update, 1, fixedPacket, 0, update.Length - 1);
                    DaemonPublisher.PublishEvent(CONTROL_PANEL_EVENT.TOGGLE_FUSION_HIGH_RESOLUTION, fixedPacket);
                });
            renderStatusBot.componentStatus = new ComponentStatus
            {
                ID = 11, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
                Name = "Render",
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                Status = Status.Stopped,
                IP = SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true),
                Subscribe = true,
                //   CanLaunch = canLaunch // NOT used in this setup
            };
            renderStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
            {
                UpdateVersionData(renderStatusBot, update);
            });
            SetVersionDataFromMap(renderStatusBot);

            fusionStatusBot.Start();
            renderStatusBot.Start();
            //CreateVersionTextObject(renderStatusBot.UnitName, renderStatusBot.GetVersionData(), renderStatusBot.componentStatusContainer);

            // request the bots give their latest version
            BroadcastEventOnce(CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
            return true;
        }

        protected void SetVersionDataFromMap(StatusBot bot)
        {
            if (versionMap.ContainsKey(bot.UnitName))
            {
                // set the version data in the bot to whatever we read from disk.
                bot.VersionData = versionMap[bot.UnitName].versionData;
            }
            else
            {
                OutputHelper.OutputLog($"Version Map doesn't contain an entry for [{bot.UnitName}]", OutputHelper.Verbosity.Error);
                foreach (var vc in versionMap)
                {
                    OutputHelper.OutputLog($"[{vc.Key}] = [{vc.Value}]", OutputHelper.Verbosity.Trace);
                }
            }
        }

        #region CleanUp
        public void CleanupMachines()
        {
            //tell DGs to stop
            Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED);
            //tell fusion and render to stop
            Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
            Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
        }
        public void CleanUpStatusBots(BotManagerCallback cb = null, bool cleanDaemonBots = false)
        {
            IsCleaningUp = true;
            //DialogManager.Instance.UpdateMessage("Cleaning Up.", DialogManager.Instance.WarningMessage);

            //kill processes
            CleanupMachines();

            if (cb != null)
            {
                // clean previous cleanup callback
                onCleanedup = null;
                onCleanedup += cb;
            }


            //// stop outstanding broadcasts
            StopBroadcastThread();
            ////wait and check until all machines report back as stopped
            //IEnumerator stopCheckCoroutine = CheckUntilMachinesStop(cleanDaemonBots);
            //if (gameObject.activeInHierarchy)
            //{
            //    StartCoroutine(stopCheckCoroutine);
            //}
            //else
            //{
            CheckUntilMachinesStop(cleanDaemonBots, false).MoveNext();
            //}
        }


        public void CleanupDaemonBots()
        {
            runDaemonStatusThread = false;
            if (DaemonPublisher != null)
                DaemonPublisher.Stop();
            if (depthGenDaemonBots != null)
            {
                for (int i = 0; i < depthGenDaemonBots.Length; ++i)
                {
                    depthGenDaemonBots[i].Stop();
                }
            }
            if (fusionDaemonBot != null)
            {
                fusionDaemonBot.Stop();
            }
            if (renderDaemonBot != null)
            {
                renderDaemonBot.Stop();
            }
            depthDaemonBotsCreated = false;
        }

        private bool AreMachinesStillCleaning(bool checkDaemons, bool checkThreadAlive)
        {
            bool stillCleaning = false;

            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    stillCleaning = stillCleaning || depthGenStatusBots[i].IsAlive(checkThreadAlive);
                }
            }

            if (fusionStatusBot != null)
            {
                stillCleaning = stillCleaning || fusionStatusBot.IsAlive(checkThreadAlive);
            }

            if (renderStatusBot != null)
            {
                stillCleaning = stillCleaning || renderStatusBot.IsAlive(checkThreadAlive);
            }

            if (calibrationSoftwareStatusBot != null)
            {
                stillCleaning = stillCleaning || calibrationSoftwareStatusBot.IsAlive(checkThreadAlive);
            }

            if (checkDaemons && depthGenDaemonBots != null)
            {
                for (int i = 0; i < depthGenDaemonBots.Length; ++i)
                {
                    stillCleaning = stillCleaning || depthGenDaemonBots[i].IsAlive(checkThreadAlive);
                }

                if (DaemonPublisher != null)
                    stillCleaning = stillCleaning || DaemonPublisher.IsAlive(checkThreadAlive);

            }
            return stillCleaning;
        }
        public void OnAllMachinesStopped(bool cleanDaemonBots)
        {
            //force stop at this point 
            //tell daemon to stop the application as well
            if (lastStartType != SOFTWARE.SYSTEM_START)
            {
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)lastStartType));
            }

            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    depthGenStatusBots[i].Stop();
                }
            }
            if (fusionStatusBot != null)
            {
                fusionStatusBot.Stop();
            }
            if (renderStatusBot != null)
            {
                renderStatusBot.Stop();
            }
            if (calibrationSoftwareStatusBot != null)
            {
                calibrationSoftwareStatusBot.Stop();
            }

            if (cleanDaemonBots)
            {
                CleanupDaemonBots();
            }
            else
            {
                //only check for daemon if we're not cleaning up completely
                BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STATE_UPDATE_REQUESTED);
            }

            cleanupCheckThread = new Thread(() =>
            {
                CheckUntilCleanupFinishes();
            });

            //just cleanup if we're exiting completely. no coroutine waits
            bool stillCleaning = true;
            while (stillCleaning)
            {
                stillCleaning = AreMachinesStillCleaning(true, true);
            }
        }

        /// <summary>
        /// checks all machines are stopped or stops them after time out
        /// </summary>
        /// <returns></returns>
        private IEnumerator CheckUntilMachinesStop(bool cleanDaemonBot, bool shouldReturn = true)
        {
            long startTime = DateTimeOffset.Now.ToUnixTimeSeconds();
            bool stillCleaning = true;
            while (stillCleaning && (DateTimeOffset.Now.ToUnixTimeSeconds() - startTime) < cMaxCleanupTime)
            {
                stillCleaning = AreMachinesStillCleaning(false, false);
                if (shouldReturn)
                {
                    yield return new SpinWait();  //probably not correct
                }
            }

            OnAllMachinesStopped(cleanDaemonBot);
        }
        /// <summary>
        /// check all machines are cleaned up properly. 
        /// </summary>
        /// <returns></returns>
        private IEnumerator CheckUntilCleanupFinishes()
        {
            bool stillCleaning = true;
            while (stillCleaning)
            {
                stillCleaning = AreMachinesStillCleaning(false, true);

                yield return WaitForSecondsAsync(0.2);
            }

            //NOTE: can't do a full netMQcleanup here, since it cleans everything netmq related (including daemon bots which aren't cleaned here!)  
            OutputHelper.OutputLog("statusBots cleanedup!", OutputHelper.Verbosity.Debug);
            IsCleaningUp = false; // set this before the onCleanedup callback, since they might depend on this flag

            onCleanedup?.Invoke();
        }
        #endregion

        private void InitializePublisherAndStart()
        {
            DaemonPublisher = new EventBot(""); //Invalid publish address, but wiill be reInit'ed with correct ones in constructor
            DaemonPublisher.SetUseMulticast(SettingsManager.Instance.GetValueWithDefault<bool>("Network", "UseMultiCast", false, false));
            DaemonPublisher.SetUseTCP(SettingsManager.Instance.GetValueWithDefault<bool>("Network", "UseTCP", true, false));
            SetInitialEventPorts(DaemonPublisher);

            DaemonPublisher.Start();
        }


        public void BroadcastEventOnce(CONTROL_PANEL_EVENT eventType)
        {
            //lazy init
            if (DaemonPublisher == null)
            {
                InitializePublisherAndStart();
            }
            DaemonPublisher.PublishEvent(eventType);
            OutputHelper.OutputLog(String.Format("Broadcast Once {0}", eventType), OutputHelper.Verbosity.Debug);
        }

        public void BroadcastEventOnce(CONTROL_PANEL_EVENT eventType, byte[] eventData)
        {
            //lazy init
            if (DaemonPublisher == null)
            {
                InitializePublisherAndStart();
            }
            DaemonPublisher.PublishEvent(eventType, eventData);
            OutputHelper.OutputLog(String.Format("Broadcast Once {0} byte[{1}]", eventType, eventData?.Length), OutputHelper.Verbosity.Debug);
        }

        public void StopBroadcastThread()
        {
            while (broadcastThread != null && broadcastThread.IsAlive)
            {
                OutputHelper.OutputLog("Stopping broadcast thread", OutputHelper.Verbosity.Debug);
                runBroadcastThread = false;
                Thread.Sleep(100);
            }
        }

        public void BroadcastStartEventOnce(SOFTWARE startType, byte[] eventData)
        {
            byte[] dataArray = new byte[eventData.Length + 1];

            // Coying eventType into dataArray
            dataArray[0] = (byte)startType;
            // Copying payload
            eventData.CopyTo(dataArray, 1);

            BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, dataArray);
        }

        public void BroadcastStopUntilAllDGStop(SOFTWARE softwareToStop)
        {
            StopBroadcastThread();
            broadcastThread = new Thread(() =>
            {
                OutputHelper.OutputLog("Broadcast stop thread is starting");
                int runningDGBots = GetBotsWithSoftwareRunning(softwareToStop) + GetBotsSendingFPS();
                OutputHelper.OutputLog($"Broadcast stop Thread started.  Running DGs: {runningDGBots}/{depthGenStatusBots.Length}");
                byte[] dataArray =
                [
                    // Coying startType into dataArray
                    //(byte)startType,
                ];
                int loopCounter = 0;
                while (runBroadcastThread && runningDGBots > 0)
                {
                    // Only re-broadcast every 3 seconds, but check every 300ms so the UI is more responsive
                    if (loopCounter == 0)
                    {
                        BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, dataArray);
                        OutputHelper.OutputLog(String.Format("BroadcastingUntilAllStop {0} for DGs: {1}/{2}", CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, runningDGBots, BotManager.Instance.depthGenStatusBots.Length), OutputHelper.Verbosity.Warning);
                    }
                    Thread.Sleep(300);
                    loopCounter = loopCounter == 10 ? 0 : loopCounter++;
                    runningDGBots = GetBotsWithSoftwareRunning(softwareToStop);
                }
            }
            );
            runBroadcastThread = true;
            broadcastThread.Start();
        }
        public void BroadcastSystemStartUntilAllDGStartTransmitting()
        {
            StopBroadcastThread();
            broadcastThread = new Thread(() =>
            {
                OutputHelper.OutputLog("Broadcast thread is starting");
                byte[] dataArray = new byte[1];
                // Copying startType into dataArray
                dataArray[0] = (byte)SOFTWARE.CAPTURE;
                int runningDGBots = GetBotsSendingFPS();
                OutputHelper.OutputLog($"Broadcast Thread started.  Running DGs: {runningDGBots}/{depthGenStatusBots.Length}");
                int loopCounter = 0;
                while (runBroadcastThread && runningDGBots < depthGenStatusBots.Length)
                {
                    if (loopCounter == 0)
                    {
                        BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, new byte[0]);
                        OutputHelper.OutputLog(String.Format("BroadcastingUntilAllStart {0}-{1} for DGs: {2}/{3}", CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, SOFTWARE.SYSTEM_START,
                            runningDGBots, BotManager.Instance.depthGenStatusBots.Length), OutputHelper.Verbosity.Warning);
                    }
                    Thread.Sleep(300);
                    loopCounter = loopCounter == 10 ? 0 : loopCounter++;
                    runningDGBots = GetBotsSendingFPS();
                }
            }
            );
            runBroadcastThread = true;
            broadcastThread.Start();
        }
        public void BroadcastSoftwareStartUntilAllDGStart(SOFTWARE softwareToStart, byte[] eventData = null)
        {
            StopBroadcastThread();
            broadcastThread = new Thread(() =>
            {
                byte[] dataArray = null;
                OutputHelper.OutputLog("Broadcast thread is starting");
                if (softwareToStart != SOFTWARE.SYSTEM_START)
                {
                    if (eventData != null)
                    {
                        dataArray = new byte[eventData.Length + 1];
                        dataArray[0] = (byte)softwareToStart;
                        eventData.CopyTo(dataArray, 1);
                    }
                    else
                    {
                        dataArray = new byte[1] { (byte)softwareToStart };
                    }
                }
                int runningDGBots = GetBotsWithSoftwareRunning(softwareToStart);
                OutputHelper.OutputLog($"Broadcast Thread started.  Running DGs: {runningDGBots}/{depthGenStatusBots.Length}");
                int loopCounter = 0;
                while (runBroadcastThread && runningDGBots < depthGenStatusBots.Length)
                {
                    if (loopCounter == 0)
                    {
                        BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, dataArray);
                        OutputHelper.OutputLog(String.Format("BroadcastingUntilAllStart {0}-{1} for DGs: {2}/{3}", CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, SOFTWARE.CAPTURE,
                            runningDGBots, BotManager.Instance.depthGenStatusBots.Length), OutputHelper.Verbosity.Warning);
                    }
                    Thread.Sleep(300);
                    loopCounter = loopCounter == 10 ? 0 : loopCounter++;
                    runningDGBots = GetBotsWithSoftwareRunning(softwareToStart);
                }
            }
            );
            runBroadcastThread = true;
            broadcastThread.Start();
        }

        public int GetBotsWithSoftwareRunning(SOFTWARE softwareToCheck)
        {
            // Iterate over the depthGenStatusBots and count how many have componentStatus.Status == Status.Running
            int runningDGBots = 0;
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    if (depthGenStatusBots[i].componentStatus.Status == Status.Ready)
                    {
                        // do a double check to make sure the daemon says the actual process is still running, it could have crashed
                        if (depthGenDaemonBots[i].softwareStates[(int)softwareToCheck] == SOFTWARE_STATE.RUNNING)
                        {
                            runningDGBots++;
                        }
                        else
                        {
                            //OutputHelper.OutputLog($"The status bot for {depthGenStatusBots[i].UnitName} says it is running, but AKLD does not see an active process!", OutputHelper.Verbosity.Error);
                            // Most likely means the status bot has been started, but the software isn't started up yet, just wait
                        }
                    }
                }
            }
            return runningDGBots;
        }

        public bool CheckAllCalibHaveSameFrameNumber(int targetFrameNumber)
        {
            if (depthGenStatusBots == null)
            {
                return false;
            }

            for (int i = 0; i < depthGenStatusBots.Length; ++i)
            {
                if (depthGenStatusBots[i] != null)
                {
                    int frameNum = depthGenStatusBots[i].componentStatus.FrameNum;
                    if (frameNum < targetFrameNumber)
                    {
                        OutputHelper.OutputLog("CheckAllCalibHaveSameFrameNumber failed for " + i + ": got=" + frameNum + ";target=" + targetFrameNumber, OutputHelper.Verbosity.Trace);
                        return false;
                    }
                }
                else
                {
                    OutputHelper.OutputLog("CheckAllCalibHaveSameFrameNumber failed : essential statusBot component is NULL?!", OutputHelper.Verbosity.Error);
                    return false;
                }
            }
            return true;
        }

        public void OnDisable()
        {
            BotManager.Instance.CleanUpStatusBots(null, true);
        }

        public void OnDestroy()
        {
            BotManager.Instance.CleanUpStatusBots(null, true);
            // start netMQ cleanup here, but don't block
            try
            {
                NetMQ.NetMQConfig.Cleanup(false);
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog("NetMQConfig.Cleanup(false) threw an exception: " + e.ToString(), OutputHelper.Verbosity.Error);
            }

            if (depthGenDaemonBots != null)
            {
                foreach (var bot in depthGenDaemonBots)
                {
                    bot?.Stop();
                }
            }
            fusionDaemonBot?.Stop();
            renderDaemonBot?.Stop();

            if (depthGenStatusBots != null)
            {
                foreach (var bot in depthGenStatusBots)
                {
                    bot?.Stop();
                }
            }
            fusionStatusBot?.Stop();
            renderStatusBot?.Stop();
            calibrationSoftwareStatusBot?.Stop();

            DaemonPublisher?.Stop();
            runDaemonStatusThread = false;
            runBroadcastThread = false;
            if (resetDaemonStatusThread != null && resetDaemonStatusThread.ThreadState != ThreadState.Unstarted)
            {
                resetDaemonStatusThread.Join();
            }
            if (broadcastThread != null && broadcastThread.ThreadState != ThreadState.Unstarted)
            {
                broadcastThread.Join();
            }
            if (cleanupCheckThread != null && cleanupCheckThread.ThreadState != ThreadState.Unstarted)
            {
                cleanupCheckThread.Join();
            }
            // only cleanup netmq completely here
            //full netmq cleanup after everything stopped properly
            try
            {
                NetMQ.NetMQConfig.Cleanup(true);
            }
            catch (NetMQ.TerminatingException)
            {
                OutputHelper.OutputLog("NetMQConfig.Cleanup() threw a TerminatingException.  This is expected if the application is shutting down.", OutputHelper.Verbosity.Debug);
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog("NetMQConfig.Cleanup() threw an exception: " + e.ToString(), OutputHelper.Verbosity.Error);
            }
            finally
            {
                OutputHelper.OutputLog("statusBots + NETMQ cleanedup!", OutputHelper.Verbosity.Info);
                SaveVersionInformationToDisk();
            }
        }

        public void HandleInstallationResult(StatusBot currBot, byte[] update)
        {
            // parse the update
            SOFTWARE updatedSoftware = (SOFTWARE)update[cUpdateOffset];
            bool success = System.BitConverter.ToBoolean(update, cUpdateOffset + sizeof(byte));
            int messageLength = System.BitConverter.ToInt32(update, cUpdateOffset + sizeof(byte) + sizeof(bool));
            string message = "";
            OutputHelper.OutputLog($"HandleInstallationResult, success: {success} message length: {messageLength} Message: {BitConverter.ToString(update)}", OutputHelper.Verbosity.Trace);
            if (messageLength > 0)
            {
                if (messageLength > update.Length - (sizeof(char) + sizeof(char) + sizeof(char))) // byte 0 is packet type, byte 1 is software, byte 2 is success (bool) bytes 3+ are messagelength + message
                {
                    OutputHelper.OutputLog($"ERROR.  Message length was {messageLength} but packet size is only {update.Length}", OutputHelper.Verbosity.Error);
                }
                else
                {
                    message = Encoding.Default.GetString(update, cUpdateOffset + sizeof(byte) + sizeof(bool) + sizeof(int), messageLength);
                }
            }
            // put the error message in the debug log
            OutputHelper.OutputLog($"{currBot.UnitName} reports software installation of {updatedSoftware.ToString()} result: {success}.  Message: {message}", OutputHelper.Verbosity.Debug);
            // set the buttons back to their enabled state if text == " Update ", or disable them if text == " Installing... "
            switch (updatedSoftware)
            {
                case PeabodyNetworkingLibrary.SOFTWARE.LINUX_DAEMON:
                case PeabodyNetworkingLibrary.SOFTWARE.WINDOWS_SERVICE:
                    {
                        // update my component only
                        if (success)
                        {
                            SetUpdateButtonText(currBot.UnitName, " Updated ", false);
                        }
                        else
                        {
                            SetUpdateButtonText(currBot.UnitName, " Error Updating ", false);
                        }
                        break;
                    }
                case PeabodyNetworkingLibrary.SOFTWARE.CALIBRATION:
                case PeabodyNetworkingLibrary.SOFTWARE.CAPTURE:
                case PeabodyNetworkingLibrary.SOFTWARE.BACKGROUND_CAPTURE:
                    {
                        // find the similarly named AzureKinectNanoToFusion_PODx and update that button
                        string podNumber = currBot.UnitName.Substring(currBot.UnitName.IndexOf("_"));
                        string unitName = "AzureKinectNanoToFusion" + podNumber;
                        OutputHelper.OutputLog($"Turning off Update button for {unitName}", OutputHelper.Verbosity.Debug);
                        if (success)
                        {
                            SetUpdateButtonText(unitName, " Updated ", false);
                        }
                        else
                        {
                            SetUpdateButtonText(unitName, " Error Updating ", false);
                        }
                        break;
                    }
                case PeabodyNetworkingLibrary.SOFTWARE.FUSION:
                    {
                        OutputHelper.OutputLog($"Turning off Update button for Fusion", OutputHelper.Verbosity.Debug);
                        if (success)
                        {
                            SetUpdateButtonText("Fusion", " Updated ", false);
                        }
                        else
                        {
                            SetUpdateButtonText("Fusion", " Error Updating ", false);
                        }
                        break;
                    }
                case PeabodyNetworkingLibrary.SOFTWARE.RENDER:
                    {
                        OutputHelper.OutputLog($"Turning off Update button for Render", OutputHelper.Verbosity.Debug);
                        if (success)
                        {
                            SetUpdateButtonText("Render", " Updated ", false);
                        }
                        else
                        {
                            SetUpdateButtonText("Render", " Error Updating ", false);
                        }
                        break;
                    }
                default:
                    OutputHelper.OutputLog($"Don't have a handler for that type of software update.", OutputHelper.Verbosity.Warning);
                    break;
            }
        }
        // Enqueues a change for the UI that will be handled in the Update() call
        protected void SetUpdateButtonText(string unitName, string text, bool enable)
        {
            if (versionMap.ContainsKey(unitName))
            {
                VersionContent vc = versionMap[unitName];
                vc.UpdateButtonText = text;
                vc.EnableUpdateButton(enable);
            }
        }
        public void UpdateVersionData(StatusBot currBot, byte[] update)
        {
            OutputHelper.OutputLog($"{currBot.UnitName} UpdateVersionData called");
            currBot.SaveVersionData(update);

            // Update the version map as well
            if (versionMap.ContainsKey(currBot.UnitName))
            {
                OutputHelper.OutputLog($"{currBot.UnitName} updating version content", OutputHelper.Verbosity.Debug);
                VersionContent vc = versionMap[currBot.UnitName];
                vc.versionData = currBot.VersionData;
                versionMap[currBot.UnitName] = vc;
                // Update the UI text box if it's been created
                vc.UpdateButtonText = "";
            }
            else
            {
                VersionContent vc = new VersionContent();
                vc.unitName = currBot.UnitName;
                vc.versionData = currBot.VersionData;
                versionMap.TryAdd(currBot.UnitName, vc);
                vc.UpdateButtonText = "";

                OutputHelper.OutputLog($"{currBot.UnitName} not found in the version map.  Creating entry", OutputHelper.Verbosity.Trace);
            }

            controlPanel.RefreshVersionUI();
        }
        public void SaveVersionInformationToDisk()
        {
            //Save out the current version information
            var path = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "//3DTMVersionData.xml";

            // Serialize
            using (TextWriter writer = new StreamWriter(path))
            {
                XmlSerializer serializer = new XmlSerializer(typeof(VersionContent[]));
                serializer.Serialize(writer, versionMap.Values.ToArray());
            }
            OutputHelper.OutputLog($"Wrote version data to {path}", OutputHelper.Verbosity.Info);
        }

        public IDictionary<string, VersionContent> GetVersionContent()
        {
            return versionMap;
        }

        public void LoadVersionInformationFromDisk()
        {
            VersionContent[] entries;
            //Save out the current version information
            var path = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "//3DTMVersionData.xml";
            try
            {
                using (TextReader reader = new StreamReader(path))
                {
                    XmlSerializer serializer = new XmlSerializer(typeof(VersionContent[]));
                    entries = (VersionContent[])serializer.Deserialize(reader);
                }
                foreach (VersionContent vc in entries)
                {
                    OutputHelper.OutputLog($"Read {vc.unitName}: {vc.versionData.Description} from disk", OutputHelper.Verbosity.Info);
                    if (vc.versionData.Description == "" || vc.versionData.Description == null)
                    {
                        OutputHelper.OutputLog($"Version data's description field was blank.  So creating a new (unknown) version data object");
                        PeabodyNetworkingLibrary.VersionData vd = new VersionData();
                        vd.Description = "Unknown Version";
                        vc.versionData = vd;
                    }
                    vc.UpdateButtonText = "";
                    if (versionMap.ContainsKey(vc.unitName))
                    {
                        // game object is null, attach
                        VersionContent thisVC = versionMap[vc.unitName];
                        versionMap[vc.unitName] = thisVC;
                    }
                    else
                    {
                        versionMap.TryAdd(vc.unitName, vc);
                    }
                }
                OutputHelper.OutputLog($"Read version data from {path}", OutputHelper.Verbosity.Info);
            }
            catch (IOException e)
            {
                OutputHelper.OutputLog($"IOException while reading version data from {path}: {e.Message}", OutputHelper.Verbosity.Warning);
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog($"Exception while reading version data from {path}: {e.Message}", OutputHelper.Verbosity.Warning);
            }
        }

        internal int GetBotsSendingFPS()
        {
            // Iterate over the depthGenDaemonBots and count how many have softwareStates[(int)SOFTWARE.CAPTURE] == SOFTWARE_STATE.RUNNING
            int runningDGBots = 0;
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    if (depthGenStatusBots[i].componentStatus.FPS > 0)
                    {
                        runningDGBots++;
                    }
                }
            }
            return runningDGBots;
        }

        internal int GetBotsRecordingVideo()
        {
            int runningDGBots = 0;
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    if (depthGenStatusBots[i].componentStatus.VideoRecordingStarted)
                    {
                        runningDGBots++;
                    }
                }
            }
            return runningDGBots;
        }

        internal int GetBotsWithVideoTransferRunning()
        {
            int runningDGBots = 0;
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    if (depthGenStatusBots[i].componentStatus.VideoTransferStarted && depthGenStatusBots[i].componentStatus.VideoTransferFinished)
                    {
                        runningDGBots++;
                    }
                }
            }
            return runningDGBots;
        }

        internal int GetBotsWithVideoTransferComplete()
        {
            int runningDGBots = 0;
            if (depthGenStatusBots != null)
            {
                for (int i = 0; i < depthGenStatusBots.Length; ++i)
                {
                    if (depthGenStatusBots[i].componentStatus.VideoTransferFinished)
                    {
                        runningDGBots++;
                    }
                }
            }
            return runningDGBots;
        }
    }
}