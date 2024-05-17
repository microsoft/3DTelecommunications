using ControlPanel;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Threading;
using System.Text;
using UnityEngine;
using UnityEngine.UI;
using PeabodyNetworkingLibrary;
using System.Xml;
using System.Xml.Serialization;
using System.Net;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Collections.Concurrent;

public delegate void BotManagerCallback();
public class BotManager : MonoBehaviour
{
    public const long cMaxCleanupTime = 8; //in seconds max cleanupTime allocated
    const int cUpdateOffset = 1; //first byte of status packet is the status type, so skip it

    //threadsafe SINGLETON pattern
    protected static BotManager instance;
    // Explicit static constructor to tell C# compiler
    // not to mark type as beforefieldinit
    static BotManager()
    {
    }

    private BotManager()
    {
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
        OutputHelper.OutputLog("BotManager destructor called", OutputHelper.Verbosity.Info);
        
        OnDestroy();
    }
    // Start is called before the first frame update
    void Start()
    {
        if (instance == null)
        {
            instance = this;
            //kill all running things to get a fresh start
            CleanUpStatusBots(() => {
                instance.CreateDaemonMonitorsBasedOnConfig();
                CheckBotManagerReady(SOFTWARE.CAPTURE);
            });
            mainThreadCallbacks = new ConcurrentQueue<BotManagerCallback>();
        }
        LoadVersionInformationFromDisk();
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
        foreach (VersionContent vc in versionMap.Values)
        {
            vc.UpdateUIElements();
        }
    }

    //Implementation

    public EventBot DaemonPublisher;
    private IEnumerator broadcastCoroutine;
    private SOFTWARE lastStartType = SOFTWARE.CAPTURE; // START with this so we force cleanup on launch

    public bool IsCleaningUp { get; private set; } = false;
    public BotManagerCallback onCleanedup;

    //daemon related
    public DaemonBot[] depthGenDaemonBots { get; private set; }
    public DaemonBot fusionDaemonBot;
    public DaemonBot renderDaemonBot;
    public bool depthDaemonBotsCreated = false;
    public bool allDaemonBotsReadyToBeLaunched_Calib;
    public bool allDaemonBotsReadyToBeLaunched_Capture;
    public bool allDaemonBotsReadyToBeLaunched_BGCap;
    public long lastDaemonCheckTime = 0;
    const long cDaemonStatusCheckInterval = 5; // in seconds, how often we reissue a status check on daemons for app status
    //daemon status checks suceess and failure callbacks
    public BotManagerCallback onDaemonStatusUpdateCallback;
    // this facilitates many UnityEngine related calls that need to be executed on the main thread. so we queue these up to be run in Update()
    public ConcurrentQueue<BotManagerCallback> mainThreadCallbacks;

    public StatusBot[] depthGenStatusBot;
    public int runningDGBots;
    public StatusBot fusionStatusBot;
    public StatusBot renderStatusBot;
    public StatusBot calibrationSoftwareStatusBot;

    public GameObject VersionListCanvas;

    public GameObject VersionGameObject;

    // We're keeping the versionMap struct instead of just accessing the data from inside the bots that are part
    // of the BotManager class because not all bots are created at any point in time, so if the user wanted to update
    // Fusion, they would have to have fusion RUNNING for that bot to exist so we could get the data from it.
    // That's not convenient, so instead we'll keep this map of basic version data so we can 
    // save the version info to disk, load it when control panel loads, and use that to decide if there are
    // updates.
    public class VersionContent
    {
        public string unitName { get; set; }  //only used in saving and loading XML
        public PeabodyNetworkingLibrary.VersionData versionData {get; set; }
        [XmlIgnoreAttribute]
        public GameObject gameObject { get; set; }

        private string currentButtonText = "";
        private string newButtonText = "";
        private bool buttonEnable = false;

        private string currentVersionText = "";
        private string newVersionText = "";

        // should only be called in an Update function!!!
        public void UpdateUIElements()
        {
            Button updateButton = gameObject.GetComponentInChildren<Button>();
            Text buttonText = updateButton.GetComponentInChildren<Text>();
            Text[] goTexts = gameObject.GetComponentsInChildren<Text>();
            if (newButtonText != "")
            {
                OutputHelper.OutputLog($"Updating button text for {unitName} to {newButtonText}");
                buttonText.text = newButtonText;
                newButtonText = "";
            }
            if (buttonText.text != currentButtonText)
            {
                currentButtonText = buttonText.text;
            }
            if (buttonEnable != updateButton.enabled)
            {
                if (buttonEnable)
                {
                    OutputHelper.OutputLog($"Enabling update button for {unitName}", OutputHelper.Verbosity.Debug);
                }
                else
                {
                    OutputHelper.OutputLog($"Disabling update button for {unitName}", OutputHelper.Verbosity.Debug);
                }
                updateButton.enabled = buttonEnable;
            }
            if(newVersionText != currentVersionText)
            {
                OutputHelper.OutputLog($"Updating version text for {unitName} to {newVersionText}", OutputHelper.Verbosity.Debug);
                currentVersionText = newVersionText;
                goTexts[0].text = $"{unitName}: {currentVersionText}";
            }

        }

        public string GetUpdateButtonText()
        {
            return currentButtonText;
        }
        public void SetUpdateButtonText(string text)
        {
            newButtonText = text;
        }
        public void EnableUpdateButton(bool enable)
        {
            buttonEnable = enable;
        }

        public string GetVersionStringText()
        {
            return currentVersionText;
        }
        public void SetVersionStringText(string text)
        {
            newVersionText = text;
        }

    }
    public ConcurrentDictionary<string, VersionContent> versionMap = new ConcurrentDictionary<string, VersionContent>();

    public void CreateVersionTextObject(string unitName, PeabodyNetworkingLibrary.VersionData versionData, ComponentStatusContainer container = null)
    {
        Text[] texts;
        GameObject versionGameObject;
        if (versionMap.ContainsKey(unitName) && versionMap[unitName].gameObject != null)
        {
            // already created, probably ready from disk
            versionGameObject = versionMap[unitName].gameObject;            
        }
        else
        {
            // Create the version info in the version list
            versionGameObject = Instantiate(VersionGameObject);
        }
        texts = versionGameObject.GetComponentsInChildren<Text>();
        texts[0].text = $"{unitName}: {versionData.Description}";
        if (container != null)
        {
            OutputHelper.OutputLog($"CVTO called {unitName}.  Container is valid.  Setting texts.", OutputHelper.Verbosity.Trace);
            container.versionText = texts[0];
            container.cloudText = texts[1];
        }
        else
        {
            OutputHelper.OutputLog($"CVTO called {unitName}.  Container is NULL.", OutputHelper.Verbosity.Trace);
        }
        Button updateButton = versionGameObject.GetComponentInChildren<Button>();
        updateButton.onClick.AddListener(delegate { OnButton_UpdateComponent(versionMap[unitName]); });
        updateButton.enabled = false;
        Text buttonText = updateButton.GetComponentInChildren<Text>();
        buttonText.text = " No Update Available ";
        versionGameObject.transform.SetParent(VersionListCanvas.gameObject.transform);
        // don't know if I need any of this, just trying to make the text visible
        versionGameObject.transform.Translate(new Vector3(0, 0, 0));
        foreach (Text t in texts)
        {
           t.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
           t.fontStyle = FontStyle.Normal;
        }

        if (versionMap.ContainsKey(unitName))
        {
            // game object is null, attach
            VersionContent thisVC = versionMap[unitName];
            thisVC.gameObject = versionGameObject;
            versionMap[unitName] = thisVC;
        }
        else
        {
            // creating a new object
            VersionContent vc = new VersionContent();
            vc.gameObject = versionGameObject;
            vc.unitName = unitName;
            vc.versionData = versionData;
            versionMap.TryAdd(unitName, vc);
        }
        // Sort alphabetically
        var sortedMap = versionMap.OrderBy(x => x.Value.unitName);
        int siblingIndex = 0;
        foreach(var item in sortedMap)
        {
            item.Value.gameObject.transform.SetSiblingIndex(siblingIndex++);
        }
    }

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
        var url = SettingsManager.Instance.GetValueWithDefault<string>("Updates", programName+"SoftwareUrl", "");
        if(url == "")
        {
            OutputHelper.OutputLog($"Could not download an update for {programName} because the entry [Updates][{programName+"SoftwareUrl"}] does not exist", OutputHelper.Verbosity.Error);
            versionContent.EnableUpdateButton(false);
            versionContent.SetUpdateButtonText($" Error ");            
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
        vc.SetUpdateButtonText($" {e.ProgressPercentage}% ");
    }
    private void Client_DownloadFileCompleted(AsyncCompletedEventArgs e, VersionContent vc, string filename)
    {
        vc.EnableUpdateButton(false);
        vc.SetUpdateButtonText($" Installing... ");

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
        switch(vc.unitName.Substring(0, pos))
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

        if(update.Length >= minPacketLength)
        {
            int serialLength = System.BitConverter.ToInt32(update, updateOffset);
            OutputHelper.OutputLog($"Serial length is {serialLength}", OutputHelper.Verbosity.Trace);
            updateOffset += sizeof(int);
            string serial = System.BitConverter.ToString(update, updateOffset, serialLength); // We don't currently use this, but it's passed so forwards of this packet know where it came from
            OutputHelper.OutputLog($"Serial Number: {serial}", OutputHelper.Verbosity.Trace);
            updateOffset += serialLength;
            for(int i = 0; i < (int)SOFTWARE.COUNT; i++) 
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
        CancelInvoke("ResetDaemonStateAndRecheck");
        depthDaemonBotsCreated = false;
        int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration","DepthCameraCount", 0, true);
        int portBase = SettingsManager.Instance.GetValueWithDefault("Ports", "DepthPodDaemonPortBase", 14530, true);
        depthGenDaemonBots = new DaemonBot[depthGenNum];
        OutputHelper.OutputLog("Started creating DaemonBots", OutputHelper.Verbosity.Trace);
        for (int i = 0; i < depthGenNum; ++i)
        {
            int podNumber = i + 1;
            string hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "192.168.102.250", true);
            string hostNameOrIP_PN = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250", true);
            bool UsingKNCS = !hostNameOrIP.Equals(hostNameOrIP_PN);
            string dgName = "AKLauncherDaemon_Pod" + (i+1).ToString();  //pod names are 1-based, not 0-based
            string tcpPort = SettingsManager.Instance.GetValueWithDefault("Ports", "NanoDaemonStatusPort", "14501");
            if(UsingKNCS)
            {   
                tcpPort = $"{portBase + podNumber}";
            }
            else
            {
                hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "DepthPodIPBase", "192.168.101.", true) + podNumber.ToString();
            }

            //bot setup start
            depthGenDaemonBots[i] = new DaemonBot(hostNameOrIP, dgName, new ComponentStatusContainer(), tcpPort);
            depthGenDaemonBots[i].SetDepthGenID(i);
            depthGenDaemonBots[i].DepthGenMachineID = i;
            DefaultStatusForwarderSetup(depthGenDaemonBots[i]);
            depthGenDaemonBots[i].DefaultStatusUpdateSetup();
            SetupSocket(depthGenDaemonBots[i], hostNameOrIP);
            
            depthGenDaemonBots[i].componentStatusContainer.componentStatus = new ComponentStatus
            {
                ID = i,
                Name = dgName,
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                IsReady = false,
                IsRunning = false,
                IP = hostNameOrIP,
                Subscribe = true,
                //   CanLaunch = canLaunch // NOT used in this setup
            };

            DaemonBot currBot = depthGenDaemonBots[i];
            int currBotID = i;

            currBot.componentStatusContainer.onTimedOutCallback_permanent += () => 
            {
                //timed out, so no longer running
                currBot.UpdateStateAndUI(SOFTWARE.CAPTURE, SOFTWARE_STATE.TIMEDOUT);
                currBot.UpdateStateAndUI(SOFTWARE.CALIBRATION, SOFTWARE_STATE.TIMEDOUT);
                currBot.UpdateStateAndUI(SOFTWARE.LINUX_DAEMON, SOFTWARE_STATE.TIMEDOUT);
                OutputHelper.OutputLog( $"[{DateTime.UtcNow}] Daemon {currBot.UnitName} timed out. calling callback!", OutputHelper.Verbosity.Warning);

                //status updated
                RespondToDaemonUpdate();
            } ;

            currBot.RegisterUpdateFunction(CPC_STATUS.RUNNING, (byte[] update) => 
            {
                SOFTWARE_STATE[] states = ProcessDaemonStateUpdate(update);
                //ONLY update and make callbacks if the state changed (so we're not doing redudant work)               
                if(currBot.UpdateStates(states))
                {
                    OutputHelper.OutputLog($"[ {DateTime.UtcNow} ] {currBot.UnitName} Daemon UpdateContent: [{states[(int)SOFTWARE.CAPTURE]}][{states[(int)SOFTWARE.CALIBRATION]}]", OutputHelper.Verbosity.Trace);
                    //status updated
                    RespondToDaemonUpdate();
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
                   if(sizeOffset >= update.Length)
                   {
                       OutputHelper.OutputLog($"ERROR.  Not enough data for color ({colorFileLength}) + depth factory data (wanted {sizeOffset})", OutputHelper.Verbosity.Error);
                       return;
                   }
                   int depthFileLength = System.BitConverter.ToInt32(update, sizeOffset);
                   sizeOffset += sizeof(int) + depthFileLength;
                   if (sizeOffset >= update.Length)
                   {
                       OutputHelper.OutputLog($"ERROR.  Not enough data for color ({colorFileLength}), depth  ({depthFileLength}), and extrinsic factory data (wanted {sizeOffset})" ,OutputHelper.Verbosity.Error);
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
                   if(update.Length < expectedPacketSize)
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

            SetVersionDataFromMap(currBot);

            CreateVersionTextObject(currBot.UnitName, currBot.GetVersionData(), currBot.componentStatusContainer);
            currBot.Start();            
        }

        //create fusion/render daemon (3dtm service) monitors
        string daemonPort = SettingsManager.Instance.GetValueWithDefault("Ports/Fusion", "DaemonPort", SettingsManager.Instance.GetValueWithDefault("Ports", "DaemonPort", "14511",true));

        fusionDaemonBot = new DaemonBot(null, "3DTMLauncherService_Fusion", new ComponentStatusContainer(), daemonPort);
        DefaultStatusForwarderSetup(fusionDaemonBot);
        SetupSocket(fusionDaemonBot, SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true));
        fusionDaemonBot.componentStatusContainer.componentStatus = new ComponentStatus
        {
            ID = 20,
            Name = "3DTMLauncherService_Fusion",
            FPS = 0,
            FrameNum = 0,
            ErrState = CPC_ERROR.NONE,
            IsReady = false,
            IsRunning = false,
            IP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true),
            Subscribe = true,
            NewDataReceived = false,            
        };
        SetVersionDataFromMap(fusionDaemonBot);

        CreateVersionTextObject(fusionDaemonBot.UnitName, fusionDaemonBot.GetVersionData(), fusionDaemonBot.componentStatusContainer);


        //Renderer
        daemonPort = SettingsManager.Instance.GetValueWithDefault("Ports/Renderer", "DaemonPort", SettingsManager.Instance.GetValueWithDefault("Ports", "DaemonPort", "14511",true));
        renderDaemonBot = new DaemonBot(null, "3DTMLauncherService_Render", new ComponentStatusContainer(), daemonPort);     
        DefaultStatusForwarderSetup(renderDaemonBot);
        SetupSocket(renderDaemonBot, SettingsManager.Instance.GetValueWithDefault("Network","RendererIPAddress","RENDER", true));
        renderDaemonBot.componentStatusContainer.componentStatus = new ComponentStatus
        {
            ID = 21,
            Name = "3DTMLauncherService_Render",
            FPS = 0,
            FrameNum = 0,
            ErrState = CPC_ERROR.NONE,
            IsReady = false,
            IsRunning = false,
            IP = SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true),
            Subscribe = true,
            NewDataReceived = false            
        };

        SetVersionDataFromMap(renderDaemonBot);

        CreateVersionTextObject(renderDaemonBot.UnitName, renderDaemonBot.GetVersionData(), renderDaemonBot.componentStatusContainer);

        DaemonBot[] windowsDaemonBots = { fusionDaemonBot, renderDaemonBot };

        for(int i =0; i< windowsDaemonBots.Length; ++i)
        {
            DaemonBot currBot = windowsDaemonBots[i];
            currBot.DefaultStatusUpdateSetup();
            currBot.RegisterUpdateFunction(CPC_STATUS.RUNNING, (byte[] update) =>
            {
                SOFTWARE_STATE[] states = ProcessDaemonStateUpdate(update);
                if(currBot.UpdateStates(states))
                {
                    RespondToDaemonUpdate();                    
                }
                for(int j = 0; j < (int)SOFTWARE.COUNT; j++)
                {
                    if(states[j] != SOFTWARE_STATE.UNKNOWN)
                    {
                        currBot.UpdateStateAndUI((SOFTWARE)j, states[j]);
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
                OutputHelper.OutputLog($"{currBot.componentStatusContainer.componentStatus.Name} received valid calibration data.", OutputHelper.Verbosity.Trace);
                currBot.componentStatusContainer.componentStatus.NewDataReceived = true;
            });
            currBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
            {
                UpdateVersionData(currBot, update);
            });
            currBot.RegisterUpdateFunction(CPC_STATUS.SOFTWARE_INSTALLATION_RESULT, (byte[] update) =>
            {
                HandleInstallationResult(currBot, update);
            });


            currBot.componentStatusContainer.onTimedOutCallback_permanent += () =>
            {
                //timed out, so no longer running
                currBot.UpdateStateAndUI(SOFTWARE.WINDOWS_SERVICE, SOFTWARE_STATE.TIMEDOUT);
                OutputHelper.OutputLog($"[{DateTime.UtcNow}] Daemon {currBot.UnitName} timed out. calling callback!", OutputHelper.Verbosity.Warning);
            };
            currBot.Start();
        }

        InvokeRepeating("ResetDaemonStateAndRecheck", 0, cDaemonStatusCheckInterval);
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
        //status updated
        if (onDaemonStatusUpdateCallback != null)
        {
            OutputHelper.OutputLog($"Invoking the onDaemonStatusUpdateCallback in response to a daemon update", OutputHelper.Verbosity.Debug);
            onDaemonStatusUpdateCallback.Invoke();
        }
        else
        {
            OutputHelper.OutputLog($"NOT invoking onDaemonStatusUpdateCallback in response to a daemon update because it is null.  Calling CheckBotManagerReady({lastStartType}) instead.", OutputHelper.Verbosity.Debug);
            CheckBotManagerReady(lastStartType);
        }
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

    public delegate SOFTWARE_STATE GetBotStateFunction(DaemonBot bot);
    public DAEMON_GROUP_STATE GetDaemonGroupState(GetBotStateFunction GetBotStateFunc, ref bool readyToBeLaunched)
    {
        int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration", "DepthCameraCount", 0, true);

        //check for errors: early outs
        for (int daemonIndex = 0; daemonIndex < depthGenNum; ++daemonIndex)
        {
            //check for error
            if (GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.LOCKED  ||
                GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.STOPPED ||
                GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.TIMEDOUT )
            {
                readyToBeLaunched = false;
                return DAEMON_GROUP_STATE.ERROR;
            }
            //check if we haven't heard from this daemon
            if(GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.UNKNOWN)
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
            else if(GetBotStateFunc(depthGenDaemonBots[daemonIndex]) == SOFTWARE_STATE.NOT_RUNNING)
            {
                hasNotRunning = true;
            }
            else
            {
                //RUNNING
                hasRunning = true;
            }
        }
        if(hasNotRunning)
        {
            if(hasRunning)
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
            if(hasRunning)
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
        return GetDaemonGroupState((DaemonBot db) => { return db.softwareStates[(int)SOFTWARE.CALIBRATION]; }, ref allDaemonBotsReadyToBeLaunched_Calib);
    }

    public DAEMON_GROUP_STATE GetDaemonGroupState_Capture()
    {
        return GetDaemonGroupState((DaemonBot db) => { return db.softwareStates[(int)SOFTWARE.CAPTURE]; }, ref allDaemonBotsReadyToBeLaunched_Capture);
    }

    public DAEMON_GROUP_STATE GetDaemonGroupState_BGCap()
    {
        return GetDaemonGroupState((DaemonBot db) => { return db.softwareStates[(int)SOFTWARE.BACKGROUND_CAPTURE]; }, ref allDaemonBotsReadyToBeLaunched_BGCap);
    }
    public void UpdateDaemonBotsReadyStatus(SOFTWARE requestType)
    {
        OutputHelper.OutputLog("Call UpdateDaemonBotsReadyStatus", OutputHelper.Verbosity.Debug);

        DAEMON_GROUP_STATE captureGroupState = GetDaemonGroupState_Capture();
        DAEMON_GROUP_STATE calibGroupState = GetDaemonGroupState_Calib();
        DAEMON_GROUP_STATE bgcapGroupState= GetDaemonGroupState_BGCap();

        bool hasErrors = 
            captureGroupState == DAEMON_GROUP_STATE.ERROR || captureGroupState == DAEMON_GROUP_STATE.UNKNOWN ||
            calibGroupState == DAEMON_GROUP_STATE.ERROR || calibGroupState == DAEMON_GROUP_STATE.UNKNOWN ||
            bgcapGroupState == DAEMON_GROUP_STATE.ERROR || bgcapGroupState == DAEMON_GROUP_STATE.UNKNOWN;
        if (hasErrors ||
            captureGroupState == DAEMON_GROUP_STATE.NOT_RUNNING || 
            calibGroupState == DAEMON_GROUP_STATE.NOT_RUNNING || 
            bgcapGroupState == DAEMON_GROUP_STATE.NOT_RUNNING)
        {          
          
            if ((!allDaemonBotsReadyToBeLaunched_Capture && !allDaemonBotsReadyToBeLaunched_Calib && !allDaemonBotsReadyToBeLaunched_BGCap) || hasErrors)
            {
                //not ready to launch a new session
                OutputHelper.OutputLog("UpdateDaemonBotsReadyStatus: " + BotManager.Instance.GetDaemonNotReadyText(requestType), OutputHelper.Verbosity.Debug);
                DialogManager.Instance.UpdateMessage(GetDaemonNotReadyText(requestType), DialogManager.Instance.WarningMessage);
            }
            else
            {                
                OutputHelper.OutputLog($"All Daemons ready: [Capture:{captureGroupState}][Calib:{calibGroupState}][BG:{bgcapGroupState}]", OutputHelper.Verbosity.Debug);
                DialogManager.Instance.UpdateMessage($"All Daemons ready: [Capture:{captureGroupState}][Calib:{calibGroupState}][BG:{bgcapGroupState}]", DialogManager.Instance.NormalMessage);
            }

        }
    }

    public void ResetDaemonInvoke()
    {
        mainThreadCallbacks.Enqueue(() =>
        {
            CancelInvoke("ResetDaemonStateAndRecheck");
            InvokeRepeating("ResetDaemonStateAndRecheck", cDaemonStatusCheckInterval, cDaemonStatusCheckInterval);
        });
    }
    // parameter-less method so we can InvokeRepeat on it
    public void ResetDaemonStateAndRecheck()
    {
        OutputHelper.OutputLog("Resetting daemon stats", OutputHelper.Verbosity.Trace);
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
            DialogManager.Instance.UpdateMessage("Waiting for cleanup to finish.", DialogManager.Instance.WarningMessage);
            return new Tuple<bool, DAEMON_GROUP_STATE>(false, DAEMON_GROUP_STATE.UNKNOWN);
        }
        else if (!BotManager.Instance.depthDaemonBotsCreated)
        {
            OutputHelper.OutputLog("waiting to create bots", OutputHelper.Verbosity.Debug);
            DialogManager.Instance.UpdateMessage("Waiting for DaemonBot Creation.", DialogManager.Instance.WarningMessage);
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
        if(depthGenNum <=0)
        {
            return false;
        }
        if (depthGenStatusBot != null)
        {
            for (int i = 0; i < depthGenStatusBot.Length; ++i)
            {
                OutputHelper.OutputLog($"CreateDepthStatusBots called. depthGenStatusBot array is not null.  Stopping existing bots.");
                depthGenStatusBot[i].Stop();
            }
        }
        depthGenStatusBot = new StatusBot[depthGenNum];
        runningDGBots = 0;
        for (int i = 0; i < depthGenNum; ++i)
        {
            string dgName = "AzureKinectNanoToFusion_Pod" + (i+1).ToString();
            // if the IP for fusion and fusion pod network are the same, use IP addresses, not ports (no need for KNCS)
            string hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "192.168.102.250", true);
            string hostNameOrIPPods = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250", true);
            bool UsingKNCS = !hostNameOrIP.Equals(hostNameOrIPPods);
            int dgPort = SettingsManager.Instance.GetValueWithDefault("Ports", "NanoApplicationStatusPort", 14502, true);
            if(UsingKNCS)
            {
                dgPort = SettingsManager.Instance.GetValueWithDefault("Ports", "DepthPodApplicationStatusPortBase", 14570, true) + (i+1);
            }
            else
            {
                hostNameOrIP = SettingsManager.Instance.GetValueWithDefault("Network", "DepthPodIPBase", "192.168.101.", true) + (i+1).ToString();
            }
            
            StatusBot currBot = new StatusBot(hostNameOrIP, dgName, new ComponentStatusContainer(), dgPort.ToString());
            currBot.SetDepthGenID(i);
            currBot.DepthGenMachineID = i;
            currBot.DefaultStatusUpdateSetup();
            DefaultStatusForwarderSetup(currBot);
            SetupSocket(currBot, hostNameOrIP);

            currBot.componentStatusContainer.componentStatus = new ComponentStatus
            {
                ID = i,
                Name = dgName,
                FPS = 0,
                FrameNum = 0,
                ErrState = CPC_ERROR.NONE,
                IsReady = false,
                IsRunning = false,
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

            CreateVersionTextObject(currBot.UnitName, currBot.GetVersionData(), currBot.componentStatusContainer);

            depthGenStatusBot[i] = currBot;
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
        if(calibrationSoftwareStatusBot != null)
        {
            calibrationSoftwareStatusBot.Stop();
        }

        //CalibrationSoftware
        calibrationSoftwareStatusBot = new StatusBot(null, "CalibrationSoftware", new ComponentStatusContainer());

        calibrationSoftwareStatusBot.DefaultStatusUpdateSetup();
        
        //If a specific IP address is not set for the CalibrationSoftware machine, it is usually located in the same machine as Fusion
        String fusionIPaddress = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION",true);
        SetupSocket(calibrationSoftwareStatusBot, SettingsManager.Instance.GetValueWithDefault("Network","CalibrationSoftwareIPAddress", fusionIPaddress,true));
        calibrationSoftwareStatusBot.componentStatusContainer.componentStatus = new ComponentStatus
        {
            ID = 12, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
            Name = "CalibrationSoftware",
            FPS = 0,
            FrameNum = 0,
            ErrState = CPC_ERROR.NONE,
            IsReady = false,
            IsRunning = false,
            //If a specific IP address is not set for the CalibrationSoftware machine, it is usually located in the same machine as Fusion
            IP = SettingsManager.Instance.GetValueWithDefault("Network", "CalibrationSoftwareIPAddress", fusionIPaddress, true),
            Subscribe = true,            
            //   CanLaunch = canLaunch // NOT used in this setup
        };

        calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
        {
            UpdateVersionData(calibrationSoftwareStatusBot, update);
        });

        SetVersionDataFromMap(calibrationSoftwareStatusBot);

        CreateVersionTextObject(calibrationSoftwareStatusBot.UnitName, calibrationSoftwareStatusBot.GetVersionData(), calibrationSoftwareStatusBot.componentStatusContainer);

        return true;

    }

    /// <summary>
    /// Call this function to create the bots and connect up. Callbacks and sequenced tasks have to be setup separately by setting the bots' componentStatusContainer.onRunningCallback method.
    /// </summary>
    public bool CreateStatusBotsBasedOnConfig()
    {
        Tuple<bool, DAEMON_GROUP_STATE> botReadyStatus = CheckBotManagerReady(SOFTWARE.CAPTURE);
        if (!botReadyStatus.Item1 || botReadyStatus.Item2 != DAEMON_GROUP_STATE.NOT_RUNNING)
        {
            return false;
        }

        //DepthGen
        CreateDepthStatusBots();

        //Fusion
        fusionStatusBot = new StatusBot(null, "Fusion", new ComponentStatusContainer());
        fusionStatusBot.DefaultStatusUpdateSetup();
        DefaultStatusForwarderSetup(fusionStatusBot);

        SetupSocket(fusionStatusBot, SettingsManager.Instance.GetValueWithDefault("Network","FusionIPAddress","FUSION",true));
        fusionStatusBot.componentStatusContainer.componentStatus = new ComponentStatus
        {
            ID = 10, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
            Name = "Fusion",
            FPS = 0,
            FrameNum = 0,
            ErrState = CPC_ERROR.NONE,
            IsReady = false,
            IsRunning = false,
            IP = SettingsManager.Instance.GetValueWithDefault("Network", "FusionIPAddress", "FUSION", true),
            Subscribe = true,            
            //   CanLaunch = canLaunch // NOT used in this setup
        };

        fusionStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
        {
            UpdateVersionData(fusionStatusBot, update);
        });
        SetVersionDataFromMap(fusionStatusBot);

        CreateVersionTextObject(fusionStatusBot.UnitName, fusionStatusBot.GetVersionData(), fusionStatusBot.componentStatusContainer);
        //Renderer
        renderStatusBot = new StatusBot(null, "Render", new ComponentStatusContainer());
        renderStatusBot.DefaultStatusUpdateSetup();
        DefaultStatusForwarderSetup(renderStatusBot);

        SetupSocket(renderStatusBot, SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true));

        renderStatusBot.RegisterUpdateFunction(CPC_STATUS.TOGGLE_FUSION_HIGH_RESOLUTION,
            delegate (byte[] update)
            {
                //packet that needs to be forwarded to Fusion
                if(update.Length <=0)
                {
                    OutputHelper.OutputLog("TOGGLE_FUSION_HIGH_RESOLUTION packet missing data!", OutputHelper.Verbosity.Error);
                    return;
                }
                byte[] fixedPacket = new byte[update.Length - 1];
                Buffer.BlockCopy(update, 1, fixedPacket, 0, update.Length - 1);
                 DaemonPublisher.PublishEvent(CONTROL_PANEL_EVENT.TOGGLE_FUSION_HIGH_RESOLUTION,fixedPacket);
            });
        renderStatusBot.componentStatusContainer.componentStatus = new ComponentStatus
        {
            ID = 11, //TODO: increase this number to allow more than 10 depth generators to be used. ADO bug #9687
            Name = "Render",
            FPS = 0,
            FrameNum = 0,
            ErrState = CPC_ERROR.NONE,
            IsReady = false,
            IsRunning = false,
            IP = SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", "RENDER", true),
            Subscribe = true,
            //   CanLaunch = canLaunch // NOT used in this setup
        };
        renderStatusBot.RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, (byte[] update) =>
        {
            UpdateVersionData(renderStatusBot, update);
        });
        SetVersionDataFromMap(renderStatusBot);
        CreateVersionTextObject(renderStatusBot.UnitName, renderStatusBot.GetVersionData(), renderStatusBot.componentStatusContainer);

        // request the bots give their latest version
        BroadcastEventOnce(CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
        return true;
    }

    protected void SetVersionDataFromMap(StatusBot bot)
    {
        if (versionMap.ContainsKey(bot.UnitName))
        {
            // set the version data in the bot to whatever we read from disk.
            bot.SetVersionData(versionMap[bot.UnitName].versionData);
        }
        else
        {
            OutputHelper.OutputLog($"Version Map doesn't contain an entry for [{bot.UnitName}]", OutputHelper.Verbosity.Error);
            foreach(var vc in versionMap)
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
        DialogManager.Instance.UpdateMessage("Cleaning Up.", DialogManager.Instance.WarningMessage);

        //kill processes
        CleanupMachines();
        
        if (cb != null)
        {
            // clean previous cleanup callback
            onCleanedup = null;
            onCleanedup += cb;
        }


        // stop outstanding broadcasts
        StopBroadcastCoroutine();
        //wait and check until all machines report back as stopped
        IEnumerator stopCheckCoroutine = CheckUntilMachinesStop(cleanDaemonBots);
        if(gameObject.activeInHierarchy)
        {
            StartCoroutine(stopCheckCoroutine);
        }
        else
        {
            CheckUntilMachinesStop(cleanDaemonBots, false).MoveNext();
        }    
    }       


    public void CleanupDaemonBots()
    {
        CancelInvoke("ResetDaemonStateAndRecheck");
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
        
        if (depthGenStatusBot != null)
        {
            for (int i = 0; i < depthGenStatusBot.Length; ++i)
            {
                stillCleaning = stillCleaning || depthGenStatusBot[i].IsAlive(checkThreadAlive);
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

        if (depthGenStatusBot != null)
        {
            for (int i = 0; i < depthGenStatusBot.Length; ++i)
            {
                depthGenStatusBot[i].Stop();
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
        if(calibrationSoftwareStatusBot != null)
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


        if (this.gameObject.activeInHierarchy)
        {
            //otherwise cleanup happens after threads fully stop
            StartCoroutine(CheckUntilCleanupFinishes());
        }
        else
        {
            //just cleanup if we're exiting completely. no coroutine waits
            bool stillCleaning = true;
            while (stillCleaning)
            {
                stillCleaning = AreMachinesStillCleaning(true, true);
            }            
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
            if(shouldReturn)
            {
                yield return new WaitForSeconds(0.2f);
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
        while(stillCleaning)
        {
            stillCleaning = AreMachinesStillCleaning(false, true);
            yield return new WaitForSeconds(0.2f);
        }

        //NOTE: can't do a full netMQcleanup here, since it cleans everything netmq related (including daemon bots which aren't cleaned here!)  
        OutputHelper.OutputLog("statusBots cleanedup!", OutputHelper.Verbosity.Debug);
        IsCleaningUp = false; // set this before the onCleanedup callback, since they might depend on this flag
        
        onCleanedup?.Invoke();
        onDaemonStatusUpdateCallback?.Invoke();
    }
    #endregion

    private void InitializePublisherAndStart()
    {
        DaemonPublisher = new EventBot(""); //Invalid publish address, but wiill be reInit'ed with correct ones in constructor
        DaemonPublisher.SetUseMulticast(SettingsManager.Instance.GetValueWithDefault<bool>("Network", "UseMultiCast", false, false));
        DaemonPublisher.SetUseTCP(SettingsManager.Instance.GetValueWithDefault<bool>("Network","UseTCP", true, false));
        SetInitialEventPorts(DaemonPublisher);

        DaemonPublisher.Start();
    }


    public void BroadcastEventOnce(CONTROL_PANEL_EVENT eventType)
    {
        //lazy init
        if(DaemonPublisher == null)
        {
            InitializePublisherAndStart();
        }
        DaemonPublisher.PublishEvent(eventType);
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

    public void StopBroadcastCoroutine()
    {
        if (broadcastCoroutine != null)
        {
            StopCoroutine(broadcastCoroutine);
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

    public void BroadcastStartUntilAllDGStart(SOFTWARE startType, byte[] eventData = null)
    {
        mainThreadCallbacks.Enqueue(() =>
        {
            StopBroadcastCoroutine();
            if (startType != SOFTWARE.SYSTEM_START)
            {
                lastStartType = startType;
            }
            broadcastCoroutine = BroadcastCoroutineMethod(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, startType, eventData);
            StartCoroutine(broadcastCoroutine);
        });
    }

    private IEnumerator BroadcastCoroutineMethod(CONTROL_PANEL_EVENT eventType, SOFTWARE startType, byte[] eventData = null)
    {
        byte[] dataArray = null;
        if(startType != SOFTWARE.SYSTEM_START)
        {
            if (eventData != null)
            {
                dataArray = new byte[eventData.Length + 1];

                // Coying startType into dataArray
                dataArray[0] = (byte)startType;

                // Copying payload
                eventData.CopyTo(dataArray, 1);
            }
            else
            {
                dataArray = new byte[1];

                // Coying startType into dataArray
                dataArray[0] = (byte)startType;
            }
        }

        while (runningDGBots < depthGenStatusBot.Length)
        {
            if(startType != SOFTWARE.SYSTEM_START)
            {
                BroadcastEventOnce(eventType, dataArray);
            }
            else
            {
                //no real data to send 
                BroadcastEventOnce(eventType, new byte[0]);
            }
            OutputHelper.OutputLog(String.Format("Broadcasting {0}-{1} for DGs: {2}/{3}",eventType, startType,  
                BotManager.Instance.runningDGBots, BotManager.Instance.depthGenStatusBot.Length), OutputHelper.Verbosity.Debug);
            yield return new WaitForSeconds(3.0f);
        }
    }

    public bool CheckAllCalibHaveSameFrameNumber(int targetFrameNumber)
    {
        if(depthGenStatusBot == null)
        {
            return false;
        }

        for(int i = 0; i < depthGenStatusBot.Length; ++i)
        {
            if(depthGenStatusBot[i] != null && depthGenStatusBot[i].componentStatusContainer != null && depthGenStatusBot[i].componentStatusContainer.frameNumberText != null)
            {
                int frameNum = depthGenStatusBot[i].componentStatusContainer.componentStatus.FrameNum;
                if (frameNum < targetFrameNumber)
                {
                    OutputHelper.OutputLog("CheckAllCalibHaveSameFrameNumber failed for " +i + ": got=" + frameNum + ";target=" + targetFrameNumber, OutputHelper.Verbosity.Trace);
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
        // only cleanup netmq completely here
        //full netmq cleanup after everything stopped properly
        try{
            BotManager.Instance.CleanUpStatusBots(null, true);
            NetMQ.NetMQConfig.Cleanup(false);
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
            if(messageLength > update.Length - (sizeof(char)+sizeof(char)+sizeof(char))) // byte 0 is packet type, byte 1 is software, byte 2 is success (bool) bytes 3+ are messagelength + message
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
        switch(updatedSoftware)
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
            vc.SetUpdateButtonText(text);
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
            vc.versionData = currBot.GetVersionData();
            versionMap[currBot.UnitName] = vc;
            // Update the UI text box if it's been created
            vc.SetVersionStringText(vc.versionData.Description);
        }
        else
        {
            OutputHelper.OutputLog($"{currBot.UnitName} not found in the version map:", OutputHelper.Verbosity.Trace);
            foreach(var vc in versionMap)
            {
                OutputHelper.OutputLog($"[{vc.Key}] = [{vc.Value}]", OutputHelper.Verbosity.Trace);
            }
        }
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
            foreach(VersionContent vc in entries)
            {
                OutputHelper.OutputLog($"Read {vc.unitName}: {vc.versionData.Description} from disk", OutputHelper.Verbosity.Info);
                if(vc.versionData.Description == "" || vc.versionData.Description == null)
                {
                    OutputHelper.OutputLog($"Version data's description field was blank.  So creating a new (unknown) version data object");
                    PeabodyNetworkingLibrary.VersionData vd = new VersionData();
                    vd.Description = "Unknown Version";
                    vc.versionData = vd;
                }

                CreateVersionTextObject(vc.unitName, vc.versionData);
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
}
