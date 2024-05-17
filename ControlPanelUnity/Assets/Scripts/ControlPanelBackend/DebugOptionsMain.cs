using ControlPanel;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
using PeabodyNetworkingLibrary;
using System.Runtime.Remoting;
using System.Text;
using System.Diagnostics;
using System.Net;
using System.Threading.Tasks;
using System.ComponentModel;
using System.Runtime.InteropServices;

public class DebugOptionsMain : TabController
{
    public Button checkForUpdatesButton;

    [Header("Events")]
    public UpdateUITextEvent updateUITextEvent;
    public UpdateUIInteractableEvent updateUIInteractableEvent;

    [Header("UI Interaction")]    
    public GameObject MessageCanvas;
    public Text broadcastCustomEventText;

    public int broadcastButtonState = 0;

    public override void OnTabDisable()
    {
        //re-enable the message canvas when we leave this tab
        MessageCanvas.SetActive(true);
    }

    public override void OnTabEnable()
    {
        //dont show message here
        MessageCanvas.SetActive(false);

        // Assign a updateUITextEvent to any existing bots that don't already have one
        foreach (DaemonBot bot in BotManager.Instance.depthGenDaemonBots)
        {
            if(bot.componentStatusContainer.updateTextEvent == null)
            {
                bot.componentStatusContainer.updateTextEvent = updateUITextEvent;
            }
        }
        if (BotManager.Instance != null && BotManager.Instance.depthGenStatusBot != null)
        {
            foreach (StatusBot bot in BotManager.Instance.depthGenStatusBot)
            {
                if (bot.componentStatusContainer.updateTextEvent == null)
                {
                    bot.componentStatusContainer.updateTextEvent = updateUITextEvent;
                }
            }
        }
        if(BotManager.Instance != null && BotManager.Instance.fusionDaemonBot != null && BotManager.Instance.fusionDaemonBot.componentStatusContainer.updateTextEvent == null)
        {
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.updateTextEvent = updateUITextEvent;
        }
        if (BotManager.Instance != null && BotManager.Instance.fusionStatusBot != null && BotManager.Instance.fusionStatusBot.componentStatusContainer.updateTextEvent == null)
        {
            BotManager.Instance.fusionStatusBot.componentStatusContainer.updateTextEvent = updateUITextEvent;
        }
        if (BotManager.Instance != null && BotManager.Instance.renderDaemonBot != null && BotManager.Instance.renderDaemonBot.componentStatusContainer.updateTextEvent == null)
        {
            BotManager.Instance.renderDaemonBot.componentStatusContainer.updateTextEvent = updateUITextEvent;
        }
        if (BotManager.Instance != null && BotManager.Instance.renderStatusBot != null && BotManager.Instance.renderStatusBot.componentStatusContainer.updateTextEvent == null)
        {
            BotManager.Instance.renderStatusBot.componentStatusContainer.updateTextEvent = updateUITextEvent;
        }
    }
    
    private int currentTestState = 0;
    private CONTROL_PANEL_EVENT[] events = {CONTROL_PANEL_EVENT.SYSTEM_IDLE, 
        CONTROL_PANEL_EVENT.BROADCAST_MODE_RUNNING, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_STARTED, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_PROCESSING, 
        CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT, 
        CONTROL_PANEL_EVENT.BACKGROUND_CAPTURE_STARTED, 
        CONTROL_PANEL_EVENT.BACKGROUND_CAPTURE_COUNT};
    public void OnButton_TransmitStateChange()
    {
        string serial = "000462291912";
        var dataString = new byte[sizeof(int) + serial.Length + sizeof(int)];
        Buffer.BlockCopy(BitConverter.GetBytes(serial.Length), 0, dataString, 0, sizeof(int));
        Buffer.BlockCopy(Encoding.Default.GetBytes(serial), 0, dataString, sizeof(int) ,serial.Length);
        if(events[currentTestState] == CONTROL_PANEL_EVENT.BACKGROUND_CAPTURE_COUNT)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(5), 0, dataString, sizeof(int)+serial.Length, sizeof(int));
        }
        else if(events[currentTestState] == CONTROL_PANEL_EVENT.SYSTEM_IDLE || events[currentTestState] == CONTROL_PANEL_EVENT.BROADCAST_MODE_RUNNING || events[currentTestState] == CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_PROCESSING || events[currentTestState] == CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT)
        {
            dataString = new byte[0];
        }
        BotManager.Instance.BroadcastEventOnce(events[currentTestState], dataString);
        currentTestState++;
    }
    public void OnButton_PublishStartFusion()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
    }

    public void OnButton_PublishStartRender()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
    }

    public void OnButton_PublishStopFusion()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
    }

    public void OnButton_PublishStopRender()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
    }

    public void OnButton_KillPods()
    {
        BotManager.Instance.CleanUpStatusBots();
    }

    public void OnButton_CollectLogs()
    {
        string logCollectionFolderBase = SettingsManager.Instance.GetValueWithDefault("Debug", "LogCollectionFolder", Path.GetTempPath());
        string logCollectionFolder = "\\" + DateTime.Now.ToString("yyyy-MM-dd_H-mm-ss");
        // Create a new log collection folder
        // Check if the log collection folder in the config file exists
        if (!Directory.Exists(logCollectionFolderBase))
        {
            logCollectionFolder = Path.GetTempPath() + logCollectionFolder;
        }
        else
        {
            logCollectionFolder = logCollectionFolderBase + logCollectionFolder;
        }
        OutputHelper.OutputLog($"Requesting logs and collecting in {logCollectionFolder}");
        try
        {
            Directory.CreateDirectory(logCollectionFolder);
        }
        catch (Exception e)
        {
            OutputHelper.OutputLog($"Couldn't create the directory {logCollectionFolder}: {e.Message}.  Aborting log collection.");
            return;
        }
        // Update the bots with the new directory
        BotManager.Instance.fusionDaemonBot.SetLogCollectionPath(logCollectionFolder);
        BotManager.Instance.renderDaemonBot.SetLogCollectionPath(logCollectionFolder);
        foreach (DaemonBot bot in BotManager.Instance.depthGenDaemonBots)
        {
            bot.SetLogCollectionPath(logCollectionFolder);
        }
        // Request logs from everyone
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.LOG_COLLECTION_REQUESTED);
        // Copy control panel logs there as well
        Directory.CreateDirectory(logCollectionFolder + "/ControlPanel");
        string logFilename = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\..\\LocalLow\\" + Application.companyName + "\\" + Application.productName + "\\Player.log";
        try
        {
            File.Copy(logFilename, logCollectionFolder + "/ControlPanel/Player.log");
        }
        catch (FileNotFoundException) { }

        logFilename = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + "\\..\\LocalLow\\" + Application.companyName + "\\" + Application.productName + "\\Player-prev.log";

        try
        {
            File.Copy(logFilename, logCollectionFolder + "/ControlPanel/Player-prev.log");
        }
        catch (FileNotFoundException) { }

        // Copy any calibration logs as well
        string logPath = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData) + SettingsManager.Instance.GetValueWithDefault("Calibration", "LogRelativePath", "\\..\\LocalLow\\Microsoft Research\\CalibrationLogs");
        if (Directory.Exists(logPath))
        {
            foreach (string file in Directory.GetFiles(logPath))
            {
                try
                {
                    File.Copy(file, logCollectionFolder + "/" + Path.GetFileName(file));
                }
                catch (Exception e) 
                {
                    OutputHelper.OutputLog($"Couldn't copy {file} to {logCollectionFolder}/{file}.  Error: {e.Message}");
                }
            }
        }
    }

    public void OnButton_SendSystemConfiguration()
    {
        // Reload the config in case it has changed
        SettingsManager.Instance.LoadConfigFromDisk();
        string configurationFileContents = SettingsManager.Instance.config.StringRepresentation;
        byte[] packet = new byte[configurationFileContents.Length + sizeof(int)];
        byte[] lengthInBytes = BitConverter.GetBytes(configurationFileContents.Length);
        Buffer.BlockCopy(lengthInBytes, 0, packet, 0, sizeof(int));
        Buffer.BlockCopy(Encoding.Default.GetBytes(configurationFileContents), 0, packet, sizeof(int), configurationFileContents.Length);
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.SYSTEM_CONFIGURATION_UPDATE, packet);
    }

    public void OnButton_RequestAKFactoryCalib()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.KINECT_FACTORY_CALIBRATION_DATA_REQUESTED);
    }

    public void OnButton_RequestBuildVersions()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED);
    }

    private List<BotManager.VersionContent> versionContentsToCheck = new List<BotManager.VersionContent>();
    // Gets the latest cloud versions of all software
    public void OnButton_CheckForUpdates()
    {
        checkForUpdatesButton.enabled = false;
        versionContentsToCheck = new List<BotManager.VersionContent>(BotManager.Instance.versionMap.Values);
        UnityEngine.Debug.Log($"List of programs to check ({versionContentsToCheck.Count}):");
        foreach(var item in versionContentsToCheck)
        {
            UnityEngine.Debug.Log($"{item.unitName}");
        }
        BotManager.VersionContent vc = versionContentsToCheck[0];
        versionContentsToCheck.RemoveAt(0);
        DownloadLatestVersionDataForComponent(vc);
    }

    public void OnButton_BroadcastCustomEvent()
    {
        byte[] packet = new byte[sizeof(int) + sizeof(int)];
        Buffer.BlockCopy(BitConverter.GetBytes((int)SOFTWARE.CAPTURE), 0, packet, 0, sizeof(int));
        switch(broadcastButtonState)
        {
            case 0:            
                Buffer.BlockCopy(BitConverter.GetBytes(5), 0, packet, sizeof(int), sizeof(int));
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.SPEED_REQUEST, packet);

                updateUITextEvent.Raise(broadcastCustomEventText, "Broadcast Normal Speed Request to Pods");
                broadcastButtonState++;
                break;
            case 1:
                Buffer.BlockCopy(BitConverter.GetBytes(-1), 0, packet, sizeof(int), sizeof(int));
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.SPEED_REQUEST, packet);

                updateUITextEvent.Raise(broadcastCustomEventText, "Broadcast Slowdown 0 to Pods");
                broadcastButtonState++;
            break;
            case 2:
                Buffer.BlockCopy(BitConverter.GetBytes(0), 0, packet, sizeof(int), sizeof(int));
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.SPEED_REQUEST, packet);

                updateUITextEvent.Raise(broadcastCustomEventText, "Broadcast Slowdown 5 to Pods");
                broadcastButtonState = 0;
            break;
        }
    }

    private void Client_DownloadProgressChanged(object sender, DownloadProgressChangedEventArgs e)
    {
    }

    private void DownloadLatestVersionDataForComponent(BotManager.VersionContent vc)
    {
        WebClient versionDownloadClient = new WebClient();
        int pos = vc.unitName.IndexOf("_");
        pos = pos > 0 ? pos : vc.unitName.Length;
        var programName = vc.unitName.Substring(0, pos);
        var url = SettingsManager.Instance.GetValueWithDefault<string>("Updates", programName+"VersionUrl", "");

        if(url == "")
        {
            OutputHelper.OutputLog($"Can't find a version URL for {programName}", OutputHelper.Verbosity.Error);
            // continue to the next
            if (versionContentsToCheck.Count > 0)
            {
                BotManager.VersionContent nextVC = versionContentsToCheck[0];
                versionContentsToCheck.RemoveAt(0);
                UnityEngine.Debug.Log($"Starting next check for {nextVC.unitName}.  {versionContentsToCheck.Count} remaining.");
                DownloadLatestVersionDataForComponent(nextVC);
            }
            return;
        }
        // Got a URL, start the download of the .version file
        var versionFilename = Path.GetTempFileName();
        UnityEngine.Debug.Log($"Download {vc.unitName} version from {url}");
        // set the new handler for this version content object
        versionDownloadClient.DownloadProgressChanged += new DownloadProgressChangedEventHandler(Client_DownloadProgressChanged);
        versionDownloadClient.DownloadFileCompleted += (sender, eventArgs) => 
        { 
            Client_DownloadFileCompleted(eventArgs, vc, versionFilename); 
        };
        
        try
        {
            vc.EnableUpdateButton(false);
            vc.SetUpdateButtonText(" Checking... ");

            versionDownloadClient.DownloadFileAsync(new Uri(url), versionFilename);
        }
        catch (System.Net.WebException e)
        {
            UnityEngine.Debug.Log($"Couldn't connect to the web.  Check your internet connection! Error: {e.Message}");
            return;
        }
    }

    private void Client_DownloadFileCompleted(AsyncCompletedEventArgs e, BotManager.VersionContent vc, string filename)
    {
        if (!e.Cancelled && e.Error == null)
        {
            using (StreamReader sr = new StreamReader(filename))
            {
                string versionString = sr.ReadToEnd();
                UnityEngine.Debug.Log($"Download complete.  Version string for {vc.unitName} from {filename}: {versionString}");
                if (versionString == "")
                    return;
                try
                {
                    PeabodyNetworkingLibrary.VersionData cloudVersion = PeabodyNetworkingLibrary.Utility.ReadVersionDataFromDescrptionString(versionString);
                    // Update the text:
                    GameObject versionGameObject = vc.gameObject;
                    Text[] texts = versionGameObject.GetComponentsInChildren<Text>();
                    texts[1].text = cloudVersion.Description.TrimEnd(Environment.NewLine.ToCharArray());
                    // If the version there is > the local version, download
                    int majorDiff = cloudVersion.Major - vc.versionData.Major;
                    int minorDiff = cloudVersion.Minor - vc.versionData.Minor;
                    int patchDiff = cloudVersion.Patch - vc.versionData.Patch;
                    int commitDiff = cloudVersion.Commits - vc.versionData.Commits;
                    bool sameBranch = cloudVersion.BranchName.Equals(vc.versionData.BranchName);

                    vc.EnableUpdateButton(false);
                    vc.SetUpdateButtonText(" No Update Available ");
                    if (sameBranch &&
                        majorDiff > 0 ||
                        (majorDiff == 0 && minorDiff > 0) ||
                        (majorDiff == 0 && minorDiff == 0 && patchDiff > 0) ||
                        (majorDiff == 0 && minorDiff == 0 && patchDiff == 0 && commitDiff > 0))
                    {
                        UnityEngine.Debug.Log($"Cloud Version {cloudVersion.Description} is newer than {vc.versionData.Description}.  Update available!");

                        vc.EnableUpdateButton(true);
                        vc.SetUpdateButtonText(" Update ");
                    }
                }
                catch (MissingReferenceException ex)
                {
                    UnityEngine.Debug.Log($"Couldn't parse the downloaded version file: {ex.Message}");
                }
            }
        }
        // Start the next download
        if (versionContentsToCheck.Count > 0)
        {
            BotManager.VersionContent nextVC = versionContentsToCheck[0];
            versionContentsToCheck.RemoveAt(0);
            UnityEngine.Debug.Log($"Starting next check for {nextVC.unitName}.  {versionContentsToCheck.Count} remaining.");
            DownloadLatestVersionDataForComponent(nextVC);
        }
        else
        {
            checkForUpdatesButton.enabled = true;
        }
    }

    public void OnButton_OpenLogsFolder()
    {
        string logCollectionFolderBase = SettingsManager.Instance.GetValueWithDefault("Debug", "LogCollectionFolder", Path.GetTempPath());
        // Check if the log collection folder in the config file exists
        if (!Directory.Exists(logCollectionFolderBase))
        {
            logCollectionFolderBase = Path.GetTempPath();
        }

        ProcessStartInfo info = new ProcessStartInfo(@"C:\windows\explorer.exe");
        info.Arguments = "\"" + logCollectionFolderBase + "\"";
        OutputHelper.OutputLog($"Opening {info.Arguments}");
        Process.Start(info);
    }

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public override RequiredSetting[] GetAllRequiredSettings()
    {
        //doesn't require any settings
        return new RequiredSetting[0];
    }

}
