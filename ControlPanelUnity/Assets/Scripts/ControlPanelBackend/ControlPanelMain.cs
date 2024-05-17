using System;
using System.IO;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using PeabodyNetworkingLibrary;
using HoloportationDecoders.Holoportation;
using HoloportationDecoders.Color2D;
using HoloportationDecoders;
using ControlPanel;
using System.Threading.Tasks;

public delegate void OnRunningCallback();

public class ControlPanelMain : TabController
{
    //Set in editor 
    public Color mStoppedColor;
    public Color mReadyColor;
    public Color mRunningColor;
    public Color mTimedOutColor;

    public HoloportRawFrameViewer hqDecoder;
    public HoloportDecoder holoDecoder;
    public Color2DDecoder colorDecoder;
    public GameObject color2dOutput;

    [Header("UI hookup between Data and Display")]
    public GameObject legendsPanel;
    public GameObject DGPanel;
    public GameObject DGIconBackgroundPrefab;
    public GameObject fusionPanel;
    public GameObject rendererPanel;
    public CalibrationPreview broadcastPreviewObject; // preview pane so we can update camera locations
    [Header("UX UI elements")]
    public Button Button_Start;
    public Button Button_Stop;
    public Text ResolutionText;

    [Header("Events")]
    public UpdateImageColorEvent updateImageColorEvent;
    public UpdateUITextEvent updateUITextEvent;
    public UpdateUIInteractableEvent updateUIInteractableEvent;

    [Header("Tab")]
    public Color mSelectedTabColor;
    public Color mNonSelectedTabColor;
    public GameObject tabsPanel;

    private SynchronizationContext context;
    private bool connectToServer = false;
    private List<CameraCalibration> allCalibrations;
    //threadsafe SINGLETON pattern
    protected static ControlPanelMain instance; // = new ControlPanelMain();

    // Explicit static constructor to tell C# compiler
    // not to mark type as beforefieldinit
    static ControlPanelMain()
    {
    }

    private ControlPanelMain()
    {
    }

    public static ControlPanelMain Instance
    {
        get
        {
            return instance;
        }
    }

    void OnApplicationQuit()
    {
        OutputHelper.OutputLog("ControlPanelMain OnApplicationQuit called", OutputHelper.Verbosity.Trace);
    }

    // Start is called before the first frame update
    void Start()
    {
        if(instance == null)
        {
            instance = this;
        }
        context = SynchronizationContext.Current;

        IDecoderSettings settings = HoloportationDecoders.Settings.FromSharpConfig(SettingsManager.Instance.config);
        if (colorDecoder != null)
        {
            colorDecoder.Settings = settings;
        }

        if (holoDecoder != null)
        {
            holoDecoder.Settings = settings;
        }

        if (colorDecoder != null)
        {
            colorDecoder.ServerQualityChanged += ColorDecoder_ServerQualityChanged;
        }

        // Hide both models initially
        ToggleModels(false, false);

        // Get the resolution in the config and display it on the main screen
        updateUITextEvent.Raise(ResolutionText, SettingsManager.Instance.GetValueWithDefault("ColorProcessing", "ColorImageHeight", "1080"));
        
    }

    private void Update()
    {
        if(Input.GetKeyUp(KeyCode.Numlock))
        {
            holoDecoder.ClientConnect();
        }
    }

    public void OnButton_NewSession(Button hitButton)
    {
        // send a stop command in case something is still running.
        BotManager.Instance.CleanupMachines();

        //Status bots
        bool creationSuccess = BotManager.Instance.CreateStatusBotsBasedOnConfig();
        if(creationSuccess)
        {
            BotManager.Instance.ResetDaemonInvoke();
            Button_Start.interactable = false;
            Button_Stop.interactable = true;
            
            SetStatusBotUIElements();

            StartBroadcastSession();

            //setup decoder
            if (holoDecoder == null)
            {
                Debug.LogWarning("missing holoDecoder");
            }                
            if(colorDecoder == null)
            {
                Debug.LogWarning("missing colorDecoder");
            }
        }
        else
        {
            //TODO propagate failure reason!
        }  
    }


    public void OnButton_EndSession()
    {
        BotManager.Instance.ResetDaemonInvoke();
        Button_Stop.interactable = false;

        // Disconnect the decoder clients
        DisconnectClients();

        ControlPanelMain.RemoveAllChildGO(DGPanel.transform);
        //kill the statusBots
        BotManager.Instance.CleanUpStatusBots();        

    }

    public void OnDaemonStatusUpdate()
    {

        Tuple<bool, DAEMON_GROUP_STATE> readyStatus = BotManager.Instance.CheckBotManagerReady(SOFTWARE.CAPTURE);        
        if (!readyStatus.Item1)
        {
            updateUIInteractableEvent.Raise(Button_Start, false);
            //this means we're not ready for a NEW session, so check if we are in a session

            if (readyStatus.Item2 == DAEMON_GROUP_STATE.RUNNING)
            {
                updateUIInteractableEvent.Raise(Button_Stop, true);
            }
        }
        else
        {
            //nothing's running then we're ok to enable buttons
            if (readyStatus.Item2 == DAEMON_GROUP_STATE.NOT_RUNNING)
            {
                updateUIInteractableEvent.Raise(Button_Start, true);
                updateUIInteractableEvent.Raise(Button_Stop, false);
            }
        }

    }


    public static void RemoveAllChildGO(Transform parentTransform)
    {
        int childCount = parentTransform.childCount;
        for (int i = childCount - 1; i >= 0; i--)
        {
            GameObject go = parentTransform.GetChild(i).gameObject;
            if(go.activeInHierarchy)
            {
                GameObject.Destroy(go);
            }
            else
            {
                go.SetActive(true);
            }
            
        }
    }

    public void SetStatusBotUIElements()
    {
        //TODO: make pipeline more modular in terms of components
        //DepthGEn
        int depthGenNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration","DepthCameraCount",0,true);

        for (int i = 0; i < depthGenNum; ++i)        
        {
            //DG START: WAIT for all READYS
            //for DGs, if they're all ready, it signals they are prepared to START
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.onReadyCallback += OnDepthGenReady;
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.onRunningCallback += OnDepthGenRunning;

            GameObject currDGIcon = Instantiate(DGIconBackgroundPrefab);
            currDGIcon.transform.SetParent(DGPanel.transform, false);
            currDGIcon.SetActive(true);
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.uiImageIcon = currDGIcon.transform.Find("Image").GetComponent<Image>();
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.uiFPSText = currDGIcon.transform.Find("Image/Text").GetComponent<Text>();
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.updateImgClrEvent = updateImageColorEvent;
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.updateTextEvent = updateUITextEvent;

        }
        //turn prefab off
        DGIconBackgroundPrefab.SetActive(false);


        //Fusion

        BotManager.Instance.fusionStatusBot.componentStatusContainer.uiImageIcon = fusionPanel.transform.Find("Image").GetComponent<Image>();
        BotManager.Instance.fusionStatusBot.componentStatusContainer.uiFPSText = fusionPanel.transform.Find("Image/Text").GetComponent<Text>();
        BotManager.Instance.fusionStatusBot.componentStatusContainer.updateImgClrEvent = updateImageColorEvent;
        BotManager.Instance.fusionStatusBot.componentStatusContainer.updateTextEvent = updateUITextEvent;

        //Renderer

        BotManager.Instance.renderStatusBot.componentStatusContainer.uiImageIcon = rendererPanel.transform.Find("Image").GetComponent<Image>();
        BotManager.Instance.renderStatusBot.componentStatusContainer.uiFPSText = rendererPanel.transform.Find("Image/Text").GetComponent<Text>();
        BotManager.Instance.renderStatusBot.componentStatusContainer.updateImgClrEvent = updateImageColorEvent;
        BotManager.Instance.renderStatusBot.componentStatusContainer.updateTextEvent = updateUITextEvent;



    }


    public void StartBroadcastSession()
    {
        //setup sequence of events
        //create Fusion log source

        if(SettingsManager.Instance.GetValueWithDefault("Debug", "RestartTimedout_FusionRender", true))
        {
            BotManager.Instance.fusionStatusBot.componentStatusContainer.onTimedOutCallback_permanent += OnFusionTimedOut;
            BotManager.Instance.renderStatusBot.componentStatusContainer.onTimedOutCallback_permanent += OnRendererTimedOut;
        }
       
        BotManager.Instance.fusionStatusBot.componentStatusContainer.onRunningCallback += OnFusionRunning;
        BotManager.Instance.renderStatusBot.componentStatusContainer.onRunningCallback += OnRendererRunning;


        int dgNum = SettingsManager.Instance.GetValueWithDefault("DepthGeneration","DepthCameraCount",0,true);
        //kick off the sequence of tasks
        //start the DG's status bots 
        for (int i = 0; i < dgNum; ++i)
        {
            BotManager.Instance.depthGenStatusBot[i].Start();
        }
        //Tell the DG Daemons to start in Capture mode
        BotManager.Instance.BroadcastStartUntilAllDGStart(SOFTWARE.CAPTURE);
        

        //DEBUG option: Setting this flag to true in the config will skip waiting for depth gens to report in
        if(SettingsManager.Instance.GetValueWithDefault("Debug", "SkipControlPanelDepthStartCheck", false))
        {
            for (int i = 0; i < dgNum; ++i)
            {
                OnDepthGenRunning();
            }
        }
        
    }


    public static void OnDepthGenReady()
    {
        //atomically increase 
        Interlocked.Increment(ref BotManager.Instance.runningDGBots);
        // if all bots started
        if(BotManager.Instance.runningDGBots == BotManager.Instance.depthGenStatusBot.Length)
        {
            //DG START: all have started, so now we're waiting for all running as indicated by FPS packets
            OutputHelper.OutputLog($"OnDepthGenReady says all bots are ready.  Transmitting the system start command now. ", OutputHelper.Verbosity.Debug);
            //Tell the K4As to start
            BotManager.Instance.runningDGBots = 0;
            BotManager.Instance.BroadcastStartUntilAllDGStart(SOFTWARE.SYSTEM_START);

        }
        
    }

    public void OnDepthGenRunning()
    {
        //atomically increase 
        Interlocked.Increment(ref BotManager.Instance.runningDGBots);
        // if all K4As have started
        if (BotManager.Instance.runningDGBots == BotManager.Instance.depthGenStatusBot.Length)
        {
            BotManager.Instance.fusionStatusBot.Start();
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));            
        }
    }

    public void OnFusionRunning()
    {
        BotManager.Instance.renderStatusBot.Start();
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));        
    }

    public void OnFusionTimedOut()
    {
        //restart fusion if we realize it timed out
        OutputHelper.OutputLog("Fusion Timedout. Restarting");

        DisconnectClients();
        // stop fusion + render
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));

        BotManager.Instance.fusionStatusBot.ResetHBTime();
        BotManager.Instance.renderStatusBot.ResetHBTime();
        BotManager.Instance.renderStatusBot.componentStatusContainer.SetToStopped();

        //reregister fusion running callbacks
        BotManager.Instance.fusionStatusBot.componentStatusContainer.onRunningCallback += OnFusionRunning;
        BotManager.Instance.renderStatusBot.componentStatusContainer.onRunningCallback += OnRendererRunning;

        Task.Factory.StartNew( () =>
            {
                Thread.Sleep(StatusBot.C3DTM_KILL_TIMEOUT_SEC);
                //start fusion again, which will kickstart renderer again
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.FUSION));
            });
        
    }

    public void OnRendererRunning()
    {
        ConnectClients();
    }

    public void OnRendererTimedOut()
    {
        //restart render if we realize it timed out
        OutputHelper.OutputLog("Renderer Timedout. Restarting");

        DisconnectClients();

        //stopped render 
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_STOP_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));

        BotManager.Instance.renderStatusBot.ResetHBTime();
        BotManager.Instance.renderStatusBot.componentStatusContainer.onRunningCallback += OnRendererRunning;

        Task.Factory.StartNew(() =>
        {
            Thread.Sleep(StatusBot.C3DTM_KILL_TIMEOUT_SEC);
            //start fusion again, which will kickstart renderer again
            BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CONTROL_PANEL_START_REQUESTED, BitConverter.GetBytes((char)SOFTWARE.RENDER));
        });
        
    }


    private void OnDisable()
    {
        DisconnectClients();
    }

    private void ConnectClients()
    {
        // Just show the holoport decoder
        ToggleModels(false, true);

        if (connectToServer)
        {
            if (holoDecoder != null)
            {
                //start holoport decoder
                holoDecoder.ClientConnect();
            }

            if (colorDecoder != null)
            {
                colorDecoder.Connect();
            }
        }
    }

    private void DisconnectClients()
    {
        if (connectToServer)
        {
            if (holoDecoder != null)
            {
                holoDecoder.ClientDisconnect();
            }

            if (colorDecoder != null)
            {
                colorDecoder.Disconnect();
            }
        }

        // Hide both models
        ToggleModels(false, false);
    }

    public override void OnTabEnable()
    {
       
        //Legends Color set
        Color[] legendColors = { mStoppedColor, mReadyColor, mRunningColor, mTimedOutColor };
        for (int i = 0; i < legendColors.Length; ++i)
        {
            Transform childTransform = legendsPanel.transform.Find("legendItemPanel" + i.ToString() + "/Image");
            childTransform.GetComponent<Image>().color = legendColors[i];
        }

        if (!SettingsManager.Instance.GetValueWithDefault("Debug", "SkipControlPanelDaemonStatusCheck", false))
        {
            BotManager.Instance.onDaemonStatusUpdateCallback += OnDaemonStatusUpdate;
            OnDaemonStatusUpdate();// force update
        }

        //register daemon updateevent and ui
        BotManagerCallback registerEventsCallback = () =>
        {
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.uiFPSText = fusionPanel.transform.Find("DaemonTitle/daemonText").GetComponent<Text>();
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.updateTextEvent = updateUITextEvent;
            BotManager.Instance.renderDaemonBot.componentStatusContainer.uiFPSText = rendererPanel.transform.Find("DaemonTitle/daemonText").GetComponent<Text>();
            BotManager.Instance.renderDaemonBot.componentStatusContainer.updateTextEvent = updateUITextEvent;

        };
        if (BotManager.Instance.depthDaemonBotsCreated)
        {
            registerEventsCallback.Invoke();
        }
        else
        {
            BotManager.Instance.onCleanedup += registerEventsCallback;
        }
        // If no calibration has been loaded, load the default from disk and update camera locations
        if (allCalibrations == null)
        {
            allCalibrations = CalibrationMain.InitAllCalibrationsList();
            string oldCalibFolderPath = SettingsManager.Instance.GetValueWithDefault("Calibration", "DefaultCalibrationFileLocation", "");
            string calibDGFile = Path.Combine(oldCalibFolderPath + "/DepthCalibs", SettingsManager.Instance.GetValueWithDefault("Calibration", "OutputColorCalibFile", "calibColorCams.txt"));
            if (File.Exists(calibDGFile))
            {
                allCalibrations = CameraCalibrationHelper.LoadFromCalibColorCamsFile(calibDGFile, allCalibrations);

                broadcastPreviewObject.CreateCamRepresentations(allCalibrations);
            }
        }
    }

    public override void OnTabDisable()
    {
        if (!SettingsManager.Instance.GetValueWithDefault("Debug", "SkipControlPanelDaemonStatusCheck", false)
            && BotManager.Instance.onDaemonStatusUpdateCallback != null)
            BotManager.Instance.onDaemonStatusUpdateCallback -= OnDaemonStatusUpdate;

        //unregister daemon updateevent and ui.
        if (BotManager.Instance.depthDaemonBotsCreated)
        {
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.uiFPSText = null;
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.updateTextEvent = null;
            BotManager.Instance.renderDaemonBot.componentStatusContainer.uiFPSText = null;
            BotManager.Instance.renderDaemonBot.componentStatusContainer.updateTextEvent = null;
        }
    }

    public override RequiredSetting[] GetAllRequiredSettings()
    {
        RequiredSetting[] RequiredSettings = {            
            new RequiredSetting("Network","FusionIPAddress"),
            new RequiredSetting("Network","RendererIPAddress"),
            new RequiredSetting("DepthGeneration","DepthCameraCount"),
        };

        return RequiredSettings;
        //optionals:
        //Debug SkipWindowEventLog_Fusion
        //Debug SkipWindowEventLog_Render
        //Debug SkipDaemonStatusCheck
    }

    private void ColorDecoder_ServerQualityChanged(bool inHQMode)
    {
        // Toggle the models based on whether or not the server is in HQ mode
        ToggleModels(inHQMode, !inHQMode);
    }

    public void ToggleModels(bool hqVisible, bool lqVisible)
    {
        // This can get called from multiple threads, make sure it runs on the main thread
        context.Post(_ =>
        {
            if (holoDecoder != null)
            {
                holoDecoder.IsModelVisible = lqVisible;
            }

            if (hqDecoder != null)
            {
                hqDecoder.IsModelVisible = hqVisible;
            }
            
            if (color2dOutput != null) {
                color2dOutput.SetActive(hqVisible || lqVisible);
            }
        }, null);
    }
}
