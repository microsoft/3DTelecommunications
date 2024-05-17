using System;
using System.Configuration;
using System.Runtime.InteropServices;
using UnityEngine;
using PeabodyNetworkingLibrary;
using System.Text;

public class HoloportControlPanelConnector : MonoBehaviour
{

    public unsafe delegate long ControlPanelEventCallback(CONTROL_PANEL_EVENT eventType, int* eventData, int dataSize);


    const string ControlPanelConnectorString = "ControlPanelConnectorDLL";
    [DllImport(ControlPanelConnectorString)]
    private unsafe static extern IntPtr create_control_panel_connector(string controlPanelIP,
        string eventTCPPort,
        string eventEPGMPort,
        string myInterfaceIP,
        string statusTCPPort,
        string statusEPGMPort);

    [DllImport(ControlPanelConnectorString)]
    private unsafe static extern void destroy_control_panel_connector(IntPtr pointer);

    //set callback function
    [DllImport(ControlPanelConnectorString)]
    private unsafe static extern void connector_set_callback(IntPtr pointer, ControlPanelEventCallback erc);

    [DllImport(ControlPanelConnectorString)]
    private unsafe static extern long connector_send_status(IntPtr pointer, CPC_STATUS status, void* statusData, int dataSize);

    [DllImport(ControlPanelConnectorString)]
    private unsafe static extern void connector_set_build_version(IntPtr pointer, int versionMajor, int versionMinor, int versionPatch, int versionCommits, string branchName, string description, string sha);

    IntPtr controlPanelConnector;

    // overwritten in AwakeWithConfig()
    protected string ControlPanelIP = "ControlPanel";
    protected string MyInterfaceIP = "Render";
    protected string EventTCPPort = "62026";
    protected string EventEPGMPort = "62027";
    protected string StatusTCPPort = "62028";
    protected string StatusEPGMPort = "62029";


    public bool initialized = false;
    public float fpsSendFrequency = 0.5f; // every half second

    private HoloportModelToTexture modelToTextureComponent;

    PeabodyNetworkingLibrary.VersionData versionData;

    void AwakeWithConfig()
    {

        ControlPanelIP = SettingsManager.Instance.GetValueWithDefault("Network", "ControlPanelIP", ControlPanelIP);
        MyInterfaceIP = SettingsManager.Instance.GetValueWithDefault("Network", "RendererIPAddress", MyInterfaceIP);
        EventTCPPort = SettingsManager.Instance.GetValueWithDefault("Ports", "EventPort", EventTCPPort);
        StatusTCPPort = SettingsManager.Instance.GetValueWithDefault("Ports", "StatusPort", StatusTCPPort);
    }
    private void Awake()
    {
        AwakeWithConfig();
        controlPanelConnector = create_control_panel_connector(ControlPanelIP, EventTCPPort, EventEPGMPort, MyInterfaceIP, StatusTCPPort, StatusEPGMPort);
        Debug.Log("Setting the control panel connector callback function.");
        SetCallback();
        initialized = true;
        InvokeRepeating("SendCurrentFPS", 0, fpsSendFrequency);

        if (modelToTextureComponent == null)
        {
            modelToTextureComponent = GetComponent<HoloportModelToTexture>();
        }

        versionData = PeabodyNetworkingLibrary.Utility.ReadVersionDataFromConfig(AppDomain.CurrentDomain.BaseDirectory + "\\3dtm_version.config");
        Debug.Log($"Renderer Version {versionData.Major}.{versionData.Minor}.{versionData.Patch}-{versionData.BranchName}.{versionData.Commits} ({versionData.Description}) ({versionData.Sha1})");
        // Send one build version on startup
        connector_set_build_version(controlPanelConnector, versionData.Major, versionData.Minor, versionData.Patch, versionData.Commits, versionData.BranchName, versionData.Description, versionData.Sha1);
    }

    public unsafe void SendReadyStatus()
    {
        if(initialized)
           connector_send_status(controlPanelConnector, CPC_STATUS.READY, null, 0);
    }

    public unsafe void SendFPSStatus(double fps)
    {
        if (initialized)
           connector_send_status(controlPanelConnector, CPC_STATUS.FPS, &fps, sizeof(double));
    }

    public unsafe void SendStoppedStatus()
    {
        if (initialized)
            connector_send_status(controlPanelConnector, CPC_STATUS.STOPPED, null, 0);
    }

    public unsafe void SendToggleFusionQuality(bool changeToHigh)
    {
        if (initialized)
        {
            byte[] dataBytes = BitConverter.GetBytes(changeToHigh);
            fixed (byte* datBytePointer = dataBytes)
            {
                connector_send_status(controlPanelConnector, CPC_STATUS.TOGGLE_FUSION_HIGH_RESOLUTION, datBytePointer, dataBytes.Length);
            }
        }
    }

    private void OnDisable()
    {
        if (initialized)
        {
            SendStoppedStatus();

            initialized = false;

            // TODO: This takes a while to stop
            // BUG: https://dev.azure.com/msrp/PeabodyMain/_workitems/edit/9640
            destroy_control_panel_connector(controlPanelConnector);
        }
    }

    void Update()
    {
        if(Input.GetKeyUp(KeyCode.Tab))
        {
            SendFPSStatus(49.0);
        }
    }

    public unsafe void SetCallback()
    {
        connector_set_callback(controlPanelConnector, ReceivedControlPanelEvent);
    }

    public unsafe long ReceivedControlPanelEvent(CONTROL_PANEL_EVENT eventType, int* eventData, int dataSize)
    {
        switch(eventType)
        {
            case CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED:
                // Send build version is handled by the underlying ControlPanelConnector class in c++
                break;
            default:
                Debug.Log("No handler for event type " + eventType.ToString());
                break;
        }
        return 0;
    }

    /// <summary>
    /// Sends the current FPS to the control panel
    /// </summary>
    private void SendCurrentFPS()
    {
        double fps = 1.0 / Time.deltaTime;
        if (modelToTextureComponent != null)
        {
            fps = modelToTextureComponent.CurrentFPS; // grab the same FPS text that we calculate for Canvas UI display
        }
        if (fps > 0)
        {
            SendFPSStatus(fps);
        }
    }
};
