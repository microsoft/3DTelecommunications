using ControlPanel;
using SharpDX.DXGI;
using SharpDX.MediaFoundation;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
using PeabodyNetworkingLibrary;
using UnityEngine.EventSystems; 
using System.Runtime.InteropServices;

//sequentially, stages of the entire calibration process
public sealed class CalibrationStages
{
    public static CalibrationStages Ready { get; } = new CalibrationStages("Ready");
    public static CalibrationStages CapturingBackground { get; } = new CalibrationStages("Capturing Background");
    //CalibrationSoftware stages
    public static CalibrationStages StartingCalibVideoCaptureSoftware { get; } = new CalibrationStages("Starting Calibration Software, please wait...");
    public static CalibrationStages ReadyForVideoCapture { get; } = new CalibrationStages("Ready for Video Capture, press the 'Record Video' button to begin.");
    public static CalibrationStages CaptureVideoStarting {get; } = new CalibrationStages("Starting Video Capture, please wait...");
    
    public static CalibrationStages CapturingVideos { get; } = new CalibrationStages("Capturing Videos. Press 'Finish Recording' when done.");
    public static CalibrationStages TransferringVideos { get; } = new CalibrationStages("Transferring Videos, please wait...");
    public static CalibrationStages ReadyForCalibration { get; } = new CalibrationStages("Transfer Complete.  Press Calibrate to Begin Calibration.");
    public static CalibrationStages RunningCalibrationSoftware { get; } = new CalibrationStages("Running Calibration Software");
    public static CalibrationStages Distributing3DTMCalibs { get; } = new CalibrationStages("Distributing 3DTM Calibration Files");

    public static CalibrationStages NotReady { get; } = new CalibrationStages("Not Ready");

    public CalibrationStages(string name)
    {
        ID = stageCount++;
        Name = name;
    }
    private static int stageCount = 0;
    public int ID;
    public string Name;
}

public class CalibrationMain : TabController
{

    [Header("UI hookup between Data and Display")]
    public GameObject calibrationRecorderPanel;
    public GameObject calibrationRecorderUIPrefab;

    public Button Button_CaptureDataAndCalibrate;
    public Text newCalibrationStatusText;
    public Text calibrationResultText;
    public Text resolutionText;

    public Button Button_NewCalib;
    public Button Button_BackgroundCalib;

    public Text Text_Button_NewCalibText;
    public Text Text_Button_BackgroundCalibText;
    public Text Text_Button_CaptureVideoText;

    public CalibrationPreview calibrationPreviewObject; // this is the preview scene located on the calibration tab
    public CalibrationPreview broadcastPreviewObject; // this is the preview scene located on the broadcast tab

    [Header("Events")]
    public UpdateImageColorEvent updateImageColorEvent;
    public UpdateUITextEvent updateUITextEvent;
    public UpdateUIInteractableEvent updateUIInteractableEvent;

    //CALIBRATION Software Session variables
    private int runningDGVideoCapture = 0;
    private int transferredVideos = 0;

    [Header("Calibration Data")]
    public List<CameraCalibration> allCalibrations;
    private IEnumerator AwaitCameraSerialNumCoroutine;
    private bool startAwaitCamSerialCoroutine;

    public CalibrationStages currentCalibrationStage = CalibrationStages.NotReady;
    const int cUpdateOffset = 1; //first byte of status packet is the status type, so skip it

    private bool cleanupBotRequest = false;
    public TimeSpan BotResponseTimeout = TimeSpan.FromSeconds(5);
    public UnityEngine.Transform calibrationRecorderPanelTransform;

    // Start is called before the first frame update
    void Start()
    {
        calibrationRecorderPanelTransform = calibrationRecorderPanel.transform;
        calibrationResultText.text = "";
        //Load
        string oldCalibFolderPath = SettingsManager.Instance.GetValueWithDefault("Calibration", "CalibrationWorkingDirectory", "");

        UpdateButtonsBasedOnState();
        UpdateCalibrationProcessState(CalibrationStages.NotReady);
        updateUITextEvent.Raise(resolutionText, SettingsManager.Instance.GetValueWithDefault("K4ARecorder", "ColorMode", "1080p"));
    }

    void UpdateCalibrationProcessState(CalibrationStages newStage, string additionalInfo = "")
    {
        Debug.Log($"Setting calibration state to {newStage.Name}:{additionalInfo}. Current state was {currentCalibrationStage.Name}");
        CalibrationStages last = currentCalibrationStage;
        currentCalibrationStage = newStage;
        updateUITextEvent.Raise(newCalibrationStatusText, newStage.Name + additionalInfo);
        if (newStage != last)
        {
            UpdateButtonsBasedOnState();
        }
    }

    public static List<CameraCalibration> InitAllCalibrationsList()
    {
        List<CameraCalibration> allCalibrations = new List<CameraCalibration>();
        int depthGenNum = SettingsManager.Instance.config["DepthGeneration"]["DepthCameraCount"].IntValue;
        for (int i = 0; i < depthGenNum; ++i)
        {
            CameraCalibration cc = new CameraCalibration();
            cc.Name = "defaultCamName" + i;
            allCalibrations.Add(cc);
        }
        return allCalibrations;
    }
    public bool InitializeCalibration(OnRunningCallback orc, SOFTWARE software = SOFTWARE.CALIBRATION)
    {
        allCalibrations = InitAllCalibrationsList();
        bool createdBots = BotManager.Instance.CreateCalibrationStatusBotsBasedOnConfig(software);

        if (createdBots)
        {
            SetStatusBotUIElements(orc);

            //kick start the bots
            int depthGenNum = SettingsManager.Instance.config["DepthGeneration"]["DepthCameraCount"].IntValue;
            for (int i = 0; i < depthGenNum; ++i)
            {
                BotManager.Instance.depthGenStatusBot[i].Start();
            }

            BotManager.Instance.calibrationSoftwareStatusBot.Start();
            
            // Set the status bots component status to indicate that they have not received the new calibration yet
            // this gives us something to wait for.  This value gets updated by the registeredUpdateFunction
            // inside the daemon bots (status bots for DG) for CPC_STATUS.NewDataReceived
            BotManager.Instance.renderDaemonBot.componentStatusContainer.componentStatus.NewDataReceived = false;
            BotManager.Instance.fusionDaemonBot.componentStatusContainer.componentStatus.NewDataReceived = false;
        }
        return createdBots;

    }

    public void SetStatusBotUIElements(OnRunningCallback orc)
    {
        //DepthGEn
        int depthGenNum = SettingsManager.Instance.config["DepthGeneration"]["DepthCameraCount"].IntValue;

        for (int i = 0; i < depthGenNum; ++i)
        {
            int camIndex = i;
            StatusBot currBot = BotManager.Instance.depthGenStatusBot[i];
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.onReadyCallback += ControlPanelMain.OnDepthGenReady;
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.onRunningCallback += orc;

            GameObject currDGIcon = Instantiate(calibrationRecorderUIPrefab);
            currDGIcon.transform.SetParent(calibrationRecorderPanel.transform, false);
            currDGIcon.SetActive(true);
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.uiImageIcon = currDGIcon.transform.Find("iconBackground/Image").GetComponent<UnityEngine.UI.Image>();
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.frameNumberText = currDGIcon.transform.Find("textPanel/frameCaptNumberText").GetComponent<Text>();
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.uiFPSText = currDGIcon.transform.Find("textPanel/fpsText").GetComponent<Text>();
            currDGIcon.transform.Find("textPanel/nameText").GetComponent<Text>().text = $"DepthPod{(i + 1)}"; // update machineName

            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.updateImgClrEvent = updateImageColorEvent;
            BotManager.Instance.depthGenStatusBot[i].componentStatusContainer.updateTextEvent = updateUITextEvent;

            CameraCalibration currCalibration = allCalibrations[i];
            StatusResponseDelegate registerSerialFunc = delegate (byte[] update)
            {
                if (update.Length < sizeof(int))
                {
                    OutputHelper.OutputLog("ERROR: wrong lengthed update packet in registerSerialFunc " + update.Length);
                    return;
                }
                int serialNumLength = System.BitConverter.ToInt32(update, cUpdateOffset);
                OutputHelper.OutputLog($"From DG {camIndex} received serial number size {serialNumLength}", OutputHelper.Verbosity.Debug);

                string newString = System.Text.Encoding.UTF8.GetString(update, cUpdateOffset + sizeof(int), serialNumLength);

                OutputHelper.OutputLog($"From DG {camIndex} received serial number {newString}", OutputHelper.Verbosity.Debug);

                if (!newString.Equals(currCalibration.Name))
                {
                    currCalibration.Name = newString;
                }
            };

            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.IS_ALIVE, registerSerialFunc);
            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.READY, registerSerialFunc);

            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.CALIBRATION_PROGRESS,

                delegate (byte[] update)
                {
                    if (currentCalibrationStage == CalibrationStages.CapturingBackground)
                    {
                        OnCallback_CalibrationProgressForBGDownload(update, currBot, camIndex);
                    }
                }
                );

            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.CALIBRATION_DONE,
                delegate (byte[] update)
                {
                    if (currentCalibrationStage == CalibrationStages.CapturingBackground)
                    {
                        OnBackgroundCaptureFinished(update);
                    }
                });


            // CALIBRATION_SOFTWARE events
            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED,
                delegate (byte[] update)
                {
                    OutputHelper.OutputLog($"Depth generator {camIndex} ({currCalibration.Name}) started capturing calibration videos correctly");

                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, update);

                    Interlocked.Increment(ref runningDGVideoCapture);

                    if (runningDGVideoCapture == depthGenNum)
                    {
                        OutputHelper.OutputLog($"All deph generators started capturing calibration videos correctly {runningDGVideoCapture} == {depthGenNum}. Enabling capture data button to stop video capture.", OutputHelper.Verbosity.Debug);
                        //Re-enable the CaptureFrame button to stop video capture
                        UpdateCalibrationProcessState(CalibrationStages.CapturingVideos, "");
                    }
                });

            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO,
                delegate (byte[] update)
                {
                    // Just forward the packet so LEDs can be updated
                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO, update);
                });

            // When the depth generators finish transferring the videos, we enable the calibration button
            BotManager.Instance.depthGenStatusBot[i].RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE,
                delegate (byte[] update)
                {
                    OutputHelper.OutputLog($"Pod {currBot.DepthGenMachineID} is done transferring video.", OutputHelper.Verbosity.Debug);
                    Interlocked.Increment(ref transferredVideos);

                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE, update);

                    if (transferredVideos == depthGenNum)
                    {
                        OutputHelper.OutputLog($"All pods have transferred their videos correctly {transferredVideos} == {depthGenNum}. Enabling calibration button.", OutputHelper.Verbosity.Debug);

                        // Since the PAI calibration may take a long time for long videos, DebugMode separates the video capture step from
                        // calibration and allows the user to only capture videos, if desired. Otherwise, calibration is run automatically
                        // once the videos are transferred to the calibration computer
                        if(SettingsManager.Instance.GetValueWithDefault("Calibration", "DebugMode", false)) {
                            
                            UpdateCalibrationProcessState(CalibrationStages.ReadyForCalibration);

                        } else { 
                            UpdateCalibrationProcessState(CalibrationStages.RunningCalibrationSoftware);
                            RunCalibration();
                        }

                        // Resetting the video capture flag in case the user wants to redo a capture
                        runningDGVideoCapture = 0;

                    }
                });
        }
        //turn prefab off
        calibrationRecorderUIPrefab.SetActive(false);

        BotManager.Instance.calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_PROCESSING,
            delegate (byte[] update)
            {
                BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_PROCESSING, update);
                if(update.Length > cUpdateOffset) { 
                    String progress_msg =  System.Text.Encoding.UTF8.GetString(update, cUpdateOffset, update.Length - cUpdateOffset);
                    updateUITextEvent.Raise(calibrationResultText, progress_msg);
                }
                OutputHelper.OutputLog($"Forwarding a calibration software processing event.", OutputHelper.Verbosity.Debug);
            });

        BotManager.Instance.calibrationSoftwareStatusBot.RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_RESULT,
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
                        Buffer.BlockCopy(update, cUpdateOffset+sizeof(int), jsonPacketBytes, sizeof(int), calibJsonLength);

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

                    UpdateCalibrationProcessState(CalibrationStages.NotReady, "Failed Calibration");
                    updateUITextEvent.Raise(calibrationResultText, $"Calibration Failed.  Please re-try calibration.");
                    
                    // Returning empty Json string to indicate a failed calibration to Fusion, Render, and the PODs
                    BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_RESULT, new byte[0]);

                    // Shut down the session if we failed.
                    StopCalibration();
                } 
                else 
                {                    
                    float ReprojectionError = json["calibrationSession"]["metrics"]["ReprojectionError"].ToObject<float>();
                    float DepthError = json["calibrationSession"]["metrics"]["DepthError"].ToObject<float>();
                    float MultiViewMisalignment = json["calibrationSession"]["metrics"]["MultiViewMisalignment"].ToObject<float>();
                    String metric_msg = $"Successful Calibration. Reprojection error {ReprojectionError:F3} pixels. Depth error {DepthError:F3}mm. MultiViewMisalignment {MultiViewMisalignment:F3}mm";
                    if(MultiViewMisalignment > 2.0)
                    {
                        metric_msg = $"Calibration succeeded, but the MultiViewMisalignment is greater than 2.0.  Quality of the output may be degraded with this calibration.  You may way to recalibrate. Reprojection error {ReprojectionError:F3} pixels. Depth error {DepthError:F3}mm. MultiViewMisalignment {MultiViewMisalignment:F3}mm";
                    }

                    OutputHelper.OutputLog(metric_msg);
                    OutputHelper.OutputLog($"Distributing 3DTM calibration files");

                    UpdateCalibrationProcessState(CalibrationStages.Distributing3DTMCalibs);
                    updateUITextEvent.Raise(calibrationResultText, metric_msg);
                    
                    // Wait for the calibration files to be distributed and stop bots
                    startAwaitCamSerialCoroutine = true;
                }
            });
    }

    public void OnCallback_CalibrationProgressForBGDownload(byte[] update, StatusBot currBot, int camIndex)
    {
        int BGframeNumber = (int)System.BitConverter.ToUInt32(update, cUpdateOffset);
        OutputHelper.OutputLog(string.Format("BG Progress: {0}={1}", currBot.UnitName, BGframeNumber));
        currBot.componentStatusContainer.SetFrameNumberText(BGframeNumber);
    }

    public void OnCalibrationSoftwareRecorderRunning()
    {
        OutputHelper.OutputLog("OnCalibrationSoftwareRecorderRunning issued");

        //atomically increase 
        Interlocked.Increment(ref BotManager.Instance.runningDGBots);

        // if all bots started
        if (BotManager.Instance.runningDGBots == BotManager.Instance.depthGenStatusBot.Length)
        {
            OutputHelper.OutputLog("OnCalibrationSoftwareRecorderRunning all DG Bots are running");

            // Now it is ok to enable capture video button
            UpdateCalibrationProcessState(CalibrationStages.ReadyForVideoCapture, "");
        }
    }

    public IEnumerator WaitForAllSerialNumberThenDistributeCalib()
    {
        OutputHelper.OutputLog("Awaiting Cam serials");
        bool haveAllNames = false;
        while (!haveAllNames)
        {
            haveAllNames = true;
            //Then need to check if all Calibrations have names now
            if (allCalibrations != null)

                for (int i = 0; i < allCalibrations.Count; ++i)
                {
                    if (allCalibrations[i].Name.StartsWith("defaultCamName"))
                    {
                        haveAllNames = false;
                        break;
                    }
                }
            yield return new WaitForSeconds(0.1f);
        }
        OutputHelper.OutputLog("Got all Cam serials! loading and distributing calib!");
        // have now established all have names

        //distribute
        if (DistributeCalibrations())
        {
            updateUITextEvent.Raise(calibrationResultText, calibrationResultText.text + ". Calibration files distributed.");
        }
        else
        {
            updateUITextEvent.Raise(calibrationResultText, calibrationResultText.text + ". Error distributing calibration files.  See log for more details.");
        }
        UpdateCalibrationProcessState(CalibrationStages.NotReady);
        // Shut down the session
        StopCalibration();
    }

    public void OnBackgroundCaptureFinished(byte[] update)
    {
        Interlocked.Increment(ref BotManager.Instance.runningDGBots);
        UpdateCalibrationProcessState(CalibrationStages.CapturingBackground, $"{{{BotManager.Instance.runningDGBots}}}/{{{BotManager.Instance.depthGenStatusBot.Length}}}");
        // if all bots finished
        if (BotManager.Instance.runningDGBots == BotManager.Instance.depthGenStatusBot.Length)
        {
            UpdateCalibrationProcessState(CalibrationStages.NotReady);
            // Shut down the session
            StopCalibration();
            // indicate success in the calibrationResultText
            updateUITextEvent.Raise(calibrationResultText, calibrationResultText.text + " Background Capture Completed.");
        }
    }

    public void CheckAndUnselectButtons()
    {
        //dont interfere with active selection from a diff thread
        EventSystem es = EventSystem.current;
        if (!es.alreadySelecting)
        {
            es.SetSelectedGameObject(null);
        }
    }

    public void UpdateButtonsBasedOnState()
    {
        string defaultNewCalibText = "Start New Calibration";
        string defaultBGCaptureText = "Capture Background";
        string defaultCaptureVideoText = "";

        BotManager.Instance.mainThreadCallbacks.Enqueue(CheckAndUnselectButtons);
        if (currentCalibrationStage == CalibrationStages.CapturingBackground)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, false);
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, true);
            updateUITextEvent.Raise(Text_Button_BackgroundCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, defaultCaptureVideoText);
        }
        else if (currentCalibrationStage == CalibrationStages.StartingCalibVideoCaptureSoftware)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Disabling Button_CaptureDataAndCalibrate (1) and Button_CalibrateAll (2) buttons 
            // until the depth generators signal that (1) they have started capturing
            // the videos and (2) the user stopped video captured and the files were
            // correctly transferred to the CalibrationSoftware machine
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Software Starting...");
        }
        else if (currentCalibrationStage == CalibrationStages.ReadyForVideoCapture)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Now enable the capture button so the user can start the recording
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, true);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Record Video");
        }
        else if (currentCalibrationStage == CalibrationStages.CaptureVideoStarting)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Disabling Button_CaptureDataAndCalibrate (1) and Button_CalibrateAll (2) buttons 
            // until the depth generators signal that (1) they have started capturing
            // the videos and (2) the user stopped video captured and the files were
            // correctly transferred to the CalibrationSoftware machine
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Recording Starting...");
        }
        else if (currentCalibrationStage == CalibrationStages.CapturingVideos)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Now re-enable the capture button so we can stop capturing
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, true);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Finish Recording");
        }    
        else if (currentCalibrationStage == CalibrationStages.TransferringVideos)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Disable the capture button while videos are transferring. Don't enable the calibrate button until
            // all transfers are completed
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Transferring Videos...");
        }    
        else if(currentCalibrationStage == CalibrationStages.ReadyForCalibration)
        {
            updateUITextEvent.Raise(calibrationResultText, "");
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUITextEvent.Raise(Text_Button_NewCalibText, "Cancel");
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            // Now re-enable the calibrate button so we start calibration
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, true);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, "Calibrate");
        }    
        else if (currentCalibrationStage == CalibrationStages.NotReady)
        {
            updateUIInteractableEvent.Raise(Button_NewCalib, false);
            updateUITextEvent.Raise(Text_Button_NewCalibText, defaultNewCalibText);
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, false);
            updateUITextEvent.Raise(Text_Button_BackgroundCalibText, defaultBGCaptureText);
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
            updateUITextEvent.Raise(Text_Button_CaptureVideoText, defaultCaptureVideoText);
        }
        else
        {
            //CalibrationStages.Ready
            //default
            updateUIInteractableEvent.Raise(Button_NewCalib, true);
            updateUIInteractableEvent.Raise(Button_BackgroundCalib, true);
            updateUIInteractableEvent.Raise(Button_CaptureDataAndCalibrate, false);
        }

        BotManager.Instance.ResetDaemonInvoke();
    }

    public void OnDaemonStatusUpdate()
    {
        //mainly used for turning buttons off if things are not working

        Tuple<bool, DAEMON_GROUP_STATE> readyStatus = BotManager.Instance.CheckBotManagerReady(SOFTWARE.CALIBRATION);
        OutputHelper.OutputLog("calib daemonupdate:" + readyStatus.Item1 + " + " + readyStatus.Item2);
        OutputHelper.OutputLog(BotManager.Instance.GetDaemonNotReadyText(SOFTWARE.CALIBRATION));
        if (!readyStatus.Item1)  // Not ready to launch, i.e. something is already running
        {
            switch (readyStatus.Item2)
            {
                case DAEMON_GROUP_STATE.NOT_RUNNING:
                    UpdateCalibrationProcessState(CalibrationStages.NotReady, "Waiting for cameras");
                    break;
                case DAEMON_GROUP_STATE.RUNNING:
                case DAEMON_GROUP_STATE.LAUNCHING:
                default:
                    // currently running or launching, so desired state is ready to launch = false, do not change state
                    break;
            }
        }
        else  // daemons report the system is ready to launch software
        {
            if (readyStatus.Item2 == DAEMON_GROUP_STATE.NOT_RUNNING)
            {
                // We should get to here any time we're running correctly, but only want to update state if we were in a not-ready state
                if (currentCalibrationStage == CalibrationStages.NotReady)
                {
                    UpdateCalibrationProcessState(CalibrationStages.Ready);
                }
            }
        }


    }

    public void OnButton_NewCalibration()
    {
        NewCalibrationSoftwareSession();
    }

    public void NewCalibrationSoftwareSession()
    {
        OutputHelper.OutputLog($"On NewCalibrationSoftwareSession {currentCalibrationStage}");
        if (currentCalibrationStage == CalibrationStages.Ready)
        {
            OutputHelper.OutputLog($"On NewCalibrationSoftwareSession1 ");
            bool successfulCreation = InitializeCalibration(OnCalibrationSoftwareRecorderRunning);
            OutputHelper.OutputLog($"On NewCalibrationSoftwareSession2 {successfulCreation}");

            if (successfulCreation)
            {
                transferredVideos = 0;
                runningDGVideoCapture = 0;

                BotManager.Instance.BroadcastStartUntilAllDGStart(SOFTWARE.CALIBRATION, BitConverter.GetBytes((char)PeabodyNetworkingLibrary.CALIBRATION_SOFTWARE_COMPONENT.POD));
                // Moving to StartingCalibVideoCapture state, which will disable CaptureFrame button until
                // all depth generators are running
                UpdateCalibrationProcessState(CalibrationStages.StartingCalibVideoCaptureSoftware);
            }
            else
            {
                OutputHelper.OutputLog("Could not initCalibration when New Calibration");
            }
        }
        else
        {
            // Send to not-ready before we go back to the "Ready" state
            UpdateCalibrationProcessState(CalibrationStages.NotReady);
            StopCalibration();
        }

    }

    public void StopCalibration()
    {
        // Cleanup of game objects has to happen in the main thread, so signal that a cleaup is required
        cleanupBotRequest = true;
        UpdateCalibrationProcessState(CalibrationStages.Ready);
    }

    public void OnButton_BackgroundCapture()
    {
        //start up the DGS just to get BG capture
        bool successfulCreation = InitializeCalibration(null, SOFTWARE.BACKGROUND_CAPTURE);
        if (successfulCreation)
        {
            BotManager.Instance.BroadcastStartUntilAllDGStart(SOFTWARE.BACKGROUND_CAPTURE);
            UpdateCalibrationProcessState(CalibrationStages.CapturingBackground);
        }
    }

    public void OnButton_CaptureDataAndCalibrate()
    {
        // If no DG is performing calibration video capture, we issue a start event for the DGs
        if (runningDGVideoCapture == 0 && currentCalibrationStage != CalibrationStages.ReadyForCalibration)
        {
            RunCalibrationSoftwareDataCapture();
        } // If all DGs are running, the CaptureFrame button sends a stop command when the user presses it
        else if(runningDGVideoCapture == BotManager.Instance.depthGenStatusBot.Length && currentCalibrationStage != CalibrationStages.ReadyForCalibration)
        {
            StopCalibrationSoftwareDataCapture();
        }
        else if(currentCalibrationStage == CalibrationStages.ReadyForCalibration)
        {
            RunCalibration();
        }
        else
        {
            OutputHelper.OutputLog($"Warning! User clicked on CaptureFrame and not all depth generators started capturing calibration video yet ({runningDGVideoCapture}/{BotManager.Instance.depthGenStatusBot.Length}). Ignoring action until all DGs start.");
        }
    }

    public void RunCalibrationSoftwareDataCapture()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_START_CAPTURING_VIDEO);
        OutputHelper.OutputLog("Broadcasting Capture Calibration Video Signal!");

        //disable the trigger button until all depth generators start capturing videos
        UpdateCalibrationProcessState(CalibrationStages.CaptureVideoStarting);
    }

    public void StopCalibrationSoftwareDataCapture()
    {
        BotManager.Instance.BroadcastEventOnce(CONTROL_PANEL_EVENT.CALIBRATION_SOFTWARE_STOP_CAPTURING_VIDEO);
        OutputHelper.OutputLog("Broadcasting Stop Calibration Video Capture Signal!");

        UpdateCalibrationProcessState(CalibrationStages.TransferringVideos);
    }

    public void RunCalibration()
    {
        // clear the calibrationResultText
        updateUITextEvent.Raise(calibrationResultText, "");
        RunCalibrationSoftwareCalibration();
    }

    public void RunCalibrationSoftwareCalibration()
    {
        BotManager.Instance.BroadcastStartEventOnce(SOFTWARE.CALIBRATION, BitConverter.GetBytes((char)1));
        UpdateCalibrationProcessState(CalibrationStages.RunningCalibrationSoftware);
    }

    public bool DistributeCalibrations()
    {
        return WaitForCalibrationSoftwareResultDistribution();
    }

    public bool WaitForCalibrationSoftwareResultDistribution()
    {
        //IMPORTANT: this function simply waits for the CalibrationSoftware result to be distributed to Render and Fusion,
        //in order to provide feedback to the user in case of distribution error. The Json result was already forwarded
        //to Render and Fusion on the registeredUpdateFunction of calibrationSoftwareStatusBot, which
        //handles the CPS_STATUS.CALIBRATION_SOFTWARE_RESULT event

        //Reference variables to simplify the code
        ComponentStatus renderStatus = BotManager.Instance.renderDaemonBot.componentStatusContainer.componentStatus;
        ComponentStatus fusionStatus = BotManager.Instance.fusionDaemonBot.componentStatusContainer.componentStatus;       

        // Here we should wait for reception of a NEW_DATA_RECEIVED packet
        // have a timeout so we don't wait forever
        bool success = false;
        int elapsed = 0;
        while (!success && (elapsed < BotResponseTimeout.TotalMilliseconds))
        {
            if (renderStatus.NewDataReceived && fusionStatus.NewDataReceived)
            {
                OutputHelper.OutputLog($"Render and Fusion both report new data received. Successful transmission.");
                success = true;
                break;
            }
            Thread.Sleep(100);
            elapsed += 100;
        }
        if (!renderStatus.NewDataReceived)
        {
            OutputHelper.OutputLog("ERROR: Render never acknowledged receiving new calibration data");
        }
        if (!fusionStatus.NewDataReceived)
        {
            OutputHelper.OutputLog("ERROR: Fusion never acknowledged receiving new calibration data");
        }

        return success;
    }

// Update is called once per frame
void Update()
    {
        if(startAwaitCamSerialCoroutine)
        {
            if(AwaitCameraSerialNumCoroutine !=null)
            {
                StopCoroutine(AwaitCameraSerialNumCoroutine);
            }
            AwaitCameraSerialNumCoroutine = WaitForAllSerialNumberThenDistributeCalib();
            StartCoroutine(AwaitCameraSerialNumCoroutine);
            startAwaitCamSerialCoroutine = false;
        }
        if (cleanupBotRequest)
        {
            //send stop 
            ControlPanelMain.RemoveAllChildGO(calibrationRecorderPanel.transform);
            BotManager.Instance.CleanUpStatusBots();
            cleanupBotRequest = false;
        }
    }

    private void OnDisable()
    {
        if(AwaitCameraSerialNumCoroutine!= null)
        {
            StopCoroutine(AwaitCameraSerialNumCoroutine);
        }
    }

    public override void OnTabEnable()
    {       
        if (!SettingsManager.Instance.GetValueWithDefault("Debug", "SkipControlPanelDaemonStatusCheck", false))
        {
            BotManager.Instance.onDaemonStatusUpdateCallback += OnDaemonStatusUpdate;
            OnDaemonStatusUpdate();// force update
        }        

        if (calibrationPreviewObject == null)
        {
            calibrationPreviewObject = gameObject.AddComponent<CalibrationPreview>();
            calibrationPreviewObject.cameraModelPrefab = GameObject.CreatePrimitive(PrimitiveType.Cube);
        }

        startAwaitCamSerialCoroutine = false;

    }

    public override void OnTabDisable()
    {
        if (!SettingsManager.Instance.GetValueWithDefault("Debug","SkipControlPanelDaemonStatusCheck", false) 
            && BotManager.Instance.onDaemonStatusUpdateCallback != null)
            BotManager.Instance.onDaemonStatusUpdateCallback -= OnDaemonStatusUpdate;
    }

    public override bool CheckTabPrerequisites(RequiredSetting[] requiredSettings, out List<string> errorMessages)
    {
        bool basicCheckResult = base.CheckTabPrerequisites(requiredSettings, out errorMessages);

        return basicCheckResult;
    }

    public override RequiredSetting[] GetAllRequiredSettings()
    {
        RequiredSetting[] RequiredSettings = { 
            new RequiredSetting("Calibration","CalibrationWorkingDirectory", true, true),
            new RequiredSetting("DepthGeneration","DepthCameraCount"),
            new RequiredSetting("ColorProcessing","ColorImageWidth"),
            new RequiredSetting("ColorProcessing","ColorImageHeight"),
            new RequiredSetting("DepthGeneration","DepthImageWidth"),
            new RequiredSetting("DepthGeneration","DepthImageHeight"),
        };

        return RequiredSettings;
        //optionals:
        //Calibration BackgroundCaptureWaitTime
        //Debug SkipDaemonStatusCheck
       
    }
}
