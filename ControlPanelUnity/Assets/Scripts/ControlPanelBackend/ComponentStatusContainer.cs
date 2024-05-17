using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using UnityEngine.UI;
using System;

[Serializable]
public class ComponentStatusContainer
{
    public UpdateImageColorEvent updateImgClrEvent;
    public UpdateUITextEvent updateTextEvent;
    // ControlPanel.StatusBot statusBot;
    public ControlPanel.ComponentStatus componentStatus;
    public OnRunningCallback onRunningCallback; //onetime callback
    public OnRunningCallback onReadyCallback; //onetime callabck
    public OnRunningCallback onTimedOutCallback_permanent; // not a one time callback

    //UI display
    public Text uiFPSText;
    public Text frameNumberText;
    public Image uiImageBackground;
    public Image uiImageIcon;
    // displayed in the debug pane version list
    public Text versionText;
    public Text cloudText;
    //COLOR STATES:
    // -- Running = runnig color
    // -- ready = ready color
    // -- stopped = stopped color
    // -- timedOut = timedOutColor    

    public void SetToRunning()
    {
        //if already running, dont need to continue repinging 
        if(!componentStatus.IsRunning)
        {
            componentStatus.IsRunning = true;

            if(uiImageIcon!= null)
            {
                updateImgClrEvent.Raise(uiImageIcon, ControlPanelMain.Instance.mRunningColor);
            }

            //one time callback
            if (onRunningCallback != null)
            {
                onRunningCallback.Invoke();
                onRunningCallback = null;
            }
        }      
    }

    public void SetToReady()
    {
        componentStatus.IsReady = true;

        if(uiImageIcon != null)
        {
            updateImgClrEvent.Raise(uiImageIcon, ControlPanelMain.Instance.mReadyColor);
        }
        //one time callback
        if( onReadyCallback != null)
        {
            onReadyCallback.Invoke();
            onReadyCallback = null;
        }
        componentStatus.FPS_max = 0;
        componentStatus.FPS_min = 1000;
    }

    public void SetToStopped()
    {
        componentStatus.IsRunning = false;
        componentStatus.IsReady = false;
        if (updateImgClrEvent != null && uiImageIcon != null)
            updateImgClrEvent.Raise(uiImageIcon, ControlPanelMain.Instance.mStoppedColor);
        SetFPSText(0.0);
    }

    public void SetToTimedOut()
    {
        componentStatus.IsRunning = false;
        componentStatus.IsReady = false;

        if(updateImgClrEvent!= null && uiImageIcon != null)
        {
            updateImgClrEvent.Raise(uiImageIcon, ControlPanelMain.Instance.mTimedOutColor);
        }
        

        if(onTimedOutCallback_permanent!= null)
        {
            onTimedOutCallback_permanent.Invoke();
        }
    }

    public void SetFPSText(double fps)
    {
        componentStatus.FPS = fps;
        if (uiFPSText != null)
        {
            updateTextEvent.Raise(uiFPSText, fps.ToString("0.00"));
        }
        double podFPSWarning = SettingsManager.Instance.GetValueWithDefault("Debug", "ControlPanelPodFPSWarning", 14.8);
        double fusionFPSWarning = SettingsManager.Instance.GetValueWithDefault("Debug", "ControlPanelFusionFPSWarning", 5);
        double renderFPSWarning = SettingsManager.Instance.GetValueWithDefault("Debug", "ControlPanelRenderFPSWarning", 0);
        if(componentStatus.Name.Contains("AzureKinectNanoToFusion") && fps < podFPSWarning)
        {
            OutputHelper.OutputLog($"[WARNING] {componentStatus.Name} FPS is BELOW {podFPSWarning}: {fps}  Max: {componentStatus.FPS_max} Min: {componentStatus.FPS_min}", OutputHelper.Verbosity.Warning);
        }
        else if(componentStatus.Name.Contains("Fusion") && fps < fusionFPSWarning)
        {
            OutputHelper.OutputLog($"[WARNING] {componentStatus.Name} FPS is BELOW {fusionFPSWarning}: {fps}  Max: {componentStatus.FPS_max} Min: {componentStatus.FPS_min}", OutputHelper.Verbosity.Warning);
        }
        else if(componentStatus.Name.Contains("Render") && fps < renderFPSWarning)
        {
            OutputHelper.OutputLog($"[WARNING] {componentStatus.Name} FPS is BELOW {renderFPSWarning}: {fps}  Max: {componentStatus.FPS_max} Min: {componentStatus.FPS_min}", OutputHelper.Verbosity.Warning);
        }
    }

    public void SetFrameNumberText(int frameNum)
    {
        componentStatus.FrameNum = frameNum;
        if(frameNumberText!= null)
        {
            updateTextEvent.Raise(frameNumberText, frameNum.ToString());
        }
        
    }
}
