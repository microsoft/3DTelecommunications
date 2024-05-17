using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

public class ProfileManager : MonoBehaviour
{
    public int TargetFrameRate = 20;
    public string ProfilerFileName = "profiledLog.raw";
    public bool EnableProfileDump;
    public float TimeToIdle = 30;
    public int IdleRenderFrameInteval = 5;
    protected DateTime LastInputReceived;
    void Awake()
    {
        //overwrite default/editor settings if these are set in cfg 
        TargetFrameRate = SettingsManager.Instance.GetValueWithDefault("ControlPanelProfile", "TargetFrameRate", TargetFrameRate);
        EnableProfileDump = SettingsManager.Instance.GetValueWithDefault("ControlPanelProfile", "EnableProfileDump", EnableProfileDump);
        ProfilerFileName = SettingsManager.Instance.GetValueWithDefault("ControlPanelProfile", "ProfilerFileName", ProfilerFileName);
        TimeToIdle = SettingsManager.Instance.GetValueWithDefault("ControlPanelProfile", "TimeToIdle", TimeToIdle);
        IdleRenderFrameInteval = SettingsManager.Instance.GetValueWithDefault("ControlPanelProfile", "IdleRenderFrameInteval", IdleRenderFrameInteval);

        Application.targetFrameRate = TargetFrameRate;
        UnityEngine.Rendering.OnDemandRendering.renderFrameInterval = IdleRenderFrameInteval;
        if(EnableProfileDump)
        {           
            Profiler.logFile = Application.persistentDataPath + "/" + ProfilerFileName;
            Debug.Log("Profiling to : " + Profiler.logFile);
        }

        Profiler.enableBinaryLog = EnableProfileDump;
        Profiler.enabled = EnableProfileDump;
        Profiler.SetAreaEnabled(ProfilerArea.GPU, EnableProfileDump);
        Profiler.SetAreaEnabled(ProfilerArea.Rendering, EnableProfileDump);
        Profiler.SetAreaEnabled(ProfilerArea.UI, EnableProfileDump);

        LastInputReceived = DateTime.UtcNow;
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.anyKey)
        {
            LastInputReceived = DateTime.UtcNow;  
        }

        TimeSpan timeSinceLastHB = DateTime.UtcNow - LastInputReceived;
        if (timeSinceLastHB > TimeSpan.FromSeconds(TimeToIdle))
        {
            //slow down rendering
            UnityEngine.Rendering.OnDemandRendering.renderFrameInterval = IdleRenderFrameInteval;
        }
        else
        {
            //go back to targetFrameRAte
            UnityEngine.Rendering.OnDemandRendering.renderFrameInterval = 1;
        }
    }

    private void OnDisable()
    {
        Profiler.logFile = "";
        Profiler.enabled = false;
    }
}
