using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class OutputHelper 
{
    public static void OutputLog(string message, Verbosity verbosity = Verbosity.Warning)
    {
        if(SettingsManager.Instance.Verbosity() >= (int)verbosity)
        {
            DateTime now = DateTime.Now;
            Debug.Log("[" + now.ToString() + "]:" + message);
        }
    }
    public enum Verbosity{
        Fatal = 0,
        Error,
        Warning,
        Info,
        Debug,
        Trace
    }

}
