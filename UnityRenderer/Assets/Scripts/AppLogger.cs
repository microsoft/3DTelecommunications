using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class AppLogger
{
    /// <summary>
    /// Logs a trace event
    /// </summary>
    /// <param name="message"></param>
    public static void Trace(object message)
    {
        if (SettingsManager.Instance != null && SettingsManager.Instance.Verbosity() > 0)
        {
            Debug.Log(message);
        }
    }
}
