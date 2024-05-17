using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PeabodyNetworkingLibrary
{
    /// <summary>
    /// Verifies and, if not exists, creats an event log source in the Application event log.  Also 
    /// allows the control panel to fire off events to the event log of connected computers
    /// </summary>
    public class WindowsEventLog
    {
        const String EventLogSource = "PeabodyControlPanel";
        const String EventLogName = "Application";
        const int LaunchDepthGenEvent = 1;
        const int LaunchFusionEvent = 2;
        const int LaunchCameraRecorderEvent = 3;
        const int LaunchCalibrationSoftwareEvent = 4;
        const int LaunchRenderEvent = 7;
        const int KillDepthGenEvent = 10;
        const int KillFusionEvent = 20;
        const int KillCameraRecorderEvent = 30;
        const int KillCalibrationSoftwareEvent = 40;
        const int KillRenderEvent = 70;
        public WindowsEventLog()
        {
        }

        public bool CreateLogSource(String logSource = EventLogSource, String logName = EventLogName)
        {
            try
            {
                // Check if the log source already exists
                if (!EventLog.SourceExists(logSource))
                {
                    Console.WriteLine("Creating the source");
                    EventSourceCreationData data = new EventSourceCreationData(logSource, logName);
                    EventLog.CreateEventSource(data);
                    return true;
                }
                else
                {
                    return true;
                }
            }
            catch (System.IO.IOException)
            {
               // Console.WriteLine("Couldn't connect to " + computerName);
                return false;
            }
        }

        public void WriteLaunchDepthGenEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Start depth gen!", EventLogEntryType.Information, LaunchDepthGenEvent);
            }
        }
        public void WriteKillDepthGenEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Kill depth gen!", EventLogEntryType.Information, KillDepthGenEvent);
            }
        }
        public void WriteLaunchFusionEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Start fusion!", EventLogEntryType.Information, LaunchFusionEvent);
            }
        }
        public void WriteKillFusionEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Kill fusion!", EventLogEntryType.Information, KillFusionEvent);
            }
        }
        public void WriteLaunchCalibrationSoftwareEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Start calibration software!", EventLogEntryType.Information, LaunchCalibrationSoftwareEvent);
            }
        }
        public void WriteKillCalibrationSoftwareEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Kill calibration software!", EventLogEntryType.Information, KillCalibrationSoftwareEvent);
            }
        }
        public void WriteLaunchRenderEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Start Render!", EventLogEntryType.Information, LaunchRenderEvent);
            }
        }
        public void WriteKillRenderEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Kill render!", EventLogEntryType.Information, KillRenderEvent);
            }
        }
        public void WriteLaunchCameraRecorderEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Start camera recorder!", EventLogEntryType.Information, LaunchCameraRecorderEvent);
            }
        }
        public void WriteKillCameraRecorderEvent(String computerName, String logSource = EventLogSource, String logName = EventLogName)
        {
            using (EventLog eventLog = new EventLog(logName, computerName, logSource))
            {
                eventLog.WriteEntry("Kill fusion!", EventLogEntryType.Information, KillCameraRecorderEvent);
            }
        }
    }
}
