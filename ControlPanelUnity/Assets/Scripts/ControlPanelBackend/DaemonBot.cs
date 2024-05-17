using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PeabodyNetworkingLibrary;

namespace ControlPanel
{
    //indicate what the overall daemon groups are doing status-wise
    public enum DAEMON_GROUP_STATE
    {
        UNKNOWN, // +1 daemon with this
        ERROR, // + 1 daemon with this 

        NOT_RUNNING, // ALL daemons are not running
        LAUNCHING,  //Not ALL daemons are RUNNING, but not errors
        RUNNING, // ALL daemons are this

        
    }

    public class DaemonBot : StatusBot
    {
        public SOFTWARE_STATE[] softwareStates { get; private set; }
        public DaemonBot(String ip, String name, ComponentStatusContainer csc, string overrideTCPPort = "", string overrideEPGMPORT = "")
            : base(ip, name, csc, overrideTCPPort, overrideEPGMPORT)
        {
            ResetKnownStatus();
        }

        public void UpdateStateAndUI(SOFTWARE software, SOFTWARE_STATE newStatus)
        {
            softwareStates[(int)software] = newStatus;
            // This is dangerous, because the bot _could_ receive a state update from any software, 
            // and regardless of _which_ software it will update it's UI to the latest received state,
            // whether or not the UI corresponds to the SOFTWARE component passed into this function
            if(componentStatusContainer.updateTextEvent != null && componentStatusContainer.uiFPSText != null)
            {
                OutputHelper.OutputLog($"{UnitName} updating {software.ToString()} status to {newStatus}", OutputHelper.Verbosity.Debug);
                componentStatusContainer.updateTextEvent.Raise(componentStatusContainer.uiFPSText, "Status: " + newStatus.ToString());                
            }
        }

        public bool UpdateStates(SOFTWARE_STATE[] states)
        {
            bool changed = false;
            for(int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                if(softwareStates[i] != states[i])
                {                    
                    softwareStates[i] = states[i];
                    changed = true;
                }
            }
            return changed;
        }

        public void ResetKnownStatus()
        {
            softwareStates = new SOFTWARE_STATE[(int)SOFTWARE.COUNT];
            for(int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                softwareStates[i] = SOFTWARE_STATE.UNKNOWN;
            }
        }

        public void PrintStates(OutputHelper.Verbosity verbosity = OutputHelper.Verbosity.Trace)
        {
            OutputHelper.OutputLog($"Print States  Software count: {(int)SOFTWARE.COUNT}", verbosity);
            for(int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                OutputHelper.OutputLog($"{((SOFTWARE)i).ToString()}:{softwareStates[i]}", verbosity);
            }
        }
    }


}

