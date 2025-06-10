// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using NetMQ;
using NetMQ.Sockets;
using System.Configuration;
using SharpConfig;
using PeabodyNetworkingLibrary;

namespace ControlPanel
{

    [Serializable]
    public class SocketBot
    {
        SharpConfig.Configuration config = new SharpConfig.Configuration();
        protected string EventTCPPort = SettingsManager.Instance.GetValueWithDefault("Ports","EventPort", "62026",true);
        protected string EventEPGMPort = SettingsManager.Instance.GetValueWithDefault("Ports", "EventEPGMPort", "62027");
        protected string StatusTCPPort = SettingsManager.Instance.GetValueWithDefault("Ports", "StatusPort", "62028",true);
        protected string StatusEPGMPort = SettingsManager.Instance.GetValueWithDefault("Ports", "StatusEPGMPort", "62029"); 
        protected string LocalIP = "127.0.0.1"; // required for multicast so it knows which interface to listen on.  Should be the IP of the local interface you want to subscribe to
        protected String IP;
        protected bool Running;
        protected bool IPHasChanged;
        protected bool UseMulticast;
        protected bool UseTCP;
        public String UnitName { get; protected set; }

        public SocketBot(String ip, String name)
        {
            this.IP = ip;
            this.Running = true;
            this.IPHasChanged = false;
            this.UnitName = name;
        }
        public virtual void Stop()
        {
            this.Running = false;
        }
        public void SetIP(String newIp)
        {
            this.IP = newIp;
            this.IPHasChanged = true;
            // Validation happens when we try to use it, not here
        }

        public void SetEventPorts(string eventTCPPort, string eventEPGMPort)
        {
            if (eventTCPPort.Length > 0)
            {
                EventTCPPort = eventTCPPort;
            }

            if (eventEPGMPort.Length > 0)
            {
                EventEPGMPort = eventEPGMPort;
            }
        }
        public void SetStatusPorts( string statusTCPPort, string statusEPGMPort)
        {           
            if(statusTCPPort.Length > 0)
            {
                StatusTCPPort = statusTCPPort;
            }

            if(statusEPGMPort.Length > 0)
            {
                StatusEPGMPort = statusEPGMPort;
            }
        }

        public void SetUseTCP(bool useTcp)
        {
            this.UseTCP = useTcp;
        }

        public void SetUseMulticast(bool useMulti)
        {
            this.UseMulticast = useMulti;
        }

        public bool IsValidIP(string addr)
        {
            //create our match pattern
            string pattern = @"^([1-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])(\.([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])){3}$";
            //create our Regular Expression object
            Regex check = new Regex(pattern);
            //boolean variable to hold the status
            bool valid = false;
            //check to make sure an ip address was provided
            if (String.IsNullOrEmpty(addr))
            {
                //no address provided so return false
                valid = false;
            }
            else
            {
                //address provided so use the IsMatch Method
                //of the Regular Expression object
                valid = check.IsMatch(addr, 0) || (!addr.Contains(' '));
            }
            //return the results
            return valid;
        }

    }

}
