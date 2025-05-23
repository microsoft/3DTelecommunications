// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using NetMQ;
using NetMQ.Sockets;
using PeabodyNetworkingLibrary;

namespace ControlPanel
{
    public class EventBot : SocketBot
    {
        private Thread eventBotThread;
        private object QueueMutex;

        protected Queue<byte[]> EventQueue;
        protected List<String> ListenerIPs;
        public EventBot(String ip)
            : base(ip, "Control Panel")
        {
            QueueMutex = new object();
            ListenerIPs = new List<string>();
            IPHasChanged = false;
            EventQueue = new Queue<byte[]>();

            CreateListenOnAllIPAddresses();
        }

        public void Start()
        {
            if (IsThreadAlive())
            {
                OutputHelper.OutputLog("Warning! attemping to start eventbot while it's still running");
            }
            eventBotThread = new Thread(PublisherThread);
            eventBotThread.Start();
        }

        public override void Stop()
        {
            base.Stop();
            //Note: not joining threads here, otherwise it blocks the main thread. Should instead check the IsAlive() to see if we've exited 
        }

        public bool IsAlive(bool checkThreadAlive)
        {
            bool overall = Running || (checkThreadAlive && IsThreadAlive());
            return overall;
        }

        public bool IsThreadAlive()
        {
            bool overall = false;
            if (eventBotThread != null)
            {
                overall = overall || eventBotThread.IsAlive;
            }
            return overall;
        }

        public void AddIP(String ip)
        {
            if(!ListenerIPs.Contains(ip))
            {
                ListenerIPs.Add(ip);
            }
        }

        public void RemoveIP(String ip)
        {
            if(ListenerIPs.Contains(ip))
            {
                ListenerIPs.Remove(ip);
            }
        }

        public void ReInit()
        {
            IPHasChanged = true;
        }


        private static List<String> GetLocalIPAddresses()
        {
            List<String> Addresses = new List<string>();
            var host = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var ip in host.AddressList)
            {
                if (ip.AddressFamily == AddressFamily.InterNetwork)
                {
                    Addresses.Add(ip.ToString());
                }
            }
            if (Addresses.Count > 0)
            {
                return Addresses;
            }
            throw new Exception("Local IP Address Not Found!");
        }

        private void CreateListenOnAllIPAddresses()
        {
            List<String> addresses = GetLocalIPAddresses();
            foreach (string address in addresses)
            {
                AddIP(address);
            }
            ReInit();
        }



        public void PublishEvent(CONTROL_PANEL_EVENT eventType)
        {
            PublishEvent(eventType, new byte[0]);
        }

        public void PublishEvent(CONTROL_PANEL_EVENT eventType, byte[] eventData)
        {
            byte[] dataArray = new byte[eventData.Length + 1];

            dataArray[0] = (byte)eventType;
            if (eventData.Count() > 0)
            {
                eventData.CopyTo(dataArray, 1);
            }

            lock (QueueMutex)
            {
                EventQueue.Enqueue(dataArray);
            }
        }

        public void PublisherThread()
        {
            AsyncIO.ForceDotNet.Force();
            OutputHelper.OutputLog(this.UnitName+" publisher Thread starting.");
            while (Running)
            {
                // validate the IP address
                if (ListenerIPs.Count > 0)
                {
                    using (var publisher = new PublisherSocket())
                    {
                        if (this.UseTCP)
                        {
                            foreach (String ip in ListenerIPs)
                            {
                                try
                                {
                                    OutputHelper.OutputLog($"{this.UnitName} Publisher binding to {ip}:{EventTCPPort} over TCP");
                                    publisher.Bind("tcp://" + ip + ":" + EventTCPPort);
                                }
                                catch (NetMQException e)
                                {
                                    OutputHelper.OutputLog($"Error binding publisher to {ip}:{EventTCPPort}. Message: {e.Message}");
                                    //DialogManager.Instance.UpdateMessage("Failed to bind publisher: " + e.Message, DialogManager.Instance.ErrorMessage);
                                }
                            }
                        }
                        if (this.UseMulticast)
                        {
                            OutputHelper.OutputLog(this.UnitName+" published is trying to use multicast.  This is NOT IMPLEMENTED.");
                        }

                        IPHasChanged = false; // avoid a double call on class instantiation and initial IP setting before starting the thread
                        while (Running)
                        {
                            if (IPHasChanged)
                            {
                                IPHasChanged = false;
                                break; // Start the outer loop over again to cause a reconnect.
                            }
                            int cnt = 0;
                            lock(QueueMutex)
                            {
                                cnt = EventQueue.Count;
                            }
                            while (cnt > 0)
                            {
                                byte[] message = null;
                                lock (QueueMutex)
                                {
                                    if(EventQueue.Count > 0)  // Odd race condition could cause us to get here and the queue be empty
                                    {
                                        message = EventQueue.Dequeue();
                                    }
                                    cnt = EventQueue.Count;
                                }
                                if (message != null)
                                {
                                    publisher.SendFrame(message);
                                }
                            }
                        }
                        publisher.Dispose();
                    }

                }
            }
        }
    }
}
