using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.ServiceProcess;
using System.Text;
using System.Configuration;
using System.Threading;
using NetMQ.Sockets;
using NetMQ;
using System.Net;
using System.Net.Sockets;
using PeabodyNetworkingLibrary;
using System.IO;

namespace KinectNanoCommunicatorService
{
    public partial class KinectNanoCommunicatorService : ServiceBase
    {
        readonly int SocketHighWaterMark = 10000; // default is 1000, but we surpass that when sending calibration data on smaller pipes
        readonly int NumberOfPacketsToPrintInDebug = 20;  //number of packets to print to the console if we're running in debug
        readonly int ReconnectTimeoutSeconds = 90; // should be at least 2x the IS_ALIVE_INTERNAL_MSEC defined in ControlPanelConnector.cpp
        readonly double SubscriberTimeoutSeparationInSeconds = 0.7;
        readonly int HalfSecondWaitInMilliseconds = 500;
        Thread subscriberTimeoutThread;  // checks when each subscriber last heard from a publisher, and if larger than ReconnectTimeoutSeconds, it disconnects and re-subscribes
        Thread publisherHeartbeatThread;
        const int publisherHeartbeatIntervalInSeconds = 90;
        
        List<SubscriberSocket> statusSubscribers; // receives status updates from applications and daemons
        Dictionary<byte[], DateTime> statusSubscriberLast;
        List<PublisherSocket> controlPanelStatusPublishers; // publishes app and daemon status updates to the control panel
        NetMQPoller SocketPoller;

        SubscriberSocket controlPanelEventSubscriber; // receives event requests from the control panel  
        DateTime controlPanelEventSubscriberLast;
        List<PublisherSocket> controlPanelEventPublishers; // (re)publishes control panel event requests to the Nanos (Binds to all interfaces)   

        System.Diagnostics.EventLog AppLog;

        SharpConfig.Configuration Config;
        readonly string configFilename = "3DTelemedicine.cfg";
        string configPath;

        object subscriberLock;
        object publisherLock;

        bool Running = false;
        static void Main(string[] args)
        {
            KinectNanoCommunicatorService service = new KinectNanoCommunicatorService();

            if (Environment.UserInteractive)
            {
                service.OnStart(args);
                Console.WriteLine("Press Enter to stop program");
                Console.Read();
                service.OnStop();
            }
            else
            {
                ServiceBase.Run(service);
            }

        }

        public KinectNanoCommunicatorService()
        {
            InitializeComponent();
            this.statusSubscribers = new List<SubscriberSocket>();
            this.controlPanelStatusPublishers = new List<PublisherSocket>();
            this.controlPanelEventPublishers = new List<PublisherSocket>();
            this.statusSubscriberLast = new Dictionary<byte[], DateTime>();

            subscriberLock = new object();
            publisherLock = new object();

            AppLog = new System.Diagnostics.EventLog();
            AppLog.Source = "Kinect Nano Communcator Service";
            AppLog.WriteEntry("Service started.");
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
        private int GetVerbosity()
        {
            return Config.Contains("KinectNanoCommunicatorService", "Verbosity")?Config["KinectNanoCommunicatorService"]["Verbosity"].IntValue : 0;
        }
        protected override void OnStart(string[] args)
        {
            // Look for an environment variable for the config file location
            configPath = Environment.GetEnvironmentVariable("3DTelemedicine_Dir");
            if (!String.IsNullOrEmpty(configPath))
            {
                configPath += Path.DirectorySeparatorChar + configFilename;
            }
            else
            {
                // Otherwise, try and load it from the current directory
                configPath = Environment.CurrentDirectory + Path.DirectorySeparatorChar + configFilename;
            }
            AppLog.WriteEntry("Setting configPath to " + configPath, EventLogEntryType.Information);

            try
            {
                Config = SharpConfig.Configuration.LoadFromFile(configPath);
                if (GetVerbosity() > 0)
                {
                    AppLog.WriteEntry("Verbose mode activated.  To reduce logging, set [KinectNanoCommunicatorService][Verbose] to 0", EventLogEntryType.Information);
                }
            }
            catch (Exception e)
            {
                AppLog.WriteEntry("Could not load the configuration file from " + configPath + ". Error: " + e.Message, EventLogEntryType.Error);
                Environment.Exit(-1);
            }

            // Read the config file to know how many threads to create
            int NanoCount = Config["DepthGeneration"]["DepthCameraCount"].IntValue;
            int appStartingPort = Config["Ports"]["DepthPodApplicationStatusPortBase"].IntValue;
            int daemonStartingPort = Config["Ports"]["DepthPodDaemonPortBase"].IntValue;

            Running = true;
            SocketPoller = new NetMQPoller();

            List<String> ips = GetLocalIPAddresses();
            // Creat the connection strings
            string controlPanelEventPublisherTCPUri = $"tcp://{Config["Network"]["ControlPanelIPAddress"].StringValue}:{Config["Ports"]["EventPort"].StringValue}";
            foreach (string ip in ips)
            {
                string nanoEventPublisherTCPUri = $"tcp://{ip}:{Config["Ports"]["KNCSPublishEventPort"].StringValue}";
                if (GetVerbosity() > 1)
                    AppLog.WriteEntry("Creating control panel event publisher on port " + nanoEventPublisherTCPUri, EventLogEntryType.Information);
                // Create a publisher socket that the Kinect Nano's can subscribe to
                PublisherSocket publisherSocket = new PublisherSocket(nanoEventPublisherTCPUri);
                publisherSocket.Options.ReceiveHighWatermark = SocketHighWaterMark;
                publisherSocket.Options.SendHighWatermark = SocketHighWaterMark;
                controlPanelEventPublishers.Add(publisherSocket);
            }
            controlPanelEventSubscriber = new SubscriberSocket();
            controlPanelEventSubscriber.Options.ReceiveHighWatermark = SocketHighWaterMark;
            controlPanelEventSubscriber.Options.SendHighWatermark = SocketHighWaterMark;
            controlPanelEventSubscriber.Options.Identity = Encoding.Default.GetBytes(controlPanelEventPublisherTCPUri);
            if(GetVerbosity() > 1)
                AppLog.WriteEntry("Creating control panel event subscriber on port " + Encoding.Default.GetString(controlPanelEventSubscriber.Options.Identity), EventLogEntryType.Information);
            controlPanelEventSubscriber.Connect(Encoding.Default.GetString(controlPanelEventSubscriber.Options.Identity));
            controlPanelEventSubscriber.Subscribe(""); // Subscribe to everything
            controlPanelEventSubscriber.ReceiveReady += ForwardControlPanelEvent;
            controlPanelEventSubscriberLast = DateTime.Now;
            // Create an event thread that subscribes to the control panel publisher socket
            SocketPoller.Add(controlPanelEventSubscriber);
            // For each Kinect nano in the config file
            for(int i = 0; i < NanoCount; i++)
            {
                string nanoName = "Nano" + (i+1);
                int octet = i + 1;
                string nanoIP = $"{Config["Network"]["DepthPodIPBase"].StringValue}{octet}";
                int currentPort = daemonStartingPort + (i+1);
                //// create a daemon publisher socket that the control panel can connect to
                PublisherSocket daemonPublisherSocket = new PublisherSocket();
                daemonPublisherSocket.Options.SendHighWatermark = SocketHighWaterMark;
                daemonPublisherSocket.Options.ReceiveHighWatermark = SocketHighWaterMark;
                if(GetVerbosity() > 0)
                    AppLog.WriteEntry("Creating daemon publisher socket for "+nanoName+" on port " + currentPort, EventLogEntryType.Information);
                foreach (string ip in ips)
                {
                    daemonPublisherSocket.Bind($"tcp://{ip}:{currentPort}");
                }
                controlPanelStatusPublishers.Add(daemonPublisherSocket);
                //// create a daemon status thread
                ////// subscribe to the kinect daemon's publishing IP:socket
                SubscriberSocket daemonSubscriberSocket = new SubscriberSocket();
                daemonSubscriberSocket.Options.ReceiveHighWatermark = SocketHighWaterMark;
                daemonSubscriberSocket.Options.SendHighWatermark = SocketHighWaterMark;
                daemonSubscriberSocket.Subscribe("");
                int idx = statusSubscribers.Count();
                string daemonSubscriberSocketUri = $"tcp://{nanoIP}:{Config["Ports"]["NanoDaemonStatusPort"].StringValue}";
                daemonSubscriberSocket.Options.Identity = Encoding.Default.GetBytes(daemonSubscriberSocketUri);
                if(GetVerbosity() > 0)
                    AppLog.WriteEntry($"Creating daemon subscriber socket for Nano {i} on port {Encoding.Default.GetString(daemonSubscriberSocket.Options.Identity)}", EventLogEntryType.Information);
                daemonSubscriberSocket.Connect(Encoding.Default.GetString(daemonSubscriberSocket.Options.Identity));
                daemonSubscriberSocket.ReceiveReady += (sender, e) => SubscriberSocket_ReceiveReady(sender, e, daemonPublisherSocket);
                statusSubscriberLast.Add(daemonSubscriberSocket.Options.Identity, DateTime.Now + TimeSpan.FromSeconds(SubscriberTimeoutSeparationInSeconds*statusSubscriberLast.Count()));
                statusSubscribers.Add(daemonSubscriberSocket);
                SocketPoller.Add(daemonSubscriberSocket);

                //// create an application publisher socket that the control panel can connect to
                currentPort = appStartingPort + (i + 1);
                PublisherSocket applicationPublisherSocket = new PublisherSocket();
                applicationPublisherSocket.Options.SendHighWatermark = SocketHighWaterMark;
                applicationPublisherSocket.Options.ReceiveHighWatermark = SocketHighWaterMark;
                if(GetVerbosity() > 0)
                    AppLog.WriteEntry($"Creating app publisher socket for Nano {i} on port {currentPort}", EventLogEntryType.Information);
                foreach (string ip in ips)
                {
                    applicationPublisherSocket.Bind($"tcp://{ip}:{ currentPort}");
                }
                controlPanelStatusPublishers.Add(daemonPublisherSocket);
                //// create an application status thread
                ////// subscribe to the kinect application's publishing IP:socket
                SubscriberSocket applicationSubscriberSocket = new SubscriberSocket();
                applicationSubscriberSocket.Options.SendHighWatermark = SocketHighWaterMark;
                applicationSubscriberSocket.Options.ReceiveHighWatermark = SocketHighWaterMark;
                applicationSubscriberSocket.Subscribe("");
                idx = statusSubscribers.Count();
                string applicationSubscriberSocketUri = $"tcp://{nanoIP}:{Config["Ports"]["NanoApplicationStatusPort"].StringValue}";
                applicationSubscriberSocket.Options.Identity = Encoding.Default.GetBytes(applicationSubscriberSocketUri);
                applicationSubscriberSocket.Connect(Encoding.Default.GetString(applicationSubscriberSocket.Options.Identity));
                applicationSubscriberSocket.ReceiveReady += (sender, e) => SubscriberSocket_ReceiveReady(sender, e, applicationPublisherSocket);
                if(GetVerbosity() > 0)
                    AppLog.WriteEntry($"Creating app subscriber socket for Nano {i} on port {nanoIP}:{Config["Ports"]["NanoApplicationStatusPort"].StringValue}", EventLogEntryType.Information);
                statusSubscriberLast.Add(applicationSubscriberSocket.Options.Identity, DateTime.Now + TimeSpan.FromSeconds(SubscriberTimeoutSeparationInSeconds * statusSubscriberLast.Count()));
                statusSubscribers.Add(applicationSubscriberSocket);
                SocketPoller.Add(applicationSubscriberSocket);
            }

            SocketPoller.RunAsync();

            subscriberTimeoutThread = new Thread(SubscriberTimeoutCheck);
            subscriberTimeoutThread.Start();

            publisherHeartbeatThread = new Thread(PublisherHeartbeat);
            publisherHeartbeatThread.Start();
        }

        private void PublisherHeartbeat()
        {
            byte[] data = new byte[1];
            data[0] = (byte)CONTROL_PANEL_EVENT.CONTROL_PANEL_STATE_UPDATE_REQUESTED;
            if (Config["KinectNanoCommunicatorService"].Contains("DisableHeartbeat") && Config["KinectNanoCommunicatorService"]["DisableHeartbeat"].BoolValue)
            {
                AppLog.WriteEntry("Disabling heartbeat thread per config DisableHeartbeat value.", EventLogEntryType.Warning);
                return;
            }
            while (Running)
            {
                foreach (PublisherSocket p in controlPanelEventPublishers)
                {
                    lock (publisherLock)
                    {
                        p.SendFrame(data);
                    }
                }
                int elapsedMilliseconds = 0;
                while(Running && elapsedMilliseconds < publisherHeartbeatIntervalInSeconds*1000)
                {
                    System.Threading.Thread.Sleep(HalfSecondWaitInMilliseconds);
                    elapsedMilliseconds += HalfSecondWaitInMilliseconds;
                }
            }
        }

        private void SubscriberTimeoutCheck()
        {
            while(Running)
            {
                TimeSpan elapsed;
                // Check the last timestamps for all subscribers.  If one is too stale, reconnect it
                for(int i = 0; i < statusSubscribers.Count(); i++)
                {
                    elapsed = DateTime.Now - statusSubscriberLast[statusSubscribers[i].Options.Identity];
                    if(Convert.ToInt32(elapsed.TotalSeconds) > ReconnectTimeoutSeconds)
                    {
                        string Uri = Encoding.Default.GetString(statusSubscribers[i].Options.Identity);
                        string infoString = $"Subscriber connected to {Uri} is stale {elapsed.TotalSeconds}s.  Re-connecting.";
                        if (Environment.UserInteractive)
                        {
                            Console.WriteLine(infoString);
                        }
                        if(GetVerbosity() > 1)
                            AppLog.WriteEntry(infoString, EventLogEntryType.Information);
                        lock (subscriberLock)
                        {
                            SocketPoller.Remove(statusSubscribers[i]);
                            Thread.Sleep(HalfSecondWaitInMilliseconds); // give the poller time to fully remove before disconnecting
                            try
                            {
                                statusSubscribers[i].Disconnect(Uri);
                            }
                            catch (NetMQ.EndpointNotFoundException)
                            {
                                infoString = $"Could not disconnect {Uri}, endpoint not found.  Re-connecting anyway";
                                AppLog.WriteEntry(infoString, EventLogEntryType.Warning);
                                if (Environment.UserInteractive)
                                {
                                    Console.WriteLine(infoString);
                                }
                            }
                            finally
                            {
                                Thread.Sleep(HalfSecondWaitInMilliseconds); // give the socket time to fully disconnect before re-connecting
                                statusSubscribers[i].Subscribe("");
                                statusSubscribers[i].Connect(Uri);
                                statusSubscriberLast[statusSubscribers[i].Options.Identity] = DateTime.Now;
                            }
                            SocketPoller.Add(statusSubscribers[i]);
                        }
                    }
                }
                // Also check the controlPanelEventSubscriber
                elapsed = DateTime.Now - controlPanelEventSubscriberLast;
                if(Convert.ToInt32(elapsed.TotalSeconds) > ReconnectTimeoutSeconds)
                {
                    SocketPoller.Remove(controlPanelEventSubscriber);
                    Thread.Sleep(HalfSecondWaitInMilliseconds); // give the poller time to fully remove before disconnecting
                    string Uri = Encoding.Default.GetString(controlPanelEventSubscriber.Options.Identity);
                    string infoString = $"Control panel event subscriber connected to {Uri} is stale {elapsed.TotalSeconds}s.  Re-connecting.";
                    if (Environment.UserInteractive)
                    {
                        Console.WriteLine(infoString);
                    }
                    if(GetVerbosity() > 1)
                        AppLog.WriteEntry(infoString, EventLogEntryType.Information);
                    lock (subscriberLock)
                    {
                        Thread.Sleep(HalfSecondWaitInMilliseconds); // give the socket time to fully disconnect before reconnecting
                        controlPanelEventSubscriber.Disconnect(Uri);
                        controlPanelEventSubscriber.Subscribe("");
                        controlPanelEventSubscriber.Connect(Uri);
                        controlPanelEventSubscriberLast = DateTime.Now;
                    }
                    SocketPoller.Add(controlPanelEventSubscriber);
                }
                Thread.Sleep(HalfSecondWaitInMilliseconds);
            }
        }

        private void SubscriberSocket_ReceiveReady(object sender, NetMQSocketEventArgs e, PublisherSocket publisher)
        {
            statusSubscriberLast[e.Socket.Options.Identity] = DateTime.Now;
            ////// when a status update is received, re-transmit over the daemon publisher socket
            byte[] results;
            bool bytesAvailable = true;
            while (this.Running && bytesAvailable)
            {
                lock (subscriberLock)
                {
                    e.Socket.TryReceiveFrameBytes(System.TimeSpan.FromMilliseconds(100), out results, out bytesAvailable);
                }
                if (results != null)
                {
                    if (results[0] != Convert.ToByte(PeabodyNetworkingLibrary.CPC_STATUS.RUNNING)) // reduce noise by not printing the READY signals
                    {
                        string infoString = $"Sending[{results.Length}] {Utility.PeabodyPacketToString(results, typeof(PeabodyNetworkingLibrary.CPC_STATUS), NumberOfPacketsToPrintInDebug)} from {e.Socket.Options.LastEndpoint} {statusSubscriberLast[e.Socket.Options.Identity].ToString()}";
                        if (Environment.UserInteractive)
                        {
                            Console.WriteLine(infoString);
                        }
                        if(GetVerbosity() > 1)
                            AppLog.WriteEntry(infoString, EventLogEntryType.Information);
                    }
                    lock (publisherLock)
                    {
                        publisher.SendFrame(results);
                    }
                }
            }
            statusSubscriberLast[e.Socket.Options.Identity] = DateTime.Now;
        }

        private void ForwardControlPanelEvent(object sender, NetMQSocketEventArgs e)
        {
            controlPanelEventSubscriberLast = DateTime.Now;
            bool moreAvailable = true;
            byte[] results;
            while (moreAvailable && this.Running)
            {
                lock (subscriberLock)
                {
                    controlPanelEventSubscriber.TryReceiveFrameBytes(System.TimeSpan.FromMilliseconds(100), out results, out moreAvailable);
                }
                if (results != null)
                {
                    if(results[0] == Convert.ToByte(PeabodyNetworkingLibrary.CONTROL_PANEL_EVENT.BUILD_VERSION_REQUESTED))
                    {
                        // I could also forward my own build version here, but I don't right now, because control panel can't distinguish between the pods and the KNCS application
                    }
                    string infoString = $"Forwarding control panel event: [{results.Length}] {Utility.PeabodyPacketToString(results, typeof(PeabodyNetworkingLibrary.CONTROL_PANEL_EVENT), NumberOfPacketsToPrintInDebug)}";
                    if (Environment.UserInteractive)
                    {                        
                        Console.WriteLine(infoString);
                    }
                    if(GetVerbosity() > 1)
                    {
                        AppLog.WriteEntry(infoString, EventLogEntryType.Information);
                    }
                    foreach (PublisherSocket p in controlPanelEventPublishers)
                    {
                        lock (publisherLock)
                        {
                            p.SendFrame(results);
                        }
                    }
                }
            }
            // Update again in case it was a large transmission that took a while
            controlPanelEventSubscriberLast = DateTime.Now;
        }

        protected override void OnStop()
        {
            SocketPoller.Stop();
            AppLog.WriteEntry("OnStop called.  Service terminating.", EventLogEntryType.Warning);
            this.Running = false;
            publisherHeartbeatThread.Join();
            foreach (SubscriberSocket s in statusSubscribers)
            {
                lock (subscriberLock)
                {
                    s.Close();
                }
            }
            foreach (PublisherSocket p in controlPanelStatusPublishers)
            {
                lock (publisherLock)
                {
                    p.Close();
                }
            }
            subscriberTimeoutThread.Join();
            foreach (PublisherSocket p in controlPanelEventPublishers)
            {
                lock (publisherLock)
                {
                    p.Close();
                }
            }
            controlPanelEventSubscriber.Close();
        }
    }
}
