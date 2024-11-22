using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using NetMQ;
using NetMQ.Sockets;
using PeabodyNetworkingLibrary;
using UnityEngine;

namespace ControlPanel
{
    public delegate void StatusResponseDelegate(byte[] update);
    public class StatusBot : SocketBot
    {
        protected const int MAX_HB_TIMEOUT_SEC = 15;
        protected const int WRN_HB_TIMEOUT_SEC = 10;
        public const int C3DTM_KILL_TIMEOUT_SEC = 8;
        protected const int MAX_FRAMES_PER_RECEIVE_CALL = 100;

        protected int DepthGenID; // if this is a depth gen bot, we set this to correspond to its index in the array of depth gen bots, threads, and GUI controls
        protected int depthGenMachineID;
        protected DateTime LastHBReceived;

        protected bool heartBeatRunning;
        private Thread HeartbeatThread;

        private Dictionary<CPC_STATUS, StatusResponseDelegate> statusResponseTable;

        public ComponentStatusContainer componentStatusContainer;

        private string receivedLogDestinationPath = Path.GetTempPath();

        private NetMQPoller controlPanelEventPoller;
        private SubscriberSocket eventSubscriberSocket;
        private PeabodyNetworkingLibrary.VersionData versionData;

        public StatusBot(String ip, String name, ComponentStatusContainer csc, string overrideTCPPort ="", string overrideEPGMPORT="")
            : base(ip, name)
        {
            this.DepthGenID = -1;
            this.depthGenMachineID = -1;
            ResetHBTime();

            this.componentStatusContainer = csc;

            statusResponseTable = new Dictionary<CPC_STATUS, StatusResponseDelegate>();

            if(overrideTCPPort.Length > 0)
            {
                StatusTCPPort = overrideTCPPort;
            }
            if(overrideEPGMPORT.Length > 0)
            {
                StatusEPGMPort = overrideEPGMPORT;
            }
        }

        // create a destructor
        ~StatusBot()
        {
            OutputHelper.OutputLog("StatusBot destructor called");
            Stop();
        }

        private void SetDefaultSocketOptions(SocketOptions options)
        {
            options.ReceiveBuffer = 1000000;
            options.ReceiveHighWatermark = 10000;
            options.SendBuffer = 1000000;
            options.SendHighWatermark = 10000;
        }

        public void Start()
        {
            if (IsThreadAlive())
            {
                OutputHelper.OutputLog("Warning! attemping to start statusbot while it's still running");
                return; // don't actually reset the threads
            }
            heartBeatRunning = true;
            HeartbeatThread = new Thread(CheckHeartbeatThread);
            HeartbeatThread.Start();

            controlPanelEventPoller = new NetMQPoller();
            // validate the IP address
            if (IsValidIP(this.IP))
            {
                eventSubscriberSocket = new SubscriberSocket();
                SetDefaultSocketOptions(eventSubscriberSocket.Options);
                try
                {
                    OutputHelper.OutputLog("Subscriber connecting to " + this.UnitName + " publisher at " + this.IP + ":" + this.StatusTCPPort + " over TCP");

                    //extra debug DNS host name check
                    IPAddress[] hostAddresses = Dns.GetHostAddresses(this.IP);
                    eventSubscriberSocket.Connect("tcp://" + this.IP + ":" + StatusTCPPort);
                }
                catch (System.Net.Sockets.SocketException e)
                {
                    OutputHelper.OutputLog("SocketException - Could not connect to " + this.IP + ": " + e.Message);
                }
                catch (NetMQException e)
                {
                    OutputHelper.OutputLog("NetMQException - Could not connect to " + this.IP + ": " + e.ErrorCode);
                }
                finally
                {
                    eventSubscriberSocket.Subscribe(""); // Subscribe to everything
                    eventSubscriberSocket.ReceiveReady += ProcessStatusUpdates;
                    controlPanelEventPoller.Add(eventSubscriberSocket);
                    controlPanelEventPoller.RunAsync();
                }
            }

        }

        public override void Stop()
        {
            OutputHelper.OutputLog($"{UnitName}::Stop called.", OutputHelper.Verbosity.Debug);
            try
            {
                if (controlPanelEventPoller != null && controlPanelEventPoller.IsRunning)
                {
                    controlPanelEventPoller.Stop();
                }
            }
            catch (TerminatingException e)
            {
                OutputHelper.OutputLog("TerminatingException in StatusBot destructor: " + e.Message);
            }
            catch (Exception e)
            {
                OutputHelper.OutputLog("Exception in StatusBot destructor: " + e.Message);
            }
            finally
            {
                base.Stop();
                componentStatusContainer.SetToStopped();
            }
            //Note: not joining threads here, otherwise it blocks the main thread. Should instead check the IsAlive() to see if we've exited 
        }

        public void SetLogCollectionPath(string path)
        {
            receivedLogDestinationPath = path;
        }

        public bool IsAlive(bool checkThreadAlive)
        {
            bool overall = Running || (checkThreadAlive && IsThreadAlive());
            return overall;       
        }

        public void SaveLogFile(string fileName, byte[] fileData)
        {
            Directory.CreateDirectory(receivedLogDestinationPath + "\\" + UnitName);
            File.WriteAllBytes(receivedLogDestinationPath + "\\" + UnitName + "\\" + fileName, fileData);
            OutputHelper.OutputLog($"Wrote received log file to {receivedLogDestinationPath}\\{UnitName}\\{fileName}");
        }

        private bool IsThreadAlive()
        {
            bool overall = false;
            if (controlPanelEventPoller != null)
            {
                overall = overall || controlPanelEventPoller.IsRunning;
            }
            if (HeartbeatThread != null)
            {
                overall = overall || HeartbeatThread.IsAlive;
            }
            return overall;
        }

        public void RegisterUpdateFunction(CPC_STATUS eventType, StatusResponseDelegate responseFunction)
        {
            if (statusResponseTable.ContainsKey(eventType))
            {
                OutputHelper.OutputLog($"{UnitName} is registering an ADDITIONAL response for {eventType.ToString()}", OutputHelper.Verbosity.Trace);
                statusResponseTable[eventType] += responseFunction;
            }
            else
            {
                OutputHelper.OutputLog($"{UnitName} is registering a NEW response for {eventType.ToString()}", OutputHelper.Verbosity.Trace);
                statusResponseTable.Add(eventType, responseFunction);
            }
        }

        public void SetDepthGenID(int id)
        {
            this.DepthGenID = id;
        }

        public int DepthGenMachineID
        {
            get
            {
                return this.depthGenMachineID;
            }
            set
            {
                this.depthGenMachineID = value;
            }
        }

        private void CheckHeartbeatThread()
        {
            while (this.Running)
            {
                if (heartBeatRunning)
                {
                    CheckHeartbeatStatus();
                }
                Thread.Sleep(2000);
            }
        }
        public void ResetHBTime()
        {
            heartBeatRunning = true;
            LastHBReceived = new DateTime();
        }
        private bool HBReceived()
        {
            return LastHBReceived != new DateTime();
        }

        private TimeSpan TimeSinceLastHB()
        {
            return (DateTime.UtcNow - LastHBReceived);
        }

        private void CheckHeartbeatStatus()
        {
            if (HBReceived())
            {
                TimeSpan timeSinceLastHB = TimeSinceLastHB();
                if (timeSinceLastHB > TimeSpan.FromSeconds(MAX_HB_TIMEOUT_SEC))
                {
                    OutputHelper.OutputLog(String.Format("Havent heard from {0} in > {1} sec. Resetting known state...", this.UnitName, MAX_HB_TIMEOUT_SEC));


                    componentStatusContainer.SetToTimedOut();

                }
                else if (timeSinceLastHB > TimeSpan.FromSeconds(WRN_HB_TIMEOUT_SEC))
                {
                    OutputHelper.OutputLog(String.Format("Havent heard from {0} in {1} sec", this.UnitName, timeSinceLastHB.TotalSeconds));
                }
            }
        }


        public void UpdateTimeLastHBReceived()
        {
            LastHBReceived = DateTime.UtcNow;
        }

        public void DefaultStatusUpdateSetup(bool setupFPS = true)
        {
            // Clear the table (prevents adding the same calls twice if this is called more than once)
            statusResponseTable.Clear();
            RegisterUpdateFunction(CPC_STATUS.READY,
                delegate (byte[] update)
                {
                    UpdateTimeLastHBReceived();
                    if(componentStatusContainer != null)
                    {
                        componentStatusContainer.SetToReady();
                    }
                });
            RegisterUpdateFunction( CPC_STATUS.BUILD_VERSION,
                delegate ( byte [] update )
                {
                    OutputHelper.OutputLog($"{UnitName} default updater calling only SaveVersionData");
                    SaveVersionData(update);
                } );

            RegisterUpdateFunction(CPC_STATUS.RECEIVED_NEW_DATA, 
                delegate (byte[] update)
            {
                OutputHelper.OutputLog($"{UnitName} received valid calibration data.");
                    if(componentStatusContainer != null)
                    {
                        componentStatusContainer.componentStatus.NewDataReceived = true;
                    }
            });

            RegisterUpdateFunction( CPC_STATUS.RUNNING,
               delegate (byte[] update)
               {
                    if(componentStatusContainer != null)
                    {
                       componentStatusContainer.SetToRunning();
                    }
                   heartBeatRunning = true;
               });

            RegisterUpdateFunction(CPC_STATUS.STOPPED,
               delegate (byte[] update)
               {
                    if(componentStatusContainer != null)
                    {
                       componentStatusContainer.SetToStopped();
                    }
                   if (setupFPS)//don't update if we dont have fpsSETUp
                   {
                       componentStatusContainer.SetFPSText(new double[] {0.0, 0.0});
                   }
                   heartBeatRunning = false; // no need to detect heartbeat if we know it stopped correctly
               });
            RegisterUpdateFunction(CPC_STATUS.IS_ALIVE, (byte[] update) =>
            {
                OutputHelper.OutputLog($"{UnitName} Daemon isAlive", OutputHelper.Verbosity.Debug);
                UpdateTimeLastHBReceived();
            });
           
            //FPS related 
            if (setupFPS)
            {
                RegisterUpdateFunction(CPC_STATUS.FPS,
                delegate (byte[] update)
                {
                    UpdateTimeLastHBReceived();
                  
                    if(update.Length < sizeof(double)+sizeof(byte))
                    {
                        int expectedSize = sizeof(double)+sizeof(byte);
                        OutputHelper.OutputLog($"FPS packetsize ({expectedSize} expected): " + update.Length);
                        return;
                    }
                    double fps = BitConverter.ToDouble(update, sizeof(byte));
                    double temperature = 0.0;
                    if(update.Length > sizeof(double)+sizeof(byte))
                    {
                        temperature = BitConverter.ToDouble(update, sizeof(byte) + sizeof(double));
                    }
                    if(fps > componentStatusContainer.componentStatus.FPS_max)
                    {
                        componentStatusContainer.componentStatus.FPS_max = fps;
                    }
                    if(fps < componentStatusContainer.componentStatus.FPS_min)
                    {
                        componentStatusContainer.componentStatus.FPS_min = fps;
                    }
                    //this.Form.OutputLog("Fusion FPS: " + fps.ToString());
                    componentStatusContainer.SetFPSText(new double[] {fps, temperature});
                    componentStatusContainer.SetToRunning();
                    heartBeatRunning = true;

                    // Determining if the number of acquired frames was also sent as payload.
                    // This typically occurs during video-based calibration and allows the user
                    // to easily determining when video recording started.
                    if(update.Length >= 1+sizeof(double)+sizeof(int))
                    {
                        int frameNumber = (int)System.BitConverter.ToInt32(update, 1+sizeof(double));
                        componentStatusContainer.SetFrameNumberText(frameNumber);
                    }
                    // Don't update status here, FPS status is just used for sending data back to the control panel
                });
            }                 
        }

        public PeabodyNetworkingLibrary.VersionData GetVersionData()
        {
            return versionData;
        }

        public void SetVersionData(PeabodyNetworkingLibrary.VersionData data)
        {
            versionData = data;
        }

        public void SaveVersionData(byte[] update)
        {
            PeabodyNetworkingLibrary.Utility.ParseVersionDataFromPacket(update, ref versionData);

            OutputHelper.OutputLog($"{UnitName} build version {versionData.Description}", OutputHelper.Verbosity.Info);
        }
        private void ProcessStatusUpdates(object sender, NetMQSocketEventArgs e)
        {
            byte[] update;
            int framesReceived = 0;
            while (this.Running && framesReceived < MAX_FRAMES_PER_RECEIVE_CALL) 
            {
                bool more = false;
                bool success = e.Socket.TryReceiveFrameBytes(System.TimeSpan.FromMilliseconds(100), out update, out more);
                if(!success)
                {
                    // no bytes to read
                    return;
                }
                if (update != null && update.Length > 0)
                {
                    // process bytes
                    lock (baseStateChangeLock)
                    {
                        if (this.Running)
                        {
                            CPC_STATUS StatusType = (CPC_STATUS)update[0];
                            OutputHelper.OutputLog($"{UnitName} got a status update.  Type {StatusType.ToString()}: packet size: {update.Length}", OutputHelper.Verbosity.Trace);
                            if (statusResponseTable.ContainsKey(StatusType))
                            {
                                statusResponseTable[StatusType](update);
                            }
                            else
                            {
                                OutputHelper.OutputLog(UnitName + " Got an unexpected status type: " + StatusType.ToString() + " " + update[0]);
                            }
                        }
                    }
                    framesReceived++;  //only receive MAX_FRAMES_PER_RECEIVE_CALL to avoid starving any other sockets
                }
                if(!more)
                {
                    // no more frames, done.
                    return;
                }
            }
        }
    }

}
