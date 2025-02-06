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
using System.Windows.Forms.Design;
using AsyncIO;
using NetMQ;
using NetMQ.Sockets;
using NLog.LayoutRenderers;
using PeabodyNetworkingLibrary;


namespace ControlPanel
{
    public enum DAEMON_GROUP_STATE
    {
        UNKNOWN, // +1 daemon with this
        ERROR, // + 1 daemon with this 

        NOT_RUNNING, // ALL daemons are not running
        LAUNCHING,  //Not ALL daemons are RUNNING, but not errors
        RUNNING, // ALL daemons are this


    }

    public delegate void StatusResponseDelegate(byte[] update);
    public class StatusBot : SocketBot, INotifyPropertyChanged
    {
        private System.Threading.Timer _updateTimer;
        private bool _updatePending;
        private SOFTWARE_STATE[] _softwareStates;
        
        public SOFTWARE_STATE[] softwareStates { get => _softwareStates; protected set
            {
                if (softwareStates == null)
                {
                    _softwareStates = value;
                }
                else
                {
                    for (int i = 0; i < _softwareStates.Length; i++)
                    {
                        if (_softwareStates[i] != value[i])
                        {
                            _softwareStates[i] = value[i];
                            OnPropertyChanged(nameof(softwareStates));
                        }
                    }
                }
            }
        }
        // Need these so that we can display the status of each daemon in the DataGridView
        public string LinuxDaemonStatus => softwareStates[(int)SOFTWARE.LINUX_DAEMON].ToString();
        public string WindowsServiceStatus => softwareStates[(int)SOFTWARE.WINDOWS_SERVICE].ToString();
        public string CaptureStatus => softwareStates[(int)SOFTWARE.CAPTURE].ToString();
        public string CalibrationStatus => softwareStates[(int)SOFTWARE.CALIBRATION].ToString();

        public string FusionStatus => softwareStates[(int)SOFTWARE.FUSION].ToString();
        public string CalibrationSoftwareStatus => softwareStates[(int)SOFTWARE.CALIBRATION].ToString();
        public string RenderStatus => softwareStates[(int)SOFTWARE.RENDER].ToString();
        public string VersionString => VersionData.Description;
        public string FPS => componentStatus.FPS.ToString();
        public string MaxFPS => componentStatus.FPS_max.ToString();
        public string MinFPS => componentStatus.FPS_min.ToString();
        public string MinVoltage => componentStatus.Voltage_min.ToString();
        public string MaxVoltage => componentStatus.Voltage_max.ToString();
        public string Temp => componentStatus.Temperature.ToString();

        public string ComponentStatus => componentStatus.Status.ToString();
        public string CameraName => "Camera " + (DepthGenID + 1).ToString();

        public string Frames => componentStatus.FrameNum.ToString();

        public event PropertyChangedEventHandler PropertyChanged;

        public void OnPropertyChanged(string propertyName)
        {
            if(_controlPanel.InvokeRequired)
            {
                _controlPanel.Invoke(new Action(() => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName))));
            }
            else
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
                if (!_updatePending)
                {
                    _updatePending = true;
                    _updateTimer.Change(100, Timeout.Infinite);
                }
            }
        }


        protected const int MAX_HB_TIMEOUT_SEC = 15;
        protected const int WRN_HB_TIMEOUT_SEC = 10;
        protected const double MAX_HEART_RATE = 2.0;
        public const int C3DTM_KILL_TIMEOUT_SEC = 8;
        protected const int MAX_FRAMES_PER_RECEIVE_CALL = 100;

        public int DepthGenID { get; protected set; } // if this is a depth gen bot, we set this to correspond to its index in the array of depth gen bots, threads, and GUI controls
        protected int depthGenMachineID;
        protected DateTime LastHBReceived;
        public double HeartRate
        {
            get
            {
                return 1.0 / TimeSinceLastHB().TotalSeconds;
            }
        }

        protected bool heartBeatRunning;
        private Thread HeartbeatThread;

        private Dictionary<CPC_STATUS, StatusResponseDelegate[]> statusResponseTable;

        private string receivedLogDestinationPath = Path.GetTempPath();

        private NetMQPoller controlPanelEventPoller;
        private SubscriberSocket eventSubscriberSocket;
        public VersionData VersionData;

        // Status State
        public ComponentStatus componentStatus;

        private ControlPanel _controlPanel;

        public StatusBot(String ip, String name, ControlPanel controlPanel, string overrideTCPPort ="", string overrideEPGMPORT="")
            : base(ip, name)
        {
            this._controlPanel = controlPanel;
            this.DepthGenID = -1;
            this.depthGenMachineID = -1;
            _updateTimer = new System.Threading.Timer(UpdateUI, null, Timeout.Infinite, Timeout.Infinite);
            ResetHBTime();
            ResetKnownStatus();

            statusResponseTable = new Dictionary<CPC_STATUS, StatusResponseDelegate[]>();

            if(overrideTCPPort.Length > 0)
            {
                StatusTCPPort = overrideTCPPort;
            }
            if(overrideEPGMPORT.Length > 0)
            {
                StatusEPGMPort = overrideEPGMPORT;
            }

        }

        private void UpdateUI(object state)
        {
            if (_controlPanel.InvokeRequired)
            {
                _controlPanel.Invoke(new Action(() => UpdateUI(state)));
                return;
            }

            // Perform the UI update
            _controlPanel.Refresh();
            _updatePending = false;
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

        public void Reconnect()
        {
            if (eventSubscriberSocket != null)
            {
                OutputHelper.OutputLog("Reinitializing status bot for pod " + this.DepthGenID);
                eventSubscriberSocket.Disconnect("tcp://" + this.IP + ":" + StatusTCPPort);
                Thread.Sleep(100);
                eventSubscriberSocket.Connect("tcp://" + this.IP + ":" + StatusTCPPort);
            }
            else if(!IsThreadAlive())
            {
                Start();
            }
        }

        public void Start()
        {
            if (IsThreadAlive())
            {
                OutputHelper.OutputLog("Warning! attemping to start statusbot while it's still running");
                Reconnect();
                return; // don't actually reset the threads
            }
            OutputHelper.OutputLog($"Starting up status bot for pod {this.DepthGenID}");
            heartBeatRunning = true;
            HeartbeatThread = new Thread(CheckHeartbeatThread);
            HeartbeatThread.Start();

            controlPanelEventPoller = new NetMQPoller();
            // validate the IP address
            if (IsValidIP(this.IP))
            {
                eventSubscriberSocket = new SubscriberSocket();
                SetDefaultSocketOptions(eventSubscriberSocket.Options);
                eventSubscriberSocket.Options.TcpKeepalive = true;
                eventSubscriberSocket.Options.ReconnectInterval = TimeSpan.FromSeconds(5);
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
            componentStatus.Status = Status.Ready;
        }

        public override void Stop()
        {
            OutputHelper.OutputLog($"{UnitName}::Stop called.", OutputHelper.Verbosity.Debug);
            try
            {
                if (controlPanelEventPoller != null && controlPanelEventPoller.IsRunning)
                {
                    controlPanelEventPoller.StopAsync();
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
                OutputHelper.OutputLog($"{UnitName}::Calling Base Stop.", OutputHelper.Verbosity.Debug);
                base.Stop();
                componentStatus.Status = Status.Stopped;
            }
            OutputHelper.OutputLog($"{UnitName}::Stop completed.", OutputHelper.Verbosity.Debug);
            //Note: not joining threads here, otherwise it blocks the main thread. Should instead check the IsAlive() to see if we've exited 
        }
        public void ResetKnownStatus()
        {
            softwareStates = new SOFTWARE_STATE[(int)SOFTWARE.COUNT];
            for (int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                softwareStates[i] = SOFTWARE_STATE.UNKNOWN;
            }
            if (componentStatus != null)
            {
                componentStatus.FPS = 0.0;
                componentStatus.FPS_max = 0.0;
                componentStatus.FPS_min = 100.0;
                componentStatus.FrameNum = 0;
                componentStatus.NewDataReceived = false;
                componentStatus.Temperature = 0.0;
                componentStatus.VideoRecordingStarted = false;
                componentStatus.VideoTransferFinished = false;
                componentStatus.VideoTransferStarted = false;
            }
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
                var existingFunctions = statusResponseTable[eventType];
                var newFunctions = new StatusResponseDelegate[existingFunctions.Length + 1];
                Array.Copy(existingFunctions, newFunctions, existingFunctions.Length);
                newFunctions[existingFunctions.Length] = responseFunction;
                statusResponseTable[eventType] = newFunctions;
            }
            else
            {
                OutputHelper.OutputLog($"{UnitName} is registering a NEW response for {eventType.ToString()}", OutputHelper.Verbosity.Trace);
                statusResponseTable.Add(eventType, [responseFunction]);
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
                    baseStatusBotStoppedFunction(new byte[0]);
                }
                else if (timeSinceLastHB > TimeSpan.FromSeconds(WRN_HB_TIMEOUT_SEC))
                {
                    OutputHelper.OutputLog(String.Format("Havent heard from {0} in {1} sec", this.UnitName, timeSinceLastHB.TotalSeconds));
                    if (componentStatus.Status != Status.TimedOut)
                    {
                        componentStatus.Status = Status.TimedOut;
                        OnPropertyChanged(nameof(ComponentStatus));
                    }
                }
            }
        }

        public void UpdateTimeLastHBReceived()
        {
            LastHBReceived = DateTime.UtcNow;
            if(componentStatus.Status != Status.Running)
            {
                if(componentStatus.Status != Status.Running)
                {
                    componentStatus.Status = Status.Running;
                    OnPropertyChanged(nameof(ComponentStatus));
                }
            }
        }

        // Inside the StatusBot class, replace the method UpdateSoftwareState with the following:

        public bool UpdateSoftwareState(SOFTWARE software, SOFTWARE_STATE newStatus)
        {
            if (softwareStates[(int)software] != newStatus)
            {
                softwareStates[(int)software] = newStatus;
                OutputHelper.OutputLog($"Updating bot {this.UnitName} software state for {software.ToString()} to {newStatus.ToString()}", OutputHelper.Verbosity.Trace);
                if (software == SOFTWARE.WINDOWS_SERVICE)
                {
                    OnPropertyChanged(nameof(WindowsServiceStatus));
                }
                if (software == SOFTWARE.LINUX_DAEMON)
                {
                    OnPropertyChanged(nameof(LinuxDaemonStatus));
                }
                if (software == SOFTWARE.CAPTURE)
                {
                    OnPropertyChanged(nameof(CaptureStatus));
                }
                if (software == SOFTWARE.CALIBRATION)
                {
                    OnPropertyChanged(nameof(CalibrationStatus));
                }
                if (software == SOFTWARE.FUSION)
                {
                    OnPropertyChanged(nameof(FusionStatus));
                }
                if (software == SOFTWARE.CALIBRATION)
                {
                    OnPropertyChanged(nameof(CalibrationSoftwareStatus));
                }
                if (software == SOFTWARE.RENDER)
                {
                    OnPropertyChanged(nameof(RenderStatus));
                }
                return true;
            }
            return false;
        }

        public bool UpdateSoftwareStates(SOFTWARE_STATE[] states)
        {
            bool changed = false;
            for (int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                if (states[i] != SOFTWARE_STATE.UNKNOWN) // don't "forget" the state I got from some other update
                {
                    if(UpdateSoftwareState((SOFTWARE)i, states[i]))
                    {  changed = true; }
                }
            }
            return changed;
        }

        public void PrintStates(OutputHelper.Verbosity verbosity = OutputHelper.Verbosity.Trace)
        {
            OutputHelper.OutputLog($"Print States  Software count: {(int)SOFTWARE.COUNT}", verbosity);
            for (int i = 0; i < (int)SOFTWARE.COUNT; i++)
            {
                OutputHelper.OutputLog($"{((SOFTWARE)i).ToString()}:{softwareStates[i]}", verbosity);
            }
        }

        public void DefaultStatusUpdateSetup(bool setupFPS = true)
        {
            // Clear the table (prevents adding the same calls twice if this is called more than once)
            statusResponseTable.Clear();
            RegisterUpdateFunction(CPC_STATUS.READY, baseStatusBotReadyFunction);
            RegisterUpdateFunction(CPC_STATUS.BUILD_VERSION, baseStatusBotBuildVersionFunction);
            RegisterUpdateFunction(CPC_STATUS.RECEIVED_NEW_DATA, baseStatusBotReceivedNewDataFunction);
            RegisterUpdateFunction(CPC_STATUS.RUNNING, baseStatusBotRunningFunction);
            RegisterUpdateFunction(CPC_STATUS.STOPPED, baseStatusBotStoppedFunction);
            RegisterUpdateFunction(CPC_STATUS.IS_ALIVE, baseStatusBotIsAliveFunction);
            RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, baseStatusBotCalibrationSoftwareCapturingVideoStartedFunction);
            RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO, baseStatusBotCalibrationSoftwareTransferringVideoFunction);
            RegisterUpdateFunction(CPC_STATUS.CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE, baseStatusBotCalibrationSoftwareVideoTransferDoneFunction);
            //FPS related 
            if (setupFPS)
            {
                RegisterUpdateFunction(CPC_STATUS.FPS, baseStatusBotFPSFunction);
            }                 
        }

        private void baseStatusBotCalibrationSoftwareVideoTransferDoneFunction(byte[] update)
        {
            componentStatus.VideoTransferStarted = false;
            componentStatus.VideoTransferFinished = true;
            OnPropertyChanged(nameof(ComponentStatus));
        }

        private void baseStatusBotCalibrationSoftwareTransferringVideoFunction(byte[] update)
        {
            componentStatus.VideoRecordingStarted = false;
            componentStatus.VideoTransferStarted = true;
            componentStatus.VideoTransferFinished = false;
            OnPropertyChanged(nameof(ComponentStatus));
        }

        private void baseStatusBotCalibrationSoftwareCapturingVideoStartedFunction(byte[] update)
        {
            componentStatus.VideoRecordingStarted = true;
            componentStatus.VideoTransferStarted = false;
            componentStatus.VideoTransferFinished = false;
            OnPropertyChanged(nameof(ComponentStatus));
        }

        private void baseStatusBotIsAliveFunction(byte[] update)
        {
            UpdateTimeLastHBReceived();
            if(componentStatus.Status != Status.Running)
            {
                componentStatus.Status = Status.Running;
                OnPropertyChanged(nameof(ComponentStatus));
            }
        }

        private void baseStatusBotReadyFunction(byte[] update)
        {
            UpdateTimeLastHBReceived();
            if (componentStatus.Status != Status.Ready)
            {
                componentStatus.Status = Status.Ready;
                OnPropertyChanged(nameof(ComponentStatus));
            }
        }

        private void baseStatusBotBuildVersionFunction(byte[] update)
        {
            OutputHelper.OutputLog($"{UnitName} default updater calling only SaveVersionData");
            SaveVersionData(update);
            OnPropertyChanged(nameof(VersionString));
        }

        private void baseStatusBotReceivedNewDataFunction(byte[] update)
        {
            OutputHelper.OutputLog($"{UnitName} received valid calibration data.");
            componentStatus.NewDataReceived = true;
            OnPropertyChanged(nameof(ComponentStatus));
        }

        private void baseStatusBotStoppedFunction(byte[] update)
        {
            ResetKnownStatus();
            componentStatus.VideoRecordingStarted = false;
            if(componentStatus.Status != Status.Stopped)
            {
                componentStatus.Status = Status.Stopped;
                OnPropertyChanged(nameof(ComponentStatus));
            }
            heartBeatRunning = false; // no need to detect heartbeat if we know it stopped correctly
        }

        private void baseStatusBotFPSFunction(byte[] update)
        {
            if (HeartRate > MAX_HEART_RATE)
            {
                //OutputHelper.OutputLog($"Bot {UnitName} is running too fast.  Heart rate: {HeartRate}", OutputHelper.Verbosity.Warning);
                // Ignore, running too often
                return;
            }
            UpdateTimeLastHBReceived();
            if (update.Length < sizeof(double) + sizeof(byte))
            {
                int expectedSize = sizeof(double) + sizeof(byte);
                OutputHelper.OutputLog($"FPS packetsize ({expectedSize} expected): " + update.Length);
                return;
            }
            if (update.Length > sizeof(double))
            {
                double fps = BitConverter.ToDouble(update, sizeof(byte));
                componentStatus.FPS = fps;

                if (update.Length > sizeof(double) * 2)
                {
                    double temperature = 0.0;
                    temperature = BitConverter.ToDouble(update, sizeof(byte) + sizeof(double));
                    componentStatus.Temperature = temperature;
                }
                // Determining if the number of acquired frames was also sent as payload.
                // This typically occurs during video-based calibration and allows the user
                // to easily determining when video recording started.
                if (update.Length > sizeof(double) + sizeof(int))
                {
                    int frameNumber = (int)System.BitConverter.ToInt32(update, 1 + sizeof(double));
                    componentStatus.FrameNum = frameNumber;
                    componentStatus.VideoRecordingStarted = true;
                }
            }
            heartBeatRunning = true;
            OnPropertyChanged(nameof(FPS));
            OnPropertyChanged(nameof(MinFPS));
            OnPropertyChanged(nameof(MaxFPS));
            OnPropertyChanged(nameof(Temp));
            // Don't update status here, FPS status is just used for sending data back to the control panel
        }
        // Daemon bots don't actually read FPS, we're piggybacking on the packet
        // to transmit voltage and temperature from the Nanos
        public void daemonStatusBotFPSFunction(byte[] update)
        {
            UpdateTimeLastHBReceived();
            if (update.Length < sizeof(double) + sizeof(byte))
            {
                int expectedSize = sizeof(double) + sizeof(byte);
                OutputHelper.OutputLog($"FPS packetsize ({expectedSize} expected): " + update.Length);
                return;
            }
            if (update.Length > sizeof(double))
            {
                double voltage = BitConverter.ToDouble(update, sizeof(byte));
                //this.Form.OutputLog("Fusion FPS: " + fps.ToString());
                componentStatus.Voltage = voltage;

                if (update.Length > sizeof(double) * 2)
                {
                    double temperature = 0.0;
                    temperature = BitConverter.ToDouble(update, sizeof(byte) + sizeof(double));
                    componentStatus.Temperature = temperature;
                    OnPropertyChanged(nameof(Temp));
                }
                OnPropertyChanged(nameof(MinVoltage));
                OnPropertyChanged(nameof(MaxVoltage));
            }
            heartBeatRunning = true;
        }

        public void baseStatusBotRunningFunction(byte[] update)
        {
            if (componentStatus.Status != Status.Running)
            {
                componentStatus.Status = Status.Running;
                OnPropertyChanged(nameof(ComponentStatus));
            }
            heartBeatRunning = true;
        }

        public void SaveVersionData(byte[] update)
        {
            var tempVersionData = VersionData;
            PeabodyNetworkingLibrary.Utility.ParseVersionDataFromPacket(update, ref tempVersionData);
            VersionData = tempVersionData;

            OutputHelper.OutputLog($"{UnitName} build version {VersionData.Description}", OutputHelper.Verbosity.Info);
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
                    if (this.Running)
                    {
                        CPC_STATUS StatusType = (CPC_STATUS)update[0];
                        OutputHelper.OutputLog($"{UnitName} got a status update.  Type {StatusType.ToString()}: packet size: {update.Length}", OutputHelper.Verbosity.Trace);
                        if (statusResponseTable.ContainsKey(StatusType))
                        {
                            for(int i = 0; i < statusResponseTable[StatusType].Length; i++)
                            {
                                statusResponseTable[StatusType][i](update);
                            }
                        }
                        else
                        {
                            OutputHelper.OutputLog(UnitName + " Got an unexpected status type: " + StatusType.ToString() + " " + update[0]);
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
