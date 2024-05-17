using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Configuration;
using System.Text.RegularExpressions;
using System.IO;

namespace PeabodyNetworkingLibrary
{
    /*** Enums defined here are MOST LIKELY also defined in $\Networking\NetworkingDataTypes.h
     * These two files MUST BE KEPT IN SYNC
     * Please update the documentation as well here https://microsoft.sharepoint.com/teams/3DTelemedicineInternal/Shared%20Documents/Glasgow/3D%20Telemedicine%20Control%20Network%20Packet%20Structures%20Definition.docx?web=1
     * if you modify this file
     * ***/
    public enum CONTROL_PANEL_EVENT
    {
        // Ready/Start/Stop
        DEPTH_GEN_STARTED,                          // Re-broadcast (from launcher software) that the depth camera software has been started
        DEPTH_GEN_STOPPED,                          // Re-broadcast (from depth software) that the camera software is stopping
        FUSION_STARTED,                             // Re-broadcast (from launcher software) that the fusion software has been started
        FUSION_STOPPED,                             // Re-broadcast (from fusion software) that the fusion software is stopping
        RENDER_STARTED,                             // Re-broadcast (from launcher software) that the render software has been started
        RENDER_STOPPED,                             // Re-broadcast (from render software) that the render software is stopping
        BACKGROUND_CAPTURE_STARTED,                 // Re-broadcast that the background capture software has started

        // While-running updates
        SYSTEM_IDLE,                                // Control panel will broadcast this when the system goes back to an Idle state, but it's not continuously broadcast
        DEPTH_GEN_FPS,                              // Re-broadcast of the FPS from a depth camera.  Can be used as indicator that Depth Gen is running
        FUSION_FPS,                                 // Re-broadcast of the FPS from the fusion software.  Can be used as indicator that Fusion is running
        RENDER_FPS,                                 // Re-broadcast of the FPS from the render software.  Can be used as indicator that Render is running
        BROADCAST_MODE_RUNNING,                     // Control panel will broadcast this when the system is completely running (Depth Gen running + Fusion running + Render running)
        BACKGROUND_CAPTURE_COUNT,                   // Re-broadcast from a POD indicating number of BG frames captured (see data packet for pod ID)

        // While-running requests
        SPEED_REQUEST,                              // A component in the system (typically fusion or control panel) is requesting that component change to a different speed

        // Global Start/Stop
        CONTROL_PANEL_START_REQUESTED,              // Signal to all components that a start has been requested.  If data packet is empty, this means full system broadcast start.  Otherwise, check data packed for software to be started (launchers)
        CONTROL_PANEL_STOP_REQUESTED,               // Signal to all components to stop software.
        CONTROL_PANEL_STATE_UPDATE_REQUESTED,       // Signal to all components requesting an update on their current state

        // Calibration
        CALIBRATION_DATA,                           // Trasmission includes a complete data packet of calibration files, up to the receiver to parse out the pertinent portion
        CALIBRATION_CAPTURE_FRAME,                  // Signal to calibration software to capture the current frame (signal is received by master and then master queues subordinates
        CALIBRATION_DOWNLOAD_OLDEST,                // Signal to calibration software to transmit (publish) the oldest captured frame
        KINECT_FACTORY_CALIBRATION_DATA_REQUESTED,  // Signal to AKLauncherDaemon to transmit (publish) the previously saved-to-disk factory calibration files
        CALIBRATION_SOFTWARE_STARTED,               // Re-broadcast (from Launcher software) that calibration software has started
        CALIBRATION_SOFTWARE_START_CAPTURING_VIDEO, // Request to the pod-side calibration software to start capturing video
        CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, // Re-broadcast (from Pod-Side Calibration software) that software is capturing video
        CALIBRATION_SOFTWARE_STOP_CAPTURING_VIDEO,  // Request to the pod-side calibration software to stop capturing video
        CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO,    // Re-broadcast (from Pod-Side Calibration software) that software is transferring video
        CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE,   // Re-broadcast (from Pod-Side Calibraiton software) that video transfer is done
        CALIBRATION_SOFTWARE_PROCESSING,            // Re-broadcast (from Fusion-Side Calibration software) that software is processing video and computing calibration
        CALIBRATION_SOFTWARE_RESULT,                // Re-broadcast (from Fusion-Side Calibration software) that software is done processing.  Data contains result

        SYSTEM_CONFIGURATION_UPDATE,                // Broadcast of new system configuration parameters
        LOG_COLLECTION_REQUESTED,                   // Signal to all components to transmit collected logs
        NEW_BINARY_TRANSFER,                        // Signal containing updated binary, see data packet for the details of which component

        TOGGLE_FUSION_HIGH_RESOLUTION = 100, 
        BUILD_VERSION_REQUESTED                     // Components receiving this packet should immediately respond with a CPC_STATUS::BUILD_VERSION to indicate current running software version
    };

    public enum CPC_STATUS
    {
        // Basic status
        READY,                                      // From any component indicating the software is running and ready to receive data
        RUNNING,                                    // From any component indicating the software is running and is processing data
        STOPPED,                                    // From any component indicating the software has stopped
        FPS,                                        // From any component indicating current frame rate (used as a "running" indicator as well)
        SPEED_REQUEST,                              // From any component indicating it needs an upstream component to change speed
        CALIBRATION_START,                          // [MVC] A calibration frame has been captured.  This is a typical response to the CONTROL_PANEL_EVENT::CALIBRATION_CAPTURE_FRAME
        CALIBRATION_I_SEE_CHECKERBOARD,             // [MVC] The calibration detects a checkerboard in the current frame [NOT IMPLEMENTED]
        CALIBRATION_PROGRESS,                       // [MVC] Subsystem is sending a calibration frame to the control panel in this packet.  This is a typical response to CONTROL_PANEL_EVENT::CALIBRATION_DOWNLOAD_OLDEST
        CALIBRATION_DONE,                           // [MVC+BG Capture] Subsystem has finished calibration. In the 3DTM system, this is sent when background capture has finished.
        CALIBRATION_SOFTWARE_STARTED,               // [PAI] Packet from Launcher software notifying that calibration software has started
        CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, // [PAI] Packet from Pod-Side Calibration software notifying that software is capturing video
        CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO,    // [PAI] Packet from Pod-Side Calibration software notifying that software is transferring video
        CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE,   // [PAI] Packet from Pod-Side Calibraiton software notifying that video transfer is done
        CALIBRATION_SOFTWARE_PROCESSING,            // [PAI] Packet from Fusion-Side Calibration software notifying that software is processing video and computing calibration
        CALIBRATION_SOFTWARE_RESULT,                // [PAI] Packet from Fusion-Side Calibration software notifying that software is done processing.  Data contains result
        IS_ALIVE,                                   // Heartbeat sent from the subsystem to verify network connection.
        FRAMENUMBER,                                // [UNUSED]
        ERROR_MSG,                                  // The subsystem has encountered an error, details are included in the packet
        KINECT_FACTORY_CALIBRATION_DATA,            // This packet contains the factory intrinsics from the Kinect
        RECEIVED_NEW_DATA,                          // Response to a CONTROL_PANEL_EVENT::SYSTEM_CONFIGURATION_UPDATE packet to indicate that the new configuration was received correctly
        LOG_DATA,                                   // Response to a CONTROL_PANEL_EVENT::LOG_COLLECTION_REQUESTED containing log data from the component
        SOFTWARE_INSTALLATION_RESULT,               // Response to a CONTROL_PANEL_EVENT::NEW_BINARY_TRANSFER to indicate the software was installed successfully or containing an error message
        TOGGLE_FUSION_HIGH_RESOLUTION = 100,        // Used by the fusion component to indicate current quality setting
        BUILD_VERSION = 200,                        // Response to a CONTROL_PANEL_EVENT::BUILD_VERSION_REQUESTED including the current software version of the component
    };

    public enum CPC_ERROR
    {
        // Depth gen status
        NONE,
        BUFFERS_OUT_OF_SYNC,
        CAMERA_FAILURE
    };

    public enum SOFTWARE_STATE
    {
        UNKNOWN,            // daemon is not initialized
        NOT_RUNNING,        // software is not running
        STARTED,            // daemon received start command and is in the process of starting the requested application
        RUNNING,            // software process is running
        LOCKED,             // software process exists, but is frozen/non-responsive
        STOPPED,            // daemon received the command to kill the application and is attempting to do so
        TIMEDOUT = 99       // Shouldn't ever receive this, but this represents when we stop hearing back from DAEMON for > timeoutInterval
    };

    public enum SOFTWARE
    {
        SYSTEM_START = 0, // this is used to indicate a request from control panel to transition from "ready" to "running" state for Kinects
        CALIBRATION,
        CAPTURE,
        BACKGROUND_CAPTURE,
        FUSION,
        RENDER,
        LINUX_DAEMON,
        WINDOWS_SERVICE,
        KINECT_COMMUNICATOR,
        COUNT // used to allocate arrays to hold this stuff
    };
    public enum CALIBRATION_SOFTWARE_COMPONENT
    {
        POD = 0,
        BACKEND
    };

    public struct VersionData
    {
        public int Major { get; set; }
        public int Minor { get; set; }
        public int Patch { get; set; }
        public int Commits { get; set; }
        public string Description { get; set; }
        public string Sha1 { get; set; }
        public string BranchName { get; set; }
    }
    public static class Utility
    {
        public static VersionData ReadVersionDataFromDescrptionString(string description)
        {
            VersionData versionData = new VersionData();
            // parse the string to compare to my versionMap
            // format is Major.Minor.Patch-BranchName.Commits e.g. 0.3.1-develop.0
            string[] bits = Regex.Split(description, "([0-9]*).([0-9]*).([0-9]*)-([^.]*).([0-9]*)");
            try
            {
                versionData.Major = Convert.ToInt32(bits[1]);
                versionData.Minor = Convert.ToInt32(bits[2]);
                versionData.Patch = Convert.ToInt32(bits[3]);
                versionData.BranchName = bits[4];
                versionData.Commits = Convert.ToInt32(bits[5]);
                versionData.Description = description;
            }
            catch (FormatException e)
            {
                Console.WriteLine($"ERROR.  Couldn't parse the version data.  Leaving it empty.  Error was: {e.Message}");
            }

            return versionData;
        }
        public static VersionData ReadVersionDataFromConfig(string path)
        {
            VersionData versionData = new VersionData();
            // Read version information from 3dtm_version.config
            System.Configuration.KeyValueConfigurationCollection versionDataCollection;
            System.Configuration.Configuration versionConfig;
            System.Configuration.ExeConfigurationFileMap versionConfigFile = new System.Configuration.ExeConfigurationFileMap();
            versionConfigFile.ExeConfigFilename = path;
            Console.WriteLine($"Loading version information from {versionConfigFile.ExeConfigFilename}");
            if (System.IO.File.Exists(versionConfigFile.ExeConfigFilename))
            {
                versionConfig = System.Configuration.ConfigurationManager.OpenMappedExeConfiguration(versionConfigFile, System.Configuration.ConfigurationUserLevel.None);
                versionDataCollection = versionConfig.AppSettings.Settings;
                versionData.Major = Convert.ToInt32(versionDataCollection["Major"].Value);
                versionData.Minor = Convert.ToInt32(versionDataCollection["Minor"].Value);
                versionData.Patch = Convert.ToInt32(versionDataCollection["Patch"].Value);
                versionData.Commits = Convert.ToInt32(versionDataCollection["Commits"].Value);
                versionData.BranchName = versionDataCollection["BranchName"].Value.ToString();
                versionData.Description = versionDataCollection["Description"].Value.ToString();
                versionData.Sha1 = versionDataCollection["Sha"].Value.ToString();
            }
            else
            {
                Console.WriteLine($"ERROR.  Couldn't find the version data file {versionConfigFile.ExeConfigFilename}");
                versionData.Major = 0;
                versionData.Minor = 0;
                versionData.Patch = 0;
                versionData.Commits = 0;
                versionData.Description = "Unknown Version";
                versionData.BranchName = "Unknown";
                versionData.Sha1 = "";
            }
            return versionData;
        }
        public static string PeabodyPacketToString(byte[] results, Type type, int numberOfPacketsToPrint)
        {
            String consoleString = "";
            for (int i = 0; i < numberOfPacketsToPrint; i++)
            {
                if (i >= results.Length)
                    break;
                if (i == 0)
                {
                    try
                    {
                        consoleString += $"[{Enum.GetName(type, results[i])}]";
                    }
                    catch (ArgumentException)
                    {
                        consoleString += $"[{results[i]}]";
                    }
                }
                else
                    consoleString += $"[{Convert.ToInt32(results[i])}]";
            }
            return consoleString;
        }

        //[CPC_STATUS.VERSION][(int)Major][(int)Minor][(int)Patch][(int)CommitTimestamp][(int)branchNameLength][(string[branchNameLength])branchName][(int)descriptionLength][(string[descriptionLength])description][(string[BUILD_VERSION_SHA1_LENGTH])sha1]
        public static byte[] CreateBuildVersionPacket(int major, int minor, int patch, int commits, string branchName, string description, string sha)            
        {
            int packetSize = sizeof(byte) + 6 * sizeof(int) + branchName.Length + description.Length + sha.Length;
            byte[] data = new byte[packetSize];
            int index = 0;
            data[index++] = (byte)CPC_STATUS.BUILD_VERSION;
            BitConverter.GetBytes(major).CopyTo(data, index);
            index += sizeof(int);
            BitConverter.GetBytes(minor).CopyTo(data, index);
            index += sizeof(int);
            BitConverter.GetBytes(patch).CopyTo(data, index);
            index += sizeof(int);
            BitConverter.GetBytes(commits).CopyTo(data, index);
            index += sizeof(int);
            BitConverter.GetBytes(branchName.Length).CopyTo(data, index);
            index += sizeof(int);
            Encoding.Default.GetBytes(branchName).CopyTo(data, index);
            index += branchName.Length;
            BitConverter.GetBytes(description.Length).CopyTo(data, index);
            index += sizeof(int);
            Encoding.Default.GetBytes(description).CopyTo(data, index);
            index += description.Length;
            Encoding.Default.GetBytes(sha).CopyTo(data, index);

            return data;
        }

        public static void ParseVersionDataFromPacket(byte[] packet, ref VersionData versionData, int verbosity = 0)
        {
            try
            {
                using (MemoryStream ms = new MemoryStream(packet))
                {
                    using (BinaryReader br = new BinaryReader(ms))
                    {
                        br.ReadByte(); // read off the packet type
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"Update is [{packet.Length}] bytes");
                        }
                        versionData.Major = br.ReadInt32(); // correct byte order?
                        versionData.Minor = br.ReadInt32();
                        versionData.Patch = br.ReadInt32();
                        versionData.Commits = br.ReadInt32();
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"Build version {versionData.Major}.{versionData.Minor}.{versionData.Patch}-{versionData.Commits}");
                        }
                        
                        int branchNameLength = br.ReadInt32();
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"Branch name length is {branchNameLength}");
                        }                       
                        byte[] bzBranchName = br.ReadBytes(branchNameLength);
                        if (bzBranchName[0] != 0)
                        {
                            versionData.BranchName = System.Text.Encoding.Default.GetString(bzBranchName);
                        }
                        int descLength = br.ReadInt32();
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"Description length is {descLength}");
                        }                        
                        byte[] bzDescription = br.ReadBytes(descLength);
                        if (bzDescription[0] != 0)
                        {
                            versionData.Description = System.Text.Encoding.Default.GetString(bzDescription);
                        }
                        int sizeRemaining = packet.Length - (sizeof(int) * 5 + descLength);
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"String remaining for SHA is {sizeRemaining}");
                        }                        
                        byte[] bzSha1 = br.ReadBytes(40);
                        if (bzSha1[0] != 0)
                        {
                            versionData.Sha1 = System.Text.Encoding.Default.GetString(bzSha1);
                        }
                        if (verbosity > 0)
                        {
                            Console.WriteLine($"Sha1={versionData.Sha1}, description={versionData.Description}");
                        }                        
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Build version packet came across the network wrong: {ex.Message}.  Packet: {BitConverter.ToString(packet)}");
            }
            return;
        }
    }
}
