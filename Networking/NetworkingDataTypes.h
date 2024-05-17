#pragma once

#define ZMQ_RETRIES 2   // number of times ZMQ will re-try sending if it doesn't get an ACK from the distributor
#define ZMQ_REQUEST_TIMEOUT     2500 

namespace ControlPanelConnector
{
	enum class CPC_STATUS
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
        CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED,// [PAI] Packet from Pod-Side Calibration software notifying that software is capturing video
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
        BUILD_VERSION = 200                         // Response to a CONTROL_PANEL_EVENT::BUILD_VERSION_REQUESTED including the current software version of the component
	};

	enum CPC_ERROR
	{
		NONE,
		BUFFERS_OUTOFSYNC,
		CAMERA_FAILURE
	};

	enum SOFTWARE_STATE
	{
		SS_UNKNOWN,	// Not yet initialized
		SS_NOT_RUNNING,// No known instance is running on the machine
		SS_STARTED,	// We got a start request and send the command, waiting to see process 
		SS_RUNNING,	// Process exists and is running
		SS_LOCKED,		// Process exists but is non-responsive
		SS_STOPPED		// We got a stop request and sent the command, waiting for process to disappear
	};	

    enum SOFTWARE
    {
        SYSTEM_START = 0,
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

    enum CALIBRATION_SOFTWARE_COMPONENT
    {
        POD,
        BACKEND
    };

	enum class CONTROL_PANEL_EVENT
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

	const int BUILD_VERSION_SHA1_LENGTH = 40;
	
}

