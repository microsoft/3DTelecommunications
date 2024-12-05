#include "K4ALauncherDaemon.h"
#include "3dtm_version.h"
#include <experimental/filesystem>
#include "LEDSerialWriter.hpp"
#include <json/json.h>

std::string LockFile("/var/lock/AKLauncherDaemon"); 
std::string LogFile("/var/log/K4AToFusion/AKLauncherDaemon.log"); // default--overwritten by config file
std::string ErrorFile("/var/log/K4AToFusion/AKLauncherDaemon.error");
// Note, can't use relative paths like ~/ because we use this as a start command, AND a regex to find the running process
// and the running proc will have expanded any relatives
const std::string defaultWorkingDirectory("/home/peabody");
const std::string defaultCalibrationDirectory(defaultWorkingDirectory+"/K4AToFusion");
const std::string commandLineRegexDefault[SOFTWARE::COUNT] = 
{
	"", // SYSTEM_START
	"AzureKinectNanoToFusion "+defaultCalibrationDirectory+"/3DTelemedicine.cfg 0 ", // STREAM
	"AzureKinectNanoToFusion "+defaultCalibrationDirectory+"/3DTelemedicine.cfg 1 ", // CALIBRATION - no longer used
	"AzureKinectNanoToFusion "+defaultCalibrationDirectory+"/3DTelemedicine.cfg 2 ", // BACKGROUND_CAPTURE
	"LiveFusionDemo-MultiView.exe", // FUSION 
	"HoloportRenderer.exe", // RENDER
	"", // LINUX_DAEMON, we don't actually want to monitor our own state here
    "", // WINDOWS_SERVICE,
	"" // KINECT_COMMUNICATOR
};
const std::string K4ARecorderCommandLineRegex = "K4ARecorder " + defaultCalibrationDirectory + "/3DTelemedicine.cfg"; // PAI CalibrationSoftware


std::string configfile(defaultCalibrationDirectory+"/3DTelemedicine.cfg");
const std::string resetCommandLine("AzureKinectFirmwareTool -r");

std::string commandLineRegex[SOFTWARE::COUNT];
std::string workingDirectory;
std::string calibrationDirectory;
K4ALauncherDaemon* launcherDaemon;

static int GetIntFromDataPacket(void* eventData, int pos)
{
	char* eventDataChar = (char*)eventData;
	eventDataChar += pos;
	int retInt = ((int*)eventDataChar)[0];
	return retInt;
}
static std::string GetStringFromDataPacket(void* eventData, int pos, int length)
{
	char* eventDataChar = (char*)eventData;
	eventDataChar += pos;
	LOGGER()->debug("Parsing %s into a string from data packet starting at position %d for length %d", eventDataChar, pos, length);
	return std::string(eventDataChar, length);
}
static bool SaveBinaryPacketToFile(void* eventData, const int pos, const int fileSize, const char* path)
{
	LOGGER()->info("SavingBinaryPacket (%d bytes) to %s", fileSize, path);
	std::ofstream ofs;
	ofs.open(path, std::ofstream::binary|std::ofstream::trunc);
	if(!ofs)
	{
		LOGGER()->error("SaveBinaryPacketToFile", "Failed to open file for writing.");
		return false;
	}
	char* eventChar = (char*)eventData;
	eventChar += pos;
	ofs.write(eventChar, fileSize);
	if(!ofs)
	{
		LOGGER()->error("SaveBinaryPacketToFile", "Failed to write to the file.");
	}
	ofs.close();
	if(!ofs)
	{
		LOGGER()->error("SaveBinaryPacketToFile", "Failed to close the file.");
	}
	return ofs.good();
}

bool ParseJsonFromString(const char *rawJson, size_t rawJsonLength, Json::Value &root) {
    JSONCPP_STRING err;
 
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(rawJson, rawJson + rawJsonLength, &root, &err)) 
	{
        LOGGER()->error("ParseJsonFromString", "Error parsing Json");
        return false;
    }

    return true;
}

bool Save3DTMCalibrationJsonFormat(const std::string & calib_dir, const Json::Value & json_converted_calib)
{
    bool success = false;
	std::string json_converted_calib_filename = calib_dir + "/calibCameras3DTM.json";

	LOGGER()->info("Writing converted Json format to file %s", json_converted_calib_filename);

	Json::StreamWriterBuilder wbuilder;
	std::string document = Json::writeString(wbuilder, json_converted_calib);
	std::ofstream ofs(json_converted_calib_filename, std::ios::trunc);

    if(ofs.is_open()){
        ofs << std::fixed;
        ofs.precision(6);
        ofs << document << std::endl;

        success = true;
    }

    return success;
}

K4ALauncherDaemon::K4ALauncherDaemon(std::string controlPanelIP,
	std::string eventTCPPort,
	std::string myInterfaceIP,
	std::string statusTCPPort,
	bool disableHeartbeat) : ControlPanelConnector(controlPanelIP,
		eventTCPPort,
		myInterfaceIP,
		statusTCPPort,
		disableHeartbeat)
{
	GetKinectSerialNumber();
	std::lock_guard<std::mutex> lock(stateMutex);
	runThread = true;
	for(int i = 0; i < SOFTWARE::COUNT; i++)
	{
		SoftwareStates[i] = SOFTWARE_STATE::SS_UNKNOWN;
	}
	stateMonitoringThread = std::thread(StateMonitor, this);
	applicationThread = std::thread(ApplicationThread, this);
	Verbosity = 0;
	calibrationSoftwarePart2Running = false;
}

void K4ALauncherDaemon::GetKinectSerialNumber()
{
	// query the kinect serial number and store it so we can compare received packets
    char*                      serial;
    k4a_device_t               deviceHandle;

	k4a_result_t openResult = k4a_device_open(0, &deviceHandle);
	if (K4A_FAILED(openResult))
	{
		LOGGER()->fatal("K4ALauncherDaemon::GetKinectSerialNumber", "!!!!! Failed to open device\n");
		return;
	}

	size_t serialSize;
	k4a_device_get_serialnum(deviceHandle, NULL, &serialSize);
	serial = new char[serialSize];
	memset(serial, '\0', serialSize);
	k4a_device_get_serialnum(deviceHandle, serial, &serialSize);

	kinectSerialNumber = std::string(serial);

	k4a_device_close(deviceHandle);
}

void K4ALauncherDaemon::ApplicationThread(K4ALauncherDaemon* daemon)
{
	LOGGER()->info("K4ALauncherDaemon ApplicationThread is starting.");
	// We use linux's system() command, which does not return until execution is complete
	// this means we can only operate one of the sub-applications at a time, which is actually perfect
	// because we will NEVER want to run both calibration and capture at the same time -- they cannot because
	// they both need exclusive access to the Kinects.  Therefore, if you send a start event and the software is running,
	// any new run commands will be QUEUED until the other program stops (or you send another stop command)
	while(daemon->runThread)
	{
		int retVal = 0;
		// Check if I've been given a start request
		for(int i = 0; i < SOFTWARE::COUNT; i++)
		{
			if(daemon->GetState(i) == SOFTWARE_STATE::SS_STARTED)
			{
				if(daemon->Verbosity > 0)
				{
					daemon->PrintStates();
				}
				LOGGER()->info("Launching %s", commandLineRegex[i].c_str());
				retVal = system(commandLineRegex[i].c_str());

				LOGGER()->info("Application has exited with code %d", retVal);
				LOGGER()->info("Resetting the AK for the next session: ");
				retVal = system(resetCommandLine.c_str());

				LOGGER()->info("%d", retVal);
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	LOGGER()->info("Application thread terminating.");
}

void K4ALauncherDaemon::SetConfig(CConfig* new_config)
{
	config = new_config;
}

void K4ALauncherDaemon::StateMonitor(K4ALauncherDaemon* daemon)
{
	LOGGER()->info("K4ALauncherDaemon StateMonitor is starting.");
	PROCTAB *ptp;
	int flags = PROC_FILLCOM;
	char cmd[4096];
	regex_t regular_expressions[SOFTWARE::COUNT];
	for(int i = 0; i < SOFTWARE::COUNT; i++)
	{
		if(commandLineRegex[i] == "")
			continue;
		if(regcomp(&regular_expressions[i], commandLineRegex[i].c_str(), REG_EXTENDED|REG_NOSUB|REG_ICASE) != 0)
		{
			LOGGER()->error("Failed to compile regex for %s", commandLineRegex[i].c_str());
			return;
		}
		else
		{
			LOGGER()->debug("Compiled regex for %s", commandLineRegex[i].c_str());
		}
	}
	while(daemon->runThread)
	{
		for(int i = 0; i < SOFTWARE::COUNT; i++)
		{
			daemon->SetPID(i, -1);
		}
		ptp = openproc(flags);

		proc_t task;
		memset(&task, 0, sizeof(task));
		
		while(readproc(ptp, &task)) {
			if(task.cmdline)
			{
				int bytes = sizeof(cmd) - 1;
				int i = 0;
				cmd[bytes] = 0;
				--bytes;

				strncpy(cmd, task.cmdline[i], bytes);
				bytes -= strlen(task.cmdline[i++]);
				while(task.cmdline[i] && bytes > 0)
				{
					strncat (cmd, " ", bytes);
					strncat (cmd, task.cmdline[i], bytes);
					bytes -= strlen(task.cmdline[i++]) + 1;
				}

				LOGGER()->debug("[RAW CMDLINE] %s", task.cmdline[0]);
				for (int j = 1; task.cmdline[j] != nullptr; ++j) {
					LOGGER()->debug("[RAW CMDLINE] %s", task.cmdline[j]);
				}
				LOGGER()->debug("[PROCLIST] %s", cmd);
				for(int i = 0; i < SOFTWARE::COUNT; i++)
				{
					if(commandLineRegex[i] == "")
						continue;
					// match with the commands I care about
					if(regexec (&regular_expressions[i], cmd, 0, NULL, 0) == 0)
					{					
						LOGGER()->debug("Process %d (%s) matched (%s).  Setting PID", task.tid, cmd, commandLineRegex[i].c_str());
						daemon->SetPID(i, task.tid);
						// don't change to RUNNING if I've received a stop command, because the process is still stopping
						if(daemon->GetState(i) != SOFTWARE_STATE::SS_STOPPED && daemon->GetState(i) != SOFTWARE_STATE::SS_RUNNING) 
						{ 
							LOGGER()->debug("Setting application %d PID [%d] to RUNNING", i, task.tid); 
							daemon->SetState(i, SOFTWARE_STATE::SS_RUNNING);
						}
					}
				}
			}
		}

		closeproc(ptp);

		// after looping through the full process list I never set the PID for a process, set it's state to NOT_RUNNING
		for(int i = 0; i < SOFTWARE::COUNT; i++)
		{
			if(daemon->GetPID(i) == -1)
			{				
				// don't change to NOT_STARTED if I've received a start command, because the process is still starting
				if(daemon->GetState(i) != SOFTWARE_STATE::SS_STARTED && daemon->GetState(i) != SOFTWARE_STATE::SS_NOT_RUNNING) 
				{
					LOGGER()->debug("Didn't find application %d setting state to NOT_RUNNING", i);
					daemon->SetState(i, SOFTWARE_STATE::SS_NOT_RUNNING);					
				}
			}
		}
		if(daemon->AllApplicationsStopped() && !daemon->calibrationSoftwarePart2Running)
		{
			// Running session stop request.  Transition the LEDs back to idle
			LOGGER()->debug("All applications stopped.  Transitioning LED back to idle.");
			Trinket::SetState(Trinket::Idle);
		}
		// Always send an update.  Small packet every 0.5s should not take up any noticeable bandwidth
		daemon->SendSoftwareStateUpdate();
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	
	for(int i = 0; i < SOFTWARE::COUNT; i++)
	{
		if(commandLineRegex[i] != "")
		{
			regfree(&regular_expressions[i]);
		}
	}
	LOGGER()->info("State monitor thread terminating.");
}

K4ALauncherDaemon::~K4ALauncherDaemon()
{
	runThread = false;
	if(stateMonitoringThread.joinable())
	{
		stateMonitoringThread.join();
	}
	if(applicationThread.joinable())
	{
		applicationThread.join();
	}
}

bool K4ALauncherDaemon::SetStateIfSerialNumberMatches(void* eventData, int dataSize, std::string stateString)
{
	if(dataSize < 2)
	{
		LOGGER()->error("SetStateIfSerialNumberMatches", "Packet is too small!  It must contain a serial number.  Size: %d", dataSize);
	}
	int serial_length = GetIntFromDataPacket(eventData, 2); //forwarded packets from control panel will have the CPE packet type AND the CPC_STATUS packet, so the size is at position 2!
	LOGGER()->debug("Serial length is %d", serial_length);
	std::string serialNumber = GetStringFromDataPacket(eventData, 2+sizeof(int), serial_length);
	LOGGER()->debug("Packet received for serial number %s", serialNumber.c_str());
	if(strcmp(serialNumber.c_str(), kinectSerialNumber.c_str()) == 0)
	{
		LOGGER()->info("Packet received for this pod. Setting state to %s", stateString.c_str());
		Trinket::SetState(stateString);
		return true;
	}
	else
	{
		LOGGER()->debug("Packet received for serial number [%s]!=[%s]", serialNumber.c_str(), kinectSerialNumber.c_str());
	}
	return false;
}
// global start/stop
// distributor slowdown
// possibly distributor FPS
long K4ALauncherDaemon::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);
	LOGGER()->debug("Event type: %d Data size: %d", (int)eventType, dataSize);
	SendSoftwareStateUpdate();
	auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	if (Verbosity > 1) 
	{
		LOGGER()->trace("Data: ");
		for (int i = 0; i < dataSize; i++) {
			LOGGER()->trace("%x ", (int)((char*)eventData)[i]);
		}
	}

	switch (eventType)
	{
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_START_REQUESTED:
        if(dataSize > 1)
        {	
			LOGGER()->debug("Got a start request.  Packet size is %d program requested is %x", dataSize, ((char*)eventData)[1]);
			if(!AllApplicationsStopped())
			{
				LOGGER()->debug("    ignoring.  Not all applications are stopped");
			}
            // parse which program is being asked to start
            switch( ((char*)eventData)[1])
            {
                case CALIBRATION:
					// We run Calibration if either dataSize is at most 2, in which case we run MultiViewCalib,
					// or it is higher than 2 and the third parameter is CALIBRATION_SOFTWARE_COMPONENT::POD, in which case we run
					// the CalibrationSoftware. 
					if(dataSize <= 2 || (CALIBRATION_SOFTWARE_COMPONENT)(((char*)eventData)[2]) == CALIBRATION_SOFTWARE_COMPONENT::POD) {
						// We only run one application at a time, so both must be NOT running in order to start one
						if(AllApplicationsStopped())
						{
							LOGGER()->info("Starting calibration software");
							SetState(SOFTWARE::CALIBRATION, SOFTWARE_STATE::SS_STARTED);	
							Trinket::SetState(Trinket::Starting_Calibration);
						}
					}
                break;
                case CAPTURE:
					// We only run one application at a time, so both must be NOT running in order to start one
					if(AllApplicationsStopped())
					{
						LOGGER()->info("[%s] Starting capture software", ctime(&timenow));
						SetState(SOFTWARE::CAPTURE, SOFTWARE_STATE::SS_STARTED);
						Trinket::SetState(Trinket::Starting_Broadcast);					
					}
                break;				
                case BACKGROUND_CAPTURE:
					// We only run one application at a time, so both must be NOT running in order to start one
					if(AllApplicationsStopped())
					{
						LOGGER()->info("[%s] Starting background capture software", ctime(&timenow));
						SetState(SOFTWARE::BACKGROUND_CAPTURE, SOFTWARE_STATE::SS_STARTED);
						Trinket::SetState(Trinket::Starting_BG_Capture);
					}
                break;				
				default:
					LOGGER()->debug("Ignoring start request: %d", (int)((char*)eventData)[1]);
            }
        }
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
		if(dataSize > 1)
        {
			*LOGGER() << Logger::Verbosity::Debug << "Got a software stop request.  Packet size is " << dataSize << Logger::endl;
            // parse which program is being asked to stop
			// Note that this is more of a kill event, not a graceful shutdown.
			// You should first have the control panel send a quit event to the program
            switch( ((char*)eventData)[1])
            {
                case SOFTWARE::CALIBRATION:				
					*LOGGER() << Logger::Verbosity::Info << "[" <<  ctime(&timenow) << "] Daemon is killing calibration software" << Logger::endl;					
					KillSoftware(SOFTWARE::CALIBRATION, SIGTERM);
                break;
                case SOFTWARE::CAPTURE:
					*LOGGER() << Logger::Verbosity::Info << "[" <<  ctime(&timenow) << "] Daemon is killing capture software" << Logger::endl;
					KillSoftware(SOFTWARE::CAPTURE, SIGTERM);
					KillSoftware(SOFTWARE::CAPTURE, SIGINT);
                break;
				case SOFTWARE::BACKGROUND_CAPTURE:
					*LOGGER() << Logger::Verbosity::Info << "[" <<  ctime(&timenow) << "] Daemon is killing background capture software" << Logger::endl;
					KillSoftware(SOFTWARE::BACKGROUND_CAPTURE, SIGTERM);
				break;
				case SOFTWARE::FUSION:
				case SOFTWARE::RENDER:
					// ignore
				break;
				default:
					*LOGGER() << Logger::Verbosity::Debug << "[" <<  ctime(&timenow) << "] Got a stop request with an unsupported data packet: " << (int)((char*)eventData)[1] << Logger::endl;
				break;
            }
        }
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_STATE_UPDATE_REQUESTED:
		SendSoftwareStateUpdate();
	break;
	case CONTROL_PANEL_EVENT::KINECT_FACTORY_CALIBRATION_DATA_REQUESTED:
		{
		const int numCalibFiles = 4;
		char* dataPacket;
		int packetSize = 0;
		// load the factory calibs and send them to the control panel
		std::string calibFiles[numCalibFiles] = { config->GetValueWithDefault("AzureKinectLauncherDaemon", "ColorFactoryCalibFilename", calibrationDirectory+"/colorConfig00.json"),
								config->GetValueWithDefault("AzureKinectLauncherDaemon", "DepthFactoryCalibFilename", calibrationDirectory+"/depthConfig00.json"),
								config->GetValueWithDefault("AzureKinectLauncherDaemon", "ExtrinsicsFactoryCalibFilename", calibrationDirectory+"/extrinsics00.json"),
								config->GetValueWithDefault("AzureKinectLauncherDaemon", "IMUSampleFilename", calibrationDirectory+"/imusample.json")
		};
		for(int i = 0; i < numCalibFiles; i++)
		{
			std::ifstream infile(calibFiles[i].c_str());
			if(!infile.is_open())
			{
				*LOGGER() << Logger::Verbosity::Error << "ReceiveEvent:Kinect Factory Calibration Data Request" << "Error.  Couldn't open " << calibFiles[i] << Logger::endl;
				calibFiles[i] = ""; // in case reading the file fails, we'll still set this empty so we don't add it to the packet
				break;
			}
			auto ss = std::ostringstream{};
			ss << infile.rdbuf();
			calibFiles[i] = ss.str();
			*LOGGER() << Logger::Verbosity::Debug << "Read " << calibFiles[i].length() << " bytes from " << calibFiles[i] << Logger::endl;
			packetSize += sizeof(int);
			packetSize += calibFiles[i].length();
		}

		dataPacket = new char[packetSize];
		char* dstPointer = dataPacket;
		for(int i = 0; i < numCalibFiles; i++)
		{		
			int fileLength = calibFiles[i].length();
			if(fileLength > 0)
			{
				memcpy(dstPointer, (char*)&fileLength, sizeof(int));				
				dstPointer += sizeof(int);
				memcpy(dstPointer, calibFiles[i].c_str(), fileLength);
				dstPointer += fileLength;
			}
		}
		*LOGGER() << Logger::Verbosity::Debug << "Sending factory calib data packet [" << calibFiles[0].length() << "][colordata][" << calibFiles[1].length() << "][depthdata][" << calibFiles[2].length() << "][extrinsicsdata][" << calibFiles[3].length() << "][imudata] total size: " << packetSize << Logger::endl;
		SendStatusUpdate(CPC_STATUS::KINECT_FACTORY_CALIBRATION_DATA, dataPacket, packetSize);		
		delete[] dataPacket;
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_STARTED:
		{
			if(SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Starting_Calibration))
			{
				calibrationSoftwarePart2Running = false;
			}
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED:
		{
			if(SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Calibration_Capturing_Video))
			{
				calibrationSoftwarePart2Running = false;
			}
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO:
		{
			if(SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Calibration_Data_Transferring))
			{
				calibrationSoftwarePart2Running = false;
			}
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE:
		{
			if(SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Calibration_Data_Transfer_Complete))
			{
				calibrationSoftwarePart2Running = false;
			}
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_PROCESSING:
		{
			*LOGGER() << Logger::Verbosity::Debug << "Got a calibration software processing state update from control panel" << Logger::endl;
			// We don't check for serial numbers because this should come from the fusion-side of calibration
			Trinket::SetState(Trinket::Calibration_Running_Calib);
			calibrationSoftwarePart2Running = true; // don't set the LED back to idle, system is still running, just not on the pod
		}
	break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_RESULT:
		{
			LOGGER()->info("Received calibration software result");

			if(dataSize < 2) {
				LOGGER()->error("EventReceived", "CalibrationSoftware failed and received bad calibration data! Keeping the old files");
				Trinket::SetState(Trinket::Calibration_Failed);
				break;
			}
			const char *output_dir = config->GetValueWithDefault("AzureKinectLauncherDaemon", "CalibrationDirectory", "/home/peabody/K4AToFusion");
			int json_length = GetIntFromDataPacket(eventData, 1);
			std::string json_calib_str = GetStringFromDataPacket(eventData, 1+sizeof(int), json_length);
			Json::Value json;
			LOGGER()->info("Parsing calibration Json from string (length %d) before writing to file as a sanity check measure...", json_length);
			// Note: we could save the file 
			if(!ParseJsonFromString(json_calib_str.c_str(), json_length, json)) {
				LOGGER()->error("EventReceived::ParseJsonFromString", "Unable to parse Json format from string!");
				Trinket::SetState(Trinket::Calibration_Failed);
				break;
			}
			if(!Save3DTMCalibrationJsonFormat(output_dir, json)) {
				LOGGER()->error("EventReceived::Save3DTMCalibrationJsonFormat", "Unable to save Json format to %s from string!", output_dir);
				Trinket::SetState(Trinket::Calibration_Failed);
				break;
			}
			LOGGER()->debug("Calibration successful.  File successfully saved to %s.", output_dir);
			Trinket::SetState(Trinket::Calibration_Successful);
			calibrationSoftwarePart2Running = true; // don't set the LED back to idle, system is still running, just not on the pod
		}	
	break;
	case CONTROL_PANEL_EVENT::BACKGROUND_CAPTURE_STARTED:
		{
			if(SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Capturing_BG_Images))
			{
				calibrationSoftwarePart2Running = false;
			}
		}
	break;
	case CONTROL_PANEL_EVENT::SYSTEM_CONFIGURATION_UPDATE:
		// Save the config file to disk.  Should we call a reboot automatically so this process gets reloaded?
		{
			if(dataSize < 2)
			{
				*LOGGER() << Logger::Verbosity::Debug << "Got an empty system configuration update.  Skipping." << Logger::endl;
				break;
			}
			int index = 0;
			index++; // skip the event type
			int fileSize = GetIntFromDataPacket(eventData, index);
			index += sizeof(int);
			if(fileSize > dataSize)
			{
				*LOGGER() << Logger::Verbosity::Debug << "Error.  File size is reported as " << fileSize << " but dataSize is reported as " << dataSize << " you're probably parsing the wrong bit for fileSize" << Logger::endl;
				break;
			}
			std::string configurationFile = GetStringFromDataPacket(eventData, index, fileSize);
			std::ofstream ofs(configfile, std::ofstream::out | std::ofstream::trunc);
			ofs << configurationFile;
			ofs.close();
			*LOGGER() << Logger::Verbosity::Info << "New config file (" << fileSize << " bytes) has been written.  Please consider rebooting to reload this service and re-read the config file." << Logger::endl;
		}
	break;
	case CONTROL_PANEL_EVENT::LOG_COLLECTION_REQUESTED:
	{
		*LOGGER() << Logger::Verbosity::Info << "Got a log collection request." << Logger::endl;
		// Transmit the latest AzureKinectNanoToFusion logfile
		std::string logfilename = config->GetValueWithDefault("PodGlobalConfig", "LogFilename", "/var/log/K4AToFusion/K4AToFusion.log");
		SendLogFile(logfilename);
		// Also send the previous (.1)
		SendLogFile(logfilename+".1");

		// Transmit the latest K4ALauncherDaemon logfile
		SendLogFile(LogFile);
		// Also send the previous (.1)
		SendLogFile(LogFile+".1");
	}
	break;
	case CONTROL_PANEL_EVENT::NEW_BINARY_TRANSFER:
	{
		if(dataSize < 5)
		{
			*LOGGER() << Logger::Verbosity::Error << "Error.  Data packet is too small to hold software type and file size, bad binary packet." << Logger::endl;
			break;
		}
		int fileSize = GetIntFromDataPacket(eventData, 2);
		if(fileSize > dataSize)
		{
			*LOGGER() << Logger::Verbosity::Error << "Error.  File size is reported as " << fileSize << " but dataSize is reported as " << dataSize << " you're probably parsing the wrong bit for fileSize" << Logger::endl;
			break;
		}		
		switch( ((char*)eventData)[1])
		{
			case CALIBRATION:
			case CAPTURE:
			case BACKGROUND_CAPTURE:
			{
				*LOGGER() << Logger::Verbosity::Info << "Got a binary transfer for new AzureKinectNanoToFusion" << Logger::endl;
				// Make sure the application is stopped
				KillSoftware(SOFTWARE::CALIBRATION, SIGTERM);
				KillSoftware(SOFTWARE::BACKGROUND_CAPTURE, SIGTERM);
				KillSoftware(SOFTWARE::CAPTURE, SIGTERM);
				// Copy the new version		
				std::string filename(config->GetValueWithDefault("PodGlobalConfig", "BinaryLocation", "/usr/local/bin/"));
				filename += "AzureKinectNanoToFusion";
				if(SaveBinaryPacketToFile(eventData, sizeof(char)+sizeof(char)+sizeof(int), fileSize, filename.c_str()))
				{
					SendInstallationResult((SOFTWARE)(((char*)eventData)[1]), true, "AzureKinectNanoToFusion installed successfully.");
				}
				else
				{
					SendInstallationResult((SOFTWARE)(((char*)eventData)[1]), true, "Couldn't save the packet to disk");
				}
				break;
			}
			case LINUX_DAEMON:
			{
				*LOGGER() << Logger::Verbosity::Info << "Got a binary transfer for new AKLauncherDaemon" << Logger::endl;
				// Save to the staging folder
				std::string filename(config->GetValueWithDefault("AKLauncherDaemon", "StagingFolder", "/home/peabody/staging/"));
				filename += "AKLauncherDaemon";
				if(SaveBinaryPacketToFile(eventData, sizeof(char)+sizeof(char)+sizeof(int), fileSize, filename.c_str()))
				{
					SendInstallationResult((SOFTWARE)((char*)eventData)[1], true, "AKLauncherDaemon installed to staging.  Pod is now rebooting.");
				}
				else
				{
					SendInstallationResult((SOFTWARE)((char*)eventData)[1], true, "Couldn't save the packet to disk");
				}				
				// Request a reboot			
				sync();
				execl("/bin/shutdown", "shutdown", "-r", "now", (char *)0);
			break;
			}
			default:
			{
				// Ignore all other software
				break;
			}
		}
	}
	case CONTROL_PANEL_EVENT::SYSTEM_IDLE:
	{
		*LOGGER() << Logger::Verbosity::Debug << "Got a system idle state update from control panel" << Logger::endl;
		Trinket::SetState(Trinket::Idle);
		break;
	}
	case CONTROL_PANEL_EVENT::DEPTH_GEN_STARTED:
		{
			SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Starting_Broadcast);
		}
		break;	
	case CONTROL_PANEL_EVENT::BROADCAST_MODE_RUNNING:
	{
		*LOGGER() << Logger::Verbosity::Debug << "Got a broadcast mode running state update from control panel" << Logger::endl;
		Trinket::SetState(Trinket::Broadcast_Running);
		break;
	}
	case CONTROL_PANEL_EVENT::DEPTH_GEN_STOPPED:
		{
			SetStateIfSerialNumberMatches(eventData, dataSize, Trinket::Broadcast_Stopping);
		}
		break;	
	default:
		auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		*LOGGER() << Logger::Verbosity::Debug << "[" << ctime(&timenow) << "] Saw event " << static_cast<int>(eventType) << ", ignoring." << Logger::endl;
		break;
	}

	return S_OK;
}

void K4ALauncherDaemon::SendInstallationResult(SOFTWARE software, bool success, std::string errorMessage)
{
	*LOGGER() << Logger::Verbosity::Info << "Sending installation of software " << (int)software << " result: " << success << " message: " << errorMessage << Logger::endl;
	int messagelength = errorMessage.length();
	int index = 0;
	int packetSize = sizeof(char) + sizeof(char) + sizeof(int) + messagelength;
	char* data = new char[packetSize];
	data[index] = (char)software; index++;
	data[index] = (char)success; index++;
	memcpy(data+index, (char*)&messagelength, sizeof(int)); index += sizeof(int);
	memcpy(data+index, errorMessage.c_str(), messagelength); index += messagelength;
	SendStatusUpdate(CPC_STATUS::SOFTWARE_INSTALLATION_RESULT, data, packetSize, Logger::Verbosity::Trace);
	delete[] data;
}

void K4ALauncherDaemon::SendLogFile(std::string filenameFullPath)
{
	*LOGGER() << Logger::Verbosity::Debug << "Sending Log file " << filenameFullPath << Logger::endl;
	std::ifstream filestream(filenameFullPath, std::ifstream::in);
	if(filestream)
	{
		int index = 0;
		int start = filenameFullPath.rfind("/");
		if(start < 0)
			start = 0;
		std::string filename = filenameFullPath.substr(start, std::string::npos);

		int nameLength = filename.length();
		int serialLength = kinectSerialNumber.length();

		filestream.seekg(0, filestream.end);
		int length = filestream.tellg();
		filestream.seekg(0, filestream.beg);
		
		int dataLength = sizeof(int) + nameLength + sizeof(int) + length + sizeof(int) + serialLength;
		char* data = new char[dataLength];
		memcpy(data+index, (char*)&nameLength, sizeof(int));	index += sizeof(int);
		memcpy(data+index, filename.c_str(), nameLength);		index += nameLength;
		memcpy(data+index, (char*)&length, sizeof(int));		index += sizeof(int);
		filestream.read(data+index, length);					index += length;
		memcpy(data+index, (char*)&serialLength, sizeof(int));	index += sizeof(int);
		memcpy(data+index, (char*)kinectSerialNumber.c_str(), serialLength); index += serialLength;

		SendStatusUpdate(CPC_STATUS::LOG_DATA, data, dataLength);
		delete[] data;
	}
	else
	{
		std::cerr << "Error opening logfile " << filenameFullPath << " to send to control panel" << std::endl;
	}
}

bool K4ALauncherDaemon::AllApplicationsStopped()
{
	for(SOFTWARE_STATE state : SoftwareStates)
	{
		if(state == SOFTWARE_STATE::SS_RUNNING || state == SOFTWARE_STATE::SS_STARTED || state == SOFTWARE_STATE::SS_LOCKED)
		{
			return false;
		}
	}
	return true;
}

void K4ALauncherDaemon::SendSoftwareStateUpdate()
{
	SOFTWARE_STATE states[SOFTWARE::COUNT];
	GetAllStates(states);
	
	*LOGGER() << Logger::Verbosity::Trace << "Sending state update.  Sizeof(states) = " << sizeof(states) << Logger::endl;
	int index = 0;
	int serialLength = kinectSerialNumber.length();
	int dataLength = sizeof(int)+serialLength+sizeof(states);
	char* data = new char[dataLength];
	*LOGGER() << Logger::Verbosity::Trace << "[SendSoftwareStateUpdate] Serial length is " << serialLength << " Serial Number: " << kinectSerialNumber.c_str() << Logger::endl;
	memcpy(data+index, (char*)&serialLength, sizeof(int));					
	index += sizeof(int);
	memcpy(data+index, (char*)kinectSerialNumber.c_str(), serialLength);	index += serialLength;
	memcpy(data+index, (char*)&states, sizeof(states));						index += sizeof(states);
	SendStatusUpdate(CPC_STATUS::RUNNING, data, dataLength, Logger::Verbosity::Trace);
	delete data;
}

void K4ALauncherDaemon::PrintStates()
{
	*LOGGER() << Logger::Verbosity::Info << "======== SOFTWARE STATUS ========" << Logger::endl;
	for(int i = 0; i < SOFTWARE::COUNT; i++)
	{		
		*LOGGER() << Logger::Verbosity::Info << "==== " << commandLineRegex[i] << " ====" << Logger::endl;
		PrintState(GetState(i));
	}
	*LOGGER() << Logger::Verbosity::Info << "=================================" << Logger::endl;
}

void K4ALauncherDaemon::PrintState(SOFTWARE_STATE state)
{
	*LOGGER() << Logger::Verbosity::Info << (state == SOFTWARE_STATE::SS_UNKNOWN?"UNKNOWN":
		state == SOFTWARE_STATE::SS_NOT_RUNNING?"NOT RUNNING":
		state == SOFTWARE_STATE::SS_STARTED?"STARTED":
		state == SOFTWARE_STATE::SS_RUNNING?"RUNNING":
		state == SOFTWARE_STATE::SS_LOCKED?"LOCKED":
		state == SOFTWARE_STATE::SS_STOPPED?"STOPPED":"BAD STATE"
		) << Logger::endl;
}

void K4ALauncherDaemon::GetAllStates(SOFTWARE_STATE (&states)[SOFTWARE::COUNT])
{
	std::lock_guard<std::mutex> lck(stateMutex);
	for(int i = 0; i < SOFTWARE::COUNT; i++)
	{
		states[i] = SoftwareStates[i];
	}
}

SOFTWARE_STATE K4ALauncherDaemon::GetState(int idx)
{
	std::lock_guard<std::mutex> lck(stateMutex);
	SOFTWARE_STATE stateCopy = SoftwareStates[idx];
	return stateCopy;
}
void K4ALauncherDaemon::SetState(int idx, SOFTWARE_STATE state)
{
	std::lock_guard<std::mutex> lck(stateMutex);
	SoftwareStates[idx] = state;
}

void K4ALauncherDaemon::SetPID(int idx, int pid)
{
	std::lock_guard<std::mutex> lck(stateMutex);
	PIDs[idx] = pid;
}

int K4ALauncherDaemon::GetPID(int idx)
{
	std::lock_guard<std::mutex> lck(stateMutex);
	return PIDs[idx];
}

void K4ALauncherDaemon::KillSoftware(int idx, int signal)
{
	int pid = GetPID(idx);
	if(pid != -1)
	{
		*LOGGER() << Logger::Verbosity::Info << "Sending " << signal << " to proc " << pid << Logger::endl;
		kill(GetPID(idx), signal);
		std::string command = "pkill -" + std::to_string(signal) + " -P " + std::to_string(pid);
		int retval = system(command.c_str());
		*LOGGER() << Logger::Verbosity::Trace << "pkill returned " << retval << Logger::endl;
	}
	else
	{
		*LOGGER() << Logger::Verbosity::Error << "Can't kill software " << idx << ", process doesn't exist." << Logger::endl;
	}
}

void K4ALauncherDaemon::Cleanup(int signal)
{
	if(signal > 0)
		*LOGGER() << Logger::Verbosity::Info << "Got a signal " << signal << Logger::endl;
	// pass down the signal if something is running
	bool processesRunning = true;
	int tries = 0;
	while(processesRunning)
	{
		processesRunning = false;
		for(int i = 0; i < SOFTWARE::COUNT; i++)
		{
			if((GetState(i) == SS_RUNNING || GetState(i) == SS_STARTED) && GetPID(i) != -1)
			{
				*LOGGER() << Logger::Verbosity::Info << "Passing down signal " << signal << " to SOFTWARE::" << i << Logger::endl;
				if(tries > numRetriesBeforeSigTerm)
				{
					*LOGGER() << Logger::Verbosity::Debug << "Sending a SIGTERM because we've exceeded " << numRetriesBeforeSigTerm << " tries with signal " << signal << Logger::endl;
					KillSoftware(i, SIGTERM);
				}
				else
				{
					KillSoftware(i, signal);
				}
				processesRunning = true;
			}
		}
		if(processesRunning)
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		tries++;
	}
}
void usage()
{
	std::cout << "Usage: K4ALauncherDaemon config.cfg" << std::endl;// ControlPanelIP eventTCPPort statusTCPPort [daemon] [captureCommand] [calibrateCommand] [bgCommand]" << std::endl;
}

/* Check if the lockfile exists.  If it does, Daemon must already be running
 * if it does not exist, create it and put my PID in there
 */
bool CreateLockFile()
{
	struct stat buffer;
	if (stat (LockFile.c_str(), &buffer) == 0)
	{
		return false;
	}
	else
	{
		*LOGGER() << Logger::Verbosity::Info << "Trying to open " << LockFile.c_str() << Logger::endl;
		FILE* lock = fopen(LockFile.c_str(), "w+");
		if(lock != NULL)
		{
			fprintf(lock, "%ld", (long)getpid());
			fclose(lock);
			return true;
		}
		else
		{
			*LOGGER() << Logger::Verbosity::Fatal << "ERROR.  Could not open the lock file for writing." << Logger::endl;
			return false;
		}
	}
}
void Cleanup(int signal)
{
	if(signal != 0)
		*LOGGER() << Logger::Verbosity::Info << "Received signal " << signal << Logger::endl;
	*LOGGER() << Logger::Verbosity::Info << "Exiting..." << Logger::endl;
	launcherDaemon->Cleanup(signal);
	unlink(LockFile.c_str());
	LOGGER()->close();
	exit(signal);
}
int main(int argc, char* argv[])
{
	bool daemonMode = false;
	if(argc > 1)
	{
		if(!std::experimental::filesystem::exists(argv[1]))
		{
			std::cerr << "Could not open config file: " << argv[1] << " exiting." << std::endl;
			usage();
			return -1;
		}
		configfile = std::string(argv[1]);
	}
	
	CConfig * config = new CConfig(configfile);
	LogFile = config->GetValueWithDefault("AzureKinectLauncherDaemon", "LogFilename", LogFile);
	LockFile = config->GetValueWithDefault("AzureKinectLauncherDaemon", "LockFilename", LockFile);
	ErrorFile = config->GetValueWithDefault("AzureKinectLauncherDaemon", "ErrorFilename", ErrorFile);
	daemonMode = config->GetValueWithDefault("AzureKinectLauncherDaemon", "DaemonMode", false);
	configfile = config->GetValueWithDefault("Configuration", "ConfigurationFileLinux", configfile);

	if(daemonMode)
	{
		LOGGER()->set_filename(LogFile);
		LOGGER()->set_error_filename(ErrorFile);
		LOGGER()->info("AKLauncherDaemon stdout/stderr redirected to %s and %s\n", LogFile.c_str(), ErrorFile.c_str());
	}
	LOGGER()->warning("Main", "AKLauncherDaemon Output Log start");
	LOGGER()->error("AKLauncherDaemon", "Error Log start");
	LOGGER()->warning("Main", "Loaded configuration file %s\n", configfile.c_str());
    char hostname[1024];
    gethostname(hostname, 1024);
	uint thisPodNumber = atoi(&hostname[4]);
	LOGGER()->warning("Main", "Got a hostname of \"%s\", setting this pod number to %d", hostname, thisPodNumber);
	std::string overrideSection = "Pod"+std::to_string(thisPodNumber)+"Override";

	// Initializing Control Panel Connector
	std::string controlPanelIP = config->GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250");
	std::string eventTCPPort = config->GetValueWithDefault("Ports", "KNCSPublishEventPort", "14502");
	std::string myIP("*");
	std::string statusTCPPort = config->GetValueWithDefault("Ports", "NanoDaemonStatusPort", "14501");

	launcherDaemon = new K4ALauncherDaemon(controlPanelIP, eventTCPPort, myIP, statusTCPPort, config->GetValueWithDefault(overrideSection, "AzureKinectLauncherDaemon", "DisableHeartbeat", false));
	launcherDaemon->SetConfig(config);
	launcherDaemon->SetVersionInformation(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMITS, VERSION_BRANCH_NAME, VERSION_DESCRIPTION, VERSION_SHA1);

	launcherDaemon->Verbosity = config->GetValueWithDefault(overrideSection, "AzureKinectLauncherDaemon", "Verbosity", launcherDaemon->Verbosity);
	LOGGER()->set_verbosity((Logger::Verbosity)config->GetValueWithDefault("AzureKinectLauncherDaemon", "Verbosity", launcherDaemon->Verbosity));
	launcherDaemon->serialPortFilename = config->GetValueWithDefault(overrideSection, "PodGlobalConfig", "DisplaySerialPort", "/dev/ttyACM1");
	commandLineRegex[SOFTWARE::CAPTURE].assign(config->GetValueWithDefault(overrideSection, "AzureKinectLauncherDaemon", "CaptureRegex", commandLineRegexDefault[SOFTWARE::CAPTURE]));
	commandLineRegex[SOFTWARE::CALIBRATION].assign(config->GetValueWithDefault(overrideSection, "AzureKinectLauncherDaemon", "K4ARecorderRegex", K4ARecorderCommandLineRegex));

	LOGGER()->info("Using command line for calibration as %s", commandLineRegex[SOFTWARE::CALIBRATION].c_str());

	commandLineRegex[SOFTWARE::BACKGROUND_CAPTURE].assign(config->GetValueWithDefault(overrideSection, "AzureKinectLauncherDaemon", "BackgroundCaptureRegex", commandLineRegexDefault[SOFTWARE::BACKGROUND_CAPTURE]));
	commandLineRegex[SOFTWARE::FUSION] = commandLineRegexDefault[SOFTWARE::FUSION];
	commandLineRegex[SOFTWARE::RENDER] = commandLineRegexDefault[SOFTWARE::RENDER];

	LOGGER()->info("Setting capture command line regex to %s", commandLineRegex[SOFTWARE::CAPTURE].c_str());
	LOGGER()->info("Setting calibration command line regex to %s", commandLineRegex[SOFTWARE::CALIBRATION].c_str());
	LOGGER()->info("Setting background command line regex to %s", commandLineRegex[SOFTWARE::BACKGROUND_CAPTURE].c_str());

	workingDirectory = config->GetValueWithDefault("AzureKinectLauncherDaemon", "WorkingDirectory", defaultWorkingDirectory);
	calibrationDirectory = config->GetValueWithDefault("AzureKinectLauncherDaemon", "CalibrationDirectory", defaultCalibrationDirectory);
	// Check for existing lock file
	if(!CreateLockFile())
	{
		LOGGER()->fatal("Main", "Only one running daemon allowed at a time.  Lockfile exists.");
		return 0;
	}
	signal(SIGINT, Cleanup);
	signal(SIGTERM, Cleanup);

	// Change to correct working directory
	LOGGER()->info("Changing working path to %s", workingDirectory.c_str());
	std::experimental::filesystem::current_path(workingDirectory);
	if(!std::experimental::filesystem::exists(workingDirectory))
	{
		LOGGER()->warning("Main", "Working directory %s does not exist.  I will create, but no existing calibrations will be loaded until they are copied into this folder.", workingDirectory.c_str());
		std::experimental::filesystem::create_directory(workingDirectory);
	}

	bool g_IsRunning = true;

	launcherDaemon->SendStatusUpdate(CPC_STATUS::READY, NULL, 0);
	LOGGER()->warning("Main", "Daemon running.  BuildVer = %d.%d.%d-%s.%d. Build descrip = %s. Press q to stop if in interactive mode.  Otherwise use sudo service aklauncherdaemon stop.\r\n",
		VERSION_MAJOR,
		VERSION_MINOR,
		VERSION_PATCH,
		VERSION_BRANCH_NAME,
		VERSION_COMMITS,
		VERSION_DESCRIPTION);

    while (g_IsRunning)
    {       
		if(!daemonMode)
		{ 
        char c = getchar();
            switch (c)
            {
            case 'q':
                g_IsRunning = false;
                break;
			case 's':
				launcherDaemon->SendSoftwareStateUpdate();
				launcherDaemon->PrintStates();
				break;
            }
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

	Cleanup(0);
}
