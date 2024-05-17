#include <k4a/k4a.h>
#include "recorder.h"
#include "K4ARecorder.h"
#include "3dtm_version.h"
#include <experimental/filesystem>
#include <math.h>
#include "Logger.h"

#if defined(_WIN32)
#include <windows.h>
#endif

#include <atomic>
#include <ctime>
#include <csignal>

int device_index = 0;
k4a_image_format_t recording_color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
k4a_color_resolution_t recording_color_resolution = K4A_COLOR_RESOLUTION_1080P;
k4a_depth_mode_t recording_depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
k4a_fps_t recording_rate = K4A_FRAMES_PER_SECOND_5;
k4a_wired_sync_mode_t wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
int32_t depth_delay_off_color_usec = 0;
uint32_t subordinate_delay_off_master_usec = 0;
int recording_length = -1;
bool recording_imu_enabled = true;
int absoluteExposureValue = defaultExposureAuto;
int whiteBalance = defaultWhiteBalance;
int brightness = defaultBrightness;
int contrast = defaultContrast;
int saturation = defaultSaturation;
int sharpness = defaultSharpness;
int gain = defaultGainAuto;
char* recording_filename;
uint thisPodNumber;
char *serial_number_message = NULL;
size_t serial_number_length = 0;

std::string controlPanelIP, eventTCPPort, myIP, statusTCPPort;
bool disableHeartbeat;

const char* reset_cmd = "AzureKinectFirmwareTool -r";

bool g_delayStart = true;

std::string MKVDestination;
std::string transferDestination;

struct K4ARecorderState {
	K4ARecorder *recorder;
	std::chrono::high_resolution_clock::time_point start;
	int frame_counter;
};

void SendFPS(void *state, bool send_frame_counter) {
	K4ARecorderState *state_ref = (K4ARecorderState*)state;

	if(state_ref->frame_counter % 5 == 0) {
		double frameRate;
		size_t eventDataSize = sizeof(frameRate) + sizeof(state_ref->frame_counter);
		char eventData[eventDataSize];

		std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - state_ref->start;
		frameRate = (double)5/elapsed_seconds.count();
		LOGGER()->info("[PROCFR: %.2lf]", frameRate);
		state_ref->start = std::chrono::high_resolution_clock::now();

		((double*)eventData)[0] = frameRate;
		((int*)&((double*)eventData)[1])[0] = state_ref->frame_counter;
		
		if(!send_frame_counter) {
			eventDataSize = eventDataSize - sizeof(state_ref->frame_counter);
		}
		state_ref->recorder->SendStatusUpdate(CPC_STATUS::FPS, eventData, eventDataSize);
	}
	state_ref->frame_counter++;

}

K4ARecorder::K4ARecorder(std::string controlPanelIP,
	std::string eventTCPPort,
	std::string myInterfaceIP,
	std::string statusTCPPort,
	bool disableHeartbeat) : ControlPanelConnector(controlPanelIP,
		eventTCPPort,
		myInterfaceIP,
		statusTCPPort,
		disableHeartbeat)
{

}

K4ARecorder::~K4ARecorder()
{

}

void run_system_command(const char* cmd_line)
{
    std::array<char, 128> buffer;
    std::string result;
	std::string full_cmd_line(cmd_line);
	full_cmd_line += " 2>&1";

    auto pipe = popen(full_cmd_line.c_str(), "r");
    
    if (!pipe) {
		LOGGER()->fatal("K4ARecorder", "ERROR running %s - popen() failed!", cmd_line);
        exit(1);
	}
    
    while (!feof(pipe)) {
        if (fgets(buffer.data(), 128, pipe) != nullptr)
            result += buffer.data();
    }
    
    auto rc = pclose(pipe);
    
    if (rc == EXIT_SUCCESS) {
		LOGGER()->info("Command result: %s", result.c_str());
    }
    else {
		LOGGER()->error("K4ARecorder", "ERROR running %s", cmd_line);
        LOGGER()->error("K4ARecorder", "%s", result.c_str());
    }
}


int start_video_capture(K4ARecorderState *state, bool send_frame_counter)
{
	// reset AzureKinectFirmwareTool
	run_system_command(reset_cmd);

	// wait 5 seconds before starting master
	if (wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER) {
		LOGGER()->info("[MASTER] Waiting for subordinates to finish initialization");
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	}

	LOGGER()->debug("Beginning recording!!");
	LOGGER()->debug("Sending %s as serial number message of length %d", serial_number_message+sizeof(int), serial_number_length);
	state->recorder->SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_CAPTURING_VIDEO_STARTED, serial_number_message, serial_number_length, Logger::Verbosity::Trace);

    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.color_format = recording_color_format;
    device_config.color_resolution = recording_color_resolution;
    device_config.depth_mode = recording_depth_mode;
    device_config.camera_fps = recording_rate;
    device_config.wired_sync_mode = wired_sync_mode;
    device_config.depth_delay_off_color_usec = depth_delay_off_color_usec;
    device_config.subordinate_delay_off_master_usec = subordinate_delay_off_master_usec;

    do_recording((uint8_t)device_index,
                        const_cast<char*>((MKVDestination + "/" + recording_filename).c_str()),
                        recording_length,
                        &device_config,
                        recording_imu_enabled,
                        absoluteExposureValue,
                        whiteBalance,
                        brightness,
                        contrast,
                        saturation,
                        sharpness,
                        gain,
						state, 
						send_frame_counter,
						SendFPS);
	return 0;
}

long K4ARecorder::EventReceived(CONTROL_PANEL_EVENT eventType, void* eventData, int dataSize)
{
	ControlPanelConnector::EventReceived(eventType, eventData, dataSize);

	auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
 
	switch (eventType)
	{
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_START_CAPTURING_VIDEO:
		{
			LOGGER()->info("Received start recording signal.  Starting recording...");
			g_delayStart = false;		
		}
		break;
	case CONTROL_PANEL_EVENT::CALIBRATION_SOFTWARE_STOP_CAPTURING_VIDEO:
		{			
			if (!exiting) {
				LOGGER()->info("Stopping recording...");
				exiting = true;
			}		
		}
		break;
	case CONTROL_PANEL_EVENT::CONTROL_PANEL_STOP_REQUESTED:
		{
			if(dataSize == 1)
			{
				LOGGER()->info("Got a system stop request!  Shutting down.");
				exiting = true;
			}			
		}
		break;
	default:
		LOGGER()->trace("[%s] Saw event %d, ignoring.", ctime(&timenow), static_cast<int>(eventType));
		break;
	}

	return S_OK;
}

static int string_compare(const char *s1, const char *s2)
{
    assert(s1 != NULL);
    assert(s2 != NULL);

    while (tolower((unsigned char)*s1) == tolower((unsigned char)*s2))
    {
        if (*s1 == '\0')
        {
            return 0;
        }
        s1++;
        s2++;
    }
    return (int)tolower((unsigned char)*s1) - (int)tolower((unsigned char)*s2);
}

int read_config_file(std::string configFile)
{
	CConfig* config = new CConfig(configFile);

	// Creating log files
	std::string LogFile("/var/log/K4AToFusion/K4ARecorder.log"); // default--overwritten by config file
	std::string ErrorFile("/var/log/K4AToFusion/K4ARecorder.error"); // default--overwritten by config file

	LogFile = config->GetValueWithDefault("K4ARecorder", "LogFilename", LogFile);
	ErrorFile = config->GetValueWithDefault("K4ARecorder", "ErrorFilename", ErrorFile);

	LOGGER()->set_verbosity((Logger::Verbosity)config->GetValueWithDefault("K4ARecorder", "Verbosity", 0));

	LOGGER()->set_filename(LogFile);
	LOGGER()->set_error_filename(ErrorFile);
	
	LOGGER()->info("K4ARecorder Output Log start");
	LOGGER()->error("K4ARecorder", "Error Log start");
	LOGGER()->info("K4ARecorder stdout/stderr redirected to %s and %s\n", LogFile.c_str(), ErrorFile.c_str());
	LOGGER()->info("Loaded configuration file %s\n", configFile.c_str());

	controlPanelIP = config->GetValueWithDefault("Network", "FusionIPAddress_PODNetwork", "192.168.101.250");
	eventTCPPort = config->GetValueWithDefault("Ports", "KNCSPublishEventPort", "14502");
	myIP = "*";
	statusTCPPort = config->GetValueWithDefault("Ports", "NanoApplicationStatusPort", "14502");
	disableHeartbeat = config->GetValueWithDefault("K4ARecorder", "DisableHeartbeat", false);

	// Retrieving pod number
    char hostname[1024];
    gethostname(hostname, 1024);
	thisPodNumber = atoi(&hostname[4]);
	LOGGER()->info("Got a hostname of \"%s\", setting this pod number to %d\n", hostname, thisPodNumber);
	std::string overrideSection = "Pod"+std::to_string(thisPodNumber)+"Override";

	// get destination where MKVs are stored 
	MKVDestination = config->GetValueWithDefault("K4ARecorder", "MKVDestination", "/mnt/USB");
	LOGGER()->info("MKV Destination: %s\n", MKVDestination.c_str());

	// get destination where MKVs are transferred to via scp
	transferDestination = config->GetValueWithDefault("K4ARecorder", "TransferDestination", "3dtm@peabodycontrolpanel:/3DTelemedicine/DATA/Calibration");
	LOGGER()->info("Transfer Destination: %s\n", transferDestination.c_str());

	// get device index from config file (if defined) or use default (one Kinect per nano)
	if (config->HasKey("K4ARecorder", "DeviceIndex")) {
		device_index = config->GetValue<int>("K4ARecorder", "DeviceIndex");
	}
	if (device_index < 0 || device_index > 255) {
		LOGGER()->error("K4ARecorder", "ERROR: Device index must be 0-255 [%d]", device_index);
		return 0;
	}
	LOGGER()->info("Device Index: %d", device_index);

	// get length from config file (if defined) or use default (no fixed recording length)
	if (config->HasKey("K4ARecorder", "RecordingLength")) {
		recording_length = config->GetValue<int>("K4ARecorder", "RecordingLength");
		if (recording_length < 0) {
			LOGGER()->error("K4ARecorder", "ERROR: Recording length must be positive [%d]", recording_length);
			return 0;
		}
	}
	LOGGER()->info("Recording Length: %d", recording_length);

	// get color mode from config file
	const char* color_mode = config->GetValueWithDefault("K4ARecorder", "ColorMode", "1080p");
	if (string_compare(color_mode, "3072p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_3072P;
	}
	else if (string_compare(color_mode, "2160p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_2160P;
	}
	else if (string_compare(color_mode, "1536p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_1536P;
	}
	else if (string_compare(color_mode, "1440p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_1440P;
	}
	else if (string_compare(color_mode, "1080p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_1080P;
	}
	else if (string_compare(color_mode, "720p") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_720P;
	}
	else if (string_compare(color_mode, "720p_NV12") == 0) {
		recording_color_format = K4A_IMAGE_FORMAT_COLOR_NV12;
		recording_color_resolution = K4A_COLOR_RESOLUTION_720P;
	}
	else if (string_compare(color_mode, "720p_YUY2") == 0) {
		recording_color_format = K4A_IMAGE_FORMAT_COLOR_YUY2;
		recording_color_resolution = K4A_COLOR_RESOLUTION_720P;
	}
	else if (string_compare(color_mode, "off") == 0) {
		recording_color_resolution = K4A_COLOR_RESOLUTION_OFF;
	}
	else {
		LOGGER()->error("K4ARecorder", "ERROR: Unknown color mode specified [%s]", color_mode);
		return 0;
	}
	LOGGER()->info("Color Mode: %s", color_mode);

	// get depth mode from config file
	const char* depth_mode = config->GetValueWithDefault("K4ARecorder", "DepthMode", "NFOV_UNBINNED");
	if (string_compare(depth_mode, "NFOV_2X2BINNED") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
	}
	else if (string_compare(depth_mode, "NFOV_UNBINNED") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	}
	else if (string_compare(depth_mode, "WFOV_2X2BINNED") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
	}
	else if (string_compare(depth_mode, "WFOV_UNBINNED") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
	}
	else if (string_compare(depth_mode, "PASSIVE_IR") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
	}
	else if (string_compare(depth_mode, "off") == 0) {
		recording_depth_mode = K4A_DEPTH_MODE_OFF;
	}
	else {
		LOGGER()->error("K4ARecorder", "ERROR: Unknown depth mode specified [%s]", depth_mode);
		return 0;
	}
	LOGGER()->info("Depth Mode: %s", depth_mode);

	// get depth delay from config file (if defined) or use default (no delay between color and depth) 
	if (config->HasKey("K4ARecorder", "DepthDelay")) {
		depth_delay_off_color_usec = config->GetValue<int32_t>("K4ARecorder", "DepthDelay");
	}
	LOGGER()->info("Depth Delay: %d", depth_delay_off_color_usec);

	// get recording frame rate from config file
	int rate = config->GetValueWithDefault("K4ARecorder", "FrameRate", 5);
	if (rate == 30) {
		recording_rate = K4A_FRAMES_PER_SECOND_30;
	}
	else if (rate == 15) {
		recording_rate = K4A_FRAMES_PER_SECOND_15;
	}
	else if (rate == 5) {
		recording_rate = K4A_FRAMES_PER_SECOND_5;
	}
	else {
		LOGGER()->error("K4ARecorder", "ERROR: Unknown frame rate specified [%d]", rate);
		return 0;
	}
	LOGGER()->info("Rate: %d", rate);

	if (recording_rate == K4A_FRAMES_PER_SECOND_30 && (recording_depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED || recording_color_resolution == K4A_COLOR_RESOLUTION_3072P)) {
		LOGGER()->error("K4ARecorder", "ERROR: 30 Frames per second is not supported by this camera mode.");
		return 0;
    }

	// get IMU from config file (if defined) or use deafult (true)
	if (config->HasKey("K4ARecorder", "IMU")) {
		recording_imu_enabled = config->GetValue<bool>("K4ARecorder", "IMU");
	}
	LOGGER()->info("IMU: %d", recording_imu_enabled);

	// get sync mode from config file (override section)
	std::string external_sync_str = "";
	int external_sync = 2; 	// For compatibility with AzureKinectNanoToFusion, we need to use the
							// numbering scheme instead of strings (0 - standalone, 1 - master, 2 - subordinate)
	if (config->HasKey(overrideSection, "K4ASettingsSyncMode")) {
		external_sync = config->GetValue<int>(overrideSection, "K4ASettingsSyncMode");
	}
	if (external_sync == 1) {
		wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
		external_sync_str = "master";
	}
	else if (external_sync == 2) {
		wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
		external_sync_str = "subordinate";
	}
	else if (external_sync == 0) {
		wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		external_sync_str = "standalone";
	}
	else {
		LOGGER()->error("K4ARecorder", "ERROR: Unknown sync mode specified [%d]", external_sync);
		return 0;
	}
	LOGGER()->info("External Sync: %s", external_sync_str.c_str());

	// get sync delay from config file (override section)
	int sync_delay = config->GetValueWithDefault(overrideSection, "K4ASettingsSyncDelay", 0);
	if (sync_delay < 0) {
		LOGGER()->error("K4ARecorder", "ERROR: External sync delay must be positive [%d]", sync_delay);
		return 0;
	}
	subordinate_delay_off_master_usec = (uint32_t)sync_delay;
	LOGGER()->info("Sync Delay: %d", subordinate_delay_off_master_usec);
	if (subordinate_delay_off_master_usec > 0 && wired_sync_mode != K4A_WIRED_SYNC_MODE_SUBORDINATE) {
		LOGGER()->error("K4ARecorder", "ERROR: sync-delay is only valid for Subordinate mode.");
        return 0;
    }

	// get exposure from config file
	int exposureValue = config->GetValueWithDefault("K4ARecorder", "ExposureValue", -9);
	if (exposureValue != defaultExposureAuto) {
		if (exposureValue >= -11 && exposureValue <= 1) {
			absoluteExposureValue = static_cast<int32_t>(exp2f((float)exposureValue) * 1000000.0f);
		}
		else if (exposureValue >= 2 && exposureValue <= 200000) {
			absoluteExposureValue = exposureValue;
		}
		else {
			LOGGER()->error("K4ARecorder", "ERROR: Exposure value range is 2 to 5s, or -11 to 1 [%d]", exposureValue);
			return 0;
		}
	}
	LOGGER()->info("Exposure Value: %d", exposureValue);

	// set white balance to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "WhiteBalance")) {
		int whiteBalanceSetting = config->GetValue<int>("K4ARecorder", "WhiteBalance");
		if (whiteBalanceSetting != defaultWhiteBalance) {
			if (whiteBalanceSetting < 2500 || whiteBalanceSetting > 12500 || whiteBalanceSetting % 10 != 0) {
				LOGGER()->error("K4ARecorder", "ERROR: White balance setting has invalid value [%d]", whiteBalanceSetting);
				return 0;
			}
			whiteBalance = whiteBalanceSetting;
		}
	}
	LOGGER()->info("White Balance: %d", whiteBalance);
	
	// set brightness to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "Brightness")) {
		int brightnessSetting = config->GetValue<int>("K4ARecorder", "Brightness");
		if (brightnessSetting != defaultBrightness) {
			if (brightnessSetting < 0 || brightnessSetting > 255) {
				LOGGER()->error("K4ARecorder", "ERROR: Brightness setting has invalid value [%d]", brightnessSetting);
				return 0;

			}
			brightness = brightnessSetting;
		}
	}
	LOGGER()->info("Brightness: %d", brightness);

	// set contrast to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "Contrast")) {
		int contrastSetting = config->GetValue<int>("K4ARecorder", "Contrast");
		if (contrastSetting != defaultContrast) {
			if (contrastSetting < 0 || contrastSetting > 10) {
				LOGGER()->error("K4ARecorder", "ERROR: Contrast setting has invalid value [%d]", contrastSetting);
				return 0;
			}
			contrast = contrastSetting;
		}
	}
	LOGGER()->info("Contrast: %d", contrast);

	// set saturation to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "Saturation")) {
		int saturationSetting = config->GetValue<int>("K4ARecorder", "Saturation");
		if (saturationSetting != defaultSaturation) {
			if (saturationSetting < 0 || saturationSetting > 63) {
				LOGGER()->error("K4ARecorder", "ERROR: Saturation setting has invalid value [%d]", saturationSetting);
				return 0;
			}
			saturation = saturationSetting;
		}
	}
	LOGGER()->info("Saturation: %d", saturation);

	// set sharpness to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "Sharpness")) {
		int sharpnessSetting = config->GetValue<int>("K4ARecorder", "Sharpness");
		if (sharpnessSetting != defaultSharpness) {
			if (sharpnessSetting < 0 || sharpnessSetting > 4) {
				LOGGER()->error("K4ARecorder", "ERROR: Sharpness setting has invalid value [%d]", sharpnessSetting);
				return 0;
			}
			sharpness = sharpnessSetting;
		}
	}
	LOGGER()->info("Sharpness: %d", sharpness);

	// set gain to auto or get it from config file (if defined)
	if (config->HasKey("K4ARecorder", "Gain")) {
		int gainSetting = config->GetValue<int>("K4ARecorder", "Gain");
		if (gainSetting != defaultGainAuto) {
			if (gainSetting < 0 || gainSetting > 255) {
				LOGGER()->error("K4ARecorder", "ERROR: Gain setting has invalid value [%d]", gainSetting);
				return 0;
			}
			gain = gainSetting;
		}
	}
	LOGGER()->info("Gain: %d", gain);

	return 1;
}

void Cleanup(K4ARecorder &recorder)
{
	std::string umount_cmd = "umount " + MKVDestination;
	run_system_command(umount_cmd.c_str());
	LOGGER()->debug("USB Unmounted.  Sending Stopped update");
	// Letting control panel know we are done executing
	recorder.SendStatusUpdate(CPC_STATUS::STOPPED);
	
	LOGGER()->close();
}

int main(int argc, char* argv[])
{
	exiting = false;
	// Loads config file
	const std::string defaultWorkingDirectory("/home/peabody");
	const std::string defaultCalibrationDirectory(defaultWorkingDirectory+"/K4AToFusion");
	std::string configFile(defaultCalibrationDirectory+"/3DTelemedicine.cfg");

	if(argc > 1) {
		if(!std::experimental::filesystem::exists(argv[1])) {
			LOGGER()->fatal("K4ARecorder", "Unable to load configuration file!");
			return 0;
		}
		configFile = std::string(argv[1]);
	}
	
	if (!read_config_file(configFile)) {
		LOGGER()->error("K4ARecorder", "Exiting");
		return 0;
	}

	// set output MKV filename according to the pod number
	std::string output_file = "output-" + std::to_string((int)thisPodNumber - 1) + ".mkv";
	recording_filename = const_cast<char*>(output_file.c_str());
	LOGGER()->info("Output MKV Filename: %s\n", recording_filename);

	// Initializing Control Panel Connector
	K4ARecorder recorder(controlPanelIP, eventTCPPort, myIP, statusTCPPort, disableHeartbeat);
	recorder.SetVersionInformation(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMITS, VERSION_BRANCH_NAME, VERSION_DESCRIPTION, VERSION_SHA1);

	// reset AzureKinectFirmwareTool
	run_system_command(reset_cmd);
	
	// send serial number to control panel
	k4a_device_t device;
    if (K4A_FAILED(k4a_device_open(device_index, &device))) {
		LOGGER()->error("K4ARecorder", "Runtime error: k4a_device_open() failed.");
		return 0;
    }

	if (k4a_device_get_serialnum(device, NULL, &serial_number_length) != K4A_BUFFER_RESULT_TOO_SMALL) {
		LOGGER()->error("K4ARecorder", "Failed to get serial number length.");
		k4a_device_close(device);
		return 0;
	}

	serial_number_message = (char*)malloc(serial_number_length + sizeof(int));
	if (serial_number_message == NULL) {
		LOGGER()->error("K4ARecorder", "Failed to allocate memory for serial number (%zu bytes)\n", serial_number_length);
		k4a_device_close(device);
		return 0;
	}

	if (k4a_device_get_serialnum(device, serial_number_message + sizeof(int), &serial_number_length) != K4A_BUFFER_RESULT_SUCCEEDED) {
		LOGGER()->error("K4ARecorder", "Failed to get serial number.");
		free(serial_number_message);
		k4a_device_close(device);
		return 0;
	}

	k4a_device_close(device);
	
		// mount USB disk
	std::string mount_cmd = "mount " + MKVDestination + " -v";
	run_system_command(mount_cmd.c_str());

	// Passing the serial number size first and then the serial number chracters in a single message.
	// Control Panel expects to receive the size of the serial number as an int.
	((int*)serial_number_message)[0] = (int)serial_number_length;
	serial_number_length += sizeof(int);
	LOGGER()->info("Transmitting device serial number %s with length %d", serial_number_message + sizeof(int), serial_number_length);
	recorder.SendStatusUpdate(CPC_STATUS::READY, serial_number_message, serial_number_length);
	
	K4ARecorderState wait_state;

	wait_state.frame_counter = 0;
	wait_state.start = std::chrono::high_resolution_clock::now();
	wait_state.recorder = &recorder;
	
	while (g_delayStart && !exiting)
    {
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		
		SendFPS((void*)&wait_state, false);
    }
	if(exiting)
	{
		// Got a cancel request, exit gracefully.
		Cleanup(recorder);
		return 0;
	}

	K4ARecorderState recorder_state;

	recorder_state.frame_counter = 0;
	recorder_state.start = std::chrono::high_resolution_clock::now();
	recorder_state.recorder = &recorder;

	start_video_capture(&recorder_state, true);

	recorder.SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_TRANSFERRING_VIDEO, serial_number_message, serial_number_length);

	std::string transfer_cmd = "scp " + MKVDestination + "/" + recording_filename + " " + transferDestination + "/" + recording_filename;
	LOGGER()->info("Transferring data to Fusion: %s\n", transfer_cmd.c_str());
	run_system_command(transfer_cmd.c_str());
	LOGGER()->debug("Transfer completed.  Sending the done status signal.");
	recorder.SendStatusUpdate(CPC_STATUS::CALIBRATION_SOFTWARE_VIDEO_TRANSFER_DONE, serial_number_message, serial_number_length);
	LOGGER()->debug("Done status signal sent.");
	Cleanup(recorder);

	return 0;

}

