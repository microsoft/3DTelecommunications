#include "stdafx.h"
#include "K4ASourceManager.h"
#include "K4ACaptureSource.h"
#include "Util.h"
#include "K4ADepthProcessing.h"
#include "LibUtility/include/Config.h"


#define SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

////////////
// 
//  Private internal functions / definitions


using namespace K4AToFusionUtils;

const std::string GlobalConfigSection = "PodGlobalConfig";

struct CameraInstanceConfig
{
    int32_t idx;
    int32_t syncMode;
    int32_t syncDelay;
    K4ADeviceColorControls colorControls;
};

bool LoadCameraFile(const char* path, int thisPodNumber, SharedSettings& settings, k4a_device_configuration_t& config)
{

	CConfig currentConfig;
	currentConfig.Load(path);
    std::string overrideSection = "Pod"+std::to_string(thisPodNumber)+"Override";

    config.color_format = (k4a_image_format_t)currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "ColorFormat");

    switch(currentConfig.GetValue<int>("ColorProcessing", "ColorImageHeight"))
    {
        case 720:
            config.color_resolution = K4A_COLOR_RESOLUTION_720P;
            break;
        case 1080:
            config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
            break;
        case 1440:
            config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
            break;
        case 1536:
            config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
            break;
        case 2160:
            config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
            break;
        case 3072:
            config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
            break;
        default:
            printf("ColorFormat is not an acceptible resolution for the kinect.  Turning OFF color.\n");
            config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
            break;
    }

    config.depth_mode = (k4a_depth_mode_t)currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "DepthMode");
    std::cout << "Set depth mode to " << config.depth_mode << std::endl;

    config.camera_fps = (k4a_fps_t)currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "CameraFPS");

    config.synchronized_images_only = currentConfig.GetValue<bool>(overrideSection, GlobalConfigSection, "SyncImages");

	settings.skipFirstNFrames = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "SkipFirstNFrames");

    settings.minIRRange = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "MinIRRange");
	settings.maxIRRange = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "MaxIRRange");

    settings.minDepth = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "MinDepth");
    settings.maxDepth = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "MaxDepth");

	settings.fusionWidth = currentConfig.GetValue<int>("DepthGeneration", "DepthImageWidth");
	settings.fusionHeight = currentConfig.GetValue<int>("DepthGeneration", "DepthImageHeight");

    settings.applyMask = currentConfig.GetValue<bool>(overrideSection, GlobalConfigSection, "ApplyMask");

	settings.numStdDevFromBG = currentConfig.GetValue<double>(overrideSection, GlobalConfigSection, "NumStdDevFromBG");

	settings.recordingPath = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "RecordingPath", "/home/peabody/K4AToFusion");

	settings.calibPath = currentConfig.GetValue<std::string>(overrideSection, GlobalConfigSection, "CalibrationPath");

	settings.numKParams = currentConfig.GetValue<int>(overrideSection, GlobalConfigSection, "NumKParams");

	settings.outputForMultiViewCalib = currentConfig.GetValue<bool>(overrideSection, GlobalConfigSection, "OutputForMultiViewCalib");
	
	settings.calibPublisherIPAddress = currentConfig.GetValue<std::string>("Network", "DepthPodIPBase") + "1";
	settings.calibPublisherPort = currentConfig.GetValue<std::string>("Ports", "NanoInterCalibrationSyncPort");

    settings.controlPanelIPAddress = currentConfig.GetValue<std::string>("Network", "FusionIPAddress_PODNetwork");
    settings.eventTCPPort = currentConfig.GetValueWithDefault("Ports", "KNCSPublishEventPort", "14502");
    settings.statusTCPPort = currentConfig.GetValueWithDefault("Ports", "NanoApplicationStatusPort", "14502");

	settings.frameRecordingFrequency = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "FrameRecordingFrequency", 1);
	settings.framesToGrabEachBurst = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "FrameToGrabEachBurst", 1);

	settings.calibFormat = (CalibFormat) currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "CalibFormat", 0);

    settings.maxSavedCalibrationFrames = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "MaxSavedCalibrationFrames", DEFAULT_MAX_SAVED_CALIBRATION_FRAMES); 
    settings.minBackgroundSamples = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "MinBackgroundSamples", DEFAULT_MIN_BACKGROUND_SAMPLES);
    settings.maxBufferSize = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "MaxBufferSize", DEFAULT_MAX_BUFFER_SIZE);
    // Do some sanity checks
    if(settings.maxSavedCalibrationFrames + settings.maxBufferSize > MAX_ALLOWABLE_BUFFER_SIZE)
    {
        std::cout << "Sorry, setting maxSavedCalibrationFrames to " << settings.maxSavedCalibrationFrames << " and maxBufferSize to " \
        << settings.maxBufferSize << " may cause out of memory issues.  I'm forcing these back to " << DEFAULT_MAX_SAVED_CALIBRATION_FRAMES \
        << " and " << DEFAULT_MIN_BACKGROUND_SAMPLES << ".  These two values must be <= " << MAX_ALLOWABLE_BUFFER_SIZE << std::endl;
        settings.maxSavedCalibrationFrames = DEFAULT_MAX_SAVED_CALIBRATION_FRAMES; 
        settings.maxBufferSize = DEFAULT_MAX_BUFFER_SIZE;
    }
    // Depth interpolation and smoothing knobs
    settings.interpolationEdgeThreshold = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "InterpolationEdgeThreshold", 0);
    settings.temporalSmoothingThreshold = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "TemporalSmoothingThreshold", 0);    
    settings.depthErosionMaxZeros = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "DepthErosionMaxZeros", 0);
    settings.depthErosionSearchRadius =  currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "DepthErosionSearchRadius", 1);
    settings.temporalErosionFrames = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "TemporalErosionFrames", 0);
    if(settings.temporalErosionFrames > 8)
    {
        std::cerr << "Tried to set temporal erosion frames > 8.  Forcing to max of 8." << std::endl;
        settings.temporalErosionFrames = 8;
    }
    settings.statisticsMode = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "StatisticsMode", false);

    // K4A Camera settings
    settings.startMode = (StartMode)currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "CameraStartMode", DEFAULT_CAMERA_START_MODE);
    settings.startDelayMS = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "StartDelayMS", DEFAULT_START_DELAY_MS);
    settings.brightness = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsBrightness", DEFAULT_K4A_BRIGHTNESS);
    settings.contrast = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsContrast", DEFAULT_K4A_CONTRAST);
    settings.saturation = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsSaturation", DEFAULT_K4A_SATURATION);
    settings.sharpness = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsSharpness", DEFAULT_K4A_SHARPNESS);
    settings.whitebalance = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsWhitebalance", DEFAULT_K4A_WHITEBALANCE);
    settings.backlightCompensation = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsBacklightCompensation", DEFAULT_K4A_BACKLIGHT_COMPENSATION);
    settings.gain = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsGain", DEFAULT_K4A_GAIN);
    settings.exposureTimeUS = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsExposure", DEFAULT_K4A_EXPOSURE);
    settings.powerlineFrequency = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsPowerlineFrequency", DEFAULT_K4A_POWERLINE_FREQUENCY);
    settings.syncMode = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsSyncMode", DEFAULT_K4A_SYNC_MODE);
    settings.syncDelay = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "K4ASettingsSyncDelay", DEFAULT_K4A_SYNC_DELAY);
    settings.verbosity = currentConfig.GetValueWithDefault(overrideSection, GlobalConfigSection, "Verbosity", 0);

    printf(" %d: Settings:\nStart Mode: %d Start Delay: %dms Sync: %d  Delay: %d\nBrightness: %d Contrast: %d Saturation: %d Sharpness: %d Whitebalance: %d Backlight Compensation: %d Gain: %d Exposure: %d Powerline Frequency: %s\n", 
        thisPodNumber, settings.startMode, settings.startDelayMS, settings.syncMode, settings.syncDelay, 
        settings.brightness, settings.contrast, settings.saturation, settings.sharpness, 
        settings.whitebalance, settings.backlightCompensation, settings.gain, settings.exposureTimeUS, settings.powerlineFrequency == 2?"60Hz":"50Hz");

    return true;
}


int InitDevices(
    const k4a_device_configuration_t& config, 
    const SharedSettings& settings,
    uint32_t& nDevices, 
    K4ACaptureSource** captureSources, 
    int32_t& masterIdx, 
    bool captureIR, 
    bool ignoreSyncDelay)
{
    // Audio initialization
    const int audioInitStatus = k4aviewer::K4AAudioManager::Instance().Initialize();
    if(audioInitStatus != SoundIoErrorNone)
    {
        std::stringstream errorBuilder;
        errorBuilder << "Failed to initialize audio backend: " << soundio_strerror(audioInitStatus) << "!";
        return 0;
    }

    // # of cameras connected
    uint32_t connectedCnt = k4a_device_get_installed_count();
    if(connectedCnt != 1)
    {
        printf("  Found %d devices connected...Only 1 device is allowed to be connected in this version of the software.\n", connectedCnt);
        printf("Aborting...\n");
        return 0;
    }
    nDevices = connectedCnt;

    *captureSources = new K4ACaptureSource[connectedCnt];
	for (unsigned int i = 0; i < nDevices; ++i)
	{
		captureSources[i]->SetCalibFormat(settings.calibFormat);
	}

    printf("  Initializing %d devices...\n", nDevices);
    masterIdx = 0; // device 0 is always the master _on_this_pod_ because we only ever have one device per pod

    // initialize each camera, and store them in expected order by serial number (if given)
    for (uint32_t i = 0; i < connectedCnt; ++i)
    {
        K4ADevice device;
        device.initialized = false;
        k4a_result_t openResult = k4a_device_open(i, &device.deviceHandle);
        if (K4A_FAILED(openResult))
        {
            printf("!!!!! Failed to open device %d\n", i);
            return 0;
        }

        size_t serialSize;
        k4a_device_get_serialnum(device.deviceHandle, NULL, &serialSize);
        device.serial = new char[serialSize];
        memset(device.serial, '\0', serialSize);
        k4a_device_get_serialnum(device.deviceHandle, device.serial, &serialSize);
        printf("  Opened device [%s]\n", device.serial);

        device.initialized = true;
        device.id = 0;
        device.config = config;
        device.config.wired_sync_mode = (k4a_wired_sync_mode_t)settings.syncMode;
        device.config.subordinate_delay_off_master_usec = settings.syncDelay;
        K4ADeviceColorControls colorControls {settings.exposureTimeUS, settings.brightness, settings.contrast, settings.saturation, settings.sharpness, settings.gain, settings.whitebalance, settings.backlightCompensation, settings.powerlineFrequency};
        device.colorControls = colorControls;
        K4ACaptureSource &thisDevice = (*captureSources)[device.id];
        
        if (!thisDevice.InitDevice(device, captureIR))
        {
            printf("!!!!! Failed to initialized device.\n");            
            return 0;
        }        
        
        // Init microphone
        thisDevice.g_K4AMicrophone = k4aviewer::K4AAudioManager::Instance().GetMicrophoneForDevice(device.serial);
        if(thisDevice.g_K4AMicrophone == nullptr)
        {
            std::cout << "Could not get Kinect microphone. Aborting." << std::endl;
            return 0;
        }
        if(!thisDevice.g_K4AMicrophone->IsStarted())
        {
            std::cout << "Starting K4A Microphone" << std::endl;
            thisDevice.g_K4AMicrophone->Start();
        }
        thisDevice.g_K4AMicrophoneListener = thisDevice.g_K4AMicrophone->CreateListener();
        if(thisDevice.g_K4AMicrophoneListener == nullptr)
        {
            std::cout << "Error creating microphone listener." << soundio_strerror(thisDevice.g_K4AMicrophone->GetStatusCode()) << std::endl;
            return 0;
        }
    }

    if ((settings.startMode == START_MODE_MANUAL_MASTER || settings.startMode == START_MODE_CONTROL_PANEL_MASTER) &&
        (masterIdx < 0))
    {
        printf("!!!!! Start mode set to master, but no master camera specified\n");
        return 0;
    }

    return 1;
}





/////////////////////////////////////////////////
///
/// Exposed Member functions

K4ASourceManager::K4ASourceManager() :
    m_masterIdx(-1),
    m_numSources(0),
    m_sources(nullptr),
    m_thisPodNumber(0)
{    
    SharedSettings m_settings{};
}


K4ASourceManager::~K4ASourceManager()
{
    Stop();    
    delete[] m_sources;
}

void K4ASourceManager::SetThisPodNumber(const uint num)
{
    m_thisPodNumber = num;
}

void K4ASourceManager::SetControlPanelConnector(K4AControlPanelConnector* cpc)
{
       // pass in the ControlPanelConnector
    m_k4acpc = cpc;
}

int K4ASourceManager::Initialize(const k4a_device_configuration_t& defaultConfig, const char* configFilePath, bool captureIR, bool isCalibCapture)
{
    // specify camera configuration: NOTE, these are for ALL cameras, camera instance specific settings are within LoadCameraFile    
    k4a_device_configuration_t config = defaultConfig;    

    // load camera file
    // NOTE this will overwrite default settings if config file exists
    if ((configFilePath != nullptr) && !LoadCameraFile(configFilePath, m_thisPodNumber, m_settings, config))
    {
        printf("WARNING: Failed to load camera serial file, using connected cameras.\n");
    }
    if(isCalibCapture)
    {
        std::cout << "WARNING:  Calibration mode selected.  Forcing color output to BRGA and FPS to 15" << std::endl;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }
   

    CConfig currentConfig;
    currentConfig.Load(configFilePath);
    bool localDelayIgnore = currentConfig.GetValueWithDefault(GlobalConfigSection, "K4AIgnoreDelay", false);

    // initialize devices
    m_masterIdx = -1;    
    if (!InitDevices(config, m_settings, m_numSources, &m_sources, m_masterIdx, captureIR, localDelayIgnore))
    {
        printf("Failed to initialize cameras, aborting!\n");
        return 0;
    }

    return 1;
}


void K4ASourceManager::HandleStartDelays(bool isMaster)
{

    if ((m_settings.startMode == START_MODE_AUTO) &&
        (m_settings.startDelayMS > 0))
    {
        printf("Waiting for %d MS to start camera...\n", m_settings.startDelayMS);
        SLEEP(m_settings.startDelayMS);
    }
    else if (m_settings.startMode == START_MODE_MANUAL_EACH)
    {
        printf("Waiting for user input to start camera. Press enter to continue...\n");
        getchar();
    }
    else if (m_settings.startMode == START_MODE_MANUAL_MASTER &&
        isMaster)
    {
        printf("Waiting for user input to start camera. Press enter to continue...\n");
        getchar();
    }
    else if (m_settings.startMode == START_MODE_CONTROL_PANEL_MASTER && 
        isMaster)
    {
        int statsUpdateCounter = 0;
        if(m_settings.verbosity > 0)
            printf("Waiting for start signal from control panel...\n");
        m_k4acpc->SendStatusUpdate(CPC_STATUS::READY, m_k4acpc->GetIdentificationString().c_str(), m_k4acpc->GetIdentificationString().length());
        while(!m_k4acpc->GetStartCommand() && !m_k4acpc->GetStopCommand())  //check stop as well so I can exit even if I never got a start
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            statsUpdateCounter++;
            if(statsUpdateCounter > 100)
            {
                if(m_settings.verbosity > 0)
                    printf("Waiting for start signal from control panel...\n");
                m_k4acpc->SendStatusUpdate(CPC_STATUS::READY, m_k4acpc->GetIdentificationString().c_str(), m_k4acpc->GetIdentificationString().length());
                statsUpdateCounter = 0;
            }
        }
        m_k4acpc->SetStartCommand(false);        
    }
    else
    {
        if(m_settings.verbosity > 0)
            std::cout << "Camera is subordinate.  Waiting for sync pulse from primary camera" << std::endl;
    }
    
}


bool K4ASourceManager::StartCamera(K4ACaptureSource& source, bool isMaster, const char* label)
{
    HandleStartDelays(isMaster);
    printf(" Starting %s camera [%s]...\n", label, source.GetSerial());
    if (source.StartCameras() == 0)
    {
        printf(" FAILED\n");
        return false;
    }
    printf(" SUCCESS\n");
    return true;
}


int K4ASourceManager::Start()
{
    // start all cameras
    for (uint32_t i = 0; i < m_numSources; ++i)
    {
        // skip master camera (still valid if all cameras in standalone mode: masterIdx < 0)
        if (i == (uint32_t)m_masterIdx)
            continue;

        if (!StartCamera(m_sources[i], false, m_masterIdx < 0 ? "" : "SUB"))
            return 0;
    }

    // start master last
    if (m_masterIdx >= 0)
    {
        if (!StartCamera(m_sources[m_masterIdx], true, "MASTER"))
            return 0;
    }

    m_isStarted = true;
    return 1;
}


void K4ASourceManager::Stop()
{
    if(m_isStarted)
    {
        // stop all cameras
        for (uint32_t i = 0; i < m_numSources; ++i)
        {
            std::cout << "Stopping camera " << i;
            m_sources[i].StopCameras();
            std::cout << "...Done. Stopping microphone ";
            m_sources[i].g_K4AMicrophone->Stop();
            std::cout << "Done." << std::endl;

        }
        m_isStarted = false;
    }
}


uint32_t K4ASourceManager::GetNumSources() const
{
    return m_numSources;
}


K4ACaptureSource* K4ASourceManager::GetCaptureSources()
{
    return m_sources;
}

const SharedSettings& K4ASourceManager::GetSharedSettings() const
{
    return m_settings;
}


void K4ASourceManager::SaveCalibrations(const char* path)
{
    for (uint32_t i = 0; i < m_numSources; ++i)
    {
        m_sources[i].SaveCalibration(path);
    }
}

bool K4ASourceManager::LoadCalibrations(const char* path, uint16_t numKParams)
{
    bool success = true;
    for (uint32_t i = 0; i < m_numSources; ++i)
    {
        success = success && m_sources[i].LoadCalibrationFromFile(path, numKParams);
    }

    return success;
}

bool K4ASourceManager::LoadFactoryCalibrations()
{
    for (uint32_t i = 0; i < m_numSources; ++i)
    {
        m_sources[i].LoadFactoryCalibration();
    }

    return true;
}
