#pragma once

#include "K4ACommon.h"
#include "K4AControlPanelConnector.h"

#define DEFAULT_MAX_SAVED_CALIBRATION_FRAMES    100
#define DEFAULT_MIN_BACKGROUND_SAMPLES          200
#define DEFAULT_MAX_BUFFER_SIZE                 50 //this is max buffer for overflow, not for calibration.  That is set in the config file
#define MAX_ALLOWABLE_BUFFER_SIZE               150   //50 images took 25% of the ram on the nano, we really shouldn't go over 75% with buffer size and calibration frames
#define DEFAULT_K4A_BRIGHTNESS                  128
#define DEFAULT_K4A_CONTRAST                    5
#define DEFAULT_K4A_SATURATION                  32
#define DEFAULT_K4A_SHARPNESS                   2
#define DEFAULT_K4A_BACKLIGHT_COMPENSATION      false
#define DEFAULT_K4A_POWERLINE_FREQUENCY         2 // 1 = 50Hz, 2 = 60Hz
#define DEFAULT_K4A_WHITEBALANCE                -1
#define DEFAULT_K4A_GAIN                        0
#define DEFAULT_K4A_EXPOSURE                    16000
#define DEFAULT_CAMERA_START_MODE               0 // auto start with delay
#define DEFAULT_START_DELAY_MS                  0
#define DEFAULT_K4A_SYNC_MODE                   0 // standalone
#define DEFAULT_K4A_SYNC_DELAY                  0

class K4ACaptureSource;

class K4ASourceManager
{
private:
    int32_t              m_masterIdx;
    uint32_t             m_numSources;
    SharedSettings       m_settings;
    K4ACaptureSource*    m_sources;
    K4AControlPanelConnector* m_k4acpc;

    uint m_thisPodNumber;
    bool m_isStarted;
    void  HandleStartDelays(bool isMaster);    
    bool  StartCamera(K4ACaptureSource& source, bool isMaster, const char* label);

public:
    K4ASourceManager();
    ~K4ASourceManager();

    void SetControlPanelConnector(K4AControlPanelConnector* cpc);
    void SetThisPodNumber(const uint podNumber);
    
    int32_t   Initialize(const k4a_device_configuration_t& defaultConfig, const char* configFilePath = nullptr, bool captureIR = false, bool ignoreSyncDelay = false);
    int32_t   Start();
    void      Stop();
  
    uint32_t                GetNumSources() const;
    K4ACaptureSource*       GetCaptureSources();
    const SharedSettings&   GetSharedSettings() const;
    void                    SaveCalibrations(const char* path);
    bool                    LoadCalibrations(const char* path, uint16_t numKParams = 6);
    bool                    LoadFactoryCalibrations();
};

