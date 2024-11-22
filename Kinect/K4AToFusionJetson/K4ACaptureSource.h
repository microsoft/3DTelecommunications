#pragma once

#include "Util.h"
#include "StreamsDef.h"
#include <k4a/k4a.h>
// Audio
#include "k4aaudiomanager.h"
#include "k4amicrophone.h"
#include "k4amicrophonelistener.h"

struct K4ADeviceColorControls
{
    int      exposureTimeUS;
    int16_t  brightness;
    uint8_t  contrast;
    short    saturation; // [0-63]
    short    sharpness; // [0-4]
    int16_t  gain;
    int16_t  colorTemp; //whitebalance. The unit is degrees Kelvin. The setting must be set to a value evenly divisible by 10 degrees.
    bool     backlightCompensation; 
    short    powerlineFrequency;
};


struct K4ADevice
{
    int                        id;
    bool                       initialized;
    char*                      serial;
    k4a_device_t               deviceHandle;
    k4a_device_configuration_t config;
    K4ADeviceColorControls     colorControls;
};


typedef struct _k4a_image_descriptor_t
{
    int width_pixels;  /**< image width in pixels */
    int height_pixels; /**< image height in pixels */
    int stride_bytes;  /**< image stride in bytes */
} k4a_image_descriptor_t;


class K4ACaptureSource
{
private:
    K4ADevice  m_device;
    bool       m_camerasStarted;
    bool       m_captureIR;

    k4a_capture_t m_capture;

    k4a_image_t m_colorImage;
    k4a_image_descriptor_t m_colorImgDesc;

    k4a_image_t m_depthImage;
    k4a_image_descriptor_t m_depthImgDesc;    

    k4a_image_t m_irImage;

    uint64_t m_latestTimestamp;

    Calibration m_colorCal;
    Calibration m_depthCal;

	k4a_calibration_t m_factoryCalibration;

	CalibFormat m_calibFormat;

    uint8_t* m_audioBuffer;

    k4a_imu_sample_t m_imuReading;

public:
    K4ACaptureSource();
    ~K4ACaptureSource();

    const char* GetSerial();

    bool IsInitialized();
    int InitDevice(K4ADevice device, bool captureIR = false);

    int StartCameras();
    void StopCameras();

    int CaptureFrame();

    void ReleaseFrame();

    double GetFrameDelayUSec();

	void SetCalibFormat(CalibFormat cf) { m_calibFormat = cf; }

    const k4a_image_descriptor_t& GetDepthImgDesc();    
    const k4a_image_descriptor_t& GetColorImgDesc();
    uint8_t* GetColorBuff();
    uint8_t* GetIRBuff();
    uint8_t* GetDepthBuff();
    uint8_t* GetAudioBuff();
    uint64_t GetLatestTimestamp();
    int GetColorImageBufferSize();
    int GetDepthImageBufferSize();
    int GetColorWidth();
    int GetColorHeight();
    int GetDepthWidth();
    int GetDepthHeight();

	const k4a_image_t GetColorImageHandle() { return m_colorImage; };
	const k4a_image_t GetDepthImageHandle() { return m_depthImage; };
	const K4ADevice GetDevice() { return m_device; };
	const k4a_calibration_t GetFactoryCalibration() { return m_factoryCalibration; }; 
    const k4a_imu_sample_t GetIMUReading();
    const Calibration& GetDepthCalibration();
    const Calibration& GetColorCalibration();

    void LoadFactoryCalibration();
    bool LoadCalibrationFromFile(const char* path, uint16_t numKParams);
    void SaveCalibration(const char* path);
    std::shared_ptr<k4aviewer::K4AMicrophone> g_K4AMicrophone;
    std::shared_ptr<k4aviewer::K4AMicrophoneListener> g_K4AMicrophoneListener;

protected:
    int GetImageBufferSize(k4a_image_t image);
    int GetImageWidth(k4a_image_t image);
    int GetImageHeight(k4a_image_t image);
    int GetImageStride(k4a_image_t image);
};