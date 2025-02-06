#include "stdafx.h"
#include "K4ACaptureSource.h"
#include "Util.h"

#define SET_AUTO_IF_NEGATIVE(v) ((v < 0) ? K4A_COLOR_CONTROL_MODE_AUTO : K4A_COLOR_CONTROL_MODE_MANUAL)


K4ACaptureSource::K4ACaptureSource() :
	m_camerasStarted(false),
	m_capture(NULL),
	m_colorImage(NULL),
	m_depthImage(NULL),
	m_irImage(NULL),
	m_latestTimestamp(0),
	m_calibFormat(CalibFormat::CalibFormat_CalibStudio)
{
    m_device.deviceHandle = nullptr;
    m_device.initialized = false;
}


K4ACaptureSource::~K4ACaptureSource()
{
    if (m_device.initialized)
    {
        if (m_camerasStarted)
        {
            std::cout << "[K4ACS] Stopping Cameras." << std::endl;
            StopCameras();
        }

        if(g_K4AMicrophone->IsStarted())
        {
            std::cout << "[K4ACS] Stopping microphone." << std::endl;
            g_K4AMicrophone->Stop();
        }

        //std::cout << "[K4ACS] Closing device " << m_device.serial << std::endl;
        k4a_device_close(m_device.deviceHandle);

        delete[] m_device.serial;
    }
}


const char* K4ACaptureSource::GetSerial()
{
    return m_device.serial;
}


bool K4ACaptureSource::IsInitialized()
{
    return m_device.initialized;
}


void SetValue(k4a_device_t deviceHandle, k4a_color_control_command_t colorControl, k4a_color_control_mode_t newMode, int32_t newVal)
{
    int32_t oldVal;
    k4a_color_control_mode_t mode;
    k4a_device_get_color_control(deviceHandle, colorControl, &mode, &oldVal);
    if ((oldVal != newVal) || (mode != newMode))
    {
        k4a_device_set_color_control(deviceHandle, colorControl, newMode, newVal);
    }
}


int K4ACaptureSource::InitDevice(K4ADevice device, bool captureIR)
{
    m_device = device;    
    m_captureIR = captureIR;

    switch (m_device.config.color_resolution)
    {
    case K4A_COLOR_RESOLUTION_720P: /**< 1280 * 720  16:9 */
        m_colorImgDesc.width_pixels = 1280;
        m_colorImgDesc.height_pixels = 720;
        break;
    case K4A_COLOR_RESOLUTION_1080P:/**< 1920 * 1080 16:9 */
        m_colorImgDesc.width_pixels = 1920;
        m_colorImgDesc.height_pixels = 1080;
        break;
    case K4A_COLOR_RESOLUTION_1440P:/**< 2560 * 1440 16:9 */
        m_colorImgDesc.width_pixels = 2560;
        m_colorImgDesc.height_pixels = 1440;
        break;
    case K4A_COLOR_RESOLUTION_1536P:/**< 2048 * 1536 4:3  */
        m_colorImgDesc.width_pixels = 2048;
        m_colorImgDesc.height_pixels = 1536;
        break;
    case K4A_COLOR_RESOLUTION_2160P:/**< 3840 * 2160 16:9 */
        m_colorImgDesc.width_pixels = 3840;
        m_colorImgDesc.height_pixels = 2160;
        break;
    case K4A_COLOR_RESOLUTION_3072P:/**< 4096 * 3072 4:3  */
        m_colorImgDesc.width_pixels = 4096;
        m_colorImgDesc.height_pixels = 3072;
        break;
    case K4A_COLOR_RESOLUTION_OFF: /**< Color camera will be turned off with this setting */
    default:
        break;
    }

    switch (m_device.config.color_format)
    {
    case K4A_IMAGE_FORMAT_COLOR_MJPG:        
        // according to the SDK, each MJPG encoded image in a stream may be of differing size depending on the compression efficiency
        // so we'll leave this as maximum size, but the user should perform checks in code based on color_format.
        m_colorImgDesc.stride_bytes = m_colorImgDesc.width_pixels * sizeof(uint32_t);
        break;
    case K4A_IMAGE_FORMAT_COLOR_BGRA32:
        m_colorImgDesc.stride_bytes = m_colorImgDesc.width_pixels * sizeof(uint32_t);
        break;
    case K4A_IMAGE_FORMAT_COLOR_NV12:
    case K4A_IMAGE_FORMAT_COLOR_YUY2:
    default:
        printf("Color format not implemented... aborting\n");
        throw "not implemented";
        break;
    }

    switch (m_device.config.depth_mode)
    {
    case K4A_DEPTH_MODE_NFOV_2X2BINNED: /**< Depth captured at 320x288. Passive IR is also captured at 320x288. */
        m_depthImgDesc.width_pixels = 320;
        m_depthImgDesc.height_pixels = 288;
        break;
    case K4A_DEPTH_MODE_NFOV_UNBINNED:  /**< Depth captured at 640x576. Passive IR is also captured at 640x576. */
        m_depthImgDesc.width_pixels = 640;
        m_depthImgDesc.height_pixels = 576;
        break;
    case K4A_DEPTH_MODE_WFOV_2X2BINNED: /**< Depth captured at 512x512. Passive IR is also captured at 512x512. */
        m_depthImgDesc.width_pixels = 512;
        m_depthImgDesc.height_pixels = 512;
        break;
    case K4A_DEPTH_MODE_WFOV_UNBINNED:  /**< Depth captured at 1024x1024. Passive IR is also captured at 1024x1024. */
    case K4A_DEPTH_MODE_PASSIVE_IR:     /**< Passive IR only, captured at 1024x1024. */
        m_depthImgDesc.width_pixels = 1024;
        m_depthImgDesc.height_pixels = 1024;
        break;
    case K4A_DEPTH_MODE_OFF:        /**< Depth sensor will be turned off with this setting. */
    default:
        break;
    }

    m_depthImgDesc.stride_bytes = m_depthImgDesc.width_pixels * sizeof(uint16_t);       
    
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, SET_AUTO_IF_NEGATIVE(m_device.colorControls.exposureTimeUS), m_device.colorControls.exposureTimeUS);

    // other color settings
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.brightness);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.gain);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.contrast);    
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_WHITEBALANCE, SET_AUTO_IF_NEGATIVE(m_device.colorControls.colorTemp), m_device.colorControls.colorTemp);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.saturation);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.sharpness);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.backlightCompensation);
    SetValue(m_device.deviceHandle, K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, m_device.colorControls.powerlineFrequency);

    LoadFactoryCalibration();

    return 1;
}


void SetCalibration(const k4a_calibration_camera_t& factoryCal, int width, int height, Calibration& cal)
{
    auto intrParams = factoryCal.intrinsics.parameters.param;

    cal.width = width;
    cal.height = height;

    cal.K[0] = intrParams.fx;
    cal.K[1] = intrParams.fy;
    cal.K[2] = intrParams.cx;
    cal.K[3] = intrParams.cy;

    cal.dist[0] = intrParams.k1;
    cal.dist[1] = intrParams.k2;
    cal.dist[2] = intrParams.p1;
    cal.dist[3] = intrParams.p2;
    cal.dist[4] = intrParams.k3;
    cal.dist[5] = intrParams.k4;
    cal.dist[6] = intrParams.k5;
    cal.dist[7] = intrParams.k6;

    for (int i = 0; i < 9; ++i)
        cal.R[i] = factoryCal.extrinsics.rotation[i];

    for (int i = 0; i < 3; ++i)
        cal.t[i] = factoryCal.extrinsics.translation[i];

    cal.zScale = 1.0;
    cal.zShift = 0.0;
}

void PrintCalibration(Calibration & cal, const char *cal_type) {
    std::cerr << "Printing calibration " << cal_type << std::endl;

    std::cerr << "width " << cal.width << std::endl;
    std::cerr << "height " << cal.height << std::endl;

    std::cerr << "Fx " << cal.K[0] << std::endl;
    std::cerr << "Fy " << cal.K[1] << std::endl;
    std::cerr << "Cx " << cal.K[2] << std::endl;
    std::cerr << "Cy " << cal.K[3] << std::endl;

    std::cerr << "K1 " << cal.dist[0] << std::endl;
    std::cerr << "K2 " << cal.dist[1] << std::endl;
    std::cerr << "P1 " << cal.dist[2] << std::endl;
    std::cerr << "P2 " << cal.dist[3] << std::endl;
    std::cerr << "K3 " << cal.dist[4] << std::endl;
    std::cerr << "K4 " << cal.dist[5] << std::endl;
    std::cerr << "K5 " << cal.dist[6] << std::endl;
    std::cerr << "K6 " << cal.dist[7] << std::endl;

    std::cerr << "Rotation" << std::endl;
    for (int i = 0; i < 9; ++i)
        std::cerr << cal.R[i] << std::endl;

    std::cerr << "Translation" << std::endl;
    for (int i = 0; i < 3; ++i)
        std::cerr << cal.t[i] << std::endl;

    std::cerr << "ZScale " << cal.zScale << std::endl;
    std::cerr << "ZShift " << cal.zShift << std::endl;

}

void K4ACaptureSource::LoadFactoryCalibration()
{    
    k4a_device_get_calibration(m_device.deviceHandle, m_device.config.depth_mode, m_device.config.color_resolution, &m_factoryCalibration);

    auto colorCalib = m_factoryCalibration.color_camera_calibration;
    SetCalibration(colorCalib, m_colorImgDesc.width_pixels, m_colorImgDesc.height_pixels, m_colorCal);

    auto depthCalib = m_factoryCalibration.depth_camera_calibration;
    SetCalibration(depthCalib, m_depthImgDesc.width_pixels, m_depthImgDesc.height_pixels, m_depthCal);
}


bool K4ACaptureSource::LoadCalibrationFromFile(const char* path, uint16_t numKParams)
{
    if(m_calibFormat == CalibFormat_3DTM) {
        if (!K4AToFusionUtils::LoadCamParameters(path, m_device.serial, m_calibFormat, m_colorCal, numKParams, "rgb"))
            return false;

        if (!K4AToFusionUtils::LoadCamParameters(path, m_device.serial, m_calibFormat, m_depthCal, numKParams, "depth"))
            return false;
    } else {
        if (!K4AToFusionUtils::LoadCamParameters(path, m_device.serial, m_calibFormat, m_colorCal, numKParams))
            return false;

        if (!K4AToFusionUtils::LoadCamParameters(path, m_device.serial, m_calibFormat, m_depthCal, numKParams, "_ir"))
            return false;
    }

    return true;
}



void K4ACaptureSource::SaveCalibration(const char* path)
{
    // color
    K4AToFusionUtils::SaveCamParameters(
        path, m_device.id, m_calibFormat,
        m_colorCal,
        "");

    // ir / depth
    K4AToFusionUtils::SaveCamParameters(
        path, m_device.id, m_calibFormat,
        m_depthCal,
        "_ir");
}


int K4ACaptureSource::StartCameras()
{
    if (K4A_FAILED(k4a_device_start_cameras(m_device.deviceHandle, &m_device.config)))
    {
        return 0;
    }

    m_camerasStarted = true;

    if(K4A_FAILED(k4a_device_start_imu(m_device.deviceHandle)))
    {
        std::cout << "ERROR: Could not start the IMU!" << std::endl;
    }
    return 1;
}


void K4ACaptureSource::StopCameras()
{
    if (!m_camerasStarted)
    {
        return;
    }

    k4a_device_stop_cameras(m_device.deviceHandle);
    m_camerasStarted = false;
}


int K4ACaptureSource::CaptureFrame()
{
    if (!m_camerasStarted)
    {
        return 0;
    }

    switch (k4a_device_get_capture(m_device.deviceHandle, &m_capture, 1000))// 200))
    {
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timeout waiting for capture\n");
        return -1;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read capture\n");
        return 0;
    case K4A_WAIT_RESULT_SUCCEEDED:
    default:
        break;
    }    
    
    // get color image from capture
    m_colorImage = k4a_capture_get_color_image(m_capture);
    if (m_colorImage == NULL)
    {
        k4a_capture_release(m_capture);
        return 0;
    }

    // get depth image from capture
    m_depthImage = k4a_capture_get_depth_image(m_capture);
    if (m_depthImage == NULL)
    {
        k4a_capture_release(m_capture);
        return 0;
    }

    if (m_captureIR)
    {
        // get ir image from capture
        m_irImage = k4a_capture_get_ir_image(m_capture);
        if (m_irImage == NULL)
        {
            k4a_capture_release(m_capture);
            return 0;
        }
    }

    // Get the audio data
    // TODO: utilize the actual frameCount value below to allocate only as much memory as we need. 
    // We currently don't do this because fusion is hard-coded to expect MAX_AUDIO_PACKET_BYTES
    // even though the audio playback will use the internal header to know how much of the packet is valid
    m_audioBuffer = (uint8_t*)malloc(MAX_AUDIO_PACKET_SIZE);
    if(!g_K4AMicrophoneListener)
    {
        std::cout << "Tried to get audio, but microphone listener is dead." << std::endl;
        return 0;
    }
    if(g_K4AMicrophoneListener->GetStatus() != SoundIoErrorNone)
    {
        std::stringstream errorBuilder;
        errorBuilder << "Error while recording " << soundio_strerror(g_K4AMicrophoneListener->GetStatus()) << "!";
        g_K4AMicrophoneListener.reset();
    }
    if(g_K4AMicrophoneListener->Overflowed())
    {
        std::cout << "WARNING.  Audio is overflowing.  Consider moving audio process to a separate thread rathern than in K4ACaptureSource::CaptureFrame()" << std::endl;        
        g_K4AMicrophoneListener->ClearOverflowed();
    }
    g_K4AMicrophoneListener->ProcessFrames([this](k4aviewer::K4AMicrophoneFrame* frame, const size_t frameCount) {
        int frameCount_int = (int)frameCount;
        // set the first 4 bytes of the buffer as the size header
        memcpy(m_audioBuffer, &(frameCount_int), sizeof(int));
        for(size_t frameId = 0; frameId < frameCount; frameId++)
        {
            memcpy(&(m_audioBuffer[1+frameId]), &(frame[frameId].Channel[0]), sizeof(short));            
        }
        return frameCount;
    });

    m_latestTimestamp = k4a_image_get_device_timestamp_usec(m_colorImage);

    return 1;
}

double K4ACaptureSource::GetFrameDelayUSec()
{
    switch (m_device.config.camera_fps)
    {
        case K4A_FRAMES_PER_SECOND_5:
            return 200000.0;
        case K4A_FRAMES_PER_SECOND_15:
            return 66666.0;
        case K4A_FRAMES_PER_SECOND_30:
            return 33333.0;
        default:
            throw "invalid frame rate";
    }
}


void K4ACaptureSource::ReleaseFrame()
{
    if (m_colorImage)
    {
        k4a_image_release(m_colorImage);
    }
    if (m_depthImage)
    {
        k4a_image_release(m_depthImage);
    }
    if (m_irImage)
    {
        k4a_image_release(m_irImage);
    }
    k4a_capture_release(m_capture);
    delete m_audioBuffer;
}


const k4a_image_descriptor_t& K4ACaptureSource::GetDepthImgDesc()
{
    return m_depthImgDesc;
}


const k4a_image_descriptor_t& K4ACaptureSource::GetColorImgDesc()
{
    return m_colorImgDesc;
}


const Calibration& K4ACaptureSource::GetColorCalibration()
{
    return m_colorCal;
}


const Calibration& K4ACaptureSource::GetDepthCalibration()
{
    return m_depthCal;
}

const k4a_imu_sample_t K4ACaptureSource::GetIMUReading()
{
    k4a_wait_result_t result = k4a_device_get_imu_sample(m_device.deviceHandle, &m_imuReading, 100);
    if(result != K4A_WAIT_RESULT_SUCCEEDED)
    {
        if(result == K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "TIMEOUT waiting for IMU sample." << std::endl;
        }
        else
        {
            std::cout << "ERROR waiting for IMU sample." << std::endl;
        }
    }
    return m_imuReading;
}


uint64_t K4ACaptureSource::GetLatestTimestamp()
{
    return m_latestTimestamp;
}


uint8_t* K4ACaptureSource::GetColorBuff()
{
    return k4a_image_get_buffer(m_colorImage);
}


uint8_t* K4ACaptureSource::GetDepthBuff()
{
    return k4a_image_get_buffer(m_depthImage);
}


uint8_t* K4ACaptureSource::GetIRBuff()
{
    return k4a_image_get_buffer(m_irImage);
}

uint8_t* K4ACaptureSource::GetAudioBuff()
{
    return m_audioBuffer;
}

int K4ACaptureSource::GetImageBufferSize(k4a_image_t image)
{
    if(image != 0)
    {
        int size = k4a_image_get_size(image);
        return size;
    }
    else
        return -1;
}
int K4ACaptureSource::GetImageWidth(k4a_image_t image)
{
    if(image)
        return k4a_image_get_width_pixels(image);
    else
        return -1;
}
int K4ACaptureSource::GetImageHeight(k4a_image_t image)
{
    if(image)
        return k4a_image_get_height_pixels(image);
    else
        return -1;
}
int K4ACaptureSource::GetImageStride(k4a_image_t image)
{
    if(image)
        return k4a_image_get_stride_bytes(image);
    else 
        return -1;
}
int K4ACaptureSource::GetDepthImageBufferSize()
{
    return GetImageBufferSize(m_depthImage);
}
int K4ACaptureSource::GetColorWidth()
{
    return GetImageWidth(m_colorImage);
}
int K4ACaptureSource::GetColorHeight()
{
    return GetImageHeight(m_colorImage);
}
int K4ACaptureSource::GetDepthWidth()
{
    return GetImageWidth(m_depthImage);
}
int K4ACaptureSource::GetDepthHeight()
{
    return GetImageHeight(m_depthImage);
}
int K4ACaptureSource::GetColorImageBufferSize()
{
    return GetImageBufferSize(m_colorImage);
}