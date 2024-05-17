#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

const int MaxKinectFramesPerSecond = 30;

enum StartMode
{
    START_MODE_AUTO = 0,
    START_MODE_MANUAL_EACH,
    START_MODE_MANUAL_MASTER,
	START_MODE_CONTROL_PANEL_EACH,
	START_MODE_CONTROL_PANEL_MASTER
};

enum CalibFormat
{
	CalibFormat_Face3D = 0,
	CalibFormat_CalibStudio,
	CalibFormat_3DTM
};

struct Calibration
{
    double       R[9];
    double       t[3];
    double       K[4];
    double       dist[8];
    int          width;
    int          height;
    double       zScale;
    double       zShift;
	double		 colorScale[3];
	double 		 colorBias[3];
	int			 numKParams;
};


struct SourceInfo
{
    uint32_t     srcWidth;
    uint32_t     srcHeight;
    uint32_t     outWidth;
    uint32_t     outHeight;
    uint32_t     leftOffset;
    uint32_t     topOffset;
    uint32_t     outSize;
    Calibration  cal;
    cv::Mat      mapX;
    cv::Mat      mapY;
};


struct SharedSettings
{
	uint8_t		verbosity;
    uint32_t     minIRRange;
    uint32_t     maxIRRange;
    uint32_t     minDepth;
    uint32_t     maxDepth;
    uint32_t     portOffset;
    uint32_t     fusionWidth;
    uint32_t     fusionHeight;
    int32_t      skipFirstNFrames;
    int32_t      startDelayMS;
    StartMode    startMode;
    std::string  recordingPath;
    std::string  calibPath;
    uint16_t     numKParams;
    bool         applyMask;
    double       numStdDevFromBG;
	bool		 outputForMultiViewCalib;
	std::string  calibPublisherIPAddress;
	std::string  calibPublisherPort;
	std::string  controlPanelIPAddress;
	std::string  statusTCPPort;
	std::string	 eventTCPPort;
	int			 framesToGrabEachBurst;
	int			 frameRecordingFrequency;
	CalibFormat  calibFormat;
	uint32_t	 maxSavedCalibrationFrames;
	uint32_t 	 minBackgroundSamples;
	uint32_t 	 maxBufferSize;
	// K4ASettings 
	uint16_t	interpolationEdgeThreshold;
	uint16_t	temporalSmoothingThreshold;
	uint16_t	depthErosionMaxZeros;
	uint16_t 	depthErosionSearchRadius;
	uint8_t		temporalErosionFrames;
	int16_t		 brightness;
	uint8_t		 contrast;
	uint8_t		saturation;
	uint8_t		sharpness;
	int16_t		 whitebalance;
	bool		backlightCompensation;
	int16_t		 gain;
	int			 exposureTimeUS;
	short		powerlineFrequency;
	int16_t		 syncMode;
	int16_t		 syncDelay;
	bool		 statisticsMode;
};

enum CaptureMode
{
	CaptureMode_Stream = 0,
	CaptureMode_Calibration,
	CaptureMode_BG,
	CaptureMode_Offline,
	CaptureMode_Config,
	CaptureMode_WriteFactoryCalib,
	CaptureMode_Idle = 10	
};

struct K4AFrame
{
	std::shared_ptr<uint8_t[]> depthBuffer;
	std::shared_ptr<uint8_t[]> colorBuffer;
	std::shared_ptr<uint8_t[]> audioBuffer;
	std::shared_ptr<uint8_t[]> irBuffer;
	std::shared_ptr<uint8_t[]> transformedColorBuffer;
	std::shared_ptr<uint8_t[]> paddedTransformColorBuffer;
	int _colorBufferSize;
	int _depthBufferSize;
	int _irBufferSize;
	int _transformedColorBufferSize;
	int _paddedTransformColorBufferSize;
	int _audioBufferSize;
	uint64_t timestampUSec;
	uint32_t framesPassed;

	K4AFrame() : depthBuffer(nullptr), 
		colorBuffer(nullptr), 
		audioBuffer(nullptr),
		irBuffer(nullptr), 
		transformedColorBuffer(nullptr), 
		paddedTransformColorBuffer(nullptr), 
		_colorBufferSize(0),
		_depthBufferSize(0),
		_irBufferSize(0),
		_transformedColorBufferSize(0),
		_paddedTransformColorBufferSize(0),
		_audioBufferSize(0),
		timestampUSec(0), 
		framesPassed(0)
	{
	}

	K4AFrame(int depthBufferSize, int colorBufferSize, int irBufferSize, int transformedColorBufferSize = 0, int audioBufferSize = 0) :
		_colorBufferSize(colorBufferSize),
		_depthBufferSize(depthBufferSize),
		_irBufferSize(irBufferSize),
		_transformedColorBufferSize(transformedColorBufferSize),
		_paddedTransformColorBufferSize(colorBufferSize),
		_audioBufferSize(audioBufferSize),
		timestampUSec(0), 
		framesPassed(0)
	{
		if (depthBufferSize > 0)
		{
			depthBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[depthBufferSize]);
		}
		if (colorBufferSize > 0)
		{
			colorBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[colorBufferSize]);
		}
		if (irBufferSize > 0)
		{
			irBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[irBufferSize]);
		}
		if (transformedColorBufferSize > 0)
		{
			transformedColorBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[transformedColorBufferSize]);
			memset(transformedColorBuffer.get(), 0, transformedColorBufferSize);
			paddedTransformColorBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[colorBufferSize]);
			memset(paddedTransformColorBuffer.get(), 0, colorBufferSize);
		}
		if(audioBufferSize > 0)
		{
			audioBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[audioBufferSize]);
		}
	}
};

