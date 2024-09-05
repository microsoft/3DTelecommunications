#include "stdafx.h"
#include "../DepthPodToFusionARM/DepthPodToFusionAPI.h"
#include "K4ACaptureSource.h"
#include "K4ASourceManager.h"
#include "K4ADepthProcessing.h"
#include "Util.h"
#include "SourceStats.h"
#include "zmq.hpp"
#include <iostream>
#include <chrono>
#include <ctime>
#include "StreamsDef.h"
#include "K4AControlPanelConnector.h"
#include "LibUtility/include/Config.h"
#include <unistd.h>
#include "3dtm_version.h"
#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include <fcntl.h>
#include <json/json.h>
#include "LEDSerialWriter.hpp"

using namespace K4AToFusionUtils;

#define BUFFER_STATUS_FREQ        75
#define SHOW_IMAGE_RATE           2

//used for tiling the windows nicely
#define DEFAULT_SCREEN_WIDTH  1080  
#define DEFAULT_SCREEN_HEIGHT 1920
#define BGCAPTURE_PROGRESS_UPDATE_FREQUENCY 10
#define PROCESS_FRAME_CONTROL_PANEL_UPDATE_FREQUENCY 50
CaptureMode g_captureMode = CaptureMode_Stream;

#define IS_CALIB_CAPTURE()   (g_captureMode == CaptureMode_Calibration)
#define IS_BG_CAPTURE()      (g_captureMode == CaptureMode_BG)
#define IS_STREAM_CAPTURE()  (g_captureMode == CaptureMode_Stream)
#define IS_OFFLINE_CAPTURE() (g_captureMode == CaptureMode_Offline)
#define IS_CONFIG_CAPTURE()  (g_captureMode == CaptureMode_Config)
#define IS_WRITING_FACTORY_CALIB_CAPTURE() (g_captureMode == CaptureMode_WriteFactoryCalib)

#define NO_MASK(v)  (v & ~0x8000)
#define GET_MASK(v) (v & 0x8000)

const std::string myIP("*");
std::string logfile("/var/log/K4AToFusion/"); // file name will get set with +argv[0]+".log");
std::string errorfile("/var/log/K4AToFusion/");
std::string serialPortFilename("/dev/ttyACM0");

K4ASourceManager sourceManager;
bool g_IsRunning = false;
bool g_writeToDisk = false;
bool g_captureIR = false;
bool g_writeDepthToBinFiles = false;
bool g_showImages = false;
bool g_processDepth = true;
bool g_undistortDepth = true;
bool g_processMask = true;
bool g_profile = false;

//calib related
bool g_calib_transmit_activeIR = false;
bool g_use_factory_calib = false;
int undistortBorderPixels;

std::vector<bool> g_SaveNextCalibFrame;
std::mutex saveNexCalibFrameMutex;

std::atomic<unsigned int> g_StreamReadyCnt;
std::string               g_recordingName = "";

//related to syncing between streams
const int cFramesToKeep = 5;
std::mutex lastXFramesMutex;
std::vector<std::deque<std::shared_ptr<K4AFrame>>> lastXFrames;

std::deque<K4AFrame*> g_calibrationSavedFrames;
std::mutex calibrationSavedFramesMutex;

std::mutex zmqMutex;
uint64_t timestampForNextCapture = 0;
bool gShouldPublishTimeStamp = false;

static K4AControlPanelConnector* K4ACPC = nullptr;

uint g_verbosity;
//bool g_CalibDetectCheckerboard = true;
int g_CurrentRequestedFPS = -1;
int g_ConfigFileRequestedFPS = 15;
int g_FramePerSecondCounter = 1;

struct K4AFusionStream
{
	int serverId;
	int portOffset;
	K4ACaptureSource* source;
	std::thread* transferThread;
	std::thread* captureThread;
	std::mutex* bufferMtx;
    std::queue<std::shared_ptr<K4AFrame>> frameBuffer;
};

K4AFusionStream* BuildFusionStreamsFromSources(K4ASourceManager& sourceManager)
{
	K4AFusionStream* streams = new K4AFusionStream[sourceManager.GetNumSources()];
	K4ACaptureSource* sources = sourceManager.GetCaptureSources();
	const SharedSettings& settings = sourceManager.GetSharedSettings();
	for (uint32_t i = 0; i < sourceManager.GetNumSources(); ++i)
	{
		streams[i].portOffset = settings.portOffset + i;
		streams[i].serverId = -1; // will be set when fusion server is started for this source
		streams[i].source = &sources[i];
		streams[i].transferThread = nullptr;
		streams[i].captureThread = nullptr;
	}

	return streams;
}

const char* GetCaptureModeString(CaptureMode captureMode)
{
    switch (captureMode)
    {
    case CaptureMode_Stream:
        return "STREAM";
    case CaptureMode_Calibration:
        return "CALIB";
    case CaptureMode_BG:
        return "BG_CAP";
    case CaptureMode_Offline:
        return "OFFLINE_CAP";
    case CaptureMode_Config:
        return "CONFIG";
    case CaptureMode_WriteFactoryCalib:
        return "WRITE_FACTORY_CALIB";
    default:
        return "UNDEFINED";
    }
}

void InitCVUndistortMaps(cv::Mat& camMat, cv::Mat& distCoeff, SourceInfo& srcInf, int numKParams)
{
    camMat = cv::Mat::eye(3, 3, CV_64F);
    camMat.at<double>(0, 0) = srcInf.cal.K[0];
    camMat.at<double>(0, 1) = 0.0;
    camMat.at<double>(0, 2) = srcInf.cal.K[2];
    camMat.at<double>(1, 0) = 0.0;
    camMat.at<double>(1, 1) = srcInf.cal.K[1];
    camMat.at<double>(1, 2) = srcInf.cal.K[3];
    camMat.at<double>(2, 0) = 0.0;
    camMat.at<double>(2, 1) = 0.0;
    camMat.at<double>(2, 2) = 1.0;

    int numParams = numKParams + 2;
    distCoeff = cv::Mat::zeros(numParams, 1, CV_64F);
    for (int i = 0; i < numParams; ++i)
    {
        distCoeff.at<double>(i) = srcInf.cal.dist[i];
    }

    static cv::Mat i3 = cv::Mat::eye(3, 3, CV_32FC1);    
    cv::initUndistortRectifyMap(camMat, distCoeff, i3, camMat, cv::Size(srcInf.outWidth, srcInf.outHeight), CV_32FC1, srcInf.mapX, srcInf.mapY);
}
/**
 * ProcessBackgroundFrame processes captured frames as background images to be used in background segmentation
 * @param  {K4AFusionStream*} stream     :  the stream we are pulling images from (used to identify the server/camera)
 * @param  {SharedSettings} shrdSettings :  holds the configuration values such as minBackgroundSamples and recordingPath
 * @param  {SourceStats} srcStats        :  class used to compute the background mask
 * @param  {uint16_t*} outBuffer         :  buffer used to hold the accumulated background mast
 * @return {bool}                        :  True if we have captured minBackgroundSamples (and we can stop gathering more), False if more must be captured
 */
bool ProcessBackgroundFrame(K4AFusionStream* stream, SharedSettings& shrdSettings, SourceStats& srcStats, uint16_t* outBuffer)
{

    // still frames to add?
    if (srcStats.TotalFrames() < shrdSettings.minBackgroundSamples)
    {
        std::cout << stream->serverId << "] Accumulating BG frames " << srcStats.TotalFrames() + 1 << "/" << shrdSettings.minBackgroundSamples << std::endl;

        // accumulate frame into buffer per pixel
        srcStats.AddFrame<uint16_t>(outBuffer);
        return false;
    }
    // otherwise we're done (not putting else so we can fall out on last frame rather than capturing one additional)
    else  // (srcStats.TotalFrames() >= shrdSettings.minBackgroundSamples)
    {
        std::cout << stream->serverId << "] Computing bg stats..." << std::endl;

        // compute statistics
        srcStats.ComputeStats();

        // save bg + variance to disk
        if (!srcStats.SaveToDisk(shrdSettings.recordingPath.c_str(), stream->source->GetSerial()))
        {
            printf("WARNING: Failed to save BG stats for camera: %d\n", stream->serverId);
        }

        // quit!
        std::cout << stream->serverId << "] BG Complete!" << std::endl;
        return true;
    }
}


//used for the master server
void ZMQPublisherThread(std::string zmqTCPPort)
{
	std::unique_ptr<zmq::context_t > publisherContext(new zmq::context_t(1));
	std::unique_ptr<zmq::socket_t> publisher(new zmq::socket_t(*publisherContext, ZMQ_PUB));

	std::string publisherTCPPort = "tcp://*:" + zmqTCPPort;
	publisher->bind(publisherTCPPort);

	while (g_IsRunning)
	{
		if (gShouldPublishTimeStamp)
		{
			//just send the frame number
			zmq::message_t message = zmq::message_t(&timestampForNextCapture, sizeof(timestampForNextCapture));
            if(g_verbosity > 0)
			    std::cout << "Send CAPTURE Frame packet: " << timestampForNextCapture << std::endl;
			zmqMutex.lock();
			publisher->send(message, zmq::send_flags::none);
			zmqMutex.unlock();
			gShouldPublishTimeStamp = false;
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

//used for children who receive trigger to figure out which frame to save
void ZMQSubscriberThread(std::string publisherIP, std::string zmqTCPPort)
{
	std::unique_ptr<zmq::context_t > subscriberContext(new zmq::context_t(1));
	std::unique_ptr<zmq::socket_t> subscriber(new zmq::socket_t(*subscriberContext, ZMQ_SUB));

	std::string subscriberTCPPort = "tcp://" + publisherIP + ":" + zmqTCPPort;
	subscriber->set(zmq::sockopt::subscribe, ""); // Subscribe to all events
	subscriber->set(zmq::sockopt::rcvtimeo, ZMQ_REQUEST_TIMEOUT);
	subscriber->set(zmq::sockopt::tcp_keepalive, 1);
	subscriber->set(zmq::sockopt::tcp_keepalive_cnt, 2);
	subscriber->set(zmq::sockopt::tcp_keepalive_idle, 10);
	subscriber->set(zmq::sockopt::tcp_keepalive_intvl, 10);
	subscriber->set(zmq::sockopt::rcvbuf, 1000000);
	subscriber->set(zmq::sockopt::sndbuf, 1000000);
	subscriber->set(zmq::sockopt::sndhwm, 10000);
	subscriber->set(zmq::sockopt::rcvhwm, 10000);
	subscriber->connect(subscriberTCPPort);

	while (g_IsRunning)
	{
		zmq::message_t receivedMessage;
		zmqMutex.lock();
		auto res = subscriber->recv(receivedMessage, zmq::recv_flags::dontwait);
		zmqMutex.unlock();
        if(!res)
        {
            if(zmq_errno() == EAGAIN)
            {
                // no error, just no data
                continue;
            }
            else
            {
                std::cout << "Error receiving an available packet" << std::endl;
            }
        }
		else
        {
            std::memcpy(&timestampForNextCapture, receivedMessage.data(), sizeof(uint64_t));
            if(g_verbosity > 0)
                std::cout << "Received CAPTURE Frame packet: " << timestampForNextCapture << std::endl;
            std::lock_guard<std::mutex> guard(saveNexCalibFrameMutex);
            for (std::vector<bool>::iterator it = g_SaveNextCalibFrame.begin(); it != g_SaveNextCalibFrame.end(); ++it)
            {
                *it = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}
void DumpIMUReadingToDisk(k4a_imu_sample_t imuSample, const char* filePath)
{
    Json::Value root;

    root["imu_data"]["temperature"] = imuSample.temperature;
    root["imu_data"]["accelerometer"]["x"] = imuSample.acc_sample.xyz.x;
    root["imu_data"]["accelerometer"]["y"] = imuSample.acc_sample.xyz.y;
    root["imu_data"]["accelerometer"]["z"] = imuSample.acc_sample.xyz.z;
    root["imu_data"]["accelerometer"]["timestamp"] = std::to_string(imuSample.acc_timestamp_usec); //jsoncpp 1.7 (latest available on Jetson) doesn't handle Uint64, so cast to a string since we're just saving to file anyway
    root["imu_data"]["gyroscope"]["x"] = imuSample.gyro_sample.xyz.x;
    root["imu_data"]["gyroscope"]["y"] = imuSample.gyro_sample.xyz.y;
    root["imu_data"]["gyroscope"]["z"] = imuSample.gyro_sample.xyz.z;
    root["imu_data"]["gyroscope"]["timestamp"] = std::to_string(imuSample.gyro_timestamp_usec);

    Json::StreamWriterBuilder wbuilder;
    wbuilder["indentation"] = "\t";
    std::string document = Json::writeString(wbuilder, root);
    std::ofstream ofs(filePath, std::ios::trunc);
    ofs << std::fixed;
    ofs.precision(6);
    ofs << document << std::endl;
}
void ProcessFramesFn(K4AFusionStream* stream, SharedSettings shrdSettings)
{    
    SourceInfo colorInfo;
    Calibration colorCal = stream->source->GetColorCalibration();
    int colorWidth       = stream->source->GetColorImgDesc().width_pixels;
    int colorHeight      = stream->source->GetColorImgDesc().height_pixels;

    colorInfo.outWidth = colorWidth;
    colorInfo.outHeight = colorHeight;
    colorInfo.cal = colorCal;


    SourceInfo depthInfo;
    depthInfo.cal       = stream->source->GetDepthCalibration();
    depthInfo.srcWidth  = stream->source->GetDepthImgDesc().width_pixels;
    depthInfo.srcHeight = stream->source->GetDepthImgDesc().height_pixels;

    //TODO FIX FUSION TO WORK WITH OTHER DIMENSIONS!!!! After fix, this should be depthDesc.width_pixels * depthDesc.height_pixels    
    depthInfo.outWidth  = shrdSettings.fusionWidth;
    depthInfo.outHeight = shrdSettings.fusionHeight;
    if (!g_processDepth && !IS_CALIB_CAPTURE())
    {
        depthInfo.outWidth  = depthInfo.srcWidth;
        depthInfo.outHeight = depthInfo.srcHeight;
    }

    depthInfo.outSize = depthInfo.outWidth * depthInfo.outHeight;
    // The 3DTM Format already computes the intrinsics updated to the target
    // output depth dimension. We should *not* redo the work here. 
    // Before creating the 3DTM Format, there was very likely a bug in the code
    // since updating calibration to the target dimension should only really
    // be necessary when loading calibration from factory parameters. I'm not
    // entirely sure about this affirmation, given that using intrinsics computed
    // by MultiViewCalib seemed to be broken. Thiago
    if(shrdSettings.calibFormat != CalibFormat_3DTM) {
        K4AToFusionUtils::UpdateCalibrationWTargetDim(depthInfo.cal, depthInfo.outWidth, depthInfo.outHeight);
    }
    depthInfo.leftOffset = (depthInfo.outWidth - depthInfo.srcWidth) / 2;
    depthInfo.topOffset  = (depthInfo.outHeight - depthInfo.srcHeight) / 2;

    uint16_t* processedDepthBuffer = new uint16_t[depthInfo.outSize];
    memset(processedDepthBuffer, 0, sizeof(uint16_t) * depthInfo.outSize);

    uint16_t* temporalAvgBuf = new uint16_t[depthInfo.outSize];
    memset(temporalAvgBuf, 0, sizeof(uint16_t)*depthInfo.outSize);

    uint16_t* erodedDepthBuffer = new uint16_t[depthInfo.outSize];
    memset(erodedDepthBuffer, 0, sizeof(uint16_t)*depthInfo.outSize);
    
    cv::Mat camMat, distMat;
    InitCVUndistortMaps(camMat, distMat, depthInfo, shrdSettings.numKParams);

    if (g_use_factory_calib)
    {
        cv::Mat colorCamMat, colorDistMat;
        InitCVUndistortMaps(colorCamMat, colorDistMat, colorInfo, shrdSettings.numKParams);
    }

    int fNum = 0;
	int framesWritten = 0;
    int framesToGrab = 0;

    // prepare recording folders for camera
    char outPath[256];
    sprintf(outPath, "%s", shrdSettings.recordingPath.c_str());

    // load background stats (per-pixel average/std deviation)
    SourceStats srcStats;
    srcStats.Initialize(depthInfo.srcWidth, depthInfo.srcHeight);
    bool hasBG = false;
    if (!IS_BG_CAPTURE())
    {
        hasBG = srcStats.LoadFromDisk(shrdSettings.recordingPath.c_str(), stream->source->GetSerial());
    }
    
    double* avgBufUndistorted = new double[depthInfo.outSize];
    double* stdDevUndistorted = new double[depthInfo.outSize];
    double* avgBufPadded = new double[depthInfo.outSize];
    double* stdDevPadded = new double[depthInfo.outSize];

    memset(avgBufUndistorted, 0, sizeof(double) * depthInfo.outSize);
    memset(stdDevUndistorted, 0, sizeof(double) * depthInfo.outSize);
    memset(avgBufPadded, 0, sizeof(double) * depthInfo.outSize);
    memset(stdDevPadded, 0, sizeof(double) * depthInfo.outSize);
    
    if (hasBG && !IS_CALIB_CAPTURE())
    {
        K4ADepthProcess::UndistortBuffer<double>(depthInfo, srcStats.GetAvgBuf(), avgBufUndistorted);
        K4ADepthProcess::UndistortBuffer<double>(depthInfo, srcStats.GetStdBuf(), stdDevUndistorted);

        K4ADepthProcess::PadBuffer<double>(depthInfo, srcStats.GetAvgBuf(), avgBufPadded);
        K4ADepthProcess::PadBuffer<double>(depthInfo, srcStats.GetStdBuf(), stdDevPadded);
    }

    if(g_verbosity > 0)
    {
        printf("[ProcessFramesFn Startup] BG Present? %s\n", hasBG ? "YES" : "NO");
        // begin processing loop
        printf("[ProcessFramesFn Startup] Waiting for frames...\n");
    }

    uint64_t processCnt = 0;
    bool prevUndistort = g_undistortDepth;
    bool framesRemaining = false;
    auto start = std::chrono::high_resolution_clock::now();
    auto profStart = start;
    unsigned int historyMask = 0;
    std::shared_ptr<uint8_t[]> foregroundHistory = std::shared_ptr<uint8_t[]>(new uint8_t[depthInfo.outSize]);
    std::vector<double> foregroundStatistics(16, 0.0);
    //double runningForegroundAverage = 0.0;
    //double runningFluctuationAverage = 0.0;
    
    if(shrdSettings.temporalErosionFrames > 0 || shrdSettings.statisticsMode)
    {
        historyMask = (1 << shrdSettings.temporalErosionFrames) - 1;
        memset(foregroundHistory.get(), 0, sizeof(uint16_t) * depthInfo.outSize);
    }

    int frameCounter = 0;
    while (g_IsRunning || (g_writeToDisk && framesRemaining))
    {                    
        // pull capture from buffer
        size_t bufSize = 0;
        std::shared_ptr<K4AFrame> curFrame;
        stream->bufferMtx->lock();
        {
            bufSize = stream->frameBuffer.size();
            if (bufSize == 0)
            {
                if(g_verbosity > 1)
                {
                    std::cout << "[P] -\r"; //don't want to endl, this could print a LOT
                }
                stream->bufferMtx->unlock();
                std::this_thread::yield();
                framesRemaining = false;
                continue;
            }
            curFrame = stream->frameBuffer.front();
            stream->frameBuffer.pop();
            framesRemaining = true;
        }
        stream->bufferMtx->unlock();
        auto profEnd = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds;
        if(g_profile)
        {
            elapsed_seconds = profEnd - profStart;
            std::cout << "[PROC][Get frame from buffer][" << elapsed_seconds.count() << "]" << std::endl;
        }

        // general debug stats
        if (processCnt++ % BUFFER_STATUS_FREQ == 0)
        {
            printf("  %d] Buffer size: %lu\r\n", (int)stream->serverId, bufSize);
        }
        if (bufSize > shrdSettings.maxBufferSize)
        {
            printf("  %d] WARNING: PROCESSING SLOWER THAN CAPTURE!\r\n", stream->serverId);
        }

        if(g_verbosity > 1)
        {
            std::cout << "[PROC] Grabbing buffers from curFrame" << std::endl;
        }
        std::shared_ptr<uint8_t[]> depthBuffer = curFrame->depthBuffer;
        std::shared_ptr<uint8_t[]> colorBuffer = curFrame->colorBuffer;
        std::shared_ptr<uint8_t[]> audioBuffer = curFrame->audioBuffer;
        std::shared_ptr<uint8_t[]> irBuffer = curFrame->irBuffer;
        uint64_t timestampUSec = curFrame->timestampUSec;
        uint32_t framesPassed = curFrame->framesPassed;
        fNum += framesPassed;
        if (framesPassed > 1)
            printf("%d] %d %lu (skipped: %d)\n", stream->serverId, fNum, timestampUSec, framesPassed - 1);

        uint16_t* outBuffer = reinterpret_cast<uint16_t*>(depthBuffer.get());

        if (IS_BG_CAPTURE())
        {
            if(g_profile)
                profStart = std::chrono::high_resolution_clock::now();

            // accumulate BG frames
            if (ProcessBackgroundFrame(stream, shrdSettings, srcStats, outBuffer))
            {
                K4ACPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_DONE);
                // We're done, don't keep re-processing BG, so go to Idle state
                g_captureMode = CaptureMode_Idle;
            }
            else
            {
                int data = srcStats.TotalFrames();
                if(data%BGCAPTURE_PROGRESS_UPDATE_FREQUENCY == 0)
                {
                    K4ACPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_PROGRESS, &data, sizeof(int));
                    Trinket::SetBackgroundCaptureNumber(data%20);
                }
            }

            if(g_profile)
            {
                profEnd = std::chrono::high_resolution_clock::now();
                elapsed_seconds = profEnd - profStart;
                std::cout << "[PROC][ProcessBackgroundFrame][" << elapsed_seconds.count() << "]" << std::endl;
            }
        }

        if (g_processDepth)
        {            
            if (g_undistortDepth)
            {
                if (g_profile)
                    profStart = std::chrono::high_resolution_clock::now();
                if (g_processMask)
                {                    
                    K4ADepthProcess::UndistortDepthMap<uint16_t>(depthInfo, outBuffer, avgBufUndistorted, stdDevUndistorted, temporalAvgBuf, shrdSettings, processedDepthBuffer);
                    if(g_profile)
                    {
                        profEnd = std::chrono::high_resolution_clock::now();
                        elapsed_seconds = profEnd - profStart;
                        std::cout << "[PROC][UndistortDepthMap][" << elapsed_seconds.count() << "]" << std::endl;
                        profStart = profEnd;
                    }
                    if(shrdSettings.depthErosionMaxZeros > 0 || shrdSettings.statisticsMode)
                    {
		    	        memcpy(erodedDepthBuffer, processedDepthBuffer, depthInfo.outSize*sizeof(uint16_t));
                        K4ADepthProcess::ErodeDepthMap<uint16_t>(depthInfo, processedDepthBuffer, foregroundHistory, foregroundStatistics, historyMask, shrdSettings, erodedDepthBuffer);                        
                        if(g_profile)
                        {
                            profEnd = std::chrono::high_resolution_clock::now();
                            elapsed_seconds = profEnd - profStart;
                            std::cout << "[PROC][ErodeDepthMap][" << elapsed_seconds.count() << "]" << std::endl;
                            profStart = profEnd;
                        }
                    }
		            if(shrdSettings.interpolationEdgeThreshold > 0 || shrdSettings.temporalSmoothingThreshold > 0)
                    {
                        if(shrdSettings.depthErosionMaxZeros > 0)
                            memcpy(temporalAvgBuf, erodedDepthBuffer, depthInfo.outSize*sizeof(uint16_t));
                        else
                            memcpy(temporalAvgBuf, processedDepthBuffer, depthInfo.outSize*sizeof(uint16_t));
                    }
                }
                else
                {
                    K4ADepthProcess::UndistortBuffer<uint16_t>(depthInfo, outBuffer, processedDepthBuffer);
                }
                prevUndistort = true;
                if(g_profile)
                {
                    profEnd = std::chrono::high_resolution_clock::now();
                    elapsed_seconds = profEnd - profStart;
                    std::cout << "[PROC][UndistortDepth][" << elapsed_seconds.count() << "]" << std::endl;
                }
            }
            else
            {
                if (g_profile)
                    profStart = std::chrono::high_resolution_clock::now();
                if (prevUndistort)
                {
                    prevUndistort = false;
                    memset(processedDepthBuffer, 0, sizeof(uint16_t) * depthInfo.outSize);
                }
                if (g_processMask)
                {
                    K4ADepthProcess::PadDepthMap<uint16_t>(depthInfo, outBuffer, avgBufPadded, stdDevPadded, shrdSettings, processedDepthBuffer);
                }
                else
                {
                    K4ADepthProcess::PadBuffer<uint16_t>(depthInfo, outBuffer, processedDepthBuffer);
                }
                if (g_profile)
                {
                    profEnd = std::chrono::high_resolution_clock::now();
                    elapsed_seconds = profEnd - profStart;
                    std::cout << "[PROC][PadBuffer][" << elapsed_seconds.count() << "]" << std::endl;
                }
            }

            if(shrdSettings.depthErosionMaxZeros > 0)
                outBuffer = erodedDepthBuffer;
            else
                outBuffer = processedDepthBuffer;
        }
   //     if(IS_CALIB_CAPTURE() && g_CalibDetectCheckerboard)
   //     {
   //         if(g_verbosity > 2)
   //         {
   //             std::cout << "Detecting checkerboard in image...";
   //         }
   //         // get a grayscale image
   //         // in calibration mode I've forced my output to be BRGA, not MJPEG
   //         if (g_profile)
   //             profStart = std::chrono::high_resolution_clock::now();
   //         cv::Mat colorImage(colorInfo.outHeight, colorInfo.outWidth, CV_8UC4, colorBuffer.get());            
   //         cv::Mat grayImage;
   //         cv::cuda::GpuMat colorImageGPU;
   //         cv::cuda::GpuMat grayImageGPU(colorImage.size(), CV_8UC1, cv::Scalar(0));
   //         colorImageGPU.upload(colorImage);
   //         cv::cuda::cvtColor(colorImageGPU, grayImageGPU, cv::COLOR_BGRA2GRAY);
   //         grayImageGPU.download(grayImage);
   //         if (g_profile)
   //         {
   //             profEnd = std::chrono::high_resolution_clock::now();
   //             elapsed_seconds = profEnd - profStart;
   //             std::cout << "[PROC][CALIB-CUDAConvert][" << elapsed_seconds.count() << "]" << std::endl;
   //             profStart = std::chrono::high_resolution_clock::now();
   //         }
   //         // Run a checkerboard detector and send packets indicating how many corners I've detected in this image
   //         bool success = checkerboardDetector.Detect(&grayImage, minCheckerboardSize, maxCheckerboardSize, undistortBorderPixels);
   //         if (g_profile)
   //         {
   //             profEnd = std::chrono::high_resolution_clock::now();
   //             elapsed_seconds = profEnd - profStart;
   //             std::cout << "[PROC][CALIB-Checkerboard Detect][" << elapsed_seconds.count() << "]" << std::endl;
   //         }
   //         CheckerBoardPose pose;
   //         checkerboardDetector.GetCurrCBPose(&pose);
			//int num = pose.GetNumFound();
   //         if(success && num > maxPointsToDetectInACheckerboard/2) // we'll send this packet if we see parts of the checkerboard too.  That way we can indicate that the image is _almost_ good enough
   //         {
   //             if(g_verbosity > 1)
   //             {
   //                 std::cout << "I can see a valid checkerboard!" << std::endl;
   //             }
   //             K4ACPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_I_SEE_CHECKERBOARD, &num, sizeof(num));
   //             // send the number of corners, because this could be an "I see _some_ corners situation, and we need to tell the tech to move slightly"
   //         }
   //         else
   //         {
   //             if(g_verbosity > 2)
   //             {
   //                 std::cout << "I can NOT see a valid checkerboard.  Pts: " << num << std::endl;
   //             }
   //         }
   //     }

        // don't preprocess calibration or offline frames
        if (IS_STREAM_CAPTURE())
        {
            if(g_profile)
                profStart = std::chrono::high_resolution_clock::now();
            // get the actual color frame sizes (we only have one in K4AToFusion)
            std::vector<int> colorFrameSizes(1, curFrame->_colorBufferSize);            

            // send aggregated results
            if(g_verbosity > 1)
                std::cout << "[PROC] SendFrame" << std::endl;
            HRESULT hr = DepthToFusion_SendFrame(
                stream->serverId,
                (uchar**)&outBuffer,
                (uchar**)&colorBuffer,
                (uchar**)&audioBuffer,
                colorFrameSizes,
                fNum,
                timestampUSec
                );

            if (FAILED(hr))
            {
                printf("Failed to send frame...\n");
            }
            if (g_profile)
            {
                profEnd = std::chrono::high_resolution_clock::now();
                elapsed_seconds = profEnd - profStart;
                std::cout << "[PROC][XMIT Frame][" << elapsed_seconds.count() << "]" << std::endl;
            }
        }


        if (g_showImages && (processCnt % SHOW_IMAGE_RATE == 0))
        {
            if (g_profile)
                profStart = std::chrono::high_resolution_clock::now();
            char winName[20];
            const int windowWidth = 600;
            const int offsetX = DEFAULT_SCREEN_WIDTH - (stream->serverId + 1) * windowWidth - 20;
            const int offsetY = 20;
            const int yStep = DEFAULT_SCREEN_HEIGHT / 4;
            sprintf(winName, "Color %d", stream->serverId);
            K4AToFusionUtils::ShowImage(winName, colorHeight, colorWidth, CV_8UC4, colorBuffer.get(), windowWidth, offsetX, offsetY + (int)(yStep * 0));
            sprintf(winName, "IR %d", stream->serverId);
            K4AToFusionUtils::ShowImage(winName, depthInfo.srcHeight, depthInfo.srcWidth, CV_16UC1, irBuffer.get(), windowWidth, offsetX, offsetY + (int)(yStep * 1));
            sprintf(winName, "Depth %d", stream->serverId);
            K4AToFusionUtils::ShowImage(winName, depthInfo.outHeight, depthInfo.outWidth, CV_16UC1, (uchar*)outBuffer, windowWidth, offsetX, offsetY + (int)(yStep * 2), 20.0);
            //K4AToFusionUtils::ShowImageColorScale(winName, depthInfo.outHeight, depthInfo.outWidth, CV_16UC1, (uchar*)outBuffer, windowWidth, offsetX, offsetY + (int)(yStep * 2), 2.0);
            if (g_profile) 
            {
                profEnd = std::chrono::high_resolution_clock::now();
                elapsed_seconds = profEnd - profStart;
                std::cout << "[PROC][Show images][" << elapsed_seconds.count() << "]" << std::endl;
            }
        }

        if (g_writeToDisk)
        {
            if (fNum % shrdSettings.frameRecordingFrequency == 0)
            {
                framesToGrab = shrdSettings.framesToGrabEachBurst;
            }

            // capture all frames if not calibration, otherwise capture framesToGrab frames every FRAME_REC_FREQ frames
            if (!IS_CALIB_CAPTURE() || (framesToGrab > 0))
            {
                char fileName[1024];
                if (g_profile)
                    profStart = std::chrono::high_resolution_clock::now();
				
				if (shrdSettings.outputForMultiViewCalib)
				{
					//only save the relevant frames
					if (g_SaveNextCalibFrame[stream->serverId])
					{
						uint64_t diff = ULLONG_MAX;
						int closestID = 0;
                        std::lock_guard<std::mutex> lock(lastXFramesMutex);
						//search for closest timestamp
						for (unsigned int i = 0; i < lastXFrames[stream->serverId].size(); ++i)
						{
							uint64_t currDiff =  std::max(lastXFrames[stream->serverId][i]->timestampUSec, timestampForNextCapture) - std::min(lastXFrames[stream->serverId][i]->timestampUSec, timestampForNextCapture);
							
							if (currDiff < diff)
							{
								diff = currDiff;
								closestID = i; 
							}
						} 
						std::cout << "Saving frame with time diff " << diff << std::endl;
						std::shared_ptr<K4AFrame> frameToSave = lastXFrames[stream->serverId][closestID];

						//MultiViewCalib takes .bmp files
						if(shrdSettings.startMode != START_MODE_CONTROL_PANEL_EACH && shrdSettings.startMode != START_MODE_CONTROL_PANEL_MASTER)  // Not connected to Control Panel => Write to disk
                        {
						    // write the original image in outPath to %s/img%.4d.bmp
                            sprintf(fileName, "%s/img%.4d.bmp", outPath, framesWritten);
                            K4AToFusionUtils::SaveColorToImage(fileName, colorHeight, colorWidth, frameToSave->colorBuffer.get());
                            std::cout << "Writing frames to disk as " << fileName;
                            //transformedColorWrite
                            sprintf(fileName, "%s/img%.4d.bmp", outPath, framesWritten);
                            std::cout << " and " << fileName << std::endl;
                        }

						//pad it the same way we would pad our depth
						SourceInfo paddedInfo;
						paddedInfo.srcHeight = depthInfo.srcHeight;
						paddedInfo.srcWidth = depthInfo.srcWidth;
						paddedInfo.outHeight = shrdSettings.fusionHeight;
						paddedInfo.outWidth = shrdSettings.fusionWidth;
						paddedInfo.leftOffset = (paddedInfo.outWidth - paddedInfo.srcWidth) / 2;
						paddedInfo.topOffset = (paddedInfo.outHeight - paddedInfo.srcHeight) / 2;
						K4ADepthProcess::PadBuffer<uint32_t>(paddedInfo, 
							reinterpret_cast<uint32_t*>(frameToSave->transformedColorBuffer.get()),
							reinterpret_cast<uint32_t*>(frameToSave->paddedTransformColorBuffer.get()));
                        
						if(shrdSettings.startMode != START_MODE_CONTROL_PANEL_EACH && shrdSettings.startMode != START_MODE_CONTROL_PANEL_MASTER)  // Not connected to Control Panel => Write to disk
                        {
    						K4AToFusionUtils::SaveColorToImage(fileName, paddedInfo.outHeight, paddedInfo.outWidth, frameToSave->paddedTransformColorBuffer.get());
                        }
                        else  // Connected to Control Panel => Save to a buffer for network transmission
                        {
                            if(g_verbosity > 0)
                                std::cout << "Saving frame struct in memory for future transmission" << std::endl;
                            // Copy frameToSave into a new K4AFrame
                            K4AFrame* cFrame = new K4AFrame();
                            cFrame->timestampUSec = frameToSave->timestampUSec;
                            cFrame->_colorBufferSize = frameToSave->_colorBufferSize;
                            cFrame->colorBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[cFrame->_colorBufferSize]);

                            cFrame->_paddedTransformColorBufferSize = (paddedInfo.outHeight * paddedInfo.outWidth * sizeof(uint32_t));//frameToSave->_paddedTransformColorBufferSize;
                            cFrame->paddedTransformColorBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[cFrame->_paddedTransformColorBufferSize]);
                            memcpy(cFrame->paddedTransformColorBuffer.get(), frameToSave->paddedTransformColorBuffer.get(), cFrame->_paddedTransformColorBufferSize);

                            if (g_use_factory_calib)
                            {
                                if(g_verbosity > 0)
                                    std::cout << "Remapping colorBuffer to depth space" << std::endl;
                                cv::Mat inputMat(cv::Size(colorInfo.outWidth, colorInfo.outHeight), CV_8UC4, frameToSave->colorBuffer.get());
                                cv::Mat outputMat;
                                cv::remap(inputMat, outputMat, colorInfo.mapX, colorInfo.mapY, cv::INTER_LINEAR);
                                memcpy(cFrame->colorBuffer.get(), outputMat.data, cFrame->_colorBufferSize);
                            }
                            else
                            {
                                memcpy(cFrame->colorBuffer.get(), frameToSave->colorBuffer.get(), cFrame->_colorBufferSize);
                            }

                            if (g_calib_transmit_activeIR)
                            {
                                cFrame->_irBufferSize = frameToSave->_irBufferSize;
                                cFrame->irBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[cFrame->_irBufferSize]);
                                if(g_verbosity > 0)
                                    std::cout << "Copying " << frameToSave->_irBufferSize << " to the ir buffer" << std::endl;
                                memcpy(cFrame->irBuffer.get(), frameToSave->irBuffer.get(), cFrame->_irBufferSize);
                            }

                            if(g_writeDepthToBinFiles)
                            {                                
                                cFrame->_depthBufferSize = paddedInfo.outHeight * paddedInfo.outWidth * sizeof(uint16_t);
                                cFrame->depthBuffer = std::shared_ptr<uint8_t[]>(new uint8_t[cFrame->_depthBufferSize]);
                                memset(cFrame->depthBuffer.get(), 0, sizeof(uint8_t) * cFrame->_depthBufferSize);

                                if(g_verbosity > 0)
                                {
                                    std::cout << "Copying " << cFrame->_depthBufferSize << " to the depth buffer" << std::endl;
                                    std::cout << "DepthInfo in: " << depthInfo.srcWidth << "x" << depthInfo.srcHeight << " Out: " << depthInfo.outWidth << "x" << depthInfo.outHeight << std::endl;
                                    std::cout << "PaddedInfo in: " << paddedInfo.srcWidth << "x" << paddedInfo.srcHeight << " Out: " << paddedInfo.outWidth << "x" << paddedInfo.outHeight << std::endl;                                                            
                                }
	        					K4ADepthProcess::PadBuffer<uint16_t>(depthInfo, 
							    reinterpret_cast<uint16_t*>(frameToSave->depthBuffer.get()),
							    reinterpret_cast<uint16_t*>(cFrame->depthBuffer.get()));
                                if(g_verbosity > 1)
                                {
                                    sprintf(fileName, "%s/%s_img%.4d.png", outPath, g_recordingName.c_str(), framesWritten);
                                    std::cout << "Saving depth image as image to " << fileName << std::endl;
                                    K4AToFusionUtils::SaveDepthToImage(fileName, paddedInfo.outHeight, paddedInfo.outWidth,  reinterpret_cast<uint16_t*>(cFrame->depthBuffer.get()));
                                    sprintf(fileName, "%s/%s_img%.4d.bin", outPath, g_recordingName.c_str(), framesWritten);
                                    std::cout << "Saving depth image binary to " << fileName << std::endl;
                                    K4AToFusionUtils::SaveBufferToBin(fileName, paddedInfo.outHeight, paddedInfo.outWidth, reinterpret_cast<uint16_t*>(cFrame->depthBuffer.get()));
                                    if(g_calib_transmit_activeIR)
                                    {
                                        sprintf(fileName, "%s/%s__img_ir%.4d.png", outPath, g_recordingName.c_str(), framesWritten);
                                        std::cout << "Saving IR image " << depthInfo.srcHeight << "x" << depthInfo.srcWidth << " as PNG to " << fileName << std::endl;
                                        K4AToFusionUtils::SaveIRToImage(fileName, depthInfo.srcHeight, depthInfo.srcWidth, cFrame->irBuffer.get(), 0, 255);
                                    }
                                }
                            }                            

                            // Push it onto g_calibrationSavedFrames
                            std::lock_guard<std::mutex> lck(calibrationSavedFramesMutex);
                            while(g_calibrationSavedFrames.size() >= shrdSettings.maxSavedCalibrationFrames)
                            {
                                std::cout << "Deque is too full.  Deleting oldest saved frame." << std::endl;
                                // Captured too many.  Delete the oldest.
                                K4AFrame* frameToDelete = g_calibrationSavedFrames.front();
                                g_calibrationSavedFrames.pop_front();
                                delete frameToDelete;     
                                std::string errorMessage = "Deque is too full.  Deleting oldest saved frame.";
                                K4ACPC->SendStatusUpdate(CPC_STATUS::ERROR_MSG, errorMessage.c_str(), errorMessage.length());                   
                            }
                            g_calibrationSavedFrames.push_back(cFrame);
                            int capturedFrameNumber = g_calibrationSavedFrames.size();
                            std::cout << "Frame saved to deque.  Sending number (" << capturedFrameNumber << ") to control panel" << std::endl;
                            K4ACPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_START, &capturedFrameNumber, sizeof(int));                            
                            if(capturedFrameNumber == 1)
                            {
                                std::cout << "Obtaining an IMU sample" << std::endl;
                                // once per calibration capture, grab the IMU data and save it to local disk
                                k4a_imu_sample_t imuSample = stream->source->GetIMUReading();
                                char filePath[256];
                                sprintf(filePath, "%simusample.json", sourceManager.GetSharedSettings().calibPath.c_str());
                                if(g_verbosity > 0)
                                {
                                    std::cout << "Writing IMU data to " << filePath << std::endl;
                                }
                                DumpIMUReadingToDisk(imuSample, filePath);                                
                                if(g_verbosity > 0)
                                {
                                    std::cout << "Done!" << std::endl;
                                }
                            }                      
                        }

						std::lock_guard<std::mutex> guard(saveNexCalibFrameMutex);
						g_SaveNextCalibFrame[stream->serverId] = false;
						framesWritten++;
					}					
				}
				else
				{
                    std::cout << "NOT IMPLEMENTED IN K4AToFusionJetson!!!" << std::endl;
				}
				
                if (g_profile) 
                {
                    profEnd = std::chrono::high_resolution_clock::now();
                    elapsed_seconds = profEnd - profStart;
                    std::cout << "[PROC][Save Frame][" << elapsed_seconds.count() << "]" << std::endl;
                }

			
                framesToGrab--;
            }
        }                      
        frameCounter++;
        if(frameCounter >= PROCESS_FRAME_CONTROL_PANEL_UPDATE_FREQUENCY)
        {            
            if(IS_CALIB_CAPTURE())
            {
                Trinket::SetState(Trinket::Calibration_Running_Calib);
            }
            else if(IS_BG_CAPTURE())
            {
                Trinket::SetState(Trinket::Capturing_BG_Images);
            }
            else
            {
                Trinket::SetState(Trinket::Broadcast_Running);
            }
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double frameRate = (double)frameCounter/elapsed_seconds.count();
            if(g_verbosity > 0)
            {
                printf("[PROCFR: %.2f]\r\n", frameRate);;
            }
            frameCounter = 0;
            start = std::chrono::high_resolution_clock::now();
            K4ACPC->SendStatusUpdate(CPC_STATUS::FPS, &frameRate, sizeof(frameRate));
        }
    }

    K4ACPC->SendStatusUpdate(CPC_STATUS::STOPPED);
    delete[] processedDepthBuffer;
    delete[] avgBufUndistorted;
    delete[] stdDevUndistorted;
    delete[] avgBufPadded;
    delete[] stdDevPadded;
    delete[] temporalAvgBuf;
}


void CaptureFramesFn(K4AFusionStream* stream, SharedSettings shrdSettings)
{
    if(g_verbosity > 1)
        std::cout << "[CAP] Starting" << std::endl;
    g_StreamReadyCnt.fetch_add(1);

    int depthFrameSize = stream->source->GetDepthImgDesc().height_pixels * stream->source->GetDepthImgDesc().stride_bytes;
    int colorFrameSize = 0; // defined each frame because MJPG format may change this value on a per-frame basis, used to be: stream->source->GetColorImgDesc().height_pixels * stream->source->GetColorImgDesc().stride_bytes;    
	int transformedColorFramesize = stream->source->GetDepthImgDesc().height_pixels * stream->source->GetDepthImgDesc().width_pixels * 4 * (int) sizeof(uint8_t);


    int framesSkipped = 0;
    uint64_t prevTimestampUsec = 0;


	//used for color transformation
	k4a_image_t transformedImage;
	k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, stream->source->GetDepthImgDesc().width_pixels, stream->source->GetDepthImgDesc().height_pixels, stream->source->GetDepthImgDesc().width_pixels * 4 * (int) sizeof(uint8_t), &transformedImage);

	k4a_calibration_t currCalibration = stream->source->GetFactoryCalibration();
	k4a_transformation_t currTransform = k4a_transformation_create(&currCalibration);

    //use preset Depth for Color->Depth transformation to avoid holes

    int frameCounter = 0;
    auto start = std::chrono::high_resolution_clock::now();    
    while (g_IsRunning)
    {
        // capture all sources in parallel
        int result = stream->source->CaptureFrame();
        if (result == 0)
        {
            if(g_verbosity > 1)
                std::cout << "[C] -\r";
            continue;
        }
        else if(result == -1)
        {
            K4ACPC->SendStatusUpdate(CPC_STATUS::READY, K4ACPC->GetIdentificationString().c_str(), K4ACPC->GetIdentificationString().length());
            Trinket::SetState(Trinket::Starting_Broadcast);
            if(g_verbosity > 1)
                std::cout << "[C] +\r";            
            continue;
        }
        else if (framesSkipped < shrdSettings.skipFirstNFrames)
        {
            ++framesSkipped;
            prevTimestampUsec = stream->source->GetLatestTimestamp();
            if(g_verbosity > 0)
                printf("[CAPTURE] -- %03d/%03d %lu\n", framesSkipped, shrdSettings.skipFirstNFrames, prevTimestampUsec);
            if (framesSkipped == shrdSettings.skipFirstNFrames)
            {
                printf("[CAPTURE] -- Ready. Queuing frames.\n");
            }
            continue;
        }

        // aggregate captures into single buffer as they complete      
		int irFrameSize = depthFrameSize;
		if (!g_captureIR)
		{
			irFrameSize = 0;
		}
        //Get the actual color frame size
        colorFrameSize = stream->source->GetColorImageBufferSize();
        depthFrameSize = stream->source->GetDepthImageBufferSize();
		std::shared_ptr<K4AFrame> curFrame = std::make_shared<K4AFrame>(depthFrameSize, colorFrameSize, irFrameSize, transformedColorFramesize, MAX_AUDIO_PACKET_SIZE);

        // TODO use a ring buffer so we don't have to regularly reallocate
        memcpy(curFrame->depthBuffer.get(), stream->source->GetDepthBuff(), depthFrameSize);

        memcpy(curFrame->colorBuffer.get(), stream->source->GetColorBuff(), colorFrameSize);

        memcpy(curFrame->audioBuffer.get(), stream->source->GetAudioBuff(), MAX_AUDIO_PACKET_SIZE);

        if (g_captureIR)
        {
            memcpy(curFrame->irBuffer.get(), stream->source->GetIRBuff(), depthFrameSize);
        }
		// grab the color camera, transform it into depth sapce, write it out and calibrate using it

        // Only do this if we are in BGRA mode     
        if(k4a_image_get_format(stream->source->GetColorImageHandle()) == K4A_IMAGE_FORMAT_COLOR_BGRA32)
        {
            k4a_image_get_buffer(stream->source->GetDepthImageHandle());
            k4a_transformation_color_image_to_depth_camera(
                currTransform,
                stream->source->GetDepthImageHandle(),
                stream->source->GetColorImageHandle(),
                transformedImage);
            memcpy(curFrame->transformedColorBuffer.get(), k4a_image_get_buffer(transformedImage), transformedColorFramesize);
        }
        curFrame->timestampUSec = stream->source->GetLatestTimestamp();
        curFrame->framesPassed = (uint32_t)round((curFrame->timestampUSec - prevTimestampUsec) / stream->source->GetFrameDelayUSec());
        prevTimestampUsec = curFrame->timestampUSec;

        int requestedSpeed = K4ACPC->GetRequestedSpeed();
        if(requestedSpeed != g_CurrentRequestedFPS)
        {
            std::cout << "Speed change requested.  Current speed is " << g_CurrentRequestedFPS << " new request is for " << requestedSpeed << " config file speed is " << g_ConfigFileRequestedFPS << std::endl;
            // If this is a change from "full speed" to something else, dump the frame queue
            if(g_CurrentRequestedFPS < 0)
            {
                std::cout << "Dumping the stream->frameBuffer queue" << std::endl;
                g_FramePerSecondCounter = 1;
                stream->bufferMtx->lock();
                {
                    // Clear the frame buffer
                    while (!stream->frameBuffer.empty())
                    {
                        stream->frameBuffer.pop();
                    }                    
                }
                stream->bufferMtx->unlock();
            }
            // Update my "current speed"           
            g_CurrentRequestedFPS = requestedSpeed;
        }

        
        {
            std::lock_guard<std::mutex> lock(lastXFramesMutex);        
            if (lastXFrames[stream->serverId].size() >= cFramesToKeep)
            {
                lastXFrames[stream->serverId].pop_front(); // remove one			
            }
            lastXFrames[stream->serverId].push_back(curFrame);	
        }

        // put capture into buffer
        bool shouldEnqueue = true;

        // If the buffer is getting too big, drop the oldest
        // Compare my current speed to the configured speed
        // If my current speed is slower, figure out if I should skip enqueueing this frame
        if(g_CurrentRequestedFPS >= 0 && g_CurrentRequestedFPS < g_ConfigFileRequestedFPS)
        {            
            double ratio = (double)g_CurrentRequestedFPS / (double)g_ConfigFileRequestedFPS;
            // which "frame" in a second am I on right now?
            if(g_FramePerSecondCounter > g_ConfigFileRequestedFPS)
            {
                g_FramePerSecondCounter = 1;
            }

            double t = std::truncf( (ratio * (double)g_FramePerSecondCounter) );
            double f = (ratio * (double)g_FramePerSecondCounter) - t;

            std::cout << "Frame " << g_FramePerSecondCounter << " f: " << f << " ratio: " << ratio << std::endl;
            g_FramePerSecondCounter++;
            if(f - ratio > 0.01)
            {
                std::cout << "[SLOWDOWN] Enqueueing frame" << std::endl;
            }
            else
            {
                std::cout << "[SLOWDOWN] skip this frame" << std::endl;
                shouldEnqueue = false;
            }
        }
        if(shouldEnqueue)
        {
            stream->bufferMtx->lock();
            {
                stream->frameBuffer.push(curFrame);
                if(stream->frameBuffer.size() >= shrdSettings.maxBufferSize)
                {
                    std::cout << "WARNING: Frame queue is full.  Dropping oldest frame.\r\n";
                    stream->frameBuffer.pop();
                }
            }
            stream->bufferMtx->unlock();
        }

        // release captures
        stream->source->ReleaseFrame();

        frameCounter++;
        if(frameCounter >= 50)
        {
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double frameRate = (double)frameCounter/elapsed_seconds.count();
            printf("[CAPTFR: %.2f]\r\n", frameRate);
            frameCounter = 0;
            start = std::chrono::high_resolution_clock::now();
        }
    }
    K4ACPC->SendStatusUpdate(CPC_STATUS::STOPPED);
}


int StartFusionConnection(K4AFusionStream& stream, const SharedSettings& settings)
{
    // initialize fusion connection    
    const k4a_image_descriptor_t& depthDesc = stream.source->GetDepthImgDesc();
    const k4a_image_descriptor_t& colorDesc = stream.source->GetColorImgDesc();

    printf("\n==> Initializing server for camera [%s] on port %d...\n", stream.source->GetSerial(), DEFAULT_PORT_BASE + stream.portOffset);

    int depthBpp = depthDesc.stride_bytes / depthDesc.width_pixels;
    printf(" Depth: %d x %d (%d bpp)\n", depthDesc.width_pixels, depthDesc.height_pixels, depthBpp);
    
    int colorBpp = colorDesc.stride_bytes / colorDesc.width_pixels;
    printf(" Color: %d x %d (%d bpp)\n", colorDesc.width_pixels, colorDesc.height_pixels, colorBpp);

    //TODO FIX FUSION TO WORK WITH OTHER DIMENSIONS!!!! After fix use -->> depthDesc.width_pixels * depthDesc.height_pixels, // depth size
    HRESULT hr = DepthToFusion_InitServer(
        1, // number of streams
        settings.fusionWidth * settings.fusionHeight,
        depthBpp,  // bytes per depth
        colorDesc.width_pixels * colorDesc.height_pixels, // MAX color size, using MJPG WILL result in smaller color frame sizes, so this should not be used for accessing capture frame buffers
        colorBpp,  // bytes per color
        stream.serverId,    // server id out
        DEFAULT_PORT_BASE,
        stream.portOffset);

    if (FAILED(hr) || stream.serverId < 0)
    {        
        return 0;
    }
    
    stream.bufferMtx = new std::mutex();
    stream.transferThread = new std::thread(ProcessFramesFn, &stream, settings);
    stream.captureThread = new std::thread(CaptureFramesFn, &stream, settings);

    return 1;
}

void StopFusionConnection(K4AFusionStream& stream)
{
    printf("\nStopping fusion connection for camera [%s] on port %d...\n", stream.source->GetSerial(), DEFAULT_PORT_BASE + stream.portOffset);
    DepthToFusion_DestroyServer(stream.serverId);
    printf("Fusion connection stopped.\n");
}

void MainLoop()
{
    while (g_IsRunning)
    {        
        char c = getchar();
            switch (c)
            {
            case 'E':
                // increase exposure
                break;
            case 'e':
                // decrease exposure
                break;
            case 'u':
                g_undistortDepth = !g_undistortDepth;
                printf("  Undistort Depth: %s\n", g_undistortDepth ? "YES" : "NO");
                break;
            case 'm':
                g_processMask = !g_processMask;
                printf("  Process Mask: %s\n", g_processMask ? "YES" : "NO");
                break;
            case 'q':
                g_IsRunning = false;
                break;
			case 'c':
				if (IS_CALIB_CAPTURE()) 
				{					
					std::lock_guard<std::mutex> guard(saveNexCalibFrameMutex);
					timestampForNextCapture = sourceManager.GetCaptureSources()[0].GetLatestTimestamp();
					gShouldPublishTimeStamp = true;
					for (std::vector<bool>::iterator it = g_SaveNextCalibFrame.begin(); it != g_SaveNextCalibFrame.end(); ++it)
					{
						*it = true;
					}
					printf(" Calib Image Captured. \n");
					std::cout << timestampForNextCapture << std::endl;
				}
				break;
					
            }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void DumpCalibToDisk(k4a_calibration_camera_t calib, const char* filePath)
{
    Json::Value root;

    root["resolution"]["width"] = calib.resolution_width;
    root["resolution"]["height"] = calib.resolution_height;
    //rotation
    const int rotLength = sizeof(calib.extrinsics.rotation) / sizeof(calib.extrinsics.rotation[0]);
    for (int j = 0; j < rotLength; ++j)
    {
        root["rotation"][j] = calib.extrinsics.rotation[j];
    }

    //translation
    const int transLength = sizeof(calib.extrinsics.translation) / sizeof(calib.extrinsics.translation[0]);
    for (int j = 0; j < transLength; ++j)
    {
        root["translation"][j] = calib.extrinsics.translation[j];
    }
    //Intrinsics
    std::string names[15] = {"cx", "cy", "fx", "fy", "k1", "k2", "k3", "k4", "k5", "k6", "codx", "cody", "p2", "p1", "metric radius"};
    for (unsigned int j = 0; j < calib.intrinsics.parameter_count; ++j)
    {
        root["intrinsics"][names[j]] = calib.intrinsics.parameters.v[j];
    }
    Json::StreamWriterBuilder wbuilder;
    wbuilder["indentation"] = "\t";
    std::string document = Json::writeString(wbuilder, root);
    std::ofstream ofs(filePath, std::ios::trunc);
    ofs << std::fixed;
    ofs.precision(6);
    ofs << document << std::endl;
}

void DumpExtrinsicsToDisk(k4a_calibration_extrinsics_t extrinsics[K4A_CALIBRATION_TYPE_NUM][K4A_CALIBRATION_TYPE_NUM], const char* filePath)
{
    Json::Value root;
    root["Extrinsics"] = Json::arrayValue;

    std::string names[K4A_CALIBRATION_TYPE_NUM] = {"Depth", "Color", "Gyroscope", "Accelerometer"};
    for(int i = 0; i < K4A_CALIBRATION_TYPE_NUM; i++ )
    {
        for(int j = 0; j < K4A_CALIBRATION_TYPE_NUM; j++)
        {
            if(i == j)
                continue;
            Json::Value extrinsic;
            extrinsic["source"] = names[i];
            extrinsic["destination"] = names[j];
            for(int r = 0; r < 9; r++)
            {
                extrinsic["rotation"][r] = extrinsics[i][j].rotation[r];
                }
            for(int t = 0; t < 3; t++)
            {
                extrinsic["translation"][t] = extrinsics[i][j].translation[t];
                }
            root["Extrinsics"].append(extrinsic);
            }
            }
    Json::StreamWriterBuilder wbuilder;
    wbuilder["indentation"] = "\t";
    std::string document = Json::writeString(wbuilder, root);
    std::ofstream ofs(filePath, std::ios::trunc);
    ofs << std::fixed;
    ofs.precision(6);
    ofs << document << std::endl;
}

int main(int argc, char* argv[])
{
    // this is the default logfile name and will be overwritten if present in the config
    logfile.append(argv[0]);
    errorfile.append(argv[0]);
    logfile.append(".log");
    errorfile.append(".error");
    
    if (argc != 3)
    {
        printf("Usage: k4atofusion.exe <path to config.txt> [capture_mode] \n");
        printf("   path to config.txt:  path to camera configuration file.\n");
        printf("   capture_mode:        [optional, default: 0], specific recording modes\n");
        printf("                                   0 - Stream mode -- enables network streaming service for connected clients\n");
        printf("                                       on specified ports\n");
        printf("                                   1 - Calibration mode -- will write color, unprocessed depth, and ir frames\n");
        printf("                                       to disk in bursts of %s frames every %s frames\n", "[FrameToGrabEachBurst]", "[FrameRecordingFrequenc] ");
        printf("                                   2 - BG mode -- will create a background depth image per camera averaged over %d\n", DEFAULT_MIN_BACKGROUND_SAMPLES);
        printf("                                       frames, and compute per - pixel std deviation.\n");
        printf("                                   3 - Offline capture mode -- will write color and depth in binary mode w/mask\n");
        printf("                                       and padding applied\n");
        printf("                                   4 - Config mode -- will only display frames for purposes of visualization / setting exposure settings\n");
        printf("                                   5 - Factory Calib Write mode -- will grab factory calibration off Kinects and write it to disk(into CalibrationPath)\n");
        return -1;
    }

    const char* configPath = argv[1];

    CConfig currentConfig;
    currentConfig.SetErrorIfNameNotFound(false);
    currentConfig.Load(configPath);
    g_verbosity = currentConfig.GetValueWithDefault("PodGlobalConfig", "Verbosity", 0);
    g_profile = currentConfig.GetValueWithDefault("PodGlobalConfig", "Profile", false);
    if(g_verbosity > 0)
        currentConfig.SetErrorIfNameNotFound(true);

    g_captureMode = (argc > 2) ? (CaptureMode)atoi(argv[2]) : g_captureMode;
    //std::ofstream out("/var/log/K4AToFusion/output.log", std::ios_base::out);
    //std::streambuf* coutbuf;

    if(currentConfig.GetValueWithDefault("PodGlobalConfig", "DaemonMode", false))
    {            
        logfile = currentConfig.GetValueWithDefault("PodGlobalConfig", "LogFilename", logfile);
        errorfile = currentConfig.GetValueWithDefault("PodGlobalConfig", "ErrorLogFilename", errorfile);
        std::cout << "Redirecting stdout to " << logfile << " and stderr to " << errorfile << std::endl;

        if(!freopen(logfile.c_str(), "w", stdout))
        {
            std::cerr << "ERROR.  Could not redirect output to logfile!" << std::endl;
        }
        if(!freopen(errorfile.c_str(), "w", stderr))
        {
            std::cerr << "ERROR.  Could not redirect error to logfile!" << std::endl;
        }
        auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::cout << "[" << ctime(&timenow) << "] Starting logfile." << std::endl;
    }
    std::cout << argv[0] << " Version " << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_PATCH << "-" << VERSION_BRANCH_NAME << "." << VERSION_COMMITS << " " << VERSION_DESCRIPTION << std::endl;
    g_recordingName = currentConfig.GetValueWithDefault("PodGlobalConfig", "RecordingFolderName", "rec");
    g_showImages = currentConfig.GetValueWithDefault("PodGlobalConfig", "ShowColorFrame", IS_CONFIG_CAPTURE());    

    g_processDepth = !IS_CALIB_CAPTURE() && !IS_BG_CAPTURE() && !IS_OFFLINE_CAPTURE();
    g_writeToDisk = (IS_CALIB_CAPTURE() || IS_OFFLINE_CAPTURE());
    g_captureIR = IS_CALIB_CAPTURE() || IS_CONFIG_CAPTURE();
    g_writeDepthToBinFiles = currentConfig.GetValueWithDefault("PodGlobalConfig", "calibCaptureDepth", IS_OFFLINE_CAPTURE());

    g_calib_transmit_activeIR = currentConfig.GetValueWithDefault("PodGlobalConfig", "calibCaptureActiveIR",false);
    
    // We should always load intrinsics provided in the 3DTM calibration file for the PAI algorithm, since 
    // factory intrinsics are used during calibration if requested and stored in the 3DTM Json format.
    g_use_factory_calib = false;

	serialPortFilename = currentConfig.GetValueWithDefault("PodGlobalConfig", "DisplaySerialPort", serialPortFilename);
    //g_CalibDetectCheckerboard = currentConfig.GetValueWithDefault("PodGlobalConfig", "DetectCheckerboardDuringCalibration", true);
    switch(currentConfig.GetValueWithDefault("PodGlobalConfig", "CameraFPS", 1))
    {
        case 0:
            g_ConfigFileRequestedFPS = 5;
            break;
        case 2:
            g_ConfigFileRequestedFPS = 30;
            break;
        case 1:
        default:
            g_ConfigFileRequestedFPS = 15;
    }
    printf("Config requested FPS: %d\n", g_ConfigFileRequestedFPS);
    if (!g_captureIR)
        g_captureIR = g_calib_transmit_activeIR;

    char tBuf[50];
    K4AToFusionUtils::GetDateStr(tBuf, 50);
    g_recordingName += std::string("_") + tBuf;
    printf("Recrd name:       %s\n", g_recordingName.c_str());
    printf("CaptureMode:      %s\n", GetCaptureModeString(g_captureMode));
    printf("Write to disk:    %s\n", g_writeToDisk ? "YES" : "NO");
    printf("Capture IR:       %s\n", g_captureIR ? "YES" : "NO");
    printf("Depth write:      %s\n", g_writeDepthToBinFiles ? "BIN" : "PNG");
    printf("Show images:      %s\n", g_showImages ? "YES" : "NO");
    printf("Undistort depth:  %s\n", g_undistortDepth ? "YES" : "NO");

    std::thread ourZMQThread;
    K4AFusionStream* streams = nullptr;
    std::string serialStr;        

    // determine this pod's number based on the hostname
    // the pod number is used to determine which sets of override settings in the config we care about
    // and sets things like leader/follower
    char hostname[1024];
    gethostname(hostname, 1024);
    uint thisPodNumber = atoi(&hostname[4]);
    printf("Got a hostname of \"%s\", setting this pod number to %d", hostname, thisPodNumber);
    sourceManager.SetThisPodNumber(thisPodNumber);
    // specify camera configuration: NOTE, these are for ALL cameras, camera instance specific settings are within LoadCameraFile    
    k4a_device_configuration_t k4a_device_config;
    SetDefaultConfig(k4a_device_config);

    //must initialize before the first goto exit call, but since this is a reference, the updates in Initialize should be visible
    const SharedSettings& shrdSettings = sourceManager.GetSharedSettings();

    // initialize camera sources (using config file if present)
    printf("\n==> Initializing camera...\n");
    if (!sourceManager.Initialize(k4a_device_config, configPath, g_captureIR || g_calib_transmit_activeIR, IS_CALIB_CAPTURE()))
    {
        printf("Failed to initialize devices, aborting!\n");
        goto exit;
    }
        
    if(!g_use_factory_calib)
    {
        // load calibration files from disk
        printf("\n==> Loading calibration files from: '%s'...", shrdSettings.calibPath.c_str());
        if (!sourceManager.LoadCalibrations(shrdSettings.calibPath.c_str(), shrdSettings.numKParams))
        {
            printf("Failed\n");
        }
        else
        {
            printf("SUCCESS\n");
        }
    }
    else
    {
        std::cout << std::endl << "==> Using internal factory intrinsic calibration values" << std::endl;
        sourceManager.LoadFactoryCalibrations();
    }

    // initialize the checkerboard detector if we're in calib mode
  //  if(IS_CALIB_CAPTURE())
  //  {
  //      checkerboardDetector.m_verbose = g_verbosity;
  //      std::string checkerboardDefinitionSection = currentConfig.GetValueWithDefault<std::string>("Calibration", "CalibrationBoardDefinition", "CalibrationBoard-CostcoMedium");
  //      if(!checkerboardDetector.LoadCheckerBoardInfo(checkerboardDefinitionSection, &currentConfig))
  //      {
  //          std::cout << "Can't detect checkerboards, couldn't load checkerboard definition!" << std::endl;
  //      }
  //      maxPointsToDetectInACheckerboard = (checkerboardDetector.m_CBInfo.m_nNumX * checkerboardDetector.m_CBInfo.m_nNumY) -
  //          ((checkerboardDetector.m_CBInfo.m_nClNum[0] * checkerboardDetector.m_CBInfo.m_nClDist[0] + 1) * (checkerboardDetector.m_CBInfo.m_nClNum[1] * checkerboardDetector.m_CBInfo.m_nClDist[1] + 1));        
  //      undistortBorderPixels = currentConfig.GetValueWithDefault("Calibration", "UndistortBorderPixels", MIN_UNDISTORT_BORDER_PIXELS);
  //      minCheckerboardSize = currentConfig.GetValue<int>("Calibration", "MinLandmarkSize");
		//maxCheckerboardSize = currentConfig.GetValue<int>("Calibration", "MaxLandmarkSize");
  //  }

    // prepare fusion streams
    streams = BuildFusionStreamsFromSources(sourceManager);   

    // set flag before starting fusion streams
    g_IsRunning = true;

    // initialize stream instances (one per camera)
    for (uint32_t i = 0; i < sourceManager.GetNumSources(); ++i)
    {        
		g_SaveNextCalibFrame.push_back(false);
        if (StartFusionConnection(streams[i], shrdSettings) == 0)
        {
            printf("Failed to start fusion stream: %d\n", i);
            goto exit;
        }
		std::deque<std::shared_ptr<K4AFrame>> initDeque;
        std::lock_guard<std::mutex> lock(lastXFramesMutex);
		lastXFrames.push_back(initDeque);
    }

    //Code to dump k4aCalib to disk
    if (IS_WRITING_FACTORY_CALIB_CAPTURE())
    {
        printf("Writing Factory Calib from Kinects to Disk \n");
        for (uint32_t i = 0; i < sourceManager.GetNumSources(); ++i)
        {
            k4a_calibration_t factCalib = streams[i].source->GetFactoryCalibration();

            char filePath[256];
            sprintf(filePath, "%s/depthConfig%02d.json", sourceManager.GetSharedSettings().calibPath.c_str(), i);
            DumpCalibToDisk(factCalib.depth_camera_calibration, filePath);

            sprintf(filePath, "%s/colorConfig%02d.json", sourceManager.GetSharedSettings().calibPath.c_str(), i);
            DumpCalibToDisk(factCalib.color_camera_calibration, filePath);
                        
            sprintf(filePath, "%s/extrinsics%02d.json", sourceManager.GetSharedSettings().calibPath.c_str(), i);
            DumpExtrinsicsToDisk(factCalib.extrinsics, filePath);
        }
        printf("DONE");
        goto exit;
    }
    
	//Start ZMQ related threads	
	if (IS_CALIB_CAPTURE())
	{
        std::cout << "Calibration mode selected.  Starting inter-kinect ZMQ thread " << std::endl;
		if (shrdSettings.startMode == StartMode::START_MODE_MANUAL_MASTER || shrdSettings.startMode == StartMode::START_MODE_CONTROL_PANEL_MASTER)
		{
            std::cout << " publishing on port " << shrdSettings.calibPublisherPort << std::endl;
			ourZMQThread = std::thread(ZMQPublisherThread, shrdSettings.calibPublisherPort);
		}
		else if (shrdSettings.startMode == StartMode::START_MODE_MANUAL_EACH || shrdSettings.startMode == StartMode::START_MODE_CONTROL_PANEL_EACH)
		{
            std::cout << " subscribing to " << shrdSettings.calibPublisherIPAddress << ":" << shrdSettings.calibPublisherPort << std::endl;
			ourZMQThread = std::thread(ZMQSubscriberThread, shrdSettings.calibPublisherIPAddress, shrdSettings.calibPublisherPort);
		}
        else
        {
            std::cout << " NOT start the ZMQ thread because start mode is not set to a master/slave setting" << std::endl;
        }
	}

    K4ACPC = new K4AControlPanelConnector(shrdSettings.controlPanelIPAddress,
        shrdSettings.eventTCPPort,
        myIP,
        shrdSettings.statusTCPPort,
        currentConfig.GetValueWithDefault("PodGlobalConfig", "DisableHeartbeat", false));
    K4ACPC->SetVersionInformation(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMITS, std::string(VERSION_BRANCH_NAME), std::string(VERSION_DESCRIPTION), std::string(VERSION_SHA1));
    if(g_verbosity > 0)
        std::cout << "Setting control panel connector." << std::endl;
    sourceManager.SetControlPanelConnector(K4ACPC);

    // send serial number with the IS_ALIVE signal
    serialStr = "0000"; // hold the four chars for the size int I'll replace them with later
    {
        for (uint32_t i = 0; i < sourceManager.GetNumSources(); ++i)
        {
            const char* serial = streams[i].source->GetSerial();
            serialStr.append(serial);
            //std::cout << "Got " << i << " source.  Serial string " << serialStr << std::endl;
            if(i > 0)
            {
                serialStr.append(";");
            }
        }
        int len = serialStr.length();
        ((int*)serialStr.c_str())[0] = len - sizeof(int);

        for(int j = 0; j < 5; j++)
        {            
            K4ACPC->SendStatusUpdate(CPC_STATUS::IS_ALIVE, (const void*)serialStr.c_str(), len);
        }
    }
    // Setting the serial number allows us to send the serial number with every watchdog network packet
    // this is useful in cases where we've already been running, and then the control panel connects,
    K4ACPC->SetKinectSerialNumber(serialStr);

    // wait until all streams are ready for data
    while (g_StreamReadyCnt.load() < sourceManager.GetNumSources())
    {
        std::this_thread::yield();        
    }

    // start all sources
    K4ACPC->SetStartCommand(false);
    K4ACPC->SetStopCommand(false);
    if (!sourceManager.Start())
    {
        goto exit;
    }

    // Control-panel-controlled MainLoop            
    if (shrdSettings.startMode == START_MODE_CONTROL_PANEL_MASTER || shrdSettings.startMode == START_MODE_CONTROL_PANEL_EACH)
    {
        int calibrationFramesSent = 1;
        while(!K4ACPC->GetStopCommand())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(K4ACPC->GetHasNewCalibrationData() && !IS_CALIB_CAPTURE())
            {
                // Load the new calibration data
                if(g_verbosity > 0)
                    std::cout << "New calibration data received!  LIVE loading new calibration(s)." << std::endl;
                std::string ser(streams[0].source->GetSerial());
                Calibration newCalib = K4ACPC->GetCalibration(ser);  //just so I know how many k params are in the new calibration
                // could load directly from memory, right now let's test reading/writing
                sourceManager.LoadCalibrations("./K4AToFusion", newCalib.numKParams);
                
                K4ACPC->SetHasNewCalibrationData(false);
            }

            if(K4ACPC->GetCalibrationFrameRequest())
            {
                K4ACPC->SetCalibrationFrameRequest(false);
                // Pull oldest from the queue and send it
                if(g_calibrationSavedFrames.size() > 0)
                {                    
                    if(g_verbosity > 0)
                    {
                        std::cout << "Grabbing frame to transmit..." << std::endl;
                    }
                    std::lock_guard<std::mutex> lck(calibrationSavedFramesMutex);
                    {
                        K4AFrame* frameToSend = g_calibrationSavedFrames.front();
                        g_calibrationSavedFrames.pop_front();
                        // make a data packet
                        // [(int)FrameNumber][(uint64_t)FrameTimestampUSec][RemainingPacketSize][(int)colorFrameSize][(char[colorFrameSize])colorFrame][(int)transformedColorFrameSize][(char[transformedColorFrameSize])transformedColorFrame]                        
                        int remainingPacketSize = sizeof(frameToSend->_colorBufferSize) 
                                                    + frameToSend->_colorBufferSize 
                                                    + sizeof(frameToSend->_paddedTransformColorBufferSize)
                                                    + frameToSend->_paddedTransformColorBufferSize;
                        if (g_calib_transmit_activeIR)
                        {
                            remainingPacketSize += sizeof(frameToSend->_irBufferSize) + frameToSend->_irBufferSize;
                        }
                        if (g_writeDepthToBinFiles)
                        {
                            remainingPacketSize += sizeof(frameToSend->_depthBufferSize) + frameToSend->_depthBufferSize;
                        }


                        int fullPacketSize = (sizeof(calibrationFramesSent) + sizeof(frameToSend->timestampUSec) + sizeof(remainingPacketSize) + remainingPacketSize) * sizeof(uint8_t);
                        uint8_t* data = (uint8_t*)malloc( fullPacketSize );
                        int ptr = 0;
                        *((int*)(&data[ptr])) = calibrationFramesSent;
                        ptr += sizeof(int);

                        *((uint64_t*)(&data[ptr])) = frameToSend->timestampUSec;  //sizeof(uint_64t) on ARM is 8
                        ptr += sizeof(uint64_t);                        
                        
                        *((int*)(&data[ptr])) = remainingPacketSize;
                        ptr += sizeof(int);

                        *((int*)(&data[ptr])) = frameToSend->_colorBufferSize;
                        ptr += sizeof(int);                        

                        memcpy(&data[ptr], frameToSend->colorBuffer.get(), frameToSend->_colorBufferSize);
                        ptr += frameToSend->_colorBufferSize;

                        *((int*)(&data[ptr])) = frameToSend->_paddedTransformColorBufferSize;
                        ptr += sizeof(int);                        

                        memcpy(&data[ptr], frameToSend->paddedTransformColorBuffer.get(), frameToSend->_paddedTransformColorBufferSize);
                        ptr += frameToSend->_paddedTransformColorBufferSize;

                        if (g_calib_transmit_activeIR)
                        {
                            *((int*)(&data[ptr])) = frameToSend->_irBufferSize;
                            ptr += sizeof(int);

                            memcpy(&data[ptr], frameToSend->irBuffer.get(), frameToSend->_irBufferSize);
                            ptr += frameToSend->_irBufferSize;
                        }
                        if (g_writeDepthToBinFiles)
                        {
                            *((int*)(&data[ptr])) = frameToSend->_depthBufferSize;
                            ptr += sizeof(int);

                            memcpy(&data[ptr], frameToSend->depthBuffer.get(), frameToSend->_depthBufferSize);
                            ptr += frameToSend->_depthBufferSize;
                        }
                        if(g_verbosity > 0)                       
                            std::cout << "Sending packet.  Timestamp: " << frameToSend->timestampUSec << 
                                "Color Buffer Size: " << frameToSend->_colorBufferSize << 
                                "Transformed Color Buffer Size: " << frameToSend->_paddedTransformColorBufferSize << 
                                "IRBuffer Size: " << frameToSend->_irBufferSize << 
                                "DepthBuffer size: " << frameToSend->_depthBufferSize << std::endl;
                        K4ACPC->SendStatusUpdate(CPC_STATUS::CALIBRATION_PROGRESS, data, fullPacketSize);

                        calibrationFramesSent++;
                        delete data;
                        delete frameToSend;
                    }
                }
                else
                {
                    if(g_verbosity > 0)
                    {
                        std::cout << "Got a transmit frame request, but I don't have any captured frames." << std::endl;
                    }                    
                }
            }
            if(shrdSettings.startMode == START_MODE_CONTROL_PANEL_MASTER && K4ACPC->GetCalibrationCaptureFrame())
            {
                // I am the master Kinect. I need to set the desired timestamp.  Equivalent of pressing 'C' in non-control-panel-mode
                // BUG: this should really get the _middle_ timestamp, NOT the latest
                std::lock_guard<std::mutex> guard(saveNexCalibFrameMutex);
                timestampForNextCapture = sourceManager.GetCaptureSources()[0].GetLatestTimestamp();
                gShouldPublishTimeStamp = true;
                for (std::vector<bool>::iterator it = g_SaveNextCalibFrame.begin(); it != g_SaveNextCalibFrame.end(); ++it)
                {
                    *it = true;
                }
                if(g_verbosity > 0)
                {
                    std::cout << " Calib Image Captured." << std::endl;
                    std::cout << timestampForNextCapture << std::endl;
                }
                K4ACPC->SetCalibrationCaptureFrame(false);
            }
        }  
        // Send this a few times to make sure the control panel sees it
        for(int i = 0; i < 5; i++)
        {
            K4ACPC->SetStartCommand(false);
            K4ACPC->SendStatusUpdate(CPC_STATUS::STOPPED, NULL, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else
    {
        // Keyboard-input-blocking MainLoop
        MainLoop();    
    }

 exit:
    {
    // kill all capture threads
    printf("\n==> Shutting down...\n");
    g_IsRunning = false;
    if (streams != nullptr)
    {
        for (uint32_t i = 0; i < sourceManager.GetNumSources(); ++i)
        {
            std::cout << "Stopping fusion connection...";
            StopFusionConnection(streams[i]);
            std::cout << "SFC Done." << std::endl;
            if(streams[i].transferThread != nullptr)
            {
                std::cout << "Joining transfer thread..." << std::endl;
                streams[i].transferThread->join();
                delete streams[i].transferThread;
            }
            std::cout << "Done." << std::endl;         
            if(streams[i].captureThread != nullptr)
            {
                std::cout << "Joining capture thread..." << std::endl;
                streams[i].captureThread->join();
                delete streams[i].captureThread;
            }
            std::cout << "Done." << std::endl;
            if(streams[i].bufferMtx != nullptr)
            {
                std::cout << "Freeing memory..." << std::endl;
                delete streams[i].bufferMtx;            
            }
            std::cout << "Done." << std::endl;
        }

        delete[] streams;
    }

    sourceManager.Stop();
    std::cout << "All stopped." << std::endl;
    std::cout << "Done." << std::endl;

	return 0;
    }
}

