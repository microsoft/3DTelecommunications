#ifndef __DATACAPTURETHREAD_H__
#define __DATACAPTURETHREAD_H__


#include <map>
#include <queue>
#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include <thread>

#include "BufferAggregator.h"

#include <opencv2\opencv.hpp>

#define NOMINMAX
#include <Windows.h>

#include <Unknwn.h>

#include "UtilMatrix.h"
#include "GlobalDataStatic.h"
#include "basic_structure.h"
#include "MorphologicalOperation.h"
//#include "depthmap_segmentation.h"

#include "../Common/debug.h"
#include "Peabody.h"

#include "voxel_data_io.h"
#include "TimeUtil.h"
#include "FusionConfig.h"
#include "data_format.h"

namespace {
	const size_t BUFFER_RING_COUNT_TEMP = 1;
	const size_t BUFFER_RING_COUNT_STORAGE = 20;
	const size_t BUFFER_RING_COUNT = BUFFER_RING_COUNT_TEMP + BUFFER_RING_COUNT_STORAGE;

	const size_t PACKET_HEADER_SIZE = 3 * sizeof(unsigned long);// total size in bytes, some time representation
	const size_t IMAGE_HEADER_SIZE = 1 * sizeof(int);  // size in byte for image
	const size_t METADATA_SIZE = PACKET_HEADER_SIZE + 4 * IMAGE_HEADER_SIZE;
	const size_t FRONT_BUFFER_RING_COUNT = 3;

	const int DEPTH_SIZE = data_format::DEPTH_WIDTH * data_format::DEPTH_HEIGHT * data_format::DEPTH_BYTES_PER_PIXEL;
	const int COLOR_SIZE = data_format::COLOR_WIDTH * data_format::COLOR_HEIGHT * data_format::COLOR_BYTES_PER_PIXEL;


}

class DataCapture
{
public:
	virtual void operator()() = 0;

public:
	//called by the other threads
	virtual bool grab_next_frame(std::vector<cv::Mat>& depthImgs, std::vector<cv::Mat>& colorImgs, std::vector<int>& colorSizeToCopy) = 0;

};

inline
std::string get_user_name()
{
	DWORD const buffer_size = 1024;
	char user_name[buffer_size];

	DWORD tmp = buffer_size;
	GetUserNameA(user_name, &tmp);

	return user_name;
}

inline 	//helper for constructing openCV types based on numebr of bits we need
unsigned int GetCVDepthMaskFromBits(unsigned int nBits)
{
	switch (nBits)
	{
	case 8:
		return CV_8U;
	case 16:
		return CV_16U;
	case 32:
		return CV_32F; // used for 4 byte per pixel color atm
	default:
		throw "unknown bit depth";
	}
}

class DataCaptureFileMultiView : public GlobalDataStatic
{
public:
	DataCaptureFileMultiView() {}
public:
	void operator()()
	{
		LOGGER()->info("[Capture] Starting data capture thread!");
		std::string data_dir = DEFAULT_DEPTH_BINS_FOLDER;
		char name[500];

		auto conf = FusionConfig::current();
		if (!conf)
		{
			*LOGGER() <<Logger::Error << "ERROR.  Config file not set.  Aborting." << Logger::endl;
			return;
		}
		conf->SetErrorIfNameNotFound(true);

		data_dir = conf->GetValueWithDefault("Fusion", "OfflineDataDirectory", DEFAULT_DEPTH_BINS_FOLDER);

		bool bUsePNGForDepth = conf->GetValueWithDefault("Fusion", "usePNGAsDepth", false);
		bool bUsePNGForColor = conf->GetValueWithDefault("Fusion", "usePNGAsColor", false);
		int depth_num = conf->GetValueWithDefault<int>("DepthGeneration", "DepthCameraCount", DEPTH_CAMERAS_NUM);
		int depth_width_ori = conf->GetValueWithDefault<int>("ColorProcessing", "ColorImageWidth", DEFAULT_COLOR_WIDTH);
		int depth_height_ori = conf->GetValueWithDefault<int>("ColorProcessing", "ColorImageHeight", DEFAULT_COLOR_HEIGHT);
		int depth_width = conf->GetValueWithDefault<int>("DepthGeneration", "DepthImageWidth", DEFAULT_DEPTH_WIDTH);
		int depth_height = conf->GetValueWithDefault<int>("DepthGeneration", "DepthImageHeight", DEFAULT_DEPTH_HEIGHT);

		frmIdx_1st = conf->GetValueWithDefault<int>("Fusion", "frame_start", 1);
		frmIdx_end = frmIdx_1st + conf->GetValueWithDefault<int>("Fusion", "frames_num", DEFAULT_FRAMES_NUM) - 1;
		int frmIdx_start = frmIdx_1st;

		int frmIdx = frmIdx_start;

		frame_count = 0;
		while (true)
		{
			//load depth and color
			std::vector<cv::Mat> depthImgs;
			std::vector<cv::Mat> clrImgs;
			std::vector<int> clrSizes;
			for (int i = 0; i < depth_num; i++)
			{
				cv::Mat depthImg = cv::Mat();
				if (bUsePNGForDepth)
				{
					sprintf(name, "%s/cam%02d_depth/frames/%04d.png", data_dir.c_str(), i, frmIdx);
					depthImg = loadDepthImageFromPNG(name, false);
				}
				else
				{
					sprintf(name, "%s/cam%02d_depth/frames/%04d.bin", data_dir.c_str(), i, frmIdx);
					depthImg = read_depth_bin_sean_as_img(name, depth_width, depth_height);
				}
				//quick check on depth dimensions, and pad it if we need to
				if (depthImg.size().width < data_format::DEPTH_WIDTH ||
					depthImg.size().height < data_format::DEPTH_HEIGHT)
				{

					int yOff = (data_format::DEPTH_HEIGHT - depthImg.size().height) / 2;
					int xOff = (data_format::DEPTH_WIDTH - depthImg.size().width) / 2;
					cv::copyMakeBorder(depthImg, depthImg, yOff, yOff, xOff, xOff, cv::BORDER_CONSTANT);
				}

				depthImgs.push_back(depthImg);

				//establish base path for color files
				sprintf(name, "%s/cam%02d/frames/%04d", data_dir.c_str(), i, frmIdx);

				if (!bUsePNGForColor)
				{
					sprintf(name, "%s.colorBin", name);
					int size = 0;
					cv::Mat clrImg = read_color_bin(name, data_format::COLOR_WIDTH, data_format::COLOR_HEIGHT, size);
					clrImgs.push_back(clrImg);
					clrSizes.push_back(size);
				}
				else
				{
					sprintf(name, "%s.png", name);
					cv::Mat clrImg = cv::imread(name);

					//this is dependent on the format of the bmp file we save. this assumes 8 bit depth and 3 channels per pixel
					cv::Mat clrImg_small = cv::Mat(data_format::COLOR_HEIGHT, data_format::COLOR_WIDTH, CV_8UC3);
					cv::resize(clrImg, clrImg_small, clrImg_small.size());
					clrImg.release();
					clrImgs.push_back(clrImg_small);
					clrSizes.push_back(data_format::COLOR_HEIGHT * data_format::COLOR_WIDTH * data_format::COLOR_BYTES_PER_PIXEL);
				}


			}

			LOGGER()->info("[capture]: data captured %d", frame_count);
			//push data into queue
			{
				std::lock_guard<std::mutex> data_lock(mutex_data);
				depthImgs_queue.push_back(depthImgs);
				colorImgs_queue.push_back(clrImgs);
				color_size_queue.push_back(clrSizes);
			}

			frmIdx++;
			frame_count++;

			if (frmIdx > frmIdx_end)
			{
				bool bEmpty = false;
				do {
					std::lock_guard<std::mutex> data_lock(mutex_data);
					bEmpty = depthImgs_queue.empty();
				} while (!bEmpty);

				frmIdx = frmIdx_start + 1;

				LOGGER()->info("->->->RESTART THE SEQUENCE!!!!!!!!!!!!!!");
			}
		}
	}



public:
	//called by the other threads
	static bool load_complete();
	static bool grab_next_frame(std::vector<cv::Mat>& depthImgs, std::vector<cv::Mat>& colorImgs, std::vector<int>& colorSizeToCopy)
	{
		std::lock_guard<std::mutex> data_lock(mutex_data);
		if (!depthImgs_queue.empty())
		{
			depthImgs = depthImgs_queue.front();
			colorImgs = colorImgs_queue.front();

			depthImgs_queue.pop_front();
			colorImgs_queue.pop_front();

			if (!color_size_queue.empty())
			{
				colorSizeToCopy = color_size_queue.front();
				color_size_queue.pop_front();
			}
			else
			{
				//if we dont know, fall back to expected size
				for (int i = 0; i < colorImgs.size(); ++i)
				{
					colorSizeToCopy.push_back(COLOR_SIZE);
				}
			}

			return true;
		}
		else
		{
			return false;
		}
	}

	static std::deque<std::vector<cv::Mat>> depthImgs_queue;
	static std::deque<std::vector<cv::Mat>> colorImgs_queue;
	static std::deque<std::vector<int>> color_size_queue;
private:
	static std::mutex mutex_data;
public:
	static int frmIdx_1st;
	static int frmIdx_end;
private:
	int frame_count;
};

class network_loader;

struct capture_pod_data_frame
{
	char* buffer = nullptr;
	size_t buffer_size = 0;
};

struct rig_data_frame
{
	int						id = 0;
	int						frame = -1;
	prec_time::timepoint	timestamp;

	std::vector<char*>		raw_buffers;
	std::vector<size_t>		raw_sizes;

	std::vector<capture_pod_data_frame> pod_frames;

	// These are useul reference pointers into pod_frames, which refers into raw_buffers, 
	// which is allocated outside of rig_data_frame, and assigned to this struct via assign_data_pointers_POD or assign_data_pointers

	//Start references
	std::vector<cv::Mat> depth_images;
	std::vector<cv::Mat> color_images;

	std::vector<int*> audioSampleCountLocations;
	std::vector<char*> audioData;

	char* packetHeader;
	std::vector<char*> depth_imageHeaders;
	std::vector<char*> color_imageHeaders;
	//End references

	void assign_data_pointers_HOST(int podIndex, char* buffer, size_t size);
	void assign_data_pointers(int nbuffers, char** buffers, size_t* size);
	void clear_data_pointers();
};

struct AllTempCapturePackets
{
	std::vector<TempCapturePacket*> currPacketVector;
	//TempCapturepacket is a raw char* that is being block copied to rig_data_frame eventually,
	// so we should refer to its inner blocks using the same references to avoid misalignment between the two buffers
	// REFER to this if possible;
	rig_data_frame currPacketsDataPointers;
};

class DataCaptureNetworkMultiView
	: public GlobalDataStatic, public BufferAggregator
{
public:
	DataCaptureNetworkMultiView()
	{
		hosts = { DEFAULT_HOST };
		ports = { DEFAULT_PORT };

		auto conf = FusionConfig::current();
		if (conf)
		{
			auto num_hosts = conf->GetValueWithDefault<int>("DepthGeneration", "DepthCameraCount", DEPTH_CAMERAS_NUM);
			hosts.resize(num_hosts);
			ports.resize(num_hosts);
			string ipBase = conf->GetValueWithDefault("Network", "DepthPodIPBase", DEFAULT_HOST);
			for (uint i = 0; i < num_hosts; ++i) {
				hosts[i] = ipBase + std::to_string(i + 1);
				ports[i] = conf->GetValueWithDefault("Ports", "DataStreamPort", DEFAULT_PORT);
				*LOGGER() << Logger::Warning << "Setting up host[" << i << "] as " << hosts[i] << ":" << ports[i] << Logger::endl;
			}
		}
	}
public:
	void operator()();
	void start_loading();
	void shutdown();

protected:
	// For BufferAggregator
	void InitializePackets();

	virtual int FetchSourceFrames();
	virtual void WriteFunction(int dataSourceIndex, bool writeFrameNumber);
	virtual void GrabDataFunction(int index);

	std::shared_ptr<rig_data_frame> currFetchedFrame;

protected:
	//separately just dumb dumping of frames
	std::thread* t_NetworkCaptureThread; //just grabs and dumps, nothing smart here
	std::vector<std::shared_ptr<AllTempCapturePackets>> mTempCaptureRingBuffer; // ringbuffersize amount of tempcapturePackets(one for each source)
	int mTempCaptureRingBufferWriteIndex;
	int mTempCaptureRingBufferReadIndex;


public:
	typedef std::shared_ptr<rig_data_frame> rig_data_ptr;
	//called by the other threads
	bool grab_next_frame(std::vector<cv::Mat>& depthImgs, std::vector<cv::Mat>& colorImgs, std::vector<int>& colorSizeToCopy);
	void grab_next_audio_frame(std::vector<char*>& audioBuffer, std::vector<int>& sizeToCopy);
	virtual void ResetPackets();

private:
	static std::mutex mutex_data;

public:
	size_t get_buffer_required_size();

	size_t get_buffer_required_count();
	void set_buffer_storage(int nBuffers, char** buffers, size_t* sizes);

	size_t	get_front_buffer_required_count();
	void		set_front_buffer_storage(int nBuffers, char** buffers, size_t* sizes);
private:
	//parallel struct with SynchronizedPacket (which tracks other stats)
	std::vector<rig_data_ptr> mRigFrameDataBuffer;
private:
	int latest_front_frame;
	std::queue<rig_data_ptr>	rig_front_queue;
	std::map<int, rig_data_ptr> rig_active_set;


public:
	int frmIdx_1st;
	int frmIdx_end;
	bool using_color_data;

private:
	bool end_capture = false;
	std::vector<std::string> hosts;
	std::vector<std::string> ports;

	std::shared_ptr<network_loader> loader;

	std::vector<int> last_fetched_frame;

	bool mShouldStartCapturingData = false;
};

#endif