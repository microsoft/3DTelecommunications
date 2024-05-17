#include "DataCaptureThread.h"

#include <SDKDDKVer.h>
#include "asio.hpp"

#include "cuda_runtime.h"

#include "network_loader.h"
#include "TimeUtil.h"
#include "Peabody.h"


std::mutex DataCaptureFileMultiView::mutex_data;
std::deque<std::vector<cv::Mat>> DataCaptureFileMultiView::depthImgs_queue;
std::deque<std::vector<cv::Mat>> DataCaptureFileMultiView::colorImgs_queue;
std::deque<std::vector<int>> DataCaptureFileMultiView::color_size_queue;
int DataCaptureFileMultiView::frmIdx_1st = 0;
int DataCaptureFileMultiView::frmIdx_end = 0;

//network multiview
std::mutex DataCaptureNetworkMultiView::mutex_data;

void rig_data_frame::assign_data_pointers_HOST(int podIndex, char* buffer, size_t size)
{	
	packetHeader = buffer;

	pod_frames.push_back(capture_pod_data_frame());

	pod_frames[podIndex].buffer = buffer;
	pod_frames[podIndex].buffer_size = size;

	size_t depth_offset = PACKET_HEADER_SIZE;
	size_t color_offset = depth_offset + DEFAULT_PODS_PER_HOST * (IMAGE_HEADER_SIZE + DEPTH_SIZE);

	for (int j = 0; j < DEFAULT_PODS_PER_HOST; ++j)
	{
		size_t offset_dj = depth_offset + j * (IMAGE_HEADER_SIZE + DEPTH_SIZE);
		size_t offset_cj = color_offset + j * (IMAGE_HEADER_SIZE + COLOR_SIZE);

		// We are implicity pushing pod frames into the vector using a row-major order. In case we have more than one 
		// POD per host, we should be able to access them using a linearized camera index of cam_ij = podIndex * DEFAULT_PODS_PER_HOST + j;
		depth_imageHeaders.push_back(pod_frames[podIndex].buffer + offset_dj);
		color_imageHeaders.push_back(pod_frames[podIndex].buffer + offset_cj);

		depth_images.push_back(cv::Mat(
			data_format::DEPTH_HEIGHT,
			data_format::DEPTH_WIDTH,
			CV_MAKETYPE(GetCVDepthMaskFromBits(data_format::DEPTH_BYTES_PER_PIXEL * CHAR_BIT), 1), pod_frames[podIndex].buffer + offset_dj + IMAGE_HEADER_SIZE));  // extra IMAGE_HEADER_SIZE to get into the depth data
		color_images.push_back(cv::Mat(
			data_format::COLOR_HEIGHT,
			data_format::COLOR_WIDTH,
			CV_MAKETYPE(GetCVDepthMaskFromBits(data_format::COLOR_BYTES_PER_PIXEL * CHAR_BIT), 1), pod_frames[podIndex].buffer + offset_cj + IMAGE_HEADER_SIZE));  // extra IMAGE_HEADER_SIZE to get into the color data

	}
	size_t audio_offset = color_offset + DEFAULT_PODS_PER_HOST * (IMAGE_HEADER_SIZE + COLOR_SIZE);
	audioSampleCountLocations.push_back(reinterpret_cast<int*>(pod_frames[podIndex].buffer + audio_offset));
	audioData.push_back(pod_frames[podIndex].buffer + audio_offset + sizeof(int)); // the int  = sample count
}

void rig_data_frame::assign_data_pointers(int nbuffers, char** buffers, size_t* sizes) 
{
	raw_buffers = std::vector<char*>(buffers, buffers + nbuffers);
	raw_sizes = std::vector<size_t>(sizes, sizes + nbuffers);


	for (auto i = 0; i < DEPTH_CAMERAS_NUM; ++i) 
	{
		assign_data_pointers_HOST(i, buffers[i], sizes[i]);
	}
}

size_t DataCaptureNetworkMultiView::get_buffer_required_count() {
	return BUFFER_RING_COUNT * DEPTH_CAMERAS_NUM;
}

// required size for each DG connection
size_t DataCaptureNetworkMultiView::get_buffer_required_size()
{
	
	size_t potential_audio_size = 0;
	if (data_format::AUDIO_SAMPLE_SIZE > 0 && data_format::AUDIO_SAMPLE_RATE > 0)
	{
		potential_audio_size = data_format::AUDIO_SAMPLE_SIZE * data_format::AUDIO_SAMPLE_RATE + sizeof(int); // audio data + audio header that says how many samples in the audio 
	}
	return DEPTH_SIZE + COLOR_SIZE + potential_audio_size +  METADATA_SIZE;
}

void DataCaptureNetworkMultiView::set_buffer_storage(int nBuffers, char** buffers, size_t* sizes)
{
	for (int i = 0; i < BUFFER_RING_COUNT; ++i)
	{
		mRigFrameDataBuffer.push_back(std::make_shared<rig_data_frame>());

		auto& rig_i = mRigFrameDataBuffer[i];
		rig_i->id = i;
		rig_i->assign_data_pointers(DEPTH_CAMERAS_NUM, buffers + i * DEPTH_CAMERAS_NUM, sizes + i * DEPTH_CAMERAS_NUM);
	}

	InitializePackets();
}

size_t DataCaptureNetworkMultiView::get_front_buffer_required_count()
{
	return FRONT_BUFFER_RING_COUNT * DEPTH_CAMERAS_NUM;
}

void DataCaptureNetworkMultiView::set_front_buffer_storage(int nBuffers, char** buffers, size_t* sizes)
{
	for (int i = 0; i < FRONT_BUFFER_RING_COUNT; ++i)
	{
		auto rig_frame = make_shared<rig_data_frame>();
		rig_frame->id = BUFFER_RING_COUNT + i;
		rig_frame->assign_data_pointers(DEPTH_CAMERAS_NUM, buffers + i * DEPTH_CAMERAS_NUM, sizes + i * DEPTH_CAMERAS_NUM);
		rig_front_queue.push(rig_frame);
	}
}

void DataCaptureNetworkMultiView::shutdown()
{
	end_capture = true;
}

void DataCaptureNetworkMultiView::operator()()
{
	LOGGER()->info("Starting data capture thread!");

	loader = std::make_shared<network_loader>(hosts, ports);

	char name[500];

	mShouldStartCapturingData = false;


	bool bNewCapture = true;
	bool bUsePNGForDepth = true;
	bool bFreeRun = true;

	frmIdx_1st = 6;
	frmIdx_end = DEFAULT_FRAMES_NUM;
	int frmIdx_start = frmIdx_1st;

	bool allow_buffering_of_frames = false;

	using_color_data = true;

	int depth_num = DEPTH_CAMERAS_NUM;
	int depth_width = data_format::DEPTH_WIDTH;
	int depth_height = data_format::DEPTH_HEIGHT;

	int color_num = DEPTH_CAMERAS_NUM; 
	int color_width = data_format::COLOR_WIDTH;
	int color_height = data_format::COLOR_HEIGHT;

	
	char ch;
	prec_time::timepoint t3;
	while (!end_capture)
	{
		if (!mInitialized) {
			LOGGER()->high_priority("Waiting for data buffer prep");
			Sleep(50);
			continue;
		};
		std::vector<cv::Mat> depthImgs;
		std::vector<cv::Mat> clrImgs;

		prec_time::timepoint t0, t1, t2;

		{
			t0 = prec_time::now();

			auto time_ms = prec_time::as_ms(t0, t3);
			
			LOGGER()->trace("capture outside latency: %lf ms",time_ms);
			// We don't let the capture thread to be updated if out of sync buffers were detected. 
			// We expect this situation to be detected by the consumer/main thread and then wait
			// until ask us to reset our buffers
			if (mShouldStartCapturingData && !mOutofSyncBuffers)
				CaptureThreadUpdate();
			
			t3 = prec_time::now();
		}

		auto time_ms = prec_time::as_ms(t0, t3);
		
		LOGGER()->trace("Capture latency: %lfms", time_ms);

		LOGGER()->trace("[Capture]: Data captured %d  %3.4lfms (%3.4lfms/%3.4lfms/%3.4lf ms)", last_fetched_frame[0],
			prec_time::as_ms(t0, t3), prec_time::as_ms(t0, t1), prec_time::as_ms(t1, t2), prec_time::as_ms(t2, t3));
		//push data into queue
		{
			{
				{
					//swap old "queues" with new. 
					std::lock_guard<std::mutex> data_lock(mutex_data);
				}
			}
		}
	
		if (end_capture) {
			LOGGER()->trace("CAPTURE ENDED");
			break;
		}

		if (!bFreeRun)
		{
			ch = getchar();
			if (ch == 'c')
				bFreeRun = true;
		}
	}
	LOGGER()->trace("CAPTURE ENDED");
}


void DataCaptureNetworkMultiView::grab_next_audio_frame(std::vector<char*> &audioBuffer, std::vector<int>& sizeToCopy)
{
	for (int i = 0; i < currFetchedFrame->audioData.size(); ++i)
	{
		sizeToCopy.push_back((*currFetchedFrame->audioSampleCountLocations[i] ) * data_format::AUDIO_SAMPLE_SIZE);
		
	}
	audioBuffer = currFetchedFrame->audioData;
}

bool DataCaptureNetworkMultiView::grab_next_frame(std::vector<cv::Mat> &depthImgs, std::vector<cv::Mat> &colorImgs, std::vector<int> &colorSizeToCopy)
{
	mShouldStartCapturingData = true;

	unsigned long frameNumber = 0;
	int result = FetchNextSynchronizedFrame(NULL, &frameNumber);

	if (frameNumber == 0)
	{
		return false;
	}
	else
	{
		depthImgs = currFetchedFrame->depth_images;
		if (using_color_data) {
			colorImgs = currFetchedFrame->color_images;
		}
		else {
			colorImgs = std::vector<cv::Mat>(); //empty
		}

		for (int i = 0; i < currFetchedFrame->color_images.size(); ++i)
		{
			colorSizeToCopy.push_back(reinterpret_cast<int*>(currFetchedFrame->color_imageHeaders[i])[0]);
		}
		return true;
	}

}


//BufferAggregator functions
void DataCaptureNetworkMultiView::InitializePackets()
{
	mBufferSize = BUFFER_RING_COUNT_STORAGE;
	mNumberDataSources = DEPTH_CAMERAS_NUM;


	
	//setup the temporary buffer
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		TempCapturePacket* currTempPacket = new TempCapturePacket();
		currTempPacket->mFrameNumber = 0;
		//not using this buffer
		mTempCapturePackets.push_back(currTempPacket);

		FrameStats* currStat = new FrameStats();
        currStat->mFrameOffset = 0;
		currStat->mBufferIndex = 0;
		currStat->mLastCapturedFrameNumber = -1;
		mFrameStats.push_back(currStat);

		last_fetched_frame.push_back(0);
	}

	for (int i = 0; i < mBufferSize; ++i)
	{
		std::shared_ptr<AllTempCapturePackets> allTempPackets = std::make_shared<AllTempCapturePackets>();
		for (int j = 0; j < mNumberDataSources; ++j)
		{
			TempCapturePacket* currTempPacket = new TempCapturePacket();
			currTempPacket->mFrameNumber = 0;
			currTempPacket->mHasData = false;
			currTempPacket->mRawBuffer_Char = new char[get_buffer_required_size()];
			
			//not using this buffer
			allTempPackets->currPacketVector.push_back(currTempPacket);
			allTempPackets->currPacketsDataPointers.assign_data_pointers_HOST(j, currTempPacket->mRawBuffer_Char, get_buffer_required_size());
		}
		mTempCaptureRingBuffer.push_back(allTempPackets);
	}
	mTempCaptureRingBufferWriteIndex = 0;
	mTempCaptureRingBufferReadIndex = 0;
	//only using mBufferedPackets for stat tracking, not data storage
	for (int i = 0; i < mBufferSize; ++i)
	{
		SynchronizedPacket* currPacket = new SynchronizedPacket();
		currPacket->mFrameNumber = 0;
		currPacket->mHasData = false;
		currPacket->mNumDataFilled = 0;
		currPacket->mIsDeadFrame = false;
		mBufferedPackets.push_back(currPacket);
	}
	
	//flip the flag
	BufferAggregator::InitializePackets();

	t_NetworkCaptureThread = new std::thread([=]()
	{ 
		while (!end_capture)
		{
			if (!mShouldStartCapturingData)
			{
				Sleep(2);
				continue;
			}
			
			loader->read_frame_to(last_fetched_frame, DEPTH_CAMERAS_NUM, mTempCaptureRingBuffer[mTempCaptureRingBufferWriteIndex]);
			if (network_loader::checkReceiveResult() == S_OK)
			{
				//Overflow check
				bool hasDataOverall = false;
				for (int i = 0; i < mNumberDataSources; ++i)
				{
					hasDataOverall = hasDataOverall || mTempCaptureRingBuffer[mTempCaptureRingBufferWriteIndex]->currPacketVector[i]->mHasData;
				}

				if (hasDataOverall)
				{
					LOGGER()->error("NetworkCaptureThread()", "NETWORK CAPTURE!!! overflow");
				}

				//actually write the data
				for (int j = 0; j < mNumberDataSources; ++j)
				{
					mTempCaptureRingBuffer[mTempCaptureRingBufferWriteIndex]->currPacketVector[j]->mHasData = true;
				}
				mTempCaptureRingBufferWriteIndex = (mTempCaptureRingBufferWriteIndex + 1) % mBufferSize;
			}
			else
			{
				Sleep(1); // dont burn CPU
			}
		}
		

	});

}

void DataCaptureNetworkMultiView::ResetPackets() {
	LOGGER()->warning("DataCaptureNetworkMultiView::ResetPackets", "Resetting packets!");
		
	// Stopping data capture thread
	mShouldStartCapturingData = false;
	// Stopping frame sync thread
	mInitialized = false;

	std::lock_guard<std::mutex> guard(mDataMutex);

	last_fetched_frame.clear();

	//setup the temporary buffer
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		mTempCapturePackets[i]->mHasData = false;
		mTempCapturePackets[i]->mFrameNumber = 0;
		mTempCapturePackets[i]->mTimestamp = 0;

		mFrameStats[i]->mFrameOffset = 0;
		mFrameStats[i]->mBufferIndex = 0;
		mFrameStats[i]->mLastCapturedFrameNumber = -1;

		last_fetched_frame.push_back(0);
	}

	for (int i = 0; i < mBufferSize; ++i)
	{
		for (int j = 0; j < mNumberDataSources; ++j)
		{
			mTempCaptureRingBuffer[i]->currPacketVector[j]->mFrameNumber = 0;
			mTempCaptureRingBuffer[i]->currPacketVector[j]->mHasData = false;
		}
	}
	mTempCaptureRingBufferWriteIndex = 0;
	mTempCaptureRingBufferReadIndex = 0;

	//only using mBufferedPackets for stat tracking, not data storage
	for (int i = 0; i < mBufferSize; ++i)
	{
		mBufferedPackets[i]->mFrameNumber = 0;
		mBufferedPackets[i]->mHasData = false;
		mBufferedPackets[i]->mNumDataFilled = 0;
		mBufferedPackets[i]->mIsDeadFrame = false;
	}

	BufferAggregator::ResetPackets();

	mInitialized = true;
}


int DataCaptureNetworkMultiView::FetchSourceFrames()
{


	if(!mTempCaptureRingBuffer[mTempCaptureRingBufferReadIndex]->currPacketVector[0]->mHasData)
	//aggregate data from the different capture threads
	{
		return -1;
	}
	else
	{

		for (int i = 0; i < mNumberDataSources; ++i)
		{
			mTempCapturePackets[i] = mTempCaptureRingBuffer[mTempCaptureRingBufferReadIndex]->currPacketVector[i];
		}
		mTempCaptureRingBufferReadIndex = (mTempCaptureRingBufferReadIndex + 1) % mBufferSize;
		return 1;//successfully got some form of data
	}
}

void DataCaptureNetworkMultiView::WriteFunction(int dataSourceIndex, bool writeFrameNumber)
{
	std::lock_guard<std::mutex> guard(mDataMutex);
	int index = mFrameStats[dataSourceIndex]->mBufferIndex;

	//copy data into next swapbuffer

	CopyMemory(mRigFrameDataBuffer[index]->raw_buffers[dataSourceIndex], mTempCapturePackets[dataSourceIndex]->mRawBuffer_Char, get_buffer_required_size());

	mTempCapturePackets[dataSourceIndex]->mHasData = false;

	mBufferedPackets[index]->mNumDataFilled += 1;
	mBufferedPackets[index]->mHasData = true;
	mFrameStats[dataSourceIndex]->mLastCapturedFrameNumber = mTempCapturePackets[dataSourceIndex]->mFrameNumber;
	if (writeFrameNumber)
	{
		mBufferedPackets[index]->mFrameNumber = mTempCapturePackets[dataSourceIndex]->mFrameNumber;
	}
	mFrameStats[dataSourceIndex]->mBufferIndex = (mFrameStats[dataSourceIndex]->mBufferIndex + 1) % mBufferSize;
}

void DataCaptureNetworkMultiView::GrabDataFunction(int index)
{
	currFetchedFrame = mRigFrameDataBuffer[index];
}

bool DataCaptureFileMultiView::load_complete()
{
	std::lock_guard<std::mutex> data_lock(mutex_data);
	return depthImgs_queue.size() == frmIdx_end - frmIdx_1st + 1;
}


