#pragma once
#include <iostream>
#include <cstdio>
#include <ctime>
#include <vector>
#include <algorithm>
#include <string>
#include <mutex>
#include <map>
#include <filesystem>
#include "Logger.h"
#include "../Common/debug.h"

struct SynchronizedPacket
{
	unsigned long mFrameNumber;
	unsigned long mTimestamp;
	bool mHasData;
	bool mIsDeadFrame; // when camera skips frame, we can no longer use this packet, so we need to know that happened
	int mNumDataFilled; // number of camera data that's been written to // whether this frame has been polled. if so, it's safe to overwrite it
	std::vector<const unsigned char*> mCameraData;
};

struct TempCapturePacket
{
	unsigned long mFrameNumber;
	unsigned long mTimestamp;
	bool mHasData;
	bool mRetained;
	union
	{
		char* mRawBuffer_Char; // fusion uses this
		const unsigned char** mRawBuffer_UChar; // DG uses this
	};


};

struct FrameStats
{
	unsigned long mLastCapturedFrameNumber;
	int mBufferIndex; // next index to write into the mBufferedPackets
	unsigned long mFirstFrameNumberPG;
	long mFrameOffset; // offset could be +/-
};

struct FrameAlignment
{
	int frameNum;
	unsigned long timestamp;
};

class BufferAggregator
{
public:
	BufferAggregator();
	virtual ~BufferAggregator() {};

	virtual void InitializePackets() { mInitialized = true; };

	virtual int FetchSourceFrames() = 0;

	virtual void WriteFunction(int dataSourceIndex, bool writeFrameNumber) = 0;

	virtual void GrabDataFunction(int index);
	int FetchNextSynchronizedFrame(const unsigned char** packetData, unsigned long* frameNumber);

	bool ComputeFrameNumberOffsetsFromTimestamps();
	int CaptureThreadUpdate();

	void MoveReadPointerToWritePointer();
	void SetSynchronizationMode(bool mode);
	void DebugPrintCurrentState(int bufferIndex);

	virtual void ResetPackets();

	bool AreBuffersOutOfSync() {
		return mOutofSyncBuffers;
	}

protected:
	bool mEstablishedEndpointOffsets;
	bool mInitialized;
	bool mOutofSyncBuffers;
	volatile bool m_bManualSynchronizationMode;

	int  mBuffersOverwritten;

	int mBufferSize;
	int mNumberDataSources;


	std::map<unsigned long, std::vector<FrameAlignment>> mFrameAlignment;
	std::vector<TempCapturePacket*> mTempCapturePackets; // one for each source
	std::vector<FrameStats*> mFrameStats; // one for each source

	std::vector<SynchronizedPacket*> mBufferedPackets;

	//for fetching
	int mLastFetchBufferIndex;

	//debuging 
	unsigned long mLastFetchFrameNumber;

	//mutex
	std::mutex mDataMutex;
};