#include "BufferAggregator.h"
#include <fstream>

// assuming going from microseconds to milliseconds (if we use 2000.0)
// This determines the size of the time bucket when we try to synchronize multiple sources. 
#define TIMESTAMP_SCALE 2000.0


BufferAggregator::BufferAggregator()
{
	mNumberDataSources = 1;
	mBufferSize = 1;
	mLastFetchBufferIndex = -1;
	mLastFetchFrameNumber = 0;
	mEstablishedEndpointOffsets = false;
	mInitialized = false;
	mOutofSyncBuffers = false;
	mBuffersOverwritten = 0;
	m_bManualSynchronizationMode = false;
}

void BufferAggregator::ResetPackets() {

	LOGGER()->warning("BufferAggregator::ResetPackets", "Resetting packets!");

	// Clearing frame aligntment to ensure that we don't keep old data from previously
	// determined buckets
	mFrameAlignment.clear();

	// Setup the temporary buffer
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		mFrameStats[i]->mFrameOffset = 0;
		mFrameStats[i]->mBufferIndex = 0;
		mFrameStats[i]->mLastCapturedFrameNumber = -1;
	}

	mLastFetchBufferIndex = -1;
	mLastFetchFrameNumber = 0;
	mEstablishedEndpointOffsets = false;
	mInitialized = false;
	mOutofSyncBuffers = false;
	mBuffersOverwritten = 0;
}


void BufferAggregator::GrabDataFunction(int index)
{
	return;
}


int BufferAggregator::FetchNextSynchronizedFrame(const unsigned char** packetData, unsigned long* frameNumber)
{
	std::lock_guard<std::mutex> guard(mDataMutex);
	int nextFrameToFetch = (mLastFetchBufferIndex + 1) % mBufferSize;

	bool fixed = false;

	while (mBufferedPackets[nextFrameToFetch]->mIsDeadFrame && mBufferedPackets[nextFrameToFetch]->mNumDataFilled == mNumberDataSources)
	{
		//skip ahead of dead frame
		mBufferedPackets[nextFrameToFetch]->mIsDeadFrame = false;
		mBufferedPackets[nextFrameToFetch]->mHasData = false;
		mBufferedPackets[nextFrameToFetch]->mNumDataFilled = 0;

		nextFrameToFetch = (nextFrameToFetch + 1) % mBufferSize;
		fixed = true;
	}

	if (mBufferedPackets[nextFrameToFetch]->mHasData && mBufferedPackets[nextFrameToFetch]->mNumDataFilled == mNumberDataSources)
	{
		//everything in this packet is ready
		//copy it
		//first part is mono; second part is color

		if (packetData == NULL)
		{
			//means we're storing in custom buffer, so we need custom fetcher
			GrabDataFunction(nextFrameToFetch);
		}
		else
		{
			for (int i = 0; i < mBufferedPackets[nextFrameToFetch]->mCameraData.size(); ++i)
			{
				packetData[i] = mBufferedPackets[nextFrameToFetch]->mCameraData[i];
			}
		}

		*frameNumber = mBufferedPackets[nextFrameToFetch]->mFrameNumber;
		mLastFetchFrameNumber = *frameNumber;

		//reset it

		mBufferedPackets[nextFrameToFetch]->mHasData = false;
		mBufferedPackets[nextFrameToFetch]->mNumDataFilled = 0;
		mLastFetchBufferIndex = nextFrameToFetch;

	}
	else
	{
		if (fixed)
		{
			mLastFetchBufferIndex = (nextFrameToFetch - 1 + mBufferSize) % mBufferSize;
		}

		*frameNumber = 0;
		return -1;
	}

	return 1;
}


bool BufferAggregator::ComputeFrameNumberOffsetsFromTimestamps()
{
	unsigned long timeWithAllSources = 0;
	bool allSourcesAtGivenTime = false;
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		if (!mTempCapturePackets[i]->mHasData)
			continue;
		unsigned long bucketedTS = floor(mTempCapturePackets[i]->mTimestamp / TIMESTAMP_SCALE);
		LOGGER()->warning("BufferAggregator::ComputeFrameNumberOffsetsFromTimestamps", "%d] % lu-->% lu, % lu\n", i, mTempCapturePackets[i]->mTimestamp, bucketedTS, mTempCapturePackets[i]->mFrameNumber);

		// initialize bucket count if first visit
		if (mFrameAlignment.find(bucketedTS) == mFrameAlignment.end())
		{
			mFrameAlignment[bucketedTS].resize(mNumberDataSources + 1, { 0, 0 });
		}

		// initialize neighbor buckets + 1 as well if needed
		if (mFrameAlignment.find(bucketedTS + 1) == mFrameAlignment.end())
		{
			mFrameAlignment[bucketedTS + 1].resize(mNumberDataSources + 1, { 0, 0 });
		}

		// initialize neighbor buckets - 1 as well if needed
		if (mFrameAlignment.find(bucketedTS - 1) == mFrameAlignment.end())
		{
			mFrameAlignment[bucketedTS - 1].resize(mNumberDataSources + 1, { 0, 0 });
		}

		// we've already received a frame for this source at this timestamp
		if (mFrameAlignment[bucketedTS][i].frameNum != 0)
		{
			LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "Received %d of the same frame #/timestamp for a single source: %d %lu\n", mFrameAlignment[bucketedTS][i].frameNum, bucketedTS, i);
			continue;
		}

		mFrameAlignment[bucketedTS][i] = { (int)mTempCapturePackets[i]->mFrameNumber, mTempCapturePackets[i]->mTimestamp };

		// Nth index for each bucket is a counter for # of cameras w/a frame w/in this ts bucket
		mFrameAlignment[bucketedTS][mNumberDataSources].frameNum++;

		// check +/- one bucket for visited
		int cntAtCurrentBucket = mFrameAlignment[bucketedTS][mNumberDataSources].frameNum;
		int cntAtNeighbors = cntAtCurrentBucket + mFrameAlignment[bucketedTS - 1][mNumberDataSources].frameNum + mFrameAlignment[bucketedTS + 1][mNumberDataSources].frameNum;
		if ((cntAtCurrentBucket == mNumberDataSources) || (cntAtNeighbors == mNumberDataSources))
		{
			allSourcesAtGivenTime = true;
			timeWithAllSources = bucketedTS;
			break;
		}
	}

	// compute offsets!!
	if (allSourcesAtGivenTime)
	{
		LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "Received frames from all sources for ts: %lu. Computing offsets\n", timeWithAllSources);
		if (timeWithAllSources == 0)
		{
			LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "  Time with all sources is 0, defaulting to 0 frame offset\n");
		}

		// pick largest frame number so all have positive offsets        
		int maxFrameNum = -1;
		int maxFrameNumIdx = -1;
		unsigned long maxFrameTSBucket = 0;
		for (int i = 0; i < mNumberDataSources; ++i)
		{
			FrameAlignment& curTSFrame = mFrameAlignment[timeWithAllSources][i];
			if (curTSFrame.frameNum > maxFrameNum)
			{
				maxFrameNum = curTSFrame.frameNum;
				maxFrameNumIdx = i;
				maxFrameTSBucket = timeWithAllSources;
			}

			FrameAlignment& prevTSFrame = mFrameAlignment[timeWithAllSources - 1][i];
			if (prevTSFrame.frameNum > maxFrameNum)
			{
				maxFrameNum = prevTSFrame.frameNum;
				maxFrameNumIdx = i;
				maxFrameTSBucket = timeWithAllSources - 1;
			}

			FrameAlignment& nextTSFrame = mFrameAlignment[timeWithAllSources + 1][i];
			if (nextTSFrame.frameNum > maxFrameNum)
			{
				maxFrameNum = nextTSFrame.frameNum;
				maxFrameNumIdx = i;
				maxFrameTSBucket = timeWithAllSources + 1;
			}
		}

		FrameAlignment& referenceFrame = mFrameAlignment[maxFrameTSBucket][maxFrameNumIdx];

		// skipping first source (using its frame number as reference)
		double avgTimeDelta = 0.0;
		for (int i = 0; i < mNumberDataSources; ++i)
		{
			// this will be 0 if sources are ignoring timestamp
			if (timeWithAllSources == 0)
			{
				mFrameStats[i]->mFrameOffset = 0;
				continue;
			}

			// which bucket did this frame contribute to? (there should only be one for a camera given our buckets are 1 ms wide)
			int nearestTSOffset = (mFrameAlignment[timeWithAllSources][i].frameNum != 0) ? 0 : // current bucket
				((mFrameAlignment[timeWithAllSources - 1][i].frameNum != 0) ? -1 : // prev bucket
					((mFrameAlignment[timeWithAllSources + 1][i].frameNum != 0) ? 1 : // next bucket
						2));
			if (nearestTSOffset == 2)
			{
				LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "FOUND NO NEAREST OFFSET FOR CAM: %d\n", i);
				return false;
			}

			unsigned long selectedTSBucket = timeWithAllSources + nearestTSOffset;
			FrameAlignment& matchedFrame = mFrameAlignment[selectedTSBucket][i];
			double timeDelta = ((long long)referenceFrame.timestamp - (long long)matchedFrame.timestamp);
			long frameOffset = (referenceFrame.frameNum - matchedFrame.frameNum);
			mFrameStats[i]->mFrameOffset = frameOffset;
			LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "  %d] frame offset: %d, time delta: %f\n", i, mFrameStats[i]->mFrameOffset, timeDelta);

			avgTimeDelta += abs(timeDelta);
		}

		avgTimeDelta /= mNumberDataSources;
		LOGGER()->warning("ComputeFrameNumberOffsetsFromTimestamps", "  Average time delta: %f us\n", avgTimeDelta);

		return true;
	}

	return false;
}
void BufferAggregator::SetSynchronizationMode(bool mode)
{
	m_bManualSynchronizationMode = mode;
}

int BufferAggregator::CaptureThreadUpdate()
{
	if (!mInitialized)
	{
		return -1;
	}

	//aggregate data from the different capture threads
	int result = FetchSourceFrames();

	auto minMaxElement = std::minmax_element(mTempCapturePackets.begin(), mTempCapturePackets.end(),
		[](TempCapturePacket* a, TempCapturePacket* b) { return a->mFrameNumber < b->mFrameNumber; });
	unsigned long maxFrameNum = mTempCapturePackets[minMaxElement.second - mTempCapturePackets.begin()]->mFrameNumber;

	if (result < 0 || maxFrameNum == 0)
	{
		//no valid frame grabbed from any source
		return -1;
	}



	// determine offsets for each source based on provided framenumber and timestamp
	// only need to do this at the beginning
	if (!mEstablishedEndpointOffsets)
	{
		mEstablishedEndpointOffsets = ComputeFrameNumberOffsetsFromTimestamps();
		//only exit if we havent established. otherwise, use the frame
		if (!mEstablishedEndpointOffsets && !m_bManualSynchronizationMode)
		{
			//clean up tempcaptureframes
			for (int i = 0; i < mNumberDataSources; ++i)
			{
				mTempCapturePackets[i]->mHasData = false; // clean it up
			}
			return -1;
		}
	}

	//iterate over every datasource
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		WriteFunction(i, true);
	}//end if lastframe > -1 check


	//clean up tempcaptureframes
	for (int i = 0; i < mNumberDataSources; ++i)
	{
		mTempCapturePackets[i]->mHasData = false; // clean it up
	}

	return 1;

}

//print to stdout, the current inner states of BufferAggregator
inline void BufferAggregator::DebugPrintCurrentState(int bufferIndex)
{
#ifdef DEBUG_PRINT_FRAMENUM
	if (LOGGER()->check_verbosity(Logger::Debug)) {
		//PRINT CURRENT STATE OF THINGS
		for (int x = 0; x < mNumberDataSources; ++x)
		{
			*LOGGER() << Logger::Debug << "FrameStat: [" << x << "]: bufferIndex=" << mFrameStats[x]->mBufferIndex
				<< "FirstFrameNumPG=" << mFrameStats[x]->mFirstFrameNumberPG
				<< "FrameOffset=" << mFrameStats[x]->mFrameOffset
				<< "LastCapturedFrame#=" << mFrameStats[x]->mLastCapturedFrameNumber << Logger::endl;
		}
		*LOGGER() << Logger::Debug << "BufferPacket[" << bufferIndex << "]: datafilled=" << mBufferedPackets[bufferIndex]->mNumDataFilled << Logger::endl;

		for (int x = 0; x < mNumberDataSources; ++x)
		{
			*LOGGER() << Logger::Debug << "TempCapPacket: [" << x << "]: Frame#= " << mTempCapturePackets[x]->mFrameNumber << Logger::endl;
		}
	}
#endif
}

void BufferAggregator::MoveReadPointerToWritePointer()
{
	//Note: special case for fusion. We can skip ahead with no worries, since there's no other bufferaggregators further down the line (otherwise we might end up in a "never-sync" situation)

	//skip this frame; 
	// TODO: then move pointer half way between consumer and producer
	// OR    move directly behind producing pointer (current implementation)

	auto minMaxElement = std::minmax_element(mFrameStats.begin(), mFrameStats.end(),
		[](FrameStats* a, FrameStats* b) { return a->mLastCapturedFrameNumber < b->mLastCapturedFrameNumber; });
	int minSyncedBufferIndex = mFrameStats[minMaxElement.first - mFrameStats.begin()]->mBufferIndex;

	std::lock_guard<std::mutex> guard(mDataMutex);
	int updatedIndex = (minSyncedBufferIndex - 1 + mBufferSize) % mBufferSize;

	//need to clear the packetes in between
	if (mLastFetchBufferIndex < updatedIndex)
	{
		for (int i = mLastFetchBufferIndex; i <= updatedIndex; ++i)
		{
			//clear the packets
			mBufferedPackets[i]->mIsDeadFrame = false;
			mBufferedPackets[i]->mHasData = false;
			mBufferedPackets[i]->mNumDataFilled = 0;
		}
	}
	else
	{
		for (int i = 0; i <= updatedIndex; ++i)
		{
			//clear the packets
			mBufferedPackets[i]->mIsDeadFrame = false;
			mBufferedPackets[i]->mHasData = false;
			mBufferedPackets[i]->mNumDataFilled = 0;
		}

		for (int i = mLastFetchBufferIndex + 1; i < mBufferSize; ++i)
		{
			//clear the packets
			mBufferedPackets[i]->mIsDeadFrame = false;
			mBufferedPackets[i]->mHasData = false;
			mBufferedPackets[i]->mNumDataFilled = 0;
		}
	}
	mLastFetchBufferIndex = updatedIndex;
	mLastFetchFrameNumber = mBufferedPackets[updatedIndex]->mFrameNumber;

}