///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once
#include "../Win2Linux.h"
#include "ICompressor.h"
#include "StreamsDef.h"
#include <memory>
#include <vector>
#include <mutex>
#include <thread>

struct SwapBuffer
{
	DWORD frameNumber;
	DWORD packetSize;
	BYTE* packets;
	std::mutex inUse;
};

class DecodedCompressor :
	public ICompressor
{
public:
	DWORD GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) override;
	void ReleaseLastFetchedPacket(DWORD swapBufferIndex) override;
	HRESULT Compress(
		USHORT** pDepthData, 
		BYTE** pColorData, 
		USHORT** pAudioData,
		DWORD framenumber, 
		DWORD timestamp, 
		DWORD depthSize, 
		std::vector<int> colorSizes, 	
		DWORD &completeBufferSize);

	DecodedCompressor();
	~DecodedCompressor();

    HRESULT InitStreams(const StreamsDef& streamDef);

	// For CWorkerThread
	HRESULT Initialize();
	HRESULT Uninitialize();
	HRESULT Poll();    

protected:	
    
    ULONG m_maxPacketSize;
	DWORD m_lastFrameNumber;
	DWORD m_idxCurrentFrame;	    
    StreamsDef  m_streamsDef;
	SwapBuffer* m_pSwapBuffers;

	CHAR* m_compressedDepthMap;
	CHAR* m_compressedColorMap;

	std::shared_ptr<char> m_compressedColor;
	std::shared_ptr<char> m_decompressedColor;
};
