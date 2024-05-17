///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once

#include "ICompressor.h"
#include "StreamsDef.h"
#include <memory>

struct SwapBuffer
{
	DWORD frameNumber;
	DWORD packetSize;
	BYTE* packets;
};

class DecodedCompressor :
	public ICompressor
{
public:
	HRESULT GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) override;
    HRESULT Compress(USHORT** pDepthData, BYTE** pColorData, DWORD framenumber, DWORD timestamp, DWORD depthSize, DWORD colorSize, DWORD &completeBufferSize);

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
