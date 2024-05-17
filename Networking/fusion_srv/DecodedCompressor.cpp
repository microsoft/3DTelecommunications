// Includes
#include <iostream>
#include <fstream>
#include <memory>
#include "DecodedCompressor.h"
#include "lz4.h"

const int g_nSwapBuffers = 10;

#undef  COMPRESS_COLOR

//-------------------------------------------------------------------

DecodedCompressor::DecodedCompressor() :
m_lastFrameNumber(0),
m_idxCurrentFrame(0)
{
    m_streamsDef.nStreams = -1;
}

DecodedCompressor::~DecodedCompressor()
{
    for (DWORD i = 0; i < g_nSwapBuffers; i++)
	{
		if (NULL != m_pSwapBuffers[i].packets)
		{
			delete[] m_pSwapBuffers[i].packets;
		}
	}

	delete[] m_pSwapBuffers;
	delete[] m_compressedDepthMap;
	delete[] m_compressedColorMap;
}


HRESULT DecodedCompressor::InitStreams(const StreamsDef& streamDef)
{
    m_streamsDef = streamDef;

    // max packet size will be header (3 dwords) plus depth (MAX_DEPTH_SIZE) plus color (MAX_COLOR_SIZE) 
    ULONG maxDepthSize = streamDef.nStreams * (sizeof(INT) + (streamDef.depthSize * streamDef.bytesPerDepthPixel));
    ULONG maxColorSize = streamDef.nStreams * (sizeof(INT) + (streamDef.colorSize * streamDef.bytesPerColorPixel));
    m_maxPacketSize = maxDepthSize + maxColorSize + sizeof(DWORD) + sizeof(DWORD) + sizeof(DWORD); // "WILL BE" big enough (roughly 12.5 mb if uncompressed)

    m_compressedDepthMap = new CHAR[streamDef.depthSize * sizeof(USHORT)];
    m_compressedColorMap = new CHAR[streamDef.colorSize];

    m_pSwapBuffers = new SwapBuffer[g_nSwapBuffers];
    for (DWORD i = 0; i < g_nSwapBuffers; i++)
    {
        m_pSwapBuffers[i].packets = new BYTE[m_maxPacketSize];
        m_pSwapBuffers[i].packetSize = 0;
        m_pSwapBuffers[i].frameNumber = 0;
    }

    m_compressedColor = std::shared_ptr<char>(new char[streamDef.colorSize * streamDef.bytesPerColorPixel]);
    m_decompressedColor = std::shared_ptr<char>(new char[streamDef.colorSize]);

    return S_OK;
}


HRESULT DecodedCompressor::Initialize()
{
	HRESULT hr = S_OK;

    for (DWORD i = 0; i < g_nSwapBuffers; i++)
	{
		if (NULL == m_pSwapBuffers[i].packets)
		{
			hr = E_OUTOFMEMORY;
			goto bailout;
		}
	}

bailout:
	return hr;
}

HRESULT DecodedCompressor::Uninitialize()
{
	return S_OK;
}

/* In SHStereoMultiPod this is called by the Poll function (why did they make it Public?  I dont' know)  
   In this implementation, it is called directly because the compressor in this case isn't polling, it just works
   when this function is called 
   This function now will take in pointers to the depth data and the decoded color data,
   compress it using LZ4 and put it into the buffers swap_buffers referenced by GetLatestPacket.

   This function should be called for a single depth/RGB pair, so there should be multiple instances of this class running at the same time, each feeding their data into fusion as if from 
   a distinct pod.

   cbDataSize should be set to the size of the uncompressed data, it will be overwritten with the compressed size  upon return
   */
HRESULT DecodedCompressor::Compress(USHORT** pDepthData, BYTE** pColorData, DWORD framenumber, DWORD timestamp, DWORD depthSize, DWORD colorSize, DWORD &completeBufferSize)
{
	HRESULT hr = S_OK;

	completeBufferSize = 0;

    DWORD idxLast = (m_idxCurrentFrame + 1) % g_nSwapBuffers;
	BYTE* pBuffer = m_pSwapBuffers[idxLast].packets;
	// ALLOCATE HEADER SPACE AT THE TOP OF THE BUFFER
	
	// first byte of header is the size.  We don't know it yet, so set to zero and advance pointer
	*(UNALIGNED DWORD*)pBuffer = 0; // will fill in size later
	pBuffer += sizeof(DWORD);
	completeBufferSize += sizeof(DWORD);
    // second byte of header is frame number
    *(UNALIGNED DWORD*)pBuffer = framenumber;
    pBuffer += sizeof(DWORD);
    completeBufferSize += sizeof(DWORD);
	// third byte of header is timestamp
	*(UNALIGNED DWORD*)pBuffer = timestamp;
	pBuffer += sizeof(DWORD);
	completeBufferSize += sizeof(DWORD);

	DWORD cbBuffer = 0;
	int compressedSize = 0;

	// Concatenate the depth maps into the output packet
    for (UINT k = 0; k < m_streamsDef.nStreams; k++)
	{
    
		//compressedSize = LZ4_compress(reinterpret_cast<char*>(pDepthData[k]), m_compressedDepthMap, m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
		
		// Internal header for each depth frame
		// internal header is one byte - size of depth packet
		completeBufferSize += sizeof(INT);
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}
        *(UNALIGNED INT*)pBuffer = (m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
		pBuffer += sizeof(INT);

		// check
        completeBufferSize += (m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}

		// Copy depth data into the buffer and advance the pointer
        memcpy(pBuffer, (BYTE *)pDepthData[k], (m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel));
        pBuffer += (m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
	}

	// Compress color and concatenate to the output packet
	// LZ4 compression
    for (UINT k = 0; k < m_streamsDef.nStreams; k++)
	{
	#ifdef COMPRESS_COLOR
			compressedSize = LZ4_compress_fast(reinterpret_cast<char*>(pColorData[k]), 
				m_compressedColor.get(), 
				colorSize, 
				colorSize,
				3);
	#endif

		// Internal header is one byte - size of the color packet
		completeBufferSize += sizeof(INT);
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}

	#ifdef COMPRESS_COLOR
		*(UNALIGNED INT*)pBuffer = compressedSize; //
	#else
		*(UNALIGNED INT*)pBuffer = m_streamsDef.colorSize * m_streamsDef.bytesPerColorPixel;
	#endif

		pBuffer += sizeof(INT);

			// check 
	#ifdef COMPRESS_COLOR
		completeBufferSize += compressedSize;
	#else
        completeBufferSize += m_streamsDef.colorSize * m_streamsDef.bytesPerColorPixel;
	#endif
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}

	#ifdef COMPRESS_COLOR
		// Copy the compressed buffer into the output buffer and advance the pointer
		memcpy(pBuffer, m_compressedColor.get(), compressedSize);
		pBuffer += compressedSize;
	#else		
        memcpy(pBuffer, pColorData[k], m_streamsDef.colorSize * m_streamsDef.bytesPerColorPixel);
        pBuffer += m_streamsDef.colorSize * m_streamsDef.bytesPerColorPixel;
	#endif
	}

	// ASSIGN PROPER SIZE VALUES TO THE HEADER
	// reset the pointer to the top
	pBuffer = m_pSwapBuffers[idxLast].packets;
	*(UNALIGNED DWORD*)pBuffer = completeBufferSize;

	m_pSwapBuffers[idxLast].frameNumber = framenumber;
	m_pSwapBuffers[idxLast].packetSize = completeBufferSize;
	m_idxCurrentFrame = idxLast;

bailout:
	if (FAILED(hr))
		compressedSize = 0;
	return hr;
}

/* Main thread function, gets called by the WorkerThreadT interface 
Option 1 - Don't do anything here.  The compressor doesn't need to poll, I can just call CompressDepthData (RENAME THIS) from EncoderToFusion instead
Option 2 - Pull the data from some buffer in EncoderToFusion.  Sounds like a really dumb idea.
*/
HRESULT DecodedCompressor::Poll()
{
	HRESULT hr = S_OK;

	return hr;
}

/* This is called by TCPConnection to get the data to transmit */
HRESULT DecodedCompressor::GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber)
{
	HRESULT hr = S_OK;

	DWORD idxLast = m_idxCurrentFrame;

	*ppData = m_pSwapBuffers[idxLast].packets;
	*pcbData = m_pSwapBuffers[idxLast].packetSize;
	*pdFrameNumber = m_pSwapBuffers[idxLast].frameNumber;

	return hr;
}