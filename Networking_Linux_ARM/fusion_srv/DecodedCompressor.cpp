// Includes
#include <iostream>
#include <fstream>
#include <memory.h>
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
    ULONG maxDepthSize = streamDef.nStreams * (sizeof(int) + (streamDef.depthSize * streamDef.bytesPerDepthPixel));
    ULONG maxColorSize = streamDef.nStreams * (sizeof(int) + (streamDef.colorSize * streamDef.bytesPerColorPixel));
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
HRESULT DecodedCompressor::Compress(
	USHORT** pDepthData, 
	BYTE** pColorData, 
	USHORT** pAudioData,
	DWORD framenumber, 
	DWORD timestamp, 
	DWORD depthSize, 
	std::vector<int> colorSizes, 	
	DWORD &completeBufferSize)
{
	HRESULT hr = S_OK;

	completeBufferSize = 0;

    DWORD idxLast = (m_idxCurrentFrame + 1) % g_nSwapBuffers;
	// Lock this buffer so it can't be used while I write to it
	//std::cout << "WL " << idxLast << std:: endl;
	std::lock_guard<std::mutex> lck(m_pSwapBuffers[idxLast].inUse);
	BYTE* pBuffer = m_pSwapBuffers[idxLast].packets;
	// ALLOCATE HEADER SPACE AT THE TOP OF THE BUFFER
	
	// first byte of header is the size.  We don't know it yet, so set to zero and advance pointer
	*(DWORD*)pBuffer = 0; // will fill in size later
	pBuffer += sizeof(DWORD);
	completeBufferSize += sizeof(DWORD);
    // second byte of header is frame number
    *(DWORD*)pBuffer = framenumber;
    pBuffer += sizeof(DWORD);
    completeBufferSize += sizeof(DWORD);
	// third byte of header is timestamp
	*(DWORD*)pBuffer = timestamp;
	pBuffer += sizeof(DWORD);
	completeBufferSize += sizeof(DWORD);

	int compressedSize = 0;

	// Concatenate the depth maps into the output packet
    for (UINT k = 0; k < m_streamsDef.nStreams; k++)
	{
    
		//compressedSize = LZ4_compress(reinterpret_cast<char*>(pDepthData[k]), m_compressedDepthMap, m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
		
		// Internal header for each depth frame
		// internal header is one byte - size of depth packet
		completeBufferSize += sizeof(int);
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}
        *(DWORD*)pBuffer = (m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel);
		pBuffer += sizeof(int);

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
					colorSizes[k], 
					colorSizes[k],
					3);				
		#else
			compressedSize = colorSizes[k];
		#endif	

		// Internal header is one byte - size of the color packet
		completeBufferSize += sizeof(int);
        if (completeBufferSize > m_maxPacketSize)
		{
			printf("ERR: Compressed packet is greater than max packet size.  Aborting\n");
			hr = E_FAIL;
			goto bailout;
		}

		*(int*)pBuffer = compressedSize; //

		pBuffer += sizeof(int);
		completeBufferSize += compressedSize;
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
			memcpy(pBuffer, pColorData[k], colorSizes[k]);
			pBuffer += colorSizes[k];
		#endif
	}

	// AUDIO
    for (UINT k = 0; k < m_streamsDef.nStreams; k++)
	{
		memcpy(pBuffer, (void*)pAudioData[k], MAX_AUDIO_PACKET_SIZE);
		completeBufferSize += MAX_AUDIO_PACKET_SIZE;
	}

	// ASSIGN PROPER SIZE VALUES TO THE HEADER
	*(DWORD*)(m_pSwapBuffers[idxLast].packets) = completeBufferSize;

	//std::cout << "[" << framenumber << "] PacketSize: " << ((int*)pBuffer)[0] << " == " << completeBufferSize << " DepthSize: " << m_streamsDef.depthSize * m_streamsDef.bytesPerDepthPixel << " ColorSize: " << colorSizes[0] << std::endl;
	m_pSwapBuffers[idxLast].frameNumber = framenumber;
	m_pSwapBuffers[idxLast].packetSize = completeBufferSize;


	m_idxCurrentFrame = idxLast;

bailout:
	//std::cout << "WU " << idxLast << std:: endl;

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
DWORD DecodedCompressor::GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber)
{
	DWORD idxLast = m_idxCurrentFrame;

	// Lock this buffer for reading so it can't be overwritten (and guarantee it's not being written-to right now)
	if(!m_pSwapBuffers[idxLast].inUse.try_lock())  // this MUST be unlocked once we're done using the packet.
	{
		return -1;
	}
	//std::cout << "[" << std::this_thread::get_id() << "]RL " << idxLast << std::endl;
	*ppData = m_pSwapBuffers[idxLast].packets;
	*pcbData = m_pSwapBuffers[idxLast].packetSize;
	*pdFrameNumber = m_pSwapBuffers[idxLast].frameNumber;

	return idxLast;
}

void DecodedCompressor::ReleaseLastFetchedPacket(DWORD swapBufferIndex)
{
	//std::cout <<  "[" << std::this_thread::get_id() << "]RU " << swapBufferIndex << std::endl;
	m_pSwapBuffers[swapBufferIndex].inUse.unlock();
}