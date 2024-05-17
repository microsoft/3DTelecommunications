#pragma once
#define _WINSOCKAPI_   

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <chrono>

#define LOOP_DATA 

struct DataPacketHeader
{
	int mPacketSize; //encoded frame size
	int mDepthWidth;
	int mDepthHeight;
	int mPodCount;  //assumes same # depth and color
	long timestamp;
	long checksum;
};

struct BufferDataPacket
{
	DataPacketHeader mDataHeader;
	char * m_Data;
};

class OfflineDataSource
{
public:
	OfflineDataSource(std::string dataFilePath, std::string dataSizeFilePath, std::string depthDataFolderPath);
	~OfflineDataSource();
	int LoadData(int maxFrameNumber = 1000);

	bool HasNewData();
	const char* GetNewData();
	unsigned int GetNewDataSize();
	void FinishedNewData();

protected:
	
	std::vector<BufferDataPacket> m_BufferedData;
	std::string m_DataFilePath;
	std::string m_DataSizeFilePath;
	std::string m_DepthDataFolderPath;

	int m_CurrBufferDataIndex;
	bool m_ShouldLoop;

	const int cDefaultDepthWidth = 1024;
	const int cDefaultDepthHeight = 1024;
	const int cDefaultPodCount = 4;
};