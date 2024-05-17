#include "OfflineDataSource.h"
#include "Peabody.h"

OfflineDataSource::OfflineDataSource(std::string dataFilePath, std::string dataSizeFilePath, std::string depthFolderPath):
m_CurrBufferDataIndex(0)
{
	m_DataFilePath = dataFilePath;
	m_DataSizeFilePath = dataSizeFilePath;
	m_DepthDataFolderPath = depthFolderPath;
}


OfflineDataSource::~OfflineDataSource()
{
	const int length = m_BufferedData.size();
	for (int i = 0; i < length; ++i)
	{
		delete m_BufferedData[i].m_Data;
	}
	m_BufferedData.clear();
}

int OfflineDataSource::LoadData(int maxFrameNumber)
{
	std::ifstream dataSizeFile(m_DataSizeFilePath.c_str());
	std::ifstream dataFile(m_DataFilePath.c_str(), std::ios::in | std::ios::binary);
	if (dataSizeFile.is_open() && dataFile.is_open())
	{
		dataFile.seekg(0, std::ios::beg);
		BufferDataPacket currentBufferPacket;
		std::string currentPacketSize;
		int frameCount = 0;
		//read packet size so we know how much to read from h264 file
		while (getline(dataSizeFile, currentPacketSize) && frameCount < maxFrameNumber)
		{
			int colorFrameSize = stoi(currentPacketSize);
			const int cDepthFrameSize = cDefaultDepthHeight * cDefaultDepthWidth * TRANSMITTED_RAW_DEPTH_BYTES_PER_PIXEL;
			currentBufferPacket.mDataHeader.mPacketSize = colorFrameSize + cDefaultPodCount * cDepthFrameSize;
			currentBufferPacket.mDataHeader.mDepthWidth = cDefaultDepthWidth;
			currentBufferPacket.mDataHeader.mDepthHeight = cDefaultDepthHeight;
			currentBufferPacket.mDataHeader.mPodCount = cDefaultPodCount;

			std::chrono::system_clock::duration dtn = std::chrono::system_clock::now().time_since_epoch();
			std::cout << "Loading frame with timestamp " << (long)dtn.count() << std::endl;
			currentBufferPacket.mDataHeader.timestamp = (long)dtn.count();
			currentBufferPacket.mDataHeader.checksum = 0; // TODO:  Calculate an actual checksum?


			// data = header + colorstream +depthdata
			currentBufferPacket.m_Data = new char[
				sizeof(DataPacketHeader) 
					+colorFrameSize
				+ cDefaultPodCount * cDepthFrameSize];
			memcpy(currentBufferPacket.m_Data, reinterpret_cast<char*>(&currentBufferPacket.mDataHeader), sizeof(DataPacketHeader));
			//Copy color data first
			dataFile.read(currentBufferPacket.m_Data + sizeof(DataPacketHeader), colorFrameSize);
			if (!dataFile)
			{
				std::cout << "ERROR: Can't read all data from color data file for frame "<<frameCount << std::endl;
				return false;
			}

			//then copy depth data for each pod
			char depthFileName[500];
			int offset = sizeof(DataPacketHeader)+colorFrameSize;
			for (int i = 0; i < currentBufferPacket.mDataHeader.mPodCount; ++i)
			{
				sprintf_s(depthFileName, "%s\\depthCam%d\\img%04d.bin", m_DepthDataFolderPath.c_str(), i, frameCount);
				std::ifstream depthFile(depthFileName, std::ios::in | std::ios::binary);
				depthFile.seekg(0, std::ios::beg);
				depthFile.read(currentBufferPacket.m_Data + offset, cDepthFrameSize);
				if (!depthFile)
				{
					std::cout << "ERROR: Can't read all data from detphCam" << i << " frame " << frameCount << std::endl;
					return false;
				}

				offset += cDepthFrameSize;
			}
			m_BufferedData.push_back(currentBufferPacket);
			frameCount++; 
			// print to indicate we're still reading
			if (frameCount % 50 == 0)
			{
				printf(".");
			}
		}
	}
	else
	{
		printf("Error opening offline data \n");
		return -1;
	}
	return 1;
}

bool OfflineDataSource::HasNewData()
{
	return m_CurrBufferDataIndex < m_BufferedData.size();
}

const char* OfflineDataSource::GetNewData()
{
	return m_BufferedData[m_CurrBufferDataIndex].m_Data;
}

unsigned int OfflineDataSource::GetNewDataSize()
{
	return m_BufferedData[m_CurrBufferDataIndex].mDataHeader.mPacketSize;
}

void OfflineDataSource::FinishedNewData()
{
#ifdef LOOP_DATA
	m_CurrBufferDataIndex = (m_CurrBufferDataIndex + 1) % m_BufferedData.size();
#else
	m_CurrBufferDataIndex = (m_CurrBufferDataIndex + 1);
#endif
}