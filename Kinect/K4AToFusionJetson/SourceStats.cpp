#include "stdafx.h"
#include "SourceStats.h"
#include "Util.h"

SourceStats::SourceStats() :
    m_numFrames(0),    
    m_numValidSamples(nullptr),
    m_avgBuf(nullptr),
    m_stdDevBuf(nullptr),
    m_accumulateBuf(nullptr),
    m_accumulateXSqrdBuf(nullptr)
{

}

SourceStats::~SourceStats()
{
    if (m_accumulateBuf != nullptr)
        delete[] m_accumulateBuf;

    if (m_avgBuf != nullptr)
        delete[] m_avgBuf;

    if (m_stdDevBuf != nullptr)
        delete[] m_stdDevBuf;

    if (m_accumulateXSqrdBuf != nullptr)
        delete[] m_accumulateXSqrdBuf;

    if (m_numValidSamples != nullptr)
        delete[] m_numValidSamples;
}

void SourceStats::Initialize(uint16_t width, uint16_t height)
{
    m_width = width;
    m_height = height;

    size_t size = width * height;
    m_accumulateBuf = new double[size];
    memset(m_accumulateBuf, 0, sizeof(double) * size);

    m_avgBuf = new double[size];
    memset(m_avgBuf, 0, sizeof(double) * size);

    m_stdDevBuf = new double[size];
    memset(m_stdDevBuf, 0, sizeof(double) * size);

    m_accumulateXSqrdBuf = new double[size];
    memset(m_accumulateXSqrdBuf, 0, sizeof(double) * size);

    m_numValidSamples = new uint32_t[size];
    memset(m_numValidSamples, 0, sizeof(uint32_t) * size);
}


void SourceStats::ComputeStats()
{
    #pragma omp parallel for
    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            int idx = i * m_width + j;
            
            // if there were no valid samples, set everything to 0
            if (m_numValidSamples[idx] == 0)
            {
                m_avgBuf[idx] = m_stdDevBuf[idx] = 0;
                continue;
            }

            // otherwise compute avg and std dev based on number of valid samples
            double avg = (m_accumulateBuf[idx] / m_numValidSamples[idx]);
            m_avgBuf[idx] = avg;

            // compute standard 
            double stdDev = sqrt(m_accumulateXSqrdBuf[idx] / m_numValidSamples[idx] - avg * avg);
            m_stdDevBuf[idx] = stdDev;
        }
    }    
}


const double* SourceStats::GetStdBuf()
{
    return m_stdDevBuf;
}


const double* SourceStats::GetAvgBuf()
{
    return m_avgBuf;
}


bool SourceStats::SaveToDisk(const char* path, const char* prefix)
{
    char fileName[250];
    sprintf(fileName, "%s/%s_bg_avg.bin", path, prefix);
    std::cout << "Saving background to " << fileName << std::endl;
    if (!K4AToFusionUtils::SaveBufferToBin<double>(fileName, m_height, m_width, m_avgBuf))
        return false;

    sprintf(fileName, "%s/%s_bg_stdev.bin", path, prefix);
    std::cout << "Saving background standard deviation to " << fileName << std::endl;
    if (!K4AToFusionUtils::SaveBufferToBin<double>(fileName, m_height, m_width, m_stdDevBuf))
        return false;

    return true;
}


bool SourceStats::LoadFromDisk(const char* path, const char* prefix)
{
    char fileName[250];
    sprintf(fileName, "%s/%s_bg_avg.bin", path, prefix);
    std::cout << "Loading background from " << fileName << std::endl;

    if (!K4AToFusionUtils::LoadBufferFromBin<double>(fileName, m_height, m_width, m_avgBuf))
        return false;

    sprintf(fileName, "%s/%s_bg_stdev.bin", path, prefix);
    std::cout << "Loading background standard deviation from " << fileName << std::endl;
    if (!K4AToFusionUtils::LoadBufferFromBin<double>(fileName, m_height, m_width, m_stdDevBuf))
        return false;

    return true;
}
