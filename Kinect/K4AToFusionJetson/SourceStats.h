#pragma once

class SourceStats
{
private:
    uint32_t  m_numFrames;
    uint16_t  m_width;
    uint16_t  m_height;
    uint32_t* m_numValidSamples;

    double*   m_avgBuf;
    double*   m_stdDevBuf;
    double*   m_accumulateBuf;
    double*   m_accumulateXSqrdBuf;

public:
    SourceStats();
    ~SourceStats();
    template <class PixelT>
    void AddFrame(const PixelT* frame);
    void ComputeStats();

    void Initialize(uint16_t width, uint16_t height);

    const double* GetStdBuf();
    const double* GetAvgBuf();

    bool SaveToDisk(const char* outPath, const char* prefix);
    bool LoadFromDisk(const char* inPath, const char* prefix);

    uint32_t TotalFrames() { return m_numFrames; }
};


template <class PixelT>
void SourceStats::AddFrame(const PixelT* frame)
{
    #pragma omp parallel for
    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < m_width; ++j)
        {
            int idx = i * m_width + j;
            double val = (double)frame[idx];

            m_accumulateBuf[idx] += val;
            m_accumulateXSqrdBuf[idx] += val * val;

            // keep track of good points (0 is invalid depth)
            if (val != 0)
                m_numValidSamples[idx]++;
        }
    }
    m_numFrames++;
}