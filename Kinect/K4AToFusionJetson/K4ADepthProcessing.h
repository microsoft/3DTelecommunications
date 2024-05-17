#pragma once

#include "K4ACommon.h"


//DBL_EPSILON
#define EPSILON       1.0
#define IS_ZERO(v)    ((-EPSILON < v) && (v < EPSILON))
#define MIN_DEPTH     20.0
#define TRASH_PIXELS  10
#define MAX_STD_DEV   50

namespace K4ADepthProcess
{
    const static int g_mask[2] =
    {
        0x0000,
        0x8000
    };
 
    template <typename BuffT>
    inline double GetInterpolatedValue(double x, double y, uint32_t width, uint32_t height, const BuffT* buffer, uint16_t edge_threshold = 0, uint16_t prev_pixel_value = 0);

    template <typename BuffT>
    inline void UndistortBuffer(const SourceInfo& srcInf, const BuffT* inBuff, BuffT* outBuff);

    template <typename BuffT>
    inline void PadBuffer(const SourceInfo& srcInf, const BuffT* inBuff, BuffT* outBuff);

    template <typename DepthTIn, typename DepthTOut>
    inline void ProcessDepth(
        DepthTIn zIn,
        double avg,
        double stdDev,
        DepthTOut timeAvg,
        const SharedSettings& shrdSettings,
        const Calibration& cal,
        DepthTOut& zOut);

    template <typename BuffT>
    inline void UndistortDepthMap(
        const SourceInfo& depthInfo,
        const BuffT* inBuff,
        const double* avgBuff,
        const double* stdDevBuff,
        const BuffT* timeAvgBuff,
        const SharedSettings& shrdSettings,
        BuffT* outBuff);
    template <typename BuffT>
    inline void ErodeDepthMap(
        const SourceInfo& depthInfo,
        const BuffT* inBuff,
        std::shared_ptr<uint8_t[]> erodeHistory,
        std::vector<double> statistics,
        const unsigned int historyMask,
        const SharedSettings& shrdSettings,
        BuffT* outBuff
    );

    template <typename BuffT>
    inline void PadDepthMap(
        const SourceInfo& depthInf,
        const BuffT* inBuff,
        const double* avgBuff,
        const double* stdDevBuff,
        const SharedSettings& shrdSettings,
        BuffT* outBuff);

}

template <typename BuffT>
inline double K4ADepthProcess::GetInterpolatedValue(double x, double y, uint32_t width, uint32_t height, const BuffT* buffer, uint16_t edge_threshold, uint16_t prev_pixel_value)
{
    // bi-linear interpolation
    uint32_t x1 = (uint32_t)floor(x);
    uint32_t x2 = (uint32_t)ceil(x);
    uint32_t y1 = (uint32_t)floor(y);
    uint32_t y2 = (uint32_t)ceil(y);

    // throwing out perimeter 2 pixels
    if ((x1 < TRASH_PIXELS) || (x1 >= (width - TRASH_PIXELS)) ||
        (x2 < TRASH_PIXELS) || (x2 >= (width - TRASH_PIXELS)) ||
        (y1 < TRASH_PIXELS) || (y1 >= (height - TRASH_PIXELS)) ||
        (y2 < TRASH_PIXELS) || (y2 >= (height - TRASH_PIXELS)))
    {
        return 0;
    }

    uint32_t row1 = y1 * width;
    uint32_t row2 = y2 * width;

    double a = (double)buffer[row1 + x1];
    double b = (double)buffer[row1 + x2];
    double c = (double)buffer[row2 + x1];
    double d = (double)buffer[row2 + x2];

    // find the two closest surfaces
    // assume that most of the time there's just a single surface across the 4 pixels
    uint32_t au = (uint32_t)buffer[row1 + x1] - 1;
    uint32_t bu = (uint32_t)buffer[row1 + x2] - 1;
    uint32_t cu = (uint32_t)buffer[row2 + x1] - 1;
    uint32_t du = (uint32_t)buffer[row2 + x2] - 1;

    // I seach of a,b,c, and d a member of the closest surface?
    bool a0, b0, c0, d0;

    // calculate distance to the closest (z) pixel
    uint32_t closest = au;
    closest = bu < closest ? bu : closest;
    closest = cu < closest ? cu : closest;
    closest = du < closest ? du : closest;

    // Calculate membership in the closest surface
    a0 = au <= closest + edge_threshold;
    b0 = bu <= closest + edge_threshold;
    c0 = cu <= closest + edge_threshold;
    d0 = du <= closest + edge_threshold;

    // calculate interpolation contributions
    double xFrac = x - (double)x1;
    double yFrac = y - (double)y1;

    // calculate interpolated values for the closest surface
    double ab0 = !a0 ? b : (!b0 ? a : (xFrac * (b - a) + a));
    double cd0 = !c0 ? d : (!d0 ? c : (xFrac * (d - c) + c));
    double v0 = !a0 && !b0 ? cd0 : (!c0 && !d0 ? ab0 : (yFrac * (cd0 - ab0) +  ab0));

    double v1 = v0; //base case, only one surface

    // If there is more than one surface among the 4 pixels (note that 0 or no distance is considered a surface), calulate its value
    if(!(a0 && b0 && c0 && d0))
    {
        // Is each of a,b,c, and d a member of th enext closest surface?
        bool a1, b1, c1, d1;

        // calculate the distance to the closest pixel not in the closest surface
        uint32_t next_closest = -1;
        next_closest = au < next_closest && !a0 ? au : next_closest;
        next_closest = bu < next_closest && !b0 ? bu : next_closest;
        next_closest = cu < next_closest && !c0 ? cu : next_closest;
        next_closest = du < next_closest && !d0 ? du : next_closest;

        // calculate membership in the next closest surface
        a1 = !a0 && au <= next_closest + edge_threshold;
        b1 = !b0 && bu <= next_closest + edge_threshold;
        c1 = !c0 && cu <= next_closest + edge_threshold;
        d1 = !d0 && du <= next_closest + edge_threshold;

        // calculate interpolated values for the next closest surface
        double ab1 = !a1 ? b : (!b1 ? a : (xFrac * (b - a) + a));
        double cd1 = !c1 ? d : (!d1 ? c : (xFrac * (d - c) + c));
        v1 = !a1 && !b1 ? cd1 : (!c1 && !d1 ? ab1 : (yFrac * (cd1 - ab1) + ab1));
    }

    // Here's the temporal piece.  We want to avoid epth oscillation on object boundaries.  We return the interpolated value of the closest surface
    // unless th next closest surface appears to ahve been used in the prior frame, in which case we stick with that.
    return (abs(prev_pixel_value - v1) <= edge_threshold ? v1 : v0);
}


template <typename BuffT>
inline void K4ADepthProcess::UndistortBuffer(const SourceInfo& srcInf, const BuffT* inBuff, BuffT* outBuff)
{
#pragma omp parallel for
    for (unsigned int i = 0; i < srcInf.outHeight; ++i)
    {
        const float* mapXRowI = srcInf.mapX.ptr<float>(i);
        const float* mapYRowI = srcInf.mapY.ptr<float>(i);
        uint32_t dstRowIdx = i * srcInf.outWidth;
        for (unsigned int j = 0; j < srcInf.outWidth; ++j)
        {
            uint32_t dstIdx = dstRowIdx + j;
            float x = mapXRowI[j] - srcInf.leftOffset;
            float y = mapYRowI[j] - srcInf.topOffset;

            double zVal = GetInterpolatedValue<BuffT>(x, y, srcInf.srcWidth, srcInf.srcHeight, inBuff) * srcInf.cal.zScale + srcInf.cal.zShift;
            outBuff[dstIdx] = (BuffT)zVal;
        }
    }
}


template <typename BuffT>
inline void K4ADepthProcess::PadBuffer(const SourceInfo& srcInf, const BuffT* inBuff, BuffT* outBuff)
{
#pragma omp parallel for
    for (unsigned int i = 0; i < srcInf.srcHeight; ++i)
    {
        uint32_t dstRowIdx = (i + srcInf.topOffset) * srcInf.outWidth;
        uint32_t srcRowIdx = i * srcInf.srcWidth;
        
        for (unsigned int j = 0; j < srcInf.srcWidth; ++j)
        {
            uint32_t dstIdx = dstRowIdx + (j + srcInf.leftOffset);
            outBuff[dstIdx] = inBuff[srcRowIdx + j];
        }
    }
}


template <typename DepthTIn, typename DepthTOut>
inline void K4ADepthProcess::ProcessDepth(
    DepthTIn zIn,
    double avg,
    double stdDev,
    DepthTOut timeAvg,
    const SharedSettings& shrdSettings,
    const Calibration& cal,
    DepthTOut& zOut)
{
    // +/- (3 * stddev) = 99%, only care about things closer than BG so keep if diff < -3*stddev
    double correctedZVal = (double)zIn * cal.zScale + cal.zShift;
    timeAvg = timeAvg & 0x7FFF;
    if(shrdSettings.temporalSmoothingThreshold > 0)
    {
        correctedZVal = std::abs(correctedZVal - (double)timeAvg) < shrdSettings.temporalSmoothingThreshold ? (double)timeAvg : correctedZVal;
    }
    bool invalidBG = IS_ZERO(avg) || (avg < shrdSettings.minDepth) || (stdDev > MAX_STD_DEV);
    bool withinStdDev = ((correctedZVal - avg) + (shrdSettings.numStdDevFromBG * stdDev)) < EPSILON;
    bool withinRange = (correctedZVal > shrdSettings.minDepth) && ((correctedZVal - shrdSettings.maxDepth) < EPSILON);
    bool isFG = withinRange && (invalidBG || withinStdDev);
    if(isFG)
        zOut = 0x8000 | (DepthTOut)round(correctedZVal);
    else 
        zOut = 0;
}


template <typename BuffT>
inline void K4ADepthProcess::UndistortDepthMap(
    const SourceInfo& depthInfo,
    const BuffT* inBuff,
    const double* avgBuff,
    const double* stdDevBuff,
    const BuffT* timeAvgBuff,
    const SharedSettings& shrdSettings,
    BuffT* outBuff)
{
    #pragma omp parallel for
    for (unsigned int i = 0; i < depthInfo.outHeight; ++i)
    {     
        const float* mapXRowI = depthInfo.mapX.ptr<float>(i);
        const float* mapYRowI = depthInfo.mapY.ptr<float>(i);

        for (unsigned int j = 0; j < depthInfo.outWidth; ++j)
        {            
            float x = mapXRowI[j] - depthInfo.leftOffset;
            float y = mapYRowI[j] - depthInfo.topOffset;
            uint32_t dstIdx = i * depthInfo.outWidth + j;
	    
            double zVal = GetInterpolatedValue<uint16_t>(x, y, depthInfo.srcWidth, depthInfo.srcHeight, inBuff, shrdSettings.interpolationEdgeThreshold, timeAvgBuff[dstIdx]);
            if (IS_ZERO(zVal))
            {
                outBuff[dstIdx] = 0;
                continue;
            }
            ProcessDepth<double, BuffT>(zVal, avgBuff[dstIdx], stdDevBuff[dstIdx], timeAvgBuff[dstIdx], shrdSettings, depthInfo.cal, outBuff[dstIdx]);
        }
    }
}

template <typename BuffT>
inline void K4ADepthProcess::ErodeDepthMap(
    const SourceInfo& depthInfo,
    const BuffT* inBuff,
    std::shared_ptr<uint8_t[]> erodeHistory,
    std::vector<double> statistics,
    const unsigned int historyMask,
    const SharedSettings& shrdSettings,
    BuffT* outBuff
)
{    
    int currentForegroundCount = 0;
    int steadyForegroundCount = 0;
    int steadyBackgroundCount = 0;
    int fluctuatingCount = 0;
    #pragma omp parallel for default(shared) reduction (+: fluctuatingCount, currentForegroundCount, steadyForegroundCount, steadyBackgroundCount)
    for (unsigned int i = shrdSettings.depthErosionSearchRadius; i < depthInfo.outHeight - shrdSettings.depthErosionSearchRadius; ++i)
    {
        for (unsigned int j = shrdSettings.depthErosionSearchRadius; j < depthInfo.outWidth - shrdSettings.depthErosionSearchRadius; ++j)
        {    
            // Current-frame Erosion   
            int idx = i*depthInfo.outWidth + j; 
            if(inBuff[idx] != 0)
            {
                int zeroCount = 0;
                for(unsigned int ii = i-shrdSettings.depthErosionSearchRadius; ii <= i+shrdSettings.depthErosionSearchRadius; ii++)
                {
                    for(unsigned int jj = j-shrdSettings.depthErosionSearchRadius; jj <= j+shrdSettings.depthErosionSearchRadius; jj++)
                    {
                        if(inBuff[ii * depthInfo.outWidth + jj] == 0)
                            zeroCount++;
                    }
                }
                if(shrdSettings.depthErosionMaxZeros > 0 && zeroCount > shrdSettings.depthErosionMaxZeros)
                {
                    outBuff[idx] = 0;
                }
            }
            
            // Temporal Erosion
            // We still use OutBuff, because OutBuff will contain all of InBuff except those pixels which were zeroed out by the erosion code above
            if(shrdSettings.temporalErosionFrames > 0 || shrdSettings.statisticsMode)
            {
                uint16_t tempPixel = (erodeHistory[idx] & historyMask) ? 0 : outBuff[idx];
                erodeHistory[idx] = (erodeHistory[idx] << 1) | (outBuff[idx] == 0 ? 1 : 0);
                outBuff[idx] = tempPixel;

                if(shrdSettings.statisticsMode)
                {
                    if(erodeHistory[idx] != 0xFF && erodeHistory[idx] != 0)
                        fluctuatingCount++;
                    else if(erodeHistory[idx] == 0xFF)
                        steadyBackgroundCount++;
                    else if(erodeHistory[idx] == 0)
                        steadyForegroundCount++;

                    
                    if( outBuff[idx] != 0 )
                        currentForegroundCount++;
                }
            }
        }
    }
    if(shrdSettings.statisticsMode)
    {
        // reuse the statistics array top left corner for other data
        // first 8 elements show number of foreground pixels in each of the last 8 frames
        // next 8 elements show number of fluctuating pixels in each of the last 8 frames
        for(int i = statistics.size() - 1; i > 0; i--)
        {
            statistics[i] = statistics[i-1];
        }
        std::cout << currentForegroundCount << "," << steadyForegroundCount << "," << steadyBackgroundCount << "," << fluctuatingCount << std::endl;
        statistics[0] = currentForegroundCount;
        statistics[8] = fluctuatingCount;
    }
}


template <typename BuffT>
inline void K4ADepthProcess::PadDepthMap(
    const SourceInfo& depthInf,
    const BuffT* inBuff,
    const double* avgBuff,
    const double* stdDevBuff,
    const SharedSettings& shrdSettings,
    BuffT* outBuff)
{
#pragma omp parallel for
    for (unsigned int i = 0; i < depthInf.srcHeight; ++i)
    {
        uint32_t dstRowIdx = (i + depthInf.topOffset) * depthInf.outWidth;
        uint32_t srcRowIdx = i * depthInf.srcWidth;
        for (unsigned int j = 0; j < depthInf.srcWidth; ++j)
        {
            uint32_t dstIdx = dstRowIdx + (j + depthInf.leftOffset);
            uint32_t srcIdx = srcRowIdx + j;

            /*if (i < TRASH_PIXELS || i >(depthInf.srcHeight - TRASH_PIXELS) ||
                j < TRASH_PIXELS || j >(depthInf.srcWidth - TRASH_PIXELS))
            {
                outBuff[dstIdx] = 0;
                continue;
            }*/

            double zVal = (double)inBuff[srcIdx];
            if (IS_ZERO(zVal))
            {
                outBuff[dstIdx] = 0;
                continue;
            }

            ProcessDepth<double, BuffT>(zVal, avgBuff[dstIdx], stdDevBuff[dstIdx], 0, shrdSettings, depthInf.cal, outBuff[dstIdx]);
        }
    }
}
