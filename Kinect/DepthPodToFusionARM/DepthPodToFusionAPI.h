#pragma once
#include "Win2Linux.h"
#include <vector>

// fusion ports start at this base number and use ports FUSION_PORT_BASE to FUSION_PORT_BASE + DEPTH_CAMERAS_NUM-1
#define DEFAULT_PORT_BASE   14900
#define DEFAULT_PORT_OFFSET 0

long DepthToFusion_InitServer(
    unsigned int nStreams,
    unsigned long depthSize, unsigned short bytesPerDepthPixel,
    unsigned long colorSize, unsigned short bytesPerColorPixel,
    int& serverIdOut,
    unsigned short fusionPort = DEFAULT_PORT_BASE,
    unsigned short portOffset = DEFAULT_PORT_OFFSET);

long DepthToFusion_DestroyServer(unsigned int serverId);

long DepthToFusion_SendFrame(
    unsigned int serverId, 
    unsigned char** depthFrames, 
    unsigned char** colorFrames, 
    unsigned char** audioFrames,
    std::vector<int> colorFrameSizes, 
    unsigned long frameNumber, 
    unsigned long timestamp);