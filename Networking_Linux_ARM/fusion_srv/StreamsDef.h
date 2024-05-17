#pragma once

struct StreamsDef
{
    unsigned int   nStreams;
    unsigned long  depthSize;
    unsigned long  colorSize;   //these are set to define MAX size, color size may change if MJPG is used
    unsigned short bytesPerDepthPixel;
    unsigned short bytesPerColorPixel; //these are set to define MAX size, color size may change if MJPG is used
};

#define MAX_AUDIO_PACKET_SIZE 96004 // 48000 samples/second * 2 bytes per sample, this gives us enough space to hold one full second of audio before overflowing