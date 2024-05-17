#pragma once

#include "../Common/debug.h"
#include "Peabody.h"
#include "TofinoPacket.h"

class k4a_data_format
{
public:
    static const bool OLD_FORMAT = false;
    static const bool DEPTH_MASK = false;

    static const int COLOR_WIDTH = COLOR_WIDTH;
    static const int COLOR_HEIGHT = COLOR_HEIGHT;
    static const int COLOR_BYTES_PER_PIXEL = sizeof(int); /// rgba
    static const int DEPTH_WIDTH = cDepthWidth;
    static const int DEPTH_HEIGHT = cDepthHeight;
    static const int DEPTH_BYTES_PER_PIXEL = sizeof(unsigned short);
    static const int MIPMAP_LEVELS = 4;

	static const unsigned int COLOR_FORMAT = TofinoFormat::FORMAT_UBYTE | TofinoCompress::MJPEG;

	static const int LZ4_COMPRESS_FAST_LEVEL = 3; // each increase in this increases LZ4 compression speed by ~3%
	static const int AUDIO_SAMPLE_SIZE = sizeof(short); // 16 bit samples 
	static const int AUDIO_SAMPLE_RATE = 48000; // per second

    static const unsigned short DEPTH_MAX = 10000; // 10 meters
};

typedef  k4a_data_format data_format;

