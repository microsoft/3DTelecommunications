//// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
// From Fusion4D/TED

struct TofinoPacketHeader
{
	int  frame_number = 0;
	char frame_format[4];
	int  frame_format_ver = 0;
};

struct TofinoCompress {
	typedef unsigned int T;
	static const unsigned int RAW_DATA = 0x00000000;
	static const unsigned int LZ4   = 0x80000000;
	static const unsigned int RLE   = 0x40000000;
	static const unsigned int MJPEG = 0x20000000;
	static const unsigned int HIGHQUALITY = 0x10000000; // indicates this is a hq frame 

	static const unsigned int GetCompression(unsigned int fmtcmp)
	{
		return (fmtcmp & 0xffff0000);
	}
};

struct TofinoFormat
{
	typedef unsigned int T;
	static const unsigned int FORMAT_NONE = 0x0;
	static const unsigned int FORMAT_UBYTE = 0x1;
	static const unsigned int FORMAT_USHORT = 0x2;
	static const unsigned int FORMAT_FLOAT3 = 0x3;
	static const unsigned int FORMAT_INT3 = 0x4;
	static const unsigned int FORMAT_USHORT3 = 0x5;
	static const unsigned int FORMAT_BYTE3 = 0x6;
	static const unsigned int FORMAT_FLOAT6_P_N = 0x7;
	static const unsigned int FORMAT_FLOAT9_P_N_C = 0x8;
	static const unsigned int FORMAT_HALF_FLOAT6_P_N = 0x9;
	static const unsigned int FORMAT_HALF_FLOAT9_P_N_C = 0xa;

	static unsigned int GetFormat(unsigned int fmtcmp) {
		return fmtcmp & 0x0000ffff;
	}

	static size_t GetSize(unsigned int fmtcmp) {
		auto v = GetFormat(fmtcmp);
		switch (v)
		{
		case FORMAT_NONE: return 0;
		case FORMAT_UBYTE: return sizeof(unsigned char);
		case FORMAT_USHORT: return sizeof(unsigned short);
		case FORMAT_FLOAT3: return 3 * sizeof(float);
		case FORMAT_INT3: return 3 * sizeof(int);
		case FORMAT_USHORT3: return 3 * sizeof(unsigned short);
		case FORMAT_BYTE3: return 3 * sizeof(unsigned char);
		case FORMAT_FLOAT6_P_N: return 6 * sizeof(float);
		case FORMAT_FLOAT9_P_N_C: return 9 * sizeof(float);
		case FORMAT_HALF_FLOAT6_P_N: return 6 * sizeof(short);
		case FORMAT_HALF_FLOAT9_P_N_C: return 9 * sizeof(short);
		}
	}

};

struct TofinoMeshHeaderV1
{
	TofinoPacketHeader	packet_header;
	unsigned int		color_data_size;
	unsigned int		color_data_format;
	unsigned int		color_data_width;
	unsigned int		color_data_height;
	unsigned int		color_data_count;
	unsigned int		vertex_data_size; // size + bytes
	unsigned int		vertex_data_format; // COMPRESS | FORMAT
	unsigned int		vertex_count;
	unsigned int		vertex_full_size_bytes;
	unsigned int		tri_data_size; // size + bytes.  if size != index_size_bytes
	unsigned int		tri_data_format; // COMPRESS | FORMAT
	unsigned int		tri_count;
	unsigned int		tri_full_size_bytes;
};

struct TofinoMeshHeaderV2
{
	TofinoPacketHeader	packet_header;
	unsigned int		color_data_size;
	unsigned int		color_data_format;
	unsigned int		color_data_width;
	unsigned int		color_data_height;
	unsigned int		color_data_count;
	unsigned int		vertex_data_size; // size + bytes
	unsigned int		vertex_data_format; // COMPRESS | FORMAT
	unsigned int		vertex_count;
	unsigned int		vertex_full_size_bytes;
	unsigned int		tri_data_size; // size + bytes.  if size != index_size_bytes
	unsigned int		tri_data_format; // COMPRESS | FORMAT
	unsigned int		tri_count;
	unsigned int		tri_full_size_bytes;

	unsigned int	    audio_data_size;
	unsigned int		audio_data_count;
};

typedef TofinoMeshHeaderV2  MeshHeaderVersion;
