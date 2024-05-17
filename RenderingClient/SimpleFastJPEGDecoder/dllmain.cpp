// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "turbojpeg.h"



class JPEGDecoder
{
	tjhandle _jpegDecompressor;
public:
	JPEGDecoder()
	{
		_jpegDecompressor = tjInitDecompress();
	}

	void Decompress(unsigned char* encodedDataPointer, int jpegSize, unsigned char* outputBuffer)
	{
		int jpegSubsamp, width, height;

		tjDecompressHeader2(_jpegDecompressor, encodedDataPointer, jpegSize, &width, &height, &jpegSubsamp);

		tjDecompress2(_jpegDecompressor, encodedDataPointer, jpegSize,
			outputBuffer, width,
			0/*pitch*/, height, TJPF_RGB, TJFLAG_FASTDCT);
	}

	void Destroy()
	{
		tjDestroy(_jpegDecompressor);
	}
};


#define DLLEXPORT __declspec( dllexport )

extern "C"
{
	//constructor/destructor
	DLLEXPORT void* JPEGDecoder_Create();

	DLLEXPORT void JPEGDecoder_Decompress(void* pointer, unsigned char* encodedDataPointer, int jpegSize, unsigned char* outputBuffer);

	DLLEXPORT void JPEGDecoder_Destroy(void* pointer);
}


void* JPEGDecoder_Create()
{
	return new JPEGDecoder();
}

void JPEGDecoder_Decompress(void* pointer, unsigned char* encodedDataPointer, int jpegSize, unsigned char* outputBuffer)
{
	JPEGDecoder* decoder = reinterpret_cast<JPEGDecoder *>(pointer);
	decoder->Decompress(encodedDataPointer, jpegSize, outputBuffer);
}

void JPEGDecoder_Destroy(void* pointer)
{
	JPEGDecoder* decoder = reinterpret_cast<JPEGDecoder*>(pointer);
	decoder->Destroy();
}
