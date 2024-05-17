#include "stdafx.h"
#include "zlib_compression.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "utility.h"

bool zlib_compress(unsigned char* inbuf, unsigned long inlen, unsigned char* &outbuf, unsigned long &outlen, int level)
{
	unsigned long boundlen = compressBound(inlen);
	unsigned char* tmpbuf = new unsigned char[boundlen];

	int status = compress2(tmpbuf, &boundlen, inbuf, inlen, level);

	if( status != Z_OK )
	{
		delete [] tmpbuf;
		outlen = 0;
		outbuf = NULL;

		if( status == Z_MEM_ERROR )  printf("Error: there was not enough memory!\n");
		if( status == Z_BUF_ERROR )  printf("Error: there was not enough room in the output buffer!\n");
		if( status == Z_STREAM_ERROR )  printf("Error: the level parameter<0-9> is invalid !\n");

		return false;
	}
	
	outbuf = new unsigned char[boundlen];
	memcpy(outbuf, tmpbuf, boundlen);
	outlen = boundlen;
	
	delete [] tmpbuf;

	return true;
}

bool zlib_uncompress(unsigned char* inbuf, unsigned long inlen, unsigned char* outbuf, unsigned long &outlen)
{
	int status = uncompress(outbuf, &outlen, inbuf, inlen);
	if( status == Z_OK)
		return true;
	else
	{
		if( status == Z_MEM_ERROR )  printf("Error: there was not enough memory!\n");
		if( status == Z_BUF_ERROR )  printf("Error: there was not enough room in the output buffer!\n");
		if( status == Z_DATA_ERROR )  printf("Error: the input data was corrupted or incomplete!\n");
		return false;
	}
}
