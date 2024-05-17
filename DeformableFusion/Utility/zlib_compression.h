//===============================================
//			zlib_compression.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================

#ifndef __ZLIB_COMPRESSION_H__
#define __ZLIB_COMPRESSION_H__
#include "zlib.h"

/* outbuf should NOT be pre-allocated:
 * usage:
		char* outbuf = NULL;
		long outlen = 0;
		zlib_compress(inbuf, inlen, outbuf, outlen, level);
 */
bool zlib_compress(unsigned char* inbuf, unsigned long inlen, unsigned char* &outbuf, unsigned long &outlen, int level=6);

/* outbuf MUST be pre-allocated.
 */
bool zlib_uncompress(unsigned char* inbuf, unsigned long inlen, unsigned char* outbuf, unsigned long &outlen);


#endif