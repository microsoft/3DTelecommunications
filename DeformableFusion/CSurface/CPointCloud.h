//===============================================
//			CPointCloud.h
//			Mingsong Dou (doums@cs.unc.edu)
//			August, 2010
//===============================================

#pragma once
#include <stdio.h>

#define CLAMP(n, lower, upper) ( ((n) <= (lower)) ? (lower) : ( ((n) >= (upper)) ? (upper) : (n)))


template<class T>
class CPointCloud
{
public:
	CPointCloud()
		: points_num(0),
		  points(NULL),
		  normals(NULL),
		  colors(NULL)
	{}

	CPointCloud( CPointCloud<T> const& others );

	~CPointCloud()
	{
		if(points != NULL)	
			delete [] points;
		if(normals != NULL) 
			delete [] normals;
		if( colors != NULL )
			delete [] colors;
	}

public:
	CPointCloud<T>& operator=( CPointCloud<T> const& rhs );

	bool hasColor() {return this->colors != NULL;}
	bool hasNormal() {return this->normals != NULL;}

	bool writeToFileASCII(const char *filename) const;
	bool readFromFileASCII(const char *filename);
	bool writeToFileBIN(const char *filename) const;
	bool readFromFileBIN(const char *filename);
	bool readFromVoxelBIN(const char *flename);

	bool readFromFile(char const*filename)
	{
		int len = strlen(filename);
		if( len > 3 )
		{
			if (strcmp("bin", &filename[len - 3]) == 0)
				return this->readFromFileBIN(filename);
			else if (strcmp("vox", &filename[len - 3]) == 0)
				return this->readFromVoxelBIN(filename);
			else if (strcmp("txt", &filename[len - 3]) == 0)
				return this->readFromFileASCII(filename);
		}

		printf("Error<CPointCloud::readFromFile>: file name <%s> is illegal!\n", filename);
		return false;
	}

public:
	T const* vt_data(int vtIdx) const { return points+3*vtIdx; }
	T* vt_data(int vtIdx) { return points + 3 * vtIdx; }
	T const* vt_normal(int vtIdx) const { return normals ? normals + 3 * vtIdx : NULL; }
	T* vt_normal(int vtIdx) { return normals ? normals + 3 * vtIdx : NULL; }
	T const* vt_color(int vtIdx) const { return colors ? colors + 3 * vtIdx : NULL; }
	T* vt_color(int vtIdx) { return colors ? colors + 3 * vtIdx : NULL; }

public:
	int points_num;
	T* points;
	T* normals;
	T *colors;

public:
	static void RGBtoYUV(T rgb[3], T yuv[3])
	{
		yuv[0] = CLAMP(0.29899999*rgb[0] + 0.587*rgb[1] + 0.114*rgb[2], 0.0, 1.0);
		yuv[1] = CLAMP(0.50196078 - 0.1687*rgb[0] - 0.3313*rgb[1] + 0.5*rgb[2], 0.0, 1.0);
		yuv[2] = CLAMP(0.50196078 + 0.5*rgb[0] - 0.4187*rgb[1] - 0.0813*rgb[2], 0.0, 1.0);

	}

	static void YUVtoRGB(T yuv[3], T rgb[3])
	{
		rgb[0] = CLAMP(-0.703749019 + yuv[0] + 1.402*yuv[2], 0.0, 1.0);
		rgb[1] = CLAMP(0.53121505 + yuv[0] - 0.34414*yuv[1] - 0.71414*yuv[2], 0.0, 1.0);
		rgb[2] = CLAMP(-0.88947451 + yuv[0] + 1.772*yuv[1], 0.0, 1.0);
	}

public:
	void freeMemory()
	{
		if( this->points != NULL )
		{
			delete [] this->points;
			this->points = NULL;
		}
		if(this->normals != NULL )
		{
			delete [] this->normals;
			this->normals = NULL;
		}
		if(this->colors != NULL )
		{
			delete [] this->colors;
			this->colors = NULL;
		}		
	}
};

#include "CPointCloud.hpp"
