//===============================================
//			CPointCloud.hpp
//			Mingsong Dou (doums@cs.unc.edu)
//			August, 2010
//===============================================

#pragma once
#include "CPointCloud.h"
#include <string.h>
#include <exception>
#include "assert.h"

template<class T>
CPointCloud<T>::CPointCloud(CPointCloud<T> const& other)
{
	this->points_num = other.points_num;

	if( other.points != NULL && other.points_num != 0)
	{
		this->points = new T[this->points_num*3];
		memcpy(this->points, other.points, sizeof(T)*this->points_num*3);
	}
	else
		this->points = NULL;

	if( other.normals != NULL && other.points_num != 0)
	{
		this->normals = new T[this->points_num*3];
		memcpy(this->normals, other.normals, sizeof(T)*this->points_num*3);
	}
	else
		this->normals = NULL;

	if( other.colors != NULL && other.points_num != 0)
	{
		this->colors = new T[this->points_num*3];
		memcpy(this->colors, other.colors, sizeof(T)*this->points_num*3);
	}
	else
		this->colors = NULL;
}

template<class T>
CPointCloud<T>& CPointCloud<T>::operator=(CPointCloud<T> const& rhs)
{
	this->points_num = rhs.points_num;

	if(this->points != NULL )
		delete [] this->points;
	if( rhs.points != NULL && rhs.points_num != 0)
	{
		this->points = new T[this->points_num*3];
		memcpy(this->points, rhs.points, sizeof(T)*this->points_num*3);
	}
	else
		this->points = NULL;

	if(this->normals != NULL)
		delete [] this->normals;
	if( rhs.normals != NULL && rhs.points_num != 0)
	{
		this->normals = new T[this->points_num*3];
		memcpy(this->normals, rhs.normals, sizeof(T)*this->points_num*3);
	}
	else
		this->normals = NULL;

	if(this->colors != NULL)
		delete [] this->colors;
	if( rhs.colors != NULL && rhs.points_num != 0)
	{
		this->colors = new T[this->points_num*3];
		memcpy(this->colors, rhs.colors, sizeof(T)*this->points_num*3);
	}
	else
		this->colors = NULL;

	return *this;
}


template<class T>
bool CPointCloud<T>::writeToFileASCII(const char* filename) const
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "w")) != 0 )
	{
		printf("Error when Writing CPointCloud File <%s>\n", filename);
		return false;
	}

	fprintf(file, "Points Num: %d, Normal: %d, Color: %d\n", this->points_num, 
															(this->normals!=NULL)?1:0,
															(this->colors!=NULL)?1:0 );

	for(int i=0; i<this->points_num; i++)
	{
		fprintf(file, "%f %f %f", this->points[i*3], this->points[i*3+1],this->points[i*3+2]);
		if( this->normals != NULL )
			fprintf(file, " %f %f %f", this->normals[i*3], this->normals[i*3+1],this->normals[i*3+2]);
		if( this->colors != NULL )
			fprintf(file, " %f %f %f", this->colors[i*3], this->colors[i*3+1],this->colors[i*3+2]);
		fprintf(file, "\n");
	}

	fclose(file);
	return true;
}

template<class T>
bool CPointCloud<T>::readFromFileASCII(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "r")) != 0 )
	{
		printf("Error when Reading CPointCloud from File <%s>\n", filename);
		return false;
	}
	
	this->freeMemory();

	int bNormal = 0;
	int bColor = 0;
	fscanf(file, "Points Num: %d, Normal: %d, Color: %d\n", &(this->points_num), &bNormal, &bColor );

	
	points = new T[this->points_num * 3];
	if( bNormal )
		this->normals = new T[this->points_num * 3];
	else
		this->normals = NULL;
	if( bColor )
		this->colors = new T[this->points_num * 3];
	else
		this->colors = NULL;

	for(int i=0; i<this->points_num; i++)
	{
		double d1, d2, d3;
		fscanf(file, "%lf %lf %lf", &d1, &d2, &d3);
		this->points[i*3] = d1;
		this->points[i*3+1] = d2;
		this->points[i*3+2] = d3;

		if( this->normals != NULL )
		{
			fscanf(file, " %lf %lf %lf", &d1, &d2, &d3);
			this->normals[i*3] = d1;
			this->normals[i*3+1] = d2;
			this->normals[i*3+2] = d3;
		}
		if( this->colors != NULL )
		{
			fscanf(file, " %lf %lf %lf", &d1, &d2, &d3);
			this->colors[i*3] = d1;
			this->colors[i*3+1] = d2;
			this->colors[i*3+2] = d3;
		}

		fscanf(file, "\n");
	}

	fclose(file);
	return true;
}

template<class T>
bool CPointCloud<T>::writeToFileBIN(const char* filename) const
{
	int numwritten;
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing CPointCloud File <%s>\n", filename);
		return false;
	}

	char type = (sizeof(T) == 8)?'8':'4';
	fwrite(&type, 1, 1, file);
	fwrite(&this->points_num, sizeof(int), 1, file);
	int bNormal = (this->normals == NULL )? 0 : 1;
	fwrite(&bNormal, sizeof(int), 1, file);
	int bColor = (this->colors == NULL )? 0 : 1;
	fwrite(&bColor, sizeof(int), 1, file);

	if(this->points != NULL )
	{
		numwritten = fwrite(this->points, sizeof(T), this->points_num*3, file);
		assert(numwritten == this->points_num*3);
	}
	if(this->normals != NULL )
	{
		fwrite(this->normals, sizeof(T), this->points_num*3, file);
		assert(numwritten == this->points_num*3);
	}
	if(this->colors != NULL )
	{
		fwrite(this->colors, sizeof(T), this->points_num*3, file);
		assert(numwritten == this->points_num*3);
	}

	fclose(file);
	return true;
}

template<class T>
bool CPointCloud<T>::readFromFileBIN(const char* filename)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File <%s>\n", filename);
		return false;
	}

	this->freeMemory();

	char type_cur = (sizeof(T) == 8) ? '8' : '4';

	char type;
	numread = fread(&type, 1, 1, file);
	fread(&this->points_num, sizeof(int), 1, file);
	int bNormal = 0;
	fread(&bNormal, sizeof(int), 1, file);
	int bColor = 0;
	fread(&bColor, sizeof(int), 1, file);

	//read to buf
	int bytes_per_item = (type_cur == '8')? 8 : 4;
	int buf_size = bytes_per_item * this->points_num * 3;
	char *buf = new char[buf_size];
	numread = fread(buf, 1, buf_size, file);
	assert(numread == buf_size);

	//copy to this->points
	this->points = new T[this->points_num*3];
	if( type == '8' )
	{
		double *buf_d = (double*) buf;
		for(int i=0; i<points_num*3; i++)
			this->points[i] = buf_d[i];
	}
	else
	{
		float *buf_f = (float*) buf;
		for(int i=0; i<points_num*3; i++)
			this->points[i] = buf_f[i];
	}

	if(bNormal)
	{
		numread = fread(buf, 1, buf_size, file);
		assert(numread == buf_size);

		//copy to this->normals
		this->normals = new T[this->points_num*3];
		if( type == '8' )
		{
			double *buf_d = (double*) buf;
			for(int i=0; i<points_num*3; i++)
				this->normals[i] = buf_d[i];
		}
		else if( type == '4' )
		{
			float *buf_f = (float*) buf;
			for(int i=0; i<points_num*3; i++)
				this->normals[i] = buf_f[i];
		}
		else
			throw std::exception("type error!");
	}

	if(bColor)
	{
		numread = fread(buf, 1, buf_size, file);
		assert(numread == buf_size);

		//copy to this->colors
		this->colors = new T[this->points_num*3];
		if( type == '8' )
		{
			double *buf_d = (double*) buf;
			for(int i=0; i<points_num*3; i++)
				this->colors[i] = buf_d[i];
		}
		else if( type == '4' )
		{
			float *buf_f = (float*) buf;
			for(int i=0; i<points_num*3; i++)
				this->colors[i] = buf_f[i];
		}
		else
			throw std::exception("type error!");
	}

	delete [] buf;

	fclose(file);
	return true;
}


template<class T>
bool CPointCloud<T>::readFromVoxelBIN(const char* filename)
{
	FILE *file = NULL;
	int numread = 0;
	if ((fopen_s(&file, filename, "rb")) != 0)
	{
		printf("Error<CPointCloud::readFromVoxelBIN>: when Reading File <%s>\n", filename);
		return false;
	}

	this->freeMemory();

	numread = fread(&this->points_num, sizeof(int), 1, file);
	this->points = new T[points_num * 3];
	this->colors = new T[points_num * 3];


	float cube_side_length = 0.0;
	fread(&cube_side_length, sizeof(float), 1, file);

	for (int i = 0; i < points_num; i++)
	{
		float p[3];
		fread(p, sizeof(float), 3, file);
		T *pt = this->vt_data(i);
		pt[0] = p[0]/10.0; 
		pt[1] = -p[1]/10.0; 
		pt[2] = p[2]/10.0;

		unsigned char yuv_[3];
		fread(yuv_, sizeof(unsigned char), 3, file);
		float yuv[3];
		yuv[0] = yuv_[0] / 255.0;
		yuv[1] = yuv_[1] / 255.0;
		yuv[2] = yuv_[2] / 255.0;
		float rgb[3];
		YUVtoRGB(yuv, rgb);

		T *clr = this->vt_color(i);
		clr[0] = rgb[0];
		clr[1] = rgb[1];
		clr[2] = rgb[2];
	}

	fclose(file);
	return true;
}
