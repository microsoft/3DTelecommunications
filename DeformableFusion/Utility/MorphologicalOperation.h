//===============================================
//			MorphologicalOperation.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __MORPHOLOGICALOPERATION_H__
#define __MORPHOLOGICALOPERATION_H__
#include "opencv2\opencv.hpp"

bool MorphCloseOper(cv::Mat& bImg, int diameter=7, bool bComplex = false);
bool MorphOpenOper(cv::Mat& bImg, int diameter);
bool MorphDilateOper(cv::Mat& bImg, int diameter);
bool MorphErodeOper(cv::Mat& bImg, int diameter);

bool refineFGMask(cv::Mat& fgMask, cv::Mat& img);
bool refineFGMask2(cv::Mat& fgMask, cv::Mat& img, bool preferInside);
bool refineFGMask3(cv::Mat& fgMask, cv::Mat& img, bool preferInside);

struct HSV
{
	float h;
	float s;
	float v;
};

HSV RGB2HSV(float r, float g, float b);

inline bool isPixelSimilar(unsigned char pixel1[3], unsigned char pixel2[2])
{
	HSV hsv1 = RGB2HSV(pixel1[2], pixel1[1], pixel1[0]);
	HSV hsv2 = RGB2HSV(pixel2[2], pixel2[1], pixel2[0]);

	if ( abs(hsv1.h - hsv2.h) > 0.1 &&
		 abs(hsv1.s - hsv2.s) > 0.1
		)
	{					
		return false;
	}	
	else
		return true;

}

#endif