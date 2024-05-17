#include "stdafx.h"
#include "MorphologicalOperation.h"
#include "UtilMatrix.h"
#include "VecOperation.h"
#include "math.h"
#include "opencv2\highgui.hpp"

bool MorphErodeOper(cv::Mat& bImg, int diameter)
{
	int emSize_h1 = diameter;
	int emSize_v1 = diameter;
	cv::Mat em1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v1, emSize_h1), cv::Point(emSize_v1/2, emSize_h1/2));	
	
	cv::erode(bImg, bImg, em1);

	em1.release();

	return true;
}

bool MorphDilateOper(cv::Mat& bImg, int diameter)
{
	int emSize_h1 = diameter;
	int emSize_v1 = diameter;
	cv::Mat em1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v1, emSize_h1), cv::Point(emSize_v1 / 2, emSize_h1 / 2));

	cv::dilate(bImg, bImg, em1);

	em1.release();

	return true;
}

bool MorphCloseOper(cv::Mat& bImg, int diameter, bool bComplex)
{
	int emSize_h1 = diameter;
	int emSize_v1 = diameter;
	int emSize_h2 = diameter;
	int emSize_v2 = diameter;
	cv::Mat em1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v1, emSize_h1), cv::Point(emSize_v1 / 2, emSize_h1 / 2));
	cv::Mat em2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v2, emSize_h2), cv::Point(emSize_v2 / 2, emSize_h2 / 2));

	if( bComplex )
	{
		cv::Mat imgBak = bImg.clone();
		cv::dilate(bImg, bImg, em1);
		cv::erode(bImg, bImg, em2);
		if( bImg.depth() == CV_16U )
		{
			for(int i=0; i<bImg.rows; i++)
			{
				for(int j=0; j<bImg.cols; j++)
				{
					if(  imgBak.at<unsigned short>(i, j) != 0 )
					{
						bImg.at<unsigned short>(i, j) = imgBak.at<unsigned short>(i, j);
					}
				}
			}
		}
		else if( bImg.depth() == CV_8U )
		{
			for(int i=0; i<bImg.rows; i++)
			{
				for(int j=0; j<bImg.cols; j++)
				{
					if(  imgBak.at<unsigned char>(i, j) != 0 )
					{
						bImg.at<uchar>(i, j) = imgBak.at<uchar>(i, j);
					}
				}
			}
		}

		imgBak.release();
	}
	else
	{
		cv::dilate(bImg, bImg, em1);
		cv::erode(bImg, bImg, em2);
	}
	em1.release();
	em2.release();

	return true;
}

bool MorphOpenOper(cv::Mat& bImg, int diameter)
{
	int emSize_h1 = diameter;
	int emSize_v1 = diameter;
	int emSize_h2 = diameter;
	int emSize_v2 = diameter;
	cv::Mat em1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v1, emSize_h1), cv::Point(emSize_v1 / 2, emSize_h1 / 2));
	cv::Mat em2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(emSize_v2, emSize_h2), cv::Point(emSize_v2 / 2, emSize_h2 / 2));

	cv::erode(bImg, bImg, em2);
	cv::dilate(bImg, bImg, em1);

	em1.release();
	em2.release();

	return true;
}



bool refineFGMask(cv::Mat& fgMask, cv::Mat& img)
{
	if( fgMask.empty() || img.empty())
	{
		printf("Input is invalid!\n");
		return false;
	}

	int h = fgMask.rows;
	int w = fgMask.cols;
	cv::Mat img_resized = cv::Mat();
	if( img.cols != fgMask.cols ||
		img.rows != fgMask.rows)
	{
		img_resized = cv::Mat(fgMask.rows, fgMask.cols, CV_MAKETYPE(CV_8U, img.channels()));
		
		cv::resize(img, img_resized, img_resized.size());
		img = img_resized;
	}

	cv::imwrite("../data/img_resized.bmp", img);
	
	cv::Mat gray = cv::Mat(fgMask.rows, fgMask.cols, CV_8UC1);
	cv::Mat Gx = cv::Mat(fgMask.rows, fgMask.cols, CV_16S, 1);
	cv::Mat Gy = cv::Mat(fgMask.rows, fgMask.cols, CV_16S, 1);

	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);	
	cv::Sobel(gray, Gx, 1, 1, 0);
	cv::Sobel(gray, Gy, 1, 0, 1);
	
	cv::imwrite("../data/img_gray.bmp", gray);

	cv::Mat fgMask_new = fgMask.clone();
	
	int search_radius = 15;
	double sigma = 5.0;
	double *kernel = new double[search_radius+1];
	for(int i=0; i<=search_radius; i++)
		kernel[i] = exp(-i*i/(2*sigma*sigma));

	cv::Mat boundary = cv::Mat(h, w, CV_8UC3);
	boundary = 0;

	int gx, gy;
	int sign;
	double *response = new double[2*search_radius+1];
	for(int i=1; i<h-1; i++)
	{
		for(int j=1; j<w-1; j++)
		{
			if( fgMask.at<uchar>(i, j) != 0 &&
				( fgMask.at<uchar>(i-1, j) == 0 ||
				  fgMask.at<uchar>(i+1, j) == 0 ||
				  fgMask.at<uchar>(i, j-1) == 0 ||
				  fgMask.at<uchar>(i, j+1) == 0 ) 
			  )
			{
				int val_11=0, val_12=0, val_21=0, val_22=0;
				for(int m=-3; m<=-1; m++)
					for(int n=-3; n<=-1; n++)
						val_11 += fgMask.at<uchar>(i+m, j+n);
				for(int m=-3; m<=-1; m++)
					for(int n=1; n<=3; n++)
						val_12 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=-3; n<=-1; n++)
						val_21 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=1; n<=3; n++)
						val_22 += fgMask.at<uchar>(i+m, j+n);
				
				if( abs(val_11+val_12-val_21-val_22) < abs(val_11+val_21-val_12-val_22) )
				{
					boundary.at<cv::Vec3b>(i, j)[0] = 255;
					//horizontal direction (boundary is vertical)
					sign = (val_12+val_22) - (val_11+val_21);

					if(sign != 0)
					{
						for(int k=-search_radius; k<=search_radius; k++)
						{
							if( j+k >= 0 && j+k < w)
							{
								gx = Gx.at<short>(i, j+k);
								gy = Gy.at<short>(i, j+k);
							}
							else
							{
								gx = 0;
								gy = 0;
							}
							response[k+search_radius] = kernel[abs(k)]*(gx*gx+gy*gy);
						}

						double max_val=0;
						int max_idx = -search_radius-1;
						for(int k=-search_radius; k<=search_radius; k++)
						{
							if( response[k+search_radius] > max_val )
							{
								max_val = response[k+search_radius];
								max_idx = k;
							}
						}


						if( max_idx > 0)
						{
							if( sign < 0)
							{
								for(int k=0; k<=max_idx; k++)
									fgMask_new.at<uchar>(i, j+k) = 255;
							}
							else
							{
								for(int k=0; k<max_idx; k++)
									fgMask_new.at<uchar>(i, j+k) = 0;
							}
						}
						else
						{
							if(sign < 0)
							{
								for(int k=max_idx; k<=0; k++)
									fgMask_new.at<uchar>(i, j+k) = 0;
							}
							else
							{
								for(int k=max_idx; k<0; k++)
									fgMask_new.at<uchar>(i, j+k) = 255;
							}
						}
					}
				}
				else
				{
					boundary.at<cv::Vec3b>(i, j)[1] = 255;

					// vertical direction (line is horizontal)
					sign = val_21+val_22 - (val_11+val_12);
					
					if(sign != 0)
					{
						for(int k=-search_radius; k<=search_radius; k++)
						{
							if( i+k >= 0 && i+k < h)
							{
								gx = Gx.at<short>(i+k, j);
								gy = Gy.at<short>(i+k, j);
							}
							else
							{
								gx = 0;
								gy = 0;
							}
							response[k+search_radius] = kernel[abs(k)]*(gx*gx+gy*gy);
						}

						double max_val=0;
						int max_idx = -search_radius-1;
						for(int k=-search_radius; k<=search_radius; k++)
						{
							if( response[k+search_radius] > max_val )
							{
								max_val = response[k+search_radius];
								max_idx = k;
							}
						}

						if( max_idx > 0)
						{
							if( sign < 0)
							{
								for(int k=0; k<=max_idx; k++)
									fgMask_new.at<uchar>(i+k, j) = 255;
							}
							else
							{
								for(int k=0; k<max_idx; k++)
									fgMask_new.at<uchar>(i+k, j) = 0;
							}
						}
						else
						{
							if(sign < 0)
							{
								for(int k=max_idx; k<=0; k++)
									fgMask_new.at<uchar>(i+k, j) = 0;
							}
							else
							{
								for(int k=max_idx; k<0; k++)
									fgMask_new.at<uchar>(i+k, j) = 255;
							}
						}
					}
				}
			
			
			}
		}
	}


	cv::imwrite("../data/fgMask_new.bmp", fgMask_new);
	cv::imwrite("../data/boundary.bmp", boundary);

	if (!img_resized.empty())
		img_resized.release();
	return true;
}


bool refineFGMask2(cv::Mat& fgMask, cv::Mat& img, bool preferInside)
{
	if( fgMask.empty()|| img.empty())
	{
		printf("Input is invalid!\n");
		return false;
	}

	int h = fgMask.rows;
	int w = fgMask.cols;
	cv::Mat img_resized = cv::Mat();
	if( img.cols != fgMask.cols ||
		img.rows != fgMask.rows)
	{
		img_resized = cv::Mat(fgMask.rows, fgMask.cols, CV_MAKETYPE(CV_8U, img.channels()));
		cv::resize(img, img_resized, img_resized.size());
		
		img = img_resized;
	}

	cv::imwrite("../data/img_resized.bmp", img);
	
	cv::Mat fgMask_new = fgMask.clone();
	
	int search_radius = 15;
	int prefer_radius = 8;

	cv::Mat boundary = img.clone();

	int sign;
	unsigned char *pixel1;
	unsigned char *pixel2;
	double *response = new double[2*search_radius+1];
	
	int window_radius = 1;
	int blocksize = (2*window_radius+1)*(2*window_radius+1)*3;
	double *pixelsBlock_1 = new double[blocksize];
	double *pixelsBlock_2 = new double[blocksize];
	double similarity_thres = 30.0*sqrt((double)blocksize);

	for(int i=1; i<h-1; i++)
	{
		for(int j=1; j<w-1; j++)
		{
			if( fgMask.at<uchar>(i, j) != 0 &&
				( fgMask.at<uchar>(i-1, j) == 0 ||
				  fgMask.at<uchar>(i+1, j) == 0 ||
				  fgMask.at<uchar>(i, j-1) == 0 ||
				  fgMask.at<uchar>(i, j+1) == 0 ) 
			  )
			{
				int val_11=0, val_12=0, val_21=0, val_22=0;
				for(int m=-3; m<=-1; m++)
					for(int n=-3; n<=-1; n++)
						val_11 += fgMask.at<uchar>(i+m, j+n);
				for(int m=-3; m<=-1; m++)
					for(int n=1; n<=3; n++)
						val_12 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=-3; n<=-1; n++)
						val_21 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=1; n<=3; n++)
						val_22 += fgMask.at<uchar>(i+m, j+n);
				
				if( abs(val_11+val_12-val_21-val_22) < abs(val_11+val_21-val_12-val_22) )
				{
					boundary.at<cv::Vec3b>(i, j)[0] = 255;
					//horizontal direction (boundary is vertical)
					sign = (val_12+val_22) - (val_11+val_21);

					if(sign == 0)
						continue;

					int k1, k2;
					if( preferInside && sign<0 ||
						!preferInside && sign > 0)
					{
						k1 = -prefer_radius;
						k2 = 0;
					}
					else
					{
						k1 = 0;
						k2 = prefer_radius;
					}

					bool bBoundaryFound = false;
					int boundary_idx = 0;
					for(int k=k1; k<=k2; k++)
					{
						pixel1 = &img.at<cv::Vec3b>(i, j+k-1)[0];
						pixel2 = &img.at<cv::Vec3b>(i, j+k+1)[0];
						if( !isPixelSimilar(pixel1, pixel2) )
						{
							bBoundaryFound = true;
							boundary_idx = k;
							break;
						}
					}

					if(!bBoundaryFound)
					{
						for(int k=-k2; k<=-k1; k++)
						{
							pixel1 = &img.at<cv::Vec3b>(i, j+k-1)[0];
							pixel2 = &img.at<cv::Vec3b>(i, j+k+1)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = k;
								break;
							}
						}
					}

					if(!bBoundaryFound)
					{
						for(int k=prefer_radius+1; k<search_radius; k++)
						{
							pixel1 = &img.at<cv::Vec3b>(i, j+k-1)[0];
							pixel2 = &img.at<cv::Vec3b>(i, j+k+1)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = k;
								break;
							}

							pixel1 = &img.at<cv::Vec3b>(i, j+k-1)[0];
							pixel2 = &img.at<cv::Vec3b>(i, j+k+1)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = -k;
								break;
							}
						}
					}

					if(bBoundaryFound)
					{
						boundary.at<cv::Vec3b>(i, j+boundary_idx)[2] = 255;
						if( boundary_idx > 0)
						{
							if( sign < 0)
							{
								for(int k=0; k<=boundary_idx; k++)
									fgMask_new.at<uchar>(i, j+k) = 255;
							}
							else
							{
								for(int k=0; k<boundary_idx; k++)
									fgMask_new.at<uchar>(i, j+k) = 0;
							}
						}
						else
						{
							if(sign < 0)
							{
								for(int k=boundary_idx; k<=0; k++)
									fgMask_new.at<uchar>(i, j+k) = 0;
							}
							else
							{
								for(int k=boundary_idx; k<0; k++)
									fgMask_new.at<uchar>(i, j+k) = 255;
							}
						}
					}
				}
				else
				{
					boundary.at<cv::Vec3b>(i, j)[1] = 255;

					// vertical direction (line is horizontal)
					sign = val_21+val_22 - (val_11+val_12);
					
					if(sign == 0)
						continue;
					
					int k1, k2;
					if( preferInside && sign<0 ||
						!preferInside && sign > 0)
					{
						k1 = -prefer_radius;
						k2 = 0;
					}
					else
					{
						k1 = 0;
						k2 = prefer_radius;
					}

					bool bBoundaryFound = false;
					int boundary_idx = 0;
					for(int k=k1; k<=k2; k++)
					{
						pixel1 = &img.at<cv::Vec3b>(i+k-1, j)[0];
						pixel2 = &img.at<cv::Vec3b>(i+k+1, j)[0];
						if( !isPixelSimilar(pixel1, pixel2) )
						{
							bBoundaryFound = true;
							boundary_idx = k;
							break;
						}
					}

					if(!bBoundaryFound)
					{
						for(int k=-k2; k<=-k1; k++)
						{
							pixel1 = &img.at<cv::Vec3b>(i+k-1, j)[0];
							pixel2 = &img.at<cv::Vec3b>(i+k+1, j)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = k;
								break;
							}
						}
					}

					if(!bBoundaryFound)
					{
						for(int k=prefer_radius+1; k<search_radius; k++)
						{
							pixel1 = &img.at<cv::Vec3b>(i+k-1, j)[0];
							pixel2 = &img.at<cv::Vec3b>(i+k+1, j)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = k;
								break;
							}

							pixel1 = &img.at<cv::Vec3b>(i+k-1, j)[0];
							pixel2 = &img.at<cv::Vec3b>(i+k+1, j)[0];
							if( !isPixelSimilar(pixel1, pixel2) )
							{
								bBoundaryFound = true;
								boundary_idx = -k;
								break;
							}
						}
					}

					if(bBoundaryFound)
					{
						boundary.at<cv::Vec3b>(i+boundary_idx, j)[2] = 255;
						boundary.at<cv::Vec3b>(i+boundary_idx, j)[1] = 255;

						if( boundary_idx > 0)
						{
							if( sign < 0)
							{
								for(int k=0; k<=boundary_idx; k++)
									fgMask_new.at<uchar>(i+k, j) = 255;
							}
							else
							{
								for(int k=0; k<boundary_idx; k++)
									fgMask_new.at<uchar>(i+k, j) = 0;
							}
						}
						else
						{
							if(sign < 0)
							{
								for(int k=boundary_idx; k<=0; k++)
									fgMask_new.at<uchar>(i+k, j) = 0;
							}
							else
							{
								for(int k=boundary_idx; k<0; k++)
									fgMask_new.at<uchar>(i+k, j) = 255;
							}
						}
					}

				} //if(line is vertical or horizontal).. else ..

			}// if the pixel is boundary
		}
	}
	
	MorphOpenOper(fgMask_new, 3);
	MorphCloseOper(fgMask_new, 3);

	cv::imwrite("../data/fgMask_new.bmp", fgMask_new);
	cv::imwrite("../data/boundary.bmp", boundary);

	if (!img_resized.empty())
		img_resized.release();
	return true;
}


bool refineFGMask3(cv::Mat& fgMask, cv::Mat& img, bool preferInside)
{
	if( !fgMask.empty() || !img.empty())
	{
		printf("Input is invalid!\n");
		return false;
	}

	int h = fgMask.rows;
	int w = fgMask.cols;
	cv::Mat img_resized = cv::Mat();
	if( img.cols != fgMask.cols ||
		img.rows != fgMask.rows)
	{
		img_resized = cv::Mat(fgMask.rows, fgMask.cols, CV_MAKETYPE(CV_8U, img.channels()));
		cv::resize(img, img_resized, img_resized.size());
		img = img_resized;
	}
	
	cv::Mat fgMask_new = fgMask.clone();
	
	int search_radius = 15;
	int prefer_radius = 8;


	int sign;
	unsigned char *pixel;
	int clusterNum;
	int fgPointNum, bgPointNum;
	int k1, k2;
	double clusterCenters[50][3];
	double clusterStds[50][3];
	int pointCounts[50];
	bool bFGClusterFound;
	bool bBGClusterFound;
	double fgClusterCenter[3];
	double fgClusterStd[3];
	double bgClusterCenter[3];
	double bgClusterStd[3];
	HSV hsv;
	for(int i=4; i<h-4; i++)
	{
		for(int j=4; j<w-4; j++)
		{
			if( fgMask.at<uchar>(i, j) != 0 &&
				( fgMask.at<uchar>(i-1, j) == 0 ||
				  fgMask.at<uchar>(i+1, j) == 0 ||
				  fgMask.at<uchar>(i, j-1) == 0 ||
				  fgMask.at<uchar>(i, j+1) == 0 ) 
			  )
			{
				int val_11=0, val_12=0, val_21=0, val_22=0;
				for(int m=-3; m<=-1; m++)
					for(int n=-3; n<=-1; n++)
						val_11 += fgMask.at<uchar>(i+m, j+n);
				for(int m=-3; m<=-1; m++)
					for(int n=1; n<=3; n++)
						val_12 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=-3; n<=-1; n++)
						val_21 += fgMask.at<uchar>(i+m, j+n);
				for(int m=1; m<=3; m++)
					for(int n=1; n<=3; n++)
						val_22 += fgMask.at<uchar>(i+m, j+n);
				
				if( abs(val_11+val_12-val_21-val_22) < abs(val_11+val_21-val_12-val_22) )
				{
					//horizontal direction (boundary is vertical)
					sign = (val_12+val_22) - (val_11+val_21);

					if(sign == 0)
						continue;

					sign = sign>0?1:-1;

				//============ seek the Foreground Color Cluster along a line ===========
					for(int k=0; k<50; k++)
						pointCounts[k] = 0;
					pixel = &img.at<cv::Vec3b>(i, j)[0];
					hsv = RGB2HSV(pixel[2], pixel[1], pixel[0]);
					clusterCenters[0][0] = hsv.h;
					clusterCenters[0][1] = hsv.s;
					clusterCenters[0][2] = hsv.v;
					clusterStds[0][0] = 0.0;
					clusterStds[0][1] = 0.0;
					clusterStds[0][2] = 0.0;
					pointCounts[0]++;
					clusterNum = 1;
					fgPointNum = 1;
					for(int k=1; k<50; k++)
					{
						if( j+k*sign >= w ||
							j+k*sign < 0 ||
							fgMask.at<uchar>(i, j+k*sign) == 0)
							continue;
						fgPointNum++;
						pixel = &img.at<cv::Vec3b>(i, j+k*sign)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						bool bMatchedClusterFound = false;
						for(int c=0; c<clusterNum; c++)
						{
							if( abs(hsv_cur.h-clusterCenters[c][0]) < 0.1 &&
								abs(hsv_cur.s-clusterCenters[c][1]) < 0.1 )
							{
								clusterCenters[c][0] = (clusterCenters[c][0]*pointCounts[c] + hsv_cur.h)/(pointCounts[c]+1);
								clusterCenters[c][1] = (clusterCenters[c][1]*pointCounts[c] + hsv_cur.s)/(pointCounts[c]+1);
								clusterCenters[c][2] = (clusterCenters[c][2]*pointCounts[c] + hsv_cur.v)/(pointCounts[c]+1);
								clusterStds[c][0] = (clusterStds[c][0]*pointCounts[c] + abs(hsv_cur.h-clusterCenters[c][0]))/(pointCounts[c]+1);
								clusterStds[c][1] = (clusterStds[c][1]*pointCounts[c] + abs(hsv_cur.s-clusterCenters[c][1]))/(pointCounts[c]+1);
								clusterStds[c][2] = (clusterStds[c][2]*pointCounts[c] + abs(hsv_cur.v-clusterCenters[c][2]))/(pointCounts[c]+1);
								pointCounts[c]++;
								bMatchedClusterFound = true;
								break;
							}
						}
						if(!bMatchedClusterFound)
						{
							clusterCenters[clusterNum][0] = hsv_cur.h;
							clusterCenters[clusterNum][1] = hsv_cur.s;
							clusterCenters[clusterNum][2] = hsv_cur.v;
							clusterStds[clusterNum][0] = 0.0;
							clusterStds[clusterNum][1] = 0.0;
							clusterStds[clusterNum][2] = 0.0;
							pointCounts[clusterNum] = 1;
							clusterNum++;
						}
					}
					bFGClusterFound = false;
					for(int c=0; c<clusterNum; c++)
					{
						if( pointCounts[c] / double(fgPointNum) > 0.6 && 
							pointCounts[c] > 20 )
						{
							fgClusterCenter[0] = clusterCenters[c][0];
							fgClusterCenter[1] = clusterCenters[c][1];
							fgClusterCenter[2] = clusterCenters[c][2];
							fgClusterStd[0] = clusterStds[c][0];
							fgClusterStd[1] = clusterStds[c][1];
							fgClusterStd[2] = clusterStds[c][2];
							bFGClusterFound = true;
							break;
						}
					}

				//============ seek the Background Color Cluster along a line ===========
					for(int k=0; k<50; k++)
						pointCounts[k] = 0;
					pixel = &img.at<cv::Vec3b>(i, j)[0];
					hsv = RGB2HSV(pixel[2], pixel[1], pixel[0]);
					clusterCenters[0][0] = hsv.h;
					clusterCenters[0][1] = hsv.s;
					clusterCenters[0][2] = hsv.v;
					pointCounts[0]++;
					clusterNum = 1;
					bgPointNum = 1;
					for(int k=1; k<50; k++)
					{
						if( j-k*sign < 0 ||
							j-k*sign >= w ||
							fgMask.at<uchar>(i, j-k*sign) != 0)
							continue;
						bgPointNum++;
						pixel = &img.at<cv::Vec3b>(i, j-k*sign)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						bool bMatchedClusterFound = false;
						for(int c=0; c<clusterNum; c++)
						{
							if( abs(hsv_cur.h-clusterCenters[c][0]) < 0.1 &&
								abs(hsv_cur.s-clusterCenters[c][1]) < 0.1 )
							{
								clusterCenters[c][0] = (clusterCenters[c][0]*pointCounts[c] + hsv_cur.h)/(pointCounts[c]+1);
								clusterCenters[c][1] = (clusterCenters[c][1]*pointCounts[c] + hsv_cur.s)/(pointCounts[c]+1);
								clusterCenters[c][2] = (clusterCenters[c][2]*pointCounts[c] + hsv_cur.v)/(pointCounts[c]+1);
								clusterStds[c][0] = (clusterStds[c][0]*pointCounts[c] + abs(hsv_cur.h-clusterCenters[c][0]))/(pointCounts[c]+1);
								clusterStds[c][1] = (clusterStds[c][1]*pointCounts[c] + abs(hsv_cur.s-clusterCenters[c][1]))/(pointCounts[c]+1);
								clusterStds[c][2] = (clusterStds[c][2]*pointCounts[c] + abs(hsv_cur.v-clusterCenters[c][2]))/(pointCounts[c]+1);
								pointCounts[c]++;
								bMatchedClusterFound = true;
								break;
							}
						}
						if(!bMatchedClusterFound)
						{
							clusterCenters[clusterNum][0] = hsv_cur.h;
							clusterCenters[clusterNum][1] = hsv_cur.s;
							clusterCenters[clusterNum][2] = hsv_cur.v;
							clusterStds[clusterNum][0] = 0.0;
							clusterStds[clusterNum][1] = 0.0;
							clusterStds[clusterNum][2] = 0.0;
							pointCounts[clusterNum] = 1;
							clusterNum++;
						}
					}
					bBGClusterFound = false;
					for(int c=0; c<clusterNum; c++)
					{
						if( pointCounts[c] / double(bgPointNum) > 0.6 && 
							pointCounts[c] > 20 )
						{
							bgClusterCenter[0] = clusterCenters[c][0];
							bgClusterCenter[1] = clusterCenters[c][1];
							bgClusterCenter[2] = clusterCenters[c][2];
							bgClusterStd[0] = clusterStds[c][0];
							bgClusterStd[1] = clusterStds[c][1];
							bgClusterStd[2] = clusterStds[c][2];
							bBGClusterFound = true;
							break;
						}
					}

				//================== Judge each pixel based on its similarity with BG/FG cluster =====
					for(int k=-20; k<=20; k++)
					{
						if( j+k < 0 ||
							j+k >= w )
							continue;
						pixel = &img.at<cv::Vec3b>(i, j+k)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						if(bFGClusterFound)
						{
							if( (abs(hsv_cur.h-fgClusterCenter[0]) < fgClusterStd[0]*2&&
								abs(hsv_cur.s-fgClusterCenter[1]) < fgClusterStd[1]*2.5 &&
								abs(hsv_cur.v-fgClusterCenter[2]) < fgClusterStd[2]*4.0) || 
								(abs(hsv_cur.h-fgClusterCenter[0]) < fgClusterStd[0]*2.5 &&
								abs(hsv_cur.s-fgClusterCenter[1]) < fgClusterStd[1]*2 &&
								abs(hsv_cur.v-fgClusterCenter[2]) < fgClusterStd[2]*4.0) )
								fgMask_new.at<uchar>(i, j+k) = 255;
							else if(bgPointNum > 40)
								fgMask_new.at<uchar>(i, j+k) = 0;
						}
						else if( bBGClusterFound)
						{
							if( abs(hsv_cur.h-bgClusterCenter[0]) < bgClusterStd[0]*2 &&
								abs(hsv_cur.s-bgClusterCenter[1]) < bgClusterStd[1]*2 )
								fgMask_new.at<uchar>(i, j+k) = 0;
						}

						if( hsv_cur.h > 0.04 && hsv_cur.h<0.07 &&
							hsv_cur.s > 0.5 && hsv_cur.s<0.7)
							fgMask_new.at<uchar>(i, j+k) = 255;
					}
				}
				else
				{
		//============vertical direction (line is horizontal)=============
					sign = val_21+val_22 - (val_11+val_12);
					if(sign == 0)
						continue;

					sign = sign>0?1:-1;
										
				//============ seek the Foreground Color Cluster along a line ===========
					for(int k=0; k<50; k++)
						pointCounts[k] = 0;
					pixel = &img.at<cv::Vec3b>(i, j)[0];
					hsv = RGB2HSV(pixel[2], pixel[1], pixel[0]);
					clusterCenters[0][0] = hsv.h;
					clusterCenters[0][1] = hsv.s;
					clusterCenters[0][2] = hsv.v;
					pointCounts[0]++;
					clusterNum = 1;
					fgPointNum = 1;
					for(int k=1; k<50; k++)
					{
						if( i+k*sign >= h ||
							i+k*sign < 0 ||
							fgMask.at<uchar>(i+k*sign, j) == 0)
							continue;
						fgPointNum++;
						pixel = &img.at<cv::Vec3b>(i+k*sign, j)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						bool bMatchedClusterFound = false;
						for(int c=0; c<clusterNum; c++)
						{
							if( abs(hsv_cur.h-clusterCenters[c][0]) < 0.1 &&
								abs(hsv_cur.s-clusterCenters[c][1]) < 0.1 )
							{
								clusterCenters[c][0] = (clusterCenters[c][0]*pointCounts[c] + hsv_cur.h)/(pointCounts[c]+1);
								clusterCenters[c][1] = (clusterCenters[c][1]*pointCounts[c] + hsv_cur.s)/(pointCounts[c]+1);
								clusterCenters[c][2] = (clusterCenters[c][2]*pointCounts[c] + hsv_cur.v)/(pointCounts[c]+1);
								clusterStds[c][0] = (clusterStds[c][0]*pointCounts[c] + abs(hsv_cur.h-clusterCenters[c][0]))/(pointCounts[c]+1);
								clusterStds[c][1] = (clusterStds[c][1]*pointCounts[c] + abs(hsv_cur.s-clusterCenters[c][1]))/(pointCounts[c]+1);
								clusterStds[c][2] = (clusterStds[c][2]*pointCounts[c] + abs(hsv_cur.v-clusterCenters[c][2]))/(pointCounts[c]+1);
								pointCounts[c]++;
								bMatchedClusterFound = true;
								break;
							}
						}
						if(!bMatchedClusterFound)
						{
							clusterCenters[clusterNum][0] = hsv_cur.h;
							clusterCenters[clusterNum][1] = hsv_cur.s;
							clusterCenters[clusterNum][2] = hsv_cur.v;
							clusterStds[clusterNum][0] = 0.0;
							clusterStds[clusterNum][1] = 0.0;
							clusterStds[clusterNum][2] = 0.0;
							pointCounts[clusterNum] = 1;
							clusterNum++;
						}
					}
					bFGClusterFound = false;
					for(int c=0; c<clusterNum; c++)
					{
						if( pointCounts[c] / double(fgPointNum) > 0.6 && 
							pointCounts[c] > 20 )
						{
							fgClusterCenter[0] = clusterCenters[c][0];
							fgClusterCenter[1] = clusterCenters[c][1];
							fgClusterCenter[2] = clusterCenters[c][2];
							fgClusterStd[0] = clusterStds[c][0];
							fgClusterStd[1] = clusterStds[c][1];
							fgClusterStd[2] = clusterStds[c][2];
							bFGClusterFound = true;
							break;
						}
					}

				//============ seek the Background Color Cluster along a line ===========
					for(int k=0; k<50; k++)
						pointCounts[k] = 0;
					pixel = &img.at<cv::Vec3b>(i, j)[0];
					hsv = RGB2HSV(pixel[2], pixel[1], pixel[0]);
					clusterCenters[0][0] = hsv.h;
					clusterCenters[0][1] = hsv.s;
					clusterCenters[0][2] = hsv.v;
					pointCounts[0]++;
					clusterNum = 1;
					bgPointNum = 1;
					for(int k=1; k<50; k++)
					{
						if( i-k*sign < 0 ||
							i-k*sign >= h ||
							fgMask.at<uchar>(i-k*sign, j) != 0)
							continue;
						bgPointNum++;
						pixel = &img.at<cv::Vec3b>(i-k*sign, j)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						bool bMatchedClusterFound = false;
						for(int c=0; c<clusterNum; c++)
						{
							if( abs(hsv_cur.h-clusterCenters[c][0]) < 0.1 &&
								abs(hsv_cur.s-clusterCenters[c][1]) < 0.1 )
							{
								clusterCenters[c][0] = (clusterCenters[c][0]*pointCounts[c] + hsv_cur.h)/(pointCounts[c]+1);
								clusterCenters[c][1] = (clusterCenters[c][1]*pointCounts[c] + hsv_cur.s)/(pointCounts[c]+1);
								clusterCenters[c][2] = (clusterCenters[c][2]*pointCounts[c] + hsv_cur.v)/(pointCounts[c]+1);
								clusterStds[c][0] = (clusterStds[c][0]*pointCounts[c] + abs(hsv_cur.h-clusterCenters[c][0]))/(pointCounts[c]+1);
								clusterStds[c][1] = (clusterStds[c][1]*pointCounts[c] + abs(hsv_cur.s-clusterCenters[c][1]))/(pointCounts[c]+1);
								clusterStds[c][2] = (clusterStds[c][2]*pointCounts[c] + abs(hsv_cur.v-clusterCenters[c][2]))/(pointCounts[c]+1);
								pointCounts[c]++;
								bMatchedClusterFound = true;
								break;
							}
						}
						if(!bMatchedClusterFound)
						{
							clusterCenters[clusterNum][0] = hsv_cur.h;
							clusterCenters[clusterNum][1] = hsv_cur.s;
							clusterCenters[clusterNum][2] = hsv_cur.v;
							clusterStds[clusterNum][0] = 0.0;
							clusterStds[clusterNum][1] = 0.0;
							clusterStds[clusterNum][2] = 0.0;
							pointCounts[clusterNum] = 1;
							clusterNum++;
						}
					}
					bBGClusterFound = false;
					for(int c=0; c<clusterNum; c++)
					{
						if( pointCounts[c] / double(bgPointNum) > 0.6 && 
							pointCounts[c] > 20 )
						{
							bgClusterCenter[0] = clusterCenters[c][0];
							bgClusterCenter[1] = clusterCenters[c][1];
							bgClusterCenter[2] = clusterCenters[c][2];
							bgClusterStd[0] = clusterStds[c][0];
							bgClusterStd[1] = clusterStds[c][1];
							bgClusterStd[2] = clusterStds[c][2];
							bBGClusterFound = true;
							break;
						}
					}

				//================== Judge each pixel based on its similarity with BG/FG cluster =====
					for(int k=-20; k<=20; k++)
					{
						if( i+k < 0 ||
							i+k >= h )
							continue;
						pixel = &img.at<cv::Vec3b>(i+k, j)[0];
						HSV hsv_cur = RGB2HSV(pixel[2], pixel[1], pixel[0]);
						if(bFGClusterFound)
						{
							if( (abs(hsv_cur.h-fgClusterCenter[0]) < fgClusterStd[0]*2&&
								abs(hsv_cur.s-fgClusterCenter[1]) < fgClusterStd[1]*2.5 &&
								abs(hsv_cur.v-fgClusterCenter[2]) < fgClusterStd[2]*4.0) || 
								(abs(hsv_cur.h-fgClusterCenter[0]) < fgClusterStd[0]*2.5 &&
								abs(hsv_cur.s-fgClusterCenter[1]) < fgClusterStd[1]*2 &&
								abs(hsv_cur.v-fgClusterCenter[2]) < fgClusterStd[2]*4.0) )
								fgMask_new.at<uchar>(i+k, j) = 255;
							else if(bgPointNum > 40)
								fgMask_new.at<uchar>(i+k, j) = 0;
						}
						else if( bBGClusterFound)
						{
							if( abs(hsv_cur.h-bgClusterCenter[0]) < bgClusterStd[0]*2 &&
								abs(hsv_cur.s-bgClusterCenter[1]) < bgClusterStd[1]*2 )
								fgMask_new.at<uchar>(i+k, j) = 0;
						}

						if( hsv_cur.h > 0.04 && hsv_cur.h<0.07 &&
							hsv_cur.s > 0.5 && hsv_cur.s<0.7)
							fgMask_new.at<uchar>(i+k, j) = 255;

					}


				} //if(line is vertical or horizontal).. else ..

			}// if the pixel is boundary
		}
	}
	
	if (!img_resized.empty())
		img_resized.release();

	fgMask_new.copyTo(fgMask);
	
	fgMask_new.release();
	return true;
}

HSV RGB2HSV(float r, float g, float b)
{
	float hue, sat, val;
	float min_v;
	if (r == g && r == b)
	{
		hue = 0;
		sat = 0;
		val = (float)(r / 255.0);
	}
	else if (r >= g && r >= b)
	{
		min_v = min(g, b);
		hue = (float)(g - b) / (r - min_v) / 6.0;
		if (hue < 0)
			hue = 1 - hue;
		sat = 1 - min_v / float(r);
		val = (float)r / 255.0;
	}
	else if (g >= r && g >= b)
	{
		min_v = min(r, b);
		hue = (float)(60 * (b - r) / (g - min_v) + 120) / 360.0;
		sat = 1 - min_v / float(g);
		val = (float)g / 255.0;
	}
	else if (b >= r && b >= g)
	{
		min_v = min(r, g);
		hue = (float)(60 * (r - g) / (b - min_v) + 240) / 360.0;
		sat = 1 - min_v / float(b);
		val = (float)b / 255.0;
	}
	HSV m_hsv;
	m_hsv.h = hue;
	m_hsv.s = sat;
	m_hsv.v = val;
	return m_hsv;
}