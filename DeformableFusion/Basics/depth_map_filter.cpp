// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "depth_map_filter.h"

//close operation on the depth map
// 1. close operation on its depth mask
// 2. fill the new pixels with its nearest pixel
void depthmap_morph_close_filter( cv::Mat &depthMat, int radius )
{
	cv::Mat msk = mask_from_depthMap(depthMat);
	MorphCloseOper(msk, 2*radius+1, false);
	
	cv::Mat depthMat_bak = depthMat.clone();
	int height = depthMat.rows;
	int width = depthMat.cols;

	for(int i=0; i<depthMat.rows; i++)
	{
		for(int j=0; j<depthMat.cols; j++)
		{
			if( msk.at<uchar>(i, j) == 0 )
				continue;

			if(  depthMat.at<double>(i, j) > 0.0 )
				continue;

			bool bBreak = false;
			for( int r=1; r<=radius; r++)
			{
				for(int m=0; m<=r; m++)
				{
					if( i+m<height && i+m<width && depthMat_bak.at<double>(i+m, j+r) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i+m, j+r);
						break;
					}
					if( i-m>=0 && j+r<width && depthMat_bak.at<double>(i-m, j+r) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i-m, j+r);										
						break;
					}
					if( i+m<height && j-r>=0 &&  depthMat_bak.at<double>(i+m, j-r) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i+m, j-r);
						break;
					}
					if( i-m>=0 && j-r>=0 &&  depthMat_bak.at<double>(i-m, j-r) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i-m, j-r);;
						break;
					}
					
					if( i+r<height && j+m<width && depthMat_bak.at<double>(i+r, j+m) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i+r, j+m);
						break;
					}
					if( i-r>=0 && j+m<width && depthMat_bak.at<double>(i-r, j+m) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i-r, j+m);
						break;
					}
					if( i+r<height && j-m>=0 && depthMat_bak.at<double>(i+r, j-m) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i+r, j-m);
						break;
					}
					if( i-r>=0 && j-m>=0 && depthMat_bak.at<double>(i-r, j-m) > 0.0 )
					{
						depthMat.at<double>(i, j) = depthMat_bak.at<double>(i-r, j-m);
						break;
					}
				}
				if( bBreak ) break;
			}
		}
	}

	msk.release();
	depthMat_bak.release();
}

void depthmap_normal_filter(cv::Mat& depthMat, vnl_matrix_fixed<double, 3, 3> const&K, double thres_angle)
{
	int w = depthMat.cols;
	int h = depthMat.rows;

	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];

	cv::Mat depthMat_bak = depthMat.clone();

//#pragma omp parallel for
	for (int i = 1; i < h-1; i++)
	{
		for (int j = 1; j < h-1; j++)
		{
			if (depthMat_bak.at<double>(i, j) <= 0.0)
				continue;

			vector<vnl_vector_fixed<double, 3> > points;
			for (int m = -1; m <= 1; m++)
			{
				for (int n = -1; n <= 1; n++)
				{
					double z = depthMat_bak.at<double>(i + m, j + n);
					if (z <= 0.0)
						continue;

					double x = (j+n - cx)*z / fx;
					double y = (i+m - cy)*z / fy;
					points.push_back(vnl_vector_fixed<double, 3>(x, y, z));
				}
			}

			if (points.size() <= 1)
			{
				depthMat.at<double>(i, j) = 0.0;
				continue;
			}
			else if (points.size() < 3)
				continue;

			vnl_vector_fixed<double, 3> normal;
			double d;
			plane_fitting(points, normal, d);
			
			double z = depthMat_bak.at<double>(i, j);;
			double x = (j - cx)*z / fx;
			double y = (i - cy)*z / fy;
			vnl_vector_fixed<double, 3> view_ray(x,y,z);

			double angle = angle_btw_two_vec(view_ray, normal) * 180 / M_PI;
			if (angle > 90 - thres_angle && angle < 90 + thres_angle)
			{
				depthMat.at<double>(i, j) = 0.0;
			}
		}
	}
}

bool depthmap_bilateral_filter( vnl_matrix<double> const&depth_mat_in, vnl_matrix<double> &depth_mat_out,
								double sigma_s, //control the size of neighborhood 
								double sigma_r_  //control the smoothness
							   )
{
	int window_radius = MAX(1, ROUND(3*sigma_s));
	int w = depth_mat_in.cols();
	int h = depth_mat_in.rows();
	depth_mat_out.set_size(h, w);
	depth_mat_out.fill(0.0);

#pragma omp parallel for schedule(dynamic)
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double cur_depth = depth_mat_in[i][j];
			if( cur_depth <= 0.0)
				continue;

			double sigma_r = sigma_r_ * cur_depth / 150;

			double weights = 0.0;
			double vals = 0.0;
			bool bNeighborFound = false;
			for(int m=-window_radius; m<=window_radius; m++)
			{
				for(int n=-window_radius; n<=window_radius; n++)
				{
					if( i+m<0 || i+m>=h ||
						j+n<0 || j+n>=w )
						continue;

					double new_depth = depth_mat_in[i+m][j+n];
					if(new_depth <= 0.0)
						continue;

					bNeighborFound = true;
					double new_weights = exp(-(m*m+n*n)/(2*sigma_s*sigma_s)-(cur_depth-new_depth)*(cur_depth-new_depth)/(2*sigma_r*sigma_r) );
					vals += new_weights*new_depth;
					weights += new_weights;
				}
			}
			if( bNeighborFound )
				depth_mat_out[i][j] = vals/weights;
			else
				depth_mat_out[i][j] = 0.0;
		}
	}

	return true;
}

bool depthmap_bilateral_filter( cv::Mat const& depth_mat_in, vnl_matrix<double> &depth_mat_out,
								double sigma_s, //control the size of neighborhood 
								double sigma_r  //control the smoothness
								)
{
	vnl_matrix<double> depth_mat_in_vnl;
	cvmat_to_vnlmatrix(depth_mat_in, depth_mat_in_vnl);

	return depthmap_bilateral_filter(depth_mat_in_vnl, depth_mat_out, sigma_s, sigma_r);;
}

cv::Mat depthmap_bilateral_filter( cv::Mat const& depth_mat_in, 
								  double sigma_s, //control the size of neighborhood 
								  double sigma_r  //control the smoothness
								)
{
	vnl_matrix<double> depth_mat_in_vnl;
	cvmat_to_vnlmatrix(depth_mat_in, depth_mat_in_vnl);

	vnl_matrix<double> depth_mat_out_vnl;
	depthmap_bilateral_filter(depth_mat_in_vnl, depth_mat_out_vnl, sigma_s, sigma_r);

	return vnlmatrix_to_cvmat(depth_mat_out_vnl);
}

bool depthmap_filter_boundary( cv::Mat& depth_mat, double thres_depth, int thickness)
{
	if( depth_mat.empty() )
	{
		printf("Warning: the input depth map is NULL!\n");
		return false;
	}
	int width = depth_mat.cols;
	int height = depth_mat.rows;
	cv::Mat depth_mat_f = depth_mat.clone();

	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			double d = depth_mat.at<double>(i, j);;
			if( d <= 0)
				continue;

			bool bBreak = false;
			for(int m=-thickness; m<=thickness; m++)
			{
				for(int n=-thickness; n<=thickness; n++)
				{
					if( i+m<0 || i+m>=height || j+n<0 || j+n>=width)
						continue;
					double d_ = depth_mat.at<double>(i+m, j+n);				
					if( d_ > 0 && abs(d - d_) > thres_depth )
					{
						bBreak = true;
						depth_mat_f.at<double>(i, j) = -2;
						break;
					}
				}
				if( bBreak )	break;
			}
		}
	}

	depth_mat_f.copyTo(depth_mat);
	depth_mat_f.release();
	
	return true;
}
