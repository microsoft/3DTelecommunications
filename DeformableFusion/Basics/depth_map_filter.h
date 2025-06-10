// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __DEPTH_MAP_FILTER_H__
#define __DEPTH_MAP_FILTER_H__
#include "basic_structure.h"
#include "VilToOpenCv.h"

//close operation on the depth map
// 1. close operation on its depth mask
// 2. fill the new pixels with its nearest pixel
// Problem: lead to unnecessary expanding if there are both foreground and background
void depthmap_morph_close_filter( cv::Mat *depthMat, int radius = 3 );

//delete points whose normals point away from the view ray
void depthmap_normal_filter(cv::Mat* depthMat, vnl_matrix_fixed<double, 3, 3> const&K, double thres_angle);

/* bilateral filter
 */
bool depthmap_bilateral_filter( vnl_matrix<double> const&depth_mat_in, vnl_matrix<double> &depth_mat_out,
								double sigma_s = 2.0, //control the size of neighborhood 
								double sigma_r = 2.0  //control the smoothness
							   );

bool depthmap_bilateral_filter( cv::Mat const& depth_mat_in, vnl_matrix<double> &depth_mat_out,
								double sigma_s = 2.0, //control the size of neighborhood 
								double sigma_r = 2.0  //control the smoothness
							   );

cv::Mat depthmap_bilateral_filter( cv::Mat const& depth_mat_in,
								  double sigma_s = 2.0, //control the size of neighborhood 
								  double sigma_r = 2.0  //control the smoothness
								 );

/* filter out boundary pixels
 */
bool depthmap_filter_boundary( cv::Mat& depth_mat, double thres_depth = 10.0, int thickness = 5);

template<typename T>
bool depthmap_filter_boundary( vnl_matrix<T> &depth_mat, double thres_depth = 10.0, int thickness = 5 )
{
	cv::Mat depth_mat_cv = vnlmatrix_to_cvmat(depth_mat);
	bool ret = depthmap_filter_boundary(depth_mat_cv, thres_depth, thickness);
	cvmat_to_vnlmatrix(depth_mat_cv, depth_mat);
	depth_mat_cv.release();
	return ret;
}
#endif