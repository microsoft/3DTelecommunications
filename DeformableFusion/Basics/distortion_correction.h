// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __DISTORTION_CORRECTION_H__
#define __DISTORTION_CORRECTION_H__
//#include "basic_data_structure.h"
#include "UtilVnlMatrix.h"
#include <vil/vil_bilin_interp.h>

void distort_a_point( double x_in, double y_in, 
					  double &x_out, double &y_out,
					  vnl_matrix_fixed<double, 3, 3> const&K,
					  vnl_vector_fixed<double, 5> const&dist_coef );

void undistort_a_point( double x_in, double y_in, 
					    double &x_out, double &y_out,
					    vnl_matrix_fixed<double, 3, 3> const&K,
					    vnl_vector<double> const&dist_coef );

void distort_a_point( double x_in, double y_in, 
					  double &x_out, double &y_out,
					  vnl_matrix_fixed<double, 3, 3> const&K,
					  vnl_vector<double> const&dist_coef );

void undistort_a_point( double x_in, double y_in, 
					    double &x_out, double &y_out,
					    cv::Mat const*K,
					    cv::Mat const*dist_coef );

void distort_a_point( double x_in, double y_in, 
					  double &x_out, double &y_out,
					    cv::Mat const*K,
					    cv::Mat const*dist_coef );

enum interpolation_method
{
	inter_nearest_neighborhood,
	inter_bilinear
};

/* a combination of build_undistortion_map and remap.
 * note the intrinsics must match the size of the input matrix or image
 */
template <class T>
bool undistort( vnl_matrix<T> &mat_in,
			    vnl_matrix<T> &mat_out,
			    vnl_matrix_fixed<double, 3, 3> &K, 
			    vnl_vector<double> &dist_coef,
			    T fill_value = (T)0.0,
			    interpolation_method method = inter_bilinear);

template <class T>
bool undistort( vil_image_view<T> &img_in,
			    vil_image_view<T> &img_out,
			    vnl_matrix_fixed<double, 3, 3> &K, 
			    vnl_vector<double> &dist_coef,
			    T fill_value = (T)0.0,
			    interpolation_method method = inter_bilinear);

/* Build the undistortion map which will be used in function "remap"
 * undistorted_image(u, v)<--original_image( mapx(u, v), mapy(u, v) )
 * image_size = (width, height)
 * dist_coef = (k1, k2, p1, p2, k3)
 */
bool build_undistortion_map( vnl_matrix<double> &MapX, 
							 vnl_matrix<double> &MapY, 
						     vnl_matrix_fixed<double, 3, 3> &K, 
							 vnl_vector<double> &dist_coef,
							 vnl_vector_fixed<int, 2> &image_size);
template <class T> 
bool remap( vnl_matrix<T> &mat_in, 
		    vnl_matrix<T> &mat_out, 
		    vnl_matrix<double> &MapX, 
		    vnl_matrix<double> &MapY,
		    T fill_value=(T)0.0,
		    interpolation_method method = inter_bilinear );

template <class T> 
bool remap( vil_image_view<T> &img_in, 
		    vil_image_view<T> &img_out, 
		    vnl_matrix<double> &MapX, 
		    vnl_matrix<double> &MapY,
		    T fill_value=(T)0.0,
		    interpolation_method method = inter_bilinear);

#include "distortion_correction.hpp"

#endif