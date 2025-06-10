// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __NORMAL_ESTIMATION_H__
#define __NORMAL_ESTIMATION_H__
#include "basic_geometry.h"
#include <vil\vil_image_view.h>
#include <vil\vil_save.h>
#include <vil\vil_load.h>

///**********Local Plane Fitting or Normal Estimation for the depth map************************
/* calculate the point coordinate and point normal in the camera space
 * 5 is recommended for radius
 */
bool local_plane_estimation_for_depthMap( vnl_matrix<double> const& depthMapOri, // the depth map from the depth camera
										  vnl_matrix_fixed<double, 3, 3> const& K,
										  CDepthMap &depthMap,
										  int radius = 5, //search radius to find NNs for normal computation
										  double thres_neighbors_z_dif = 10.0
										  );

/* different way to compute local normal, faster than the above method
 */
bool local_plane_estimation_for_depthMap2( vnl_matrix<double> const& depthMapOri, // the depth map from the depth camera
										   vnl_matrix_fixed<double, 3, 3> const& K,
										   CDepthMap &depthMap,
										   int radius = 5, //search radius to find NNs for normal computation
										   double thres_neighbors_z_dif = 10.0,
										   int skip_step = 2, //the step for neighborhood search
										   int min_points_num = 10, //minimum number of points to fit a local plane
										   bool bSkipHalf = true // when true, calc normals for only half points
										  );

//estimate local plane in world space: the 2nd method is used to compute normal
bool local_plane_estimation_for_depthMap3( vnl_matrix<double> const& depthMapOri, // the depth map from the depth camera
										   vpgl_perspective_camera<double> const& cam,
										   CDepthMap &depthMap,
										   int radius = 5, //search radius to find NNs for normal computation
										   double thres_neighbors_z_dif = 10.0,
										   int skip_step = 1, //the step for neighborhood search
										   int min_points_num = 3, //minimum number of points to fit a local plane
										   bool bSkipHalf = true // when true, calc normals for only half points
										  );

//estimate local plane in world space: the 2nd method is used to compute normal
bool local_plane_estimation_for_depthMap_ortho( cv::Mat& depthMat, // the depth map from the depth camera
												CDepthMap &depthMap,
												double pixel_dist,
												int radius = 5, //search radius to find NNs for normal computation
												double thres_neighbors_z_dif = 5.0,
												int skip_step = 1, //the step for neighborhood search
												int min_points_num = 3, //minimum number of points to fit a local plane
												bool bSkipHalf = true // when true, calc normals for only half points
												);
bool local_plane_estimation_for_depthMap_ortho( cv::Mat& depthMat, // the depth map from the depth camera
												vnl_matrix<vnl_vector_fixed<double,3>> &normalMap,
												double pixel_dist,
												int radius = 5, //search radius to find NNs for normal computation
												double thres_neighbors_z_dif = 5.0,
												int skip_step = 1, //the step for neighborhood search
												int min_points_num = 3, //minimum number of points to fit a local plane
												bool bSkipHalf = true // when true, calc normals for only half points
												);

bool local_plane_estimation_for_depthMap( vnl_matrix<double> const& X, // <X, Y, Z> represent the point cloud in camera space from a depth camera
										  vnl_matrix<double> const& Y,
										  vnl_matrix<double> const& Z,
										  CDepthMap &depthMap,
										  int radius = 5 //search radius to find NNs for normal computation
										  );

bool calc_normal_at_imgCoord( int x, int y, 
							  vnl_matrix<double> const&depthMat, 
							  vnl_matrix_fixed<double, 3, 3> const&intrinsics,
							  vnl_vector_fixed<double, 3> &normal,
							  int radius = 8);

/* extract normal to a matrix
 */
void extract_normal_to_matrix( CDepthMap &depthMap,
							   vnl_matrix< vnl_vector_fixed<double, 3> > &normal_matrix );
/* extract normal to an image
 */
void extract_normal_to_image( CDepthMap const& depthMap,
							  vil_image_view<vxl_byte>& normal_img );

cv::Mat extract_normal_to_image( CDepthMap const& depthMap );

template<class T>
cv::Mat normalMap_to_image(vnl_matrix< vnl_vector_fixed<T, 3> > const&normalMap)
{
	int h = normalMap.rows();
	int w = normalMap.cols();

	cv::Mat img = cv::Mat(h, w, CV_MAKETYPE(8, 3), cv::Scalar(0));

	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			vnl_vector_fixed<T, 3> const& normal = normalMap[i][j];
			if (normal != vnl_vector_fixed<T, 3>(0.0))
			{
				img.at<cv::Vec3b>(i, j)[2] = ROUND(((normal[0] + 1.0) / 2.0) * 255);
				img.at<cv::Vec3b>(i, j)[1] = ROUND(((normal[1] + 1.0) / 2.0) * 255);
				img.at<cv::Vec3b>(i, j)[0] = ROUND(((normal[2] + 1.0) / 2.0) * 255);
			}
		}
	}

	return img;
}

#endif