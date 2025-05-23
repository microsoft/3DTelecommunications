// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __KL_FEATURES_H__
#define __KL_FEATURES_H__
#include "UtilMatrix.h"
#include "UtilVnlMatrix.h"
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"

#define MAX_CORNER_NUMBER 800

/* corners--out. each row is a 2d image point.
 * mask_img could be NULL
 */
bool extract_corner_points( cv::Mat const& img, cv::Mat const& mask_img, vnl_matrix<double> &corners,
							int max_corner_num = 200,
							double quality_level = 0.005, 
							double min_dist_btw_coners = 5.0,
							bool bHistEqual=false);

/* matched_corners--out. each row is a 4d vector representing a pair of matched point
 * mask_img could be NULL
 */
bool track_corner_points(cv::Mat const& img_prev, vnl_matrix<double> const &corners_prev,
						 cv::Mat const& img_cur, cv::Mat const& mask_cur,
						 vnl_matrix<double> &matched_corners, 
						 vector<int> &matched_corner_indices_prev,
						 int search_radius=20,
						 bool bHistEqual = false);
bool track_corner_points(cv::Mat const& img_prev, vnl_matrix<double> const &corners_prev,
						 cv::Mat const& img_cur, cv::Mat const& mask_cur,
						 vnl_matrix<double> &matched_corners,
						 int search_radius=20,
						 bool bhistEqual = false);
template <class T> bool drawCornersOnImage(cv::Mat& img, vnl_matrix<T> const& corners, cv::Scalar color);
template <class T> bool drawCornersOnImage(cv::Mat& img, vector< vnl_vector_fixed<T, 2> > const& corners, cv::Scalar color);
template<class T> bool drawCornersOnImageAndSave(cv::Mat& img,
	vnl_matrix<T> const& corners,
	char const* filename,
	cv::Scalar color);
template<class T>
bool drawCornersOnImageAndSave(cv::Mat& img,
	vector< vnl_vector_fixed<T, 2> > const& corners,
	char const* filename,
	cv::Scalar color);
template<class T>
bool drawMatchedCornersOnImages(cv::Mat const& img_1, cv::Mat const& img_2, vnl_matrix<T>& matched_corners,
	const char* filename);



struct ImagePointAndFrmIdx
{
public:
	ImagePointAndFrmIdx( int frmidx_, vnl_vector_fixed<double, 2> pt_)
		: frmIdx(frmidx_), pt(pt_)
	{}
	int frmIdx;
	vnl_vector_fixed<double, 2> pt;
};
/* extract corner point tracks for images frames centered around frmIdx_cen
 */
bool extract_corner_tracks_bracket( int frmIdx_cen, std::vector<cv::Mat> const& imgs, std::vector<cv::Mat> const& masks,
									std::vector< vector<ImagePointAndFrmIdx> > &corner_point_tracks );

/* Will NOT contaminate the original images */
bool draw_corner_tracks_and_save( std::vector<cv::Mat> const& imgs,
								  std::vector< vector<ImagePointAndFrmIdx> > const&corner_point_tracks,
								  char const* base_name);

template<class T>
bool drawMatchedCornersOnImages(cv::Mat& img_1, cv::Mat& img_2, vnl_matrix<T> &matched_corners,
								const char* filename);

template<class T>
bool drawCornersOnImage( cv::Mat& img,
						 vnl_matrix<T> const &corners, 
						 cv::Scalar color=CV_RGB(0, 255.0, 0));

template<class T>
bool drawCornersOnImage( cv::Mat& img,
						 vector< vnl_vector_fixed<T, 2> > const &corners, 
						 cv::Scalar color=CV_RGB(0, 255.0, 0) );

//will NOT contaminate the input image
template<class T>
bool drawCornersOnImageAndSave( cv::Mat& img,
								vnl_matrix<T> const &corners,
								char const* filename,
								cv::Scalar color=CV_RGB(0, 255.0, 0));

//will NOT contaminate the input image
template<class T>
bool drawCornersOnImageAndSave( cv::Mat& img,
								vector< vnl_vector_fixed<T, 2> > const &corners, 
								char const* filename,
								cv::Scalar color=CV_RGB(0, 255.0, 0) );

#endif