//===============================================
//			VilToOpenCv.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __VILTOOPENCV_H__
#define __VILTOOPENCV_H__
#include <vil/vil_image_view.h>
#include <vil/vil_save.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matrix.hxx>
#include "opencv2\opencv.hpp"
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"

template<typename T>
cv::Mat vnlmatrix_to_cvmat(vnl_matrix<T> const &vnl_mat)
{
	int w = vnl_mat.cols();
	int h = vnl_mat.rows();

	if( w<=0 || h<=0)
		return cv::Mat();

	cv::Mat ret = cv::Mat(h, w, CV_64F);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			ret.at<double>(i, j) = (double) vnl_mat[i][j];
		}
	}
	return ret;
}

template<typename T>
bool cvmat_to_vnlmatrix(cv::Mat const& cv_mat, vnl_matrix<T> &vnl_mat)
{
	if( cv_mat.empty() )
		return false;

	int w = cv_mat.cols;
	int h = cv_mat.rows;
	vnl_mat.set_size(h, w);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			vnl_mat[i][j] = (T) cv_mat.at<double>(i, j);;
		}
	}
	return true;
};



#endif