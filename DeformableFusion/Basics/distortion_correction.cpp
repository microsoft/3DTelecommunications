// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "distortion_correction.h"

void distort_a_point( double x_in, double y_in, 
					  double &x_out, double &y_out,
					  vnl_matrix_fixed<double, 3, 3> const&K,
					  vnl_vector_fixed<double, 5> const&dist_coef )
{
	vnl_vector<double> dist_vec(5);
	dist_vec.set(dist_coef.data_block());
	distort_a_point(x_in, y_in, x_out, y_out, K, dist_vec);
}

/* interative method to undistort a point
 * The method is copied from Matlab Calibration ToolBox
 */
void undistort_a_point( double x_in, double y_in, 
						double &x, double &y,
						vnl_matrix_fixed<double, 3, 3> const&K,
						vnl_vector<double> const&dist_coef )
{
	double k1=0, k2=0, k3=0;
	double p1=0, p2=0;
	switch(dist_coef.size())
	{
	case 0:
	case 1:
	case 3:
		printf("Error: distort_coef has incorrect size!\n");
		return;
	case 5:
		k3 = dist_coef[4];
	case 4:
		p1 = dist_coef[2];
		p2 = dist_coef[3];
	case 2:
		k1 = dist_coef[0];
		k2 = dist_coef[1];
	default:
		break;
	}

	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];

	x = (x_in-cx)/fx;
	y = (y_in-cy)/fy;

	double x_ = x;
	double y_ = y;

	for(int i=0; i<20; i++)
	{
		double r2 = x*x+y*y;
		double k_radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
		x = x_ - (2*p1*x*y + p2*(r2+2*x*x));
		y = y_ - (p1*(r2+2*y*y) + 2*p2*x*y);
		x /= k_radial;
		y /= k_radial;
	}

	x = x*fx+cx;
	y = y*fy+cy;
}

void distort_a_point( double x_in, double y_in, 
					  double &x, double &y,
					  vnl_matrix_fixed<double, 3, 3> const&K,
					  vnl_vector<double> const&dist_coef )
{
	double k1=0, k2=0, k3=0;
	double p1=0, p2=0;
	switch(dist_coef.size())
	{
	case 0:
	case 1:
	case 3:
		printf("Error: distort_coef has incorrect size!\n");
		return;
	case 5:
		k3 = dist_coef[4];
	case 4:
		p1 = dist_coef[2];
		p2 = dist_coef[3];
	case 2:
		k1 = dist_coef[0];
		k2 = dist_coef[1];
	default:
		break;
	}

	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];

	x = (x_in-cx)/fx;
	y = (y_in-cy)/fy;

	double r2 = x*x+y*y;
	double radial_distort = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2;
	x = x*radial_distort + 2*p1*x*y + p2*(r2+2*x*x);
	y = y*radial_distort + p1*(r2+2*y*y) + 2*p2*x*y;

	x = x*fx+cx;
	y = y*fy+cy;
}

void undistort_a_point( double x_in, double y_in, 
						double &x, double &y,
						cv::Mat const*K,
					    cv::Mat const*dist_coef )
{
	double k1=0, k2=0, k3=0;
	double p1=0, p2=0;
	switch(MAX(dist_coef->rows, dist_coef->cols))
	{
	case 0:
	case 1:
	case 3:
		printf("Error: distort_coef has incorrect size!\n");
		return;
	case 5:
		k3 = dist_coef->ptr<double>()[4];
	case 4:
		p1 = dist_coef->ptr<double>()[2];
		p2 = dist_coef->ptr<double>()[3];
	case 2:
		k1 = dist_coef->ptr<double>()[0];
		k2 = dist_coef->ptr<double>()[1];
	default:
		break;
	}

	double fx = K->at<double>(0, 0);;
	double cx = K->at<double>(0, 2);;
	double fy = K->at<double>(1, 1);;
	double cy = K->at<double>(1, 2);;

	x = (x_in-cx)/fx;
	y = (y_in-cy)/fy;

	double x_ = x;
	double y_ = y;

	for(int i=0; i<20; i++)
	{
		double r2 = x*x+y*y;
		double k_radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2;
		x = x_ - (2*p1*x*y + p2*(r2+2*x*x));
		y = y_ - (p1*(r2+2*y*y) + 2*p2*x*y);
		x /= k_radial;
		y /= k_radial;
	}

	x = x*fx+cx;
	y = y*fy+cy;
}

void distort_a_point( double x_in, double y_in, 
					  double &x, double &y,
					  cv::Mat const*K,
					  cv::Mat const*dist_coef )
{
	double k1=0, k2=0, k3=0;
	double p1=0, p2=0;
	switch(MAX(dist_coef->rows, dist_coef->cols))
	{
	case 0:
	case 1:
	case 3:
		printf("Error: distort_coef has incorrect size!\n");
		return;
	case 5:
		k3 = dist_coef->ptr<double>()[4];
	case 4:
		p1 = dist_coef->ptr<double>()[2];
		p2 = dist_coef->ptr<double>()[3];
	case 2:
		k1 = dist_coef->ptr<double>()[0];
		k2 = dist_coef->ptr<double>()[1];
	default:
		break;
	}

	double fx = K->at<double>(0, 0);;
	double cx = K->at<double>(0, 2);;
	double fy = K->at<double>(1, 1);;
	double cy = K->at<double>(1, 2);;

	x = (x_in-cx)/fx;
	y = (y_in-cy)/fy;

	double r2 = x*x+y*y;
	double radial_distort = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2;
	x = x*radial_distort + 2*p1*x*y + p2*(r2+2*x*x);
	y = y*radial_distort + p1*(r2+2*y*y) + 2*p2*x*y;

	x = x*fx+cx;
	y = y*fy+cy;
}

/* implementation based on corresponding OpenCv functions, please refer following URL for more details
 * http://opencv.willowgarage.com/documentation/c/calib3d_camera_calibration_and_3d_reconstruction.html#initundistortrectifymap
 * given distortion vector (k1, k2, p1, p2, [k3])
 * x<--(u-cx)/fx
 * y<--(v-cy)/fy
 * x'<--distort(x)
 * y'<--distrot(y)
 * mapx(u, v)<--x'fx + cx
 * mapy(u, v)<--y'fy + cy
 */

bool build_undistortion_map( vnl_matrix<double> &MapX, 
							 vnl_matrix<double> &MapY, 
							 vnl_matrix_fixed<double,3,3> &K, 
							 vnl_vector<double> &dist_coef,
							 vnl_vector_fixed<int, 2> &image_size)
{
	int width = image_size[0];
	int height = image_size[1];
	MapX.set_size(height, width);
	MapY.set_size(height, width);

	double k1=0, k2=0, k3=0;
	double p1=0, p2=0;
	switch(dist_coef.size())
	{
	case 0:
	case 1:
	case 3:
		return false;
	case 5:
		k3 = dist_coef[4];
	case 4:
		p1 = dist_coef[2];
		p2 = dist_coef[3];
	case 2:
		k1 = dist_coef[0];
		k2 = dist_coef[1];
	default:
		break;
	}

	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];

	for(int v=0; v<height; v++)
	{
		for(int u=0; u<width; u++)
		{
			double x = (u-cx)/fx;
			double y = (v-cy)/fy;
			double r2 = x*x + y*y;
			double r4 = r2*r2;
			double r6 = r4*r2;
			double radial_distort = 1+k1*r2+k2*r4+k3*r6;
			double x_p = x*radial_distort + 2*p1*x*y + p2*(r2+2*x*x);
			double y_p = y*radial_distort + p1*(r2+2*y*y) + 2*p2*x*y;

			MapX[v][u] = x_p*fx+cx;
			MapY[v][u] = y_p*fy+cy;
		}
	}

	return true;
}