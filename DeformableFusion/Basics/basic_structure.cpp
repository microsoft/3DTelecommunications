#include "stdafx.h"
#include "basic_structure.h"

bool apply_mask_to_depthMap( vnl_matrix<double> &depthMap, cv::Mat const& mask)
{
	if( depthMap.rows() != mask.rows ||
		depthMap.cols() != mask.cols )
	{
		printf("Error: the sizes of depth map and mask do not match!\n");
		return false;
	}

	for(int i= 0; i<depthMap.rows(); i++)
	{
		for(int j=0; j<depthMap.cols(); j++)
		{
			if( mask.at<uchar>(i, j) == 0 )
				depthMap[i][j] = 0;
		}
	}
	return true;
}

bool apply_mask_to_depthMap( cv::Mat& depthMap, cv::Mat const& mask)
{
	if( depthMap.rows != mask.rows ||
		depthMap.cols != mask.cols )
	{
		printf("Error: the sizes of depth map and mask do not match!\n");
		return false;
	}

	for(int i= 0; i<depthMap.rows; i++)
	{
		for(int j=0; j<depthMap.cols; j++)
		{
			if( mask.at<uchar>(i, j) == 0 )
				depthMap.at<double>(i, j) = 0.0;
		}
	}
	return true;
}

bool apply_mask_to_cvMat(cv::Mat& img, cv::Mat const& mask, int outer_val, bool bScaleImage)
{
	if( img.rows != mask.rows ||
		img.cols != mask.cols )
	{
		printf("Error: the sizes of image and mask do not match!\n");
		return false;
	}

	for(int i= 0; i<img.rows; i++)
	{
		for(int j=0; j<img.cols; j++)
		{
			if( mask.at<uchar>(i, j) == 0 )
			{
				if (img.channels() == 3 && img.depth() == CHAR_BIT)
				{
					img.at<cv::Vec3b>(i, j)[0] = outer_val;
					img.at<cv::Vec3b>(i, j)[1] = outer_val;
					img.at<cv::Vec3b>(i, j)[2] = outer_val;
				}
				else if (img.channels() == 1 && img.depth() == CHAR_BIT*2)
				{
					img.at<unsigned short>(i, j) = outer_val;
				}
				else if (img.channels() == 1 && img.depth() == CHAR_BIT)
				{
					img.at<uchar>(i, j) = outer_val;
				}
			}
			else
			{
				if( bScaleImage )
				{
					double s = mask.at<uchar>(i, j)/255.0;
					if (img.channels() == 1 && img.depth() == CHAR_BIT)
					{
						img.at<uchar>(i, j) *= s;
					}
					else if (img.channels() == 1 && img.depth() == CHAR_BIT*2)
					{
						img.at<unsigned short>(i, j) *= s;
					}
					else if (img.channels() == 3 && img.depth() == CHAR_BIT)
					{
						img.at<cv::Vec3b>(i, j)[0] *= s;
						img.at<cv::Vec3b>(i, j)[1] *= s;
						img.at<cv::Vec3b>(i, j)[2] *= s;
					}
				}
			}

		}
	}
	return true;	
}

cv::Mat mask_from_depthMap( vnl_matrix<double> const& depthMap)
{
	int h = depthMap.rows();
	int w = depthMap.cols();
	if( h<=0 || w<=0)
		return cv::Mat();
	cv::Mat ret = cv::Mat(h, w, CV_8UC1);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( depthMap[i][j] > 0 )
				ret.at<uchar>(i, j) = 255;
			else
				ret.at<uchar>(i, j) = 0;
		}
	}
	return ret;
}

cv::Mat mask_from_depthMap( cv::Mat const& depthMap)
{
	int h = depthMap.rows;
	int w = depthMap.cols;
	if( h<=0 || w<=0)
		return cv::Mat();
	cv::Mat ret = cv::Mat(h, w, CV_8UC1);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( depthMap.at<double>(i, j) > 0 )
				ret.at<uchar>(i, j) = 255;
			else
				ret.at<uchar>(i, j) = 0;
		}
	}
	return ret;
}

BoundingBox3D extract_bounding_box(cv::Mat const& depthMat, GCameraView const* cam, cv::Mat const& mask, int thres_blob_size)
{
	vector<cv::Mat> depthMats;
	depthMats.push_back((cv::Mat)depthMat);
	vector<GCameraView*> cams;
	cams.push_back( (GCameraView*)cam);
	vector<cv::Mat> masks;
	masks.push_back( (cv::Mat) mask);

	return extract_bounding_box(depthMats, cams, masks, thres_blob_size);
}

BoundingBox3D extract_bounding_box(vector<cv::Mat> depthMats, vector<GCameraView const*> cams, vector<cv::Mat> masks, int thres_blob_size)
{
	assert( depthMats.size() == cams.size() );
	int cam_num = cams.size();

	double x_min = 1.0e+20;
	double x_max = -1.0e+20;
	double y_min = 1.0e+20;
	double y_max = -1.0e+20;
	double z_min = 1.0e+20;
	double z_max = -1.0e+20;

	vector<cv::Mat> masks_new;
	if( thres_blob_size > 0 )
	{
		for(int i=0; i<depthMats.size(); i++)
		{
			if( depthMats[i].empty() )
			{
				masks_new.push_back(cv::Mat());
				continue;
			}

			cv::Mat mask = cv::Mat();
			if(masks.size() > i )
				mask = masks[i];

			cv::Mat labelImg = cv::Mat(depthMats[i].rows, depthMats[i].cols, CV_8UC1, 0);
			vector<CComponent> m_ccomps;
			CConnectedComponent::FindDConnectedComps(depthMats[i], mask, labelImg, m_ccomps, 5.0, thres_blob_size);
			masks_new.push_back(labelImg);
		}
	}
	else
	{
		for (int i = 0; i < masks.size(); i++)
			masks_new.push_back(masks[i]);
	}

	for(int c=0; c<cam_num; c++)
	{
		cv::Mat depthMat = depthMats[c];

		cv::Mat mask = cv::Mat();
		if(masks_new.size() > c )
			mask = masks_new[c];
		
		if( depthMat.empty() )
			continue;

		int h = depthMats[c].rows;
		int w = depthMats[c].cols;
		double scale_factor_x = w/double(cams[c]->m_Width);
		double scale_factor_y = h/double(cams[c]->m_Height);
		double fx = cams[c]->K[0][0];
		double fy = cams[c]->K[1][1];
		double cx = cams[c]->K[0][2];
		double cy = cams[c]->K[1][2];
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				if( !mask.empty() && mask.at<uchar>(i, j) == 0 )
					continue;
				
				double z = depthMat.at<double>(i, j);;
				if(z <= 0)
					continue;

				double x = (j/scale_factor_x - cx)*z/fx;
				double y = (i/scale_factor_y - cy)*z/fy;

				double p_cam[3];
				p_cam[0] = x; p_cam[1] = y; p_cam[2] = z;
				double p_wld[3];
				cams[c]->cameraSpace2worldSpace(p_cam, p_wld);
				if( x_min > p_wld[0] )
					x_min = p_wld[0];
				if( x_max < p_wld[0] )
					x_max = p_wld[0];
				if( y_min > p_wld[1] )
					y_min = p_wld[1];
				if( y_max < p_wld[1] )
					y_max = p_wld[1];
				if( z_min > p_wld[2] )
					z_min = p_wld[2];
				if( z_max < p_wld[2] )
					z_max = p_wld[2];
			}
		}
	}

	if( thres_blob_size > 0 )
		releaseCvMats(masks_new);


	if( x_min >= x_max || y_min >= y_max || z_min >= z_max)
		return BoundingBox3D(0, 0, 0, 0, 0, 0);
	else
		return BoundingBox3D(x_min, x_max, y_min, y_max, z_min, z_max);
}

double extract_depth_at_visual_feature( vnl_matrix<double> const &depthMap, 
										 double x, double y,
										 depth_extract_method method)
{
	if( x < 3 || x > depthMap.cols() - 3 ||
		y < 3 || y > depthMap.rows() - 3 )
		return 0.0;

	int row = ROUND(y);
	int col = ROUND(x);

	if( method == depth_extract_NN)
	{
		if( depthMap[row][col] > 0.0 )
			return depthMap[row][col];
		else if( depthMap[row][col-1] > 0.0)
			return depthMap[row][col-1];
		else if( depthMap[row][col+1] > 0.0)
			return depthMap[row][col+1];
		else if( depthMap[row-1][col] > 0.0)
			return depthMap[row-1][col];
		else if( depthMap[row+1][col] > 0.0)
			return depthMap[row+1][col];
		else if( depthMap[row+1][col+1] > 0.0)
			return depthMap[row+1][col+1];
		else if( depthMap[row+1][col-1] > 0.0)
			return depthMap[row+1][col-1];
		else if( depthMap[row-1][col+1] > 0.0)
			return depthMap[row-1][col+1];
		else if( depthMap[row-1][col-1] > 0.0)
			return depthMap[row-1][col-1];
		else
			return 0.0;
	}

	if( method == depth_extract_InterLinear )
	{
		int y1 = (int)y;
		int x1 = (int)x;
		int y2 = y1+1;
		int x2 = x1+1;
		double d1 = depthMap[y1][x1];
		double d2 = depthMap[y1][x2];
		double d3 = depthMap[y2][x1];
		double d4 = depthMap[y2][x2];
		double d_min = MIN(d1, MIN(d2, MIN(d3, d4)));
		double d_max = MAX(d1, MAX(d2, MAX(d3, d4)));

		//the depths at four points must be close enough to do interpolation
		if( d1 > 0 && d2 > 0 &&
			d3 > 0 && d4 > 0 && 
			d_max-d_min < 5.0)
		{
			return pickAMatElement(depthMap, x, y);
		}
		else
			return extract_depth_at_visual_feature(depthMap, x, y, depth_extract_NN);
	}

	if( method == depth_extract_simpleAvg )
	{
		double z_mean = 0.0;
		int num = 0;
		for(int i=row-1; i<=row+1; i++)
		{
			for(int j=col-1; j<=col+1; j++)
			{
				if( depthMap[i][j] <= 0.0 )
					continue;

				z_mean += depthMap[i][j];
				num++;
			}
		}
		if( num == 0)
			return 0.0;

		z_mean /= num;

		return z_mean;
	}

	if( method == depth_extract_complexAvg)
	{
		double z_mean = 0.0;
		int num = 0;
		for(int i=row-2; i<=row+2; i++)
		{
			for(int j=col-2; j<=col+2; j++)
			{
				if( depthMap[i][j] <= 0.0 )
					continue;

				z_mean += depthMap[i][j];
				num++;
			}
		}
		if( num == 0)
			return 0.0;

		z_mean /= num;

		bool b_flat_area = true;
		double thres = 3.0 + z_mean/500.0*3.0;
		for(int i=row-2; i<=row+2; i++)
		{
			for(int j=col-2; j<=col+2; j++)
			{
				if( depthMap[i][j] <= 0.0 )
					continue;

				if( abs(z_mean - depthMap[i][j]) > thres )
				{
					b_flat_area = false;
					break;
				}
			}
		}

		if( b_flat_area )
		{
			double z = pickAMatElement(depthMap, x, y);
			if( abs( z-z_mean ) < thres )
				return z;
			else
				return z_mean;
		}
		else
			return 0.0;
	}

	return 0.0;
}

double extract_depth_at_visual_feature( cv::Mat const& depthMap,
										 double x, double y,
										 depth_extract_method method)
{
	if( x < 3 || x > depthMap.cols - 3 ||
		y < 3 || y > depthMap.rows - 3 )
		return 0.0;

	int row = ROUND(y);
	int col = ROUND(x);

	if( method == depth_extract_NN)
	{
		if( depthMap.at<double>(row, col) > 0.0 )
			return depthMap.at<double>(row, col);
		else if( depthMap.at<double>(row, col-1) > 0.0)
			return depthMap.at<double>(row, col-1);
		else if( depthMap.at<double>(row, col+1) > 0.0)
			return depthMap.at<double>(row, col+1);
		else if( depthMap.at<double>(row-1, col) > 0.0)
			return depthMap.at<double>(row-1, col);
		else if( depthMap.at<double>(row+1, col) > 0.0)
			return depthMap.at<double>(row+1, col);
		else if( depthMap.at<double>(row+1, col+1) > 0.0)
			return depthMap.at<double>(row+1, col+1);
		else if( depthMap.at<double>(row+1, col-1) > 0.0)
			return depthMap.at<double>(row+1, col-1);
		else if( depthMap.at<double>(row-1, col+1) > 0.0)
			return depthMap.at<double>(row-1, col+1);
		else if( depthMap.at<double>(row-1, col-1) > 0.0)
			return depthMap.at<double>(row-1, col-1);
		else
			return 0.0;
	}

	if( method == depth_extract_InterLinear )
	{
		int y1 = (int)y;
		int x1 = (int)x;
		int y2 = y1+1;
		int x2 = x1+1;
		double d1 = depthMap.at<double>(y1, x1);
		double d2 = depthMap.at<double>(y1, x2);
		double d3 = depthMap.at<double>(y2, x1);
		double d4 = depthMap.at<double>(y2, x2);
		double d_min = MIN(d1, MIN(d2, MIN(d3, d4)));
		double d_max = MAX(d1, MAX(d2, MAX(d3, d4)));

		//the depths at four points must be close enough to do interpolation
		if( d1 > 0 && d2 > 0 &&
			d3 > 0 && d4 > 0 && 
			d_max-d_min < 5.0)
		{
			return pickAMatElement(depthMap, x, y);
		}
		else
			return extract_depth_at_visual_feature(depthMap, x, y, depth_extract_NN);
	}

	if( method == depth_extract_simpleAvg )
	{
		double z_mean = 0.0;
		int num = 0;
		for(int i=row-1; i<=row+1; i++)
		{
			for(int j=col-1; j<=col+1; j++)
			{
				if( depthMap.at<double>(i, j) <= 0.0 )
					continue;

				z_mean += depthMap.at<double>(i, j);
				num++;
			}
		}
		if( num == 0)
			return 0.0;

		z_mean /= num;

		return z_mean;
	}

	if( method == depth_extract_complexAvg)
	{
		double z_mean = 0.0;
		int num = 0;
		for(int i=row-2; i<=row+2; i++)
		{
			for(int j=col-2; j<=col+2; j++)
			{
				if( depthMap.at<double>(i, j) <= 0.0 )
					continue;

				z_mean += depthMap.at<double>(i, j);
				num++;
			}
		}
		if( num == 0)
			return 0.0;

		z_mean /= num;

		bool b_flat_area = true;
		double thres = 3.0 + z_mean/500.0*3.0;
		for(int i=row-2; i<=row+2; i++)
		{
			for(int j=col-2; j<=col+2; j++)
			{
				if( depthMap.at<double>(i, j) <= 0.0 )
					continue;

				if( abs(z_mean - depthMap.at<double>(i, j)) > thres )
				{
					b_flat_area = false;
					break;
				}
			}
		}

		if( b_flat_area )
		{
			double z = pickAMatElement(depthMap, x, y);
			if( abs( z-z_mean ) < thres )
				return z;
			else
				return z_mean;
		}
		else
			return 0.0;
	}
}

bool extract_depth_contour(cv::Mat const& depthMat,
	std::vector<vnl_vector_fixed<int, 2>> &points_2d,
	double thres_dist_nn,
	cv::Mat& img
	)
{
	const int dx[8] = { -1, 1,  0, 0, -1, 1, -1,  1};
	const int dy[8] = {  0, 0, -1, 1,  1, 1, -1, -1 };
	if (depthMat.empty())
		return false;
	int w = depthMat.cols;
	int h = depthMat.rows;

	if (!img.empty())
	{
		if (img.cols != w || img.rows != h)
		{
			LOGGER()->error("extract_depth_boundary", "image and depth have different size");
			return false;
		}
		img = cv::Scalar(0);
	}

	for (int i = 1; i < h-1; i++)
	{
		for (int j = 1; j < w-1; j++)
		{
			double d = depthMat.at<double>(i, j);;
			if (d <= 0.0)
				continue;

			for (int k = 0; k < 4; k++)
			{
				double d_n = depthMat.at<double>(i+dy[k], j+dx[k]);
				if (d_n <= 0.0 || d < d_n - thres_dist_nn)
				{
					points_2d.push_back(vnl_vector_fixed<int, 2>(j, i));
					if (!img.empty())
						img.at<uchar>(i, j) = 255;
					break;
				}				
			}
		}
	}

	return true;
}

cv::Mat contour_to_image(std::vector<vnl_vector_fixed<int, 2>> & points_contour, int width, int height)
{
	cv::Mat img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
	for (int i = 0; i < points_contour.size(); i++)
	{
		int x = points_contour[i][0];
		int y = points_contour[i][1];
		if (x >= 0 && x < width &&
			y >= 0 && y < height)
		{
			img.at<uchar>(y, x) = 255;
		}
	}
	return img;
}
