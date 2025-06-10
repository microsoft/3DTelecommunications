// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "KL_features.h"

template <class T>
bool drawCornersOnImage(cv::Mat& img, vnl_matrix<T> const& corners, cv::Scalar color)
{
	if (img.empty())
		return false;
	for (int i = 0; i < corners.rows(); i++)
	{
		double x = corners[i][0];
		double y = corners[i][1];

		cv::Point p1 = cv::Point(ROUND(x - 3), ROUND(y - 3));
		cv::Point p2 = cv::Point(ROUND(x + 3), ROUND(y + 3));
		cv::rectangle(img, p1, p2, color, 1);
	}
	return true;
}

template <class T>
bool drawCornersOnImage(cv::Mat& img, vector< vnl_vector_fixed<T, 2> > const& corners, cv::Scalar color)
{
	if (img.empty())
		return false;
	for (int i = 0; i < corners.size(); i++)
	{
		double x = corners[i][0];
		double y = corners[i][1];

		cv::Point p1 = cv::Point(ROUND(x - 3), ROUND(y - 3));
		cv::Point p2 = cv::Point(ROUND(x + 3), ROUND(y + 3));
		cv::rectangle(img, p1, p2, color, 1);
	}
	return true;
}

//will NOT contaminate the input image
template<class T>
bool drawCornersOnImageAndSave(cv::Mat& img,
	vnl_matrix<T> const& corners,
	char const* filename,
	cv::Scalar color)
{
	if (filename == NULL)
	{
		printf("Error<drawCornersOnImageAndSave>: file name null!\n");
		return false;
	}

	cv::Mat img_bak = img.clone();
	bool ret = drawCornersOnImage(img_bak, corners, color);
	cv::imwrite(filename, img_bak);
	img_bak.release();
	return ret;
}

//will NOT contaminate the input image
template<class T>
bool drawCornersOnImageAndSave(cv::Mat& img,
	vector< vnl_vector_fixed<T, 2> > const& corners,
	char const* filename,
	cv::Scalar color)
{
	if (filename == NULL)
	{
		printf("Error<drawCornersOnImageAndSave>: file name null!\n");
		return false;
	}

	cv::Mat img_bak = img.clone();
	bool ret = drawCornersOnImage(img_bak, corners, color);
	imwrite(filename, img_bak);
	img_bak.release();
	return ret;
}


template<class T>
bool drawMatchedCornersOnImages(cv::Mat const& img_1, cv::Mat const& img_2, vnl_matrix<T>& matched_corners,
	const char* filename)
{
	if (img_1.cols != img_2.cols ||
		img_1.rows != img_2.rows)
		return false;

	if (img_1.channels() != 3 ||
		img_2.channels() != 3)
		return false;

	int w = img_1.cols;
	int h = img_1.rows;
	cv::Mat img = cv::Mat(h, 2 * w, CV_MAKETYPE(8, 3));
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			img.at<cv::Vec3b>(i, j)[0] = img_1.at<cv::Vec3b>(i, j)[0];
			img.at<cv::Vec3b>(i, j)[1] = img_1.at<cv::Vec3b>(i, j)[1];
			img.at<cv::Vec3b>(i, j)[2] = img_1.at<cv::Vec3b>(i, j)[2];
		}

		for (int j = 0; j < w; j++)
		{
			img.at<cv::Vec3b>(i, j + w)[0] = img_2.at<cv::Vec3b>(i, j)[0];
			img.at<cv::Vec3b>(i, j + w)[1] = img_2.at<cv::Vec3b>(i, j)[1];
			img.at<cv::Vec3b>(i, j + w)[2] = img_2.at<cv::Vec3b>(i, j)[2];
		}
	}

	for (int i = 0; i < matched_corners.rows(); i++)
	{
		T x1 = matched_corners[i][0];
		T y1 = matched_corners[i][1];

		cv::Point p1 = cv::Point(ROUND(x1 - 2), ROUND(y1 - 2));
		cv::Point p2 = cv::Point(ROUND(x1 + 2), ROUND(y1 + 2));
		cv::rectangle(img, p1, p2, CV_RGB(0, 255.0, 0), 1);

		double x2 = matched_corners[i][2] + w;
		double y2 = matched_corners[i][3];

		cv::Point p3 = cv::Point(ROUND(x2 - 2), ROUND(y2 - 2));
		cv::Point p4 = cv::Point(ROUND(x2 + 2), ROUND(y2 + 2));
		cv::rectangle(img, p3, p4,  CV_RGB(0, 255.0, 0), 1);

		if (matched_corners.rows() > 100 && i % 4 != 0)
			continue;

		cv::Point p5 = cv::Point(ROUND(x1), ROUND(y1));
		cv::Point p6 = cv::Point(ROUND(x2), ROUND(y2));
		cv::line(img, p5, p6, CV_RGB(0,0,255), 1);
	}
	imwrite(filename, img);

	img.release();
	return true;
}

bool extract_corner_points(cv::Mat const& img, cv::Mat const& mask_img, vnl_matrix<double> &corners_out,
							int max_corner_num, double quality_level, double min_dist_btw_coners,bool bHistEqual)
{
	if( img.empty() )
		return false;

	int h = img.rows;
	int w = img.cols;
	bool bReleaseGrayImage = false;
	cv::Mat gray = img;
	if( img.channels() > 1 )
	{
		bReleaseGrayImage = true;
		gray = cv::Mat(h,w, CV_MAKETYPE(img.depth(), 1));
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
		if( bHistEqual )
			cv::equalizeHist(gray, gray);
	}

	//extract corners
	vector<cv::Point2f> corners;
	
	double qualityLevel = quality_level;//0.005;
	double minDistance = min_dist_btw_coners;//5.0;)
	
	cv::goodFeaturesToTrack(gray, corners, max_corner_num, qualityLevel, minDistance, mask_img);

	//refine corners
	int refineMaxIter = 500;
	double refineEpsilon = 1.0E-8;
	cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, refineMaxIter, refineEpsilon));
	if( !mask_img.empty() )
	{
		int inlier_count = 0;
		for(int i=0; i<corners.size(); i++)
		{
			int u = ROUND(corners[i].x);
			int v = ROUND(corners[i].y);
			if( u < 0 || v < 0 || u >= w || v >= h )
				continue;

			if( mask_img.at<uchar>(ROUND(corners[i].y), ROUND(corners[i].x)) != 0)
				inlier_count++;
		}
		corners_out.set_size(inlier_count, 2);
		int idx = 0;
		for(int i=0; i< corners.size(); i++)
		{
			int u = ROUND(corners[i].x);
			int v = ROUND(corners[i].y);
			if( u < 0 || v < 0 || u >= w || v >= h )
				continue;

			if( mask_img.at<uchar>(ROUND(corners[i].y), ROUND(corners[i].x)) != 0)
			{
				corners_out[idx][0] = corners[i].x;
				corners_out[idx][1] = corners[i].y;
				idx++;
			}
		}
	}
	else
	{
		corners_out.set_size(corners.size(), 2);
		int idx = 0;
		for(int i=0; i< corners.size(); i++)
		{
			corners_out[i][0] = corners[i].x;
			corners_out[i][1] = corners[i].y;
		}
	}

	if (bReleaseGrayImage)
		gray.release();
	return true;
}

bool track_corner_points(cv::Mat const& img_prev, vnl_matrix<double> const &corners_prev,
						 cv::Mat const& img_cur, cv::Mat const& mask_cur,
						 vnl_matrix<double> &matched_corners,
						 int search_radius,
						 bool bHistEqual)
{
	vector<int> matched_corner_indices_prev;
	return track_corner_points(img_prev, corners_prev, img_cur, mask_cur, matched_corners, matched_corner_indices_prev, search_radius, bHistEqual);
}

bool track_corner_points(cv::Mat const& img_prev, vnl_matrix<double> const &corners_prev,
						 cv::Mat const& img_cur, cv::Mat const& mask_cur,
						 vnl_matrix<double> &matched_corners, 
						 vector<int> &matched_corner_indices_prev,
						 int search_radius,
						 bool bHistEqual)
{
	int corner_count = corners_prev.rows();
	vector<cv::Point2f> prevFeatures = vector<cv::Point2f>();
	vector<cv::Point2f> currFeatures = vector<cv::Point2f>();
	for(int i=0; i<corners_prev.rows(); i++)
	{
		cv::Point2f prevFeature;
		prevFeature.x = corners_prev[i][0];
		prevFeature.y = corners_prev[i][1];
		prevFeatures.push_back(prevFeature);

		cv::Point2f currFeature;
		currFeature.x = corners_prev[i][0];
		currFeature.y = corners_prev[i][1];
		currFeatures.push_back(currFeature);
	}
	
	cv::Mat gray_prev = img_prev;
	cv::Mat gray_cur = img_cur;
	bool bReleaseGrayImage_prev = false;
	bool bReleaseGrayImage_cur = false;
	if( img_prev.channels() > 1 )
	{
		bReleaseGrayImage_prev = true;
		gray_prev = cv::Mat(img_prev.rows, img_prev.cols, CV_MAKETYPE(img_prev.depth(), 1));
		cv::cvtColor(img_prev, gray_prev, cv::COLOR_BGR2GRAY);
	}
	if( img_cur.channels() > 1 )
	{
		bReleaseGrayImage_cur = true;
		gray_cur = cv::Mat(img_cur.rows, img_cur.cols, CV_MAKETYPE(img_cur.depth(), 1));
		cv::cvtColor(img_cur, gray_cur, cv::COLOR_BGR2GRAY);
	}
	if( bHistEqual )
	{
		cv::equalizeHist(gray_prev, gray_prev);
		cv::equalizeHist(gray_cur, gray_cur);
	}

	int refineMaxIter = 500;
	double refineEpsilon = 1.0E-8;
	vector<unsigned char> status;
	vector<float> err;
	cv::TermCriteria termCrit;
	termCrit.type = cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER;
	cv::calcOpticalFlowPyrLK(gray_prev, gray_cur, prevFeatures, currFeatures,
		status, err, cv::Size(search_radius * 2 + 1, search_radius * 2 + 1), 1,
		termCrit, cv::OPTFLOW_USE_INITIAL_FLOW);

	vector< bool > bMatchFound(corner_count);
	int match_count = 0;
	for(int i=0; i<corner_count; i++)
	{
		if( status[i] != 1 )
		{
			bMatchFound[i] = false;
			continue;
		}

		int x = ROUND( currFeatures[i].x );
		int y = ROUND( currFeatures[i].y );
		if( mask_cur.empty() ||
			(x > 0 && x < mask_cur.cols &&
			 y > 0 && y < mask_cur.rows &&
			 mask_cur.at<uchar>(y, x) != 0) )
		{
			bMatchFound[i] = true;
			match_count++;
		}
		else
			bMatchFound[i] = false;
	}

	matched_corners.set_size(match_count, 4);
	matched_corner_indices_prev.resize(match_count, 0);
	int count = 0;
	for(int i=0; i<corner_count; i++)
	{
		if( !bMatchFound[i] )	continue;

		matched_corner_indices_prev[count] = i;
		matched_corners[count][0] = prevFeatures[i].x;
		matched_corners[count][1] = prevFeatures[i].y;
		matched_corners[count][2] = currFeatures[i].x;
		matched_corners[count][3] = currFeatures[i].y;
		count++;
	}

	if (bReleaseGrayImage_prev && !gray_prev.empty())
		gray_prev.release();
	if (bReleaseGrayImage_cur && !gray_cur.empty())
		gray_cur.release();
	return true;
}


bool extract_corner_tracks_bracket( int frmIdx_cen, std::vector<cv::Mat> const& imgs, std::vector<cv::Mat> const& masks,
									vector< vector<ImagePointAndFrmIdx> > &corner_point_tracks )
{
	int search_radius = 40;
	if( frmIdx_cen >= imgs.size() )
	{
		printf("Error<extract_corner_points_tracks_bracket>: illegal frmIdx_cen<%d>\n", frmIdx_cen);
		return false;
	}

	cv::Mat img_cen = imgs[frmIdx_cen];
	cv::Mat mask_cen = masks[frmIdx_cen];

	vnl_matrix<double> corners_cen;
	extract_corner_points(img_cen, mask_cen, corners_cen, 2000, 0.006, 5.0, true);
	corner_point_tracks.resize(corners_cen.rows());
	for(int i=0; i<corners_cen.rows(); i++)
	{
		corner_point_tracks[i].push_back( ImagePointAndFrmIdx(frmIdx_cen, vnl_vector_fixed<double, 2>(corners_cen(i, 0), corners_cen(i, 1))) );
	}

	//matched with frames following frmIdx_cen
	vector<int> trkIdx_of_prev_corners;
	for(int i=0; i<corners_cen.size(); i++)
		trkIdx_of_prev_corners.push_back(i);
	cv::Mat img_prev = img_cen;
	vnl_matrix<double> corners_prev = corners_cen;
	int frmIdx_prev = frmIdx_cen;
	for(int frmIdx=frmIdx_cen+1; frmIdx<imgs.size(); frmIdx++)
	{
		cv::Mat img_cur = imgs[frmIdx];
		cv::Mat mask_cur = masks[frmIdx];

		vnl_matrix<double> matched_corners;
		vector<int> matched_corners_indices_prev;
		track_corner_points(img_prev, corners_prev, img_cur, mask_cur, matched_corners, matched_corners_indices_prev, search_radius, true);

		vector<int> trkIdx_of_cur_corners(matched_corners.rows(), -1);
		for(int i=0; i<matched_corners.rows(); i++)
		{
			int prev_corner_idx = matched_corners_indices_prev[i];
			int trk_idx = trkIdx_of_prev_corners[prev_corner_idx];
			trkIdx_of_cur_corners[i] = trk_idx;

			corner_point_tracks[trk_idx].push_back(ImagePointAndFrmIdx(frmIdx, vnl_vector_fixed<double, 2>(matched_corners(i, 2), matched_corners(i, 3))) );
		}
		img_prev = img_cur;
		corners_prev = matched_corners.get_n_columns(2, 2);
		frmIdx_prev = frmIdx;
		trkIdx_of_prev_corners = trkIdx_of_cur_corners;
	}

	//matched with frames ahead of frmIdx_cen
	trkIdx_of_prev_corners.clear();
	for(int i=0; i<corners_cen.size(); i++)
		trkIdx_of_prev_corners.push_back(i);
	img_prev = img_cen;
	corners_prev = corners_cen;
	frmIdx_prev = frmIdx_cen;
	for(int frmIdx=frmIdx_cen-1; frmIdx>=0; frmIdx--)
	{
		cv::Mat img_cur = imgs[frmIdx];
		cv::Mat mask_cur = masks[frmIdx];

		vnl_matrix<double> matched_corners;
		vector<int> matched_corners_indices_prev;
		track_corner_points(img_prev, corners_prev, img_cur, mask_cur, matched_corners, matched_corners_indices_prev, search_radius, true);

		vector<int> trkIdx_of_cur_corners(matched_corners.rows(), -1);
		for(int i=0; i<matched_corners.rows(); i++)
		{
			int prev_corner_idx = matched_corners_indices_prev[i];
			int trk_idx = trkIdx_of_prev_corners[prev_corner_idx];
			trkIdx_of_cur_corners[i] = trk_idx;

			corner_point_tracks[trk_idx].push_back(ImagePointAndFrmIdx( frmIdx, 
																	    vnl_vector_fixed<double, 2>( matched_corners(i, 2), 
																									 matched_corners(i, 3) )
																	   )
												  );
		}
		img_prev = img_cur;
		corners_prev = matched_corners.get_n_columns(2, 2);
		frmIdx_prev = frmIdx;
		trkIdx_of_prev_corners = trkIdx_of_cur_corners;
	}

	return true;
}

bool draw_corner_tracks_and_save( std::vector<cv::Mat> const& imgs,
								  std::vector< vector<ImagePointAndFrmIdx> > const&corner_point_tracks,
								  char const* base_name)
{
	vector<cv::Mat> imgs_bak;
	for(int i=0; i<imgs.size(); i++)
	{
		cv::Mat img = imgs[i].clone();
		imgs_bak.push_back(img);
	}

	for(int i=0; i<corner_point_tracks.size(); i++)
	{
		cv::Scalar color = cv::Scalar(RANDOM*255, RANDOM*255, RANDOM*255, 0);
		for(int j=0; j<corner_point_tracks[i].size(); j++)
		{
			int frmIdx = corner_point_tracks[i][j].frmIdx;
			vnl_vector_fixed<double, 2> const& pt = corner_point_tracks[i][j].pt;

			cv::Point p1 = cv::Point(ROUND(pt[0]-3), ROUND(pt[1]-3));
			cv::Point p2 = cv::Point(ROUND(pt[0]+3), ROUND(pt[1]+3));
			cv::rectangle(imgs_bak[frmIdx], p1, p2, color, 1);	
		}
	}

	for(int i=0; i<imgs_bak.size(); i++)
	{
		char name[500];
		strcpy(name, base_name); chgFN(name, '*', i);
		cv::imwrite(name, imgs_bak[i]);
		imgs_bak[i].release();
	}

	return true;
}

