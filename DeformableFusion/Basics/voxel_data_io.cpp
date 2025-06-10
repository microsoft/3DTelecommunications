// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "voxel_data_io.h"
using namespace std;
// Sometimes MultiViewCalib can produce an invalid calib.txt that has crazy numbers for height and width
// this is used as a sanity check 
#define MAX_IMAGE_HEIGHT_OR_WIDTH 100000

bool read_calibration_bundle_qin(char const* file_name, std::vector<GCameraView*> &cams)
{
	ifstream in(file_name);

	if (!in)
	{
		cout << "Error<read_calibration_bundle_qin>: cannot open file: " << file_name << endl;
		return false;
	}

	string name;
	int version, w, h, nCams;
	vnl_matrix_fixed<double, 3, 3> K;
	K.set_identity();
	vnl_vector_fixed<double, 5> distort(0.0);
	vnl_vector_fixed<double, 3> rot(0.0);
	vnl_vector_fixed<double, 3> C;
	double colorScale[3], colorBias[3];

	in >> nCams;
	if (nCams == 0)    // no cameras exist
		return false;

	colorScale[0] = colorScale[1] = colorScale[2] = 1.0f;
	memset(colorBias, 0, sizeof(double)* 3);

	for (int i = 0; i < nCams; i++)
	{
		double fx, fy, px, py, s;
		in >> name >>  w >> h >> fx >> fy >> s >> px >> py;
		in >> distort;
		in >> rot;
		in >> C;
		in >> colorScale[0] >> colorScale[1] >> colorScale[2];
		in >> colorBias[0] >> colorBias[1] >> colorBias[2];

		K[0][0] = fx; K[1][1] = fy;
		K[0][1] = s;
		K[0][2] = px; K[1][2] = py;
		if (h > MAX_IMAGE_HEIGHT_OR_WIDTH || w > MAX_IMAGE_HEIGHT_OR_WIDTH)
		{
			return false;
		}
		GCameraView *cam = new GCameraView(h, w);
		cam->SetIntrinsics(K.data_block(), distort.data_block());
		vnl_matrix_fixed<double, 3, 3> R;
		rodrigues_to_matrix(rot, R);
		R = R.transpose();
		vnl_vector_fixed<double, 3> T = -R*C;
		T /= 10.0;
		cam->SetExtrinsics(R.data_block(), T.data_block());
		cam->name = name;

		cams.push_back(cam);
	}

	in.close();
	return true;
}

bool get_global_depth_calibration(std::vector<std::vector<GCameraView*>> const& cams_pods, std::vector<GCameraView*> const& color_cams,
								  std::vector<GCameraView*> &depth_cams)
{
	if (cams_pods.size() != color_cams.size())
	{
		printf("Error<get_global_depth_calibration>: camera size does not match!\n");
		return false;
	}

	int pods_num = cams_pods.size();
	for (int i = 0; i < pods_num; i++)
	{
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;
		camera_pose_to_transf(cams_pods[i][2], cams_pods[i][3], R, T);

		GCameraView *cam_depth = new GCameraView(*(cams_pods[i][3]));
		transf_to_camera_pose(color_cams[i], R, T, cam_depth);
		depth_cams.push_back(cam_depth);
	}

	return true;
}

cv::Mat read_depth_bin_qin(char const* file_name)
{
	cv::Mat img = read_depth_bin_qin_as_img(file_name);

	cv::Mat ret = loadDepthMatFromPNG(img, false);

	img.release();
	return ret;
}

cv::Mat read_depth_bin_qin_as_img(char const* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if (fp == NULL)
	{
		printf("Error<read_depth_bin_qin_as_img>: cannot open file %s\n", file_name);
		return cv::Mat();
	}

	int width, height, bbp;
	fscanf_s(fp, "%d%d\n%d\n", &width, &height, &bbp);
	if (bbp != 2)
	{
		printf("Error<read_depth_bin_qin>: bbp != 2\n");
		return cv::Mat();
	}

	cv::Mat img = cv::Mat(height, width, CV_16UC1);

	int size = width*height * 2;
	fseek(fp, -size, SEEK_END);
	for (int row = 0; row<height; row++)
	{
		unsigned short *p = (unsigned short *)(img.ptr<unsigned short>(row));
		int readsize = (int)fread((void *)p, 1, (size_t)width * 2, fp);
		if (readsize != width * 2)
			return cv::Mat();
	}
	fclose(fp);

	return img;
}
cv::Mat read_color_bin(char const* file_name, int width, int height, int &readColorSize)
{
	FILE* fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if (fp == NULL)
	{
		printf("Error<read_color_bin>: cannot open file %s\n", file_name);
		return cv::Mat();
	}

	cv::Mat img = cv::Mat(
		height,
		width,
		CV_32FC1);
	const int expectedFrameSize = width * height * sizeof(int);

	readColorSize =	(int)fread((void*)img.data, 1, expectedFrameSize, fp);

	fclose(fp);
	return img;

}

cv::Mat read_depth_bin_sean(char const* file_name, int width, int height)
{
	cv::Mat img = read_depth_bin_sean_as_img(file_name,  width,  height);

	cv::Mat ret = loadDepthMatFromPNG(img, false);

	img.release();
	return ret;
}

cv::Mat read_depth_bin_sean_as_img(char const* file_name, int width, int height)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if (fp == NULL)
	{
		printf("Error<read_depth_bin_sean_as_img>: cannot open file %s\n", file_name);
		return cv::Mat();
	}

	cv::Mat img = cv::Mat(height, width, CV_16UC1);

	int size = width * height * 2;
	fseek(fp, -size, SEEK_END);
	for (int row = 0; row < height; row++)
	{
		unsigned short* p = (unsigned short*)(img.ptr<unsigned short>(row));
		int readsize = (int)fread((void *)p, 1, (size_t)width * 2, fp);
		if (readsize != width * 2)
			return cv::Mat();
	}
	fclose(fp);

	return img;
}

void depth_remove_top_bit(cv::Mat& depthImg)
{
	int w = depthImg.cols;
	int h = depthImg.rows;
	for (int i = 0; i < h; i++)
	for (int j = 0; j < w; j++)
		depthImg.at<unsigned short>(i, j) &= 0x7fff;
}


void depth_extract_fg(cv::Mat& depthImg)
{
	int w = depthImg.cols;
	int h = depthImg.rows;
	for (int i = 0; i < h; i++)
	for (int j = 0; j < w; j++)
	{
		unsigned short &d = depthImg.at<unsigned short>(i, j);
		if (d > 0x8000)
			d &= 0x7fff;
		else
			d = 0;
	}
}

bool fuse_depth_color_volumetric(std::vector<cv::Mat> const& depthMats, std::vector<cv::Mat> const& imgs,
	std::vector<GCameraView*> const& cams_depth,
	std::vector<GCameraView*> const& cams_clr, 
	DDF &ddf,	
	double ddf_res,
	double ddf_mu,
	BoundingBox3D bbox_init)
{
	BoundingBox3D bbox = extract_bounding_box(depthMats, cams_depth);
	BoundingBox3D bbox_out = bbox;
	if (bbox_init.x_e>bbox_init.x_s &&
		bbox_init.y_e>bbox_init.y_s &&
		bbox_init.y_e>bbox_init.y_s)
		bbox_out = bbox_min(bbox_init, bbox);
	bbox_out.extend(5.0);

	ddf.Init(bbox_out, ddf_res, ddf_res, ddf_res, ddf_mu, false, false, true);

	for (int i = 0; i < depthMats.size(); i++)
	{
		cv::Mat depthMat_f = depthmap_bilateral_filter(depthMats[i], 3.0, 3.0);
		ddf.add_a_frame(depthMat_f, cams_depth[i], imgs.size()>i?imgs[i]:cv::Mat(), cams_clr.size()>i?cams_clr[i]:NULL);
		depthMat_f.release();
	}

	return true;
}

bool fuse_depth_color_occupancy(std::vector<cv::Mat> const& depthMats, std::vector<cv::Mat> const& imgs,
	std::vector<GCameraView*> const& cams_depth,
	std::vector<GCameraView*> const& cams_clr,
	TSDF &sdf)
{
	BoundingBox3D bbox = extract_bounding_box(depthMats, cams_depth);
	bbox.extend(5.0);

	sdf.Init(bbox, 0.2, 0.2, 0.2, 3.0, false, false, true);

	for (int i = 0; i < depthMats.size(); i++)
	{
		cv::Mat depthMat_f = depthmap_bilateral_filter(depthMats[i], 3.0, 3.0);
		sdf.add_a_frame_occupancy(depthMat_f, cams_depth[i], imgs[i], cams_clr[i]);
		depthMat_f.release();
	}

	return true;
}

bool read_patch_collider_matches(char const* filename, std::vector<S2DPointMatchSet> &matches_all, double scale)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "r");
	if (!fp)
	{
		LOGGER()->error("read_patch_collider_matches", "Cannot open the file <%s>.", filename);
		return false;
	}

	matches_all.clear();
	while (true)
	{
		S2DPointMatchSet match_set;
		int count;
		fscanf(fp, "Src Frame %d Target Frame %d Tot Matches %d\n", &(match_set.frmIdx_1), &(match_set.frmIdx_2), &count);
		if (feof(fp))
			break;
		if (count > 0)
		{
			match_set.matches.set_size(count, 4);
			for (int i = 0; i < count; i++)
			{
				int x1, x2, y1, y2;
				fscanf(fp, "%d %d %d %d\n", &x1, &y1, &x2, &y2);
				match_set.matches[i][0] = x1 * scale;
				match_set.matches[i][1] = y1 * scale;
				match_set.matches[i][2] = x2 * scale;
				match_set.matches[i][3] = y2 * scale;
			}
		}
		matches_all.push_back(match_set);
	}

	fclose(fp);
	return true;
}