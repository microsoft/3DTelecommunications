#ifndef __VOXEL_DATA_IO_H__
#define __VOXEL_DATA_IO_H__
#include "CameraView.h"
#include <vector>
#include "DDF.h"

bool read_calibration_bundle_qin(char const* file_name, std::vector<GCameraView*> &cams);

//get depth calibration in the world space.
// cams_pods: the calibration data for all pods
// color_cams: the calibration in the world space for color cameras from all pods
bool get_global_depth_calibration(std::vector<std::vector<GCameraView*>> const& cams_pods, std::vector<GCameraView*> const& color_cams,
	std::vector<GCameraView*> &depth_cams);


cv::Mat read_color_bin(char const* file_name, int width, int height, int& readColorSize);
//in cm
cv::Mat read_depth_bin_qin(char const* file_name);
//in mm
cv::Mat read_depth_bin_qin_as_img(char const* file_name);

cv::Mat read_depth_bin_sean(char const* file_name, int width, int height);
cv::Mat read_depth_bin_sean_as_img(char const* file_name, int width, int height);

void depth_remove_top_bit(cv::Mat& depthImg);
void depth_extract_fg(cv::Mat& depthImg);

//ddf doesn't need to be initialized
bool fuse_depth_color_volumetric(std::vector<cv::Mat> const& depthMats, std::vector<cv::Mat> const& imgs,
								 std::vector<GCameraView*> const& cams_depth,
								 std::vector<GCameraView*> const& cams_clr,
								 DDF &ddf,								 
								 double ddf_res = 0.5,
								 double ddf_mu = 2.5,
								 BoundingBox3D bbox = BoundingBox3D(0, 0, 0, 0, 0, 0));

bool fuse_depth_color_occupancy(std::vector<cv::Mat> const& depthMats, std::vector<cv::Mat> const& imgs,
	std::vector<GCameraView*> const& cams_depth,
	std::vector<GCameraView*> const& cams_clr,
	TSDF &sdf);

struct S2DPointMatchSet
{
	int frmIdx_1;
	int frmIdx_2;
	vnl_matrix<double> matches; //each row is a 4d vector. [x1, y1, x2, y2]
};

bool read_patch_collider_matches(char const* filename, std::vector<S2DPointMatchSet> &matches, double scale);


#endif