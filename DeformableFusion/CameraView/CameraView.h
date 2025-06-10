// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef _CAMERA_VIEW_H_
#define _CAMERA_VIEW_H_

#include <stdio.h>
#include <string.h>
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include <json/json.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

#ifndef ValidateKey
#define ValidateKey(keyStr, value) ((value).isMember((keyStr)))
#endif

bool ValidateParameterList(const Json::Value & json, const char* keys[], size_t nkeys);

double MillimetersToCentimeters(double v);
double MillimetersToFusionMetricSystem(double v);

double MetersToCentimeters(double v);
double MetersToMillimeters(double v);
double MetersTo3DTMMetricSystem(double v);
double Convert3DTMMetricSystemToFusion(double v);

bool Validate3DTMFormatForCamera(const Json::Value & camera);

struct Rect3D
{
	double p1[3];
	double p2[3];
	double p3[3];
	double p4[3];
};

class GCameraView
{
public:
	GCameraView(int imHeight=-1, int imWidth=-1);
	GCameraView(GCameraView const& other);
	~GCameraView();

public:
	GCameraView& operator=(GCameraView const& rhs);

public:
	/* the following several (four, actually) functions will automatically fill K, R, T, Radial(all 0 in 2nd func), projMat and also their corresponding 
	 * cv::Mat forms. They also compute OpenGL matrices
	 */
	bool LoadCalibrationFrom3DTMFormat(const char* calib_filename, unsigned int camera_idx, const char* camera_type, double scale = -1.0);
	bool SaveCalibrationWithSize(const char* filename);
	// Radial should be 5x1 vector
	void SetCalibrationMatrix(double const*K, double const*R, double const*T, double const*Radial=NULL);
	void SetCalibrationMatrix(cv::Mat const &intrinsics_mat, cv::Mat const &rot_mat, cv::Mat const &trans_mat, cv::Mat const& distort_mat=cv::Mat());
	//Radial should be 5-d vector
	void SetIntrinsics( double const*K, double const* Radial=NULL);
	void SetExtrinsics( double const*R, double const*T);
	// T *= scale
	void ScaleCamera(double scale);
	void ScaleIntrinsics(double scale);
	void SetImageSize(int width, int height);

public:
	std::string name;
	int m_Width;
	int m_Height;
	cv::Mat intrinsics;
	cv::Mat rotation;
	cv::Mat translation;
	cv::Mat distort_coef;
	cv::Mat pers_proj_mat;
	double K[3][3];
	double R[3][3]; //R should be orthonormal
	double T[3];
	double Radial[5];
	double projMat[3][4]; //perspective projection matrix

private:
	void allocateCameraMatrix();
	void releaseCameraMatrix();
	void copyArray2CvMat();

public:
	void ComputeOpenGLMat(double near, double far);

public:
	// after model_view and projection operation, the viewing volume is [-1, 1]^3
	// after view port operation, the viewing volume is [0 w]x[0 h]x[-1 1]
	// after textureMat operation, the viewing volume is [0 1]x[0 1]x[-inf inf]
	double projection[16]; //stored in column-major order
	double model_view[16]; //stored in column-major order
	double view_port[16];  //stored in column-major order (useless, because OpenGL use glViewPort()...)
	double textureMat[16]; //stored in column-major order

public:
	/* depthMap_undistort -- in
	 * labelImg -- out. The pixels belongs to floor is set to 255 in labelImg
	 * The floor is define as z = offset */
	void DetectFloor(cv::Mat const& depthMap_undistort, cv::Mat& labelImg, double thres = 1, double offset = 0) const;

// interface for undistort image
public:
	/* it takes 26ms to undistort 1600x1200 image 
	 * it takes 4.0ms to undistort 480x640 image
	 * it almost takes no time to create an image, but it takes about 3.5 ms to clone an 1600x1200 image
	 */
	void UndistortImage(cv::InputArray src, cv::OutputArray dst, int flags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);

	/* it takes 4.5ms to undistort an 480x640 image
	 * src and dst must be CV_64F type
	 */
	int UndistortDepthMap(cv::InputArray src, cv::OutputArray dst);
	
	void initUndistortMap();
public:
	cv::Mat mapX; //float tpye
	cv::Mat mapY; //float type

public:
	double distance_to_cam_center(double const* pt) const;
	// return angle in radius, it should lie in (0, pi)
	double angle_btw_viewray_normal( double const*pt, double const* normal) const;
	double angle_btw_viewray_normal(float const*pt, float const* normal) const;

private:
	double cam_center[3];

public:
	double* getCameraCenter() const;
	void getCameraCenter(double center[3]) const;
	double* getCameraOrientation() const;
	Rect3D* getImagePlane(double d) const;

	/* all the following function return a 3D vector*/
	double* cameraSpace2worldSpace(double p[3]) const;
	double* cameraSpace2worldSpace(double x, double y, double z) const;
	inline void cameraSpace2worldSpace(double p[3], double c[3]) const;
	float* cameraSpace2worldSpace(float p[3]) const;
	float* cameraSpace2worldSpace(float x, float y, float z) const;
	void cameraSpace2worldSpace(float p[3], float c[3]) const;

	double* worldSpace2cameraSpace(double p[3]) const;
	double* worldSpace2cameraSpace(double x, double y, double z) const;
	void worldSpace2cameraSpace(double p[3], double c[3]) const;
	float* worldSpace2cameraSpace(float p[3]) const;
	float* worldSpace2cameraSpace(float x, float y, float z) const;
	void worldSpace2cameraSpace(float p[3], float c[3]) const;

	double* worldSpace2ImageCoord(double p[3]) const;
	double* worldSpace2ImageCoord(double x, double y, double z) const;
	void worldSpace2ImageCoord(double p[3], double v[2]) const;
	float* worldSpace2ImageCoord(float p[3]) const;
	float* worldSpace2ImageCoord(float x, float y, float z) const;
	void worldSpace2ImageCoord(float p[3], float v[2]) const;

	//the third component of v is z in the camera space
	void worldSpace2ImageCoordV3(double const* p, double v[3]) const;
	void worldSpace2ImageCoordV3(float const* p, float v[3]) const;

	void World2ImgWithDistortionV3(double const*p, double v[3]) const;
	void World2ImgWithDistortion(double const*p, double v[2]) const;
	void World2ImgWithDistortionV3(float const*p, float v[3]) const;
	void World2ImgWithDistortion(float const*p, float v[2]) const;

	bool ImgCoordAndDepth2WorldCoord(double x_in[3], double x_w[3]) const;
	double *ImgCoordAndDepth2WorldCoord(double x_in[3]) const;
	bool ImgCoordAndDepth2WorldCoord(float x_in[3], float x_w[3]) const;
	float *ImgCoordAndDepth2WorldCoord(float x_in[3]) const;

/* input   projMat: a 3x4 matrix with double type. projmat is intact after operation
 * output  K (3x3), R(3x3), T(3x1): the memory must be pre-allocated
 */

private:
	void normalizeVec(double* v, int len);
};

/* s in [0 1];
 * cam3 = cam1 * (1-s) + cam2 *s
 */
GCameraView* GenerateIntermediateCameraView(GCameraView* cam1, GCameraView *cam2, double s);

#endif