// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.

#include "CameraView.h"
#include "UtilMatrix.h"
#include "VecOperation.h"
#include <math.h>
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\calib3d.hpp"

bool ValidateParameterList(const Json::Value & json, const char* keys[], size_t nkeys) {

	for (size_t i = 0; i < nkeys; i++) {
		if (!ValidateKey(keys[i], json))
			return false;
	}

	return true;
}

double MillimetersToCentimeters(double v) {
	return v / 10.0f;
}

double MillimetersToFusionMetricSystem(double v) {
	return MillimetersToCentimeters(v);
}


double MetersToCentimeters(double v) {
	return v * 100.0f;
}

double MetersToMillimeters(double v) {
	return v * 1000.0f;
}

double MetersTo3DTMMetricSystem(double v) {
	return MetersToMillimeters(v);
}

double Convert3DTMMetricSystemToFusion(double v) {
	return MillimetersToFusionMetricSystem(v);
}


bool Validate3DTMFormatForCamera(const Json::Value & camera) {
	const char* intrinsics_req[] = { "resolution", "codx", "cody", "fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2" };
	const char* extrinsics_req[] = { "rotation", "translation" };

	bool valid = true;

	valid = valid && ValidateParameterList(camera["intrinsics"], intrinsics_req, 12);
	valid = valid && ValidateParameterList(camera["extrinsics"], extrinsics_req, 2);

	if (valid) {
		valid = valid && camera["extrinsics"]["rotation"].size() == 3 && camera["extrinsics"]["rotation"][0].size() == 3 && camera["extrinsics"]["rotation"][1].size() == 3 && camera["extrinsics"]["rotation"][2].size() == 3;
		valid = valid && camera["extrinsics"]["translation"].size() == 3;
	}

	return valid;
}

void PrintCameraView(GCameraView& cam, const std::string& camName) {
	std::cout << "Printing camera " << camName << std::endl;
	std::cout << "name " << cam.name << std::endl;
	std::cout << "m_Width " << cam.m_Width << std::endl;
	std::cout << "m_Height " << cam.m_Height << std::endl;
	std::cout << "intrinsics " << cam.intrinsics << std::endl;
	std::cout << "rotation " << cam.rotation << std::endl;
	std::cout << "translation " << cam.translation << std::endl;
	std::cout << "distort_coef " << cam.distort_coef << std::endl;
	std::cout << "pers_proj_mat " << cam.pers_proj_mat << std::endl;
	std::cout << "K " << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << " " << cam.K[i][j];
		}
		std::cout << std::endl;
	}

	std::cout << "R " << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << " " << cam.R[i][j];
		}
		std::cout << std::endl;
	}

	std::cout << "T " << std::endl;

	for (int j = 0; j < 3; j++) {
		std::cout << " " << cam.T[j];
	}
	std::cout << std::endl;


	std::cout << "Radial " << std::endl;

	for (int j = 0; j < 5; j++) {
		std::cout << " " << cam.Radial[j];
	}
	std::cout << std::endl;

	std::cout << "projMat " << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			std::cout << " " << cam.projMat[i][j];
		}
		std::cout << std::endl;
	}
}

GCameraView::GCameraView(int imHeight, int imWidth)
{
	this->m_Width = imWidth; 
	this->m_Height = imHeight;

	this->mapX = cv::Mat();
	this->mapY = cv::Mat();

	this->allocateCameraMatrix();

	if (imHeight > 0 && imWidth > 0)
	{
		//set initial value
		cv::Mat intrinsics = cv::Mat(3, 3, CV_64F);
		intrinsics.at<double>(0, 0) = 500;
		intrinsics.at<double>(1, 1) = 500;
		intrinsics.at<double>(2, 2) = 1.0;
		intrinsics.at<double>(0, 2) = this->m_Width / 2.0;
		intrinsics.at<double>(1, 2) = this->m_Height / 2.0;
		cv::Mat rotation = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
		
		rotation.at<double>(0, 0) = 1.0;
		rotation.at<double>(1, 1) = 1.0;
		rotation.at<double>(2, 2) = 1.0;
		cv::Mat translation = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
		
		cv::Mat distortion = cv::Mat(5, 1, CV_64F, cv::Scalar(0));

		this->SetCalibrationMatrix(intrinsics, rotation, translation, distortion);
		intrinsics.release();
		rotation.release();
		translation.release();
		distortion.release();

		this->getCameraCenter(this->cam_center);
	}
}

GCameraView::GCameraView(GCameraView const& other)
{
	this->m_Width = other.m_Width;
	this->m_Height = other.m_Height;

	if (!other.mapX.empty())
		this->mapX = other.mapX.clone();
	else
		this->mapX = NULL;
	if (!other.mapY.empty())
		this->mapY = other.mapY.clone();
	else
		this->mapY = NULL;

	this->allocateCameraMatrix();
	this->SetCalibrationMatrix(other.intrinsics, other.rotation, other.translation, other.distort_coef);
	this->getCameraCenter(this->cam_center);
}

GCameraView& GCameraView::operator=(GCameraView const& rhs)
{
	this->m_Width = rhs.m_Width;
	this->m_Height = rhs.m_Height;

	if( !this->mapX.empty() )
	{
		this->mapX.release();
		this->mapX = cv::Mat();
	}
	if( !this->mapY.empty() )
	{
		this->mapY.release();
		this->mapY = cv::Mat();
	}

	if( !rhs.mapX.empty())
		this->mapX = rhs.mapX.clone();		
	if( !rhs.mapY.empty())
		this->mapY = rhs.mapY.clone();

	this->SetCalibrationMatrix(rhs.intrinsics, rhs.rotation, rhs.translation, rhs.distort_coef);
	this->getCameraCenter(this->cam_center);
	return (*this);
}

GCameraView::~GCameraView()
{
	if(!mapX.empty())
		mapX.release();
	if(!mapY.empty())
		mapY.release();

	this->releaseCameraMatrix();
}

void GCameraView::allocateCameraMatrix()
{
	this->intrinsics = cv::Mat(3, 3, CV_64F, cv::Scalar(0));
	this->distort_coef = cv::Mat(5, 1, CV_64F, cv::Scalar(0));
	this->rotation = cv::Mat(3, 3, CV_64F);
	cv::setIdentity(this->rotation);
	this->translation = cv::Mat(3, 1, CV_64F, cv::Scalar(0));
	this->pers_proj_mat = cv::Mat(3, 4, CV_64F, cv::Scalar(0));
}

void GCameraView::releaseCameraMatrix()
{
	if (!intrinsics.empty()) intrinsics.release();;
	if(!distort_coef.empty()) distort_coef.release();
	if(!rotation.empty()) rotation.release();
	if(!translation.empty()) translation.release();
	if(!pers_proj_mat.empty()) pers_proj_mat.release();
}

void GCameraView::copyArray2CvMat()
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			this->intrinsics.at<double>(i, j) = this->K[i][j];
			this->rotation.at<double>(i, j) = this->R[i][j];
		}
		this->translation.at<double>(i, 0) = this->T[i];
	}

	for(int i=0; i<5; i++)
	{
		this->distort_coef.at<double>(i, 0) = this->Radial[i];
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
		{
			pers_proj_mat.at<double>(i, j) = this->projMat[i][j];
		}
	}
}

bool GCameraView::LoadCalibrationFrom3DTMFormat(const char* calib_filename, unsigned int camera_idx, const char* camera_type, double scale)
{

	std::ifstream calibIFS(calib_filename);
	if (!calibIFS.is_open())
	{
		std::cerr << "Error<GCameraView::LoadCalibrationFrom3DTMFormat>:Can't open 3DTM Json calibration file" << calib_filename << std::endl;
		return false;
	}

	Json::Value root;

	// Trying to parse Json format
	try {
		calibIFS >> root;
	}
	catch (const std::exception& e) {
		std::cerr << "Error<GCameraView::LoadCalibrationFrom3DTMFormat>: Invalid 3DTM Format in " << calib_filename << std::endl;
		return false;
	}

	calibIFS.close();

	// Validating basic information about the file
	if (!ValidateKey("inputCameras", root) || camera_idx > root["inputCameras"].size() || !ValidateKey(camera_type, root["inputCameras"][camera_idx])) {
		std::cerr << "Error<GCameraView::LoadCalibrationFrom3DTMFormat>: Invalid 3DTM Format for camera " << camera_idx << " in file " << calib_filename << std::endl;
		return false;
	}

	Json::Value& camera = root["inputCameras"][camera_idx][camera_type];

	// Validating 3DTM format for a given camera
	if (!Validate3DTMFormatForCamera(camera)) {
		std::cerr << "Error<GCameraView::LoadCalibrationFrom3DTMFormat>: Invalid 3DTM Format for camera " << camera_idx << " in file " << calib_filename << std::endl;
		return false;
	}

	// We can now safely read the file

	double temp;

	double width, height;
	width = camera["intrinsics"]["resolution"]["width"].asDouble();
	height = camera["intrinsics"]["resolution"]["height"].asDouble();

	double scale_x;
	double scale_y;
	if (scale <= 0)
	{
		if (this->m_Height > 0 && this->m_Width > 0)
		{
			scale_x = this->m_Width / width;
			scale_y = this->m_Height / height;
			printf("Warning: the size in the calibration file is scaled <%f, %f> to match the image size!\n", scale_x, scale_y);
		}
		else
		{
			scale_x = 1.0;
			scale_y = 1.0;
			this->m_Width = width;
			this->m_Height = height;
		}
	}
	else
	{
		scale_x = scale;
		scale_y = scale;
	}

	// Initializing K as identity matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			K[i][j] = (i == j) ? 1.0 : 0.0;
		}
	}

	// Copying focal lengths and principal point into K	
	K[0][0] = scale_x * camera["intrinsics"]["fx"].asDouble();
	K[1][1] = scale_y * camera["intrinsics"]["fy"].asDouble();
	K[0][2] = scale_x * camera["intrinsics"]["cx"].asDouble();
	K[1][2] = scale_y * camera["intrinsics"]["cy"].asDouble();


	double rTotalCheck = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			temp = camera["extrinsics"]["rotation"][i][j].asDouble();
			R[i][j] = temp;
			rTotalCheck += temp;
		}
	}

	//extra calib file validity check on rotation. All zero rotation matrix is non-valid
	if (rTotalCheck == 0)
	{
		fprintf(stderr, "Error<GCameraView::LoadCalibrationFrom3DTMFormat>: camera rotation matrix can't be all 0's");
		return false;
	}

	// Copying translation and converting millimeters to centimeters
	for (int i = 0; i < 3; i++)
	{
		T[i] = MillimetersToFusionMetricSystem(camera["extrinsics"]["translation"][i].asDouble());
	}

	const char* k_name[] = { "k1", "k2", "p1", "p2", "k3" };
	for (int i = 0; i < 5; i++)
	{
		Radial[i] = camera["intrinsics"][k_name[i]].asDouble();
	}

	//projMat = K*[R|T]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			this->projMat[i][j] = K[i][0] * R[0][j] + K[i][1] * R[1][j] + K[i][2] * R[2][j];
		}
		this->projMat[i][3] = K[i][0] * T[0] + K[i][1] * T[1] + K[i][2] * T[2];
	}


	this->copyArray2CvMat();

	this->initUndistortMap();

	this->ComputeOpenGLMat(10, 800);

	this->getCameraCenter(this->cam_center);

	return true;
}


void GCameraView::SetImageSize(int width, int height)
{
	if (this->m_Height <= 0 || this->m_Width <= 0)
	{
		this->m_Height = height;
		this->m_Width = width;

		this->initUndistortMap();
		this->ComputeOpenGLMat(10, 800);
	}
	else
	{
		double scale_x = (float)(width) / (float)this->m_Width;
		double scale_y = (float)(height) / (float) this->m_Height;

		this->m_Height = height;
		this->m_Width = width;
		
		K[0][0] *= scale_x;
		K[0][1] *= scale_x;
		K[0][2] *= scale_x;
		K[1][1] *= scale_y;
		K[1][2] *= scale_y;

		SetIntrinsics(&(K[0][0]), NULL);
	}
}


bool GCameraView::SaveCalibrationWithSize(const char *filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if(!fp)
	{
		printf("Cannot open the matrix file.\n");
		return false;
	}

	fprintf_s(fp, "%d ", this->m_Width);
	fprintf_s(fp, "%d", this->m_Height);
	fprintf_s(fp, "\n");

	if( !this->intrinsics.empty() &&
		this->intrinsics.rows == 3 &&
		this->intrinsics.cols == 3 )
	{
		for(int i=0; i<3; i++)
		{
			for(int j=0; j<3; j++)
			{
				fprintf_s(fp, "%15.15f ", intrinsics.at<double>(i, j));
			}
			fprintf_s(fp, "\n");
		}
	}
	else
	{
		printf("Error: Intrins Matrix is empty or not a 3x3 matrix with double type\n");
		fclose(fp);
		return false;
	}

	if( !this->rotation.empty() &&
		this->rotation.rows == 3 &&
		this->rotation.cols == 3 )
	{
		for(int i=0; i<3; i++)
		{
			for(int j=0; j<3; j++)
			{
				fprintf_s(fp, "%15.15f ", this->rotation.at<double>(i, j));
			}
			fprintf_s(fp, "\n");
		}
	}
	else
	{
		printf("Error: Rotation Matrix is empty or not a 3x3 matrix with double type\n");
		fclose(fp);
		return false;
	}

	if( !this->translation.empty() &&
		(this->translation.rows == 3 || this->translation.cols == 3)
		)
	{
		if( this->translation.rows == 3)
		{
			for(int i=0; i<3; i++)
				fprintf_s(fp, "%15.15f\n", this->translation.at<double>(i, 0));
		}
		else
		{
			for(int i=0; i<3; i++)
				fprintf_s(fp, "%15.15f\n", this->translation.at<double>(0, i));
		}
	}
	else
	{
		printf("Error: Rotation Matrix is empty or not a 3x1 (or 1x3) matrix with double type\n");
		fclose(fp);
		return false;
	}

	if( !this->distort_coef.empty() &&
		( this->distort_coef.rows == 5 || this->distort_coef.rows == 4 ||
		  this->distort_coef.cols == 5 || this->distort_coef.cols == 4 )
	   )
	{
		if( this->distort_coef.rows == 5 || this->distort_coef.rows == 4 )
		{
			for(int i=0; i<this->distort_coef.rows; i++)
				fprintf_s(fp, "%15.15f\n", this->distort_coef.at<double>(i, 0));
		}
		else
		{
			for(int i=0; i<this->distort_coef.rows; i++)
				fprintf_s(fp, "%15.15f\n", this->distort_coef.at<double>(0, i));
		}
	}
	else
	{
		printf("Error: Translate Matrix is empty or not a 5x1 (or 1x5, 4x1, 1x4) matrix with double type\n");
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}

void GCameraView::SetCalibrationMatrix(cv::Mat const &intrinsics_mat, cv::Mat const &rot_mat, cv::Mat const &trans_mat, cv::Mat const &distort_mat)
{
	double const*p_K = NULL;
	double const*p_R = NULL;
	double const*p_T = NULL;
	double const*p_dist = NULL;
	if( !intrinsics_mat.empty() )
		p_K = intrinsics_mat.ptr<double>(0);
	if( !rot_mat.empty() )
		p_R = rot_mat.ptr<double>(0);
	if( !trans_mat.empty() )
		p_T = trans_mat.ptr<double>(0);
	if( !distort_mat.empty() )
		p_dist = distort_mat.ptr<double>(0);

	SetCalibrationMatrix(p_K, p_R, p_T, p_dist);
}

void GCameraView::ScaleCamera(double scale)
{
	translation.at<double>(0, 0) *= scale;
	translation.at<double>(1, 0) *= scale;
	translation.at<double>(2, 0) *= scale;
	SetCalibrationMatrix(this->intrinsics, this->rotation, this->translation, this->distort_coef);
}

void GCameraView::SetCalibrationMatrix(double const*K, double const*R, double const*T, double const*Radial)
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			if( K!=NULL)
				this->K[i][j] = K[i*3+j];
			
			if( R!=NULL)
				this->R[i][j] = R[i*3+j];
		}
		if(T!=NULL)
			this->T[i] = T[i];
	}

	if(Radial != NULL)
	{
		for(int i=0; i<5; i++)
			this->Radial[i] = Radial[i];
	}

	//projMat = K*[R|T]
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			this->projMat[i][j] = this->K[i][0]*this->R[0][j] + this->K[i][1]*this->R[1][j] + this->K[i][2]*this->R[2][j];
		}
		this->projMat[i][3] = this->K[i][0]*this->T[0] + this->K[i][1]*this->T[1] + this->K[i][2]*this->T[2];
	}

	this->copyArray2CvMat();

	this->initUndistortMap();

	this->ComputeOpenGLMat(10, 800);

	this->getCameraCenter(this->cam_center);
}

void GCameraView::SetExtrinsics( double const*R, double const*T)
{
	SetCalibrationMatrix(NULL, R, T, NULL);
}

void GCameraView::SetIntrinsics( double const*K, double const* Radial)
{
	SetCalibrationMatrix(K, NULL, NULL, Radial);
}

void GCameraView::ScaleIntrinsics(double scale)
{
	K[0][0] *= scale;
	K[0][1] *= scale;
	K[0][2] *= scale;
	K[1][1] *= scale;
	K[1][2] *= scale;
	SetCalibrationMatrix(&K[0][0], NULL, NULL, Radial);
}

void GCameraView::ComputeOpenGLMat(double n, double f)
{
	if (this->m_Width <= 0 || this->m_Height <= 0)
		return;

	cv::Mat viewport_proj_mat = cv::Mat(4, 4, CV_64F);
	cv::Mat viewport_mat = cv::Mat(4, 4, CV_64F, cv::Scalar(0));
	cv::Mat viewport_mat_inv = cv::Mat(4, 4, CV_64F);
	cv::Mat projection_mat = cv::Mat(4, 4, CV_64F);

	viewport_proj_mat.at<double>(0, 0) = this->K[0][0];
	viewport_proj_mat.at<double>(0, 1) = this->K[0][1];
	viewport_proj_mat.at<double>(0, 2) = this->K[0][2];
	viewport_proj_mat.at<double>(0, 3) = 0.0;
	viewport_proj_mat.at<double>(1, 0) = this->K[1][0];
	viewport_proj_mat.at<double>(1, 1) = this->K[1][1];
	viewport_proj_mat.at<double>(1, 2) = this->K[1][2];
	viewport_proj_mat.at<double>(1, 3) = 0.0;
	viewport_proj_mat.at<double>(2, 0) = 0.0;
	viewport_proj_mat.at<double>(2, 1) = 0.0;
	viewport_proj_mat.at<double>(2, 2) = (f+n)/(f-n);
	viewport_proj_mat.at<double>(2, 3) = -2*f*n/(f-n);
	viewport_proj_mat.at<double>(3, 0) = 0.0;
	viewport_proj_mat.at<double>(3, 1) = 0.0;
	viewport_proj_mat.at<double>(3, 2) = 1.0;
	viewport_proj_mat.at<double>(3, 3) = 0.0;

	viewport_mat.at<double>(0, 0) =  this->m_Width/2.0;
	viewport_mat.at<double>(0, 3) = (this->m_Width-1)/2.0;
	viewport_mat.at<double>(1, 1) =  this->m_Height/2.0;
	viewport_mat.at<double>(1, 3) = (this->m_Height-1)/2.0;
	viewport_mat.at<double>(2, 2) = 1.0;
	viewport_mat.at<double>(3, 3) = 1.0;

	cv::invert(viewport_mat, viewport_mat_inv);

	cv::gemm(viewport_mat_inv, viewport_proj_mat, 1, 0, 0, projection_mat);
	
	//fill the projection array 
	for(int i=0; i<4; i++){ //row
		for(int j=0; j<4; j++){ //column
			this->projection[j*4+i] = projection_mat.at<double>(i, j);;
		}
	}

	//fill the view_port array
	for(int i=0; i<4; i++){ //row
		for(int j=0; j<4; j++){ //column
			this->view_port[j*4+i] = viewport_mat.at<double>(i, j);;
		}
	}

	//construct ModelView Matrix
	memset(this->model_view, 0, sizeof(double)*16);
	for(int i=0; i<3; i++) {//row
		for(int j=0; j<3; j++) {//column
			this->model_view[j*4+i] = this->R[i][j];
		}
	}
	for(int i=0; i<3; i++)
		this->model_view[3*4+i] = this->T[i];
	this->model_view[15] = 1.0; 

	projection_mat.release();
	viewport_mat.release();
	viewport_mat_inv.release();
	viewport_proj_mat.release();

	// fill the textureMat array
	for(int i=0; i<4; i++)
	{
		this->textureMat[i*4 + 0] = this->projMat[0][i]/this->m_Width;//the first row
		this->textureMat[i*4 + 1] = this->projMat[1][i]/this->m_Height;//the 2nd row
		this->textureMat[i*4 + 3] = this->projMat[3][i];		
	}
	this->textureMat[0*4 + 2] = 0.0;
	this->textureMat[1*4 + 2] = 0.0;
	this->textureMat[2*4 + 2] = 1.0;
	this->textureMat[3*4 + 2] = 0.0;

}

void GCameraView::DetectFloor(cv::Mat const& depthMap, cv::Mat& labelImg, double thres, double offset) const
{
	if(labelImg.empty())
	{
		printf("Error: the input label iamge is NULL!\n");
		return;
	}

	if( labelImg.cols != depthMap.cols ||
		labelImg.rows != depthMap.rows )
	{
		printf("Error: the size of the label does not match the size of the depth map!\n");
		return;
	}

	if( labelImg.channels() != 1 ||
		labelImg.depth() != CHAR_BIT)
	{
		printf("Error: the label image should be 8 bits per pixel and channel 1!\n");
		return;
	}

	int h = depthMap.rows;
	int w = depthMap.cols;	
	double scale_factor_x = w/double(this->m_Width);
	double scale_factor_y = h/double(this->m_Height);
	double fx = this->K[0][0];
	double fy = this->K[1][1];
	double cx = this->K[0][2];
	double cy = this->K[1][2];

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double z = depthMap.at<double>(i, j);; 

			if( z <= 0.0 )
				continue;

			double x = (j/scale_factor_x - cx)*z/fx;
			double y = (i/scale_factor_y - cy)*z/fy;
			
			double z_world = R[0][2]*(x-T[0]) + R[1][2]*(y-T[1]) + R[2][2]*(z-T[2]);

			labelImg.at<uchar>(i, j) = (z_world > offset -  thres*z*3.0/500.0)?255:0;
		}
	}
}

double GCameraView::distance_to_cam_center(double const* pt) const
{
	return sqrt( (pt[0]-this->cam_center[0])*(pt[0]-this->cam_center[0]) + 
				 (pt[1]-this->cam_center[1])*(pt[1]-this->cam_center[1]) +
				 (pt[2]-this->cam_center[2])*(pt[2]-this->cam_center[2]) );
}

double GCameraView::angle_btw_viewray_normal( double const*pt, double const* normal) const
{
	double view_ray[3];
	view_ray[0] = pt[0] - this->cam_center[0];
	view_ray[1] = pt[1] - this->cam_center[1];
	view_ray[2] = pt[2] - this->cam_center[2];
	return VecOperation<double>::AngleBtwVecs(view_ray, normal);
}

double GCameraView::angle_btw_viewray_normal( float const*pt, float const* normal) const
{
	float view_ray[3];
	view_ray[0] = pt[0] - this->cam_center[0];
	view_ray[1] = pt[1] - this->cam_center[1];
	view_ray[2] = pt[2] - this->cam_center[2];
	return VecOperation<float>::AngleBtwVecs(view_ray, normal);
}

double* GCameraView::getCameraCenter() const
{
	double *c = new double[3];
	getCameraCenter(c);
	return c;
}

void GCameraView::getCameraCenter(double c[3]) const
{
	for(int i=0; i<3; i++)
	{
		c[i] = -( this->R[0][i]*this->T[0] + this->R[1][i]*this->T[1] + this->R[2][i]*this->T[2]);
	}
}

double* GCameraView::getCameraOrientation() const
{
	double* c = this->cameraSpace2worldSpace(0.0, 0.0, 0.0);
	double* dz = this->cameraSpace2worldSpace(0.0, 0.0, 1.0);
	double* ori = new double[3];
	VecOperation<double>::VecSub(dz, c, ori, 3);
	VecOperation<double>::Unitalization(ori, 3);

	// Possible memory leak of c and dz?
	return ori;
}

Rect3D* GCameraView::getImagePlane(double d) const
{
	double* c = this->cameraSpace2worldSpace(0.0, 0.0, 0.0);
	double* dx = this->cameraSpace2worldSpace(1.0, 0.0, 0.0);	
	double* dy = this->cameraSpace2worldSpace(0.0, 1.0, 0.0);
	double* dz = this->cameraSpace2worldSpace(0.0, 0.0, 1.0);

	for(int i=0; i<3; i++)
	{
		dx[i] -= c[i];
		dy[i] -= c[i];
		dz[i] -= c[i];
	}

	VecOperation<double>::Unitalization(dx, 3);
	VecOperation<double>::Unitalization(dy, 3);
	VecOperation<double>::Unitalization(dz, 3);

	double dp1[3];
	double dp2[3];
	double dp3[3];
	double dp4[3];

	double f = (this->K[0][0]+this->K[1][1])/2.0;
	for(int i=0; i<3; i++)
	{
		dp1[i] = (dz[i]*f + dx[i]*this->m_Width/2.0 + dy[i]*this->m_Height/2.0)/100.0;
		dp2[i] = (dz[i]*f - dx[i]*this->m_Width/2.0 + dy[i]*this->m_Height/2.0)/100.0;
		dp3[i] = (dz[i]*f - dx[i]*this->m_Width/2.0 - dy[i]*this->m_Height/2.0)/100.0;
		dp4[i] = (dz[i]*f + dx[i]*this->m_Width/2.0 - dy[i]*this->m_Height/2.0)/100.0;
	}
	VecOperation<double>::Unitalization(dp1, 3);
	VecOperation<double>::Unitalization(dp2, 3);
	VecOperation<double>::Unitalization(dp3, 3);
	VecOperation<double>::Unitalization(dp4, 3);

	double len = d * (sqrt(double(this->m_Height*this->m_Height + this->m_Width*this->m_Width))/(2.0*f));
	
	Rect3D *rect = new Rect3D();

	for(int i=0; i<3; i++)
	{
		rect->p1[i] = c[i] + len * dp1[i];
		rect->p2[i] = c[i] + len * dp2[i];
		rect->p3[i] = c[i] + len * dp3[i];
		rect->p4[i] = c[i] + len * dp4[i];
	}
	
	delete [] c;
	delete [] dx;
	delete [] dy;
	delete [] dz;
	return rect;
}


inline void GCameraView::cameraSpace2worldSpace(double p[3], double c[3]) const
{
	for(int i=0; i<3; i++)
	{
		c[i] =  this->R[0][i]*(p[0]-this->T[0]) + this->R[1][i]*(p[1]-this->T[1]) + this->R[2][i]*(p[2]-this->T[2]);
	}
}

void GCameraView::cameraSpace2worldSpace(float p[3], float c[3]) const
{
	for(int i=0; i<3; i++)
	{
		c[i] =  this->R[0][i]*(p[0]-this->T[0]) + this->R[1][i]*(p[1]-this->T[1]) + this->R[2][i]*(p[2]-this->T[2]);
	}
}

double* GCameraView::cameraSpace2worldSpace(double p[]) const
{
	double* c = new double[3];
	this->cameraSpace2worldSpace(p, c);
	return c;
}

float* GCameraView::cameraSpace2worldSpace(float p[]) const
{
	float* c = new float[3];
	this->cameraSpace2worldSpace(p, c);
	return c;
}

double* GCameraView::cameraSpace2worldSpace(double x, double y, double z) const
{
	double p[] = {x, y, z};
	return this->cameraSpace2worldSpace(p);
}

float* GCameraView::cameraSpace2worldSpace(float x, float y, float z) const
{
	float p[] = {x, y, z};
	return this->cameraSpace2worldSpace(p);
}

void GCameraView::worldSpace2cameraSpace(double p[3], double c[3]) const
{
	for(int i=0; i<3; i++)
	{
		c[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}
}

double* GCameraView::worldSpace2cameraSpace(double p[3]) const
{
	double *ret = new double[3];
	worldSpace2cameraSpace(p, ret);
	return ret;	
}

double* GCameraView::worldSpace2cameraSpace(double x, double y, double z) const
{
	double p[] = {x, y, z};
	return this->worldSpace2cameraSpace(p);
}

void GCameraView::worldSpace2cameraSpace(float p[3], float c[3]) const
{
	for(int i=0; i<3; i++)
	{
		c[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}
}

float* GCameraView::worldSpace2cameraSpace(float p[3]) const
{
	float *ret = new float[3];
	worldSpace2cameraSpace(p, ret);
	return ret;	
}

float* GCameraView::worldSpace2cameraSpace(float x, float y, float z) const
{
	float p[] = {x, y, z};
	return this->worldSpace2cameraSpace(p);
}

void GCameraView::worldSpace2ImageCoordV3(double const* p, double v[3]) const
{
	for(int i=0; i<3; i++)
	{
		v[i] = this->projMat[i][0]*p[0] + this->projMat[i][1]*p[1] + this->projMat[i][2]*p[2] + this->projMat[i][3];
	}
	v[0] /= v[2];
	v[1] /= v[2];
}

void GCameraView::worldSpace2ImageCoordV3(float const* p, float v[3]) const
{
	for(int i=0; i<3; i++)
	{
		v[i] = this->projMat[i][0]*p[0] + this->projMat[i][1]*p[1] + this->projMat[i][2]*p[2] + this->projMat[i][3];
	}
	v[0] /= v[2];
	v[1] /= v[2];
}

void GCameraView::worldSpace2ImageCoord(double p[3], double v[2]) const
{
	double c[3];
	for(int i=0; i<3; i++)
	{
		c[i] = this->projMat[i][0]*p[0] + this->projMat[i][1]*p[1] + this->projMat[i][2]*p[2] + this->projMat[i][3];
	}

	v[0] = c[0]/c[2];
	v[1] = c[1]/c[2];
}

double* GCameraView::worldSpace2ImageCoord(double p[3]) const
{
	double *v = new double[2];
	worldSpace2ImageCoord(p, v);
	return v;
}

double* GCameraView::worldSpace2ImageCoord(double x, double y, double z) const
{
	double p[] = {x, y, z};
	return this->worldSpace2ImageCoord(p);
}

void GCameraView::worldSpace2ImageCoord(float p[3], float v[2]) const
{
	float c[3];
	for(int i=0; i<3; i++)
	{
		c[i] = this->projMat[i][0]*p[0] + this->projMat[i][1]*p[1] + this->projMat[i][2]*p[2] + this->projMat[i][3];
	}

	v[0] = c[0]/c[2];
	v[1] = c[1]/c[2];
}

float* GCameraView::worldSpace2ImageCoord(float p[3]) const
{
	float *v = new float[2];
	worldSpace2ImageCoord(p, v);
	return v;
}

float* GCameraView::worldSpace2ImageCoord(float x, float y, float z) const
{
	float p[] = {x, y, z};
	return this->worldSpace2ImageCoord(p);
}

void GCameraView::World2ImgWithDistortionV3(double const*p, double u[3]) const
{
	double X[3];
	for(int i=0; i<3; i++)
	{
		X[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}

	double x = X[0]/X[2];
	double y = X[1]/X[2];
	double rr = x*x + y*y;
	double c = 1.0 + this->Radial[0]*rr + this->Radial[1]*rr*rr + this->Radial[4]*rr*rr*rr;
	
	u[0] = c*x + 2.0*this->Radial[2]*x*y + this->Radial[3]*(rr+2*x*x);
	u[1] = c*y + this->Radial[2]*(rr+2*y*y) + 2.0*this->Radial[3]*x*y;

	u[0] = this->K[0][0]*u[0] + this->K[0][1]*u[1] + this->K[0][2];
	u[1] = this->K[1][1]*u[1] + this->K[1][2];
	u[2] = X[2];

}
void GCameraView::World2ImgWithDistortion(double const*p, double u[2]) const
{
	double X[3];
	for(int i=0; i<3; i++)
	{
		X[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}

	double x = X[0]/X[2];
	double y = X[1]/X[2];
	double rr = x*x + y*y;
	double c = 1.0 + this->Radial[0]*rr + this->Radial[1]*rr*rr + this->Radial[4]*rr*rr*rr;
	
	u[0] = c*x + 2.0*this->Radial[2]*x*y + this->Radial[3]*(rr+2*x*x);
	u[1] = c*y + this->Radial[2]*(rr+2*y*y) + 2.0*this->Radial[3]*x*y;

	u[0] = this->K[0][0]*u[0] + this->K[0][1]*u[1] + this->K[0][2];
	u[1] = this->K[1][1]*u[1] + this->K[1][2];
}

void GCameraView::World2ImgWithDistortionV3(float const*p, float u[3]) const
{
	float X[3];
	for(int i=0; i<3; i++)
	{
		X[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}

	float x = X[0]/X[2];
	float y = X[1]/X[2];
	float rr = x*x + y*y;
	float c = 1.0 + this->Radial[0]*rr + this->Radial[1]*rr*rr + this->Radial[4]*rr*rr*rr;
	
	u[0] = c*x + 2.0*this->Radial[2]*x*y + this->Radial[3]*(rr+2*x*x);
	u[1] = c*y + this->Radial[2]*(rr+2*y*y) + 2.0*this->Radial[3]*x*y;

	u[0] = this->K[0][0]*u[0] + this->K[0][2];
	u[1] = this->K[1][1]*u[1] + this->K[1][2];
	u[2] = X[2];
}
void GCameraView::World2ImgWithDistortion(float const*p, float u[2]) const
{
	float X[3];
	for(int i=0; i<3; i++)
	{
		X[i] = this->R[i][0]*p[0] + this->R[i][1]*p[1] + this->R[i][2]*p[2] + this->T[i];
	}

	float x = X[0]/X[2];
	float y = X[1]/X[2];
	float rr = x*x + y*y;
	float c = 1.0 + this->Radial[0]*rr + this->Radial[1]*rr*rr + this->Radial[4]*rr*rr*rr;
	
	u[0] = c*x + 2.0*this->Radial[2]*x*y + this->Radial[3]*(rr+2*x*x);
	u[1] = c*y + this->Radial[2]*(rr+2*y*y) + 2.0*this->Radial[3]*x*y;

	u[0] = this->K[0][0]*u[0] + this->K[0][1]*u[1] + this->K[0][2];
	u[1] = this->K[1][1]*u[1] + this->K[1][2];
}

bool GCameraView::ImgCoordAndDepth2WorldCoord(double x_in[3], double x_w[3]) const
{
	double p_in_cam[3];
	double fx = this->K[0][0];
	double fy = this->K[1][1];
	double cx = this->K[0][2];
	double cy = this->K[1][2];

	double z = x_in[2];
	double x = (x_in[0] - cx)*z/fx;
	double y = (x_in[1] - cy)*z/fy;
	p_in_cam[0] = x;
	p_in_cam[1] = y;
	p_in_cam[2] = z;

	this->cameraSpace2worldSpace(p_in_cam, x_w);
	
	return true;
}

double* GCameraView::ImgCoordAndDepth2WorldCoord(double x_in[3]) const
{
	double *ret = new double[3];
	this->ImgCoordAndDepth2WorldCoord(x_in, ret);

	return ret;
}

bool GCameraView::ImgCoordAndDepth2WorldCoord(float x_in[3], float x_w[3]) const
{
	float p_in_cam[3];
	float fx = this->K[0][0];
	float fy = this->K[1][1];
	float cx = this->K[0][2];
	float cy = this->K[1][2];

	float z = x_in[2];
	float x = (x_in[0] - cx)*z/fx;
	float y = (x_in[1] - cy)*z/fy;
	p_in_cam[0] = x;
	p_in_cam[1] = y;
	p_in_cam[2] = z;

	this->cameraSpace2worldSpace(p_in_cam, x_w);
	
	return true;
}

float* GCameraView::ImgCoordAndDepth2WorldCoord(float x_in[3]) const
{
	float *ret = new float[3];
	this->ImgCoordAndDepth2WorldCoord(x_in, ret);

	return ret;
}

void GCameraView::normalizeVec(double *v, int len)
{
	double sum=0;
	for(int i=0; i<len; i++)
	{
		sum += v[i]*v[i];
	}
	sum = sqrt(sum);

	if( sum > 0)
	{
		for(int i=0; i<len; i++)	
			v[i] /= sum;
	}
}

void GCameraView::initUndistortMap()
{
	if (!this->mapX.empty())
		mapX.release();
	if(!this->mapY.empty())
		mapY.release();

	if (this->m_Width <= 0 || this->m_Height <= 0)
		return;

	this->mapX = cv::Mat(this->m_Height, this->m_Width, CV_32FC1);
	this->mapY = cv::Mat(this->m_Height, this->m_Width, CV_32FC1);

	cv::Mat intrinsic = cv::Mat(3, 3, CV_64F, &this->K[0][0]);
	cv::Mat distortCoef = cv::Mat(5, 1, CV_64F, &this->Radial[0]); 
	
	cv::initUndistortRectifyMap(intrinsic, distortCoef, cv::Mat(), intrinsic, cv::Size(this->m_Width, this->m_Height), CV_32FC1, this->mapX, this->mapY);
}

void GCameraView::UndistortImage(cv::InputArray src, cv::OutputArray dst, int flags)
{
	if(this->mapX.empty() || this->mapY.empty())
		this->initUndistortMap();
	cv::remap(src, dst, this->mapX, this->mapY, flags);
}

int GCameraView::UndistortDepthMap(cv::InputArray src, cv::OutputArray dst)
{
	if(this->mapX.empty() || this->mapY.empty())
		this->initUndistortMap();
	if( src.cols() != dst.cols() ||
		src.rows() != dst.rows())
	{
		printf("Error: the input Matrices have different size!\n");
		return -1;
	}

	int h = src.rows();
	int w = src.cols();

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			int x_src = ROUND(mapX.at<float>(i, j));
			int y_src = ROUND(mapY.at<float>(i, j));

			if( x_src >=0 && x_src < w &&
				y_src >=0 && y_src < h)
			{				
				dst.getMat().at<float>(i, j) = src.getMat().at<float>(y_src, x_src);
			}
			else
				dst.getMat().at<float>(i, j) = 0.0;
		}
	}
	return 1;
}

GCameraView* GenerateIntermediateCameraView(GCameraView* cam1, GCameraView *cam2, double s)
{
	if( cam1 == NULL || cam2 == NULL )
		return NULL;
	int height_new = cam1->m_Height*(1-s) + cam2->m_Height*s;
	int width_new = cam1->m_Width*(1-s) + cam2->m_Width*s;
	GCameraView *cam3 = new GCameraView(height_new, width_new);

	//new intrinsics
	cv::Mat A_new = cv::Mat(3, 3, CV_64F);
	cv::addWeighted(cam1->intrinsics, 1 - s, cam2->intrinsics, s, 0, A_new);

	cv::Mat R_new = cv::Mat(3, 3, CV_64F);
	cv::Mat R_rod_new = cv::Mat(3, 1, CV_64F);
	cv::Mat R_rod_1 = cv::Mat(3, 1, CV_64F);
	cv::Mat R_rod_2 = cv::Mat(3, 1, CV_64F);
	cv::Rodrigues(cam1->rotation, R_rod_1);
	cv::Rodrigues(cam2->rotation, R_rod_2);
	cv::addWeighted(R_rod_1, 1 - s, R_rod_2, s, 0, R_rod_new);
	cv::Rodrigues(R_rod_new, R_new);

	cv::Mat T_new = cv::Mat(3, 1, CV_64F);

	cv::addWeighted(cam1->translation, 1 - s, cam2->translation, s, 0, T_new);

	cam3->SetCalibrationMatrix(A_new, R_new, T_new);
	
	A_new.release();
	R_new.release();
	R_rod_new.release();
	R_rod_1.release();
	R_rod_2.release();
	T_new.release();

	return cam3;
}
