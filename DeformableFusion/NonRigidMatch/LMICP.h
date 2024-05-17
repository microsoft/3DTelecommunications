//===============================================
//			LMICP.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================

#ifndef __LMICP_H__
#define __LMICP_H__
#include "DeformGraphOptDensePtsAndClr.h"

class LMICP
{
public:
	LMICP(char *ddf_dir_ = NULL){this->ddf_dir = ddf_dir_;};
	~LMICP(){};

public:
	//surface_ref: reference surface in the world space; normal must be precalculated and pointing against camera
	// depthMat and img are aligned and with the camera view of 'cams_t'
	// <R, T>--in and out: will transform surface_ref to the target, should be properly intialized
	void operator() (CSurface<float> const& surface_ref,
						vector<cv::Mat> depthMats_t, //without len distortion
						vector<cv::Mat> imgs_t, //without lens distortion, 
						vector<GCameraView*> cams_t,
						vector<GCameraView*> cams_clr_t,
						vnl_matrix_fixed<double, 3, 3> &R,
						vnl_vector_fixed<double, 3> &T,
						int frmIdx_t = 0,
						int level = 1,
						double res_ddf = 1.0,
						bool bUseRoughDDFForMatching = true
					  );

	void operator() ( CSurface<float> const& surface_ref, 
					  vector<cv::Mat> imgs_t, //without lens distortion, 
					  vector<GCameraView*> cams_t,
					  vnl_matrix_fixed<double, 3, 3> &R, 
					  vnl_vector_fixed<double, 3> &T,
					  int frmIdx_t = 0
					  );

	void operator() ( CSurface<float> const& surface_ref,
					  cv::Mat const& depthMat_t,
					  cv::Mat const& img_t,
					  GCameraView const* cam_t,
					  GCameraView const* cam_clr_t,
					  vnl_matrix_fixed<double, 3, 3> &R, 
					  vnl_vector_fixed<double, 3> &T,
					  int frmIdx_t = 0 );

	//<R, T>--in and out: will transform points in reference camera space to those in the template camera space.
	void operator() ( cv::Mat const& depthMat_ref, //reference 
					  cv::Mat const& img_ref,
					  cv::Mat const& depthMat_t, //template
					  cv::Mat const& img_t,
					  vnl_matrix_fixed<double, 3, 3> const& intrinsics_ref,
					  vnl_matrix_fixed<double, 3, 3> const& intrinsics_t,
					  vnl_matrix_fixed<double, 3, 3> &R, 
					  vnl_vector_fixed<double, 3> &T,
					  int frmIdx_t = 0 );

private:
	char* ddf_dir;
#ifdef USE_CERES_SOLVER
	void run_opt_ceres( NonrigidMatching::DeformGraphOptMultiDataDensePtsAndClr &data, int step = 1 );
#endif

};

#endif