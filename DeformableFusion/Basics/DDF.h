// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __DDF_H__
#define __DDF_H__
#include "TSDF.h"

// directional distance function
// direction points insides (against camera)
class DDF : public TSDF
{
public:
	DDF() 
		: TSDF(),
		drts(0,0,0, vnl_vector_fixed<float, 3>(0.0,0.0,0.0),vnl_vector_fixed<float, 3>(0.0,0.0,0.0))
	{}
	DDF( double x_start, double x_end, 
		 double y_start, double y_end, 
		 double z_start, double z_end,
		 double x_res, double y_res, double z_res,
		 double mu_,
		 bool bDynamicMu_ = false,
		 bool bDynamicWeight_ = false,
		 bool bBuildColorField_ = false)
		 : TSDF(x_start, x_end, y_start, y_end, z_start, z_end, x_res, y_res, z_res, mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_)
	{
		this->drts.set_size(nx, ny, nz);
		this->drts.fill( vnl_vector_fixed<float, 3>(0,0,0) );
		this->drts.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));
	}
	DDF( BoundingBox3D bbox,
		 double x_res, double y_res, double z_res,
		 double mu_,
		 bool bDynamicMu_ = false,
		 bool bDynamicWeight_ = false,
		 bool bBuildColorField_ = false)
		 : TSDF(bbox, x_res, y_res, z_res, mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_)
	{
		this->drts.set_size(nx, ny, nz);
		this->drts.fill( vnl_vector_fixed<float, 3>(0,0,0) );
		this->drts.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));
	}

public:
	void Init( double x_start, double x_end, 
			   double y_start, double y_end, 
			   double z_start, double z_end,
			   double x_res, double y_res, double z_res,
			   double mu_,
			   bool bDynamicMu_=false,
			   bool bDynamicWeight_=false,
			   bool bBuildColorField_ = false)
	{
		TSDF::Init(x_start, x_end, y_start, y_end, z_start, z_end, x_res, y_res, z_res, mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_);
		this->drts.set_size(nx, ny, nz);
		this->drts.fill( vnl_vector_fixed<float, 3>(0,0,0) );
		this->drts.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));

	}
	virtual void Init( double x_start, double y_start,double z_start, 
				   	   int nx_, int ny_, int nz_,
					   double x_res, double y_res, double z_res,
					   double mu_,
	  				   bool bDynamicMu_=false,
					   bool bDynamicWeight_=false,
					   bool bBuildColorField_ = false)
	{
		TSDF::Init(x_start, y_start, z_start, nx_, ny_, nz_, x_res, y_res, z_res, mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_);
		this->drts.set_size(nx, ny, nz);
		this->drts.fill( vnl_vector_fixed<float, 3>(0,0,0) );
	}
	void Init( BoundingBox3D bbox,
			   double x_res, double y_res, double z_res,
			   double mu_,
			   bool bDynamicMu_=false,
			   bool bDynamicWeight_=false,
			   bool bBuildColorField_ = false)
	{
		TSDF::Init(bbox, x_res, y_res, z_res, mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_);
		this->drts.set_size(nx, ny, nz);
		this->drts.fill( vnl_vector_fixed<float, 3>(0,0,0) );
		this->drts.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));
	}

	//extra_radius: extra voxels for filtering. e.g., -1, filter size: rate-2 X rate-2 X rate-2
	DDF* downsample(int rate, int extra_radius, double thres_count_ratio = 0.55);

public:
	// compute the first order partial dirivative at point ( x, y, z) in the 
	// camera space--gradient---the direction of the greatest rate of increase of the scarlar field
	vnl_vector_fixed<double, 3> der_at(double x, double y, double z) const
	{
		vnl_vector_fixed<double, 3> ret(0.0, 0.0, 0.0);
		double x_grid = (x-xoffset)/xres;
		double y_grid = (y-yoffset)/yres;
		double z_grid = (z-zoffset)/zres;

		vnl_vector_fixed<float, 3> drt = this->drts.val_at(x_grid, y_grid, z_grid);
		if (drt != this->drts.null_value())
		{
			ret[0] = -drt[0];
			ret[1] = -drt[1];
			ret[2] = -drt[2];
			ret.normalize();
			ret = ret/mu;
		}

		return ret;
	}

	virtual vnl_vector_fixed<double, 3> der_at(double const* pos, double const* normal, double angle_thres) const
	{
		//normal points inwards, while gradient(partial derivatives) points outwards
		vnl_vector_fixed<double, 3> ret = der_at(pos[0], pos[1], pos[2]);
		if( normal == NULL || 
			(ret[0] == 0.0 && ret[1] == 0.0 && ret[2] == 0.0)) 
			return ret;
		else
		{
			vnl_vector_fixed<double, 3> normal_vec(normal[0], normal[1], normal[2]);
			double alpha = angle_btw_two_vec(ret, -normal_vec) * 180 / M_PI;
			if( alpha < angle_thres )
				return ret;
			else
				return vnl_vector_fixed<double, 3>(0.0, 0.0, 0.0);
		}
	}
	

public:

	bool add_a_frame( vnl_matrix<double> const&depthMat, vpgl_perspective_camera<double> const&cam_pose, 
					  cv::Mat const& img=cv::Mat(), vpgl_perspective_camera<double> const& cam_clr=vpgl_perspective_camera<double>(),
					  bool bDecay = false, float a=0.7);
	bool add_a_frame( cv::Mat const& depthMat, GCameraView const*cam,
					  cv::Mat const& img = cv::Mat(), GCameraView const* cam_clr = NULL,
					  bool bDecay = false, float a = 0.7)
	{
		vnl_matrix<double> depthMat_vnl;
		cvmat_to_vnlmatrix(depthMat, depthMat_vnl);
		vpgl_perspective_camera<double> cam_vpgl;
		GCameraView_to_vpgl_camera_view(cam, cam_vpgl);
		vpgl_perspective_camera<double> cam_clr_vpgl;
		if( cam_clr != NULL )
			GCameraView_to_vpgl_camera_view(cam_clr, cam_clr_vpgl);

		return add_a_frame(depthMat_vnl, cam_vpgl, img, cam_clr_vpgl, bDecay, a);
	}

	//find the nearest vertex and compute distance function
	bool add_a_frame2(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam_pose, cv::Mat const& img=cv::Mat());
	bool add_a_frame2(cv::Mat const& depthMat, GCameraView const*cam_pose, cv::Mat const& img=cv::Mat())
	{
		vnl_matrix<double> depthMat_vnl;
		cvmat_to_vnlmatrix(depthMat, depthMat_vnl);
		vpgl_perspective_camera<double> cam;
		GCameraView_to_vpgl_camera_view(cam_pose, cam);
		return add_a_frame2(depthMat_vnl, cam, img);
	}

	//assume normal pointing inwards
	// if bDecay = true:
	//		   v_n+1 = [v_n * w_n * a + v_new * w_new *(1.0-a)] / [w_n*a + w_new*(1.0-a)]
	//         w_n+1 = w_n * a + w_new * (1.0-a);
	bool add_a_frame( CSurface<float> &surface, bool bDecay = false, float a = 0.7); 
	bool add_a_frame( DDF const& ddf, bool bDecay = false, float a = 0.7, bool bWeightUseDrt = false, 
					  vnl_vector_fixed<float, 3> const& avg_drt = vnl_vector_fixed<float, 3>(0.0));

	bool texture_surface(CSurface<float> &surface) const;

	vnl_vector_fixed<float, 3> get_avg_drt() const;

public:
	VoxelMatrix< vnl_vector_fixed<float, 3> > drts; //the direction of distance descenting, not normalized

public:
	//deprecated: not support read and load color_field
	bool save_data_to_file(const char* filename);
	bool load_data_from_file(const char* filename);

	bool save_data_to_file_zip(const char* filename);
	bool load_data_from_file_zip(const char* filename);
};

//smooth directional distance function
DDF* filter_DDF(DDF const* ddf);

#endif