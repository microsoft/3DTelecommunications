#ifndef __TSDF_H__
#define __TSDF_H__
#include "VoxelMatrix.h"
#include "UtilVnlMatrix.h"
#include "CSurface.h"
#include <vpgl/vpgl_perspective_camera.h>
#include "surface_misc.h"
#include "CameraView.h"
#include "basic_structure.h"
#include "func_jacobians.h"

#define SDF_NULL_VALUE -2.0f

class SignedDistanceFunc
{
public:
	//normal is pointing inwards
	virtual float val_at(double const* pos, double const* normal) const = 0;
	virtual float val_at(double const*pos, double const*normal, double val_for_null_cell) const
	{
		return val_at(pos, normal);
	}
	// compute the first order partial dirivative at point ( x, y, z) in the 
	// world space--gradient---the direction of the greatest rate of increase of the scarlar field
	// the last parameter is the threshold on the angle between normal and der, is the angle (in degree) is greater than angle_thres, return der with 0s.
	virtual vnl_vector_fixed<double, 3> der_at(double const* pos, double const* normal, double angle_thres = 60) const = 0; 

	//return true if the color dertimination is defined; false otherwise
	virtual bool clr_fld_der_x_at(double const*pos, double *der_clr) const = 0;
	virtual bool clr_fld_der_y_at(double const*pos, double *der_clr) const = 0;
	virtual bool clr_fld_der_z_at(double const*pos, double *der_clr) const = 0;
	virtual vnl_vector_fixed<float, 3> clr_at(double x, double y, double z) const = 0;

	virtual BoundingBox3D get_bbox() const = 0;
};


//truncated signed distance function: value descenting direction points away from the camera center
class TSDF : public SignedDistanceFunc
{
public:
	TSDF()
		: nx(0), ny(0),nz(0),
		  func_val(0,0,0,SDF_NULL_VALUE, SDF_NULL_VALUE),
		  weights(0,0,0, -1.0, -1.0),
		  clr_field(0,0,0, vnl_vector_fixed<float, 3>(0.0,0.0,0.0), vnl_vector_fixed<float, 3>(0.0,0.0,0.0))
	{
	}
	TSDF( double x_start, double x_end, 
		  double y_start, double y_end, 
		  double z_start, double z_end,
		  double x_res, double y_res, double z_res,
		  double mu_,
		  bool bDynamicMu_ = false,
		  bool bDynamicWeight_ = false,
		  bool bBuildColorField_ = false);
	TSDF( BoundingBox3D bbox,
		  double x_res, double y_res, double z_res,
		  double mu_,
		  bool bDynamicMu_ = false,
		  bool bDynamicWeight_ = false,
		  bool bBuildColorField_ = false);
	~TSDF()	{}
public:
	virtual void Init( double x_start, double x_end, 
					   double y_start, double y_end, 
					   double z_start, double z_end,
					   double x_res, double y_res, double z_res,
					   double mu_,
	  				   bool bDynamicMu_,
					   bool bDynamicWeight_,
					   bool bBuildColorField);
	virtual void Init( double x_start, double y_start,double z_start, 
				   	   int nx_, int ny_, int nz_,
					   double x_res, double y_res, double z_res,
					   double mu_,
	  				   bool bDynamicMu_,
					   bool bDynamicWeight_,
					   bool bBuildColorField);
	virtual void Init( BoundingBox3D bbox,
					   double x_res, double y_res, double z_res,
					   double mu_,
					   bool bDynamicMu_,
					   bool bDynamicWeight_,
					   bool bBuildColorField);
public:
	/* the intrinsics must be set in cam_pose.
	 * voxels in the front of the surface have postive vals, while voxels behind surface have 
	 * negative values.
	 */
	bool TSDF::add_a_frame_occupancy(cv::Mat const& depthMat, GCameraView *cam_d, cv::Mat const& img, GCameraView *cam_clr);

	virtual bool add_a_frame(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam_pose, cv::Mat& img = cv::Mat() );
	virtual bool add_a_frame(cv::Mat& depthMat, GCameraView *cam, cv::Mat& img = cv::Mat());
	
	//find the closest vertex for each voxel
	virtual bool add_a_frame2(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam_pose);

	//have a set of auxiliary cameras to improve the sdf
	//assume all auxiliary cameras have the same intrinsics
	bool add_a_frame( vnl_matrix<double> const&depthMat, vpgl_perspective_camera<double> const&cam_ori, 
					  vector<vpgl_perspective_camera<double> > const&cams_aux);

	//bool add_a_frame(CSurface<float> const& surface);
	bool add_a_frame(TSDF const& sdf);
	bool generate_surface(CSurface<float> &surface, double iso_level = 0.0, bool bGenerateTexture = false, bool bVerbose = true);
	bool generate_surface_occupancy(CSurface<float> &surface, double iso_level = 0.0, bool bGenerateTexture = false, bool bVerbose = true);

	virtual bool texture_surface(CSurface<float> &surface) const;
	void get_error_map_on_surface(CSurface<float> &surface, bool bNormalCorrected = true) const;


public:
//---------------------Color Field Operations------------------------------------
	vnl_vector_fixed<float, 3> clr_at(double x, double y, double z) const
	{
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;
		vnl_vector_fixed<float, 3> ret = this->clr_field.val_at(x_grid, y_grid, z_grid);
		return ret;
	}

	bool clr_fld_der_x_at(double const*pos, double *der_clr) const
	{
		double x_grid = (pos[0] - xoffset) / xres;
		double y_grid = (pos[1] - yoffset) / yres;
		double z_grid = (pos[2] - zoffset) / zres;

		vnl_vector_fixed<float, 3> Gx;
		if (!this->clr_field.sobel_x(x_grid, y_grid, z_grid, Gx))
			return false;
		der_clr[0] = Gx[0] / xres;
		der_clr[1] = Gx[1] / xres;
		der_clr[2] = Gx[2] / xres;
		return true;
	}

	bool clr_fld_der_y_at(double const*pos, double *der_clr) const
	{
		double x_grid = (pos[0] - xoffset) / xres;
		double y_grid = (pos[1] - yoffset) / yres;
		double z_grid = (pos[2] - zoffset) / zres;

		vnl_vector_fixed<float, 3> Gy;
		if (!this->clr_field.sobel_y(x_grid, y_grid, z_grid, Gy))
			return false;
		der_clr[0] = Gy[0] / yres;
		der_clr[1] = Gy[1] / yres;
		der_clr[2] = Gy[2] / yres;
		return true;
	}

	bool clr_fld_der_z_at(double const*pos, double *der_clr) const
	{
		double x_grid = (pos[0] - xoffset) / xres;
		double y_grid = (pos[1] - yoffset) / yres;
		double z_grid = (pos[2] - zoffset) / zres;

		vnl_vector_fixed<float, 3> Gz;
		if (!this->clr_field.sobel_z(x_grid, y_grid, z_grid, Gz))
			return false;
		der_clr[0] = Gz[0] / zres;
		der_clr[1] = Gz[1] / zres;
		der_clr[2] = Gz[2] / zres;
		return true;
	}

//-----------------------Signed Distance Field Operations-------------------
	double val_at_awf(double x, double y, double z) const
	{
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;

		return this->func_val.val_at_awf(x_grid, y_grid, z_grid);
	}

	// compute the first order partial dirivative at point ( x, y, z) in the 
	// camera space--gradient---the direction of the greatest rate of increase of the scarlar field
	vnl_vector_fixed<double, 3> der_at_awf(double x, double y, double z) const
	{
		vnl_vector_fixed<double, 3> ret(0.0);
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;

		float der[3];
		this->func_val.der_at_awf(x_grid, y_grid, z_grid, der);
		ret[0] = der[0] / xres;
		ret[1] = der[1] / yres;
		ret[2] = der[2] / zres;
		return ret;
	}

	vnl_matrix_fixed<double, 3, 3> der2nds_at_awf(double x, double y, double z) const
	{
		vnl_matrix_fixed<double, 3, 3> ret(0.0);
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;

		float der2nd[9];
		this->func_val.der2nd_at_awf(x_grid, y_grid, z_grid, der2nd);
		ret(0, 0) = der2nd[0] / (xres*xres);
		ret(0, 1) = der2nd[1] / (xres*yres);
		ret(0, 2) = der2nd[2] / (xres*zres);
		ret(1, 0) = der2nd[3] / (yres*xres);
		ret(1, 1) = der2nd[4] / (yres*yres);
		ret(1, 2) = der2nd[5] / (yres*zres);
		ret(2, 0) = der2nd[6] / (zres*xres);
		ret(2, 1) = der2nd[7] / (zres*yres);
		ret(2, 2) = der2nd[8] / (zres*zres);
		return ret;
	} 

	double val_at_awf_ck(double x, double y, double z) const
	{
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;
		double ret = this->func_val.val_at(x_grid, y_grid, z_grid);

		if (ret == SDF_NULL_VALUE)
			ret = 0.5;

		return ret;
	}

	double val_at_raw(double x, double y, double z) const
	{
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;
		return this->func_val.val_at(x_grid, y_grid, z_grid);
	}

	// compute the first order partial dirivative at point ( x, y, z) in the 
	// camera space--gradient---the direction of the greatest rate of increase of the scarlar field
	vnl_vector_fixed<double, 3> der_at_awf_ck(double x, double y, double z) const
	{
		vnl_vector_fixed<double, 3> ret(0.0);
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;

		float der[3];
		this->func_val.der_at(x_grid, y_grid, z_grid, der);
		ret[0] = der[0] / xres;
		ret[1] = der[1] / yres;
		ret[2] = der[2] / zres;
		return ret;
	}

	vnl_matrix_fixed<double, 3, 3> der2nds_at_awf_ck(double x, double y, double z) const
	{
		vnl_matrix_fixed<double, 3, 3> ret(0.0);
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;

		float der2nd[9];
		this->func_val.der2nd_at(x_grid, y_grid, z_grid, der2nd);
		ret(0, 0) = der2nd[0] / (xres*xres);
		ret(0, 1) = der2nd[1] / (xres*yres);
		ret(0, 2) = der2nd[2] / (xres*zres);
		ret(1, 0) = der2nd[3] / (yres*xres);
		ret(1, 1) = der2nd[4] / (yres*yres);
		ret(1, 2) = der2nd[5] / (yres*zres);
		ret(2, 0) = der2nd[6] / (zres*xres);
		ret(2, 1) = der2nd[7] / (zres*yres);
		ret(2, 2) = der2nd[8] / (zres*zres);
		return ret;
	}

	// f = |D(x)|*sigmoid(d(D)/dx, n) + 1.0 - sigmoid(d(D)/dx, n) 
	void val_der_sigmoid(double const*x, double const*n, double &val, vnl_vector_fixed<double, 3> &df_dx, vnl_vector_fixed<double, 3> &df_dn, double *p_theta = NULL) const
	{
		const double lamda = 20.0;
		const double tau = 0.50;
		const double lower_bound = 0.35;
		const double upper_bound = 0.99;

		vnl_vector_fixed<double, 3> dD_dx_hack = this->der_at(x, n, 180);
		vnl_vector_fixed<double, 3> dD_dx = dD_dx_hack;
		vnl_vector_fixed<double, 3> dDabs_dx = dD_dx;
		double val_ori = val_at_awf(x[0], x[1], x[2]);
		if (val_ori < 0)	dDabs_dx = -dDabs_dx;

		vnl_vector_fixed<double, 3> dD_dx_n = dD_dx;
		dD_dx_n.normalize();
		vnl_vector_fixed<double, 3> normal(n);
		double theta = dot_product(dD_dx_n, -normal);
		double tmp = std::exp(-lamda*(theta - tau));
		double sig = 1.0 / (1.0 + tmp);
		double d_sig = lamda*tmp / ((1.0 + tmp)*(1.0 + tmp));

		if (theta < lower_bound)
		{
			sig = 0.0;
			d_sig = 0.0;
		}
		else if (theta > upper_bound)
		{
			sig = 1.0;
			d_sig = 0.0;
		}

		if (p_theta != NULL)
			*p_theta = theta;

		double Dx_abs = std::abs(val_ori);
		val = Dx_abs*sig + 1.0 - sig;

		vnl_matrix_fixed<double, 3, 3> T;
		vnl_vector_fixed<double, 3> dD_dx_o;
		Jac_normalization(dD_dx, dD_dx_o, T);
		vnl_matrix_fixed<double, 3, 3> Dx2nd = this->der2nds_at_awf(x[0], x[1], x[2]);
		df_dx = sig*dDabs_dx +(Dx_abs - 1.0)*d_sig*(-normal)*(T*Dx2nd);

		df_dn = -(Dx_abs - 1.0)*d_sig*dD_dx_n;
	}

	//deal with undefined cells
	void val_der_sigmoid_ck(double const*x, double const*n, double &val, vnl_vector_fixed<double, 3> &df_dx, vnl_vector_fixed<double, 3> &df_dn, double *p_theta = NULL) const
	{
		const double lamda = 20.0;
		const double tau = 0.70;
		const double lower_bound = 0.35;
		const double upper_bound = 0.99;

		vnl_vector_fixed<double, 3> dD_dx = this->der_at_awf_ck(x[0], x[1], x[2]);
		vnl_vector_fixed<double, 3> dDabs_dx = dD_dx;
		double val_raw = val_at_raw(x[0], x[1], x[2]);
		double val_ori = (val_raw == SDF_NULL_VALUE) ? 1.0 : val_raw;
		if (val_ori < 0)	dDabs_dx = -dDabs_dx;

		vnl_vector_fixed<double, 3> dD_dx_n = dD_dx;
		dD_dx_n.normalize();
		vnl_vector_fixed<double, 3> normal(n);
		double theta = dot_product(dD_dx_n, -normal);
		double tmp = std::exp(-lamda*(theta - tau));
		double sig = 1.0 / (1.0 + tmp);
		double d_sig = lamda*tmp / ((1.0 + tmp)*(1.0 + tmp));

		if (p_theta != NULL)
			*p_theta = theta;

		double Dx_abs = std::abs(val_ori);
		val = Dx_abs*sig + 1.0 - sig;

		if (val_raw == SDF_NULL_VALUE)
			val = 1.0;

		vnl_matrix_fixed<double, 3, 3> T;
		vnl_vector_fixed<double, 3> dD_dx_o;
		Jac_normalization(dD_dx, dD_dx_o, T);
		vnl_matrix_fixed<double, 3, 3> Dx2nd = this->der2nds_at_awf_ck(x[0], x[1], x[2]);
		df_dx = sig*dDabs_dx +(Dx_abs - 1.0)*d_sig*(-normal)*(T*Dx2nd);

		df_dn = -(Dx_abs - 1.0)*d_sig*dD_dx_n;
	}

	//assume sigmoid is constant
	void val_der_sigmoid_ck_approx(double const*x, double const*n, double &val, vnl_vector_fixed<double, 3> &df_dx, double *p_theta = NULL) const
	{
		const double lamda = 20.0;
		const double tau = 0.6;
		const double lower_bound = 0.1;
		const double upper_bound = 0.99;

		vnl_vector_fixed<double, 3> dD_dx = this->der_at_awf_ck(x[0], x[1], x[2]);
		vnl_vector_fixed<double, 3> dDabs_dx = dD_dx;
		double val_raw = val_at_raw(x[0], x[1], x[2]);
		double val_ori = (val_raw == SDF_NULL_VALUE) ? 1.0 : val_raw;
		if (val_ori < 0)	dDabs_dx = -dDabs_dx;

		vnl_vector_fixed<double, 3> dD_dx_n = dD_dx;
		dD_dx_n.normalize();
		vnl_vector_fixed<double, 3> normal(n);
		double theta = dot_product(dD_dx_n, -normal);
		double tmp = std::exp(-lamda*(theta - tau));
		double sig = 1.0 / (1.0 + tmp);

		if (p_theta != NULL)
			*p_theta = theta;

		double Dx_abs = std::abs(val_ori);
		val = Dx_abs*sig + 1.0 - sig;

		if (val_raw == SDF_NULL_VALUE)
			val = 1.0;

		df_dx = sig*dDabs_dx;		
	}

	float val_at(double x, double y, double z, double val_for_null_cell = 1.0) const
	{
		double x_grid = (x - xoffset) / xres;
		double y_grid = (y - yoffset) / yres;
		double z_grid = (z - zoffset) / zres;
		float ret = this->func_val.val_at(x_grid, y_grid, z_grid);

		if (ret == SDF_NULL_VALUE)
			return val_for_null_cell;
		else
			return ret;
	}

	virtual float val_at(double const* pos, double const* normal) const
	{
		return val_at(pos[0], pos[1], pos[2], 1.0);
	}
	virtual float val_at(double const*pos, double const*normal, double val_for_null_cell) const
	{
		return val_at(pos[0], pos[1], pos[2], val_for_null_cell);
	}

	//compute the first order dirivative at point ( x, y, z) in the camera space
	vnl_vector_fixed<double, 3> der_at(double x, double y, double z) const
	{
		double x_grid = (x-xoffset)/xres;
		double y_grid = (y-yoffset)/yres;
		double z_grid = (z-zoffset)/zres;
		vnl_vector_fixed<double, 3> ret;
		if (!func_val.dif_x(x_grid, y_grid, z_grid, ret[0]) ||
			!func_val.dif_y(x_grid, y_grid, z_grid, ret[1]) ||
			!func_val.dif_z(x_grid, y_grid, z_grid, ret[2]))
		{
			ret[0] = 0.0; ret[1] = 0.0; ret[2] = 0.0;
		}

		ret[0] /= xres;
		ret[1] /= yres;
		ret[2] /= zres;
		return ret;		
	}
	
	virtual vnl_vector_fixed<double, 3> der_at(double const* pos, double const* normal, double angle_thres) const 
	{
		return der_at(pos[0], pos[1], pos[2]);
	}

	//return false if out of boundary
	bool get_val_and_weight(double x, double y, double z, double &val, double &weight) const
	{
		double x_grid = (x-xoffset)/xres;
		double y_grid = (y-yoffset)/yres;
		double z_grid = (z-zoffset)/zres;
		val = this->func_val.val_at(x_grid, y_grid, z_grid);
		weight = this->weights.val_at(x_grid, y_grid, z_grid);

		if( x_grid < -0.5 || x_grid > nx-0.5 ||
			y_grid < -0.5 || y_grid > ny-0.5 ||
			z_grid < -0.5 || z_grid > nz-0.5  )
			return false;
		else
			return true;
	}

	BoundingBox3D get_bbox() const;

//I/O
public:
	void printDim(char const* str="") const {printf("DDF %s: Dim<%d, %d, %d>, Res<%f, %f, %f>, Offset<%f, %f, %f>\n", str, this->nx, this->ny, this->nz,
																														 this->xres, this->yres, this->zres, 
																														 this->xoffset, this->yoffset, this->zoffset);}
	virtual bool save_data_to_file(const char* filename);
	virtual bool load_data_from_file(const char* filename);
	bool save_func_val_to_file_bin(const char* filename)	{ return this->func_val.save_to_file_bin(filename); }
	bool load_func_val_from_file_bin(const char* filename)	{ return this->func_val.load_from_file_bin(filename); }
	bool save_weights_to_file_bin(const char* filename)		{ return this->weights.save_to_file_bin(filename); }
	bool load_weights_from_file_bin(const char* filename)	{ return this->weights.load_from_file_bin(filename); }

public:
	static bool generate_surface(CSurface<float> &m_surface, double iso_level, 
								 VoxelMatrix<float> const& volume, 
								 float x_res, float y_res, float z_res, 
								 float x_offset, float y_offset_, float z_offset, 
								 bool bVerbose);

//test code
public:
	void vals_at(vector< vnl_vector_fixed<double, 3> > const& points, vnl_vector<double> &vals);
	void ders_at(vector< vnl_vector_fixed<double, 3> > const& points, vector< vnl_vector_fixed<double, 3> > &ders);

public:
	VoxelMatrix<float> func_val;
	VoxelMatrix< vnl_vector_fixed<float, 3> > clr_field;
	VoxelMatrix<float> weights;

	double mu;
	bool bDynamicMu;
	bool bDynamicWeight;
	bool bBuildColorField;
public:
	int nx;
	int ny;
	int nz;
	double xres;
	double yres;
	double zres;
	double xoffset; // the left-bottom-front corner of the cube
	double yoffset;
	double zoffset;

public:
	static bool IsNull( float val )
	{ 
		double thres = 1.5;
		if(val > thres || val < -thres)
			return true;
		else
			return false;
	}

	static bool IsNull_MarchingCubes( float val )
	{ 
		double thres = 0.75;
		if(val > thres || val < -thres)
			return true;
		else
			return false;
	}

	static bool IsNull_MarchingCubes_Occupancy(float val)
	{
		if (val > 1.0 || val < 0.0)
			return true;
		else
			return false;
	}
};

#endif