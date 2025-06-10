// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "TSDF.h"

using namespace DirectX;

#ifndef float4
#define float4 XMFLOAT4
#endif

#ifndef float3
#define float3 XMFLOAT3
#endif

#ifndef float2
#define float2 XMFLOAT2
#endif



#define ONE_HALF  0.5f
#define ONE_SIXTH 1.0f/6.0f


float4 BPSVals[] = {
	{ 0.0f, 0.0f, 0.0f, 2.0f },
	{ 0.0f, 0.0f, 2.0f, 8.0f },
	{ 0.0f, 2.0f, 8.0f, 14.0f },
	{ 2.0f, 8.0f, 14.0f, 15.0f },
	{ 8.0f, 14.0f, 15.0f, 12.0f },
	{ 14.0f, 15.0f, 12.0f, 9.0f },
	{ 15.0f, 12.0f, 9.0f, 8.0f },
	{ 12.0f, 9.0f, 8.0f, 8.0f },
	{ 9.0f, 8.0f, 8.0f, 8.0f }
};

float mydot(float2& a, float2& b)
{
	return a.x*b.x + a.y*b.y;
}

float2 BPS(float x)
{
	if (x <= -3.0)
	{
		return float2(0.0, 0.0);
	}
	else if (x >= 6.0)
	{
		return float2(0.5, 0.0);
	}
	else
	{
		float tmp = floor(x + 3.0f);
		float4 cp0 = BPSVals[(int)tmp];
		float t = x - (tmp - 3.0);

		///deBoor Algorithm	
		float3 cp1 = float3((1.0 - t)*cp0.x + (2.0 + t)*cp0.y, (2.0 - t)*cp0.y + (1.0 + t)*cp0.z, (3.0 - t)*cp0.z + t*cp0.w); // implied factor of 1/3	
		float2 cp2 = float2((1.0 - t)*cp1.x + (1.0 + t)*cp1.y, (2.0 - t)*cp1.y + t*cp1.z); // implied factor of 1/2
		// (value, deriviative)
		return float2(ONE_SIXTH * mydot(float2(1.0 - t, t), cp2) / 16.0, ONE_HALF * mydot(float2(-1.0, 1.0), cp2) / 16.0);
	}
}


void TSDF::Init( double x_start, double x_end, 
				 double y_start, double y_end, 
				 double z_start, double z_end,
				 double x_res, double y_res, double z_res,
				 double mu_,
				 bool bDynamicMu_,
				 bool bDynamicWeight_,
				 bool bBuildColorField_)
{
	if (mu_ == 0.0)
	{
		LOGGER()->error("TSDF::Init","!!!!!!!!!!!!!!!!!!!!!!!!!!!Error: mu==0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		return;
	}
	LOGGER()->trace("TSDF:Init\n");
	this->nx = ROUND(abs(x_start-x_end)/x_res);
	this->xres = abs(x_start-x_end)/this->nx;
	this->xoffset = x_start;
	this->ny = ROUND(abs(y_start-y_end)/y_res);
	this->yres = abs(y_start-y_end)/this->ny;
	this->yoffset = y_start;
	this->nz = ROUND(abs(z_start-z_end)/z_res);
	this->zres = abs(z_start-z_end)/this->nz;
	this->zoffset = z_start;

	this->func_val.set_size(nx, ny, nz);
	this->func_val.fill(SDF_NULL_VALUE);
	this->func_val.set_outer_val(1.0);
	this->func_val.set_null_val(SDF_NULL_VALUE);
	this->weights.set_size(nx, ny, nz);
	this->weights.fill(0.0);
	if( bBuildColorField_ )
	{
		LOGGER()->trace("Color Field Build!\n");
		this->clr_field.set_size(nx, ny, nz);
		this->clr_field.fill(vnl_vector_fixed<float, 3>(0, 0, 0));
		this->clr_field.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));
	}

	this->mu = mu_;
	this->bDynamicMu = bDynamicMu_;
	this->bDynamicWeight = bDynamicWeight_;
	this->bBuildColorField = bBuildColorField_;
}

void TSDF::Init( double x_start, double y_start,double z_start, 
				 int nx_, int ny_, int nz_,
				 double x_res, double y_res, double z_res,
				 double mu_,
	  			 bool bDynamicMu_,
				 bool bDynamicWeight_,
				 bool bBuildColorField_)
{
	if (mu_ == 0.0)
	{
		LOGGER()->error("TSDF::Init","!!!!!!!!!!!!!!!!!!!!!!!!!!!Error: mu==0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		return;
	}
	this->nx = nx_;
	this->xres = x_res;
	this->xoffset = x_start;
	this->ny = ny_;
	this->yres = y_res;
	this->yoffset = y_start;
	this->nz = nz_;
	this->zres = z_res;
	this->zoffset = z_start;

	this->func_val.set_size(nx, ny, nz);
	this->func_val.fill(-2.0);
	this->func_val.set_outer_val(1.0);
	this->func_val.set_null_val(SDF_NULL_VALUE);
	this->weights.set_size(nx, ny, nz);
	this->weights.fill(0.0);
	if( bBuildColorField_ )
	{
		LOGGER()->info("Color Field Build!\n");
		this->clr_field.set_size(nx, ny, nz);
		this->clr_field.fill(vnl_vector_fixed<float, 3>(0, 0, 0));
		this->clr_field.set_null_val(vnl_vector_fixed<float, 3>(0, 0, 0));
	}

	this->mu = mu_;
	this->bDynamicMu = bDynamicMu_;
	this->bDynamicWeight = bDynamicWeight_;
	this->bBuildColorField = bBuildColorField_;
}

BoundingBox3D TSDF::get_bbox() const
{
	BoundingBox3D bbox;
	bbox.x_s = this->xoffset;
	bbox.y_s = this->yoffset;
	bbox.z_s = this->zoffset;
	bbox.x_e = this->xoffset + this->xres*this->nx;
	bbox.y_e = this->yoffset + this->yres*this->ny;
	bbox.z_e = this->zoffset + this->zres*this->nz;
	return bbox;
}

void TSDF::Init( BoundingBox3D bbox,
				 double x_res, double y_res, double z_res,
				 double mu_,
				 bool bDynamicMu_,
				 bool bDynamicWeight_,
				 bool bBuildColorField)
{
	this->Init( bbox.x_s, bbox.x_e, bbox.y_s, bbox.y_e, bbox.z_s, bbox.z_e,
				x_res, y_res, z_res,
				mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField);	
}

TSDF::TSDF( double x_start, double x_end, 
		    double y_start, double y_end, 
		    double z_start, double z_end,
		    double x_res, double y_res, double z_res,
			double mu_,
			bool bDynamicMu_,
			bool bDynamicWeight_,
			bool bBuildColorField_)
{
	this->Init( x_start, x_end, y_start, y_end, z_start, z_end, 
				x_res, y_res, z_res,
				mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_);
}

TSDF::TSDF( BoundingBox3D bbox,
			double x_res, double y_res, double z_res,
			double mu_,
			bool bDynamicMu_,
			bool bDynamicWeight_,
			bool bBuildColorField_)
{
	this->Init( bbox.x_s, bbox.x_e, bbox.y_s, bbox.y_e, bbox.z_s, bbox.z_e,
				x_res, y_res, z_res,
				mu_, bDynamicMu_, bDynamicWeight_, bBuildColorField_);	
}

bool TSDF::add_a_frame(cv::Mat& depthMat, GCameraView *cam, cv::Mat& img)
{
	if( depthMat.empty() || cam == NULL )
		return false;
	vnl_matrix<double> depth_mat;
	cvmat_to_vnlmatrix(depthMat, depth_mat);
	vpgl_perspective_camera<double> cam_pose;
	vnl_vector_fixed<double, 5> distort_coef;
	GCameraView_to_vpgl_camera_view(cam, cam_pose, distort_coef);

	return this->add_a_frame(depth_mat, cam_pose, img);
}

bool TSDF::add_a_frame(TSDF const& sdf)
{
	//a simple version: assuming the input sdf has the same dimension parameters
	assert( sdf.nx == this->nx && sdf.ny == this->ny && sdf.nz == this->nz &&
			sdf.xres == this->xres && sdf.yres == this->yres && sdf.zres == this->zres &&
			sdf.xoffset == this->xoffset && sdf.yoffset == this->yoffset && sdf.zoffset == this->zoffset );

	for(unsigned int k=0; k<nz; k++)
	{
		for(unsigned int j=0; j<ny; j++)
		{
			for(unsigned int i=0; i<nx; i++)
			{
				float val = sdf.func_val(i, j, k);
				if( IsNull(val) )
					continue;

				if( IsNull(this->func_val(i, j, k)))
				{
					this->func_val(i, j, k) = val;
					this->weights(i, j, k) = 1.0;
				}
				else
				{
					this->func_val(i, j, k) = (this->func_val(i, j, k)*this->weights(i, j, k) + val)/(1.0 + this->weights(i, j, k));
					this->weights(i, j, k) += 1.0;
				}
			}
		}
	}
	return true;
}

bool TSDF::add_a_frame_occupancy(cv::Mat const& depthMat, GCameraView *cam_d, cv::Mat const& img, GCameraView *cam_clr)
{
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(*cam_d, K);
	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];
	int img_width = depthMat.cols;
	int img_height = depthMat.rows;

	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(*cam_d, R, T);

	const double rho = this->mu/2.0;

	double mu_cur;

	for (int k = 0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for (int j = 0; j<ny; j++)
		{
			double y = j*yres + yoffset;
			for (int i = 0; i<nx; i++)
			{
				double x = i*xres + xoffset;
				vnl_vector_fixed<double, 3> X_wld(x, y, z);
				vnl_vector_fixed<double, 3> X_cam = R*X_wld + T;
				int u = ROUND(fx*X_cam[0] / X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1] / X_cam[2] + cy);

				//if the point is behind the camera, then skip it
				if (X_cam[2] <= 0)
					continue;

				// if the point cannot be observed at the current camera pose
				if (u < 0 || u >= img_width ||
					v < 0 || v >= img_height)
					continue;

				//if no depth measurement of the surface at this point
				double ds = depthMat.at<double>(v, u);;
				if (ds <= 0)
					continue;

				vnl_vector_fixed<double, 3> Pt;
				Pt[0] = (u - cx)*ds / fx;
				Pt[1] = (v - cy)*ds / fy;
				Pt[2] = ds;

				double Mr = Pt.magnitude();
				double dxr = X_cam.magnitude();

				if (bDynamicMu)
					mu_cur = MAX(this->mu, 0.5*ds*ds / (7.5 * 520 + 0.5*ds));
				else
					mu_cur = this->mu;

				// if the point is far behind the observed surface
				if (std::abs(Mr-dxr) > mu_cur)
					continue;

				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if (!img.empty() && this->bBuildColorField)
				{
					clr_cur[0] = img.at<cv::Vec3b>(v, u)[2] / 255.0;
					clr_cur[1] = img.at<cv::Vec3b>(v, u)[1] / 255.0;
					clr_cur[2] = img.at<cv::Vec3b>(v, u)[0] / 255.0;
				}

				double Pk1 = 0.5*std::erfc((Mr - dxr) / sqrt(2.0) / rho)-0.25*std::erfc((Mr-dxr+mu)/sqrt(2.0)/rho);
				double x_in = (dxr-Mr)/mu;

				double Pk = BPS(x_in).x;

				double Ok = this->func_val(i, j, k); //old value

				if (Ok == SDF_NULL_VALUE)
				{
					this->func_val(i, j, k) = Pk;
				}
				else
				{
					Ok = Ok*Pk / (Ok*Pk + (1.0 - Ok)*(1.0 - Pk));
					this->func_val(i, j, k) = Ok;
				}
			}
		}
	}

	return true;
}

bool TSDF::add_a_frame(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam,  cv::Mat& img)
{
	vnl_matrix_fixed<double, 3, 3> K = cam.get_calibration().get_matrix();
	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];
	int img_width = depthMat.cols();
	int img_height = depthMat.rows();

	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);


	double mu_cur;

	for(int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for(int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;	
			for(int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;
				vnl_vector_fixed<double, 3> X_wld(x, y, z);
				vnl_vector_fixed<double, 3> X_cam = R*X_wld + T;
				int u = ROUND(fx*X_cam[0]/X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1]/X_cam[2] + cy);

				//if the point is behind the camera, then skip it
				if( X_cam[2] <=0 )
					continue;

				// if the point cannot be observed at the current camera pose
				if( u < 0 || u >= img_width ||
					v < 0 || v >= img_height )
					continue;

				//if no depth measurement of the surface at this point
				double ds = depthMat[v][u];
				if( ds <= 0 )
					continue;

				if( bDynamicMu )
					mu_cur = MAX(this->mu, 0.5*ds*ds/(7.5*520+0.5*ds)); 
				else
					mu_cur = this->mu;
					
				
				// if the point is far behind the observed surface
				if( X_cam[2] - ds > mu_cur )
					continue;

				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if( !img.empty() && this->bBuildColorField)
				{
					clr_cur[0] = img.at<cv::Vec3b>(v, u)[2]/255.0;
					clr_cur[1] = img.at<cv::Vec3b>(v, u)[1]/255.0;
					clr_cur[2] = img.at<cv::Vec3b>(v, u)[0]/255.0;
				}

				double val_cur = MIN(1.0, (ds - X_cam[2])/mu_cur);

				double weight_cur = 1.0;
				if( this->bDynamicWeight )
					weight_cur = 150.0/ds;

				// if the function value at current point is NULL
				if( func_val(i, j, k) == SDF_NULL_VALUE )
				{
					func_val(i, j, k) = val_cur;
					weights(i, j, k) = weight_cur;
					if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )	
					{
						this->clr_field(i, j, k) = clr_cur;
					}
				}
				else
				{
					func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
					if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )
					{
						vnl_vector_fixed<float, 3> &clr_old = this->clr_field(i, j, k);
						if( clr_old[0] > 0.0 || clr_old[1] > 0.0 || clr_old[2] > 0.0)
							clr_field(i, j, k) = (clr_field(i, j, k)*weights(i, j, k) + clr_cur*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
						else
							clr_old = clr_cur;
					}
					weights(i, j, k) += weight_cur;
				}
			}
		}
	}

	return true;
}

bool TSDF::add_a_frame2(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam)
{
	vnl_matrix_fixed<double, 3, 3> K = cam.get_calibration().get_matrix();
	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];
	int img_width = depthMat.cols();
	int img_height = depthMat.rows();

	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	vnl_matrix_fixed<double, 3, 3> Rt = R.transpose();

	CDepthMap depth_map;
	//estimate the normal with its 8-NNs
	local_plane_estimation_for_depthMap3(depthMat, cam, depth_map, 4, 3.0, 1, 7, false);

	double dist_thres = 2.0*mu;
	LOGGER()->trace("dist_to_surf=%f\n", dist_thres);
	
	VoxelMatrix<int> vtIdxVol_n(nx, ny, nz, -1, -1); //index of the nearest vertex for each voxel
	vtIdxVol_n.fill(-1);
	VoxelMatrix<float> distsVol_n(nx, ny, nz, -1.0, -1.0); //distance to nearest vertex for each voxel
	distsVol_n.fill(1.0e+20);

	//find the closed point on the surface for a voxel(NOT every voxel)
	for(int row=0; row<img_height; row++)
	{
		for(int col=0; col<img_width; col++)
		{
			DepthElement &ele = depth_map[row][col];
			if( !depth_map[row][col].bNormal )
				continue;
			vnl_vector_fixed<double, 3> &vt = depth_map[row][col].P_cam;
			int x1 = MAX(0, (int)((vt[0]-dist_thres-xoffset)/xres));
			int x2 = MIN(nx-1, ROUND((vt[0]+dist_thres-xoffset)/xres));
			int y1 = MAX(0, (int)((vt[1]-dist_thres-yoffset)/yres));
			int y2 = MIN(ny-1, ROUND((vt[1]+dist_thres-yoffset)/yres));
			int z1 = MAX(0, (int)((vt[2]-dist_thres-zoffset)/zres));
			int z2 = MIN(nz-1, ROUND((vt[2]+dist_thres-zoffset)/zres));
#pragma omp parallel for 
			for(int k=z1; k<=z2; k++)
			{
				double z = k*zres + zoffset;
				for(int j=y1; j<=y2; j++)
				{
					double y = j*yres + yoffset;			
					for(int i=x1; i<=x2; i++)
					{
						double x = i*xres + xoffset;

						double dist_2 = (x-vt[0])*(x-vt[0]) + (y-vt[1])*(y-vt[1]) + (z-vt[2])*(z-vt[2]);
						if( dist_2 < distsVol_n(i, j, k) )
						{
							distsVol_n(i, j, k) = dist_2;
							vtIdxVol_n(i, j, k) = row*img_width+col;
						}
					}
				}
			}//end of for-k
		}
	}

	int xdir[8]={-1,0,0,1,1,1,-1,-1};
	int ydir[8]={0,-1,1,0,1,-1,-1,1};
	//find the boundary points of the surface
	vnl_matrix<bool> bBoundary(img_height, img_width);
	bBoundary.fill(false);
	bBoundary.set_column(0, true);
	bBoundary.set_column(img_width-1, true);
	bBoundary.set_row(0, true);
	bBoundary.set_row(img_height-1, true);
	for(int i=1; i<img_height-1; i++)
	{
		for(int j=1; j<img_width-1; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0) 
				continue;
			for(int k=0; k<8; k++)
			{
				int y = i-ydir[k];
				int x = j-xdir[k];
				double z_ = depthMat[y][x];
				if(z_<=0 || abs(z-z_) > 3.0)
				{
					bBoundary[i][j] = true;
					break;
				}
			}
		}
	}

	//assign the sdf
#pragma omp parallel for schedule(dynamic)
	for(int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for(int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;			
			for(int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;	
				int vtIdx = vtIdxVol_n(i, j, k);
				if( vtIdx == -1 )
					continue;

				int vtIdx_row = vtIdx/img_width;
				int vtIdx_col = vtIdx % img_width;
				if( bBoundary[vtIdx_row][vtIdx_col] )
					continue;

				//check the sign
				vnl_vector_fixed<double, 3> &normal = depth_map[vtIdx_row][vtIdx_col].normal;
				vnl_vector_fixed<double, 3> &Pt = depth_map[vtIdx_row][vtIdx_col].P_cam;
				vnl_vector_fixed<double, 3> Vxl(x, y, z);
				vnl_vector_fixed<double, 3> drt = Pt - Vxl;
				double sdf = sqrt(distsVol_n(i, j, k));
				if( dot_product(normal, drt) < 0)
				{
					sdf = -sdf;
				}
				//correct distance for nearby voxels
				if( abs(sdf) < 1.0 )
					sdf = dot_product(normal, drt);

				double mu_cur = 0;
				double ds = depthMat[vtIdx_row][vtIdx_col];
				if( bDynamicMu )
					mu_cur = MAX(this->mu, 0.5*ds*ds/(5.0*600+0.5*ds)); 
				else
					mu_cur = this->mu;

				if( sdf < -mu_cur)
					continue;
				
				float val_cur = MIN(1.0, sdf/mu_cur);
				float weight_cur = 1.0;
				if( this->bDynamicWeight )
					weight_cur = 150.0/ds;
				if( IsNull(this->func_val(i, j, k)))
				{
					this->func_val(i, j, k) = val_cur;					
					this->weights(i, j, k) = weight_cur;
				}
				else
				{
					func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
					weights(i, j, k) += weight_cur;
				}
			}//for-i
		}//for-j
	}//for-k
	return true;
}

bool TSDF::add_a_frame( vnl_matrix<double> const&depthMat, vpgl_perspective_camera<double> const&cam_ori, 
					    vector<vpgl_perspective_camera<double> > const&cams_aux)
{
	int cams_num = cams_aux.size() + 1;
	vnl_matrix_fixed<double, 3, 3> K = cam_ori.get_calibration().get_matrix();
	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];
	int img_width = depthMat.cols();
	int img_height = depthMat.rows();

	CSurface<float> surface;
	DepthMap2Surface(depthMat, cam_ori, cv::Mat(), surface, 4.0);

	vector< vnl_matrix_fixed<double, 3, 3> > Rs;
	vector< vnl_vector_fixed<double, 3> > Ts;
	vector< vnl_matrix<double> > depthMats(cams_num);
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam_ori, R, T);
	Rs.push_back(R);
	Ts.push_back(T);
	depthMats[0] = depthMat;
	for(int i=0; i<cams_aux.size(); i++)
	{
		get_camera_pose(cams_aux[i], R, T);
		Rs.push_back(R);
		Ts.push_back(T);

		ComputeDepthMap(surface, cams_aux[i], img_width, img_height, depthMats[i+1], 
						vnl_vector_fixed<double, 4>(0,0,0,0), true, false);

	}

	double mu_cur;
	for(unsigned int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for(unsigned int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;			
			for(unsigned int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;
				vnl_vector_fixed<double, 3> X_wld(x, y, z);
				double sdf = -2;
				bool bDefined = false;
				double ds_ori; //used to compute dynamic mu

				vnl_vector_fixed<double, 3> X_cam = Rs[0]*X_wld + Ts[0];
				int u = ROUND(fx*X_cam[0]/X_cam[2] + cx);
				int v = ROUND(fy*X_cam[1]/X_cam[2] + cy);
				//if the point is in front of the camera, 
				//and the point can be observed at the current camera pose
				if( X_cam[2] > 0 && 
					u >= 0 && u < img_width &&
					v >= 0 && v < img_height )
				{
					//if no depth measurement of the surface at this point
					double ds = depthMat[v][u];
					if( ds > 0 )
					{	
						ds_ori = ds;
						sdf = ds - X_cam[2];						
						bDefined = true;
					}
				}

				for(int c=1; c<cams_num; c++)
				{
					vnl_matrix_fixed<double, 3, 3> &Ri = Rs[c];
					vnl_vector_fixed<double, 3> &Ti = Ts[c];
					X_cam = Ri*X_wld + Ti;
					u = ROUND(fx*X_cam[0]/X_cam[2] + cx);
					v = ROUND(fy*X_cam[1]/X_cam[2] + cy);

					//if the point is behind the camera, then skip it
					if( X_cam[2] <=0 )
						continue;

					// if the point cannot be observed at the current camera pose
					if( u < 0 || u >= img_width ||
						v < 0 || v >= img_height )
						continue;

					//if no depth measurement of the surface at this point
					double ds = depthMats[i][v][u];
					if( ds <= 0 )
						continue;

					if( !bDefined )
					{
						ds_ori = ds;
						sdf = ds - X_cam[2];
						bDefined = true;
					}
					else
					{
						double sdf_new = ds - X_cam[2];
						if( sdf_new >=0 && sdf >= 0)
							sdf = MIN(sdf_new, sdf);
						else if( sdf_new <0 && sdf < 0)
							sdf = MAX(sdf_new, sdf);
						else if( sdf_new > 0 && sdf < 0)
							sdf = sdf_new;						
					}					
				}
				if( !bDefined )
					continue;

				if( bDynamicMu )
					mu_cur = MAX(this->mu, 0.5*ds_ori*ds_ori/(5.0*600+0.5*ds_ori));
				else
					mu_cur = this->mu;
					
				
				// if the point is far behind the observed surface
				if( -sdf > mu_cur )
					continue;

				// if the function value at current point is NULL
				if( IsNull( func_val(i, j, k) ) )
				{
					func_val(i, j, k) = MIN(1.0, sdf/mu_cur);
					if( this->bDynamicWeight )
						weights(i, j, k) = 150.0/ds_ori;
					else
						weights(i, j, k) = 1.0;
				}
				else
				{
					double weight_cur = 1.0;
					if( this->bDynamicWeight )
						weight_cur = 150.0/ds_ori;

					double val_cur = MIN(1.0, sdf/mu_cur);
					func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
					weights(i, j, k) += weight_cur;
				}
			}
		}
	}

	return true;
}

bool TSDF::generate_surface(CSurface<float> &m_surface, double iso_level,
							VoxelMatrix<float> const& volume, 
							float x_res, float y_res, float z_res,
							float x_offset, float y_offset, float z_offset,
							bool bVerbose)
{
	if (bVerbose)	LOGGER()->trace("Marching cube...");
	CIsoSurface<float> *isoSurface = new CIsoSurface<float>();
	isoSurface->GenerateSurface(volume.data_block(), iso_level,
								volume.ni() - 1, volume.nj() - 1, volume.nk() - 1,
								x_res, y_res, z_res,
								NULL);

	m_surface.readFromIsoSurface(isoSurface);

	//add offset
	int vtDim = m_surface.vtDim;
	for (int i = 0; i<m_surface.vtNum; i++)
	{
		m_surface.vtData[i*vtDim] += x_offset;
		m_surface.vtData[i*vtDim + 1] += y_offset;
		m_surface.vtData[i*vtDim + 2] += z_offset;
	}
	if (bVerbose) LOGGER()->trace("<vtNum=%d, triNum=%d>end!", m_surface.vtNum, m_surface.triNum);
	delete isoSurface;
	return true;
}

bool TSDF::generate_surface(CSurface<float> &m_surface, double iso_level, bool bGenerateTexture, bool bVerbose)
{
	if(bVerbose)	LOGGER()->trace("Marching cube...");
	CIsoSurface<float> *isoSurface = new CIsoSurface<float>();
	if( bGenerateTexture && this->bBuildColorField )
		isoSurface->GenerateSurface( this->func_val.data_block(),
									 this->clr_field(0, 0, 0).data_block(),
									 iso_level, 
									 nx-1, ny-1, nz-1,
									 xres, yres, zres,
									 (TSDF::IsNull_MarchingCubes) );
	else
		isoSurface->GenerateSurface( this->func_val.data_block(), iso_level, 
									 nx-1, ny-1, nz-1,
									 xres, yres, zres,
									 (TSDF::IsNull_MarchingCubes) );
	m_surface.readFromIsoSurface(isoSurface);

	//add offset
	int vtDim = m_surface.vtDim;
	for(int i=0; i<m_surface.vtNum; i++)
	{
		m_surface.vtData[i*vtDim] += xoffset;
		m_surface.vtData[i*vtDim+1] += yoffset;
		m_surface.vtData[i*vtDim+2] += zoffset;
	}
	if(bVerbose) LOGGER()->trace("<vtNum=%d, triNum=%d>end!", m_surface.vtNum, m_surface.triNum);
	delete isoSurface;
	//m_surface.delete_extra_vertice();
	return true;
}
bool TSDF::generate_surface_occupancy(CSurface<float> &m_surface, double iso_level, bool bGenerateTexture, bool bVerbose)
{
	if (bVerbose)	LOGGER()->trace("Marching cube...");
	CIsoSurface<float> *isoSurface = new CIsoSurface<float>();
	if (bGenerateTexture && this->bBuildColorField)
		isoSurface->GenerateSurface(this->func_val.data_block(),
		this->clr_field(0, 0, 0).data_block(),
		iso_level,
		nx - 1, ny - 1, nz - 1,
		xres, yres, zres,
		(TSDF::IsNull_MarchingCubes_Occupancy));
	else
		isoSurface->GenerateSurface(this->func_val.data_block(), iso_level,
		nx - 1, ny - 1, nz - 1,
		xres, yres, zres,
		(TSDF::IsNull_MarchingCubes_Occupancy));
	m_surface.readFromIsoSurface(isoSurface);

	//add offset && reverse normal
	int vtDim = m_surface.vtDim;
	for (int i = 0; i<m_surface.vtNum; i++)
	{
		m_surface.vtData[i*vtDim] += xoffset;
		m_surface.vtData[i*vtDim + 1] += yoffset;
		m_surface.vtData[i*vtDim + 2] += zoffset;
		float *n = m_surface.vt_normal(i);
		n[0] *= -1;
		n[1] *= -1;
		n[2] *= -1;
	}
	if (bVerbose) LOGGER()->trace("<vtNum=%d, triNum=%d>end!", m_surface.vtNum, m_surface.triNum);
	delete isoSurface;
	m_surface.delete_extra_vertice();
	return true;
}

void TSDF::get_error_map_on_surface(CSurface<float> &surface, bool bNormalCorrected) const
{
	surface.expand_data(true, false);
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		float *clr = surface.vt_color(vtIdx);
		float *v_ = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> vt(v_[0], v_[1], v_[2]);
		double residual = 0.0;
		if (bNormalCorrected && surface.haveNormalInfo())
		{
			vnl_vector_fixed<double, 3> df_dvtt;
			vnl_vector_fixed<double, 3> df_dnt;
			float *n_ = surface.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n(n_[0], n_[1], n_[2]);
			this->val_der_sigmoid(vt.data_block(), n.data_block(), residual, df_dvtt, df_dnt);
		}
		else
		{
			residual = this->val_at_awf(vt[0], vt[1], vt[2]);
		}

		int idx = MAX(0, MIN(255, ROUND(residual*this->mu* 255.0/2.0)));
		clr[0] = CLRMAP_JET[idx][0];
		clr[1] = CLRMAP_JET[idx][1];
		clr[2] = CLRMAP_JET[idx][2];
	}
}

bool TSDF::texture_surface(CSurface<float> &m_surface) const
{
	if( !this->bBuildColorField )
	{
		LOGGER()->error("TSDF::texture_surface", "No Color Field!");
		return false;
	}
	m_surface.expand_data(true, false);

	for(int vtIdx=0; vtIdx<m_surface.vtNum; vtIdx++)
	{
		float const* vt = m_surface.vt_data_block(vtIdx);
		float *c = m_surface.vt_color(vtIdx);
		vnl_vector_fixed<float, 3> clr = this->clr_at(vt[0], vt[1], vt[2]);
		if( clr[0] !=0 || clr[1] != 0 || clr[2] != 0)
		{
			c[0] = clr[0];
			c[1] = clr[1];
			c[2] = clr[2];
		}
	}

	return true;
}

bool TSDF::save_data_to_file(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "wb");
	if(!fp)
	{
		LOGGER()->error("TSDF::save_data_to_file","Cannot open the file <%s> for saving TSDF.", filename);
		return false;
	}

	int count = fwrite(&(this->nx), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nx to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->ny), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write ny to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->nz), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Error: cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->mu), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(this->func_val.data_block(), sizeof(float), this->nx*this->ny*this->nz, fp);
	if( count != this->nx*this->ny*this->nz ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "When writing func_val!");
		fclose(fp);
		return false;
	}

	count = fwrite(this->weights.data_block(), sizeof(float), this->nx*this->ny*this->nz, fp);
	if( count != this->nx*this->ny*this->nz ) 
	{
		LOGGER()->error("TSDF::save_data_to_file", "When writing weights!");
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}


bool TSDF::load_data_from_file(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "rb");
	if(!fp)
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot open the file <%s> for loading TSDF.", filename);
		return false;
	}

	int nx_ = 0;
	int ny_ = 0;
	int nz_ = 0;
	double xres_ = 0;
	double yres_ = 0;
	double zres_ = 0;
	double xoffset_ = 0;
	double yoffset_ = 0;
	double zoffset_ = 0;
	double mu_;

	int count = fread(&nx_, sizeof(int), 1, fp);
	if( count != 1 || nx_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read nx from file!");
		fclose(fp);
		return false;
	}

	count = fread(&ny_, sizeof(int), 1, fp);
	if( count != 1 || ny_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read ny to file!");
		fclose(fp);
		return false;
	}

	count = fread(&nz_, sizeof(int), 1, fp);
	if( count != 1 || nz_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read nz to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xres_, sizeof(double), 1, fp);
	if( count != 1 || xres_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read xres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yres_, sizeof(double), 1, fp);
	if( count != 1 || yres_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read yres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zres_, sizeof(double), 1, fp);
	if( count != 1 || zres_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read zres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read xoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read yoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read zoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&mu_, sizeof(double), 1, fp);
	if( count != 1 || mu_ <= 0) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "Cannot read mu_ to file!");
		fclose(fp);
		return false;
	}

	this->func_val.set_size(nx_, ny_, nz_);
	this->weights.set_size(nx_, ny_, nz_);

	this->nx = nx_; this->ny = ny_; this->nz = nz_;
	this->xres = xres_; this->yres = yres_; this->zres = zres_;
	this->xoffset = xoffset_; this->yoffset = yoffset_; this->zoffset = zoffset_;
	this->mu = mu_;

	count = fread(this->func_val.data_block(), sizeof(float), nx_*ny_*nz_, fp);
	if( count != nx_*ny_*nz_ ) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "When writing func_val!");
		fclose(fp);
		return false;
	}

	count = fread(this->weights.data_block(), sizeof(float), nx_*ny_*nz_, fp);
	if( count != nx_*ny_*nz_ ) 
	{
		LOGGER()->error("TSDF::load_data_from_file", "When writing weights!");
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}

void 
TSDF::vals_at(vector< vnl_vector_fixed<double, 3> > const& points, vnl_vector<double> &vals)
{
	vals.set_size(points.size());
	for(int i=0; i<points.size(); i++)
	{
		vals[i] = this->val_at(points[i][0], points[i][1], points[i][2]);
	}
}

void 
TSDF::ders_at(vector< vnl_vector_fixed<double, 3> > const& points, vector< vnl_vector_fixed<double, 3> > &ders)
{
	ders.clear();
	for(int i=0; i<points.size(); i++)
	{
		ders.push_back( this->der_at(points[i][0], points[i][1], points[i][2]));
	}
}
