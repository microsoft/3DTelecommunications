// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "DDF.h"

bool DDF::add_a_frame( vnl_matrix<double> const&depthMat, vpgl_perspective_camera<double> const&cam,
					   cv::Mat const& img, vpgl_perspective_camera<double> const& cam_clr,
				       bool bDecay, float a)
{
	printf(".");
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);
	double fx = K[0][0];
	double fy = K[1][1];
	double cx = K[0][2];
	double cy = K[1][2];
	int depth_width = depthMat.cols();
	int depth_height = depthMat.rows();
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> Kc;
	get_calibration_matrix(cam_clr, Kc);
	double fx_c = Kc[0][0];
	double fy_c = Kc[1][1];
	double cx_c = Kc[0][2];
	double cy_c = Kc[1][2];
	vnl_matrix_fixed<double, 3, 3> Rc;
	vnl_vector_fixed<double, 3> Tc;
	get_camera_pose(cam_clr, Rc, Tc);
	int img_width = 0;
	int img_height = 0;
	if (!img.empty())
	{
		img_width = img.cols;
		img_height = img.rows;
	}	

	//estimate the normal with its 8-NNs
	CDepthMap depth_map;
	local_plane_estimation_for_depthMap3(depthMat, cam, depth_map, 3, 3.0, 1, 7, false);
	
	vnl_vector_fixed<double, 3> cam_cen;
	get_camera_center(cam, cam_cen);

	bool bColorDepthAligned = false;
	if (cam == cam_clr)
		bColorDepthAligned = true;

	__tic__();
	double mu_cur;
	for(int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
#pragma omp parallel for
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
				if( u < 0 || u >= depth_width ||
					v < 0 || v >= depth_height )
					continue;

				//if no depth measurement of the surface at this point
				double ds = depthMat[v][u];
				if( ds <= 0 )
					continue;

				if( bDynamicMu )
					mu_cur = MAX(this->mu, 0.5*ds*ds/(7.5*520+0.5*ds)); 
				else
					mu_cur = this->mu;					
				
				//correct the bias caused by incorrect nearest vertex
				DepthElement &ele = depth_map[v][u];
				vnl_vector_fixed<double, 3> &Vt = ele.P_cam; // the surface point in world space
				vnl_vector_fixed<double, 3> Pt(x, y, z); //the voxel point
				vnl_vector_fixed<double, 3> drt = Vt-Pt;
				double Mr = Vt.two_norm();
				double dxr = Pt.two_norm();
				
				double sdf_val = ds - X_cam[2];

				// if the point is far behind the observed surface
				if( sdf_val < -mu_cur )
					continue;

				//if the point is far away from the observed surface 
				if (sdf_val > mu_cur)
					continue;

				double weight_cur = 1.0;
				if (this->bDynamicWeight)
				{
					weight_cur = 150.0/ds;
					if (ele.bNormal)
					{
						vnl_vector_fixed<double, 3> ray = Vt - cam_cen;
						ray.normalize();
						double angle = dot_product(ray, ele.normal);

						weight_cur *= MAX(angle*angle*angle, 0.0);						
					}
					else
					{
						weight_cur = 0.0;
					}
				}

				double val_cur = MIN(1.0, sdf_val/mu_cur);//*this->mu;
				vnl_vector_fixed<double, 3> drt_cur = (sdf_val>0)?drt:-drt;				
				if( ele.bNormal )
					drt_cur = ele.normal;
				else
					drt_cur.normalize();
				vnl_vector_fixed<float, 3> drt_cur_f(drt_cur[0], drt_cur[1], drt_cur[2]);

				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if( !img.empty() && this->bBuildColorField)
				{
					if (bColorDepthAligned)
					{
						clr_cur[0] = img.at<cv::Vec3b>(v, u)[2] / 255.0;
						clr_cur[1] = img.at<cv::Vec3b>(v, u)[1] / 255.0;
						clr_cur[2] = img.at<cv::Vec3b>(v, u)[0] / 255.0;
					}
					else
					{
						vnl_vector_fixed<double, 3> X_clr = Rc*X_wld + Tc;
						double u_c = fx_c*X_clr[0] / X_clr[2] + cx_c;
						double v_c = fy_c*X_clr[1] / X_clr[2] + cy_c;
						float clr_tmp[3];
						pickAColorPixel(img, u_c, v_c, clr_tmp);
						clr_cur[0] = clr_tmp[2] / 255.0;
						clr_cur[1] = clr_tmp[1] / 255.0;
						clr_cur[2] = clr_tmp[0] / 255.0;
					}
				}

				// if the function value at current point is NULL
				if (IsNull(func_val(i, j, k)))
				{
					this->func_val(i, j, k) = val_cur;
					this->weights(i, j, k) = weight_cur;
					this->drts(i, j, k) = drt_cur_f;
					if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )					
						this->clr_field(i, j, k) = clr_cur;
				}
				else
				{
					if( !bDecay )
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k) + drt_cur_f*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
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
					else
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k)*a + val_cur*weight_cur*(1.0-a))/(weights(i, j, k)*a + weight_cur*(1.0-a));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k)*a + drt_cur_f*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
						if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )
						{
							vnl_vector_fixed<float, 3> &clr_old = this->clr_field(i, j, k);
							if( clr_old[0] > 0.0 || clr_old[1] > 0.0 || clr_old[2] > 0.0)
								clr_field(i, j, k) = (clr_field(i, j, k)*weights(i, j, k)*a + clr_cur*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
							else
								clr_old = clr_cur;
						}
						weights(i, j, k) = weights(i, j, k)*a + weight_cur*(1.0-a);
					}
				}
			}
		}
	}

	return true;
}

bool DDF::add_a_frame2(vnl_matrix<double> &depthMat, vpgl_perspective_camera<double> &cam, cv::Mat const& img)
{
	printf(".");
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
	local_plane_estimation_for_depthMap3(depthMat, cam, depth_map, 5, 5.0, 1, 7, false); 

	printf("mu=%f\n", this->mu);
	
	VoxelMatrix<int> vtIdxVol_n(nx, ny, nz, -1, -1); //index of the nearest vertex for each voxel
	vtIdxVol_n.fill(-1);
	VoxelMatrix<float> distsVol_n(nx, ny, nz, -1, -1); //distance to nearest vertex for each voxel
	distsVol_n.fill(1.0e+20);

	//find the closed point on the surface for a voxel(NOT every voxel)
	for(int row=0; row<img_height; row++)
	{
		for(int col=0; col<img_width; col++)
		{
			DepthElement &ele = depth_map[row][col];
			if( !depth_map[row][col].bNormal )
				continue;

			double ds = depthMat[row][col];
			if( ds <= 0.0 )
				continue;

			double mu_cur = this->mu;			
			if(this->bDynamicMu)
				mu_cur = MAX(this->mu, 0.5*ds*ds/(7.5*520+0.5*ds)); 
			double dist_thres = MIN(20.0, MAX(3.0, 2.0*mu_cur));
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
				if(z_<=0 || abs(z-z_) > MIN(z/fx*6.0, 3.0) )
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
				DepthElement &ele = depth_map[vtIdx_row][vtIdx_col];
				if( !ele.bNormal )
					continue;

				vnl_vector_fixed<double, 3> &normal = ele.normal;
				vnl_vector_fixed<double, 3> &Pt = ele.P_cam;
				vnl_vector_fixed<double, 3> Vxl(x, y, z);
				vnl_vector_fixed<double, 3> drt = Pt - Vxl;
				double sdf = sqrt(distsVol_n(i, j, k));
				if( dot_product(normal, drt) < 0)
				{
					sdf = -sdf;
				}
				vnl_vector_fixed<double, 3> drt_cur = (sdf>0)?drt:-drt; //drt pointing inwards	
				drt_cur.normalize();
				//correct distance for nearby voxels
				if( abs(sdf) < 2.0 )
				{
					sdf = dot_product(normal, drt);
				}
				//corret drt for nearby voxels
				if( abs(sdf) < 1.5)
				{
					drt_cur = ele.normal;
				}

				double mu_cur = 0;
				double ds = depthMat[vtIdx_row][vtIdx_col];
				if( bDynamicMu )
					mu_cur = MIN(20.0, MAX(this->mu, 0.5*ds*ds/(7.5*520+0.5*ds))); 
				else
					mu_cur = this->mu;

				// if the point is far behind the observed surface
				if( sdf < -mu_cur)
					continue;
				
				float val_cur = MIN(1.0, sdf/mu_cur);
				float weight_cur = 1.0;
				if( this->bDynamicWeight )
					weight_cur = 150.0/ds;
				
				//get color
				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if( !img.empty() && this->bBuildColorField )
				{
					vnl_vector_fixed<double, 3> X_wld(x, y, z);
					vnl_vector_fixed<double, 3> X_cam = R*X_wld + T;
					int u = ROUND(fx*X_cam[0]/X_cam[2] + cx);
					int v = ROUND(fy*X_cam[1]/X_cam[2] + cy);
					//if the point is behind the camera, then skip it
					if( X_cam[2] > 0 &&
						u >= 0 && u < img_width &&
						v >= 0 && v < img_height )
					{
						clr_cur[0] = img.at<cv::Vec3b>(v, u)[2]/255.0;
						clr_cur[1] = img.at<cv::Vec3b>(v, u)[1]/255.0;
						clr_cur[2] = img.at<cv::Vec3b>(v, u)[0]/255.0;
					}
				}

				vnl_vector_fixed<float, 3> drt_cur_f(drt_cur[0], drt_cur[1], drt_cur[2]);
				if (this->func_val(i, j, k) == SDF_NULL_VALUE)
				{
					this->func_val(i, j, k) = val_cur;					
					this->weights(i, j, k) = weight_cur;
					this->drts(i, j, k) = drt_cur_f;
					if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )
						this->clr_field(i, j, k) = clr_cur;
				}
				else
				{
					func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
					drts(i, j, k) = (drts(i, j, k)*weights(i, j, k) + drt_cur_f*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
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

			}//for-i
		}//for-j
	}//for-k

	return true;
}

bool DDF::add_a_frame( CSurface<float> &surface, bool bDecay, float a)
{
	if( !surface.haveNormalInfo() )
	{
		LOGGER()->error("DDF::add_a_frame(CSurface<T>&)>", "No normal included in surface!");
		return false;
	}

	double dist_thres = 1.2*mu;
	LOGGER()->trace("dist_to_surf=%f\n", dist_thres);
	
	VoxelMatrix<int> vtIdxVol_n(nx, ny, nz, -1, -1); //index of the nearest vertex for each voxel
	vtIdxVol_n.fill(-1);
	VoxelMatrix<float> distsVol_n(nx, ny, nz, -1.0, -1.0); //distance to nearest vertex for each voxel
	distsVol_n.fill(1.0e+20);

	//find the closed point on the surface for a voxel(NOT every voxel)
	__tic__();	
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		float* vt = surface.vt_data_block(vtIdx);
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
						vtIdxVol_n(i, j, k) = vtIdx;
					}
				}
			}
		}//end of for-k

	}
	LOGGER()->trace("closest surf pt.=%f\n", __toc__());

	//find the boundary points of the surface
	vnl_vector<bool> bBoundary(surface.vtNum);
	bBoundary.fill(false);
	surface.extract_boundary();
	for(int i=0; i<surface.boundary_vts.size(); i++)
	{
		int vtIdx = surface.boundary_vts[i];
		bBoundary[vtIdx] = true;
	}

	//assign the sdf
	__tic__();
	for(int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
#pragma omp parallel for schedule(dynamic)
		for(int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;			
			for(int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;	
				int vtIdx = vtIdxVol_n(i, j, k);
				if( vtIdx == -1 )
					continue;

				if( bBoundary[vtIdx] )
					continue;

				float* normal_ = surface.vt_normal(vtIdx);
				float* vt = surface.vt_data_block(vtIdx);
				if( (vt[0]==0 && vt[1]==0 && vt[2]==0) ||
					(normal_[0]==0 && normal_[1]==0 && normal_[2]==0 ) )
					continue;

				vnl_vector_fixed<double, 3> normal(normal_[0], normal_[1], normal_[2]);
				vnl_vector_fixed<double, 3> Pt(vt[0], vt[1], vt[2]);
				vnl_vector_fixed<double, 3> Vxl(x, y, z);
				vnl_vector_fixed<double, 3> drt = Pt - Vxl;
				double sdf = sqrt(distsVol_n(i, j, k));
				if( dot_product(normal, drt) < 0)
				{
					sdf = -sdf;
				}
				vnl_vector_fixed<double, 3> drt_cur = (sdf>0)?drt:-drt; //drt pointing inwards	
				drt_cur.normalize();

				//correct distance for nearby voxels
				if( fabs(sdf) < 2.0 )
				{
					sdf = dot_product(normal, drt);
				}
				//corret drt for nearby voxels
				if( fabs(sdf) < 1.5)
				{
					drt_cur = normal;
				}

				double mu_cur = this->mu;

				// if the point is far behind the observed surface
				if( std::abs(sdf) > mu_cur)
					continue;
				
				float val_cur = MAX(-1.0, MIN(1.0, sdf/mu_cur));
				float weight_cur = 1.0;

				bool bClrFound = false;
				float *clr = surface.vt_color(vtIdx);
				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if( bBuildColorField &&
					surface.haveColorInfo() && 
					(clr[0] > 0 || clr[1] > 0 || clr[2] > 0) )
				{
					clr_cur.set(clr);
					bClrFound = true;
				}

				vnl_vector_fixed<float, 3> drt_cur_f(drt_cur[0], drt_cur[1], drt_cur[2]);
				if (this->func_val(i, j, k) == SDF_NULL_VALUE)
				{
					this->func_val(i, j, k) = val_cur;					
					this->weights(i, j, k) = weight_cur;
					this->drts(i, j, k) = drt_cur_f;
					if( bClrFound )
						this->clr_field(i, j, k) = clr_cur;
				}
				else
				{
					if( !bDecay )
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k) + drt_cur_f*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
						if( bClrFound )
						{
							vnl_vector_fixed<float, 3> clr_old = this->clr_field(i, j, k);
							if( clr_old[0] > 0.0 || clr_old[1] > 0.0 || clr_old[2] > 0.0)
								clr_field(i, j, k) = (clr_field(i, j, k)*weights(i, j, k) + clr_cur*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
							else
								clr_old = clr_cur;
						}
						weights(i, j, k) += weight_cur;
					}
					else
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k)*a + val_cur*weight_cur*(1.0-a))/(weights(i, j, k)*a + weight_cur*(1.0-a));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k)*a + drt_cur_f*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
						if( bClrFound )
						{
							vnl_vector_fixed<float, 3> clr_old = this->clr_field(i, j, k);
							if( clr_old[0] > 0.0 || clr_old[1] > 0.0 || clr_old[2] > 0.0)
								clr_field(i, j, k) = (clr_field(i, j, k)*weights(i, j, k)*a + clr_cur*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
							else
								clr_old = clr_cur;
						}
						weights(i, j, k) = weights(i, j, k)*a + weight_cur*(1.0-a);
					}
				}
			}//for-i
		}//for-j
	}//for-k
	LOGGER()->trace("calc sdf=%f\n", __toc__());	
	return true;
}

bool DDF::add_a_frame(DDF const& ddf, bool bDecay, float a, bool bWeightUseDrt, vnl_vector_fixed<float, 3> const& avg_drt)
{
	const double m_eps = 1.0e-5;
	if(  !( ddf.nx == this->nx && ddf.ny == this->ny && ddf.nz == this->nz &&
			std::abs(ddf.xres - this->xres) < m_eps && 
			std::abs(ddf.yres - this->yres) < m_eps && 
			std::abs(ddf.zres - this->zres) < m_eps &&
			std::abs(ddf.xoffset - this->xoffset) < m_eps && 
			std::abs(ddf.yoffset - this->yoffset) < m_eps && 
			std::abs(ddf.zoffset - this->zoffset) < m_eps ) )
	{
		LOGGER()->error("DDF::add_a_frame","ddfs do not have same size!");
		ddf.printDim("DDF1:");
		this->printDim("DDF2:");
		return false;
	}

	for(int k=0; k<nz; k++)
	{
#pragma omp parallel for schedule(dynamic)
		for(int j=0; j<ny; j++)
		{			
			for(unsigned int i=0; i<nx; i++)
			{
				if( ddf.weights(i, j, k) == 0 || ddf.func_val(i, j, k) == SDF_NULL_VALUE)
					continue;
				
				vnl_vector_fixed<float, 3> const& drt_cur = ddf.drts(i, j, k);
				vnl_vector_fixed<float, 3> clr_cur(0.0);
				if( this->bBuildColorField && ddf.bBuildColorField )
					clr_cur = ddf.clr_field(i, j, k);
				float val_cur = ddf.func_val(i, j, k);
				float weight_cur = ddf.weights(i, j, k);
				if (bWeightUseDrt)
				{
					double angle = angle_btw_two_vec(drt_cur, avg_drt);
					weight_cur *= MAX(0.01, std::cos(angle));
				}

				if( this->func_val(i, j, k) == SDF_NULL_VALUE)
				{
					this->func_val(i, j, k) = val_cur;					
					this->weights(i, j, k) = weight_cur;
					this->drts(i, j, k) = drt_cur;
					if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )
						this->clr_field(i, j, k) = clr_cur;
				}
				else
				{
					if( !bDecay )
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k) + val_cur*weight_cur)/(weight_cur + weights(i, j, k));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k) + drt_cur*(float)weight_cur)/(float)(weight_cur + weights(i, j, k));
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
					else
					{
						func_val(i, j, k) = (func_val(i, j, k)*weights(i, j, k)*a + val_cur*weight_cur*(1.0-a))/(weights(i, j, k)*a + weight_cur*(1.0-a));
						drts(i, j, k) = (drts(i, j, k)*weights(i, j, k)*a + drt_cur*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
						if( clr_cur[0] > 0.0 || clr_cur[1] > 0.0 || clr_cur[2] > 0.0 )
						{
							vnl_vector_fixed<float, 3> clr_old = this->clr_field(i, j, k);
							if( clr_old[0] > 0.0 || clr_old[1] > 0.0 || clr_old[2] > 0.0)
								clr_field(i, j, k) = (clr_field(i, j, k)*weights(i, j, k)*a + clr_cur*(float)(weight_cur*(1.0-a)))/(float)(weights(i, j, k)*a + weight_cur*(1.0-a));
							else
								clr_old = clr_cur;
						}
						weights(i, j, k) = weights(i, j, k)*a + weight_cur*(1.0-a);
					}
				}
			}
		}
	}
	return true;
}

bool DDF::texture_surface(CSurface<float> &surface) const
{
	if( !surface.haveNormalInfo() )
	{
		printf("Warning<DDF::texture_surface>: input surface has no normal info!\n");
		return TSDF::texture_surface(surface);
	}

	if( !this->bBuildColorField )
	{
		printf("Error<DDF::texture_surface>: No Color Field!\n");
		return false;
	}

	surface.expand_data(true, false);

	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		float const* vt = surface.vt_data_block(vtIdx);
		float const* n = surface.vt_normal(vtIdx);
		vnl_vector_fixed<double, 3> normal(n[0], n[1], n[2]);
		vnl_vector_fixed<double, 3> der = this->der_at(vt[0], vt[1], vt[2]);
		double alpha = angle_btw_two_vec(der, normal) * 180 / M_PI;
		if( alpha > 180 - 60 )
		{
			float *c = surface.vt_color(vtIdx);
			vnl_vector_fixed<float, 3> clr = this->clr_at(vt[0], vt[1], vt[2]);
			if( clr[0] !=0 || clr[1] != 0 || clr[2] != 0)
			{
				c[0] = clr[0];
				c[1] = clr[1];
				c[2] = clr[2];
			}
		}
	}
	return true;
}

vnl_vector_fixed<float, 3> DDF::get_avg_drt() const
{
	vnl_vector_fixed<float, 3> drt(0.0);
	int count = 0;
	for (int k = 0; k < nz; k++)
	{
		for (unsigned int j = 0; j < ny; j++)
		{
			for (unsigned int i = 0; i < nx; i++)
			{
				if (this->func_val(i, j, k) != SDF_NULL_VALUE)
				{
					drt += this->drts(i, j, k);
					count++;
				}
			}
		}
	}
	drt /= count;
	return drt;
}

bool DDF::save_data_to_file(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "wb");
	if(!fp)
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot open the file <%s> for saving TSDF.", filename);
		return false;
	}

	int count = fwrite(&(this->nx), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nx to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->ny), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write ny to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->nz), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->mu), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(this->func_val.data_block(), sizeof(float), this->nx*this->ny*this->nz, fp);
	if( count != this->nx*this->ny*this->nz ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "When writing func_val!");
		fclose(fp);
		return false;
	}

	count = fwrite(this->weights.data_block(), sizeof(float), this->nx*this->ny*this->nz, fp);
	if( count != this->nx*this->ny*this->nz ) 
	{
		LOGGER()->error("DDF::save_data_to_file", "When writing weights!");
		fclose(fp);
		return false;
	}

	//save directions
	for(int k=0; k<nz; k++)
	for(int j=0; j<ny; j++)
	for(int i=0; i<nx; i++)
	{
		count = fwrite(this->drts(i, j, k).data_block(), sizeof(float), 3, fp);
		if( count != 3 ) 
		{
			LOGGER()->error("DDF::save_data_to_file", "When writing drts!");
			fclose(fp);
			return false;
		}
	}

	fclose(fp);
	return true;
}

bool DDF::save_data_to_file_zip(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "wb");
	if(!fp)
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot open the file <% s> for saving TSDF.", filename);
		return false;
	}

	int count = fwrite(&(this->nx), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nx to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->ny), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write ny to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->nz), sizeof(int), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zres), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->xoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->yoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->zoffset), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	count = fwrite(&(this->mu), sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "Cannot write nz to file!");
		fclose(fp);
		return false;
	}

	//save sdf value
	unsigned char* outbuf = NULL;
	unsigned long outlen = 0;
	int level = 3;
	zlib_compress((unsigned char*)this->func_val.data_block(), this->nx*this->ny*this->nz*sizeof(float), outbuf, outlen, level);
	fwrite(&outlen, sizeof(unsigned long), 1, fp);
	count = fwrite(outbuf, 1, outlen, fp);
	delete [] outbuf;
	outbuf = NULL;
	if( count != outlen ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "When writing func_val!");
		fclose(fp);
		return false;
	}

	//save weights
	zlib_compress((unsigned char*)this->weights.data_block(), this->nx*this->ny*this->nz*sizeof(float), outbuf, outlen, level);
	fwrite(&outlen, sizeof(unsigned long), 1, fp);	
	count = fwrite(outbuf, 1, outlen, fp);
	delete [] outbuf;
	outbuf = NULL;
	if( count != outlen ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "When writing weights!");
		fclose(fp);
		return false;
	}

	//save directions
	zlib_compress((unsigned char*)this->drts.data_block(), this->nx*this->ny*this->nz*sizeof(vnl_vector_fixed<float, 3>), outbuf, outlen, level);
	fwrite(&outlen, sizeof(unsigned long), 1, fp);	
	count = fwrite(outbuf, 1, outlen, fp);
	delete [] outbuf;
	outbuf = NULL;
	if( count != outlen ) 
	{
		LOGGER()->error("DDF::save_data_to_file_zip", "When writing directions!");
		fclose(fp);
		return false;
	}

	//save color field
	if(bBuildColorField )
	{
		zlib_compress((unsigned char*)this->clr_field.data_block(), this->nx*this->ny*this->nz*sizeof(vnl_vector_fixed<float, 3>), outbuf, outlen, level);
		fwrite(&outlen, sizeof(unsigned long), 1, fp);	
		count = fwrite(outbuf, 1, outlen, fp);
		delete [] outbuf;
		outbuf = NULL;
		if( count != outlen ) 
		{
			LOGGER()->error("DDF::save_data_to_file_zip", "When writing color field!");
			fclose(fp);
			return false;
		}
	}
	fclose(fp);
	return true;
}


bool DDF::load_data_from_file_zip(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "rb");
	if(!fp)
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot open the file <%s>.", filename);
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
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read nx from file!");
		fclose(fp);
		return false;
	}

	count = fread(&ny_, sizeof(int), 1, fp);
	if( count != 1 || ny_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read ny to file!");
		fclose(fp);
		return false;
	}

	count = fread(&nz_, sizeof(int), 1, fp);
	if( count != 1 || nz_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read nz to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xres_, sizeof(double), 1, fp);
	if( count != 1 || xres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read xres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yres_, sizeof(double), 1, fp);
	if( count != 1 || yres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read yres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zres_, sizeof(double), 1, fp);
	if( count != 1 || zres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read zres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read xoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read yoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read zoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&mu_, sizeof(double), 1, fp);
	if( count != 1 || mu_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "Cannot read mu_ to file!");
		fclose(fp);
		return false;
	}

	this->func_val.set_size(nx_, ny_, nz_);
	this->weights.set_size(nx_, ny_, nz_);
	this->drts.set_size(nx_, ny_, nz_);

	this->nx = nx_; this->ny = ny_; this->nz = nz_;
	this->xres = xres_; this->yres = yres_; this->zres = zres_;
	this->xoffset = xoffset_; this->yoffset = yoffset_; this->zoffset = zoffset_;
	this->mu = mu_;

	//load sdf value
	unsigned char* zipbuf = NULL;
	unsigned long ziplen = 0;
	fread(&ziplen, sizeof(unsigned long), 1, fp);
	zipbuf = new unsigned char[ziplen];
	count = fread(zipbuf, 1, ziplen, fp);
	unsigned long outlen = nx_*ny_*nz_*sizeof(float);
	zlib_uncompress(zipbuf, ziplen, (unsigned char*)this->func_val.data_block(), outlen);
	delete [] zipbuf;
	zipbuf = NULL;
	if( count != ziplen || outlen != nx_*ny_*nz_*sizeof(float)) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "When reading func_val!");
		fclose(fp);
		return false;
	}

	//load weights
	fread(&ziplen, sizeof(unsigned long), 1, fp);
	zipbuf = new unsigned char[ziplen];
	count = fread(zipbuf, 1, ziplen, fp);
	outlen = nx_*ny_*nz_*sizeof(float);
	zlib_uncompress(zipbuf, ziplen, (unsigned char*)this->weights.data_block(), outlen);
	delete [] zipbuf;
	zipbuf = NULL;
	if( count != ziplen || outlen != nx_*ny_*nz_*sizeof(float)) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "When reading weights!");
		fclose(fp);
		return false;
	}

	//load directions
	fread(&ziplen, sizeof(unsigned long), 1, fp);
	zipbuf = new unsigned char[ziplen];
	count = fread(zipbuf, 1, ziplen, fp);
	outlen = nx_*ny_*nz_*sizeof(vnl_vector_fixed<float, 3>);
	zlib_uncompress(zipbuf, ziplen, (unsigned char*)this->drts.data_block(), outlen);
	delete [] zipbuf;
	zipbuf = NULL;
	if( count != ziplen || outlen != nx_*ny_*nz_*sizeof(vnl_vector_fixed<float, 3>)) 
	{
		LOGGER()->error("DDF::load_data_from_file_zip", "When reading drt!");
		fclose(fp);
		return false;
	}
	this->bBuildColorField = false;

	//load colors if necessary
	fread(&ziplen, sizeof(unsigned long), 1, fp);
	if( !feof(fp) )
	{
		zipbuf = new unsigned char[ziplen];
		count = fread(zipbuf, 1, ziplen, fp);
		if(count != ziplen)
		{
			LOGGER()->error("DDF::load_data_from_file_zip", "When reading color field!");
			fclose(fp);
			return false;
		}

		outlen = nx_*ny_*nz_*sizeof(vnl_vector_fixed<float, 3>);
		this->clr_field.set_size(nx_, ny_, nz_);
		zlib_uncompress(zipbuf, ziplen, (unsigned char*)this->clr_field.data_block(), outlen);
		delete [] zipbuf;
		zipbuf = NULL;
		if( outlen != nx_*ny_*nz_*sizeof(vnl_vector_fixed<float, 3>)) 
		{
			LOGGER()->error("DDF::load_data_from_file_zip", "When reading color field!");
			fclose(fp);
			return false;
		}
		this->bBuildColorField = true;
	}

	fclose(fp);
	return true;
}


bool DDF::load_data_from_file(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "rb");
	if(!fp)
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot open the file <%s> for loading TSDF.", filename);
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
		LOGGER()->error("DDF::load_data_from_file", "Cannot read nx from file!");
		fclose(fp);
		return false;
	}

	count = fread(&ny_, sizeof(int), 1, fp);
	if( count != 1 || ny_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read ny to file!");
		fclose(fp);
		return false;
	}

	count = fread(&nz_, sizeof(int), 1, fp);
	if( count != 1 || nz_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read nz to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xres_, sizeof(double), 1, fp);
	if( count != 1 || xres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read xres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yres_, sizeof(double), 1, fp);
	if( count != 1 || yres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read yres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zres_, sizeof(double), 1, fp);
	if( count != 1 || zres_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read zres_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&xoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read xoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&yoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read yoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&zoffset_, sizeof(double), 1, fp);
	if( count != 1 ) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read zoffset_ to file!");
		fclose(fp);
		return false;
	}

	count = fread(&mu_, sizeof(double), 1, fp);
	if( count != 1 || mu_ <= 0) 
	{
		LOGGER()->error("DDF::load_data_from_file", "Cannot read mu_ to file!");
		fclose(fp);
		return false;
	}

	this->func_val.set_size(nx_, ny_, nz_);
	this->weights.set_size(nx_, ny_, nz_);
	this->drts.set_size(nx_, ny_, nz_);

	this->nx = nx_; this->ny = ny_; this->nz = nz_;
	this->xres = xres_; this->yres = yres_; this->zres = zres_;
	this->xoffset = xoffset_; this->yoffset = yoffset_; this->zoffset = zoffset_;
	this->mu = mu_;

	count = fread(this->func_val.data_block(), sizeof(float), nx_*ny_*nz_, fp);
	if( count != nx_*ny_*nz_ ) 
	{
		LOGGER()->error("DDF::load_data_from_file", "When reading func_val!");
		fclose(fp);
		return false;
	}

	count = fread(this->weights.data_block(), sizeof(float), nx_*ny_*nz_, fp);
	if( count != nx_*ny_*nz_ ) 
	{
		LOGGER()->error("DDF::load_data_from_file", "When reading weights!");
		fclose(fp);
		return false;
	}

	//save directions
	for(int k=0; k<nz; k++)
	for(int j=0; j<ny; j++)
	for(int i=0; i<nx; i++)
	{
		count = fread(this->drts(i, j, k).data_block(), sizeof(float), 3, fp);
		if( count != 3 ) 
		{
			LOGGER()->error("DDF::load_data_from_file", "When reading drts!");
			fclose(fp);
			return false;
		}
	}

	fclose(fp);
	return true;
}

//smooth directional distance function
DDF* filter_DDF(DDF const* ddf)
{
	if( ddf == NULL )
		return NULL;

	DDF* ddf_f = new DDF();
	*ddf_f = *ddf;

	int nx = ddf_f->nx;
	int ny = ddf_f->ny;
	int nz = ddf_f->nz;
	double xres = ddf_f->xres;
	double yres = ddf_f->yres;
	double zres = ddf_f->zres;
	double xoffset = ddf_f->xoffset;
	double yoffset = ddf_f->yoffset;
	double zoffset = ddf_f->zoffset;
	double mu = ddf_f->mu;

	for(unsigned int k=0; k<nz; k++)
	{
		for(unsigned int j=0; j<ny; j++)
		{		
			for(unsigned int i=0; i<nx; i++)
			{
				if( ddf->func_val(i, j, k) == SDF_NULL_VALUE )
					continue;

				float val_cur = ddf->func_val(i, j, k);

				double sigma_s = abs(val_cur)*3;
				double sigma_n = MAX(1.2, abs(val_cur)*3);
				int radius = MIN(4, MAX(1, ROUND(sigma_s*2.5)));

				float w_total = 0.0;
				float w_n_total = 0.0;
				float val_total = 0.0;
				float weight_total = 0.0;
				vnl_vector_fixed<float, 3> der_total(0.0, 0.0, 0.0);
				for(int kk=-radius; kk<=radius; kk++)
				for(int jj=-radius; jj<=radius; jj++)
				for(int ii=-radius; ii<=radius; ii++)
				{
					int z = k+kk;
					int y = j+jj;
					int x = i+ii;
					if( x >= nx || x < 0 ||
						y >= ny || y < 0 ||
						z >= nz || z < 0 )
						continue;

					if( ddf->func_val(x, y, z) == SDF_NULL_VALUE )
						continue;

					float val = ddf->func_val(x, y, z);

					float w = exp(-0.5*(kk*kk+jj*jj+ii*ii)/(sigma_s*sigma_s)); 
					float w_n = exp(-0.5*(kk*kk+jj*jj+ii*ii)/(sigma_n*sigma_n));
					
					val_total += w*val;
					der_total += w_n*ddf->drts(x, y, z);
					weight_total += w*ddf->weights(x, y, z);
					w_total += w;
					w_n_total += w_n;
				}

				ddf_f->func_val(i, j, k) = val_total/w_total;
				ddf_f->drts(i, j, k) = der_total/w_n_total;
				ddf_f->weights(i, j, k) = weight_total/w_total;
			}
		}
	}

	return ddf_f;
}

DDF* DDF::downsample(int rate, int extra_radius, double thres_count_ratio_)
{
	int nx_new = nx / rate;
	int ny_new = ny / rate;
	int nz_new = nz / rate;
	
	double x_res_new = xres*rate;
	double y_res_new = yres*rate;
	double z_res_new = zres*rate;

	DDF* ddf_new = new DDF();
	ddf_new->Init( xoffset + xres*(rate-1) / 2.0, yoffset + yres*(rate-1)/ 2.0, zoffset + zres*(rate-1) / 2.0, 
				   nx_new, ny_new, nz_new, 
				   x_res_new, y_res_new, z_res_new,
				   mu, bDynamicMu, bDynamicWeight, bBuildColorField );

	const double thres_count_ratio = thres_count_ratio_;
	const double thres_count = thres_count_ratio * (rate + 2 * extra_radius) * (rate + 2 * extra_radius) * (rate + 2 * extra_radius);

	for (int k = 0; k < nz_new; k++){
		for (int j = 0; j < ny_new; j++){
			for (int i = 0; i < nx_new; i++){
				int count = 0;
				double val_sum = 0.0;
				double weight_sum = 0.0;
				vnl_vector_fixed<float, 3> clr_sum(0.0);
				vnl_vector_fixed<float, 3> drt_sum(0.0);

				for (int kk = k*rate - extra_radius; kk <= k*rate + rate + extra_radius - 1; kk++){
					for (int jj = j*rate - extra_radius; jj <= j*rate + rate + extra_radius - 1; jj++){
						for (int ii = i*rate - extra_radius; ii <= i*rate + rate + extra_radius - 1; ii++){
							if (this->func_val.val_at(ii, jj, kk) != SDF_NULL_VALUE)
							{
								val_sum += this->func_val(ii, jj, kk);
								weight_sum += this->weights(ii, jj, kk);
								if (bBuildColorField)
									clr_sum += this->clr_field(ii, jj, kk);
								drt_sum += this->drts(ii, jj, kk);
								count++;
							}
						}
					}
				}

				if (count > thres_count)
				{
					ddf_new->func_val(i, j, k) = val_sum / count;
					ddf_new->weights(i, j, k) = weight_sum / count;
					if (bBuildColorField)
						ddf_new->clr_field(i, j, k) = clr_sum / count;
					ddf_new->drts(i, j, k) = drt_sum / count;
				}
			}
		}
	}

	return ddf_new;
}