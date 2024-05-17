#include "stdafx.h"
#include "normal_estimation.h"

///**********Local Plane Fitting or Normal Estimation for the depth map************************

bool 
local_plane_estimation_for_depthMap( vnl_matrix<double> const& depthMat, 
									 vnl_matrix_fixed<double, 3, 3> const& K,
									 CDepthMap &depthMap,
									 int radius_ori,
									 double thres_neighbors_z_dif)
{
#define MIN_NEIGHBOR_NUM 10
	int h = depthMat.rows();
	int w = depthMat.cols();
	depthMap.set_size(h, w);

	DepthElement depth_ele;

	depthMap.fill(depth_ele);
	
	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];

#pragma omp parallel for schedule(dynamic)
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0.0 )
				continue;

			double x = (j-cx)*z/fx;
			double y = (i-cy)*z/fy;

			depthMap[i][j].d = z;
			depthMap[i][j].P_cam[0] = x;
			depthMap[i][j].P_cam[1] = y;
			depthMap[i][j].P_cam[2] = z;
		}
	}

	//calculate normal
#pragma omp parallel for schedule(dynamic)
	for(int i=0; i<h; i++)
	{
		for(int j=i%2; j<w; j+=2)
		//for(int j=0; j<w; j++)
		{
			//if( i==26 && j==66 )
			//	int debug = 1;
			//else
			//	continue;

			double z = depthMat[i][j];
			if( z <= 0 )
				continue;

			int radius = radius_ori;//+ ROUND((z-125.0)/250.0);
			if( z > 250.0)
				radius = radius + 1;

			int points_num = (radius+1)*(radius+1);
			vnl_matrix<double> M(points_num, 4);

			int count = 0;
			for(int m=max(0,i-radius); m<=min(i+radius,h-1); m+=2)
			{
				for(int n=max(0,j-radius); n<=min(j+radius,w-1); n+=2)
				{
					double z_ = depthMat[m][n];
					if( z_ > 0 &&
						abs(z_-z) < thres_neighbors_z_dif *(1.0+z/300.0)) // the discontinuities in the depth map
					{
						M[count][0] = depthMap[m][n].P_cam[0]/1000.0;
						M[count][1] = depthMap[m][n].P_cam[1]/1000.0;
						M[count][2] = depthMap[m][n].P_cam[2]/1000.0;
						M[count][3] = 1.0;
						count++;
					}
				}
			}
			
			//SVD to get normal
			if(count >= MIN_NEIGHBOR_NUM)
			{
				//vcl_cout<<vcl_endl;
				//if( i==405 && j == 393)
				//	vcl_cout<<"begin:"<<vcl_endl<<M;

				if( count != points_num )
				{
					vnl_svd<double> svd_solver(M.get_n_rows(0, count));
					vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();

					//vnl_vector_fixed<double, 4> Q;
					//Q[0] = (svd_solver.W(0, 0))*(svd_solver.W(0, 0));
					//Q[1] = (svd_solver.W(1, 1))*(svd_solver.W(1, 1));
					//Q[2] = (svd_solver.W(2, 2))*(svd_solver.W(2, 2));
					//Q[3] = (svd_solver.W(3, 3))*(svd_solver.W(3, 3));

					
					// make sure the normal is always pointing against the camera center (origin)
					// or the camera center is below the local surface
					if( N_[3] > 0) 
						N_ *= -1.0;

					vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);
					
					double mag = N.magnitude();
					depthMap[i][j].normal = N / mag;
					
					depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;
					depthMap[i][j].theta = acos(N_[2]/mag);
					depthMap[i][j].phi = atan2(N_[1], N_[0]); //atan2(y, x);
				}
				else
				{
					vnl_svd<double> svd_solver(M);
					//vcl_cout<<M<<vcl_endl;
					vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();

					// make sure the normal is always pointing against the camera center (origin)
					// or the camera center is below the local surface
					if( N_[3] > 0) 
						N_ *= -1.0;

					vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);

					double mag = N.magnitude();
					depthMap[i][j].normal = N / mag;
					
					depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;	
					depthMap[i][j].theta = acos(N_[2]/mag);
					depthMap[i][j].phi = atan2(N_[1], N_[0]); //atan2(y, x)
				}

				//vcl_cout<<depthMap[i][j].normal;

				depthMap[i][j].bNormal = true;
			}
			else
				depthMap[i][j].bNormal = false;
		}
	}

	for(int i=0; i<h; i++)
	{
		for(int j=(i+1)%2; j<w; j+=2)
		{
			if( depthMat[i][j] <= 0 )
				continue;

			vnl_vector_fixed<double, 3> n(0.0, 0.0, 0.0);
			double rho = 0.0;
			int count = 0;
			if( i>=1 && depthMap[i-1][j].bNormal )
			{		
				rho += depthMap[i-1][j].rho;
				n += depthMap[i-1][j].normal;
				count++;
			}
			if( i<=h-2 && depthMap[i+1][j].bNormal )
			{
				rho += depthMap[i+1][j].rho;
				n += depthMap[i+1][j].normal;
				count++;
			}
			if( j>=1 && depthMap[i][j-1].bNormal )
			{
				rho += depthMap[i][j-1].rho;
				n += depthMap[i][j-1].normal;
				count++;
			}
			if( j<=w-2 && depthMap[i][j+1].bNormal )
			{
				rho += depthMap[i][j+1].rho;
				n += depthMap[i][j+1].normal;
				count++;
			}

			if( count > 2 )
			{			
				rho /= double(count);
				n /= double(count);
				n.normalize();

				double theta, phi;
				normal_to_sphere_coord(n, theta, phi);

				depthMap[i][j].normal = n;
				depthMap[i][j].rho = rho;
				depthMap[i][j].theta = theta;
				depthMap[i][j].phi = phi;
				depthMap[i][j].bNormal = true;
			}
		}
	}

	return true;
}

bool local_plane_estimation_for_depthMap2( vnl_matrix<double> const& depthMat, 
										   vnl_matrix_fixed<double, 3, 3> const& K,
										   CDepthMap &depthMap,
										   int radius_ori,
										   double thres_neighbors_z_dif,										   										   
										   int skip_step,
										   int min_points_num,
										   bool bSkipHalf // when true, calc normals for only half points
										   )
{
	int h = depthMat.rows();
	int w = depthMat.cols();
	depthMap.set_size(h, w);

	DepthElement depth_ele;

	depthMap.fill(depth_ele);
	
	double const&fx = K[0][0];
	double const&cx = K[0][2];
	double const&fy = K[1][1];
	double const&cy = K[1][2];

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0 )
				continue;

			double x = (j-cx)*z/fx;
			double y = (i-cy)*z/fy;

			depthMap[i][j].d = z;
			depthMap[i][j].P_cam[0] = x;
			depthMap[i][j].P_cam[1] = y;
			depthMap[i][j].P_cam[2] = z;
		}
	}

	//vnl_matrix<double> normal_confid(h, w);
	//normal_confid.fill(0.0);

	//calculate normal
#pragma omp parallel for
	for(int i=0; i<h; i++)
	{
		int j0 = 0;
		int js = 1;
		if( bSkipHalf )
		{
			j0 = i%2;
			js = 2;
		}
		for(int j=j0; j<w; j+=js) //for(int j=0; j<w; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0.0 )
				continue;

			int radius = radius_ori;//+ ROUND((z-125.0)/250.0);
			if( z > 250.0)
				radius = radius + 1;

			//int points_num = (radius+1)*(radius+1);

			double xy = 0.0, xz = 0.0, yz = 0.0;
			double xx = 0.0, yy = 0.0, zz = 0.0;
			double xc = 0.0, yc = 0.0, zc = 0.0;
			double cc = 0.0;
			for(int m=max(0,i-radius); m<=min(i+radius,h-1); m += skip_step)
			{
				for(int n=max(0,j-radius); n<=min(j+radius,w-1); n += skip_step)
				{
					vnl_vector_fixed<double, 3> Pcam_tmp = depthMap[m][n].P_cam;

					double z_ = Pcam_tmp[2];			
					if( z_ <= 0 ||
						abs(z_-z) > thres_neighbors_z_dif *(1.0+z/300.0)) // the discontinuities in the depth map
						continue;
						
					cc += 1.0;

					Pcam_tmp[0] /= 1000;
					Pcam_tmp[1] /= 1000;
					Pcam_tmp[2] /= 1000;

					xy += Pcam_tmp[0]*Pcam_tmp[1];
					xz += Pcam_tmp[0]*Pcam_tmp[2];
					yz += Pcam_tmp[1]*Pcam_tmp[2];
					xx += Pcam_tmp[0]*Pcam_tmp[0];
					yy += Pcam_tmp[1]*Pcam_tmp[1];
					zz += Pcam_tmp[2]*Pcam_tmp[2];
					xc += Pcam_tmp[0];
					yc += Pcam_tmp[1];
					zc += Pcam_tmp[2];
				}
			}
			if( cc >= min_points_num)
			{
				vnl_matrix<double> a(4, 4);
				a[0][0] = xx; a[0][1] = xy; a[0][2] = xz; a[0][3] = xc;
				a[1][0] = xy; a[1][1] = yy; a[1][2] = yz; a[1][3] = yc;
				a[2][0] = xz; a[2][1] = yz; a[2][2] = zz; a[2][3] = zc;
				a[3][0] = xc; a[3][1] = yc; a[3][2] = zc; a[3][3] = cc;
				vnl_svd<double> svd_solver(a);
				vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();
				//double s0 = svd_solver.W(0,0);
				//double s1 = svd_solver.W(1,1);
				//double s2 = svd_solver.W(2,2);
				//double s3 = svd_solver.W(3,3);
				//normal_confid[i][j] = svd_solver.W(2, 2)/svd_solver.W(3, 3);
				//normal_confid[i][j] = 1/svd_solver.W(3, 3);

				// make sure the normal is always pointing against the camera center (origin)
				// or the camera center is below the local surface
				if( N_[3] > 0) 
					N_ *= -1.0;

				vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);

				double mag = N.magnitude();
				depthMap[i][j].normal = N / mag;
				
				depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;	
				depthMap[i][j].theta = acos(N_[2]/mag);
				depthMap[i][j].phi = atan2(N_[1], N_[0]); //atan2(y, x)

				depthMap[i][j].bNormal = true;
			}
			else
				depthMap[i][j].bNormal = false;	
		}
	}

	if( !bSkipHalf )
		return true;

	for(int i=0; i<h; i++)
	{
		for(int j=(i+1)%2; j<w; j+=2)
		{
			if( depthMat[i][j] <= 0 )
				continue;

			vnl_vector_fixed<double, 3> n(0.0, 0.0, 0.0);
			double rho = 0.0;
			int count = 0;
			if( i>=1 && depthMap[i-1][j].bNormal )
			{		
				rho += depthMap[i-1][j].rho;
				n += depthMap[i-1][j].normal;
				count++;
			}
			if( i<=h-2 && depthMap[i+1][j].bNormal )
			{
				rho += depthMap[i+1][j].rho;
				n += depthMap[i+1][j].normal;
				count++;
			}
			if( j>=1 && depthMap[i][j-1].bNormal )
			{
				rho += depthMap[i][j-1].rho;
				n += depthMap[i][j-1].normal;
				count++;
			}
			if( j<=w-2 && depthMap[i][j+1].bNormal )
			{
				rho += depthMap[i][j+1].rho;
				n += depthMap[i][j+1].normal;
				count++;
			}

			if( count > 2 )
			{			
				rho /= double(count);
				n /= double(count);
				n.normalize();

				double theta, phi;
				normal_to_sphere_coord(n, theta, phi);

				depthMap[i][j].normal = n;
				depthMap[i][j].rho = rho;
				depthMap[i][j].theta = theta;
				depthMap[i][j].phi = phi;
				depthMap[i][j].bNormal = true;
			}
		}
	}
	return true;
}

bool local_plane_estimation_for_depthMap3( vnl_matrix<double> const& depthMat, 
										   vpgl_perspective_camera<double> const& cam,
										   CDepthMap &depthMap,
										   int radius_ori,
										   double thres_neighbors_z_dif,										   										   
										   int skip_step,
										   int min_points_num,
										   bool bSkipHalf // when true, calc normals for only half points
										   )
{
	int h = depthMat.rows();
	int w = depthMat.cols();
	depthMap.set_size(h, w);

	DepthElement depth_ele;

	depthMap.fill(depth_ele);
	
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);
	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	vnl_matrix_fixed<double, 3, 3> Rt = R.transpose();
	vnl_vector_fixed<double, 3> cam_cen;
	get_camera_center(cam, cam_cen);
#pragma omp parallel for schedule(dynamic)
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0 )
				continue;

			double x = (j-cx)*z/fx;
			double y = (i-cy)*z/fy;

			depthMap[i][j].d = z;
			vnl_vector_fixed<double, 3> P(x, y, z);
			depthMap[i][j].P_cam = Rt*(P-T);			
		}
	}

	int radius = radius_ori;
	int points_num = (2*radius+1)*(2*radius+1);
	if( bSkipHalf)
		points_num = (radius+1)*(radius+1);
	//calculate normal
#pragma omp parallel for schedule(dynamic)
	for(int i=0; i<h; i++)
	{
		int j0 = 0;
		int js = 1;
		if( bSkipHalf )
		{
			j0 = i%2;
			js = 2;
		}
		for(int j=j0; j<w; j+=js) //for(int j=0; j<w; j++)
		{
			double z = depthMat[i][j];
			if( z <= 0 )
				continue;

			double xy = 0.0, xz = 0.0, yz = 0.0;
			double xx = 0.0, yy = 0.0, zz = 0.0;
			double xc = 0.0, yc = 0.0, zc = 0.0;
			double cc = 0.0;
			for(int m=max(0,i-radius); m<=min(i+radius,h-1); m += skip_step)
			{
				for(int n=max(0,j-radius); n<=min(j+radius,w-1); n += skip_step)
				{
					vnl_vector_fixed<double, 3> Pcam_tmp = depthMap[m][n].P_cam;

					double z_ = depthMat[m][n];			
					if( z_ <= 0 ||
						abs(z_-z) > thres_neighbors_z_dif) // the discontinuities in the depth map
						continue;
						
					cc += 1.0;

					Pcam_tmp[0] /= 1000;
					Pcam_tmp[1] /= 1000;
					Pcam_tmp[2] /= 1000;

					xy += Pcam_tmp[0]*Pcam_tmp[1];
					xz += Pcam_tmp[0]*Pcam_tmp[2];
					yz += Pcam_tmp[1]*Pcam_tmp[2];
					xx += Pcam_tmp[0]*Pcam_tmp[0];
					yy += Pcam_tmp[1]*Pcam_tmp[1];
					zz += Pcam_tmp[2]*Pcam_tmp[2];
					xc += Pcam_tmp[0];
					yc += Pcam_tmp[1];
					zc += Pcam_tmp[2];
				}
			}
			if( cc >= min_points_num)
			{
				vnl_matrix<double> a(4, 4);
				a[0][0] = xx; a[0][1] = xy; a[0][2] = xz; a[0][3] = xc;
				a[1][0] = xy; a[1][1] = yy; a[1][2] = yz; a[1][3] = yc;
				a[2][0] = xz; a[2][1] = yz; a[2][2] = zz; a[2][3] = zc;
				a[3][0] = xc; a[3][1] = yc; a[3][2] = zc; a[3][3] = cc;
				vnl_svd<double> svd_solver(a);
				vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();
				//double s0 = svd_solver.W(0,0);
				//double s1 = svd_solver.W(1,1);
				//double s2 = svd_solver.W(2,2);
				//double s3 = svd_solver.W(3,3);
				//normal_confid[i][j] = svd_solver.W(2, 2)/svd_solver.W(3, 3);
				//normal_confid[i][j] = 1/svd_solver.W(3, 3);

				// make sure the normal is always pointing against the camera center
				vnl_vector_fixed<double, 3> lookat = depthMap[i][j].P_cam - cam_cen;
				if( (N_[0]*lookat[0] + N_[1]*lookat[1] + N_[2]*lookat[2]) < 0) 
					N_ *= -1.0;

				vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);

				double mag = N.magnitude();
				depthMap[i][j].normal = N / mag;
				
				//depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;	
				//depthMap[i][j].theta = vcl_acos(N_[2]/mag);
				//depthMap[i][j].phi = vcl_atan2(N_[1], N_[0]); //atan2(y, x)

				depthMap[i][j].bNormal = true;
			}
			else
				depthMap[i][j].bNormal = false;	
		}
	}

	if( !bSkipHalf )
		return true;

#pragma omp parallel for 
	for(int i=0; i<h; i++)
	{
		for(int j=(i+1)%2; j<w; j+=2)
		{
			if( depthMat[i][j] <= 0 )
				continue;

			vnl_vector_fixed<double, 3> n(0.0, 0.0, 0.0);
			double rho = 0.0;
			int count = 0;
			if( i>=1 && depthMap[i-1][j].bNormal )
			{		
				rho += depthMap[i-1][j].rho;
				n += depthMap[i-1][j].normal;
				count++;
			}
			if( i<=h-2 && depthMap[i+1][j].bNormal )
			{
				rho += depthMap[i+1][j].rho;
				n += depthMap[i+1][j].normal;
				count++;
			}
			if( j>=1 && depthMap[i][j-1].bNormal )
			{
				rho += depthMap[i][j-1].rho;
				n += depthMap[i][j-1].normal;
				count++;
			}
			if( j<=w-2 && depthMap[i][j+1].bNormal )
			{
				rho += depthMap[i][j+1].rho;
				n += depthMap[i][j+1].normal;
				count++;
			}

			if( count > 2 )
			{			
				rho /= double(count);
				n /= double(count);
				n.normalize();

				double theta, phi;
				normal_to_sphere_coord(n, theta, phi);

				depthMap[i][j].normal = n;
				depthMap[i][j].rho = rho;
				depthMap[i][j].theta = theta;
				depthMap[i][j].phi = phi;
				depthMap[i][j].bNormal = true;
			}
		}
	}
	return true;
}

bool local_plane_estimation_for_depthMap_ortho( cv::Mat& depthMat, // the depth map from the depth camera
												CDepthMap &depthMap,
												double pixel_dist,
												int radius_ori, //search radius to find NNs for normal computation
												double thres_neighbors_z_dif,
												int skip_step, //the step for neighborhood search
												int min_points_num, //minimum number of points to fit a local plane
												bool bSkipHalf // when true, calc normals for only half points
												)
{
	int h = depthMat.rows;
	int w = depthMat.cols;

	DepthElement depth_ele;

	depthMap.set_size(h, w);
	depthMap.fill(depth_ele);
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			double z = depthMat.at<double>(i, j);;
			if (z <= 0.0)
				continue;

			double x = (j - w / 2)*pixel_dist;
			double y = (i - h / 2)*pixel_dist;

			depthMap[i][j].d = z;
			 depthMap[i][j].P_cam = vnl_vector_fixed<double, 3>(x, y, z);
		}
	}

	int radius = radius_ori;
	int points_num = (2 * radius + 1)*(2 * radius + 1);
	if (bSkipHalf)
		points_num = (radius + 1)*(radius + 1);
	//calculate normal
#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i<h; i++)
	{
		int j0 = 0;
		int js = 1;
		if (bSkipHalf)
		{
			j0 = i % 2;
			js = 2;
		}
		for (int j = j0; j<w; j += js) //for(int j=0; j<w; j++)
		{
			double z = depthMat.at<double>(i, j);;
			if (z <= 0)
				continue;

			double xy = 0.0, xz = 0.0, yz = 0.0;
			double xx = 0.0, yy = 0.0, zz = 0.0;
			double xc = 0.0, yc = 0.0, zc = 0.0;
			double cc = 0.0;
			for (int m = max(0, i - radius); m <= min(i + radius, h - 1); m += skip_step)
			{
				for (int n = max(0, j - radius); n <= min(j + radius, w - 1); n += skip_step)
				{
					vnl_vector_fixed<double, 3> Pcam_tmp = depthMap[m][n].P_cam;

					double z_ = depthMat.at<double>(m, n);;
					if (z_ <= 0 ||
						abs(z_ - z) > thres_neighbors_z_dif) // the discontinuities in the depth map
						continue;

					cc += 1.0;

					Pcam_tmp[0] /= 1000;
					Pcam_tmp[1] /= 1000;
					Pcam_tmp[2] /= 1000;

					xy += Pcam_tmp[0] * Pcam_tmp[1];
					xz += Pcam_tmp[0] * Pcam_tmp[2];
					yz += Pcam_tmp[1] * Pcam_tmp[2];
					xx += Pcam_tmp[0] * Pcam_tmp[0];
					yy += Pcam_tmp[1] * Pcam_tmp[1];
					zz += Pcam_tmp[2] * Pcam_tmp[2];
					xc += Pcam_tmp[0];
					yc += Pcam_tmp[1];
					zc += Pcam_tmp[2];
				}
			}
			if (cc >= min_points_num)
			{
				vnl_matrix<double> a(4, 4);
				a[0][0] = xx; a[0][1] = xy; a[0][2] = xz; a[0][3] = xc;
				a[1][0] = xy; a[1][1] = yy; a[1][2] = yz; a[1][3] = yc;
				a[2][0] = xz; a[2][1] = yz; a[2][2] = zz; a[2][3] = zc;
				a[3][0] = xc; a[3][1] = yc; a[3][2] = zc; a[3][3] = cc;
				vnl_svd<double> svd_solver(a);
				vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();
				//double s0 = svd_solver.W(0,0);
				//double s1 = svd_solver.W(1,1);
				//double s2 = svd_solver.W(2,2);
				//double s3 = svd_solver.W(3,3);
				//normal_confid[i][j] = svd_solver.W(2, 2)/svd_solver.W(3, 3);
				//normal_confid[i][j] = 1/svd_solver.W(3, 3);

				// make sure the normal is always pointing against the camera center
				vnl_vector_fixed<double, 3> lookat(0.0, 0.0, 1.0);// depthMap[i][j].P_cam - cam_cen;
				if ((N_[0] * lookat[0] + N_[1] * lookat[1] + N_[2] * lookat[2]) < 0)
					N_ *= -1.0;

				vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);

				double mag = N.magnitude();
				depthMap[i][j].normal = N / mag;

				//depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;	
				//depthMap[i][j].theta = vcl_acos(N_[2]/mag);
				//depthMap[i][j].phi = vcl_atan2(N_[1], N_[0]); //atan2(y, x)

				depthMap[i][j].bNormal = true;
			}
			else
				depthMap[i][j].bNormal = false;
		}
	}

	if (!bSkipHalf)
		return true;

#pragma omp parallel for 
	for (int i = 0; i<h; i++)
	{
		for (int j = (i + 1) % 2; j<w; j += 2)
		{
			if (depthMat.at<double>(i, j) <= 0)
				continue;

			vnl_vector_fixed<double, 3> n(0.0, 0.0, 0.0);
			double rho = 0.0;
			int count = 0;
			if (i >= 1 && depthMap[i - 1][j].bNormal)
			{
				rho += depthMap[i - 1][j].rho;
				n += depthMap[i - 1][j].normal;
				count++;
			}
			if (i <= h - 2 && depthMap[i + 1][j].bNormal)
			{
				rho += depthMap[i + 1][j].rho;
				n += depthMap[i + 1][j].normal;
				count++;
			}
			if (j >= 1 && depthMap[i][j - 1].bNormal)
			{
				rho += depthMap[i][j - 1].rho;
				n += depthMap[i][j - 1].normal;
				count++;
			}
			if (j <= w - 2 && depthMap[i][j + 1].bNormal)
			{
				rho += depthMap[i][j + 1].rho;
				n += depthMap[i][j + 1].normal;
				count++;
			}

			if (count > 2)
			{
				rho /= double(count);
				n /= double(count);
				n.normalize();

				double theta, phi;
				normal_to_sphere_coord(n, theta, phi);

				depthMap[i][j].normal = n;
				depthMap[i][j].rho = rho;
				depthMap[i][j].theta = theta;
				depthMap[i][j].phi = phi;
				depthMap[i][j].bNormal = true;
			}
		}
	}
	return true;
}

bool local_plane_estimation_for_depthMap_ortho( cv::Mat& depthMat, // the depth map from the depth camera
												vnl_matrix<vnl_vector_fixed<double, 3>> &normalMap,
												double pixel_dist,
												int radius, //search radius to find NNs for normal computation
												double thres_neighbors_z_dif,
												int skip_step, //the step for neighborhood search
												int min_points_num, //minimum number of points to fit a local plane
												bool bSkipHalf // when true, calc normals for only half points
	)
{
	CDepthMap depthMap;
	local_plane_estimation_for_depthMap_ortho(depthMat, depthMap, pixel_dist, radius, thres_neighbors_z_dif, skip_step, min_points_num, bSkipHalf);
	normalMap.set_size(depthMat.rows, depthMat.cols);

	extract_normal_to_matrix(depthMap, normalMap);
	return true;
}

bool local_plane_estimation_for_depthMap( vnl_matrix<double> const& X, 
										  vnl_matrix<double> const& Y,
										  vnl_matrix<double> const& Z,
										  CDepthMap &depthMap,
										  int radius_ori )
{
	int h = X.rows();
	int w = X.cols();
	if( Y.rows() != h || Y.cols() != w ||
		Y.rows() != h || Y.cols() != w )
	{
		printf("Error: the size of input matrice do not agree with each other!\n");
		return false;
	}

	depthMap.set_size(h, w);
	DepthElement depth_ele;
	depthMap.fill(depth_ele);

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( Z[i][j] <= 0 )
				continue;

			depthMap[i][j].d = Z[i][j];
			depthMap[i][j].P_cam[0] = X[i][j];
			depthMap[i][j].P_cam[1] = Y[i][j];
			depthMap[i][j].P_cam[2] = Z[i][j];
		}
	}

	//calculate normal
	for(int i=0; i<h; i++)
	{
		for(int j=i%2; j<w; j+=2)
		//for(int j=0; j<w; j++)
		{
			double z = depthMap[i][j].d;
			if( z <= 0 )
				continue;

			int radius = radius_ori;//+ ROUND((z-125.0)/250.0);
			if( z > 250.0)
				radius = radius + 1;

			int points_num = (radius+1)*(radius+1);
			vnl_matrix<double> M(points_num, 4);

			int count = 0;
			for(int m=max(0,i-radius); m<=min(i+radius,h-1); m+=2)
			{
				for(int n=max(0,j-radius); n<=min(j+radius,w-1); n+=2)
				{
					double z_ = depthMap[m][n].d;
					if( z_ > 0 &&
						abs(z_-z) < 30.0*(1.0+z/300.0)) // the discontinuities in the depth map
					{
						M[count][0] = depthMap[m][n].P_cam[0]/1000.0;
						M[count][1] = depthMap[m][n].P_cam[1]/1000.0;
						M[count][2] = depthMap[m][n].P_cam[2]/1000.0;
						M[count][3] = 1.0;
						count++;
					}
				}
			}
			
			//SVD to get normal
			if(count >= MIN_NEIGHBOR_NUM)
			{
				//vcl_cout<<vcl_endl;
				//if( i==405 && j == 393)
				//	vcl_cout<<"begin:"<<vcl_endl<<M;

				if( count != points_num )
				{
					vnl_svd<double> svd_solver(M.get_n_rows(0, count));
					vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();
					
					// make sure the normal is always pointing against the camera center (origin)
					// or the camera center is below the local surface
					if( N_[3] > 0) 
						N_ *= -1.0;

					vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);
					
					double mag = N.magnitude();
					depthMap[i][j].normal = N / mag;
					
					depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;
					depthMap[i][j].theta = acos(N_[2]/mag);
					depthMap[i][j].phi = atan2(N_[1], N_[0]); //atan2(y, x);
				}
				else
				{
					vnl_svd<double> svd_solver(M);
					vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();

					// make sure the normal is always pointing against the camera center (origin)
					// or the camera center is below the local surface
					if( N_[3] > 0) 
						N_ *= -1.0;

					vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);

					double mag = N.magnitude();
					depthMap[i][j].normal = N / mag;
					
					depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;	
					depthMap[i][j].theta = acos(N_[2]/mag);
					depthMap[i][j].phi = atan2(N_[1], N_[0]); //atan2(y, x)
				}

				//vcl_cout<<depthMap[i][j].normal;

				depthMap[i][j].bNormal = true;
			}
			else
				depthMap[i][j].bNormal = false;
		}
	}

	for(int i=0; i<h; i++)
	{
		for(int j=(i+1)%2; j<w; j+=2)
		{
			if( depthMap[i][j].d <= 0 )
				continue;

			vnl_vector_fixed<double, 3> n(0.0, 0.0, 0.0);
			double rho = 0.0;
			int count = 0;
			if( i>=1 && depthMap[i-1][j].bNormal )
			{		
				rho += depthMap[i-1][j].rho;
				n += depthMap[i-1][j].normal;
				count++;
			}
			if( i<=h-2 && depthMap[i+1][j].bNormal )
			{
				rho += depthMap[i+1][j].rho;
				n += depthMap[i+1][j].normal;
				count++;
			}
			if( j>=1 && depthMap[i][j-1].bNormal )
			{
				rho += depthMap[i][j-1].rho;
				n += depthMap[i][j-1].normal;
				count++;
			}
			if( j<=w-2 && depthMap[i][j+1].bNormal )
			{
				rho += depthMap[i][j+1].rho;
				n += depthMap[i][j+1].normal;
				count++;
			}

			if( count > 2 )
			{			
				rho /= double(count);
				n /= double(count);
				n.normalize();

				double theta, phi;
				normal_to_sphere_coord(n, theta, phi);

				depthMap[i][j].normal = n;
				depthMap[i][j].rho = rho;
				depthMap[i][j].theta = theta;
				depthMap[i][j].phi = phi;
				depthMap[i][j].bNormal = true;
			}
		}
	}

	return true;
}


bool calc_normal_at_imgCoord( int x, int y, 
							  vnl_matrix<double> const&depthMat, 
							  vnl_matrix_fixed<double, 3, 3> const&K,
							  vnl_vector_fixed<double, 3> &normal,
							  int radius)
{
	double fx = K[0][0];
	double cx = K[0][2];
	double fy = K[1][1];
	double cy = K[1][2];
	int h = depthMat.rows();
	int w = depthMat.cols();
	
	int points_num = (2*radius+1)*(2*radius+1);
	vnl_matrix<double> M(points_num, 4);

	double z = extract_depth_at_visual_feature(depthMat, x, y, depth_extract_NN);
	if( z <= 0.0)
		return false;

	int count = 0;
	for(int i=max(0,y-radius); i<=min(y+radius,h-1); i++)
	{
		for(int j=max(0,x-radius); j<=min(x+radius,w-1); j++)
		{
			double z_ = depthMat[i][j];
			if( z_ > 0 &&
				abs(z_-z) < 30.0*(1.0+z/300.0)) // the discontinuities in the depth map
			{
				M[count][0] = (j-cx+0.5)*z_/fx/1000.0;
				M[count][1] = (i-cy+0.5)*z_/fy/1000.0;
				M[count][2] = z_/1000.0;
				M[count][3] = 1.0;
				count++;
			}
		}
	}

	if(count < 10)
		return false;

	vnl_svd<double> svd_solver(M.get_n_rows(0, count));
	vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();
			
	// make sure the normal is always pointing against the camera center (origin)
	// or the camera center is below the local surface
	if( N_[3] > 0) 
		N_ *= -1.0;

	vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);
					
	double mag = N.magnitude();
	normal = N / mag;
					
	//depthMap[i][j].rho = abs(N_[3])/mag * 1000.0;
	//depthMap[i][j].theta = vcl_acos(N_[2]/mag);
	//depthMap[i][j].phi = vcl_atan2(N_[1], N_[0]); //atan2(y, x);
	return true;

}

/* extract normal to a matrix
 */
void extract_normal_to_matrix( CDepthMap &depthMap,
							   vnl_matrix< vnl_vector_fixed<double, 3> > &normal_matrix )
{
	int h = depthMap.rows();
	int w = depthMap.cols();
	normal_matrix.set_size(h, w);
	normal_matrix.fill( vnl_vector_fixed<double, 3>(0.0, 0.0, 0.0) );
	
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( depthMap[i][j].bNormal )
				normal_matrix[i][j] = depthMap[i][j].normal;
		}
	}	
}
/* extract normal to an image
 */
void extract_normal_to_image( CDepthMap const& depthMap,
							  vil_image_view<vxl_byte>& normal_img )
{
	int h = depthMap.rows();
	int w = depthMap.cols();

	normal_img.set_size(w, h, 3);
	normal_img.fill(0);

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( depthMap[i][j].bNormal )
			{
				vnl_vector_fixed<double, 3> normal = depthMap[i][j].normal;
				normal_img(j, i, 0) = ROUND(((normal[0]+1.0)/2.0) * 255);
				normal_img(j, i, 1) = ROUND(((normal[1]+1.0)/2.0) * 255);
				normal_img(j, i, 2) = ROUND(((normal[2]+1.0)/2.0) * 255);
			}
		}
	}
}

cv::Mat extract_normal_to_image( CDepthMap const& depthMap )
{
	int h = depthMap.rows();
	int w = depthMap.cols();
	cv::Mat ret = cv::Mat(h, w, CV_8UC3, cv::Scalar(0));

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if( depthMap[i][j].bNormal )
			{
				vnl_vector_fixed<double, 3> normal = depthMap[i][j].normal;
				ret.at<cv::Vec3b>(i, j)[2] = ROUND(((normal[0]+1.0)/2.0) * 255);
				ret.at<cv::Vec3b>(i, j)[1] = ROUND(((normal[1]+1.0)/2.0) * 255);
				ret.at<cv::Vec3b>(i, j)[0] = ROUND(((normal[2]+1.0)/2.0) * 255);
			}
		}
	}
	return ret;	
}