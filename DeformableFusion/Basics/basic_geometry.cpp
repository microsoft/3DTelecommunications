// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "basic_geometry.h"

//return true, if p is inside quad. assuming the quad is convex
bool is_point_inside_quad(vgl_point_2d<double> &p, Quadrilateral2D &quad)
{
	vgl_point_2d<double> &p1 = quad.p1;
	vgl_point_2d<double> &p2 = quad.p2;
	vgl_point_2d<double> &p3 = quad.p3;
	vgl_point_2d<double> &p4 = quad.p4;

	//p stays at the same side of line p1->p2 as p3
	double d1, d2;
	d1 = (p3.y() - p1.y())*(p2.x() - p1.x()) - (p2.y() - p1.y())*(p3.x() - p1.x());
	d2 = (p.y() - p1.y())*(p2.x() - p1.x()) - (p2.y() - p1.y())*(p.x() - p1.x());
	if (d1 * d2 < 0)
		return false;

	//p stays at the same side of line p2->p3 as p4
	d1 = (p4.y() - p2.y())*(p3.x() - p2.x()) - (p3.y() - p2.y())*(p4.x() - p2.x());
	d2 = (p.y() - p2.y())*(p3.x() - p2.x()) - (p3.y() - p2.y())*(p.x() - p2.x());
	if (d1 * d2 < 0)
		return false;

	d1 = (p1.y() - p3.y())*(p4.x() - p3.x()) - (p4.y() - p3.y())*(p1.x() - p3.x());
	d2 = (p.y() - p3.y())*(p4.x() - p3.x()) - (p4.y() - p3.y())*(p.x() - p3.x());
	if (d1 * d2 < 0)
		return false;

	d1 = (p2.y() - p4.y())*(p1.x() - p4.x()) - (p1.y() - p4.y())*(p2.x() - p4.x());
	d2 = (p.y() - p4.y())*(p1.x() - p4.x()) - (p1.y() - p4.y())*(p.x() - p4.x());
	if (d1 * d2 < 0)
		return false;

	return true;
}

bool test_line_segments_intersect(vgl_point_2d<double> l_p1, vgl_point_2d<double> l_p2,
	vgl_point_2d<double> r_p1, vgl_point_2d<double> r_p2)
{
	vgl_line_2d<double> L1(l_p1, l_p2);
	vgl_line_2d<double> L2(r_p1, r_p2);

	vgl_point_2d<double> p;
	vgl_intersection(L1, L2, p);

	double dx = abs(l_p1.x() - l_p2.x());
	double dy = abs(l_p1.y() - l_p2.y());
	bool bInside1 = false;
	if (dx > dy)
	{
		if (p.x() >= min(l_p1.x(), l_p2.x()) &&
			p.x() <= max(l_p1.x(), l_p2.x()))
			bInside1 = true;
	}
	else
	{
		if (p.y() >= min(l_p1.y(), l_p2.y()) &&
			p.y() <= max(l_p1.y(), l_p2.y()))
			bInside1 = true;
	}

	dx = abs(r_p1.x() - r_p2.x());
	dy = abs(r_p1.y() - r_p2.y());
	bool bInside2 = false;
	if (dx > dy)
	{
		if (p.x() >= min(r_p1.x(), r_p2.x()) &&
			p.x() <= max(r_p1.x(), r_p2.x()))
			bInside2 = true;
	}
	else
	{
		if (p.y() >= min(r_p1.y(), r_p2.y()) &&
			p.y() <= max(r_p1.y(), r_p2.y()))
			bInside2 = true;
	}

	if (bInside1 && bInside2)
		return true;
	else
		return false;
}


//return true, if they overlap
bool test_quads_overlap(Quadrilateral2D &quad1, Quadrilateral2D &quad2)
{
	if (is_point_inside_quad(quad1.p1, quad2) ||
		is_point_inside_quad(quad1.p2, quad2) ||
		is_point_inside_quad(quad1.p3, quad2) ||
		is_point_inside_quad(quad1.p4, quad2) ||
		is_point_inside_quad(quad2.p1, quad1) ||
		is_point_inside_quad(quad2.p2, quad1) ||
		is_point_inside_quad(quad2.p3, quad1) ||
		is_point_inside_quad(quad2.p4, quad1))
		return true;

	if (test_line_segments_intersect(quad1.p1, quad1.p2, quad2.p1, quad2.p2) ||
		test_line_segments_intersect(quad1.p1, quad1.p2, quad2.p2, quad2.p3) ||
		test_line_segments_intersect(quad1.p1, quad1.p2, quad2.p3, quad2.p4) ||
		test_line_segments_intersect(quad1.p1, quad1.p2, quad2.p4, quad2.p1) ||
		test_line_segments_intersect(quad1.p2, quad1.p3, quad2.p1, quad2.p2) ||
		test_line_segments_intersect(quad1.p2, quad1.p3, quad2.p2, quad2.p3) ||
		test_line_segments_intersect(quad1.p2, quad1.p3, quad2.p3, quad2.p4) ||
		test_line_segments_intersect(quad1.p2, quad1.p3, quad2.p4, quad2.p1) ||
		test_line_segments_intersect(quad1.p3, quad1.p4, quad2.p1, quad2.p2) ||
		test_line_segments_intersect(quad1.p3, quad1.p4, quad2.p2, quad2.p3) ||
		test_line_segments_intersect(quad1.p3, quad1.p4, quad2.p3, quad2.p4) ||
		test_line_segments_intersect(quad1.p3, quad1.p4, quad2.p4, quad2.p1) ||
		test_line_segments_intersect(quad1.p4, quad1.p1, quad2.p1, quad2.p2) ||
		test_line_segments_intersect(quad1.p4, quad1.p1, quad2.p2, quad2.p3) ||
		test_line_segments_intersect(quad1.p4, quad1.p1, quad2.p3, quad2.p4) ||
		test_line_segments_intersect(quad1.p4, quad1.p1, quad2.p4, quad2.p1))
		return true;

	return false;
}

bool find_maxima_by_polynomial_fitting_2D( double x_center, double y_center, double step,
										   vnl_matrix_fixed<double, 3, 3> const& vals, 
										   double &x_maxima, double &y_maxima, double &val_maxima)
{
	vnl_matrix<double> Fx(9, 6);
	vnl_vector<double> v(9);
	int count = 0;
	for(int i=-1; i<=1; i++) // row
	{
		for(int j=-1; j<=1; j++) //col
		{
			// a x*x + b y*y + c x*y + d x + e y + f = v
			double x = j;//x_center+j*step;
			double y = i;//y_center+i*step;
			Fx(count, 0) = x*x;
			Fx(count, 1) = y*y;
			Fx(count, 2) = x*y;
			Fx(count, 3) = x;
			Fx(count, 4) = y;
			Fx(count, 5) = 1.0;
			v(count) = vals(i+1, j+1);
			count++;
		}
	}

	vnl_svd<double> svd_solver(Fx);
	vnl_vector<double> c = svd_solver.solve(v);

	vnl_matrix_fixed<double, 2, 2> H;
	H[0][0] = c[0];
	H[1][1] = c[1];
	H[0][1] = c[2]/2.0;
	H[1][0] = H[0][1];

	vnl_vector_fixed<double, 2> G;
	G[0] = c[3];
	G[1] = c[4];

	vnl_symmetric_eigensystem<double> eig(H);
	vnl_vector<double> eval(2);
	eval[0] = eig.get_eigenvalue(0);
	eval[1] = eig.get_eigenvalue(1);
	if( eval[0] >= 0 || eval[1] >= 0 )
	{
		return false;
	}

	vnl_svd<double> svd_H(H);
	vnl_vector<double> x = svd_H.solve(-0.5*G);
	x_maxima = x_center + x[0]*step;
	y_maxima = y_center + x[1]*step;
	val_maxima = dot_product<double>(x, H*x) + dot_product<double>(G, x) + c[5];
	return true;
}

void plane_fitting( vector< vnl_vector_fixed<double, 3> > &points, 
				    vnl_vector_fixed<double, 3> &normal,
				    double &d,
					double scalar,
					double *dist_mean,
					double *dist_std)
{
	int points_num = points.size();
	vnl_matrix<double> M(points_num, 4);
	M.fill(1.0);
	for(int j=0; j<points_num; j++)
	{
		vnl_vector_fixed<double, 3> &p = points[j];
		M[j][0] = p[0]/scalar;
		M[j][1] = p[1]/scalar;
		M[j][2] = p[2]/scalar;
		M[j][3] = 1.0;
	}

	vnl_svd<double> svd_solver(M);
	vnl_vector_fixed<double, 4> N_ = svd_solver.nullvector();

	// make sure the normal is always pointing against the camera center (origin)
	// or the camera center is below the local surface
	if( N_[3] > 0)  N_ *= -1.0;
	vnl_vector_fixed<double, 3> N(N_[0], N_[1], N_[2]);
	double mag = N.magnitude();

	normal = N / mag;
	d = abs(N_[3])/mag * scalar;	

	//statistics
	if( dist_mean != NULL )
	{
		vnl_vector_fixed<double, 4> t(normal[0], normal[1], normal[2], -d);
		for(int j=0; j<points_num; j++)
		{
			vnl_vector_fixed<double, 3> &p = points[j];
			M[j][0] = p[0];
			M[j][1] = p[1];
			M[j][2] = p[2];
			M[j][3] = 1.0;
		}
		vnl_vector<double> dists = M*t;
		for(int i=0; i<dists.size(); i++)
			dists[i] = std::abs(dists[i]);
		*dist_mean = VecOperation<double>::GetMean(dists.data_block(), dists.size());
		if( dist_std != NULL )
			*dist_std = VecOperation<double>::GetStd(dists.data_block(), *dist_mean, dists.size(), 0);
	}
}

//load calibration file
bool load_calibration_file_with_size( char const* file_name,
									  vnl_matrix_fixed<double, 3, 3> &K, 
									  vnl_matrix_fixed<double, 3, 3> &R,
									  vnl_vector_fixed<double, 3> &T,
									  vnl_vector_fixed<double, 5> &radial_distort)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "r");
	if(!fp){
		printf("Cannot open the camera calibration file<%s>.\n", file_name);
		return false;
	}

	double width, height;
	fscanf_s(fp, "%lf", &width);
	fscanf_s(fp, "%lf", &height);

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			int numread = fscanf_s(fp,"%lf",&(K[i][j]));
			if(numread != 1)
				return false;
		}
	}

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			int numread = fscanf_s(fp,"%lf",&(R[i][j]));
			if(numread != 1)
				return false;
		}
	}
	for(int i=0;i<3;i++)
	{
		int numread = fscanf_s(fp,"%lf",&(T[i]));
		if(numread != 1)
			return false;
	}

	for(int i=0;i<5;i++)
	{
		int numread = fscanf_s(fp,"%lf",&(radial_distort[i]));
		if(numread != 1)
			return false;
	}

	fclose(fp);
	return true;
}

bool load_calibration_file_with_size( char const* file_name,
									  vpgl_perspective_camera<double> &cam,
									  vnl_vector_fixed<double, 5> &radial_distort)
{
	vnl_matrix_fixed<double, 3, 3> K;
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	if( !load_calibration_file_with_size(file_name, K, R, T, radial_distort) )
		return false;
	set_calibration_matrix(cam, K);
	set_camera_pose(cam, R, T);
	return true;
}

/*========================================================================================
// Implementation of:
//						Geometry Operations
//
========================================================================================*/

void sphere_coord_to_normal(double theta, double phi, vnl_vector_fixed<double, 3> &n)
{
	n[0] = sin(theta)*cos(phi);
	n[1] = sin(theta)*sin(phi);
	n[2] = cos(theta);
}

void normal_to_sphere_coord(vnl_vector_fixed<double, 3> n, double &theta, double &phi)
{
	n.normalize();
	theta = acos(n[2]);
	phi = atan2(n[1], n[0]);
}

double rotation_angle(vnl_matrix_fixed<double, 3, 3> &R)
{
	vnl_vector_fixed<double, 3> rod;
	matrix_to_rodrigues(R, rod);

	double ret = rod.two_norm();
	ret = ret*180.0/M_PI;
	if(ret > 180.0)
		ret = 360.0-ret;

	return ret;
}


/* the corresponding rows of X, Y are 3-d vectors <x, y>, where exist a desiring R satisfying Rx = y;
 * the overall cost function to *MAXIMIZE* is \sum_i x_i^T * R * y_i
 * the X and Y should have the same number of rows and the row numbers should at least be *TWO*
 */
bool calcRotationMat( vnl_matrix<double> const&X, vnl_matrix<double> const&Y,
					 vnl_matrix_fixed<double, 3, 3> &R)
{
	if( X.rows() != Y.rows() ||
		X.cols() != 3 ||
		Y.cols() != 3 ||
		X.rows() < 2)
	{
		printf("Error: the input data is invalid!\n");
		return false;
	}
	
	vnl_matrix<double> M = X.transpose() * Y;
	double Sxx = M[0][0];
	double Sxy = M[0][1];
	double Sxz = M[0][2];
	double Syx = M[1][0];
	double Syy = M[1][1];
	double Syz = M[1][2];
	double Szx = M[2][0];
	double Szy = M[2][1];
	double Szz = M[2][2];

	vnl_matrix<double> N(4, 4);
	N[0][0] = Sxx+Syy+Szz; N[0][1] = Syz-Szy;	  N[0][2] = Szx-Sxz;	  N[0][3] = Sxy-Syx;
	N[1][0] = N[0][1];	   N[1][1] = Sxx-Syy-Szz; N[1][2] = Sxy+Syx;	  N[1][3] = Szx+Sxz;
	N[2][0] = N[0][2];	   N[2][1] = N[1][2];	  N[2][2] = -Sxx+Syy-Szz; N[2][3] = Syz+Szy;
	N[3][0] = N[0][3];	   N[3][1] = N[1][3];	  N[3][2] = N[2][3];	  N[3][3] = -Sxx-Syy+Szz;

	vnl_symmetric_eigensystem<double> eig(N);
	vnl_vector<double> q_ = eig.get_eigenvector(3);

	vnl_quaternion<double> q(q_[1], q_[2], q_[3], q_[0]);
	
	vgl_rotation_3d<double> R_(q);

	R = R_.as_matrix();

	return true;
}

bool calcRotationMat( vnl_matrix<double> const&X, vnl_matrix<double> const&Y, vnl_vector<double> const&W,
					 vnl_matrix_fixed<double, 3, 3> &R)
{
	if( X.rows() != Y.rows() ||
		X.rows() != W.size() ||
		X.cols() != 3 ||
		Y.cols() != 3 ||
		X.rows() < 2)
	{
		printf("Error: the input data is invalid!\n");
		return false;
	}
	
	vnl_matrix<double> X_w = X.transpose();
	for(unsigned int i=0; i<X_w.cols(); i++)
	{
		X_w[0][i] *= W[i];
		X_w[1][i] *= W[i];
		X_w[2][i] *= W[i];
	}

	vnl_matrix<double> M = X_w * Y;
	double Sxx = M[0][0];
	double Sxy = M[0][1];
	double Sxz = M[0][2];
	double Syx = M[1][0];
	double Syy = M[1][1];
	double Syz = M[1][2];
	double Szx = M[2][0];
	double Szy = M[2][1];
	double Szz = M[2][2];

	vnl_matrix<double> N(4, 4);
	N[0][0] = Sxx+Syy+Szz; N[0][1] = Syz-Szy;	  N[0][2] = Szx-Sxz;	  N[0][3] = Sxy-Syx;
	N[1][0] = N[0][1];	   N[1][1] = Sxx-Syy-Szz; N[1][2] = Sxy+Syx;	  N[1][3] = Szx+Sxz;
	N[2][0] = N[0][2];	   N[2][1] = N[1][2];	  N[2][2] = -Sxx+Syy-Szz; N[2][3] = Syz+Szy;
	N[3][0] = N[0][3];	   N[3][1] = N[1][3];	  N[3][2] = N[2][3];	  N[3][3] = -Sxx-Syy+Szz;

	vnl_symmetric_eigensystem<double> eig(N);
	vnl_vector<double> q_ = eig.get_eigenvector(3);

	vnl_quaternion<double> q(q_[1], q_[2], q_[3], q_[0]);
	
	vgl_rotation_3d<double> R_(q);

	R = R_.as_matrix();

	return true;
}

bool calcTransform( vnl_matrix<double> &X, vnl_matrix<double> &Y,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T)
{
	if( X.rows() != Y.rows() ||
		X.cols() != 3 ||
		Y.cols() != 3 ||
		X.rows() < 2)
	{
		printf("Error: the input data is invalid!\n");
		return false;
	}

	vnl_vector_fixed<double, 3> x_m = get_mean_vector(X, 1);
	vnl_vector_fixed<double, 3> y_m = get_mean_vector(Y, 1);
	sub_vec_from_mat(X, x_m, 1);
	sub_vec_from_mat(Y, y_m, 1);

	calcRotationMat(X, Y, R);

	T = y_m - R*x_m;
	return true;
}

bool calcTransform( S3DPointMatchSet const& match_set_3d,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T)
{
	int match_size = match_set_3d.size();
	if( match_size<=2)
	{
		printf("Error: two few matched points to compute transformation!\n");
		return false;
	}

	vnl_matrix<double> X(match_size, 3);
	vnl_matrix<double> Y(match_size, 3);

	for(int i=0; i<match_size; i++)
	{
		X.set_row(i, match_set_3d.points_1[i]);
		Y.set_row(i, match_set_3d.points_2[i]);
	}
	return calcTransform(X, Y, R, T);
}

bool calcTransform( vector< vnl_vector_fixed<double, 3> > const& points_1,
					vector< vnl_vector_fixed<double, 3> > const& points_2,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T)
{
	if( points_1.size() != points_2.size() ||
		points_1.size() <= 2)
	{
		printf("Error: sizes do not match or two few matched points to compute transformation!\n");
		return false;
	}

	int match_size = points_1.size();
	vnl_matrix<double> X(match_size, 3);
	vnl_matrix<double> Y(match_size, 3);

	for(int i=0; i<match_size; i++)
	{
		X.set_row(i, points_1[i]);
		Y.set_row(i, points_2[i]);
	}
	return calcTransform(X, Y, R, T);
}

//===============================================================================
//Implementation for:
//					Camera Parameter Related Functions
//
//===============================================================================
bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam, 
									  vnl_vector_fixed<double, 5> &distort_coef)
{
	if( cam_veiw == NULL )
		return false;
	vnl_matrix_fixed<double, 3, 3> intrinsics;
	intrinsics.set(&(cam_veiw->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> R;
	R.set(&(cam_veiw->R[0][0]));
	vnl_vector_fixed<double, 3> T;
	T.set(&(cam_veiw->T[0]));
	set_calibration_matrix(cam, intrinsics);
	set_camera_pose(cam, R, T);

	distort_coef.set(&(cam_veiw->Radial[0]));
	return true;
}

bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam, 
									  vnl_vector<double> &distort_coef)
{
	if( cam_veiw == NULL )
		return false;
	vnl_matrix_fixed<double, 3, 3> intrinsics;
	intrinsics.set(&(cam_veiw->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> R;
	R.set(&(cam_veiw->R[0][0]));
	vnl_vector_fixed<double, 3> T;
	T.set(&(cam_veiw->T[0]));
	set_calibration_matrix(cam, intrinsics);
	set_camera_pose(cam, R, T);

	distort_coef.set_size(5);
	distort_coef.set(&(cam_veiw->Radial[0]));
	return true;
}

bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam)
{
	if( cam_veiw == NULL )
		return false;
	vnl_matrix_fixed<double, 3, 3> intrinsics;
	intrinsics.set(&(cam_veiw->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> R;
	R.set(&(cam_veiw->R[0][0]));
	vnl_vector_fixed<double, 3> T;
	T.set(&(cam_veiw->T[0]));
	set_calibration_matrix(cam, intrinsics);
	set_camera_pose(cam, R, T);
	return true;
}

GCameraView* vpgl_camera_view_to_GCameraView( vpgl_perspective_camera<double> const&cam, 
											  vnl_vector_fixed<double, 5> const&distort_coef,
											  int width, int height)
{
	vnl_matrix_fixed<double, 3, 3> intrinsic = cam.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	GCameraView *cam_view = new GCameraView(height, width);
	cam_view->SetCalibrationMatrix(intrinsic.data_block(), R.data_block(), T.data_block(), distort_coef.data_block());

	return cam_view;
}

void copy_extrinsic_from_vpgl_camera_view_to_GCameraView(vpgl_perspective_camera<double> const&cam, GCameraView *cam_view)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	cam_view->SetExtrinsics(R.data_block(), T.data_block());
}

bool SaveCalibrationDataWithSize( const char* filename, 
								  vpgl_perspective_camera<double> const& cam,
								  vnl_vector_fixed<double, 5> const&distortVec, 
								  int width, int height)
{
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	return SaveCalibrationDataWithSize(filename, K, distortVec, R, T, width, height);
}

bool SaveCalibrationDataWithSize(const char* filename, 
								 vnl_matrix_fixed<double, 3, 3> const&intrinsics, 
								 vnl_vector_fixed<double, 5> const&distortVec, 
								 vnl_matrix_fixed<double, 3, 3> const&rotMat, 
								 vnl_vector_fixed<double, 3> const&transVec,
								 int width, int height)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if(!fp)
	{
		printf("Cannot open the calibration file <%s> to write.\n", filename);
		return false;
	}

	fprintf_s(fp, "%d ", width);
	fprintf_s(fp, "%d", height);
	fprintf_s(fp, "\n");

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			fprintf_s(fp, "%15.15f ", intrinsics[i][j]);
		}
		fprintf_s(fp, "\n");
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			fprintf_s(fp, "%15.15f ", rotMat[i][j]);
		}
		fprintf_s(fp, "\n");
	}

	for(int i=0; i<3; i++)
	{
		fprintf_s(fp, "%15.15f ", transVec[i]);
		fprintf_s(fp, "\n");
	}

	for(int i=0; i<5; i++)
	{
		fprintf_s(fp, "%15.15f ", distortVec[i]);
		fprintf_s(fp, "\n");
	}

	fclose(fp);

	return true;
}
bool LoadCalibrationDataWithSize(const char* filename, 
								 vnl_matrix_fixed<double, 3, 3> &intrinsics, 
								 vnl_vector_fixed<double, 5> &distortVec, 
								 vnl_matrix_fixed<double, 3, 3> &rotMat, 
								 vnl_vector_fixed<double, 3> &transVec, 
								 int &width, int &height)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "r");
	if(!fp)
	{
		printf("Cannot open the calibraion file.\n");
		return false;
	}

	int numread = 0;

	double temp;
	numread = fscanf_s(fp, "%lf", &temp);
	if(numread != 1)
		return false;
	width = ROUND(temp);
	numread = fscanf_s(fp, "%lf", &temp);
	if(numread != 1)
		return false;
	height = ROUND(temp);

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			numread = fscanf_s(fp,"%lf",&temp);
			if(numread != 1)
				return false;
			intrinsics[i][j] = temp;
		}
	}

	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			numread = fscanf_s(fp,"%lf",&temp);
			if(numread != 1)
				return false;
			rotMat[i][j] = temp;
		}
	}

	for(int i=0;i<3;i++)
	{
		numread = fscanf_s(fp,"%lf",&temp);
		if(numread != 1)
			return false;
		transVec[i] = temp;
	}

	distortVec.fill(0.0);
	for(int i=0; i<5; i++)
		fscanf_s(fp,"%lf",&distortVec[i]);

	fclose(fp);
	return true;
}

void set_camera_pose( vpgl_perspective_camera<double> &cam, 
					  vnl_vector_fixed<double, 3> const& cam_cen,
					  vnl_vector_fixed<double, 3> const& lookat_pt,
					  vnl_vector_fixed<double, 3> const& up)
{
	vnl_vector_fixed<double, 3> dz = lookat_pt-cam_cen;
	dz.normalize();
	vnl_vector_fixed<double, 3> dy = -up; //axis y points downwards
	dy.normalize();
	vnl_vector_fixed<double, 3> dx = cross_product(dy, dz);
	dx.normalize();
	dy = cross_product(dz, dx);

	vnl_matrix_fixed<double, 3, 3> R;
	R.set_row(0, dx);
	R.set_row(1, dy);
	R.set_row(2, dz);
	vnl_vector_fixed<double, 3> T = -R*cam_cen;
	set_camera_pose(cam, R, T);
}

void set_camera_pose(GCameraView &cam,
	vnl_vector_fixed<double, 3> const& cam_cen,
	vnl_vector_fixed<double, 3> const& lookat_pt,
	vnl_vector_fixed<double, 3> const& up)
{
	vnl_vector_fixed<double, 3> dz = lookat_pt - cam_cen;
	dz.normalize();
	vnl_vector_fixed<double, 3> dy = -up; //axis y points downwards
	dy.normalize();
	vnl_vector_fixed<double, 3> dx = cross_product(dy, dz);
	dx.normalize();
	dy = cross_product(dz, dx);

	vnl_matrix_fixed<double, 3, 3> R;
	R.set_row(0, dx);
	R.set_row(1, dy);
	R.set_row(2, dz);
	vnl_vector_fixed<double, 3> T = -R*cam_cen;
	set_camera_pose(cam, R, T);
}

void camera_pose_to_transf( vpgl_perspective_camera<double> const& cam1, 
						     vpgl_perspective_camera<double> const& cam2, 
							 vnl_matrix_fixed<double,3,3> &dR, 
							 vnl_vector_fixed<double,3> &dT)
{
	vnl_matrix_fixed<double, 3, 3> R1 = cam1.get_rotation().as_matrix();
	vgl_vector_3d<double> T1_ = cam1.get_translation();
	vnl_vector_fixed<double, 3> T1(T1_.x(), T1_.y(), T1_.z());
	vnl_matrix_fixed<double, 3, 3> R2 = cam2.get_rotation().as_matrix();
	vgl_vector_3d<double> T2_ = cam2.get_translation();
	vnl_vector_fixed<double, 3> T2(T2_.x(), T2_.y(), T2_.z());

	//transform points in 1st camera space to points in 2nd cmaera space
	dR = R2 * R1.transpose();
	dT = -dR*T1 + T2;
}

void camera_pose_to_transf( GCameraView const* cam1, 
						    GCameraView const* cam2, 
							vnl_matrix_fixed<double,3,3> &dR, 
							vnl_vector_fixed<double,3> &dT)
{
	vnl_matrix_fixed<double, 3, 3> R1(&(cam1->R[0][0]));
	vnl_vector_fixed<double, 3> T1(&(cam1->T[0]));
	vnl_matrix_fixed<double, 3, 3> R2(&(cam2->R[0][0]));
	vnl_vector_fixed<double, 3> T2(&(cam2->T[0]));

	//transform points in 1st camera space to points in 2nd cmaera space
	dR = R2 * R1.transpose();
	dT = -dR*T1 + T2;
}

void transf_to_camera_pose( vpgl_perspective_camera<double> const& cam1, 
							 vnl_matrix_fixed<double,3,3> const& dR, 
							 vnl_vector_fixed<double,3> const& dT,
							 vpgl_perspective_camera<double> &cam2
							 )
{
	vnl_matrix_fixed<double, 3, 3> R1 = cam1.get_rotation().as_matrix();
	vgl_vector_3d<double> T1_ = cam1.get_translation();
	vnl_vector_fixed<double, 3> T1(T1_.x(), T1_.y(), T1_.z());
	vnl_matrix_fixed<double, 3, 3> R2 = dR * R1;
	vnl_vector_fixed<double, 3> T2 = dT + dR*T1;

	set_camera_pose(cam2, R2, T2);
}

void transf_to_camera_pose( GCameraView const*cam1,
						    vnl_matrix_fixed<double, 3, 3> const&dR,
						    vnl_vector_fixed<double, 3> const&dT, 
						    GCameraView *cam2)
{
	vnl_matrix_fixed<double, 3, 3> R1;
	vnl_vector_fixed<double, 3> T1;
	get_camera_pose(*cam1, R1, T1);
	vnl_matrix_fixed<double, 3, 3> R2 = dR * R1;
	vnl_vector_fixed<double, 3> T2 = dT + dR*T1;

	set_camera_pose(*cam2, R2, T2);
}

/*
 */
void get_camera_image_plane( vpgl_perspective_camera<double> &cam,
							vnl_matrix_fixed<double, 4, 3> &image_plane,
							double d)
{
	vnl_matrix_fixed<double, 3, 3> K = cam.get_calibration().get_matrix();
	double f = (K[0][0]+K[1][1])/2.0;
	double cx = K[0][2];
	double cy = K[1][2];

	image_plane[0][0] = -cx/f * d;
	image_plane[0][1] = -cy/f * d;
	image_plane[0][2] = d;

	//the image width need to be known to get a more accurate corner point
	image_plane[1][0] = cx/f * d;
	image_plane[1][1] = -cy/f * d;
	image_plane[1][2] = d;

	image_plane[2][0] = cx/f * d;
	image_plane[2][1] = cy/f * d;
	image_plane[2][2] = d;

	image_plane[3][0] = -cx/f * d;
	image_plane[3][1] = cy/f * d;
	image_plane[3][2] = d;

	vgl_point_3d<double> cam_center_ = cam.get_camera_center();
	vnl_vector_fixed<double, 3> cam_center(cam_center_.x(), cam_center_.y(), cam_center_.z());

	vnl_matrix_fixed<double, 3, 3> R = cam.get_rotation().as_matrix();

	image_plane = image_plane*R;

	for(int i=0; i<4; i++)
	{
		for(int j=0; j<3; j++)
		{
			image_plane[i][j] += cam_center[j];
		}
	}
}

/* save and load camera poses
 */
bool save_camera_poses( const char* filename, 
						vector<vpgl_perspective_camera<double>*> &camera_pose_list)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for writing!\n", filename);
		return false;
	}

	fprintf(file, "#Camera Poses File, v1.0. [camera num, R1, T1, ..., Rn, Tn.]\n");

	fprintf(file, "%zd\n", camera_pose_list.size());
	
	for(unsigned int i=0; i<camera_pose_list.size(); i++)
	{
		vpgl_perspective_camera<double> *cam = camera_pose_list[i];
		vnl_matrix_fixed<double, 3, 3> R = cam->get_rotation().as_matrix();
		vgl_vector_3d<double> T = cam->get_translation();

		for(int j=0; j<3; j++)
			fprintf(file, "%15.15f %15.15f %15.15f\n", R[j][0], R[j][1], R[j][2]);
		fprintf(file, "%15.15f %15.15f %15.15f\n\n", T.x(), T.y(), T.z());
	}

	fclose(file);
	return true;
}

bool load_camera_poses(const char *filename, 
						vector<vpgl_perspective_camera<double>*> &camera_pose_list,
						bool bInMeters)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading!\n", filename);
		return false;
	}

	//clear the list
	camera_pose_list.clear();

	char line[1000];

	//read the header
	fgets(line, 1000, file);

	int cam_num = 0;
	fscanf_s(file, "%d", &cam_num);

	for(int i=0; i<cam_num; i++)
	{
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;

		for(int j=0; j<3; j++)
			fscanf_s(file, "%lf %lf %lf", &(R[j][0]), &(R[j][1]), &(R[j][2]));
		fscanf_s(file, "%lf %lf %lf", &(T[0]), &(T[1]), &(T[2]));

		if( vnl_det<double>(R) < 0)
			R = R*-1.0;

		if( bInMeters )
			T = T*100.0;

		vpgl_perspective_camera<double> *cam = new vpgl_perspective_camera<double>();
		set_camera_pose(*cam, R, T);
		camera_pose_list.push_back(cam);		

		if( feof(file) )
			break;
	}

	fclose(file);
	return true;
}

bool save_camera_poses( const char* filename, 
						vector< vpgl_perspective_camera<double> > &camera_pose_list)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for writing!\n", filename);
		return false;
	}

	fprintf(file, "#Camera Poses File, v1.0. [camera num, R1, T1, ..., Rn, Tn.]\n");

	fprintf(file, "%zd\n", camera_pose_list.size());
	
	for(unsigned int i=0; i<camera_pose_list.size(); i++)
	{
		vpgl_perspective_camera<double> &cam = camera_pose_list[i];
		vnl_matrix_fixed<double, 3, 3> R = cam.get_rotation().as_matrix();
		vgl_vector_3d<double> T = cam.get_translation();

		for(int j=0; j<3; j++)
			fprintf(file, "%15.15f %15.15f %15.15f\n", R[j][0], R[j][1], R[j][2]);
		fprintf(file, "%15.15f %15.15f %15.15f\n\n", T.x(), T.y(), T.z());
	}

	fclose(file);
	return true;
}

bool load_camera_poses( const char *filename, 
						vector< vpgl_perspective_camera<double> > &camera_pose_list,
						bool bInMeters)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading!\n", filename);
		return false;
	}

	//clear the list
	camera_pose_list.clear();

	char line[1000];

	//read the header
	fgets(line, 1000, file);

	int cam_num = 0;
	fscanf_s(file, "%d", &cam_num);

	for(int i=0; i<cam_num; i++)
	{
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;

		for(int j=0; j<3; j++)
			fscanf_s(file, "%lf %lf %lf", &(R[j][0]), &(R[j][1]), &(R[j][2]));
		fscanf_s(file, "%lf %lf %lf", &(T[0]), &(T[1]), &(T[2]));

		if( vnl_det<double>(R) < 0)
			R = R*-1.0;

		if( bInMeters )
			T = T*100.0;

		vpgl_perspective_camera<double> cam;
		set_camera_pose(cam, R, T);
		camera_pose_list.push_back(cam);		

		if( feof(file) )
			break;
	}

	fclose(file);
	return true;
}

/* save and load camera initilization status
 */
bool save_camera_poses_init_status( const char *filename, 
									vector<bool> &camera_poses_init_stutus)
{
	FILE *file = fopen(filename, "w");
	if( file == NULL )
	{
		printf("Cannot Open file %s for writing!\n", filename);
		return false;
	}

	fprintf(file, "#Camera Pose Initialization Status, v1.0. [1/0, 1/0, ...]\n");

	for(unsigned int i=0; i<camera_poses_init_stutus.size(); i++)
	{
		fprintf(file, "%d ", int(camera_poses_init_stutus[i]));
	}

	fclose(file);
	return true;
}

bool load_camera_poses_init_status( const char *filename, 
									vector<bool> &camera_poses_init_stutus)
{
	FILE *file = fopen(filename, "r");
	if( file == NULL )
	{
		printf("Cannot Open file %s for reading!\n", filename);
		return false;
	}

	char line[1000];

	//read the header
	fgets(line, 1000, file);

	camera_poses_init_stutus.clear();

	while(1)
	{
		int status = -1;
		int num = fscanf_s(file, "%d", &status);
		if( num != 1)
			break;

		if( status == 1)
			camera_poses_init_stutus.push_back(true);
		else
			camera_poses_init_stutus.push_back(false);
	}

	fclose(file);
	return true;
}

bool load_intrinsics_ini( const char* filename, 
						  vnl_matrix_fixed<double, 3, 3> &K,
						  vnl_vector<double> &distort_vec)
{
	iniFile file(filename);
	int len;
	double* intrinsics = file.getDoubleList(L"Intrinsics", L"Matrix", len);
	K.set(intrinsics);
	delete [] intrinsics;

	double* distort_coef = file.getDoubleList(L"Intrinsics", L"Distortion", len);
	distort_vec.set_size(len);
	distort_vec.set(distort_coef);
	delete [] distort_coef;

	return true;
}

bool load_intrinsics_ini( const char* filename, 
						  vnl_matrix_fixed<double, 3, 3> &K,
						  vnl_vector<double> &distort_vec,
						  int &width,
						  int &height)
{
	iniFile file(filename);
	int len;
	double* intrinsics = file.getDoubleList(L"Intrinsics", L"Matrix", len);
	K.set(intrinsics);
	delete [] intrinsics;

	double* distort_coef = file.getDoubleList(L"Intrinsics", L"Distortion", len);
	distort_vec.set_size(len);
	distort_vec.set(distort_coef);
	delete [] distort_coef;

	int* size = file.getIntList(L"Intrinsics", L"ImageSize", len);
	if( len >= 2)
	{
		width = size[0];
		height = size[1];
	}
	else
	{
		width = -1;
		height = -1;
	}
	delete [] size;

	return true;
}
