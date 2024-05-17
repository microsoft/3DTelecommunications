#pragma once
#ifndef __BASIC_GEOMETRY_H__
#define __BASIC_GEOMETRY_H__
#include "UtilVnlMatrix.h"
#include "basic_structure.h"
#include <vpgl\vpgl_perspective_camera.h>
#include <vgl\algo\vgl_rotation_3d.h>
#include "CameraView.h"

#ifndef M_PI
#define M_PI 3.1415926
#endif
struct RectBoundary
{
public:
	RectBoundary(int x_ = -1, int y_ = -1, int width_ = 0, int height_ = 0)
		: x(x_), y(y_), width(width_), height(height_)
	{
		;
	}
public:
	int x;
	int y;
	int width;
	int height;
};

/* p1, p2, p3, p4 can be in clockwise or counter-clockwise
*/
struct Quadrilateral2D
{
public:
	Quadrilateral2D(vgl_point_2d<double> &p1_, vgl_point_2d<double> &p2_,
		vgl_point_2d<double> &p3_, vgl_point_2d<double> &p4_)
		: p1(p1_), p2(p2_), p3(p3_), p4(p4_)
	{
		;
	}
	Quadrilateral2D(RectBoundary const&rect)
	{
		p1.x() = rect.x;
		p1.y() = rect.y;
		p2.x() = rect.x + rect.width;
		p2.y() = rect.y;
		p3.x() = rect.x + rect.width;
		p3.y() = rect.y + rect.height;
		p4.x() = rect.x;
		p4.y() = rect.y + rect.height;
	}

	Quadrilateral2D()
		:p1(0.0, 0.0), p2(0.0, 0.0), p3(0.0, 0.0), p4(0.0, 0.0)
	{
		;
	}

public:
	vgl_point_2d<double> p1;
	vgl_point_2d<double> p2;
	vgl_point_2d<double> p3;
	vgl_point_2d<double> p4;
};

bool test_line_segments_intersect(vgl_point_2d<double> l_p1, vgl_point_2d<double> l_p2,
								  vgl_point_2d<double> r_p1, vgl_point_2d<double> r_p2);

//return true, if p is inside quad. assuming the quad is convex
bool is_point_inside_quad(vgl_point_2d<double> &p, Quadrilateral2D &quad);
//return true, if they overlap.
//assuming both quads are convexes
bool test_quads_overlap(Quadrilateral2D &quad1, Quadrilateral2D &quad2);

// fit a 2D polynomial function and find the maxima
// vals contains the values around <x_center, y_center>
//   e.g., val(0, 0) is the value at <x_center-step, y_center-step> 
bool find_maxima_by_polynomial_fitting_2D( double x_center, double y_center, double step,
										   vnl_matrix_fixed<double, 3, 3> const& vals, 
										   double &x_maxima, double &y_maxima, double &val_maxima);

//plane equation: x .* normal - d = 0. normal point against origin
// scalar will scale point coordinate to be around 1.0
void plane_fitting( vector< vnl_vector_fixed<double, 3> > &points, 
				    vnl_vector_fixed<double, 3> &normal,
				    double &d,
					double scalar = 1000.0,
					double *dist_mean = NULL, //point to plane mean distance 
					double *dist_std = NULL   //points to plane distance std
					);

//load calibration file
bool load_calibration_file_with_size( char const* file_name,
									  vnl_matrix_fixed<double, 3, 3> &K, 
									  vnl_matrix_fixed<double, 3, 3> &R,
									  vnl_vector_fixed<double, 3> &T,
									  vnl_vector_fixed<double, 5> &radial_distort);
bool load_calibration_file_with_size( char const* file_name,
									  vpgl_perspective_camera<double> &cam,
									  vnl_vector_fixed<double, 5> &radial_distort);


//check if <v1 v2, v3> is in clockwise order when look at from the direction lookat
template<class T>
inline bool is_clockwise_tri(T lookat[3], T v1[3], T v2[3], T v3[3]);

//theta and phi are in radius 
void sphere_coord_to_normal(double theta, double phi, vnl_vector_fixed<double, 3> &n);
//theta--[0, pi]; phi--[-pi, pi]
void normal_to_sphere_coord(vnl_vector_fixed<double, 3> n, double &theta, double &phi);

//matrix orthogonalize
inline void matrix_orthogonalize(vnl_matrix_fixed<double, 3, 3> &mat)
{
	vnl_svd<double> svd_solver(mat);
	mat = svd_solver.U() * svd_solver.V().transpose();
}

/* return rotation angle in degrees [0, 180]
 */
double rotation_angle(vnl_matrix_fixed<double, 3, 3> &R);

/* rodrigues vector, rotation matrix, and euler angles related functions
 */
inline void complementary_rodrigues_vector(vnl_vector_fixed<double, 3> const&in, 
									vnl_vector_fixed<double, 3> &out)
{
	double theta = in.two_norm();
	
	double i;
	double f = std::modf(theta/(2*M_PI), &i); //i--integral part; f---fraction part
	if( i != 0.0 ) //theta > 0
		theta = f*2.0*M_PI;

	out = in;
	out.normalize();
	out = -out*(2*M_PI-theta);
}

template<class T>
inline void rodrigues_to_matrix(vnl_vector_fixed<T, 3> const&rod, vnl_matrix_fixed<T, 3, 3> &mat)
{
	vgl_rotation_3d<T> rotation(rod);
	mat = rotation.as_matrix();
}

template<class T>
inline void matrix_to_rodrigues(vnl_matrix_fixed<T, 3, 3> const&mat, vnl_vector_fixed<T, 3> &rod)
{
	vgl_rotation_3d<T> rotation(mat);
	rod = rotation.as_rodrigues();
}

template<class T>
inline void rodrigues_to_euler(vnl_vector_fixed<T, 3> const&rod, vnl_vector_fixed<T, 3> &euler)
{
	vgl_rotation_3d<T> rotation(rod);
	euler = rotation.as_euler_angles();
}

// the rodrigues verctor is not totally determined, e.g. it can be the complimentary vector
template<class T>
inline void euler_to_rodrigues(vnl_vector_fixed<T, 3> const&euler, vnl_vector_fixed<T, 3> &rod)
{
	vgl_rotation_3d<T> rotation(euler[0], euler[1], euler[2]);
	rod = rotation.as_rodrigues();
}

template<class T>
inline void matrix_to_euler(vnl_matrix_fixed<T, 3, 3> const&rot_mat, vnl_vector_fixed<T, 3> &euler)
{
	vgl_rotation_3d<T> rotation(rot_mat);
	euler = rotation.as_euler_angles();
}

template<class T>
inline void euler_to_matrix(vnl_vector_fixed<T, 3> const&euler, vnl_matrix_fixed<T, 3, 3> &rot_mat)
{
	vgl_rotation_3d<T> rotation(euler[0], euler[1], euler[2]);
	rot_mat = rotation.as_matrix();
}

// alpha in radius;
inline vnl_matrix_fixed<double, 3, 3> rotation_around_axis(vnl_vector_fixed<double, 3> axis, double alpha)
{
	vnl_vector_fixed<double, 3> rod = axis;
	rod.normalize();
	rod *= alpha;

	vnl_matrix_fixed<double, 3, 3> R;
	rodrigues_to_matrix(rod, R);
	return R;
}

/* the corresponding rows of X, Y are 3-d vectors <x, y>, where exist a desiring R satisfying Rx = y;
 *
 * the objective function to *MAXIMIZE* is \sum_i x_i^T * R * y_i, please refere to the following paper 
 * for details, "Closed-form Solution of Absolute Orientation using Unit Quaternions", Journal of Optial
 * Society of America, 1987.
 *
 * Use it with Cautions:
 *		the X and Y should have the same number of rows and the row numbers should at least be *TWO*
 */
bool calcRotationMat( vnl_matrix<double> const&X, vnl_matrix<double> const&Y,
					 vnl_matrix_fixed<double, 3, 3> &R);

bool calcRotationMat( vnl_matrix<double> const&X, vnl_matrix<double> const&Y, vnl_vector<double> const&W,
					 vnl_matrix_fixed<double, 3, 3> &R);

/* Y = RX + T
 *
 * ***********************************************************************
 *    There is a similar routine in "visual_feature_matching.h" with
 *    the same function but different implementation. 
 *    It seems they give the same results, but I did not test them exhaustively
 * ***********************************************************************
 * 
 * Note that X and Y will be centralized after computation
 */
bool calcTransform( vnl_matrix<double> &X, vnl_matrix<double> &Y,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T);
bool calcTransform( S3DPointMatchSet const& match_set_3d,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T);
bool calcTransform( vector< vnl_vector_fixed<double, 3> > const& points_1,
					vector< vnl_vector_fixed<double, 3> > const& points_2,
					vnl_matrix_fixed<double, 3, 3> &R,
					vnl_vector_fixed<double, 3> &T);

/* compute R satifying R*n1 = n2
 */
inline void calcRotationMat( vnl_vector_fixed<double, 3> n1, vnl_vector_fixed<double, 3> n2, 
					  vnl_matrix_fixed<double, 3, 3> &R)
{
	vgl_rotation_3d<double> rot(n1, n2);
	R = rot.as_matrix();	
}

/* angle between n1 and n2 (n1 to n2) about a in right-handed rule. it should lie in (0, 2*pi)
 */
inline double angleAboutAxis( vnl_vector_fixed<double, 3> n1, vnl_vector_fixed<double, 3> n2,
					   vnl_vector_fixed<double, 3> a )
{
	vnl_vector_fixed<double, 3> b1 = n1 - n1*dot_product(n1, a);
	vnl_vector_fixed<double, 3> b2 = n2 - n2*dot_product(n2, a);

	double alpha = angle(b1, b2);

	vnl_vector<double> b3 = vnl_cross_3d(b1, b2);
	if( dot_product(b3, a) > 0 )
		return alpha;
	else
		return 2*M_PI-alpha;
}

/* return angle in radius, it should lie in (0, pi)
 */
template<class T, int N>
double angle_btw_two_vec(vnl_vector_fixed<T, N> normal1, vnl_vector_fixed<T, N> normal2);

//===============================================================================
//
//					Camera Parameter Related Functions
//
//===============================================================================
bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam, 
									  vnl_vector_fixed<double, 5> &distort_coef);
bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam, 
									  vnl_vector<double> &distort_coef);

bool GCameraView_to_vpgl_camera_view( GCameraView const*cam_veiw, 
									  vpgl_perspective_camera<double> &cam);

GCameraView* vpgl_camera_view_to_GCameraView( vpgl_perspective_camera<double> const&cam, 
											  vnl_vector_fixed<double, 5> const&distort_coef,
											  int width, int height);

void copy_extrinsic_from_vpgl_camera_view_to_GCameraView(vpgl_perspective_camera<double> const&cam, GCameraView *cam_view);

bool SaveCalibrationDataWithSize(const char* filename, 
								 vnl_matrix_fixed<double, 3, 3> const&intrinsics, 
								 vnl_vector_fixed<double, 5> const&distortVec, 
								 vnl_matrix_fixed<double, 3, 3> const&rotMat, 
								 vnl_vector_fixed<double, 3> const&transVec,
								 int width, int height);

bool SaveCalibrationDataWithSize( const char* filename, 
								  vpgl_perspective_camera<double> const& cam,
								  vnl_vector_fixed<double, 5> const&distortVec, 
								  int width, int height);

bool LoadCalibrationDataWithSize(const char* filename, 
								 vnl_matrix_fixed<double, 3, 3> &intrinsics, 
								 vnl_vector_fixed<double, 5> &distortVec, 
								 vnl_matrix_fixed<double, 3, 3> &rotMat, 
								 vnl_vector_fixed<double, 3> &transVec, 
								 int &width, int &height);							   

inline void cam_pose_up_dir( vpgl_perspective_camera<double> const&cam,
					  vnl_vector_fixed<double, 3> &up)
{
	vnl_matrix_fixed<double, 3, 3> R = cam.get_rotation().as_matrix();
	up[0] = -R[1][0];
	up[1] = -R[1][1];
	up[2] = -R[1][2];
}

/* X_cam = R * X_wld + T
 */
inline void set_camera_pose( vpgl_perspective_camera<double> &cam,
							 vnl_matrix_fixed<double, 3, 3> const&R,
							 vnl_vector_fixed<double, 3> const&T)
{
	cam.set_rotation( vgl_rotation_3d<double>(R));
	cam.set_translation(vgl_vector_3d<double>(T[0], T[1], T[2]));
}

inline void set_camera_pose( GCameraView &cam,
							 vnl_matrix_fixed<double, 3, 3> const&R,
							 vnl_vector_fixed<double, 3> const&T)
{
	cam.SetExtrinsics(R.data_block(), T.data_block());
}

void set_camera_pose( vpgl_perspective_camera<double> &cam, 
					  vnl_vector_fixed<double, 3> const& cam_cen,
					  vnl_vector_fixed<double, 3> const& lookat_pt,
					  vnl_vector_fixed<double, 3> const& up);
void set_camera_pose(GCameraView &cam,
	vnl_vector_fixed<double, 3> const& cam_cen,
	vnl_vector_fixed<double, 3> const& lookat_pt,
	vnl_vector_fixed<double, 3> const& up);

inline void get_camera_pose( vpgl_perspective_camera<double> const& cam,
					 vnl_matrix_fixed<double, 3, 3> &R,
					 vnl_vector_fixed<double, 3> &T)
{
	R = cam.get_rotation().as_matrix();
	vgl_vector_3d<double> T1_ = cam.get_translation();
	T[0] = T1_.x(); T[1] = T1_.y(); T[2] = T1_.z();
}

inline void get_camera_pose( GCameraView const& cam,
							 vnl_matrix_fixed<double, 3, 3> &R,
							 vnl_vector_fixed<double, 3> &T)
{
	R.set(&(cam.R[0][0]));
	T.set(&(cam.T[0]));
}

inline void get_camera_center( vpgl_perspective_camera<double> const& cam,
							   vnl_vector_fixed<double, 3> &cam_cen)
{
	vgl_homg_point_3d<double> cen_ = cam.camera_center();
	cen_.get_nonhomogeneous(cam_cen[0], cam_cen[1], cam_cen[2]);
}


inline void set_calibration_matrix( vpgl_perspective_camera<double> &cam, 
									vnl_matrix_fixed<double, 3, 3> const& K)
{
	cam.set_calibration(vpgl_calibration_matrix<double>(K));
}
inline void get_calibration_matrix( vpgl_perspective_camera<double> const&cam, 
									vnl_matrix_fixed<double, 3, 3> &K)
{
	K = cam.get_calibration().get_matrix();
}
inline void get_calibration_matrix(GCameraView const&cam,
	vnl_matrix_fixed<double, 3, 3> &K)
{
	K.set(&(cam.K[0][0]));
}

/* set camera to be locating at origin, and set its rotation matrix to be identity
 */
inline void set_initial_camera_pose(vpgl_perspective_camera<double> &cam)
{
	vnl_matrix_fixed<double, 3, 3> R;
	R.set_identity();
	vnl_vector_fixed<double, 3> T(0.0);
	set_camera_pose(cam, R, T);
}

/* get the image plane which represented by a 4 corner points(leaded by up left corner in the clock-wise order)
 */
void get_camera_image_plane( vpgl_perspective_camera<double> &cam,
							vnl_matrix_fixed<double, 4, 3> &image_plane,
							double d = 1.0);

/* calculate the camera pose of 2nd camera, based cam1, R and T
 * [R|T] describes the transformation that turns points in 1st camera space to 2nd camera space
 * the intrinsics of cam2 is kept untouched
 */
void transf_to_camera_pose( vpgl_perspective_camera<double> const&cam1,
						    vnl_matrix_fixed<double, 3, 3> const&R,
						    vnl_vector_fixed<double, 3> const&T, 
						    vpgl_perspective_camera<double> &cam2);
void transf_to_camera_pose( GCameraView const*cam1,
						    vnl_matrix_fixed<double, 3, 3> const&R,
						    vnl_vector_fixed<double, 3> const&T, 
						    GCameraView *cam2);

/* calculate the transformation between cam1 and cam2
 * R and T will transform Point in 1st camera space to 2nd camera space
 * X2 = R*X1 + T;
 */
void camera_pose_to_transf( vpgl_perspective_camera<double> const& cam1,
						    vpgl_perspective_camera<double> const& cam2,
					        vnl_matrix_fixed<double, 3, 3> &R,
						    vnl_vector_fixed<double, 3> &T);
void camera_pose_to_transf( GCameraView const* cam1, 
						    GCameraView const* cam2, 
							vnl_matrix_fixed<double,3,3> &dR, 
							vnl_vector_fixed<double,3> &dT);

//reverse the rigid transformation:
// <R, T> transform X to Y, while <R_inv, T_inv> will transform Y to X
//				R_inv = R^t
//				T_inv = -R^t * T
inline void reverse_transf( vnl_matrix_fixed<double, 3, 3> const&R,
					 vnl_vector_fixed<double, 3> const&T,
					 vnl_matrix_fixed<double, 3, 3> &R_inv,
					 vnl_vector_fixed<double, 3> &T_inv)
{
	R_inv = R.transpose();
	T_inv = -R_inv * T;
}

/*  <dR, dT>: transformtion on the points in the world spaces. The adjustment on
 *  camera extrinsics makes sure that these points have 
 *  the same coodinates in camera space before and after transformation.
 *  Before adjustment: Xcam = R*p + T    ......(1)
 *  After adjustment: Xcam = R'*p' + T'  ......(2)
 *                    where p' = dR*p + dT or p = dR^T(p'-dT)
 *    substitute p = dR^t(p'-dT) to Eq(1), we have
 *                   Xcam = R*dR^t*p'-R*dR^t*dT+T
 *         Thus, R' = R*dR^t; T' = -R'*dT+T
 */
inline void move_camera_along_points( vnl_matrix_fixed<double, 3, 3> const& dR,
									  vnl_vector_fixed<double, 3> const& dT,
									  vpgl_perspective_camera<double> &cam // in and out
									)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> Rn = R*dR.transpose();
	vnl_vector_fixed<double, 3> Tn = -Rn*dT + T;
	set_camera_pose(cam, Rn, Tn);
}
inline void move_camera_along_points( vnl_matrix_fixed<double, 3, 3> const& dR,
									  vnl_vector_fixed<double, 3> const& dT,
									  GCameraView &cam  // in and out
									)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> Rn = R*dR.transpose();
	vnl_vector_fixed<double, 3> Tn = -Rn*dT + T;
	set_camera_pose(cam, Rn, Tn);
}

/*  the movement/transformation on points (in world space) when moving the camera from cam to cam_n
 * the points could be those observed by the camera
 */
inline void points_movement_along_camera( vpgl_perspective_camera<double> const& cam,
										  vpgl_perspective_camera<double> const& cam_n,
										  vnl_matrix_fixed<double, 3, 3> &dR,
										  vnl_vector_fixed<double, 3> &dT)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> Rn;
	vnl_vector_fixed<double, 3> Tn;
	get_camera_pose(cam_n, Rn, Tn);

	dR = Rn.transpose()*R;
	dT = Rn.transpose()*(T-Tn);
}

inline void points_movement_along_camera( GCameraView const& cam,
										  GCameraView const& cam_n,
										  vnl_matrix_fixed<double, 3, 3> &dR,
										  vnl_vector_fixed<double, 3> &dT)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> Rn;
	vnl_vector_fixed<double, 3> Tn;
	get_camera_pose(cam_n, Rn, Tn);

	dR = Rn.transpose()*R;
	dT = Rn.transpose()*(T-Tn);
}

template<class T>
vnl_vector_fixed<T, 3> camera_projection(vpgl_perspective_camera<double> const&cam, vnl_vector_fixed<T, 3> const& X)
{
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);
	
	vnl_vector_fixed<double, 3> ret = K * (R*X + T);
	ret[0] /= ret[2];
	ret[1] /= ret[2];
	return ret;
}

/* original plane parameter  n*X1 + d = 0
   X2 = R*X1+T
   after deformation, plane: n*X2+d = 0
 */
inline void transform_a_plane(vnl_matrix_fixed<double, 3, 3> const& R, vnl_vector_fixed<double, 3> const& T,
	vnl_vector_fixed<double, 3> &n, double &d)
{
	n = R*n;
	d = d - dot_product(n, T);
}

/* intersection of the viewing ray of x with the plane
 * plane equation in camera space: nx+d=0
 */
inline vnl_vector_fixed<double, 3> plane_point_2d_to_3d( vnl_matrix_fixed<double, 3, 3> const&K, 
														 vnl_vector_fixed<double, 3> const& n, double d, double x[2])
{
	vnl_vector_fixed<double, 3> x_h(x[0], x[1], 1.0);
	vnl_matrix_fixed<double, 3, 3> K_inv = vnl_inverse(K);
	vnl_vector_fixed<double, 3> X = K_inv * x_h;
	double lamda = -d / dot_product(n, X);
	return lamda*X;
}

//plane equation in the world space: n * x + d = 0;
// H--transform a point in cam_ref to a point in cam
inline void calc_Homography_PlnSwp( GCameraView const*cam_ref, GCameraView const*cam, 
									vnl_vector_fixed<double, 3> normal_pln, double d, 
									vnl_matrix_fixed<double, 3, 3> &H )
{
	vnl_matrix_fixed<double, 3, 3> K_ref(&(cam_ref->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> K(&(cam->K[0][0]));

	vnl_matrix_fixed<double, 3, 3> R_ref(&(cam_ref->R[0][0]));
	vnl_vector_fixed<double, 3> T_ref(cam_ref->T);
	vnl_matrix_fixed<double, 3, 3> R(&(cam->R[0][0]));
	vnl_vector_fixed<double, 3> T(cam->T);

	vnl_matrix_fixed<double, 3, 3> R_n;
	vnl_vector_fixed<double, 3> T_n;
	R_n = R * R_ref.transpose();
	T_n = T - R_n * T_ref;

	// the plane parameter in the reference camera space
	vnl_vector_fixed<double, 3> normal_pln_n = R_ref * normal_pln;
	double d_n = d - dot_product(T_ref, normal_pln_n);

	vnl_matrix_fixed<double, 3, 3> tmp = outer_product(T_n, normal_pln_n);
	H = K * (R_n - tmp/d_n) * vnl_inverse(K_ref);
}

//plane equation in the world space: n * x + d = 0;
// H--transform a point in cam_ref to a point in cam
inline void calc_Homography_PlnSwp( vpgl_perspective_camera<double> const& cam_ref, vpgl_perspective_camera<double> const&cam, 
									vnl_vector_fixed<double, 3> normal_pln, double d, 
									vnl_matrix_fixed<double, 3, 3> &H )
{
	vnl_matrix_fixed<double, 3, 3> K_ref;
	get_calibration_matrix(cam_ref, K_ref);
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);

	vnl_matrix_fixed<double, 3, 3> R_ref;
	vnl_vector_fixed<double, 3> T_ref;
	get_camera_pose(cam_ref, R_ref, T_ref);
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> R_n;
	vnl_vector_fixed<double, 3> T_n;
	R_n = R * R_ref.transpose();
	T_n = T - R_n * T_ref;

	// the plane parameter in the reference camera space
	vnl_vector_fixed<double, 3> normal_pln_n = R_ref * normal_pln;
	double d_n = d - dot_product(T_ref, normal_pln_n);

	vnl_matrix_fixed<double, 3, 3> tmp = outer_product(T_n, normal_pln_n);
	H = K * (R_n - tmp/d_n) * vnl_inverse(K_ref);
}

//plane equation in the reference camera space: n * x + d = 0;
// H--transform a point in cam_ref to a point in cam
inline void calc_Homography_RefCamPlnSwp( GCameraView const*cam_ref, GCameraView const*cam, 
										  vnl_vector_fixed<double, 3> normal_pln, double d, 
										  vnl_matrix_fixed<double, 3, 3> &H )
{
	vnl_matrix_fixed<double, 3, 3> K_ref(&(cam_ref->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> K(&(cam->K[0][0]));

	vnl_matrix_fixed<double, 3, 3> R_ref(&(cam_ref->R[0][0]));
	vnl_vector_fixed<double, 3> T_ref(cam_ref->T);
	vnl_matrix_fixed<double, 3, 3> R(&(cam->R[0][0]));
	vnl_vector_fixed<double, 3> T(cam->T);

	vnl_matrix_fixed<double, 3, 3> R_n;
	vnl_vector_fixed<double, 3> T_n;
	R_n = R * R_ref.transpose();
	T_n = T - R_n * T_ref;

	vnl_matrix_fixed<double, 3, 3> tmp = outer_product(T_n, normal_pln);
	H = K * (R_n - tmp/d) * vnl_inverse(K_ref);
}

//plane equation in the reference camera space: n * x + d = 0;
// H--transform a point in cam_ref to a point in cam
inline void calc_Homography_RefCamPlnSwp( vpgl_perspective_camera<double> const& cam_ref, vpgl_perspective_camera<double> const& cam, 
										  vnl_vector_fixed<double, 3> normal_pln, double d, 
										  vnl_matrix_fixed<double, 3, 3> &H )
{
	vnl_matrix_fixed<double, 3, 3> K_ref;
	get_calibration_matrix(cam_ref, K_ref);
	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(cam, K);

	vnl_matrix_fixed<double, 3, 3> R_ref;
	vnl_vector_fixed<double, 3> T_ref;
	get_camera_pose(cam_ref, R_ref, T_ref);
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);

	vnl_matrix_fixed<double, 3, 3> R_n;
	vnl_vector_fixed<double, 3> T_n;
	R_n = R * R_ref.transpose();
	T_n = T - R_n * T_ref;

	vnl_matrix_fixed<double, 3, 3> tmp = outer_product(T_n, normal_pln);
	H = K * (R_n - tmp/d) * vnl_inverse(K_ref);
}

/* loading and saving camera pose, the translation vector normally is defined in Centimeters otherwisely stated
*/
bool save_camera_poses( const char* filename, 
						vector<vpgl_perspective_camera<double>*> &camer_pose_list);
bool load_camera_poses( const char* filename, 
						vector<vpgl_perspective_camera<double>*> &camer_pose_list,
						bool bInMeters = false);

bool save_camera_poses( const char* filename, 
							   vector< vpgl_perspective_camera<double> > &camer_pose_list);
bool load_camera_poses( const char* filename, 
						vector< vpgl_perspective_camera<double> > &camer_pose_list,
						bool bInMeters = false);

bool save_camera_poses_init_status( const char* file,
										   vector<bool> &camera_poses_init_stutus);	
bool load_camera_poses_init_status( const char* file,
										   vector<bool> &camera_poses_init_stutus);	

bool load_intrinsics_ini( const char* filename, 
						  vnl_matrix_fixed<double, 3, 3> &K,
						  vnl_vector<double> &distort_vec);

bool load_intrinsics_ini( const char* filename, 
						  vnl_matrix_fixed<double, 3, 3> &K,
						  vnl_vector<double> &distort_vec,
						  int &width,
						  int &height);

#include "basic_geometry.hpp"
#endif