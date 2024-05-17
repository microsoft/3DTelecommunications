#ifndef __VISUAL_FEATURE_MATCHING_H__
#define __VISUAL_FEATURE_MATCHING_H__
#include <brl/bseg/bapl/bapl_lowe_keypoint_sptr.h>
#include <brl/bseg/bapl/bapl_keypoint_extractor.h>
#include <brl/bseg/bapl/bapl_dense_sift.h>

#include <vpgl/vpgl_essential_matrix.h>
#include <vpgl/vpgl_fundamental_matrix.h>
//#include <vpgl/algo/vpgl_fm_compute_ransac.h>
#include <vpgl/algo/vpgl_fm_compute_8_point.h>

#include <mvl/FMatrixComputeRANSAC.h>
#include "distortion_correction.h"
#include <cul/bundler/bundler.h>
#include <cul/bundler/bundler_inters.h>
#include "basic_structure.h"
#include <vgl/algo/vgl_homg_operators_2d.h>

#include "track_extraction_basic.h"
#include "basic_geometry.h"

//visual feature
//typedef vpgl_bundler_inters_feature SImageVisualFeature;
typedef bundler_inters_feature_sptr SImageVisualFeature_sptr;

struct SImageVisualFeature : public bundler_inters_feature
{
	SImageVisualFeature(int row=0.0, int col=0.0, double scale_=0.0, double orientation_=0.0)
	: bundler_inters_feature(row, col, vnl_vector<double>(), NULL, -1),
		scale(scale_),
		  orientation(orientation_)
	{
	}
	double scale;
	double orientation;
	double x(){ return this->point.y(); }
	double y(){ return this->point.x(); }

};

//visual feature set
typedef bundler_inters_image SImageVisualFeatureSet;
typedef bundler_inters_image_sptr SImageVisualFeatureSet_sptr;

//two matched visual feature sets
typedef bundler_inters_match_set SImageVisualFeatureMatchSet;

class CFundamentalMatrix : public vpgl_fundamental_matrix<double>
{
public:
	CFundamentalMatrix() {};
	CFundamentalMatrix( const vpgl_proj_camera<double>& cr,
						const vpgl_proj_camera<double>& cl )
						: vpgl_fundamental_matrix<double>(cr, cl)
	{;}
	CFundamentalMatrix( const vnl_matrix_fixed<double,3,3>& F )
		: vpgl_fundamental_matrix<double>(F)
	{;}

public:
	double get_residual(vgl_point_2d<double> &pr, vgl_point_2d<double> &pl)
	{
	/*	vgl_homg_line_2d<double> line_l = this->l_epipolar_line(vgl_homg_point_2d<double>(pr));
		vgl_homg_line_2d<double> line_r = this->r_epipolar_line(vgl_homg_point_2d<double>(pl));
		return (vgl_homg_operators_2d<double>::perp_dist_squared( vgl_homg_point_2d<double>(pl), line_l) +  
				vgl_homg_operators_2d<double>::perp_dist_squared( vgl_homg_point_2d<double>(pr), line_r)); */
		vnl_matrix_fixed<double, 3, 3> F = this->get_matrix();
		vnl_vector_fixed<double, 3> p_lf(pl.x(), pl.y(), 1.0);
		vnl_vector_fixed<double, 3> p_rt(pr.x(), pr.y(), 1.0);
		vnl_vector_fixed<double, 3> line_lf = F * p_rt;
		vnl_vector_fixed<double, 3> line_rt = p_lf * F;
		double res = std::abs(dot_product( p_lf, line_lf));
		return 0.5*(res/std::sqrt(line_lf(0)*line_lf(0)+line_lf(1)*line_lf(1))+res/std::sqrt(line_rt(0)*line_rt(0)+line_rt(1)*line_rt(1)));
	}
	double get_residual(vnl_vector_fixed<double, 2> &pr, vnl_vector_fixed<double, 2> &pl)
	{
		return get_residual( vgl_point_2d<double>(pr.data_block()), vgl_point_2d<double>(pl.data_block()));
	}

public:
	/*test code*/
	void printMatrix()
	{
		printf("\n=====Fundamental Mat======\n");
		cout<<(*this);
	}
};

typedef vpgl_essential_matrix<double> CEssentialMatrix;
//class CEssentialMatrix : public vpgl_essential_matrix<double>
//{
//public:
//	CEssentialMatrix(const vnl_matrix_fixed<double, 3, 3> &E)
//		: vpgl_essential_matrix(E)
//	{;}
//};
/* input: one image
 */

// bigger keypoint_curve_ratio results with more features extracted
SImageVisualFeatureSet_sptr visual_feature_extraction( vil_image_resource_sptr &img, 
													   double keypoint_curve_ratio = 10.0);
SImageVisualFeatureSet_sptr visual_feature_extraction( const char* filename, 
													   double keypoint_curve_ratio = 10.0);
SImageVisualFeatureSet_sptr visual_feature_extraction( cv::Mat const& img, 
													   double keypoint_curve_ratio = 10.0, bool bHistEq = false);
SImageVisualFeatureSet_sptr visual_feature_extraction( vil_image_view<vxl_byte> &img, 
													   double keypoint_curve_ratio = 10.0);

/* remove the lens distortion
 */
void undistort_visual_feature_points( SImageVisualFeatureSet_sptr &feature_set, 
									  vnl_matrix_fixed<double, 3, 3> &K,
									  vnl_vector<double> &distort_coef);
void distort_visual_feature_points( SImageVisualFeatureSet_sptr &feature_set,
									vnl_matrix_fixed<double, 3, 3> &K,
									vnl_vector<double> &distort_coef);

/* match the features and do refinement by fitting a fundamental matrix
 * out- match_set, fundamental_mat, out_good_match_ratio
 */
bool visual_feature_matching_and_refining( SImageVisualFeatureSet_sptr &feature_set_1, 
										   SImageVisualFeatureSet_sptr &feature_set_2,
										   SImageVisualFeatureMatchSet &match_set,
										   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
										   int &num_before_refinement,
										   double min_dist_ratio_init_matching = 0.7,
										   int min_inliers = 16,
										   double outlier_threshold = 5.0,
										   double probability_good_f_mat = 0.99,
										   double max_outlier_frac = 0.5,
	 									   bool recompute_after_ransac = true
										   );
//save as visual_feature_matching_and_refining except that return PairwiseMatchIdxSet
bool visual_feature_matching_and_refining2( SImageVisualFeatureSet_sptr &feature_set_1, 
										   SImageVisualFeatureSet_sptr &feature_set_2,
										   PairwiseMatchIdxSet &match_set_idx,
										   SImageVisualFeatureMatchSet &match_set_pts,
										   vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
										   int &num_before_refinement,
										   double min_dist_ratio_init_matching = 0.7,
										   int min_inliers = 16,
										   double outlier_threshold = 5.0,
										   double probability_good_f_mat = 0.99,
										   double max_outlier_frac = 0.5,
	 									   bool recompute_after_ransac = true,
										   int frmIdx_1 = -1,
										   int frmIdx_2 = -1
										   );

//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits< vector< vnl_vector_fixed<int, 2>> > : public vnl_numeric_traits<int>
{
public:
	//: Additive identity
	static constexpr int zero = 0;
	//: Multiplicative identity
	static constexpr int one = 1;
	//: Maximum value which this type can assume
	static constexpr int maxval = 0x7fffffff; // = 0x7fffffff;
	//: Return value of abs()
	typedef unsigned int abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef long double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};


/* input: one image pair with features
 * output: the initial matched feature pairs by only checking their descriptor
 * min_dist_ratio-- the bigger it is, more matches is found
 */
bool visual_feature_matching(SImageVisualFeatureSet_sptr &feature_set_1, 
							 SImageVisualFeatureSet_sptr &feature_set_2,
							 SImageVisualFeatureMatchSet &match_set,
							 double min_dist_ratio = 0.7);

bool visual_feature_matching(SImageVisualFeatureSet_sptr &feature_set_1, 
							 SImageVisualFeatureSet_sptr &feature_set_2,
							 PairwiseMatchIdxSet &match_set_indices,
							 double min_dist_ratio = 0.7,
							 int frmIdx_1 = -1,
							 int frmIdx_2 = -1);


// Make sure each feature only appears once in the list. Remove any
// pair that has a duplicate.
void remove_all_duplicates(SImageVisualFeatureMatchSet &matches);

/* estimate fundamental matrix F that satisfying:
 * feature_set_2 * F * feature_set_1 = 0;
 * match_set: as input it is the initial matched point pairs, as output it is the refined pairs
 */
bool estimate_fundamental_matrix( SImageVisualFeatureMatchSet &match_set,
								  vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
								  int min_num_inliers = 16,
								  double outlier_threshold = 9.0,
								  double probability_good_f_mat = 0.99,
								  double max_outlier_frac = 0.5,
								  bool recompute_after_ransac = true);

bool estimate_fundamental_matrix( SImageVisualFeatureMatchSet &match_set,
								  vnl_matrix_fixed<double, 3, 3> &fundamental_mat,
								  int min_inliers,
								  double feature_loc_std = 1.5,
								  bool recompute_after_ransac = true);

/* [R|T] will transform feature_set_1 to feature_set_2
 */
bool extract_transform( vector< vnl_vector_fixed<double, 3> > const &points_1,
						vector< vnl_vector_fixed<double, 3> > const &points_2,
						vnl_matrix_fixed<double, 3, 3> &R,
						vnl_vector_fixed<double, 3> &T);

bool extract_transform( S3DPointMatchSet const& match_set_3d,
						vnl_matrix_fixed<double, 3, 3> &R,
						vnl_vector_fixed<double, 3> &T);

bool extract_transform_ransac( S3DPointMatchSet &match_set_3d,
							   vnl_matrix_fixed<double, 3, 3> &R,
							   vnl_vector_fixed<double, 3> &T );

/* [R|T] will transform feature_set_1 to feature_set_2
 */
bool extract_transform( vnl_matrix_fixed<double,3,3> const&F, 
						vnl_matrix_fixed<double,3,3> const&K_left, //intriniscs
						vnl_matrix_fixed<double,3,3> const&K_right, //intriniscs
						vgl_point_2d<double> left_corr, //one point from feature_set_2
						vgl_point_2d<double> right_corr, ////one point from feature_set_1
						vnl_matrix_fixed<double,3,3> &R, 
						vnl_vector_fixed<double,3> &T,
						double T_mag = 1.0);

bool triangulation( SImageVisualFeatureMatchSet &match_set,
					vnl_matrix_fixed<double, 3, 3> &intrinsics,
					vnl_matrix_fixed<double, 3, 3> &R, 
					vnl_vector_fixed<double, 3> &T,
					vector< vgl_point_3d<double> > &points_3d );

bool triangulation( SImageVisualFeatureMatchSet &match_set,
					vpgl_proj_camera<double> &cam1,
					vpgl_proj_camera<double> &cam2,
					vector< vgl_point_3d<double> > &points_3d);

void triangulation( std::vector< vnl_vector_fixed<double, 2> > const& image_pts,
					std::vector< vnl_matrix_fixed<double, 3, 4> > const& Ps, //projective matrix
					vnl_vector_fixed<double, 3> &p_3d);
void triangulation( std::vector< vnl_vector_fixed<double, 2> > const& image_pts,
					std::vector< vpgl_perspective_camera<double>* > const& cams, //camera poses
					vnl_vector_fixed<double, 3> &p_3d);

// P1&P2: perspective matrix
inline void triangulation( double x1[2], double x2[2], 
						   vnl_matrix_fixed<double, 3, 4> const&P1,
						   vnl_matrix_fixed<double, 3, 4> const&P2,
						   vnl_vector_fixed<double, 3> &p_3d)
{
	/* x1 x P1 * X = 0
	 * x2 x P2 * X = 0
	 */
	vnl_matrix_fixed<double,4,4> A;
	for(int i=0; i<4; i++) 
	{
		A[0][i] = -x1[0]*P1[2][i] + P1[0][i];
		A[1][i] =  x1[1]*P1[2][i] - P1[1][i];
		A[2][i] = -x2[0]*P2[2][i] + P2[0][i];
		A[3][i] =  x2[1]*P2[2][i] - P2[1][i];
	}
	vnl_svd<double> svd_solver(A);
	vnl_vector_fixed<double, 4> p = svd_solver.nullvector();
	p_3d[0] = p[0]/p[3];
	p_3d[1] = p[1]/p[3];
	p_3d[2] = p[2]/p[3];

	//vector< vnl_vector_fixed<double, 2> > image_pts;
	//image_pts.push_back( vnl_vector_fixed<double, 2>(x1) );
	//image_pts.push_back( vnl_vector_fixed<double, 2>(x2) );
	//vector< vnl_matrix_fixed<double, 3, 4> > Ps;
	//Ps.push_back(P1);
	//Ps.push_back(P2);
	//triangulation(image_pts, Ps, p_3d);
}

inline void triangulation( double x1[2], double x2[2],
						   vpgl_proj_camera<double> const& cam1,
						   vpgl_proj_camera<double> const& cam2,
						   vnl_vector_fixed<double, 3> &p_3d)
{
	triangulation( x1, x2, cam1.get_matrix(), cam2.get_matrix(), p_3d);
}

//mid-point method
// K1*(R1*X+T1) = lamda_1*m1
// K2*(R2*X+T2) = lamda_2*m2
inline bool triangulation2( double x1[2], double x2[2], 
							vnl_matrix_fixed<double, 3, 3> const&K1, 
							vnl_matrix_fixed<double, 3, 3> const&R1,
							vnl_vector_fixed<double, 3> const&T1,
							vnl_matrix_fixed<double, 3, 3> const&K2, 
							vnl_matrix_fixed<double, 3, 3> const&R2,
							vnl_vector_fixed<double, 3> const&T2,
							vnl_vector_fixed<double, 3> &p_3d)
{
	vnl_vector_fixed<double, 3> m1(x1[0], x1[1], 1.0);
	vnl_vector_fixed<double, 3> m2(x2[0], x2[1], 1.0);
	vnl_matrix_fixed<double, 3, 3> dR = R2*R1.transpose();
	
	//lamda_1 *a + b = lamda_2*c
	vnl_vector_fixed<double, 3> a = dR*(vnl_inverse(K1)*m1);
	vnl_vector_fixed<double, 3> b = -dR*T1+T2;
	vnl_vector_fixed<double, 3> c = vnl_inverse(K2)*m2;

	vnl_matrix_fixed<double, 2, 2> A;
	A[0][0] = dot_product(a, a);
	A[0][1] = dot_product(a, -c);
	A[1][0] = A[0][1];
	A[1][1] = dot_product(c, c);

	vnl_vector_fixed<double, 2> d;
	d[0] = dot_product(a, -b);
	d[1] = dot_product(c, b);

	vnl_vector_fixed<double, 2> lamda = vnl_inverse(A)*d;
	vnl_vector_fixed<double, 3> X1 = R1.transpose()*(lamda[0]*vnl_inverse(K1)*m1-T1);
	vnl_vector_fixed<double, 3> X2 = R2.transpose()*(lamda[1]*vnl_inverse(K2)*m2-T2);
	p_3d = (X1+X2)/2.0;
	if( lamda[0] < 0.0 || lamda[1] < 0.0)
		return false;
	else
		return true;
}

inline bool triangulation2( double x1[2], double x2[2], 
							GCameraView const* cam1,
							GCameraView const* cam2,
							vnl_vector_fixed<double, 3> &p_3d)
{
	vnl_matrix_fixed<double, 3, 3> K1(&(cam1->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> R1(&(cam1->R[0][0]));
	vnl_vector_fixed<double, 3> T1(cam1->T);
	vnl_matrix_fixed<double, 3, 3> K2(&(cam2->K[0][0]));
	vnl_matrix_fixed<double, 3, 3> R2(&(cam2->R[0][0]));
	vnl_vector_fixed<double, 3> T2(cam2->T);
	return triangulation2(x1, x2, K1, R1, T1, K2, R2, T2, p_3d);
}

inline bool triangulation2( double x1[2], double x2[2], 
							vpgl_perspective_camera<double> const& cam1,
						    vpgl_perspective_camera<double> const& cam2,
							vnl_vector_fixed<double, 3> &p_3d)
{
	vnl_matrix_fixed<double, 3, 3> K1 = cam1.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R1;
	vnl_vector_fixed<double, 3> T1;
	get_camera_pose(cam1, R1, T1);
	vnl_matrix_fixed<double, 3, 3> K2 = cam2.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R2;
	vnl_vector_fixed<double, 3> T2;
	get_camera_pose(cam2, R2, T2);
	return triangulation2(x1, x2, K1, R1, T1, K2, R2, T2, p_3d);
}

/* compute the inliers of the 3D point match set under transformation <R, T>
 */
void compute_inliers( S3DPointMatchSet const&match_3d_in, S3DPointMatchSet &match_3d_out,
					  vnl_matrix_fixed<double, 3, 3> const&R, vnl_vector_fixed<double, 3> const&T, 
					  double thres_dist = 8,
					  bool bDeleteOneVsMany = true);

/* refine the 3D match set with RANSAC
 */
bool refine_3DPointsMatchSet_RANSAC(S3DPointMatchSet &match_set_in, 
									S3DPointMatchSet &match_set_out,
									vnl_matrix_fixed<double, 3, 3> &R,
									vnl_vector_fixed<double, 3> &T,
									double dist_thres = 5.0,
									double inlier_percentage = 0.4,
									double ransac_accracy = 0.9999);
/* get the 3D point in the camera space;
 * a point will be <0,0,0> if no depth associated with it
 */
void calc_Point3DSet( SImageVisualFeatureSet_sptr fea, 
					  vnl_matrix<double> const&depthMat,
					  vnl_matrix_fixed<double, 3, 3> const&K, 
					  std::vector< vnl_vector_fixed<double, 3> > &points_3d,
					  depth_extract_method method = depth_extract_complexAvg);

void Point3DSet_to_IndexSet( std::vector< vnl_vector_fixed<double, 3> > const&points_full,
							 std::vector< vnl_vector_fixed<double, 3> > const&points_part,
							 std::vector<int> &pt_indices);

/* the 3d point is in its camera space, the depth value is obtained from the depth map
 * only the 2d feature point sitting on a flat plane is selected for 3d point calculation
 */
void calc_3DPointsMatchSet( SImageVisualFeatureMatchSet const&match_set_undistort,
							vnl_matrix<double> const&depthMap_1_undistort,
							vnl_matrix<double> const&depthMap_2_undistort,
							vnl_matrix_fixed<double, 3, 3> const&K1,
							vnl_matrix_fixed<double, 3, 3> const&K2,
							S3DPointMatchSet &match_set_3d,
							depth_extract_method method = depth_extract_complexAvg);
///image space to camera space
void calc_3DPointsMatchSet( vnl_matrix<double> const&matched_corners,
							vnl_matrix<double> const&depthMap_1_undistort,
							vnl_matrix<double> const&depthMap_2_undistort,
							vnl_matrix_fixed<double, 3, 3> const&K1,
							vnl_matrix_fixed<double, 3, 3> const&K2,
							S3DPointMatchSet &match_set_3d,
							depth_extract_method method = depth_extract_complexAvg);

//image space to world space
void calc_3DPointsMatchSet( vnl_matrix<double> const&matched_corners,
							vnl_matrix<double> const&depthMap_1_undistort,
							vnl_matrix<double> const&depthMap_2_undistort,
							vpgl_perspective_camera<double> const&cam1,
							vpgl_perspective_camera<double> const&cam2,
							S3DPointMatchSet &match_set_3d,
							depth_extract_method method = depth_extract_complexAvg);

void calc_3DPointsMatchSet(vnl_matrix<double> const&matched_corners,
						   cv::Mat const& depthMap_1_undistort,
						   cv::Mat const& depthMap_2_undistort,
						   GCameraView const&cam1,
						   GCameraView const&cam2,
						   S3DPointMatchSet &match_set_3d,
						   depth_extract_method method = depth_extract_complexAvg,
						   bool bStartOver = true);

//camera space to image space
void PointMatchSet_3DTo2D( S3DPointMatchSet const&match_set_3d,
						   vnl_matrix_fixed<double, 3, 3> const&K1,
						   vnl_matrix_fixed<double, 3, 3> const&K2,
						   SImageVisualFeatureMatchSet &match_set_undistort );
void PointMatchSet_3DTo2D( S3DPointMatchSet const&match_set_3d,
						   vnl_matrix_fixed<double, 3, 3> const&K1,
						   vnl_matrix_fixed<double, 3, 3> const&K2,
						   vnl_matrix<double> &matched_points );

//world space to image space
void PointMatchSet_3DTo2D( S3DPointMatchSet const&match_set_3d,
						   vpgl_perspective_camera<double> const&cam1,
						   vpgl_perspective_camera<double> const&cam2,
						   vnl_matrix<double> &matched_points );


/** Featrue Index: find the feature point at <row, col>
**  return -1 if not found
**/
SImageVisualFeature_sptr feature_set_index( SImageVisualFeatureSet_sptr fea_set, 
											double row, double col,
											int *idx = NULL);


/*=======================================================================
 *				I/O Operations
 *
 =========================================================================*/
bool save_visual_feature_set_ASCII( SImageVisualFeatureSet_sptr feature_set, 
									const char* filename_loc_scale_ori,
									const char* filename_features);

bool save_visual_feature_set_ASCII( SImageVisualFeatureSet_sptr feature_set, 
									const char *filename);
SImageVisualFeatureSet_sptr load_visual_feature_set_ASCII(const char *filename);

bool save_visual_feature_set_BIN( SImageVisualFeatureSet_sptr feature_set, 
									const char *filename);
SImageVisualFeatureSet_sptr load_visual_feature_set_BIN(const char *filename);
SImageVisualFeatureSet_sptr load_visual_feature_set(const char *filename);


bool save_visual_feature_match_set_ASCII( SImageVisualFeatureMatchSet &match_set,
										  const char* filename);

// draw matched features on images, two given images must be color images
bool draw_visual_feature_match_set( SImageVisualFeatureMatchSet &match_set,
									cv::Mat const& img1,
									cv::Mat const& img2,
									const char* filename_out );
bool draw_visual_features_on_image( SImageVisualFeatureSet_sptr feas, 
									cv::Mat& img );

bool load_visual_feature_match_set_ASCII( SImageVisualFeatureMatchSet &match_set,
										  const char* filename);

bool save_3D_points(const char* filename, vector< vgl_point_3d<double> > &points_3d);
bool save_3D_points(const char* filename, vector< vnl_vector_fixed<double, 3> > &points_3d);
bool load_3D_points(const char* filename, vector< vgl_point_3d<double> > &points_3d);
bool load_3D_points(const char* filename, vector< vnl_vector_fixed<double, 3> > &points_3d);


bool saveAsBundleFileOnePair( const char* filename,
							  SImageVisualFeatureMatchSet &match_set,
							  vpgl_perspective_camera<double> &cam1,
							  vpgl_perspective_camera<double> &cam2,
							  vector< vgl_point_3d<double> > &points_3d );

#endif
