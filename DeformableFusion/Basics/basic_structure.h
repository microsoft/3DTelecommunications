#ifndef __BASIC_STRUCTURE_H__
#define __BASIC_STRUCTURE_H__

#include "UtilVnlMatrix.h"
#include <vector>
#include <vil\vil_image_view.h>
#include <vil\vil_save.h>
#include <vil\vil_load.h>
#include "CameraView.h"
#include "BoundingBox3D.h"
#include "CSurface.h"
#include <map>
#include "S3DPointMatchSet.h"

using namespace std;

typedef pair<int, double> IndexDistPair;
inline bool less_comparator_IndexDistPair( const IndexDistPair& l, //or DoubleIndexPair const& l
										   const IndexDistPair& r)
{ 
	return l.second < r.second; 
}

inline bool greater_comparator_IndexDistPair( const IndexDistPair& l, //or DoubleIndexPair const& l
											  const IndexDistPair& r)
{ 
	return l.second > r.second; 
}

typedef vector< IndexDistPair > IndexDistList;
typedef map<int, double> IndexDistHashTable;

//============================================================================================
//				HSV Operations
//=============================================================================================

inline float calc_hue_dist( float h1, float h2 )
{
	float d = abs( h1 - h2 );
	return MIN( d, 1.0 - d );
}

inline int calc_hue_dist( int h1, int h2 )
{
	int d = std::abs(h1-h2);
	return MIN(d, 255-d);
}

/* convert a rgb vector its hsv vector, all the components are scaled to [0, 255]
 * note that for hue component, 0 == 255
 */
template <class T>
inline void rgb_to_hsv( vnl_vector_fixed<T, 3> const&rgb, vnl_vector_fixed<T, 3> &hsv );

bool apply_mask_to_depthMap( vnl_matrix<double> &depthMap, cv::Mat const& mask);
bool apply_mask_to_depthMap( cv::Mat& depthMap, cv::Mat const& mask);
bool apply_mask_to_cvMat(cv::Mat& img, cv::Mat const& mask, int outer_val = 0, bool bScaleImage = false);
cv::Mat mask_from_depthMap( vnl_matrix<double> const& depthMap);
cv::Mat mask_from_depthMap( cv::Mat const& depthMap);
cv::Mat mask_from_depthImg(cv::Mat const& depthImg);

bool extract_depth_contour(cv::Mat const& depthMat, // the depthMat contains only foreground object
	std::vector<vnl_vector_fixed<int, 2>> &points_2d,
	double thres_dist_nn = 5, //in cm
	cv::Mat img = cv::Mat()
	);

cv::Mat contour_to_image(std::vector<vnl_vector_fixed<int, 2>> &points_contour, int width, int height);

inline bool is_mask_empty(cv::Mat& mask)
{
	return (mask.empty() || (cv::countNonZero(mask) == 0));
}

bool calc_clr_imgs_gradient( vector<cv::Mat> const& imgs_ori, 						 
						 vector< cv::Mat > &imgs_f,
						 vector< cv::Mat > &mats_Rx,
						 vector< cv::Mat > &mats_Ry,
						 vector< cv::Mat > &mats_Gx,
						 vector< cv::Mat > &mats_Gy,
						 vector< cv::Mat > &mats_Bx,
						 vector< cv::Mat > &mats_By,
						 int kernal_radius = 2,
						 vector<cv::Mat> const& masks = vector<cv::Mat>() //input
						 );

// assume depthMats have not lens distortion
// if thres_blob_size > 0, only the big blobs of the depthMats are used to calc boundary
BoundingBox3D extract_bounding_box(vector<cv::Mat> depthMats, vector<GCameraView const*> cams, vector<cv::Mat> masks = std::vector<cv::Mat>(), int thres_blob_size = -1);
inline BoundingBox3D extract_bounding_box(vector<cv::Mat> depthMats, vector<GCameraView*> cams, vector<cv::Mat> masks = vector<cv::Mat>(), int thres_blob_size = -1)
{
	vector<cv::Mat> depthMats_const;
	for (int i = 0; i < depthMats.size(); i++)
		depthMats_const.push_back(depthMats[i]);
	vector<GCameraView const*> cams_const;
	for (int i = 0; i < cams.size(); i++)
		cams_const.push_back(cams[i]);
	vector<cv::Mat> masks_const;
	for (int i = 0; i < masks.size(); i++)
		masks_const.push_back(masks[i]);
	return  extract_bounding_box(depthMats_const, cams_const, masks_const, thres_blob_size);
}

BoundingBox3D extract_bounding_box(cv::Mat const& depthMat, GCameraView const* cam, cv::Mat const& mask, int thres_blob_size=-1);


enum depth_extract_method
{
	depth_extract_simpleAvg,
	depth_extract_complexAvg,
	depth_extract_NN,
	depth_extract_InterLinear,
};
/* return 0 if no reasonable depth is found
 */
double extract_depth_at_visual_feature( vnl_matrix<double> const&depthMap, 
									    double x, double y, 
									    depth_extract_method method = depth_extract_complexAvg);
double extract_depth_at_visual_feature( cv::Mat const& depthMap, 
									    double x, double y, 
									    depth_extract_method method = depth_extract_complexAvg);

struct DepthElement
{
public:
	DepthElement()
		: d(-1.0),
		  P_cam(0.0, 0.0, 0.0),
		  normal(0.0, 0.0, 0.0),
		  bNormal(false),
		  rho(0.0),
		  theta(0.0),
		  phi(0.0)
	{;}
		
public:
	double d; //original depth value from depth map
	vnl_vector_fixed<double, 3> P_cam; //p in camera space
	vnl_vector_fixed<double, 3> normal; //normal in camera sapce; sometimes in world space
	bool bNormal;
//plane equation: n x - rho = 0;
public:
// for plane voting
	double rho; // distance from the camera center(origin of the camera space) to the local plane
	double theta; //in [0, pi/2]
	double phi; //in [-pi, pi]
};

//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<DepthElement> : public vnl_numeric_traits<double>
{
public:
	//: Additive identity
	static constexpr double zero = 0.0;
	//: Multiplicative identity
	static constexpr double one = 1.0;
	//: Maximum value which this type can assume
	static constexpr double maxval = 1.7976931348623157E+308;
	//: Return value of abs()
	typedef double abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef long double double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};

typedef vnl_matrix<DepthElement> CDepthMap;

#include "basic_structure.hpp"

#endif

