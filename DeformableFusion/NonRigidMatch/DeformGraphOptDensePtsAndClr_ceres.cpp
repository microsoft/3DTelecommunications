#include "DeformGraphOptDensePtsAndClr.h"
#include "nonrigid_matching_surface.h"
#include <omp.h>
namespace NonrigidMatching{
#ifdef USE_CERES_SOLVER


vector<GCameraView*> DeformGraphOptMultiDataDensePtsAndClrStatic::cams = vector<GCameraView*>(); //at target
vector< vnl_matrix_fixed<double, 3, 4> > DeformGraphOptMultiDataDensePtsAndClrStatic::cam_prj_mats = vector< vnl_matrix_fixed<double, 3, 4> >(); //at target
vector< cv::Mat> DeformGraphOptMultiDataDensePtsAndClrStatic::imgs = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Rx = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Ry = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Gx = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Gy = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_Bx = vector< cv::Mat >();
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::mats_By = vector< cv::Mat >();
double DeformGraphOptMultiDataDensePtsAndClrStatic::w_color = 0.0; //weight for the color
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::depthMats_prj = vector<cv::Mat>(); //project surface_t onto various camera spaces
vector< cv::Mat > DeformGraphOptMultiDataDensePtsAndClrStatic::imgs_proj = vector< cv::Mat >();
int DeformGraphOptMultiDataDensePtsAndClrStatic::frmIdx = 0;
int DeformGraphOptMultiDataDensePtsAndClrStatic::iterIdx = 0;
char const* DeformGraphOptMultiDataDensePtsAndClrStatic::tmp_dir = NULL;


#endif
};