// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once
#ifndef __SURFACE_MISC_H__
#define __SURFACE_MISC_H__
#include <stdio.h>
#include <string.h>
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include "CameraView.h"
#include "CSurface.h"
#include <vector>
#include "basic_geometry.h"
#include "UtilVnlMatrix.h"

//sum over the neighboring points round by round;
//Problems: holes expand
template<class T>
void smooth_surface(CSurface<T> &surface, int round = 1);

template<class T>
void smooth_surface_normals(CSurface<T> &surface, int round = 1);

template<class T1, class T2>
void transform_Surface_with_RigidModel( CSurface<T1> &surface, 
										vnl_matrix_fixed<T2, 3, 3> const& R,
										vnl_vector_fixed<T2, 3> const& t);

//note the component label starts from 1;
//the small blobs has a label of 0;
template<class T>
void ConnectedComponentOnSurface( CSurface<T> const& surface, 
								  vector<int> &comp_label,
								  int &comp_num,
								  int comp_num_thres);

template<class T>
void removeSurfaceSmallUnconnectedBlobs(CSurface<T> &surface, int thres_blob_size=1000);

// surface_1 = surface_1 - surface_2
template<class T>
void SurfaceSubtraction( CSurface<T> &surface_1, CSurface<T> const&surface_2 );

//surface_1 = surface_1 + surface_2
template<class T>
void SurfaceAddition( CSurface<T> &surface_1, CSurface<T> const&surface_2 );

	
/* get surface: triangle vertices is in the clockwise order	
 *    depthMap and image should have *NO* lens distortion
 * color is also attached with the surface if img != NULL
	* it takes 97ms for 480x640 depth map
	* it takes 173ms for 480x640 depth map if generating normals
	*/
template<class T>
CSurface<T> *DepthMap2Surface( GCameraView const*cam, cv::Mat const& depthMap_undistort, cv::Mat const& img,
							   double neighbor_pixel_max_dist = 5.0, int offsetX_bw_rgb_detph=0, int offsetY_bw_rgb_depth = 0, 
							   double a=1.0, double b = 0.0, //a and b is working on depth value z = a*z+b
							   bool bNormal=false);


//use the orthorgraphic camera to generate surface
//if normalMap/clr_img is not NULL, will attach the normal/color to the surface
template<class T>
CSurface<T> *DepthMap2SurfaceOrtho(cv::Mat const& depthMap, cv::Mat const& clr_img, vnl_matrix<vnl_vector_fixed<double, 3>> const* normalMap,
	double pixel_dist, double neighbor_pixel_max_dist = 5.0);

template<class T>
bool DepthMap2Surface( vpgl_perspective_camera<double> const& cam, cv::Mat const& depthMap_undistort, cv::Mat const& img,
					   CSurface<T> &m_surface,
					   double neighbor_pixel_max_dist = 5.0, int offsetX_bw_rgb_detph=0, int offsetY_bw_rgb_depth = 0, 
					   double a=1.0, double b = 0.0, //a and b is working on depth value z = a*z+b
					   bool bNormal=false);

template<class T>
bool  DepthMap2Surface( vnl_matrix<double> const& depthMat_undist, 
						vpgl_perspective_camera<double> const &cam, 
						cv::Mat const& img,
					    CSurface<T> &m_surface, 
						double neighbor_pixel_max_dist = 5.0, 
						int offsetX_bw_rgb_detph=0, int offsetY_bw_rgb_depth = 0, 
						double a=1.0, double b = 0.0, //a and b is working on depth value z = a*z+b
						bool bNormal=false);

enum triangle_removal_method
{
	remove_counterclock_tri,
	remove_clockwise_tri,
	remove_none_tri
};
/* Note: 1). lens distortion is considered.
	*       2). may choose to delete triangles with specified order in the image space.
	*/
template<class T>
cv::Mat ComputeDepthMap( CSurface<T> const*m_surf, GCameraView const* cam, 
						triangle_removal_method remove_tri = remove_none_tri,
						bool bDistortDepthMap = false,
						double near_clip_plane = 0.0);
template<class T>
cv::Mat ComputeDepthMap( vector< CSurface<T>* > const& m_surfs, GCameraView const* cam, 
						triangle_removal_method remove_tri = remove_none_tri,
						bool bDistortDepthMap = false,
						double near_clip_plane = 0.0);

//depth_mat and img_proj should be preallocated 
template<class T>
void ProjectSurface( vector< CSurface<T>* > const& m_surfs, GCameraView const* cam,
					 cv::Mat& depth_mat, //projected depth map (could be set to NULL)
					 cv::Mat& img_proj,//projected color image (could be set to NULL)
					 int color_view = -1,// which one to choose for color: channels of surface.colors_of_view or surface.vtData(-1)
					 triangle_removal_method remove_tri = remove_none_tri,
					 bool bDistortDepthMap = false,
					 double near_clip_plane = 0.0);

template<class T>
void ProjectSurface( CSurface<T> const*m_surf, GCameraView const* cam,
					 cv::Mat& depth_mat, //projected depth map (could be set to NULL)
					 cv::Mat& img_proj,//projected color image (could be set to NULL)
					 int color_view = -1,// which one to choose for color: channels of surface.colors_of_view or surface.vtData(-1)
					 triangle_removal_method remove_tri = remove_none_tri,
					 bool bDistortDepthMap = false,
					 double near_clip_plane = 0.0)
{
	vector< CSurface<T>* > m_p_surfs;
	m_p_surfs.push_back((CSurface<T>*)m_surf);
	ProjectSurface(m_p_surfs, cam, depth_mat, img_proj, color_view, remove_tri, bDistortDepthMap, near_clip_plane);
}

template<class T>
void ProjectSurface( vector< CSurface<T> > const& m_surfs, GCameraView const* cam,
					 cv::Mat& depth_mat, //projected depth map (could be set to NULL)
					 cv::Mat& img_proj,//projected color image (could be set to NULL)
					 int color_view = -1,// which one to choose for color: channels of surface.colors_of_view or surface.vtData(-1)
					 triangle_removal_method remove_tri = remove_none_tri,
					 bool bDistortDepthMap = false,
					 double near_clip_plane = 0.0)
{
	vector< CSurface<T>* > m_p_surfs;
	for(int i=0; i<m_surfs.size(); i++)
	{
		m_p_surfs.push_back( (CSurface<T>* )(&m_surfs[i]) );
	}
	ProjectSurface(m_p_surfs, cam, depth_mat, img_proj, color_view, remove_tri, bDistortDepthMap, near_clip_plane);
}

template<class T>
cv::Mat* ComputeDepthMap( vector< CSurface<T> > const& m_surfs, GCameraView const* cam, 
						triangle_removal_method remove_tri = remove_none_tri,
						bool bDistortDepthMap = false,
						double near_clip_plane = 0.0)
{
	vector< CSurface<T>* > m_p_surfs;
	for(int i=0; i<m_surfs.size(); i++)
	{
		m_p_surfs.push_back( (CSurface<T>* )(&m_surfs[i]) );
	}
	return ComputeDepthMap(m_p_surfs, cam, remove_tri, bDistortDepthMap, near_clip_plane);
}

template<class T>
cv::Mat* ComputeDepthMap( CSurface<T> const*m_surf, GCameraView const* cam, 
						bool bRemoveCounterClockTri = false, 
						bool bDistortDepthMap = false,
						double near_clip_plane = 0.0)
{
	if( bRemoveCounterClockTri)
		return ComputeDepthMap(m_surf, cam, remove_counterclock_tri, bDistortDepthMap, near_clip_plane);
	else
		return ComputeDepthMap(m_surf, cam, remove_none_tri, bDistortDepthMap, near_clip_plane);
} 

//no lens distortion
template<class T>
cv::Mat* ComputeDepthMap( CSurface<T> const*m_surf, vnl_matrix_fixed<double, 3, 4> const& prj_mat, int width, int height);

// in : m_surface, cam
// out: depthMat
template<class T>
bool  ComputeDepthMap( CSurface<T> const &m_surface, vpgl_perspective_camera<double> const& cam, 
					   int width, int height, // the size of the depthMat
					   vnl_matrix<double> &depthMat, 
					   vnl_vector<double> const& distort_vec = vnl_vector_fixed<double, 4>(0,0,0,0),
					   triangle_removal_method remove_tri = remove_none_tri,
					   bool bDistortDepthMap = false,
					   double near_clip_plane = 0.0);
template<class T>
bool  ComputeDepthMap( CSurface<T> const &m_surface, vpgl_perspective_camera<double> const& cam, 
					   int width, int height, // the size of the depthMat
					   vnl_matrix<double> &depthMat, 
					   vnl_vector<double> const& distort_vec = vnl_vector_fixed<double, 4>(0,0,0,0),
					   bool bRemoveCounterClockTri = false,
					   bool bDistortDepthMap = false,
					   double near_clip_plane = 0.0)
{
	if( bRemoveCounterClockTri)
		return ComputeDepthMap(m_surface, cam, width, height, depthMat, distort_vec, remove_counterclock_tri, bDistortDepthMap, near_clip_plane);
	else
		return ComputeDepthMap(m_surface, cam, width, height, depthMat, distort_vec, remove_none_tri, bDistortDepthMap, near_clip_plane);
}


/* DepthMap -- in/out: each pixel in DepthMap must be initialized as -1. 
	* v1, v2, v3-- in: the input vertex position in world space
	* clr1, clr2, clr3 -- in: the rgb at each vertex, in the range of 0~1.0
	*/
template<class T1, class T2>
void project_a_triangle_onto_cam( GCameraView const* cam, //in
								  T1 *v1, T1 *v2, T1 *v3, //in
								  cv::Mat& depthMap,
								  T2 *clr1, T2 *clr2, T2 *clr3, //in
								  cv::Mat& img_proj,
								  bool bDistortDepthMap = false,
								  cv::Mat& mask_front_oppositetri = cv::Mat(), bool bOppositeTri = false,
								  double near_clip_plane = 0.0);

template<class T>
void project_a_triangle_onto_cam(vnl_matrix_fixed<double, 3, 4> const& prj_mat, T v1[], T v2[], T v3[], cv::Mat& depthMap, int width, int height);

template<class T>
inline T triangle_area(T v1[3], T v2[3], T v3[3]);

//check the area of each triangle, and remove the one above some threshold
template<class T>
bool RemoveUnnecessaryPolygon(CSurface<T> *m_surf, GCameraView *cam1);

// remove the stretched polygon on surface_t, a deformed version of surface
template<class T>
bool RemoveStretchedPolygon(CSurface<T> &surface_t, CSurface<T> const&surface, double thres_stretch = 1.5, double thres_stretch_abs = 1);

template<class T>
void FillColorInfo(CSurface<T> *m_surf, GCameraView *cam1, cv::Mat& img1);
//this function is not perfect! maybe some bugs hide somewhere <-- Shame on whoever left this comment.  FIX YOUR STUFF!
template<class T>
void FillColorInfoNCams(CSurface<T>* m_surf, vector<GCameraView*> cams, vector<cv::Mat> imgs, bool bImgWithDistortion = true, bool bBlending = true, double gamma = 1.0);

// color order: r, g, b
template<class T>
bool readout_surface_texture(CSurface<T> const& surface, vector< vnl_vector_fixed<float, 3> > &texs);

template<class T>
bool set_surface_texture(CSurface<T>& surface, vector< vnl_vector_fixed<float, 3> > const&texs, bool bSkipTexedVert=false);

template<class T>
void freeCSurfaceArray(CSurface<T>** &surfaces, int num);

template<class T>
bool texture_surface_model( CSurface<T> &m_surface, 
							vector< vpgl_perspective_camera<double> > &camera_poses,
							vnl_vector<double> const& distort_vec,
							const char* color_img_list_file_name,
							const char* data_dir );

enum Point2SurfaceDistanceMode
{
	DistanceMode_Point2Point,
	DistanceMode_Point2Plane
};

template<class T1, class T2>
bool closest_point_on_surface_ch_normal(CSurface<T1> const&m_surface,
										vnl_vector_fixed<T2, 3> const &p,
										vnl_vector_fixed<T2, 3> const &np,
										vnl_vector_fixed<T2, 3> &q,
										int &index,
										Point2SurfaceDistanceMode dist_mode,
										double thres_angle);

template<class T1, class T2>
bool closest_point_on_surfaces( vector< CSurface<T1>* >  const& m_surfaces,
							   vnl_vector_fixed<T2, 3> const &p, 
							   vnl_vector_fixed<T2, 3> &q,
							   int &surfIdx, //closest surface Idx
							   int &vtIdx, //closest vertex Idx
							   Point2SurfaceDistanceMode dist_mode = DistanceMode_Point2Point); 

template<class T1, class T2>
bool closest_point_on_surface( CSurface<T1> const&m_surface,
							   vnl_vector_fixed<T2, 3> const &p, 
							   vnl_vector_fixed<T2, 3> &q,
							   int &index,
							   Point2SurfaceDistanceMode dist_mode = DistanceMode_Point2Point); 

template<class T1, class T2>
bool closest_point_on_surface( CSurface<T1> const&m_surface,
							   vector< vnl_vector_fixed<T2, 3> > const& points, 
							   vector< vnl_vector_fixed<T2, 3> > & points_closest,
							   vector< int > &indices,
							   Point2SurfaceDistanceMode dist_mode = DistanceMode_Point2Point); 

//calculate the visibility
template<class T>
bool calc_camera_visibility( CSurface<T> const& surface, vector<GCameraView*> const& cams, 
							 vector< vector<int> > &visible_cams // each vertex has a list of ids of cameras that see the vertex
							 );

template<class T>
bool calc_camera_visibility( vector< CSurface<T>* > const& surfaces, vector<GCameraView*> const& cams, 
							 vector< vector<vector<int>> > &visible_cams );
//test
template<class T>
bool color_surface_with_visibility( CSurface<T> &surface, vector<vector<int>> const&visible_cams, 
									int cam_num, vector<int> which_cameras = vector<int>());


#include "surface_misc.hpp"
#endif