// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once
#ifndef __SURFACE_MISC_HPP__
#define __SURFACE_MISC_HPP__
#include "surface_misc.h"
#include "utility.h"
#include "VilToOpenCv.h"
#include "distortion_correction.h"
#include "depth_map_filter.h"

#include <omp.h>

template<class T>
void smooth_surface(CSurface<T> &surface, int round)
{
	//compute the neighbor list for each vertex
	vector< vector<int> > neighbor_indices(surface.vtNum);
	for(int i=0; i<surface.triNum; i++)
	{
		int vtIdx1 = surface.triangles[i*3];
		int vtIdx2 = surface.triangles[i*3+1];
		int vtIdx3 = surface.triangles[i*3+2];

		if( search_val_list(neighbor_indices[vtIdx1], vtIdx2) == -1 )
			neighbor_indices[vtIdx1].push_back(vtIdx2);
		if( search_val_list(neighbor_indices[vtIdx1], vtIdx3) == -1 )
			neighbor_indices[vtIdx1].push_back(vtIdx3);

		if( search_val_list(neighbor_indices[vtIdx2], vtIdx1) == -1 )
			neighbor_indices[vtIdx2].push_back(vtIdx1);
		if( search_val_list(neighbor_indices[vtIdx2], vtIdx3) == -1 )
			neighbor_indices[vtIdx2].push_back(vtIdx3);

		if( search_val_list(neighbor_indices[vtIdx3], vtIdx1) == -1 )
			neighbor_indices[vtIdx3].push_back(vtIdx1);
		if( search_val_list(neighbor_indices[vtIdx3], vtIdx2) == -1 )
			neighbor_indices[vtIdx3].push_back(vtIdx2);
	}

	for(int rd=0; rd<round; rd++)
	{
		printf("rd: %03d/%03d\r", rd, round);
		//sum over the value of its neighbors
		CSurface<float> surface_new = surface;
#pragma omp parallel for
		for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
		{
			int count=1;
			T* vt = surface_new.vt_data_block(vtIdx);
			for(int k=0; k<neighbor_indices[vtIdx].size(); k++)
			{
				int vtIdx_n = neighbor_indices[vtIdx][k];
				T* vt_n = surface.vt_data_block(vtIdx_n);
				vt[0] += vt_n[0];
				vt[1] += vt_n[1];
				vt[2] += vt_n[2];
				count++;
			}
			vt[0] /= count;
			vt[1] /= count;
			vt[2] /= count;

			if( surface.haveNormalInfo())
			{
				int count=1;
				T* n = surface_new.vt_normal(vtIdx);
				for(int k=0; k<neighbor_indices[vtIdx].size(); k++)
				{
					int vtIdx_n = neighbor_indices[vtIdx][k];
					T* n_n = surface.vt_normal(vtIdx_n);
					n[0] += n_n[0];
					n[1] += n_n[1];
					n[2] += n_n[2];
					count++;
				}
				n[0] /= count;
				n[1] /= count;
				n[2] /= count;
				VecOperation<T>::Unitalization(n, 3);
			}
		}
		surface = surface_new;
	}
}


template<class T>
void smooth_surface_normals(CSurface<T> &surface, int round)
{
	//compute the neighbor list for each vertex
	vector< vector<int> > neighbor_indices(surface.vtNum);
	for (int i = 0; i<surface.triNum; i++)
	{
		int vtIdx1 = surface.triangles[i * 3];
		int vtIdx2 = surface.triangles[i * 3 + 1];
		int vtIdx3 = surface.triangles[i * 3 + 2];

		if (search_val_list(neighbor_indices[vtIdx1], vtIdx2) == -1)
			neighbor_indices[vtIdx1].push_back(vtIdx2);
		if (search_val_list(neighbor_indices[vtIdx1], vtIdx3) == -1)
			neighbor_indices[vtIdx1].push_back(vtIdx3);

		if (search_val_list(neighbor_indices[vtIdx2], vtIdx1) == -1)
			neighbor_indices[vtIdx2].push_back(vtIdx1);
		if (search_val_list(neighbor_indices[vtIdx2], vtIdx3) == -1)
			neighbor_indices[vtIdx2].push_back(vtIdx3);

		if (search_val_list(neighbor_indices[vtIdx3], vtIdx1) == -1)
			neighbor_indices[vtIdx3].push_back(vtIdx1);
		if (search_val_list(neighbor_indices[vtIdx3], vtIdx2) == -1)
			neighbor_indices[vtIdx3].push_back(vtIdx2);
	}

	for (int rd = 0; rd<round; rd++)
	{
		printf("rd: %03d/%03d\r", rd, round);
		//sum over the value of its neighbors
		CSurface<T> surface_new = surface;
#pragma omp parallel for
		for (int vtIdx = 0; vtIdx<surface.vtNum; vtIdx++)
		{
			int count = 1;
			T* n = surface_new.vt_normal(vtIdx);
			for (int k = 0; k<neighbor_indices[vtIdx].size(); k++)
			{
				int vtIdx_n = neighbor_indices[vtIdx][k];
				T* n_n = surface.vt_normal(vtIdx_n);
				n[0] += n_n[0];
				n[1] += n_n[1];
				n[2] += n_n[2];
				count++;
			}
			n[0] /= count;
			n[1] /= count;
			n[2] /= count;
			VecOperation<T>::Unitalization(n, 3);
		}
		surface = surface_new;
	}
}

template<class T1, class T2>
void transform_Surface_with_RigidModel( CSurface<T1> &surface, 
										vnl_matrix_fixed<T2, 3, 3> const& R,
										vnl_vector_fixed<T2, 3> const& t)
{
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		T1 *p = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<T2, 3> vp(p[0], p[1], p[2]);		
		vnl_vector_fixed<T2, 3> vq = R*vp + t;
		p[0] = vq[0];
		p[1] = vq[1];
		p[2] = vq[2];
		if( surface.haveNormalInfo() )
		{
			T1 *n = surface.vt_normal(vtIdx);
			vnl_vector_fixed<T2, 3> np(n[0], n[1], n[2]);
			vnl_vector_fixed<T2, 3> nq = R*np;
			n[0] = nq[0];
			n[1] = nq[1];
			n[2] = nq[2];
		}
	}
}

template<class T>
void ConnectedComponentOnSurface( CSurface<T> const& surface, 
								  vector<int> &comp_label,
								  int &comp_num,
								  int comp_num_thres)
{
	int vtNum = surface.vtNum;
	vector< vector<int> > neighbor_indices(vtNum);
	for(int i=0; i<surface.triNum; i++)
	{
		int vtIdx1 = surface.triangles[i*3];
		int vtIdx2 = surface.triangles[i*3+1];
		int vtIdx3 = surface.triangles[i*3+2];

		if( search_val_list(neighbor_indices[vtIdx1], vtIdx2) == -1 )
			neighbor_indices[vtIdx1].push_back(vtIdx2);
		if( search_val_list(neighbor_indices[vtIdx1], vtIdx3) == -1 )
			neighbor_indices[vtIdx1].push_back(vtIdx3);

		if( search_val_list(neighbor_indices[vtIdx2], vtIdx1) == -1 )
			neighbor_indices[vtIdx2].push_back(vtIdx1);
		if( search_val_list(neighbor_indices[vtIdx2], vtIdx3) == -1 )
			neighbor_indices[vtIdx2].push_back(vtIdx3);

		if( search_val_list(neighbor_indices[vtIdx3], vtIdx1) == -1 )
			neighbor_indices[vtIdx3].push_back(vtIdx1);
		if( search_val_list(neighbor_indices[vtIdx3], vtIdx2) == -1 )
			neighbor_indices[vtIdx3].push_back(vtIdx2);
	}

	vector<int> vt_stack(vtNum);
	int cur_pos=0;//start from 0
	int tail_pos=0;//point to the empty cell

	printf("ccomp\n");
	int label_count = 0;
	comp_label.resize(vtNum, -1);
	for(int i=0; i<vtNum; i++)
	{
		if( comp_label[i] != -1 )
			continue;
		//start a new label
		label_count++;
		comp_label[i] = label_count;
		cur_pos = 0;
		tail_pos = 0;
		vt_stack[tail_pos] = i;
		tail_pos++;

		while(cur_pos < tail_pos)
		{
			int cur_vt = vt_stack[cur_pos];
			cur_pos++;
			//check its neighbors
			vector<int> const& neighbors = neighbor_indices[cur_vt];
			for(int k=0; k<neighbors.size(); k++)
			{
				int nb_vt = neighbors[k];
				//skip if visited
				if( comp_label[nb_vt] != -1 )
				{
					assert(comp_label[nb_vt] == label_count);
					continue;
				}

				//put into stack and assign the label
				vt_stack[tail_pos] = nb_vt;
				tail_pos++;
				comp_label[nb_vt] = label_count;
			}
		}

		if( cur_pos < comp_num_thres )
		{
			label_count--;
			for(int k=0; k<tail_pos; k++)
			{
				int vt = vt_stack[k];
				comp_label[vt] = 0; // small blobs has the label of 0;	
			}
			//printf("Comp deleted %d\n", tail_pos);
		}
		else
		{
			;//printf("Comp %d: %d\n", label_count, tail_pos);
		}
	}

	comp_num = label_count;
}

template<class T>
void removeSurfaceSmallUnconnectedBlobs(CSurface<T> &surface, int thres_blob_size)
{
	vector<int> comp_label;
	int comp_num;
	ConnectedComponentOnSurface(surface, comp_label, comp_num, thres_blob_size);

	vector<int> vt_out;
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		if( comp_label[vtIdx] == 0 )
			vt_out.push_back(vtIdx);
	}

	surface.delete_vertices(vt_out);
}

// surface_1 = surface_1 - surface_2
template<class T>
void SurfaceSubtraction( CSurface<T> &surface_1, CSurface<T> const&surface_2 )
{
	//vector<int> vertices_to_delete;
	//for(int i=0; i<surface_2.vtNum; i++)
	//{	
	//	if( i%1000 == 0 )
	//		printf("%d/%d\r", i, surface_2.vtNum);
	//	float const*p = surface_2.vt_data_block(i);
	//	//vnl_vector_fixed<float, 3> vp(p);
	//	//vnl_vector_fixed<float, 3> vq;
	//	//int vtIdx_nn;
	//	//bool bFound = closest_point_on_surface(surface_1, vp, vq, vtIdx_nn, DistanceMode_Point2Point);
	//	//if( bFound )
	//	//	vertices_to_delete.push_back(vtIdx_nn);
	//	for(int vtIdx=0; vtIdx<surface_1.vtNum; vtIdx++)
	//	{
	//		float const* q= surface_1.vt_data_block(vtIdx);
	//		if( p[0] == q[0] && 
	//			p[1] == q[1] &&
	//			p[2] == q[2] )
	//		{
	//			vertices_to_delete.push_back(vtIdx);
	//			break;
	//		}
	//	}
	//}

	vnl_vector< bool > bDeleteFlag(surface_1.vtNum, false);
#pragma omp parallel for
	for(int i=0; i<surface_1.vtNum; i++)
	{
		if( i%1000 == 0 )
			printf("%d/%d\r", i, surface_1.vtNum);
		float const*p = surface_1.vt_data_block(i);
		for(int j=0; j<surface_2.vtNum; j++)
		{
			float const* q = surface_2.vt_data_block(j);
			if( p[0] == q[0] && 
				p[1] == q[1] &&
				p[2] == q[2] )
			{
				bDeleteFlag[i] = true;
				break;
			}
		}
	}

	vector<int> vertices_to_delete;
	for(int i=0; i<bDeleteFlag.size(); i++)
		if(bDeleteFlag[i] )
			vertices_to_delete.push_back(i);
	surface_1.delete_vertices(vertices_to_delete);
}

//surface_1 = surface_1 + surface_2
template<class T>
void SurfaceAddition( CSurface<T> &surface_1, CSurface<T> const&surface_2 )
{
	if( surface_1.vtDim != surface_2.vtDim ||
		surface_1.haveColorInfo() != surface_2.haveColorInfo() ||
		surface_1.haveNormalInfo() != surface_2.haveNormalInfo() )
	{
		printf("Error<SurfaceAddition>: surfaces format disagree!\n");
		return;
	}

	int vtNum_1 = surface_1.vtNum;
	int vtNum_2 = surface_2.vtNum;
	int triNum_1 = surface_1.triNum;
	int triNum_2 = surface_2.triNum;
	int vtDim = surface_1.vtDim;
	T* vtData_new = new T[(vtNum_1+vtNum_2)*vtDim];
	int* triangles_new = new int[(triNum_1+triNum_2)*3];
	memcpy(vtData_new, surface_1.vtData, sizeof(T)*vtNum_1*vtDim);
	memcpy(&(vtData_new[vtNum_1*vtDim]), surface_2.vtData, sizeof(T)*vtNum_2*vtDim);
	memcpy(triangles_new, surface_1.triangles, sizeof(int)*triNum_1*3);
	for(int i=0; i<triNum_2; i++)
	{
		triangles_new[(triNum_1+i)*3] = surface_2.triangles[i*3]+vtNum_1;
		triangles_new[(triNum_1+i)*3+1] = surface_2.triangles[i*3+1]+vtNum_1;
		triangles_new[(triNum_1+i)*3+2] = surface_2.triangles[i*3+2]+vtNum_1;
	}

	delete [] surface_1.triangles;
	delete [] surface_1.vtData;
	surface_1.triangles = triangles_new;
	surface_1.vtData = vtData_new;
	surface_1.triNum = triNum_1 + triNum_2;
	surface_1.vtNum = vtNum_1 + vtNum_2;
}

template<class T>
CSurface<T>* DepthMap2Surface( GCameraView const* cam, cv::Mat const& depthMap, cv::Mat const& img,
							   double neighbor_pixel_max_dist, 
							   int offsetX_bw_rgb_depth, int offsetY_bw_rgb_depth, 
							   double a, double b, //a and b is working on depth value z = a*z+b
							   bool bNormal)
{
	if( a==0.0 && b==0.0)
	{
		a = 1.0;
		b = 0.0;
		printf("Warning: <1.0, 0.0> is used instead for depth bias correction!\n");
	}
	int h = depthMap.rows;
	int w = depthMap.cols;
	double scale_factor_x = w/double(cam->m_Width);
	double scale_factor_y = h/double(cam->m_Height);
	double fx = cam->K[0][0];
	double fy = cam->K[1][1];
	double cx = cam->K[0][2];
	double cy = cam->K[1][2];

	int points_num = 0;
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if(depthMap.at<double>(i, j) <= 0.0)
				continue;

			points_num ++;
		}
	}

	int idx = 0;
	T *p_world = NULL;
	T p_in_cam[3];
	T *points_in_world = new T[3*points_num];
	T *points_color = new T[3*points_num];
	int *idx_array = new int[w*h];
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			double z = depthMap.at<double>(i, j);; 
			// if z value is invalid, skip
			if(z <= 0.0 )
			{
				idx_array[i*w+j] = -1;
				continue;
			}
			z = a*z+b;
			
			p_world = &points_in_world[idx*3];

			double x = (j/scale_factor_x - cx)*z/fx;
			double y = (i/scale_factor_y - cy)*z/fy;

			p_in_cam[0] = (T)x;
			p_in_cam[1] = (T)y;
			p_in_cam[2] = (T)z;
			cam->cameraSpace2worldSpace(p_in_cam, p_world);
			
			idx_array[i*w+j] = idx;

			if(!img.empty())
			{
				if( i+offsetY_bw_rgb_depth > 0 && i+offsetY_bw_rgb_depth < h &&
					j+offsetX_bw_rgb_depth > 0 && j+offsetY_bw_rgb_depth < w )
				{
					points_color[3*idx] = img.at<cv::Vec3b>(i+offsetY_bw_rgb_depth, j+offsetX_bw_rgb_depth)[2]/255.0;
					points_color[3*idx+1] = img.at<cv::Vec3b>(i+offsetY_bw_rgb_depth, j+offsetX_bw_rgb_depth)[1]/255.0;
					points_color[3*idx+2] = img.at<cv::Vec3b>(i+offsetY_bw_rgb_depth, j+offsetX_bw_rgb_depth)[0]/255.0;
				}
				else
				{
					points_color[3*idx] = 0;
					points_color[3*idx+1] = 0;
					points_color[3*idx+2] = 0;
				}
			}

			idx ++;
		}
	}

	double *cam_cen_ = cam->getCameraCenter();
	T cam_cen[3];
	cam_cen[0] = cam_cen_[0];
	cam_cen[1] = cam_cen_[1];
	cam_cen[2] = cam_cen_[2];
	T lookat[3];

	vector<int> triangles_vec;
	for(int i=0; i<h-1; i++)
	{
		for(int j=0; j<w-1; j++)
		{
			int idx1 = i*w + j;
			int idx2 = i*w + (j+1);
			int idx3 = (i+1)*w + (j+1);

			double z1 = depthMap.at<double>(i, j);;
			double z2 = depthMap.at<double>(i, j+1);
			double z3 = depthMap.at<double>(i+1, j+1);

			if( z1 <= 0.0 || z2 <= 0.0 || z3 <= 0.0 )
				continue;

			//if either two of three points are far away from each other, skip
			if( neighbor_pixel_max_dist > 0 &&
/*				(fabs(z1 - z2) > MIN(z1 / fx * 4, neighbor_pixel_max_dist) ||
				 fabs(z2 - z3) > MIN(z1 / fx * 4, neighbor_pixel_max_dist) ||
				 fabs(z1 - z3) > MIN(z1 / fx * 4, neighbor_pixel_max_dist))*/
				 (fabs(z1 - z2) >  neighbor_pixel_max_dist ||
				  fabs(z2 - z3) >  neighbor_pixel_max_dist ||
				  fabs(z1 - z3) >  neighbor_pixel_max_dist))
				continue;

			//keep in the counter-clockwise order			
			int i1 = idx_array[idx1];
			int i2 = idx_array[idx2];
			int i3 = idx_array[idx3];
			T *v1 = &points_in_world[i1*3];
			T *v2 = &points_in_world[i2*3];
			T *v3 = &points_in_world[i3*3];

			VecOperation<T>::VecSub(v1, cam_cen, lookat, 3);
			if( !is_clockwise_tri(lookat, v1, v2, v3) )
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i2);
				triangles_vec.push_back(i3);
			}
			else
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i3);
				triangles_vec.push_back(i2);
			}
		}
	}

	for(int i=0; i<h-1; i++)
	{
		for(int j=0; j<w-1; j++)
		{
			int idx1 = i*w + j;
			int idx2 = (i+1)*w + j;
			int idx3 = (i+1)*w + (j+1);

			double z1 = depthMap.at<double>(i, j);;
			double z2 = depthMap.at<double>(i+1, j);
			double z3 = depthMap.at<double>(i+1, j+1);

			if( z1 <= 0.0 || z2 <= 0.0 || z3 <= 0.0 )
				continue;

			//if either two of three points are far away from each other, skip
			if( neighbor_pixel_max_dist > 0 &&
/*				(fabs(z1 - z2) > MIN(z1 / fx * 4, neighbor_pixel_max_dist) ||
				 fabs(z2 - z3) > MIN(z1 / fx * 4, neighbor_pixel_max_dist) ||
				 fabs(z1 - z3) > MIN(z1 / fx * 4, neighbor_pixel_max_dist)*/ 
				 (fabs(z1 - z2) > neighbor_pixel_max_dist ||
				  fabs(z2 - z3) > neighbor_pixel_max_dist ||
				  fabs(z1 - z3) > neighbor_pixel_max_dist))
				continue;

			//keep in the counter-clockwise order			
			int i1 = idx_array[idx1];
			int i2 = idx_array[idx2];
			int i3 = idx_array[idx3];
			T *v1 = &points_in_world[i1*3];
			T *v2 = &points_in_world[i2*3];
			T *v3 = &points_in_world[i3*3];
			
			VecOperation<T>::VecSub(v1, cam_cen, lookat, 3);
			if( !is_clockwise_tri(lookat, v1, v2, v3) )
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i2);
				triangles_vec.push_back(i3);
			}
			else
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i3);
				triangles_vec.push_back(i2);
			}
		}
	}

	CSurface<T> *ret = new CSurface<T>();
	int vtDim = 0;
	if(!img.empty())
		vtDim = 6;
	else
		vtDim = 3;

	T *vtData = new T[vtDim*points_num];
	for(int i=0; i<points_num; i++)
	{
		vtData[vtDim*i] = points_in_world[3*i];
		vtData[vtDim*i+1] = points_in_world[3*i+1];
		vtData[vtDim*i+2] = points_in_world[3*i+2];
	}
	if(!img.empty())
	{
		for(int i=0; i<points_num; i++)
		{
			vtData[vtDim*i+3] = points_color[3*i];
			vtData[vtDim*i+4] = points_color[3*i+1];
			vtData[vtDim*i+5] = points_color[3*i+2];
		}
	}
	
	int tri_num = triangles_vec.size()/3;
	int *triangles = NULL;
	if(tri_num > 0)
		triangles = new int[triangles_vec.size()];
	for(int i=0; i<tri_num*3; i++)
		triangles[i] = triangles_vec[i];

	if(!img.empty())
		ret->color = true;
	else 
		ret->color = false;
	ret->normal = false;
	ret->vtNum = points_num;
	ret->vtData = vtData;
	ret->vtDim = vtDim;
	ret->triNum = tri_num;
	ret->triangles = triangles;

	if(bNormal)
		ret->generateNormals();

	delete [] points_in_world;
	delete [] points_color;
	delete [] idx_array;
	delete [] cam_cen_;

	return ret;
}

template<class T>
CSurface<T> *DepthMap2SurfaceOrtho(cv::Mat const& depthMap, cv::Mat const& img, vnl_matrix<vnl_vector_fixed<double, 3>> const* normalMap,
	double pixel_dist, double neighbor_pixel_max_dist)
{
	int h = depthMap.rows;
	int w = depthMap.cols;

	int points_num = 0;
	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			if (depthMap.at<double>(i, j); <= 0.0)
				continue;

			points_num++;
		}
	}

	T *points = new T[3 * points_num];
	T *colors = NULL;
	if (img != NULL)
		colors = new T[3 * points_num];
	T *normals = NULL;
	if (normalMap != NULL)
		normals = new T[3 * points_num];
	int *idx_array = new int[w*h];
	int idx = 0;
	for (int i = 0; i<h; i++)
	{
		for (int j = 0; j<w; j++)
		{
			double z = depthMap.at<double>(i, j);;
			// if z value is invalid, skip
			if (z <= 0.0)
			{
				idx_array[i*w + j] = -1;
				continue;
			}

			points[3 * idx] = (j - w / 2)*pixel_dist;
			points[3 * idx + 1] = (i - h / 2)*pixel_dist;
			points[3 * idx + 2] = z;

			idx_array[i*w + j] = idx;

			if (img != NULL)
			{
				colors[3 * idx] = img.at<cv::Vec3b>(i, j)[2] / 255.0;
				colors[3 * idx + 1] = img.at<cv::Vec3b>(i, j)[1] / 255.0;
				colors[3 * idx + 2] = img.at<cv::Vec3b>(i, j)[0] / 255.0;
			}

			if (normalMap != NULL)
			{
				normals[3 * idx]     = (*normalMap)[i][j][0];
				normals[3 * idx + 1] = (*normalMap)[i][j][1];
				normals[3 * idx + 2] = (*normalMap)[i][j][2];
			}

			idx++;
		}
	}

	T lookat[3] = {0.0, 0.0, 1.0};

	vector<int> triangles_vec;
	for (int i = 0; i<h - 1; i++)
	{
		for (int j = 0; j<w - 1; j++)
		{
			int idx1 = i*w + j;
			int idx2 = i*w + (j + 1);
			int idx3 = (i + 1)*w + (j + 1);

			double z1 = depthMap.at<double>(i, j);;
			double z2 = depthMap.at<double>(i, j + 1);
			double z3 = depthMap.at<double>(i + 1, j + 1);

			if (z1 <= 0.0 || z2 <= 0.0 || z3 <= 0.0)
				continue;

			//if either two of three points are far away from each other, skip
			if (neighbor_pixel_max_dist > 0 &&
				( fabs(z1 - z2) >  neighbor_pixel_max_dist ||
				  fabs(z2 - z3) > neighbor_pixel_max_dist ||
				  fabs(z1 - z3) > neighbor_pixel_max_dist ) )
				continue;

			//keep in the counter-clockwise order			
			int i1 = idx_array[idx1];
			int i2 = idx_array[idx2];
			int i3 = idx_array[idx3];
			T *v1 = &points[i1 * 3];
			T *v2 = &points[i2 * 3];
			T *v3 = &points[i3 * 3];

			if (!is_clockwise_tri(lookat, v1, v2, v3))
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i2);
				triangles_vec.push_back(i3);
			}
			else
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i3);
				triangles_vec.push_back(i2);
			}
		}
	}

	for (int i = 0; i<h - 1; i++)
	{
		for (int j = 0; j<w - 1; j++)
		{
			int idx1 = i*w + j;
			int idx2 = (i + 1)*w + j;
			int idx3 = (i + 1)*w + (j + 1);

			double z1 = depthMap.at<double>(i, j);;
			double z2 = depthMap.at<double>(i + 1, j);
			double z3 = depthMap.at<double>(i + 1, j + 1);

			if (z1 <= 0.0 || z2 <= 0.0 || z3 <= 0.0)
				continue;

			//if either two of three points are far away from each other, skip
			if (neighbor_pixel_max_dist > 0 &&
				(fabs(z1 - z2) >  neighbor_pixel_max_dist ||
				fabs(z2 - z3) > neighbor_pixel_max_dist ||
				fabs(z1 - z3) > neighbor_pixel_max_dist))
				continue;

			//keep in the counter-clockwise order			
			int i1 = idx_array[idx1];
			int i2 = idx_array[idx2];
			int i3 = idx_array[idx3];
			T *v1 = &points[i1 * 3];
			T *v2 = &points[i2 * 3];
			T *v3 = &points[i3 * 3];

			if (!is_clockwise_tri(lookat, v1, v2, v3))
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i2);
				triangles_vec.push_back(i3);
			}
			else
			{
				triangles_vec.push_back(i1);
				triangles_vec.push_back(i3);
				triangles_vec.push_back(i2);
			}
		}
	}

	CSurface<T> *ret = new CSurface<T>();
	int vtDim = 3;
	if (img != NULL)
		vtDim += 3;
	if (normalMap != NULL)
		vtDim += 3;

	T *vtData = new T[vtDim*points_num];
	for (int i = 0; i<points_num; i++)
	{
		vtData[vtDim*i] = points[3 * i];
		vtData[vtDim*i + 1] = points[3 * i + 1];
		vtData[vtDim*i + 2] = points[3 * i + 2];
	}
	if (img != NULL)
	{
		for (int i = 0; i<points_num; i++)
		{
			vtData[vtDim*i + 3] = colors[3 * i];
			vtData[vtDim*i + 4] = colors[3 * i + 1];
			vtData[vtDim*i + 5] = colors[3 * i + 2];
		}
	}
	if (normalMap != NULL)
	{
		for (int i = 0; i<points_num; i++)
		{
			vtData[vtDim*(i + 1) - 3] = normals[3 * i];
			vtData[vtDim*(i + 1) - 2] = normals[3 * i + 1];
			vtData[vtDim*(i + 1) - 1] = normals[3 * i + 2];
		}
	}

	int tri_num = triangles_vec.size() / 3;
	int *triangles = NULL;
	if (tri_num > 0)
		triangles = new int[triangles_vec.size()];
	for (int i = 0; i<tri_num * 3; i++)
		triangles[i] = triangles_vec[i];

	ret->color = (img != NULL) ? true : false;
	ret->normal = (normalMap != NULL) ? true : false;
	ret->vtNum = points_num;
	ret->vtData = vtData;
	ret->vtDim = vtDim;
	ret->triNum = tri_num;
	ret->triangles = triangles;

	delete[] points;
	if (colors != NULL) delete[] colors;
	if (normals != NULL) delete[] normals;
	delete[] idx_array;
	return ret;
}

template<class T>
bool DepthMap2Surface(  vnl_matrix<double> const& depthMat, 
						vpgl_perspective_camera<double> const &cam, 
						cv::Mat const& img,
					    CSurface<T> &m_surface, 
						double neighbor_pixel_max_dist, 
						int offsetX_bw_rgb_detph, int offsetY_bw_rgb_depth, 
						double a, double b, //a and b is working on depth value z = a*z+b
						bool bNormal)
{
	vnl_matrix_fixed<double, 3, 3> intrinsics = cam.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> t;
	get_camera_pose(cam, R, t);
	GCameraView *cam_view = new GCameraView(depthMat.rows(), depthMat.cols());
	cam_view->SetCalibrationMatrix(intrinsics.data_block(), R.data_block(), t.data_block());
	cv::Mat depth_mat = vnlmatrix_to_cvmat(depthMat);
	CSurface<T>* tmp = DepthMap2Surface<T>(cam_view, depth_mat, img, neighbor_pixel_max_dist, offsetX_bw_rgb_detph, offsetY_bw_rgb_depth, a, b, bNormal);
	if(tmp == NULL)
	{
		delete cam_view;
		depth_mat.release();
		return false;
	}

	m_surface = *tmp;

	depth_mat.release();
	delete tmp;
	delete cam_view;
	return true;
}

template<class T>
bool DepthMap2Surface( vpgl_perspective_camera<double> const& cam, cv::Mat const& depthMap_undistort, cv::Mat const& img,
					   CSurface<T> &m_surface,
					   double neighbor_pixel_max_dist, int offsetX_bw_rgb_detph, int offsetY_bw_rgb_depth, 
					   double a, double b, //a and b is working on depth value z = a*z+b
					   bool bNormal)
{
	if( depthMap_undistort == NULL )
	{
		printf("Error<DepthMap2Surface>: depthMat is NULL!\n");
		return false;
	}

	GCameraView *cam_view = vpgl_camera_view_to_GCameraView(cam, vnl_vector_fixed<double, 5>(0.0), depthMap_undistort.cols, depthMap_undistort.rows);
	CSurface<T>* tmp = DepthMap2Surface<T>(cam_view, depthMap_undistort, img, neighbor_pixel_max_dist, offsetX_bw_rgb_detph, offsetY_bw_rgb_depth, a, b, bNormal);
	if(tmp == NULL)
	{
		delete cam_view;
		return false;
	}
	else
	{
		m_surface = *tmp;
		delete tmp;
		delete cam_view;
		return true;
	}
}

//template<class T>
//cv::Mat* ComputeDepthMap(CSurface<T> const*m_surf, GCameraView const* cam, bool bRemoveCounterClockTri)
template<class T>
cv::Mat ComputeDepthMap( CSurface<T> const*m_surf, GCameraView const* cam, 
						triangle_removal_method remove_tri_method,
						bool bDistortDepthMap,
						double near_clip_plane)
{
	//printf("compute depth map...");
	int height = cam->m_Height;
	int width = cam->m_Width;
	cv::Mat depthMap = cv::Mat(height, width, CV_64F, cv::Scalar(0));
	cv::Mat mask_oppositetri = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
	
	double *cam_cen_ = cam->getCameraCenter();
	T cam_cen[3];
	cam_cen[0] = cam_cen_[0];
	cam_cen[1] = cam_cen_[1];
	cam_cen[2] = cam_cen_[2];
	T lookat[3];

	int vtDim = m_surf->vtDim;
	T* vtData = m_surf->vtData;
	for(int i=0; i<m_surf->triNum; i++)
	{
		int idx1 = m_surf->triangles[3*i];
		int idx2 = m_surf->triangles[3*i+1];
		int idx3 = m_surf->triangles[3*i+2];
		T *v1 = &vtData[idx1*vtDim];
		T *v2 = &vtData[idx2*vtDim];
		T *v3 = &vtData[idx3*vtDim];

		if( remove_tri_method != remove_none_tri )
		{
			VecOperation<T>::VecSub(v1, cam_cen, lookat, 3);
			bool tri_order = is_clockwise_tri(lookat, v1, v2, v3);
			if( tri_order && remove_tri_method == remove_clockwise_tri ||
				!tri_order && remove_tri_method == remove_counterclock_tri )
			{
				//printf(".");
				project_a_triangle_onto_cam<T, T>(cam, v1, v2, v3, depthMap, NULL, NULL, NULL, NULL, bDistortDepthMap, mask_oppositetri, true, near_clip_plane);		
				continue;
			}
		}

		project_a_triangle_onto_cam<T, T>(cam, v1, v2, v3, depthMap, NULL, NULL, NULL, NULL, bDistortDepthMap, mask_oppositetri, false, near_clip_plane);		
	}

	//remove the depth pixels due to opposite triangles
	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			if( mask_oppositetri.at<uchar>(i, j) != 0 )
			{
				depthMap.at<double>(i, j) = 0.0;
			}
		}
	}
	//printf("end!\n");
	mask_oppositetri.release();
	delete [] cam_cen_;
	return depthMap;
}

template<class T>
cv::Mat ComputeDepthMap( vector< CSurface<T>* > const& m_surfs, GCameraView const* cam, 
						triangle_removal_method remove_tri_method,
						bool bDistortDepthMap,
						double near_clip_plane)
{
	//printf("compute depth map...");
	int height = cam->m_Height;
	int width = cam->m_Width;
	cv::Mat depthMap = cv::Mat(height, width, CV_64F, cv::Scalar(0));
	cv::Mat mask_oppositetri = cv::Mat(height, width, CV_8UC1, cv::Scalar(0.0));
	
	double *cam_cen_ = cam->getCameraCenter();
	T cam_cen[3];
	cam_cen[0] = cam_cen_[0];
	cam_cen[1] = cam_cen_[1];
	cam_cen[2] = cam_cen_[2];
	T lookat[3];

	for(int surfIdx=0; surfIdx<m_surfs.size(); surfIdx++)
	{
		CSurface<float> * m_surf = m_surfs[surfIdx];
		int vtDim = m_surf->vtDim;
		T* vtData = m_surf->vtData;
		for(int i=0; i<m_surf->triNum; i++)
		{
			int idx1 = m_surf->triangles[3*i];
			int idx2 = m_surf->triangles[3*i+1];
			int idx3 = m_surf->triangles[3*i+2];
			T *v1 = &vtData[idx1*vtDim];
			T *v2 = &vtData[idx2*vtDim];
			T *v3 = &vtData[idx3*vtDim];

			if( remove_tri_method != remove_none_tri )
			{
				VecOperation<T>::VecSub(v1, cam_cen, lookat, 3);
				bool tri_order = is_clockwise_tri(lookat, v1, v2, v3);
				if( tri_order && remove_tri_method == remove_clockwise_tri ||
					!tri_order && remove_tri_method == remove_counterclock_tri )
				{
					//printf(".");
					project_a_triangle_onto_cam<T, T>(cam, v1, v2, v3, depthMap, NULL, NULL, NULL, NULL, bDistortDepthMap, mask_oppositetri, true, near_clip_plane);		
					continue;
				}
			}

			project_a_triangle_onto_cam<T, T>(cam, v1, v2, v3, depthMap, NULL, NULL, NULL, NULL, bDistortDepthMap, mask_oppositetri, false, near_clip_plane);		
		}
	}

	//remove the depth pixels due to opposite triangles
	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			if( mask_oppositetri.at<uchar>(i, j) != 0 )
			{
				depthMap.at<double>(i, j) = 0.0;
			}
		}
	}
	//printf("end!\n");
	mask_oppositetri.release();
	delete [] cam_cen_;
	return depthMap;
}

template<class T>
void ProjectSurface( vector< CSurface<T>* > const& m_surfs, GCameraView const* cam,
					 cv::Mat& depth_mat, //projected depth map (could be set to NULL)
					 cv::Mat& img_proj,//projected color image (could be set to NULL)
					 int color_view_idx,// which one to choose for color: channels of surface.colors_of_view or surface.vtData(-1)
					 triangle_removal_method remove_tri_method,
					 bool bDistortDepthMap,
					 double near_clip_plane)
{
	if( depth_mat.empty() && img_proj.empty() )
		return;

	int height = cam->m_Height;
	int width = cam->m_Width;

	cv::Mat depth_mat_intern = cv::Mat(height, width, CV_64F, cv::Scalar(0));

	cv::Mat img_proj_intern = cv::Mat(height, width, CV_8UC3, cv::Scalar(125, 125, 125));

	cv::Mat mask_oppositetri = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
	
	double *cam_cen_ = cam->getCameraCenter();
	T cam_cen[3];
	cam_cen[0] = cam_cen_[0];
	cam_cen[1] = cam_cen_[1];
	cam_cen[2] = cam_cen_[2];
	T lookat[3];

	for(int surfIdx=0; surfIdx<m_surfs.size(); surfIdx++)
	{
		CSurface<float> * m_surf = m_surfs[surfIdx];
		int vtDim = m_surf->vtDim;
		T* vtData = m_surf->vtData;
		for(int i=0; i<m_surf->triNum; i++)
		{
			int idx1 = m_surf->triangles[3*i];
			int idx2 = m_surf->triangles[3*i+1];
			int idx3 = m_surf->triangles[3*i+2];
			T *v1 = &vtData[idx1*vtDim];
			T *v2 = &vtData[idx2*vtDim];
			T *v3 = &vtData[idx3*vtDim];

			float *clr1 = NULL;
			float *clr2 = NULL;
			float *clr3 = NULL;
			if( m_surf->haveColorInfo() ){
				clr1 = m_surf->vt_color(idx1);
				clr2 = m_surf->vt_color(idx2);
				clr3 = m_surf->vt_color(idx3);
			}
			if( color_view_idx >= 0 && color_view_idx < m_surf->colors_of_view.size() ){
				float *colors = m_surf->colors_of_view[color_view_idx];
				if( colors != NULL ){
					if( colors[3*idx1] != 0 || colors[3*idx1+1] != 0 || colors[3*idx1+2] != 0 )
						clr1 = &colors[3*idx1];
					if( colors[3*idx2] != 0 || colors[3*idx2+1] != 0 || colors[3*idx2+2] != 0 )
						clr2 = &colors[3*idx2];
					if( colors[3*idx3] != 0 || colors[3*idx3+1] != 0 || colors[3*idx3+2] != 0 )
						clr3 = &colors[3*idx3];
				}
			}

			if( remove_tri_method != remove_none_tri )
			{
				VecOperation<T>::VecSub(v1, cam_cen, lookat, 3);
				bool tri_order = is_clockwise_tri(lookat, v1, v2, v3);
				if( tri_order && remove_tri_method == remove_clockwise_tri ||
					!tri_order && remove_tri_method == remove_counterclock_tri )
				{
					//printf(".");
					project_a_triangle_onto_cam<T, float>(cam, v1, v2, v3, depth_mat_intern, clr1, clr2, clr3, img_proj_intern, bDistortDepthMap, mask_oppositetri, true, near_clip_plane);		
					continue;
				}
			}

			project_a_triangle_onto_cam<T, float>(cam, v1, v2, v3, depth_mat_intern, clr1, clr2, clr3, img_proj_intern, bDistortDepthMap, mask_oppositetri, false, near_clip_plane);		
		}
	}

	//remove the depth pixels due to opposite triangles
	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			if( mask_oppositetri.at<uchar>(i, j) != 0 )
			{
				depth_mat_intern.at<double>(i, j) = 0.0;
				img_proj_intern.at<cv::Vec3b>(i, j)[0] = 125;
				img_proj_intern.at<cv::Vec3b>(i, j)[1] = 125;
				img_proj_intern.at<cv::Vec3b>(i, j)[2] = 125;
			}
		}
	}

	if( !depth_mat.empty())
		depth_mat_intern.copyTo(depth_mat);
	if (!img_proj.empty())
		img_proj_intern.copyTo(img_proj);

	mask_oppositetri.release();
	img_proj_intern.release();
	depth_mat_intern.release();
	delete [] cam_cen_;
}

template<class T>
cv::Mat ComputeDepthMap(CSurface<T> const*m_surf, vnl_matrix_fixed<double, 3, 4> const& prj_mat, int width, int height)
{
	//printf("compute depth map...");
	cv::Mat depthMap = cv::Mat(height, width, CV_64F, cv::Scalar(0));

	int vtDim = m_surf->vtDim;
	T* vtData = m_surf->vtData;
	for(int i=0; i<m_surf->triNum; i++)
	{
		int idx1 = m_surf->triangles[3*i];
		int idx2 = m_surf->triangles[3*i+1];
		int idx3 = m_surf->triangles[3*i+2];
		T *v1 = &vtData[idx1*vtDim];
		T *v2 = &vtData[idx2*vtDim];
		T *v3 = &vtData[idx3*vtDim];

		project_a_triangle_onto_cam(prj_mat, v1, v2, v3, depthMap, width, height);		
	}
	return depthMap;
}


template<class T>
bool ComputeDepthMap( CSurface<T> const&m_surface, vpgl_perspective_camera<double> const& cam, 
					  int width, int height,
					  vnl_matrix<double> &depthMat, 
					  vnl_vector<double> const& distort_vec,
					  triangle_removal_method remove_tri,
					  bool bDistortDepthMap,
					  double near_clip_plane)
{
	vnl_matrix_fixed<double, 3, 3> intrinsic = cam.get_calibration().get_matrix();
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	get_camera_pose(cam, R, T);
	vnl_vector<double> radial_distort(5);
	radial_distort.fill(0.0);
	for(int i=0; i<MIN(distort_vec.size(), 5); i++)
		radial_distort[i] = distort_vec[i];
	
	GCameraView cam_view(height, width);
	cam_view.SetCalibrationMatrix(intrinsic.data_block(), R.data_block(), T.data_block(), distort_vec.data_block());


	cv::Mat depth_mat = ComputeDepthMap(&m_surface, &cam_view, remove_tri, bDistortDepthMap, near_clip_plane);
	bool ret = cvmat_to_vnlmatrix(depth_mat, depthMat);
	depth_mat.release();
	return ret;
}



template<class T1, class T2>
void project_a_triangle_onto_cam( GCameraView const* cam, 
								  T1 *v1, T1 *v2, T1 *v3, 
								  cv::Mat& depthMap,
								  T2 *clr1, T2 *clr2, T2 *clr3,
								  cv::Mat *img_proj,
								  bool bDistortDepthMap,
								  cv::Mat& mask_front_oppositetri, bool bOppositeTri,
								  double near_clip_plane)
{
	int height = cam->m_Height;
	int width = cam->m_Width;
	// project vertices onto the image plane
	T1 p1[3];
	T1 p2[3];
	T1 p3[3];

	cam->worldSpace2ImageCoordV3(v1, p1);
	cam->worldSpace2ImageCoordV3(v2, p2);
	cam->worldSpace2ImageCoordV3(v3, p3);

	if( bDistortDepthMap )
	{
		if( p1[0] > -20 && p1[0] < width+20 && //will lead to artifact without this constraint
			p1[1] > -20 && p1[1] < height+20)
			cam->World2ImgWithDistortionV3(v1, p1);
		if( p2[0] > -20 && p2[0] < width+20 &&
			p2[1] > -20 && p2[1] < height+20)
			cam->World2ImgWithDistortionV3(v2, p2);
		if( p3[0] > -20 && p3[0] < width+20 &&
			p3[1] > -20 && p3[1] < height+20)
			cam->World2ImgWithDistortionV3(v3, p3);
	}
	
	T1 x_L = MIN(MIN(p1[0], p2[0]), p3[0]); //left most x
	T1 x_R = MAX(MAX(p1[0], p2[0]), p3[0]); //right most x
	T1 y_B = MIN(MIN(p1[1], p2[1]), p3[1]); //bottom most y
	T1 y_T = MAX(MAX(p1[1], p2[1]), p3[1]); //top most y
	T1 z_N = MIN(MIN(p1[2], p2[2]), p3[2]); //nearest z
	T1 z_F = MAX(MAX(p1[2], p2[2]), p3[2]); //futher y
	
	//if the triangle is not fully fallen into the image or the point is behind the camera, then skip it
	if( x_L > width-1 || x_R < 0 || y_B > height-1 || y_T < 0 || z_N < near_clip_plane)
		return;

	for(int y=MAX( ROUND(y_B), 0); y <= MIN( ROUND(y_T), height-1); y++)	
	{
		for(int x=MAX(ROUND(x_L), 0); x <= MIN( ROUND(x_R), width-1 ); x++)
		{
			double c = ((x-p1[0])*(p2[1]-p1[1])-(p2[0]-p1[0])*(y-p1[1])) / (double)((p2[1]-p1[1])*(p3[0]-p1[0])-(p2[0]-p1[0])*(p3[1]-p1[1]));
			double b = ((x-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(y-p1[1])) / (double)((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]));
			double a = 1.0-b-c;

			//if(c<0||b<0||a<0||c>1||b>1||a>1)
			if(c<-EPS||b<-EPS||a<-EPS||c>1.0+EPS||b>1.0+EPS||a>1.0+EPS)
				continue;
			
			/* perspective-correctly interpolate the depth */
			double z = 1.0 / (a * 1.0 / p1[2] + b * 1.0 / p2[2] + c * 1.0 / p3[2]);

			double clr[3];//color
			if( clr1 != NULL && clr2 != NULL && clr3 != NULL &&
				img_proj != NULL )
			{
				clr[0] = a*clr1[0] + b*clr2[0] + c*clr3[0];
				clr[1] = a*clr1[1] + b*clr2[1] + c*clr3[1];
				clr[2] = a*clr1[2] + b*clr2[2] + c*clr3[2];
			}

			if( z <= 0.0 )
			{
				LOGGER()->warning("project_a_triangle_onto_cam", "negative z!");
				continue;
			}

			//update z-buffer and color and mask_front_opposite_tri
			if( depthMap.at<double>(y, x) <= 0.0  ||
				z < depthMap.at<double>(y, x) )
			{
				depthMap.at<double>(y, x) = z;

				if( clr1 != NULL && clr2 != NULL && clr3 != NULL &&
					img_proj != NULL )
				{
					assert( img_proj->channels() == 3);
					img_proj->at<cv::Vec3b>(y, x)[0] = clr[2]*255.0;
					img_proj->at<cv::Vec3b>(y, x)[1] = clr[1]*255.0;
					img_proj->at<cv::Vec3b>(y, x)[2] = clr[0]*255.0;
				}

				if( !mask_front_oppositetri.empty())
				{
					if( bOppositeTri )
						mask_front_oppositetri.at<uchar>(y, x) = 255;
					else
						mask_front_oppositetri.at<uchar>(y, x) = 0;
				}
			}

		}//end of for-x
	}//end of for-y
}

template<class T>
void project_a_triangle_onto_cam( vnl_matrix_fixed<double, 3, 4> const&prj_mat, T v1[], T v2[], T v3[], 
								  cv::Mat& depthMap, int width, int height)
{
	// project vertices onto the image plane
	vnl_vector_fixed<double, 3> p1 = prj_mat * vnl_vector_fixed<double, 4>(v1[0], v1[1], v1[2], 1.0);
	vnl_vector_fixed<double, 3> p2 = prj_mat * vnl_vector_fixed<double, 4>(v2[0], v2[1], v2[2], 1.0);
	vnl_vector_fixed<double, 3> p3 = prj_mat * vnl_vector_fixed<double, 4>(v3[0], v3[1], v3[2], 1.0);
	p1[0] /= p1[2]; p1[1] /= p1[2];
	p2[0] /= p2[2]; p2[1] /= p2[2];
	p3[0] /= p3[2]; p3[1] /= p3[2];

	
	double x_L = MIN(MIN(p1[0], p2[0]), p3[0]); //left most x
	double x_R = MAX(MAX(p1[0], p2[0]), p3[0]); //right most x
	double y_B = MIN(MIN(p1[1], p2[1]), p3[1]); //bottom most y
	double y_T = MAX(MAX(p1[1], p2[1]), p3[1]); //top most y
	double z_N = MIN(MIN(p1[2], p2[2]), p3[2]); //nearest z
	double z_F = MAX(MAX(p1[2], p2[2]), p3[2]); //futher y

	//if the triangle is not fully fallen into the image, then skip it
	if( x_L > width-1 || x_R < 0 || y_B > height-1 || y_T < 0 || z_N < 0)
		return;

	for(int y=MAX( ROUND(y_B), 0); y <= MIN( ROUND(y_T), height-1); y++)	
	{
		for(int x=MAX(ROUND(x_L), 0); x <= MIN( ROUND(x_R), width-1 ); x++)
		{
			double c = ((x-p1[0])*(p2[1]-p1[1])-(p2[0]-p1[0])*(y-p1[1])) / (double)((p2[1]-p1[1])*(p3[0]-p1[0])-(p2[0]-p1[0])*(p3[1]-p1[1]));
			double b = ((x-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(y-p1[1])) / (double)((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]));
			double a = 1.0-b-c;

			//if(c<0||b<0||a<0||c>1||b>1||a>1)
			if(c<-EPS||b<-EPS||a<-EPS||c>1.0+EPS||b>1.0+EPS||a>1.0+EPS)
				continue;
			
			/* perspective-correctly interpolate the depth */
			double z = 1.0 / (a * 1.0 / p1[2] + b * 1.0 / p2[2] + c * 1.0 / p3[2]);
			if( z < 0.0 )
			{
				LOGGER()->warning("project_a_triangle_onto_cam", "negative z!");
				continue;
			}
			if( depthMap.at<double>(y, x); <= 0.0 )
				depthMap.at<double>(y, x) = z;
			else if( z < depthMap.at<double>(y, x); )
				depthMap.at<double>(y, x) = z;
		}
	}
}

template<class T>
inline T triangle_area(T *p1, T *p2, T *p3)
{
	T e1[3];
	T e2[3];
	VecOperation<T>::VecSub(p2, p1, e1, 3);
	VecOperation<T>::VecSub(p3, p1, e2, 3);
	T c[3];
	VecOperation<T>::CrossProdVec3(e1, e2, c);
	T ret = VecOperation<T>::Norm(c, 3);
	return ret;
}

template<class T>
void RemoveUnnecessaryPolygon(CSurface<T> *m_surf, GCameraView *cam1)
{
	int vtDim = m_surf->vtDim;
	int *triangles = m_surf->triangles;
	T *vtData = m_surf->vtData;
	
	vector<int> unqualified_triangles;
	//check the area of triangles
	for(int i=0; i<m_surf->triNum; i++)
	{
		int idx1 = triangles[3*i];
		int idx2 = triangles[3*i+1];
		int idx3 = triangles[3*i+2];
		
		T *p1 = &vtData[idx1*vtDim];
		T *p2 = &vtData[idx2*vtDim]; 
		T *p3 = &vtData[idx3*vtDim]; 

		T area = triangle_area(p1, p2, p3); //unit is cm^2
		if(area > 0.6) 
			unqualified_triangles.push_back(i);
	}

	int new_triNum = m_surf->triNum - unqualified_triangles.size();
	int *new_triangles = new int[new_triNum*3];
	
	//take the advantage of the fact that the indices in unqualified_triangles
	//are in acsending order
	int c = 0;
	int k = 0;
	for(int i=0; i<m_surf->triNum; i++)
	{
		if(c<unqualified_triangles.size() && i>=unqualified_triangles[c])
		{
			c++;
			continue;
		}
		
		new_triangles[3*k] = triangles[3*i];
		new_triangles[3*k+1] = triangles[3*i+1];
		new_triangles[3*k+2] = triangles[3*i+2];
		k++;
	}
	
	delete[] triangles;
	m_surf->triangles = new_triangles;
	m_surf->triNum = new_triNum;
}

template<class T>
bool RemoveStretchedPolygon(CSurface<T> &surface_t, CSurface<T> const&surface, double thres_stretch, double thres_stretch_abs)
{
	if (surface_t.vtNum != surface.vtNum )
	{
		printf("Error<RemoveStretchedPolygon>: surface_t and surface do not have same number of vertices");
		return false;
	}

	std::vector<bool> bRemove(surface_t.triNum, false);
	int tri_num_new = 0;
	for (int triIdx = 0; triIdx < surface_t.triNum; triIdx++)
	{
		int vtIdx1 = surface_t.triangles[3 * triIdx];
		int vtIdx2 = surface_t.triangles[3 * triIdx + 1];
		int vtIdx3 = surface_t.triangles[3 * triIdx + 2];

		vnl_vector_fixed<T, 3> vt1_t(surface_t.vt_data_block(vtIdx1));
		vnl_vector_fixed<T, 3> vt2_t(surface_t.vt_data_block(vtIdx2));
		vnl_vector_fixed<T, 3> vt3_t(surface_t.vt_data_block(vtIdx3));

		vnl_vector_fixed<T, 3> vt1(surface.vt_data_block(vtIdx1));
		vnl_vector_fixed<T, 3> vt2(surface.vt_data_block(vtIdx2));
		vnl_vector_fixed<T, 3> vt3(surface.vt_data_block(vtIdx3));

		double d_12 = dist_3d(vt1_t, vt2_t);
		double r_12 = d_12 / dist_3d(vt1, vt2);
		if (r_12 > thres_stretch && d_12 > thres_stretch_abs)
		{
			bRemove[triIdx] = true;
			continue;
		}

		double d_13 = dist_3d(vt1_t, vt3_t);
		double r_13 = d_13 / dist_3d(vt1, vt3);
		if (r_13 > thres_stretch && d_13 > thres_stretch_abs)
		{
			bRemove[triIdx] = true;
			continue;
		}

		double d_23 = dist_3d(vt2_t, vt3_t);
		double r_23 = d_23 / dist_3d(vt2, vt3);
		if (r_23 > thres_stretch && d_23 > thres_stretch_abs)
		{
			bRemove[triIdx] = true;
			continue;
		}

		tri_num_new++;
	}

	int *triangles_new = new int[3 * tri_num_new];
	int count = 0;
	for (int i = 0; i < surface_t.triNum; i++)
	{
		if (!bRemove[i])
		{
			triangles_new[3 * count] = surface_t.triangles[3 * i];
			triangles_new[3 * count + 1] = surface_t.triangles[3 * i + 1];
			triangles_new[3 * count + 2] = surface_t.triangles[3 * i + 2];
			count++;
		}
	}

	delete [] surface_t.triangles;
	surface_t.triangles = triangles_new;
	surface_t.triNum = tri_num_new;
	return true;
}

template<class T>
void FillColorInfoNCams(CSurface<T> *m_surf, vector<GCameraView *>cams, vector<cv::Mat>imgs[], int camNum, bool bImgWithDistortion, bool bBlending, double gamma)
{
	if( m_surf == NULL || cams==NULL || imgs == NULL)
	{
		printf("Error: input is invalid!\n");
		return;
	}
	
	for(int i=0; i<camNum; i++)
	{
		if( cams[i]==NULL || imgs[i].empty())
		{
			printf("Error: input is invalid!\n");
			return;
		}
		if( imgs[i].rows != cams[i]->m_Height || imgs[i].cols != cams[i]->m_Width)
		{
			printf("Error: the image size does not match with camera's parameter<%d, %d>, <%d, %d>!\n", 
				imgs[i].rows, imgs[i].cols, cams[i]->m_Height, cams[i]->m_Width);
			return;
		}
	}

	if(m_surf->haveColorInfo())
	{
		printf("Warning: there is already color info in the triangle mesh!\n");
		//return;
	}

	//compute the depth map for all the views
	vector<cv::Mat> depthMaps = vector<cv::Mat>(camNum);
	for(int i=0; i<camNum; i++)
	{
		depthMaps[i] = ComputeDepthMap(m_surf, cams[i], false, bImgWithDistortion);
	}

	//compute the center of camera
	T **cs = new T*[camNum];
	for(int i=0; i<camNum; i++)
	{
		 double* cam_cen= cams[i]->getCameraCenter();
		 cs[i] = new T[3];
		 cs[i][0] = cam_cen[0];
		 cs[i][1] = cam_cen[1];
		 cs[i][2] = cam_cen[2];
		 delete [] cam_cen;
	}

	//generate the normals
	if(!m_surf->haveNormalInfo())
	{
		m_surf->generateNormals();
	}

	//get the color info from the image
	int vtDim = m_surf->vtDim;
	int vtNum = m_surf->vtNum;
	T *vtData = m_surf->vtData;
	int new_vtDim = 9;
	T *new_vtData = new T[vtNum*new_vtDim];

	int num_points_without_color = 0;
	for(int i=0; i<vtNum; i++)
	{
		memcpy(&new_vtData[i*new_vtDim], &vtData[i*vtDim], sizeof(T)*vtDim);
		T *p = m_surf->vt_data_block(i);
		T *n = m_surf->vt_normal(i);
		
		vnl_vector_fixed<double, 3> color(0.0);
		vnl_vector_fixed<double, 3> color_best(0.0);
		double w_total = 0.0;
		double w_best = 0.0;
		for(int c=0; c<camNum; c++)
		{
			vnl_vector_fixed<T, 3> v;
			if( bImgWithDistortion )
				cams[c]->World2ImgWithDistortionV3(p, v.data_block());
			else
				cams[c]->worldSpace2ImageCoordV3(p, v.data_block());

			int x = ROUND(v[0]);
			int y = ROUND(v[1]);
			if (x >= 0 && x<cams[c]->m_Width && y >= 0 && y<cams[c]->m_Height &&
				fabs(v[2] - depthMaps[c]->at<double>(y, x))<0.2)
			{
				//check normal
				double theta = cams[c]->angle_btw_viewray_normal(p, n);
				
				double w_cur = MAX(0, std::cos(theta));
				vnl_vector_fixed<double, 3> clr_cur;
				pickAColorPixel(imgs[c], ROUND(v[0]), ROUND(v[1]), clr_cur.data_block());
				if (clr_cur[0] == 0.0 && clr_cur[1] == 0.0 && clr_cur[2] == 0.0)
					continue;

				if (w_best < w_cur)
				{
					w_best = w_cur;
					color_best = clr_cur;
				}

				color += clr_cur*w_cur;
				w_total += w_cur;
			}
		}

		if (w_total > 0)
		{
			if (bBlending)
				color /= w_total;
			else
				color = color_best;
			color /= 255.0;
			color[0] = std::pow(color[0], 1.0 / gamma);
			color[1] = std::pow(color[1], 1.0 / gamma);
			color[2] = std::pow(color[2], 1.0 / gamma);
		}
		else
		{
			if( m_surf->haveColorInfo() )
			{
				T const* clr = m_surf->vt_color(i);
				color[0] = clr[2];
				color[1] = clr[1];
				color[2] = clr[0];
			}
		}


		new_vtData[i*9 + 6] = (T)color[2];
		new_vtData[i*9 + 7] = (T)color[1];
		new_vtData[i*9 + 8] = (T)color[0];
	}

	delete [] vtData;
	for(int i=0; i<camNum; i++)
		depthMaps[i].release();
	depthMaps.clear();
	m_surf->vtDim = new_vtDim;
	m_surf->vtData = new_vtData;
	m_surf->color = true;
	printf("FillColorInfoNCams: points without colors = %d\n", num_points_without_color);
}

template<class T>
void FillColorInfo(CSurface<T> *m_surf, GCameraView *cam1, cv::Mat& img1)
{
	if( m_surf == NULL || cam1==NULL || img1.empty() )
	{
		printf("Error: input is invalid!\n");
		return;
	}
	if( img1.rows != cam1->m_Height || img1.cols != cam1->m_Width)
	{
		printf("Error: the image size does not match with camera's parameter!\n");
		return;
	}
	if(m_surf->haveColorInfo())
	{
		printf("Warning: there is already color info in the triangle mesh!\n");
		return;
	}
	
	int c = 0;
	int vtDim = m_surf->vtDim;
	int vtNum = m_surf->vtNum;
	T *vtData = m_surf->vtData;
	int new_vtDim = vtDim+3;
	T *new_vtData = new T[vtNum*new_vtDim];
	for(int i=0; i<vtNum; i++)
	{
		memcpy(&new_vtData[i*new_vtDim], &vtData[i*vtDim], sizeof(T)*vtDim);
		T* p = &vtData[i*vtDim];
		
		T v[2];
		cam1->World2ImgWithDistortion(p, v);
		//double *v = cam1->worldSpace2ImageCoord(p);

		if(v[0]<0 || v[0]>cam1->m_Width || v[1]<0 || v[1]>cam1->m_Height)
		{
			c++;
		}
		
		double color[3];
		pickAColorPixel(img1, (double)v[0], (double)v[1], color);
		
		color[0] /= 255.0;
		color[1] /= 255.0;
		color[2] /= 255.0;
		
		new_vtData[i*new_vtDim + vtDim] = (T)(color[2]);
		new_vtData[i*new_vtDim + vtDim+1] = (T)(color[1]);
		new_vtData[i*new_vtDim + vtDim+2] = (T)(color[0]);

		//delete [] v;
	}

	delete [] vtData;
	m_surf->vtDim = new_vtDim;
	m_surf->vtData = new_vtData;
	m_surf->color = true;
	printf("%d, %d", c, vtNum);
}

template<class T>
bool readout_surface_texture(CSurface<T> const& surface, vector< vnl_vector_fixed<float, 3> > &texs)
{
	if( surface.vtNum <= 0)
		return false;

	if(!surface.haveColorInfo())
	{
		printf("Error<readout_surface_texture>: no color information!\n");
		return false;
	}

	texs.resize(surface.vtNum);
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		T const* clr = surface.vt_color(vtIdx);
		texs[vtIdx][0] = clr[0];
		texs[vtIdx][1] = clr[1];
		texs[vtIdx][2] = clr[2];
	}

	return true;
}

template<class T>
bool set_surface_texture(CSurface<T>& surface, vector< vnl_vector_fixed<float, 3> > const&texs, bool bSkipTexedVert)
{
	if( surface.vtNum <= 0 || texs.size() <= 0)
		return false;

	if(!surface.haveColorInfo())
	{
		printf("Error<readout_surface_texture>: no color information!\n");
		return false;
	}

	if( texs.size() != surface.vtNum )
	{
		printf("Error<set_surface_texture>: data size does not match(%d/%d)!\n", surface.vtNum, texs.size());
		return false;
	}

	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		T* clr = surface.vt_color(vtIdx);
		if( bSkipTexedVert &&
			(clr[0] !=0 || clr[1] !=0 || clr[2] != 0) )
			continue;

		clr[0] = texs[vtIdx][0];
		clr[1] = texs[vtIdx][1];
		clr[2] = texs[vtIdx][2];
	}
	return true;
}

template<class T>
void freeCSurfaceArray(CSurface<T>** &surfaces, int num)
{
	if( surfaces != NULL && num > 0)
	{
		for(int i=0; i<num; i++)
		{
			if(surfaces[i] != NULL)
				delete surfaces[i];
		}
		delete [] surfaces;
		surfaces = NULL;
	}
}

template<class T>
bool texture_surface_model( CSurface<T> &m_surface, 
							vector< vpgl_perspective_camera<double> > &camera_poses,
							vnl_vector<double> const& distort_vec,
							const char* color_img_list_file_name,
							const char* data_dir )
{
	printf("Read image list file(%s)...", color_img_list_file_name);
	int color_img_num = 0;
	char** color_img_list = NULL;
	readNameFile(color_img_list_file_name, color_img_num, color_img_list);
	printf("<%d> end!\n", color_img_num);

	if( camera_poses.size() != color_img_num )
	{
		printf("Error: the input camera poses does not match with the colors images!\n");
		return false;
	}

	int vtNum = m_surface.vtNum;
	int vtDim = m_surface.vtDim;
	T *vtData = m_surface.vtData;
	T* colors = new T[vtNum*3];
	memset(colors, 0, sizeof(T)*vtNum*3);
	float* weights = new float[vtNum];
	memset(weights, 0, sizeof(float)*vtNum);

	char name[500];
	for(int i=0; i<color_img_num; i++)
	{
		printf("%03d/%03d\r", i, color_img_num);

		char* img_name = color_img_list[i];
		sprintf(name, "%s/%s", data_dir, img_name);
		cv::Mat* img_ori = cvLoadImage(name, CV_LOAD_IMAGE_UNCHANGED);	

		int img_width = img_ori->cols;
		int img_height = img_ori->rows;

		vpgl_perspective_camera<double> cam = camera_poses[i];
		GCameraView* cam_view = vpgl_camera_view_to_GCameraView(cam, distort_vec, img_width, img_height);
		cv::Mat* img = cvCloneImage(img_ori);
		cam_view->UndistortImage(img, img_ori);
		img_ori->release();
		delete cam_view;

		vnl_matrix<double> depthMat_re;
		ComputeDepthMap(m_surface, cam, img_width, img_height, depthMat_re, vnl_vector_fixed<double, 4>(0,0,0,0), remove_clockwise_tri, false);

		//saveDepthMatToPNG("I:/RoomScanDebug/depthMat_re.png", depthMat_re, true);

		vnl_matrix_fixed<double, 3, 3> K = cam.get_calibration().get_matrix();
		double fx = K[0][0];
		double fy = K[1][1];
		double cx = K[0][2];
		double cy = K[1][2];
		vnl_matrix_fixed<double, 3, 3> R;
		vnl_vector_fixed<double, 3> T;
		get_camera_pose(cam, R, T);
		vnl_vector_fixed<double, 3> cam_cen = -R.transpose()*T;
		for(int k=0; k<vtNum; k++)
		{
			vnl_vector_fixed<double, 3> vt;
			vt[0] = vtData[k*vtDim];
			vt[1] = vtData[k*vtDim+1];
			vt[2] = vtData[k*vtDim+2];

			vnl_vector_fixed<double, 3> X_cam = R*vt+T;

			//if the point locates behind the camera, then 
			if(X_cam[2] <= 0)
				continue;

			int u = ROUND(fx*X_cam[0]/X_cam[2] + cx);
			int v = ROUND(fy*X_cam[1]/X_cam[2] + cy);

			// if the point cannot be observed at the current camera pose
			if( u < 5 || u >= img_width-5 ||
				v < 5 || v >= img_height-5 )
				continue;

			//if no depth measurement of the surface at this point
			double ds = depthMat_re[v][u];
			if( ds <= 0 )
				continue;
			
			//if current vertex is invisible
			if( X_cam[2] > ds+0.2 )
				continue;

			float cur_color[3];
			cur_color[0] = img->at<cv::Vec3b>(v, u)[0];
			cur_color[1] = img->at<cv::Vec3b>(v, u)[1];
			cur_color[2] = img->at<cv::Vec3b>(v, u)[2];

			if( cur_color[0] == 0 && cur_color[1] == 0 && cur_color[2] == 0)
				continue;

			float cur_weight = 1.0;
			if(m_surface.haveNormalInfo())
			{
				vnl_vector_fixed<double, 3> vt_to_cam_cen = vt - cam_cen;
				vt_to_cam_cen.normalize();

				vnl_vector_fixed<double, 3> normal(vtData[k*vtDim+3], vtData[k*vtDim+4], vtData[k*vtDim+5]);

				if( normal[0] != 0 || normal[1] != 0|| normal[2] != 0)
				{
					cur_weight = abs(dot_product(normal, vt_to_cam_cen));
					cur_weight = cur_weight*cur_weight;
				}
			}

			if(cur_weight > 0.0 )
			{
				float old_weight = weights[k];
				colors[3*k] = (colors[3*k]*old_weight + cur_color[0]*cur_weight)/(old_weight+cur_weight);
				colors[3*k+1] = (colors[3*k+1]*old_weight + cur_color[1]*cur_weight)/(old_weight+cur_weight);
				colors[3*k+2] = (colors[3*k+2]*old_weight + cur_color[2]*cur_weight)/(old_weight+cur_weight);
				weights[k] += cur_weight;	
			}
		}

		img->release();
	}

	if( m_surface.haveColorInfo() )
	{
		for(int i=0; i<vtNum; i++)
		{
			vtData[i*vtDim+vtDim-3] = colors[3*i]/255.0;
			vtData[i*vtDim+vtDim-2] = colors[3*i+1]/255.0;
			vtData[i*vtDim+vtDim-1] = colors[3*i+2]/255.0;
		}
	}
	else
	{
		vtDim += 3;
		T *vtData_new = new T[vtNum*vtDim];
		for(int i=0; i<vtNum; i++)
		{
			memcpy(&(vtData_new[i*vtDim]), &(vtData[i*(vtDim-3)]), (vtDim-3)*sizeof(T) );
			vtData_new[(i+1)*vtDim-3] = colors[3*i+2]/255.0;
			vtData_new[(i+1)*vtDim-2] = colors[3*i+1]/255.0;
			vtData_new[(i+1)*vtDim-1] = colors[3*i]/255.0;
		}
		delete [] vtData;
		m_surface.vtData = vtData_new;
		m_surface.color = true;
		m_surface.vtDim = vtDim;
	}

	delete [] colors;
	delete [] weights;

	return true;
}


template<class T1, class T2>
bool closest_point_on_surface( CSurface<T1> const&m_surface,
							   vnl_vector_fixed<T2, 3> const &p, 
							   vnl_vector_fixed<T2, 3> &q,
							   int &index,
							   Point2SurfaceDistanceMode dist_mode)
{
	int vtIdx_min = -1;
	double dist_min = 1.0e+100;
	T1 *vtData = m_surface.vtData;
	int vtDim = m_surface.vtDim;
	for(int i=0; i<m_surface.vtNum; i++)
	{
		T1 *pVt = &(vtData[i*vtDim]);
		double dist = (pVt[0]-p[0])*(pVt[0]-p[0]) + 
					  (pVt[1]-p[1])*(pVt[1]-p[1]) + 
					  (pVt[2]-p[2])*(pVt[2]-p[2]);
		if( dist < dist_min )
		{
			dist_min = dist;
			vtIdx_min = i;
		}
	}
	assert( vtIdx_min >=0 && vtIdx_min < m_surface.vtNum );
	index = vtIdx_min;

	T1 *pVt = &(vtData[vtIdx_min*vtDim]);
	q[0] = pVt[0];
	q[1] = pVt[1];
	q[2] = pVt[2];

	if( dist_mode == DistanceMode_Point2Plane && m_surface.haveNormalInfo() )
	{
		T1 *pN = &(pVt[3]);
		T2 t = (pVt[0]-p[0])*pN[0] + 
			   (pVt[1]-p[1])*pN[1] + 
			   (pVt[2]-p[2])*pN[2];

		q[0] = p[0] + t * pN[0];
		q[1] = p[1] + t * pN[1];
		q[2] = p[2] + t * pN[2];
	}
	return true;
}

template<class T1, class T2>
bool closest_point_on_surface_ch_normal(CSurface<T1> const&m_surface,
	vnl_vector_fixed<T2, 3> const &p,
	vnl_vector_fixed<T2, 3> const &np,
	vnl_vector_fixed<T2, 3> &q,
	int &index,
	Point2SurfaceDistanceMode dist_mode,
	double thres_angle)
{
	const double thres_dot_prod = std::cos(thres_angle*M_PI / 180.0);
	assert(m_surface.haveNormalInfo());

	int vtIdx_ch_min = -1;
	double dist_ch_min = 1.0e+100;
	int vtIdx_min = -1;
	double dist_min = 1.0e+100;
	T1 *vtData = m_surface.vtData;
	int vtDim = m_surface.vtDim;
	for (int i = 0; i<m_surface.vtNum; i++)
	{
		T1 const*pNm = m_surface.vt_normal(i);
		double angle = pNm[0] * np[0] + pNm[1] * np[1] + pNm[2] * np[2];
		T1 *pVt = &(vtData[i*vtDim]);
		double dist = (pVt[0] - p[0])*(pVt[0] - p[0]) +
			(pVt[1] - p[1])*(pVt[1] - p[1]) +
			(pVt[2] - p[2])*(pVt[2] - p[2]);
		if (dist < dist_min)
		{
			dist_min = dist;
			vtIdx_min = i;
		}
		
		if (angle > thres_dot_prod && dist < dist_ch_min)
		{
			dist_ch_min = dist;
			vtIdx_ch_min = i;
		}		
	}

	if (vtIdx_min == vtIdx_ch_min)
	{
		index = vtIdx_min;
	}
	else
	{
		if (vtIdx_ch_min == -1)
		{
			index = vtIdx_min;
		}
		else
		{
			T1 const* pN1_ = m_surface.vt_normal(vtIdx_min);
			vnl_vector_fixed<T2, 3> pN1(pN1_[0], pN1_[1], pN1_[2]);
			double angle1 = angle_btw_two_vec(np, pN1);

			T1 const* pN2_ = m_surface.vt_normal(vtIdx_ch_min);
			vnl_vector_fixed<T2, 3> pN2(pN2_[0], pN2_[1], pN2_[2]);
			double angle2 = angle_btw_two_vec(np, pN2);

			double angle_ratio = angle1 / angle2;
			double dist_ratio = dist_ch_min/dist_min;
			if (angle_ratio > dist_ratio)
				index = vtIdx_ch_min;
			else
				index = vtIdx_min;
		}
	}

	if (index != -1)
	{
		T1 *pVt = &(vtData[vtIdx_min*vtDim]);
		q[0] = pVt[0];
		q[1] = pVt[1];
		q[2] = pVt[2];
		if (dist_mode == DistanceMode_Point2Plane && vtIdx_min != -1)
		{
			T1 const*pN = m_surface.vt_normal(vtIdx_min);
			T2 t = (pVt[0] - p[0])*pN[0] +
				(pVt[1] - p[1])*pN[1] +
				(pVt[2] - p[2])*pN[2];

			q[0] = p[0] + t * pN[0];
			q[1] = p[1] + t * pN[1];
			q[2] = p[2] + t * pN[2];
		}
		return true;
	}
	else
		return false;
}

template<class T1, class T2>
bool closest_point_on_surfaces( vector< CSurface<T1>* >  const& m_surfaces,
							   vnl_vector_fixed<T2, 3> const &p, 
							   vnl_vector_fixed<T2, 3> &q,
							   int &surfIdx, //closest surface Idx
							   int &vtIdx, //closest vertex Idx
							   Point2SurfaceDistanceMode dist_mode)
{
	if (m_surfaces.size() == 0)
	{
		surfIdx = -1;
		vtIdx = -1;
		return false;
	}

	surfIdx = -1;
	vtIdx = -1;
	double min_dist = 1.0e+30;
	for(int i=0; i<m_surfaces.size(); i++)
	{
		if (m_surfaces[i] == NULL)
			continue;

		int vtIdx_tmp = -1;
		vnl_vector_fixed<T2, 3> q_tmp;
		closest_point_on_surface(*(m_surfaces[i]), p, q_tmp, vtIdx_tmp, dist_mode);
		double dist_tmp = dist_3d(p, q_tmp);
		if( dist_tmp < min_dist)
		{
			vtIdx = vtIdx_tmp;
			surfIdx = i;
			q = q_tmp;
			min_dist = dist_tmp;
		}
	}
	return true;
}

template<class T1, class T2>
bool closest_point_on_surface( CSurface<T1> const&m_surface,
							   vector< vnl_vector_fixed<T2, 3> > const& points, 
							   vector< vnl_vector_fixed<T2, 3> > &points_closest,
							   vector< int > &indices,
							   Point2SurfaceDistanceMode dist_mode)
{
	points_closest.clear();
	indices.clear();
	for(int i=0; i<points.size(); i++)
	{
		vnl_vector_fixed<T2, 3> q;
		int index;
		closest_point_on_surface(m_surface, points[i], q, index, dist_mode);
		points_closest.push_back(q);
		indices.push_back(index);
	}
	return true;
}

template<class T>
bool calc_camera_visibility( CSurface<T> const& surface, 
							 vector<GCameraView*> const& cams, 
							 vector< vector<int> > &visible_cams)
{
	int cam_num = cams.size();
	int vtNum = surface.vtNum;
	visible_cams.clear();
	visible_cams.resize(vtNum);

	vector<cv::Mat> depthMats;
	for(int i=0; i<cam_num; i++)
	{
		cv::Mat depthMat = ComputeDepthMap(&surface, cams[i], remove_clockwise_tri, false);
		depthMats.push_back(depthMat);
	}

	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		float const*pVt = surface.vt_data_block(vtIdx);
		for(int camIdx=0; camIdx<cam_num; camIdx++)
		{
			float X[3];
			cams[camIdx]->worldSpace2ImageCoordV3(pVt, X);
			
			int u = ROUND(X[0]);
			int v = ROUND(X[1]);
			if( u < 0 || u >= cams[camIdx]->m_Width ||
				v < 0 || v >= cams[camIdx]->m_Height )
				continue;

			double depth = depthMats[camIdx].at<double>(v, u);
			if(depth > 0.0 && X[2] <= depth + 0.2)//fabs(X[2]-depth) < 0.2 )
			{
				visible_cams[vtIdx].push_back(camIdx);
			}
		}
	}

	for(int i=0; i<cam_num; i++)
		depthMats[i].release();

	return true;
}

template<class T>
bool calc_camera_visibility( vector<CSurface<T>*> const& surfaces, 
							 vector<GCameraView*> const& cams, 
							 vector< vector<vector<int>> > &visible_cams)
{
	int cam_num = cams.size();
	vector<cv::Mat> depthMats;
	for(int i=0; i<cam_num; i++)
	{
		cv::Mat depthMat = ComputeDepthMap(surfaces, cams[i], remove_clockwise_tri, false);
		depthMats.push_back(depthMat);
	}

	int surf_num = surfaces.size();
	visible_cams.clear();
	visible_cams.resize(surf_num);
	for(int surfIdx=0; surfIdx<surf_num; surfIdx++)
	{
		CSurface<float> const*surface = surfaces[surfIdx];
		int vtNum = surface->vtNum;
		visible_cams[surfIdx].resize(vtNum);
		for(int vtIdx=0; vtIdx<vtNum; vtIdx++)
		{
			float const*pVt = surface->vt_data_block(vtIdx);
			for(int camIdx=0; camIdx<cam_num; camIdx++)
			{
				float X[3];
				cams[camIdx]->worldSpace2ImageCoordV3(pVt, X);
			
				int u = ROUND(X[0]);
				int v = ROUND(X[1]);
				if( u < 0 || u >= cams[camIdx]->m_Width ||
					v < 0 || v >= cams[camIdx]->m_Height )
					continue;

				double depth = depthMats[camIdx].at<double>(v, u);
				if(depth > 0.0 && fabs(X[2]-depth) < 0.2 )
				{
					visible_cams[surfIdx][vtIdx].push_back(camIdx);
				}
			}
		}
	}

	for (int i = 0; i < cam_num; i++)
		depthMats[i].release();

	return true;
}

//test code
template<class T>
bool color_surface_with_visibility( CSurface<T> &surface, vector<vector<int>> const&visible_cams, 
									int cam_num, 
									vector<int> which_cameras )
{
	assert(surface.vtNum == visible_cams.size() );
	vnl_matrix<double> colorMat(cam_num, 3);
	for(int i=0; i<cam_num; i++)
	{
		colorMat[i][0] = RANDOM;
		colorMat[i][1] = RANDOM;
		colorMat[i][2] = RANDOM;
	}

	vnl_vector<bool> bCamFlags(cam_num);
	bCamFlags.fill(false);
	for(int i=0; i<which_cameras.size(); i++)
	{
		int camIdx = which_cameras[i];
		bCamFlags[camIdx] = true;
	}
	if( which_cameras.size() == 0 )
		bCamFlags.fill(true);

	if( !surface.haveColorInfo() )
	{
		T* vtData_old = surface.vtData;
		int vtDim_old = surface.vtDim;

		int vtDim_new = vtDim_old+3;
		T* vtData_new = new T[vtDim_new*surface.vtNum];
		memset(vtData_new, 0, sizeof(T)*vtDim_new*surface.vtNum);

		for(int i=0; i<surface.vtNum; i++)
		{
			memcpy(&(vtData_new[i*vtDim_new]), &(vtData_old[i*vtDim_old]), vtDim_old*sizeof(T));
		}
		delete [] vtData_old;
		surface.vtData = vtData_new;
		surface.vtDim = vtDim_new;
		surface.color = true;
	}

	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		double color[3];
		color[0] = 0;
		color[1] = 0;
		color[2] = 0;
		int count = 0;
		for(int i=0; i<visible_cams[vtIdx].size(); i++)
		{
			int camIdx = visible_cams[vtIdx][i];
			if( !bCamFlags[camIdx] )
				continue;
			color[0] += colorMat[camIdx][0];
			color[1] += colorMat[camIdx][1];
			color[2] += colorMat[camIdx][2];
			count++;
		}
		if( count > 0 )
		{
			color[0] /= count;
			color[1] /= count;
			color[2] /= count;
		}


		T *pVtClr = surface.vt_color(vtIdx);
		if( count > 0 )
		{
		pVtClr[0] = color[0];
		pVtClr[1] = color[1];
		pVtClr[2] = color[2];
		}
		else
		{
		pVtClr[0] = 0.0;
		pVtClr[1] = 0.0;
		pVtClr[2] = 0.0;
		}
	}
	return true;
}



template<class T>
bool mask_surface(CSurface<T> &surface, GCameraView const* cam, cv::Mat const* mask)
{
	if (cam->m_Width != mask->cols ||
		cam->m_Height != mask->rows)
	{
		printf("Error<mask_surface>: GcameraView size does not agree with mask size.\n");
		return false;
	}
	cv::Mat depthMat_prj = ComputeDepthMap(&surface, cam, false, false);

	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> t;
	get_camera_pose(*cam, R, t);

	vnl_matrix_fixed<double, 3, 3> K;
	get_calibration_matrix(*cam, K);
	double &fx = K[0][0];
	double &fy = K[1][1];
	double &cx = K[0][2];
	double &cy = K[1][2];

	int w = mask->cols;
	int h = mask->rows;

	vector<bool> bFlags(surface.vtNum, false);
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		T const* vt_ = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);
		vt = R*vt + t;

		double u_ = fx*vt[0] / vt[2] + cx;
		double v_ = fy*vt[1] / vt[2] + cy;

		int u = ROUND(u_);
		int v = ROUND(v_);

		if (u < 0 || u >= w ||
			v < 0 || v >= h)
			continue;

		if (mask->at<uchar>(v, u) == 0)
			continue;

		double z = depthMat_prj.at<double>(v, u);;
		if (z <= 0.0)
			continue;

		int ui = u_;
		int vi = v_;
		if (depthMat_prj.at<double>(vi, ui); > 0.0 &&
			depthMat_prj.at<double>(vi, ui + 1) > 0.0 &&
			depthMat_prj.at<double>(vi + 1, ui) > 0.0 &&
			depthMat_prj.at<double>(vi + 1, ui + 1))
			z = pickAMatElement(depthMat_prj, u_, v_);

		if (vt[2] >= z + 0.2)
			continue;

		bFlags[vtIdx] = true;
	}

	depthMat_prj.release();

	vector<int> vt_indices;
	for (int i = 0; i < bFlags.size(); i++)
	{
		if (!bFlags[i])
			vt_indices.push_back(i);
	}
	surface.delete_vertices(vt_indices);
	return true;
}

#endif