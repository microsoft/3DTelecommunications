#ifndef __EDMATCHINGHELPERCUDAIMPL_CU__
#define __EDMATCHINGHELPERCUDAIMPL_CU__

#define DEPTH_VALUE_FAR 8000

texture<unsigned short, cudaTextureType2DLayered, cudaReadModeElementType> tex_depthImgs;
texture<float4, cudaTextureType2DLayered, cudaReadModeElementType> tex_normalMaps;
surface<void, cudaSurfaceType3D> surf_visHull; //short. occupied 1. empty 0. (invalid -1)
texture<float, cudaTextureType3D, cudaReadModeElementType> tex_visHull;
__constant__ __device__ CameraViewCuda dev_cam_views[MAX_NUM_DEPTH_CAMERAS];
__constant__ __device__ int dev_num_cam_views;
int host_num_cam_views = 10; // 10 is the usual number of PODs in a rig. This value is just used to initialize global variable 'host_num_cam_views' and will be dynamically set by the config file

__device__ float dev_global_cost;

#include "EDMatchingHelperCudaImpl_visualhull.cu"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//             NON-RIGID ALIGNMENT
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


__global__ void init_depth_map_as_far(int* dev_depth_proj, int depth_proj_size)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < depth_proj_size)
		dev_depth_proj[id] = DEPTH_VALUE_FAR;
}

__global__ void init_depth_map_corr_vtIdx_as_invalid(int* dev_depth_mat_corr_vtIdx, int depth_proj_size)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < depth_proj_size)
		dev_depth_mat_corr_vtIdx[id] = -1;
}

template<int step> //down-sample factor on vts
__global__ void project_points_to_depth_kernel( int* __restrict__ dev_depth_proj, int depth_width_out, int depth_height_out, int vId,
														int depth_width_ori, int depth_height_ori, float vxl_res,
														float const* __restrict__ dev_vts, int vts_num, int vt_dim)
{
	int vtIdx = (threadIdx.x + blockIdx.x*blockDim.x)*step;
	if (vtIdx < vts_num)
	{
		float const* p_vt = dev_vts + vt_dim*vtIdx;
		cuda_vector_fixed<float, 3> vt(p_vt);
		cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vt + dev_cam_views[vId].cam_pose.T;
		if (X[2] > 0.1f)
		{
			cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
			float fx = K[0][0];
			float fy = K[1][1];
			float cx = K[0][2];
			float cy = K[1][2];
			
			float sx = float(depth_width_out) / depth_width_ori;
			float sy = float(depth_height_out) / depth_height_ori;
			fx *= sx;
			fy *= sy;
			cx *= sx;
			cy *= sy;

			float r = vxl_res *0.70f / X[2] * fx;

			float u0 = fx*X[0] / X[2] + cx;
			float v0 = fy*X[1] / X[2] + cy;

			for (int u = ceilf(u0 - r); u <= u0 + r; u++)
			for (int v = ceilf(v0 - r); v <= v0 + r; v++)
			{
				// if the point cannot be observed at the current camera pose
				if (u >= 0 && u < depth_width_out &&
					v >= 0 && v < depth_height_out)
				{
					atomicMin(&(dev_depth_proj[v*depth_width_out + u]), ROUND(X[2]*10.0f));
				}
			}
		}
	}
}


__global__ 
void project_points_to_all_depth_maps_kernel(int* dev_depth_maps_proj, int depth_width_out, int depth_height_out,
											int depth_width_ori, int depth_height_ori, float vxl_res,
											float const* __restrict__ dev_vts, int const* dev_vts_num, int vts_num_max, int vt_dim)
{
	const int vts_num = MIN(vts_num_max, *dev_vts_num);

	for (int id = threadIdx.x + blockIdx.x*blockDim.x; id < vts_num*dev_num_cam_views; id += blockDim.x*gridDim.x)
	{
		int vtIdx = id / dev_num_cam_views;
		int vId = id % dev_num_cam_views;
		if (vtIdx < vts_num && vId < dev_num_cam_views)
		{
			float const* p_vt = dev_vts + vt_dim*vtIdx;
			cuda_vector_fixed<float, 3> vt(p_vt);
			if (!(isnan(vt[0])) )
			{
				cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vt + dev_cam_views[vId].cam_pose.T;
				if (X[2] > 0.1f)
				{
					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
					float fx = K(0, 0);
					float fy = K(1, 1);
					float cx = K(0, 2);
					float cy = K(1, 2);

					float sx = (float)(depth_width_out) / depth_width_ori;
					float sy = (float)(depth_height_out) / depth_height_ori;
					fx *= sx;
					fy *= sy;
					cx *= sx;
					cy *= sy;

					float r = MIN(10.0f, vxl_res *0.70f / X[2] * fx);

					float u0 = fx*X[0] / X[2] + cx;
					float v0 = fy*X[1] / X[2] + cy;

					for (int u = ceilf(u0 - r); u <= u0 + r; u++)
					for (int v = ceilf(v0 - r); v <= v0 + r; v++)
					{
						// if the point cannot be observed at the current camera pose
						if (u >= 0 && u < depth_width_out &&
							v >= 0 && v < depth_height_out)
						{
							int val = ROUND(X[2] * 10.0f);
							atomicMin(&(dev_depth_maps_proj[vId*depth_width_out*depth_height_out + v*depth_width_out + u]), val);
						}
					}
				}
			}
		}
	}
}

__global__ 
void record_front_vtIdx_all_depth_maps_kernel(int* dev_depth_mats_corr_vtIdx, int const* __restrict__ dev_depth_maps_proj, int depth_width_out, int depth_height_out,
											   int depth_width_ori, int depth_height_ori, float vxl_res,
											   float const* __restrict__ dev_vts, int const* dev_vts_num, int vts_num_max, int vt_dim)
{
	const int vts_num = MIN(vts_num_max, *dev_vts_num);

	for (int id = threadIdx.x + blockIdx.x*blockDim.x; id < vts_num*dev_num_cam_views; id += blockDim.x*gridDim.x)
	{
		int vtIdx = id / dev_num_cam_views;
		int vId = id % dev_num_cam_views;
		if (vtIdx < vts_num && vId < dev_num_cam_views)
		{
			float const* p_vt = dev_vts + vt_dim*vtIdx;
			if (!(isnan(p_vt[0])) && !(isnan(p_vt[1])) && !(isnan(p_vt[2])))
			{
				cuda_vector_fixed<float, 3> vt(p_vt);
				cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vt + dev_cam_views[vId].cam_pose.T;
				if (X[2] > 0.1f)
				{
					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
					float fx = K(0, 0);
					float fy = K(1, 1);
					float cx = K(0, 2);
					float cy = K(1, 2);

					float sx = (float)(depth_width_out) / depth_width_ori;
					float sy = (float)(depth_height_out) / depth_height_ori;
					fx *= sx;
					fy *= sy;
					cx *= sx;
					cy *= sy;

					float r = MIN(10.0f, vxl_res *0.70f / X[2] * fx);

					float u0 = fx*X[0] / X[2] + cx;
					float v0 = fy*X[1] / X[2] + cy;

					for (int u = ceilf(u0 - r); u <= u0 + r; u++)
					for (int v = ceilf(v0 - r); v <= v0 + r; v++)
					{
						// if the point cannot be observed at the current camera pose
						if (u >= 0 && u < depth_width_out &&
							v >= 0 && v < depth_height_out)
						{
							int val = ROUND(X[2] * 10.0f);
							int val_min = dev_depth_maps_proj[vId*depth_width_out*depth_height_out + v*depth_width_out + u];
							if (val == val_min)
							{
								atomicExch(&(dev_depth_mats_corr_vtIdx[vId*depth_width_out*depth_height_out + v*depth_width_out + u]), vtIdx);
							}
						}
					}
				}
			}
		}
	}
}

void EDMatchingHelperCudaImpl::
project_points_to_depth_map(float const*dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, float vxl_res, bool bSaveFrontVtIdx)
{
	int threads_per_block = 64;
	int blocks_per_grid = (depth_width_proj_*depth_height_proj_*host_num_cam_views + threads_per_block - 1) / threads_per_block;
	init_depth_map_as_far<<<blocks_per_grid, threads_per_block>>>(dev_depth_mats_proj_, (depth_width_proj_*depth_height_proj_*host_num_cam_views));
	m_checkCudaErrors();
	
	// Selecting 64 threads per block for the kernels "arbitrarily." This number can be further optimized for performance
	threads_per_block = 64;
	blocks_per_grid = (vts_num_gpu.sync_read() * host_num_cam_views + threads_per_block - 1) / threads_per_block;
	
	// If we have the number of vertices equals to 0 (i.e., nothing in the scene), we should not invoke the kernel
	if (blocks_per_grid > 0) {
		project_points_to_all_depth_maps_kernel << <blocks_per_grid, threads_per_block >> > (dev_depth_mats_proj_, depth_width_proj_, depth_height_proj_,
			depth_width_, depth_height_, vxl_res,
			dev_vts_t, vts_num_gpu.dev_ptr, vts_num_gpu.max_size, vt_dim);
	}
	m_checkCudaErrors();

	if (bSaveFrontVtIdx)
	{
		threads_per_block = 64;
		blocks_per_grid = (depth_width_proj_*depth_height_proj_*host_num_cam_views + threads_per_block - 1) / threads_per_block;
		init_depth_map_corr_vtIdx_as_invalid<<<blocks_per_grid, threads_per_block>>>(dev_depth_mats_corr_vtIdx_, (depth_width_proj_*depth_height_proj_*host_num_cam_views));
		m_checkCudaErrors();

		threads_per_block = 64;
		blocks_per_grid = (vts_num_gpu.max_size*host_num_cam_views + threads_per_block - 1) / threads_per_block;
		// If we have the number of vertices equals to 0 (i.e., nothing in the scene), we should not invoke the kernel
		if (blocks_per_grid > 0) {
			record_front_vtIdx_all_depth_maps_kernel << <blocks_per_grid, threads_per_block >> > (dev_depth_mats_corr_vtIdx_, dev_depth_mats_proj_, depth_width_proj_, depth_height_proj_,
				depth_width_, depth_height_, vxl_res,
				dev_vts_t, vts_num_gpu.dev_ptr, vts_num_gpu.max_size, vt_dim);
		}
		m_checkCudaErrors();
	}
}

void EDMatchingHelperCudaImpl::
allocate_memory_visibility_check(int depth_width, int depth_height, int down_scale_trg)
{
	this->depth_height_proj_ = depth_height / down_scale_trg;
	this->depth_width_proj_ = depth_width / down_scale_trg;

	checkCudaErrors(cudaMalloc(&dev_depth_mats_proj_, sizeof(int)*depth_width_proj_*depth_height_proj_*host_num_cam_views));
	checkCudaErrors(cudaMalloc(&dev_depth_mats_corr_vtIdx_, sizeof(int)*depth_width_proj_*depth_height_proj_*host_num_cam_views));
	checkCudaErrors(cudaMalloc(&dev_depth_align_residual_, sizeof(float)*depth_width_proj_*depth_height_proj_*host_num_cam_views));
	checkCudaErrors(cudaMalloc(&dev_depth_align_residual_f_, sizeof(float)*depth_width_proj_*depth_height_proj_*host_num_cam_views));
}

//NOTE: support at most 16 cameras. May need to switch to ushort3/uint2 eventually
__global__ void PCDVisibilityCheckKernel(ushort2 * __restrict__ dev_cam_vis, //visible: the source point is close enough to the target points (within distance of mu)
										  float const* __restrict__ dev_vts_t, int const* dev_vts_num, int vt_dim, 
										  int const* __restrict__ dev_depth_maps_proj, int depth_width_prj, int depth_height_prj,
										  int depth_width, int depth_height, float mu)
{
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	const int vts_num = *dev_vts_num;

	if (vtIdx < vts_num)
	{
		float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
		cuda_vector_fixed<float, 3> v_t(v_t_);

		ushort2 cam_vis;// x: count of visible views. y: visibility of each view
		cam_vis.x = 0;
		cam_vis.y = 0;

		if (!(isnan(v_t[0])))
		{
			for (int vId = 0; vId<dev_num_cam_views; vId++)
			{
				cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*v_t + dev_cam_views[vId].cam_pose.T;

				//if the point is behind the camera, then skip it
				if (X[2] > 0.1f)
				{
					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
					float const&fx = K[0][0];
					float const&fy = K[1][1];
					float const&cx = K[0][2];
					float const&cy = K[1][2];

					float u_ = fx*X[0] / X[2] + cx;
					float v_ = fy*X[1] / X[2] + cy;

					int u = ROUND(u_);
					int v = ROUND(v_);

					// if the point cannot be observed at the current camera pose
					if (u >= 0 && u < depth_width &&
						v >= 0 && v < depth_height)
					{

						unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
						depth_remove_top_bit(d);

						if (d > 0)
						{
							//check the visibility
							int u_proj = MAX(0, MIN(depth_width_prj - 1, ROUND(u_ * depth_width_prj / depth_width)));
							int v_proj = MAX(0, MIN(depth_height_prj - 1, ROUND(v_ * depth_height_prj / depth_height)));
							int d_proj = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
							if (X[2] < d_proj/10.0f + 0.5f) //0.5 cm
							{
								cuda_vector_fixed<float, 3> p;
								p[2] = d / 10.0f;
								p[0] = (u - cx)*p[2] / fx;
								p[1] = (v - cy)*p[2] / fy;

								//to world space
								p -= dev_cam_views[vId].cam_pose.T;
								cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

								if (dist_square<3>(p_wld.data_block(), v_t.data_block()) < mu*mu)
								{
									float const*n_t_ = v_t_ + 3;
									cuda_vector_fixed<float, 3> n_t(n_t_);
									float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
									cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
									cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);
									if (dot_product<float, 3>(n_t, nd_t) > NORMAL_CHECK_THRES)
									{
										cam_vis.x++;
										cam_vis.y |= 1 << vId;
									}
								}
							}
						}
					}
				}
			}
		}


		dev_cam_vis[vtIdx] = cam_vis;
	}
}

#define THREADS_PER_BLOCK_COSTKERNEL 1024
//data term and visual hull term
__global__ void EveluateCostKernel( float *dev_global_cost_data, float *dev_global_cost_vis_hull, 
									float const* __restrict__ dev_vts, float const* __restrict__ dev_vts_t, 
									int const* dev_vts_num, int vts_num_max, int vt_dim, 
									int const* __restrict__ dev_depth_maps_proj, int depth_width_prj, int depth_height_prj,
									int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
									int depth_width, int depth_height, float mu, float w_data, float w_vis_hull)
{
	__shared__ float costs[THREADS_PER_BLOCK_COSTKERNEL];
	__shared__ float costs_vis_hull[THREADS_PER_BLOCK_COSTKERNEL];

	const int vts_num = MIN(vts_num_max, *dev_vts_num);

	float cost_sum = 0.0;
	float cost_vis_hull = 0.0;
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	if (vtIdx < vts_num)
	{
		float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
		cuda_vector_fixed<float, 3> v_t(v_t_);
		float const*n_t_ = v_t_ + 3;
		cuda_vector_fixed<float, 3> n_t(n_t_);

		if (!(isnan(v_t[0])))
		{
			int bVisible = 0;
			for (int vId = 0; vId<dev_num_cam_views; vId++)
			{
				cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*v_t + dev_cam_views[vId].cam_pose.T;

				//if the point is behind the camera, then skip it
				if (X[2] > 0.1f)
				{
					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
					float const&fx = K[0][0];
					float const&fy = K[1][1];
					float const&cx = K[0][2];
					float const&cy = K[1][2];

					float u_ = fx*X[0] / X[2] + cx;
					float v_ = fy*X[1] / X[2] + cy;

					int u = ROUND(u_);
					int v = ROUND(v_);

					// if the point cannot be observed at the current camera pose
					if (u >= 0 && u < depth_width &&
						v >= 0 && v < depth_height)
					{
						unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
						depth_remove_top_bit(d);

						if (d > 0)
						{
							//check the visibility
							int u_proj = MAX(0, MIN(depth_width_prj-1, ROUND(u_ * depth_width_prj / depth_width)));
							int v_proj = MAX(0, MIN(depth_height_prj-1, ROUND(v_ * depth_height_prj / depth_height)));
							int d_proj = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
							if (X[2] < d_proj/10.0f + 0.5f) //0.5 cm
							{
								cuda_vector_fixed<float, 3> p;
								p[2] = d / 10.0;
								p[0] = (u - cx)*p[2] / fx;
								p[1] = (v - cy)*p[2] / fy;

								//to world space
								p -= dev_cam_views[vId].cam_pose.T;
								cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);
								if (dist_square<3>(p_wld.data_block(), v_t.data_block()) < mu*mu)
								{
									float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
									cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
									cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);
									if (dot_product<float, 3>(n_t, nd_t) > NORMAL_CHECK_THRES)
									{
										//cost
										float f = dot_product<float, 3>(n_t, p_wld - v_t);
										cost_sum += f*f;
										bVisible = 1;
									}
								}
							}
						}
					}
				}
			}
			

			//visual hull term
			if (!bVisible & w_vis_hull > M_EPS)
			{
				const int3 visual_hull_dim = *dev_visual_hull_dim;
				const float3 visual_hull_offset = *dev_visual_hull_offset;

				int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
				int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
				int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
				if (xId > 0 && xId < visual_hull_dim.x - 1 &&
					yId > 0 && yId < visual_hull_dim.y - 1 &&
					zId > 0 && zId < visual_hull_dim.z - 1)
				{
					float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);
					if (0.0f <= val_vis_hull && val_vis_hull < 1.0f)
						cost_vis_hull = val_vis_hull;
					cost_vis_hull = cost_vis_hull*cost_vis_hull;
				}
			}
		}
	}
	costs[threadIdx.x] = cost_sum * w_data;
	costs_vis_hull[threadIdx.x] = cost_vis_hull * w_vis_hull;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
			costs_vis_hull[threadIdx.x] += costs_vis_hull[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_data, costs[0]);
		atomicAdd(dev_global_cost_vis_hull, costs_vis_hull[0]);
	}
}

//shared memory version for JitJj
__global__ 
void ComputeJtJDataOffDiagonalTermKernel_SM2(float* dev_jtj, HessianBlockInfoCuda const* __restrict__ dev_jtj_blk_info,
											 float const* __restrict__ dev_vts, float const* __restrict__ dev_vts_t, int vt_dim,
											 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, RigidTransformCuda const* __restrict__ dev_rigid_transf,
											 int const* __restrict__ dev_vt_indices, float const* __restrict__ dev_node_weights,
											 ushort2 const* __restrict__ dev_cam_vis,
											 int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
											 int const * __restrict__ para_blk_num_ptr, int const * __restrict__ jtj_blk_num_ptr,
											 float w_data_sqrt, float w_vis_hull_sqrt)
{
	__shared__ float jacobians[1920]; //160*12*2

	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = dev_rigid_transf[0];
	}
	__syncthreads();

	int const para_blk_num = *para_blk_num_ptr;
	int const jtj_blk_num = *jtj_blk_num_ptr;
	for (int blkId = blockIdx.x + para_blk_num; blkId < jtj_blk_num; blkId += gridDim.x)
	{
		HessianBlockInfoCuda const* blk_info = dev_jtj_blk_info + blkId;
		int blk_vtii_first = blk_info->vtii_first;
		int blk_vts_num = blk_info->vts_num;
		int data_offset = blk_info->data_offset;

		int ndIdx;
		if (threadIdx.x % 2 == 0)
			ndIdx = blk_info->pi_idx;
		else
			ndIdx = blk_info->pj_idx;

		cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndIdx].g;

		float jtj_val = 0.0f; //jtj of threadIdx.x-th ellement or (row, col) element as defined later

		// TODO: evaluate if using blockDim.x / 2 instead of 80 breaks code
		for (int wSt = 0; wSt < blk_vts_num; wSt += 80)
		{
			for (int i = 0; i < 12; i++)
				jacobians[12 * threadIdx.x + i] = 0.0f;

			//compute J_pi J_pj
			int idx = blk_vtii_first + wSt + threadIdx.x / 2;
			if (idx < blk_vtii_first + blk_vts_num)
			{
				int vtIdx = dev_vt_indices[idx];
				float w2 = dev_node_weights[idx]; //w^2 or wi*wj
				ushort2 cam_vis = dev_cam_vis[vtIdx];

				if (cam_vis.x > 0)
				{
					float const*vt_ = dev_vts + vtIdx * vt_dim;
					cuda_vector_fixed<float, 3> vt(vt_);
					float const*n_t_ = dev_vts_t + vtIdx * vt_dim + 3;
					cuda_vector_fixed<float, 3> n_t(n_t_);

					float w;
					if (threadIdx.x % 2 == 0)
						w = w2 * w_data_sqrt*cam_vis.x;
					else
						w = w_data_sqrt;

					cuda_vector_fixed<float, 3> df_dvtn = -n_t*sh_rigid_transf.R;

					vt -= g;

					//jacobian
					float *df_dpi = jacobians + 12 * threadIdx.x;
					df_dpi[0] = df_dvtn[0] * (vt[0]) * w;
					df_dpi[1] = df_dvtn[0] * (vt[1]) * w;
					df_dpi[2] = df_dvtn[0] * (vt[2]) * w;
					df_dpi[3] = df_dvtn[1] * (vt[0]) * w;
					df_dpi[4] = df_dvtn[1] * (vt[1]) * w;
					df_dpi[5] = df_dvtn[1] * (vt[2]) * w;
					df_dpi[6] = df_dvtn[2] * (vt[0]) * w;
					df_dpi[7] = df_dvtn[2] * (vt[1]) * w;
					df_dpi[8] = df_dvtn[2] * (vt[2]) * w;
					df_dpi[9] = df_dvtn[0] * w;
					df_dpi[10] = df_dvtn[1] * w;
					df_dpi[11] = df_dvtn[2] * w;
				}
				else if (w_vis_hull_sqrt > M_EPS)
				{
					float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
					cuda_vector_fixed<float, 3> v_t(v_t_);

					const int3 visual_hull_dim = *dev_visual_hull_dim;
					const float3 visual_hull_offset = *dev_visual_hull_offset;

					//visual hull idx
					int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
					int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
					int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
					if (xId > 0 && xId < visual_hull_dim.x - 1 &&
						yId > 0 && yId < visual_hull_dim.y - 1 &&
						zId > 0 && zId < visual_hull_dim.z - 1)
					{
						float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);
						if (0.0f <= val_vis_hull && val_vis_hull < 1.0f)
						{
							cuda_vector_fixed<float, 3> g_vis_hull = visual_hull_gradient(xId, yId, zId, visual_hull_res);

							float const*vt_ = dev_vts + vtIdx * vt_dim;
							cuda_vector_fixed<float, 3> vt(vt_);

							float w;
							if (threadIdx.x % 2 == 0)
								w = w2 * w_vis_hull_sqrt*cam_vis.x;
							else
								w = w_vis_hull_sqrt;

							cuda_vector_fixed<float, 3> df_dvtn = g_vis_hull*sh_rigid_transf.R;

							vt -= g;

							//jocobian
							float *df_dpi = jacobians + 12 * threadIdx.x;
							df_dpi[0] = df_dvtn[0] * (vt[0]) * w;
							df_dpi[1] = df_dvtn[0] * (vt[1]) * w;
							df_dpi[2] = df_dvtn[0] * (vt[2]) * w;
							df_dpi[3] = df_dvtn[1] * (vt[0]) * w;
							df_dpi[4] = df_dvtn[1] * (vt[1]) * w;
							df_dpi[5] = df_dvtn[1] * (vt[2]) * w;
							df_dpi[6] = df_dvtn[2] * (vt[0]) * w;
							df_dpi[7] = df_dvtn[2] * (vt[1]) * w;
							df_dpi[8] = df_dvtn[2] * (vt[2]) * w;
							df_dpi[9] = df_dvtn[0] * w;
							df_dpi[10] = df_dvtn[1] * w;
							df_dpi[11] = df_dvtn[2] * w;
						}
					}
				}
			}
			__syncthreads();

			//compute JtJ
			if (threadIdx.x < 144)
			{
				int row = threadIdx.x / 12;
				int col = threadIdx.x % 12;
				// TODO: evaluate if using blockDim.x instead of 80 breaks code
				for (int i = 0; i < 80; i++)
					jtj_val += jacobians[i * 24 + row] * jacobians[i * 24 + 12 + col];
			}
			__syncthreads();
		}

		//write data back
		if (threadIdx.x < 144)
			dev_jtj[data_offset + threadIdx.x] = jtj_val;
	}
}

//shared memory version for JitJj
__global__ 
__launch_bounds__(160, 6)
void ComputeJtJDataOffDiagonalTermKernel_SM( float* __restrict__ dev_jtj, HessianBlockInfoCuda * __restrict__ dev_jtj_blk_info,
											 float const* __restrict__ dev_vts, float const* __restrict__ dev_vts_t, int vt_dim,
											 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, RigidTransformCuda const* __restrict__ dev_rigid_transf,
											 int const* __restrict__ dev_vt_indices, float const* __restrict__ dev_node_weights,
											 ushort2 const* __restrict__ dev_cam_vis,
											 int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
											 int para_blk_num, int jtj_blk_num,
											 float w_data_sqrt, float w_vis_hull_sqrt)
{
	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = dev_rigid_transf[0];
	}
	__syncthreads();

	if (blockIdx.x >= jtj_blk_num - para_blk_num)
		return;
	if (blockDim.x != 160)
		return;

	HessianBlockInfoCuda *blk_info = dev_jtj_blk_info + blockIdx.x + para_blk_num;

	short ndIdx_i = blk_info->pi_idx;
	short ndIdx_j = blk_info->pj_idx;
	int data_offset = blk_info->data_offset;

	cuda_vector_fixed<float, 3> gi = dev_ed_nodes[ndIdx_i].g;
	cuda_vector_fixed<float, 3> gj = dev_ed_nodes[ndIdx_j].g;

	__shared__ float jacobians[3840]; //160*12*2

	float jtj_val = 0.0; //jtj of threadIdx.x-th ellement or (row, col) element as defined later

	for (int wSt = 0; wSt < blk_info->vts_num; wSt += blockDim.x)
	{
		for (short i = 0; i < 24; i++)
			jacobians[24 * threadIdx.x + i] = 0.0f;

		//compute J_pi J_pj
		int idx = blk_info->vtii_first + wSt + threadIdx.x;
		if (idx < blk_info->vtii_first + blk_info->vts_num)
		{
			int vtIdx = dev_vt_indices[idx];
			float w2 = dev_node_weights[idx]; //w^2 or w1*w2
			ushort2 cam_vis = dev_cam_vis[vtIdx];

			if (cam_vis.x > 0)
			{
				float const*n_t_ = dev_vts_t + vtIdx * vt_dim + 3;
				float const*vt_ = dev_vts + vtIdx * vt_dim;
				cuda_vector_fixed<float, 3> n_t(n_t_);
				cuda_vector_fixed<float, 3> vt(vt_);

				float w_i = w2 * w_data_sqrt*cam_vis.x; //all the weights goes to w_i
				float w_j = w_data_sqrt;
				cuda_vector_fixed<float, 3> df_dvtn = -n_t*sh_rigid_transf.R;

				cuda_vector_fixed<float, 3> tmp1 = vt-gi;

				//jocobian
				float *df_dpi = jacobians + 24 * threadIdx.x;
				df_dpi[0] = df_dvtn[0] * (tmp1[0]) * w_i;
				df_dpi[1] = df_dvtn[0] * (tmp1[1]) * w_i;
				df_dpi[2] = df_dvtn[0] * (tmp1[2]) * w_i;
				df_dpi[3] = df_dvtn[1] * (tmp1[0]) * w_i;
				df_dpi[4] = df_dvtn[1] * (tmp1[1]) * w_i;
				df_dpi[5] = df_dvtn[1] * (tmp1[2]) * w_i;
				df_dpi[6] = df_dvtn[2] * (tmp1[0]) * w_i;
				df_dpi[7] = df_dvtn[2] * (tmp1[1]) * w_i;
				df_dpi[8] = df_dvtn[2] * (tmp1[2]) * w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;

				cuda_vector_fixed<float, 3> tmp2 = vt - gj;

				float *df_dpj = df_dpi + 12;
				df_dpj[0] = df_dvtn[0] * (tmp2[0]) * w_j;
				df_dpj[1] = df_dvtn[0] * (tmp2[1]) * w_j;
				df_dpj[2] = df_dvtn[0] * (tmp2[2]) * w_j;
				df_dpj[3] = df_dvtn[1] * (tmp2[0]) * w_j;
				df_dpj[4] = df_dvtn[1] * (tmp2[1]) * w_j;
				df_dpj[5] = df_dvtn[1] * (tmp2[2]) * w_j;
				df_dpj[6] = df_dvtn[2] * (tmp2[0]) * w_j;
				df_dpj[7] = df_dvtn[2] * (tmp2[1]) * w_j;
				df_dpj[8] = df_dvtn[2] * (tmp2[2]) * w_j;
				df_dpj[9] = df_dvtn[0] * w_j;
				df_dpj[10] = df_dvtn[1] * w_j;
				df_dpj[11] = df_dvtn[2] * w_j;
			}
			else if(w_vis_hull_sqrt > M_EPS)
			{
				//visual hull term
				const int3 visual_hull_dim = *dev_visual_hull_dim;
				const float3 visual_hull_offset = *dev_visual_hull_offset;

				float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
				cuda_vector_fixed<float, 3> v_t(v_t_);

				//visual hull idx
				int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
				int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
				int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
				if (xId > 0 && xId < visual_hull_dim.x - 1 &&
					yId > 0 && yId < visual_hull_dim.y - 1 &&
					zId > 0 && zId < visual_hull_dim.z - 1)
				{
					float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);
					if (0.0f <= val_vis_hull && val_vis_hull < 1.0f)
					{
						//gradient of the visual hull
						cuda_vector_fixed<float, 3> g_vis_hull = visual_hull_gradient(xId, yId, zId, visual_hull_res);

						float const*vt_ = dev_vts + vtIdx * vt_dim;
						cuda_vector_fixed<float, 3> vt(vt_);

						float w_i = w2 * w_vis_hull_sqrt; //all the weights goes to w_i
						float w_j = w_vis_hull_sqrt;
						cuda_vector_fixed<float, 3> df_dvtn = g_vis_hull*sh_rigid_transf.R;

						vt -= gi;

						//jocobian
						float *df_dpi = jacobians + 24 * threadIdx.x;
						df_dpi[0] = df_dvtn[0] * (vt[0]) * w_i;
						df_dpi[1] = df_dvtn[0] * (vt[1]) * w_i;
						df_dpi[2] = df_dvtn[0] * (vt[2]) * w_i;
						df_dpi[3] = df_dvtn[1] * (vt[0]) * w_i;
						df_dpi[4] = df_dvtn[1] * (vt[1]) * w_i;
						df_dpi[5] = df_dvtn[1] * (vt[2]) * w_i;
						df_dpi[6] = df_dvtn[2] * (vt[0]) * w_i;
						df_dpi[7] = df_dvtn[2] * (vt[1]) * w_i;
						df_dpi[8] = df_dvtn[2] * (vt[2]) * w_i;
						df_dpi[9] = df_dvtn[0] * w_i;
						df_dpi[10] = df_dvtn[1] * w_i;
						df_dpi[11] = df_dvtn[2] * w_i;

						vt += gi;
						vt -= gj;
						float *df_dpj = df_dpi + 12;
						df_dpj[0] = df_dvtn[0] * (vt[0]) * w_j;
						df_dpj[1] = df_dvtn[0] * (vt[1]) * w_j;
						df_dpj[2] = df_dvtn[0] * (vt[2]) * w_j;
						df_dpj[3] = df_dvtn[1] * (vt[0]) * w_j;
						df_dpj[4] = df_dvtn[1] * (vt[1]) * w_j;
						df_dpj[5] = df_dvtn[1] * (vt[2]) * w_j;
						df_dpj[6] = df_dvtn[2] * (vt[0]) * w_j;
						df_dpj[7] = df_dvtn[2] * (vt[1]) * w_j;
						df_dpj[8] = df_dvtn[2] * (vt[2]) * w_j;
						df_dpj[9] = df_dvtn[0] * w_j;
						df_dpj[10] = df_dvtn[1] * w_j;
						df_dpj[11] = df_dvtn[2] * w_j;
					}
				}
			}
		}
		__syncthreads();

		//compute JtJ
		if (threadIdx.x < 144)
		{
			short row = threadIdx.x / 12;
			short col = threadIdx.x % 12;

			for (short i = 0; i < blockDim.x; i++)
				jtj_val += jacobians[24 * i + row] * jacobians[24 * i + 12 + col];
		}
		__syncthreads();
	}

	//write data back
	if (threadIdx.x < 144)
		dev_jtj[data_offset + threadIdx.x] = jtj_val;

}

//shared memory version for JtJ
__global__ 
void ComputeJtJDataDiagonalTermKernel_SM(float* dev_jtj, HessianBlockInfoCuda *dev_jtj_blk_info,
										 float const* __restrict__ dev_vts, float const* __restrict__ dev_vts_t, int vt_dim,
										 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, RigidTransformCuda const* __restrict__ dev_rigid_transf,
										 int const* __restrict__ dev_vt_indices, float const* __restrict__ dev_node_weights,
										 ushort2 *dev_cam_vis,
										 int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
										 int const * __restrict__ para_blk_num_ptr, float w_data_sqrt, float w_vis_hull_sqrt)
{
	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = dev_rigid_transf[0];
	}
	__syncthreads();

	if (blockIdx.x >= *para_blk_num_ptr)
		return;
	if (blockDim.x != 160)
		return;

	HessianBlockInfoCuda *blk_info = dev_jtj_blk_info+blockIdx.x;

	int ndIdx_i = blk_info->pi_idx;
	int data_offset = blk_info->data_offset;

	cuda_vector_fixed<float, 3> gi = dev_ed_nodes[ndIdx_i].g;

	__shared__ float jacobians[1920]; //160*12

	float jtj_val = 0.0; //jtj of threadIdx.x-th ellement or (row, col) element as defined later

	for (int wSt = 0; wSt < blk_info->vts_num; wSt+= blockDim.x)
	{
		for (short i = 0; i < 12; i++)
			jacobians[12 * threadIdx.x + i] = 0.0;

		//compute J_pi
		int idx = blk_info->vtii_first + wSt + threadIdx.x;
		if (idx < blk_info->vtii_first + blk_info->vts_num)
		{
			int vtIdx = dev_vt_indices[idx];
			ushort2 cam_vis = dev_cam_vis[vtIdx];
			if (cam_vis.x > 0)
			{
				//data term
				float w = sqrtf(dev_node_weights[idx] * cam_vis.x); //w and vis

				float const*n_t_ = dev_vts_t + vtIdx * vt_dim + 3;
				cuda_vector_fixed<float, 3> n_t(n_t_);

				float const*vt_ = dev_vts + vtIdx * vt_dim;
				cuda_vector_fixed<float, 3> vt(vt_);

				float w_i = w*w_data_sqrt;
				cuda_vector_fixed<float, 3> df_dvtn = -n_t*sh_rigid_transf.R;
				vt -= gi;

				//jocobian
				float *df_dpi = jacobians + 12 * threadIdx.x;
				df_dpi[0] = df_dvtn[0] * vt[0] * w_i;
				df_dpi[1] = df_dvtn[0] * vt[1] * w_i;
				df_dpi[2] = df_dvtn[0] * vt[2] * w_i;
				df_dpi[3] = df_dvtn[1] * vt[0] * w_i;
				df_dpi[4] = df_dvtn[1] * vt[1] * w_i;
				df_dpi[5] = df_dvtn[1] * vt[2] * w_i;
				df_dpi[6] = df_dvtn[2] * vt[0] * w_i;
				df_dpi[7] = df_dvtn[2] * vt[1] * w_i;
				df_dpi[8] = df_dvtn[2] * vt[2] * w_i;
				df_dpi[9] = df_dvtn[0] * w_i;
				df_dpi[10] = df_dvtn[1] * w_i;
				df_dpi[11] = df_dvtn[2] * w_i;
			}
			else if (w_vis_hull_sqrt > M_EPS)
			{
				//visual hull term
				const int3 visual_hull_dim = *dev_visual_hull_dim;
				const float3 visual_hull_offset = *dev_visual_hull_offset;

				float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
				cuda_vector_fixed<float, 3> v_t(v_t_);

				//visual hull idx
				int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
				int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
				int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
				if (xId > 0 && xId < visual_hull_dim.x - 1 &&
					yId > 0 && yId < visual_hull_dim.y - 1 &&
					zId > 0 && zId < visual_hull_dim.z - 1)
				{
					float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);
					if (0.0f <= val_vis_hull && val_vis_hull < 1.0f)
					{
						//gradient of the visual hull
						cuda_vector_fixed<float, 3> g_vis_hull = visual_hull_gradient(xId, yId, zId, visual_hull_res);

						float const*vt_ = dev_vts + vtIdx * vt_dim;
						cuda_vector_fixed<float, 3> vt(vt_);

						float w = sqrtf(dev_node_weights[idx]); //w
						float w_i = w*w_vis_hull_sqrt;
						cuda_vector_fixed<float, 3> df_dvtn = g_vis_hull*sh_rigid_transf.R;
						vt -= gi;

						//jocobian
						float *df_dpi = jacobians + 12 * threadIdx.x;
						df_dpi[0] = df_dvtn[0] * vt[0] * w_i;
						df_dpi[1] = df_dvtn[0] * vt[1] * w_i;
						df_dpi[2] = df_dvtn[0] * vt[2] * w_i;
						df_dpi[3] = df_dvtn[1] * vt[0] * w_i;
						df_dpi[4] = df_dvtn[1] * vt[1] * w_i;
						df_dpi[5] = df_dvtn[1] * vt[2] * w_i;
						df_dpi[6] = df_dvtn[2] * vt[0] * w_i;
						df_dpi[7] = df_dvtn[2] * vt[1] * w_i;
						df_dpi[8] = df_dvtn[2] * vt[2] * w_i;
						df_dpi[9] = df_dvtn[0] * w_i;
						df_dpi[10] = df_dvtn[1] * w_i;
						df_dpi[11] = df_dvtn[2] * w_i;
					}
				}
			}
		}
		__syncthreads();

		//compute JtJ
		if (threadIdx.x < 144)
		{
			short row = threadIdx.x / 12;
			short col = threadIdx.x % 12;
			for (short i = 0; i < blockDim.x; i++)
				jtj_val += jacobians[12 * i + row] * jacobians[12 * i + col];
		}
		__syncthreads();
	}

	//write data back
	if (threadIdx.x < 144)
		dev_jtj[data_offset + threadIdx.x] = jtj_val;

}

//shared memory version for JtF
__global__ 
__launch_bounds__(160, 6)
void ComputeJtFDataTermKernel_SM(float* __restrict__ dev_jtf, HessianBlockInfoCuda const* __restrict__ dev_jtj_blk_info,
								 float const* __restrict__ dev_vts, float const* __restrict__ dev_vts_t, int vt_dim,
								 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, RigidTransformCuda const* __restrict__ dev_rigid_transf,
								 int const* __restrict__ dev_vt_indices, float const* __restrict__ dev_node_weights,
								 ushort2 const* __restrict__ dev_cam_vis,
								 int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
								 int depth_width, int depth_height, int const * __restrict__ para_blk_num_ptr,
								 float mu, float w_data_sqrt, float w_vis_hull_sqrt)
{
	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = dev_rigid_transf[0];
	}
	__syncthreads();

	if (blockIdx.x >= *para_blk_num_ptr)
		return;
	if (blockDim.x != 160)
		return;


	HessianBlockInfoCuda blk_info = dev_jtj_blk_info[blockIdx.x];

	int const& ndIdx_i = blk_info.pi_idx;

	cuda_vector_fixed<float, 3> gi = dev_ed_nodes[ndIdx_i].g;

	__shared__ float jtfs[1920]; //160*12
#pragma unroll
	for (int i = 0; i < 12; i++)
		jtfs[12 * threadIdx.x + i] = 0.0f;

	int iters = (blk_info.vts_num + blockDim.x - 1) / blockDim.x;
	for (int iter = 0; iter < iters; iter++)
	{
		//compute J_pi
		int idx = blk_info.vtii_first + iter*blockDim.x + threadIdx.x;
		if (idx < blk_info.vtii_first + blk_info.vts_num)
		{
			int vtIdx = dev_vt_indices[idx];
			ushort2 cam_vis = dev_cam_vis[vtIdx];

			float const*vt_ = dev_vts + vtIdx * vt_dim;
			cuda_vector_fixed<float, 3> vt(vt_);
			float const* v_t_ = dev_vts_t + vtIdx * vt_dim;
			cuda_vector_fixed<float, 3> v_t(v_t_);

			if (cam_vis.x > 0)
			{
				// data term
				float const* n_t_ = v_t_ + 3;
				cuda_vector_fixed<float, 3> n_t(n_t_);

				float w = sqrtf(dev_node_weights[idx]);
				if (!(isnan(v_t[0]) || isnan(n_t[0]) || isnan(vt[0])))
				{
					#pragma unroll
					for (int vId = 0; vId < dev_num_cam_views; vId++)
					{
						if (cam_vis.y & (1 << vId))
						{
							//note: does not need to check visibility: X[2]>0; [u,v]\in image; d>0
							cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*v_t + dev_cam_views[vId].cam_pose.T;

							//if the point is behind the camera, then skip it
							cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
							float const&fx = K[0][0];
							float const&fy = K[1][1];
							float const&cx = K[0][2];
							float const&cy = K[1][2];

							int u = ROUND(fx*X[0] / X[2] + cx);
							int v = ROUND(fy*X[1] / X[2] + cy);

							// if the point cannot be observed at the current camera pos
							unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
							depth_remove_top_bit(d);

							cuda_vector_fixed<float, 3> p;
							p[2] = d / 10.0f;
							p[0] = (u - cx)*p[2] / fx;
							p[1] = (v - cy)*p[2] / fy;

							//to world space
							p -= dev_cam_views[vId].cam_pose.T;
							cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

							//cost
							float f = dot_product<float, 3>(n_t, p_wld - v_t);
							f *= w_data_sqrt;

							float w_i = w * w_data_sqrt*f;
							cuda_vector_fixed<float, 3> df_dvtn = -n_t*sh_rigid_transf.R;

							//jocobian ---bank conflicts
							float *jtf_i = jtfs + 12 * threadIdx.x;
							jtf_i[0] += df_dvtn[0] * (vt[0] - gi[0]) * w_i;
							jtf_i[1] += df_dvtn[0] * (vt[1] - gi[1]) * w_i;
							jtf_i[2] += df_dvtn[0] * (vt[2] - gi[2]) * w_i;
							jtf_i[3] += df_dvtn[1] * (vt[0] - gi[0]) * w_i;
							jtf_i[4] += df_dvtn[1] * (vt[1] - gi[1]) * w_i;
							jtf_i[5] += df_dvtn[1] * (vt[2] - gi[2]) * w_i;
							jtf_i[6] += df_dvtn[2] * (vt[0] - gi[0]) * w_i;
							jtf_i[7] += df_dvtn[2] * (vt[1] - gi[1]) * w_i;
							jtf_i[8] += df_dvtn[2] * (vt[2] - gi[2]) * w_i;
							jtf_i[9] += df_dvtn[0] * w_i;
							jtf_i[10] += df_dvtn[1] * w_i;
							jtf_i[11] += df_dvtn[2] * w_i;
						}
					}
				}
			}
			else if (w_vis_hull_sqrt > M_EPS)
			{
				//visual hull term
				const int3 visual_hull_dim = *dev_visual_hull_dim;
				const float3 visual_hull_offset = *dev_visual_hull_offset;

				//visual hull idx
				if (!(isnan(v_t[0]) || isnan(vt[0])))
				{
					int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
					int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
					int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
					if (xId > 0 && xId < visual_hull_dim.x - 1 &&
						yId > 0 && yId < visual_hull_dim.y - 1 &&
						zId > 0 && zId < visual_hull_dim.z - 1)
					{
						float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);

						if (0.0f <= val_vis_hull && val_vis_hull < 1.0f)
						{
							float f = val_vis_hull*w_vis_hull_sqrt;

							//gradient of the visual hull
							cuda_vector_fixed<float, 3> g_vis_hull = visual_hull_gradient(xId, yId, zId, visual_hull_res);

							float w = sqrtf(dev_node_weights[idx]); //w
							float w_i = w*w_vis_hull_sqrt * f;
							cuda_vector_fixed<float, 3> df_dvtn = g_vis_hull*sh_rigid_transf.R;
							vt -= gi;

							//jocobian
							float *jtf_i = jtfs + 12 * threadIdx.x;
							jtf_i[0] += df_dvtn[0] * vt[0] * w_i;
							jtf_i[1] += df_dvtn[0] * vt[1] * w_i;
							jtf_i[2] += df_dvtn[0] * vt[2] * w_i;
							jtf_i[3] += df_dvtn[1] * vt[0] * w_i;
							jtf_i[4] += df_dvtn[1] * vt[1] * w_i;
							jtf_i[5] += df_dvtn[1] * vt[2] * w_i;
							jtf_i[6] += df_dvtn[2] * vt[0] * w_i;
							jtf_i[7] += df_dvtn[2] * vt[1] * w_i;
							jtf_i[8] += df_dvtn[2] * vt[2] * w_i;
							jtf_i[9] += df_dvtn[0] * w_i;
							jtf_i[10] += df_dvtn[1] * w_i;
							jtf_i[11] += df_dvtn[2] * w_i;
						}
					}
				}
			}
		}
	}
	__syncthreads();

	//write data back
	if (threadIdx.x < 12)
	{
		float jtf_val = 0.0;
		for (int i = 0; i < blockDim.x; i++)
		{
			jtf_val += jtfs[i * 12 + threadIdx.x];
			// bank conflits
		}

		dev_jtf[ndIdx_i * 12 + threadIdx.x] = jtf_val;
	}
}

//shared memory version for JtF
__global__ void ComputeJtFDataTermKernel_SM_vDataOnly(float* dev_jtf, HessianBlockInfoCuda const* dev_jtj_blk_info,
										float const* dev_vts, float const* dev_vts_t, int vt_dim,
										DeformGraphNodeCuda const*dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf,
										int const*dev_vt_indices, float const*dev_node_weights,
										ushort2 *dev_cam_vis,
										short depth_width, short depth_height, int para_blk_num,										
										float mu, float w_data_sqrt)
{
	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = dev_rigid_transf[0];
	}
	__syncthreads();

	if (blockIdx.x >= para_blk_num)
		return;
	if (blockDim.x != 160)
		return;

	HessianBlockInfoCuda blk_info = dev_jtj_blk_info[blockIdx.x];

	int const& ndIdx_i = blk_info.pi_idx;

	cuda_vector_fixed<float, 3> gi = dev_ed_nodes[ndIdx_i].g;

	__shared__ float jtfs[1920]; //160*12
#pragma unroll
	for (int i = 0; i < 12; i++)
		jtfs[12 * threadIdx.x + i] = 0.0;

	int iters = (blk_info.vts_num + blockDim.x - 1) / blockDim.x;
	for (int iter = 0; iter < iters; iter++)
	{
		//compute J_pi
		int idx = blk_info.vtii_first + iter*blockDim.x + threadIdx.x;
		if (idx < blk_info.vtii_first + blk_info.vts_num)
		{
			int vtIdx = dev_vt_indices[idx];
			float w = sqrtf(dev_node_weights[idx]);

			float const* v_t_ = dev_vts_t + vtIdx * vt_dim;
			float const* n_t_ = v_t_ + 3;
			float const* vt_ = dev_vts + vtIdx * vt_dim;
			cuda_vector_fixed<float, 3> v_t(v_t_);
			cuda_vector_fixed<float, 3> n_t(n_t_);
			cuda_vector_fixed<float, 3> vt(vt_);
			if (!(isnan(v_t[0]) || isnan(n_t[0]) || isnan(vt[0])))
			{
				ushort2 cam_vis = dev_cam_vis[vtIdx];
				//if the point is behind the camera, then skip it
				#pragma unroll
				for (int vId = 0; vId < dev_num_cam_views; vId++)
				{
					if (cam_vis.y & (1 << vId))
					{
						cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*v_t + dev_cam_views[vId].cam_pose.T;

						//if the point is behind the camera, then skip it
						if (X[2] > 0.1f)
						{
							cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
							float const&fx = K[0][0];
							float const&fy = K[1][1];
							float const&cx = K[0][2];
							float const&cy = K[1][2];

							int u = ROUND(fx*X[0] / X[2] + cx);
							int v = ROUND(fy*X[1] / X[2] + cy);

							// if the point cannot be observed at the current camera pose
							if (u >= 0 && u < depth_width &&
								v >= 0 && v < depth_height)
							{
								unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
								depth_remove_top_bit(d);

								if (d > 0)
								{
									cuda_vector_fixed<float, 3> p;
									p[2] = d / 10.0f;
									p[0] = (u - cx)*p[2] / fx;
									p[1] = (v - cy)*p[2] / fy;

									//to world space
									p -= dev_cam_views[vId].cam_pose.T;
									cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

									if (dist_square<3>(p_wld.data_block(), v_t.data_block()) < mu*mu)
									{
										//cost
										float f = dot_product<float, 3>(n_t, p_wld - v_t);
										f *= w_data_sqrt;

										float w_i = w * w_data_sqrt*f;
										cuda_vector_fixed<float, 3> df_dvtn = -n_t*sh_rigid_transf.R;

										//jocobian ---bank conflicts
										float *jtf_i = jtfs + 12 * threadIdx.x;
										jtf_i[0] += df_dvtn[0] * (vt[0] - gi[0]) * w_i;
										jtf_i[1] += df_dvtn[0] * (vt[1] - gi[1]) * w_i;
										jtf_i[2] += df_dvtn[0] * (vt[2] - gi[2]) * w_i;
										jtf_i[3] += df_dvtn[1] * (vt[0] - gi[0]) * w_i;
										jtf_i[4] += df_dvtn[1] * (vt[1] - gi[1]) * w_i;
										jtf_i[5] += df_dvtn[1] * (vt[2] - gi[2]) * w_i;
										jtf_i[6] += df_dvtn[2] * (vt[0] - gi[0]) * w_i;
										jtf_i[7] += df_dvtn[2] * (vt[1] - gi[1]) * w_i;
										jtf_i[8] += df_dvtn[2] * (vt[2] - gi[2]) * w_i;
										jtf_i[9] += df_dvtn[0] * w_i;
										jtf_i[10] += df_dvtn[1] * w_i;
										jtf_i[11] += df_dvtn[2] * w_i;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	__syncthreads();

	//write data back
	if (threadIdx.x < 12)
	{
		float jtf_val = 0.0;
		for (int i = 0; i < blockDim.x; i++)
		{
			jtf_val += jtfs[i * 12 + threadIdx.x];
			// bank conflits
		}

		dev_jtf[ndIdx_i*12 + threadIdx.x] = jtf_val;
	}
}

bool EDMatchingHelperCudaImpl::
compute_jtj_jtf(float* dev_vts,
				float* dev_vts_t,
				cuda::gpu_size_data vts_num_gpu, int vt_dim,
				DeformGraphNodeCuda const*dev_ed_nodes,
				RigidTransformCuda const*dev_rigid_transf,
				double w_data,
				double w_vis_hull,
				double mu,
				bool bEveluateJtJOffDiag
				)
{
	project_points_to_depth_map(dev_vts_t, vts_num_gpu, vt_dim, vxl_res_, false);

	int threads_per_block_cam_vis = 512;
	int blocks_per_grid_cam_vis = (vts_num_gpu.max_size + threads_per_block_cam_vis - 1)/threads_per_block_cam_vis;
	PCDVisibilityCheckKernel<<<blocks_per_grid_cam_vis, threads_per_block_cam_vis>>>(dev_cam_vis_,
																					 dev_vts_t, vts_num_gpu.dev_ptr, vt_dim,
																					 dev_depth_mats_proj_, depth_width_proj_, depth_height_proj_,
																					 depth_width_, depth_height_, mu);
	m_checkCudaErrors();


	int blocks_per_grid = this->jtj_para_blks_num_.max_size;
	int threads_per_block = 160;
	double w_data_sqrt = std::sqrt(w_data);
	double w_vis_hull_sqrt = std::sqrt(w_vis_hull);

	ComputeJtFDataTermKernel_SM<<<blocks_per_grid, threads_per_block >>>(dev_jtf_, dev_jtj_blk_info_buf_,
																	dev_vts, dev_vts_t, vt_dim,
																	dev_ed_nodes, dev_rigid_transf,
																	dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_,
																	dev_cam_vis_,
																	dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
																	depth_width_, depth_height_,
																	jtj_para_blks_num_.dev_ptr, mu, w_data_sqrt, w_vis_hull_sqrt
																	);
	m_checkCudaErrors();
	
	ComputeJtJDataDiagonalTermKernel_SM<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtj_blk_info_buf_,
																				dev_vts, dev_vts_t, vt_dim,
																				dev_ed_nodes, dev_rigid_transf,
																				dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_,
																				dev_cam_vis_,
																				dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
																				jtj_para_blks_num_.dev_ptr, w_data_sqrt, w_vis_hull_sqrt
																				);
	m_checkCudaErrors();


	if (!bEveluateJtJOffDiag)
	{
		w_data_sqrt = 0.0;
		w_vis_hull_sqrt = 0.0;
	}

	auto jtj_blks_num = jtj_blks_num_.sync_read();
	auto jtj_para_blks_num = jtj_para_blks_num_.sync_read();
	blocks_per_grid = jtj_blks_num - jtj_para_blks_num;

	if (blocks_per_grid > 0) {
		ComputeJtJDataOffDiagonalTermKernel_SM2 << <blocks_per_grid, threads_per_block >> > (dev_jtj_, dev_jtj_blk_info_buf_,
			dev_vts, dev_vts_t, vt_dim,
			dev_ed_nodes, dev_rigid_transf,
			dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_,
			dev_cam_vis_,
			dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
			jtj_para_blks_num_.dev_ptr, jtj_blks_num_.dev_ptr, w_data_sqrt, w_vis_hull_sqrt);
	}
	m_checkCudaErrors();

	return true;

}

void EDMatchingHelperCudaImpl::
evaluate_cost(float const* dev_vts, float const*dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, double w_data, double w_vis_hull, double mu, float2 *costs)
{

	project_points_to_depth_map(dev_vts_t, vts_num_gpu, vt_dim, vxl_res_, false);

	m_checkCudaErrors()

	checkCudaErrors(cudaMemsetAsync(dev_global_cost_data_, 0, sizeof(float)));
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_vis_hull_, 0, sizeof(float)));

	int threads_per_block = THREADS_PER_BLOCK_COSTKERNEL;
	int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	EveluateCostKernel<<<blocks_per_grid, threads_per_block >>>(dev_global_cost_data_, dev_global_cost_vis_hull_,
																dev_vts, dev_vts_t, vts_num_gpu.dev_ptr, vts_num_gpu.max_size, vt_dim, 
																dev_depth_mats_proj_, depth_width_proj_, depth_height_proj_,
																dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
															    depth_width_, depth_height_, mu, w_data, w_vis_hull);
	m_checkCudaErrors();


	if (costs != NULL)
	{
		checkCudaErrors(cudaMemcpy(&(costs->x), dev_global_cost_data_, sizeof(float), cudaMemcpyDeviceToHost));
		checkCudaErrors(cudaMemcpy(&(costs->y), dev_global_cost_vis_hull_, sizeof(float), cudaMemcpyDeviceToHost));
	}
}

__global__
void set_gpu_costs_to_zero_kernel(float *dev_cost1,
								  float *dev_cost2,
								  float *dev_cost3,
								  float *dev_cost4,
								  float *dev_cost5,
								  float *dev_cost6)
{
	*dev_cost1 = 0.0f;
	*dev_cost2 = 0.0f;
	*dev_cost3 = 0.0f;
	*dev_cost4 = 0.0f;
	*dev_cost5 = 0.0f;
	*dev_cost6 = 0.0f;
}

void EDMatchingHelperCudaImpl::set_gpu_costs_to_zero()
{
	set_gpu_costs_to_zero_kernel<<<1, 1>>>(dev_global_cost_vis_hull_,
											dev_global_cost_data_,
											dev_global_cost_reg_,
											dev_global_cost_rot_,
											dev_global_cost_temporal_,
											dev_global_cost_keypts_);
	m_checkCudaErrors();
}

__global__
void sum_over_gpu_costs_kernel(float *dev_cost_sum,
							   float const*dev_cost1,
							   float const*dev_cost2,
							   float const*dev_cost3,
							   float const*dev_cost4,
							   float const*dev_cost5,
							   float const*dev_cost6)
{
	*dev_cost_sum = *dev_cost1 + *dev_cost2 + *dev_cost3 +
		*dev_cost4 + *dev_cost5 + *dev_cost6;
}

void EDMatchingHelperCudaImpl::sum_over_gpu_costs(float* dev_cost_sum)
{
	sum_over_gpu_costs_kernel<<<1, 1>>>(dev_cost_sum, 
											dev_global_cost_vis_hull_,
											dev_global_cost_data_,
											dev_global_cost_reg_,
											dev_global_cost_rot_,
											dev_global_cost_temporal_,
											dev_global_cost_keypts_);
	m_checkCudaErrors();
}

void EDMatchingHelperCudaImpl::feed_camera_view(CameraViewCuda* host_cam_views, int view_num)
{
	LOGGER()->info("EDMatchingHelperCudaImpl::feed_camera_view", "View num %d", view_num);
	host_num_cam_views = view_num;
	checkCudaErrors(cudaMemcpyToSymbol(dev_num_cam_views, &view_num, sizeof(int)));
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_cam_views, host_cam_views, sizeof(CameraViewCuda)*view_num));
}

bool EDMatchingHelperCudaImpl::allocate_and_texBind_cuda_arrays_depth(int tex_num, int width, int height)
{
	this->num_depthmaps_ = tex_num;
	this->depth_height_ = height;
	this->depth_width_ = width;

	cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_depth_, &channelDesc_tex, make_cudaExtent(width, height, tex_num), cudaArrayLayered));

	// set texture parameters
	tex_depthImgs.addressMode[0] = cudaAddressModeClamp;
	tex_depthImgs.addressMode[1] = cudaAddressModeClamp;
	tex_depthImgs.filterMode = cudaFilterModePoint;
	tex_depthImgs.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_depthImgs, cu_3dArr_depth_, channelDesc_tex));

	return true;
}

void EDMatchingHelperCudaImpl::bind_cuda_array_to_texture_depth(cudaArray *cu_3dArr_depth)
{
	this->cu_3dArr_depth_ = cu_3dArr_depth;
	// set texture parameters
	tex_depthImgs.addressMode[0] = cudaAddressModeClamp;
	tex_depthImgs.addressMode[1] = cudaAddressModeClamp;
	tex_depthImgs.filterMode = cudaFilterModePoint;
	tex_depthImgs.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
	checkCudaErrors(cudaBindTextureToArray(tex_depthImgs, cu_3dArr_depth, channelDesc_tex));
}


void EDMatchingHelperCudaImpl::bind_cuda_array_to_texture_normal(cudaArray *cu_3dArr_normal)
{
	this->cu_3dArr_normal_ = cu_3dArr_normal;
	// set texture parameters
	tex_normalMaps.addressMode[0] = cudaAddressModeClamp;
	tex_normalMaps.addressMode[1] = cudaAddressModeClamp;
	tex_normalMaps.filterMode = cudaFilterModePoint;
	tex_normalMaps.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
	checkCudaErrors(cudaBindTextureToArray(tex_normalMaps, cu_3dArr_normal, channelDesc_tex));
}

#include "EDMatchingHelperCudaImpl_rigid.cu"
#include "EDMatchingHelperCudaImpl_residual.cu"
#include "EDMatchingHelperCudaImpl_keypts.cu"
#endif