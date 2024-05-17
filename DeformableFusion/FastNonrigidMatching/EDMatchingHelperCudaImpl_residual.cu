#ifndef __EDMATCHINGHELPERCUDAIMPL_RESIDUAL_CU__
#define __EDMATCHINGHELPERCUDAIMPL_RESIDUAL_CU__

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//             Alignment Residual
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//visual hull constrain is implicitly enforced. if a foreground surface point is projected to the background data depth pixel, the residual will
//be maximized.
//one vertex per thread
__global__ 
void evaluate_per_vertex_residual_kernel( float *dev_per_vertex_align_residual,
										  float const* dev_vts_t, int* dev_vts_num, int vt_dim,
										  int const* dev_depth_maps_proj, int depth_width_prj, int depth_height_prj,
										  int depth_width, int depth_height, float residual_cap, float mu, bool bUseSegmentation)
{
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	const int vts_num = *dev_vts_num;
	if (vtIdx < vts_num)
	{
		float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
		cuda_vector_fixed<float, 3> v_t(v_t_);
		float const*n_t_ = v_t_ + 3;
		cuda_vector_fixed<float, 3> n_t(n_t_);

		float cost_sum = 0.0f;
		float cost_min = 1.0e+10f;
		int cost_count = 0;
		if (!(isnan(v_t[0])))
		{
			for (int vId = 0; vId< dev_num_cam_views; vId++)
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
						if (bUseSegmentation)
							depth_extract_fg_set_bg_as_far(d);
						else
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

							float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
							cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
							cuda_vector_fixed<float, 3> nd_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);

							float dist2 = dist_square<3>(v_t.data_block(), p_wld.data_block());
							if (dist2 > mu*mu && X[2] * 10.0f < d) //the current vertex is in front of a depth
							{
								float cost_cur = MIN(residual_cap*residual_cap, dist2);
								cost_min = MIN(cost_min, cost_cur);
								cost_count++;
							}
							else if (dist2 < mu*mu) //
							{
								//check the visibility
								int u_proj = MAX(0, MIN(depth_width_prj - 1, ROUND(u_ * depth_width_prj / depth_width)));
								int v_proj = MAX(0, MIN(depth_height_prj - 1, ROUND(v_ * depth_height_prj / depth_height)));
								int d_proj = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
								if (X[2] < d_proj / 10.0f + 0.3f) //0.5 cm
								{
									if (dot_product<float, 3>(n_t, nd_wld)> 0.5f)
									{
										float cost_cur = MIN(residual_cap*residual_cap, dist2);
										cost_min = MIN(cost_min, cost_cur);
										cost_count++;
									}
								}
							}					
						}
					}
				}
			}			
		}

		float cost = 0.0f;
		if (cost_count > 0)
			cost = sqrtf(cost_min);
		dev_per_vertex_align_residual[vtIdx] = cost/residual_cap;
	}
}

__global__
void evaluate_per_vertex_residual_kernel_vVolume(float* __restrict__ dev_per_vertex_align_residual,
												 float const* __restrict__ dev_vts_t, int* __restrict__ dev_vts_num, int vt_dim,
												 float const* __restrict__ dev_vxl_data, OccupcyCube const* __restrict__ dev_cubes,
												 int3 const* __restrict__ dev_cubes_num, float3 const* __restrict__ dev_cubes_offset,
												 float cube_res, float vxl_res, int cube_size_in_vxl, int vxls_per_cube)
{
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	const int vts_num = *dev_vts_num;

	if (vtIdx < vts_num)
	{
		float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
		cuda_vector_fixed<float, 3> v_t(v_t_);
		float const*n_t_ = v_t_ + 3;
		cuda_vector_fixed<float, 3> n_t(n_t_);

		float cost = 1.0f;
		if (!(isnan(v_t[0])))
		{
			int3 cubes_num = *dev_cubes_num;
			float3 cubes_offset = *dev_cubes_offset;

			float xId0_ = (v_t[0] - cubes_offset.x) / vxl_res;
			float yId0_ = (v_t[1] - cubes_offset.y) / vxl_res;
			float zId0_ = (v_t[2] - cubes_offset.z) / vxl_res;
			int xId0 = xId0_;
			int yId0 = yId0_;
			int zId0 = zId0_;

			const int dx[8] = {0, 1, 0, 1, 0, 1, 0, 1};
			const int dy[8] = {0, 0, 1, 1, 0, 0, 1, 1};
			const int dz[8] = {0, 0, 0, 0, 1, 1, 1, 1};
			float sdfs[8];
			int bAllValid = 1;
			for (int i = 0; i < 8; i++)
			{
				int xId = xId0 + dx[i];
				int yId = yId0 + dy[i];
				int zId = zId0 + dz[i];

				if (0 <= xId && xId < cubes_num.x * cube_size_in_vxl &&
					0 <= yId && yId < cubes_num.y * cube_size_in_vxl &&
					0 <= zId && zId < cubes_num.z * cube_size_in_vxl)
				{
					int xCubeId = xId / cube_size_in_vxl;
					int yCubeId = yId / cube_size_in_vxl;
					int zCubeId = zId / cube_size_in_vxl;

					int cubeId = zCubeId*cubes_num.x*cubes_num.y + yCubeId*cubes_num.x + xCubeId;

					int data_offset = dev_cubes[cubeId].offset * vxls_per_cube;
					if (data_offset > 0)
					{
						xId -= xCubeId*cube_size_in_vxl;
						yId -= yCubeId*cube_size_in_vxl;
						zId -= zCubeId*cube_size_in_vxl;
						int lcIdx = zId*cube_size_in_vxl*cube_size_in_vxl + yId*cube_size_in_vxl + xId;
						sdfs[i] = dev_vxl_data[data_offset + lcIdx];
						if (sdfs[i] == SDF_NULL_VALUE)
							bAllValid = 0;
					}
					else{
						bAllValid = 0;
					}
				}
				else{
					bAllValid = 0;
				}
			}

			float residual = 1.0f;
			if (bAllValid)
			{
				float a = xId0_ - xId0;
				float b = yId0_ - yId0;
				float c = zId0_ - zId0;
				float sdf = ((sdfs[0] * (1.0f - a) + sdfs[1] * a)*(1.0f - b) + (sdfs[2] * (1.0f - a) + sdfs[3] * a)*b)*(1.0f - c) +
					        ((sdfs[4] * (1.0f - a) + sdfs[5] * a)*(1.0f - b) + (sdfs[6] * (1.0f - a) + sdfs[7] * a)*b)* c;

				cuda_vector_fixed<float, 3> der;
				der[0] = (sdfs[1] - sdfs[0])*(1.0f - b)*(1.0f - c) + (sdfs[3] - sdfs[2])*b*(1.0f - c) + (sdfs[5] - sdfs[4])*(1.0f - b)*c + (sdfs[7] - sdfs[6])*b*c;
				der[1] = (sdfs[2] - sdfs[0])*(1.0f - a)*(1.0f - c) + (sdfs[3] - sdfs[1])*a*(1.0f - c) + (sdfs[6] - sdfs[4])*(1.0f - a)*c + (sdfs[7] - sdfs[5])*a*c;
				der[2] = (sdfs[4] - sdfs[0])*(1.0f - a)*(1.0f - b) + (sdfs[5] - sdfs[1])*a*(1.0f - b) + (sdfs[6] - sdfs[2])*(1.0f - a)*b + (sdfs[7] - sdfs[3])*a*b;
				der.normalize();

				residual = fabsf(sdf);
				float dot = dot_product(der, -n_t);
				if (dot > 0.75f)
					residual = fabsf(sdf);
				else
					bAllValid = 0;
			}

			dev_per_vertex_align_residual[vtIdx] = MAX(0.0, MIN(1.0, residual));
		}
	}
}

__global__ void 
evaluate_per_vertex_residual_kernel_vVolume_VisualHull(float* dev_per_vertex_align_residual,
													   int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
													   float const* __restrict__ dev_vts_t, int* dev_vts_num, int vt_dim,
													   float const* __restrict__ dev_vxl_data, OccupcyCube const* __restrict__ dev_cubes,
													   int3 const* __restrict__ dev_cubes_num, float3 const* __restrict__ dev_cubes_offset,
													   float cube_res, float vxl_res, int cube_size_in_vxl, int vxls_per_cube)
{
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	const int vts_num = *dev_vts_num;

	if (vtIdx < vts_num)
	{
		float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
		cuda_vector_fixed<float, 3> v_t(v_t_);
		float const*n_t_ = v_t_ + 3;
		cuda_vector_fixed<float, 3> n_t(n_t_);

		float cost = 1.0f;
		if (!(isnan(v_t[0])))
		{
			int3 cubes_num = *dev_cubes_num;
			float3 cubes_offset = *dev_cubes_offset;

			float xId0_ = (v_t[0] - cubes_offset.x) / vxl_res;
			float yId0_ = (v_t[1] - cubes_offset.y) / vxl_res;
			float zId0_ = (v_t[2] - cubes_offset.z) / vxl_res;
			int xId0 = xId0_;
			int yId0 = yId0_;
			int zId0 = zId0_;

			const int dx[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };
			const int dy[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
			const int dz[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
			float sdfs[8];
			int bAllValid = 1;
			#pragma unroll
			for (int i = 0; i < 8; i++)
			{
				int xId = xId0 + dx[i];
				int yId = yId0 + dy[i];
				int zId = zId0 + dz[i];

				if (0 <= xId && xId < cubes_num.x * cube_size_in_vxl &&
					0 <= yId && yId < cubes_num.y * cube_size_in_vxl &&
					0 <= zId && zId < cubes_num.z * cube_size_in_vxl)
				{
					int xCubeId = xId / cube_size_in_vxl;
					int yCubeId = yId / cube_size_in_vxl;
					int zCubeId = zId / cube_size_in_vxl;

					int cubeId = zCubeId*cubes_num.x*cubes_num.y + yCubeId*cubes_num.x + xCubeId;

					int data_offset = dev_cubes[cubeId].offset * vxls_per_cube;
					if (data_offset > 0)
					{
						xId -= xCubeId*cube_size_in_vxl;
						yId -= yCubeId*cube_size_in_vxl;
						zId -= zCubeId*cube_size_in_vxl;
						int lcIdx = zId*cube_size_in_vxl*cube_size_in_vxl + yId*cube_size_in_vxl + xId;
						sdfs[i] = dev_vxl_data[data_offset + lcIdx];
						if (sdfs[i] == SDF_NULL_VALUE)
							bAllValid = 0;
					}
					else{
						bAllValid = 0;
					}
				}
				else{
					bAllValid = 0;
				}
			}

			float residual_sdf = 1.0f;
			if (bAllValid)
			{
				float a = xId0_ - xId0;
				float b = yId0_ - yId0;
				float c = zId0_ - zId0;
				float sdf = ((sdfs[0] * (1.0f - a) + sdfs[1] * a)*(1.0f - b) + (sdfs[2] * (1.0f - a) + sdfs[3] * a)*b)*(1.0f - c) +
					((sdfs[4] * (1.0f - a) + sdfs[5] * a)*(1.0f - b) + (sdfs[6] * (1.0f - a) + sdfs[7] * a)*b)* c;

				cuda_vector_fixed<float, 3> der;
				der[0] = (sdfs[1] - sdfs[0])*(1.0f - b)*(1.0f - c) + (sdfs[3] - sdfs[2])*b*(1.0f - c) + (sdfs[5] - sdfs[4])*(1.0f - b)*c + (sdfs[7] - sdfs[6])*b*c;
				der[1] = (sdfs[2] - sdfs[0])*(1.0f - a)*(1.0f - c) + (sdfs[3] - sdfs[1])*a*(1.0f - c) + (sdfs[6] - sdfs[4])*(1.0f - a)*c + (sdfs[7] - sdfs[5])*a*c;
				der[2] = (sdfs[4] - sdfs[0])*(1.0f - a)*(1.0f - b) + (sdfs[5] - sdfs[1])*a*(1.0f - b) + (sdfs[6] - sdfs[2])*(1.0f - a)*b + (sdfs[7] - sdfs[3])*a*b;
				der.normalize();

				float dot = dot_product(der, -n_t);

				if (dot > 0.3f)
					residual_sdf = fabsf(sdf);
				else
					bAllValid = 0;
			}

			float residual_hull = 1.0f;
			const int3 visual_hull_dim = *dev_visual_hull_dim;
			const float3 visual_hull_offset = *dev_visual_hull_offset;

			float x = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
			float y = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
			float z = (v_t[2] - visual_hull_offset.z) / visual_hull_res;
			if (0 <= x && x <= visual_hull_dim.x &&
				0 <= y && y <= visual_hull_dim.y &&
				0 <= z && z <= visual_hull_dim.z)
			{
				float hull_val = tex3D(tex_visHull, x, y, z);
				cuda_vector_fixed<float, 3> hull_grad = visual_hull_gradient(x, y, z, 1.0f);
				hull_grad.normalize();
				float dot = dot_product(hull_grad, -n_t);

				if (dot >= 0.3f)
					residual_hull = MAX(0.0f, fabs(hull_val) - 0.2f)*2.0f;
			}

			float residual = MIN(residual_hull, residual_sdf);

			dev_per_vertex_align_residual[vtIdx] = MAX(0.0, MIN(1.0, residual));
		}
	}
}


void EDMatchingHelperCudaImpl::calc_per_vertex_residual(float const* dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, float residual_cap, bool bUseSegmentation)
{
	int threads_per_block = 256;
	int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	evaluate_per_vertex_residual_kernel<<<blocks_per_grid, threads_per_block>>>(dev_per_vertex_align_residual_, dev_vts_t, vts_num_gpu.dev_ptr, vt_dim, 
																				dev_depth_mats_proj_, depth_width_proj_, depth_height_proj_, 
																				depth_width_, depth_height_, residual_cap, 3.0f, bUseSegmentation);
	m_checkCudaErrors();
}

void EDMatchingHelperCudaImpl::
calc_per_vertex_residual(float const* dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, VolumeTwoLevelHierachy const* volume, bool bUseVisualHull)
{
	int threads_per_block = 256;
	int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	if (!bUseVisualHull)
		evaluate_per_vertex_residual_kernel_vVolume<<<blocks_per_grid, threads_per_block>>>(dev_per_vertex_align_residual_, dev_vts_t, vts_num_gpu.dev_ptr, vt_dim, 
																	volume->data, volume->cubes, volume->ptr_cubes_num, volume->ptr_cubes_offset, 
																	volume->cube_res, volume->vxl_res, volume->cube_size_in_voxel, volume->vxls_per_cube);
	else
		evaluate_per_vertex_residual_kernel_vVolume_VisualHull<<<blocks_per_grid, threads_per_block>>>(dev_per_vertex_align_residual_, 
																	dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
																	dev_vts_t, vts_num_gpu.dev_ptr, vt_dim, 
																	volume->data, volume->cubes, volume->ptr_cubes_num, volume->ptr_cubes_offset, 
																	volume->cube_res, volume->vxl_res, volume->cube_size_in_voxel, volume->vxls_per_cube);
	m_checkCudaErrors();
}


//one ed nodes per cuda block
__global__
void aggregate_ed_nodes_residual_kernel_v2(float* __restrict__ dev_ed_nodes_align_residual,
										float const* __restrict__ dev_per_vertex_align_residual,
										HessianBlockInfoCuda const* __restrict__ dev_jtj_blk_info, //diagnoal jtj blocks is in front
										int const* __restrict__ dev_vt_indices_for_jtj,
										float const* __restrict__ dev_vt_weights_for_jtj,
										int const* __restrict__ dev_ed_nodes_num //TODO
										)
{
	__shared__ float sh_vals[MAX_THREADS_PER_BLOCK];
	__shared__ float sh_weights[MAX_THREADS_PER_BLOCK];

	const int ed_nodes_num = *dev_ed_nodes_num;
	int ndIdx = blockIdx.x;
	if (ndIdx < ed_nodes_num)
	{
		int vti_first = dev_jtj_blk_info[ndIdx].vtii_first;
		int vts_num = dev_jtj_blk_info[ndIdx].vts_num;

		if (vts_num > 0)
		{
			float val = 0.0f;
			float weight = 0.0f;
			for (int i = threadIdx.x; i < vts_num; i += blockDim.x)
			{
				int idx = vti_first + i;
				int vtIdx = dev_vt_indices_for_jtj[idx];
				float weight_cur = sqrtf(dev_vt_weights_for_jtj[idx]);
				val += dev_per_vertex_align_residual[vtIdx] * weight_cur;
				weight += weight_cur;
			}
			sh_vals[threadIdx.x] = val;
			sh_weights[threadIdx.x] = weight;
			__syncthreads();

			//reduction on shared memory
			for (int s = blockDim.x / 2; s > 0; s >>= 1){
				if (threadIdx.x < s)
				{
					sh_vals[threadIdx.x] += sh_vals[threadIdx.x + s];
					sh_weights[threadIdx.x] += sh_weights[threadIdx.x + s];
				}
				__syncthreads();
			}

			float val_avg = sh_vals[0] / (sh_weights[0] + M_EPS);
			val = 0.0f;
			weight = 0.0f;
			for (int i = threadIdx.x; i < vts_num; i += blockDim.x)
			{
				int idx = vti_first + i;
				int vtIdx = dev_vt_indices_for_jtj[idx];
				float weight_cur = sqrtf(dev_vt_weights_for_jtj[idx]);
				if (weight_cur > val_avg)
				{
					val += dev_per_vertex_align_residual[vtIdx] * weight_cur;
					weight += weight_cur;
				}
			}
			sh_vals[threadIdx.x] = val;
			sh_weights[threadIdx.x] = weight;
			__syncthreads();

			//reduction on shared memory
			for (int s = blockDim.x / 2; s > 0; s >>= 1){
				if (threadIdx.x < s)
				{
					sh_vals[threadIdx.x] += sh_vals[threadIdx.x + s];
					sh_weights[threadIdx.x] += sh_weights[threadIdx.x + s];
				}
				__syncthreads();
			}

			//write data to the global memory
			if (threadIdx.x == 0)
				dev_ed_nodes_align_residual[ndIdx] = sh_vals[0] / (sh_weights[0] + M_EPS);
		}
		else
		{
			if (threadIdx.x == 0)
				dev_ed_nodes_align_residual[ndIdx] = 0.0f;
		}
	}
}

//one ed nodes per cuda block
__global__
void aggregate_ed_nodes_residual_kernel(float* dev_ed_nodes_align_residual,
										float const*dev_per_vertex_align_residual, 
										HessianBlockInfoCuda const*dev_jtj_blk_info, //diagnoal jtj blocks is in front
										int const* dev_vt_indices_for_jtj,
										float const* dev_vt_weights_for_jtj,
										int const* dev_ed_nodes_num
										)
{
	__shared__ float sh_vals[MAX_THREADS_PER_BLOCK];
	__shared__ float sh_weights[MAX_THREADS_PER_BLOCK];

	const int ed_nodes_num = *dev_ed_nodes_num;

	int ndIdx = blockIdx.x;
	if (ndIdx < ed_nodes_num)
	{
		int vti_first = dev_jtj_blk_info[ndIdx].vtii_first;
		int vts_num = dev_jtj_blk_info[ndIdx].vts_num;

		if (vts_num > 0)
		{
			float val = 0.0f;
			float weight = 0.0f;
			for (int i = threadIdx.x; i < vts_num; i += blockDim.x)
			{
				int idx = vti_first + i;
				int vtIdx = dev_vt_indices_for_jtj[idx];
				float weight_cur = sqrtf(dev_vt_weights_for_jtj[idx]);
				val += dev_per_vertex_align_residual[vtIdx] * weight_cur;
				weight += weight_cur;
			}
			sh_vals[threadIdx.x] = val;
			sh_weights[threadIdx.x] = weight;
			__syncthreads();

			//reduction on shared memory
			for (int s = blockDim.x / 2; s > 0; s >>= 1){
				if (threadIdx.x < s)
				{
					sh_vals[threadIdx.x] += sh_vals[threadIdx.x + s];
					sh_weights[threadIdx.x] += sh_weights[threadIdx.x + s];
				}
				__syncthreads();
			}

			//write data to the global memory
			if (threadIdx.x == 0)
				dev_ed_nodes_align_residual[ndIdx] = sh_vals[0] / (sh_weights[0]+M_EPS);
		}
		else
		{
			if (threadIdx.x == 0)
				dev_ed_nodes_align_residual[ndIdx] = 0.0f;
		}
	}
}


void EDMatchingHelperCudaImpl::
aggregate_ed_nodes_residual(cuda::gpu_size_data ed_nodes_num_gpu)
{
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = ed_nodes_num_gpu.max_size;
	aggregate_ed_nodes_residual_kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_align_residual_, dev_per_vertex_align_residual_, 
																				dev_jtj_blk_info_buf_, dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_, 
																				ed_nodes_num_gpu.dev_ptr);
	m_checkCudaErrors();
}

//one pair of ed nodes per thread
__global__
void ed_nodes_reg_residual_check_kernel(float* dev_ed_nodes_align_residual,
										ushort2 const* __restrict__ dev_reg_node_pairs_list_buf, int const* __restrict__ dev_ed_reg_pairs_num,
										DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int const* dev_ed_nodes_num,
										float thres_reg, float delta_on_residual)
{
	const int node_pairs_num = *dev_ed_reg_pairs_num;

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < node_pairs_num)
	{
		const int ed_nodes_num = *dev_ed_nodes_num;
		ushort2 ndId_pair = dev_reg_node_pairs_list_buf[id];
		int ndIdx_i = ndId_pair.x;
		int ndIdx_j = ndId_pair.y;
		if (0 <= ndIdx_i && ndIdx_i < ed_nodes_num &&
			0 <= ndIdx_j && ndIdx_j < ed_nodes_num)
		{
			cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_j].g - dev_ed_nodes[ndIdx_i].g; //gj-gi

			cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
			cuda_vector_fixed<float, 3> f1 = dev_ed_nodes[ndIdx_i].A*tmp - tmp + ti_m_tj;
			float cost1 = dot_product(f1, f1);

			cuda_vector_fixed<float, 3> f2 = -dev_ed_nodes[ndIdx_j].A*tmp + tmp - ti_m_tj;
			float cost2 = dot_product(f2, f2);

			if ((cost1 + cost2) > 2 * thres_reg*thres_reg)
			{
				atomicAdd(&(dev_ed_nodes_align_residual[ndIdx_i]), delta_on_residual);
				atomicAdd(&(dev_ed_nodes_align_residual[ndIdx_j]), delta_on_residual);
			}
		}
	}
}

void EDMatchingHelperCudaImpl::
update_ed_nodes_residual_with_reg_term(DeformGraphNodeCuda const* dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu, float thres_reg, float delta_on_ed_residual)
{
	int threads_per_block = 64;
	int blocks_per_grid = (ed_reg_pairs_count_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	ed_nodes_reg_residual_check_kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_align_residual_, 
																			dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.dev_ptr,
																			dev_ed_nodes, ed_nodes_num_gpu.dev_ptr, thres_reg, delta_on_ed_residual);
	m_checkCudaErrors();
}


//one vertex per threads
__global__
void interp_per_vertex_residual_from_ed_kernel(float* dev_per_vertex_align_residual_ed_interp, 
										int const* dev_ngns_indices, float const* dev_ngns_weights, int vts_num, //TODO
										float const* dev_ed_nodes_align_residual, int ed_nodes_num//TODO
										)
{
	int vtIdx = threadIdx.x + blockIdx.x*blockDim.x;
	if (vtIdx < vts_num)
	{
		float val = 0.0f;
		int count = 0;
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			int ndId = dev_ngns_indices[vtIdx*NEIGHBOR_EDNODE_NUM + i];
			if (ndId >= 0)
			{
				float w = dev_ngns_weights[vtIdx*NEIGHBOR_EDNODE_NUM + i];
				float ed_residual = dev_ed_nodes_align_residual[ndId];
				val += w*ed_residual;
				count++;
			}
		}

		if (count == 0)
			dev_per_vertex_align_residual_ed_interp[vtIdx] = 1.0f;
		else
			dev_per_vertex_align_residual_ed_interp[vtIdx] = val;		
	}
}

void EDMatchingHelperCudaImpl::interp_per_vertex_residual_from_ed(int const* dev_ngns_indices, float const* dev_ngns_weights, int vts_num, int ed_nodes_num)
{
	int threads_per_block = 256;
	int blocks_per_grid = (vts_num + threads_per_block - 1) / threads_per_block;
	interp_per_vertex_residual_from_ed_kernel<<<blocks_per_grid, threads_per_block>>>(dev_per_vertex_align_residual_ed_interp_, 
																				dev_ngns_indices, dev_ngns_weights, vts_num,
																				dev_ed_nodes_align_residual_, ed_nodes_num);
	m_checkCudaErrors();
}


void EDMatchingHelperCudaImpl::
allocate_memory_ed_vertex_residual(int vts_num_max, int ed_nodes_num_max)
{
	//allocate memory
	checkCudaErrors(cudaMalloc(&dev_per_vertex_align_residual_, sizeof(float)*vts_num_max));
	checkCudaErrors(cudaMalloc(&dev_ed_nodes_align_residual_, sizeof(float)*ed_nodes_num_max));
	checkCudaErrors(cudaMalloc(&dev_per_vertex_align_residual_ed_interp_, sizeof(float)*vts_num_max));
}



__global__
void calc_depth_align_residual_kernel(float * dev_depth_align_residual_,
								 int const* dev_depth_maps_proj, int depth_width_out, int depth_height_out,
								 int depth_width_ori, int depth_height_ori, float residual_cap)
{
	int i = threadIdx.x + blockDim.x*blockIdx.x;
	if (i < dev_num_cam_views * depth_width_out*depth_height_out)
	{
		int vId = i / (depth_width_out*depth_height_out);
		int idx = i - vId*depth_width_out*depth_height_out;
		int u = idx % depth_width_out;
		int v = idx / depth_width_out;

		int u_ori = u*depth_width_ori / depth_width_out;
		int v_ori = v*depth_height_ori / depth_height_out;

		unsigned short d = tex2DLayered(tex_depthImgs, u_ori, v_ori, vId);
		depth_extract_fg(d);
		int d_proj = dev_depth_maps_proj[i];

		float residual_cap_dynamic = residual_cap;

		float residual = -residual_cap_dynamic;
		if (d > 0 && d_proj < DEPTH_VALUE_FAR)
			residual = MIN(residual_cap, fabsf((d_proj - d) / 10.0f));
		else if (d > 0 && d_proj == DEPTH_VALUE_FAR)
			residual = residual_cap;
		else if (d == 0 && d_proj < DEPTH_VALUE_FAR)
			residual = -residual_cap;

		dev_depth_align_residual_[i] = residual / residual_cap_dynamic;
	}
}

__global__
void calc_depth_align_residual_kernel_vVolume(float * dev_depth_align_residual_, 
											  int depth_width_out, int depth_height_out,
											  int depth_width_ori, int depth_height_ori,
											  float const* __restrict__ dev_vxl_data, OccupcyCube const* __restrict__ dev_cubes,
											  int3 const* __restrict__ dev_cubes_num, float3 const* __restrict__ dev_cubes_offset,
											  float cube_res, float vxl_res, int cube_size_in_vxl, int vxls_per_cube)
{
	int i = threadIdx.x + blockDim.x*blockIdx.x;
	if (i < dev_num_cam_views * depth_width_out*depth_height_out)
	{
		int vId = i / (depth_width_out*depth_height_out);
		int idx = i - vId*depth_width_out*depth_height_out;
		int u_ = idx % depth_width_out;
		int v_ = idx / depth_width_out;

		int u = u_*depth_width_ori / depth_width_out;
		int v = v_*depth_height_ori / depth_height_out;

		unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
		depth_extract_fg(d);
		
		float residual = 1.0f;
		if (d > 0)
		{
			cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
			float const&fx = K[0][0];
			float const&fy = K[1][1];
			float const&cx = K[0][2];
			float const&cy = K[1][2];

			//get point from depth in world space
			cuda_vector_fixed<float, 3> p;
			p[2] = d / 10.0f;
			p[0] = (u - cx)*p[2] / fx;
			p[1] = (v - cy)*p[2] / fy;
			//to world space
			p -= dev_cam_views[vId].cam_pose.T;
			cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

			//get normal in world space
			float4 nd_4d = tex2DLayered(tex_normalMaps, u, v, vId);
			cuda_vector_fixed<float, 3> nd_(nd_4d.x, nd_4d.y, nd_4d.z);
			cuda_vector_fixed<float, 3> nd = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd_);

			int3 cubes_num = *dev_cubes_num;
			float3 cubes_offset = *dev_cubes_offset;

			//get vxl id
			float xId0_ = (p_wld[0] - cubes_offset.x) / vxl_res;
			float yId0_ = (p_wld[1] - cubes_offset.y) / vxl_res;
			float zId0_ = (p_wld[2] - cubes_offset.z) / vxl_res;
			int xId0 = xId0_;
			int yId0 = yId0_;
			int zId0 = zId0_;

			const int dx[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };
			const int dy[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
			const int dz[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
			float sdfs[8];
			int bAllValid = 1;
			#pragma unroll
			for (int i = 0; i < 8; i++)
			{
				int xId = xId0 + dx[i];
				int yId = yId0 + dy[i];
				int zId = zId0 + dz[i];

				if (0 <= xId && xId < cubes_num.x * cube_size_in_vxl &&
					0 <= yId && yId < cubes_num.y * cube_size_in_vxl &&
					0 <= zId && zId < cubes_num.z * cube_size_in_vxl)
				{
					int xCubeId = xId / cube_size_in_vxl;
					int yCubeId = yId / cube_size_in_vxl;
					int zCubeId = zId / cube_size_in_vxl;

					int cubeId = zCubeId*cubes_num.x*cubes_num.y + yCubeId*cubes_num.x + xCubeId;

					int data_offset = dev_cubes[cubeId].offset * vxls_per_cube;
					if (data_offset > 0)
					{
						xId -= xCubeId*cube_size_in_vxl;
						yId -= yCubeId*cube_size_in_vxl;
						zId -= zCubeId*cube_size_in_vxl;
						int lcIdx = zId*cube_size_in_vxl*cube_size_in_vxl + yId*cube_size_in_vxl + xId;
						sdfs[i] = dev_vxl_data[data_offset + lcIdx];
						if (sdfs[i] == SDF_NULL_VALUE)
							bAllValid = 0;
					}
					else{
						bAllValid = 0;
					}
				}
				else{
					bAllValid = 0;
				}
			}

			if (bAllValid)
			{
				float a = xId0_ - xId0;
				float b = yId0_ - yId0;
				float c = zId0_ - zId0;
				float sdf = ((sdfs[0] * (1.0f - a) + sdfs[1] * a)*(1.0f - b) + (sdfs[2] * (1.0f - a) + sdfs[3] * a)*b)*(1.0f - c) +
					((sdfs[4] * (1.0f - a) + sdfs[5] * a)*(1.0f - b) + (sdfs[6] * (1.0f - a) + sdfs[7] * a)*b)* c;

				cuda_vector_fixed<float, 3> der;
				der[0] = (sdfs[1] - sdfs[0])*(1.0f - b)*(1.0f - c) + (sdfs[3] - sdfs[2])*b*(1.0f - c) + (sdfs[5] - sdfs[4])*(1.0f - b)*c + (sdfs[7] - sdfs[6])*b*c;
				der[1] = (sdfs[2] - sdfs[0])*(1.0f - a)*(1.0f - c) + (sdfs[3] - sdfs[1])*a*(1.0f - c) + (sdfs[6] - sdfs[4])*(1.0f - a)*c + (sdfs[7] - sdfs[5])*a*c;
				der[2] = (sdfs[4] - sdfs[0])*(1.0f - a)*(1.0f - b) + (sdfs[5] - sdfs[1])*a*(1.0f - b) + (sdfs[6] - sdfs[2])*(1.0f - a)*b + (sdfs[7] - sdfs[3])*a*b;
				der.normalize();

				float dot = dot_product(der, -nd);

				if (dot > 0.50f)
					residual = fabsf(sdf);
			}
		} else{
			residual = -1.0f;
		}

		dev_depth_align_residual_[i] = residual;
	}
}

__global__
void depth_align_residual_filter_kernel(float const* dev_depth_align_residual_in, 
float * dev_depth_align_residual_out, 
int depth_width_out, int depth_height_out, int radius)
{
	extern __shared__ float sh_vals[];

	int blks_per_view = (depth_width_out + blockDim.x - 1) / blockDim.x;
	int vId = blockIdx.x / blks_per_view;

	int x_s = (blockIdx.x - vId*blks_per_view)*blockDim.x;
	int y_s = blockIdx.y*blockDim.y;

	int sh_arr_width = blockDim.x + 2 * radius;
	//load data to shared memory
	for (int i = threadIdx.x; i < blockDim.x + 2*radius; i+=blockDim.x)
	{
		for (int j = threadIdx.y; j < blockDim.y + 2 * radius; j+=blockDim.y)
		{
			int x = i + x_s - radius;
			int y = j + y_s - radius;
			if (x >= 0 && x < depth_width_out &&
				y >= 0 && y < depth_height_out)
			{
				sh_vals[j*sh_arr_width + i] = dev_depth_align_residual_in[vId*depth_width_out*depth_height_out + y*depth_width_out + x];
			}
			else
			{
				sh_vals[j*sh_arr_width + i] = -1.0f;
			}
		}
	}
	__syncthreads();

	int x0 = threadIdx.x+radius;
	int y0 = threadIdx.y+radius;
	float val_sum = 0.0f;
	int count = 0;
	for (int i = -radius; i <= radius; i++)
	{
		for (int j = -radius; j <= radius; j++)
		{
			int x = x0 + i;
			int y = y0 + j;

			float val = sh_vals[y*sh_arr_width + x];
			if (val > 0)
			{
				val_sum += val;
				count++;
			}
		}
	}

	int x = x_s + threadIdx.x;
	int y = y_s + threadIdx.y;
	if (x < depth_width_out && y<depth_height_out)
	{
		if (count > 0 && sh_vals[y0*sh_arr_width + x0] >= 0.0f)
			dev_depth_align_residual_out[vId*depth_width_out*depth_height_out + y*depth_width_out + x] = val_sum / count;
		else
			dev_depth_align_residual_out[vId*depth_width_out*depth_height_out + y*depth_width_out + x] = -1.0f;
	}
}

void EDMatchingHelperCudaImpl::
calc_alignment_residual_on_depth(int filter_radius, float residual_cap)
{
	int threads_per_block = 64;
	int blocks_per_grid = (depth_width_proj_*depth_height_proj_* host_num_cam_views + threads_per_block - 1) / threads_per_block;
	calc_depth_align_residual_kernel<<<blocks_per_grid, threads_per_block>>>(dev_depth_align_residual_, dev_depth_mats_proj_, 
																			 depth_width_proj_, depth_height_proj_,
																			 depth_width_, depth_height_, residual_cap);
	m_checkCudaErrors();

	if (filter_radius > 0)
	{
		int blk_dim = 16;
		dim3 threads(blk_dim, blk_dim);
		dim3 blocks((depth_width_proj_ + blk_dim - 1) / blk_dim * host_num_cam_views, (depth_height_proj_ + blk_dim-1) / blk_dim);
		int sh_bytes = (blk_dim + 2 * filter_radius)*(blk_dim + 2 * filter_radius)*sizeof(float);
		depth_align_residual_filter_kernel<<<blocks, threads, sh_bytes>>>(dev_depth_align_residual_, dev_depth_align_residual_f_, 
																		  depth_width_proj_, depth_height_proj_, filter_radius);
		m_checkCudaErrors();
	}
}

void EDMatchingHelperCudaImpl::
calc_alignment_residual_on_depth(VolumeTwoLevelHierachy const* volume, int filter_radius)
{
	int threads_per_block = 64;
	int blocks_per_grid = (depth_width_proj_*depth_height_proj_* host_num_cam_views + threads_per_block - 1) / threads_per_block;
	calc_depth_align_residual_kernel_vVolume<<<blocks_per_grid, threads_per_block>>>(dev_depth_align_residual_, 
																			depth_width_proj_, depth_height_proj_,
																			depth_width_, depth_height_,
																			volume->data, volume->cubes, volume->ptr_cubes_num, volume->ptr_cubes_offset,
																			volume->cube_res, volume->vxl_res, volume->cube_size_in_voxel, volume->vxls_per_cube
																			);
	m_checkCudaErrors();

	if (filter_radius > 0)
	{
		int blk_dim = 16;
		dim3 threads(blk_dim, blk_dim);
		dim3 blocks((depth_width_proj_ + blk_dim - 1) / blk_dim * host_num_cam_views, (depth_height_proj_ + blk_dim - 1) / blk_dim);
		int sh_bytes = (blk_dim + 2 * filter_radius)*(blk_dim + 2 * filter_radius)*sizeof(float);
		depth_align_residual_filter_kernel<<<blocks, threads, sh_bytes>>>(dev_depth_align_residual_, dev_depth_align_residual_f_,
			depth_width_proj_, depth_height_proj_, filter_radius);
		m_checkCudaErrors();
	}
}


#endif