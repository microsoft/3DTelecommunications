#ifndef __EDMATCHINGHELPERCUDAIMPL_KEYPTS_CU__
#define __EDMATCHINGHELPERCUDAIMPL_KEYPTS_CU__


__global__
void calc_key_points_in_reference_kernel(float const* dev_vts, float const* dev_vts_t, int vt_dim, 
										 int const* __restrict__ dev_depth_maps_proj, int const* __restrict__ dev_depth_maps_corr_vtIdx,
										 int depth_width_prj, int depth_height_prj,
										 int depth_width, int depth_height,
										 ushort3 const* __restrict__ dev_keypts_2d, float3 * __restrict__ dev_keypts_3d, int const* dev_keypts_num)
{
	const int keypts_num = *dev_keypts_num;
	int ptIdx = threadIdx.x + blockDim.x * blockIdx.x;
	if (ptIdx < keypts_num)
	{
		const ushort3 pt_2d = dev_keypts_2d[ptIdx];
		int vId = pt_2d.x;
		int u = pt_2d.y;
		int v = pt_2d.z;
		int u_proj = MAX(0, MIN(depth_width_prj - 1, ROUND((float)(u) * depth_width_prj / depth_width)));
		int v_proj = MAX(0, MIN(depth_height_prj - 1, ROUND((float)(v) * depth_height_prj / depth_height)));
		int d_proj = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
		int vtIdx = dev_depth_maps_corr_vtIdx[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];

		unsigned short ds = tex2DLayered(tex_depthImgs, u, v, vId);
		depth_extract_fg(ds);
		float3 p_ref = make_float3(0.0f, 0.0f, 0.0f);
		if (ds > 0 && d_proj > 0 &&
			abs(ds-d_proj) < 25)// in mm
		{
			float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
			cuda_vector_fixed<float, 3> v_t(v_t_);
			cuda_vector_fixed<float, 3> n_t(v_t_ + 3);

			//check normal
			float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
			cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
			cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);
			if (dot_product<float, 3>(n_t, nd_t) > NORMAL_CHECK_THRES)
			{
				cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
				float fx = K[0][0];
				float fy = K[1][1];
				float cx = K[0][2];
				float cy = K[1][2];
				cuda_vector_fixed<float, 3> p;
				p[2] = ds / 10.0f;
				p[0] = (u - cx)*p[2] / fx;
				p[1] = (v - cy)*p[2] / fy;
				//to world space
				p -= dev_cam_views[vId].cam_pose.T;
				cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

				//find its neighboring vtIdx
				const int dx[16] = {-1, 1, 0, 0,-2, 2, 0, 0,-3, 3, 0, 0,-4, 4, 0, 0};
				const int dy[16] = { 0, 0,-1, 1, 0, 0,-2, 2, 0, 0,-3, 3, 0, 0,-4, 4};
				int vtIdx_n = -1;
				#pragma unroll
				for (int k = 0; k < 16; k++)
				{
					if (vtIdx_n == -1)
					{
						int u_proj_ = u_proj + dx[k];
						int v_proj_ = v_proj + dy[k];
						if (0 <= u_proj_ && u_proj_ < depth_width_prj &&
							0 <= v_proj_ && v_proj_ < depth_height_prj)
						{
							int vtIdx_ = dev_depth_maps_corr_vtIdx[vId*depth_height_prj*depth_width_prj + v_proj_*depth_width_prj + u_proj_];
							if (vtIdx_ >= 0 && vtIdx_ != vtIdx)
							{
								int d_proj_ = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj_*depth_width_prj + u_proj_];
								if(abs(d_proj_ - d_proj) < 5)
									vtIdx_n = vtIdx_;
							}
						}
					}
				}

				if (vtIdx_n >= 0)
				{
					float const*vn_t_ = dev_vts_t + vtIdx_n * vt_dim;
					cuda_vector_fixed<float, 3> vn_t(vn_t_);
					cuda_vector_fixed<float, 3> dx_t = vn_t - v_t;
					cuda_vector_fixed<float, 3> dy_t = cross_product(n_t, dx_t);
					dy_t.normalize();
					dx_t = cross_product(dy_t, n_t);
					dx_t.normalize();

					cuda_vector_fixed<float, 3> dpt_vt = p_wld - v_t;
					float s_x = dot_product(dx_t, dpt_vt);
					float s_y = dot_product(dy_t, dpt_vt);
					float s_z = dot_product(n_t, dpt_vt);

					float const* v_ = dev_vts + vtIdx * vt_dim;
					cuda_vector_fixed<float, 3> v(v_);
					cuda_vector_fixed<float, 3> n(v_ + 3);
					cuda_vector_fixed<float, 3> vn(dev_vts + vtIdx_n*vt_dim);
					cuda_vector_fixed<float, 3> dx = vn - v;
					cuda_vector_fixed<float, 3> dy = cross_product(n, dx);
					dy.normalize();
					dx = cross_product(dy, n);
					dx.normalize();

					p_ref.x = v[0] + dx[0] * s_x + dy[0] * s_y + n[0] * s_z;
					p_ref.y = v[1] + dx[1] * s_x + dy[1] * s_y + n[1] * s_z;
					p_ref.z = v[2] + dx[2] * s_x + dy[2] * s_y + n[2] * s_z;
				}
				else
				{
					float const* v_ = dev_vts + vtIdx * vt_dim;
					cuda_vector_fixed<float, 3> v(v_);
					p_ref.x = v[0];
					p_ref.y = v[1];
					p_ref.z = v[2];
				}
			}
		}
		dev_keypts_3d[ptIdx] = p_ref;
	}
}

__global__
void calc_key_points_in_curr_kernel(ushort3 const* dev_keypts_2d, float3 *dev_keypts_3d, int const* dev_keypts_num,
							int depth_width, int depth_height)
{
	const int keypts_num = *dev_keypts_num;
	int ptIdx = threadIdx.x + blockDim.x * blockIdx.x;
	if (ptIdx < keypts_num)
	{
		const ushort3 pt_2d = dev_keypts_2d[ptIdx];
		int vId = pt_2d.x;
		int u = pt_2d.y;
		int v = pt_2d.z;
		unsigned short ds = tex2DLayered(tex_depthImgs, u, v, vId);
		depth_extract_fg(ds);
		float3 pt_3d = make_float3(0.0f, 0.0f, 0.0f);
		if (ds > 0)
		{
			const int radius = 1;
			const int thres_dif_d = 5; //5mm
			int d_sum = 0;
			int count = 0;
			for (int i = u - radius; i <= u + radius; i++)
			for (int j = v - radius; j <= v + radius; j++)
			{
				if (0 <= i && i < depth_width &&
					0 <= j && j < depth_height)
				{
					unsigned short d = tex2DLayered(tex_depthImgs, i, j, vId);
					depth_extract_fg(d);
					if (abs(d - ds) < thres_dif_d)
					{
						d_sum += d;
						count++;
					}
				}
			}
			
			float d_f = d_sum;
			d_f /= count; //count >=1 

			cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
			float fx = K[0][0];
			float fy = K[1][1];
			float cx = K[0][2];
			float cy = K[1][2];
			cuda_vector_fixed<float, 3> p;
			p[2] = d_f / 10.0f;
			p[0] = (u - cx)*p[2] / fx;
			p[1] = (v - cy)*p[2] / fy;
			//to world space
			p -= dev_cam_views[vId].cam_pose.T;
			cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);
			pt_3d.x = p_wld[0];
			pt_3d.y = p_wld[1];
			pt_3d.z = p_wld[2];
		}
		dev_keypts_3d[ptIdx] = pt_3d;
	}
}


void EDMatchingHelperCudaImpl::
calc_3d_keypts_in_reference(float3* dev_keypts_3d, ushort3 const* dev_keypts_2d,
							float const* dev_vts, float const* dev_vts_t, int vt_dim,
							int const* dev_depth_maps_proj, int const* dev_depth_maps_vtIdx,
							int depth_width_prj, int depth_height_prj)
{
	int threads_per_blk = 256;
	int blks_per_grid = (keypts_num_gpu_.max_size + threads_per_blk - 1) / threads_per_blk;
	calc_key_points_in_reference_kernel<<<blks_per_grid, threads_per_blk>>>(dev_vts, dev_vts_t, vt_dim, 
										dev_depth_maps_proj, dev_depth_maps_vtIdx, 
										depth_width_prj, depth_height_prj, depth_width_, depth_height_, 
										dev_keypts_2d, dev_keypts_3d, keypts_num_gpu_.dev_ptr);
	m_checkCudaErrors();
}
void EDMatchingHelperCudaImpl::
calc_3d_keypts_in_curr(float3* dev_keypts_3d, ushort3 const* dev_keypts_2d)
{
	int threads_per_blk = 256;
	int blks_per_grid = (keypts_num_gpu_.max_size + threads_per_blk - 1) / threads_per_blk;
	calc_key_points_in_curr_kernel<<<blks_per_grid, threads_per_blk>>>(dev_keypts_2d, dev_keypts_3d, keypts_num_gpu_.dev_ptr,
									depth_width_, depth_height_);
	m_checkCudaErrors();

}


//one key pt pair per thread
template<bool bRobustified>
__global__
void evaluate_cost_keypts_kernel(float *dev_global_cost_keypts,
								 float3 const* dev_keypts_p, float3 const* dev_keypts_q, int const* dev_keypts_num, 
								 int const* dev_ngns_indices_keypts, float const* dev_ngns_weights_keypts,
								 DeformGraphNodeCuda const* dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf,
								 float w_keypts, float tau)
{
	const int keypts_num = *dev_keypts_num;
	if (blockIdx.x*blockDim.x > keypts_num)
		return;

	__shared__ RigidTransformCuda sh_rigid_transf;
	__shared__ float sh_costs[THREADS_PER_BLOCK_COSTKERNEL];

	if (threadIdx.x == 0)
	{
		sh_rigid_transf = *dev_rigid_transf;
	}
	__syncthreads();

	int idx = threadIdx.x + blockIdx.x*blockDim.x;
	float cost = 0.0f;
	if (idx < keypts_num)
	{
		float3 p_ = dev_keypts_p[idx];
		float3 q_ = dev_keypts_q[idx];
		cuda_vector_fixed<float, 3> p(p_.x, p_.y, p_.z);
		cuda_vector_fixed<float, 3> q(q_.x, q_.y, q_.z);
		if ((p[0] != 0.0f || p[1] != 0.0f || p[2] != 0.0f) &&
			(q[0] != 0.0f || q[1] != 0.0f || q[2] != 0.0f))
		{
			int const*ngn_indices = dev_ngns_indices_keypts + NEIGHBOR_EDNODE_NUM*idx;
			float const*ngn_weights = dev_ngns_weights_keypts + NEIGHBOR_EDNODE_NUM*idx;

			cuda_vector_fixed<float, 3> pt(0.0);
			int bWarpValid = 0;
			#pragma unroll
			for (int k = 0; k < NEIGHBOR_EDNODE_NUM; k++)
			{
				int ndIdx_k = ngn_indices[k];
				float w_k = ngn_weights[k];

				if (ndIdx_k >= 0)
				{
					DeformGraphNodeCuda const&nd = dev_ed_nodes[ndIdx_k];
					cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
					cuda_vector_fixed<float, 3> const&g = nd.g;
					cuda_vector_fixed<float, 3> const&t = nd.t;

					pt += w_k*(A*(p - g) + g + t);
					bWarpValid = 1;
				}
			}

			if (bWarpValid)
			{
				pt = sh_rigid_transf.R*pt + sh_rigid_transf.T;

				float rr = dist_square<3>(pt.data_block(), q.data_block());
				if (bRobustified)
					rr = phi(rr, tau);
				cost = rr * w_keypts;
			}
		}
	}
	sh_costs[threadIdx.x] = cost;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			sh_costs[threadIdx.x] += sh_costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_keypts, sh_costs[0]);
	}
}

//64 threads per cuda block
//one key pt pair per cuda block
template<bool bRobustified=false>
__global__
void compute_jtj_jtf_keypts_kernel(float * __restrict__ dev_jtjs, float * __restrict__ dev_jtfs, short2 const* __restrict__ dev_jtj_2d_infos,
								   float3 const* __restrict__ dev_keypts_p, float3 const* __restrict__ dev_keypts_q, int const* __restrict__ dev_keypts_num,
								   int const* __restrict__ dev_ngns_indices_keypts, float const* __restrict__ dev_ngns_weights_keypts,
								   DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int const* dev_ed_nodes_num, 
								   RigidTransformCuda const* __restrict__ dev_rigid_transf,
								   float w_keypts, float tau, bool bUseOffDiagJtJ)
{
	__shared__ float sh_tmps[NEIGHBOR_EDNODE_NUM][3];
	__shared__ int sh_ndIds[NEIGHBOR_EDNODE_NUM];
	__shared__ float sh_ndWeights[NEIGHBOR_EDNODE_NUM];
	__shared__ cuda_vector_fixed<float, 3> sh_f;
	__shared__ float sh_Js[NEIGHBOR_EDNODE_NUM][12][3];
	__shared__ float sh_JtFs[NEIGHBOR_EDNODE_NUM][12];
	__shared__ RigidTransformCuda sh_rigid_transf;
	__shared__ float sh_c, sh_d, sh_e;

	if (threadIdx.x == 0)
		sh_rigid_transf = *dev_rigid_transf;

	const int keypts_num = *dev_keypts_num;
	const int ed_nodes_num = *dev_ed_nodes_num;
	for (int ptIdx = blockIdx.x; ptIdx < keypts_num; ptIdx += gridDim.x)
	{
		float3 p_ = dev_keypts_p[ptIdx];
		cuda_vector_fixed<float, 3> p(p_.x, p_.y, p_.z);
		float3 q = dev_keypts_q[ptIdx];
		if ((p[0] != 0.0f || p[1] != 0.0f || p[2] != 0.0f) &&
			(q.x != 0.0f || q.y != 0.0f || q.z != 0.0f))
		{
			if (threadIdx.x < 3)
				sh_f[threadIdx.x] = 0.0f;
			__syncthreads();

			int const*ngn_indices = dev_ngns_indices_keypts + NEIGHBOR_EDNODE_NUM*ptIdx;
			float const*ngn_weights = dev_ngns_weights_keypts + NEIGHBOR_EDNODE_NUM*ptIdx;
			if (threadIdx.x < NEIGHBOR_EDNODE_NUM)
			{
				int k = threadIdx.x;
				int ndIdx_k = ngn_indices[k];
				float w_k = ngn_weights[k];
				sh_ndIds[k] = ndIdx_k;
				sh_ndWeights[k] = w_k;

				if (ndIdx_k >= 0)
				{
					DeformGraphNodeCuda const&nd = dev_ed_nodes[ndIdx_k];
					cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
					cuda_vector_fixed<float, 3> const&g = nd.g;
					cuda_vector_fixed<float, 3> const&t = nd.t;

					sh_tmps[k][0] = (p[0] - g[0])*w_k;
					sh_tmps[k][1] = (p[1] - g[1])*w_k;
					sh_tmps[k][2] = (p[2] - g[2])*w_k;

					cuda_vector_fixed<float, 3> f_k = w_k*(A*(p - g) + g + t);
					atomicAdd(&sh_f[0], f_k[0]);
					atomicAdd(&sh_f[1], f_k[1]);
					atomicAdd(&sh_f[2], f_k[2]);
				}
				else
				{
					sh_tmps[k][0] = 0;
					sh_tmps[k][1] = 0;
					sh_tmps[k][2] = 0;
				}
			}
			if (threadIdx.x == 0)
			{
				sh_f = sh_rigid_transf.R * sh_f + sh_rigid_transf.T;
				sh_f[0] -= q.x;
				sh_f[1] -= q.y;
				sh_f[2] -= q.z;

				if (bRobustified)
				{
					float rr = sh_f[0] * sh_f[0] + sh_f[1] * sh_f[1] + sh_f[2] * sh_f[2];
					float r = sqrtf(rr);
					robust_kernel_paras(r, tau, sh_c, sh_d, sh_e);
				}
			}
			__syncthreads();

			//compute Js
			for (int id = threadIdx.x; id < NEIGHBOR_EDNODE_NUM * 9; id += blockDim.x)
			{
				int k = id / 9;
				if (sh_ndIds[k] >= 0)
				{
					int c = id - k * 9;
					int i = c / 3;
					int j = c - 3 * i;

					sh_Js[k][i][j] = sh_tmps[k][i] * sh_rigid_transf.R[j][0];
					sh_Js[k][i + 3][j] = sh_tmps[k][i] * sh_rigid_transf.R[j][1];
					sh_Js[k][i + 6][j] = sh_tmps[k][i] * sh_rigid_transf.R[j][2];
					sh_Js[k][i + 9][j] = sh_rigid_transf.R[j][i] * sh_ndWeights[k];
				}
			}
			__syncthreads();

			for (int id = threadIdx.x; id < 12 * NEIGHBOR_EDNODE_NUM; id += blockDim.x)
			{
				int k = id / 12;
				int i = id - 12 * k;
				int ndIdx_k = sh_ndIds[k];
				if (ndIdx_k >= 0)
				{
					float *p_jtf = dev_jtfs + ndIdx_k * 12;

					float jtf_val = (sh_Js[k][i][0] * sh_f[0] + sh_Js[k][i][1] * sh_f[1] + sh_Js[k][i][2] * sh_f[2]);
					sh_JtFs[k][i] = jtf_val;

					if (bRobustified)
						jtf_val *= sh_e;

					atomicAdd(&(p_jtf[i]), jtf_val*w_keypts);
				}
			}
			__syncthreads(); //no need to sync for non-robustified version

			//compute JtJ
			int pi, pj, lc_pi, lc_pj;
			if (bUseOffDiagJtJ)
			{
				for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
				{
					int ndIdx_i = sh_ndIds[i];
					for (int j = 0; j <= i; j++)
					{
						int ndIdx_j = sh_ndIds[j];
						if (ndIdx_i > ndIdx_j) {
							pi = ndIdx_i;
							pj = ndIdx_j;
							lc_pi = i;
							lc_pj = j;
						}
						else {
							pi = ndIdx_j;
							pj = ndIdx_i;
							lc_pi = j;
							lc_pj = i;
						}

						if (pj >= 0)
						{
							int data_offset = dev_jtj_2d_infos[pi*ed_nodes_num + pj].x * 144;
							if (data_offset >= 0)
							{
								float *p_jtj = dev_jtjs + data_offset;
								for (int id = threadIdx.x; id < 12 * 12; id += blockDim.x)
								{
									int r = id / 12;
									int c = id - 12 * r;
									float jtj_val = sh_Js[lc_pi][r][0] * sh_Js[lc_pj][c][0] +
										sh_Js[lc_pi][r][1] * sh_Js[lc_pj][c][1] +
										sh_Js[lc_pi][r][2] * sh_Js[lc_pj][c][2];

									if (bRobustified)
										jtj_val = sh_c*jtj_val + sh_d*sh_JtFs[lc_pi][r] * sh_JtFs[lc_pj][c];

									atomicAdd(&(p_jtj[id]), jtj_val * w_keypts);
								}
							}
						}
					}
				}
			}
			else
			{
				for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
				{
					int ndIdx_i = sh_ndIds[i];
					if (ndIdx_i >= 0)
					{
						pi = ndIdx_i;
						pj = ndIdx_i;
						lc_pi = i;
						lc_pj = i;

						int data_offset = dev_jtj_2d_infos[pi*ed_nodes_num + pj].x * 144;
						if (data_offset >= 0)
						{
							float *p_jtj = dev_jtjs + data_offset;
							for (int id = threadIdx.x; id < 12 * 12; id += blockDim.x)
							{
								int r = id / 12;
								int c = id - 12 * r;
								float jtj_val = sh_Js[lc_pi][r][0] * sh_Js[lc_pj][c][0] +
									sh_Js[lc_pi][r][1] * sh_Js[lc_pj][c][1] +
									sh_Js[lc_pi][r][2] * sh_Js[lc_pj][c][2];

								if (bRobustified)
									jtj_val = sh_c*jtj_val + sh_d*sh_JtFs[lc_pi][r] * sh_JtFs[lc_pj][c];

								atomicAdd(&(p_jtj[id]), jtj_val * w_keypts);
							}
						}
					}
				}				
			}
			__syncthreads();
		}
	}
}

void EDMatchingHelperCudaImpl::
evaluate_cost_keypts(DeformGraphNodeCuda const* dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf, float w_keypts, float *cost)
{
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (keypts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;

	checkCudaErrors(cudaMemsetAsync(dev_global_cost_keypts_, 0, sizeof(int)));
	evaluate_cost_keypts_kernel<false><<<blocks_per_grid, threads_per_block>>>(dev_global_cost_keypts_, dev_keypts_p_, dev_keypts_q_, keypts_num_gpu_.dev_ptr,
										dev_ngns_indices_keypts_, dev_ngns_weights_keypts_,
										dev_ed_nodes, dev_rigid_transf, w_keypts, 1.0);
	m_checkCudaErrors();

	if (cost != NULL)
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_keypts_, sizeof(int), cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCudaImpl::
evaluate_cost_keypts_vRobust(DeformGraphNodeCuda const* dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf, float w_keypts, float tau, float *cost)
{
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (keypts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;

	checkCudaErrors(cudaMemsetAsync(dev_global_cost_keypts_, 0, sizeof(int)));
	evaluate_cost_keypts_kernel<true><<<blocks_per_grid, threads_per_block>>>(dev_global_cost_keypts_, dev_keypts_p_, dev_keypts_q_, keypts_num_gpu_.dev_ptr,
										dev_ngns_indices_keypts_, dev_ngns_weights_keypts_,
										dev_ed_nodes, dev_rigid_transf, w_keypts, tau);
	m_checkCudaErrors();

	if (cost != NULL)
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_keypts_, sizeof(int), cudaMemcpyDeviceToHost));	
}

void EDMatchingHelperCudaImpl::
compute_jtj_jtf_keypts(DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num, RigidTransformCuda const* dev_rigid_transf, 
						float w_keypts, bool bEveluateJtJOffDiag)
{
	int threads_per_block = 64;
	int blocks_per_grid = 2048;
	compute_jtj_jtf_keypts_kernel<false><<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtf_, dev_jtj_2d_infos_, 
		dev_keypts_p_, dev_keypts_q_, keypts_num_gpu_.dev_ptr,
		dev_ngns_indices_keypts_, dev_ngns_weights_keypts_,
		dev_ed_nodes, dev_ed_nodes_num, dev_rigid_transf, w_keypts, 1.0, bEveluateJtJOffDiag);
	m_checkCudaErrors();
}

void EDMatchingHelperCudaImpl::
compute_jtj_jtf_keypts_vRobust(DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num, 
							   RigidTransformCuda const* dev_rigid_transf, float w_keypts, float tau, bool bEveluateJtJOffDiag)
{
	int threads_per_block = 64;
	int blocks_per_grid = 2048;
	compute_jtj_jtf_keypts_kernel<true><<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtf_, dev_jtj_2d_infos_, 
		dev_keypts_p_, dev_keypts_q_, keypts_num_gpu_.dev_ptr,
		dev_ngns_indices_keypts_, dev_ngns_weights_keypts_,
		dev_ed_nodes, dev_ed_nodes_num, dev_rigid_transf, w_keypts, tau, bEveluateJtJOffDiag);
	m_checkCudaErrors();
}

void EDMatchingHelperCudaImpl::
compute_ngns_keypts(float sigma_vt_node_dist)
{
	graph_cuda_->compute_ngns((float const*)dev_keypts_p_, 3, keypts_num_gpu_,
		dev_ngns_indices_keypts_, dev_ngns_weights_keypts_, sigma_vt_node_dist);
}

void EDMatchingHelperCudaImpl::setup_keypts(float sigma_vt_node_dist)
{
	compute_ngns_keypts(sigma_vt_node_dist);
}


void EDMatchingHelperCudaImpl::allocate_mem_for_keypts_matching(int keypts_num_max)
{
	checkCudaErrors(cudaMalloc(&(keypts_num_gpu_.dev_ptr), sizeof(int)));
	checkCudaErrors(cudaMemset(keypts_num_gpu_.dev_ptr, 0, sizeof(int)));
	keypts_num_gpu_.max_size = keypts_num_max;

	checkCudaErrors(cudaMalloc(&(dev_keypts_p_), sizeof(float3)*keypts_num_max));
	checkCudaErrors(cudaMalloc(&(dev_keypts_q_), sizeof(float3)*keypts_num_max));
	checkCudaErrors(cudaMalloc(&(dev_keypts_2d_p_), sizeof(ushort3)*keypts_num_max));
	checkCudaErrors(cudaMalloc(&(dev_keypts_2d_q_), sizeof(ushort3)*keypts_num_max));

	checkCudaErrors(cudaMalloc(&(dev_ngns_indices_keypts_), sizeof(int)*keypts_num_max*NEIGHBOR_EDNODE_NUM));
	checkCudaErrors(cudaMalloc(&(dev_ngns_weights_keypts_), sizeof(float)*keypts_num_max*NEIGHBOR_EDNODE_NUM));
}







#endif