#ifndef __EDMATCHINGHELPERCUDAIMPLINIT_V2_CU__
#define __EDMATCHINGHELPERCUDAIMPLINIT_V2_CU__
#include "EDMatchingHelperCudaImplInit.cu"

//one vertices per thread
__global__
void associate_pts_to_jtj_kernel_stepCount(int const*dev_ngns_indices, int const* dev_vts_num,
										   int* dev_pt_counts_Diag_block, //must be initialized as 0
										   int* dev_pt_counts_offDiag_block//must be initialized as 0
										   )
{
	const int vts_num = *dev_vts_num;
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	if (vtIdx < vts_num)
	{
		int ngn_idx[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_idx[i] = dev_ngns_indices[NEIGHBOR_EDNODE_NUM*vtIdx + i];

		//diagonal term
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			int ndIdx = ngn_idx[i];
			if (ndIdx >= 0)
				atomicAdd(&(dev_pt_counts_Diag_block[ndIdx]), 1);
		}

		//off-diagonal term
		int id1, id2, id;
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		for (int j = 0; j < i; j++)
		{
			id1 = MAX(ngn_idx[i], ngn_idx[j]);
			id2 = MIN(ngn_idx[i], ngn_idx[j]); //id1/id2 might be invalid (-1)
			id = id1*(id1 - 1) / 2 + id2;
			if (id2 >= 0)
				atomicAdd(&(dev_pt_counts_offDiag_block[id]), 1);
		}		
	}
}

//one diagonal block per thread
__global__
void associate_pts_to_jtj_kernel_stepAggreg_Diag(short2 *dev_jtj_2d_infos, 
												 HessianBlockInfoCuda *dev_block_info_buf,
												 int *dev_global_pts_count,
												 int * dev_pt_counts_Diag_block,
												 int const* dev_ed_nodes_num,
												 int const* dev_ed_nodes_num_all_levels)
{
	const int ed_nodes_num_all_levels = *dev_ed_nodes_num_all_levels;
	const int ed_nodes_num = *dev_ed_nodes_num;

	int i = threadIdx.x + blockDim.x*blockIdx.x;
	__shared__ int sh_pts_count;
	if (threadIdx.x == 0)
		sh_pts_count = 0;
	__syncthreads();

	int lc_pts_offset = 0;
	int pts_count = 0;
	if (i < ed_nodes_num_all_levels)
	{
		if ( i <ed_nodes_num )
			pts_count = dev_pt_counts_Diag_block[i];

		if (pts_count > 0)
			lc_pts_offset = atomicAdd(&sh_pts_count, pts_count);
	}
	__syncthreads();

	if (threadIdx.x == 0)
		sh_pts_count = atomicAdd(dev_global_pts_count, sh_pts_count);
	__syncthreads();

	if (i < ed_nodes_num_all_levels)
	{
		lc_pts_offset += sh_pts_count;

		dev_block_info_buf[i].vtii_first = lc_pts_offset;
		dev_block_info_buf[i].vts_num = pts_count;
		dev_block_info_buf[i].data_offset = i * 144;
		dev_block_info_buf[i].pi_idx = i;
		dev_block_info_buf[i].pj_idx = i;
		dev_block_info_buf[i].pi_dim = 12;
		dev_block_info_buf[i].pj_dim = 12;

		if (i < ed_nodes_num )
			dev_pt_counts_Diag_block[i] = lc_pts_offset;

		short2 info;
		info.x = i;
		info.y = 1;
		dev_jtj_2d_infos[i*ed_nodes_num_all_levels + i] = info;
	}
}

//one off diagonal block per thread
__global__
void associate_pts_to_jtj_kernel_stepAggreg_OffDiag(short2 *dev_jtj_2d_infos,
													HessianBlockInfoCuda *dev_block_info_buf,
													int block_info_buf_size,
													int *dev_global_pts_count,
													int *dev_global_jtj_blks_count, //must be initialized as ed_nodes_num_all_levels
													int *dev_pt_counts_OffDiag_block,
													int const* dev_ed_nodes_num, //nodes at first level
													int const* dev_ed_nodes_num_all_levels,
													int thres_pts_count
													)
{
	int id = threadIdx.x + blockDim.x*blockIdx.x;		
	const int ed_nodes_num = MIN(ED_NODES_NUM_MAX, *dev_ed_nodes_num);

	if (blockIdx.x * blockDim.x > ed_nodes_num*(ed_nodes_num - 1) / 2)
		return;

	__shared__ int sh_pts_count;
	__shared__ int sh_offDiag_blks_count;
	if (threadIdx.x == 0)
	{
		sh_pts_count = 0;
		sh_offDiag_blks_count = 0;
	}
	__syncthreads();

	int lc_pts_offset = 0;
	int lc_blk_offset = 0;
	int pts_count = 0;
	if (id < ed_nodes_num*(ed_nodes_num - 1) / 2)
	{
		pts_count = dev_pt_counts_OffDiag_block[id];

		if (pts_count > thres_pts_count)
		{
			lc_pts_offset = atomicAdd(&sh_pts_count, pts_count);
			lc_blk_offset = atomicAdd(&sh_offDiag_blks_count, 1);
		}
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		sh_pts_count = atomicAdd(dev_global_pts_count, sh_pts_count);
		sh_offDiag_blks_count = atomicAdd(dev_global_jtj_blks_count, sh_offDiag_blks_count);
	}
	__syncthreads();

	if (id < ed_nodes_num*(ed_nodes_num - 1) / 2)
	{
		lc_pts_offset += sh_pts_count;
		int pos = lc_blk_offset + sh_offDiag_blks_count;

		if (pos < block_info_buf_size && pts_count > thres_pts_count)
		{
			int ndIdx_1 = 0;
			int ndIdx_2 = 0;
			decompose_triangle_id(id, ndIdx_1, ndIdx_2);

			dev_block_info_buf[pos].vtii_first = lc_pts_offset;
			dev_block_info_buf[pos].vts_num = pts_count;
			dev_block_info_buf[pos].data_offset = pos * 144;
			dev_block_info_buf[pos].pi_idx = ndIdx_1;
			dev_block_info_buf[pos].pj_idx = ndIdx_2;
			dev_block_info_buf[pos].pi_dim = 12;
			dev_block_info_buf[pos].pj_dim = 12;

			const int ed_nodes_num_all_levels = *dev_ed_nodes_num_all_levels;
			short2 info;
			info.x = pos;
			info.y = 1;
			dev_jtj_2d_infos[ndIdx_1*ed_nodes_num_all_levels + ndIdx_2] = info;

			dev_pt_counts_OffDiag_block[id] = lc_pts_offset;
		}
		else if (0 < pts_count && pts_count <= thres_pts_count)
		{
			dev_pt_counts_OffDiag_block[id] = 0;
		}
	}
}

//one vertex per thread
__global__
void associate_pts_to_jtj_kernel_stepSavingPtIdx(int* dev_vt_indices_for_jtj, float* dev_vt_weights_for_jtj, 
												int const*dev_ngns_indices, float const* dev_ngns_weights, int* dev_vts_num,
												int* dev_pt_counts_Diag_block, 
												int* dev_pt_counts_offDiag_block)

{
	const int vts_num = *dev_vts_num;
	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	if (vtIdx < vts_num)
	{
		int ngn_idx[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_idx[i] = dev_ngns_indices[NEIGHBOR_EDNODE_NUM*vtIdx + i];

		float ngn_weights[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_weights[i] = dev_ngns_weights[NEIGHBOR_EDNODE_NUM * vtIdx + i];

		//diagonal term
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			int ndIdx = ngn_idx[i];
			if (ndIdx >= 0)
			{
				int pos = atomicAdd(&(dev_pt_counts_Diag_block[ndIdx]), 1);
				dev_vt_indices_for_jtj[pos] = vtIdx;
				dev_vt_weights_for_jtj[pos] = ngn_weights[i] * ngn_weights[i];
			}
		}

		//off-diagonal term
		int id1, id2, id;
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		for (int j = 0; j < i; j++)
		{
			id1 = MAX(ngn_idx[i], ngn_idx[j]);
			id2 = MIN(ngn_idx[i], ngn_idx[j]); //id1/id2 might be invalid (-1)
			id = id1*(id1 - 1) / 2 + id2;
			if (id2 >= 0 && dev_pt_counts_offDiag_block[id] > 0)
			{
				int pos = atomicAdd(&(dev_pt_counts_offDiag_block[id]), 1);
				dev_vt_indices_for_jtj[pos] = vtIdx;
				dev_vt_weights_for_jtj[pos] = ngn_weights[i] * ngn_weights[j];
			}
		}		
	}
}


void EDMatchingHelperCudaImpl::
setup_vt_data_for_jtj_vHierED(DeformGraphNodeCuda const*dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu, int const*dev_ed_nodes_num_all_levels,
							 int *dev_ngns_indices, float* dev_ngns_weights, cuda::gpu_size_data vts_num_gpu,
							 int thres_pts_num //ignore jtj off-diagnal block if its associates points is leff than thres_pts_num
							)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_pts_jtj_count_, 0, sizeof(int)));

	short2 info;
	info.x = -1;
	info.y = 0;
	int threads_per_block = 512;
	int blocks_per_grid = (ED_NODES_NUM_MAX*ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	init_short2_array_2d<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos_, ED_NODES_NUM_MAX, info);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemsetAsync(dev_pt_counts_Diag_block_, 0, sizeof(int)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_pt_counts_offDiag_block_, 0, sizeof(int)*ED_NODES_NUM_MAX*(ED_NODES_NUM_MAX - 1) / 2));

	//======= Step 1: voting/counting
	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	associate_pts_to_jtj_kernel_stepCount<<<blocks_per_grid, threads_per_block>>>(dev_ngns_indices, vts_num_gpu.dev_ptr, 
															dev_pt_counts_Diag_block_, dev_pt_counts_offDiag_block_);
	m_checkCudaErrors();

	//======== Step 2: Aggregation of Diagonal blocks
	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	associate_pts_to_jtj_kernel_stepAggreg_Diag<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos_, dev_jtj_blk_info_buf_, 
																					dev_global_pts_jtj_count_, 
																					dev_pt_counts_Diag_block_, 
																					ed_nodes_num_gpu.dev_ptr, dev_ed_nodes_num_all_levels);
	m_checkCudaErrors();

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int global_pts_count = 0;
		checkCudaErrors(cudaMemcpy(&global_pts_count, dev_global_pts_jtj_count_, sizeof(int), cudaMemcpyDeviceToHost));
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> global pts counts: %d", global_pts_count);
		int vts_num = vts_num_gpu.sync_read();
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> vts num*%d: %d", NEIGHBOR_EDNODE_NUM, vts_num*NEIGHBOR_EDNODE_NUM);
	}

	//======== Step 3: Aggregation of off-Diagonal blocks
	checkCudaErrors(cudaMemcpyAsync(jtj_blks_count_gpu_.dev_ptr, dev_ed_nodes_num_all_levels, sizeof(int), cudaMemcpyDeviceToDevice));
	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ED_NODES_NUM_MAX*(ED_NODES_NUM_MAX-1) / 2 + threads_per_block - 1) / threads_per_block;
	associate_pts_to_jtj_kernel_stepAggreg_OffDiag<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos_, 
																		dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size, 
																		dev_global_pts_jtj_count_, jtj_blks_count_gpu_.dev_ptr,
																		dev_pt_counts_offDiag_block_, 
																		ed_nodes_num_gpu.dev_ptr, dev_ed_nodes_num_all_levels,  thres_pts_num);
	m_checkCudaErrors();

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int jtj_blks_count = jtj_blks_count_gpu_.sync_read();
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> JtJ blks count: %d", jtj_blks_count);
	}

	//======== Step 3: save vertex indices
	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	associate_pts_to_jtj_kernel_stepSavingPtIdx<<<blocks_per_grid, threads_per_block>>>(dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_, 
																				dev_ngns_indices, dev_ngns_weights, vts_num_gpu.dev_ptr, 
																				dev_pt_counts_Diag_block_, 
																				dev_pt_counts_offDiag_block_);
	m_checkCudaErrors();
}


#endif