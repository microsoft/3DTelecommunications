#ifndef __EDMATCHINGHELPERCUDAIMPLINIT_CU__
#define __EDMATCHINGHELPERCUDAIMPLINIT_CU__

#define THREADS_PER_BLOCK 1024
#define DIAGONALJTJBLKS_PER_CUDA_BLOCK (1024*3)
#define OFFDIAGONALBLKS_PER_CUDA_BLOCK (1024*11)
#define CHUNK_SIZE 1536 //how many points info to load each time, related to shared memory buffer size

texture<short, cudaTextureType3D, cudaReadModeElementType> tex_ndIds;

__device__ int dev_globalPos; //count of jtj blocks (diagonal only)
__device__ int dev_globalOffset;

__device__ int dev_globalOffset2;

__device__ int dev_globalRegCount;

//each thread handle one node
//for each nodes, find its k nearest neighbors, and flag corresponding row of dev_jtj_2d_infos
__global__ 
void FindEDNodesNeighbors_Kernel(short2 *dev_jtj_2d_infos,
								DeformGraphNodeCuda *dev_ed_nodes, int ed_nodes_num,
								int3 ed_cubes_dims,
								float3 ed_cubes_offsets,
								float ed_cube_res,
								float nodes_min_dist
								)
{
	int ndId = threadIdx.x + blockDim.x*blockIdx.x;
	if (ndId < ed_nodes_num)
	{
		cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndId].g;
		int xCubeId = (g[0] - ed_cubes_offsets.x) / ed_cube_res;
		int yCubeId = (g[1] - ed_cubes_offsets.y) / ed_cube_res;
		int zCubeId = (g[2] - ed_cubes_offsets.z) / ed_cube_res;

		float dists_sq[EDNODE_NN];
		int ngn_idx[EDNODE_NN];
		for (int i = 0; i < EDNODE_NN; i++)
		{
			dists_sq[i] = 1.0e+10f;
			ngn_idx[i] = -1;
		}

		for (int i = -2; i <= 2; i++)
		for (int j = -2; j <= 2; j++)
		for (int k = -2; k <= 2; k++)
		{
			int xId = xCubeId + i;
			int yId = yCubeId + j;
			int zId = zCubeId + k;

			if (xId >= 0 && yId >= 0 && zId >= 0 &&
				xId < ed_cubes_dims.x && yId < ed_cubes_dims.y && zId < ed_cubes_dims.z)
			{
				short ndId2 = tex3D(tex_ndIds, xId, yId, zId);
				if (ndId2 >= 0 && ndId2 != ndId)
				{
					cuda_vector_fixed<float, 3> g2 = dev_ed_nodes[ndId2].g;
					float dist_sq = dist_square<3>(g.data_block(), g2.data_block());

					if (dist_sq < 4.0f*nodes_min_dist*nodes_min_dist) //2*nodes_min_dist away
					{
						if (dist_sq < dists_sq[0])
						{
							dists_sq[0] = dist_sq;
							ngn_idx[0] = ndId2;
						}

						for (int c = 1; c < EDNODE_NN; c++)
						{
							if (dist_sq < dists_sq[c])
							{
								dists_sq[c - 1] = dists_sq[c];
								ngn_idx[c - 1] = ngn_idx[c];
								dists_sq[c] = dist_sq;
								ngn_idx[c] = ndId2;
							}
						}
					}
				}
			}
		}

		for (int i = 0; i < EDNODE_NN; i++)
		{
			dev_ed_nodes[ndId].neighbors[i] = ngn_idx[i];
			if (ngn_idx[i] != -1)
			{
				int id = ndId*ed_nodes_num + ngn_idx[i];
				dev_jtj_2d_infos[id].y |= 2;
			}
		}
		for (int i = EDNODE_NN; i < EDNODE_NN_MAX; i++)
			dev_ed_nodes[ndId].neighbors[i] = -1;
	}
}


//TODO
//NOTE: only one cuda block is used, can handle upto 24k ED nodes with GTX980
//get the symmetric version of ed node neighbors
__global__
void CompleteEDNodeNeighbors_Kernel(short2 const*dev_jtj_2d_infos, //the one before running kernel ComposeJtJ2DInfoRegTerm_Kernal
									DeformGraphNodeCuda *dev_ed_nodes, int ed_nodes_num)
{
	__shared__ int ndNeighborsCounts[ED_NODES_NUM_MAX]; //can up to 96k/4 ed nodes
	for (int i = threadIdx.x; i < ed_nodes_num; i += blockDim.x)
		ndNeighborsCounts[i] = EDNODE_NN;
	__syncthreads();

	for (int ndIdx = threadIdx.x; ndIdx < ed_nodes_num; ndIdx += blockDim.x)
	{
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 >= 0 && !(dev_jtj_2d_infos[ed_nodes_num*ndIdx2+ndIdx].y & 2) ) //if ndIdx is not a neighbor of ndIdx2
			{
				int count = atomicAdd(&(ndNeighborsCounts[ndIdx2]), 1);
				dev_ed_nodes[ndIdx2].neighbors[count] = ndIdx;
			}
		}
	}
}

//get dev_jtj offset for each reg-only jtj block
//get the list of node pairs for reg term
__global__
void SetupEDNodesRegTerm_Kernal(ushort2 *dev_reg_node_pairs_list_buf, int node_pairs_list_buf_size, int *dev_global_reg_count,
							    short2 *dev_jtj_2d_infos, int ed_nodes_num, int* dev_jtj_blks_count,
								HessianBlockInfoCuda *dev_block_info_buf, int block_info_buf_size)
{
	__shared__ int shPos;
	__shared__ int shRegCount;

	if (threadIdx.x == 0)
	{
		shPos = 0;
		shRegCount = 0;
	}
	__syncthreads();

	int idx = threadIdx.x + blockIdx.x*blockDim.x;

	int lcPos = 0;//data position for jtj in dev_jtj
	int lcRegCount = 0;
	short2 t_new;
	if (idx < ed_nodes_num*ed_nodes_num)
	{
		int i = idx / ed_nodes_num;
		int j = idx % ed_nodes_num;

		if (i > j)
		{
			t_new = dev_jtj_2d_infos[idx];
			t_new.y |= dev_jtj_2d_infos[j*ed_nodes_num + i].y;

			if (t_new.y & 2) //if reg
			{
				lcRegCount = atomicAdd(&shRegCount, 1);
				if (!(t_new.y & 1))  //if reg only
				{
					lcPos = atomicAdd(&shPos, 1);
				}
			}
		}
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		shPos = atomicAdd(dev_jtj_blks_count, shPos);
		shRegCount = atomicAdd(dev_global_reg_count, shRegCount);
	}
	__syncthreads();

	if (idx < ed_nodes_num*ed_nodes_num)
	{
		int i = idx / ed_nodes_num;
		int j = idx % ed_nodes_num;
		if (i > j)
		{
			//write jtj data offset
			if ((t_new.y & 2) && !(t_new.y & 1))
			{
				int pos = shPos + lcPos;
				t_new.x = pos;

				//write the jtj block info
				dev_block_info_buf[pos].vtii_first = -1;
				dev_block_info_buf[pos].vts_num = 0;
				dev_block_info_buf[pos].data_offset = pos * 144;
				dev_block_info_buf[pos].pi_idx = i;
				dev_block_info_buf[pos].pj_idx = j;
				dev_block_info_buf[pos].pi_dim = 12;
				dev_block_info_buf[pos].pj_dim = 12;
			}
			dev_jtj_2d_infos[idx] = t_new;

			//write node pair indices
			if (t_new.y & 2)
			{
				lcRegCount += shRegCount;
				if (lcRegCount < node_pairs_list_buf_size)
				{
					ushort2 node_pair;
					node_pair.x = i;
					node_pair.y = j;
					dev_reg_node_pairs_list_buf[lcRegCount] = node_pair;
				}
			}
		}
	}
}


__global__ 
void init_short2_array_2d(short2 *p_array, int dim, short2 val)
{
	int idx = threadIdx.x + blockIdx.x*blockDim.x;
	if (idx < dim*dim)
		p_array[idx] = val;	
}

//Note: all the diagonal blocks have HessianBlockInfoCuda items in the successive order, and their jtj data is also in order.
//      even when no points associated with it
__global__
void CountPtsForDiagKernel_vAtomic( short2 *dev_jtj_2d_infos,
									int const*dev_ngns_indices, float const* dev_ngns_weights, int vts_num,
									int* dev_vt_indices_for_jtj, float* dev_vt_weights_for_jtj,
									HessianBlockInfoCuda *dev_block_info_buf, 
									int block_info_buf_size,
									int ed_nodes_num
									)
{
	__shared__ int jtjBlkPtsCounts[MAX_THREADS_PER_BLOCK]; //number of points associated with a jtj block
	int blkId = threadIdx.x + MAX_THREADS_PER_BLOCK*blockIdx.x;
	int blockStart = MAX_THREADS_PER_BLOCK*blockIdx.x;
	int blockEnd = MAX_THREADS_PER_BLOCK;
	jtjBlkPtsCounts[threadIdx.x] = 0;

	__shared__ int shOffset;

	if (threadIdx.x == 0)
	{
		//shPos = 0;
		shOffset = 0;
	}
	__syncthreads();

	int localPos = 0;
	int localOffset = 0;

	for (int ckSt = threadIdx.x; ckSt < vts_num* NEIGHBOR_EDNODE_NUM; ckSt += MAX_THREADS_PER_BLOCK)
	{
		int ngn_idx = dev_ngns_indices[ckSt];
		if (ngn_idx >= 0) //ngn_idx might be invalid (-1)
		{
			int id = ngn_idx - blockStart;
			if (id >= 0 && id < blockEnd)
			{
				atomicAdd(&(jtjBlkPtsCounts[id]), 1);
			}
		}
	}
	__syncthreads();

	if (jtjBlkPtsCounts[threadIdx.x])
	{
		localOffset = atomicAdd(&shOffset, jtjBlkPtsCounts[threadIdx.x]);
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		shOffset = atomicAdd(&dev_globalOffset, shOffset);
	}
	__syncthreads();

	//write out count information
	int pos = blockStart+threadIdx.x;//in order
	int offset = shOffset + localOffset;
	if (pos < block_info_buf_size && pos < ed_nodes_num)
	{
		dev_block_info_buf[pos].vtii_first = offset;
		dev_block_info_buf[pos].vts_num = jtjBlkPtsCounts[threadIdx.x];
		dev_block_info_buf[pos].data_offset = pos * 144;
		dev_block_info_buf[pos].pi_idx = blkId;
		dev_block_info_buf[pos].pj_idx = blkId;
		dev_block_info_buf[pos].pi_dim = 12;
		dev_block_info_buf[pos].pj_dim = 12;
		jtjBlkPtsCounts[threadIdx.x] = offset;

		short2 info;
		info.x = pos;
		info.y = 1;
		dev_jtj_2d_infos[pos*ed_nodes_num + pos] = info;
	}
	__syncthreads();
	
	//write out indices
	for (int ckSt = threadIdx.x; ckSt < vts_num* NEIGHBOR_EDNODE_NUM; ckSt += MAX_THREADS_PER_BLOCK)
	{
		int ngn_idx = dev_ngns_indices[ckSt];
		if (ngn_idx >= 0) //ngn_idx might be invalid (-1)
		{
			int id = ngn_idx - blockStart;
			float w = dev_ngns_weights[ckSt];
			if (id >= 0 && id < blockEnd)
			{
				//where to place the idx
				int vt_offset = atomicAdd(&(jtjBlkPtsCounts[id]), 1);
				dev_vt_indices_for_jtj[vt_offset] = ckSt / NEIGHBOR_EDNODE_NUM;
				dev_vt_weights_for_jtj[vt_offset] = w * w;
			}
		}
	}
	
}


__global__
void CountPtsForOffDiagKernel_vAtomic(short2 * __restrict__ dev_jtj_2d_infos,
									  int const* __restrict__ dev_ngns_indices, float const* __restrict__ dev_ngns_weights, int vts_num,
									  int* __restrict__ dev_vt_indices_for_jtj, float* __restrict__ dev_vt_weights_for_jtj,
									  HessianBlockInfoCuda * __restrict__ dev_block_info_buf,
									  int block_info_buf_size,
									  int* __restrict__ dev_jtj_blks_count,
									  int ed_nodes_num,
									  int thres_pt_count)
{
	__shared__ int jtjBlkPtsCounts[OFFDIAGONALBLKS_PER_CUDA_BLOCK]; //number of points associated with a jtj block
	__shared__ int shPos;
	__shared__ int shOffset;

	int blockStart = OFFDIAGONALBLKS_PER_CUDA_BLOCK*blockIdx.x;
	int blockEnd = OFFDIAGONALBLKS_PER_CUDA_BLOCK;
	for (int i = threadIdx.x; i < OFFDIAGONALBLKS_PER_CUDA_BLOCK; i += blockDim.x)
	{
		jtjBlkPtsCounts[i] = 0;
	}
	if (threadIdx.x == 0)
	{
		shPos = 0;
		shOffset = 0;
	}
	__syncthreads();

	for (int ckSt = threadIdx.x; ckSt < vts_num; ckSt += MAX_THREADS_PER_BLOCK)
	{
		int ngn_idx[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_idx[i] = dev_ngns_indices[NEIGHBOR_EDNODE_NUM*ckSt + i];

		int id1, id2, id;
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		for (int j = 0; j < i; j++)
		{
			id1 = MAX(ngn_idx[i], ngn_idx[j]);
			id2 = MIN(ngn_idx[i], ngn_idx[j]); //id1/id2 might be invalid (-1)
			id = id1*(id1 - 1) / 2 + id2 - blockStart;
			if (id2 >= 0 && id >= 0 && id < blockEnd)
				atomicAdd(&(jtjBlkPtsCounts[id]), 1);
		}	
	}
	__syncthreads();

	int localPos[OFFDIAGONALBLKS_PER_CUDA_BLOCK / MAX_THREADS_PER_BLOCK];
	int localOffset[OFFDIAGONALBLKS_PER_CUDA_BLOCK / MAX_THREADS_PER_BLOCK];
	for (int i = threadIdx.x, li = 0; i < OFFDIAGONALBLKS_PER_CUDA_BLOCK; i += MAX_THREADS_PER_BLOCK, li++)
	{
		if (jtjBlkPtsCounts[i]>thres_pt_count)
		{
			localPos[li] = atomicAdd(&shPos, 1);
			localOffset[li] = atomicAdd(&shOffset, jtjBlkPtsCounts[i]);
		}
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		shPos = atomicAdd(dev_jtj_blks_count, shPos);
		shOffset = atomicAdd(&dev_globalOffset2, shOffset);
	}
	__syncthreads();

	//write out count information
	for (int i = threadIdx.x, li = 0; i < OFFDIAGONALBLKS_PER_CUDA_BLOCK; i += MAX_THREADS_PER_BLOCK, li++)
	{
		int pos = shPos + localPos[li];
		int offset = shOffset + localOffset[li];
		if (pos < block_info_buf_size && jtjBlkPtsCounts[i] > thres_pt_count)
		{
			dev_block_info_buf[pos].vtii_first = offset;
			dev_block_info_buf[pos].vts_num = jtjBlkPtsCounts[i];
			dev_block_info_buf[pos].data_offset = pos * 144;
			int pi_idx = 0;
			int pj_idx = 0;
			decompose_triangle_id(blockStart + i, pi_idx, pj_idx);
			dev_block_info_buf[pos].pi_idx = pi_idx;
			dev_block_info_buf[pos].pj_idx = pj_idx;
			dev_block_info_buf[pos].pi_dim = 12;
			dev_block_info_buf[pos].pj_dim = 12;
			jtjBlkPtsCounts[i] = offset;

			short2 info;
			info.x = pos;
			info.y = 1;
			dev_jtj_2d_infos[pi_idx*ed_nodes_num + pj_idx] = info;
		}
	}
	__syncthreads();

	//write out indices
	for (int ckSt = threadIdx.x; ckSt < vts_num; ckSt += MAX_THREADS_PER_BLOCK)
	{
		int ngn_idx[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_idx[i] = dev_ngns_indices[NEIGHBOR_EDNODE_NUM * ckSt + i];

		float ngn_weights[NEIGHBOR_EDNODE_NUM];
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
			ngn_weights[i] = dev_ngns_weights[NEIGHBOR_EDNODE_NUM * ckSt + i];

		int id1, id2, id;
		int vt_offset, blk_offset;

		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		for (int j = 0; j < i; j++)
		{
			id1 = MAX(ngn_idx[i], ngn_idx[j]);
			id2 = MIN(ngn_idx[i], ngn_idx[j]);
			id = id1*(id1 - 1) / 2 + id2 - blockStart;
			if (id2 >= 0 && id >= 0 && id < blockEnd && jtjBlkPtsCounts[id] > thres_pt_count)
			{
				vt_offset = atomicAdd(&(jtjBlkPtsCounts[id]), 1);
				dev_vt_indices_for_jtj[vt_offset] = ckSt;
				dev_vt_weights_for_jtj[vt_offset] = ngn_weights[i] * ngn_weights[j];
			}
		}		
	}
}

bool EDMatchingHelperCudaImpl::setup_vt_data_for_jtj(DeformGraphNodeCuda const*dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu,
													int *dev_ngns_indices, float* dev_ngns_weights, int vts_num,
													int thres_pts_num)
{
	short2 info;
	info.x = -1;
	info.y = 0;
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_NODES_NUM_MAX*ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	init_short2_array_2d<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos_, ED_NODES_NUM_MAX, info);
	m_checkCudaErrors();

	int ed_nodes_num = ed_nodes_num_gpu.sync_read();

	int tmp = 0;
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_globalPos, &tmp, sizeof(int)));
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_globalOffset, &tmp, sizeof(int)));

	checkCudaErrors(cudaMemcpyAsync(jtj_blks_count_gpu_.dev_ptr, ed_nodes_num_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	tmp = vts_num * NEIGHBOR_EDNODE_NUM;
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_globalOffset2, &tmp, sizeof(int)));


	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ed_nodes_num*(ed_nodes_num - 1) / 2 + OFFDIAGONALBLKS_PER_CUDA_BLOCK - 1) / OFFDIAGONALBLKS_PER_CUDA_BLOCK;
	CountPtsForOffDiagKernel_vAtomic<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos_,
																			  dev_ngns_indices, dev_ngns_weights, vts_num,
																			  dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_,
																			  dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size,
																			  jtj_blks_count_gpu_.dev_ptr,
																			  ed_nodes_num,
																			  thres_pts_num);
	m_checkCudaErrors();


	blocks_per_grid = (ed_nodes_num + threads_per_block - 1) / threads_per_block;
	CountPtsForDiagKernel_vAtomic<<<blocks_per_grid, threads_per_block >>>(dev_jtj_2d_infos_,
																			dev_ngns_indices, dev_ngns_weights, vts_num,
																			dev_vt_indices_for_jtj_, dev_vt_weights_for_jtj_,
																			dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size,
																			ed_nodes_num);
	m_checkCudaErrors();


	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int global_pts_count = 0;
		checkCudaErrors(cudaMemcpyFromSymbol(&global_pts_count, dev_globalOffset, sizeof(int)));
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> global pts counts: %d", global_pts_count);
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> vts num*4: %d", vts_num * 4);

		int diag_blks_count = 0;
		checkCudaErrors(cudaMemcpyFromSymbol(&diag_blks_count, dev_globalPos, sizeof(int)));
		int jtj_blks_count = jtj_blks_count_gpu_.sync_read();
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> Diag blks count: %d", diag_blks_count);
		LOGGER()->debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>> jtj blks count: %d", jtj_blks_count);
	}

	return true;
}

bool EDMatchingHelperCudaImpl::setup_ed_nodes_reg_term( short2 *dev_jtj_2d_infos,
														DeformGraphNodeCuda *dev_ed_nodes, int ed_nodes_num,
														cudaArray* cu_3dArr_ndIds,
														int3 ed_cubes_dims,
														float3 ed_cubes_offsets,
														float ed_cube_res,
														float nodes_min_dist
														)
{
	int threads_per_block = 64;
	int blocks_per_grid = (ed_nodes_num + threads_per_block-1) / threads_per_block;
	FindEDNodesNeighbors_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_jtj_2d_infos,
																		dev_ed_nodes, ed_nodes_num,
																		ed_cubes_dims, ed_cubes_offsets, ed_cube_res,
																		nodes_min_dist);
	m_checkCudaErrors();
	
	CompleteEDNodeNeighbors_Kernel<<<1, MAX_THREADS_PER_BLOCK>>>(dev_jtj_2d_infos, dev_ed_nodes, ed_nodes_num);
	m_checkCudaErrors();

	int jtj_blk_num_data = jtj_blks_count_gpu_.sync_read();
	LOGGER()->debug("jtj blk num data = %d", jtj_blk_num_data);

	int tmp = 0;

	checkCudaErrors(cudaMemsetAsync(ed_reg_pairs_count_gpu_.dev_ptr, 0, sizeof(int)));
	threads_per_block = 1024;
	blocks_per_grid = (ed_nodes_num*ed_nodes_num + threads_per_block - 1) / threads_per_block;
	SetupEDNodesRegTerm_Kernal<<<blocks_per_grid, threads_per_block>>>(dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.max_size, ed_reg_pairs_count_gpu_.dev_ptr,
																	   dev_jtj_2d_infos, ed_nodes_num, jtj_blks_count_gpu_.dev_ptr,
																	   dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size);
	m_checkCudaErrors();

	int jtj_blk_num_all = jtj_blks_count_gpu_.sync_read();
	LOGGER()->debug("jtj blk num all = %d", jtj_blk_num_all);
	LOGGER()->debug("para blks num = %d", jtj_para_blks_num_);

	int reg_count = ed_reg_pairs_count_gpu_.sync_read();
	LOGGER()->debug("reg counts = %d", reg_count);

	return true;
}

int EDMatchingHelperCudaImpl::read_globalRegCount()
{
	int ret;
	checkCudaErrors(cudaMemcpyFromSymbol(&ret, dev_globalRegCount, sizeof(int)));
	return ret;
}

int EDMatchingHelperCudaImpl::read_globalPos()
{
	int ret;
	checkCudaErrors(cudaMemcpyFromSymbol(&ret, dev_globalPos, sizeof(int)));
	return ret;
}
int EDMatchingHelperCudaImpl::read_globalOffset()
{
	int ret;
	checkCudaErrors(cudaMemcpyFromSymbol(&ret, dev_globalOffset, sizeof(int)));
	return ret;
}

int EDMatchingHelperCudaImpl::read_globalOffset2()
{
	int ret;
	checkCudaErrors(cudaMemcpyFromSymbol(&ret, dev_globalOffset2, sizeof(int)));
	return ret;
}

bool EDMatchingHelperCudaImpl::bind_tex_ndId(cudaArray* cu_3dArr_ndIds)
{
	// set texture parameters
	tex_ndIds.addressMode[0] = cudaAddressModeClamp;
	tex_ndIds.addressMode[1] = cudaAddressModeClamp;
	tex_ndIds.addressMode[2] = cudaAddressModeClamp;
	tex_ndIds.filterMode = cudaFilterModePoint;
	tex_ndIds.normalized = false;  // access with un-normalized texture coordinates
	// Bind the array to the texture
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
	checkCudaErrors(cudaBindTextureToArray(tex_ndIds, cu_3dArr_ndIds, channelDesc));

	return true;
}

void EDMatchingHelperCudaImpl::allocate_jtj_related_memory(int vts_num_max)
{
	checkCudaErrors(cudaMalloc(&(jtj_blks_count_gpu_.dev_ptr), sizeof(int)));
	jtj_blks_count_gpu_.max_size = JTJ_BLKS_NUM_MAX;
	checkCudaErrors(cudaMalloc(&dev_jtj_blk_info_buf_, sizeof(HessianBlockInfoCuda)*JTJ_BLKS_NUM_MAX));
	checkCudaErrors(cudaMalloc(&dev_jtj_2d_infos_, sizeof(short2)*ED_NODES_NUM_MAX*ED_NODES_NUM_MAX));

	checkCudaErrors(cudaMalloc(&dev_vt_indices_for_jtj_, sizeof(int)*vts_num_max*JTJ_BLKS_NUM_INDUCED_PER_VERTEX));
	checkCudaErrors(cudaMalloc(&dev_vt_weights_for_jtj_, sizeof(float)*vts_num_max*JTJ_BLKS_NUM_INDUCED_PER_VERTEX));
	checkCudaErrors(cudaMalloc(&dev_jtj_, sizeof(float)*JTJ_BLKS_NUM_MAX * 144));
	checkCudaErrors(cudaMalloc(&dev_jtf_, sizeof(float)*ED_NODES_NUM_MAX * 12));
	checkCudaErrors(cudaMalloc(&dev_cam_vis_, sizeof(ushort2)*vts_num_max));

	//for jtj block-vertex association
	checkCudaErrors(cudaMalloc(&dev_global_pts_jtj_count_, sizeof(int)));
	checkCudaErrors(cudaMalloc(&dev_pt_counts_Diag_block_, sizeof(int)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMalloc(&dev_pt_counts_offDiag_block_, sizeof(int)*ED_NODES_NUM_MAX*(ED_NODES_NUM_MAX - 1) / 2));


	//ed node regualization term
	checkCudaErrors(cudaMalloc(&(ed_reg_pairs_count_gpu_.dev_ptr), sizeof(int)));
	ed_reg_pairs_count_gpu_.max_size = ED_NODES_NUM_MAX * EDNODE_NN;
	checkCudaErrors(cudaMalloc(&dev_reg_node_pairs_list_buf_, sizeof(ushort2)*ed_reg_pairs_count_gpu_.max_size));
}

#include "EDMatchingHelperCudaImplInit_v2.cu"
#include "EDMatchingHelperCudaImplInit_v2_reg.cu"
#endif