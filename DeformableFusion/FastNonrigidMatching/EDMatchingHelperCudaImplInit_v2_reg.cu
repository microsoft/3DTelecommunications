#ifndef __EDMATCHINGHELPERCUDAIMPLINIT_V2_REG_CU__
#define __EDMATCHINGHELPERCUDAIMPLINIT_V2_REG_CU__
#include "EDMatchingHelperCudaImplInit.cu"

//one ed nodes per thread: 
// each kernel only writes to one element of dev_jtj_2d_infos, not its symmetric element
__global__ 
void connect_ed_nodes_two_levels_kernel(short2 *dev_jtj_2d_infos,
										DeformGraphNodeCuda* dev_ed_nodes, 
										int const*dev_ed_nodes_num_all_levels,
										int2 const* dev_ed_nodes_range_L1, int2 const* dev_ed_nodes_range_L2, 
										float nodes_dist_L2 //average dist 
										)
{
	const int2 ed_nodes_range_L1 = dev_ed_nodes_range_L1[0];
	if (blockIdx.x*blockDim.x > ed_nodes_range_L1.y)
		return;

	extern __shared__ float sh_nodes_pos_L2[]; 

	//load ed nodes of L2
	const int2 ed_nodes_range_L2 = *dev_ed_nodes_range_L2;
	for (int idx = threadIdx.x; idx < ed_nodes_range_L2.y * 3; idx += blockDim.x)
		sh_nodes_pos_L2[idx] = dev_ed_nodes[ed_nodes_range_L2.x + idx / 3].g[idx % 3];
	__syncthreads();

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < ed_nodes_range_L1.y)
	{
		int ndIdx = ed_nodes_range_L1.x + id;
		cuda_vector_fixed<float, 3> g1 = dev_ed_nodes[ndIdx].g;
		
		float dists_sq[EDNODE_NN];
		int ngn_idx[EDNODE_NN];
		for (int i = 0; i < EDNODE_NN; i++)
		{
			dists_sq[i] = 1.0e+10f;
			ngn_idx[i] = -1;
		}

		//find the nearest neighbors
		for (int i = 0; i < ed_nodes_range_L2.y; i++)
		{
			float const* g2 = &(sh_nodes_pos_L2[3 * i]);
			float dist_sq = dist_square<3>(g1.data_block(), g2);

			if (dist_sq < 2.0f*nodes_dist_L2*nodes_dist_L2) //1.7321*nodes_min_dist away
			{
				if (dist_sq < dists_sq[0])
				{
					dists_sq[0] = dist_sq;
					ngn_idx[0] = i;
				}

				for (int c = 1; c < EDNODE_NN; c++)
				{
					if (dist_sq < dists_sq[c])
					{
						dists_sq[c - 1] = dists_sq[c];
						ngn_idx[c - 1] = ngn_idx[c];
						dists_sq[c] = dist_sq;
						ngn_idx[c] = i;
					}
				}
			}
		}

		for (int i = 0; i < EDNODE_NN; i++)
		{
			int ndIdx2 = ngn_idx[i];
			if (ndIdx2 != -1)
			{
				ndIdx2 += ed_nodes_range_L2.x;
				const int ed_nodes_num_all_levels = *dev_ed_nodes_num_all_levels;
				int ele_id = ndIdx*ed_nodes_num_all_levels + ndIdx2;
				dev_jtj_2d_infos[ele_id].y |= 2;
			}
			dev_ed_nodes[ndIdx].neighbors[i] = ndIdx2;
		}
		for (int i = EDNODE_NN; i < EDNODE_NN_MAX; i++)
			dev_ed_nodes[ndIdx].neighbors[i] = -1;
	}
}


//each thread handle one node
//for each nodes, find its k nearest neighbors, and flag corresponding row of dev_jtj_2d_infos
__global__
void connect_ed_nodes_1st_level_kernel_vDist(short2 *dev_jtj_2d_infos,
											 DeformGraphNodeCuda *dev_ed_nodes, 
											 int const *dev_ed_nodes_num,
											 int const*dev_ed_nodes_num_all_levels,
											 int3 const* dev_ed_cubes_dims,
											 float3 const* dev_ed_cubes_offsets,
											 float ed_cube_res,
											 float thres_neighbor_nodes_dist
											 )
{
	int const ed_nodes_num = *dev_ed_nodes_num;
	int ndId = threadIdx.x + blockDim.x*blockIdx.x;

	if (ndId < ed_nodes_num)
	{
		const int ed_nodes_num_all_levels = *dev_ed_nodes_num_all_levels;
		const int3 ed_cubes_dims = *dev_ed_cubes_dims;
		const float3 ed_cubes_offsets = *dev_ed_cubes_offsets;

		cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndId].g;
		int xCubeId = (g[0] - ed_cubes_offsets.x) / ed_cube_res;
		int yCubeId = (g[1] - ed_cubes_offsets.y) / ed_cube_res;
		int zCubeId = (g[2] - ed_cubes_offsets.z) / ed_cube_res;

		int count = 0;
		for (int i = -4; i <= 4; i++)
		for (int j = -4; j <= 4; j++)
		for (int k = -4; k <= 4; k++)
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

					if (dist_sq < thres_neighbor_nodes_dist*thres_neighbor_nodes_dist)
					{
						if (count < EDNODE_NN_MAX)
						{
							dev_ed_nodes[ndId].neighbors[count] = ndId2;

							int id1 = MAX(ndId, ndId2);
							int id2 = MIN(ndId, ndId2);
							int ele_id = id1*ed_nodes_num_all_levels + id2;
							dev_jtj_2d_infos[ele_id].y |= 2;
						}
						count++;
					}
				}
			}
		}

		for (int i = count; i < EDNODE_NN_MAX; i++)
			dev_ed_nodes[ndId].neighbors[i] = -1;
	}
}

//one ed nodes per thread
// each kernel only writes to one element of dev_jtj_2d_infos, not its symmetric element
__global__
void connect_ed_nodes_same_level_kernel(short2 *dev_jtj_2d_infos,
										DeformGraphNodeCuda* dev_ed_nodes,
										int const*dev_ed_nodes_num_all_levels,
										int2 const* dev_ed_nodes_range,
										float nodes_min_dist //average dist at current level
										)
{
	const int2 ed_nodes_range = *dev_ed_nodes_range;
	if (blockIdx.x*blockDim.x > ed_nodes_range.y)
		return;

	extern __shared__ float sh_nodes_pos[]; 

	//load ed nodes
	for (int idx = threadIdx.x; idx < ed_nodes_range.y * 3; idx += blockDim.x)
		sh_nodes_pos[idx] = dev_ed_nodes[ed_nodes_range.x + idx / 3].g[idx % 3];
	__syncthreads();

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < ed_nodes_range.y)
	{
		int ndIdx = ed_nodes_range.x + id;
		cuda_vector_fixed<float, 3> g1 = dev_ed_nodes[ndIdx].g;

		float dists_sq[EDNODE_NN];
		int ngn_idx[EDNODE_NN];
		for (int i = 0; i < EDNODE_NN; i++)
		{
			dists_sq[i] = 1.0e+10f;
			ngn_idx[i] = -1;
		}

		//find the nearest neighbors
		for (int i = 0; i < ed_nodes_range.y; i++)
		{
			if (i != id)
			{
				float const* g2 = &(sh_nodes_pos[3 * i]);
				float dist_sq = dist_square<3>(g1.data_block(), g2);

				if (dist_sq < 16.0f*nodes_min_dist*nodes_min_dist) //2*nodes_min_dist away
				{
					if (dist_sq < dists_sq[0])
					{
						dists_sq[0] = dist_sq;
						ngn_idx[0] = i;
					}

					for (int c = 1; c < EDNODE_NN; c++)
					{
						if (dist_sq < dists_sq[c])
						{
							dists_sq[c - 1] = dists_sq[c];
							ngn_idx[c - 1] = ngn_idx[c];
							dists_sq[c] = dist_sq;
							ngn_idx[c] = i;
						}
					}
				}
			}
		}

		const int ed_nodes_num_all_levels = *dev_ed_nodes_num_all_levels;
		for (int i = 0; i < EDNODE_NN; i++)
		{
			int ndIdx2 = ngn_idx[i];
			if (ndIdx2 != -1)
			{
				ndIdx2 += ed_nodes_range.x;
				int ele_id = ndIdx*ed_nodes_num_all_levels + ndIdx2;
				dev_jtj_2d_infos[ele_id].y |= 2;
			}
			dev_ed_nodes[ndIdx].neighbors[i] = ndIdx2;
		}
		for (int i = EDNODE_NN; i < EDNODE_NN_MAX; i++)
			dev_ed_nodes[ndIdx].neighbors[i] = -1;
	}
}

//NOTE: only one cuda block is used, can handle upto 24k ED nodes with GTX980
//get the symmetric version of ed node neighbors
__global__
void complete_ed_nodes_neighbors_kernel(short2 const*dev_jtj_2d_infos, //the one before running kernel ComposeJtJ2DInfoRegTerm_Kernal
										DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num_all_levels)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;
	__shared__ int ndNeighborsCounts[ED_NODES_NUM_MAX]; //can up to 96k/4 ed nodes
	for (int i = threadIdx.x; i < ed_nodes_num; i += blockDim.x)
		ndNeighborsCounts[i] = EDNODE_NN;
	__syncthreads();

	for (int ndIdx = threadIdx.x; ndIdx < ed_nodes_num; ndIdx += blockDim.x)
	{
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 >= 0 && !(dev_jtj_2d_infos[ed_nodes_num*ndIdx2 + ndIdx].y & 2)) //if ndIdx is not a neighbor of ndIdx2
			{
				int count = atomicAdd(&(ndNeighborsCounts[ndIdx2]), 1);
				if (count < EDNODE_NN_MAX)
					dev_ed_nodes[ndIdx2].neighbors[count] = ndIdx;
			}
		}
	}
	
	////test
	//__syncthreads();
	//for (int i = threadIdx.x; i < ed_nodes_num; i += blockDim.x)
	//	ndNeighborsCounts[threadIdx.x] = MAX(ndNeighborsCounts[threadIdx.x], ndNeighborsCounts[i]);
	//__syncthreads();
	//for (int s = blockDim.x / 2; s > 0; s >>= 1)
	//{
	//	if (threadIdx.x < s)
	//	{
	//		ndNeighborsCounts[threadIdx.x] = MAX(ndNeighborsCounts[threadIdx.x], ndNeighborsCounts[threadIdx.x + s]);
	//	}
	//	__syncthreads();
	//}
	//if (threadIdx.x == 0)
	//	printf("max ed neighbors: %d\n", ndNeighborsCounts[0]);
	
}

//only search the neccesary elements ofdev_jtj_2d_infos to find new new JtJ blocks( for regualarization term)
// one ed nodes per threads: loop over its initial EDNODE_NN neighbors
__global__
void setup_ed_nodes_reg_term_kernel_vFast(ushort2 *dev_reg_node_pairs_list_buf, int node_pairs_list_buf_size, int* dev_global_reg_count,
									short2 *dev_jtj_2d_infos, DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num_all_levels,
									HessianBlockInfoCuda *dev_block_info_buf, int block_info_buf_size, int* dev_jtj_blks_count)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;
	
	if (blockIdx.x*blockDim.x > ed_nodes_num)
		return;

	__shared__ int sh_JtJBlkCount;
	__shared__ int sh_RegCount;

	if (threadIdx.x == 0)
	{
		sh_JtJBlkCount = 0;
		sh_RegCount = 0;
	}
	__syncthreads();

	int ndIdx = threadIdx.x + blockIdx.x*blockDim.x;
	short2 info, info_sym, info_combo;
	int lc_JtJBlkCount = 0;//data position for jtj in dev_jtj
	int lc_RegCount = 0;
	if (ndIdx < ed_nodes_num)
	{
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 != -1)
			{
				info = dev_jtj_2d_infos[ndIdx*ed_nodes_num + ndIdx2];
				short2 info_sym = dev_jtj_2d_infos[ndIdx2*ed_nodes_num + ndIdx];
				info_combo = (ndIdx > ndIdx2)? info : info_sym;
				info_combo.y = info.y | info_sym.y;
				//only proceed if ndIdx > ndIdx2 or ndIdx is not a neighbor of ndIdx2
				if (ndIdx > ndIdx2 || (info_sym.y & 2) == 0)
				{
					lc_RegCount++;
					if (!(info_combo.y & 1))  //if reg only
					{
						lc_JtJBlkCount++;
					}
				}
			}
		}
		lc_RegCount = atomicAdd(&sh_RegCount, lc_RegCount);
		lc_JtJBlkCount = atomicAdd(&sh_JtJBlkCount, lc_JtJBlkCount);
		__syncthreads();

		if (threadIdx.x == 0)
		{
			sh_JtJBlkCount = atomicAdd(dev_jtj_blks_count, sh_JtJBlkCount);
			sh_RegCount = atomicAdd(dev_global_reg_count, sh_RegCount);
		}
		__syncthreads();

		lc_JtJBlkCount += sh_JtJBlkCount;
		lc_RegCount += sh_RegCount;
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 != -1)
			{
				info = dev_jtj_2d_infos[ndIdx*ed_nodes_num + ndIdx2];
				short2 info_sym = dev_jtj_2d_infos[ndIdx2*ed_nodes_num + ndIdx];
				info_combo = (ndIdx > ndIdx2) ? info : info_sym;
				info_combo.y = info.y | info_sym.y;
				//only proceed if ndIdx > ndIdx2 or ndIdx is not a neighbor of ndIdx2
				if (ndIdx > ndIdx2 || (info_sym.y & 2) == 0)
				{
					int i = MAX(ndIdx, ndIdx2);
					int j = MIN(ndIdx, ndIdx2);

					if (lc_RegCount < node_pairs_list_buf_size)
					{
						ushort2 node_pair;
						node_pair.x = i;
						node_pair.y = j; //note: i > j
						dev_reg_node_pairs_list_buf[lc_RegCount] = node_pair;
					}
					lc_RegCount++;

					//if reg only
					if (!(info_combo.y & 1))  
					{
						int pos = lc_JtJBlkCount;
						//write the jtj block info
						dev_block_info_buf[pos].vtii_first = -1;
						dev_block_info_buf[pos].vts_num = 0;
						dev_block_info_buf[pos].data_offset = pos * 144;
						dev_block_info_buf[pos].pi_idx = i;
						dev_block_info_buf[pos].pj_idx = j;
						dev_block_info_buf[pos].pi_dim = 12;
						dev_block_info_buf[pos].pj_dim = 12;

						info_combo.x = pos;

						lc_JtJBlkCount++;
					}

					dev_jtj_2d_infos[i*ed_nodes_num + j] = info_combo;
				}
			}
		}
	}
}

__global__
void setup_ed_nodes_reg_term_kernel_vFast2(ushort2 *dev_reg_node_pairs_list_buf, int node_pairs_list_buf_size, int* dev_global_reg_count,
										   short2 *dev_jtj_2d_infos, DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num_all_levels,
										   HessianBlockInfoCuda *dev_block_info_buf, int block_info_buf_size, int* dev_jtj_blks_count)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

	if (blockIdx.x*blockDim.x > ed_nodes_num)
		return;

	__shared__ int sh_JtJBlkCount;
	__shared__ int sh_RegCount;

	if (threadIdx.x == 0)
	{
		sh_JtJBlkCount = 0;
		sh_RegCount = 0;
	}
	__syncthreads();

	int ndIdx = threadIdx.x + blockIdx.x*blockDim.x;
	short2 info, info_sym, info_combo;
	int lc_JtJBlkCount = 0;//data position for jtj in dev_jtj
	int lc_RegCount = 0;
	if (ndIdx < ed_nodes_num)
	{
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 != -1)
			{
				info = dev_jtj_2d_infos[ndIdx*ed_nodes_num + ndIdx2];
				short2 info_sym = dev_jtj_2d_infos[ndIdx2*ed_nodes_num + ndIdx];
				info_combo = (ndIdx > ndIdx2) ? info : info_sym;
				info_combo.y = info.y | info_sym.y;
				//only proceed if ndIdx > ndIdx2 or ndIdx is not a neighbor of ndIdx2
				if (ndIdx > ndIdx2 || (info_sym.y & 2) == 0)
				{
					lc_RegCount++;
					if (!(info_combo.y & 1))  //if reg only
					{
						lc_JtJBlkCount++;
					}
				}
			}
		}
		lc_RegCount = atomicAdd(&sh_RegCount, lc_RegCount);
		lc_JtJBlkCount = atomicAdd(&sh_JtJBlkCount, lc_JtJBlkCount);
		__syncthreads();

		if (threadIdx.x == 0)
		{
			sh_JtJBlkCount = atomicAdd(dev_jtj_blks_count, sh_JtJBlkCount);
			sh_RegCount = atomicAdd(dev_global_reg_count, sh_RegCount);
		}
		__syncthreads();

		lc_JtJBlkCount += sh_JtJBlkCount;
		lc_RegCount += sh_RegCount;
		for (int k = 0; k < EDNODE_NN; k++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[k];
			if (ndIdx2 != -1)
			{
				info = dev_jtj_2d_infos[ndIdx*ed_nodes_num + ndIdx2];
				short2 info_sym = dev_jtj_2d_infos[ndIdx2*ed_nodes_num + ndIdx];
				info_combo = (ndIdx > ndIdx2) ? info : info_sym;
				info_combo.y = info.y | info_sym.y;
				//only proceed if ndIdx > ndIdx2 or ndIdx is not a neighbor of ndIdx2
				if (ndIdx > ndIdx2 || (info_sym.y & 2) == 0)
				{
					int i = MAX(ndIdx, ndIdx2);
					int j = MIN(ndIdx, ndIdx2);

					if (lc_RegCount < node_pairs_list_buf_size)
					{
						ushort2 node_pair;
						node_pair.x = i;
						node_pair.y = j; //note: i > j
						dev_reg_node_pairs_list_buf[lc_RegCount] = node_pair;
					}
					lc_RegCount++;

					//if reg only
					if (!(info_combo.y & 1))
					{
						int pos = lc_JtJBlkCount;
						//write the jtj block info
						dev_block_info_buf[pos].vtii_first = -1;
						dev_block_info_buf[pos].vts_num = 0;
						dev_block_info_buf[pos].data_offset = pos * 144;
						dev_block_info_buf[pos].pi_idx = i;
						dev_block_info_buf[pos].pj_idx = j;
						dev_block_info_buf[pos].pi_dim = 12;
						dev_block_info_buf[pos].pj_dim = 12;

						info_combo.x = pos;

						lc_JtJBlkCount++;
					}

					dev_jtj_2d_infos[i*ed_nodes_num + j] = info_combo;
				}
			}
		}
	}
}

//search the lower triangle matrix of dev_jtj_2d_infos to find new JtJ blocks( for regualarization term)
//one element per thread
//
//get dev_jtj offset for each reg-only jtj block
//get the list of node pairs for reg term
__global__
void setup_ed_nodes_reg_term_kernel(ushort2 *dev_reg_node_pairs_list_buf, int node_pairs_list_buf_size, int* dev_global_reg_count,
									short2 *dev_jtj_2d_infos, int const* dev_ed_nodes_num_all_levels,
									HessianBlockInfoCuda *dev_block_info_buf, int block_info_buf_size, int* dev_jtj_blks_count )
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

	if (blockIdx.x*blockDim.x > ed_nodes_num*ed_nodes_num)
		return;

	__shared__ int sh_JtJBlkCount;
	__shared__ int shRegCount;

	if (threadIdx.x == 0)
	{
		sh_JtJBlkCount = 0;
		shRegCount = 0;
	}
	__syncthreads();

	int idx = threadIdx.x + blockIdx.x*blockDim.x;

	int lc_JtJBlkCount = 0;//data position for jtj in dev_jtj
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
					lc_JtJBlkCount = atomicAdd(&sh_JtJBlkCount, 1);
				}
			}
		}
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		sh_JtJBlkCount = atomicAdd(dev_jtj_blks_count, sh_JtJBlkCount);
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
				int pos = sh_JtJBlkCount + lc_JtJBlkCount;
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
					node_pair.y = j; //note: i > j
					dev_reg_node_pairs_list_buf[lcRegCount] = node_pair;
				}
			}
		}
	}
}

void EDMatchingHelperCudaImpl::
setup_ed_nodes_reg_term_vHierED(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num_all_levels,
								int2* dev_ed_nodes_ranges[],
								int ed_hierachy_levels,
								int3 const* ed_cubes_dims,
								float3 const* ed_cubes_offsets,
								float ed_cube_res,
								float nodes_min_dist
								)
{
	const int ed_hier_downsample_factor = 3;
	for (int i = 0; i < ed_hierachy_levels - 1; i++)
	{
		nodes_min_dist *= ed_hier_downsample_factor;

		int ed_nodes_num_max_L1 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i);
		int ed_nodes_num_max_L2 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i+1);


		int threads_per_block = MAX_THREADS_PER_BLOCK;
		int blocks_per_grid = (ed_nodes_num_max_L1 + threads_per_block - 1) / threads_per_block;
		connect_ed_nodes_two_levels_kernel<<<blocks_per_grid, threads_per_block, ed_nodes_num_max_L2*12>>>(dev_jtj_2d_infos_, dev_ed_nodes, dev_ed_nodes_num_all_levels, 
																					dev_ed_nodes_ranges[i], dev_ed_nodes_ranges[i + 1], 
																					nodes_min_dist);
		m_checkCudaErrors();
	}

	int ed_nodes_num_max_last = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, ed_hierachy_levels - 1);
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ed_nodes_num_max_last + threads_per_block - 1) / threads_per_block;
	connect_ed_nodes_same_level_kernel<<<blocks_per_grid, threads_per_block, ed_nodes_num_max_last*12>>>(dev_jtj_2d_infos_, dev_ed_nodes, 
																	dev_ed_nodes_num_all_levels, 
																	dev_ed_nodes_ranges[ed_hierachy_levels - 1],
																	nodes_min_dist);
	m_checkCudaErrors();

	complete_ed_nodes_neighbors_kernel<<<1, MAX_THREADS_PER_BLOCK>>>(dev_jtj_2d_infos_, dev_ed_nodes, dev_ed_nodes_num_all_levels);
	m_checkCudaErrors();

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int jtj_blk_num = jtj_blks_count_gpu_.sync_read();
		LOGGER()->debug("<<<<<<<<<<<<<<<<<<< jtj blk num (data only): %d", jtj_blk_num);
	}

	checkCudaErrors(cudaMemsetAsync(ed_reg_pairs_count_gpu_.dev_ptr, 0, sizeof(int)));
	
	//threads_per_block = MAX_THREADS_PER_BLOCK;
	//blocks_per_grid = (ED_NODES_NUM_MAX*ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	//setup_ed_nodes_reg_term_kernel<<<blocks_per_grid, threads_per_block>>>(dev_reg_node_pairs_list_buf_, 
	//												ed_reg_pairs_count_gpu_.max_size, ed_reg_pairs_count_gpu_.dev_ptr, 
	//												dev_jtj_2d_infos_, dev_ed_nodes_num_all_levels,
	//												dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size, jtj_blks_count_gpu_.dev_ptr);

	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	setup_ed_nodes_reg_term_kernel_vFast<<<blocks_per_grid, threads_per_block>>>(dev_reg_node_pairs_list_buf_,
														ed_reg_pairs_count_gpu_.max_size, ed_reg_pairs_count_gpu_.dev_ptr, 
														dev_jtj_2d_infos_, dev_ed_nodes, dev_ed_nodes_num_all_levels,
														dev_jtj_blk_info_buf_, jtj_blks_count_gpu_.max_size, jtj_blks_count_gpu_.dev_ptr);
	m_checkCudaErrors();

	if(LOGGER()->check_verbosity(Logger::Debug))
	{
		int jtj_blk_num = jtj_blks_count_gpu_.sync_read();
		LOGGER()->debug("<<<<<<<<<<<<<<<<<<< jtj blk num (with reg): %d", jtj_blk_num);

		int reg_count = ed_reg_pairs_count_gpu_.sync_read();
		LOGGER()->debug("<<<<<<<<<<<<<<<<<<< reg pairs count: %d", ed_reg_pairs_count_gpu_.sync_read());

	}
}



#endif
