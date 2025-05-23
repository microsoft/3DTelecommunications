// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
__device__ int dev_ed_cubes_count = 0;
surface<void, cudaSurfaceType3D> surf_ndIds; //short
texture<short, cudaTextureType3D, cudaReadModeElementType> tex_ndIds;
texture<short, cudaTextureType3D, cudaReadModeElementType> tex_ndIds_old;


//one points per thread
__global__
void EDNodesSamplerKernel_vGMem_stepVoting( int *dev_pt_counts_per_cube, float3 *dev_pt_centroids_per_cube, float3 *dev_pt_norms_per_cube,
											float const* dev_vts, int const* dev_vts_num, int vts_num_max, int stride,
											float3 const* dev_ed_cubes_offset,
											int3  const* dev_ed_cubes_num,
											int const *dev_pt_counts_per_cube_background,
											float3 const* dev_ed_cubes_offset_background,
											int3  const* dev_ed_cubes_num_background,
											int thres_pt_count,
											float ed_cube_res)
{
	int const vts_num = MIN(vts_num_max, *dev_vts_num);

	int vtIdx = threadIdx.x + blockDim.x*blockIdx.x;
	if (vtIdx < vts_num)
	{
		float const*p_vt = dev_vts + vtIdx * stride;
		float vt[3];
		vt[0] = p_vt[0];
		vt[1] = p_vt[1];
		vt[2] = p_vt[2];
		float n[3];
		n[0] = p_vt[3];
		n[1] = p_vt[4];
		n[2] = p_vt[5];

		if (!isnan(vt[0]))
		{
			if (dev_pt_counts_per_cube_background)
			{
				float3 const ed_cubes_offset_background = dev_ed_cubes_offset_background[0];
				int3 const ed_cubes_num_background = dev_ed_cubes_num_background[0];

				int i = (vt[0] - ed_cubes_offset_background.x) / ed_cube_res;
				int j = (vt[1] - ed_cubes_offset_background.y) / ed_cube_res;
				int k = (vt[2] - ed_cubes_offset_background.z) / ed_cube_res;
				if (i >= 0 && i < ed_cubes_num_background.x &&
					j >= 0 && j < ed_cubes_num_background.y &&
					k >= 0 && k < ed_cubes_num_background.z)
				{
					int cubeId = k*(ed_cubes_num_background.x*ed_cubes_num_background.y) + j*ed_cubes_num_background.x + i;
					if (dev_pt_counts_per_cube_background[cubeId] > thres_pt_count) return;
				}
			}

			float3 const ed_cubes_offset = dev_ed_cubes_offset[0];
			int3 const ed_cubes_num = dev_ed_cubes_num[0];

			int i = (vt[0] - ed_cubes_offset.x) / ed_cube_res;
			int j = (vt[1] - ed_cubes_offset.y) / ed_cube_res;
			int k = (vt[2] - ed_cubes_offset.z) / ed_cube_res;
			if (i >= 0 && i < ed_cubes_num.x &&
				j >= 0 && j < ed_cubes_num.y &&
				k >= 0 && k < ed_cubes_num.z)
			{
				int cubeId = k*(ed_cubes_num.x*ed_cubes_num.y) + j*ed_cubes_num.x + i;
				atomicAdd(&(dev_pt_counts_per_cube[cubeId]), 1);
				atomicAdd(&(dev_pt_centroids_per_cube[cubeId].x), vt[0]);
				atomicAdd(&(dev_pt_centroids_per_cube[cubeId].y), vt[1]);
				atomicAdd(&(dev_pt_centroids_per_cube[cubeId].z), vt[2]);
				atomicAdd(&(dev_pt_norms_per_cube[cubeId].x), n[0]);
				atomicAdd(&(dev_pt_norms_per_cube[cubeId].y), n[1]);
				atomicAdd(&(dev_pt_norms_per_cube[cubeId].z), n[2]);
			}
		}
	}	
}

//multiple cubes per thread
__global__
void EDNodesSamplerKernel_vGMem_stepSaving(DeformGraphNodeCuda* __restrict__ dev_ed_nodes_buf, int ed_buf_size,
											int* __restrict__ dev_global_ed_nodes_num,
											int3  const* __restrict__ dev_ed_cubes_num,
											int const*  __restrict__ dev_pt_counts_per_cube,
											float3 const* __restrict__ dev_pt_centroids_per_cube,
											float3 const* __restrict__ dev_pt_norms_per_cube,
											int thres_pt_count)
{
	__shared__ int sh_OccuCubes_count;
	if (threadIdx.x == 0)
		sh_OccuCubes_count = 0;
	__syncthreads();

	const int3 ed_cubes_num = *dev_ed_cubes_num;

	int cubes_num = ed_cubes_num.x*ed_cubes_num.y*ed_cubes_num.z;
	int threads_num = gridDim.x * blockDim.x;
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	
	int lc_OccuCubes_count = 0;
	for (int i = id; i < cubes_num; i += threads_num)
	{
		int count = dev_pt_counts_per_cube[i];
		if (count > thres_pt_count)
		{
			lc_OccuCubes_count++;
		}
	}
	if (lc_OccuCubes_count > 0)
		lc_OccuCubes_count = atomicAdd(&sh_OccuCubes_count, lc_OccuCubes_count);
	__syncthreads();

	if (sh_OccuCubes_count > 0)
	{
		if (threadIdx.x == 0)
			sh_OccuCubes_count = atomicAdd(dev_global_ed_nodes_num, sh_OccuCubes_count);
		__syncthreads();

		int ndIdx = sh_OccuCubes_count + lc_OccuCubes_count;
		for (int i = id; i < cubes_num; i += threads_num)
		{
			int cubeId = i;
			int zCubeId = cubeId / (ed_cubes_num.x*ed_cubes_num.y);
			cubeId -= zCubeId*ed_cubes_num.x*ed_cubes_num.y;
			int yCubeId = cubeId / ed_cubes_num.x;
			int xCubeId = cubeId - yCubeId*ed_cubes_num.x;

			int count = dev_pt_counts_per_cube[i];
			if (count > thres_pt_count)
			{
				surf3Dwrite((short)ndIdx, surf_ndIds, xCubeId*sizeof(short), yCubeId, zCubeId, cudaBoundaryModeClamp);

				float3 pt_centroid = dev_pt_centroids_per_cube[i];
				pt_centroid.x /= count;
				pt_centroid.y /= count;
				pt_centroid.z /= count;
				float3 pt_normal = dev_pt_norms_per_cube[i];
				normalize(pt_normal);

				if (ndIdx < ed_buf_size)
				{
					dev_ed_nodes_buf[ndIdx].g[0] = pt_centroid.x;
					dev_ed_nodes_buf[ndIdx].g[1] = pt_centroid.y;
					dev_ed_nodes_buf[ndIdx].g[2] = pt_centroid.z;

					dev_ed_nodes_buf[ndIdx].n[0] = pt_normal.x;
					dev_ed_nodes_buf[ndIdx].n[1] = pt_normal.y;
					dev_ed_nodes_buf[ndIdx].n[2] = pt_normal.z;
				}
				ndIdx++;
			}
			else
			{
				surf3Dwrite((short)(-1), surf_ndIds, xCubeId*sizeof(short), yCubeId, zCubeId, cudaBoundaryModeClamp);
			}
		}
	}
	else
	{
		for (int i = id; i < cubes_num; i += threads_num)
		{
			int cubeId = i;
			int zCubeId = cubeId / (ed_cubes_num.x*ed_cubes_num.y);
			cubeId -= zCubeId*ed_cubes_num.x*ed_cubes_num.y;
			int yCubeId = cubeId / ed_cubes_num.x;
			int xCubeId = cubeId - yCubeId*ed_cubes_num.x;

			surf3Dwrite((short)(-1), surf_ndIds, xCubeId*sizeof(short), yCubeId, zCubeId, cudaBoundaryModeClamp);
		}
	}
	
}

__global__
void setup_ed_cubes_dim_kernel(float3* dev_ed_cubes_offset, int3* dev_ed_cubes_dims, BoundingBox3DCuda const* dev_bbox, int ed_cube_res)
{
	int3 cubes_dims;
	float3 cubes_offset;
	BoundingBox3DCuda bbox = dev_bbox[0];
	cubes_dims.x = MIN(ED_CUBE_DIM_MAX, int((bbox.x_e - bbox.x_s) / ed_cube_res) + 1);
	cubes_dims.y = MIN(ED_CUBE_DIM_MAX, int((bbox.y_e - bbox.y_s) / ed_cube_res) + 1);
	cubes_dims.z = MIN(ED_CUBE_DIM_MAX, int((bbox.z_e - bbox.z_s) / ed_cube_res) + 1);
	cubes_offset.x = bbox.x_s;
	cubes_offset.y = bbox.y_s;
	cubes_offset.z = bbox.z_s;

	dev_ed_cubes_dims[0] = cubes_dims;
	dev_ed_cubes_offset[0] = cubes_offset;
}


__global__
void copy_ed_static_counts_volume_data_gpu_kernel(int* __restrict__ dev_pt_counts_per_cube_background, int* __restrict__ dev_pt_counts_per_cube,
int3 const* __restrict__ dev_ed_cubes_num
)
{
	int3 const cubes_num = dev_ed_cubes_num[0];
	int const cubes_count = cubes_num.x*cubes_num.y*cubes_num.z;
	int cubeId = threadIdx.x + blockIdx.x * blockDim.x;
	if (cubeId >= 0 && cubeId < cubes_count)
	{
		int ptCount = dev_pt_counts_per_cube[cubeId];
		if (ptCount < 5) return;
		int cubeIdx = cubeId % cubes_num.x;
		int yz = cubeId / cubes_num.x;
		int cubeIdy = yz % cubes_num.y;
		int cubeIdz = yz / cubes_num.y;
		for (int xx = -1; xx < 1; xx++)
		{
			if (xx + cubeIdx < 0) continue;
			if (xx + cubeIdx >= cubes_num.x) continue;
			for (int yy = -1; yy < 1; yy++)
			{
				if (yy + cubeIdy < 0) continue;
				if (yy + cubeIdy >= cubes_num.y) continue;
				for (int zz = -1; zz < 1; zz++)
				{
					if (zz + cubeIdz < 0) continue;
					if (zz + cubeIdz >= cubes_num.z) continue;
					dev_pt_counts_per_cube_background[cubeId + xx + yy*cubes_num.x + zz*cubes_num.x*cubes_num.y] = ptCount;
				}
			}
		}
	}
}

bool DeformGraphCudaImpl::
sample_ed_nodes_impl_vGMem(BoundingBox3DCuda const*dev_bbox,
							float min_nodes_dist, int thres_pt_count, bool initBackground, bool bSwitch_ed_nodes_buf)
{
	if (bSwitch_ed_nodes_buf) {
		this->switch_ed_nodes_buf();
	}

	ed_cubes_res_ = min_nodes_dist;

	checkCudaErrors(cudaMemsetAsync(dev_pt_counts_per_cube_, 0, sizeof(int)*ED_CUBE_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_pt_centroids_per_cube_, 0, sizeof(float3)*ED_CUBE_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_pt_norms_per_cube_, 0, sizeof(float3)*ED_CUBE_NUM_MAX));

	setup_ed_cubes_dim_kernel<<<1, 1>>>(dev_ed_cubes_offset_, dev_ed_cubes_dims_, dev_bbox, ed_cubes_res_);
	checkCudaErrors(cudaMemsetAsync(nodes_num_gpu_.dev_ptr, 0, sizeof(int)));
	m_checkCudaErrors();

	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (vts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	if (_backgroundSet)
	{
		EDNodesSamplerKernel_vGMem_stepVoting << <blocks_per_grid, threads_per_block >> >(dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_,
			dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
			dev_ed_cubes_offset_, dev_ed_cubes_dims_, 
			dev_pt_counts_per_cube_background_, dev_ed_cubes_offset_background_, dev_ed_cubes_dims_background_, thres_pt_count,
			ed_cubes_res_);
	}
	else
	{
		EDNodesSamplerKernel_vGMem_stepVoting << <blocks_per_grid, threads_per_block >> >(dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_,
			dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
			dev_ed_cubes_offset_, dev_ed_cubes_dims_, 
			NULL, NULL, NULL, thres_pt_count, 
			ed_cubes_res_);
	}
	m_checkCudaErrors();

	if (initBackground)
	{
		checkCudaErrors(cudaMemcpyAsync(dev_pt_counts_per_cube_background_, dev_pt_counts_per_cube_, sizeof(int)*ED_CUBE_NUM_MAX, cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_offset_background_, dev_ed_cubes_offset_, sizeof(int3), cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_dims_background_, dev_ed_cubes_dims_, sizeof(float3), cudaMemcpyDeviceToDevice));
		threads_per_block = 64;
		blocks_per_grid = (ED_CUBE_NUM_MAX + threads_per_block - 1) / threads_per_block;
		copy_ed_static_counts_volume_data_gpu_kernel << <blocks_per_grid, threads_per_block >> >(dev_pt_counts_per_cube_background_, dev_pt_counts_per_cube_, dev_ed_cubes_dims_background_);
		m_checkCudaErrors();
		_backgroundSet = true;
	}

	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ED_CUBE_NUM_MAX + threads_per_block - 1) / threads_per_block;
	blocks_per_grid = MIN(blocks_per_grid, 100);
	EDNodesSamplerKernel_vGMem_stepSaving<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, nodes_num_gpu_.max_size, nodes_num_gpu_.dev_ptr, 
																				dev_ed_cubes_dims_,
																				dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_, 
																				thres_pt_count);
	m_checkCudaErrors();

	int nodes_num = nodes_num_gpu_.sync_read();

	if (nodes_num > ED_NODES_NUM_MAX)
		LOGGER()->warning("DeformGraphCudaImpl::sample_ed_nodes","nodes number<%d> beyond buffer size<%d>!\n", nodes_num, ED_NODES_NUM_MAX);
	nodes_num_gpu_.cap_gpu_size(ED_NODES_NUM_MAX);

	checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));

	return nodes_num <= ED_NODES_NUM_MAX;
}


bool DeformGraphCudaImpl::sample_ed_nodes_impl_vGMem(BoundingBox3DCuda bbox, float min_nodes_dist, int thres_pt_count, bool initBackground, bool bSwitch_ed_nodes_buf)
{
	if (bSwitch_ed_nodes_buf) {
		this->switch_ed_nodes_buf();
	}

	ed_cubes_res_ = min_nodes_dist;

	checkCudaErrors(cudaMemsetAsync(dev_pt_counts_per_cube_, 0, sizeof(int)*ED_CUBE_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_pt_centroids_per_cube_, 0, sizeof(float3)*ED_CUBE_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_pt_norms_per_cube_, 0, sizeof(float3)*ED_CUBE_NUM_MAX));

	//set ed cube info in GPU
	static cuda::PinnedMemory<float3> ed_cubes_offset;
	ed_cubes_offset.memory->x = bbox.x_s;
	ed_cubes_offset.memory->y = bbox.y_s;
	ed_cubes_offset.memory->z = bbox.z_s;
	static cuda::PinnedMemory<int3> ed_cubes_num;
	ed_cubes_num.memory->x = int((bbox.x_e - bbox.x_s) / min_nodes_dist) + 1;
	ed_cubes_num.memory->y = int((bbox.y_e - bbox.y_s) / min_nodes_dist) + 1;
	ed_cubes_num.memory->z = int((bbox.z_e - bbox.z_s) / min_nodes_dist) + 1;
	if (ed_cubes_num.memory->x > ED_CUBE_DIM_MAX ||
		ed_cubes_num.memory->y > ED_CUBE_DIM_MAX || 
		ed_cubes_num.memory->z > ED_CUBE_DIM_MAX )
	{
		LOGGER()->warning("DeformGraphCudaImpl::sample_ed_nodes_impl_vGMem", "!!!!!!!!!!!!Warning: ED Cube beyond maximum number");
		ed_cubes_num.memory->x = ED_CUBE_DIM_MAX;
		ed_cubes_num.memory->y = ED_CUBE_DIM_MAX;
		ed_cubes_num.memory->z = ED_CUBE_DIM_MAX;
	}
	checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_offset_, ed_cubes_offset.memory, sizeof(float3), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_dims_, ed_cubes_num.memory, sizeof(int3), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemsetAsync(nodes_num_gpu_.dev_ptr, 0, sizeof(int)));


	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (vts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	if (_backgroundSet)
	{
		EDNodesSamplerKernel_vGMem_stepVoting << <blocks_per_grid, threads_per_block >> >(dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_,
			dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
			dev_ed_cubes_offset_, dev_ed_cubes_dims_, 
			dev_pt_counts_per_cube_background_, dev_ed_cubes_offset_background_, dev_ed_cubes_dims_background_, thres_pt_count,
			ed_cubes_res_);
	}
	else
	{
		EDNodesSamplerKernel_vGMem_stepVoting << <blocks_per_grid, threads_per_block >> >(dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_,
			dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
			dev_ed_cubes_offset_, dev_ed_cubes_dims_, 
			NULL, NULL, NULL, thres_pt_count,
			ed_cubes_res_);
	}
	m_checkCudaErrors();
	if (initBackground)
	{
		checkCudaErrors(cudaMemcpyAsync(dev_pt_counts_per_cube_background_, dev_pt_counts_per_cube_, sizeof(int)*ED_CUBE_NUM_MAX, cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_offset_background_, dev_ed_cubes_offset_, sizeof(int3), cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_dims_background_, dev_ed_cubes_dims_, sizeof(float3), cudaMemcpyDeviceToDevice));
		_backgroundSet = true;
	}

	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (ED_CUBE_NUM_MAX + threads_per_block - 1) / threads_per_block;
	blocks_per_grid = MIN(blocks_per_grid, 100);
	EDNodesSamplerKernel_vGMem_stepSaving<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, nodes_num_gpu_.max_size, nodes_num_gpu_.dev_ptr, 
																				dev_ed_cubes_dims_,
																				dev_pt_counts_per_cube_, dev_pt_centroids_per_cube_, dev_pt_norms_per_cube_, 
																				thres_pt_count);
	m_checkCudaErrors();

	int nodes_num = nodes_num_gpu_.sync_read();

	if (nodes_num > ED_NODES_NUM_MAX)
		LOGGER()->warning("DeformGraphCudaImpl::sample_ed_nodes","nodes number<%d> beyond buffer size<%d>!", nodes_num, ED_NODES_NUM_MAX);
	nodes_num_gpu_.cap_gpu_size(ED_NODES_NUM_MAX);

	checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));

	return nodes_num <= ED_NODES_NUM_MAX;
}

#define ED_CUBES_PER_BLOCK 1024
#define EDSAMPLER_THREADS_PER_BLOCK MAX_THREADS_PER_BLOCK
#define CUBES_PER_THREAD ((ED_CUBES_PER_BLOCK + EDSAMPLER_THREADS_PER_BLOCK-1)/ EDSAMPLER_THREADS_PER_BLOCK)
__global__
void EDNodesSamplerKernel(DeformGraphNodeCuda* __restrict__ dev_ed_nodes_buf, int buf_size,
						  int* __restrict__ dev_global_ed_nodes_num,
						  float const* __restrict__ dev_vts, int const* __restrict__ dev_vts_num,  int vts_num_max, int stride,
						  float3 const* __restrict__ dev_ed_cubes_offset,
						  int3  const* __restrict__ dev_ed_cubes_num,
						  float ed_cube_res,
						  int thres_pt_count)
{
	int3 const ed_cubes_num = dev_ed_cubes_num[0];
	int ed_cubes_count = ed_cubes_num.x*ed_cubes_num.y*ed_cubes_num.z;
	if (blockIdx.x*ED_CUBES_PER_BLOCK > ed_cubes_count)
		return;

	__shared__ int pt_counts[ED_CUBES_PER_BLOCK];
	__shared__ float pt_centroids[ED_CUBES_PER_BLOCK * 3];
	__shared__ float pt_avg_normals[ED_CUBES_PER_BLOCK * 3];
	__shared__ int shPos;

	if (threadIdx.x == 0)
		shPos = 0;
	for (int i = threadIdx.x; i < ED_CUBES_PER_BLOCK; i += blockDim.x)
		pt_counts[i] = 0;
	for (int i = threadIdx.x; i < ED_CUBES_PER_BLOCK * 3; i += blockDim.x)
	{
		pt_centroids[i] = 0.0;
		pt_avg_normals[i] = 0.0;
	}
	__syncthreads();

	int const vts_num = MIN(vts_num_max, dev_vts_num[0]);
	float3 const ed_cubes_offset = dev_ed_cubes_offset[0];

	int lc_offsets;
	int chunkSize = (vts_num + blockDim.x - 1) / blockDim.x;
	for (int vtIdx = threadIdx.x*chunkSize, count = 0; vtIdx < vts_num && count < chunkSize; vtIdx++, count++)
	{
		float const*p_vt = dev_vts + vtIdx * stride;
		float vt[3];
		vt[0] = p_vt[0];
		vt[1] = p_vt[1];
		vt[2] = p_vt[2];
		float n[3];
		n[0] = p_vt[3];
		n[1] = p_vt[4];
		n[2] = p_vt[5];

		if (isnan(vt[0]))
			continue;

		int i = (vt[0] - ed_cubes_offset.x) / ed_cube_res;
		int j = (vt[1] - ed_cubes_offset.y) / ed_cube_res;
		int k = (vt[2] - ed_cubes_offset.z) / ed_cube_res;
		if (i >= 0 && i < ed_cubes_num.x &&
			j >= 0 && j < ed_cubes_num.y &&
			k >= 0 && k < ed_cubes_num.z)
		{
			int cubeId = k*(ed_cubes_num.x*ed_cubes_num.y) + j*ed_cubes_num.x + i;
			cubeId -= blockIdx.x * ED_CUBES_PER_BLOCK;

			if (cubeId >= 0 && cubeId < ED_CUBES_PER_BLOCK)
			{
				atomicAdd(&(pt_counts[cubeId]), 1);
				atomicAdd(&(pt_centroids[3 * cubeId]), vt[0]);
				atomicAdd(&(pt_centroids[3 * cubeId + 1]), vt[1]);
				atomicAdd(&(pt_centroids[3 * cubeId + 2]), vt[2]);

				atomicAdd(&(pt_avg_normals[3 * cubeId]), n[0]);
				atomicAdd(&(pt_avg_normals[3 * cubeId + 1]), n[1]);
				atomicAdd(&(pt_avg_normals[3 * cubeId + 2]), n[2]);
			}
		}
	}
	__syncthreads();

	int localPos[CUBES_PER_THREAD];
	for (int i = threadIdx.x, li = 0; i < ED_CUBES_PER_BLOCK; i += EDSAMPLER_THREADS_PER_BLOCK, li++)
	{
		if (pt_counts[i] > thres_pt_count)
		{
			pt_centroids[3 * i] /= pt_counts[i];
			pt_centroids[3 * i+1] /= pt_counts[i];
			pt_centroids[3 * i+2] /= pt_counts[i];

			normalize<3>(&(pt_avg_normals[3 * i]));

			localPos[li] = atomicAdd(&shPos, 1);
		}
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		shPos = atomicAdd(dev_global_ed_nodes_num, shPos);
	}
	__syncthreads();

	for (int i = threadIdx.x, li = 0; i < ED_CUBES_PER_BLOCK; i += EDSAMPLER_THREADS_PER_BLOCK, li++)
	{
		int cubeId = ED_CUBES_PER_BLOCK*blockIdx.x + i;
		if (cubeId < ed_cubes_num.x*ed_cubes_num.y*ed_cubes_num.z)
		{
			int zCubeId = cubeId / (ed_cubes_num.x*ed_cubes_num.y);
			cubeId -= zCubeId*ed_cubes_num.x*ed_cubes_num.y;
			int yCubeId = cubeId / ed_cubes_num.x;
			int xCubeId = cubeId - yCubeId*ed_cubes_num.x;

			if (pt_counts[i] > thres_pt_count)
			{
				int ndIdx = shPos + localPos[li];
				if (ndIdx < buf_size)
				{
					dev_ed_nodes_buf[ndIdx].g[0] = pt_centroids[3 * i];
					dev_ed_nodes_buf[ndIdx].g[1] = pt_centroids[3 * i + 1];
					dev_ed_nodes_buf[ndIdx].g[2] = pt_centroids[3 * i + 2];

					dev_ed_nodes_buf[ndIdx].n[0] = pt_avg_normals[3 * i];
					dev_ed_nodes_buf[ndIdx].n[1] = pt_avg_normals[3 * i + 1];
					dev_ed_nodes_buf[ndIdx].n[2] = pt_avg_normals[3 * i + 2];
				}
				surf3Dwrite((short)ndIdx, surf_ndIds, xCubeId*sizeof(short), yCubeId, zCubeId, cudaBoundaryModeClamp);
			}
			else
			{
				surf3Dwrite((short)-1, surf_ndIds, xCubeId*sizeof(short), yCubeId, zCubeId, cudaBoundaryModeClamp);
			}
		}
	}
}

void DeformGraphCudaImpl::
sample_ed_nodes_impl(BoundingBox3DCuda const*dev_bbox, 
				float min_nodes_dist, int thres_pt_count)
{
	this->switch_ed_nodes_buf();
	ed_cubes_res_ = min_nodes_dist;

	setup_ed_cubes_dim_kernel<<<1, 1>>>(dev_ed_cubes_offset_, dev_ed_cubes_dims_, dev_bbox, ed_cubes_res_);
	checkCudaErrors(cudaMemsetAsync(nodes_num_gpu_.dev_ptr, 0, sizeof(int)));
	m_checkCudaErrors();

	int threads_per_block = EDSAMPLER_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_CUBE_NUM_MAX + ED_CUBES_PER_BLOCK - 1) / ED_CUBES_PER_BLOCK;
	EDNodesSamplerKernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, 
																 nodes_num_gpu_.max_size, nodes_num_gpu_.dev_ptr, 
																 dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_, 
																 dev_ed_cubes_offset_, dev_ed_cubes_dims_, 
																 ed_cubes_res_, thres_pt_count);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
}

void DeformGraphCudaImpl::sample_ed_nodes_impl(BoundingBox3DCuda bbox, float min_nodes_dist, int thres_pt_count)
{
	this->switch_ed_nodes_buf();
	ed_cubes_res_ = min_nodes_dist;

	//set ed cube info in GPU
	static cuda::PinnedMemory<float3> ed_cubes_offset;
	ed_cubes_offset.memory->x = bbox.x_s;
	ed_cubes_offset.memory->y = bbox.y_s;
	ed_cubes_offset.memory->z = bbox.z_s;
	static cuda::PinnedMemory<int3> ed_cubes_num;
	ed_cubes_num.memory->x = int((bbox.x_e - bbox.x_s) / min_nodes_dist) + 1;
	ed_cubes_num.memory->y = int((bbox.y_e - bbox.y_s) / min_nodes_dist) + 1;
	ed_cubes_num.memory->z = int((bbox.z_e - bbox.z_s) / min_nodes_dist) + 1;
	checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_offset_, ed_cubes_offset.memory, sizeof(float3), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpyAsync(dev_ed_cubes_dims_, ed_cubes_num.memory, sizeof(int3), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemsetAsync(nodes_num_gpu_.dev_ptr, 0, sizeof(int)));

	int threads_per_block = EDSAMPLER_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_CUBE_NUM_MAX + ED_CUBES_PER_BLOCK - 1) / ED_CUBES_PER_BLOCK;
	EDNodesSamplerKernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, 
																 nodes_num_gpu_.max_size, nodes_num_gpu_.dev_ptr,
																 dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
																 dev_ed_cubes_offset_, dev_ed_cubes_dims_,
																 ed_cubes_res_, thres_pt_count);
	m_checkCudaErrors();

	int nodes_num = nodes_num_gpu_.sync_read();

	if (nodes_num > ED_NODES_NUM_MAX)
		LOGGER()->warning("DeformGraphCudaImpl::sample_ed_nodes","nodes number<%d> beyond buffer size<%d>!", nodes_num, ED_NODES_NUM_MAX);

	checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
}

//one cuda block generate one node at higher level
//one ed node (of original level) per threads
__global__
void build_hierachy_ed_nodes_kernel(DeformGraphNodeCuda *dev_ed_nodes, int down_sample_factor,
									  int* dev_ed_nodes_num_all_levels_,
									  int3 const* dev_ed_cubes_dims)
{
	int3 const ed_cubes_dims = dev_ed_cubes_dims[0];
	int3 ed_cubes_dims_ds; //after downsample
	ed_cubes_dims_ds.x = (ed_cubes_dims.x + down_sample_factor - 1) / down_sample_factor;
	ed_cubes_dims_ds.y = (ed_cubes_dims.y + down_sample_factor - 1) / down_sample_factor;
	ed_cubes_dims_ds.z = (ed_cubes_dims.z + down_sample_factor - 1) / down_sample_factor;

	if (blockIdx.x > ed_cubes_dims_ds.x*ed_cubes_dims_ds.y*ed_cubes_dims_ds.z)
		return;

	__shared__ cuda_vector_fixed<float, 3> sh_nodes_centroid;
	__shared__ cuda_vector_fixed<float, 3> sh_nodes_normal;
	__shared__ int sh_nodes_count;

	if (threadIdx.x == 0)
	{
		sh_nodes_count = 0;
		sh_nodes_centroid[0] = 0.0f;
		sh_nodes_centroid[1] = 0.0f;
		sh_nodes_centroid[2] = 0.0f;
		sh_nodes_normal[0] = 0.0f;
		sh_nodes_normal[1] = 0.0f;
		sh_nodes_normal[2] = 0.0f;
	}
	__syncthreads();

	int xCubeId_ds = blockIdx.x;
	int zCubeId_ds = xCubeId_ds / (ed_cubes_dims_ds.x*ed_cubes_dims_ds.y);
	xCubeId_ds -= zCubeId_ds*ed_cubes_dims_ds.x*ed_cubes_dims_ds.y;
	int yCubeId_ds = xCubeId_ds / ed_cubes_dims_ds.x;
	xCubeId_ds -= yCubeId_ds*ed_cubes_dims_ds.x;

	int cubes_num_to_avg = down_sample_factor*down_sample_factor*down_sample_factor;
	int local_nodes_count = 0;
	cuda_vector_fixed<float, 3> local_nodes_sum(0.0f);
	cuda_vector_fixed<float, 3> local_norms_sum(0.0f);
	for (int i = threadIdx.x; i < cubes_num_to_avg; i += blockDim.x)
	{
		int xId = i;
		int zId = xId / (down_sample_factor*down_sample_factor);
		xId -= zId *down_sample_factor*down_sample_factor;
		int yId = xId / down_sample_factor;
		xId -= yId * down_sample_factor;

		xId += xCubeId_ds*down_sample_factor;
		yId += yCubeId_ds*down_sample_factor;
		zId += zCubeId_ds*down_sample_factor;

		if (xId < ed_cubes_dims.x && yId < ed_cubes_dims.y && zId < ed_cubes_dims.z)
		{
			short ndId = tex3D(tex_ndIds, xId, yId, zId);
			if (ndId >= 0)
			{
				cuda_vector_fixed<float, 3> const g = dev_ed_nodes[ndId].g;
				local_nodes_count++;
				local_nodes_sum[0] += g[0];
				local_nodes_sum[1] += g[1];
				local_nodes_sum[2] += g[2];

				cuda_vector_fixed<float, 3> const n = dev_ed_nodes[ndId].n;
				local_norms_sum[0] += n[0];
				local_norms_sum[1] += n[1];
				local_norms_sum[2] += n[2];
			}
		}
	}
	if (local_nodes_count > 0)
	{
		atomicAdd(&sh_nodes_count, local_nodes_count);
		atomicAdd(&(sh_nodes_centroid[0]), local_nodes_sum[0]);
		atomicAdd(&(sh_nodes_centroid[1]), local_nodes_sum[1]);
		atomicAdd(&(sh_nodes_centroid[2]), local_nodes_sum[2]);

		atomicAdd(&(sh_nodes_normal[0]), local_norms_sum[0]);
		atomicAdd(&(sh_nodes_normal[1]), local_norms_sum[1]);
		atomicAdd(&(sh_nodes_normal[2]), local_norms_sum[2]);
	}
	__syncthreads();

	if (threadIdx.x == 0 && sh_nodes_count > 0)
	{
		int ed_node_pos = atomicAdd(dev_ed_nodes_num_all_levels_, 1);

		dev_ed_nodes[ed_node_pos].g = sh_nodes_centroid / (float)sh_nodes_count;
		dev_ed_nodes[ed_node_pos].n = sh_nodes_normal / (float)sh_nodes_count;
	}
}


__global__
void set_ed_nodes_range_kernel(int2* dev_ed_nodes_range, int const* dev_ed_nodes_num_all_levels_cur)
{
	if (threadIdx.x == 0 && blockIdx.x == 0)
	{
		const int2 ed_nodes_range = *dev_ed_nodes_range;
		const int ed_nodes_num_all_levels_cur = *dev_ed_nodes_num_all_levels_cur;
		dev_ed_nodes_range->y = ed_nodes_num_all_levels_cur - ed_nodes_range.x;
	}
}

//levels_num: overall levels include the lowest level
void DeformGraphCudaImpl::build_ed_nodes_hierarchy(int levels_num)
{
	checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemsetAsync(&(dev_ed_nodes_ranges_[0]->x), 0, sizeof(int)));
	checkCudaErrors(cudaMemcpyAsync(&(dev_ed_nodes_ranges_[0]->y), nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));

	if (levels_num > ED_HIER_LEVEL_NUM_MAX)
	{
		LOGGER()->warning("DeformGraphCudaImpl::build_ed_nodes_hierarchy", "!!!!!!!!!!Warning: levels_num > ED_HIER_LEVEL_NUM_MAX (%d v.s. %d)", levels_num, ED_HIER_LEVEL_NUM_MAX);
		levels_num = ED_HIER_LEVEL_NUM_MAX;
	}

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int2 ed_nodes_range;
		checkCudaErrors(cudaMemcpy(&ed_nodes_range, dev_ed_nodes_ranges_[0], sizeof(int2), cudaMemcpyDeviceToHost));
		LOGGER()->debug("<<<<<<<<<<<<<<<<<<<< ED Level 0: %d, %d", ed_nodes_range.x, ed_nodes_range.y);
	}

	//sample first level
	int down_smaple_factor = 3;
	this->ed_hierachy_levels_ = levels_num;

	for (int i = 1; i < levels_num; i++)
	{
		checkCudaErrors(cudaMemcpyAsync(&(dev_ed_nodes_ranges_[i]->x), dev_ed_nodes_num_all_levels_, sizeof(int), cudaMemcpyDeviceToDevice));
		int threads_per_block = MIN(MAX_THREADS_PER_BLOCK, down_smaple_factor*down_smaple_factor*down_smaple_factor);
		int grid_dim = (ED_CUBE_DIM_MAX + down_smaple_factor - 1) / down_smaple_factor; //ed cube dim after downsample
		int blocks_per_grid = grid_dim*grid_dim*grid_dim;
		build_hierachy_ed_nodes_kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, down_smaple_factor,
			dev_ed_nodes_num_all_levels_, dev_ed_cubes_dims_);
		m_checkCudaErrors();

		set_ed_nodes_range_kernel<<<1, 1>>>(dev_ed_nodes_ranges_[i], dev_ed_nodes_num_all_levels_);
		m_checkCudaErrors();


		down_smaple_factor *= down_smaple_factor;

		if (LOGGER()->check_verbosity(Logger::Debug))
		{
			int2 ed_nodes_range;
			checkCudaErrors(cudaMemcpy(&ed_nodes_range, dev_ed_nodes_ranges_[i], sizeof(int2), cudaMemcpyDeviceToHost));
			LOGGER()->debug("<<<<<<<<<<<<<<<<<<<< ED Level %d: %d, %d", i,  ed_nodes_range.x, ed_nodes_range.y);
		}
	}
}


//one ele on A per threads
__global__
void InitEDNodesParasAsIdentity_Kernel(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num, RigidTransformCuda * dev_rigid_transf)
{
	int const ed_nodes_num = MIN(ED_NODES_NUM_MAX, *dev_ed_nodes_num);
	if (blockIdx.x*blockDim.x > ed_nodes_num * 9)
		return;

	if (blockIdx.x == 0)
	{
		if (threadIdx.x < 9)
		{
			float *p_rigid_transf_R = dev_rigid_transf[0].R.data_block();

			if (threadIdx.x == 0 || threadIdx.x==4 ||threadIdx.x==8)
				p_rigid_transf_R[threadIdx.x] = 1.0f;
			else
				p_rigid_transf_R[threadIdx.x] = 0.0f;
		}

		if (threadIdx.x < 3)
		{
			float *p_rigid_transf_T = dev_rigid_transf[0].T.data_block();
			p_rigid_transf_T[threadIdx.x] = 0.0f;
		}		
	}

	int idx = threadIdx.x + blockIdx.x*blockDim.x;
	int ndId = idx / 9;
	if (ndId < ed_nodes_num)
	{
		int eleId = idx % 9;
		int r = eleId / 3; //row Idx of A
		int c = eleId % 3; //col Idx of A

		float val = 0.0f;
		if (r == c)
			val = 1.0f;

		dev_ed_nodes[ndId].A(r, c) = val;
		dev_ed_nodes[ndId].A_inv_t(r, c) = val;
		if (eleId < 3)
			dev_ed_nodes[ndId].t[eleId] = 0.0f;
	}
}

//one ed nodes per thread
__global__
void backup_initial_ed_nodes_paras(DeformGraphNodeCoreCuda *dev_ed_nodes_backup, DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num)
{
	int const ed_nodes_num = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num[0]);
	if (blockIdx.x*blockDim.x > ed_nodes_num)
		return;

	int idx = threadIdx.x + blockIdx.x*blockDim.x;
	if (idx < ed_nodes_num)
	{
		dev_ed_nodes_backup[idx].A = dev_ed_nodes[idx].A;
		dev_ed_nodes_backup[idx].t = dev_ed_nodes[idx].t;
		dev_ed_nodes_backup[idx].g = dev_ed_nodes[idx].g;
	}
}

void DeformGraphCudaImpl::
set_ed_nodes_paras_as_identity()
{
	int threads_per_block = 64;
	int blocks_per_grid = (ED_NODES_NUM_MAX * 9 + threads_per_block - 1) / threads_per_block;
	InitEDNodesParasAsIdentity_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_, dev_rigid_transf_);
	m_checkCudaErrors();

	threads_per_block = 64;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	backup_initial_ed_nodes_paras<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_initial_, dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemcpyAsync(dev_rigid_transf_prev_, dev_rigid_transf_, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice));
}

__global__
void init_surf_ndIds_kernel(int nx, int ny, int nz)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < nx*ny*nz)
	{
		int xId = id;
		int zId = id / (nx*ny);
		xId -= zId*nx*ny;
		int yId = xId / nx;
		xId -= yId*nx;

		surf3Dwrite((short)0, surf_ndIds, xId*sizeof(short), yId, zId, cudaBoundaryModeClamp);
	}
}


//one ed node per thread
__global__
void init_ed_nodes_paras_with_old_kernel(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num, 										
										 DeformGraphNodeCuda const*dev_ed_nodes_old, int const* dev_ed_nodes_num_old,
										 int3 const*dev_ed_cubes_dim_old, float3 const* dev_ed_cubes_offset_old, float ed_cube_res_old,
										 float nodes_min_dist)
{
#define EDNODE_NN_INIT 4

	int const ed_nodes_num = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num[0]);
	if (blockIdx.x*blockDim.x > ed_nodes_num)
		return;


	int idx = threadIdx.x + blockIdx.x*blockDim.x;
	if (idx < ed_nodes_num)
	{
		int3 const ed_cubes_dim_old = dev_ed_cubes_dim_old[0];
		float3 const ed_cubes_offset_old = dev_ed_cubes_offset_old[0];

		cuda_vector_fixed<float, 3> g = dev_ed_nodes[idx].g;
		int xCubeId = (g[0] - ed_cubes_offset_old.x) / ed_cube_res_old;
		int yCubeId = (g[1] - ed_cubes_offset_old.y) / ed_cube_res_old;
		int zCubeId = (g[2] - ed_cubes_offset_old.z) / ed_cube_res_old;

		float dists_sq[EDNODE_NN_INIT];
		int ngn_idx[EDNODE_NN_INIT];
		for (int i = 0; i < EDNODE_NN_INIT; i++)
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
				xId < ed_cubes_dim_old.x && yId < ed_cubes_dim_old.y && zId < ed_cubes_dim_old.z)
			{
				short ndId2 = tex3D(tex_ndIds_old, xId, yId, zId);
				if (ndId2 >= 0)
				{
					cuda_vector_fixed<float, 3> g2 = dev_ed_nodes_old[ndId2].g;
					float dist_sq = dist_square<3>(g.data_block(), g2.data_block());

					if (dist_sq < dists_sq[0])
					{
						dists_sq[0] = dist_sq;
						ngn_idx[0] = ndId2;
					}

					for (int c = 1; c < EDNODE_NN_INIT; c++)
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

		float w_sum = 0;
		#pragma unroll
		for (int i = 0; i < EDNODE_NN_INIT; i++)
		{
			if (ngn_idx[i] != -1)
			{
				dists_sq[i] = expf(-dists_sq[i] / (ed_cube_res_old*ed_cube_res_old / 4.0f));
				w_sum += dists_sq[i];
			}
		}

		if (w_sum > 1.0e-6)
		{
			for (int i = 0; i < EDNODE_NN_INIT; i++)
				dists_sq[i] /= w_sum;

			cuda_matrix_fixed<float, 3, 3> A(0.0);
			#pragma unroll
			for (int i = 0; i < 3; i++)
			#pragma unroll
			for (int j = 0; j < 3; j++)
			{
				#pragma unroll
				for (int k = 0; k < EDNODE_NN_INIT; k++)
				{
					int ndId_n = ngn_idx[k];
					if (ndId_n >= 0)
						A(i, j) += dev_ed_nodes_old[ndId_n].A(i, j) * dists_sq[k];
				}
				dev_ed_nodes[idx].A(i, j) = A(i, j);
			}

			#pragma unroll
			for (int i = 0; i < 3; i++)
			{
				float val = 0.0;
				#pragma unroll
				for (int k = 0; k < EDNODE_NN_INIT; k++)
				{
					int ndId_n = ngn_idx[k];
					if (ndId_n >= 0)
						val += dev_ed_nodes_old[ndId_n].t[i] * dists_sq[k];
				}
				dev_ed_nodes[idx].t[i] = val;
			}

			//inverse and transpose: A ---> A^(-T)
			float det = cuda_det(A);
			det = 1.0f / (det + 1.0e-6f);
			dev_ed_nodes[idx].A_inv_t[0][0] = (A(1, 1)*A(2, 2) - A(1, 2)*A(2, 1))*det;
			dev_ed_nodes[idx].A_inv_t[1][0] = (A(2, 1)*A(0, 2) - A(2, 2)*A(0, 1))*det;
			dev_ed_nodes[idx].A_inv_t[2][0] = (A(0, 1)*A(1, 2) - A(0, 2)*A(1, 1))*det;
			dev_ed_nodes[idx].A_inv_t[0][1] = (A(1, 2)*A(2, 0) - A(1, 0)*A(2, 2))*det;
			dev_ed_nodes[idx].A_inv_t[1][1] = (A(0, 0)*A(2, 2) - A(0, 2)*A(2, 0))*det;
			dev_ed_nodes[idx].A_inv_t[2][1] = (A(1, 0)*A(0, 2) - A(1, 2)*A(0, 0))*det;
			dev_ed_nodes[idx].A_inv_t[0][2] = (A(1, 0)*A(2, 1) - A(1, 1)*A(2, 0))*det;
			dev_ed_nodes[idx].A_inv_t[1][2] = (A(0, 1)*A(2, 0) - A(0, 0)*A(2, 1))*det;
			dev_ed_nodes[idx].A_inv_t[2][2] = (A(0, 0)*A(1, 1) - A(0, 1)*A(1, 0))*det;
		}
		else
		{
			dev_ed_nodes[idx].A(0, 0) = 1.0f; dev_ed_nodes[idx].A(0, 1) = 0.0f; dev_ed_nodes[idx].A(0, 2) = 0.0f;
			dev_ed_nodes[idx].A(1, 0) = 0.0f; dev_ed_nodes[idx].A(1, 1) = 1.0f; dev_ed_nodes[idx].A(1, 2) = 0.0f;
			dev_ed_nodes[idx].A(2, 0) = 0.0f; dev_ed_nodes[idx].A(2, 1) = 0.0f; dev_ed_nodes[idx].A(2, 2) = 1.0f;
			dev_ed_nodes[idx].A_inv_t(0, 0) = 1.0f; dev_ed_nodes[idx].A_inv_t(0, 1) = 0.0f; dev_ed_nodes[idx].A_inv_t(0, 2) = 0.0f;
			dev_ed_nodes[idx].A_inv_t(1, 0) = 0.0f; dev_ed_nodes[idx].A_inv_t(1, 1) = 1.0f; dev_ed_nodes[idx].A_inv_t(1, 2) = 0.0f;
			dev_ed_nodes[idx].A_inv_t(2, 0) = 0.0f; dev_ed_nodes[idx].A_inv_t(2, 1) = 0.0f; dev_ed_nodes[idx].A_inv_t(2, 2) = 1.0f;
			dev_ed_nodes[idx].t[0] = 0.0f;
			dev_ed_nodes[idx].t[1] = 0.0f;
			dev_ed_nodes[idx].t[2] = 0.0f;
		}
	}
}

//one ed nodes of L1 per threads
//each node at L1 will write data to initialize nodes at L2
__global__
void init_ed_nodes_paras_hier_kernel_stepVoting(DeformGraphNodeCuda *dev_ed_nodes, int2 const* dev_ed_nodes_range_L1, 
												float *dev_tmp_paras, float *dev_tmp_weights, float nodes_min_dist_L1)
{
	int2 ed_nodes_range = *dev_ed_nodes_range_L1;

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < ed_nodes_range.y)
	{
		int ndIdx = ed_nodes_range.x + id;

		cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndIdx].g;
		cuda_vector_fixed<float, 3> t = dev_ed_nodes[ndIdx].t;
		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[ndIdx].A;

		for (int i = 0; i < EDNODE_NN; i++)
		{
			int ndIdx2 = dev_ed_nodes[ndIdx].neighbors[i];
			if (ndIdx2 != -1)
			{
				cuda_vector_fixed<float, 3> g2 = dev_ed_nodes[ndIdx2].g;
				float dist2 = dist_square<3>(g.data_block(), g2.data_block());
				float w = __expf(-dist2 / (nodes_min_dist_L1*nodes_min_dist_L1));
				atomicAdd(&(dev_tmp_weights[ndIdx2]), w);

				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					atomicAdd(&(dev_tmp_paras[12 * ndIdx2 + 3 * m + n]), A(m, n)*w);

				for (int m = 0; m < 3; m++)
					atomicAdd(&(dev_tmp_paras[12 * ndIdx2 + 9 + m]), t[m] * w);
			}
		}
	}
}

__global__
void init_ed_nodes_paras_hier_kernel_stepAvg(DeformGraphNodeCuda *dev_ed_nodes, float const* dev_tmp_paras, float const* dev_tmp_weights, 
											 int2 const* dev_ed_nodes_range_L2)
{
	int2 ed_nodes_range = *dev_ed_nodes_range_L2;
	
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < ed_nodes_range.y)
	{
		int ndIdx = ed_nodes_range.x + id;

		float w = dev_tmp_weights[ndIdx];

		for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
			dev_ed_nodes[ndIdx].A(m, n) = dev_tmp_paras[12*ndIdx+3*m+n] / w;

		for (int m = 0; m < 3; m++)
			dev_ed_nodes[ndIdx].t[m] = dev_tmp_paras[12 * ndIdx + 9 + m] / w;
	}
}


void DeformGraphCudaImpl::init_ed_nodes_paras_with_old()
{
	bind_tex_ndIds_old(this->cu_3dArr_ndIds_old_);

	//init the ed nodes of the lowest level
	int threads_per_block = 64;
	int blocks_per_grid = (nodes_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	init_ed_nodes_paras_with_old_kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, 
																				dev_ed_nodes_buf_old_, nodes_num_gpu_old_.dev_ptr,
																				dev_ed_cubes_dims_old_, dev_ed_cubes_offset_old_, ed_cubes_res_old_, 
																				ed_cubes_res_);
	m_checkCudaErrors();

	const int ed_hier_downsample_factor = 3;

	checkCudaErrors(cudaMemsetAsync(dev_tmp_weights_ed_init_, 0, sizeof(float)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_tmp_paras_ed_init_, 0, sizeof(float)*ED_NODES_NUM_MAX * 12));

	float nodes_min_dist_cur = ed_cubes_res_;
	for (int i = 0; i < ed_hierachy_levels_ - 1; i++)
	{
		int ed_nodes_num_max_L1 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i);
		int ed_nodes_num_max_L2 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i + 1);

		int threads_per_block = 64;
		int blocks_per_grid = (ed_nodes_num_max_L1 + threads_per_block - 1) / threads_per_block;
		init_ed_nodes_paras_hier_kernel_stepVoting<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_ed_nodes_ranges_[i], 
																dev_tmp_paras_ed_init_, dev_tmp_weights_ed_init_, nodes_min_dist_cur);
		m_checkCudaErrors();
		
		threads_per_block = 64;
		blocks_per_grid = (ed_nodes_num_max_L2 + threads_per_block - 1) / threads_per_block;
		init_ed_nodes_paras_hier_kernel_stepAvg<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_tmp_paras_ed_init_, dev_tmp_weights_ed_init_, 
																				dev_ed_nodes_ranges_[i + 1]);
		m_checkCudaErrors();

		nodes_min_dist_cur *= ed_hier_downsample_factor;
	}

	threads_per_block = 64;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	backup_initial_ed_nodes_paras<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_initial_, dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemcpyAsync(dev_rigid_transf_prev_, dev_rigid_transf_, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice));
}

void DeformGraphCudaImpl::init_ed_nodes_paras(EDNodesParasGPU ed_nodes_init)
{
	bind_tex_ndIds_old(ed_nodes_init.cu_3dArr_ndIds);

	//init the ed nodes of the lowest level
	int threads_per_block = 64;
	int blocks_per_grid = (nodes_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	init_ed_nodes_paras_with_old_kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, 
																ed_nodes_init.dev_ed_nodes, ed_nodes_init.ed_nodes_num_gpu.dev_ptr,
																ed_nodes_init.dev_ed_cubes_dim, ed_nodes_init.dev_ed_cubes_offset, ed_nodes_init.ed_cubes_res, 
																ed_cubes_res_);
	m_checkCudaErrors();

	const int ed_hier_downsample_factor = 3;

	checkCudaErrors(cudaMemsetAsync(dev_tmp_weights_ed_init_, 0, sizeof(float)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMemsetAsync(dev_tmp_paras_ed_init_, 0, sizeof(float)*ED_NODES_NUM_MAX * 12));

	float nodes_min_dist_cur = ed_cubes_res_;
	for (int i = 0; i < ed_hierachy_levels_ - 1; i++)
	{
		int ed_nodes_num_max_L1 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i);
		int ed_nodes_num_max_L2 = ED_NODES_NUM_MAX / pow(ed_hier_downsample_factor, i + 1);

		int threads_per_block = 64;
		int blocks_per_grid = (ed_nodes_num_max_L1 + threads_per_block - 1) / threads_per_block;
		init_ed_nodes_paras_hier_kernel_stepVoting<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_ed_nodes_ranges_[i], 
																dev_tmp_paras_ed_init_, dev_tmp_weights_ed_init_, nodes_min_dist_cur);
		m_checkCudaErrors();
		
		threads_per_block = 64;
		blocks_per_grid = (ed_nodes_num_max_L2 + threads_per_block - 1) / threads_per_block;
		init_ed_nodes_paras_hier_kernel_stepAvg<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_tmp_paras_ed_init_, dev_tmp_weights_ed_init_, 
																				dev_ed_nodes_ranges_[i + 1]);
		m_checkCudaErrors();

		nodes_min_dist_cur *= ed_hier_downsample_factor;
	}

	threads_per_block = 64;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	backup_initial_ed_nodes_paras<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_initial_, dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemcpyAsync(dev_rigid_transf_, ed_nodes_init.dev_rigid_transf, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpyAsync(dev_rigid_transf_prev_, dev_rigid_transf_, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice));
}



//one ed nodes per thread
__global__
void UpdateEDParameter_Kernel(DeformGraphNodeCuda *dev_ed_nodes, int const *dev_ed_nodes_num,
							  int const* dev_ed_nodes_num_all_levels,
							  float const* dev_dx)
{
	int const ed_nodes_num_all_levels = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num_all_levels[0]);
	if (blockIdx.x*blockDim.x > ed_nodes_num_all_levels)
		return;

	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < ed_nodes_num_all_levels)
	{
		cuda_matrix_fixed<float, 3, 3> A;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			A[i][j] = dev_ed_nodes[id].A[i][j];
			A[i][j] += dev_dx[12 * id + 3 * i + j];
			dev_ed_nodes[id].A[i][j] = A[i][j];
		}
		for (int i = 0; i < 3; i++)
			dev_ed_nodes[id].t[i] += dev_dx[12 * id + 9 + i];

		const int ed_nodes_num = *dev_ed_nodes_num;

		if (id < ed_nodes_num)
		{
			//inverse and transpose: A ---> A^(-T)
			float det = cuda_det(A);
			det = 1.0f / (det + 1.0e-6f);
			dev_ed_nodes[id].A_inv_t[0][0] = (A(1, 1)*A(2, 2) - A(1, 2)*A(2, 1))*det;
			dev_ed_nodes[id].A_inv_t[1][0] = (A(2, 1)*A(0, 2) - A(2, 2)*A(0, 1))*det;
			dev_ed_nodes[id].A_inv_t[2][0] = (A(0, 1)*A(1, 2) - A(0, 2)*A(1, 1))*det;
			dev_ed_nodes[id].A_inv_t[0][1] = (A(1, 2)*A(2, 0) - A(1, 0)*A(2, 2))*det;
			dev_ed_nodes[id].A_inv_t[1][1] = (A(0, 0)*A(2, 2) - A(0, 2)*A(2, 0))*det;
			dev_ed_nodes[id].A_inv_t[2][1] = (A(1, 0)*A(0, 2) - A(1, 2)*A(0, 0))*det;
			dev_ed_nodes[id].A_inv_t[0][2] = (A(1, 0)*A(2, 1) - A(1, 1)*A(2, 0))*det;
			dev_ed_nodes[id].A_inv_t[1][2] = (A(0, 1)*A(2, 0) - A(0, 0)*A(2, 1))*det;
			dev_ed_nodes[id].A_inv_t[2][2] = (A(0, 0)*A(1, 1) - A(0, 1)*A(1, 0))*det;
		}
	}
}

bool DeformGraphCudaImpl::update_ed_paras(float const* dev_dx)
{
	int threads_per_block = 64;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	UpdateEDParameter_Kernel<<<threads_per_block, blocks_per_grid>>>(dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, dev_ed_nodes_num_all_levels_, dev_dx);
	m_checkCudaErrors();

	return true;
}

__global__
void CopyEDNodesParasToVector_Kernel(DeformGraphNodeCuda const*dev_ed_nodes, int const* dev_ed_nodes_num, float *dev_x)
{
	int const ed_nodes_num = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num[0]);
	if (blockIdx.x*blockDim.x > 21 * ed_nodes_num)
		return;
		
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < ed_nodes_num * 21)
	{
		int ndId = id / 21;
		int paraId = id % 21;

		float val;
		if (paraId < 9)
			val = dev_ed_nodes[ndId].A[paraId / 3][paraId % 3];
		else if (paraId<18)
			val = dev_ed_nodes[ndId].A_inv_t[(paraId-9) / 3][(paraId-9) % 3];
		else
			val = dev_ed_nodes[ndId].t[paraId - 18];
		dev_x[id] = val;
	}
}

bool DeformGraphCudaImpl::backup_current_ed_paras()
{
	int threads_per_block = 64;
	int blocks_per_grid = (ED_NODES_NUM_MAX*21 + threads_per_block - 1) / threads_per_block;
	CopyEDNodesParasToVector_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_, dev_ed_paras_backup_);
	m_checkCudaErrors();

	return true;
}

__global__
void CopyVectorToEDNodesParas_Kernel(float const*dev_x, DeformGraphNodeCuda *dev_ed_nodes, int* dev_ed_nodes_num)
{
	int const ed_nodes_num = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num[0]);
	if (blockIdx.x*blockDim.x > 21 * ed_nodes_num)
		return;

	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < ed_nodes_num * 21)
	{
		int ndId = id / 21;
		int paraId = id % 21;

		float val = dev_x[id];
		if (paraId < 9)
			dev_ed_nodes[ndId].A[paraId / 3][paraId % 3] = val;
		else if (paraId < 18)
			dev_ed_nodes[ndId].A[(paraId-9) / 3][(paraId-9) % 3] = val;
		else
			dev_ed_nodes[ndId].t[paraId - 18] = val;
	}
}

bool DeformGraphCudaImpl::recover_backupEDParas()
{
	int threads_per_block = 64;
	int blocks_per_grid = (ED_NODES_NUM_MAX *21 + threads_per_block - 1) / threads_per_block;
	CopyVectorToEDNodesParas_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_ed_paras_backup_, dev_ed_nodes_buf_, dev_ed_nodes_num_all_levels_);
	m_checkCudaErrors();

	return true;
}


//at most 8K ED nodes with 50% occupancy
//4K Ed nodes with 100% occupancy
__global__
void ComputeNgnKernal_vNdIdTex(float const* dev_vts, int const* dev_vts_num, int vts_num_max, int stride,
							   DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num, float sigma_vt_node_dist,
							   int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res,
							   int *dev_ngns_indices,
							   float *dev_ngns_weights)
{
	int vts_num = MIN(vts_num_max, dev_vts_num[0]);
	if (blockIdx.x * blockDim.x > vts_num)
		return;
	
	extern __shared__ float sh_nodes_pos[];

	const int ed_nodes_num = MIN(ED_NODES_NUM_MAX, dev_ed_nodes_num[0]);
	for (int idx = threadIdx.x; idx < ed_nodes_num * 3; idx += blockDim.x)
		sh_nodes_pos[idx] = dev_ed_nodes[idx / 3].g[idx % 3];
	__syncthreads();

	float3 ed_cubes_offsets = dev_ed_cubes_offsets[0];
	int3 ed_cubes_dims = dev_ed_cubes_dims[0];

	int vtIdx = threadIdx.x + blockIdx.x*blockDim.x;
	if (vtIdx < vts_num)
	{
		float dists_sq[NEIGHBOR_EDNODE_NUM];
		int ngn_idx[NEIGHBOR_EDNODE_NUM];
		#pragma unroll
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			dists_sq[i] = 1.0e+10f;
			ngn_idx[i] = -1;
		}

		float vt[3];
		vt[0] = dev_vts[vtIdx*stride];
		vt[1] = dev_vts[vtIdx*stride + 1];
		vt[2] = dev_vts[vtIdx*stride + 2];
		if (!isnan(vt[0]))
		{
			int xCubeId = (vt[0] - ed_cubes_offsets.x) / ed_cube_res;
			int yCubeId = (vt[1] - ed_cubes_offsets.y) / ed_cube_res;
			int zCubeId = (vt[2] - ed_cubes_offsets.z) / ed_cube_res;

			#pragma unroll
			for (int k = -2; k <= 2; k++)
			#pragma unroll
			for (int j = -2; j <= 2; j++)
			#pragma unroll
			for (int i = -2; i <= 2; i++)
			{
				int xId = xCubeId + i;
				int yId = yCubeId + j;
				int zId = zCubeId + k;

				if (xId >= 0 && yId >= 0 && zId >= 0 &&
					xId < ed_cubes_dims.x && yId < ed_cubes_dims.y && zId < ed_cubes_dims.z)
				{
					short ndId = tex3D(tex_ndIds, xId, yId, zId);
					if (ndId >= 0)
					{
						float const* g = sh_nodes_pos + 3 * ndId;
						float dist_sq = dist_square<3>(vt, g);

						
						if (dist_sq < 9.0f * sigma_vt_node_dist*sigma_vt_node_dist) //dist < 3.0*sigma_vt_node_dist
						{
							if (dist_sq < dists_sq[0])
							{
								dists_sq[0] = dist_sq;
								ngn_idx[0] = ndId;
							}

							#pragma unroll
							for (int c = 1; c < NEIGHBOR_EDNODE_NUM; c++)
							{
								if (dist_sq < dists_sq[c])
								{
									dists_sq[c - 1] = dists_sq[c];
									ngn_idx[c - 1] = ngn_idx[c];
									dists_sq[c] = dist_sq;
									ngn_idx[c] = ndId;
								}
							}
						}
					}
				}
			}
		}

		float w_sum = 0;
		#pragma unroll
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			if (ngn_idx[i] != -1)
			{
				dists_sq[i] = expf(-dists_sq[i] / (sigma_vt_node_dist*sigma_vt_node_dist*2.0f));
				w_sum += dists_sq[i];
			}
		}

		#pragma unroll
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			dev_ngns_weights[vtIdx * NEIGHBOR_EDNODE_NUM + i] = (ngn_idx[i] != -1 && w_sum > M_EPS) ? dists_sq[i] / w_sum : 0.0f;
			dev_ngns_indices[vtIdx * NEIGHBOR_EDNODE_NUM + i] = (w_sum > M_EPS) ? ngn_idx[i] : -1;
		}
	}
}

//at most 8K ED nodes with 50% occupancy
//4K Ed nodes with 100% occupancy
__global__ 
void ComputeNgnKernal( float const* dev_vts, int vts_num, int stride,
							 DeformGraphNodeCuda const*dev_ed_nodes, int ed_nodes_num, float sigma_vt_node_dist,
							 int *dev_ngns_indices,
							 float *dev_ngns_weights)
{
	extern __shared__ float node_pos[];

	for (int idx = threadIdx.x; idx < ed_nodes_num * 3; idx += blockDim.x)
		node_pos[idx] = dev_ed_nodes[idx / 3].g[idx % 3];
	__syncthreads();

	int vtIdx = threadIdx.x + blockIdx.x*blockDim.x;
	if (vtIdx < vts_num)
	{
		float vt[3];
		vt[0] = dev_vts[vtIdx*stride];
		vt[1] = dev_vts[vtIdx*stride+1];
		vt[2] = dev_vts[vtIdx*stride+2];

		float dist[NEIGHBOR_EDNODE_NUM];
		int ngn_idx[NEIGHBOR_EDNODE_NUM];

		//initial fill
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			dist[i] = dist_square<3>(vt, &(node_pos[3*i]));
			ngn_idx[i] = i;
		}

		//loop over all the nodes
		for (int ndIdx = NEIGHBOR_EDNODE_NUM; ndIdx < ed_nodes_num; ndIdx++)
		{
			float dist_n = dist_square<3>(vt, &(node_pos[3*ndIdx]));

			int id_max = 0;
			float dist_max = dist[0];
			for (int i = 1; i<NEIGHBOR_EDNODE_NUM; i++)
			{
				if (dist[i]>dist_max)
				{
					id_max = i;
					dist_max = dist[i];
				}
			}

			if (dist_n < dist_max)
			{
				for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
				{
					if (id_max == i)
					{
						dist[i] = dist_n;
						ngn_idx[i] = ndIdx;
					}
				}

			}
		}

		float w_sum = 0.0f;
		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			dist[i] = expf(-dist[i] / (sigma_vt_node_dist*sigma_vt_node_dist * 2.0f));
			w_sum += dist[i];
		}

		for (int i = 0; i < NEIGHBOR_EDNODE_NUM; i++)
		{
			dev_ngns_weights[vtIdx * NEIGHBOR_EDNODE_NUM + i] = (ngn_idx[i] != -1 && w_sum > M_EPS) ? dist[i] / w_sum : 0.0f;
			dev_ngns_indices[vtIdx * NEIGHBOR_EDNODE_NUM + i] = (w_sum > M_EPS) ? ngn_idx[i] : -1;
		}					
	}
}

bool DeformGraphCudaImpl::compute_ngns(float sigma_vt_node_dist)
{
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (vts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;

	ComputeNgnKernal_vNdIdTex<<<blocks_per_grid, threads_per_block, ED_NODES_NUM_MAX * 12>>>(dev_vts_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size, vt_dim_,
																	dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, sigma_vt_node_dist,
																	dev_ed_cubes_dims_, dev_ed_cubes_offset_, ed_cubes_res_,
																	dev_ngns_indices_, dev_ngns_weights_);
	m_checkCudaErrors();

	return true;
}

bool DeformGraphCudaImpl::compute_ngns(float const* dev_vts, int vt_dim, cuda::gpu_size_data vts_num_gpu,
									   int* dev_ngns_indices, float* dev_ngns_weights, float sigma_vt_node_dist)
{
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;

	ComputeNgnKernal_vNdIdTex<<<blocks_per_grid, threads_per_block, ED_NODES_NUM_MAX * 12>>>(dev_vts, vts_num_gpu.dev_ptr, vts_num_gpu.max_size, vt_dim,
					dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, sigma_vt_node_dist,
					dev_ed_cubes_dims_, dev_ed_cubes_offset_, ed_cubes_res_,
					dev_ngns_indices, dev_ngns_weights);
	m_checkCudaErrors();
	return true;
}


template<int vtDim_in, int vtDim_out>
__global__ 
void TransformSurfaceKernal(float const* __restrict__ dev_vts_in, float* __restrict__ dev_vts_out, int const* dev_vts_num, int vts_num_max,
							 int const* __restrict__ dev_ngns_indices, float const* __restrict__ dev_ngns_weights,
							 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int const* __restrict__ dev_ed_nodes_num,
							 RigidTransformCuda const* __restrict__ dev_rigid_transf)
{
	int vts_num = MIN(vts_num_max, dev_vts_num[0]);
	if (blockIdx.x*blockDim.x > vts_num)
		return;

	__shared__ RigidTransformCuda sh_rigid_transf;
	if (threadIdx.x == 0)
	{
		sh_rigid_transf = *dev_rigid_transf;
	}
	__syncthreads();

	int vtIdx = blockDim.x*blockIdx.x + threadIdx.x;
	if (vtIdx < vts_num)
	{
		float const*vt_ = dev_vts_in + vtIdx * vtDim_in;
		float const*n_ = vt_ + 3;
		cuda_vector_fixed<float, 3> vt(vt_);
		cuda_vector_fixed<float, 3> n(n_);

		int const*ngn_indices = dev_ngns_indices + NEIGHBOR_EDNODE_NUM*vtIdx;
		float const*ngn_weights = dev_ngns_weights + NEIGHBOR_EDNODE_NUM*vtIdx;

		cuda_vector_fixed<float, 3> vt_t(0.0);
		cuda_vector_fixed<float, 3> n_t(0.0);

		#pragma unroll
		for (int k = 0; k < NEIGHBOR_EDNODE_NUM; k++)
		{
			int ndIdx_k = ngn_indices[k];
			float w_k = ngn_weights[k];

			if (ndIdx_k >= 0)
			{
				DeformGraphNodeCuda const&nd = dev_ed_nodes[ndIdx_k];
				cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
				cuda_matrix_fixed<float, 3, 3> const&A_inv_t = nd.A_inv_t;
				cuda_vector_fixed<float, 3> const&g = nd.g;
				cuda_vector_fixed<float, 3> const&t = nd.t;

				vt_t += w_k*(A*(vt - g) + g + t);
				n_t += w_k*(A_inv_t*n);
			}
		}

		vt_t = sh_rigid_transf.R*vt_t + sh_rigid_transf.T;
		n_t = sh_rigid_transf.R*n_t;
		n_t.normalize();

		dev_vts_out[vtDim_out * vtIdx] = vt_t[0];
		dev_vts_out[vtDim_out * vtIdx + 1] = vt_t[1];
		dev_vts_out[vtDim_out * vtIdx + 2] = vt_t[2];
		
		dev_vts_out[vtDim_out * vtIdx + 3] = n_t[0];
		dev_vts_out[vtDim_out * vtIdx + 4] = n_t[1];
		dev_vts_out[vtDim_out * vtIdx + 5] = n_t[2];
	}
}

bool DeformGraphCudaImpl::transform_surface()
{
	if (dev_vts_ == NULL || dev_vts_t_ == NULL)
	{
		LOGGER()->error("DeformGraphCuda::transform_surface","dev_vts_/dev_vts_t_ == NULL!");
		return false;
	}

	//call kernals
	int threads_per_block = 64;
	int blocks_per_grid = (vts_num_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	if (vt_dim_ == 6)
		TransformSurfaceKernal<6, 6><<<blocks_per_grid, threads_per_block >>>(dev_vts_, dev_vts_t_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size,
						dev_ngns_indices_, dev_ngns_weights_, 
						dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, dev_rigid_transf_);
	else if (vt_dim_ == 9)
		TransformSurfaceKernal<9, 9><<<blocks_per_grid, threads_per_block >>>(dev_vts_, dev_vts_t_, vts_num_gpu_.dev_ptr, vts_num_gpu_.max_size,
						dev_ngns_indices_, dev_ngns_weights_, 
						dev_ed_nodes_buf_, nodes_num_gpu_.dev_ptr, dev_rigid_transf_);
	else
	{
		LOGGER()->error("DeformGraphCuda::transform_surface>","vt_dim != 6 or 9");
		return false;
	}
	m_checkCudaErrors()


	return true;
}


void DeformGraphCudaImpl::bind_tex_ndIds_old(cudaArray *cu_3dArr_ndIds)
{
	if (cu_3dArr_ndIds)
	{
		cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
		// set texture parameters
		tex_ndIds.addressMode[0] = cudaAddressModeClamp;
		tex_ndIds.addressMode[1] = cudaAddressModeClamp;
		tex_ndIds.addressMode[2] = cudaAddressModeClamp;
		tex_ndIds.filterMode = cudaFilterModePoint;
		tex_ndIds.normalized = false;  // access with un-normalized texture coordinates
		// Bind the array to the texture
		checkCudaErrors(cudaBindTextureToArray(tex_ndIds_old, cu_3dArr_ndIds, channelDesc));
	}
}
void DeformGraphCudaImpl::bind_tex_ndIds(cudaArray *cu_3dArr_ndIds)
{
	if (cu_3dArr_ndIds)
	{
		cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
		// set texture parameters
		tex_ndIds.addressMode[0] = cudaAddressModeClamp;
		tex_ndIds.addressMode[1] = cudaAddressModeClamp;
		tex_ndIds.addressMode[2] = cudaAddressModeClamp;
		tex_ndIds.filterMode = cudaFilterModePoint;
		tex_ndIds.normalized = false;  // access with un-normalized texture coordinates
		// Bind the array to the texture
		checkCudaErrors(cudaBindTextureToArray(tex_ndIds, cu_3dArr_ndIds, channelDesc));
	}
}

bool DeformGraphCudaImpl::allocate_and_bind_ndIds_cu3dArray(int nx, int ny, int nz)
{
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_ndIds_, &channelDesc, make_cudaExtent(nx, ny, nz), cudaArraySurfaceLoadStore));
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_ndIds_old_, &channelDesc, make_cudaExtent(nx, ny, nz), cudaArraySurfaceLoadStore));

	//bind cu_3dArr_ndIds_to both texture and surface 
	checkCudaErrors(cudaBindSurfaceToArray(surf_ndIds, cu_3dArr_ndIds_, channelDesc));
	// set texture parameters
	tex_ndIds.addressMode[0] = cudaAddressModeClamp;
	tex_ndIds.addressMode[1] = cudaAddressModeClamp;
	tex_ndIds.addressMode[2] = cudaAddressModeClamp;
	tex_ndIds.filterMode = cudaFilterModePoint;
	tex_ndIds.normalized = false;  // access with un-normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_ndIds, cu_3dArr_ndIds_, channelDesc));

	// set texture parameters
	tex_ndIds_old.addressMode[0] = cudaAddressModeClamp;
	tex_ndIds_old.addressMode[1] = cudaAddressModeClamp;
	tex_ndIds_old.addressMode[2] = cudaAddressModeClamp;
	tex_ndIds_old.filterMode = cudaFilterModePoint;
	tex_ndIds_old.normalized = false;  // access with un-normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_ndIds_old, cu_3dArr_ndIds_old_, channelDesc));

	int threads_per_block = 64;
	int blocks_per_grid = (nx*ny*nz + threads_per_block - 1) / threads_per_block;
	init_surf_ndIds_kernel<<<blocks_per_grid, threads_per_block>>>(nx, ny, nz);
	m_checkCudaErrors();

	return true;
}

void DeformGraphCudaImpl::allocate_ed_nodes()
{
	nodes_num_gpu_.max_size = ED_NODES_NUM_MAX;
	nodes_num_gpu_old_.max_size = ED_NODES_NUM_MAX;

	//for hierarchy
	checkCudaErrors(cudaMalloc(&(dev_ed_nodes_num_all_levels_), sizeof(int)));
	for (int i = 0; i < ED_HIER_LEVEL_NUM_MAX; i++)
		checkCudaErrors(cudaMalloc(&(dev_ed_nodes_ranges_[i]), sizeof(int2)));

	checkCudaErrors(cudaMalloc(&(nodes_num_gpu_.dev_ptr), sizeof(int)));
	checkCudaErrors(cudaMalloc(&(nodes_num_gpu_old_.dev_ptr), sizeof(int)));
	checkCudaErrors(cudaMemset(nodes_num_gpu_.dev_ptr, 0, sizeof(int)));
	checkCudaErrors(cudaMemset(nodes_num_gpu_old_.dev_ptr, 0, sizeof(int)));

	checkCudaErrors(cudaMalloc(&dev_ed_cubes_dims_, sizeof(int3)));
	checkCudaErrors(cudaMalloc(&dev_ed_cubes_offset_, sizeof(float3)));

	checkCudaErrors(cudaMalloc(&dev_ed_cubes_dims_background_, sizeof(int3)));
	checkCudaErrors(cudaMalloc(&dev_ed_cubes_offset_background_, sizeof(float3)));

	checkCudaErrors(cudaMalloc(&dev_ed_cubes_dims_old_, sizeof(int3)));
	checkCudaErrors(cudaMalloc(&dev_ed_cubes_offset_old_, sizeof(float3)));

	checkCudaErrors(cudaMalloc(&dev_ed_nodes_buf_, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMalloc(&dev_ed_nodes_buf_old_, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX));

	checkCudaErrors(cudaMalloc(&dev_ed_nodes_initial_, sizeof(DeformGraphNodeCoreCuda)*ED_NODES_NUM_MAX));

	checkCudaErrors(cudaMalloc(&dev_ed_paras_backup_, sizeof(float)*ED_NODES_NUM_MAX*21));

	//for ed nodes initialization
	checkCudaErrors(cudaMalloc(&dev_tmp_weights_ed_init_, sizeof(DeformGraphNodeCoreCuda)*ED_NODES_NUM_MAX));
	checkCudaErrors(cudaMalloc(&dev_tmp_paras_ed_init_, sizeof(DeformGraphNodeCoreCuda)*ED_NODES_NUM_MAX*12));
}


void DeformGraphCudaImpl::switch_ed_nodes_buf()
{
	//switch buffer
	DeformGraphNodeCuda *tmp = this->dev_ed_nodes_buf_;
	this->dev_ed_nodes_buf_ = this->dev_ed_nodes_buf_old_;
	this->dev_ed_nodes_buf_old_ = tmp;

	//switch array
	cudaArray* tmp_arr = this->cu_3dArr_ndIds_;
	this->cu_3dArr_ndIds_ = this->cu_3dArr_ndIds_old_;
	this->cu_3dArr_ndIds_old_ = tmp_arr;

	//switch ed nodes count
	cuda::gpu_size_data tmp_size = this->nodes_num_gpu_;
	this->nodes_num_gpu_ = this->nodes_num_gpu_old_;
	this->nodes_num_gpu_old_ = tmp_size;

	//switch cubes_offset, cubes_num
	float3 *tmp_offset = this->dev_ed_cubes_offset_;
	this->dev_ed_cubes_offset_ = this->dev_ed_cubes_offset_old_;
	this->dev_ed_cubes_offset_old_ = tmp_offset;
	int3 *tmp_cubes_dims = this->dev_ed_cubes_dims_;
	this->dev_ed_cubes_dims_ = this->dev_ed_cubes_dims_old_;
	this->dev_ed_cubes_dims_old_ = tmp_cubes_dims;

	this->ed_cubes_res_old_ = this->ed_cubes_res_;

	//bind the surface and texture
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindSigned);
	checkCudaErrors(cudaBindSurfaceToArray(surf_ndIds, cu_3dArr_ndIds_, channelDesc));
	checkCudaErrors(cudaBindTextureToArray(tex_ndIds, cu_3dArr_ndIds_, channelDesc));
	checkCudaErrors(cudaBindTextureToArray(tex_ndIds_old, cu_3dArr_ndIds_old_, channelDesc));
}

__global__ void copy_vts_t_kernel(float *dev_vts, int const* dev_vts_num, int vts_dim,
								  float *dev_vts_out, int out_buf_size, //size in vt num 
								  int vts_dim_out)
{
	int const vts_num = MIN(out_buf_size, dev_vts_num[0]);

	for (int idx = threadIdx.x + blockDim.x * blockIdx.x; idx < vts_num; idx += gridDim.x*blockDim.x)
	{
		if (idx < vts_num && idx < out_buf_size)
		{
			for (int i = 0; i < vts_dim; i++)
				dev_vts_out[idx*vts_dim_out + i] = dev_vts[idx*vts_dim + i];
		}
	}
}

void DeformGraphCudaImpl::copy_vts_t_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data vts_num_gpu_out)
{
	int threads_per_block = 64;
	int blocks_per_grid = 256;
	copy_vts_t_kernel<<<blocks_per_grid, threads_per_block>>>(dev_vts_t_, vts_num_gpu_.dev_ptr, vt_dim_, dev_buf_out, vts_num_gpu_out.max_size, stride_out);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_out.dev_ptr, vts_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
}
