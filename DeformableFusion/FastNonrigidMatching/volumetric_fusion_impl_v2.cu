#include "utility.h"

namespace VolumetricFusionCuda{

#define THREADS_PER_BLOCK_CUBEPRUNE 256
#define CUBEPRUNE_NDIMS 3

// set occupied cubes offset as 1, empty cubes as -1
__global__
void cubes_prune_with_depth_kernel( OccupcyCube* dev_cubes,
									float3 const*dev_ptr_cubes_offset, int3 const*dev_ptr_cubes_num, float cube_res,
									int width_depth, int height_depth, float mu)
{
	__shared__ float x_prjs[MAX_NUM_DEPTH_CAMERAS * CUBEPRUNE_NDIMS * THREADS_PER_BLOCK_CUBEPRUNE];
	int3 const cubes_num = dev_ptr_cubes_num[0];

	if (blockIdx.x > cubes_num.x / (blockDim.x - 1) ||
		blockIdx.y > cubes_num.y / (blockDim.y - 1) ||
		blockIdx.z > cubes_num.z / (blockDim.z - 1))
		return;

	int xId = threadIdx.x + (blockDim.x - 1)*blockIdx.x; //cube Ids
	int yId = threadIdx.y + (blockDim.y - 1)*blockIdx.y;
	int zId = threadIdx.z + (blockDim.z - 1)*blockIdx.z;

	int tid = threadIdx.z*blockDim.x*blockDim.y + threadIdx.y*blockDim.x + threadIdx.x;

	float3 const cubes_offset = dev_ptr_cubes_offset[0];

	int lc_offset = 0;
	if (xId <= cubes_num.x && 
		yId <= cubes_num.y && 
		zId <= cubes_num.z)
	{
		cuda_vector_fixed<float, 3> P_wld;
		P_wld[0] = cubes_offset.x + xId * cube_res;
		P_wld[1] = cubes_offset.y + yId * cube_res;
		P_wld[2] = cubes_offset.z + zId * cube_res;

		for (int i = 0; i < dev_num_cam_views; i++)
		{
			cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*P_wld + dev_cam_views[i].cam_pose.T;

			cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
			float const&fx = K[0][0];
			float const&fy = K[1][1];
			float const&cx = K[0][2];
			float const&cy = K[1][2];

			x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS] = fx * X[0] / X[2] + cx;
			x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 1] = fy * X[1] / X[2] + cy;
			x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 2] = X[2];
		}
	}
	__syncthreads();

	const int dx[] = { 0, 0, 0, 0, 1, 1, 1, 1 };
	const int dy[] = { 0, 0, 1, 1, 0, 0, 1, 1 };
	const int dz[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

	int bOccupied = 0;
	if (xId < cubes_num.x && yId < cubes_num.y && zId < cubes_num.z &&
		threadIdx.x < blockDim.x - 1 &&
		threadIdx.y < blockDim.y - 1 &&
		threadIdx.z < blockDim.z - 1)
	{
		for (int vId = 0; vId < dev_num_cam_views; vId++)
		{
			float u_max = 0.0f;
			float u_min = 1.0e+10f;
			float v_max = 0.0f;
			float v_min = 1.0e+10f;
			float w_max = 0.0f;
			float w_min = 1.0e+10f;
			for (int i = 0; i < 8; i++)
			{
				int idx = (threadIdx.z + dz[i])*blockDim.x*blockDim.y +
					(threadIdx.y + dy[i])*blockDim.x + threadIdx.x + dx[i];
				float const& u = x_prjs[dev_num_cam_views * 3 * idx + 3 * vId];
				float const& v = x_prjs[dev_num_cam_views * 3 * idx + 3 * vId + 1];
				float const& w = x_prjs[dev_num_cam_views * 3 * idx + 3 * vId + 2];

				if (u > u_max) u_max = u;
				if (u < u_min) u_min = u;
				if (v > v_max) v_max = v;
				if (v < v_min) v_min = v;
				if (w > w_max) w_max = w;
				if (w < w_min) w_min = w;
			}

			w_max *= 10.0f;
			w_min *= 10.0f; //cm to mm
			float r_h = u_max - u_min;
			float r_v = v_max - v_min;
			float level = MAX(r_h, r_v);
			level = MAX(2.0f, floorf(log2f(level)));
			r_h /= 2.0f;
			r_v /= 2.0f;
			ushort2 mipmap = pick_mipmap9((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level, width_depth, height_depth/*, bPrint*/);
			if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
			{
				ushort2 mipmap = pick_mipmap25((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level - 1.0f, width_depth, height_depth/*, bPrint*/);
				if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
				{
					bOccupied = 1;
				}
			}
		}
	}

	if (xId < cubes_num.x && yId < cubes_num.y && zId < cubes_num.z &&
		threadIdx.x < blockDim.x - 1 &&
		threadIdx.y < blockDim.y - 1 &&
		threadIdx.z < blockDim.z - 1)
	{
		int cubeId = zId * cubes_num.x*cubes_num.y + yId*cubes_num.x + xId;
		if (bOccupied)
			dev_cubes[cubeId].offset = 1;
		else
			dev_cubes[cubeId].offset = -1;
	}
}


//one vertex per thread
template<int dsStep>
__global__
void cubes_prune_with_vts_kernel( OccupcyCube* dev_cubes,
								  float const* dev_vts_t, int const* dev_vts_num, int vts_dim,
								  float3 const*dev_ptr_cubes_offset, int3 const*dev_ptr_cubes_num, float cube_res,
								  int width_depth, int height_depth, float mu)
{
	int const vts_num = *dev_vts_num;
	int vtIdx = (threadIdx.x + blockDim.x*blockIdx.x)*dsStep;
	if (vtIdx < vts_num)
	{
		int3 const cubes_num = dev_ptr_cubes_num[0];
		float3 const cubes_offset = dev_ptr_cubes_offset[0];

		float const* vt_ = dev_vts_t + vtIdx * vts_dim;
		float3 vt = make_float3(vt_[0], vt_[1], vt_[2]);

		int xCubeId0 = (vt.x - mu/2.0f - cubes_offset.x) / cube_res;
		int yCubeId0 = (vt.y - mu/2.0f - cubes_offset.y) / cube_res;
		int zCubeId0 = (vt.z - mu/2.0f - cubes_offset.z) / cube_res;
		int xCubeId1 = (vt.x + mu/2.0f - cubes_offset.x) / cube_res;
		int yCubeId1 = (vt.y + mu/2.0f - cubes_offset.y) / cube_res;
		int zCubeId1 = (vt.z + mu/2.0f - cubes_offset.z) / cube_res;
		for (int xCubeId = xCubeId0; xCubeId <= xCubeId1; xCubeId++)
		for (int yCubeId = yCubeId0; yCubeId <= yCubeId1; yCubeId++)
		for (int zCubeId = zCubeId0; zCubeId <= zCubeId1; zCubeId++)
		{
			if (0 <= xCubeId && xCubeId < cubes_num.x &&
				0 <= yCubeId && yCubeId < cubes_num.y &&
				0 <= zCubeId && zCubeId < cubes_num.z)
			{
				int cubeId = zCubeId * cubes_num.x*cubes_num.y + yCubeId*cubes_num.x + xCubeId;
				dev_cubes[cubeId].offset = 1;
			}
		}
	}
}

__global__
void cubes_prune_with_static_volume_kernel(OccupcyCube* __restrict__ dev_cubes, float3 const* __restrict__ dev_ptr_cubes_offset, int3 const* __restrict__ dev_ptr_cubes_num, float cube_res,
										OccupcyCube const* __restrict__ dev_cubes_static, float3 const* __restrict__ dev_ptr_cubes_static_offset, int3 const* __restrict__ dev_ptr_cubes_static_num
										)
{
	int3 const cubes_num = dev_ptr_cubes_num[0];
	float3 const cubes_offset = dev_ptr_cubes_offset[0];

	int3 const cubes_static_num = dev_ptr_cubes_static_num[0];
	float3 const cubes_static_offset = dev_ptr_cubes_static_offset[0];

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	if (idx < cubes_static_num.x*cubes_static_num.y*cubes_static_num.z)
	{
		int occupied = dev_cubes_static[idx].offset;
		if (occupied == 1)
		{
			int dx = (int)(((cubes_offset.x - cubes_static_offset.x) / cube_res) + 0.5f);
			int dy = (int)(((cubes_offset.y - cubes_static_offset.y) / cube_res) + 0.5f);
			int dz = (int)(((cubes_offset.z - cubes_static_offset.z) / cube_res) + 0.5f);

			int x = idx % cubes_static_num.x;
			int yz = idx / cubes_static_num.x;
			int y = yz % cubes_static_num.y;
			int z = yz / cubes_static_num.y;

			x -= dx;
			y -= dy;
			z -= dz;

			if (x >= 0 && x < cubes_num.x &&
				y >= 0 && y < cubes_num.y &&
				z >= 0 && z < cubes_num.z
				)
			{
				int cubeId = x + y*cubes_num.x + z * cubes_num.y*cubes_num.x;
				dev_cubes[cubeId].offset = -1;
			}
		}
	}
}

//NOTE: dev_global_count_occu_cubes should be set to 0 before the lunch of the kernel
//one cube per threads
//TODO: improve performance with prefix sum
__global__
void extract_occupied_cubes_and_set_cubes_offset_kernel( OccupcyCube* __restrict__ dev_cubes, 
														 int3 const* __restrict__ dev_ptr_cubes_num,
														 int* __restrict__ dev_buf_occupied_cube_ids,
														 int* __restrict__ dev_global_count_occu_cubes
														)
{
	__shared__ int sh_count;

	int3 const cubes_num = dev_ptr_cubes_num[0];
	int const cubes_count = cubes_num.x*cubes_num.y*cubes_num.z;

	if (blockIdx.x * blockDim.x > cubes_count)
		return;

	if (threadIdx.x == 0)
		sh_count = 0;
	__syncthreads();

	int cubeId = threadIdx.x + blockDim.x*blockIdx.x;
	int bOccupied = 0;
	int lc_count = 0;
	if (cubeId < cubes_count)
	{
		bOccupied = dev_cubes[cubeId].offset;
		if (bOccupied==1)
			lc_count = atomicAdd(&sh_count, 1);
	}
	__syncthreads();

	if (threadIdx.x == 0)
		sh_count = atomicAdd(dev_global_count_occu_cubes, sh_count);
	__syncthreads();

	if (bOccupied==1)
	{
		int lcIdx = lc_count + sh_count;//index in the list
		dev_cubes[cubeId].offset = lcIdx;
		if (lcIdx < TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX)
			dev_buf_occupied_cube_ids[lcIdx] = cubeId;
		else
			dev_cubes[cubeId].offset = -1;
	}
}

void VolumetricFusionHelperCudaImpl::
coarse_cube_prune_f2f(VolumeTwoLevelHierachy* volume, float const* dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vts_dim)
{
	dim3 blockDim = dim3(8, 8, 4);
	//each cube block processing 8x8x4 grids points (==7x7x3 cubes)
	dim3 gridDim = dim3((TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 6) / 7, (TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 6) / 7, (TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 2) / 3);
	cubes_prune_with_depth_kernel<<<gridDim, blockDim>>>(volume->cubes, volume->ptr_cubes_offset,
														 volume->ptr_cubes_num, volume->cube_res,
														 depth_width_, depth_height_, volume->mu);

	m_checkCudaErrors()

	if (dev_volume_static_->cube_res > 0.01f)
	{
		int threads_per_block = 64;
		int blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_NUM_MAX + threads_per_block - 1) / threads_per_block;
		cubes_prune_with_static_volume_kernel << <blocks_per_grid, threads_per_block >> >(volume->cubes,volume->ptr_cubes_offset, volume->ptr_cubes_num,volume->cube_res,
			dev_volume_static_->cubes, dev_volume_static_->ptr_cubes_offset, dev_volume_static_->ptr_cubes_num);
	}

	int threads_per_block = 256;
	int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	if (dev_vts_t != NULL && vts_num_gpu.dev_ptr != NULL)
	{
		cubes_prune_with_vts_kernel<1><<<blocks_per_grid, threads_per_block>>>(volume->cubes, dev_vts_t, vts_num_gpu.dev_ptr, vts_dim,
																		   volume->ptr_cubes_offset, volume->ptr_cubes_num,
																		   volume->cube_res,
																		   depth_width_, depth_height_, volume->mu);
	}
	m_checkCudaErrors()


	checkCudaErrors(cudaMemsetAsync(volume->gpu_cubes_occpied_count.dev_ptr, 0, sizeof(int)));
	threads_per_block = MAX_THREADS_PER_BLOCK;
	blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	extract_occupied_cubes_and_set_cubes_offset_kernel<<<blocks_per_grid, threads_per_block>>>(volume->cubes,
																							   volume->ptr_cubes_num,
																							   volume->buf_occupied_cube_ids,
																							   volume->gpu_cubes_occpied_count.dev_ptr);
	m_checkCudaErrors()

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		int cubes_occupied_num = volume->gpu_cubes_occpied_count.sync_read();
		LOGGER()->debug(">>>>>>>>>>>>>cubes occupied: %d", cubes_occupied_num);
	}

}

//one thread one block
__global__
void setup_volume_dim_kernel( BoundingBox3DCuda const* __restrict__ dev_bbox, 
                              float3 * __restrict__ dev_ptr_cubes_offset,
                              int3 * __restrict__ dev_ptr_cubes_num,
							  float cube_res,
							  float bbox_extension
							 )
{
	dev_ptr_cubes_num->x = ceilf((dev_bbox->x_e - dev_bbox->x_s + bbox_extension * 2.0f) / cube_res);
	dev_ptr_cubes_num->y = ceilf((dev_bbox->y_e - dev_bbox->y_s + bbox_extension * 2.0f) / cube_res);
	dev_ptr_cubes_num->z = ceilf((dev_bbox->z_e - dev_bbox->z_s + bbox_extension * 2.0f) / cube_res);

	dev_ptr_cubes_offset->x = dev_bbox->x_s - bbox_extension;
	dev_ptr_cubes_offset->y = dev_bbox->y_s - bbox_extension;
	dev_ptr_cubes_offset->z = dev_bbox->z_s - bbox_extension;
}

__global__
void setup_volume_dim_kernel2(BoundingBox3DCuda const* __restrict__ dev_bbox_cur,
							  BoundingBox3DCuda const* __restrict__ dev_bbox_ref, RigidTransformCuda const* __restrict__ dev_rigid_transf,
							  float3 * __restrict__ dev_ptr_cubes_offset,
							  int3 * __restrict__ dev_ptr_cubes_num,
							  float cube_res,
							  float bbox_extension)
{
	__shared__ cuda_vector_fixed<float, 3> sh_corners[8];

	float x_s;
	float x_e;
	float y_s;
	float y_e;
	float z_s;
	float z_e;
	const BoundingBox3DCuda bbox_cur = *dev_bbox_cur;
	if (bbox_cur.x_e - bbox_cur.x_s > 1.0f &&
		bbox_cur.y_e - bbox_cur.y_s > 1.0f &&
		bbox_cur.z_e - bbox_cur.z_s > 1.0f)
	{
		x_s = bbox_cur.x_s - bbox_extension;
		y_s = bbox_cur.y_s - bbox_extension;
		z_s = bbox_cur.z_s - bbox_extension;

		x_e = bbox_cur.x_e + bbox_extension;
		y_e = bbox_cur.y_e + bbox_extension;
		z_e = bbox_cur.z_e + bbox_extension;
	}
	else
	{
		const BoundingBox3DCuda bbox = *dev_bbox_ref;
		const RigidTransformCuda rigid_transf = *dev_rigid_transf;

		sh_corners[0][0] = bbox.x_s; sh_corners[1][0] = bbox.x_s; sh_corners[2][0] = bbox.x_s; sh_corners[3][0] = bbox.x_s;
		sh_corners[4][0] = bbox.x_e; sh_corners[5][0] = bbox.x_e; sh_corners[6][0] = bbox.x_e; sh_corners[7][0] = bbox.x_e;
		sh_corners[0][1] = bbox.y_s; sh_corners[1][1] = bbox.y_s; sh_corners[2][1] = bbox.y_e; sh_corners[3][1] = bbox.y_e;
		sh_corners[4][1] = bbox.y_s; sh_corners[5][1] = bbox.y_s; sh_corners[6][1] = bbox.y_e; sh_corners[7][1] = bbox.y_e;
		sh_corners[0][2] = bbox.z_s; sh_corners[1][2] = bbox.z_e; sh_corners[2][2] = bbox.z_s; sh_corners[3][2] = bbox.z_e;
		sh_corners[4][2] = bbox.z_s; sh_corners[5][2] = bbox.z_e; sh_corners[6][2] = bbox.z_s; sh_corners[7][2] = bbox.z_e;

		for (int i = 0; i < 8; i++)
			sh_corners[i] = rigid_transf.R * sh_corners[i] + rigid_transf.T;

		for (int j = 0; j < 3; j++)
		{
			for (int i = 0; i < 4; i++)
			{
				sh_corners[i][j] = MIN(sh_corners[i][j], sh_corners[i + 4][j]);
				sh_corners[i + 4][j] = MAX(sh_corners[i][j], sh_corners[i + 4][j]);
			}

			for (int i = 0; i < 2; i++)
			{
				sh_corners[i][j] = MIN(sh_corners[i][j], sh_corners[i + 2][j]);
				sh_corners[i + 4][j] = MAX(sh_corners[i + 4][j], sh_corners[i + 6][j]);
			}

			sh_corners[0][j] = MIN(sh_corners[0][j], sh_corners[1][j]);
			sh_corners[4][j] = MAX(sh_corners[4][j], sh_corners[5][j]);
		}

		x_s = sh_corners[0][0];
		x_e = sh_corners[4][0];
		y_s = sh_corners[0][1];
		y_e = sh_corners[4][1];
		z_s = sh_corners[0][2];
		z_e = sh_corners[4][2];
	}

	dev_ptr_cubes_num->x = ceilf((x_e - x_s) / cube_res);
	dev_ptr_cubes_num->y = ceilf((y_e - y_s) / cube_res);
	dev_ptr_cubes_num->z = ceilf((z_e - z_s) / cube_res);

	dev_ptr_cubes_offset->x = x_s;
	dev_ptr_cubes_offset->y = y_s;
	dev_ptr_cubes_offset->z = z_s;
}

void VolumetricFusionHelperCudaImpl::setup_volume_info(VolumeTwoLevelHierachy* volume, BoundingBox3DCuda const*dev_bbox, float vxl_res, float fusion_mu)
{
	volume->vxl_res = vxl_res;
	volume->cube_res = TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * vxl_res;
	volume->mu = fusion_mu;

	const float bbox_extension = 3.0f;
	setup_volume_dim_kernel<<<1, 1>>>(dev_bbox, volume->ptr_cubes_offset, volume->ptr_cubes_num, volume->cube_res, bbox_extension);
	m_checkCudaErrors();
	checkCudaErrors(cudaMemsetAsync(volume->gpu_cubes_occpied_count.dev_ptr, 0, sizeof(int)));
}

void VolumetricFusionHelperCudaImpl::setup_volume_info(VolumeTwoLevelHierachy* volume,
	BoundingBox3DCuda const*dev_bbox_cur,
	BoundingBox3DCuda const*dev_bbox_prev, RigidTransformCuda const* dev_rigid_transf,
	float vxl_res, float fusion_mu)
{
	volume->vxl_res = vxl_res;
	volume->cube_res = TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * vxl_res;
	volume->mu = fusion_mu;

	const float bbox_extension = 3.0f;
	setup_volume_dim_kernel2<<<1, 1>>>(dev_bbox_cur, dev_bbox_prev, dev_rigid_transf, volume->ptr_cubes_offset, volume->ptr_cubes_num, volume->cube_res, bbox_extension);
	m_checkCudaErrors();
	checkCudaErrors(cudaMemsetAsync(volume->gpu_cubes_occpied_count.dev_ptr, 0, sizeof(int)));
}

__global__
void copy_volume_data_gpu_kernel(OccupcyCube* __restrict__ dev_cubesDst,
						int3 const* __restrict__ dev_ptr_cubes_num,
						int* __restrict__ dev_buf_occupied_cube_idsDst, int* __restrict__ dev_buf_occupied_cube_idsSrc,
						int* __restrict__ dev_global_count_occu_cubes
)
{
	int3 const cubes_num = dev_ptr_cubes_num[0];
	int const cubes_count = cubes_num.x*cubes_num.y*cubes_num.z;
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < *dev_global_count_occu_cubes)
	{
		int cubeId = dev_buf_occupied_cube_idsSrc[idx];
		if (cubeId >= 0 && cubeId < cubes_count)
		{
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
						dev_cubesDst[cubeId + xx + yy*cubes_num.x + zz*cubes_num.x*cubes_num.y].offset = 1;
					}
				}
			}
		}
		dev_buf_occupied_cube_idsDst[idx] = dev_buf_occupied_cube_idsSrc[idx];
	}
}

void VolumetricFusionHelperCudaImpl::copy_some_gpu_volume_data(VolumeTwoLevelHierachy *volumeDst, VolumeTwoLevelHierachy *volumeSrc)
{
	volumeDst->mu = volumeSrc->mu;
	volumeDst->vxl_res = volumeSrc->vxl_res;
	volumeDst->cube_res = volumeSrc->cube_res;
	volumeDst->cube_size_in_voxel = volumeSrc->cube_size_in_voxel;
	volumeDst->vxls_per_cube = volumeSrc->vxls_per_cube;
	volumeDst->cubes_occpied_capacity = volumeSrc->cubes_occpied_capacity;
	volumeDst->cubes_offset = volumeSrc->cubes_offset;
	volumeDst->cubes_num = volumeSrc->cubes_num;
	volumeDst->cubes_occpied_count = volumeSrc->cubes_occpied_count;

	cudaMemcpyAsync(volumeDst->ptr_cubes_offset, volumeSrc->ptr_cubes_offset, sizeof(float)* 3, cudaMemcpyDeviceToDevice);
	cudaMemcpyAsync(volumeDst->ptr_cubes_num, volumeSrc->ptr_cubes_num, sizeof(int)* 3, cudaMemcpyDeviceToDevice);
	cudaMemcpyAsync(volumeDst->gpu_cubes_occpied_count.dev_ptr, volumeSrc->gpu_cubes_occpied_count.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice);
	cudaMemsetAsync(volumeDst->cubes, 0, sizeof(OccupcyCube)*TWO_LEVEL_VOLUME_CUBES_NUM_MAX);

	int threads_per_block = 64;
	int blocks_per_grid = (volumeSrc->cubes_occpied_capacity + threads_per_block - 1) / threads_per_block;
	copy_volume_data_gpu_kernel << <blocks_per_grid, threads_per_block >> >(volumeDst->cubes,
		volumeSrc->ptr_cubes_num,
		volumeDst->buf_occupied_cube_ids, volumeSrc->buf_occupied_cube_ids,
		volumeSrc->gpu_cubes_occpied_count.dev_ptr
		);
}

//one thread one block
__global__
void extract_volume_bbox_kernel(BoundingBox3DCuda * dev_bbox,
								float3 const* dev_ptr_cubes_offset,
								int3 const* dev_ptr_cubes_num,
								float cube_res)
{
	float3 cubes_offset = *dev_ptr_cubes_offset;	
	int3 cubes_num = *dev_ptr_cubes_num;

	dev_bbox->x_s = cubes_offset.x;
	dev_bbox->y_s = cubes_offset.y;
	dev_bbox->z_s = cubes_offset.z;

	dev_bbox->x_e = cubes_offset.x + cubes_num.x*cube_res;
	dev_bbox->y_e = cubes_offset.y + cubes_num.y*cube_res;
	dev_bbox->z_e = cubes_offset.z + cubes_num.z*cube_res;
}

void VolumetricFusionHelperCudaImpl::
extract_volume_bbox(VolumeTwoLevelHierachy const*volume, BoundingBox3DCuda *dev_bbox)
{
	extract_volume_bbox_kernel<<<1, 1>>>(dev_bbox, volume->ptr_cubes_offset, volume->ptr_cubes_num, volume->cube_res);
	m_checkCudaErrors()
}


void VolumetricFusionHelperCudaImpl::
allocate_volume_f2f(int cube_size_in_vxl)
{
	allocate_volume(volume_f2f_curr_, cube_size_in_vxl);
	allocate_volume(volume_f2f_prev_, cube_size_in_vxl);
}


void VolumetricFusionHelperCudaImpl::
allocate_volume(VolumeTwoLevelHierachy* &volume, int cube_size_in_vxl)
{
	volume = new VolumeTwoLevelHierachy();

	checkCudaErrors(cudaMalloc(&(volume->ptr_cubes_offset), sizeof(float3)));
	checkCudaErrors(cudaMalloc(&(volume->ptr_cubes_num), sizeof(int3)));
	checkCudaErrors(cudaMalloc(&(volume->gpu_cubes_occpied_count.dev_ptr), sizeof(int)));
	volume->gpu_cubes_occpied_count.max_size = TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX;

	checkCudaErrors(cudaMalloc(&(volume->cubes), sizeof(OccupcyCube)*TWO_LEVEL_VOLUME_CUBES_NUM_MAX));

	volume->cubes_occpied_capacity = TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX;
	checkCudaErrors(cudaMalloc(&(volume->buf_occupied_cube_ids), sizeof(int)*TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX));

	volume->cube_size_in_voxel = cube_size_in_vxl;
	volume->vxls_per_cube = cube_size_in_vxl*cube_size_in_vxl*cube_size_in_vxl;
	checkCudaErrors(cudaMalloc(&(volume->data), sizeof(float)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
	checkCudaErrors(cudaMalloc(&(volume->weights), sizeof(float)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
	checkCudaErrors(cudaMalloc(&(volume->colors), sizeof(uchar4)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
}

//one sparse cube per cuda block
//512 threads per cuda block
//only lunch cuda block for occupied cubes
template<bool bInitVolume>
__global__ void update_volume_with_curr_depth_f2f_kernel( VolumeDataGPU volume,																																				
														float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
														int depth_width, int depth_height, float mu, bool bUseVisualHull, 
														bool bUseNormal, bool bWeightBaseOnDist)
{
	__shared__ float3 cube_corner;
	float3 cubes_offset;
	__shared__ int3 cubes_num;
	__shared__ int cubeId;
	int data_offset;

	int const count_occupied_cube = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);
	int currentcube = blockIdx.x;

	if (threadIdx.x == 0)
	{
		cubes_num = volume.cubes_num[0];
		cubes_offset = volume.cubes_offset[0];
	}

	for (; currentcube < count_occupied_cube; currentcube += gridDim.x)
	{
		data_offset = currentcube * vxls_per_cube;
		if (threadIdx.x == 0)
		{
			cubeId = volume.occupied_cube_ids[currentcube];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
			xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
			int yCubeId = xCubeId / cubes_num.x;
			xCubeId -= yCubeId*cubes_num.x;

			//corner of the cube
			cube_corner.x = xCubeId*cube_res + cubes_offset.x;
			cube_corner.y = yCubeId*cube_res + cubes_offset.y;
			cube_corner.z = zCubeId*cube_res + cubes_offset.z;
		}
		__syncthreads();

		if (0 <= cubeId && cubeId < cubes_num.x*cubes_num.y*cubes_num.z)
		{
			for (int idx = threadIdx.x; idx < vxls_per_cube; idx += blockDim.x)
			{
				int xId = idx;
				int zId = xId / (cube_size_in_vxl*cube_size_in_vxl);
				xId -= zId*(cube_size_in_vxl*cube_size_in_vxl);
				int yId = xId / cube_size_in_vxl;
				xId -= yId*cube_size_in_vxl;

				cuda_vector_fixed<float, 3> V; // voxel location
				V[0] = cube_corner.x + xId*vxl_res;
				V[1] = cube_corner.y + yId*vxl_res;
				V[2] = cube_corner.z + zId*vxl_res;

				float weight_new = 0.0f;
				float sdf_new = 0.0f;
				float3 color_new = make_float3(0, 0, 0);
				float weight_color_new = 0.0f;

#pragma unroll
				for (int i = 0; i < dev_num_cam_views; i++)
				{
					cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V + dev_cam_views[i].cam_pose.T;

					if (X[2] > 0.1f)
					{
						cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
						float const&fx = K[0][0];
						float const&fy = K[1][1];
						float const&cx = K[0][2];
						float const&cy = K[1][2];

						int u = ROUND(fx * X[0] / X[2] + cx);
						int v = ROUND(fy * X[1] / X[2] + cy);

						if (u >= 0 && u < depth_width &&
							v >= 0 && v < depth_height)
						{
							unsigned short ds = tex2DLayered(tex_depthImgs, u, v, i);
							if (bUseVisualHull)
								depth_extract_fg_set_bg_as_far(ds);
							else
								depth_extract_fg(ds);

							float4 nd_ = tex2DLayered(tex_normalMaps, u, v, i);
							cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);

							if (ds > 0)
							{
								float sdf_cur = ds / 10.0f - X[2];

								if (-mu < sdf_cur && sdf_cur < mu)
								{
									cuda_vector_fixed<float, 3> p;
									p[2] = ds / 10.0f;
									p[0] = (u - cx)*p[2] / fx;
									p[1] = (v - cy)*p[2] / fy;
									p.normalize();
									float weight_cur = 1.0;
									if (bUseNormal)
										weight_cur *= MAX(0.0f, dot_product(p, nd));//1.0;
									if (bWeightBaseOnDist)
										weight_cur *= 1500.0f / ds;
									sdf_cur /= mu;
									sdf_new += sdf_cur * weight_cur;
									weight_new += weight_cur;

								}
								else if (sdf_cur > mu)
								{
									float weight_cur = 0.5f;
									sdf_cur = 1.0f;
									sdf_new += sdf_cur * weight_cur;
									weight_new += weight_cur;
								}
							}
						}
					}
				}

				if (bInitVolume)
				{
					if (weight_new > M_EPS)
					{
						volume.data[data_offset + idx] = sdf_new / weight_new;
						volume.weights[data_offset + idx] = weight_new;

						if (weight_color_new > M_EPS)
						{
							uchar4 clr = make_uchar4(color_new.x / weight_color_new, color_new.y / weight_color_new, color_new.z / weight_color_new, 0);
							volume.colors[data_offset + idx] = clr;
						}
					}
					else
					{
						volume.data[data_offset + idx] = SDF_NULL_VALUE;
						volume.weights[data_offset + idx] = 0.0f;
						volume.colors[data_offset + idx] = make_uchar4(0, 0, 0, 0);
					}
				}
				else
				{
					if (weight_new > M_EPS)
					{
						float weight_old = volume.weights[data_offset + idx];
						float sdf_old = volume.data[data_offset + idx];

						float weight = weight_old + weight_new;
						volume.data[data_offset + idx] = (sdf_new + sdf_old*weight_old) / weight;
						volume.weights[data_offset + idx] = MIN(SDF_WEIGHT_MAX, weight);

						if (weight_color_new > M_EPS)
						{
							uchar4 clr = make_uchar4(color_new.x / weight_color_new, color_new.y / weight_color_new, color_new.z / weight_color_new, 0);
							volume.colors[data_offset + idx] = clr;
						}
					}
				}
			}
		}
		__syncthreads();
	}
}


__global__ 
__launch_bounds__(512, 3)
void update_volume_with_curr_depth_f2f_kernel_vResidual(VolumeDataGPU volume,
									float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
									float const* __restrict__  dev_depth_align_residual,
									int depth_width_prj, int depth_height_prj,
									int depth_width, int depth_height, float mu, bool bUseVisualHull,
									bool bUseNormal, bool bWeightBaseOnDist)
{
	__shared__ float3 cube_corner;
	float3 cubes_offset;
	__shared__ int3 cubes_num;
	__shared__ int cubeId;
	__shared__ int data_offset;

	int const count_occupied_cube = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);
	int currentcube = blockIdx.x;

	if (threadIdx.x == 0)
	{
		cubes_num = volume.cubes_num[0];
		cubes_offset = volume.cubes_offset[0];
	}

	for (; currentcube < count_occupied_cube; currentcube += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId = volume.occupied_cube_ids[currentcube];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
			xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
			int yCubeId = xCubeId / cubes_num.x;
			xCubeId -= yCubeId*cubes_num.x;

			data_offset = currentcube * vxls_per_cube;
			//corner of the cube
			cube_corner.x = xCubeId*cube_res + cubes_offset.x;
			cube_corner.y = yCubeId*cube_res + cubes_offset.y;
			cube_corner.z = zCubeId*cube_res + cubes_offset.z;
		}
		__syncthreads();

		if (0 <= cubeId && cubeId < cubes_num.x*cubes_num.y*cubes_num.z)
		{
			for (int idx = threadIdx.x; idx < vxls_per_cube; idx += blockDim.x)
			{
				int xId = idx;
				int zId = xId / (cube_size_in_vxl*cube_size_in_vxl);
				xId -= zId*(cube_size_in_vxl*cube_size_in_vxl);
				int yId = xId / cube_size_in_vxl;
				xId -= yId*cube_size_in_vxl;

				cuda_vector_fixed<float, 3> V; // voxel location
				V[0] = cube_corner.x + xId*vxl_res;
				V[1] = cube_corner.y + yId*vxl_res;
				V[2] = cube_corner.z + zId*vxl_res;

				float weight_new = 0.0f;
				float sdf_new = 0.0f;
				float3 color_new = make_float3(0, 0, 0);
				float weight_color_new = 0.0f;

				float residual_per_pxl_avg = 0.0f;
				int residual_count = 0;

#pragma unroll
				for (int i = 0; i < dev_num_cam_views; i++)
				{
					cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V + dev_cam_views[i].cam_pose.T;

					if (X[2] > 0.1f)
					{
						cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
						float const&fx = K[0][0];
						float const&fy = K[1][1];
						float const&cx = K[0][2];
						float const&cy = K[1][2];

						float u_ = fx * X[0] / X[2] + cx;
						float v_ = fy * X[1] / X[2] + cy;
						int u = ROUND(u_);
						int v = ROUND(v_);

						if (u >= 0 && u < depth_width &&
							v >= 0 && v < depth_height)
						{
							unsigned short ds = tex2DLayered(tex_depthImgs, u, v, i);
							if (bUseVisualHull)
								depth_extract_fg_set_bg_as_far(ds);
							else
								depth_extract_fg(ds);

							float4 nd_ = tex2DLayered(tex_normalMaps, u, v, i);
							cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);

							if (ds > 0)
							{
								float sdf_cur = ds / 10.0f - X[2];

								if (-mu < sdf_cur && sdf_cur < mu)
								{
									int u_proj = MAX(0, MIN(depth_width_prj - 1, ROUND(u_ * depth_width_prj / depth_width)));
									int v_proj = MAX(0, MIN(depth_height_prj - 1, ROUND(v_ * depth_height_prj / depth_height)));
									float residual_per_pxl = dev_depth_align_residual[i*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
									if (residual_per_pxl >= 0.0f)
									{
										residual_per_pxl_avg += residual_per_pxl;
										residual_count++;
									}

									cuda_vector_fixed<float, 3> p;
									p[2] = ds / 10.0f;
									p[0] = (u - cx)*p[2] / fx;
									p[1] = (v - cy)*p[2] / fy;
									p.normalize();
									float weight_cur = 1.0f;
									if (bUseNormal)
									{
										float w_norm = MAX(0.0f, dot_product(p, nd)) / 0.75f;
										weight_cur *= w_norm*w_norm;
									}
									if (bWeightBaseOnDist)
									{
										float w_dist = 1200.0f / ds;
										weight_cur *= w_dist*w_dist;
									}

									sdf_cur /= mu;
									sdf_new += sdf_cur * weight_cur;
									weight_new += weight_cur;

								}
								else if (sdf_cur > mu)
								{
									float weight_cur = 0.5f;
									sdf_cur = 1.0f;
									sdf_new += sdf_cur * weight_cur;
									weight_new += weight_cur;
								}
							}
						}
					}
				}

				if (weight_new > M_EPS)
				{
					float weight_old = volume.weights[data_offset + idx];
					float sdf_old = volume.data[data_offset + idx];

					const float thres_residual1 = 0.5f;
					const float thres_residual2 = 0.90f;
					if (residual_count > 0)
					{
						residual_per_pxl_avg /= residual_count;
						residual_per_pxl_avg = MAX(0.0f, residual_per_pxl_avg - thres_residual1) / (thres_residual2 - thres_residual1);
						float blend_weight = MAX(0.0f, 1.0f - residual_per_pxl_avg);
						blend_weight = powf(blend_weight, 0.5f);
						weight_old *= blend_weight;
					}

					float weight = weight_old + weight_new;
					volume.data[data_offset + idx] = (sdf_new + sdf_old*weight_old) / weight;
					volume.weights[data_offset + idx] = MIN(SDF_WEIGHT_MAX, weight);

					if (weight_color_new > M_EPS)
					{
						uchar4 clr = make_uchar4(color_new.x / weight_color_new, color_new.y / weight_color_new, color_new.z / weight_color_new, 0);
						volume.colors[data_offset + idx] = clr;
					}
				}

			}
		}
		__syncthreads();
	}
}



void VolumetricFusionHelperCudaImpl::
init_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, bool bUseVisualHull, bool bUseNormal, bool bWeightBasedOnDepth)
{
	VolumeDataGPU volume_gpu(*volume);

	int threads_per_block = 512;
	int blocks_per_grid = 24*4*8;
	update_volume_with_curr_depth_f2f_kernel<true><<<blocks_per_grid, threads_per_block>>>(volume_gpu, 
													volume->cube_res, volume->cube_size_in_voxel, volume->vxl_res, volume->vxls_per_cube, 
													depth_width_, depth_height_, volume->mu, bUseVisualHull, bUseNormal, bWeightBasedOnDepth);
	m_checkCudaErrors();
}

void VolumetricFusionHelperCudaImpl::
update_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, bool bUseVisualHull, bool bUseNormal, bool bWeightBasedOnDepth)
{
	VolumeDataGPU volume_gpu(*volume);

	int threads_per_block = 512;
	int blocks_per_grid = 24 * 4 * 8;
	update_volume_with_curr_depth_f2f_kernel<false><<<blocks_per_grid, threads_per_block>>>(volume_gpu,
		volume->cube_res, volume->cube_size_in_voxel, volume->vxl_res, volume->vxls_per_cube,
		depth_width_, depth_height_, volume->mu, bUseVisualHull, bUseNormal, bWeightBasedOnDepth);
	m_checkCudaErrors();
}


void VolumetricFusionHelperCudaImpl::
update_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, 
							  float const* dev_depth_align_residual, int width_proj, int height_proj, 
							  bool bUseVisualHull, bool bUseNormal, bool bWeightBasedOnDepth)
{
	VolumeDataGPU volume_gpu(*volume);

	int threads_per_block = 512;
	int blocks_per_grid = 24 * 3 * 8;
	update_volume_with_curr_depth_f2f_kernel_vResidual<<<blocks_per_grid, threads_per_block >>>(volume_gpu,
		volume->cube_res, volume->cube_size_in_voxel, volume->vxl_res, volume->vxls_per_cube,
		dev_depth_align_residual, width_proj, height_proj,
		depth_width_, depth_height_, volume->mu, bUseVisualHull, bUseNormal, bWeightBasedOnDepth);
	m_checkCudaErrors();
}


template<class T>
__global__
void init_two_level_hierachy_volume_data_kernel(T* __restrict__ dev_vxl_data,
												int const* dev_count_occupied_cubes_dst,
												int vxls_per_cube,
												T val)
{
	const int count_occupied_cubes = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, *dev_count_occupied_cubes_dst);

	for (int blkIdx = blockIdx.x; blkIdx < count_occupied_cubes; blkIdx += gridDim.x)
	{
		for (int i = threadIdx.x; i < vxls_per_cube; i += blockDim.x)
		{
			dev_vxl_data[blkIdx*vxls_per_cube + i] = val;
		}
	}
}

void VolumetricFusionHelperCudaImpl::init_volume_to_zero(VolumeTwoLevelHierachy *volume)
{
	int threads_per_block = 512;
	int blocks_per_grid = 1024;
	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(volume->data, volume->gpu_cubes_occpied_count.dev_ptr, 
																			volume->vxls_per_cube, 0.0f);
	m_checkCudaErrors();

	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(volume->weights, volume->gpu_cubes_occpied_count.dev_ptr, 
																			volume->vxls_per_cube, 0.0f);
	m_checkCudaErrors();

	uchar4 clr_init = make_uchar4(0, 0, 0, 0);
	init_two_level_hierachy_volume_data_kernel<uchar4><<<blocks_per_grid, threads_per_block>>>(volume->colors, volume->gpu_cubes_occpied_count.dev_ptr, 
																			volume->vxls_per_cube, clr_init);
	m_checkCudaErrors();

}


//one cube per thread
//find the neighboring ed nodes for each cube and save the indices
__global__
void warp_volume_f2f_kernel_ngns(int* cube_ngns_indices,
								 int2 *cube_ngn_indices_range,//starting point, num
								 int const*dev_buf_occupied_cube_ids, int const* dev_global_count_occu_cubes,
								 float3 const* dev_ptr_cubes_offset, int3 const* dev_ptr_cubes_dim, float cube_res,
								 int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res, float ed_search_radius)
{
	__shared__ int sh_ngns_count;
	const int occupied_cubes_count = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, *dev_global_count_occu_cubes);

	if (blockIdx.x > occupied_cubes_count / blockDim.x)
		return;

	if (threadIdx.x == 0)
		sh_ngns_count = 0;
	__syncthreads();

	int3 const cubes_dim = *dev_ptr_cubes_dim;
	float3 const cubes_offset = *dev_ptr_cubes_offset;

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	int ngns_count = 0;
	int lc_offset = 0;

	int xEdCubeIdSt = 0;
	int yEdCubeIdSt = 0;
	int zEdCubeIdSt = 0;
	int xEdCubeIdEnd = 0;
	int yEdCubeIdEnd = 0;
	int zEdCubeIdEnd = 0;
	if (idx < occupied_cubes_count)
	{
		int3 const ed_cubes_dims = dev_ed_cubes_dims[0];
		float3 const ed_cubes_offsets = dev_ed_cubes_offsets[0];

		int cubeId = dev_buf_occupied_cube_ids[idx];
		int xCubeId = cubeId;
		int zCubeId = xCubeId / (cubes_dim.x*cubes_dim.y);
		xCubeId -= zCubeId*cubes_dim.x*cubes_dim.y;
		int yCubeId = xCubeId / cubes_dim.x;
		xCubeId -= yCubeId*cubes_dim.x;

		//cube center
		float pt_cen[3];
		pt_cen[0] = cubes_offset.x + (xCubeId + 0.5f)*cube_res;
		pt_cen[1] = cubes_offset.y + (yCubeId + 0.5f)*cube_res;
		pt_cen[2] = cubes_offset.z + (zCubeId + 0.5f)*cube_res;

		xEdCubeIdSt = MAX(0, (pt_cen[0] - ed_search_radius - ed_cubes_offsets.x) / ed_cube_res);
		yEdCubeIdSt = MAX(0, (pt_cen[1] - ed_search_radius - ed_cubes_offsets.y) / ed_cube_res);
		zEdCubeIdSt = MAX(0, (pt_cen[2] - ed_search_radius - ed_cubes_offsets.z) / ed_cube_res);
		xEdCubeIdEnd = MIN(ed_cubes_dims.x - 1, (pt_cen[0] + ed_search_radius - ed_cubes_offsets.x) / ed_cube_res);
		yEdCubeIdEnd = MIN(ed_cubes_dims.y - 1, (pt_cen[1] + ed_search_radius - ed_cubes_offsets.y) / ed_cube_res);
		zEdCubeIdEnd = MIN(ed_cubes_dims.z - 1, (pt_cen[2] + ed_search_radius - ed_cubes_offsets.z) / ed_cube_res);

		for (int k = zEdCubeIdSt; k <= zEdCubeIdEnd; k++)
		for (int j = yEdCubeIdSt; j <= yEdCubeIdEnd; j++)
		for (int i = xEdCubeIdSt; i <= xEdCubeIdEnd; i++)
		{
			short ndId = tex3D(tex_ndIds, i, j, k);
			if (ndId >= 0)
				ngns_count++;
		}
		if (ngns_count > 0)
			lc_offset = atomicAdd(&sh_ngns_count, ngns_count);
	}
	__syncthreads();

	if (threadIdx.x == 0)
		sh_ngns_count = atomicAdd(&dev_global_cube_ngns_count_sum, sh_ngns_count);
	__syncthreads();

	if (idx < occupied_cubes_count)
	{
		int offset = sh_ngns_count + lc_offset;
		int2 range = make_int2(offset, ngns_count);
		cube_ngn_indices_range[idx] = range;

		for (int k = zEdCubeIdSt; k <= zEdCubeIdEnd; k++)
		for (int j = yEdCubeIdSt; j <= yEdCubeIdEnd; j++)
		for (int i = xEdCubeIdSt; i <= xEdCubeIdEnd; i++)
		{
			short ndId = tex3D(tex_ndIds, i, j, k);
			if (ndId >= 0)
				cube_ngns_indices[offset++] = ndId;
		}
	}
}

//512 threads per cuda block
//one cube per cuda block
__global__
__launch_bounds__(512, 2)
void warp_and_update_volume_f2f_kernel(VolumeConstDataGPU volume_src, VolumeDataGPU volume_dst, 
									  float* __restrict__ dev_weights_interp, int* __restrict__ dev_counts_interp,
									   float cube_res_src, float vxl_res_src,
									   float cube_res_dst, float vxl_res_dst, int vxls_per_cube, int cube_size_in_vxl,
									   int const* __restrict__ cube_ngns_indices, int2 const* __restrict__ cube_ngn_indices_range,//starting point, num
									   DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, float const* __restrict__ dev_ed_nodes_align_residual,
									   int const*__restrict__ dev_ed_nodes_num, float sigma_vxl_node_dist,
									   RigidTransformCuda const* __restrict__ dev_rigid_transf,
									   float mu, float thres_align_residual
									  )
{	
	__shared__ int3 cubes_num_src;
	__shared__ float3 cubes_offset_src;
	__shared__ RigidTransformCuda sh_rigid_transf;

	__shared__ float3 cube_corner_src;
	__shared__ int cubeId_src;
	__shared__ int data_offset_src;

	__shared__ float sh_volume[10 * 10 * 10];
	__shared__ float sh_volume_f[10 * 10 * 10];

#define MAX_ED_NODES_PER_CUBE_V3 200
	__shared__ int ngn_idx_range_st;
	__shared__ int sh_ed_nodes_num;
	__shared__ DeformGraphNodeCoreCudaL2 sh_ed_nodes[MAX_ED_NODES_PER_CUBE_V3];
	__shared__ float sh_ed_node_align_residual[MAX_ED_NODES_PER_CUBE_V3];

	const int count_occupied_cubes_src = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume_src.count_occu_cubes[0]);
	if (threadIdx.x == 0)
	{
		cubes_num_src = volume_src.cubes_num[0];
		cubes_offset_src = volume_src.cubes_offset[0];
		sh_rigid_transf = dev_rigid_transf[0];
	}

	const float sigma_s = 10.0f;
	float expval[4];
#pragma unroll 
	for (int i = 0; i < 4; i++) expval[i] = __expf(-(i) / (2.0f*sigma_s*sigma_s));

	for (int blkId = blockIdx.x; blkId < count_occupied_cubes_src; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId_src = volume_src.occupied_cube_ids[blkId];
			int xCubeId = cubeId_src;
			int zCubeId = xCubeId / (cubes_num_src.x*cubes_num_src.y);
			xCubeId -= zCubeId*cubes_num_src.x*cubes_num_src.y;
			int yCubeId = xCubeId / cubes_num_src.x;
			xCubeId -= yCubeId*cubes_num_src.x;

			data_offset_src = blkId * vxls_per_cube;

			//corner of the cube
			cube_corner_src.x = xCubeId*cube_res_src + cubes_offset_src.x;
			cube_corner_src.y = yCubeId*cube_res_src + cubes_offset_src.y;
			cube_corner_src.z = zCubeId*cube_res_src + cubes_offset_src.z;

			int2 ngn_idx_range = cube_ngn_indices_range[blkId];
			ngn_idx_range_st = ngn_idx_range.x;
			sh_ed_nodes_num = MIN(MAX_ED_NODES_PER_CUBE_V3, ngn_idx_range.y);
		}
		__syncthreads();

		//load ed nodes
		for (int i = threadIdx.x; i < sh_ed_nodes_num * 9; i += blockDim.x)
		{
			int idx = i / 9;
			int ele_idx = i % 9;

			int ndId = cube_ngns_indices[ngn_idx_range_st + idx];
			sh_ed_nodes[idx].A(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A(ele_idx / 3, ele_idx % 3);
			sh_ed_nodes[idx].A_inv_t(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A_inv_t(ele_idx / 3, ele_idx % 3);

			if (ele_idx == 0)
			{
				if (dev_ed_nodes_align_residual != NULL)
					sh_ed_node_align_residual[idx] = dev_ed_nodes_align_residual[ndId];
			}

			if (ele_idx < 3)
			{
				sh_ed_nodes[idx].t[ele_idx] = dev_ed_nodes[ndId].t[ele_idx];
				sh_ed_nodes[idx].g[ele_idx] = dev_ed_nodes[ndId].g[ele_idx];
				sh_ed_nodes[idx].n[ele_idx] = dev_ed_nodes[ndId].n[ele_idx];
			}
		}
		__syncthreads();

		if (0 <= cubeId_src && cubeId_src < cubes_num_src.x*cubes_num_src.y*cubes_num_src.z &&
			sh_ed_nodes_num > 0)
		{
			const int xOffset = 0;
			const int yOffset = 0;
			const int zOffset = 0;
			{
				//load volume data to shared memory
				for (int idx = threadIdx.x; idx < 10 * 10 * 10; idx += blockDim.x)
				{
					//compute 3D idx for the source
					int xId = idx; //original 3d index on sh_volume
					int zId = idx / (10 * 10);
					xId -= zId * 10 * 10;
					int yId = xId / 10;
					xId -= yId * 10;

					xId += xOffset-1; //offset 
					yId += yOffset-1;
					zId += zOffset-1;

					if (0 <= xId && xId < cube_size_in_vxl &&
						0 <= yId && yId < cube_size_in_vxl &&
						0 <= zId && zId < cube_size_in_vxl)
					{
						int pos = vxls_per_cube*blkId +
							zId*cube_size_in_vxl*cube_size_in_vxl +
							yId*cube_size_in_vxl + xId;
						float weight = volume_src.weights[pos];
						if (weight > M_EPS)
							sh_volume[idx] = volume_src.data[pos];
						else
							sh_volume[idx] = SDF_NULL_VALUE;
					}
					else
					{
						int cubeId_tmp = cubeId_src;
						if (xId >= cube_size_in_vxl)
						{
							cubeId_tmp += 1;
							xId -= cube_size_in_vxl;
						}
						else if (xId < 0)
						{
							cubeId_tmp -= 1;
							xId += cube_size_in_vxl;
						}
						if (yId >= cube_size_in_vxl)
						{
							cubeId_tmp += cubes_num_src.x;
							yId -= cube_size_in_vxl;
						}
						else if (yId < 0)
						{
							cubeId_tmp -= cubes_num_src.x;
							yId += cube_size_in_vxl;
						}
						if (zId >= cube_size_in_vxl)
						{
							cubeId_tmp += cubes_num_src.x*cubes_num_src.y;
							zId -= cube_size_in_vxl;
						}
						else if (zId < 0)
						{
							cubeId_tmp -= cubes_num_src.x*cubes_num_src.y;
							zId += cube_size_in_vxl;
						}
						if (cubeId_tmp < 0 || cubeId_tmp >= cubes_num_src.x*cubes_num_src.y*cubes_num_src.z)
						{
							sh_volume[idx] = SDF_NULL_VALUE;
						} else {
							int offset = volume_src.cubes[cubeId_tmp].offset;
							if (offset < 0){
								sh_volume[idx] = SDF_NULL_VALUE;
							} else{
								int pos = vxls_per_cube*offset +
									zId*cube_size_in_vxl*cube_size_in_vxl +
									yId*cube_size_in_vxl + xId;
								float weight = volume_src.weights[pos];
								if (weight > M_EPS)
									sh_volume[idx] = volume_src.data[pos];
								else
									sh_volume[idx] = SDF_NULL_VALUE;
							}
						}
					}
				}
				__syncthreads();
				
				if (threadIdx.x < 5 * 10 * 10)
				{
					int idx = threadIdx.x;
					//3d index on sh_volume
					int xId = (idx % 5)<< 1; 
					int zId = idx / (5 * 10);
					int yId = (idx % (5 * 10)) / 5;

					float ws = 0;
					float vals = 0;
					float ws2 = 0;
					float vals2 = 0;
					for (int k = -1; k <= 1; k++)
					{
						int zId_n = zId + k;
						for (int j = -1; j <= 1; j++)
						{
							int yId_n = yId + j;
							if (0 <= yId_n && yId_n < 10 &&
								0 <= zId_n && zId_n < 10)
							{
								int idx_sh_n = zId_n * 10 * 10 + yId_n * 10 + xId;
								float val[4];
								if (xId > 0) val[0] = sh_volume[idx_sh_n - 1]; else val[0] = -2;
								val[1] = sh_volume[idx_sh_n];
								if (xId < 9) val[2] = sh_volume[idx_sh_n + 1]; else val[0] = -2;
								if (xId < 8) val[3] = sh_volume[idx_sh_n + 2]; else val[0] = -2;
								if (-1.0f <= val[0] && val[0] <= 1.0f)
								{
									vals += val[0] * expval[1 + j*j + k*k];
									ws += expval[1 + j*j + k*k];
								}
								if (-1.0f <= val[1] && val[1] <= 1.0f)
								{
									vals += val[1] * expval[0 + j*j + k*k];
									ws += expval[0 + j*j + k*k];
									vals2 += val[1] * expval[1 + j*j + k*k];
									ws2 += expval[1 + j*j + k*k];
								}
								if (-1.0f <= val[2] && val[2] <= 1.0f)
								{
									vals += val[2] * expval[1 + j*j + k*k];
									ws += expval[1 + j*j + k*k];
									vals2 += val[2] * expval[0 + j*j + k*k];
									ws2 += expval[0 + j*j + k*k];
								}
								if (-1.0f <= val[3] && val[3] <= 1.0f)
								{
									vals2 += val[3] * expval[1 + j*j + k*k];
									ws2 += expval[1 + j*j + k*k];
								}
							}
						}
					}
					if (ws > 0.1f)
						sh_volume_f[idx*2] = vals / ws;
					else
						sh_volume_f[idx*2] = SDF_NULL_VALUE;
					if (ws2 > 0.1f)
						sh_volume_f[idx*2 + 1] = vals2 / ws2;
					else
						sh_volume_f[idx*2 + 1] = SDF_NULL_VALUE;
				}				
				__syncthreads();

				//process one voxel: find neighboring ed nodes; warp it to target
				//grid id in the current block of the current cube
				int xId = threadIdx.x;
				int zId = xId / (8 * 8);
				xId -= zId*(8 * 8);
				int yId = xId / 8;
				xId -= yId * 8;

				//idx in the current cube
				int idx = (zId + zOffset)*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + 
						(yId + yOffset)*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + (xId + xOffset);
				float sdf_src = volume_src.data[data_offset_src + idx];
				float weight_src = volume_src.weights[data_offset_src + idx];

				if (1.0f > sdf_src && sdf_src > -1.0f && weight_src > M_EPS)
				{
					cuda_vector_fixed<float, 3> V; // voxel location
					V[0] = cube_corner_src.x + (xId + xOffset)*vxl_res_src;
					V[1] = cube_corner_src.y + (yId + yOffset)*vxl_res_src;
					V[2] = cube_corner_src.z + (zId + zOffset)*vxl_res_src;

					//find neighboring ed nodes
					float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
					int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						dists_sq[i] = 1.0e+10f;
						ngn_idx[i] = -1;
					}
					for (int i = 0; i < sh_ed_nodes_num; i++)
					{
						float dist_sq = dist_square<3>(V.data_block(), sh_ed_nodes[i].g.data_block());
						if (dist_sq < 4.0f*sigma_vxl_node_dist*sigma_vxl_node_dist + mu*mu)
						{
							if (dist_sq < dists_sq[0])
							{
								dists_sq[0] = dist_sq;
								ngn_idx[0] = i;
							}

							#pragma unroll
							for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
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
					float w_sum = 0;
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						if (ngn_idx[i] != -1)
						{
							dists_sq[i] = __expf(-dists_sq[i] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
							w_sum += dists_sq[i];
						}
					}
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						if (ngn_idx[i] != -1)  dists_sq[i] /= w_sum;
					}

					const float thres_sdf_for_grad = 0.60f;

					if (w_sum > 1.0e-2f)
					{
						//interpolate the alignment residual
						float align_residual = 0.0f;
						for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
						{
							int lc_ndIdx = ngn_idx[k];
							if (lc_ndIdx >= 0)
								align_residual = MAX(align_residual, sh_ed_node_align_residual[lc_ndIdx]);
						}

						if (align_residual < thres_align_residual)
						{
							//get the gradient at current in the source volume
							cuda_vector_fixed<float, 3> grad;
							int idx_sh = (zId + 1) * 10 * 10 + (yId + 1) * 10 + xId + 1;
							float val_1 = sh_volume_f[idx_sh];
							float val_0, val_2;
							int bValidComputation = 0;
							if (-thres_sdf_for_grad < val_1 && val_1 < thres_sdf_for_grad)
							{
								bValidComputation = 1;

								//compute dx
								val_0 = sh_volume_f[idx_sh - 1];
								val_2 = sh_volume_f[idx_sh + 1];
								if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad &&
									thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[0] = (val_0 - val_2) / 2.0f;
								else if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad)
									grad[0] = val_0 - val_1;
								else if (thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[0] = val_1 - val_2;
								else
									bValidComputation = 0;

								//compute dy
								val_0 = sh_volume_f[idx_sh - 10];
								val_2 = sh_volume_f[idx_sh + 10];
								if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad &&
									thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[1] = (val_0 - val_2) / 2.0f;
								else if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad)
									grad[1] = val_0 - val_1;
								else if (thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[1] = val_1 - val_2;
								else
									bValidComputation = 0;

								//compute dz
								val_0 = sh_volume_f[idx_sh - 100];
								val_2 = sh_volume_f[idx_sh + 100];
								if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad &&
									thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[2] = (val_0 - val_2) / 2.0f;
								else if (thres_sdf_for_grad > val_0 && val_0 > -thres_sdf_for_grad)
									grad[2] = val_0 - val_1;
								else if (thres_sdf_for_grad > val_2 && val_2 > -thres_sdf_for_grad)
									grad[2] = val_1 - val_2;
								else
									bValidComputation = 0;
							}
							if (bValidComputation == 0)
							{
								grad[0] = 0.0;
								grad[1] = 0.0;
								grad[2] = 0.0;
							}

							float grad_norm = grad.two_norm();
							if (grad_norm > M_EPS)
							{
								grad[0] /= grad_norm;
								grad[1] /= grad_norm;
								grad[2] /= grad_norm;
							}

							//warp the voxel
							cuda_vector_fixed<float, 3> V_t(0.0f);
							cuda_vector_fixed<float, 3> grad_t(0.0f);
							#pragma unroll
							for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
							{
								int &ndIdx_k = ngn_idx[k];
								if (ndIdx_k >= 0)
								{
									DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
									cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
									cuda_matrix_fixed<float, 3, 3> const&A_inv_t = nd.A_inv_t;
									cuda_vector_fixed<float, 3> const&g = nd.g;
									cuda_vector_fixed<float, 3> const&t = nd.t;

									float &w_k = dists_sq[k];
									V_t += w_k*(A*(V - g) + g + t);
									grad_t += w_k*(A_inv_t*grad);
								}
							}
							V_t = sh_rigid_transf.R*V_t + sh_rigid_transf.T;
							grad_t = sh_rigid_transf.R*grad_t;
							grad_t.normalize();

							//find the cube id at the target
							float3 const cubes_offset_dst = *(volume_dst.cubes_offset);
							int3 const cubes_num_dst = *(volume_dst.cubes_num);
							//vxl id in the regular grid
							float xVxlId_dst_ = (V_t[0] - cubes_offset_dst.x) / vxl_res_dst;
							float yVxlId_dst_ = (V_t[1] - cubes_offset_dst.y) / vxl_res_dst;
							float zVxlId_dst_ = (V_t[2] - cubes_offset_dst.z) / vxl_res_dst;

							float const radius = 1.6f; //make sure big than sqrt(3)/2
							float const sigma2 = 0.55f*0.55f;

							for (int zId_dst = MAX(0, (int)ceilf(zVxlId_dst_ - radius)); zId_dst <= (int)(zVxlId_dst_ + radius); zId_dst++)
							for (int yId_dst = MAX(0, (int)ceilf(yVxlId_dst_ - radius)); yId_dst <= (int)(yVxlId_dst_ + radius); yId_dst++)
							for (int xId_dst = MAX(0, (int)ceilf(xVxlId_dst_ - radius)); xId_dst <= (int)(xVxlId_dst_ + radius); xId_dst++)
							{
								float dist2 = (xId_dst - xVxlId_dst_)*(xId_dst - xVxlId_dst_) +
									(yId_dst - yVxlId_dst_)*(yId_dst - yVxlId_dst_) +
									(zId_dst - zVxlId_dst_)*(zId_dst - zVxlId_dst_);
								if (dist2 < radius*radius)
								{

									float w = __expf(-dist2 / (2.0f*sigma2));// -

									int xCubeId_dst = xId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int yCubeId_dst = yId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int zCubeId_dst = zId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int cubeId_dst = zCubeId_dst*cubes_num_dst.x*cubes_num_dst.y + yCubeId_dst*cubes_num_dst.x + xCubeId_dst;
									if (xCubeId_dst < cubes_num_dst.x &&
										yCubeId_dst < cubes_num_dst.y &&
										zCubeId_dst < cubes_num_dst.z)
									{
										int data_offset_dst = volume_dst.cubes[cubeId_dst].offset*vxls_per_cube;
										if (data_offset_dst >= 0)
										{
											//vxl id
											int xId_dst0 = xId_dst - xCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
											int yId_dst0 = yId_dst - yCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
											int zId_dst0 = zId_dst - zCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

											int vxlIdx_dst = zId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
												yId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId_dst0;

											cuda_vector_fixed<float, 3> delta;
											delta[0] = xId_dst - xVxlId_dst_;
											delta[1] = yId_dst - yVxlId_dst_;
											delta[2] = zId_dst - zVxlId_dst_;

											float d_sdf = -dot_product(delta, grad_t)*vxl_res_dst / mu;

											float weight_new = weight_src*w;
											float sdf_new = MAX(-1.0f, MIN(1.0f, (sdf_src + d_sdf)))*w;
											atomicAdd(&(dev_weights_interp[data_offset_dst + vxlIdx_dst]), w);
											if (dist2 <= 1.0f)
												atomicAdd(&(dev_counts_interp[data_offset_dst + vxlIdx_dst]), 1);
											atomicAdd(&(volume_dst.data[data_offset_dst + vxlIdx_dst]), sdf_new);
											atomicAdd(&(volume_dst.weights[data_offset_dst + vxlIdx_dst]), weight_new);
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
	}
}

__global__
void correct_volume_weights_kernel(float* __restrict__ vxl_data, float* __restrict__ vxl_weights, uchar4* __restrict__ vxl_colors, 
								   float* __restrict__ dev_weights_interp, int* __restrict__ dev_counts_interp,
								   int vxl_per_cube, int const* dev_occupied_cubes_count)
{
	const int count_occupied_cubes = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, dev_occupied_cubes_count[0]);
	for (int blkIdx = blockIdx.x; blkIdx < count_occupied_cubes; blkIdx += gridDim.x)
	{
		int data_offset = blkIdx*vxl_per_cube;
		for (int i = threadIdx.x; i < vxl_per_cube; i += blockDim.x)
		{
			int idx = data_offset + i;
			float w_iterp = dev_weights_interp[idx];
			int vote_counts = dev_counts_interp[idx];
			float weight = vxl_weights[idx];
			float sdf = vxl_data[idx];

			if (w_iterp > 0.5f && vote_counts >= 2)
			{
				vxl_data[idx] = sdf / w_iterp;
				vxl_weights[idx] = weight / w_iterp;
			}
			else
			{
				vxl_data[idx] = SDF_NULL_VALUE;
				vxl_weights[idx] = 0.0f;
			}
		}
	}
}

__global__
__launch_bounds__(512, 2)
void warp_volume_Step1_save_vxl_pos_kernel(ushort3* __restrict__ dev_vxl_pos_t,
										 VolumeConstDataGPU volume_src,
										 int3 const* __restrict__ dev_cubes_num_dst, float3 const* __restrict__ dev_cubes_offset_dst,
										 float cube_res_src, float vxl_res_src,
										 float cube_res_dst, float vxl_res_dst, int vxls_per_cube, int cube_size_in_vxl,
										 int const* __restrict__ cube_ngns_indices, int2 const* __restrict__ cube_ngn_indices_range,//starting point, num
										 DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, float const* __restrict__ dev_ed_nodes_align_residual,
										 int const*__restrict__ dev_ed_nodes_num, float sigma_vxl_node_dist,
										 RigidTransformCuda const* __restrict__ dev_rigid_transf,
										 float thres_align_residual,
										 float mu)
{
	__shared__ int3 cubes_num_src;
	__shared__ float3 cubes_offset_src;
	__shared__ int3 cubes_num_dst;
	__shared__ float3 cubes_offset_dst;
	__shared__ RigidTransformCuda sh_rigid_transf;

	__shared__ float3 cube_corner_src;
	__shared__ int cubeId_src;
	__shared__ int data_offset_src;

#define MAX_ED_NODES_PER_CUBE_V4 300
	__shared__ int ngn_idx_range_st;
	__shared__ int sh_ed_nodes_num;
	__shared__ DeformGraphNodeCoreCudaL2 sh_ed_nodes[MAX_ED_NODES_PER_CUBE_V4];
	__shared__ int sh_ed_node_indices[MAX_ED_NODES_PER_CUBE_V4];
	__shared__ float sh_ed_node_align_residual[MAX_ED_NODES_PER_CUBE_V4];

	const int count_occupied_cubes_src = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume_src.count_occu_cubes[0]);
	if (threadIdx.x == 0)
	{
		cubes_num_src = volume_src.cubes_num[0];
		cubes_offset_src = volume_src.cubes_offset[0];
		cubes_num_dst = dev_cubes_num_dst[0];
		cubes_offset_dst = dev_cubes_offset_dst[0];
		sh_rigid_transf = dev_rigid_transf[0];
	}

	for (int blkId = blockIdx.x; blkId < count_occupied_cubes_src; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId_src = volume_src.occupied_cube_ids[blkId];
			int xCubeId = cubeId_src;
			int zCubeId = xCubeId / (cubes_num_src.x*cubes_num_src.y);
			xCubeId -= zCubeId*cubes_num_src.x*cubes_num_src.y;
			int yCubeId = xCubeId / cubes_num_src.x;
			xCubeId -= yCubeId*cubes_num_src.x;

			data_offset_src = blkId * vxls_per_cube;

			//corner of the cube
			cube_corner_src.x = xCubeId*cube_res_src + cubes_offset_src.x;
			cube_corner_src.y = yCubeId*cube_res_src + cubes_offset_src.y;
			cube_corner_src.z = zCubeId*cube_res_src + cubes_offset_src.z;

			int2 ngn_idx_range = cube_ngn_indices_range[blkId];
			ngn_idx_range_st = ngn_idx_range.x;
			sh_ed_nodes_num = MIN(MAX_ED_NODES_PER_CUBE_V4, ngn_idx_range.y);
		}
		__syncthreads();

		//load ed nodes
		for (int i = threadIdx.x; i < sh_ed_nodes_num * 9; i += blockDim.x)
		{
			int idx = i / 9;
			int ele_idx = i % 9;

			int ndId = cube_ngns_indices[ngn_idx_range_st + idx];
			sh_ed_nodes[idx].A(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A(ele_idx / 3, ele_idx % 3);
			sh_ed_nodes[idx].A_inv_t(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A_inv_t(ele_idx / 3, ele_idx % 3);

			if (ele_idx == 0)
			{
				sh_ed_node_indices[idx] = ndId;
				if (dev_ed_nodes_align_residual != NULL)
					sh_ed_node_align_residual[idx] = dev_ed_nodes_align_residual[ndId];
			}

			if (ele_idx < 3)
			{
				sh_ed_nodes[idx].t[ele_idx] = dev_ed_nodes[ndId].t[ele_idx];
				sh_ed_nodes[idx].g[ele_idx] = dev_ed_nodes[ndId].g[ele_idx];
				sh_ed_nodes[idx].n[ele_idx] = dev_ed_nodes[ndId].n[ele_idx];
			}
		}
		__syncthreads();

		//assume: 0 <= cubeId_src && cubeId_src < cubes_num_src.x*cubes_num_src.y*cubes_num_src.z
		if (sh_ed_nodes_num > 0)
		{
			//process different sub blocks 
			#pragma unroll
			for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
			#pragma unroll
			for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
			#pragma unroll
			for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
			{
				//process one voxel: find neighboring ed nodes; warp it to target

				//grid id in the current block of the current cube
				int xId = threadIdx.x;
				int zId = xId / (8 * 8);
				xId -= zId*(8 * 8);
				int yId = xId / 8;
				xId -= yId * 8;

				xId += xOffset;
				yId += yOffset;
				zId += zOffset;

				//idx in the current cube
				int idx = zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
				float sdf_src = volume_src.data[data_offset_src + idx];
				float weight_src = volume_src.weights[data_offset_src + idx];
				ushort3 vxl_dst = make_ushort3(USHRT_MAX, USHRT_MAX, USHRT_MAX);
				if (sdf_src > -1.0f && weight_src > M_EPS)
				{
					cuda_vector_fixed<float, 3> V; // voxel location
					V[0] = cube_corner_src.x + xId*vxl_res_src;
					V[1] = cube_corner_src.y + yId*vxl_res_src;
					V[2] = cube_corner_src.z + zId*vxl_res_src;

					//find neighboring ed nodes
					float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
					int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						dists_sq[i] = 1.0e+10f;
						ngn_idx[i] = -1;
					}
					for (int i = 0; i < sh_ed_nodes_num; i++)
					{
						float dist_sq = dist_square<3>(V.data_block(), sh_ed_nodes[i].g.data_block());
						if (dist_sq < 4.0f*sigma_vxl_node_dist*sigma_vxl_node_dist + mu*mu)
						{
							if (dist_sq < dists_sq[0])
							{
								dists_sq[0] = dist_sq;
								ngn_idx[0] = i;
							}

							#pragma unroll
							for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
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
					float w_sum = 0;
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						if (ngn_idx[i] != -1)
						{
							dists_sq[i] = __expf(-dists_sq[i] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
							w_sum += dists_sq[i];
						}
					}
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						if (ngn_idx[i] != -1)  dists_sq[i] /= w_sum;
					}

					if (w_sum > 1.0e-4f)
					{
						//interpolate the alignment residual
						float align_residual = 0.0f;
						for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
						{
							int lc_ndIdx = ngn_idx[k];
							if (lc_ndIdx >= 0)
								align_residual = MAX(align_residual, sh_ed_node_align_residual[lc_ndIdx]);
						}

						if (align_residual < thres_align_residual)
						{
							//warp the voxel
							cuda_vector_fixed<float, 3> V_t(0.0f);
							#pragma unroll
							for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
							{
								int &ndIdx_k = ngn_idx[k];
								if (ndIdx_k >= 0)
								{
									DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
									cuda_matrix_fixed<float, 3, 3> const&A = nd.A;

									cuda_vector_fixed<float, 3> const&g = nd.g;
									cuda_vector_fixed<float, 3> const&t = nd.t;

									float &w_k = dists_sq[k];
									V_t += w_k*(A*(V - g) + g + t);
								}
							}
							V_t = sh_rigid_transf.R*V_t + sh_rigid_transf.T;

							//find the cube id at the target
							//vxl id in the regular grid
							vxl_dst.x = MAX(0, MIN(USHRT_MAX, (V_t[0] - cubes_offset_dst.x) / (cubes_num_dst.x*cube_res_dst) * USHRT_MAX));
							vxl_dst.y = MAX(0, MIN(USHRT_MAX, (V_t[1] - cubes_offset_dst.y) / (cubes_num_dst.y*cube_res_dst) * USHRT_MAX));
							vxl_dst.z = MAX(0, MIN(USHRT_MAX, (V_t[2] - cubes_offset_dst.z) / (cubes_num_dst.z*cube_res_dst) * USHRT_MAX));
						}
					}
				}
				dev_vxl_pos_t[data_offset_src + idx] = vxl_dst;
			}
		}
		else //if the whole cube is away from the surface, and no ed nodes is associated with it
		{
			for (int i = threadIdx.x; i < TWO_LEVEL_VOLUME_VXLS_PER_CUBE; i+=blockDim.x)
			{
				ushort3 vxl_dst = make_ushort3(USHRT_MAX, USHRT_MAX, USHRT_MAX);
				dev_vxl_pos_t[data_offset_src + i] = vxl_dst;
			}
		}
		__syncthreads();
	}
}


__global__
void warp_volume_Step2_vote_min_sdf_at_dst_kernel(float * __restrict__ dev_vxl_min_sdf_dst,
										    ushort3 const* __restrict__ dev_vxl_pos_t,
										    VolumeConstDataGPU volume_src,
											OccupcyCube const* __restrict__ dev_cubes_dst, int3 const* dev_cubes_num_dst, float3 const* dev_cubes_offset_dst,
										    float cube_res_src, float vxl_res_src,
										    float cube_res_dst, float vxl_res_dst, int vxls_per_cube, int cube_size_in_vxl)
{
	__shared__ int3 cubes_num_src;
	__shared__ int3 cubes_num_dst;

	__shared__ int cubeId_src;
	__shared__ int data_offset_src;

	const int count_occupied_cubes_src = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume_src.count_occu_cubes[0]);
	if (threadIdx.x == 0)
	{
		cubes_num_src = volume_src.cubes_num[0];
		cubes_num_dst = dev_cubes_num_dst[0];
	}

	for (int blkId = blockIdx.x; blkId < count_occupied_cubes_src; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId_src = volume_src.occupied_cube_ids[blkId];
			int xCubeId = cubeId_src;
			int zCubeId = xCubeId / (cubes_num_src.x*cubes_num_src.y);
			xCubeId -= zCubeId*cubes_num_src.x*cubes_num_src.y;
			int yCubeId = xCubeId / cubes_num_src.x;
			xCubeId -= yCubeId*cubes_num_src.x;

			data_offset_src = blkId * vxls_per_cube;
		}
		__syncthreads();

		if (0 <= cubeId_src && cubeId_src < cubes_num_src.x*cubes_num_src.y*cubes_num_src.z)
		{
			//process different sub blocks 
			#pragma unroll
			for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
			#pragma unroll
			for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
			#pragma unroll
			for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
			{
				//grid id in the current block of the current cube
				int xId = threadIdx.x;
				int zId = xId / (8 * 8);
				xId -= zId*(8 * 8);
				int yId = xId / 8;
				xId -= yId * 8;

				xId += xOffset;
				yId += yOffset;
				zId += zOffset;

				//idx in the current cube
				int idx = zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
				float sdf_src = volume_src.data[data_offset_src + idx];
				ushort3 vxl_t = dev_vxl_pos_t[data_offset_src + idx];
				if (sdf_src > -1.0f && 
					0 < vxl_t.x && vxl_t.x < USHRT_MAX &&
					0 < vxl_t.y && vxl_t.y < USHRT_MAX &&
					0 < vxl_t.z && vxl_t.z < USHRT_MAX)
				{	
					float xVxlId_dst_ = (float)(vxl_t.x) / USHRT_MAX * cubes_num_dst.x*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					float yVxlId_dst_ = (float)(vxl_t.y) / USHRT_MAX * cubes_num_dst.y*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					float zVxlId_dst_ = (float)(vxl_t.z) / USHRT_MAX * cubes_num_dst.z*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

					float const radius = 1.0f;
					for (int zId_dst = MAX(0, (int)ceilf(zVxlId_dst_ - radius)); zId_dst < zVxlId_dst_ + radius; zId_dst++)
					for (int yId_dst = MAX(0, (int)ceilf(yVxlId_dst_ - radius)); yId_dst < yVxlId_dst_ + radius; yId_dst++)
					for (int xId_dst = MAX(0, (int)ceilf(xVxlId_dst_ - radius)); xId_dst < xVxlId_dst_ + radius; xId_dst++)
					{
						float dist2 = (xId_dst - xVxlId_dst_)*(xId_dst - xVxlId_dst_) +
							(yId_dst - yVxlId_dst_)*(yId_dst - yVxlId_dst_) +
							(zId_dst - zVxlId_dst_)*(zId_dst - zVxlId_dst_);
						if (dist2 < radius*radius)
						{
							int xCubeId_dst = xId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							int yCubeId_dst = yId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							int zCubeId_dst = zId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							if (xCubeId_dst < cubes_num_dst.x &&//xCubeId_dst is already >=0 by defination
								yCubeId_dst < cubes_num_dst.y &&
								zCubeId_dst < cubes_num_dst.z)
							{
								int cubeId_dst = zCubeId_dst*cubes_num_dst.x*cubes_num_dst.y + yCubeId_dst*cubes_num_dst.x + xCubeId_dst;
								int data_offset_dst = dev_cubes_dst[cubeId_dst].offset*vxls_per_cube;
								if (data_offset_dst >= 0)
								{
									//vxl id
									int xId_dst0 = xId_dst - xCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int yId_dst0 = yId_dst - yCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int zId_dst0 = zId_dst - zCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

									int vxlIdx_dst = zId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + 
													yId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId_dst0;

									atomicAbsMin_FavorNeg(&dev_vxl_min_sdf_dst[data_offset_dst + vxlIdx_dst], sdf_src);
								}
							}
						}
					}
				}
			}
		}
		__syncthreads();
	}
}

__global__
void warp_volume_Step3_get_min_sdf_srcIdx_kernel(int* __restrict__ dev_vxl_min_sdf_srcIdx_dst,
										   float const* __restrict__ dev_vxl_min_sdf_dst,
										   ushort3 const* __restrict__ dev_vxl_pos_t,
										   VolumeConstDataGPU volume_src,
										   OccupcyCube const* __restrict__ dev_cubes_dst, int3 const* dev_cubes_num_dst, float3 const* dev_cubes_offset_dst,
										   float cube_res_src, float vxl_res_src,
										   float cube_res_dst, float vxl_res_dst, int vxls_per_cube, int cube_size_in_vxl)
{
	__shared__ int3 cubes_num_src;
	__shared__ int3 cubes_num_dst;

	__shared__ int cubeId_src;
	__shared__ int data_offset_src;

	const int count_occupied_cubes_src = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume_src.count_occu_cubes[0]);
	if (threadIdx.x == 0)
	{
		cubes_num_src = volume_src.cubes_num[0];
		cubes_num_dst = dev_cubes_num_dst[0];
	}

	for (int blkId = blockIdx.x; blkId < count_occupied_cubes_src; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId_src = volume_src.occupied_cube_ids[blkId];
			int xCubeId = cubeId_src;
			int zCubeId = xCubeId / (cubes_num_src.x*cubes_num_src.y);
			xCubeId -= zCubeId*cubes_num_src.x*cubes_num_src.y;
			int yCubeId = xCubeId / cubes_num_src.x;
			xCubeId -= yCubeId*cubes_num_src.x;

			data_offset_src = blkId * vxls_per_cube;
		}
		__syncthreads();

		if (0 <= cubeId_src && cubeId_src < cubes_num_src.x*cubes_num_src.y*cubes_num_src.z)
		{
			//process different sub blocks 
			#pragma unroll
			for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
			#pragma unroll
			for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
			#pragma unroll
			for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
			{
				//grid id in the current block of the current cube
				int xId = threadIdx.x;
				int zId = xId / (8 * 8);
				xId -= zId*(8 * 8);
				int yId = xId / 8;
				xId -= yId * 8;

				xId += xOffset;
				yId += yOffset;
				zId += zOffset;

				//idx in the current cube
				int idx = zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
				float sdf_src = volume_src.data[data_offset_src + idx];
				ushort3 vxl_t = dev_vxl_pos_t[data_offset_src + idx];
				if (sdf_src > -1.0f &&
					0 < vxl_t.x && vxl_t.x < USHRT_MAX &&
					0 < vxl_t.y && vxl_t.y < USHRT_MAX &&
					0 < vxl_t.z && vxl_t.z < USHRT_MAX)
				{
					float xVxlId_dst_ = (float)(vxl_t.x) / USHRT_MAX * cubes_num_dst.x*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					float yVxlId_dst_ = (float)(vxl_t.y) / USHRT_MAX * cubes_num_dst.y*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					float zVxlId_dst_ = (float)(vxl_t.z) / USHRT_MAX * cubes_num_dst.z*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

					float const radius = 1.0f;
					for (int zId_dst = MAX(0, (int)ceilf(zVxlId_dst_ - radius)); zId_dst < zVxlId_dst_ + radius; zId_dst++)
					for (int yId_dst = MAX(0, (int)ceilf(yVxlId_dst_ - radius)); yId_dst < yVxlId_dst_ + radius; yId_dst++)
					for (int xId_dst = MAX(0, (int)ceilf(xVxlId_dst_ - radius)); xId_dst < xVxlId_dst_ + radius; xId_dst++)
					{
						float dist2 = (xId_dst - xVxlId_dst_)*(xId_dst - xVxlId_dst_) +
							(yId_dst - yVxlId_dst_)*(yId_dst - yVxlId_dst_) +
							(zId_dst - zVxlId_dst_)*(zId_dst - zVxlId_dst_);
						if (dist2 < radius*radius)
						{
							int xCubeId_dst = xId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							int yCubeId_dst = yId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							int zCubeId_dst = zId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							if (xCubeId_dst < cubes_num_dst.x && //xCubeId_dst is already >=0 by defination
								yCubeId_dst < cubes_num_dst.y &&
								zCubeId_dst < cubes_num_dst.z)
							{
								int cubeId_dst = zCubeId_dst*cubes_num_dst.x*cubes_num_dst.y + yCubeId_dst*cubes_num_dst.x + xCubeId_dst;
								int data_offset_dst = dev_cubes_dst[cubeId_dst].offset*vxls_per_cube;
								if (data_offset_dst >= 0)
								{
									//vxl id
									int xId_dst0 = xId_dst - xCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int yId_dst0 = yId_dst - yCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
									int zId_dst0 = zId_dst - zCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

									int vxlIdx_dst = zId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + 
													yId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId_dst0;

									float sdf_min = dev_vxl_min_sdf_dst[data_offset_dst + vxlIdx_dst];

									if (fabsf(sdf_min - sdf_src) < M_EPS)
									{
										atomicExch(&(dev_vxl_min_sdf_srcIdx_dst[data_offset_dst + vxlIdx_dst]), data_offset_src + idx);
									}
								}
							}
						}
					}
				}
			}
		}
		__syncthreads();
	}
}

__global__
__launch_bounds__(512, 2)
void warp_volume_Step4_cast_sdf(ushort3 const* __restrict__ dev_vxl_pos_t,
								int const* __restrict__ dev_vxl_min_sdf_srcIdx_dst,
								VolumeConstDataGPU volume_src,
								VolumeDataGPU volume_dst, 
								float* __restrict__ dev_weights_interp, int* __restrict__ dev_counts_interp,
								float cube_res_src, float vxl_res_src,
								float cube_res_dst, float vxl_res_dst, 
								int vxls_per_cube, int cube_size_in_vxl)
{
	__shared__ int3 cubes_num_src;
	__shared__ int3 cubes_num_dst;

	__shared__ int cubeId_src;
	__shared__ int3 cubeId_3d_src;
	__shared__ int data_offset_src;

	const int count_occupied_cubes_src = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume_src.count_occu_cubes[0]);
	if (threadIdx.x == 0)
	{
		cubes_num_src = volume_src.cubes_num[0];
		cubes_num_dst = volume_dst.cubes_num[0];
	}

	for (int blkId = blockIdx.x; blkId < count_occupied_cubes_src; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId_src = volume_src.occupied_cube_ids[blkId];
			int xCubeId = cubeId_src;
			int zCubeId = xCubeId / (cubes_num_src.x*cubes_num_src.y);
			xCubeId -= zCubeId*cubes_num_src.x*cubes_num_src.y;
			int yCubeId = xCubeId / cubes_num_src.x;
			xCubeId -= yCubeId*cubes_num_src.x;
			cubeId_3d_src.x = xCubeId;
			cubeId_3d_src.y = yCubeId;
			cubeId_3d_src.z = zCubeId;

			data_offset_src = blkId * vxls_per_cube;
		}
		__syncthreads();

		if (0 <= cubeId_src && cubeId_src < cubes_num_src.x*cubes_num_src.y*cubes_num_src.z)
		{
			//process different sub blocks 
			#pragma unroll
			for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
			#pragma unroll
			for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
			#pragma unroll
			for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
			{
				//grid id in the current block of the current cube
				int lc_xId = threadIdx.x;
				int lc_zId = lc_xId / (8 * 8);
				lc_xId -= lc_zId*(8 * 8);
				int lc_yId = lc_xId / 8;
				lc_xId -= lc_yId * 8;

				lc_xId += xOffset;
				lc_yId += yOffset;
				lc_zId += zOffset;

				int xId_src = cubeId_3d_src.x * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_xId;
				int yId_src = cubeId_3d_src.y * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_yId;
				int zId_src = cubeId_3d_src.z * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_zId;

				//idx in the current cube
				int idx = lc_zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_xId;
				float sdf_src = volume_src.data[data_offset_src + idx];
				float weight_src = volume_src.weights[data_offset_src + idx];
				ushort3 vxl_t_ = dev_vxl_pos_t[data_offset_src + idx];
				int3 vxl_t;
				vxl_t.x = vxl_t_.x;
				vxl_t.y = vxl_t_.y;
				vxl_t.z = vxl_t_.z;
				if (sdf_src > -1.0f &&
					0 < vxl_t.x && vxl_t.x < USHRT_MAX &&
					0 < vxl_t.y && vxl_t.y < USHRT_MAX &&
					0 < vxl_t.z && vxl_t.z < USHRT_MAX)
				{
					float xVxlId_dst_ = float(vxl_t.x * cubes_num_dst.x*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL) / USHRT_MAX;
					float yVxlId_dst_ = float(vxl_t.y * cubes_num_dst.y*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL) / USHRT_MAX;
					float zVxlId_dst_ = float(vxl_t.z * cubes_num_dst.z*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL) / USHRT_MAX;

					float const radius = 1.6f;
					float const sigma2 = 0.55f*0.55f;
					const float thres_dist2_vxl = 1.3 * 1.3 * 3.0f;

					float dist2_vxl = 1.0e+10f;
					int xVxlId_dst = xVxlId_dst_ + 0.5f;
					int yVxlId_dst = yVxlId_dst_ + 0.5f;
					int zVxlId_dst = zVxlId_dst_ + 0.5f;
					int xCubeId_dst = xVxlId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					int yCubeId_dst = yVxlId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					int zCubeId_dst = zVxlId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					if (0 <= xCubeId_dst && xCubeId_dst < cubes_num_dst.x &&
						0 <= yCubeId_dst && yCubeId_dst < cubes_num_dst.y &&
						0 <= zCubeId_dst && zCubeId_dst < cubes_num_dst.z)
					{
						int cubeId_dst = zCubeId_dst*cubes_num_dst.x*cubes_num_dst.y + yCubeId_dst*cubes_num_dst.x + xCubeId_dst;
						int data_offset_dst = volume_dst.cubes[cubeId_dst].offset*vxls_per_cube;
						if (data_offset_dst >= 0)
						{
							xVxlId_dst -= xCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							yVxlId_dst -= yCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							zVxlId_dst -= zCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
							int vxlIdx_dst = zVxlId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + 
								yVxlId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xVxlId_dst;
							int vxl_idx_cc = dev_vxl_min_sdf_srcIdx_dst[data_offset_dst + vxlIdx_dst];
							if (vxl_idx_cc >= 0)
							{
								int cubeOffset_cc = vxl_idx_cc / vxls_per_cube;
								int lc_vxlIdx_cc = vxl_idx_cc % vxls_per_cube;

								int cubeId_cc = volume_src.occupied_cube_ids[cubeOffset_cc];
								int xCubeId_cc = cubeId_cc;
								int zCubeId_cc = xCubeId_cc / (cubes_num_src.x*cubes_num_src.y);
								xCubeId_cc -= zCubeId_cc*cubes_num_src.x*cubes_num_src.y;
								int yCubeId_cc = xCubeId_cc / cubes_num_src.x;
								xCubeId_cc -= yCubeId_cc*cubes_num_src.x;

								int lc_xId_cc = lc_vxlIdx_cc;
								int lc_zId_cc = lc_xId_cc / (TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL);
								lc_xId_cc -= lc_zId_cc*(TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL);
								int lc_yId_cc = lc_xId_cc / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
								lc_xId_cc -= lc_yId_cc * TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

								int xVxlId_cc = xCubeId_cc*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_xId_cc;
								int yVxlId_cc = yCubeId_cc*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_yId_cc;
								int zVxlId_cc = zCubeId_cc*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + lc_zId_cc;

								dist2_vxl = (xVxlId_cc - xId_src)*(xVxlId_cc - xId_src) +
									(yVxlId_cc - yId_src)*(yVxlId_cc - yId_src) +
									(zVxlId_cc - zId_src)*(zVxlId_cc - zId_src);
							}
						}
					}

					if (dist2_vxl < thres_dist2_vxl)
					{
						for (int zId_dst = MAX(0, (int)ceilf(zVxlId_dst_ - radius)); zId_dst < zVxlId_dst_ + radius; zId_dst++)
						for (int yId_dst = MAX(0, (int)ceilf(yVxlId_dst_ - radius)); yId_dst < yVxlId_dst_ + radius; yId_dst++)
						for (int xId_dst = MAX(0, (int)ceilf(xVxlId_dst_ - radius)); xId_dst < xVxlId_dst_ + radius; xId_dst++)
						{
							float dist2 = (xId_dst - xVxlId_dst_)*(xId_dst - xVxlId_dst_) +
								(yId_dst - yVxlId_dst_)*(yId_dst - yVxlId_dst_) +
								(zId_dst - zVxlId_dst_)*(zId_dst - zVxlId_dst_);
							if (dist2 < radius*radius)
							{
								float w = __expf(-dist2 / (2.0f*sigma2));// / w_sum;

								int xCubeId_dst = xId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
								int yCubeId_dst = yId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
								int zCubeId_dst = zId_dst / TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
								if (xCubeId_dst < cubes_num_dst.x && //xCubeId_dst is aldready >= 0 by defination
									yCubeId_dst < cubes_num_dst.y &&
									zCubeId_dst < cubes_num_dst.z)
								{
									int cubeId_dst = zCubeId_dst*cubes_num_dst.x*cubes_num_dst.y + yCubeId_dst*cubes_num_dst.x + xCubeId_dst;
									int data_offset_dst = volume_dst.cubes[cubeId_dst].offset*vxls_per_cube;
									if (data_offset_dst >= 0)
									{
										//vxl id
										int xId_dst0 = xId_dst - xCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
										int yId_dst0 = yId_dst - yCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
										int zId_dst0 = zId_dst - zCubeId_dst*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;

										int vxlIdx_dst = zId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + 
														yId_dst0*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId_dst0;

										float weight_new = weight_src*w;
										float sdf_new = sdf_src*w;

										atomicAdd(&(dev_weights_interp[data_offset_dst + vxlIdx_dst]), w);
										if (dist2 <= 1.0f)
											atomicAdd(&(dev_counts_interp[data_offset_dst + vxlIdx_dst]), 1);
										atomicAdd(&(volume_dst.data[data_offset_dst + vxlIdx_dst]), sdf_new);
										atomicAdd(&(volume_dst.weights[data_offset_dst + vxlIdx_dst]), weight_new);
									}
								}
							}
						}
					}
				}
			}
		}
		__syncthreads();
	}
}


void VolumetricFusionHelperCudaImpl::
warp_and_update_volume_f2f2(VolumeTwoLevelHierachy *volume_prev, VolumeTwoLevelHierachy *volume_curr,
							DeformGraphNodeCuda const* dev_ed_nodes, float const* dev_ed_nodes_align_residual,
							cuda::gpu_size_data ed_nodes_num_gpu,
							float sigma_vxl_node_dist,
							int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res, 
							RigidTransformCuda* dev_rigid_transf,
							float thres_align_residual)
{
	float cube_res = volume_prev->cube_res;
	float mu = volume_prev->mu;
	float ed_search_radius = sqrt((mu + sqrt(3.0)*cube_res / 2.0)*(mu + sqrt(3.0)*cube_res / 2.0) +
		(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)*(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)
		);
	int ed_nodes_num_per_cube_est = 2.0*(ed_search_radius*2.0 / ed_cube_res)*(ed_search_radius*2.0 / ed_cube_res);

	static cuda::PinnedMemory<int> count(0);
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_global_cube_ngns_count_sum, count.memory, sizeof(int)));

	//group the ed nodes for each occupied cube
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX + threads_per_block - 1) / threads_per_block;
	warp_volume_f2f_kernel_ngns<<<blocks_per_grid, threads_per_block>>>(dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
																		volume_prev->buf_occupied_cube_ids, volume_prev->gpu_cubes_occpied_count.dev_ptr,
																		volume_prev->ptr_cubes_offset,
																		volume_prev->ptr_cubes_num,
																		volume_prev->cube_res,
																		dev_ed_cubes_dims, dev_ed_cubes_offsets, ed_cube_res, ed_search_radius);
	m_checkCudaErrors();

	VolumeConstDataGPU volume_src(*volume_prev);
	VolumeDataGPU volume_dst(*volume_curr);

	threads_per_block = 512;
	blocks_per_grid = 1024;

	//step 1: warp voxels
	warp_volume_Step1_save_vxl_pos_kernel<<<blocks_per_grid, threads_per_block>>>(dev_vxl_pos_t_, volume_src, 
										  volume_dst.cubes_num, volume_dst.cubes_offset,
										  volume_prev->cube_res, volume_prev->vxl_res,
										  volume_curr->cube_res, volume_curr->vxl_res,
										  volume_prev->vxls_per_cube, volume_prev->cube_size_in_voxel,
										  dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
										  dev_ed_nodes, dev_ed_nodes_align_residual, ed_nodes_num_gpu.dev_ptr, sigma_vxl_node_dist,
										  dev_rigid_transf, thres_align_residual, volume_prev->mu);
	m_checkCudaErrors();

	//step 2: find min sdf at each dst vxl
	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(dev_vxl_min_sdf_dst_, volume_dst.count_occu_cubes, 
								volume_curr->vxls_per_cube, 2.0f);
	m_checkCudaErrors();
	warp_volume_Step2_vote_min_sdf_at_dst_kernel<<<blocks_per_grid, threads_per_block>>>(dev_vxl_min_sdf_dst_, dev_vxl_pos_t_, 
		volume_src, 
		volume_dst.cubes,
		volume_dst.cubes_num, volume_dst.cubes_offset,
		volume_prev->cube_res, volume_prev->vxl_res,
		volume_curr->cube_res, volume_curr->vxl_res,
		volume_prev->vxls_per_cube, volume_prev->cube_size_in_voxel);
	m_checkCudaErrors();

	//step 3: find the src voxel voting for the min sdf
	init_two_level_hierachy_volume_data_kernel<int><<<blocks_per_grid, threads_per_block>>>(dev_vxl_min_sdf_srcIdx_dst_, volume_dst.count_occu_cubes, 
														volume_curr->vxls_per_cube, -1);
	warp_volume_Step3_get_min_sdf_srcIdx_kernel<<<blocks_per_grid, threads_per_block>>>(dev_vxl_min_sdf_srcIdx_dst_, dev_vxl_min_sdf_dst_, dev_vxl_pos_t_,
		volume_src, 
		volume_dst.cubes,
		volume_dst.cubes_num, volume_dst.cubes_offset,
		volume_prev->cube_res, volume_prev->vxl_res,
		volume_curr->cube_res, volume_curr->vxl_res,
		volume_prev->vxls_per_cube, volume_prev->cube_size_in_voxel);
	m_checkCudaErrors();

	//step 4: cast sdf value
	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(dev_weights_interp_, volume_dst.count_occu_cubes, 
														volume_curr->vxls_per_cube, 0.0f);
	init_two_level_hierachy_volume_data_kernel<int><<<blocks_per_grid, threads_per_block>>>(dev_counts_interp_, volume_dst.count_occu_cubes,
											volume_curr->vxls_per_cube, 0);
	warp_volume_Step4_cast_sdf<<<blocks_per_grid, threads_per_block>>>(dev_vxl_pos_t_, dev_vxl_min_sdf_srcIdx_dst_, 
							   volume_src, volume_dst, dev_weights_interp_, dev_counts_interp_,
							   volume_prev->cube_res, volume_prev->vxl_res,
							   volume_curr->cube_res, volume_curr->vxl_res,
							   volume_prev->vxls_per_cube, volume_prev->cube_size_in_voxel);
	m_checkCudaErrors();


	correct_volume_weights_kernel<<<blocks_per_grid, threads_per_block>>>(volume_curr->data, volume_curr->weights, volume_curr->colors, 
																		dev_weights_interp_, dev_counts_interp_,
																		volume_curr->vxls_per_cube, volume_curr->gpu_cubes_occpied_count.dev_ptr);
	m_checkCudaErrors();
}

void VolumetricFusionHelperCudaImpl::
warp_and_update_volume_f2f(VolumeTwoLevelHierachy *volume_prev, VolumeTwoLevelHierachy *volume_curr, 
						   DeformGraphNodeCuda const* dev_ed_nodes, float const* dev_ed_nodes_align_residual, 
						   cuda::gpu_size_data ed_nodes_num_gpu, 
						   float sigma_vxl_node_dist,
						   int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res, RigidTransformCuda* dev_rigid_transf,
						   float thres_align_residual)
{
	float cube_res = volume_prev->cube_res;
	float mu = volume_prev->mu;
	float ed_search_radius = sqrt((mu + sqrt(3.0)*cube_res / 2.0)*(mu + sqrt(3.0)*cube_res / 2.0) +
		(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)*(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)
		);
	int ed_nodes_num_per_cube_est = 2.0*(ed_search_radius*2.0 / ed_cube_res)*(ed_search_radius*2.0 / ed_cube_res);

	LOGGER()->debug("ed nodes num per cube: %d", ed_nodes_num_per_cube_est);

	static cuda::PinnedMemory<int> count(0);
	checkCudaErrors(cudaMemcpyToSymbolAsync(dev_global_cube_ngns_count_sum, count.memory, sizeof(int)));

	//group the ed nodes for each occupied cube
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX + threads_per_block - 1) / threads_per_block;
	warp_volume_f2f_kernel_ngns<<<blocks_per_grid, threads_per_block>>>(dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
											volume_prev->buf_occupied_cube_ids, volume_prev->gpu_cubes_occpied_count.dev_ptr,
											volume_prev->ptr_cubes_offset,
											volume_prev->ptr_cubes_num,
											volume_prev->cube_res,
											dev_ed_cubes_dims, dev_ed_cubes_offsets, ed_cube_res, ed_search_radius);

	m_checkCudaErrors();
	
	VolumeConstDataGPU volume_src(*volume_prev);
	VolumeDataGPU volume_dst(*volume_curr);
	threads_per_block = 512;
	blocks_per_grid = 1024;
	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(dev_weights_interp_, volume_dst.count_occu_cubes, 
														volume_curr->vxls_per_cube, 0.0f);
	init_two_level_hierachy_volume_data_kernel<int><<<blocks_per_grid, threads_per_block>>>(dev_counts_interp_, volume_dst.count_occu_cubes,
											volume_curr->vxls_per_cube, 0.0f);
	warp_and_update_volume_f2f_kernel<<<blocks_per_grid, threads_per_block>>>(volume_src, volume_dst,  dev_weights_interp_, dev_counts_interp_,
																			  volume_prev->cube_res, volume_prev->vxl_res, 
																			  volume_curr->cube_res, volume_curr->vxl_res,
																			  volume_prev->vxls_per_cube, volume_prev->cube_size_in_voxel,
																			  dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
																			  dev_ed_nodes, dev_ed_nodes_align_residual, ed_nodes_num_gpu.dev_ptr, sigma_vxl_node_dist,
																			  dev_rigid_transf, volume_prev->mu, thres_align_residual);
	m_checkCudaErrors()

	correct_volume_weights_kernel<<<blocks_per_grid, threads_per_block>>>(volume_curr->data, volume_curr->weights, volume_curr->colors, 
																		dev_weights_interp_, dev_counts_interp_,
																		volume_curr->vxls_per_cube, volume_curr->gpu_cubes_occpied_count.dev_ptr);
	m_checkCudaErrors();
}


#define VT_DIM 9 // same as in volumetric_fusion.h
//comparing using a functor since we need the state of the vertex locations...
struct triangleCompare
{
	VolumetricFusionHelperCudaImpl& mCudaImp;
	float * vts_t;
	triangleCompare(VolumetricFusionHelperCudaImpl& cudaImp) : mCudaImp(cudaImp)
	{
		vts_t = mCudaImp.dev_vts_t();
	}

	__device__ int mortonCode(int a)
	{
		int x = a;
		x = (x | (x << 16)) & 0x030000FF;
		x = (x | (x << 8)) & 0x0300F00F;
		x = (x | (x << 4)) & 0x030C30C3;
		x = (x | (x << 2)) & 0x09249249;
		return x;
	}

	__device__ bool operator () (const int3& a, const int3& b)
	{
		const int aPos = a.x * VT_DIM;
		const int bPos = b.x * VT_DIM;

		thrust::device_ptr<float> vtsPointer = thrust::device_pointer_cast(vts_t);
																
		int negOffset = 200;
		int  aX = __float2int_rd(vtsPointer[aPos + 0] ) + negOffset ;
		int  aY = __float2int_rd(vtsPointer[aPos + 1] ) + negOffset ;
		int  aZ = __float2int_rd(vtsPointer[aPos + 2] ) + negOffset ;
				 									   	  
		int  bX = __float2int_rd(vtsPointer[bPos + 0] ) + negOffset ;
		int  bY = __float2int_rd(vtsPointer[bPos + 1] ) + negOffset ;
		int  bZ = __float2int_rd(vtsPointer[bPos + 2] ) + negOffset ;

		int mortonA = mortonCode(aX) | (mortonCode(aY) <<1) | (mortonCode(aZ) <<2);
		int mortonB = mortonCode(bX) | (mortonCode(bY) << 1) | (mortonCode(bZ) << 2);
		
		return mortonA < mortonB;
		if (aY != bY)
			return aY < bY;
		if (aX != bX)
			return aX < bX;
		return aZ < bZ;
	}
};

void VolumetricFusionHelperCudaImpl::sortTriangles()
{
	{

#ifdef USING_THRUST
		int triCount = tris_num_gpu().sync_read(); //need to sync?
		thrust::device_ptr<int3> triBuf= thrust::device_pointer_cast(this->dev_triangles_buf_);

		thrust::sort(triBuf, triBuf + triCount, triangleCompare(*this));
#endif
		//thrust::sort
	}
}

void VolumetricFusionHelperCudaImpl::simplifyTriangles(bool doQuads)
{	
	PERF_SCOPE("simplifyTriangles");
	const int triCount = tris_num_gpu().sync_read(); //need to sync?
	const int vertCount = vts_t_num_gpu().sync_read();

	if (triCount > meshSimplifier->m_maxTriangles || vertCount > meshSimplifier->m_maxVertices)
	{
		printf("VolumetricFusionHelperCudaImpl::simplifyTriangles >> Out-of-memory error, Tri: %d (max=%d), Vert: %d (max=%d)\r\n", triCount, meshSimplifier->m_maxTriangles, vertCount, meshSimplifier->m_maxVertices);
	}

	const float cMaxError = FLT_MAX; // Threshold for simplification, setting this to FLT_MAX effectively simplify as much as possible.
	const int cMaxTriangles = 30000; // Keep simplifying until the number of triangle is equal or below this value (note that one simplification pass is always performed).
	const int cMinTrianglePerComponent = 1000; // Only relevant if doQuads is true, setting this to zero effectively disable the removal of small disjoint components.	
	const int cNormalOffset = 12;

	 if (vertCount > 0 && triCount > 0)
	{
		//cpu pinned memory
		//copy from gpu to cpu so we can simplify
		{
			PERF_SCOPE("GPU->CPU");
			checkCudaErrors(cudaGetLastError());
			cudaMemcpy(meshSimplifier->mPinnedVerts, this->dev_vts_t(), (vertCount)* vt_dim() * sizeof(float), cudaMemcpyDeviceToHost);
			checkCudaErrors(cudaGetLastError());
			cudaMemcpy(meshSimplifier->mPinnedInds, this->dev_triangles_buf_, (triCount)* sizeof(int) * 3, cudaMemcpyDeviceToHost);
			checkCudaErrors(cudaGetLastError());
		}
		auto vertices_sp = McMeshProcessing::StridedPointer<uchar1>(meshSimplifier->mPinnedVerts, vt_dim() * sizeof(float));
		auto verticesPositions_sp = McMeshProcessing::StridedPointer<float3>(meshSimplifier->mPinnedVerts, vt_dim() * sizeof(float));
		auto verticesNormals_sp = McMeshProcessing::StridedPointer<float3>(meshSimplifier->mPinnedVerts + cNormalOffset, vt_dim() * sizeof(float));
		auto triangles_sp = McMeshProcessing::StridedPointer<int3>(meshSimplifier->mPinnedInds, sizeof(int3));


		int newTriCount = 0, newVertCount = 0;
		{
			PERF_SCOPE("meshSimplifier");
			// Simplify 
			for (newTriCount = triCount;;)
			{
				const int prevTriCount = newTriCount;
				newTriCount = meshSimplifier->simplify(cMaxError, vertCount, newTriCount, verticesPositions_sp, verticesNormals_sp, triangles_sp, cMaxTriangles);
				if (newTriCount == prevTriCount) break;
			}
			
			// Compact vertices.
			newVertCount = meshSimplifier->removeUnreferencedVertices(newTriCount, vertCount, triangles_sp, vertices_sp);

			if (doQuads)
			{
				// Quads processing.
				// - Convert to quads and decimate small disconnected components.
				// - Sort quads.
				// - Convert quads back to triangles (temporary work-around to avoid having to change the encoding and uv coordinates generation for the time being).
				{
					// Convert to quads.
					std::vector<int4> tempQuads(triCount);
					auto quads_sp = McMeshProcessing::StridedPointer<int4>(tempQuads.data(), sizeof(int4));
					const int numQuads = meshSimplifier->trianglesToQuads(newTriCount, triangles_sp, verticesPositions_sp, quads_sp, cMinTrianglePerComponent);

					// Sort quads.
					meshSimplifier->mortonSortQuads(newVertCount, numQuads, quads_sp, verticesPositions_sp);

					// Convert back to triangles.
					newTriCount = meshSimplifier->quadsToTriangles(numQuads, quads_sp, triangles_sp);
				}
			}
			else
			{
				// Just sort triangles.
				meshSimplifier->mortonSortTriangles(newVertCount, newTriCount, triangles_sp, verticesPositions_sp);
			}
		}	

	//feed back to gpu
		{
			PERF_SCOPE("CPU->GPU");
			checkCudaErrors(cudaMemcpy(this->dev_vts_t(), meshSimplifier->mPinnedVerts, (newVertCount)* vt_dim() * sizeof(float), cudaMemcpyHostToDevice));
			checkCudaErrors(cudaMemcpy(vts_t_num_gpu().dev_ptr, &newVertCount, sizeof(int), cudaMemcpyHostToDevice));


			checkCudaErrors(cudaMemcpy(this->dev_triangles_buf_, meshSimplifier->mPinnedInds, (newTriCount)* sizeof(int) * 3, cudaMemcpyHostToDevice));
			checkCudaErrors(cudaMemcpy(tris_num_gpu().dev_ptr, &newTriCount, sizeof(int), cudaMemcpyHostToDevice));
		}

	}
	else
	{
		LOGGER()->info("Simplifying emptiness %d %d", vertCount, triCount);
	}
}



}
