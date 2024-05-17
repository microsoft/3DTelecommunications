#include "utility.h"

namespace VolumetricFusionCuda{

//512 threads per cuda block
//one sparse cube per cuda block
//each cuda block handles 8x8x8 grid within for-loop, there might be 16x16x16 grid
__global__
void marching_cubes_kernel( VolumeConstDataGPU volume,																									
							float cube_res, int cube_size_in_voxel, float vxl_res, int vxls_per_cube,
							float* __restrict__ dev_vts_buf, int stride, int vt_buf_size,
							int* __restrict__ dev_global_vts_count)
{

	__shared__ float sh_volume[9 * 9 * 9];
	__shared__ float3 sh_grad[9 * 9 * 9];
	__shared__ uchar4 sh_color[9 * 9 * 9];

	__shared__ int shPtCount;

	__shared__ int cubeId;
	__shared__ float x_corner; //cube corner
	__shared__ float y_corner;
	__shared__ float z_corner;
	__shared__ int3 cubes_num;
	__shared__ float3 cubes_offset;

	if (threadIdx.x == 0)
	{
		cubes_num = volume.cubes_num[0];
		cubes_offset = volume.cubes_offset[0];
	}

	int const count_occupied_cube = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);
	for (int blkId = blockIdx.x; blkId < count_occupied_cube; blkId += gridDim.x)
	{
		if (threadIdx.x == 0)
		{
			cubeId = volume.occupied_cube_ids[blkId];

			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
			xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
			int yCubeId = xCubeId / cubes_num.x;
			xCubeId -= yCubeId*cubes_num.x;

			//corner of the cube
			x_corner = xCubeId*cube_res + cubes_offset.x;
			y_corner = yCubeId*cube_res + cubes_offset.y;
			z_corner = zCubeId*cube_res + cubes_offset.z;
		}
		__syncthreads();

		//process different sub blocks 
		#pragma unroll
		for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
		#pragma unroll
		for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
		#pragma unroll
		for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
		{
			if (threadIdx.x == 0)
				shPtCount = 0; //no need to syncthreads. it is synced later

			//load volume data to shared memory
			for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
			{
				//compute 3D idx for the source
				int xId = idx; //original 3d index on sh_volume
				int zId = idx / (9 * 9);
				xId -= zId * 9 * 9;
				int yId = xId / 9;
				xId -= yId * 9;

				xId += xOffset; //offset 
				yId += yOffset;
				zId += zOffset;

				if (xId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					yId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					zId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
				{
					int pos = vxls_per_cube*blkId +
						zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
						yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
					float weight = volume.weights[pos];
					sh_color[idx] = volume.colors[pos];
					if (weight > THRES_WEIGHT_MATCHING_CUBES)
						sh_volume[idx] = volume.data[pos];
					else
						sh_volume[idx] = -2.0f;
				}
				else
				{
					int cubeId_src = cubeId;
					if (xId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += 1;
						xId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (yId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x;
						yId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (zId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x*cubes_num.y;
						zId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}

					if (cubeId_src >= cubes_num.x*cubes_num.y*cubes_num.z)
					{
						sh_volume[idx] = -2.0f;
						sh_color[idx] = make_uchar4(0, 0, 0, 0);
					}
					else
					{
						int offset = volume.cubes[cubeId_src].offset;

						if (offset < 0){
							sh_volume[idx] = -2.0f;
							sh_color[idx] = make_uchar4(0, 0, 0, 0);
						}
						else 
						{
							int pos = vxls_per_cube*offset +
								zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
								yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
							float weight = volume.weights[pos];
							sh_color[idx] = volume.colors[pos];
							if (weight > THRES_WEIGHT_MATCHING_CUBES)
								sh_volume[idx] = volume.data[pos];
							else
								sh_volume[idx] = -2.0f;
						}
					}
				}
			}
			__syncthreads();			

			//grid id
			int xId = threadIdx.x;
			int zId = xId / (8 * 8);
			xId -= zId * 8 * 8;
			int yId = xId / 8;
			xId -= yId * 8;

			int idx_sh = zId * 9 * 9 + yId * 9 + xId; //idx in shared memory
			float val_cur = sh_volume[idx_sh];
			float vals[3];
			vals[0] = sh_volume[idx_sh + 1];
			vals[1] = sh_volume[idx_sh + 9];
			vals[2] = sh_volume[idx_sh + 81];
			int lcPtOffset = 0;
			if (val_cur > -1.0f && val_cur < 1.0f)
			{
				int count = 0;
				for (int i = 0; i < 3; i++)
				if (vals[i]>-1.0f && vals[i] < 1.0f && val_cur*vals[i] <= 0)
					count++;
				lcPtOffset = atomicAdd(&shPtCount, count);
			}
			__syncthreads();


			if (shPtCount > 0)
			{
				if (threadIdx.x == 0)
				{
					shPtCount = atomicAdd(dev_global_vts_count, shPtCount);
				}
				__syncthreads();


				//compute the gradient
				for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
				{
					float val_1 = sh_volume[idx];
					float val_0, val_2;
					int bValidComputation = 0;
					if (-1.0f < val_1 && val_1 < 1.0f)
					{
						bValidComputation = 1;

						int lc_xId = idx;
						int lc_zId = idx / (9 * 9);
						lc_xId -= lc_zId * 9 * 9;
						int lc_yId = lc_xId / 9;
						lc_xId -= lc_yId * 9;

						//compute dx
						val_0 = lc_xId > 0 ? sh_volume[idx - 1] : -2.0f;
						val_2 = lc_xId < 8 ? sh_volume[idx + 1] : -2.0f;
						if (val_0 > -1.0f && val_2 > -1.0f)
							sh_grad[idx].x = (val_0 - val_2) / 2.0f;
						else if (val_0 > -1.0f && val_2 <= -1.0f)
							sh_grad[idx].x = val_0 - val_1;
						else if (val_0 <= -1.0f && val_2 > -1.0f)
							sh_grad[idx].x = val_1 - val_2;
						else
							bValidComputation = 0;

						//compute dy
						val_0 = lc_yId > 0 ? sh_volume[idx - 9] : -2.0f;
						val_2 = lc_yId < 8 ? sh_volume[idx + 9] : -2.0f;
						if (val_0 > -1.0f && val_2 > -1.0f)
							sh_grad[idx].y = (val_0 - val_2) / 2.0f;
						else if (val_0 > -1.0f && val_2 <= -1.0f)
							sh_grad[idx].y = val_0 - val_1;
						else if (val_0 <= -1.0f && val_2 > -1.0f)
							sh_grad[idx].y = val_1 - val_2;
						else
							bValidComputation = 0;

						//compute dz
						val_0 = lc_zId > 0 ? sh_volume[idx - 81] : -2.0f;
						val_2 = lc_zId < 8 ? sh_volume[idx + 81] : -2.0f;
						if (val_0 > -1.0f && val_2 > -1.0f)
							sh_grad[idx].z = (val_0 - val_2) / 2.0f;
						else if (val_0 > -1.0f && val_2 <= -1.0f)
							sh_grad[idx].z = val_0 - val_1;
						else if (val_0 <= -1.0f && val_2 > -1.0f)
							sh_grad[idx].z = val_1 - val_2;
						else
							bValidComputation = 0;
					}

					if (bValidComputation == 0)
					{
						sh_grad[idx].x = 0.0;
						sh_grad[idx].y = 0.0;
						sh_grad[idx].z = 0.0;
					}
				}
				__syncthreads();

				if (val_cur > -1.0f && val_cur < 1.0f)
				{
					float3 pt;
					pt.x = (xId + xOffset)*vxl_res + x_corner;
					pt.y = (yId + yOffset)*vxl_res + y_corner;
					pt.z = (zId + zOffset)*vxl_res + z_corner;

					lcPtOffset += shPtCount;
					if (vals[0]>-1.0f && vals[0] < 1.0f && val_cur*vals[0] <= 0)
					{
						int lcCount = lcPtOffset++;
						if (lcCount < vt_buf_size)
						{
							float mu = 0.5f;
							if (fabsf(val_cur - vals[0]) > M_EPS)
								mu = val_cur / (val_cur - vals[0]);
							dev_vts_buf[stride * lcCount] = pt.x + mu * vxl_res;
							dev_vts_buf[stride * lcCount + 1] = pt.y;
							dev_vts_buf[stride * lcCount + 2] = pt.z;

							float3 n;
							n.x = sh_grad[idx_sh].x *(1.0f - mu) + sh_grad[idx_sh + 1].x * mu;
							n.y = sh_grad[idx_sh].y *(1.0f - mu) + sh_grad[idx_sh + 1].y * mu;
							n.z = sh_grad[idx_sh].z *(1.0f - mu) + sh_grad[idx_sh + 1].z * mu;
							normalize(n);
							dev_vts_buf[stride * lcCount + 3] = n.x;
							dev_vts_buf[stride * lcCount + 4] = n.y;
							dev_vts_buf[stride * lcCount + 5] = n.z;

							float3 clr;
							clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 1].x*mu;
							clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 1].y*mu;
							clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 1].z*mu;
							clr.x /= 255.0f;
							clr.y /= 255.0f;
							clr.z /= 255.0f;
							dev_vts_buf[stride*lcCount + 6] = clr.x;
							dev_vts_buf[stride*lcCount + 7] = clr.y;
							dev_vts_buf[stride*lcCount + 8] = clr.z;
						}
					}

					if (vals[1] > -1.0f && vals[1] < 1.0f && val_cur*vals[1] <= 0)
					{
						int lcCount = lcPtOffset++;
						if (lcCount < vt_buf_size)
						{
							float mu = 0.5f;
							if (fabsf(val_cur - vals[1]) > M_EPS)
								mu = val_cur / (val_cur - vals[1]);
							dev_vts_buf[stride * lcCount] = pt.x;
							dev_vts_buf[stride * lcCount + 1] = pt.y + mu * vxl_res;
							dev_vts_buf[stride * lcCount + 2] = pt.z;

							float3 n;
							n.x = sh_grad[idx_sh].x *(1.0f - mu) + sh_grad[idx_sh + 9].x * mu;
							n.y = sh_grad[idx_sh].y *(1.0f - mu) + sh_grad[idx_sh + 9].y * mu;
							n.z = sh_grad[idx_sh].z *(1.0f - mu) + sh_grad[idx_sh + 9].z * mu;
							normalize(n);
							dev_vts_buf[stride * lcCount + 3] = n.x;
							dev_vts_buf[stride * lcCount + 4] = n.y;
							dev_vts_buf[stride * lcCount + 5] = n.z;

							float3 clr;
							clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 9].x*mu;
							clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 9].y*mu;
							clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 9].z*mu;
							clr.x /= 255.0f;
							clr.y /= 255.0f;
							clr.z /= 255.0f;
							dev_vts_buf[stride*lcCount + 6] = clr.x;
							dev_vts_buf[stride*lcCount + 7] = clr.y;
							dev_vts_buf[stride*lcCount + 8] = clr.z;
						}
					}

					if (vals[2] > -1.0f && vals[2] < 1.0f && val_cur*vals[2] <= 0)
					{
						int lcCount = lcPtOffset++;
						if (lcCount < vt_buf_size)
						{
							float mu = 0.5f;
							if (fabsf(val_cur - vals[2]) > M_EPS)
								mu = val_cur / (val_cur - vals[2]);
							dev_vts_buf[stride * lcCount] = pt.x;
							dev_vts_buf[stride * lcCount + 1] = pt.y;
							dev_vts_buf[stride * lcCount + 2] = pt.z + mu * vxl_res;

							float3 n;
							n.x = sh_grad[idx_sh].x *(1.0f - mu) + sh_grad[idx_sh + 81].x * mu;
							n.y = sh_grad[idx_sh].y *(1.0f - mu) + sh_grad[idx_sh + 81].y * mu;
							n.z = sh_grad[idx_sh].z *(1.0f - mu) + sh_grad[idx_sh + 81].z * mu;
							normalize(n);
							dev_vts_buf[stride * lcCount + 3] = n.x;
							dev_vts_buf[stride * lcCount + 4] = n.y;
							dev_vts_buf[stride * lcCount + 5] = n.z;

							float3 clr;
							clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 81].x*mu;
							clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 81].y*mu;
							clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 81].z*mu;
							clr.x /= 255.0f;
							clr.y /= 255.0f;
							clr.z /= 255.0f;
							dev_vts_buf[stride*lcCount + 6] = clr.x;
							dev_vts_buf[stride*lcCount + 7] = clr.y;
							dev_vts_buf[stride*lcCount + 8] = clr.z;
						}
					}
				}
				__syncthreads();
			}

		}//end of for-subblocks
	}
}

__global__ void convert_to_half_floats_kernel(const float* __restrict__ old_mesh, short* __restrict__ mesh, const int* __restrict__ n)
{
	const int KEEP = 6;
	const int STRIDE = 9;
	int i = blockIdx.x * blockDim.x + threadIdx.x; // vertex
	if (i < *n)
	{
		int old_idx = i * STRIDE;
		int new_idx = i * KEEP;

		for (int k = 0; k < KEEP; k++)
			mesh[new_idx + k] = __half_as_ushort(__float2half_rn(old_mesh[old_idx + k]));
	}
}

void VolumetricFusionHelperCudaImpl::
convert_to_half_floats(float* old_mesh, short* mesh, int* count, int max_count)
{
	const int nthreads = 256;
	const int nblocks = (max_count + nthreads - 1) / nthreads;

	convert_to_half_floats_kernel << <nblocks, nthreads >> >(old_mesh, mesh, count);
	checkCudaErrors(cudaGetLastError());
}


void VolumetricFusionHelperCudaImpl::
marching_cubes(VolumeTwoLevelHierachy *volume, float* dev_vts_buf, cuda::gpu_size_data vts_num, int vts_dim)
{
	checkCudaErrors(cudaMemsetAsync(vts_num.dev_ptr, 0, sizeof(int)));
	
	int threads_per_block = 512;
	int blocks_per_grid = 2048;
	VolumeConstDataGPU volume_gpu(*volume);	
	marching_cubes_kernel<<<blocks_per_grid, threads_per_block>>>(volume_gpu, 
					volume->cube_res, volume->cube_size_in_voxel, volume->vxl_res, volume->vxls_per_cube, 
					dev_vts_buf, vts_dim, vts_num.max_size, vts_num.dev_ptr);
	m_checkCudaErrors();

	// This check is necessary because .sync_read() performs implicit synchronization and we don't want
	// that to be done outside debug mode
	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		LOGGER()->debug("<<<<<<<<<<<<<< marching cubes: vts_num = %d", vts_num.sync_read());
	}

	vts_num.cap_gpu_size(vts_num.max_size);
}





}