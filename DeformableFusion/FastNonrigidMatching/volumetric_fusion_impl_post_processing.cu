namespace VolumetricFusionCuda{

__global__
void fused_warped_model_with_curr(VolumeDataGPU volume_model, VolumeConstDataGPU volume_curr,
	float cube_res_src, float vxl_res_src,
	float cube_res_dst, float vxl_res_dst,
	int vxls_per_cube, int cube_size_in_vxl)
{}


__global__
void remove_extra_free_space()
{}

#define VOLUME_BLUR_RADIUS 3
__global__
void volume_filter_kernel(VolumeConstDataGPU volume, float* dev_vxl_data_f, int vxls_per_cube, int cube_size_in_vxl, float thres_weight)
{
#define VXLS_DIM_IN_SH (VOLUME_BLUR_RADIUS*2+TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
#define VSLS_NUM_IN_SH (VXLS_DIM_IN_SH*VXLS_DIM_IN_SH*VXLS_DIM_IN_SH)
	__shared__ float sh_volume[VSLS_NUM_IN_SH];

	int const count_occupied_cube = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);
	for (int blkId = blockIdx.x; blkId < count_occupied_cube; blkId += gridDim.x)
	{
		const int3 cubes_num = volume.cubes_num[0];
		int cubeId = volume.occupied_cube_ids[blkId];

		int xCubeId = cubeId;
		int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
		xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
		int yCubeId = xCubeId / cubes_num.x;
		xCubeId -= yCubeId*cubes_num.x;

		//load data to sh memory
		//process different sub blocks 
		#pragma unroll
		for (int xOffset = 0; xOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; xOffset += 8)
		#pragma unroll
		for (int yOffset = 0; yOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; yOffset += 8)
		#pragma unroll
		for (int zOffset = 0; zOffset < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL; zOffset += 8)
		{
			//load volume data into shared memory
			for (int idx = threadIdx.x; idx < VSLS_NUM_IN_SH; idx += blockDim.x)
			{
				//compute 3D idx w.r.t. the current cube corner
				int xId = idx; //original 3d index on sh_volume
				int zId = idx / (VXLS_DIM_IN_SH * VXLS_DIM_IN_SH);
				xId -= zId * VXLS_DIM_IN_SH * VXLS_DIM_IN_SH;
				int yId = xId / VXLS_DIM_IN_SH;
				xId -= yId * VXLS_DIM_IN_SH;

				xId += xOffset - VOLUME_BLUR_RADIUS;
				yId += yOffset - VOLUME_BLUR_RADIUS;
				zId += zOffset - VOLUME_BLUR_RADIUS;

				if (0 <= xId && xId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					0 <= yId && yId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					0 <= zId && zId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
				{
					int pos = vxls_per_cube*blkId +
						zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
						yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
					float weight = volume.weights[pos];
					if (weight > 0.0f)
						sh_volume[idx] = volume.data[pos];
					else
						sh_volume[idx] = -2.0f;
				}
				else
				{
					int cubeId_src = cubeId;
					if (xId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL){
						cubeId_src += 1;
						xId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					} else if (xId < 0){
						cubeId_src -= 1;
						xId += TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (yId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL){
						cubeId_src += cubes_num.x;
						yId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					} else if (yId < 0){
						cubeId_src -= cubes_num.x;
						yId += TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (zId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL){
						cubeId_src += cubes_num.x*cubes_num.y;
						zId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					} else if (zId < 0){
						cubeId_src -= cubes_num.x*cubes_num.y;
						zId += TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}

					if (cubeId_src < 0 || cubeId_src >= cubes_num.x*cubes_num.y*cubes_num.z)
					{
						sh_volume[idx] = -2.0f;
					}
					else
					{
						int offset = volume.cubes[cubeId_src].offset;
						if (offset < 0){
							sh_volume[idx] = -2.0f;
						}
						else{
							int pos = vxls_per_cube*offset +
								zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
								yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
							float weight = volume.weights[pos];
							if (weight > 0.0f)
								sh_volume[idx] = volume.data[pos];
							else
								sh_volume[idx] = -2.0f;
						}
					}
				}
			}
			__syncthreads();

			//filter current voxel if weights less than thres_weight
			int xId = threadIdx.x; //original 3d index on sh_volume
			int zId = xId / (8 * 8);
			xId -= zId * 8 * 8;
			int yId = xId / 8;
			xId -= yId * 8;

			int pos = vxls_per_cube*blkId +
				(zId+zOffset) *TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
				(yId+yOffset)*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + (xId+xOffset);
			float weight = volume.weights[pos];
			float val_cur = volume.data[pos];

			float const sigma_s = 2.0;
			float const sigma_r = 0.6;
			if (0.0f < weight && weight < thres_weight && val_cur != SDF_NULL_VALUE)
			{
				float val = 0.0f;
				float ws = 0.0f;

				#pragma unroll
				for (int dx = -VOLUME_BLUR_RADIUS; dx <= VOLUME_BLUR_RADIUS; dx++){
					int x = xId + dx + VOLUME_BLUR_RADIUS;
					#pragma unroll
					for (int dy = -VOLUME_BLUR_RADIUS; dy <= VOLUME_BLUR_RADIUS; dy++){
						int y = yId + dy + VOLUME_BLUR_RADIUS;
						#pragma unroll
						for (int dz = -VOLUME_BLUR_RADIUS; dz <= VOLUME_BLUR_RADIUS; dz++){
							int z = zId + dz + VOLUME_BLUR_RADIUS;
							int idx_in_sh = z*VXLS_DIM_IN_SH*VXLS_DIM_IN_SH + y * VXLS_DIM_IN_SH + x;							
							float sh_val = sh_volume[idx_in_sh];
							if (sh_val != SDF_NULL_VALUE){
								float w = __expf(-(dx*dx + dy*dy + dz*dz) / (2.0f*sigma_s*sigma_s) -
									(sh_val - val_cur)*(sh_val - val_cur) / (2.0f*sigma_r*sigma_r));

								val += sh_val*w;
								ws += w;
							}
						}
					}
				}
				if (ws > M_EPS)
					val /= ws;
				else
					val = SDF_NULL_VALUE;
				dev_vxl_data_f[pos] = val;
			}
			else
			{
				dev_vxl_data_f[pos] = val_cur;
			}
		}
	}
}


void VolumetricFusionHelperCudaImpl::
smooth_volume(VolumeTwoLevelHierachy* volume, float *dev_vxl_data_out, float thres_weight)
{
	int threads_per_block = 512;
	int blocks_per_grid = 2048;

	init_two_level_hierachy_volume_data_kernel<float><<<blocks_per_grid, threads_per_block>>>(dev_vxl_data_out, 
													volume->gpu_cubes_occpied_count.dev_ptr, 
													volume->vxls_per_cube, SDF_NULL_VALUE);
	m_checkCudaErrors();

	VolumeConstDataGPU volume_gpu(*volume);	
	volume_filter_kernel<<<blocks_per_grid, threads_per_block>>>(volume_gpu, dev_vxl_data_out,
											volume->vxls_per_cube, volume->cube_size_in_voxel, thres_weight);
	m_checkCudaErrors();
}






}
