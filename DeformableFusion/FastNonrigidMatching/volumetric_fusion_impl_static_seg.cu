#ifndef __VOLUMETRIC_FUSION_IMPL_STATIC_SEG_CU__
#define __VOLUMETRIC_FUSION_IMPL_STATIC_SEG_CU__

//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"
//#include "cuda_math_common.cuh"
//
//#include "volumetric_fusion_impl.cuh"
//
//#include "geometry_types_cuda.h"
//#include "cuda_vector_fixed.cuh"
//#include "DeformGraphCudaImpl.cuh"

namespace VolmetricFusionCuda{

__global__
void bgSub_update_model_kernel(BGDepthModeCuda* dev_bg_mode_all, int depth_width, int depth_height)
{
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < depth_width*depth_height*VIEW_NUM)
	{
		int vId = id / (depth_width*depth_height);
		int x = id - vId*(depth_width*depth_height);
		int y = x / depth_width;
		x -= y*depth_width;

		unsigned short d = tex2DLayered(tex_depthImgs, x, y, vId);
		depth_remove_top_bit(d);
		if (d > 0)
		{
			BGDepthModeCuda bg_mode = dev_bg_mode_all[id];
			if (bg_mode.count <= 0)
			{
				bg_mode.mean = d;
				bg_mode.var = 0;
				bg_mode.count = 1;
			}
			else if ( fabsf(d - bg_mode.mean) < 50) //if the new depth is withing 50mm of the mean
			{
				bg_mode.mean = (bg_mode.mean*bg_mode.count + d) / (bg_mode.count + 1);
				float var = (d - bg_mode.mean)*(d - bg_mode.mean);
				bg_mode.var = (bg_mode.var*bg_mode.count + var) / (bg_mode.count + 1);
				bg_mode.count++;
			}
			else if (d < bg_mode.mean - 50) //
			{
				bg_mode.mean = d;
				bg_mode.var = 0;
				bg_mode.count = 1;
			}
			dev_bg_mode_all[id] = bg_mode;
		}
	}
}

__global__
void bgSub_remove_bg_kernel(BGDepthModeCuda const* __restrict__ dev_bg_mode_all, int depth_width, int depth_height, 
								 bool bUseDepthTopBitAsSeg)
{
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < depth_width*depth_height*VIEW_NUM)
	{
		int vId = id / (depth_width*depth_height);
		int x = id - vId*(depth_width*depth_height);
		int y = x / depth_width;
		x -= y*depth_width;

		unsigned short d = tex2DLayered(tex_depthImgs, x, y, vId);
		if (bUseDepthTopBitAsSeg)
			depth_extract_fg(d);
		else
			depth_remove_top_bit(d);

		//bUseDepthTopBitAsSeg is used. for those invalid pixels labeled as fg, will not change its label 
		if (d > 0)
		{
			BGDepthModeCuda bg_mode = dev_bg_mode_all[id];
			if (bg_mode.count > 0)
			{
				if (d < bg_mode.mean - MIN(60, MAX(15, 3.0 * sqrtf(bg_mode.var))) )
					d |= 0x8000;
				else
					d = 0x8000;
				surf2DLayeredwrite(d, surf_depthImgs, x * sizeof(unsigned short), y, vId, cudaBoundaryModeClamp);
			}
		}
	}
}

void VolumetricFusionHelperCudaImpl::bgSub_update_bg_mode()
{
	int threads_per_block = 128;
	int blocks_per_grid = (depth_height_*depth_width_*VIEW_NUM + threads_per_block - 1) / threads_per_block;
	bgSub_update_model_kernel<<<blocks_per_grid, threads_per_block>>>(dev_bg_modes_all_views_, depth_width_, depth_height_);
	m_checkCudaErrors();
}

void VolumetricFusionHelperCudaImpl::bgSub_remove_bg(bool bUseDepthTopBitAsSeg)
{
	int threads_per_block = 128;
	int blocks_per_grid = (depth_height_*depth_width_*VIEW_NUM + threads_per_block - 1) / threads_per_block;
	bgSub_remove_bg_kernel<<<blocks_per_grid, threads_per_block>>>(dev_bg_modes_all_views_, depth_width_, depth_height_, bUseDepthTopBitAsSeg);
	m_checkCudaErrors();
}

void VolumetricFusionHelperCudaImpl::bgSub_clear_bg()
{
	checkCudaErrors(cudaMemsetAsync(dev_bg_modes_all_views_, 0, sizeof(BGDepthModeCuda)*depth_width_*depth_height_*VIEW_NUM));
}

void VolumetricFusionHelperCudaImpl::allocate_mem_bg_sub(int depth_width, int depth_height)
{
	checkCudaErrors(cudaMalloc(&dev_bg_modes_all_views_, sizeof(BGDepthModeCuda)*depth_width*depth_height*VIEW_NUM));
	checkCudaErrors(cudaMemsetAsync(dev_bg_modes_all_views_, 0, sizeof(BGDepthModeCuda)*depth_width*depth_height*VIEW_NUM));
}




}

#endif