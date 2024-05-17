#ifndef __VOLUMETRIC_FUSION_IMPL_CU__
#define __VOLUMETRIC_FUSION_IMPL_CU__

namespace VolumetricFusionCuda{


texture<unsigned short, cudaTextureType2DLayered, cudaReadModeElementType> tex_depthImgs;
surface<void, cudaTextureType2DLayered> surf_depthImgs;
texture<unsigned short, cudaTextureType2DLayered, cudaReadModeElementType> tex_depthImgs_f;
surface<void, cudaTextureType2DLayered> surf_depthImgs_f;
texture<uchar4, cudaTextureType2DLayered, cudaReadModeElementType> tex_colorImgs;
texture<short, cudaTextureType3D, cudaReadModeElementType> tex_ndIds;
surface<void, cudaSurfaceType2DLayered> surf_mipmaps;
texture<float4, cudaTextureType2DLayered, cudaReadModeElementType> tex_normalMaps;
surface<void, cudaSurfaceType2DLayered> surf_normalMaps;
__constant__ __device__ CameraViewCuda dev_cam_views[MAX_NUM_DEPTH_CAMERAS];
__constant__ __device__ int dev_num_cam_views;

#define SDF_WEIGHT_MAX (30.0f*dev_num_cam_views)

__device__ int dev_global_cube_ngns_count_sum; //sum of all cube_ngns counts
__device__ int dev_global_cube_ngns_count_max; //max of all cube_ngns counts

bool VolumetricFusionHelperCudaImpl::allocate_cuda_arrays_depths(int tex_num, int width, int height)
{
	this->num_depthmaps_ = tex_num;
	this->depth_height_ = height;
	this->depth_width_ = width;

	LOGGER()->warning("VolumetricFusionHelperCudaImpl::allocate_cuda_arrays_depths", "Number of PODs: %d", tex_num);
	checkCudaErrors(cudaMemcpyToSymbol(dev_num_cam_views, &tex_num, sizeof(int)));

	cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_depth_, &channelDesc_tex, make_cudaExtent(width, height, tex_num),
		cudaArraySurfaceLoadStore | cudaArrayLayered));
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_depth_f_, &channelDesc_tex, make_cudaExtent(width, height, tex_num),
		cudaArraySurfaceLoadStore | cudaArrayLayered));

	// set texture parameters
	tex_depthImgs.addressMode[0] = cudaAddressModeClamp;
	tex_depthImgs.addressMode[1] = cudaAddressModeClamp;
	tex_depthImgs.filterMode = cudaFilterModePoint;
	tex_depthImgs.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_depthImgs, cu_3dArr_depth_, channelDesc_tex));
	// Bind the array to the surface
	checkCudaErrors(cudaBindSurfaceToArray(surf_depthImgs, cu_3dArr_depth_, channelDesc_tex));

	// set texture parameters
	tex_depthImgs_f.addressMode[0] = cudaAddressModeClamp;
	tex_depthImgs_f.addressMode[1] = cudaAddressModeClamp;
	tex_depthImgs_f.filterMode = cudaFilterModePoint;
	tex_depthImgs_f.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_depthImgs_f, cu_3dArr_depth_f_, channelDesc_tex));
	// Bind the array to the surface
	checkCudaErrors(cudaBindSurfaceToArray(surf_depthImgs_f, cu_3dArr_depth_f_, channelDesc_tex));


	cudaChannelFormatDesc channelDesc_mipmap = cudaCreateChannelDesc(16, 16, 0, 0, cudaChannelFormatKindUnsigned);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_mipmap_, &channelDesc_mipmap, make_cudaExtent(width / 2, height, tex_num),
		cudaArraySurfaceLoadStore | cudaArrayLayered));

	// Bind the array to the surface
	checkCudaErrors(cudaBindSurfaceToArray(surf_mipmaps, cu_3dArr_mipmap_, channelDesc_mipmap));

	return true;
}

bool VolumetricFusionHelperCudaImpl::allocate_cuda_arrays_normals(int tex_num, int width, int height)
{
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_normal_, &channelDesc, make_cudaExtent(width, height, tex_num),
		cudaArraySurfaceLoadStore | cudaArrayLayered));

	// set texture parameters
	tex_normalMaps.addressMode[0] = cudaAddressModeClamp;
	tex_normalMaps.addressMode[1] = cudaAddressModeClamp;
	tex_normalMaps.filterMode = cudaFilterModePoint;
	tex_normalMaps.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_normalMaps, cu_3dArr_normal_, channelDesc));

	// Bind the array to the surface
	checkCudaErrors(cudaBindSurfaceToArray(surf_normalMaps, cu_3dArr_normal_, channelDesc));

	return true;
}

bool VolumetricFusionHelperCudaImpl::allocate_cuda_arrays_colors(int tex_num, int width, int height)
{
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(8, 8, 8, 8, cudaChannelFormatKindUnsigned);
	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_color_, &channelDesc, make_cudaExtent(width, height, tex_num), cudaArrayLayered));

	// set texture parameters
	tex_normalMaps.addressMode[0] = cudaAddressModeClamp;
	tex_normalMaps.addressMode[1] = cudaAddressModeClamp;
	tex_normalMaps.filterMode = cudaFilterModePoint;
	tex_normalMaps.normalized = false;  // access with normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_colorImgs, cu_3dArr_color_, channelDesc));
	return true;
}

void VolumetricFusionHelperCudaImpl::bind_cuda_array_to_texture_depth(cudaArray *cu_3dArr_depth)
{
	if (cu_3dArr_depth)
	{
		checkCudaErrors(cudaUnbindTexture(tex_depthImgs));
		// set texture parameters
		tex_depthImgs.addressMode[0] = cudaAddressModeClamp;
		tex_depthImgs.addressMode[1] = cudaAddressModeClamp;
		tex_depthImgs.filterMode = cudaFilterModePoint;
		tex_depthImgs.normalized = false;  // access with normalized texture coordinates
		// Bind the array to the texture
		cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
		checkCudaErrors(cudaBindTextureToArray(tex_depthImgs, cu_3dArr_depth, channelDesc_tex));
		checkCudaErrors(cudaBindSurfaceToArray(surf_depthImgs, cu_3dArr_depth, channelDesc_tex));
	}
}

void VolumetricFusionHelperCudaImpl::
bind_tex_ndId(cudaArray* cu_3dArr_ndIds)
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
}



__device__ ushort2 pick_mipmap4(float x, float y, float r_h, float r_v, int texId, float level, int width_depth, int height_depth)
{
	if (x < 0.0f || y < 0.0f || x >= width_depth || y >= height_depth)
	{
		ushort2 ret;
		ret.x = 0;
		ret.y = 0;
		return ret;
	}
	else
	{
		level = powf(2.0f, level);
		int h_offset = ROUND(height_depth - height_depth / level * 2.0f);

		int u = (x - r_h) / level;
		int v = (y - r_v) / level;
		ushort2 d1 = surf2DLayeredread<ushort2>(surf_mipmaps, u * sizeof(ushort2), h_offset + v, texId, cudaBoundaryModeClamp);
		u = (x + r_h) / level;
		v = (y - r_v) / level;
		ushort2 d2 = surf2DLayeredread<ushort2>(surf_mipmaps, u * sizeof(ushort2), h_offset + v, texId, cudaBoundaryModeClamp);
		u = (x - r_h) / level;
		v = (y + r_v) / level;
		ushort2 d3 = surf2DLayeredread<ushort2>(surf_mipmaps, u * sizeof(ushort2), h_offset + v, texId, cudaBoundaryModeClamp);
		u = (x + r_h) / level;
		v = (y + r_v) / level;
		ushort2 d4 = surf2DLayeredread<ushort2>(surf_mipmaps, u * sizeof(ushort2), h_offset + v, texId, cudaBoundaryModeClamp);

		ushort2 mipmap = d1;
		if (d2.x > mipmap.x)
			mipmap.x = d2.x;
		if (d3.x > mipmap.x)
			mipmap.x = d3.x;
		if (d4.x > mipmap.x)
			mipmap.x = d4.x;

		mipmap.y = mipmap.x;
		if (d1.y != 0 && d1.y < mipmap.y)
			mipmap.y = d1.y;
		if (d2.y != 0 && d2.y < mipmap.y)
			mipmap.y = d2.y;
		if (d3.y != 0 && d3.y < mipmap.y)
			mipmap.y = d3.y;
		if (d4.y != 0 && d4.y < mipmap.y)
			mipmap.y = d4.y;

		return mipmap;
	}
}
__device__ ushort2 pick_mipmap9(float x, float y, float r_h, float r_v, int texId, float level, int width_depth, int height_depth)
{
	if (x < 0.0f || y < 0.0f || x >= width_depth || y >= height_depth)
	{
		ushort2 ret;
		ret.x = 0;
		ret.y = 0;
		return ret;
	}
	else
	{
		level = powf(2.0f, level);
		int h_offset = ROUND(height_depth - height_depth / level * 2.0f);

		int u0 = (x - r_h) / level;
		int u1 = x / level;
		int u2 = (x + r_h) / level;

		int v0 = (y - r_v) / level;
		int v1 = (y) / level;
		int v2 = (y + r_v) / level;

		ushort2 d1 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d2 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d3 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d4 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d5 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d6 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d7 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d8 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d9 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);

		ushort2 mipmap = d1;
		if (d2.x > mipmap.x)
			mipmap.x = d2.x;
		if (d3.x > mipmap.x)
			mipmap.x = d3.x;
		if (d4.x > mipmap.x)
			mipmap.x = d4.x;
		if (d5.x > mipmap.x)
			mipmap.x = d5.x;
		if (d6.x > mipmap.x)
			mipmap.x = d6.x;
		if (d7.x > mipmap.x)
			mipmap.x = d7.x;
		if (d8.x > mipmap.x)
			mipmap.x = d8.x;
		if (d9.x > mipmap.x)
			mipmap.x = d9.x;

		mipmap.y = mipmap.x;
		if (d1.y != 0 && d1.y < mipmap.y)
			mipmap.y = d1.y;
		if (d2.y != 0 && d2.y < mipmap.y)
			mipmap.y = d2.y;
		if (d3.y != 0 && d3.y < mipmap.y)
			mipmap.y = d3.y;
		if (d4.y != 0 && d4.y < mipmap.y)
			mipmap.y = d4.y;
		if (d5.y != 0 && d5.y < mipmap.y)
			mipmap.y = d5.y;
		if (d6.y != 0 && d6.y < mipmap.y)
			mipmap.y = d6.y;
		if (d7.y != 0 && d7.y < mipmap.y)
			mipmap.y = d7.y;
		if (d8.y != 0 && d8.y < mipmap.y)
			mipmap.y = d8.y;
		if (d9.y != 0 && d9.y < mipmap.y)
			mipmap.y = d9.y;

		return mipmap;
	}
}
__device__ ushort2 pick_mipmap16(float x, float y, float r_h, float r_v, int texId, float level, int width_depth, int height_depth, bool bPrint)
{
	if (x < 0.0f || y < 0.0f || x >= width_depth || y >= height_depth)
	{
		ushort2 ret;
		ret.x = 0;
		ret.y = 0;
		return ret;
	}
	else
	{
		level = powf(2.0f, level);
		int h_offset = ROUND(height_depth - height_depth / level * 2.0f);

		int u0 = (x - r_h) / level;
		int u1 = (x - r_h / 3.0f) / level;
		int u2 = (x + r_h / 3.0f) / level;
		int u3 = (x + r_h) / level;

		int v0 = (y - r_v) / level;
		int v1 = (y - r_v / 3.0f) / level;
		int v2 = (y + r_v / 3.0f) / level;
		int v3 = (y + r_v) / level;

		ushort2 d1 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d2 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d3 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d4 = surf2DLayeredread<ushort2>(surf_mipmaps, u0 * sizeof(ushort2), h_offset + v3, texId, cudaBoundaryModeClamp);

		ushort2 d5 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d6 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d7 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d8 = surf2DLayeredread<ushort2>(surf_mipmaps, u1 * sizeof(ushort2), h_offset + v3, texId, cudaBoundaryModeClamp);

		ushort2 d9 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d10 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d11 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d12 = surf2DLayeredread<ushort2>(surf_mipmaps, u2 * sizeof(ushort2), h_offset + v3, texId, cudaBoundaryModeClamp);

		ushort2 d13 = surf2DLayeredread<ushort2>(surf_mipmaps, u3 * sizeof(ushort2), h_offset + v0, texId, cudaBoundaryModeClamp);
		ushort2 d14 = surf2DLayeredread<ushort2>(surf_mipmaps, u3 * sizeof(ushort2), h_offset + v1, texId, cudaBoundaryModeClamp);
		ushort2 d15 = surf2DLayeredread<ushort2>(surf_mipmaps, u3 * sizeof(ushort2), h_offset + v2, texId, cudaBoundaryModeClamp);
		ushort2 d16 = surf2DLayeredread<ushort2>(surf_mipmaps, u3 * sizeof(ushort2), h_offset + v3, texId, cudaBoundaryModeClamp);

		if (bPrint)
		{
			printf("<%d, %d>: %d, %d\n", u0, v0, d1.x, d1.y);
			printf("<%d, %d>: %d, %d\n", u0, v1, d2.x, d2.y);
			printf("<%d, %d>: %d, %d\n", u0, v2, d3.x, d3.y);
			printf("<%d, %d>: %d, %d\n", u0, v3, d4.x, d4.y);
			printf("<%d, %d>: %d, %d\n", u1, v0, d5.x, d5.y);
			printf("<%d, %d>: %d, %d\n", u1, v1, d6.x, d6.y);
			printf("<%d, %d>: %d, %d\n", u1, v2, d7.x, d7.y);
			printf("<%d, %d>: %d, %d\n", u1, v3, d8.x, d8.y);
			printf("<%d, %d>: %d, %d\n", u2, v0, d9.x, d9.y);
			printf("<%d, %d>: %d, %d\n", u2, v1, d10.x, d10.y);
			printf("<%d, %d>: %d, %d\n", u2, v2, d11.x, d11.y);
			printf("<%d, %d>: %d, %d\n", u2, v3, d12.x, d12.y);
			printf("<%d, %d>: %d, %d\n", u3, v0, d13.x, d13.y);
			printf("<%d, %d>: %d, %d\n", u3, v1, d14.x, d14.y);
			printf("<%d, %d>: %d, %d\n", u3, v2, d15.x, d15.y);
			printf("<%d, %d>: %d, %d\n", u3, v3, d16.x, d16.y);
		}

		ushort2 mipmap = d1;
		if (d2.x > mipmap.x)
			mipmap.x = d2.x;
		if (d3.x > mipmap.x)
			mipmap.x = d3.x;
		if (d4.x > mipmap.x)
			mipmap.x = d4.x;
		if (d5.x > mipmap.x)
			mipmap.x = d5.x;
		if (d6.x > mipmap.x)
			mipmap.x = d6.x;
		if (d7.x > mipmap.x)
			mipmap.x = d7.x;
		if (d8.x > mipmap.x)
			mipmap.x = d8.x;
		if (d9.x > mipmap.x)
			mipmap.x = d9.x;
		if (d10.x > mipmap.x)
			mipmap.x = d10.x;
		if (d11.x > mipmap.x)
			mipmap.x = d11.x;
		if (d12.x > mipmap.x)
			mipmap.x = d12.x;
		if (d13.x > mipmap.x)
			mipmap.x = d13.x;
		if (d14.x > mipmap.x)
			mipmap.x = d14.x;
		if (d15.x > mipmap.x)
			mipmap.x = d15.x;
		if (d16.x > mipmap.x)
			mipmap.x = d16.x;

		mipmap.y = mipmap.x;
		if (d1.y != 0 && d1.y < mipmap.y)
			mipmap.y = d1.y;
		if (d2.y != 0 && d2.y < mipmap.y)
			mipmap.y = d2.y;
		if (d3.y != 0 && d3.y < mipmap.y)
			mipmap.y = d3.y;
		if (d4.y != 0 && d4.y < mipmap.y)
			mipmap.y = d4.y;
		if (d5.y != 0 && d5.y < mipmap.y)
			mipmap.y = d5.y;
		if (d6.y != 0 && d6.y < mipmap.y)
			mipmap.y = d6.y;
		if (d7.y != 0 && d7.y < mipmap.y)
			mipmap.y = d7.y;
		if (d8.y != 0 && d8.y < mipmap.y)
			mipmap.y = d8.y;
		if (d9.y != 0 && d9.y < mipmap.y)
			mipmap.y = d9.y;
		if (d10.y != 0 && d10.y < mipmap.y)
			mipmap.y = d10.y;
		if (d11.y != 0 && d11.y < mipmap.y)
			mipmap.y = d11.y;
		if (d12.y != 0 && d12.y < mipmap.y)
			mipmap.y = d12.y;
		if (d13.y != 0 && d13.y < mipmap.y)
			mipmap.y = d13.y;
		if (d14.y != 0 && d14.y < mipmap.y)
			mipmap.y = d14.y;
		if (d15.y != 0 && d15.y < mipmap.y)
			mipmap.y = d15.y;
		if (d16.y != 0 && d16.y < mipmap.y)
			mipmap.y = d16.y;
		return mipmap;
	}
}
__device__ ushort2 pick_mipmap25(float x, float y, float r_h, float r_v, int texId, float level, int width_depth, int height_depth/*, bool bPrint*/)
{
	if (x < 0.0f || y < 0.0f || x >= width_depth || y >= height_depth)
	{
		ushort2 ret;
		ret.x = 0;
		ret.y = 0;
		return ret;
	}
	else
	{
		level = powf(2.0f, level);
		int h_offset = ROUND(height_depth - height_depth / level * 2.0f);

		int u, v;
		ushort2 mipmap;
		mipmap.x = 0;
		mipmap.y = USHRT_MAX;
		for (int i = 0; i < 5; i++)
		{
			u = (x + r_h / 2.0f*(i - 2.0f)) / level;
			for (int j = 0; j < 5; j++)
			{
				v = (y + r_v / 2.0f*(j - 2.0f)) / level;
				ushort2 d = surf2DLayeredread<ushort2>(surf_mipmaps, u * sizeof(ushort2), h_offset + v, texId, cudaBoundaryModeClamp);
				if (d.x > mipmap.x)
					mipmap.x = d.x;
				if (d.y != 0 && d.y < mipmap.y)
					mipmap.y = d.y;
			}
		}
		if (mipmap.y > mipmap.x)
			mipmap.y = 0;

		return mipmap;
	}
}

}

#include "volumetric_fusion_impl_v1.cu"
#include "volumetric_fusion_impl_v2.cu"
#include "volumetric_fusion_impl_post_processing.cu"
#endif