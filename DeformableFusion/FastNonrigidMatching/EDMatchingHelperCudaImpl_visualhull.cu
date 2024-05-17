#ifndef __EDMATCHINGHELPERCUDAIMPL_VISUALHULL_CU__
#define __EDMATCHINGHELPERCUDAIMPL_VISUALHULL_CU__
#include "EDMatchingHelperCudaImpl.cu"

template<bool bUseSegmentation>
__global__ 
void VisualHullOccupancy_Kernel(int3 const* dev_visual_hull_dim, float3 const* dev_visual_hull_offset, float visual_hull_res,
								int depth_width, int depth_height,
								float mu // the area behind the surface will always be occupied
							   )
{
	int3 visual_hull_dim = *dev_visual_hull_dim;
	
	int vxlId = blockIdx.x;	
	int xId = threadIdx.x & 0x3;
	int yId = (threadIdx.x  >> 2 )& 0x3;
	int zId = (threadIdx.x >> 4) & 0x3;
	int vshulldim4x = visual_hull_dim.x >> 2;
	int vshulldim4y = visual_hull_dim.y >> 2;
	xId += (vxlId % (vshulldim4x)) << 2;
	zId += (vxlId / (vshulldim4x*vshulldim4y)) << 2;
	yId += ((vxlId / vshulldim4x) % vshulldim4y) << 2;

	if (xId < visual_hull_dim.x &&
		yId < visual_hull_dim.y &&
		zId < visual_hull_dim.z)

	{
		float3 visual_hull_offset = *dev_visual_hull_offset;
		cuda_vector_fixed<float, 3> vxl;
		vxl[0] = visual_hull_offset.x + xId*visual_hull_res;
		vxl[1] = visual_hull_offset.y + yId*visual_hull_res;
		vxl[2] = visual_hull_offset.z + zId*visual_hull_res;

		float bOccupied = -1; //be set to 1 if in front of some depth
		int bInvalidVoxel = 1;
		#pragma unroll
		for (int vId = 0; vId < dev_num_cam_views; vId++)
		{
			cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vxl + dev_cam_views[vId].cam_pose.T;

			//if the point is behind the camera, then skip it
			if (X[2] > 0.1f)
			{
				cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
				float fx = K[0][0];
				float fy = K[1][1];
				float cx = K[0][2];
				float cy = K[1][2];

				int u = ROUND(fx*X[0] / X[2] + cx);
				int v = ROUND(fy*X[1] / X[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u >= 0 && u < depth_width &&
					v >= 0 && v < depth_height)
				{
					//TODO: average. project the voxel cube instead of point
					unsigned short d_ = tex2DLayered(tex_depthImgs, u, v, vId);
					if (bUseSegmentation)
						depth_extract_fg_set_bg_as_far(d_);
					else
						depth_remove_top_bit(d_);

					if (d_ > 100)
					{
						bInvalidVoxel = 0;
						float d = d_ / 10.0f;
						//in front
						if (X[2] < d)
						{
							bOccupied = 1;
							break;
						}
					}
				}
			}
		}

		float occupancy_val = bOccupied;

		if (bInvalidVoxel)  occupancy_val = -1.0f;

		surf3Dwrite(occupancy_val, surf_visHull, xId*sizeof(float), yId, zId, cudaBoundaryModeClamp);
	}
}

void EDMatchingHelperCudaImpl::compute_visual_hull_occupancy(bool bUseSegmentation)
{
	int threads_per_block = 64; // REALLY HARDCODED TO THIS SIZE!
	int blocks_per_grid = (VISUAL_HULL_MAX_DIM*VISUAL_HULL_MAX_DIM*VISUAL_HULL_MAX_DIM + threads_per_block - 1) / threads_per_block;
	if (bUseSegmentation)
	{
		VisualHullOccupancy_Kernel<true><<<blocks_per_grid, threads_per_block>>>(dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
														depth_width_, depth_height_, 3.0);
	}
	else
	{
		VisualHullOccupancy_Kernel<false> << <blocks_per_grid, threads_per_block >> >(dev_visual_hull_dim_, dev_visual_hull_offset_, visual_hull_res_,
			depth_width_, depth_height_, 3.0);
	}
	m_checkCudaErrors();
}

inline __device__ cuda_vector_fixed<float, 3> visual_hull_gradient(int xId, int yId, int zId, float vxl_res)
{
	//gradient of the visual hull
	cuda_vector_fixed<float, 3> g_vis_hull;
	float P_x0 = tex3D(tex_visHull, xId - 1, yId, zId);
	float P_x2 = tex3D(tex_visHull, xId + 1, yId, zId);
	g_vis_hull[0] = (P_x2 - P_x0) / (2.0f*vxl_res);
	float P_y0 = tex3D(tex_visHull, xId, yId - 1, zId);
	float P_y2 = tex3D(tex_visHull, xId, yId + 1, zId);
	g_vis_hull[1] = (P_y2 - P_y0) / (2.0f*vxl_res);
	float P_z0 = tex3D(tex_visHull, xId, yId, zId - 1);
	float P_z2 = tex3D(tex_visHull, xId, yId, zId + 1);
	g_vis_hull[2] = (P_z2 - P_z0) / (2.0f*vxl_res);

	return g_vis_hull;
}

template <int BlurSize, int VolumeSize, bool bMoveZ>
__device__ inline void SeparableBoxFilterworker(int x, int yzcoord, int limit)
{
	float negSum = 0;
	float W = 0;
	float posSum = 0;
	float linedata[2 * BlurSize + 1];
	int lineOffset = 0;

#pragma unroll 
	for (int i = 0; i < BlurSize; i++) linedata[i] = 0;
#pragma unroll 
	for (int i = BlurSize; i < 2 * BlurSize + 1; i++)
	{
		if (bMoveZ) 
			linedata[i] = tex3D(tex_visHull, x, yzcoord, lineOffset);
		else 
			linedata[i] = tex3D(tex_visHull, x, lineOffset, yzcoord);

		lineOffset++;
		if (linedata[i] != 0)
		{
			W++;
			if (linedata[i] > 0)
			{
				posSum += linedata[i];
			}
			else
			{
				negSum += linedata[i];
			}
		}
	}

	const int klimit = (limit - 1) / (2 * BlurSize + 1) + 1;
	for (int k = 0; k <= klimit; k++)
	{
#pragma unroll 
		for (int regOffset = 0; regOffset < 2 * BlurSize + 1; regOffset++)
		{
			float toWrite = 0;
			int idx = (regOffset + BlurSize) % (2 * BlurSize + 1);
			if (linedata[idx] > 0)
			{
				toWrite = posSum;
			}
			else if (linedata[idx] < 0)
			{
				toWrite = negSum;
			}
			if (bMoveZ)
				surf3Dwrite(toWrite / W, surf_visHull, x*sizeof(float), yzcoord, lineOffset - (BlurSize + 1), cudaBoundaryModeClamp);
			else
				surf3Dwrite(toWrite / W, surf_visHull, x*sizeof(float), lineOffset - (BlurSize + 1), yzcoord, cudaBoundaryModeClamp);

			if (linedata[regOffset] != 0)
			{
				W--;
				if (linedata[regOffset] > 0)
				{
					posSum -= linedata[regOffset];
				}
				else
				{
					negSum -= linedata[regOffset];
				}
			}
			if (lineOffset < limit)
			{
				if (bMoveZ)
					linedata[regOffset] = tex3D(tex_visHull, x, yzcoord, lineOffset);
				else
					linedata[regOffset] = tex3D(tex_visHull, x, lineOffset, yzcoord);
				if (linedata[regOffset] != 0)
				{
					W++;
					if (linedata[regOffset] > 0)
					{
						posSum += linedata[regOffset];
					}
					else
					{
						negSum += linedata[regOffset];
					}
				}
			} 
			lineOffset++;
			if (lineOffset > limit + BlurSize) return;
		}
	}
}

template <int BlurSize, int VolumeSize>
__launch_bounds__(256, 4)
__global__ void SeparableBoxFilterXY(int3 const* dev_visual_hull_dim)
{
	const int3 visual_hull_dim = *dev_visual_hull_dim;
	if (blockIdx.y >= visual_hull_dim.y) return;
	int x = threadIdx.x + blockIdx.x*blockDim.x;
	if (x >= visual_hull_dim.x) return;
	
	SeparableBoxFilterworker<BlurSize, VolumeSize, true>(x, blockIdx.y, visual_hull_dim.z);
}

template <int BlurSize, int VolumeSize>
__launch_bounds__(256, 4)
__global__ void SeparableBoxFilterXZ(int3 const* dev_visual_hull_dim)
{
	const int3 visual_hull_dim = *dev_visual_hull_dim;
	if (blockIdx.y >= visual_hull_dim.z) return;
	int x = threadIdx.x + blockIdx.x*blockDim.x;
	if (x >= visual_hull_dim.x) return;

	SeparableBoxFilterworker<BlurSize, VolumeSize, false>(x, blockIdx.y, visual_hull_dim.y);
}

template <int BlurSize, int VolumeSize>
__launch_bounds__(32, 32)
__global__ void SeparableBoxFilterYZ(int3 const* dev_visual_hull_dim)
{
	const int3 visual_hull_dim = *dev_visual_hull_dim;
	if (blockIdx.x >= visual_hull_dim.y) return;
	if (blockIdx.y >= visual_hull_dim.z) return;

	float val[VolumeSize/32];
	__shared__ float valsum[VolumeSize + 2 * BlurSize];
	__shared__ float valplus[VolumeSize + 2 * BlurSize];
	__shared__ float valminus[VolumeSize + 2 * BlurSize];
	if (threadIdx.x < BlurSize)
	{
		valsum[threadIdx.x] = 0;
		valplus[threadIdx.x] = 0;
		valminus[threadIdx.x] = 0;
	}
#pragma unroll 
	for (int offset = 0; offset < VolumeSize; offset += 32)
	{
		float w = 0, p = 0, m = 0;
		float curval = 0;
		if (threadIdx.x + offset < visual_hull_dim.x)
			curval = tex3D(tex_visHull, threadIdx.x + offset, blockIdx.x, blockIdx.y);
		val[offset/32] = curval;
		if (curval != 0)
		{
			w = 1;
			if (curval > 0) p = curval; else m = curval;
		}
		for (int i = 1; i <= 16; i *= 2)
		{
			float nw = __shfl_up_sync(0xFFFFFFFF, w, i);
			float nm = __shfl_up_sync(0xFFFFFFFF, m, i);
			float np = __shfl_up_sync(0xFFFFFFFF, p, i);
			if (threadIdx.x >= i)
			{
				w += nw;
				m += nm;
				p += np;
			}
		}

		valsum[threadIdx.x + BlurSize + offset] = w + valsum[BlurSize + offset - 1];
		valplus[threadIdx.x + BlurSize + offset] = p + valplus[BlurSize + offset - 1];
		valminus[threadIdx.x + BlurSize + offset] = m + valminus[BlurSize + offset - 1];
	}
	if (threadIdx.x < BlurSize)
	{
		valsum[threadIdx.x + BlurSize + visual_hull_dim.x] = valsum[BlurSize + visual_hull_dim.x - 1];
		valplus[threadIdx.x + BlurSize + visual_hull_dim.x] = valplus[BlurSize + visual_hull_dim.x - 1];
		valminus[threadIdx.x + BlurSize + visual_hull_dim.x] = valminus[BlurSize + visual_hull_dim.x - 1];
	}
#pragma unroll 
	for (int offset = 0; offset < VolumeSize; offset += 32)
	{
		int o = threadIdx.x + offset;
		if (o >= visual_hull_dim.x) return;
		if (val[offset/32] != 0)
		{
			float v = 0;
			float w = valsum[o + 2*BlurSize] - valsum[o];
			if (val[offset / 32] > 0)
				v = valplus[o + 2 * BlurSize] - valplus[o];
			else
				v = valminus[o + 2 * BlurSize] - valminus[o];
			surf3Dwrite(v / w, surf_visHull, (threadIdx.x + offset)*sizeof(float), blockIdx.x, blockIdx.y, cudaBoundaryModeClamp);
		}
	}

}


void EDMatchingHelperCudaImpl::blur_visual_hull_occupancy(int round)
{
	dim3 blocksYZ(VISUAL_HULL_MAX_DIM, VISUAL_HULL_MAX_DIM, 1);
	dim3 blocks(VISUAL_HULL_MAX_DIM / 64, VISUAL_HULL_MAX_DIM, 1);
	int blockSize = 64;
	for (int i = 0; i < round; ++i)
	{
		SeparableBoxFilterXY<4, VISUAL_HULL_MAX_DIM> << <blocks, blockSize >> >(dev_visual_hull_dim_);
		m_checkCudaErrors();
		SeparableBoxFilterXZ<4, VISUAL_HULL_MAX_DIM> << <blocks, blockSize >> >(dev_visual_hull_dim_);
		m_checkCudaErrors();
		SeparableBoxFilterYZ<4, VISUAL_HULL_MAX_DIM> << <blocksYZ, 32 >> >(dev_visual_hull_dim_);
		m_checkCudaErrors();
	}
}


//one point per thread
__global__ void 
evaluate_visual_hull_cost_kernel(float *dev_global_visual_hull_cost,
								int3 visual_hull_dim, float3 visual_hull_offset, float visual_hull_res, 
								short2 *dev_cam_vis, float const* dev_vts_t, int vts_num, int vt_dim,
								float w_visual_hull
								)
{
	__shared__ float costs[MAX_THREADS_PER_BLOCK];

	int vtIdx = threadIdx.x + blockIdx.x*blockDim.x;
	float cost = 0.0f;
	if (vtIdx < vts_num)
	{
		short2 cam_vis = dev_cam_vis[vtIdx];
		if (cam_vis.x == 0)
		{
			float const*v_t_ = dev_vts_t + vtIdx * vt_dim;
			cuda_vector_fixed<float, 3> v_t(v_t_);
			
			int xId = (v_t[0] - visual_hull_offset.x) / visual_hull_res;
			int yId = (v_t[1] - visual_hull_offset.y) / visual_hull_res;
			int zId = (v_t[2] - visual_hull_offset.z) / visual_hull_res;

			float val_vis_hull = tex3D(tex_visHull, xId, yId, zId);
			if (0.0f <= val_vis_hull && val_vis_hull <= 1.0f)
				cost = 1.0f - val_vis_hull;
			cost = cost*cost*w_visual_hull;
		}
	}
	__syncthreads();

	//reduction
	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
		atomicAdd(dev_global_visual_hull_cost, costs[0]);
}


__global__
void setup_visual_hull_info_kernel(int3 *dev_visual_hull_dim, float3 *dev_visual_hull_offset, BoundingBox3DCuda const* dev_bbox, float visual_hull_res)
{
	float max_size = visual_hull_res*VISUAL_HULL_MAX_DIM;
	BoundingBox3DCuda bbox = *dev_bbox;
	float3 visual_hull_offset;
	int3 visual_hull_dim;
	if (max_size > bbox.x_e - bbox.x_s &&
		max_size > bbox.y_e - bbox.y_s &&
		max_size > bbox.z_e - bbox.z_s)
	{
		visual_hull_offset.x = bbox.x_s;
		visual_hull_offset.y = bbox.y_s;
		visual_hull_offset.z = bbox.z_s;

		visual_hull_dim.x = ceilf((bbox.x_e - bbox.x_s) / visual_hull_res);
		if ((visual_hull_dim.x & 0x3) > 0) visual_hull_dim.x = (visual_hull_dim.x & 0xffc) + 0x4;// granularity by 4
		if (visual_hull_dim.x < 32) visual_hull_dim.x = 32;
		visual_hull_dim.y = ceilf((bbox.y_e - bbox.y_s) / visual_hull_res);
		if ((visual_hull_dim.y & 0x3) > 0) visual_hull_dim.y = (visual_hull_dim.y & 0xffc) + 0x4;// granularity by 4
		if (visual_hull_dim.y < 32) visual_hull_dim.y = 32;
		visual_hull_dim.z = ceilf((bbox.z_e - bbox.z_s) / visual_hull_res);
		if ((visual_hull_dim.z & 0x3) > 0) visual_hull_dim.z = (visual_hull_dim.z & 0xffc) + 0x4;// granularity by 4
		if (visual_hull_dim.z < 32) visual_hull_dim.z = 32;
	}
	else
	{
		visual_hull_dim.x = VISUAL_HULL_MAX_DIM;
		visual_hull_dim.y = VISUAL_HULL_MAX_DIM;
		visual_hull_dim.z = VISUAL_HULL_MAX_DIM;
		visual_hull_offset.x = (bbox.x_e + bbox.x_s) / 2.0f - max_size / 2.0f;
		visual_hull_offset.y = (bbox.y_e + bbox.y_s) / 2.0f - max_size / 2.0f;
		visual_hull_offset.z = (bbox.z_e + bbox.z_s) / 2.0f - max_size / 2.0f;
	}
	dev_visual_hull_dim[0] = visual_hull_dim;
	dev_visual_hull_offset[0] = visual_hull_offset;
}

void EDMatchingHelperCudaImpl::
setup_visual_hull_info(BoundingBox3DCuda const* dev_bbox, float visual_hull_res)
{
	setup_visual_hull_info_kernel<<<1, 1>>>(dev_visual_hull_dim_, dev_visual_hull_offset_, dev_bbox, visual_hull_res);
	m_checkCudaErrors();

	visual_hull_res_ = visual_hull_res;
}

void EDMatchingHelperCudaImpl::allocate_and_bind_vishull_cu3dArray()
{
	checkCudaErrors(cudaMalloc(&dev_visual_hull_dim_, sizeof(int3)));
	checkCudaErrors(cudaMalloc(&dev_visual_hull_offset_, sizeof(float3)));

	checkCudaErrors(cudaMemset(dev_visual_hull_dim_, 0, sizeof(int3)));

	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
	checkCudaErrors(cudaMalloc3DArray(&cu_3DArr_vishull_, &channelDesc,
		make_cudaExtent(VISUAL_HULL_MAX_DIM, VISUAL_HULL_MAX_DIM, VISUAL_HULL_MAX_DIM),
		cudaArraySurfaceLoadStore));

	//bind cu_3dArr_ndIds_to both texture and surface 
	checkCudaErrors(cudaBindSurfaceToArray(surf_visHull, cu_3DArr_vishull_, channelDesc));
	// set texture parameters
	tex_visHull.addressMode[0] = cudaAddressModeClamp;
	tex_visHull.addressMode[1] = cudaAddressModeClamp;
	tex_visHull.addressMode[2] = cudaAddressModeClamp;
	tex_visHull.filterMode = cudaFilterModePoint;
	tex_visHull.normalized = false;  // access with un-normalized texture coordinates
	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(tex_visHull, cu_3DArr_vishull_, channelDesc));
}

#endif