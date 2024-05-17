#include "CCLCudaImpl.h"
#include "cuda_runtime.h"
#include <helper_cuda.h>
#include "../Common/debug.h"

//__device__ __forceinline__
//int ccl_find_root(int* equivalence_arr, int addr)
//{
//	if (addr >= 0)
//	{
//		while (equivalence_arr[addr] != addr)
//		{
//			addr = equivalence_arr[addr];
//			if (addr < 0)
//				printf("\nError: %d\n", addr);
//		}
//	}
//	return addr;
//}

__device__ __forceinline__
int ccl_find_root(int* equivalence_arr, int addr)
{
	if (addr >= 0)
	{
		int addr_new = equivalence_arr[addr];
		while (addr_new != addr)
		{
			//if (addr_new < 0)
			//	printf("\nError: label[%d] = %d\n", addr, addr_new);
			addr = addr_new;
			addr_new = equivalence_arr[addr];
		}
	}
	return addr;
}

__device__ __forceinline__
int ccl_find_root_backtrack(int* equivalence_arr, int addr)
{
	if (addr >= 0)
	{
#define BACKTRACK_DEPTH 3
		int intermediate_labels[BACKTRACK_DEPTH];
		intermediate_labels[0] = addr;
		#pragma unroll
		for (int i = 1; i < BACKTRACK_DEPTH; i++)
			intermediate_labels[i] = -1;

		int count = 1;
		int addr_new = equivalence_arr[addr];
		while (addr_new != addr)
		{
			if (count < BACKTRACK_DEPTH)
			{
				intermediate_labels[count] = addr_new;
				count++;
			}

			addr = addr_new;
			addr_new = equivalence_arr[addr];
		}
		#pragma unroll
		for (int i = 0; i < BACKTRACK_DEPTH; i++)
		{
			int label = intermediate_labels[i];
			if (label >=0 )
				equivalence_arr[label] = addr;
		}
	}
	return addr;
}



__device__ __forceinline__
int ccl_label_local_to_global(int label, int img_width)
{
	if (label >= 0)
	{
		int lc_x = label;
		int lc_y = label / blockDim.x;
		lc_x -= lc_y*blockDim.x;

		return (blockIdx.y*blockDim.y + lc_y) * img_width +
			blockIdx.x*blockDim.x + lc_x;
	}
	else
		return label;
}


//ccl on patches
__global__
void ccl_kernel_local(cudaTextureObject_t texObj_depths, int *dev_labels,
					  int width, int height, int vId, 
					  int thres_depth,
					  bool bUseTopBitAsSegBit)
{
	extern __shared__ char sh_mem[];

	int* sh_bChanged = (int*)sh_mem;
	int* sh_labels = (int*)(sh_mem+sizeof(int));
	unsigned int* sh_depths = (unsigned int*)((sh_mem + sizeof(int)) + sizeof(int)*blockDim.x*blockDim.y);

	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;

	int lc_id = threadIdx.x + threadIdx.y*blockDim.x;
	//load depth
	if (x < width && y < height)
	{
		unsigned short d = tex2DLayered<unsigned short>(texObj_depths, x, y, vId);
		if (bUseTopBitAsSegBit)
			depth_extract_fg(d);
		sh_depths[lc_id] = d;
		if (d > 0)
			sh_labels[lc_id] = lc_id;
		else
			sh_labels[lc_id] = -1;

	}
	else
	{
		sh_depths[lc_id] = 0;
		sh_labels[lc_id] = -1;
	}
	__syncthreads(); //if all 0 for depth, then exit

	const int neighbors_num = 8;
	const int dx[] = { 0, 0, -1, 1, -1, -1,  1, 1};
	const int dy[] = {-1, 1,  0, 0, -1,  1, -1, 1};

	int depth_cur = sh_depths[lc_id];
	int label = sh_labels[lc_id];
	while (1)
	{
		if (threadIdx.x == 0 && threadIdx.y == 0)
			sh_bChanged[0] = 0;

		//Pass 1: merge equivalence trees
		int label_new = label;
		if (label >= 0)
		{
			//the find the minimal label from the neighboring elements
			for (int i = 0; i < neighbors_num; i++)
			{
				int x_n = threadIdx.x + dx[i];
				int y_n = threadIdx.y + dy[i];
				if (0 <= x_n && x_n < blockDim.x &&
					0 <= y_n && y_n < blockDim.y)
				{
					int lc_id_n = y_n*blockDim.x + x_n;
					int depth_n = sh_depths[lc_id_n];
					if (depth_n > 0 && fabsf(depth_n - depth_cur) < thres_depth)
						label_new = MIN(label_new, sh_labels[lc_id_n]);
				}
			}
		}
		__syncthreads();

		//merge equivalence tree
		if (label >= 0 && label_new < label)
		{
			atomicMin(&(sh_labels[label]), label_new);
			sh_bChanged[0] = 1;
		}
		__syncthreads();

		if (sh_bChanged[0] == 0)
			break;

		//Pass 2: flat the tree
		if (label >= 0)
			label = ccl_find_root(sh_labels, lc_id);
		__syncthreads();

		sh_labels[lc_id] = label;
	}

	//save labels
	if (x < width && y < height)
	{
		int gl_label = ccl_label_local_to_global(label, width);
		//surf2DLayeredwrite(gl_label, surfObj_labels, x*sizeof(int), y, vId, cudaBoundaryModeClamp);
		dev_labels[y*width + x] = gl_label;
	}
}

//base patches : super patches : blocks :  grid of blocks
//stitch super patches together
__global__
__launch_bounds__(1024, 1)
void ccl_kernel_seam(cudaTextureObject_t texObj_depths, int *dev_labels,
					 int width, int height, int vId, 
					 int base_patch_dim_in_pixel, int super_patch_dim_in_base_patch, int block_dim_in_super_patch,
					 int thres_depth,
					 bool bUseTopBitAsSegBit)
{
	extern __shared__ char sh_mem[];
	const int super_patch_dim_in_pixel = base_patch_dim_in_pixel*super_patch_dim_in_base_patch;
	const int pxls_per_line = super_patch_dim_in_pixel*block_dim_in_super_patch; //pxls_per_line for each block
	const int lines_num = (block_dim_in_super_patch - 1)*2;
	const int pxl_pairs_in_border_num = pxls_per_line*lines_num;

	int* sh_bChanged = (int*)sh_mem;
	unsigned short *sh_depths_border_tl = (unsigned short*)(sh_mem + sizeof(int));
	unsigned short *sh_depths_border_br = (unsigned short*)(sh_mem + sizeof(int) + sizeof(unsigned short)*pxls_per_line*lines_num);	
	int* sh_labels_border_tl = (int*)((char*)sh_depths_border_br + sizeof(unsigned short)*pxls_per_line*lines_num);
	int* sh_labels_border_br = (int*)((char*)sh_labels_border_tl + sizeof(int)*pxls_per_line*lines_num);
	
	int x_offset = blockIdx.x * pxls_per_line;
	int y_offset = blockIdx.y * pxls_per_line;
	//load depths at super patch borders
	for (int id = threadIdx.x; id < pxl_pairs_in_border_num; id += blockDim.x)
	{
		int ln_idx = id / pxls_per_line;
		int pxl_idx = id - ln_idx * pxls_per_line;

		unsigned short d;
		int label;
		if (ln_idx < block_dim_in_super_patch - 1)
		{
			//horizontal lines
			int y = (ln_idx + 1) * super_patch_dim_in_pixel - 1;
			int x = pxl_idx;
			x += x_offset;
			y += y_offset;

			if (x < width && y < height)
			{
				d = tex2DLayered<unsigned short>(texObj_depths, x, y, vId);
				if (bUseTopBitAsSegBit)
					depth_extract_fg(d);
				sh_depths_border_tl[id] = d;
				label = dev_labels[y*width + x];
				sh_labels_border_tl[id] = ccl_find_root(dev_labels, label);

				y += 1;
				d = tex2DLayered<unsigned short>(texObj_depths, x, y, vId);
				if (bUseTopBitAsSegBit)
					depth_extract_fg(d);
				sh_depths_border_br[id] = d;
				label = dev_labels[y*width + x];
				sh_labels_border_br[id] = ccl_find_root(dev_labels, label);
			}
			else
			{
				sh_depths_border_tl[id] = 0;
				sh_depths_border_br[id] = 0;
				sh_labels_border_tl[id] = -1;
				sh_labels_border_br[id] = -1;
			}
		}
		else
		{
			//vertical lines
			int x = (ln_idx - block_dim_in_super_patch + 2) * super_patch_dim_in_pixel - 1;
			int y = pxl_idx;
			x += x_offset;
			y += y_offset;

			if (x < width && y < height)
			{
				d = tex2DLayered<unsigned short>(texObj_depths, x, y, vId);
				if (bUseTopBitAsSegBit)
					depth_extract_fg(d);
				sh_depths_border_tl[id] = d;
				label = dev_labels[y*width + x];
				sh_labels_border_tl[id] = ccl_find_root(dev_labels, label);

				x += 1;
				d = tex2DLayered<unsigned short>(texObj_depths, x, y, vId);
				if (bUseTopBitAsSegBit)
					depth_extract_fg(d);
				sh_depths_border_br[id] = d;
				label = dev_labels[y*width + x];
				sh_labels_border_br[id] = ccl_find_root(dev_labels, label);
			}
			else
			{
				sh_depths_border_tl[id] = 0;
				sh_depths_border_br[id] = 0;
				sh_labels_border_tl[id] = -1;
				sh_labels_border_br[id] = -1;
			}
		}
	}
	__syncthreads();

	while (1)
	{
		if (threadIdx.x == 0)
			sh_bChanged[0] = 0;
		__syncthreads();

		//union equivalence trees
		for (int id = threadIdx.x; id < pxl_pairs_in_border_num; id += blockDim.x)
		{
			int ln_idx = id / pxls_per_line;
			int pxl_idx = id - ln_idx * pxls_per_line;
			unsigned short d = sh_depths_border_tl[id];
			int label = sh_labels_border_tl[id];
			if (d > 0)
			{
				#pragma unroll
				for (int i = -1; i <= 1; i++)
				{
					int pxl_idx_n = pxl_idx + i;
					if (0 <= pxl_idx_n && pxl_idx_n < pxls_per_line)
					{
						unsigned short d_n = sh_depths_border_br[id + i];
						int label_n = sh_labels_border_br[id + i];
						if (d_n > 0 && abs(d - d_n) < thres_depth)
						{
							//if (label == -1)
							//	printf("!!!label == -1!, d = %d\n", d);
							//if (label_n == -1)
							//	printf("!!!label_n == -1!, d_n = %d\n", d_n);

							if (label < label_n)
							{
								atomicMin(&(dev_labels[label_n]), label);
								sh_bChanged[0] = 1;
							}
							if (label > label_n)
							{
								atomicMin(&(dev_labels[label]), label_n);
								sh_bChanged[0] = 1;
							}
						}
					}
				}
			}
		}
		__syncthreads();

		if (!sh_bChanged[0])
			break;

		//flat the tree for border pixels
		for (int id = threadIdx.x; id < pxl_pairs_in_border_num; id += blockDim.x)
		{
			int label = sh_labels_border_tl[id];
			if (label >= 0)
				sh_labels_border_tl[id] = ccl_find_root(dev_labels, label);
			label = sh_labels_border_br[id];
			if (label >= 0)
				sh_labels_border_br[id] = ccl_find_root(dev_labels, label);
		}
	}
}

//flat the equivalence tree
__global__
void ccl_kernel_flat(int *dev_labels, int width, int height, int vId)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;

	if (x < width && y < height)
	{
		int id = y*width + x;
		int label = dev_labels[id];
		if (label >= 0)
			dev_labels[id] = ccl_find_root(dev_labels, label);
	} 
}

__global__
void flag_taken_labels_kernel(int const* __restrict__ dev_labels, short* dev_LUT_new_labels, int size)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < size)
	{
		int label = dev_labels[id];
		if (label >= 0)
			dev_LUT_new_labels[label] = 1;
	}
}

__global__
void flag_taken_labels_kernel2(int const* __restrict__ dev_labels, short* __restrict__ dev_LUT_new_labels, int width, int height)
{
	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;
	if (x < width && y < height)
	{
		int id = y*width + x;
		int label = dev_labels[id];
		if (label >= 0)
			dev_LUT_new_labels[label] = 1;
	}
}

__global__
void build_LUT_new_label_kernel(short* __restrict__ dev_LUT_new_labels, int* __restrict__ dev_global_label_count, int size)
{
	__shared__ int sh_count;

	if (threadIdx.x == 0)
		sh_count = 0;
	__syncthreads();

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	int lc_idx = 0;
	int bLabelTaken = 0;
	if (id < size)
	{
		bLabelTaken = dev_LUT_new_labels[id];
		if (bLabelTaken > 0)
			lc_idx = atomicAdd(&sh_count, 1);
	}
	__syncthreads();

	if (threadIdx.x == 0 && sh_count > 0)
		sh_count = atomicAdd(dev_global_label_count, sh_count);
	__syncthreads();

	if (bLabelTaken)
	{
		lc_idx += sh_count;
		dev_LUT_new_labels[id] = lc_idx;
	}
	else if (id < size)
	{
		dev_LUT_new_labels[id] = -1;
	}
}


__global__
void ccl_continuous_labeling_kenerl(int* __restrict__ dev_labels, short const* __restrict__ dev_LUT_new_labels, int size)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < size)
	{
		int old_label = dev_labels[id];
		if (old_label >= 0)
		{
			int new_label = dev_LUT_new_labels[old_label];
			dev_labels[id] = new_label;
		}
	}
}

void CCLCudaImpl::
run_ccl(cudaArray *cu_3dArr_depths, float depth_thres_in_cm, bool bUseTopBitAsSegBit)
{
	//create texture and surface object for depth maps.
	//bind depth array to texture and surface object
	struct cudaResourceDesc resDesc;
	memset(&resDesc, 0, sizeof(resDesc));
	resDesc.resType = cudaResourceTypeArray; 
	resDesc.res.array.array = cu_3dArr_depths;
	struct cudaTextureDesc texDesc;
	memset(&texDesc, 0, sizeof(texDesc));
	texDesc.addressMode[0] = cudaAddressModeClamp;
	texDesc.addressMode[1] = cudaAddressModeClamp;
	texDesc.filterMode = cudaFilterModePoint;
	texDesc.readMode = cudaReadModeElementType;
	texDesc.normalizedCoords = false;
	cudaTextureObject_t texObj_depths;
	cudaCreateTextureObject(&texObj_depths, &resDesc, &texDesc, NULL);
	cudaSurfaceObject_t surfObj_depths_;
	cudaCreateSurfaceObject(&surfObj_depths_, &resDesc);

	int depth_thres = ROUND(depth_thres_in_cm * 10.0f);
	for (int vId = 0; vId < view_num_; vId++)
	{
		int* dev_labels = &(dev_labels_all_views_[vId*width_*height_]);

		dim3 threads_per_block_lc(16, 16);
		dim3 blocks_per_grid((width_ + threads_per_block_lc.x - 1) / threads_per_block_lc.x,
			(height_ + threads_per_block_lc.y - 1) / threads_per_block_lc.y);
		int sh_mem_bytes = (2 * threads_per_block_lc.x*threads_per_block_lc.y + 1)*sizeof(int);
		ccl_kernel_local<<<blocks_per_grid, threads_per_block_lc, sh_mem_bytes>>>(texObj_depths, dev_labels,
								width_, height_, vId, depth_thres, bUseTopBitAsSegBit);
		m_checkCudaErrors();

		int block_dim_in_super_patch = 4;
		int base_patch_dim = threads_per_block_lc.x;
		int super_patch_dim = base_patch_dim;
		int block_dim = super_patch_dim*block_dim_in_super_patch;

		//for 2Kx2K image, it need 48K+4 bytes of shared memory, short of 4 bytes
		while (true)
		{
			int pixels_at_seam = block_dim * (block_dim_in_super_patch - 1) * 2;
			int threads_per_block_seam = MIN(pixels_at_seam, MAX_THREADS_PER_BLOCK);
			dim3 blocks_per_grid_seam((width_ + block_dim - 1) / block_dim, (height_ + block_dim - 1) / block_dim);
			sh_mem_bytes = pixels_at_seam * (sizeof(unsigned short)+sizeof(int)) * 2 + sizeof(int);
			ccl_kernel_seam<<<blocks_per_grid_seam, threads_per_block_seam, sh_mem_bytes>>>(texObj_depths, dev_labels,
										width_, height_, vId, 
										base_patch_dim, super_patch_dim / base_patch_dim, block_dim_in_super_patch, 
										depth_thres, bUseTopBitAsSegBit);
			m_checkCudaErrors();

			if (block_dim >= MAX(width_, height_))
				break;

			super_patch_dim = block_dim;
			if (super_patch_dim >= 256)
				block_dim_in_super_patch = 2;
			block_dim = super_patch_dim * block_dim_in_super_patch;
		}

		ccl_kernel_flat<<<blocks_per_grid, threads_per_block_lc>>>(dev_labels, width_, height_, vId);
		m_checkCudaErrors();

		//change labels to continuous
		checkCudaErrors(cudaMemsetAsync(dev_LUT_new_labels_, 0, sizeof(short)*width_*height_));
		int threads_per_block_relabel = MAX_THREADS_PER_BLOCK;
		int blocks_per_grid_relabel = (width_*height_ + threads_per_block_relabel - 1) / threads_per_block_relabel;
		//flag_taken_labels_kernel<<<blocks_per_grid_relabel, threads_per_block_relabel>>>(dev_labels, dev_LUT_new_labels_, width_*height_);
		flag_taken_labels_kernel2<<<blocks_per_grid, threads_per_block_lc>>>(dev_labels, dev_LUT_new_labels_, width_, height_);
		m_checkCudaErrors();

		checkCudaErrors(cudaMemsetAsync(labels_num_gpu_.dev_ptr, 0, sizeof(int)));
		build_LUT_new_label_kernel<<<blocks_per_grid_relabel, threads_per_block_relabel>>>(dev_LUT_new_labels_, labels_num_gpu_.dev_ptr, width_*height_);
		m_checkCudaErrors();

		ccl_continuous_labeling_kenerl<<<blocks_per_grid_relabel, threads_per_block_relabel>>>(dev_labels, dev_LUT_new_labels_, width_*height_);
		m_checkCudaErrors();
	}

}


//void CCLCudaImpl::
//allocate_and_bind_cuda_array_labels(int width, int height, int view_num)
//{
//	this->view_num_ = view_num;
//	this->height_ = height;
//	this->width_ = width;
//
//	//allocate labels array
//	cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindSigned);
//	checkCudaErrors(cudaMalloc3DArray(&cu_3dArr_labels_, &channelDesc_tex, make_cudaExtent(width, height, view_num),
//		cudaArraySurfaceLoadStore | cudaArrayLayered));
//
//	//bind label array to texture and surface object
//	struct cudaResourceDesc resDesc;
//	memset(&resDesc, 0, sizeof(resDesc));
//	resDesc.resType = cudaResourceTypeArray; 
//	resDesc.res.array.array = cu_3dArr_labels_;
//
//	struct cudaTextureDesc texDesc;
//	memset(&texDesc, 0, sizeof(texDesc));
//	texDesc.addressMode[0] = cudaAddressModeClamp;
//	texDesc.addressMode[1] = cudaAddressModeClamp;
//	texDesc.filterMode = cudaFilterModePoint;
//	texDesc.readMode = cudaReadModeElementType;
//	texDesc.normalizedCoords = false;
//
//	cudaCreateTextureObject(&texObj_labels_, &resDesc, &texDesc, NULL);
//
//	cudaCreateSurfaceObject(&surfObj_labels_, &resDesc);
//}

void CCLCudaImpl::
allocate_labels_linear_memory(int width, int height, int view_num)
{
	checkCudaErrors(cudaMalloc(&dev_labels_all_views_, width*height*view_num*sizeof(int)));
	checkCudaErrors(cudaMalloc(&dev_LUT_new_labels_, width*height*sizeof(short)));

	labels_num_gpu_.max_size = INT_MAX; //USHRT_MAX
	checkCudaErrors(cudaMalloc(&(labels_num_gpu_.dev_ptr), sizeof(int)));
}
