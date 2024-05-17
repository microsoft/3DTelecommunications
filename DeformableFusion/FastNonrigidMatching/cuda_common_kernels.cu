#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"

#include "../Common/cuda/CudaHelpers.h"

__global__
void vector_min_kernel(float* dev_x, int n, float* dev_global_min_val)
{
	__shared__ float xs[MAX_THREADS_PER_BLOCK];
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	//if ()
	xs[threadIdx.x] = dev_x[id];
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			xs[threadIdx.x] = MIN(xs[threadIdx.x], xs[threadIdx.x + s]);
		}
		__syncthreads();
	}
}

__global__
void transform_ax_b( int const * __restrict__ input, int * __restrict__ output, int a, int b )
{
	*output = a * (*input) + b;
}

__global__
void transform_ax_by_c( int a, int const * __restrict__ x, int b, int const * __restrict__ y, int c, int * __restrict__ output )
{
	*output = a * (*x) + b * (*y) + c;
}

__global__
void max_bound_gpu_count_kernel(int* dev_gpu_count, int max_bound)
{
	if (*dev_gpu_count > max_bound)
		*dev_gpu_count = max_bound;
}

namespace cuda
{
	void gpu_size_data::copy_transformed( gpu_size_data const & other, int scale, int offset )
	{
		if (!dev_ptr)
			abort();

		max_size = (scale > 0) * scale * other.max_size + offset;

		transform_ax_b<<<1, 1>>>(other.dev_ptr, dev_ptr, scale, offset);
		m_checkCudaErrors();

	}

	void gpu_size_data::cap_gpu_size(int max_bound)
	{
		max_bound_gpu_count_kernel<<<1, 1>>>(dev_ptr, max_bound);
		m_checkCudaErrors();
	}

	void gpu_size_data::multiply_add( int s1, gpu_size_data const & v1, int s2, gpu_size_data const & v2, int offset )
	{
		if (!dev_ptr)
			abort();

		max_size = (s1 > 0) * s1 * v1.max_size + (s2 > 0) * s2 * v2.max_size + offset;
		transform_ax_by_c<<<1, 1>>>(s1, v1.dev_ptr, s2, v2.dev_ptr, offset, dev_ptr);
		m_checkCudaErrors();
	}

	void gpu_size_data::multiply_add_gpu_only( int s1, gpu_size_data const & v1, int s2, gpu_size_data const & v2, int offset )
	{
		if (!dev_ptr)
			abort();

		transform_ax_by_c<<<1, 1>>>(s1, v1.dev_ptr, s2, v2.dev_ptr, offset, dev_ptr);
		m_checkCudaErrors();
	}
}

