#pragma once
#include <cuda_runtime.h>
#include <helper_cuda.h>

#include "../debug.h"


namespace cuda
{
	inline
	void * getSymbolAddress( void const * symbol )
	{
		void * res;
		checkCudaErrors(cudaGetSymbolAddress(&res, symbol));
		return res;
	}

	struct gpu_size_data
	{
		gpu_size_data()
		: dev_ptr(0)
		, max_size(0)
		{}

		gpu_size_data( int * dev_ptr, int max_size )
		: dev_ptr(dev_ptr)
		, max_size(max_size)
		{}

		int * dev_ptr;
		int   max_size;

		int debug_sync_read(bool verbose=false) const
		{
			int res;
			checkCudaErrors(cudaMemcpy(&res, dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
			if (verbose)
				printf(">>>>>>>>>>>>>>>> debug_sync_read = %d \n", res); // ignore warnings when in debug
			return res;
		}

		void allocate_once()
		{
			if (!dev_ptr)
			{
				checkCudaErrors(cudaMalloc(&dev_ptr, sizeof(int)));
			}
			// TODO: free!
		}

		void copy_transformed( gpu_size_data const & other, int scale, int offset ); // this = scale * other + offset
		void multiply_add( int s1, gpu_size_data const & v1, int s2, gpu_size_data const & v2, int offset );  // this = s1 * v1 + s2 * v2 + offset
		void multiply_add_gpu_only(int s1, gpu_size_data const & v1, int s2, gpu_size_data const & v2, int offset); //only do calculation on gpu variable
		void cap_gpu_size(int max_bound);

		void debug_set( int value, int max_value )
		{
			checkCudaErrors(cudaMemcpy(dev_ptr, &value, sizeof(int), cudaMemcpyHostToDevice));
			max_size = max_value;
		}

		int sync_read() const
		{
			int res;
			checkCudaErrors(cudaMemcpy(&res, dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
			return res;
		}

		void debug_print( char const * str ) const
		{
			printf("%s(%p, %d)\n", str, dev_ptr, max_size);
		}
	};
}

