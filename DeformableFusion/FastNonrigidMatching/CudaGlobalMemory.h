#ifndef __CUDAGLOBALMEMORY_H__
#define __CUDAGLOBALMEMORY_H__
#include <cuda_runtime.h>
#include <helper_cuda.h>
#include <stdio.h>
#include <vector>

// Keeping this define since it might be very useful to debug memory problems
//#define DEBUG_MEMORY_ACCESS_VIOLATION

//TODO: thread safe
class CudaGlobalMemoryStatic
{
public:
	static bool init(long long size)
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		bytes_allocated = 0;
		bytes_max = 0;
		dev_global_mem = NULL;
#else
		checkCudaErrors(cudaMalloc(&dev_global_mem, size));
		bytes_allocated = 0;
		bytes_max = size;
#endif
		return true;
	}
	static bool cleanup()
	{
#ifndef DEBUG_MEMORY_ACCESS_VIOLATION
		checkCudaErrors(cudaFree(dev_global_mem));
		bytes_allocated = 0;
#endif
		return true;
	}
	static void take_back_allocation()
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		for(int i=0; i<dev_pts.size(); i++)
			checkCudaErrors(cudaFree(dev_pts[i]));
#else
		bytes_allocated = 0;
#endif
	}

	static void return_memory(int size_in_bytes)
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		;
#else
		int size_cur = (size_in_bytes + 127) / 128 * 128;
		bytes_allocated -= size_cur;
		if (bytes_allocated < 0)
		{
			printf("\n\nError<CudaGlobalMemory::return_memory>: bytes_allocated < 0!!!!!!!!!!!\n\n");
		}
#endif
	}

	static void take_back_most_recent_allcoation()
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		;
#else
		long long size_cur = size_mem_segments.back();
		size_mem_segments.pop_back();
		bytes_allocated -= size_cur;
		if (bytes_allocated < 0)
		{
			printf("\n\nError<CudaGlobalMemory::return_memory>: bytes_allocated < 0!!!!!!!!!!!\n\n");
		}
#endif
	}


#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
	static char* allocate_dev_memory(int size, void* dev_ptr)
	{
		checkCudaErrors(cudaMalloc(&dev_ptr, size));
		return (char*)dev_ptr;
	}
#else
	static char* allocate_dev_memory(int size, void* dummy=NULL)
	{
		if (size == 0)
		{
			printf("!!!!!!!!!!!!!!!!!!!!!Error<CudaGlobalMemoryStatic::allocate_dev_memory>: size == 0\n");
			return NULL;
		}

		//make sure the memory is 128-bytes align
		char* ptr = dev_global_mem + bytes_allocated;
		int size_cur = (size + 127) / 128 * 128;
		bytes_allocated += size_cur;
		if (bytes_allocated < bytes_max)
		{
			size_mem_segments.push_back(size_cur);
			return ptr;
		}
		else
		{
			printf("\n\nError<CudaGlobalMemory::allocate_dev_memory>: memory limitation reached!!!!!!!!!!!\n\n");
			return NULL;
		}
	}
#endif

	static long long bytes_allocated;
	static long long bytes_max;
	static char* dev_global_mem;
	static std::vector<void*> dev_pts;
	static std::vector<long long> size_mem_segments;
};

//TODO: thread safe
class CudaGlobalMemory
{
public:
	CudaGlobalMemory(long long size)
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		bytes_allocated = 0;
		bytes_max = 0;
		dev_global_mem = NULL;
#else
		checkCudaErrors(cudaMalloc(&dev_global_mem, size));
		bytes_allocated = 0;
		bytes_max = size;
#endif
	}
	~CudaGlobalMemory()
	{
		checkCudaErrors(cudaFree(dev_global_mem));
		bytes_allocated = 0;
	}

public:
	void take_back_allocation()
	{
#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
		for (int i = 0; i < dev_pts.size(); i++)
			checkCudaErrors(cudaFree(dev_pts[i]));
#else
		bytes_allocated = 0;
#endif
	}

#ifdef DEBUG_MEMORY_ACCESS_VIOLATION
	static char* allocate_dev_memory(int size, void* dev_ptr)
	{
		checkCudaErrors(cudaMalloc(&dev_ptr, size));
		return (char*)dev_ptr;
	}
#else
	char* allocate_dev_memory(int size, void* dummy=NULL)
	{
		if (size == 0)
		{
			printf("!!!!!!!!!!!!!!!!!!!!!Error<CudaGlobalMemoryStatic::allocate_dev_memory>: size == 0\n");
			return NULL;
		}

		//make sure the memory is 128-bytes align
		char* ptr = dev_global_mem + bytes_allocated;
		bytes_allocated += (size+127)/128*128;
		if (bytes_allocated < bytes_max)
			return ptr;
		else
		{
			printf("\n\nError<CudaGlobalMemory::allocate_dev_memory>: memory limitation reached!!!!!!!!!!!\n\n");
			return NULL;
		}
	}
#endif

private:
	long long bytes_allocated;
	long long bytes_max;
	char* dev_global_mem;
	std::vector<void*> dev_pts;
};

#endif