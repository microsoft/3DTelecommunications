#pragma once
#include <cuda_runtime.h>

namespace cuda
{
	template<class T>
	class PinnedMemory
	{
	public:
		PinnedMemory()
		{
			cudaMallocHost(&memory, sizeof(T));
		}

		PinnedMemory(T const & init)
		{
			cudaMallocHost(&memory, sizeof(T));
			*memory = init;
		}

		PinnedMemory & operator = (T const & value)
		{
			*memory = value;
			return *this;
		}

		T * memory;
	};

	template<class T>
	class PinnedMemoryArray
	{
	public:
		PinnedMemoryArray(int n)
		{
			cudaMallocHost(&memory, sizeof(T)* n);
		}

		T * memory;
	};
}
