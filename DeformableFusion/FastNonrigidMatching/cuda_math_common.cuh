#ifndef __CUDA_MATH_COMMON_CUL__
#define __CUDA_MATH_COMMON_CUL__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "math_constants.h"
#include <cuda.h>
#include <math.h>
#include "../Common/debug.h"
#include "../../Peabody.h"

#define MAX_THREADS_PER_BLOCK 1024
#define M_EPS 1.0e-6f

#ifndef MAX
#define MAX(x, y) (((x)>(y))?(x):(y))
#endif
#ifndef MIN
#define MIN(x, y) (((x)<(y))?(x):(y))
#endif
#ifndef ROUND
#define ROUND(x)  (((x)>0)?((int)((x)+0.5f)):((int)((x)-0.5f))) //note: (int)(-1.8) = -1
#endif


template<class T>
__host__ __device__ void cuda_copy(T const* src, T *dst, unsigned int size)
{
#pragma unroll
	for (int i = 0; i < size; i++)
		dst[i] = src[i];
}

template<class T, int N>
__host__ __device__ void cuda_memset(T *data, T val)
{
#pragma unroll
	for (int i = 0; i < N; i++)
		data[i] = val;
}

template<class T>
__host__ __device__ void cuda_memset(T *data, int N, T val)
{
	for (int i = 0; i < N; i++)
		data[i] = val;
}

template<int N>
__forceinline__ __host__ __device__ float dist_square(float const*a, float const*b)
{
	float ret = 0.0f;
#pragma unroll
	for (int i = 0; i < N; i++)
		ret += (a[i] - b[i])*(a[i] - b[i]);
	return ret;
}

//i = id1*(id1-1)/2 + id2: does not include the diagonal
inline __host__ __device__ void decompose_triangle_id(int i, int &id1, int &id2)
{
	float x = sqrtf((float)(2 * i));
	id1 = (int)x;
	id2 = i - id1*(id1 - 1) / 2;
	if (id1 <= id2)
	{
		id1++;
		id2 = i - id1*(id1 - 1) / 2;
	}
}

//i = id1*(id1+1)/2 + id2: include the diagonal
inline __host__ __device__ void decompose_triangle_id_with_diag(int i, int &id1, int &id2)
{
	float x = sqrtf((float)(2 * i+1));
	id1 = (int)x - 1;
	id2 = i - id1*(id1 + 1) / 2;
	if (id1 < id2)
	{
		id1++;
		id2 = i - id1*(id1 + 1) / 2;
	}
}

inline __host__ __device__ void normalize(float3 &v)
{
	float s = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
	if (s > M_EPS)
	{
		v.x /= s;
		v.y /= s;
		v.z /= s;
	}
}

template<int N>
inline __host__ __device__ void normalize(float *v)
{
	float mag = 0.0f;
	#pragma unroll
	for (int i = 0; i < N; i++)
		mag += v[i]*v[i];
	if (mag > M_EPS)
	{
		mag = sqrtf(mag);
		#pragma unroll
		for (int i = 0; i < N; i++)
			v[i] /= mag;
	}
}

__forceinline__ __device__ void depth_remove_top_bit(unsigned short &d)
{
	d &= 0x7fff;
}

// foreground hard-thresholding by depth value
__forceinline__ __device__ void depth_extract_fg(unsigned short &d)
{
#ifndef DISABLE_FOREGROUND_DEPTH_THRESHOLD
	if (d >= 0x8000)
		d &= 0x7fff;
	else
		d = 0;
#endif // DISABLE_FOREGROUND_DEPTH_THRESHOLD
}

__forceinline__ __device__ void depth_extract_fg_set_bg_as_far(unsigned short &d)
{
	if (d >= 0x8000)
		d &= 0x7fff;
	else 
		d = 0x7fff;
}

#endif