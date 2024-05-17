#ifndef __MATRIX_TYPES_CUDA_H__
#define __MATRIX_TYPES_CUDA_H__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

struct HessianBlockInfoCuda
{
	int pi_idx;
	int pj_idx;
	int pi_dim; //space for future
	int pj_dim;

	int vtii_first;
	int vts_num;

	int data_offset; //where is the data in dev_jtj_
};

struct JtJBlockMatrixLTri
{
	HessianBlockInfoCuda *blk_info;
	float* data;
	//int blks_num;
	//int para_blks_num;
	cuda::gpu_size_data blks_num_gpu;
	cuda::gpu_size_data para_blks_num_gpu;
	int para_blk_dim;
};

struct JtFVector
{
	float* jtf;
	int n;
};

//full block matrix: compressed row
struct BlockMatrixFullCR
{
//note the pointers might point to resources on GPU
	int *brow_ptr;
	int *bcolind;
	float* data; //data row-major order: in the order of bcolind

	int *diag_blk_pos; // the position of the diagonal blocks
	int* browind;// the row indices
	int* extern_data_offset; //data offset for external memory

	int para_blk_dim;
	//int para_blks_num;
	//int blks_num;
	cuda::gpu_size_data para_blks_num_gpu;
	cuda::gpu_size_data blks_num_gpu;
};


#endif