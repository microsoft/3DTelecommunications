#ifndef __MATRIX_TYPES_CUDA_H__
#define __MATRIX_TYPES_CUDA_H__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

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
	cuda::gpu_size_data blks_num;
	cuda::gpu_size_data para_blks_num;
	int para_blk_dim;
};

struct JtFVector
{
	float* jtf;
	cuda::gpu_size_data n;
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
	cuda::gpu_size_data para_blks_num; //in CPU: dev_ptr=NULL; max_size is the number
	cuda::gpu_size_data blks_num;
};

inline void allocate_BlockMatrixFullCR_GPU(BlockMatrixFullCR &blk_mat_bscr, int blks_num_ltri_max, int para_blks_num_max, int para_blk_dim)
{
	blk_mat_bscr.para_blk_dim = para_blk_dim;
	blk_mat_bscr.para_blks_num.allocate_once();
	blk_mat_bscr.blks_num.allocate_once();

	blk_mat_bscr.para_blks_num.max_size = para_blks_num_max;
	blk_mat_bscr.blks_num.max_size = blks_num_ltri_max * 2;

	int blks_num_max = blks_num_ltri_max * 2;
	int blk_size = para_blk_dim*para_blk_dim;

	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.brow_ptr), sizeof(int)*(para_blks_num_max + 1)));
	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.bcolind), sizeof(int)*blks_num_max));
	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.browind), sizeof(int)*blks_num_max));
	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.diag_blk_pos), sizeof(int)*para_blks_num_max));
	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.extern_data_offset), sizeof(int)*blks_num_max));
	checkCudaErrors(cudaMalloc(&(blk_mat_bscr.data), sizeof(int)*blks_num_max*blk_size));
}


#endif