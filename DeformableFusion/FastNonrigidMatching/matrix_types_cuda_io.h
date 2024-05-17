#ifndef __MATRIX_TYPES_CUDA_IO_H__
#define __MATRIX_TYPES_CUDA_IO_H__

#include "matrix_types_cuda.h"
#include "opencv2\opencv.hpp"
#include "UtilMatrix.h"
#include "helper_cuda.h"
#include "BlockedHessianMatrix.h"
#include "CudaGlobalMemory.h"
#include "utility.h"

//operation on CPU memory
bool save_BlockMatrixFullCR_ascii(char const* filename, BlockMatrixFullCR const&A_bcsr);
bool load_BlockMatrixFullCR_ascii(char const* filename, BlockMatrixFullCR &A_bcsr);

bool save_HessianBlockInfoCuda_buf_ASCII(char const* file_name, HessianBlockInfoCuda const* hessian_block_info, int count);

bool readout_JtJBlockMatrixLTri_from_GPU(JtJBlockMatrixLTri const&jtj_gpu, BlockedHessianMatrix &jtj_cpu);

bool free_BlockMatrixFullCR_host(BlockMatrixFullCR &bcsr_mat);

// Note: if bcsr_dst is on the host, it does not need to allocate memory beforehand. 
//       otherwise the gpu memory should be setup beforehand.
bool BlockMatrixFullCR_cudaMemcpy(BlockMatrixFullCR const& bcsr_src, BlockMatrixFullCR &bcsr_dst, cudaMemcpyKind kind);

cv::Mat* BlockMatrixFullCR_to_dense(BlockMatrixFullCR const&bcsr_mat);

#endif