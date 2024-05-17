#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
namespace PCGSover_Sam{

	__global__ void sparse_gemv_kernel(int nrows, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x);
	__global__ void extract_diag_kernel(int nnz, int nrows, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag);
	__global__ void sparse_block_gemv_kernel(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x);
	__global__ void extract_block_diag_kernel(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag);
	template <bool forward = true>
	__global__ void solve_block_diag_kernel(int nblocks, int block_size, const float* L, const float* b, float* x);
	__global__ void full_solve_block_diag_kernel(int nblocks, int block_size, const float* L, const float* b, float* x);
	__global__ void icholesky_block_diag_kernel(int nblocks, int block_size, float* A);
	__global__ void axpy_kernel(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
	__global__ void aypx_kernel(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
	__global__ void dot_product_kernel(int n, const float *a, const float *b, float *c);

	class fast_pcg
	{
	public:
		fast_pcg(int nout, int block_size = 1);

		void bsr_solve(int nblocks, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x, int max_iter = 100);
		void csr_solve(const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x, int max_iter = 100) { } // TODO easy to implement now

	private:
		void axpy(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
		void aypx(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
		void forward_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x);
		void backward_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x);
		void full_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x);
		void icholesky_block_diag(int nblocks, int block_size, float* A);
		void extract_block_diag(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag);
		void dot_product(int n, const float *a, const float *b, float *c);
		void sparse_block_gemv(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x);

		thrust::device_vector<float> p, omega, z, r;
		float *d_p, *d_omega, *d_z, *d_r;
		int nout, block_size;

		thrust::device_vector<float> L;
		float* d_L;

		thrust::device_vector<float> vars_vec;
		float *floatone, *floatzero, *floatnegone;
		float *numerator, *denominator, *dot, *alpha, *negalpha, *beta;
	};
}