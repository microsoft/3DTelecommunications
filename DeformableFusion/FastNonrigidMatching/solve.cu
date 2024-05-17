#include <string>
#include <random>
#include <algorithm>
#include <chrono>

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include "cublas_v2.h"
#include "cusparse.h"

#include "fast_pcg.cuh"
#include "io.h"

void fast_pcg_test(cublasHandle_t cublas_handle, cusparseHandle_t cusparse_handle, int n, int nnz, cusparseMatDescr_t descr, float* A_val, int* A_rowptr, int* A_colind, float* b, float* x, float tol)
{
	cusparseStatus_t status;

	// Symmetrize the matrix, going to coo format
	thrust::device_ptr<float> A_val_ptr(A_val);
	thrust::host_vector<float> A_val_cpu(A_val_ptr, A_val_ptr + nnz);
	thrust::device_ptr<int> A_rowptr_ptr(A_rowptr);
	thrust::host_vector<int> A_rowptr_cpu(A_rowptr_ptr, A_rowptr_ptr + n + 1);
	thrust::device_ptr<int> A_colind_ptr(A_colind);
	thrust::host_vector<int> A_colind_cpu(A_colind_ptr, A_colind_ptr + nnz);

	int nnz_new = nnz * 2 - n;
	thrust::host_vector<float> A_val_new_unsorted_cpu(nnz_new);
	thrust::host_vector<int> A_rowind_new_cpu(nnz_new);
	thrust::host_vector<int> A_colind_new_cpu(nnz_new);

	int j = 0;
	for (auto r = 0; r < n; r++)
	{
		for (auto i = A_rowptr_cpu[r]; i < A_rowptr_cpu[r + 1]; i++)
		{
			auto c = A_colind_cpu[i];

			A_rowind_new_cpu[j] = r;
			A_colind_new_cpu[j] = c;
			A_val_new_unsorted_cpu[j] = A_val_cpu[i];
			j++;

			if (r != c)
			{
				A_rowind_new_cpu[j] = c;
				A_colind_new_cpu[j] = r;
				A_val_new_unsorted_cpu[j] = A_val_cpu[i];
				j++;
			}
		}
	}

	thrust::device_vector<int> A_rowind_new_vec(A_rowind_new_cpu.begin(), A_rowind_new_cpu.end());
	auto A_rowind_new = thrust::raw_pointer_cast(A_rowind_new_vec.data());
	thrust::device_vector<int> A_colind_new_vec(A_colind_new_cpu.begin(), A_colind_new_cpu.end());
	auto A_colind_new = thrust::raw_pointer_cast(A_colind_new_vec.data());
	thrust::device_vector<float> A_val_new_unsorted_vec(A_val_new_unsorted_cpu.begin(), A_val_new_unsorted_cpu.end());
	auto A_val_new_unsorted = thrust::raw_pointer_cast(A_val_new_unsorted_vec.data());
	thrust::device_vector<float> A_val_new_vec(nnz_new);
	auto A_val_new = thrust::raw_pointer_cast(A_val_new_vec.data());

	// Sort the coo rows and cols
	cusparseMatDescr_t descr_L;
	status = cusparseCreateMatDescr(&descr_L);
	cusparseSetMatIndexBase(descr_L, CUSPARSE_INDEX_BASE_ZERO);
	cusparseSetMatType(descr_L, CUSPARSE_MATRIX_TYPE_GENERAL);
	cusparseSetMatFillMode(descr_L, CUSPARSE_FILL_MODE_LOWER);
	cusparseSetMatDiagType(descr_L, CUSPARSE_DIAG_TYPE_NON_UNIT);

	size_t buffer_size_coo;
	status = cusparseXcoosort_bufferSizeExt(cusparse_handle, n, n, nnz_new, A_rowind_new, A_colind_new, &buffer_size_coo);
	thrust::device_vector<char> buffer_coo_vec(buffer_size_coo);
	auto buffer_coo = thrust::raw_pointer_cast(buffer_coo_vec.data());
	thrust::device_vector<int> perm_coo_vec(nnz_new);
	auto perm_coo = thrust::raw_pointer_cast(perm_coo_vec.data());
	status = cusparseCreateIdentityPermutation(cusparse_handle, nnz_new, perm_coo);
	status = cusparseXcoosortByRow(cusparse_handle, n, n, nnz_new, A_rowind_new, A_colind_new, perm_coo, buffer_coo);
	status = cusparseSgthr(cusparse_handle, nnz_new, A_val_new_unsorted, A_val_new, perm_coo, CUSPARSE_INDEX_BASE_ZERO);

	// Convert the symmetric matrix from coo to csr
	thrust::device_vector<int> A_rowptr_new_vec(n + 1);
	auto A_rowptr_new = thrust::raw_pointer_cast(A_rowptr_new_vec.data());

	status = cusparseXcoo2csr(cusparse_handle, A_rowind_new, nnz_new, n, A_rowptr_new, CUSPARSE_INDEX_BASE_ZERO);

	// Convert the symmetric matrix from csr to bsr
	const int block_size = 12;
	auto block_count = nnz_new / block_size / block_size;
	auto nrows = n / block_size;

	thrust::device_vector<float> A_val_bsr_vec(nnz_new);
	auto A_val_bsr = thrust::raw_pointer_cast(A_val_bsr_vec.data());
	thrust::device_vector<int> A_rowptr_bsr_vec(nrows + 1);
	auto A_rowptr_bsr = thrust::raw_pointer_cast(A_rowptr_bsr_vec.data());

	thrust::device_vector<int> A_colind_bsr_vec(block_count);
	auto A_colind_bsr = thrust::raw_pointer_cast(A_colind_bsr_vec.data());

	cusparseXcsr2bsrNnz(cusparse_handle, CUSPARSE_DIRECTION_ROW, n, n, descr_L, A_rowptr_new, A_colind_new, block_size,
		descr_L, A_rowptr_bsr, &nnz_new);
	status = cusparseScsr2bsr(cusparse_handle, CUSPARSE_DIRECTION_ROW, n, n, descr_L, A_val_new, A_rowptr_new, A_colind_new, block_size,
		descr_L, A_val_bsr, A_rowptr_bsr, A_colind_bsr);

	cusparseDestroyMatDescr(descr_L);

	// README!!! Test starts here, what is before that just sets up your matrix to be block sparse and general (not symmetric)
	fast_pcg pcg(n, block_size);

	auto tic = std::chrono::high_resolution_clock::now();

	pcg.bsr_solve(block_count, A_val_bsr, A_rowptr_bsr, A_colind_bsr, b, x);

	cudaDeviceSynchronize();
	auto toc = std::chrono::high_resolution_clock::now();
	std::cout << "Time: " << std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() / 1000.0f << "ms" << std::endl;
}


void cusparse_pcg_test()
{
	std::string A_val_filename("c:\\users\\samehk\\Downloads\\mingsong\\A_3k_crs_val.txt");
	std::string A_colind_filename("c:\\users\\samehk\\Downloads\\mingsong\\A_3k_colind.txt");
	std::string A_rowptr_filename("c:\\users\\samehk\\Downloads\\mingsong\\A_3k_rowptr.txt");
	std::string b_filename("c:\\users\\samehk\\Downloads\\mingsong\\b_3k.txt");
	std::string x_filename("c:\\users\\samehk\\Downloads\\mingsong\\x_3k_pcg_sameh.txt");

	// Data
	//int n = 744;
	//int nnz = 42276;
	int n = 2508;
	int nnz = 125166;

	auto A_val_cpu = read_file<float>(A_val_filename, nnz);
	thrust::device_vector<float> A_val_vec(A_val_cpu.begin(), A_val_cpu.end());
	auto A_val = thrust::raw_pointer_cast(A_val_vec.data());

	auto A_colind_cpu = read_file<int>(A_colind_filename, nnz);
	thrust::device_vector<int> A_colind_vec(A_colind_cpu.begin(), A_colind_cpu.end());
	auto A_colind = thrust::raw_pointer_cast(A_colind_vec.data());

	auto A_rowptr_cpu = read_file<int>(A_rowptr_filename, n + 1);
	thrust::device_vector<int> A_rowptr_vec(A_rowptr_cpu.begin(), A_rowptr_cpu.end());
	auto A_rowptr = thrust::raw_pointer_cast(A_rowptr_vec.data());

	auto b_cpu = read_file<float>(b_filename, n);
	thrust::device_vector<float> b_vec(b_cpu.begin(), b_cpu.end());
	auto b = thrust::raw_pointer_cast(b_vec.data());

	// x initialized to random numbers
	std::vector<float> x_cpu(n);
	std::default_random_engine generator;
	std::normal_distribution<float> distribution(0, 1.0);
	std::generate(x_cpu.begin(), x_cpu.end(), [&]() { return distribution(generator); });
	thrust::device_vector<float> x_vec(n);
	auto x = thrust::raw_pointer_cast(x_vec.data());

	// Sparse solve
	cusparseStatus_t status;
	cublasHandle_t cublas_handle;
	cublasCreate(&cublas_handle);
	cublasSetPointerMode(cublas_handle, CUBLAS_POINTER_MODE_DEVICE);
	cublasSetAtomicsMode(cublas_handle, CUBLAS_ATOMICS_ALLOWED);

	cusparseHandle_t cusparse_handle;
	cusparseCreate(&cusparse_handle);

	cusparseMatDescr_t descr;
	status = cusparseCreateMatDescr(&descr);
	cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);
	cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_SYMMETRIC);
	cusparseSetMatFillMode(descr, CUSPARSE_FILL_MODE_LOWER);
	cusparseSetMatDiagType(descr, CUSPARSE_DIAG_TYPE_NON_UNIT);
	
	fast_pcg_test(cublas_handle, cusparse_handle, n, nnz, descr, A_val, A_rowptr, A_colind, b, x, 20);

	cusparseDestroyMatDescr(descr);
	cusparseDestroy(cusparse_handle);
	cublasDestroy(cublas_handle);

	// Output
	thrust::copy(x_vec.begin(), x_vec.end(), x_cpu.begin());
	write_file(x_cpu, x_filename);
}
