#include "fast_pcg.cuh"

namespace PCGSover_Sam{

	const int CUDA_MAX_NUM_THREADS = 1024;
	const int CUDA_WARP_SIZE = 32;
	const int LOG_CUDA_WARP_SIZE = 5;
#ifdef __CUDACC__
#define CUDA_NUM_BLOCKS(N, NUM_THREADS) (((N) + (NUM_THREADS) - 1) / (NUM_THREADS))
#define KERNEL_CALL(FUNC, N, NUM_THREADS) (FUNC)<<<CUDA_NUM_BLOCKS(N, NUM_THREADS), NUM_THREADS>>>
#define KERNEL_CALL_FULL(FUNC, N, NUM_THREADS, SHMEM, STREAM) (FUNC)<<<CUDA_NUM_BLOCKS(N, NUM_THREADS), NUM_THREADS, SHMEM, STREAM>>>
#else
#define KERNEL_CALL(FUNC, N, NUM_THREADS) (FUNC)
#define KERNEL_CALL_FULL(FUNC, N, NUM_THREADS, SHMEM, STREAM) (FUNC)
#endif

#ifdef __CUDACC__
#define CUDA_LOOP_OFFSET (blockDim.x * blockIdx.x + threadIdx.x)
#define CUDA_LOOP_STRIDE (blockDim.x * gridDim.x)
#else
#define CUDA_LOOP_OFFSET 0
#define CUDA_LOOP_STRIDE 1
#endif


	fast_pcg::fast_pcg(int nout, int block_size) : nout(nout), block_size(block_size)
	{
		// Better way to allocate floats on GPU?
		vars_vec.resize(10);
		auto vars = thrust::raw_pointer_cast(vars_vec.data());

		numerator = &vars[0]; auto numerator_ptr = thrust::device_ptr<float>(numerator);
		denominator = &vars[1]; auto denominator_ptr = thrust::device_ptr<float>(denominator);
		alpha = &vars[2]; auto alpha_ptr = thrust::device_ptr<float>(alpha);
		beta = &vars[3]; auto beta_ptr = thrust::device_ptr<float>(beta);
		dot = &vars[4]; auto dot_ptr = thrust::device_ptr<float>(dot);
		negalpha = &vars[5]; auto negalpha_ptr = thrust::device_ptr<float>(negalpha);

		thrust::fill(vars_vec.begin() + 6, vars_vec.begin() + 7, 1);
		floatone = &vars[6];
		thrust::fill(vars_vec.begin() + 7, vars_vec.begin() + 8, 0);
		floatzero = &vars[7];
		thrust::fill(vars_vec.begin() + 8, vars_vec.begin() + 9, -1);
		floatnegone = &vars[8];

		// Allocate p (conjugte vector)
		p.resize(nout);
		d_p = thrust::raw_pointer_cast(p.data());

		// Allocate omega (temporary for A p)
		omega.resize(nout);
		d_omega = thrust::raw_pointer_cast(omega.data());

		// Allocate z (M^-1 r)
		z.resize(nout);
		d_z = thrust::raw_pointer_cast(z.data());

		// Allocate r (residual)
		r.resize(nout);
		d_r = thrust::raw_pointer_cast(r.data());

		// Allocate L (decomposed conditioner)
		auto nnz_diag = nout * block_size;
		L.resize(nnz_diag);
		d_L = thrust::raw_pointer_cast(L.data());
	}

	void fast_pcg::bsr_solve(int nblocks, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x, int max_iter)
	{
		int nbrows = nout / block_size;

		// Extract the block diagnal as bsr
		extract_block_diag(nblocks, nbrows, block_size, A_val, A_rowptr, A_colind, d_L);

		// Set up M = L LT (preconditioner) using incomplete Cholesky factorization of the blocks
		icholesky_block_diag(nbrows, block_size, d_L);

		// Initialize the residual
		sparse_block_gemv(nblocks, nbrows, block_size, A_val, A_rowptr, A_colind, x, d_r);
		aypx(nout, floatnegone, floatone, floatone, b, d_r); // r = A x - b

		// Initialize z and p
		full_solve_block_diag(nbrows, block_size, d_L, d_r, d_z); // solve L r = y then LT y = z
		cudaMemcpyAsync(d_p, d_z, nout * sizeof(float), cudaMemcpyDeviceToDevice); // p = z = M^-1 r
		dot_product(nout, d_r, d_z, numerator); // numerator = rT z

		int k = 0;
		for (k = 0; k < max_iter; k++)
		{
			// Calculate alpha
			sparse_block_gemv(nblocks, nbrows, block_size, A_val, A_rowptr, A_colind, d_p, d_omega); // omega = A p
			dot_product(nout, d_p, d_omega, dot); // dot = pT (A p)

			// Update x and r
			axpy(nout, floatone, numerator, dot, d_p, x); // x = x + alpha p
			axpy(nout, floatnegone, numerator, dot, d_omega, d_r); // r = r - alpha (A p)

			// Recalculate z
			full_solve_block_diag(nbrows, block_size, d_L, d_r, d_z); // solve L r = y then LT y = z

			// Update numerator and denominator
			cudaMemcpyAsync(denominator, numerator, sizeof(float), cudaMemcpyDeviceToDevice);
			dot_product(nout, d_r, d_z, numerator); // numerator = rT z

			// Recalculate p
			aypx(nout, floatone, numerator, denominator, d_z, d_p); // p = beta p + z
		}
	}

	// TODO need test
	__global__ void sparse_gemv_kernel(int nrows, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x)
	{
		extern __shared__ float temp[]; // cache
		float* valtemp = &temp[0];

		int i = blockDim.x * blockIdx.x + threadIdx.x;
		int warp = i >> LOG_CUDA_WARP_SIZE; // warp id (per output element)
		int lane = i & (CUDA_WARP_SIZE - 1); // id within warp

		if (warp < nrows)
		{
			int jstart = A_rowptr[warp], jend = A_rowptr[warp + 1]; // blocks to read
			float val = 0;

#pragma unroll
			for (int j = jstart + lane; j < jend; j += CUDA_WARP_SIZE)
				val += A_val[j] * b[A_colind[j]];

			valtemp[threadIdx.x] = val;

			// reduce over warp
			if (lane < 16) valtemp[threadIdx.x] += valtemp[threadIdx.x + 16];
			if (lane < 8) valtemp[threadIdx.x] += valtemp[threadIdx.x + 8];
			if (lane < 4) valtemp[threadIdx.x] += valtemp[threadIdx.x + 4];
			if (lane < 2) valtemp[threadIdx.x] += valtemp[threadIdx.x + 2];
			if (lane < 1) valtemp[threadIdx.x] += valtemp[threadIdx.x + 1];

			if (lane == 0)
				x[warp] = valtemp[threadIdx.x];
		}
	}

	// TODO need test
	__global__ void extract_diag_kernel(int nnz, int nrows, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag)
	{
		int i = blockDim.x * blockIdx.x + threadIdx.x; // index in A

		if (i < nnz)
		{
			int c = A_colind[i];
			int jstart = A_rowptr[c], jend = A_rowptr[c + 1];

			// Copy only if you exist in the row corresponding to that column
			if (i >= jstart && i < jend)
				A_diag[c] = A_val[i];
		}
	}

	__global__ void sparse_block_gemv_kernel(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x)
	{
		extern __shared__ float temp[]; // cache
		float* valtemp = &temp[0];
		float* btemp = &temp[block_size * CUDA_WARP_SIZE];

		int n = nrows * block_size;
		int b2 = block_size * block_size;
		int i = blockDim.x * blockIdx.x + threadIdx.x;
		int warp = i >> LOG_CUDA_WARP_SIZE; // warp id (per output element)
		int lane = i & (CUDA_WARP_SIZE - 1); // id within warp

		if (warp < n)
		{
			int br = warp / block_size; // block row for warp
			int local_warp = warp % block_size;
			int bc; // block column

			int bidstart = A_rowptr[br], bidend = A_rowptr[br + 1]; // blocks to read
			int offset = local_warp * block_size; // warp offset in block
			int ai, bi;

			// Cache b
			int bcount = (bidend - bidstart) * block_size;
#pragma unroll
			for (int j = threadIdx.x; j < bcount; j += blockDim.x)
			{
				bc = A_colind[bidstart + j / block_size];
				bi = bc * block_size + j % block_size;
				btemp[threadIdx.x] = b[bi];
			}
			__syncthreads();

			float val = 0;

#pragma unroll
			for (int bid = bidstart; bid < bidend; bid++)
			{
				ai = bid * b2 + offset;
				bi = (bid - bidstart) * block_size;

				// calculate temporary sum for current thread
				if (lane < block_size)
					val += A_val[ai + lane] * btemp[bi + lane];
			}

			valtemp[threadIdx.x] = val;

			// reduce over warp
			if (lane < 16) valtemp[threadIdx.x] += valtemp[threadIdx.x + 16];
			if (lane < 8) valtemp[threadIdx.x] += valtemp[threadIdx.x + 8];
			if (lane < 4) valtemp[threadIdx.x] += valtemp[threadIdx.x + 4];
			if (lane < 2) valtemp[threadIdx.x] += valtemp[threadIdx.x + 2];
			if (lane < 1) valtemp[threadIdx.x] += valtemp[threadIdx.x + 1];

			if (lane == 0)
				x[warp] = valtemp[threadIdx.x];
		}
	}

	__global__ void extract_block_diag_kernel(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag)
	{
		int i = blockDim.x * blockIdx.x + threadIdx.x; // index in A

		if (i < nblocks * block_size * block_size)
		{
			int bc = A_colind[blockIdx.x];
			int bidstart = A_rowptr[bc], bidend = A_rowptr[bc + 1];

			// Copy only if you exist in the row corresponding to that column
			if (blockIdx.x >= bidstart && blockIdx.x < bidend)
				A_diag[bc * blockDim.x + threadIdx.x] = A_val[i];
		}
	}

	template <bool forward>
	__global__ void solve_block_diag_kernel(int nblocks, int block_size, const float* L, const float* b, float* x)
	{
		extern __shared__ float temp[]; // caching
		float* xtemp = &temp[0];
		float* Ltemp = &temp[block_size];

		int li = blockDim.x * blockIdx.x; // first index in L
		int i = li + threadIdx.x; // index in x and b

		if (i < nblocks * block_size)
		{
			// Cache
#pragma unroll
			for (int j = 0; j < block_size; j++)
				Ltemp[threadIdx.x * block_size + j] = L[li + threadIdx.x * block_size + j];

			float diag = Ltemp[block_size * threadIdx.x + threadIdx.x];
			xtemp[threadIdx.x] = b[i] / diag;
			if (forward)
			{
				int k = block_size * threadIdx.x;
				for (int j = 0; j < block_size - 1; j++)
				{
					__syncthreads();

					if (threadIdx.x > j)
						xtemp[threadIdx.x] -= Ltemp[k++] / diag * xtemp[j];
				}
			}
			else
			{
				int k = threadIdx.x + block_size * (block_size - 1);
				for (int j = block_size - 1; j >= 1; j--)
				{
					__syncthreads();

					if (threadIdx.x < j)
						xtemp[threadIdx.x] -= Ltemp[k] / diag * xtemp[j];
					k -= block_size;
				}
			}
			x[i] = xtemp[threadIdx.x];
		}
	}

	__global__ void full_solve_block_diag_kernel(int nblocks, int block_size, const float* L, const float* b, float* x)
	{
		extern __shared__ float temp[]; // result of first solve then caching
		float* xtemp = &temp[0];
		float* Ltemp = &temp[block_size];

		int li = block_size * block_size * blockIdx.x; // first index in L
		int i = blockDim.x * blockIdx.x + threadIdx.x; // index in x and b

		if (i < nblocks * block_size)
		{
			// Cache
#pragma unroll
			for (int j = 0; j < block_size; j++)
				Ltemp[threadIdx.x * block_size + j] = L[li + threadIdx.x * block_size + j];

			float diag = Ltemp[block_size * threadIdx.x + threadIdx.x];
			xtemp[threadIdx.x] = b[i] / diag;

			int k = block_size * threadIdx.x;
			for (int j = 0; j < block_size - 1; j++)
			{
				__syncthreads();

				if (threadIdx.x > j)
					xtemp[threadIdx.x] -= Ltemp[k++] / diag * xtemp[j];
			}
			// temp now is result of L y = b, solve for LT x = y next

			xtemp[threadIdx.x] /= diag;

			k = threadIdx.x + block_size * (block_size - 1);
			for (int j = block_size - 1; j >= 1; j--)
			{
				__syncthreads();

				if (threadIdx.x < j)
					xtemp[threadIdx.x] -= Ltemp[k] / diag * xtemp[j];
				k -= block_size;
			}

			x[i] = xtemp[threadIdx.x];
		}
	}

	__global__ void icholesky_block_diag_kernel(int nblocks, int block_size, float* A)
	{
		extern __shared__ float temp[]; // caching

		int i = blockDim.x * blockIdx.x + threadIdx.x; // index in A

		if (i < nblocks * block_size * block_size)
		{
			int r = threadIdx.x / block_size;
			int c = threadIdx.x % block_size;

			temp[threadIdx.x] = A[i];
			for (int k = 0; k < block_size; k++)
			{
				if (k == c && c == r) // diagonal element for current iteration
					temp[threadIdx.x] = sqrtf(temp[threadIdx.x]);
				__syncthreads();

				if (k == c && c < r) // elements in its lower column
					temp[threadIdx.x] /= temp[k * block_size + k];
				__syncthreads();

				if (k < c && c <= r) // elements in the remaining lower triangular matrix
					temp[threadIdx.x] -= temp[r * block_size + k] * temp[c * block_size + k];
				__syncthreads();
			}
			A[i] = temp[threadIdx.x];
		}
	}

	__global__ void axpy_kernel(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
	{
		int i = blockDim.x * blockIdx.x + threadIdx.x;
		float denom = denominator[0];
		if (i < n && denom > 1e-8) y[i] = alpha[0] * (numerator[0] / denom) * x[i] + y[i];
	}

	__global__ void aypx_kernel(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
	{
		int i = blockDim.x * blockIdx.x + threadIdx.x;
		float denom = denominator[0];
		if (i < n && denom > 1e-8) y[i] = alpha[0] * (numerator[0] / denom) * y[i] + x[i];
	}


	__global__ void dot_product_kernel(int n, const float *a, const float *b, float *c)
	{
		__shared__ float temp[CUDA_MAX_NUM_THREADS];

		int i = blockDim.x * blockIdx.x + threadIdx.x;
		temp[threadIdx.x] = i < n ? a[i] * b[i] : 0;

		if (i == 0) c[0] = 0;
		__syncthreads();

		for (int s = blockDim.x >> 1; s > 0; s >>= 1)
		{
			if (threadIdx.x < s)
				temp[threadIdx.x] += temp[threadIdx.x + s];
			__syncthreads();
		}

		if (threadIdx.x == 0)
			atomicAdd(c, temp[0]);
	}

	void fast_pcg::axpy(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
	{
		KERNEL_CALL(axpy_kernel, n, CUDA_MAX_NUM_THREADS)(n, alpha, numerator, denominator, x, y);
	}

	void fast_pcg::aypx(int n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
	{
		KERNEL_CALL(aypx_kernel, n, CUDA_MAX_NUM_THREADS)(n, alpha, numerator, denominator, x, y);
	}

	void fast_pcg::forward_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x)
	{
		//int threads_per_block = int(CUDA_MAX_NUM_THREADS / block_size) * block_size;
		int shmem_size = (block_size * block_size + block_size) * sizeof(float);
		KERNEL_CALL_FULL(solve_block_diag_kernel<true>, nblocks * block_size, block_size, shmem_size, 0)(nblocks, block_size, L, b, x);
	}

	void fast_pcg::backward_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x)
	{
		//int threads_per_block = int(CUDA_MAX_NUM_THREADS / block_size) * block_size;
		int shmem_size = (block_size * block_size + block_size) * sizeof(float);
		KERNEL_CALL_FULL(solve_block_diag_kernel<false>, nblocks * block_size, block_size, shmem_size, 0)(nblocks, block_size, L, b, x);
	}

	void fast_pcg::full_solve_block_diag(int nblocks, int block_size, const float* L, const float* b, float* x)
	{
		//int threads_per_block = int(CUDA_MAX_NUM_THREADS / block_size) * block_size;
		int shmem_size = (block_size * block_size + block_size) * sizeof(float);
		KERNEL_CALL_FULL(full_solve_block_diag_kernel, nblocks * block_size, block_size, shmem_size, 0)(nblocks, block_size, L, b, x);
	}

	void fast_pcg::icholesky_block_diag(int nblocks, int block_size, float* A)
	{
		int b2 = block_size * block_size;
		//int threads_per_block = int(CUDA_MAX_NUM_THREADS / b2) * b2;
		KERNEL_CALL_FULL(icholesky_block_diag_kernel, nblocks * b2, b2, b2 * sizeof(float), 0)(nblocks, block_size, A);
	}

	void fast_pcg::extract_block_diag(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag)
	{
		int b2 = block_size * block_size;
		//int threads_per_block = int(CUDA_MAX_NUM_THREADS / b2) * b2;
		KERNEL_CALL_FULL(extract_block_diag_kernel, nblocks * b2, b2, 0, 0)(nblocks, nrows, block_size, A_val, A_rowptr, A_colind, A_diag);
	}

	void fast_pcg::dot_product(int n, const float *a, const float *b, float *c)
	{
		KERNEL_CALL(dot_product_kernel, n, CUDA_MAX_NUM_THREADS)(n, a, b, c);
	}

	void fast_pcg::sparse_block_gemv(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x)
	{
		int threads_per_block_row = block_size * CUDA_WARP_SIZE;
		int shmem_size = threads_per_block_row * sizeof(float)+threads_per_block_row * sizeof(float);
		KERNEL_CALL_FULL(sparse_block_gemv_kernel, nrows * threads_per_block_row, threads_per_block_row, shmem_size, 0)(nblocks, nrows, block_size, A_val, A_rowptr, A_colind, b, x);
	}

}