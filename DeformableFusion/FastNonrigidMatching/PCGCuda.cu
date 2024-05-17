const int CUDA_MAX_NUM_THREADS = 1024;
const int CUDA_WARP_SIZE = 32;
const int LOG_CUDA_WARP_SIZE = 5;
#define CUDA_NUM_BLOCKS(N, NUM_THREADS) (((N) + (NUM_THREADS) - 1) / (NUM_THREADS))
#define KERNEL_CALL(FUNC, N, NUM_THREADS) (FUNC)<<<CUDA_NUM_BLOCKS(N, NUM_THREADS), NUM_THREADS>>>
#define KERNEL_CALL_FULL(FUNC, N, NUM_THREADS, SHMEM, STREAM) (FUNC)<<<CUDA_NUM_BLOCKS(N, NUM_THREADS), NUM_THREADS, SHMEM, STREAM>>>


#define CUDA_LOOP_OFFSET (blockDim.x * blockIdx.x + threadIdx.x)
#define CUDA_LOOP_STRIDE (blockDim.x * gridDim.x)

#define THREADS_PER_BLOCK 160


void PCGCuda::allocate_memory(int para_blk_num_max, int para_blk_dim)
{
	int blk_size = para_blk_dim*para_blk_dim;
	checkCudaErrors(cudaMalloc(&dev_A_diag_blks_, sizeof(float)*blk_size*para_blk_num_max));
	checkCudaErrors(cudaMalloc(&dev_A_diag_blks_inv_, sizeof(float)*blk_size*para_blk_num_max));
	checkCudaErrors(cudaMalloc(&dev_p, sizeof(float)*para_blk_dim*para_blk_num_max));
	checkCudaErrors(cudaMalloc(&dev_z, sizeof(float)*para_blk_dim*para_blk_num_max));
	checkCudaErrors(cudaMalloc(&dev_r, sizeof(float)*para_blk_dim*para_blk_num_max));
	checkCudaErrors(cudaMalloc(&dev_omega, sizeof(float)*para_blk_dim*para_blk_num_max));

	checkCudaErrors(cudaMalloc(&global_denominator, sizeof(float)));
	checkCudaErrors(cudaMalloc(&global_dot, sizeof(float)));
	checkCudaErrors(cudaMalloc(&global_numerator, sizeof(float)));
	checkCudaErrors(cudaMalloc(&floatone, sizeof(float)));
	checkCudaErrors(cudaMalloc(&floatnegone, sizeof(float)));
	checkCudaErrors(cudaMalloc(&floatzero, sizeof(float)));


	float one = 1.0f;
	float negone = -1.0f;
	float zeor = 0.0f;
	cudaMemcpyAsync(floatone, &one, sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpyAsync(floatnegone, &negone, sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpyAsync(floatzero, &zeor, sizeof(float), cudaMemcpyHostToDevice);
}


void PCGCuda::
solve(BlockMatrixFullCR const&blk_mat_bscr, float const* dev_b, float *dev_x, int max_iter)
{
	cuda::gpu_size_data const & para_blks_num = blk_mat_bscr.para_blks_num;
	int para_blk_dim = blk_mat_bscr.para_blk_dim;

	n_.allocate_once();
	n_.copy_transformed(para_blks_num, para_blk_dim, 0);

	// Extract the block diagnal as bsr
	extract_block_diag(blk_mat_bscr, this->dev_A_diag_blks_);

	// Set up M = L LT (preconditioner) using incomplete Cholesky factorization of the blocks
	icholesky_block_diag(para_blks_num, para_blk_dim, dev_A_diag_blks_);
	inverse_block_diag(para_blks_num, para_blk_dim, this->dev_A_diag_blks_, this->dev_A_diag_blks_inv_);

	// Initialize the residual
	bscr_gemv(blk_mat_bscr, dev_x, dev_r);
	aypx(n_, floatnegone, floatone, floatone, dev_b, dev_r); // r = -A x + b

	// Initialize z and p
	diag_blocks_gemv(para_blks_num, para_blk_dim, this->dev_A_diag_blks_inv_, dev_r, dev_z);
	cudaMemcpyAsync(dev_p, dev_z, n_.max_size * sizeof(float), cudaMemcpyDeviceToDevice); // p = z = M^-1 r
	dot_product(n_, dev_r, dev_z, global_numerator); // numerator = rT z

	int k = 0;
	for (k = 0; k < max_iter; k++)
	{
		// Calculate alpha
		bscr_gemv(blk_mat_bscr, dev_p, dev_omega);// omega = A p

		// Update numerator and denominator
		cudaMemcpyAsync(global_denominator, global_numerator, sizeof(float), cudaMemcpyDeviceToDevice);
		
		dot_product(n_, dev_p, dev_omega, global_dot); // dot = pT (A p)


		// Update x and r
		axpy(n_, floatone, global_numerator, global_dot, dev_p, dev_x); // x = x + alpha p
		axpy(n_, floatnegone, global_numerator, global_dot, dev_omega, dev_r); // r = r - alpha (A p)

		// Recalculate z
		diag_blocks_gemv(para_blks_num, para_blk_dim, this->dev_A_diag_blks_inv_, dev_r, dev_z); // z = M^-1 r

		dot_product(n_, dev_r, dev_z, global_numerator); // numerator = rT z

		// Recalculate p
		aypx(n_, floatone, global_numerator, global_denominator, dev_z, dev_p); // p = beta p + z
	}
}

__global__ void axpy_kernel(int const * __restrict__ n_ptr, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
{
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	if (i < (*n_ptr) && denominator[0] >1.0e-8) 
		y[i] = alpha[0] * (numerator[0] / denominator[0]) * x[i] + y[i];
}

__global__ void aypx_kernel(int const * __restrict__ n_ptr, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
{
	int i = blockDim.x * blockIdx.x + threadIdx.x;
	if (i < (*n_ptr))
	{
		if (denominator[0] >1.0e-8)
			y[i] = alpha[0] * (numerator[0] / denominator[0]) * y[i] + x[i];
		else
			y[i] = x[i];
	}

}


__global__ void dot_product_kernel(int const * __restrict__ n_ptr, const float *a, const float *b, float *c)
{
	__shared__ float temp[CUDA_MAX_NUM_THREADS];

	int i = blockDim.x * blockIdx.x + threadIdx.x;
	temp[threadIdx.x] = i < (*n_ptr) ? a[i] * b[i] : 0;
	__syncthreads();

	for (int s = blockDim.x >> 1; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
			temp[threadIdx.x] += temp[threadIdx.x + s];
		__syncthreads();
	}

	// TODO:PERF for performance, we can add if i < (*n_ptr) ??
	if (threadIdx.x == 0)
		atomicAdd(c, temp[0]);
}


// A * x = b
// one block per cuda block
// 144/160 threads per cuda block
__global__
void diag_blocks_gemv_12x12_kernel(int const * __restrict__ diag_blks_num_ptr, int para_blk_dim, int block_size, const float* __restrict__ dev_A, float const* __restrict__ x, float *b)
{
	__shared__ float sh_A[144];
	__shared__ float sh_b[12]; //temp out

	const int diag_blks_num = *diag_blks_num_ptr;

	for (int blkId = blockIdx.x; blkId < diag_blks_num; blkId += gridDim.x)
	{
		//load data
		float lc_A;
		if (threadIdx.x < 144)
			lc_A = dev_A[blkId * 144 + threadIdx.x]; //???why cannot combine it with later multiplication op???

		if (threadIdx.x < 12)
			sh_b[threadIdx.x] = x[blkId * 12 + threadIdx.x];
		__syncthreads();

		int lc_row = threadIdx.x / 12;
		int lc_col = threadIdx.x - 12 * lc_row;

		if (threadIdx.x < 144)
			sh_A[threadIdx.x] = lc_A*sh_b[lc_col];
		__syncthreads();

		if (lc_row < 12 && lc_col < 6) sh_A[threadIdx.x] += sh_A[threadIdx.x + 6];
		__syncthreads();
		if (lc_row < 12 && lc_col < 3) sh_A[threadIdx.x] += sh_A[threadIdx.x + 3];
		__syncthreads();
		if (lc_row < 12 && lc_col == 0) sh_b[lc_row] = sh_A[threadIdx.x] + sh_A[threadIdx.x + 1] + sh_A[threadIdx.x + 2];
		__syncthreads();

		if (threadIdx.x < 12)
			b[threadIdx.x + blkId * 12] = sh_b[threadIdx.x];
	}
}

//one diagnoal block per cuda block
//144 threads per cuda block for 12x12 blocks
__global__ 
void inverse_block_diag_kernel(int const * __restrict__ diag_blks_num_ptr, int para_blk_dim, int block_size, const float* dev_L, float *dev_A_inv)
{
	if (blockIdx.x >= *diag_blks_num_ptr)
		return;

	extern __shared__ float sh_mem[]; // result of first solve then caching
	float* sh_A_inv = &sh_mem[0];
	float* sh_L = &sh_mem[block_size];

	int offset = block_size * blockIdx.x; // first index in L

	for (int i = threadIdx.x; i < block_size; i += blockDim.x)
	{
		sh_L[i] = dev_L[offset + i];
		sh_A_inv[i] = 0.0;
	}
	__syncthreads();

	for (int i = threadIdx.x; i < para_blk_dim; i++)
		sh_A_inv[i*para_blk_dim + i] = 1.0;
	__syncthreads();

	int i = threadIdx.x / para_blk_dim;
	int k = threadIdx.x - i*para_blk_dim;

	float diag = sh_L[para_blk_dim * i + i];
	sh_A_inv[i*para_blk_dim + k] /= diag;
	__syncthreads();

	for (int j = 0; j < para_blk_dim - 1; j++)
	{
		if (i > j)
			sh_A_inv[i*para_blk_dim + k] -= sh_L[i*para_blk_dim + j] / diag * sh_A_inv[j*para_blk_dim+k];
		__syncthreads();
	}

	// sh_x now is result of L y = b, solve for LT x = y next

	sh_A_inv[i*para_blk_dim + k] /= diag;
	__syncthreads();

	for (int j = para_blk_dim - 1; j >= 1; j--)
	{
		if (i < j)
			sh_A_inv[i*para_blk_dim + k] -= sh_L[j*para_blk_dim + i] / diag * sh_A_inv[j*para_blk_dim+k];
		__syncthreads();
	}

	dev_A_inv[offset + threadIdx.x] = sh_A_inv[threadIdx.x];
}


__global__ void
full_solve_block_diag_kernel(int nblocks, int para_blk_dim, int block_size, const float* dev_L, const float* dev_b, float* dev_x)
{
	extern __shared__ float sh_mem[]; // result of first solve then caching
	float* sh_x = &sh_mem[0];
	float* sh_L = &sh_mem[para_blk_dim];

	int offset_L = block_size * blockIdx.x; // first index in L
	int offset_b = para_blk_dim * blockIdx.x; // index in x and b

	for (int i = threadIdx.x; i < block_size; i+=blockDim.x)
		sh_L[i] = dev_L[offset_L + i];

	int i = threadIdx.x;
	float diag = sh_L[para_blk_dim * i + i];
	sh_x[i] = dev_b[i+offset_b] / diag;

	for (int j = 0; j < para_blk_dim - 1; j++)
	{
		__syncthreads();

		if (i > j)
			sh_x[threadIdx.x] -= sh_L[i*para_blk_dim+j] / diag * sh_x[j];
	}
	// sh_x now is result of L y = b, solve for LT x = y next

	sh_x[threadIdx.x] /= diag;

	for (int j = para_blk_dim - 1; j >= 1; j--)
	{
		__syncthreads();

		if (threadIdx.x < j)
			sh_x[threadIdx.x] -= sh_L[j*para_blk_dim+i] / diag * sh_x[j];
	}

	dev_x[offset_b+threadIdx.x] = sh_x[threadIdx.x];
}

//78 threads per cuda block
// one diag block per cuda block
__global__
void icholesky_block_diag_12x12_kernel(int const * __restrict__ diag_blks_num_ptr, int block_size, int para_blk_dim, float* diag_A)
{
	extern __shared__ float sh_A_tril[]; // caching
	if (blockIdx.x >= *diag_blks_num_ptr)
		return;

	int r, c;
	decompose_triangle_id_with_diag(threadIdx.x, r, c);
	int i = blockIdx.x * block_size + para_blk_dim*r+c; // index in A

	if (threadIdx.x < (block_size-para_blk_dim)/2+para_blk_dim)
		sh_A_tril[threadIdx.x] = diag_A[i];
	__syncthreads();

	for (int k = 0; k < para_blk_dim; k++)
	{
		if (k == c && c == r) // diagonal element for current iteration
			sh_A_tril[threadIdx.x] = sqrtf(sh_A_tril[threadIdx.x]);
		__syncthreads();

		if (k == c && c < r) // elements in its lower column
			sh_A_tril[threadIdx.x] /= sh_A_tril[k * (k + 1) / 2 + k];
		__syncthreads();

		if (k < c && c <= r) // elements in the remaining lower triangular matrix
			sh_A_tril[threadIdx.x] -= sh_A_tril[r * (r + 1) / 2 + k] * sh_A_tril[c * (c + 1) / 2 + k];
		__syncthreads();
	}

	diag_A[i] = sh_A_tril[threadIdx.x];
}

//one diag block per cuda block
__global__
void extract_block_diag_kernel(int const * __restrict__ blocks_num_ptr, int blk_size, int const* diag_blk_pos, float const* A_full_data, float* diag_A)
{
	if (blockIdx.x >= *blocks_num_ptr)
		return;

	int pos = diag_blk_pos[blockIdx.x];
	
	if (threadIdx.x < blk_size)
		diag_A[blockIdx.x*blk_size + threadIdx.x] = A_full_data[pos*blk_size + threadIdx.x];
}

//block compressed sparse row
// A*x = b
//one row per cuda block
//threads per cuda block: 160
__global__
void bcsr_gemv_12x12_kernel(int const * __restrict__ para_blks_num_ptr, int const* rowptr, int const* colind, 
							float const* __restrict__ data, float const* x, float* __restrict__ b)
{
	__shared__ float sh_A[144];
	__shared__ float sh_x[12];
	__shared__ float sh_b[12]; //temp out

	int para_blks_num = *para_blks_num_ptr;

	if (threadIdx.x < 12)
		sh_b[threadIdx.x] = 0.0f;

	for (int blkId = blockIdx.x; blkId < para_blks_num; blkId += gridDim.x)
	{
		int row = blkId;
		int row_st = rowptr[blkId];
		int row_end = rowptr[blkId + 1];
		for (int i = row_st; i < row_end; i++)
		{
			int col = colind[i];

			//load data
			float loc_A;
			if (threadIdx.x < 144)
				loc_A = data[i * 144 + threadIdx.x]; //???why cannot combine it with later multiplication op??? it seems slower to combine!!!

			if (threadIdx.x < 12)
				sh_x[threadIdx.x] = x[col * 12 + threadIdx.x];
			__syncthreads();

			int lc_row = threadIdx.x / 12;
			int lc_col = threadIdx.x - 12 * lc_row;

			if (threadIdx.x < 144)
				sh_A[threadIdx.x] = loc_A * sh_x[lc_col];
			__syncthreads();

			if (lc_row < 12 && lc_col < 6) sh_A[threadIdx.x] += sh_A[threadIdx.x + 6];
			__syncthreads();
			if (lc_row < 12 && lc_col < 3) sh_A[threadIdx.x] += sh_A[threadIdx.x + 3];
			__syncthreads();
			if (lc_row < 12 && lc_col == 0) sh_b[lc_row] += sh_A[threadIdx.x] + sh_A[threadIdx.x + 1] + sh_A[threadIdx.x + 2];
			__syncthreads();
		}

		if (threadIdx.x < 12){
			b[threadIdx.x + row * 12] = sh_b[threadIdx.x]; 
			sh_b[threadIdx.x] = 0.0f;
		}
	}
}

__global__ void extract_block_diag_kernel_sam(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag)
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

__global__ 
void icholesky_block_diag_kernel_sam(int nblocks, int block_size, float* A)
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


__global__ void 
full_solve_block_diag_kernel_sam(int nblocks, int block_size, const float* L, const float* b, float* x)
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

__global__ 
void sparse_block_gemv_kernel_sam(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, const float* b, float* x)
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

//only use one CUDA block with 1024 threads
__global__
void HessianBlockMatrix_To_BlockMatrixFullCR_PreProcess_Kernel( HessianBlockInfoCuda const*dev_hessian_blk_info, 
																int const * __restrict__ para_blks_num_ptr, int const * __restrict__ hessian_blks_num_ptr,
																BlockMatrixFullCR blk_mat_bscr //block compressed row
																)
{
	extern __shared__ int rowCounts[]; //counts of block within each rows
	int const para_blks_num = *para_blks_num_ptr;
	int const hessian_blks_num = *hessian_blks_num_ptr;
	
	for (int i = threadIdx.x; i < para_blks_num; i += blockDim.x)
		rowCounts[i] = 0;
	__syncthreads();

	for (int i = threadIdx.x; i < hessian_blks_num; i += blockDim.x)
	{
		int pi = dev_hessian_blk_info[i].pi_idx;
		int pj = dev_hessian_blk_info[i].pj_idx;

		if (pi < para_blks_num)
			atomicAdd(&(rowCounts[pi]), 1);
		if (pi != pj && pj < para_blks_num)
			atomicAdd(&(rowCounts[pj]), 1);
	}
	__syncthreads();

	//sequential prefix sum
	if (threadIdx.x == 0)
	{
		int sum = 0;
		for (int i = 0; i < para_blks_num; i++)
		{
			int cur_val = rowCounts[i];
			rowCounts[i] = sum;
			sum += cur_val;
		}
		rowCounts[para_blks_num] = sum;
	}
	__syncthreads();

	//write out row_ptr
	for (int i = threadIdx.x; i < para_blks_num+1; i += blockDim.x)
		blk_mat_bscr.brow_ptr[i] = rowCounts[i];
	__syncthreads();

	for (int i = threadIdx.x; i < hessian_blks_num; i += blockDim.x)
	{
		int pi = dev_hessian_blk_info[i].pi_idx;
		int pj = dev_hessian_blk_info[i].pj_idx;
		int data_offset = dev_hessian_blk_info[i].data_offset;

		if (pi < para_blks_num)
		{
			int lcPos = atomicAdd(&(rowCounts[pi]), 1);
			blk_mat_bscr.bcolind[lcPos] = pj;
			blk_mat_bscr.browind[lcPos] = pi;
			blk_mat_bscr.extern_data_offset[lcPos] = data_offset;

			if (pi == pj)			
				blk_mat_bscr.diag_blk_pos[pi] = lcPos;
			
		}
		if (pi != pj && pj < para_blks_num)
		{
			int lcPos = atomicAdd(&(rowCounts[pj]), 1);
			blk_mat_bscr.bcolind[lcPos] = pi;
			blk_mat_bscr.browind[lcPos] = pj;
			blk_mat_bscr.extern_data_offset[lcPos] = data_offset;
		}
	}
}


//b -= A x
__global__ void ComputeResidualKernel_Diag( HessianBlockInfoCuda const* dev_blk_info, float const* dev_A, int diag_blk_num, int blk_num,
									   float *dev_b, float const* dev_x)
{
	if (blockDim.x >= diag_blk_num)
		return;

	__shared__ HessianBlockInfoCuda blk;
	__shared__ float A[144];
	__shared__ float x[12];
	if (threadIdx.x == 0)
		blk = dev_blk_info[blockIdx.x];
	__syncthreads();

	int &data_offset = blk.data_offset;
	int &pi = blk.pi_idx;

	if (threadIdx.x < 144)
		A[threadIdx.x] = dev_A[data_offset + threadIdx.x];
	if (threadIdx.x <12)
		x[threadIdx.x] = dev_x[12 * pi + threadIdx.x];
	__syncthreads();

	short i = threadIdx.x / 12;

	if (threadIdx.x < 144)
		A[threadIdx.x] *= x[i];
	__syncthreads();

	if (threadIdx.x < 24)
	{
		float v = A[threadIdx.x] + A[threadIdx.x + 24] + A[threadIdx.x + 24 * 2] + A[threadIdx.x + 24 * 3] + A[threadIdx.x + 24 * 4] + A[threadIdx.x + 24 * 5];
		A[threadIdx.x] = v;
		__threadfence_block();
		if (threadIdx.x < 12)
			dev_b[12 * pi + threadIdx.x] = (A[threadIdx.x] + A[threadIdx.x + 12]);
	}

}

//b -= Ax
__global__ void ComputeResidualKernel_OffDiag(HessianBlockInfoCuda const* dev_blk_info, float const* dev_A, int diag_blk_num, int blk_num,
	float *dev_b, float const* dev_x)
{
	if (blockDim.x >= blk_num-diag_blk_num)
		return;

	__shared__ HessianBlockInfoCuda blk;
	__shared__ float A1[144];
	__shared__ float A2[144]; //A2 = A1'
	__shared__ float x1[12]; //starting at 12*pi
	__shared__ float x2[12]; //starting at 12*pj
	if (threadIdx.x == 0)
		blk = dev_blk_info[blockIdx.x];
	__syncthreads();

	int &data_offset = blk.data_offset;
	int &pi = blk.pi_idx;
	int &pj = blk.pj_idx;
	short i = threadIdx.x / 12;
	short j = threadIdx.x - 12 * i;

	if (threadIdx.x < 144)
	{
		float a = dev_A[data_offset + threadIdx.x];
		A1[threadIdx.x] = a;
		A2[12*j+i] = a; //transposed matrix
	}
	if (threadIdx.x <12)
		x1[threadIdx.x] = dev_x[12 * pi + threadIdx.x];
	if (threadIdx.x>=32 && threadIdx.x<44)
		x2[threadIdx.x-32] = dev_x[12 * pj + threadIdx.x-32];

	__syncthreads();

	if (threadIdx.x < 144)
	{
		A1[threadIdx.x] *= x1[j];
		A2[threadIdx.x] *= x2[j];
	}
	__syncthreads();

	if (threadIdx.x < 144 && threadIdx.x % 2 == 0)
	{
		A1[threadIdx.x] += A1[threadIdx.x + 1];
		A2[threadIdx.x] += A2[threadIdx.x + 1];
	}
	__syncthreads();

	if (threadIdx.x < 144 && threadIdx.x % 4 == 0)
	{
		A1[threadIdx.x] += A1[threadIdx.x + 2];
		A2[threadIdx.x] += A2[threadIdx.x + 2];
	}
	__syncthreads();

	if (threadIdx.x / 12==0)
	{
		atomicAdd(&(dev_b[12 * pi + i]), -(A1[12 * i] + A1[12 * i + 4] + A1[12 * i + 8]));
		atomicAdd(&(dev_b[12 * pj + i]), -(A2[12 * i] + A2[12 * i + 4] + A2[12 * i + 8]));
	}
}

// r = B - A*x
bool PCGCuda::
get_residual_vector(HessianBlockInfoCuda const* dev_blk_info, float const* dev_A, int diag_blk_num, int blk_num,
float const* dev_b, float const* dev_x, float *dev_r)
{
	cudaMemcpy(dev_r, dev_b, 12 * diag_blk_num*sizeof(float), cudaMemcpyDeviceToDevice);

	int threads_per_block = THREADS_PER_BLOCK;
	int blocks_per_grid = (diag_blk_num);
	ComputeResidualKernel_Diag<<<blocks_per_grid, threads_per_block>>>(dev_blk_info, dev_A, diag_blk_num, blk_num, dev_r, dev_x);
	m_checkCudaErrors();

	blocks_per_grid = (blk_num-diag_blk_num);
	ComputeResidualKernel_OffDiag<<<blocks_per_grid, threads_per_block>>>(dev_blk_info, dev_A, diag_blk_num, blk_num, dev_r, dev_x);
	m_checkCudaErrors();

	return true;
}

bool PCGCuda::
HessianBlockMatrix_to_BlockMatrixFullCR_PreProcess( HessianBlockInfoCuda const* dev_blk_info, 
													cuda::gpu_size_data const & para_blks_num,  
													cuda::gpu_size_data const & hessian_blks_num,
													BlockMatrixFullCR &blk_mat_bscr)
{
	cudaMemcpyAsync(blk_mat_bscr.para_blks_num.dev_ptr, para_blks_num.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice);
	blk_mat_bscr.blks_num.multiply_add_gpu_only(2, hessian_blks_num, -1, para_blks_num, 0); //assume the diagonal is full

	// YURY: substituted dynamic shared mem config with max_size
	HessianBlockMatrix_To_BlockMatrixFullCR_PreProcess_Kernel<<<1, MAX_THREADS_PER_BLOCK, (para_blks_num.max_size + 1) * 4>>>(dev_blk_info,
																	para_blks_num.dev_ptr, hessian_blks_num.dev_ptr, blk_mat_bscr);
	m_checkCudaErrors();

	return true;
}


//one copy of jtj block per cuda block
__global__
void HessianBlockMatrix_To_BlockMatrixFullCR_DataCopy_Kernel(int const * __restrict__ blocks_num_ptr,
									float const* dev_hessian_data,
									BlockMatrixFullCR blk_mat_bscr //block compressed row
									)
{
	int blocks_num = *blocks_num_ptr;
	for (int blkId = blockIdx.x; blkId < blocks_num; blkId += gridDim.x)
	{

		int pi = blk_mat_bscr.browind[blkId];
		int pj = blk_mat_bscr.bcolind[blkId];
		int extern_data_offset = blk_mat_bscr.extern_data_offset[blkId];
		int para_blk_dim = blk_mat_bscr.para_blk_dim;
		int data_offset = blkId*para_blk_dim*para_blk_dim;

		int r = threadIdx.y;
		int c = threadIdx.x;
		if (r < blk_mat_bscr.para_blk_dim && c < blk_mat_bscr.para_blk_dim)
		{
			int lcOffset = para_blk_dim*r + c;
			float val;
			if (pi >= pj)
			{
				val = dev_hessian_data[extern_data_offset + lcOffset];
			}
			else
			{
				//if on the upper triangle, flip the original block
				int lcOffset2 = para_blk_dim*c + r;
				val = dev_hessian_data[extern_data_offset + lcOffset2];
			}
			blk_mat_bscr.data[data_offset + lcOffset] = val;
		}
	}
}

bool PCGCuda::
HessianBlockMatrix_to_BlockMatrixFullCR_CopyData(float const* dev_mat_data, BlockMatrixFullCR &blk_mat_bscr)
{
	dim3 threads_per_block(12, 12);
	int blocks_per_grid = 1024; //loop stride trick
	HessianBlockMatrix_To_BlockMatrixFullCR_DataCopy_Kernel<<<blocks_per_grid, threads_per_block>>>(blk_mat_bscr.blks_num.dev_ptr, dev_mat_data, blk_mat_bscr);
	m_checkCudaErrors();

	return true;
}

bool PCGCuda::
HessianBlockMatrix_to_BlockMatrixFullCR(HessianBlockInfoCuda const* dev_blk_info, float const* dev_mat_data, cuda::gpu_size_data const & para_blks_num,  cuda::gpu_size_data const & hessian_blks_num, 
										BlockMatrixFullCR &blk_mat_bscr)
{
	HessianBlockMatrix_to_BlockMatrixFullCR_PreProcess(dev_blk_info, para_blks_num, hessian_blks_num, blk_mat_bscr);
	HessianBlockMatrix_to_BlockMatrixFullCR_CopyData(dev_mat_data, blk_mat_bscr);
	return true;
}

void PCGCuda::
bscr_gemv(BlockMatrixFullCR const&A_bscr, float const* dev_x, float *dev_b)
{
	int blocks_per_grid = 512; //stride loop trick to replace max bound
	bcsr_gemv_12x12_kernel<<<blocks_per_grid, 160>>>(A_bscr.para_blks_num.dev_ptr, A_bscr.brow_ptr, A_bscr.bcolind, A_bscr.data, dev_x, dev_b);
	m_checkCudaErrors();
}

void PCGCuda::
extract_block_diag(BlockMatrixFullCR const&A_bscr, float* dev_diag_blks)
{
	int blk_size = A_bscr.para_blk_dim * A_bscr.para_blk_dim;
	
	cuda::gpu_size_data const & nrows = A_bscr.para_blks_num;
	extract_block_diag_kernel<<<nrows.max_size, blk_size>>>(nrows.dev_ptr, blk_size, A_bscr.diag_blk_pos, A_bscr.data, dev_diag_blks);
	
	m_checkCudaErrors();
}

void PCGCuda::
icholesky_block_diag(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, float* dev_diag_blks)
{
	int block_size = para_blk_dim*para_blk_dim;
	int eles_num_tril = (block_size - para_blk_dim) / 2 + para_blk_dim; //elements on lower triangle matrix
	icholesky_block_diag_12x12_kernel<<<diag_blks_num.max_size, eles_num_tril, eles_num_tril * sizeof(float)>>>(diag_blks_num.dev_ptr, block_size, para_blk_dim, dev_diag_blks);
	m_checkCudaErrors();

}

void PCGCuda::
full_solve_block_diag(int diag_blks_num, int para_blk_dim, const float* diag_L, const float* b, float* x)
{
	int block_size = para_blk_dim*para_blk_dim;
	int sh_mem_size = (block_size + para_blk_dim)*sizeof(float);
	full_solve_block_diag_kernel<<<diag_blks_num, para_blk_dim, sh_mem_size>>>(diag_blks_num, para_blk_dim, block_size, diag_L, b, x);
	m_checkCudaErrors();
}

void PCGCuda::
inverse_block_diag(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, const float* diag_L, float *diag_A_inv)
{
	int block_size = para_blk_dim*para_blk_dim;
	int sh_mem_size = 2 * block_size*sizeof(float);
	inverse_block_diag_kernel<<<diag_blks_num.max_size, block_size, sh_mem_size>>>(diag_blks_num.dev_ptr, para_blk_dim, block_size, diag_L, diag_A_inv);
	m_checkCudaErrors();
}

void PCGCuda::
axpy(cuda::gpu_size_data const & n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
{
	KERNEL_CALL(axpy_kernel, n.max_size, CUDA_MAX_NUM_THREADS)(n.dev_ptr, alpha, numerator, denominator, x, y);
}

void PCGCuda::
aypx(cuda::gpu_size_data const & n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y)
{
	KERNEL_CALL(aypx_kernel, n.max_size, CUDA_MAX_NUM_THREADS)(n.dev_ptr, alpha, numerator, denominator, x, y);
	m_checkCudaErrors();
}

void PCGCuda::
dot_product(cuda::gpu_size_data const & n, const float *a, const float *b, float *c)
{
	cudaMemsetAsync(c, 0, sizeof(float));
	KERNEL_CALL(dot_product_kernel, n.max_size, CUDA_MAX_NUM_THREADS)(n.dev_ptr, a, b, c);
	m_checkCudaErrors();
}

void PCGCuda::
diag_blocks_gemv(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, const float* dev_A, float const* x, float *b)
{
	int block_size = para_blk_dim*para_blk_dim;

	int blocks_per_grid = 512;
	diag_blocks_gemv_12x12_kernel<<<blocks_per_grid, 144>>>(diag_blks_num.dev_ptr, para_blk_dim, block_size, dev_A, x, b);
	m_checkCudaErrors();
}

void PCGCuda::
bscr_gemv_sam(BlockMatrixFullCR const&A_bscr, float const* dev_x, float *dev_b)
{
	int nrows = A_bscr.para_blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	int nblocks = A_bscr.blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	int block_size = A_bscr.para_blk_dim;
	int threads_per_block_row = block_size * CUDA_WARP_SIZE;
	int shmem_size = threads_per_block_row * sizeof(float)+threads_per_block_row * sizeof(float);
	KERNEL_CALL_FULL(sparse_block_gemv_kernel_sam, nrows * threads_per_block_row, threads_per_block_row, shmem_size, 0)(nblocks, nrows, block_size, 
		A_bscr.data, A_bscr.brow_ptr, A_bscr.bcolind, dev_x, dev_b);
	m_checkCudaErrors();
}

void PCGCuda::
icholesky_block_diag_sam(int nblocks, int para_blk_dim, float* diag_A)
{
	int b2 = para_blk_dim * para_blk_dim;
	KERNEL_CALL_FULL(icholesky_block_diag_kernel_sam, nblocks * b2, b2, b2 * sizeof(float), 0)(nblocks, para_blk_dim, diag_A);
	m_checkCudaErrors();
}

void PCGCuda::
full_solve_block_diag_sam(int nblocks, int para_blk_dim, const float* L, const float* b, float* x)
{
	int shmem_size = (para_blk_dim * para_blk_dim + para_blk_dim) * sizeof(float);
	KERNEL_CALL_FULL(full_solve_block_diag_kernel_sam, nblocks * para_blk_dim, para_blk_dim, shmem_size, 0)(nblocks, para_blk_dim, L, b, x);
	m_checkCudaErrors();
}

void PCGCuda::
extract_block_diag_sam(int nblocks, int nrows, int block_size, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag)
{
	int b2 = block_size * block_size;
	
	KERNEL_CALL_FULL(extract_block_diag_kernel_sam, nblocks * b2, b2, 0, 0)(nblocks, nrows, block_size, A_val, A_rowptr, A_colind, A_diag);
	m_checkCudaErrors();
}

void PCGCuda::solve_test(BlockMatrixFullCR const&blk_mat_bscr, float const* dev_b, float *dev_x)
{
	int para_blk_num = blk_mat_bscr.para_blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	PCGSover_Sam::fast_pcg pcg(para_blk_num * 12, 12);
	pcg.bsr_solve(blk_mat_bscr.blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), blk_mat_bscr.data, blk_mat_bscr.brow_ptr, blk_mat_bscr.bcolind, dev_b, dev_x, 10);
}

