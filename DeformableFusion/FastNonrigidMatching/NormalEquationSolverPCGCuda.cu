__global__
void CopyJtFToB_Kernel(float const * __restrict__ dev_jtf, float * __restrict__ dev_b, int * const __restrict__ n)
{
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < *n)
		dev_b[id] = -dev_jtf[id];		
}

//each thread handles one diagonal element
__global__
void RegularizeJtJ_Kernel_AvergeD(int const* diag_blks_pos, float *data, 
								  int const * __restrict__ blk_rows_num, int para_blk_dim, int blk_size, 
								  int cuda_blks_per_element_reduction,
								  float* dev_D_avgs)
{
	__shared__ float diag_vals[MAX_THREADS_PER_BLOCK];
	int lc_ele_id = blockIdx.x / cuda_blks_per_element_reduction;
	int row = threadIdx.x + (blockIdx.x - lc_ele_id*cuda_blks_per_element_reduction) * blockDim.x; //blk row
	float diag_val = 0.0f;
	if (row < *blk_rows_num)
	{
		int pos = diag_blks_pos[row];
		diag_val = data[pos*blk_size + lc_ele_id*para_blk_dim + lc_ele_id];
	}
	diag_vals[threadIdx.x] = diag_val;
	__syncthreads();

	//reduction
	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			diag_vals[threadIdx.x] += diag_vals[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(&(dev_D_avgs[lc_ele_id]), diag_vals[0] / *blk_rows_num);
	}
}

//one element per threads
__global__
void RegularizeJtJ_Kernel_PlusD(int const* diag_blks_pos, float *data,
								int const * __restrict__ blk_rows_num, int para_blk_dim, int blk_size,
								int cuda_blks_per_element_reduction,
								float* dev_D_avgs, float const* dev_mu)
{
	const float mu = *dev_mu;
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < (*blk_rows_num) * para_blk_dim)
	{
		int row = id / para_blk_dim;
		int lc_ele_id = id - row*para_blk_dim;

		int pos = diag_blks_pos[row];
		
		data[pos*blk_size + lc_ele_id*para_blk_dim + lc_ele_id] += dev_D_avgs[lc_ele_id]*mu;
	}
}

// dx'*(jtf-mu*D*dx)

__global__
void PredictCostDecrease_Kernel(float* dev_global_predicted_cost_decrease, float const* dev_jtf, float const* dev_dx, 
								float const* dev_D_avgs, float const* dev_mu, int para_blk_dim, int const * __restrict__ para_blk_num)
{
	__shared__ float vals[MAX_THREADS_PER_BLOCK];
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	float mu = *dev_mu;

	float val = 0.0f;
	if (id < para_blk_dim * *para_blk_num) //n: dimension of dev_jtf
	{
		float dx = dev_dx[id];
		int lc_ele_id = id % para_blk_dim;
		val = dx * (dev_jtf[id] - mu*dev_D_avgs[lc_ele_id] * dx);
	}
	vals[threadIdx.x] = val;
	__syncthreads();

	//reduction
	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			vals[threadIdx.x] += vals[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_predicted_cost_decrease, vals[0]);
	}
}

//assume the data structure of jtj and jtf are constant through out the LM optimization
bool NormalEquationSolverPCGCuda::setup_solver(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf)
{
	PCGCuda::HessianBlockMatrix_to_BlockMatrixFullCR_PreProcess(jtj.blk_info, jtj.para_blks_num, jtj.blks_num, A_bscr_gpu_);
	return true;
}

bool NormalEquationSolverPCGCuda::update_JtJ_JtF(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf)
{
	PCGCuda::HessianBlockMatrix_to_BlockMatrixFullCR_CopyData(jtj.data, this->A_bscr_gpu_);

	//copy -jtf to dev_b_
	int threads_per_block = 64;
	int blocks_per_grid = (jtf.n.max_size + threads_per_block - 1) / threads_per_block;
	CopyJtFToB_Kernel<<<blocks_per_grid, threads_per_block>>>(jtf.jtf, dev_b_, jtf.n.dev_ptr);
	m_checkCudaErrors();

	return true;
}

bool NormalEquationSolverPCGCuda::regularize_JtJ(float mu)
{
	float *dev_mu = NULL;
	checkCudaErrors(cudaMalloc(&dev_mu, sizeof(float)));
	checkCudaErrors(cudaMemcpy(dev_mu, &mu, sizeof(float), cudaMemcpyHostToDevice));

	bool ret = regularize_JtJ(dev_mu);

	checkCudaErrors(cudaFree(dev_mu));
	return ret;
}


bool NormalEquationSolverPCGCuda::regularize_JtJ(float const* dev_mu)
{
	cuda::gpu_size_data const & blk_rows_num = A_bscr_gpu_.para_blks_num;
	int para_blk_dim = A_bscr_gpu_.para_blk_dim;
	int blk_size = para_blk_dim*para_blk_dim;
	int cuda_blks_per_element_reduction = (blk_rows_num.max_size + MAX_THREADS_PER_BLOCK - 1) / MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = cuda_blks_per_element_reduction * para_blk_dim;
	checkCudaErrors(cudaMemsetAsync(dev_D_avgs_, 0, sizeof(float)*para_blk_dim));

	RegularizeJtJ_Kernel_AvergeD<<<blocks_per_grid, MAX_THREADS_PER_BLOCK>>>(A_bscr_gpu_.diag_blk_pos, A_bscr_gpu_.data,
		blk_rows_num.dev_ptr, para_blk_dim, blk_size, cuda_blks_per_element_reduction,
		dev_D_avgs_);
	m_checkCudaErrors();

	if (LOGGER()->check_verbosity(Logger::Trace))
	{
		vnl_vector<float> D_avgs_host(12, 0.0);
		checkCudaErrors(cudaMemcpy(D_avgs_host.data_block(), dev_D_avgs_, sizeof(float)* 12, cudaMemcpyDeviceToHost));
		stringstream ss;

		ss << D_avgs_host << endl;

		LOGGER()->trace(ss.str().c_str());
	}

	int threads_per_block = 64;
	blocks_per_grid = (blk_rows_num.max_size * para_blk_dim + threads_per_block - 1) / threads_per_block;
	RegularizeJtJ_Kernel_PlusD<<<blocks_per_grid, threads_per_block>>>(A_bscr_gpu_.diag_blk_pos, A_bscr_gpu_.data,
		blk_rows_num.dev_ptr, para_blk_dim, blk_size, cuda_blks_per_element_reduction,
		dev_D_avgs_, dev_mu);
	m_checkCudaErrors();

	return true;
}

float* NormalEquationSolverPCGCuda::solve()
{
	cudaMemsetAsync(dev_dx_, 0, sizeof(float) * A_bscr_gpu_.para_blks_num.max_size * A_bscr_gpu_.para_blk_dim);
	const int linear_solver_iters = 10;
	pcg_solver_.solve(A_bscr_gpu_, dev_b_, dev_dx_, linear_solver_iters);
	return dev_dx_;
}

// dx'*(jtf-mu*D*dx)
float NormalEquationSolverPCGCuda::predicted_cost_decrease(float const* dev_jtf, float const* dev_dx, float mu)
{
	float *dev_mu = NULL;

	checkCudaErrors(cudaMalloc(&dev_mu, sizeof(float)));
	checkCudaErrors(cudaMemcpy(dev_mu, &mu, sizeof(float), cudaMemcpyHostToDevice));

	predicted_cost_decrease(dev_global_predicted_cost_decrease_, dev_jtf, dev_dx, dev_mu);
	
	checkCudaErrors(cudaFree(dev_mu));

	float ret = 0.0f;
	checkCudaErrors(cudaMemcpy(&ret, dev_global_predicted_cost_decrease_, sizeof(float), cudaMemcpyDeviceToHost));
	return ret;
}

// dx'*(jtf-mu*D*dx)
void NormalEquationSolverPCGCuda::predicted_cost_decrease(float* dev_cost_decrease, float const* dev_jtf, float const* dev_dx, float const* dev_mu)
{
	checkCudaErrors(cudaMemsetAsync(dev_cost_decrease, 0, sizeof(float)));

	int para_blk_dim = this->A_bscr_gpu_.para_blk_dim;

	cuda::gpu_size_data const & para_blks_num = this->A_bscr_gpu_.para_blks_num;
	int n_max = para_blk_dim * para_blks_num.max_size;

	int blocks_per_grid = (n_max + MAX_THREADS_PER_BLOCK - 1) / MAX_THREADS_PER_BLOCK;

	PredictCostDecrease_Kernel<<<blocks_per_grid, MAX_THREADS_PER_BLOCK>>>(dev_cost_decrease, dev_jtf, dev_dx, dev_D_avgs_, dev_mu, para_blk_dim, this->A_bscr_gpu_.para_blks_num.dev_ptr);
	m_checkCudaErrors();
}
