class NormalEquationSolverPCGCuda : public NormalEquationSolverGPU, public CudaGlobalMemoryStatic
{
public:
	NormalEquationSolverPCGCuda(int jtj_blks_ltri_max, int para_blks_max, int para_blk_dim)
		: pcg_solver_(para_blks_max, para_blk_dim)
	{
		checkCudaErrors(cudaMalloc(&dev_global_predicted_cost_decrease_, sizeof(float)));
		allocate_memory(jtj_blks_ltri_max, para_blks_max, para_blk_dim);
	}

	~NormalEquationSolverPCGCuda()
	{
		checkCudaErrors(cudaFree(dev_global_predicted_cost_decrease_));
	}

public:
	bool setup_solver(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf); //assume the data structure of jtj and jtf are constant through out the LM optimization
	bool update_JtJ_JtF(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf);
	bool regularize_JtJ(float mu);
	bool regularize_JtJ(float const* dev_mu);
	float* solve();
	float predicted_cost_decrease(float const* dev_jtf, float const* dev_dx, float mu);
	void predicted_cost_decrease(float* dev_cost_decrease, float const* dev_jtf, float const* dev_dx, float const* mu);

	BlockMatrixFullCR const* JtJ_full() const { return &(this->A_bscr_gpu_); }
	float const*  dev_b() const { return dev_b_; }

protected:
	void allocate_memory(int blks_num_max_ltri, int para_blks_num_max, int para_blk_dim)
	{
		allocate_BlockMatrixFullCR_GPU(this->A_bscr_gpu_, blks_num_max_ltri, para_blks_num_max, para_blk_dim);

		int n_max = para_blks_num_max * para_blk_dim;
		checkCudaErrors(cudaMalloc(&dev_b_, sizeof(float)*n_max));
		checkCudaErrors(cudaMalloc(&dev_dx_, sizeof(float)*n_max));
		checkCudaErrors(cudaMalloc(&dev_D_avgs_, sizeof(float)*para_blk_dim));
	}

private:
	BlockMatrixFullCR A_bscr_gpu_; //jtj
	float* dev_b_;//-jtf
	float* dev_D_avgs_;
	float* dev_dx_;
	PCGCuda pcg_solver_;

	float* dev_global_predicted_cost_decrease_;
};