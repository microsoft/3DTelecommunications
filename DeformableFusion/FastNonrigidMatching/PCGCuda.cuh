class PCGCuda : public CudaGlobalMemoryStatic
{
public:
	PCGCuda(int para_blk_num_max, int para_blk_dim)
	{
		allocate_memory(para_blk_num_max, para_blk_dim);
	}

	void allocate_memory(int para_blk_num_max, int para_blk_dim);

	void solve(BlockMatrixFullCR const&blk_mat_bscr, float const* dev_b, float *dev_x, int max_iter);
	void solve_test(BlockMatrixFullCR const&blk_mat_bscr, float const* dev_b, float *dev_x);

private:
	float *dev_A_diag_blks_;
	float *dev_A_diag_blks_inv_;
	float *dev_p;
	float *dev_z;
	float *dev_r;
	float *dev_omega;

	float *global_numerator;
	float *global_denominator;
	float *global_dot;
	float *floatnegone;
	float *floatone;
	float *floatzero;

	cuda::gpu_size_data n_;

public:
	static void extract_block_diag(BlockMatrixFullCR const&A_bscr, float* dev_diag_blks);
	static void icholesky_block_diag(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, float* diag_A);
	static void full_solve_block_diag(int diag_blks_num, int para_blk_dim, const float* diag_L, const float* b, float* x);
	static void inverse_block_diag(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, const float* diag_L, float *diag_A_inv);
	static void diag_blocks_gemv(cuda::gpu_size_data const & diag_blks_num, int para_blk_dim, const float* dev_A, float const* x, float *b);
	static void bscr_gemv(BlockMatrixFullCR const&A_bscr, float const* dev_x, float *dev_b);
	//y = ay+x
	static void aypx(cuda::gpu_size_data const & n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
	//y = ax+y
	static void axpy(cuda::gpu_size_data const & n, const float* alpha, const float* numerator, const float* denominator, const float *x, float *y);
	static void dot_product(cuda::gpu_size_data const & n, const float *a, const float *b, float *c);

	static void bscr_gemv_sam(BlockMatrixFullCR const&A_bscr, float const* dev_x, float *dev_b);
	static void icholesky_block_diag_sam(int diag_blks_num, int para_blk_dim, float* diag_A);
	static void full_solve_block_diag_sam(int diag_blks_num, int block_size, const float* L, const float* b, float* x);
	static void extract_block_diag_sam(int nblocks, int nrows, int para_blk_dim, const float* A_val, const int* A_rowptr, const int* A_colind, float* A_diag);
	static bool get_residual_vector(HessianBlockInfoCuda const* dev_blk_info, float const* dev_A, int diag_blk_num, int blk_num,
							    float const* dev_b, float const* dev_x, float *dev_r);
public:
	static bool HessianBlockMatrix_to_BlockMatrixFullCR(HessianBlockInfoCuda const* dev_blk_info, float const* dev_mat_data, cuda::gpu_size_data const & para_blks_num, cuda::gpu_size_data const & hessian_blks_num,
		BlockMatrixFullCR &blk_mat_bscr);
	static bool HessianBlockMatrix_to_BlockMatrixFullCR_PreProcess(HessianBlockInfoCuda const* dev_blk_info, cuda::gpu_size_data const & para_blks_num, cuda::gpu_size_data const & hessian_blks_num,
																	BlockMatrixFullCR &blk_mat_bscr);
	static bool HessianBlockMatrix_to_BlockMatrixFullCR_CopyData(float const*dev_hessian_data, BlockMatrixFullCR &blk_mat_bscr);
};
