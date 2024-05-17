#define NORMAL_CHECK_THRES 0.60f

#define JTJ_BLKS_NUM_MAX (ED_NODES_NUM_MAX*EDNODE_NN + ED_NODES_NUM_MAX*JTJ_BLKS_NUM_INDUCED_PER_VERTEX)

#define KEY_PTS_MAX (1024*150)

#define VISUAL_HULL_MAX_DIM 256

struct WeightsPair
{
	float w1;
	float w2;
};

class EDMatchingHelperCudaImpl : public CudaGlobalMemoryStatic
{
public:
	EDMatchingHelperCudaImpl(int vts_num_max)
		:cu_3dArr_depth_(NULL),
		cu_3dArr_normal_(NULL),
		dev_per_vertex_align_residual_(NULL),
		dev_per_vertex_align_residual_ed_interp_(NULL)
	{
		//allocate memory for costs on GPU
		checkCudaErrors(cudaMalloc(&dev_global_cost_data_, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_global_cost_vis_hull_, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_global_cost_reg_, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_global_cost_rot_, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_global_cost_temporal_, sizeof(float)));	
		checkCudaErrors(cudaMalloc(&dev_global_cost_keypts_, sizeof(float)));

		allocate_jtj_related_memory(vts_num_max);
		allocate_memory_rigid_alignment(vts_num_max);
		allocate_mem_for_keypts_matching(KEY_PTS_MAX);
		allocate_memory_ed_vertex_residual(vts_num_max, ED_NODES_NUM_MAX);
	}
	~EDMatchingHelperCudaImpl()
	{
		checkCudaErrors(cudaFree(dev_global_cost_data_));
		checkCudaErrors(cudaFree(dev_global_cost_vis_hull_));
		checkCudaErrors(cudaFree(dev_global_cost_reg_));
		checkCudaErrors(cudaFree(dev_global_cost_rot_));
		checkCudaErrors(cudaFree(dev_global_cost_temporal_));
		checkCudaErrors(cudaFree(dev_global_cost_keypts_));

	}
public:
	//should be called after init
	void clear_jtj_jtf()
	{
		checkCudaErrors(cudaMemset(this->dev_jtj_, 0, sizeof(float)*this->jtj_blks_num_.max_size * 144));
		checkCudaErrors(cudaMemset(this->dev_jtf_, 0, sizeof(float)*this->jtj_para_blks_num_.max_size * 12));
	}

	void allocate_jtj_related_memory(int vts_num_max);

	//should be called after init
	//compute jtj for both data term and visual hull term
	bool compute_jtj_jtf( float* dev_vts, 
						  float* dev_vts_t, 
						  cuda::gpu_size_data vts_num_gpu,
						  int vt_dim,
						  DeformGraphNodeCuda const*dev_ed_nodes,
						  RigidTransformCuda const*dev_rigid_transf,
						  double w_data,
						  double w_vis_hull,
						  double mu,
						  bool bEveluateJtJOffDiag = true
						);


	bool compute_jtj_jtf_reg(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
							int const* dev_ed_nodes_num_all_levels, float min_nodes_dist, float w_reg);
	bool compute_jtj_jtf_reg_vRobust(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
									int const* dev_ed_nodes_num_all_levels, float min_nodes_dist, float tau, float w_reg);
	bool compute_jtj_jtf_rot(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num_all_levels, double w_reg);
	void compute_jtj_jtf_temporal(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
								  RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
								  int const* dev_ed_nodes_num_all_levels, float w_temporal);
	void compute_jtj_jtf_temporal_vRobust(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
										  RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
										  int const* dev_ed_nodes_num_all_levels,
										  float tau_A, float tau_t, float w_temporal);

	void set_gpu_costs_to_zero();
	void sum_over_gpu_costs(float* dev_cost_sum);
	void evaluate_cost(float const* dev_vts, float const*dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, double w_data, double w_vis_hull, double mu, float2 *costs=NULL);
	void evaluate_cost_reg(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
							int const* dev_ed_nodes_num_all_levels, float min_nodes_dist, float w_reg, float *cost = NULL);
	void evaluate_cost_reg_vRobust(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
									int const* dev_ed_nodes_num_all_levels, float min_nodes_dist, float tau, float w_reg, float *cost = NULL);
	void evaluate_cost_rot(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num_all_levels, double w_rot, float* cost = NULL);
	void evaluate_cost_temporal(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
								 RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev, 
								 int const* dev_ed_nodes_num_all_levels, float w_temporal, float *cost = NULL);
	void evaluate_cost_temporal_vRobust(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
										 RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
										 int const* dev_ed_nodes_num_all_levels,
										 float tau_A, float tau_t, float w_temporal, float *cost = NULL);


	bool setup_vt_data_for_jtj( DeformGraphNodeCuda const*dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu,
								int *dev_ngns_indices, float* dev_ngns_weights, int vts_num,
							    int thres_pts_num = 100 //ignore jtj off-diagnal block if its associates points is leff than thres_pts_num
							   );

	bool setup_ed_nodes_reg_term( short2 *dev_jtj_2d_infos,
								DeformGraphNodeCuda *dev_ed_nodes, int ed_nodes_num,
								cudaArray* cu_3dArr_ndIds,
								int3 ed_cubes_dims,
								float3 ed_cubes_offsets,
								float ed_cube_res,
								float nodes_min_dist
								);

	void setup_vt_data_for_jtj_vHierED(DeformGraphNodeCuda const*dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu, int const*dev_ed_nodes_num_all_levels,
									   int *dev_ngns_indices, float* dev_ngns_weights, cuda::gpu_size_data vts_num_gpu,
									   int thres_pts_num = 100 //ignore jtj off-diagnal block if its associates points is leff than thres_pts_num
									   );

	void setup_ed_nodes_reg_term_vHierED(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num_all_levels,
										 int2* dev_ed_nodes_ranges[],
										 int ed_hierachy_levels,
										 int3 const* ed_cubes_dims,
										 float3 const* ed_cubes_offsets,
										 float ed_cube_res,
										 float nodes_min_dist //nodes dist on lowest level
										 );

public:
	void allocate_memory_visibility_check(
								int depth_width, int depth_height, //input depth map size
								int down_scale_trg
								);
	void project_points_to_depth_map(float const*dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, float vxl_res, bool bSaveFrontVtIdx=false);
	void calc_alignment_residual_on_depth(int filter_radius, float residual_cap);
	void calc_alignment_residual_on_depth(VolumeTwoLevelHierachy const* volume, int filter_radius);

	int const* dev_depth_mats_proj() { return dev_depth_mats_proj_; }
	int const* dev_depth_mats_corr_vtIdx() { return dev_depth_mats_corr_vtIdx_; }
	float const* dev_depth_align_residual() { return dev_depth_align_residual_; }
	float const* dev_depth_align_residual_f() { return dev_depth_align_residual_f_; }

	int depth_width_proj() { return depth_width_proj_; }
	int depth_height_proj() { return depth_height_proj_; }

protected:
	int depth_width_proj_;
	int depth_height_proj_;

	int* dev_depth_mats_proj_; //projected depth maps
	int* dev_depth_mats_corr_vtIdx_; //the vtIdx in front when projecting vts
	float *dev_depth_align_residual_; //the alignment residual
	float *dev_depth_align_residual_f_;

public:
	void allocate_memory_ed_vertex_residual(int vts_num_max, int ed_nodes_num_max);
	void calc_per_vertex_residual(float const* dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, float residual_cap, bool bUseSegmentation);
	void calc_per_vertex_residual(float const* dev_vts_t, cuda::gpu_size_data vts_num_gpu, int vt_dim, VolumeTwoLevelHierachy const* volume, bool bUseVisualHull=false);

	void aggregate_ed_nodes_residual(cuda::gpu_size_data ed_nodes_num_gpu);
	void update_ed_nodes_residual_with_reg_term(DeformGraphNodeCuda const* dev_ed_nodes, cuda::gpu_size_data ed_nodes_num_gpu, float thres_reg, float delta_on_ed_residual);
	void interp_per_vertex_residual_from_ed(int const* dev_ngns_indices, float const* dev_ngns_weights, int vts_num, int ed_nodes_num);
	float const* dev_per_vertex_align_residual() { return this->dev_per_vertex_align_residual_; }
	float const* dev_per_vertex_align_residual_ed_interp() { return this->dev_per_vertex_align_residual_ed_interp_; }
	float const* dev_ed_nodes_align_residual() { return this->dev_ed_nodes_align_residual_; }

protected:
	float *dev_per_vertex_align_residual_;
	float *dev_ed_nodes_align_residual_;
	float *dev_per_vertex_align_residual_ed_interp_;

	//rigid alignment: between input dev_vts and the depths
public:
	void allocate_memory_rigid_alignment(int vts_num_max);

	void setup_rigid_alignment(cuda::gpu_size_data vts_num_gpu);
	
	//Note: identity matrix is used for initialization
	void run_rigid_alignment(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vt_dim, float mu, int iter_max);
	
	//rigid_transf.R = R * rigid_transf.R
	//rigid_transf.T = R*rigid_transf.T + T
	void apply_rigid_alignment_to_RigidTransf(RigidTransformCuda *dev_rigid_transf);

	void init_rigid_alignment_to_identity();
	void set_rigid_alignment(float rod[3], float T[3]);
	void compute_jtj_jtf_rigid_alignment(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vt_dim, float mu);
	void evaluate_cost_rigid_alignment(float const* dev_vts, int vts_num, int vt_dim, float mu);
	void solve_and_update_rod_T_rigid_alignment();
	void solve_rigid_alignment(float opt_mu);
	//debug
	void readout_R_T_rigid_alignment(float rod[3], float R[9], float dR_drod[27], float T[3]);
	void readout_JtJ_JtF_rigid_alignment(float jtj[36], float jtf[6]);
	float readout_cost_rigid_alignment();
	void readout_partial_JtJs_JtFs(float *partial_jtjs, float *partial_jtfs);
	int partial_jtjs_rigid_alignment_count() { return this->partial_JtJs_rigid_align_count_; }
	void readout_drod_dT_rigid_alignment(float drod[3], float dT[3]);

protected:
	float* dev_rod_;
	float* dev_rod_dx_;
	float* dev_T_dx_;
	float* dev_R_;
	float* dev_dR_drod_;
	float* dev_T_;
	float* dev_cost_rigid_align_;
	float* dev_JtJ_rigid_align_;
	float* dev_JtF_rigid_align_;
	int partial_JtJs_rigid_align_count_;
	float* dev_partial_JtJs_rigid_align_;
	float* dev_partial_JtFs_rigid_align_;

//matched key points 
public:
	void allocate_mem_for_keypts_matching(int keypts_num_max);

	void calc_3d_keypts_in_reference(float3* dev_keypts_3d, ushort3 const* dev_keypts_2d, 
									 float const* dev_vts, float const* dev_vts_t, int vt_dim,
									 int const* dev_depth_maps_proj, int const* dev_depth_maps_vtIdx, 
									 int depth_width_prj, int depth_height_prj );
	void calc_3d_keypts_in_curr(float3* dev_keypts_3d, ushort3 const* dev_keypts_2d);
	void setup_keypts(float sigma_vt_node_dist);
	void compute_ngns_keypts(float sigma_vt_node_dist);
	void evaluate_cost_keypts(DeformGraphNodeCuda const* dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf, float w_keypts, float* cost=NULL);
	void evaluate_cost_keypts_vRobust(DeformGraphNodeCuda const* dev_ed_nodes, RigidTransformCuda const* dev_rigid_transf, float w_keypts, float tau, float*cost = NULL);
	void compute_jtj_jtf_keypts(DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num, 
								RigidTransformCuda const* dev_rigid_transf, float w_keypts, bool bEveluateJtJOffDiag = true);
	void compute_jtj_jtf_keypts_vRobust(DeformGraphNodeCuda const* dev_ed_nodes, int const* dev_ed_nodes_num,
								RigidTransformCuda const* dev_rigid_transf, float w_keypts, float tau, bool bEveluateJtJOffDiag = true);

	float3* dev_keypts_3d_p() { return dev_keypts_p_;}
	float3* dev_keypts_3d_q() { return dev_keypts_q_; }
	ushort3* dev_keypts_2d_p() { return dev_keypts_2d_p_; }
	ushort3* dev_keypts_2d_q() { return dev_keypts_2d_q_; }
	int keypts_num() { return keypts_num_gpu_.sync_read(); }

protected:
	cuda::gpu_size_data keypts_num_gpu_;
	float3* dev_keypts_p_; //source
	float3* dev_keypts_q_; //target
	ushort3* dev_keypts_2d_p_;
	ushort3* dev_keypts_2d_q_;
	int* dev_ngns_indices_keypts_;
	float* dev_ngns_weights_keypts_;

public:
	void feed_camera_view(CameraViewCuda* host_cam_views, int view_num);
	bool allocate_and_texBind_cuda_arrays_depth(int tex_num, int width, int height);
	void bind_cuda_array_to_texture_depth(cudaArray *cu_3dArr_depth);
	void bind_cuda_array_to_texture_normal(cudaArray *cu_3dArr_normal);
	bool bind_tex_ndId(cudaArray* cu_3dArr_ndIds_);

public:
	cuda::gpu_size_data const & jtj_blk_num(){ return this->jtj_blks_num_; }
	cuda::gpu_size_data const & jtj_para_blk_num() const { return this->jtj_para_blks_num_; }
	float* dev_mem_jtj(){ return this->dev_jtj_; }
	float* dev_mem_jtf(){ return this->dev_jtf_; }
	HessianBlockInfoCuda* dev_mem_blk_info(){ return this->dev_jtj_blk_info_buf_; }

public:
	void allocate_and_bind_vishull_cu3dArray();
	//setup visual hull
	void setup_visual_hull_info(BoundingBox3DCuda const* dev_bbox, float visual_hull_res);
	void compute_visual_hull_occupancy(bool bUseSegmentation);
	void blur_visual_hull_occupancy(int round);

protected:
	//for visual hull
	int3* dev_visual_hull_dim_;
	float3* dev_visual_hull_offset_;
	float visual_hull_res_;
	cudaArray* cu_3DArr_vishull_;

protected:
	float *dev_global_cost_vis_hull_;
	float *dev_global_cost_data_;
	float *dev_global_cost_reg_;
	float *dev_global_cost_rot_;
	float *dev_global_cost_temporal_;
	float *dev_global_cost_keypts_;

protected:
	int read_globalPos();
	int read_globalOffset();
	int read_globalOffset2();
	int read_globalRegCount();

	DeformGraphCudaImpl *graph_cuda_;

	HessianBlockInfoCuda *host_jtj_blk_info_;

	//for self pt-node allocation
	cuda::gpu_size_data jtj_blks_count_gpu_;
	HessianBlockInfoCuda *dev_jtj_blk_info_buf_;
	int* dev_vt_indices_for_jtj_;
	float* dev_vt_weights_for_jtj_;

	//for JtJ and JtF eveluation
	cuda::gpu_size_data jtj_blks_num_;
	cuda::gpu_size_data jtj_blks_num_data_;//the data term only
	cuda::gpu_size_data jtj_para_blks_num_;
	int para_count_max_;
	int jtj_ele_count_max_;
	//2d matrix (nodes_num X nodes_num). each element saves the data offset (in x) and 
	//flag(in y, lowest bit data term flag, second lowest bit reg term flag)
	short2 *dev_jtj_2d_infos_; 

	cuda::gpu_size_data ed_reg_pairs_count_gpu_;
	ushort2 *dev_reg_node_pairs_list_buf_;
	float *dev_jtj_;
	float *dev_jtf_;

	//for camera visibility
	ushort2 *dev_cam_vis_;

private:
	int* dev_global_pts_jtj_count_; //count of pts associated with jtj blocks
	int *dev_pt_counts_Diag_block_;
	int *dev_pt_counts_offDiag_block_;

public:
	int depth_width_;
	int depth_height_;
	int num_depthmaps_;
	float vxl_res_;
	cudaArray *cu_3dArr_depth_;
	cudaArray *cu_3dArr_normal_;
};


__forceinline__ __device__ float phi(float rr, float tau)
{
	if (rr < tau*tau)
		return rr*(1.0f - 0.5f*rr / (tau*tau));
	else
		return tau*tau / 2.0f;
}

__forceinline__ __device__
void robust_kernel_paras(float r, float tau, float &c, float &d, float &e)
{
	float a, b;
	if (r < M_EPS * tau)
	{
		c = 1.0f;
		d = 0.0f;
		e = 1.0f;
	}
	else
	{
		float rr = r*r;
		float phi_r = phi(rr, tau);
		float phi_sqrt = sqrtf(phi_r);

		a = phi_sqrt / r;
		c = a*a;
		if (rr < tau*tau)
			b = (1.0f - rr / (tau*tau)) / (r*phi_sqrt) - phi_sqrt / (r*rr);
		else
			b = -phi_sqrt / (r*rr);
		d = 2.0f * a*b + b*b*rr;

		e = a*(a + b*rr);
	}
}
