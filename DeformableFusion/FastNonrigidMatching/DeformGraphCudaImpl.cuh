class DeformGraphCudaImpl : public CudaGlobalMemoryStatic
{
public:
	DeformGraphCudaImpl()
		: dev_ed_nodes_buf_(NULL),
		 vt_dim_(0), dev_vts_(NULL), dev_vts_t_(NULL)
	{
		checkCudaErrors(cudaMalloc(&(vts_num_gpu_.dev_ptr), sizeof(int)));

		checkCudaErrors(cudaMalloc(&dev_rigid_transf_, sizeof(RigidTransformCuda)));
		checkCudaErrors(cudaMalloc(&dev_rigid_transf_prev_, sizeof(RigidTransformCuda)));

		const int cubes_num_x = ED_CUBE_DIM_MAX;
		const int cubes_num_y = ED_CUBE_DIM_MAX;
		const int cubes_num_z = ED_CUBE_DIM_MAX;
		//allocate two cuda arrays for ndIds
		allocate_and_bind_ndIds_cu3dArray(cubes_num_x, cubes_num_y, cubes_num_z);

		allocate_ed_nodes();

		checkCudaErrors(cudaMalloc(&dev_pt_counts_per_cube_, sizeof(int)*ED_CUBE_NUM_MAX));
		checkCudaErrors(cudaMalloc(&dev_pt_centroids_per_cube_, sizeof(float3)*ED_CUBE_NUM_MAX));
		checkCudaErrors(cudaMalloc(&dev_pt_norms_per_cube_, sizeof(float3)*ED_CUBE_NUM_MAX));

		checkCudaErrors(cudaMalloc(&dev_pt_counts_per_cube_background_, sizeof(int)*ED_CUBE_NUM_MAX));
		_backgroundSet = false;
	}

	~DeformGraphCudaImpl()
	{
		checkCudaErrors(cudaFree(dev_rigid_transf_));
		//TODO: free cuda memory
	}

public:
	bool transform_surface();

	void sample_ed_nodes_impl(BoundingBox3DCuda const*dev_bbox,
						 float min_nodes_dist, int thres_pt_count);
	void sample_ed_nodes_impl(BoundingBox3DCuda bbox, float min_nodes_dist, int thres_pt_count);

	bool sample_ed_nodes_impl_vGMem(BoundingBox3DCuda const*dev_bbox,
		float min_nodes_dist, int thres_pt_count, bool initBackground = false, bool bSwitch_ed_nodes_buf = true);
	bool sample_ed_nodes_impl_vGMem(BoundingBox3DCuda bbox, float min_nodes_dist, int thres_pt_count, bool initBackground = false, bool bSwitch_ed_nodes_buf = true);

	//levels_num: overall levels include the lowest level. levels_num must >= 1
	void build_ed_nodes_hierarchy(int levels_num);

	bool compute_ngns(float sigma_vt_node_dist = 1.5f);
	bool compute_ngns(float const* dev_vts, int vt_dim, cuda::gpu_size_data vts_num_gpu,
					int* dev_ngns_indices, float* dev_ngns_weights, float sigma_vt_node_dist = 1.5);

	void set_ed_nodes_paras_as_identity();

	//using the old ed nodes to initialize current ed nodes
	void init_ed_nodes_paras_with_old();

	//also initialize rigid parameters
	void init_ed_nodes_paras(EDNodesParasGPU ed_nodes_init);

	void correct_initial_ed_nodes_paras_after_rigid_align();

	void copy_vts_t_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data vts_num_gpu_out);

	bool update_ed_paras(float const* dev_dx);
	bool backup_current_ed_paras();
	bool recover_backupEDParas();

	//inplace operation, does not depent on memory in dev_vts_/dev_vts_t_
public:
	int surface_vt_num() { return this->vts_num_gpu_.sync_read(); }
	int vts_num() { return this->vts_num_gpu_.sync_read(); }
	cuda::gpu_size_data vts_num_gpu() { return this->vts_num_gpu_; }
	int surface_vt_dim() { return this->vt_dim_; }
	int vt_dim() { return vt_dim_; }
	int ed_nodes_num() { return nodes_num_gpu_.sync_read(); }
	cuda::gpu_size_data ed_nodes_num_gpu() { return nodes_num_gpu_; }
	int2** dev_ed_nodes_ranges(){ return this->dev_ed_nodes_ranges_; }
	int ed_hierarchy_levels() { return ed_hierachy_levels_; }
	int const* dev_ed_nodes_num_all_levels() { return dev_ed_nodes_num_all_levels_; }
	cuda::gpu_size_data ed_nodes_num_all_levels(){
		return cuda::gpu_size_data(dev_ed_nodes_num_all_levels_, ED_NODES_NUM_MAX);
	}

	float nodes_min_dist() { return this->ed_cubes_res_; }
	float* surface_device_memory(){ return dev_vts_; }
	float* surface_t_device_memory() { return dev_vts_t_; }
	float* dev_vts() { return dev_vts_; }
	float* dev_vts_t() { return dev_vts_t_; }
	cudaArray* cu_3dArr_ndIds(){ return this->cu_3dArr_ndIds_; }
	int3 ed_cubes_dims() { 
		int3 ret;
		checkCudaErrors(cudaMemcpy(&ret, dev_ed_cubes_dims_, sizeof(int3), cudaMemcpyDeviceToHost)); 
		return ret;
	}
	int3 const* dev_ed_cubes_dim() const { return dev_ed_cubes_dims_; }
	float3 ed_cubes_offset(){ 
		float3 ret;
		checkCudaErrors(cudaMemcpy(&ret, dev_ed_cubes_offset_, sizeof(float3), cudaMemcpyDeviceToHost));
		return ret;
	}
	float3 const* dev_ed_cubes_offset() { return dev_ed_cubes_offset_; }
	float ed_cubes_res() { return this->ed_cubes_res_; }
	DeformGraphNodeCuda* ed_nodes_device_memory() { return dev_ed_nodes_buf_; }
	DeformGraphNodeCuda* dev_ed_nodes() { return dev_ed_nodes_buf_; }
	RigidTransformCuda* dev_rigid_transf() { return this->dev_rigid_transf_; }
	RigidTransformCuda* dev_rigid_transf_prev() { return this->dev_rigid_transf_prev_; }
	int* ngns_indices_device_memory() { return this->dev_ngns_indices_; }
	float* ngns_weights_device_memory() { return this->dev_ngns_weights_; }
	int const* dev_vts_ngn_indices() { return this->dev_ngns_indices_; }
	float const* dev_vts_ngn_weights() { return this->dev_ngns_weights_; }

	DeformGraphNodeCoreCuda* dev_ed_nodes_initial(){ return dev_ed_nodes_initial_; }

	void set_ed_cubes_dims(int3 dim){ 
		checkCudaErrors(cudaMemcpy(dev_ed_cubes_dims_, &dim, sizeof(int3), cudaMemcpyHostToDevice));
		 }
	void set_ed_cubes_offset(float3 offset){ 
		checkCudaErrors(cudaMemcpy(dev_ed_cubes_offset_, &offset, sizeof(float3), cudaMemcpyHostToDevice));
	}
	void set_ed_cubes_res(float res) { this->ed_cubes_res_ = res; }

protected:
	void bind_tex_ndIds_old(cudaArray *cu_3dArr_ndIds);
	void bind_tex_ndIds(cudaArray *cu_3dArr_ndIds);
	bool allocate_and_bind_ndIds_cu3dArray(int nx, int ny, int nz);
	void allocate_ed_nodes();
	//switch the dev_ed_nodes_buf_ and cuda 3D array for ndIds
	//bind to texture or surface
	void switch_ed_nodes_buf();
protected:
	RigidTransformCuda* dev_rigid_transf_; //permanent memory
	RigidTransformCuda* dev_rigid_transf_prev_;

	//for LM solver
	float *dev_ed_paras_backup_; //backup A, t, A_inv_t
	DeformGraphNodeCoreCuda* dev_ed_nodes_initial_; //used for temporal consistency

	//for hierarchy graphs
	int ed_hierachy_levels_;
	int2* dev_ed_nodes_ranges_[ED_HIER_LEVEL_NUM_MAX]; //for all levels: x: start point; y: nodes num at current level
	int *dev_ed_nodes_num_all_levels_;

	//ed nodes at current frame
	cuda::gpu_size_data nodes_num_gpu_; //ed nodes num on the lowest level: nodes used for surface warping
	DeformGraphNodeCuda *dev_ed_nodes_buf_;
	int3* dev_ed_cubes_dims_;
	float3* dev_ed_cubes_offset_;
	float ed_cubes_res_;
	cudaArray *cu_3dArr_ndIds_;

	int3* dev_ed_cubes_dims_background_;
	float3* dev_ed_cubes_offset_background_;


	//the ed nodes at last frame
	cuda::gpu_size_data nodes_num_gpu_old_;
	DeformGraphNodeCuda *dev_ed_nodes_buf_old_;
	int3* dev_ed_cubes_dims_old_;
	float3* dev_ed_cubes_offset_old_;
	float ed_cubes_res_old_;
	cudaArray *cu_3dArr_ndIds_old_;

	//NeighborGraphNodesOfPoint
	int *dev_ngns_indices_;
	float *dev_ngns_weights_;

	//surface
	cuda::gpu_size_data vts_num_gpu_;
	int vt_dim_;
	float *dev_vts_;
	float *dev_vts_t_;

private:
	int* dev_pt_counts_per_cube_;
	float3* dev_pt_centroids_per_cube_;
	float3* dev_pt_norms_per_cube_;

	int* dev_pt_counts_per_cube_background_;
	bool _backgroundSet;

	//for initialize ed nodes
	float* dev_tmp_weights_ed_init_;
	float* dev_tmp_paras_ed_init_;
};
