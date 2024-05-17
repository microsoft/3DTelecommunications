class EDMatchingLSQProblemCuda : public LSQProblem, public LSQProblemGPU
{
public:
	EDMatchingLSQProblemCuda( char const* dir_debug, 
							  int vts_num_max, int vt_dim,
							  int depth_num, int depth_width, int depth_height, 
							  bool bAllocateDepthArray = false//should be false for gpu version: because the Fusion class will hold the depth cuArr
							  )
		: dir_debug_(dir_debug),
		graph_cuda(vts_num_max, vt_dim, 4.0),
		jtj_helper_cuda(vts_num_max, depth_num, depth_width, depth_height, bAllocateDepthArray),
		graph(NULL)
	{
	}

	//cpu routines
public:
	//use cpu surface
	//use its internal cuda array
	void set_problem(CSurface<float> const*surface,
		NonrigidMatching::DeformGraph *graph, //could be NULL
		float nodes_min_dist,
		NonrigidMatching::NeighborGraphNodesOfPointList *ngns, //could be NULL
		vector<cv::Mat> depthImgs_trg,  //should be ushort16, no shifting
		vector<GCameraView*> cams_d,
		double w_sdf, double w_vis_hull, double w_rot, double w_reg, double align_mu, double vxl_res);
	void set_problem(CSurface<float> const*surface,
		NonrigidMatching::DeformGraph *graph_in, //could be NULL
		float nodes_min_dist,
		NonrigidMatching::NeighborGraphNodesOfPointList *ngns, //could be NULL
		cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
		vector<GCameraView*> cams_d,
		double w_sdf, double w_vis_hull, double w_rot, double w_reg, double align_mu, double vxl_res
		);
public:
	bool init(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf);
	bool evaluate(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf, bool bUseOffDiagJtJData, int frmIdx = -1, int iter = -1);
	double evaluateCurrentCost(int frmIdx = -1, int iter = -1); // ||F(x)||^2
	bool updateX(vnl_vector<float> const&dx);//update state variables
	bool backup_currentX(); //return state variables as a vector
	bool recover_backupX();
	void saveIntermediate(int idx) { ; }

	//gpu routinues
public:
	//Operations: ed nodes sampling, ngns computation, init jtj compuatation
	//NOTE: the ed nodes are sampled, but NOT initialized after this function call
	//use external memeory for dev_vts
	void gpu_set_problem(float* dev_vts, int stride, cuda::gpu_size_data vts_num_gpu, 
						 BoundingBox3DCuda const*dev_bbox_vts, BoundingBox3DCuda const*dev_bbox_cur,
						 cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
						 vector<GCameraView*> cams_d,
						 float nodes_min_dist, float nodes_min_dist_low_res,
						 double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
						 double align_mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
						 bool bComputeVisualHull,
						 bool bUseSegmentationForVisualHull,
						 bool& switch_to_low_res,
						 bool initBackground = false);
	void gpu_set_problem(float* dev_vts, int stride, int vts_num, 
						BoundingBox3D bbox_vts, BoundingBox3D bbox_cur,
						cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
						vector<GCameraView*> cams_d,
						float nodes_min_dist, float nodes_min_dist_low_res,
						double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
						double align_mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
						bool bComputeVisualHull,
						bool bUseSegmentationForVisualHull,
						bool& switch_to_low_res,
						bool initBackground = false);

	//use internal memory for dev_vts
	void gpu_set_problem(CSurface<float> &surface, 
						 BoundingBox3D bbox_surf, BoundingBox3D bbox_cur,
						 cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
						 vector<GCameraView*> cams_d,
						 float nodes_min_dist, float nodes_min_dist_low_res,
						 double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
						 double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
						 bool bComputeVisualHull,
						 bool bUseSegmentationForVisualHull,
						 bool& switch_to_low_res,
						 bool initBackground = false);

	//for debuging
	void gpu_set_problem(CSurface<float> &surface_src, 
		BoundingBox3D bbox_cur,
		char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info,
		cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
		vector<GCameraView*> cams_d,
		float nodes_min_dist,
		double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
		double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist_, float sigma_vt_node_dist,
		bool bComputeVisualHull,
		bool bUseSegmentationForVisualHull);

	//must run after ed nodes initialization: it aligns warped surface with initial ed paramemters to the target
	void rigid_alignment(float mu, int iter_max, int frmIdx)
	{
		this->jtj_helper_cuda.setup_rigid_alignment(graph_cuda.vts_num_gpu());
		this->graph_cuda.transform_surface();

		this->jtj_helper_cuda.run_rigid_alignment(graph_cuda.surface_t_device_memory(), graph_cuda.vts_num_gpu(), graph_cuda.surface_vt_dim(), mu, iter_max);
		this->jtj_helper_cuda.apply_rigid_alignment_to_RigidTransf(graph_cuda.dev_rigid_transf());
	}

	bool gpu_return_internal_JtJ_JtF(JtJBlockMatrixLTri &jtj, JtFVector &jtf);
	bool gpu_evaluate_JtJ_JtF(int frmIdx, int iter, bool bUseOffDiagJtJData);
	double gpu_evaluate_cost(int frmIdx, int iter); // ||F(x)||^2
	void gpu_evaluate_cost(float* dev_cost, int frmIdx, int iter);
	bool gpu_updateX(float const* dev_dx);//update state variables
	bool gpu_backup_currentX();
	bool gpu_recover_backupX();
	bool initX(){ return true; }

	//assume graph has the same structure as internel one
	bool init_ed_graph(NonrigidMatching::DeformGraph const&graph)
	{
		graph_cuda.update(graph, graph.nodes_dist);
		return true;
	}
	void init_ed_graph_to_identy()
	{
		graph_cuda.set_ed_nodes_paras_as_identity();
	}
	void init_ed_graph_with_old()
	{
		graph_cuda.init_ed_nodes_paras_with_old();
	}
	void init_ed_graph(EDNodesParasGPU ed_nodes_init)
	{
		graph_cuda.init_ed_nodes_paras(ed_nodes_init);
	}

	void readout_deform_graph(NonrigidMatching::DeformGraph &graph)
	{
		this->graph_cuda.readout_DeformGraph(graph, false);
	}

	void copy_vts_t_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data buf_size_gpu_out)
	{
		this->graph_cuda.copy_vts_t_to_dev_buf(dev_buf_out, stride_out, buf_size_gpu_out);
	}


	//must be called after set_problem. 
	NonrigidMatching::DeformGraph* return_internal_DeformGraph(){ return this->graph; }

	void getCurrentSurfaceT(CSurface<float> &surface_t)
	{
		if (this->surface_src)
			surface_t = *(this->surface_src);
		graph_cuda.read_out_surface_t(surface_t);
		if (this->surface_src && surface_src->haveColorInfo())
		{
			for (int vtIdx = 0; vtIdx < surface_src->vtNum; vtIdx++)
			{
				float const* clr = surface_src->vt_color(vtIdx);
				float *clr_d = surface_t.vt_color(vtIdx);
				clr_d[0] = clr[0];
				clr_d[1] = clr[1];
				clr_d[2] = clr[2];
			}
		}
	}

private:
	// Tries to sample ED nodes with higher resolution and upon failure decreases resolution of ED node distances/sigma. Failure occurs if the amount of sampled ED nodes 
	// is higher than pre-allocated GPU memory resources. If low resolution sampling fails, the code is forced to exit. TODO: throw an exception and try to do some cleaning up
	// instead of exiting.
	void safe_sample_ed_nodes_vGMem(const BoundingBox3D& bbox_f, float& nodes_min_dist, float nodes_min_dist_low_res, float& sigma_vt_node_dist, float sigma_vt_node_dist_low_res, bool &switch_to_low_res, bool initBackground);
	void safe_sample_ed_nodes_impl_vGMem(BoundingBox3DCuda const* dev_bbox_vts, float& nodes_min_dist, float nodes_min_dist_low_res, float& sigma_vt_node_dist, float sigma_vt_node_dist_low_res, bool& switch_to_low_res, bool initBackground);

public:
	float* dev_mem_jtj(){ return jtj_helper_cuda.dev_mem_jtj(); }
	float* dev_mem_jtf(){ return jtj_helper_cuda.dev_mem_jtf(); }
	HessianBlockInfoCuda* dev_mem_blk_info(){ return jtj_helper_cuda.dev_mem_blk_info(); }
	cuda::gpu_size_data jtj_blk_num() { return jtj_helper_cuda.jtj_blk_num(); }
	cuda::gpu_size_data jtj_para_blk_num() { return jtj_helper_cuda.jtj_para_blk_num(); }
	DeformGraphNodeCuda* dev_ed_nodes(){ return graph_cuda.dev_ed_nodes(); }

	cuda::gpu_size_data ed_nodes_num_gpu() { return graph_cuda.ed_nodes_num_gpu(); }
	float nodes_min_dist() { return graph_cuda.nodes_min_dist(); }
	int3 ed_cubes_dim() { return graph_cuda.ed_cubes_dims(); }
	float3 ed_cubes_offset() { return graph_cuda.ed_cubes_offset(); }
	int3 const* dev_ed_cubes_dim() { return graph_cuda.dev_ed_cubes_dim(); }
	float3 const* dev_ed_cubes_offset() { return graph_cuda.dev_ed_cubes_offset(); }
	float ed_cubes_res() { return graph_cuda.ed_cubes_res(); }
	cudaArray* cu_3dArr_ndIds() { return graph_cuda.cu_3dArr_ndIds(); }
	RigidTransformCuda* dev_rigid_transf() { return graph_cuda.dev_rigid_transf(); }

public:
	bool dump_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info);
	bool feed_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info);
	//dump current jtj and jtf
	void dump_gpu_jtj_jtf(char const* filename_jtj, char const* filename_jtf);
	void dump_visual_hull(char const* dir, int idx){
		this->jtj_helper_cuda.dump_visual_hull(dir, idx);
	}

private:
	CSurface<float> const*surface_src;
	vector<cv::Mat> depthImgs_trg;  //pointer to external memory
	vector<GCameraView*> cams_d; //pointer to external memory
public:
	double w_sdf;
	double w_vis_hull;
	double w_rot;
	double w_reg;
	double w_temporal;
	double w_keypts;
	double mu;
	double vxl_res;


public:
	NonrigidMatching::DeformGraph *graph; //obsolete, due to gpu ED node generation
	float nodes_min_dist_;
	float sigma_nodes_dist_;
	float sigma_vt_node_dist_;
	DeformGraphCuda graph_cuda;
	EDMatchingHelperCuda jtj_helper_cuda;
private:
	char const* dir_debug_;
	vnl_matrix<bool> jtj_reg_mat;
};