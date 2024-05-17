using namespace NonrigidMatching;


void EDMatchingLSQProblemCuda::
gpu_set_problem(float* dev_vts, int stride, cuda::gpu_size_data vts_num_gpu,
	BoundingBox3DCuda const* dev_bbox_vts, BoundingBox3DCuda const* dev_bbox_cur,
	cudaArray* cu_3dArr_depth, cudaArray* cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
	vector<GCameraView*> cams_d,
	float nodes_min_dist, float nodes_min_dist_low_res,
	double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
	double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
	bool bComputeVisualHull,
	bool bUseSegmentationForVisualHull,
	bool& switch_to_low_res,
	bool initBackground)
{
	this->surface_src = NULL;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->w_temporal = w_temporal;
	this->w_keypts = w_keypts;
	this->mu = mu;
	this->vxl_res = vxl_res;
	this->nodes_min_dist_ = nodes_min_dist;
	this->sigma_nodes_dist_ = sigma_nodes_dist;
	this->sigma_vt_node_dist_ = sigma_vt_node_dist;

	//setup graph_cuda
	graph_cuda.set_surface_to_deform(dev_vts, stride, vts_num_gpu);

	safe_sample_ed_nodes_impl_vGMem(dev_bbox_vts, nodes_min_dist, nodes_min_dist_low_res, sigma_vt_node_dist, sigma_vt_node_dist_low_res, switch_to_low_res, initBackground);

	graph_cuda.build_ed_nodes_hierarchy(ed_hier_levels);
	graph_cuda.compute_ngns(sigma_vt_node_dist);


	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, dev_bbox_cur, bUseSegmentationForVisualHull,
		cu_3dArr_depth, cu_3dArr_normal, 
		depth_num, depth_width, depth_height, 
		cams_d, vxl_res, 
		bComputeVisualHull,
		dir_debug_);

	if (dir_debug_ != NULL)
	{
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}
}

void EDMatchingLSQProblemCuda::
gpu_set_problem(float* dev_vts, int stride, int vts_num,
	BoundingBox3D bbox_vts, BoundingBox3D bbox_cur,
	cudaArray* cu_3dArr_depth, cudaArray* cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
	vector<GCameraView*> cams_d,
	float nodes_min_dist, float nodes_min_dist_low_res,
	double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
	double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
	bool bComputeVisualHull,
	bool bUseSegmentationForVisualHull,
	bool& switch_to_low_res,
	bool initBackground)
{
	this->surface_src = NULL;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->w_temporal = w_temporal;
	this->w_keypts = w_keypts;
	this->mu = mu;
	this->vxl_res = vxl_res;
	this->nodes_min_dist_ = nodes_min_dist;
	this->sigma_nodes_dist_ = sigma_nodes_dist;
	this->sigma_vt_node_dist_ = sigma_vt_node_dist;

	//setup graph_cuda
	graph_cuda.set_surface_to_deform(dev_vts, stride, vts_num);

	safe_sample_ed_nodes_vGMem(bbox_vts, nodes_min_dist, nodes_min_dist_low_res, sigma_vt_node_dist, sigma_vt_node_dist_low_res, switch_to_low_res, initBackground);

	graph_cuda.build_ed_nodes_hierarchy(ed_hier_levels);
	graph_cuda.compute_ngns(sigma_vt_node_dist);


	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, bbox_cur, bUseSegmentationForVisualHull, 
		cu_3dArr_depth, cu_3dArr_normal, 
		depth_num, depth_width, depth_height, 
		cams_d, vxl_res, 
		bComputeVisualHull,
		dir_debug_);

	if (dir_debug_ != NULL)
	{
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}
}

void EDMatchingLSQProblemCuda::
gpu_set_problem(CSurface<float> &surface_src, 
				BoundingBox3D bbox_surf, BoundingBox3D bbox_cur,
				cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
				vector<GCameraView*> cams_d,
				float nodes_min_dist, float nodes_min_dist_low_res,
				double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
				double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist, float sigma_vt_node_dist_low_res,
				bool bComputeVisualHull,
				bool bUseSegmentationForVisualHull,
				bool& switch_to_low_res,
				bool initBackground)
{
	this->surface_src = NULL;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->w_temporal = w_temporal;
	this->w_keypts = w_keypts;
	this->mu = mu;
	this->vxl_res = vxl_res;
	this->nodes_min_dist_ = nodes_min_dist;
	this->sigma_nodes_dist_ = sigma_nodes_dist;
	this->sigma_vt_node_dist_ = sigma_vt_node_dist;

	//setup graph_cuda
	graph_cuda.set_surface_to_deform(surface_src);
	BoundingBox3D bbox_tight = surface_src.get_bbox();
	BoundingBox3D bbox_f = bbox_min(bbox_surf, bbox_tight);
	
	safe_sample_ed_nodes_vGMem(bbox_f, nodes_min_dist, nodes_min_dist_low_res, sigma_vt_node_dist, sigma_vt_node_dist_low_res, switch_to_low_res, initBackground);

	graph_cuda.build_ed_nodes_hierarchy(ed_hier_levels);
	graph_cuda.compute_ngns(sigma_vt_node_dist);

	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, bbox_cur, bUseSegmentationForVisualHull, 
		cu_3dArr_depth, cu_3dArr_normal, 
		depth_num, depth_width, depth_height, 
		cams_d, vxl_res, 
		bComputeVisualHull,
		dir_debug_);

	if (dir_debug_ != NULL)
	{
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}
}

void EDMatchingLSQProblemCuda::safe_sample_ed_nodes_vGMem(const BoundingBox3D& bbox_f, float& nodes_min_dist, float nodes_min_dist_low_res, float& sigma_vt_node_dist, float sigma_vt_node_dist_low_res, bool &switch_to_low_res, bool initBackground)
{
	bool ed_sampling_success = graph_cuda.sample_ed_nodes_vGMem(bbox_f, nodes_min_dist, 4, initBackground, true);
	
	switch_to_low_res = false;

	// Checking if high resolution sampling worked (i.e., if the amount of ED nodes is within the pre-allocated resources)
	if (!ed_sampling_success) {		
		LOGGER()->warning("EDMatchingLSQProblemCuda::safe_sample_ed_nodes_vGMem","Retrying ED node sampling (prev min_dist %f new %f - prev sigma %f new %f)", nodes_min_dist, nodes_min_dist_low_res, sigma_vt_node_dist, sigma_vt_node_dist_low_res);

		nodes_min_dist = nodes_min_dist_low_res;
		sigma_vt_node_dist = sigma_vt_node_dist_low_res;

		this->nodes_min_dist_ = nodes_min_dist;
		this->sigma_vt_node_dist_ = sigma_vt_node_dist;

		ed_sampling_success = graph_cuda.sample_ed_nodes_vGMem(bbox_f, nodes_min_dist, 4, initBackground, false);

		if (!ed_sampling_success) {
			LOGGER()->fatal("EDMatchingLSQProblemCuda::safe_sample_ed_nodes_vGMem"," -- Breaking code because the sampled amount of low resolution ED nodes surpasses pre-allocated memory resources (low res min_dist %f sigma %f)", nodes_min_dist_low_res, sigma_vt_node_dist_low_res);
		}
		// If low resolution sampling fails, the code should break because we were not able to estimate an amount of ED nodes that fits
		// into pre-allocated memory resources
		assert(ed_sampling_success);

		switch_to_low_res = true;
	}
}

void EDMatchingLSQProblemCuda::safe_sample_ed_nodes_impl_vGMem(BoundingBox3DCuda const* dev_bbox_vts, float& nodes_min_dist, float nodes_min_dist_low_res, float& sigma_vt_node_dist, float sigma_vt_node_dist_low_res, bool& switch_to_low_res, bool initBackground)
{
	bool ed_sampling_success = graph_cuda.sample_ed_nodes_impl_vGMem(dev_bbox_vts, nodes_min_dist, 4, initBackground, true);

	// Checking if high resolution sampling worked (i.e., if the amount of ED nodes is within the pre-allocated resources)
	if (!ed_sampling_success) {
		LOGGER()->warning("EDMatchingLSQProblemCuda::safe_sample_ed_nodes_impl_vGMem","Retrying ED node sampling (prev min_dist %f new %f - prev sigma %f new %f)", nodes_min_dist, nodes_min_dist_low_res, sigma_vt_node_dist, sigma_vt_node_dist_low_res);

		nodes_min_dist = nodes_min_dist_low_res;
		sigma_vt_node_dist = sigma_vt_node_dist_low_res;

		this->nodes_min_dist_ = nodes_min_dist;
		this->sigma_vt_node_dist_ = sigma_vt_node_dist;

		ed_sampling_success = graph_cuda.sample_ed_nodes_impl_vGMem(dev_bbox_vts, nodes_min_dist, 4, initBackground, false);

		if (!ed_sampling_success) {
			LOGGER()->fatal("EDMatchingLSQProblemCuda::safe_sample_ed_nodes_impl_vGMem", " -- Breaking code because the sampled amount of low resolution ED nodes surpasses pre-allocated memory resources (low res min_dist %f sigma %f)", nodes_min_dist_low_res, sigma_vt_node_dist_low_res);
		}

		// If low resolution sampling fails, the code should break because we were not able to estimate an amount of ED nodes that fits
		// into pre-allocated memory resources
		assert(ed_sampling_success);

		switch_to_low_res = true;
	}

}

void EDMatchingLSQProblemCuda::
gpu_set_problem(CSurface<float> &surface_src, 
				BoundingBox3D bbox_cur,
				char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info,
				cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
				vector<GCameraView*> cams_d,
				float nodes_min_dist,
				double w_sdf, double w_vis_hull, double w_rot, double w_reg, double w_temporal, double w_keypts,
				double mu, double vxl_res, int ed_hier_levels, float sigma_nodes_dist, float sigma_vt_node_dist,
				bool bComputeVisualHull,
				bool bUseSegmentationForVisualHull)
{
	this->surface_src = NULL;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->w_temporal = w_temporal;
	this->w_keypts = w_keypts;
	this->mu = mu;
	this->vxl_res = vxl_res;
	this->nodes_min_dist_ = nodes_min_dist;
	this->sigma_nodes_dist_ = sigma_nodes_dist;
	this->sigma_vt_node_dist_ = sigma_vt_node_dist;

	//setup graph_cuda
	graph_cuda.set_surface_to_deform(surface_src);
	graph_cuda.feed_ed_nodes_infos(filename_ed_graph, filename_arr_ndIds, filename_ed_info);

	graph_cuda.compute_ngns(sigma_vt_node_dist);

	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, bbox_cur, bUseSegmentationForVisualHull, 
		cu_3dArr_depth, cu_3dArr_normal, 
		depth_num, depth_width, depth_height, 
		cams_d, vxl_res, 
		bComputeVisualHull,
		dir_debug_);

	if (dir_debug_ != NULL)
	{
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}
}

bool EDMatchingLSQProblemCuda::
gpu_return_internal_JtJ_JtF(JtJBlockMatrixLTri &jtj, JtFVector &jtf)
{
	jtj.blk_info = this->dev_mem_blk_info();
	jtj.data = this->dev_mem_jtj();
	jtj.blks_num = this->jtj_blk_num();
	jtj.para_blks_num = this->jtj_para_blk_num();
	jtj.para_blk_dim = 12;

	jtf.jtf = this->dev_mem_jtf();

	jtf.n.copy_transformed(this->jtj_para_blk_num(), 12, 0);

	return true;
}

bool EDMatchingLSQProblemCuda::
gpu_evaluate_JtJ_JtF(int frmIdx, int iter, bool bUseOffDiagJtJData)
{
	graph_cuda.transform_surface();

	jtj_helper_cuda.compute_jtj_jtf(graph_cuda.surface_device_memory(), graph_cuda.surface_t_device_memory(),
		graph_cuda.vts_num_gpu(), graph_cuda.surface_vt_dim(),
		graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_rigid_transf(),
		this->w_sdf, this->w_vis_hull, this->mu, bUseOffDiagJtJData);

	if (dir_debug_ && iter >= 0)
	{
		char name[500];
		sprintf(name, "%s/surface_t_iter_vis%03d.bin", dir_debug_, iter);

		CSurface<float> surface_src_t;
		if (surface_src != NULL)
			surface_src_t = *surface_src;
		graph_cuda.read_out_surface_t(surface_src_t);
		vnl_vector<ushort2> cam_vis;
		jtj_helper_cuda.readout_cam_vis(cam_vis);
		surface_src_t.expand_data(true, true);
		for (int i = 0; i < surface_src_t.vtNum; i++)
		{
			float* clr = surface_src_t.vt_color(i);
			if (cam_vis[i].x)
			{
				clr[0] = 1.0;
				clr[1] = 0.2;
				clr[2] = 0.2;
			}
			else
			{
				clr[0] = 0.2;
				clr[1] = 1.0;
				clr[2] = 0.2;
			}
		}
		surface_src_t.writeToFileBIN(name);
	}

	float const tau_reg = 2.0f;
	jtj_helper_cuda.compute_jtj_jtf_reg_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(),
												graph_cuda.dev_ed_nodes_num_all_levels(),
												this->sigma_nodes_dist_, tau_reg, this->w_reg);


	jtj_helper_cuda.compute_jtj_jtf_rot(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_num_all_levels(), this->w_rot);

	float const tau_keypts = 20.0f;
	if (w_keypts > 0)
		jtj_helper_cuda.compute_jtj_jtf_keypts_vRobust(graph_cuda.dev_ed_nodes(), graph_cuda.ed_nodes_num_gpu().dev_ptr,
									graph_cuda.dev_rigid_transf(), w_keypts, tau_keypts, bUseOffDiagJtJData);



	float const tau_A = 0.2;
	float const tau_t = 1.0;
	jtj_helper_cuda.compute_jtj_jtf_temporal_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(), 
		graph_cuda.dev_rigid_transf(), graph_cuda.dev_rigid_transf_prev(),
		graph_cuda.ed_nodes_num_gpu().dev_ptr, tau_A, tau_t, w_temporal);


	return true;
}

void EDMatchingLSQProblemCuda::
gpu_evaluate_cost(float* dev_cost, int frmIdx, int iter)
{
	graph_cuda.transform_surface();

	if (dir_debug_ && iter >= 0)
	{
		char name[500];
		sprintf(name, "%s/surface_t_iter%03d.bin", dir_debug_, iter);

		CSurface<float> surface_src_t;
		if (surface_src != NULL)
			surface_src_t = *surface_src;
		graph_cuda.read_out_surface_t(surface_src_t);
		surface_src_t.writeToFileBIN(name);

		DeformGraphHierachy graph_hier;
		graph_cuda.readout_DeformGraph_all_levels(graph_hier);
		sprintf(name, "%s/graph_iter%03d.txt", dir_debug_, iter);
		graph_hier.save_to_txt(name);
	}

	jtj_helper_cuda.set_gpu_costs_to_zero();

	jtj_helper_cuda.evaluate_cost(graph_cuda.surface_device_memory(), graph_cuda.surface_t_device_memory(),
		graph_cuda.vts_num_gpu(), graph_cuda.surface_vt_dim(), w_sdf, w_vis_hull, this->mu, NULL);
	jtj_helper_cuda.evaluate_cost_rot(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_num_all_levels(), w_rot, NULL);

	float const tau_reg = 2.0f;
	jtj_helper_cuda.evaluate_cost_reg_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(),
		graph_cuda.dev_ed_nodes_num_all_levels(), this->sigma_nodes_dist_, tau_reg, w_reg, NULL);


	float const tau_keypts = 20.0f;
	if (w_keypts > 0)
		jtj_helper_cuda.evaluate_cost_keypts_vRobust(graph_cuda.dev_ed_nodes(), graph_cuda.dev_rigid_transf(), w_keypts, tau_keypts, NULL);


	float const tau_A = 0.2;
	float const tau_t = 1.0;
	if (w_temporal > 0)
		jtj_helper_cuda.evaluate_cost_temporal_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(),
			graph_cuda.dev_rigid_transf(), graph_cuda.dev_rigid_transf_prev(),
			graph_cuda.ed_nodes_num_gpu().dev_ptr, tau_A, tau_t, w_temporal, NULL);


	jtj_helper_cuda.sum_over_gpu_costs(dev_cost);
	if (LOGGER()->check_verbosity(Logger::Trace))
	{
		float cost_sum = 0;
		checkCudaErrors(cudaMemcpy(&cost_sum, dev_cost, sizeof(float), cudaMemcpyDeviceToHost));
		LOGGER()->debug("cost_sum <%f>", cost_sum);
	}
}

double EDMatchingLSQProblemCuda::
gpu_evaluate_cost(int frmIdx, int iter) // ||F(x)||^2
{
	graph_cuda.transform_surface();

	if (dir_debug_ && iter >= 0)
	{
		char name[500];
		sprintf(name, "%s/surface_t_iter%03d.bin", dir_debug_, iter);
		
		CSurface<float> surface_src_t;
		if (surface_src != NULL)
			surface_src_t = *surface_src;
		graph_cuda.read_out_surface_t(surface_src_t);
		surface_src_t.writeToFileBIN(name);

		DeformGraphHierachy graph_hier;
		graph_cuda.readout_DeformGraph_all_levels(graph_hier);
		sprintf(name, "%s/graph_iter%03d.txt", dir_debug_, iter);
		graph_hier.save_to_txt(name);
	}

	float2 cost_data_and_vishull = make_float2(0.0, 0.0);
	jtj_helper_cuda.evaluate_cost(graph_cuda.surface_device_memory(), graph_cuda.surface_t_device_memory(),
		graph_cuda.vts_num_gpu(), graph_cuda.surface_vt_dim(), w_sdf, w_vis_hull, this->mu, &cost_data_and_vishull);
	float cost_rot = 0.0f;
	jtj_helper_cuda.evaluate_cost_rot(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_num_all_levels(), w_rot, &cost_rot);
	
	float cost_reg = 0.0;

	float const tau_reg = 2.0f;
	jtj_helper_cuda.evaluate_cost_reg_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(),
		graph_cuda.dev_ed_nodes_num_all_levels(), this->sigma_nodes_dist_, tau_reg, w_reg, &cost_reg);


	float cost_keypts = 0;

	float const tau_keypts = 20.0f;
	if(w_keypts > 0)
		jtj_helper_cuda.evaluate_cost_keypts_vRobust(graph_cuda.dev_ed_nodes(), graph_cuda.dev_rigid_transf(), w_keypts, tau_keypts, &cost_keypts);

	
	float cost_temporal = 0;

	float const tau_A = 0.2;
	float const tau_t = 1.0;
	jtj_helper_cuda.evaluate_cost_temporal_vRobust(graph_cuda.ed_nodes_device_memory(), graph_cuda.dev_ed_nodes_initial(), 
																graph_cuda.dev_rigid_transf(), graph_cuda.dev_rigid_transf_prev(),
																graph_cuda.ed_nodes_num_gpu().dev_ptr, tau_A, tau_t, w_temporal, &cost_temporal);


	LOGGER()->trace("cost: data<%f>; visual hull<%f>; rot<%f>; reg<%f>; temporal<%f>; keypts<%f>", cost_data_and_vishull.x, cost_data_and_vishull.y,
		cost_rot, cost_reg, cost_temporal, cost_keypts);

	return cost_data_and_vishull.x + cost_data_and_vishull.y + cost_rot + cost_reg + cost_temporal + cost_keypts;
}

bool EDMatchingLSQProblemCuda::gpu_updateX(float const* dev_dx)//update state variables
{
	return this->graph_cuda.update_ed_paras(dev_dx);
}

bool EDMatchingLSQProblemCuda::gpu_backup_currentX()
{
	return this->graph_cuda.backup_current_ed_paras();
}

bool EDMatchingLSQProblemCuda::gpu_recover_backupX()
{
	return this->graph_cuda.recover_backupEDParas();
}