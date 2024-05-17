using namespace NonrigidMatching;

void EDMatchingLSQProblemCuda::
set_problem(CSurface<float> const*surface,
			NonrigidMatching::DeformGraph *graph_in, //could be NULL
			float nodes_min_dist,
			NonrigidMatching::NeighborGraphNodesOfPointList *ngns, //could be NULL
			vector<cv::Mat> depthImgs_trg,  //should be ushort16, no shifting
			vector<GCameraView*> cams_d,
			double w_sdf, double w_vis_hull, double w_rot, double w_reg, double mu, double vxl_res
			)
{
	this->surface_src = surface;
	this->depthImgs_trg = depthImgs_trg;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->vxl_res = vxl_res;
	this->mu = mu;

	this->nodes_min_dist_ = nodes_min_dist;
	//setup graph_cuda
	graph_cuda.set_surface_to_deform(*surface);
	graph_cuda.init_ed_from_cpu(graph_in, nodes_min_dist);
	if (graph_in == NULL)
	{
		BoundingBox3D bbox = surface->get_bbox();
		bbox.extend(0.1);
		graph_cuda.sample_ed_nodes(bbox, nodes_min_dist);
	}
	else
	{
		this->graph = graph_in;
	}
	graph_cuda.compute_ngns();

	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, depthImgs_trg, cams_d, vxl_res, true);

	if (graph_in == NULL)
	{
		if (this->graph != NULL)
			delete this->graph;
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}

}


void EDMatchingLSQProblemCuda::
set_problem(CSurface<float> const*surface,
			NonrigidMatching::DeformGraph *graph_in, //could be NULL
			float nodes_min_dist,
			NonrigidMatching::NeighborGraphNodesOfPointList *ngns, //could be NULL
			cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, int depth_num, int depth_width, int depth_height,
			vector<GCameraView*> cams_d,
			double w_sdf, double w_vis_hull, double w_rot, double w_reg, double mu, double vxl_res
			)
{
	this->surface_src = surface;
	this->cams_d = cams_d;
	this->w_sdf = w_sdf;
	this->w_vis_hull = w_vis_hull;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->mu = mu;
	this->vxl_res = vxl_res;
	this->nodes_min_dist_ = nodes_min_dist;
	//setup graph_cuda
	graph_cuda.set_surface_to_deform(*surface);
	graph_cuda.init_ed_from_cpu(graph_in, nodes_min_dist);
	if (graph_in == NULL)
	{
		BoundingBox3D bbox = surface->get_bbox();
		bbox.extend(0.1);
		graph_cuda.sample_ed_nodes(bbox, nodes_min_dist);
	}
	else
	{
		this->graph = graph_in;
	}
	graph_cuda.compute_ngns();

	//setup jtj helper: set up memory, allocate vertices to jtj blocks
	jtj_helper_cuda.init(&graph_cuda, NULL, false, cu_3dArr_depth, cu_3dArr_normal, depth_num, depth_width, depth_height, cams_d, 
		vxl_res, true, NULL);

	if (graph_in == NULL)
	{
		if (this->graph != NULL)
			delete this->graph;
		this->graph = new DeformGraph(nodes_min_dist, EDNODE_NN);
		graph_cuda.readout_DeformGraph(*(this->graph), false);
	}

}

//used only for lm-solver cpu version
bool EDMatchingLSQProblemCuda::
init(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf)
{
	//fill values to the BlockedHessianMatrix. allocate space to hold jtj values
	HessianBlockInfoCuda *jtj_blk_info = NULL;
	int jtj_blk_num_data = 0;
	int jtj_blk_num_all = 0;
	int jtj_para_blk_num = 0;
	jtj_helper_cuda.readout_jtj_block_info(jtj_blk_info, jtj_para_blk_num, jtj_blk_num_data, jtj_blk_num_all);

	jtf.set_size(jtj_para_blk_num * 12);
	jtf.fill(0.0);

	jtj.blk_num = jtj_blk_num_all;
	jtj.para_blk_num = jtj_para_blk_num;
	jtj.blocks = new HessianBlock[jtj_blk_num_all];
	jtj.data = new float[jtj_blk_num_all * 144];
	memset(jtj.data, 0, sizeof(float)* 144 * jtj_blk_num_all);

	for (int i = 0; i < jtj_blk_num_all; i++)
	{
		HessianBlockInfoCuda const& blk_cuda = jtj_blk_info[i];

		HessianBlock &blk = jtj.blocks[i];
		blk.row = blk_cuda.pi_idx;
		blk.col = blk_cuda.pj_idx;
		blk.vtii_first = blk_cuda.vtii_first;
		blk.vts_num = blk_cuda.vts_num;
		blk.para_count = 12;
		blk.bGraphEdge = false;
		blk.vals = jtj.data + blk_cuda.data_offset;
	}

	delete[] jtj_blk_info;
	return true;
}

bool EDMatchingLSQProblemCuda::
evaluate(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf, bool bUseOffDiagJtJData, int frmIdx, int iter)
{
	if (jtj.blk_num == 0 || jtf.size() == 0)
	{
		init(jtj, jtf);
	}

	gpu_evaluate_JtJ_JtF(frmIdx, iter, bUseOffDiagJtJData);

	jtj_helper_cuda.read_out(jtj, jtf);

	return true;
}

// ||F(x)||^2
double EDMatchingLSQProblemCuda::evaluateCurrentCost(int frmIdx, int iter)
{
	return gpu_evaluate_cost(frmIdx, iter);
}


//update state variables
bool EDMatchingLSQProblemCuda::updateX(vnl_vector<float> const&dx)
{
	float *dev_dx = NULL;
	checkCudaErrors(cudaMalloc(&dev_dx, sizeof(float)*dx.size()));

	checkCudaErrors(cudaMemcpy(dev_dx, dx.data_block(), sizeof(float)*dx.size(), cudaMemcpyHostToDevice));

	bool ret = gpu_updateX(dev_dx);

	checkCudaErrors(cudaFree(dev_dx));
	return ret;
}

//return state variables as a vector
bool EDMatchingLSQProblemCuda::backup_currentX()
{
	return gpu_backup_currentX();
}
bool EDMatchingLSQProblemCuda::recover_backupX()
{
	return gpu_recover_backupX();
}

void EDMatchingLSQProblemCuda::
dump_gpu_jtj_jtf(char const* filename_jtj, char const* filename_jtf)
{
	BlockedHessianMatrix jtj;
	vnl_vector<float> jtf;
	this->init(jtj, jtf);
	this->jtj_helper_cuda.read_out(jtj, jtf);

	cv::Mat jtj_mat = jtj.to_dense();
	saveMatrixAscii(filename_jtj, jtj_mat);
	saveVNLVectorASCAII(filename_jtf, jtf);
	jtj_mat.release();
}


bool EDMatchingLSQProblemCuda::
dump_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info)
{
	return graph_cuda.dump_ed_nodes_infos(filename_ed_graph, filename_arr_ndIds, filename_ed_info);
}

bool EDMatchingLSQProblemCuda::
feed_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info)
{
	return graph_cuda.feed_ed_nodes_infos(filename_ed_graph, filename_arr_ndIds, filename_ed_info);
}
