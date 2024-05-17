bool EDMatchingHelperCuda::
read_out(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf)
{
	if (jtj.para_blk_num != jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)))
	{
		printf("Error<EDMatchingHelperCuda::read_out>: data sizes do not agree. para_blk_num<%d vs %d>!\n", 
				jtj.para_blk_num, jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)));
		return false;
	}
	checkCudaErrors(cudaMemcpy(jtj.data, dev_jtj_, sizeof(float) * jtj_ele_count_max_, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(jtf.data_block(), dev_jtf_, sizeof(float) * para_count_max_, cudaMemcpyDeviceToHost));
	return true;
}

bool EDMatchingHelperCuda::
readout_jtj_block_info(HessianBlockInfoCuda* &hessian_block_info, int& para_blk_count, int &blk_count_data, int &blk_count_all)
{
	para_blk_count = this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	blk_count_all = this->jtj_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));

	abort(); // Uncomment this in the init_internal function

	blk_count_data = this->jtj_blks_num_data_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));

	hessian_block_info = new HessianBlockInfoCuda[blk_count_all];
	checkCudaErrors(cudaMemcpy(hessian_block_info, this->dev_jtj_blk_info_buf_, sizeof(HessianBlockInfoCuda)*blk_count_all, cudaMemcpyDeviceToHost));

	return true;
}



bool EDMatchingHelperCuda::
init_with_cpu_result(BlockedHessianMatrix const& jtj,
	 int const*vt_indices, WeightsPair const* node_weights, int vt_indices_count, int vts_num,
	 vector<cv::Mat> depthImgs, vector<GCameraView*> cams_d)
{
	abort();
}

void EDMatchingHelperCuda::
init(DeformGraphCuda *graph_cuda, vector<cv::Mat> depthImgs, vector<GCameraView*> cams_d, float vxl_res, 
	bool bComputeVisualHull,
	char const* debug_dir)
{
	this->graph_cuda_ = graph_cuda;

	if (depthImgs.size() != this->num_depthmaps_)
	{
		LOGGER()->error("EDMatchingHelperCuda::init", "depthImgs.size() != this->num_depthmaps_ <%d vs %d>",
			depthImgs.size(), this->num_depthmaps_);
		exit(0);
	}
	assert(depth_width_ == depthImgs[0].cols);
	assert(depth_height_ == depthImgs[0].rows);
	this->feed_textures(depthImgs);

	LOGGER()->error("EDMatchingHelperCuda::init", "depth normal check is on, but normal maps are not computed!");

	this->init_internal( NULL, false, cams_d, vxl_res, bComputeVisualHull, debug_dir);
}


void EDMatchingHelperCuda::
init(DeformGraphCuda *graph_cuda, BoundingBox3D bbox_visual_hull, //bbox of foreground
	 bool bUseSegmentationForVisualHull,
	 cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal,
	 int depth_num, int depth_width, int depth_height,
	 vector<GCameraView*> cams_d, float vxl_res, 
	 bool bComputeVisualHull,
	 char const* debug_dir)
{
	BoundingBox3DCuda* dev_bbox = NULL;

	checkCudaErrors(cudaMalloc(&dev_bbox, sizeof(BoundingBox3DCuda)));

	BoundingBox3DCuda bbox_visual_hull_cuda;
	bbox_visual_hull_cuda.x_s = bbox_visual_hull.x_s;
	bbox_visual_hull_cuda.x_e = bbox_visual_hull.x_e;
	bbox_visual_hull_cuda.y_s = bbox_visual_hull.y_s;
	bbox_visual_hull_cuda.y_e = bbox_visual_hull.y_e;
	bbox_visual_hull_cuda.z_s = bbox_visual_hull.z_s;
	bbox_visual_hull_cuda.z_e = bbox_visual_hull.z_e;
	checkCudaErrors(cudaMemcpy(dev_bbox, &bbox_visual_hull_cuda, sizeof(BoundingBox3DCuda), cudaMemcpyHostToDevice));
	init(graph_cuda, dev_bbox, bUseSegmentationForVisualHull, 
		cu_3dArr_depth, cu_3dArr_normal, 
		depth_num, depth_width, depth_height, 
		cams_d, vxl_res, 
		bComputeVisualHull,
		debug_dir);

	checkCudaErrors(cudaFree(dev_bbox));
}

void EDMatchingHelperCuda::
init(DeformGraphCuda *graph_cuda, BoundingBox3DCuda const* dev_bbox_visual_hull, //bbox of foreground
	 bool bUseSegmentationForVisualHull,
	 cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, 
	 int depth_num, int depth_width, int depth_height, 
	 vector<GCameraView*> cams_d, float vxl_res, 
	 bool bComputeVisualHull,
	 char const* debug_dir)
{
	this->graph_cuda_ = graph_cuda;


	this->depth_width_ = depth_width;
	this->depth_height_ = depth_height;
	this->num_depthmaps_ = depth_num;
	if (cu_3dArr_depth != NULL)
		this->bind_cuda_array_to_texture_depth(cu_3dArr_depth);
	if (depth_num != DEPTH_CAMERAS_NUM)
	{
		LOGGER()->error("EDMatchingHelperCuda::init", "depth_num != DEPTH_CAMERAS_NUM <%d vs %d>", depth_num, DEPTH_CAMERAS_NUM);
	}

	if (cu_3dArr_normal == NULL)
		LOGGER()->error("EDMatchingHelperCuda::init", "depth normal check is on, but normal maps are not computed!");
	else
		this->bind_cuda_array_to_texture_normal(cu_3dArr_normal);

	this->init_internal(dev_bbox_visual_hull, bUseSegmentationForVisualHull, cams_d, vxl_res, bComputeVisualHull, debug_dir);
}


void EDMatchingHelperCuda::
compute_visual_hull(BoundingBox3DCuda const* dev_bbox, bool bUseSegmentation, int blurIters)
{
	compute_visual_hull_occupancy(bUseSegmentation);

	if (blurIters > 0)
		blur_visual_hull_occupancy(blurIters);
}

bool EDMatchingHelperCuda::
init_internal(BoundingBox3DCuda const* dev_bbox_visual_hull, //bbox of foreground
			  bool bUseSegmentationForVisualHull,
			  vector<GCameraView*> cams_d,
			  float vxl_res, 
			  bool bComputeVisualHull,
			  char const* debug_dir)
{
	//set up texture and depth camera poses
	if (cams_d.size() > 0)
		this->setup_cameras(cams_d);
	this->vxl_res_ = vxl_res;

	//set up visual hull
	float const visual_hull_res = 1.0f;//0.8f; /
	setup_visual_hull_info(dev_bbox_visual_hull, visual_hull_res);
	if (bComputeVisualHull)
		compute_visual_hull(dev_bbox_visual_hull, bUseSegmentationForVisualHull, 3);

	cuda::gpu_size_data vts_num_gpu = graph_cuda_->vts_num_gpu();

	/////////////////////////////////////////////////////////////////////
	///  CUDA Kernel to SETUP the JTJ Structure for data term
	/////////////////////////////////////////////////////////////////////
	//allocate memory

	setup_vt_data_for_jtj_vHierED(graph_cuda_->dev_ed_nodes(), graph_cuda_->ed_nodes_num_gpu(), graph_cuda_->dev_ed_nodes_num_all_levels(), 
								  graph_cuda_->ngns_indices_device_memory(), graph_cuda_->ngns_weights_device_memory(),
									graph_cuda_->vts_num_gpu(), 6);


	this->jtj_para_blks_num_ = graph_cuda_->ed_nodes_num_all_levels();
	this->para_count_max_ = jtj_para_blks_num_.max_size * 12;

	const bool bDumpInitData = false;
	//debug: readout
	if (debug_dir && bDumpInitData)
	{
		vnl_matrix<int> jtj_2dInfo_offset;
		vnl_matrix<int> jtj_2dInfo_flag;
		this->readout_Jtj2DInfos(jtj_2dInfo_offset, jtj_2dInfo_flag);
		char name[500];
		sprintf(name, "%s/jtj_2d_info_offset.txt", debug_dir);
		saveVNLMatrixASCAII(name, jtj_2dInfo_offset);
		sprintf(name, "%s/jtj_2d_info_flag.txt", debug_dir);
		saveVNLMatrixASCAII(name, jtj_2dInfo_flag);
	}

	//reg terms: 
	this->bind_tex_ndId(graph_cuda_->cu_3dArr_ndIds());

	setup_ed_nodes_reg_term_vHierED(graph_cuda_->dev_ed_nodes(), graph_cuda_->dev_ed_nodes_num_all_levels(),
		graph_cuda_->dev_ed_nodes_ranges(), graph_cuda_->ed_hierarchy_levels(),
		graph_cuda_->dev_ed_cubes_dim(), graph_cuda_->dev_ed_cubes_offset(), graph_cuda_->ed_cubes_res(), graph_cuda_->nodes_min_dist());

	if (debug_dir && bDumpInitData)
	{
		//debug
		vnl_matrix<int> jtj_2dInfo_offset;
		vnl_matrix<int> jtj_2dInfo_flag;
		this->readout_Jtj2DInfos(jtj_2dInfo_offset, jtj_2dInfo_flag);
		char name[500];
		sprintf(name, "%s/jtj_2d_info_offset2.txt", debug_dir);
		saveVNLMatrixASCAII(name, jtj_2dInfo_offset);
		sprintf(name, "%s/jtj_2d_info_flag2.txt", debug_dir);
		saveVNLMatrixASCAII(name, jtj_2dInfo_flag);

		vnl_matrix<int> node_pairs;
		this->readout_reg_node_pairs(node_pairs);
		sprintf(name, "%s/node_pairs.txt", debug_dir);
		saveVNLMatrixASCAII(name, node_pairs);
	}

	//allocate memory for dev_jtj and dev_jtf
	this->jtj_blks_num_ = jtj_blks_count_gpu_; // SYNCPOINT
	this->jtj_ele_count_max_ = jtj_blks_count_gpu_.max_size * 144;

	return true;
}

bool EDMatchingHelperCuda::
save_HessianBlockInfoCuda_buf_ASCII(char const* file_name, HessianBlockInfoCuda const* hessian_block_info, int count)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if (!fp)
	{
		printf("Error<EDMatchingHelperCuda::save_HessianBlockInfoCuda_buf_ASCII>: Cannot open the file <%s>.\n", file_name);
		return false;
	}

	fprintf(fp, "HessianBlock Info.\n");
	fprintf(fp, "blk num: %d\n\n", count);

	for (int i = 0; i < count; i++)
	{
		fprintf(fp, "blk %d:\n", i);
		fprintf(fp, "pi_idx = %d\n", hessian_block_info[i].pi_idx);
		fprintf(fp, "pj_idx = %d\n", hessian_block_info[i].pj_idx);

		fprintf(fp, "pi_dim = %d\n", hessian_block_info[i].pi_dim);
		fprintf(fp, "pj_dim = %d\n", hessian_block_info[i].pj_dim);

		fprintf(fp, "vtii_first = %d\n", hessian_block_info[i].vtii_first);
		fprintf(fp, "vts_num = %d\n", hessian_block_info[i].vts_num);

		fprintf(fp, "data_offset = %d\n", hessian_block_info[i].data_offset);

		fprintf(fp, "\n\n");
	}

	fclose(fp);
	return true;
}


bool EDMatchingHelperCuda::
readout_vt_indices_weights_for_jtj(vnl_vector<int> &vt_indices_for_jtj, vnl_vector<float> &vt_weights_for_jtj)
{
	int vts_num = graph_cuda_->vts_num_gpu().sync_read();
	vt_indices_for_jtj.set_size(JTJ_BLKS_NUM_INDUCED_PER_VERTEX * vts_num);
	vt_weights_for_jtj.set_size(JTJ_BLKS_NUM_INDUCED_PER_VERTEX * vts_num);

	checkCudaErrors(cudaMemcpy(vt_indices_for_jtj.data_block(), dev_vt_indices_for_jtj_, sizeof(int)* JTJ_BLKS_NUM_INDUCED_PER_VERTEX * vts_num, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(vt_weights_for_jtj.data_block(), dev_vt_weights_for_jtj_, sizeof(int)* JTJ_BLKS_NUM_INDUCED_PER_VERTEX * vts_num, cudaMemcpyDeviceToHost));
	return true;
}


bool EDMatchingHelperCuda::dump_debug_information(HessianBlockInfoCuda const*hessian_block_info, int blk_count, int ed_node_num,
	int const*vt_indices_for_jtj, float const*vt_weights_for_jtj, int vt_indices_for_jtj_count, char const* dir_debug)
{
	char name[500];
	sprintf(name, "%s/hessian_blk_info.txt", dir_debug);
	save_HessianBlockInfoCuda_buf_ASCII(name, hessian_block_info, blk_count);

	vnl_matrix<int> countMat;
	HessianBlockInfoCudaBuf_to_PtsCountMat(hessian_block_info, blk_count, countMat, ed_node_num);
	sprintf(name, "%s/hessian_vt_countMat.txt", dir_debug);
	saveVNLMatrixASCAII(name, countMat);

	vector<int> indices_all_ordered;

	vnl_matrix<vector<int>*> vtis_mat(ed_node_num, ed_node_num);
	vnl_matrix<vector<float>*> ws_mat(ed_node_num, ed_node_num);
	vtis_mat.fill(NULL);
	ws_mat.fill(NULL);
	for (int i = 0; i < blk_count; i++)
	{
		int vtii_first = hessian_block_info[i].vtii_first;
		int vtii_num = hessian_block_info[i].vts_num;
		if (vtii_num == 0)
			continue;
		vector<int>* vtis = new vector<int>(vt_indices_for_jtj + vtii_first, vt_indices_for_jtj + vtii_first + vtii_num);

		vector<float>* ws = new vector<float>(vt_weights_for_jtj + vtii_first, vt_weights_for_jtj + vtii_first + vtii_num);

		int pi = hessian_block_info[i].pi_idx;
		int pj = hessian_block_info[i].pj_idx;
		vtis_mat(pi, pj) = vtis;
		ws_mat(pi, pj) = ws;
	}
	vector<vector<int>*> vtis_list;
	vector<vector<float>*> ws_list;
	for (int i = 0; i < vtis_mat.rows(); i++)
	for (int j = 0; j < vtis_mat.cols(); j++)
	{
		if (i != j) continue;
		if (vtis_mat[i][j])
			vtis_list.push_back(vtis_mat[i][j]);
		if (ws_mat[i][j])
			ws_list.push_back(ws_mat[i][j]);
	}
	sprintf(name, "%s/vt_indices_jtj_list.txt", dir_debug);
	saveVectorList(name, vtis_list);
	sprintf(name, "%s/vt_weights_jtj_list.txt", dir_debug);
	saveVectorList(name, ws_list);

	return true;
}

void EDMatchingHelperCuda::dump_gpu_data(char const* dir_debug)
{
	char name[500];

	HessianBlockInfoCuda *hessian_blk_info = NULL;
	int hessian_blk_count_data = 0;
	int hessian_blk_count = 0;
	int diag_blk_count = 0;
	this->readout_jtj_block_info(hessian_blk_info, diag_blk_count, hessian_blk_count_data, hessian_blk_count);

	vnl_vector<int> vt_indices_for_jtj;
	vnl_vector<float> vt_weights_for_jtj;
	readout_vt_indices_weights_for_jtj(vt_indices_for_jtj, vt_weights_for_jtj);

	sprintf(name, "%s/vt_indices_for_jtj.txt", dir_debug);
	saveVNLVectorASCAII(name, vt_indices_for_jtj);

	sprintf(name, "%s/vt_weights_for_jtj.txt", dir_debug);
	saveVNLVectorASCAII(name, vt_weights_for_jtj);

	this->dump_debug_information(hessian_blk_info, hessian_blk_count, diag_blk_count, 
		vt_indices_for_jtj.data_block(), vt_weights_for_jtj.data_block(), vt_indices_for_jtj.size(), 
		dir_debug);

	delete[] hessian_blk_info;
}

bool EDMatchingHelperCuda::
HessianBlockInfoCudaBuf_to_PtsCountMat(HessianBlockInfoCuda const* hessian_block_info, int count, vnl_matrix<int> &countMat, int ed_node_num)
{
	countMat.set_size(ed_node_num, ed_node_num);
	countMat.fill(0.0);

	for (int i = 0; i < count; i++)
	{
		int pi = hessian_block_info[i].pi_idx;
		int pj = hessian_block_info[i].pj_idx;
		countMat(pi, pj) = hessian_block_info[i].vts_num;
	}
	return true;
}


void EDMatchingHelperCuda::readout_visual_hull_occupancy(VoxelMatrix<float> &visual_hull_occupancy)
{
	int3 visual_hull_dim;
	float3 visual_hull_offset;
	checkCudaErrors(cudaMemcpy(&visual_hull_dim, dev_visual_hull_dim_, sizeof(int3), cudaMemcpyDeviceToHost));
	visual_hull_occupancy.set_size(visual_hull_dim.x, visual_hull_dim.y, visual_hull_dim.z);
	
	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcArray = cu_3DArr_vishull_;
	paras.dstPtr = make_cudaPitchedPtr(visual_hull_occupancy.data_block(), visual_hull_occupancy.ni()*sizeof(float), visual_hull_occupancy.ni(), visual_hull_occupancy.nj());
	paras.extent = make_cudaExtent(visual_hull_dim.x, visual_hull_dim.y, visual_hull_dim.z);
	paras.kind = cudaMemcpyDeviceToHost;

	checkCudaErrors(cudaMemcpy3D(&paras));
}

void EDMatchingHelperCuda::readout_visual_hull_offset(vnl_vector_fixed<float, 3> &offset)
{
	checkCudaErrors(cudaMemcpy(offset.data_block(), dev_visual_hull_offset_, sizeof(float3), cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCuda::dump_visual_hull(char const* dir, int idx)
{
	VoxelMatrix<float> vis_hull;
	this->readout_visual_hull_occupancy(vis_hull);
	char name[500];
	sprintf(name, "%s/vis_hull_%d_dim(%d_%d_%d).raw", dir, idx, vis_hull.ni(), vis_hull.nj(), vis_hull.nk());
	vis_hull.save_to_rawbin(name);
	sprintf(name, "%s/vis_hull_%d_dim(%d_%d_%d).txt", dir, idx, vis_hull.ni(), vis_hull.nj(), vis_hull.nk());
	vis_hull.save_to_file_ascii(name);
}

void EDMatchingHelperCuda::readout_depth_mats_proj(vector<cv::Mat> &depthMats_proj, bool bReSize)
{
	int *p_data = new int[depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM];
	checkCudaErrors(cudaMemcpy(p_data, dev_depth_mats_proj_, sizeof(int)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));

	depthMats_proj.clear();
	for (int k = 0; k < DEPTH_CAMERAS_NUM; k++)
	{
		cv::Mat depthMat = cv::Mat(depth_height_proj_, depth_width_proj_, CV_64F);	
		for (int i = 0; i < depth_height_proj_; i++)
		for (int j = 0; j < depth_width_proj_; j++)
		{
			int val = p_data[k*depth_width_proj_*depth_height_proj_ + i*depth_width_proj_ + j];
			depthMat.at<double>(i, j) = val == 8000 ? 0.0f : val/10.0f;
		}
		if (bReSize)
		{
			cv::Mat depthMat_ori_size = cv::Mat(depth_height_, depth_width_, CV_64F);
			cv::resize(depthMat, depthMat_ori_size, depthMat_ori_size.size(), 0, 0, cv::INTER_NEAREST);
			depthMat.release();
			depthMats_proj.push_back(depthMat_ori_size);
		}
		else
		{
			depthMats_proj.push_back(depthMat);
		}
	}
	delete[] p_data;
}

void EDMatchingHelperCuda::readout_depth_mats_align_residual(vector<vnl_matrix<float>> &residual_mats, bool bReadFiltered)
{
	float *p_data = new float[depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM];
	if (bReadFiltered)
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_f_, sizeof(float)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));
	else
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_, sizeof(float)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));

	residual_mats.clear();
	residual_mats.resize(DEPTH_CAMERAS_NUM);
	for (int k = 0; k < DEPTH_CAMERAS_NUM; k++)
	{
		residual_mats[k].set_size(depth_height_proj_, depth_width_proj_);
		for (int i = 0; i < depth_height_proj_; i++)
		for (int j = 0; j < depth_width_proj_; j++)
		{
			float val = p_data[k*depth_width_proj_*depth_height_proj_ + i*depth_width_proj_ + j];
			residual_mats[k][i][j] = val;
		}
	}

	delete[] p_data;
}


void EDMatchingHelperCuda::readout_depth_mats_corr_vtIdx(vector<vnl_matrix<int>> &depth_mat_corr_vtIdx)
{
	int *p_data = new int[depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM];
	checkCudaErrors(cudaMemcpy(p_data, dev_depth_mats_corr_vtIdx_, sizeof(int)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));

	depth_mat_corr_vtIdx.resize(DEPTH_CAMERAS_NUM);
	for (int k = 0; k < DEPTH_CAMERAS_NUM; k++)
	{
		depth_mat_corr_vtIdx[k].set_size(depth_height_proj_, depth_width_proj_);
		for (int i = 0; i < depth_height_proj_; i++)
		for (int j = 0; j < depth_width_proj_; j++)
		{
			int val = p_data[k*depth_width_proj_*depth_height_proj_ + i*depth_width_proj_ + j];
			depth_mat_corr_vtIdx[k][i][j] = val;
		}
	}

	delete[] p_data;
}

// read out the mean residual error from all 8 depth images, 1.0 corresponds to 15 mm according to [Dou et al. 2016]
void EDMatchingHelperCuda::readout_depth_residual_error(float &mean_error, bool bReadFiltered)
{
	float *p_data = new float[depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM];
	if (bReadFiltered)
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_f_, sizeof(float)*depth_height_proj_*depth_width_proj_ * DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));
	else
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_, sizeof(float)*depth_height_proj_*depth_width_proj_ * DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));

	mean_error = 0;
	float total_valid_depth = 1.0;

	for (int k = 0; k < DEPTH_CAMERAS_NUM; ++k)
	{
		for (int i = 0; i < depth_height_proj_; ++i)
			for (int j = 0; j < depth_width_proj_; ++j)
			{
				float val = p_data[k * depth_width_proj_ * depth_height_proj_ + i * depth_width_proj_ + j];
				if (val >= 0)
				{
					mean_error += val;
					total_valid_depth += 1.0;
				}
			}
	}
	mean_error /= total_valid_depth;
	delete[] p_data;
}

void EDMatchingHelperCuda::readout_depth_mats_align_residual(vector<cv::Mat> &residual_imgs, bool bReadFiltered)
{
	float *p_data = new float[depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM];
	if (bReadFiltered)
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_f_, sizeof(float)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));
	else
		checkCudaErrors(cudaMemcpy(p_data, dev_depth_align_residual_, sizeof(float)*depth_height_proj_*depth_width_proj_* DEPTH_CAMERAS_NUM, cudaMemcpyDeviceToHost));

	releaseCvMats(residual_imgs);
	for (int k = 0; k < DEPTH_CAMERAS_NUM; k++)
	{
		cv::Mat img = cv::Mat(depth_height_proj_, depth_width_proj_, CV_MAKETYPE(8, 3));
		for (int i = 0; i < depth_height_proj_; i++)
		for (int j = 0; j < depth_width_proj_; j++)
		{
			float val = p_data[k*depth_width_proj_*depth_height_proj_ + i*depth_width_proj_ + j];
			if (val < 0)
			{
				img.at<cv::Vec3b>(i, j)[0] = 0;
				img.at<cv::Vec3b>(i, j)[1] = 0;
				img.at<cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{
				int idx = MIN(255, MAX(0, ROUND(val*255.0)));
				img.at<cv::Vec3b>(i, j)[2] = CLRMAP_JET[idx][0]*255;
				img.at<cv::Vec3b>(i, j)[1] = CLRMAP_JET[idx][1]*255;
				img.at<cv::Vec3b>(i, j)[0] = CLRMAP_JET[idx][2]*255;
			}
		}
		residual_imgs.push_back(img);
	}

	delete[] p_data;
}

void EDMatchingHelperCuda::readout_per_vertex_align_residual(vnl_vector<float> &per_vertex_align_residual)
{
	int vts_num = this->graph_cuda_->vts_num();
	per_vertex_align_residual.set_size(vts_num);

	checkCudaErrors(cudaMemcpy(per_vertex_align_residual.data_block(), this->dev_per_vertex_align_residual_, sizeof(float)*vts_num, cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCuda::readout_per_vertex_align_residual_interp(vnl_vector<float> &per_vertex_align_residual)
{
	int vts_num = this->graph_cuda_->vts_num();
	per_vertex_align_residual.set_size(vts_num);

	checkCudaErrors(cudaMemcpy(per_vertex_align_residual.data_block(), this->dev_per_vertex_align_residual_ed_interp_, sizeof(float)*vts_num, cudaMemcpyDeviceToHost));
}
void EDMatchingHelperCuda::readout_ed_nodes_align_residual(vnl_vector<float> &ed_nodes_align_residual)
{
	int ed_nodes_num = this->graph_cuda_->ed_nodes_num();
	ed_nodes_align_residual.set_size(ed_nodes_num);
	checkCudaErrors(cudaMemcpy(ed_nodes_align_residual.data_block(), this->dev_ed_nodes_align_residual_, sizeof(float)*ed_nodes_num, cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCuda::readout_cam_vis(vnl_vector<ushort2> &cam_vis)//surface point visibility
{
	int vts_num = this->graph_cuda_->surface_vt_num();

	cam_vis.set_size(vts_num);
	checkCudaErrors(cudaMemcpy(cam_vis.data_block(), this->dev_cam_vis_, sizeof(ushort2)*vts_num, cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCuda::
feed_keypts(S3DPointMatchSet const&matches)
{
	int keypts_num = matches.size();
	float3 *keypts_p = new float3[keypts_num];
	float3 *keypts_q = new float3[keypts_num];
	for (int i = 0; i < keypts_num; i++)
	{
		keypts_p[i].x = matches.points_1[i][0];
		keypts_p[i].y = matches.points_1[i][1];
		keypts_p[i].z = matches.points_1[i][2];
		keypts_q[i].x = matches.points_2[i][0];
		keypts_q[i].y = matches.points_2[i][1];
		keypts_q[i].z = matches.points_2[i][2];
	}

	//TODO: make async
	checkCudaErrors(cudaMemcpy(dev_keypts_p_, keypts_p, sizeof(float3)*keypts_num, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(dev_keypts_q_, keypts_q, sizeof(float3)*keypts_num, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(keypts_num_gpu_.dev_ptr, &keypts_num, sizeof(int), cudaMemcpyHostToDevice));
	delete[] keypts_p;
	delete[] keypts_q;
}

void EDMatchingHelperCuda::
readout_keypts_3d(S3DPointMatchSet &matches)
{
	int keypts_num = 0;
	checkCudaErrors(cudaMemcpy(&keypts_num, keypts_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
	
	float3 *keypts_p = new float3[keypts_num];
	float3 *keypts_q = new float3[keypts_num];
	checkCudaErrors(cudaMemcpy(keypts_p, dev_keypts_p_, sizeof(float3)*keypts_num, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(keypts_q, dev_keypts_q_, sizeof(float3)*keypts_num, cudaMemcpyDeviceToHost));

	matches.clear();
	for (int i = 0; i < keypts_num; i++)
	{
		vnl_vector_fixed<double, 3> p(keypts_p[i].x, keypts_p[i].y, keypts_p[i].z);
		vnl_vector_fixed<double, 3> q(keypts_q[i].x, keypts_q[i].y, keypts_q[i].z);
		matches.push_back(p, q);
	}

	delete[] keypts_p;
	delete[] keypts_q;
}


void EDMatchingHelperCuda::
feed_keypts_2d(std::vector<S2DPointMatchSet*> matches_all)
{
	if (matches_all.size() != DEPTH_CAMERAS_NUM)
	{
		LOGGER()->error("EDMatchingHelperCuda::feed_keypts_2d", "matches_all.size() != DEPTH_CAMERAS_NUM <%d vs %d>", matches_all.size(), DEPTH_CAMERAS_NUM);
		return;
	}
	int keypts_num = 0;
	for (int i = 0; i < matches_all.size(); i++)
		keypts_num += matches_all[i]->matches.rows();


	ushort3 *keypts_2d_p = new ushort3[keypts_num];
	ushort3 *keypts_2d_q = new ushort3[keypts_num];

	int count = 0;
	for (int i = 0; i < matches_all.size(); i++)
	{
		for (int j = 0; j < matches_all[i]->matches.rows(); j++)
		{
			ushort3 p, q;
			p.x = i;
			p.y = ROUND(matches_all[i]->matches(j, 0));
			p.z = ROUND(matches_all[i]->matches(j, 1));
			keypts_2d_p[count] = p;

			q.x = i;
			q.y = ROUND(matches_all[i]->matches(j, 2));
			q.z = ROUND(matches_all[i]->matches(j, 3));
			keypts_2d_q[count] = q;
			count++;
		}
	}

	if (keypts_num > keypts_num_gpu_.max_size)
	{
		LOGGER()->warning("feed_keypts_2d","keypts_num > max_size <%d vs %d>", keypts_num, keypts_num_gpu_.max_size);
		keypts_num = keypts_num_gpu_.max_size;
	}

	//TODO: make async
	checkCudaErrors(cudaMemcpy(keypts_num_gpu_.dev_ptr, &keypts_num, sizeof(int), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(dev_keypts_2d_p_, keypts_2d_p, sizeof(ushort3)*keypts_num, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(dev_keypts_2d_q_, keypts_2d_q, sizeof(ushort3)*keypts_num, cudaMemcpyHostToDevice));

	delete[] keypts_2d_p;
	delete[] keypts_2d_q;
}






