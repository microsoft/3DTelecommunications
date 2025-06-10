// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
void DeformGraphCuda::
allocate_memory(int vts_num_max, int vt_dim)
{
	checkCudaErrors(cudaMalloc(&dev_vts_t_, sizeof(float)*vts_num_max*vt_dim));
	checkCudaErrors(cudaMalloc(&dev_ngns_indices_, sizeof(int)*vts_num_max*NEIGHBOR_EDNODE_NUM));
	checkCudaErrors(cudaMalloc(&dev_ngns_weights_, sizeof(float)*vts_num_max*NEIGHBOR_EDNODE_NUM));
	vts_num_gpu_.allocate_once();
	vts_num_gpu_.max_size = vts_num_max;
	this->vt_dim_ = vt_dim;
}

//NOTE: only copy the pointer for dev_vts, not copy the data over
bool DeformGraphCuda::
set_surface_to_deform(float* dev_vts, int stride, cuda::gpu_size_data vts_num)
{
	this->dev_vts_ = (float*)dev_vts;
	if (vt_dim_ != stride)
	{
		LOGGER()->warning("DeformGraphCuda::set_surface_to_deform","t_dim does not agree<%d vs %d>", vt_dim_, stride);
	}

	checkCudaErrors(cudaMemcpyAsync(this->vts_num_gpu_.dev_ptr, vts_num.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	//TODO: not necessary not. only useful later when color is included
	checkCudaErrors(cudaMemcpyAsync(dev_vts_t_, dev_vts_, sizeof(float)*vt_dim_*vts_num_gpu_.max_size, cudaMemcpyDeviceToDevice));

	return true;
}

bool DeformGraphCuda::
set_surface_to_deform(float* dev_vts, int stride, int vts_num)
{
	this->dev_vts_ = (float*)dev_vts;

	if (vt_dim_ != stride)
	{
		LOGGER()->warning("DeformGraphCuda::set_surface_to_deform","t_dim does not agree<%d vs %d>", vt_dim_, stride);
	}

	this->vts_num_gpu_.max_size = vts_num;
	static cuda::PinnedMemory<int> vts_num_pinned;
	vts_num_pinned.memory[0] = vts_num;
	checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_.dev_ptr, vts_num_pinned.memory, sizeof(int), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpyAsync(dev_vts_t_, dev_vts_, sizeof(float)*vt_dim_*vts_num, cudaMemcpyDeviceToDevice));
	return true;
}


bool DeformGraphCuda::
set_surface_to_deform(CSurface<float> const&surface)
{
	if (surface.vtNum == 0)
	{
		LOGGER()->error("DeformGraphCuda::set_surface_to_deform","surface.vtNum = 0!");
		return false;
	}

	this->vts_num_gpu_.max_size = surface.vtNum;
	static cuda::PinnedMemory<int> vts_num_pinned;
	vts_num_pinned.memory[0] = surface.vtNum;
	checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_.dev_ptr, vts_num_pinned.memory, sizeof(int), cudaMemcpyHostToDevice));

	vt_dim_ = surface.vtDim;

	if (dev_vts_ == NULL)
		checkCudaErrors(cudaMalloc(&dev_vts_, sizeof(float)*surface.vtNum*vt_dim_));

	//copy: host --> device
	checkCudaErrors(cudaMemcpy(dev_vts_, surface.vtData, sizeof(float)*surface.vtNum*vt_dim_, cudaMemcpyHostToDevice));
	return true;
}

bool DeformGraphCuda::read_out_surface_t(CSurface<float> &surface_t)
{
	int vts_num = vts_num_gpu_.sync_read();

	if (surface_t.vtDim != vt_dim_ ||
		surface_t.vtNum != vts_num)
	{
		if (surface_t.vtData != NULL)
			delete[] surface_t.vtData;
		surface_t.vtDim = vt_dim_;
		surface_t.vtNum = vts_num;
		surface_t.vtData = new float[vt_dim_*vts_num];
		surface_t.color = vt_dim_ == 9;
		surface_t.normal = vt_dim_ >= 6;
	}
	//copy: device-->host
	checkCudaErrors(cudaMemcpy(surface_t.vtData, dev_vts_t_, sizeof(float)*vts_num*vt_dim_, cudaMemcpyDeviceToHost));

	return true;
}


bool DeformGraphCuda::
feed_NeighborGraphNodes(NeighborGraphNodesOfPointList const& ngns)
{
	int ngns_num = ngns.size();
	int vts_num = vts_num_gpu_.sync_read();
	if (ngns_num != vts_num)
	{
		LOGGER()->error("DeformGraphCuda::feed_NeighborGraphNodes", "ngns_num != vts_num(%d vs %d)", ngns_num, vts_num);
		return false;
	}

	__tic__();
	//allocate host memory

	int *host_ngns_indices = new int[NEIGHBOR_EDNODE_NUM * ngns_num];
	float *host_ngns_weights = new float[NEIGHBOR_EDNODE_NUM * ngns_num];

	for (int i = 0; i < ngns_num; i++)
	{
		NeighborGraphNodesOfPoint const& ngn_src = ngns[i];
		int k = 0;
		for (; k < MIN(NEIGHBOR_EDNODE_NUM, ngn_src.neighborIndices.size()); k++)
		{
			host_ngns_indices[NEIGHBOR_EDNODE_NUM*i + k] = ngn_src.neighborIndices[k];
			host_ngns_weights[NEIGHBOR_EDNODE_NUM*i + k] = (float)ngn_src.weights[k];
		}

		if (ngn_src.neighborIndices.size() > NEIGHBOR_EDNODE_NUM)
		{
			double w_sum = 0.0;
			for (int c = 0; c < NEIGHBOR_EDNODE_NUM; c++)
				w_sum += host_ngns_weights[NEIGHBOR_EDNODE_NUM*i + c];
			for (int c = 0; c < NEIGHBOR_EDNODE_NUM; c++)
				host_ngns_weights[NEIGHBOR_EDNODE_NUM*i + c] /= w_sum;
		}
		else
		{
			for (; k < NEIGHBOR_EDNODE_NUM; k++)
			{
				host_ngns_indices[NEIGHBOR_EDNODE_NUM*i + k] = -1;
				host_ngns_weights[NEIGHBOR_EDNODE_NUM*i + k] = 0.0;
			}
		}
	}
	LOGGER()->info("ngns host memory: %f\n", __toc__());

	//copy: host --> device
	checkCudaErrors(cudaMemcpy(dev_ngns_indices_, host_ngns_indices, sizeof(int)*ngns_num*NEIGHBOR_EDNODE_NUM, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(dev_ngns_weights_, host_ngns_weights, sizeof(float)*ngns_num*NEIGHBOR_EDNODE_NUM, cudaMemcpyHostToDevice));

	delete[] host_ngns_indices;
	delete[] host_ngns_weights;
	return true;
}

bool DeformGraphCuda::
read_out_NeighborGraphNodes(NonrigidMatching::NeighborGraphNodesOfPointList &ngns)
{
	ngns.clear();
	int vts_num = vts_num_gpu_.sync_read();
	ngns.resize(vts_num);

	int *host_ngns_indices = new int[NEIGHBOR_EDNODE_NUM * vts_num];
	float *host_ngns_weights = new float[NEIGHBOR_EDNODE_NUM * vts_num];
	checkCudaErrors(cudaMemcpy(host_ngns_indices, dev_ngns_indices_, sizeof(int)*vts_num * NEIGHBOR_EDNODE_NUM, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(host_ngns_weights, dev_ngns_weights_, sizeof(float)*vts_num * NEIGHBOR_EDNODE_NUM, cudaMemcpyDeviceToHost));

	for (int i = 0; i < vts_num; i++)
	{
		ngns[i].neighborIndices.resize(NEIGHBOR_EDNODE_NUM);
		ngns[i].weights.resize(NEIGHBOR_EDNODE_NUM);
		for (int k = 0; k < NEIGHBOR_EDNODE_NUM; k++)
		{
			ngns[i].neighborIndices[k] = host_ngns_indices[NEIGHBOR_EDNODE_NUM * i + k];
			ngns[i].weights[k] = host_ngns_weights[NEIGHBOR_EDNODE_NUM * i + k];
		}
	}
	delete[] host_ngns_indices;
	delete[] host_ngns_weights;
	return true;
}


bool DeformGraphCuda::
init_ed_from_cpu(DeformGraph const* graph, float nodes_min_dist)
{
	this->ed_cubes_res_ = nodes_min_dist;

	bool ret = true;
	if (graph)
	{
		assert(graph->nodes.size() <= ED_NODES_NUM_MAX);
		nodes_num_gpu_.max_size = graph->nodes.size();
		static cuda::PinnedMemory<int> nodes_num_pinned(graph->nodes.size());
		checkCudaErrors(cudaMemcpy(nodes_num_gpu_.dev_ptr, nodes_num_pinned.memory, sizeof(int), cudaMemcpyHostToDevice));

		__tic__();
		ret = update(*graph, nodes_min_dist);
		LOGGER()->info("Update graph = %f\n", __toc__());
	}
	return ret;
}

bool  DeformGraphCuda::
feed_graph_hier(DeformGraphHierachy const& graph_hier)
{
	if (graph_hier.ed_nodes_num_all_levels() == 0)
		return false;

	if (this->ed_nodes_num_all_levels().debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)) != graph_hier.ed_nodes_num_all_levels())
	{
		LOGGER()->warning("DeformGraphCuda::update","nodes size does not agree <%d v.s. %d>",
			this->ed_nodes_num_all_levels(), graph_hier.ed_nodes_num_all_levels());
	}

	int ed_nodes_num_all_levels = graph_hier.ed_nodes_num_all_levels();
	int ed_ndoes_num_1st_level = graph_hier.ed_nodes_num_1st_level();

	checkCudaErrors(cudaMemcpy(dev_ed_nodes_num_all_levels_, &ed_nodes_num_all_levels, sizeof(int), cudaMemcpyHostToDevice));

	nodes_num_gpu_.max_size = ED_NODES_NUM_MAX;
	checkCudaErrors(cudaMemcpy(nodes_num_gpu_.dev_ptr, &ed_ndoes_num_1st_level, sizeof(int), cudaMemcpyHostToDevice));

	this->ed_hierachy_levels_ = graph_hier.nodes_range.size();
	for (int i = 0; i < graph_hier.nodes_range.size(); i++)
		checkCudaErrors(cudaMemcpy(dev_ed_nodes_ranges_[i], &(graph_hier.nodes_range[i]), sizeof(int2), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpy(dev_ed_nodes_buf_, graph_hier.nodes, sizeof(DeformGraphNodeCuda)*ed_nodes_num_all_levels, cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpy(dev_rigid_transf_, &(graph_hier.global_rigid), sizeof(RigidTransformCuda), cudaMemcpyHostToDevice));
}

bool DeformGraphCuda::
update(DeformGraph const& graph, float nodes_min_dist)
{
	int nodes_num = nodes_num_gpu_.sync_read();
	if (graph.nodes.size() != nodes_num)
	{
		LOGGER()->warning("DeformGraphCuda::update","nodes size does not agree <%d v.s. %d>",
			graph.nodes.size(), nodes_num);
		nodes_num_gpu_.max_size = graph.nodes.size();
		static cuda::PinnedMemory<int> nodes_num_pinned(graph.nodes.size());
		checkCudaErrors(cudaMemcpy(nodes_num_gpu_.dev_ptr, nodes_num_pinned.memory, sizeof(int), cudaMemcpyHostToDevice));
		checkCudaErrors(cudaMemcpy(dev_ed_nodes_num_all_levels_, nodes_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));

		nodes_num = graph.nodes.size();
	}

	this->ed_cubes_res_ = nodes_min_dist;

	//allocate host memory for ed graph
	DeformGraphNodeCuda *host_ed_nodes = new DeformGraphNodeCuda[nodes_num];

	for (int i = 0; i < nodes_num; i++)
	{
		DeformGraphNode const& nd_src = graph.nodes[i];
		DeformGraphNodeCuda &nd_dst = host_ed_nodes[i];


		vnl_matrix_fixed<double, 3, 3> A_inv_t(0.0);
		if (nd_src.A != vnl_matrix_fixed<double, 3, 3>(0.0))
			A_inv_t = vnl_inverse(nd_src.A).transpose();
		for (int m = 0; m < 3; m++)
			for (int n = 0; n < 3; n++)
			{
				nd_dst.A[m][n] = nd_src.A[m][n];
				nd_dst.A_inv_t[m][n] = A_inv_t[m][n];
			}
		for (int m = 0; m < 3; m++)
		{
			nd_dst.g[m] = nd_src.g[m];
			nd_dst.t[m] = nd_src.t[m];
			nd_dst.n[m] = nd_src.n[m];
		}

		for (int k = 0; k < EDNODE_NN_MAX; k++)
		{
			nd_dst.neighbors[k] = nd_src.neighborIndices.size() > k ? nd_src.neighborIndices[k] : -1;
		}
	}

	__tic__();
	//copy: host --> device
	checkCudaErrors(cudaMemcpy(dev_ed_nodes_buf_, host_ed_nodes, sizeof(DeformGraphNodeCuda)*nodes_num, cudaMemcpyHostToDevice));
	LOGGER()->info("graph cudaMemcpy to GPU = %f\n", __toc__());

	//copy: host-->device. rigid transformation
	RigidTransformCuda rigid_transf;
	vnl_matrix_fixed<double, 3, 3> R = graph.global_rigid.rotation();
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
			rigid_transf.R[m][n] = R[m][n];
	for (int m = 0; m < 3; m++)
		rigid_transf.T[m] = graph.global_rigid.t[m];

	checkCudaErrors(cudaMemcpy(dev_rigid_transf_, &rigid_transf, sizeof(RigidTransformCuda), cudaMemcpyHostToDevice));

	delete[] host_ed_nodes;
	return true;
}

bool DeformGraphCuda::
readout_DeformGraph(NonrigidMatching::DeformGraph &graph, bool bCPUComputeConnectivity)
{
	int nodes_num = nodes_num_gpu_.sync_read();
	DeformGraphNodeCuda *graph_cuda = new DeformGraphNodeCuda[nodes_num];
	checkCudaErrors(cudaMemcpy(graph_cuda, this->dev_ed_nodes_buf_, sizeof(DeformGraphNodeCuda)*nodes_num, cudaMemcpyDeviceToHost));

	vector<vnl_vector_fixed<float, 3>> nodes_g;
	for (int i = 0; i < nodes_num; i++)
	{
		nodes_g.push_back(vnl_vector_fixed<float, 3>(graph_cuda[i].g[0], graph_cuda[i].g[1], graph_cuda[i].g[2]));
	}

	graph.nodes_dist = this->ed_cubes_res_;
	graph.nodes.resize(nodes_g.size());

	//read out global rigid
	RigidTransformCuda rigid_transf;
	checkCudaErrors(cudaMemcpy(&rigid_transf, dev_rigid_transf_, sizeof(RigidTransformCuda), cudaMemcpyDeviceToHost));
	vnl_matrix_fixed<double, 3, 3> R;
	vnl_vector_fixed<double, 3> T;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
			R[m][n] = rigid_transf.R[m][n];
	for (int m = 0; m < 3; m++)
		T[m] = rigid_transf.T[m];
	graph.global_rigid = RigidTransformModel(R, T);

	if (!bCPUComputeConnectivity)
	{
		for (int i = 0; i < nodes_g.size(); i++)
		{
			vnl_vector_fixed<float, 3> const&node_g = nodes_g[i];
			DeformGraphNode node;
			node.idx = i;
			node.g = vnl_vector_fixed<double, 3>(node_g[0], node_g[1], node_g[2]);
			node.vtIdx = -1;
			for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					node.A(m, n) = graph_cuda[i].A(m, n);
			for (int m = 0; m < 3; m++)
			{
				node.t(m) = graph_cuda[i].t(m);
				node.n(m) = graph_cuda[i].n(m);
			}

			for (int k = 0; k < EDNODE_NN_MAX; k++)
			{
				node.neighborIndices.push_back(graph_cuda[i].neighbors[k]);
			}
			graph.nodes[i] = node;
		}
	}
	else
	{
		//compute nodes connectivity
		for (int i = 0; i < nodes_g.size(); i++)
		{
			vnl_vector_fixed<float, 3> const&node_g = nodes_g[i];
			LOGGER()->info("Checking Nearest nodes at %4d\r", i);
			DeformGraphNode node;
			node.idx = i;
			node.g = vnl_vector_fixed<double, 3>(node_g[0], node_g[1], node_g[2]);
			node.vtIdx = -1;
			for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					node.A(m, n) = graph_cuda[i].A(m, n);
			for (int m = 0; m < 3; m++)
			{
				node.t(m) = graph_cuda[i].t(m);
				node.n(m) = graph_cuda[i].n(m);
			}

			vector<DoubleIndexPair> dist_vec;
			for (int j = 0; j < nodes_g.size(); j++)
			{
				double d = dist_3d(node_g, nodes_g[j]);
				dist_vec.push_back(DoubleIndexPair(d, j));
			}
			sort(dist_vec.begin(), dist_vec.end(), less_comparator_DoubleIndexPair);
			assert(dist_vec[0].second == i);

			for (int j = 0; j < MIN(dist_vec.size() - 1, EDNODE_NN); j++)
			{
				int node_idx = dist_vec[j + 1].second;
				if (dist_vec[j + 1].first < this->ed_cubes_res_*1.5)
					node.neighborIndices.push_back(node_idx);
			}


			if (node.neighborIndices.size() == 0)
				LOGGER()->warning("DeformGraphCuda::readout_DeformGraph", "isolated node found!\n");

			graph.nodes[i] = node;
		}
	}
	graph.nn_max = EDNODE_NN;
	graph.nodes_dist = ed_cubes_res_;
	return true;
}


bool DeformGraphCuda::
readout_DeformGraph_all_levels(DeformGraphHierachy &graph_hier)
{
	int ed_nodes_num_all_levels = 0;
	checkCudaErrors(cudaMemcpy(&ed_nodes_num_all_levels, dev_ed_nodes_num_all_levels_, sizeof(int), cudaMemcpyDeviceToHost));
	graph_hier.set_size(ed_nodes_num_all_levels, ed_hierachy_levels_);

	graph_hier.node_min_dist_level0 = ed_cubes_res_;
	graph_hier.nn_num = EDNODE_NN;

	//readout rigid transf
	checkCudaErrors(cudaMemcpy((&graph_hier.global_rigid), dev_rigid_transf_, sizeof(RigidTransformCuda), cudaMemcpyDeviceToHost));

	//readout nodes
	checkCudaErrors(cudaMemcpy(graph_hier.nodes, dev_ed_nodes_buf_, sizeof(DeformGraphNodeCuda)*ed_nodes_num_all_levels, cudaMemcpyDeviceToHost));
	//readout lelvel range
	for (int i = 0; i < ed_hierachy_levels_; i++)
	{
		checkCudaErrors(cudaMemcpy(&(graph_hier.nodes_range[i]), dev_ed_nodes_ranges_[i], sizeof(int2), cudaMemcpyDeviceToHost));
	}
	return true;
}

bool DeformGraphCuda::
readout_DeformGraph_all_levels(EDNodesParasGPU ed_nodes, DeformGraphHierachy &graph_hier)
{
	int ed_nodes_num = 0;
	checkCudaErrors(cudaMemcpy(&ed_nodes_num, ed_nodes.ed_nodes_num_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
	graph_hier.set_size(ed_nodes_num, 1);

	graph_hier.node_min_dist_level0 = ed_nodes.ed_cubes_res;
	graph_hier.nn_num = EDNODE_NN;

	//readout rigid transf
	checkCudaErrors(cudaMemcpy((&graph_hier.global_rigid), ed_nodes.dev_rigid_transf, sizeof(RigidTransformCuda), cudaMemcpyDeviceToHost));

	//readout nodes
	checkCudaErrors(cudaMemcpy(graph_hier.nodes, ed_nodes.dev_ed_nodes, sizeof(DeformGraphNodeCuda)*ed_nodes_num, cudaMemcpyDeviceToHost));
	graph_hier.nodes_range[0].x = 0;
	graph_hier.nodes_range[0].y = ed_nodes_num;


	return true;
}

bool DeformGraphCuda::
dump_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info)
{
	DeformGraphHierachy graph;
	readout_DeformGraph_all_levels(graph);
	graph.save_to_txt(filename_ed_graph);

	VoxelMatrix<short> ndIds;
	readout_cu_arr_ndIds(ndIds);
	ndIds.save_to_file_ascii(filename_arr_ndIds);

	FILE *fp = NULL;
	fopen_s(&fp, filename_ed_info, "w");
	if (!fp)
	{
		LOGGER()->warning("DeformGraphCuda::dump_ed_nodes_infos", "Cannot open the file <%s> for saving VoxelMatrix.\n", filename_ed_info);
		return false;
	}
	fprintf(fp, "ED sparse cubes info.\n");
	int3 dim = ed_cubes_dims();
	fprintf(fp, "dim: <%d, %d, %d>\n", dim.x, dim.y, dim.z);
	float3 offset = ed_cubes_offset();
	fprintf(fp, "offset: <%f, %f, %f>\n", offset.x, offset.y, offset.z);
	fprintf(fp, "res: %f\n", ed_cubes_res());
	fclose(fp);

	return true;
}

bool DeformGraphCuda::
feed_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info)
{
	DeformGraphHierachy graph_hier;
	graph_hier.load_from_txt(filename_ed_graph);
	feed_graph_hier(graph_hier);


	VoxelMatrix<short> ndIds;
	ndIds.load_from_file_ascii(filename_arr_ndIds);
	feed_cu_arr_ndIds(ndIds);

	FILE *fp = NULL;
	fopen_s(&fp, filename_ed_info, "r");
	if (!fp)
	{
		LOGGER()->warning("DeformGraphCuda::feed_ed_nodes_infos", "Cannot open the file <%s> for saving VoxelMatrix.\n", filename_ed_info);
		return false;
	}
	fscanf(fp, "ED sparse cubes info.\n");
	int3 dim;
	fscanf(fp, "dim: <%d, %d, %d>\n", &(dim.x), &(dim.y), &(dim.z));
	set_ed_cubes_dims(dim);
	float3 offset;
	fscanf(fp, "offset: <%f, %f, %f>\n", &(offset.x), &(offset.y), &(offset.z));
	set_ed_cubes_offset(offset);
	float cubes_res;
	fscanf(fp, "res: %f\n", &cubes_res);
	set_ed_cubes_res(cubes_res);
	fclose(fp);

	return true;
}


bool DeformGraphHierachy::save_to_txt(char const* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if (!fp)
	{
		LOGGER()->error("DeformGraphHierachy::save_to_txt", "Cannot open the file <%s>.", filename);
		return false;
	}

	fprintf(fp, "Hierachy Deform Graph Data(#version 1.0)\n");
	fprintf(fp, "Basic Info: node dist=%f, nn_num=%d\n", node_min_dist_level0, nn_num);
	fprintf(fp, "Global Rigid: <%f, %f, %f; %f, %f, %f; %f, %f, %f>, <%f, %f, %f>\n", global_rigid.R(0, 0), global_rigid.R(0, 1), global_rigid.R(0, 2),
		global_rigid.R(1, 0), global_rigid.R(1, 1), global_rigid.R(1, 2),
		global_rigid.R(2, 0), global_rigid.R(2, 1), global_rigid.R(2, 2),
		global_rigid.T(0), global_rigid.T(1), global_rigid.T(2));
	fprintf(fp, "Number of graph nodes of all levels %d\n", nodes_num_all_levels);
	fprintf(fp, "Number of levels %d\n", nodes_range.size());
	for (int i = 0; i < nodes_range.size(); i++)
	{
		fprintf(fp, "\tLevel %d: <%d, %d>\n", i, nodes_range[i].x, nodes_range[i].y);
	}
	fprintf(fp, "\n");

	for (int i = 0; i<nodes_num_all_levels; i++)
	{
		DeformGraphNodeCuda const &node = nodes[i];
		fprintf(fp, "Graph node %d: g = <%f %f %f>, n = <%f %f %f>, A = <%.15f %.15f %.15f; %.15f %.15f %.15f; %.15f %.15f %.15f>, t = <%.15f %.15f %.15f>\n",
			i,
			node.g[0], node.g[1], node.g[2],
			node.n[0], node.n[1], node.n[2],
			node.A[0][0], node.A[0][1], node.A[0][2],
			node.A[1][0], node.A[1][1], node.A[1][2],
			node.A[2][0], node.A[2][1], node.A[2][2],
			node.t[0], node.t[1], node.t[2]);

		fprintf(fp, "\t\t neighbor indices <%d>:", EDNODE_NN_MAX);
		for (int j = 0; j<EDNODE_NN_MAX; j++)
			fprintf(fp, " %d", node.neighbors[j]);
		fprintf(fp, "\n\n");
	}

	fclose(fp);

	return true;
}
bool DeformGraphHierachy::load_from_txt(char const* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "r");
	if (!fp)
	{
		LOGGER()->error("DeformGraphHierachy::load_from_txt", "Cannot open the file <%s>.", filename);
		return false;
	}

	free();

	double version = 0.0;
	fscanf(fp, "Hierachy Deform Graph Data(#version %lf)\n", &version);
	fscanf(fp, "Basic Info: node dist=%f, nn_num=%d\n", &(node_min_dist_level0), &(nn_num));
	fscanf(fp, "Global Rigid: <%f, %f, %f; %f, %f, %f; %f, %f, %f>, <%f, %f, %f>\n", &(global_rigid.R(0, 0)), &(global_rigid.R(0, 1)), &(global_rigid.R(0, 2)),
		&(global_rigid.R(1, 0)), &(global_rigid.R(1, 1)), &(global_rigid.R(1, 2)),
		&(global_rigid.R(2, 0)), &(global_rigid.R(2, 1)), &(global_rigid.R(2, 2)),
		&(global_rigid.T(0)), &(global_rigid.T(1)), &(global_rigid.T(2)));
	fscanf(fp, "Number of graph nodes of all levels %d\n", &(nodes_num_all_levels));
	int levels_num = 0;
	fscanf(fp, "Number of levels %d\n", &levels_num);
	nodes_range.resize(levels_num);
	for (int i = 0; i < nodes_range.size(); i++)
	{
		int dummy;
		fscanf(fp, "\tLevel %d: <%d, %d>\n", &dummy, &(nodes_range[i]).x, &(nodes_range[i].y));
	}
	fscanf(fp, "\n");

	if (nodes_num_all_levels > 0)
	{
		nodes = new DeformGraphNodeCuda[nodes_num_all_levels];
		for (int i = 0; i < nodes_num_all_levels; i++)
		{
			int dummy = 0;
			DeformGraphNodeCuda &node = nodes[i];
			fscanf(fp, "Graph node %d: g = <%f %f %f>, n = <%f %f %f>, A = <%f %f %f; %f %f %f; %f %f %f>, t = <%f %f %f>\n",
				&dummy,
				&(node.g[0]), &(node.g[1]), &(node.g[2]),
				&(node.n[0]), &(node.n[1]), &(node.n[2]),
				&(node.A[0][0]), &(node.A[0][1]), &(node.A[0][2]),
				&(node.A[1][0]), &(node.A[1][1]), &(node.A[1][2]),
				&(node.A[2][0]), &(node.A[2][1]), &(node.A[2][2]),
				&(node.t[0]), &(node.t[1]), &(node.t[2]));

			vnl_matrix_fixed<float, 3, 3> A;
			A.set(node.A.data_block());
			vnl_matrix_fixed<float, 3, 3> A_inv_t(0.0);
			if (A != vnl_matrix_fixed<float, 3, 3>(0.0))
				A_inv_t = vnl_inverse(A).transpose();
			node.A_inv_t.set(A_inv_t.data_block());

			int neighbors_num = 0;
			fscanf(fp, "\t\t neighbor indices <%d>:", &neighbors_num);
			for (int j = 0; j < neighbors_num; j++)
			{
				int idx = 0;
				fscanf(fp, " %d", &idx);
				node.neighbors[j] = idx;
			}
			fscanf(fp, "\n\n");
		}
	}

	fclose(fp);

	return true;
}