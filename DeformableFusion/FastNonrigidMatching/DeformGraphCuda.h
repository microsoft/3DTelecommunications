// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
class DeformGraphHierachy
{
public:
	DeformGraphHierachy(int nodes_num_all = 0, int levels_num = 0)
		:nodes(NULL)
	{
		set_size(nodes_num_all, levels_num);
	}

	~DeformGraphHierachy()
	{
		if (nodes != NULL)
			delete[] nodes;
	}
public:
	int ed_nodes_num_all_levels() const
	{
		if (nodes_range.size() > 0)
			return nodes_range.back().x + nodes_range.back().y;
		else
			return 0;
	}

	int ed_nodes_num_1st_level() const
	{
		if (nodes_range.size() > 0)
			return nodes_range[0].y;
		else
			return 0;
	}

	void set_size(int nodes_num_all = 0, int levels_num = 0)
	{
		free();
		if (nodes_num_all > 0 && levels_num > 0)
		{
			nodes_num_all_levels = nodes_num_all;
			nodes = new DeformGraphNodeCuda[nodes_num_all_levels];
			nodes_range.resize(levels_num);
		}
	}
	void free()
	{
		if (nodes != NULL){
			delete[] nodes;
			nodes = NULL;
		}
		nodes_range.clear();
		nodes_num_all_levels = 0;
	}
	bool save_to_txt(char const* filename);
	bool load_from_txt(char const* filename);

public:
	RigidTransformCuda global_rigid;
	int nodes_num_all_levels;
	vector<int2> nodes_range;
	DeformGraphNodeCuda *nodes;
	float node_min_dist_level0;
	int nn_num;
};

class DeformGraphCuda : public DeformGraphCudaImpl
{
public:
	DeformGraphCuda(int vts_num_max, int vt_dim, float nodes_min_dist = 4.0)
	{
		this->ed_cubes_res_ = nodes_min_dist;
		allocate_memory(vts_num_max, vt_dim);
	}

	~DeformGraphCuda()
	{
		cudaFreeArray(cu_3dArr_ndIds_);
		cudaFreeArray(cu_3dArr_ndIds_old_);

		cudaFree(this->dev_ed_nodes_buf_);
		cudaFree(this->dev_ed_nodes_buf_old_);
	}

public:
	void allocate_memory(int vts_num, int vt_dim);

public:
	//set the surface/points for deformation: the surface/points is either from cpu or gpu
	//ed nodes are sampled from the surface/points
	bool set_surface_to_deform(float* dev_vts, int strid, cuda::gpu_size_data vts_num);
	bool set_surface_to_deform(float* dev_vts, int stride, int vts_num);
	bool set_surface_to_deform(CSurface<float> const&surface);
	bool read_out_surface_t(CSurface<float> &surface_t);

	//for debug
	bool feed_NeighborGraphNodes(NonrigidMatching::NeighborGraphNodesOfPointList const& ngns);
	bool read_out_NeighborGraphNodes(NonrigidMatching::NeighborGraphNodesOfPointList &ngns);

	//allocate necessary memory for ed nodes, if cpu graph is provided, it will be copied to gpu memory
	bool init_ed_from_cpu(NonrigidMatching::DeformGraph const* graph = NULL, float nodes_min_dist = 4.0);
	bool update(NonrigidMatching::DeformGraph const& graph, float nodes_min_dist); //update the device memory
	bool feed_graph_hier(DeformGraphHierachy const& graph_hier);

	void set_nodes_min_dist(float nodes_min_dist) { this->ed_cubes_res_ = nodes_min_dist; }

	//setup_surface should be called beforehand
	void sample_ed_nodes(BoundingBox3D bbox, float min_nodes_dist, int thres_pt_count = 50)
	{
		BoundingBox3DCuda bbox_cuda;
		bbox_cuda.x_s = bbox.x_s;
		bbox_cuda.x_e = bbox.x_e;
		bbox_cuda.y_s = bbox.y_s;
		bbox_cuda.y_e = bbox.y_e;
		bbox_cuda.z_s = bbox.z_s;
		bbox_cuda.z_e = bbox.z_e;
		DeformGraphCudaImpl::sample_ed_nodes_impl(bbox_cuda, min_nodes_dist, thres_pt_count);
	}
	bool sample_ed_nodes_vGMem(BoundingBox3D bbox, float min_nodes_dist, int thres_pt_count = 50, bool initBackground = false, bool bSwitch_ed_nodes_buf = true)
	{
		BoundingBox3DCuda bbox_cuda;
		bbox_cuda.x_s = bbox.x_s;
		bbox_cuda.x_e = bbox.x_e;
		bbox_cuda.y_s = bbox.y_s;
		bbox_cuda.y_e = bbox.y_e;
		bbox_cuda.z_s = bbox.z_s;
		bbox_cuda.z_e = bbox.z_e;
		return DeformGraphCudaImpl::sample_ed_nodes_impl_vGMem(bbox_cuda, min_nodes_dist, thres_pt_count, initBackground, bSwitch_ed_nodes_buf);
	}

	//readout only the nodes of the lowest level
	bool readout_DeformGraph(NonrigidMatching::DeformGraph &graph, bool bCPUComputeConnectivity = true);
	bool readout_DeformGraph_all_levels(DeformGraphHierachy &graph_hier);
	static bool readout_DeformGraph_all_levels(EDNodesParasGPU ed_nodes, DeformGraphHierachy &graph_hier);


	bool readout_cu_arr_ndIds(VoxelMatrix<short> &ndIds_arr)
	{
		int3 ed_cubes_dims;
		checkCudaErrors(cudaMemcpy(&ed_cubes_dims, this->dev_ed_cubes_dims_, sizeof(int3), cudaMemcpyDeviceToHost));

		ndIds_arr.set_size(ed_cubes_dims.x, ed_cubes_dims.y, ed_cubes_dims.z);

		cudaMemcpy3DParms paras = { 0 };
		paras.srcPos = make_cudaPos(0, 0, 0);
		paras.dstPos = make_cudaPos(0, 0, 0);
		paras.srcArray = cu_3dArr_ndIds_;
		paras.dstPtr = make_cudaPitchedPtr(ndIds_arr.data_block(), ndIds_arr.ni()*sizeof(short), ndIds_arr.ni(), ndIds_arr.nj());
		paras.extent = make_cudaExtent(ed_cubes_dims.x, ed_cubes_dims.y, ed_cubes_dims.z);
		paras.kind = cudaMemcpyDeviceToHost;

		checkCudaErrors(cudaMemcpy3D(&paras));
		return true;
	}
	bool feed_cu_arr_ndIds(VoxelMatrix<short> const& ndIds_arr)
	{
		cudaMemcpy3DParms paras = { 0 };
		paras.srcPos = make_cudaPos(0, 0, 0);
		paras.dstPos = make_cudaPos(0, 0, 0);
		paras.srcPtr = make_cudaPitchedPtr((void*) ndIds_arr.data_block(), ndIds_arr.ni()*sizeof(short), ndIds_arr.ni(), ndIds_arr.nj());
		paras.dstArray = cu_3dArr_ndIds_;
		paras.extent = make_cudaExtent(ndIds_arr.ni(), ndIds_arr.nj(), ndIds_arr.nk());
		paras.kind = cudaMemcpyHostToDevice;

		checkCudaErrors(cudaMemcpy3D(&paras));
		return true;
	}

	//debugging
public:
	bool dump_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info);
	bool feed_ed_nodes_infos(char const* filename_ed_graph, char const* filename_arr_ndIds, char const* filename_ed_info);
};
