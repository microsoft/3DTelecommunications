#ifndef VNL_EDMATCHING_DEFS
#define VNL_EDMATCHING_DEFS
//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<short2> : public vnl_numeric_traits<short>
{
public:
	//: Additive identity
	static constexpr short zero = 0;
	//: Multiplicative identity
	static constexpr short one = 1;
	//: Maximum value which this type can assume
	static constexpr short maxval = 0x7fff; // = 0x7fff;
	//: Return value of abs()
	typedef unsigned short abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef int double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};
//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<ushort2> : public vnl_numeric_traits<ushort>
{
public:
	//: Additive identity
	static constexpr ushort zero = 0;
	//: Multiplicative identity
	static constexpr ushort one = 1;
	//: Maximum value which this type can assume
	static constexpr ushort maxval = 0x7fff; // = 0x7fff;
	//: Return value of abs()
	typedef ushort abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef int double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};


//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<std::vector<int, std::allocator<int>>*> : public vnl_numeric_traits<int>
{
public:
	//: Additive identity
	static constexpr ushort zero = 0;
	//: Multiplicative identity
	static constexpr ushort one = 1;
	//: Maximum value which this type can assume
	static constexpr ushort maxval = 0x7fff; // = 0x7fff;
	//: Return value of abs()
	typedef ushort abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef int double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};


//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<std::vector<float, std::allocator<float>>*> : public vnl_numeric_traits<float>
{
public:
	//: Additive identity
	static constexpr ushort zero = 0;
	//: Multiplicative identity
	static constexpr ushort one = 1;
	//: Maximum value which this type can assume
	static constexpr ushort maxval = 0x7fff; // = 0x7fff;
	//: Return value of abs()
	typedef ushort abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef int double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};
#endif
// Class constructor: do the texture memory/cuda array allocation
// Init: given the problem, have the cuda memory set up (create and copy data). build data structure for jtj eveluation
//       
class EDMatchingHelperCuda : public EDMatchingHelperCudaImpl
{
public:
	EDMatchingHelperCuda(int vts_num_max, int num_depth_map, int width_depth, int height_depth, bool bAllocateDepthArray)
		:EDMatchingHelperCudaImpl(vts_num_max)
	{
		depth_width_ = width_depth;
		depth_height_ = height_depth;
		if (bAllocateDepthArray && num_depth_map > 0)
		{
			allocate_and_texBind_cuda_arrays_depth(num_depth_map, width_depth, height_depth);
		}
		allocate_and_bind_vishull_cu3dArray();
		const int downsample_scale_proj = 2;
		allocate_memory_visibility_check(width_depth, height_depth, downsample_scale_proj);

	}
	~EDMatchingHelperCuda()
	{
		checkCudaErrors(cudaFreeArray(cu_3dArr_depth_));
	}


public:
	//set the data structure (vt_indices_for_jtj_ and vt_weights_for_jtj_) from CPU
	bool init_with_cpu_result(BlockedHessianMatrix const& jtj, 
			  int const*vt_indices, WeightsPair const* node_weights, int vt_indices_count, int vts_num,
			  vector<cv::Mat> depthImgs, vector<GCameraView*> cams_d);

	//compute in GPU the data structure (vt_indices_for_jtj_ and vt_weights_for_jtj_)
	void init(DeformGraphCuda *graph_cuda, vector<cv::Mat> depthImgs, vector<GCameraView*> cams_d, float vxl_res,
			  bool bComputeVisualHull,
				char const* debug_dir = NULL);
	void init(DeformGraphCuda *graph_cuda, BoundingBox3DCuda const* dev_bbox_visual_hull, //bbox of foreground
		bool bUseSegmentationForVisualHull,
		cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal, 
		int depth_num, int depth_width, int depth_height, 
		vector<GCameraView*> cams_d, float vxl_res, bool bComputeVisualHull, char const* debug_dir);
	void init(DeformGraphCuda *graph_cuda, BoundingBox3D bbox_visual_hull, //bbox of foreground
			  bool bUseSegmentationForVisualHull,
			  cudaArray *cu_3dArr_depth, cudaArray *cu_3dArr_normal,
			  int depth_num, int depth_width, int depth_height,
			  vector<GCameraView*> cams_d, float vxl_res, 
			  bool bComputeVisualHull,
			  char const* debug_dir);

	//jtj and jtf should be initialized beforehand
	bool read_out(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf);

	bool readout_jtj_block_info(HessianBlockInfoCuda* &hessian_block_info, int& para_blk_count, int &blk_count_data, int &blk_count_all);
	
	//debugging
	bool readout_vt_indices_weights_for_jtj(vnl_vector<int> &vt_indices_for_jtj, vnl_vector<float> &vt_weights_for_jtj);
	void dump_gpu_data(char const* dir_debug);
	void readout_Jtj2DInfos(vnl_matrix<int> &jtj_2d_info_offset, vnl_matrix<int> &jtj_2d_info_flag)
	{
		vnl_matrix<short2> jtj_2d_infos(this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)));
		
		checkCudaErrors(cudaMemcpy(jtj_2d_infos.data_block(), this->dev_jtj_2d_infos_, 
			sizeof(short2)*this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)) * this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), cudaMemcpyDeviceToHost));

		jtj_2d_info_offset.set_size(this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)));
		jtj_2d_info_flag.set_size(this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)));
		for (int i = 0; i < this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)); i++)
		for (int j = 0; j < this->jtj_para_blks_num_.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)); j++)
		{
			jtj_2d_info_offset(i, j) = jtj_2d_infos(i, j).x;
			jtj_2d_info_flag(i, j) = jtj_2d_infos(i, j).y;
		}
	}
	void readout_reg_node_pairs(vnl_matrix<int> &node_pairs)
	{
		int reg_count = ed_reg_pairs_count_gpu_.sync_read();
		vnl_vector<ushort2> node_pairs_list(reg_count);
		checkCudaErrors(cudaMemcpy(node_pairs_list.data_block(), this->dev_reg_node_pairs_list_buf_, sizeof(ushort2)*reg_count, cudaMemcpyDeviceToHost));

		node_pairs.set_size(reg_count, 2);
		for (int i = 0; i < reg_count; i++)
		{
			node_pairs(i, 0) = node_pairs_list[i].x;
			node_pairs(i, 1) = node_pairs_list[i].y;
		}
	}

public:
	static bool save_HessianBlockInfoCuda_buf_ASCII(char const* file_name, HessianBlockInfoCuda const* hessian_block_info, int count);
	static bool HessianBlockInfoCudaBuf_to_PtsCountMat(HessianBlockInfoCuda const* hessian_block_info, int count, vnl_matrix<int> &countMat, int ed_node_num);
	static bool dump_debug_information(HessianBlockInfoCuda const*hessian_block_info, int blk_count, int ed_node_num,
		int const*vt_indices_for_jtj, float const*vt_weights_for_jtj, int vt_indices_for_jtj_count, char const* dir_debug);

public:
	void compute_visual_hull(BoundingBox3DCuda const* dev_bbox, bool bUseSegmentation, int blurIters);
	bool init_internal( BoundingBox3DCuda const* dev_bbox_visual_hull, //bbox of foreground
						bool bUseSegmentationForVisualHull,
						vector<GCameraView*> cams_d, float vxl_res = 0.3f,
						bool bComputeVisualHull = true,
						char const* debug_dir=NULL);

	void feed_keypts(S3DPointMatchSet const&matches);
	void feed_keypts_2d(std::vector<S2DPointMatchSet*> matches_all);

	void feed_textures(std::vector<cv::Mat> depthImgs)
	{
		if (cu_3dArr_depth_ == NULL)
		{
			LOGGER()->error("EDMatchingHelperCuda::feed_textures", "cu_3dArr_depth_ is NULL!");
		}
		cudaMemcpy3DParms paras = { 0 };
		paras.srcPos = make_cudaPos(0, 0, 0);
		paras.dstPos = make_cudaPos(0, 0, 0);
		paras.srcPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(unsigned short), depth_width_, depth_height_);
		paras.dstArray = cu_3dArr_depth_;
		paras.extent = make_cudaExtent(depth_width_, depth_height_, 1);
		paras.kind = cudaMemcpyHostToDevice;
		for (int i = 0; i < depthImgs.size(); i++)
		{
			assert(depth_height_ == depthImgs[i].rows &&
				depth_width_ == depthImgs[i].cols);
			paras.dstPos = make_cudaPos(0, 0, i);
			paras.srcPtr = make_cudaPitchedPtr(depthImgs[i].ptr<unsigned short>(), depthImgs[i].step, depth_width_, depth_height_);

			checkCudaErrors(cudaMemcpy3D(&paras));
		}
	}

	void dump_textures(std::vector<cv::Mat> &depthImgs)
	{
		depthImgs.clear();
		cudaMemcpy3DParms paras = { 0 };
		paras.srcPos = make_cudaPos(0, 0, 0);
		paras.dstPos = make_cudaPos(0, 0, 0);
		paras.srcArray = cu_3dArr_depth_;
		paras.dstPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(unsigned short), depth_width_, depth_height_);
		paras.extent = make_cudaExtent(depth_width_, depth_height_, 1);
		paras.kind = cudaMemcpyDeviceToHost;
		for (int i = 0; i < DEPTH_CAMERAS_NUM; i++)
		{
			cv::Mat img = cv::Mat(depth_height_, depth_width_, CV_MAKETYPE(16, 1));
			paras.srcPos = make_cudaPos(0, 0, i);
			paras.dstPtr = make_cudaPitchedPtr(img.ptr<unsigned short>(), img.step, depth_width_, depth_height_);
			checkCudaErrors(cudaMemcpy3D(&paras));

			depthImgs.push_back(img);
		}
	}

	bool setup_cameras(std::vector<GCameraView*> cams)
	{
		if (cams.size() == 0)
		{
			LOGGER()->error("VolumetricFusionHelperCuda::setup_cameras", "cam number == 0");
			return false;
		}

		CameraViewCuda *cams_cuda = new CameraViewCuda[cams.size()];
		for (int i = 0; i < cams.size(); i++)
		{
			cuda_matrix_fixed<float, 3, 3> &R = cams_cuda[i].cam_pose.R;
			cuda_vector_fixed<float, 3> &T = cams_cuda[i].cam_pose.T;
			cuda_matrix_fixed<float, 3, 3> &K = cams_cuda[i].K;

			for (int m = 0; m < 3; m++)
			{
				T[m] = cams[i]->T[m];
				for (int n = 0; n < 3; n++)
				{
					R[m][n] = cams[i]->R[m][n];
					K[m][n] = cams[i]->K[m][n];
				}
			}
		}

		if (cams.size() != DEPTH_CAMERAS_NUM)
		{
			LOGGER()->error("VolumetricFusionHelperCuda::setup_cameras", "cam number != view num <%d v.s. %d>", cams.size(), DEPTH_CAMERAS_NUM);
			return false;
		}

		this->feed_camera_view(cams_cuda, cams.size());
		delete[] cams_cuda;
		return true;
	}

public:
	void readout_visual_hull_occupancy(VoxelMatrix<float> &visual_hull_occupancy);
	void readout_visual_hull_offset(vnl_vector_fixed<float, 3> &offset);
	void dump_visual_hull(char const* dir, int idx);
	//does not need to pre-allocate memory
	void readout_depth_residual_error(float &mean_error, bool bReadFiltered = false);
	void readout_depth_mats_proj(vector<cv::Mat> &depthMats_proj, bool bReSize);
	void readout_depth_mats_corr_vtIdx(vector<vnl_matrix<int>> &depth_mat_corr_vtIdx);
	void readout_depth_mats_align_residual(vector<cv::Mat> &residual_imgs, bool bReadFiltered = false);
	void readout_depth_mats_align_residual(vector<vnl_matrix<float>> &residual_mats, bool bReadFiltered = false);
	void readout_per_vertex_align_residual(vnl_vector<float> &per_vertex_align_residual);
	void readout_per_vertex_align_residual_interp(vnl_vector<float> &per_vertex_align_residual);
	void readout_ed_nodes_align_residual(vnl_vector<float> &ed_nodes_align_residual);
	void readout_cam_vis(vnl_vector<ushort2> &cam_vis);//surface point visibility
	void readout_keypts_3d(S3DPointMatchSet &matches);

};