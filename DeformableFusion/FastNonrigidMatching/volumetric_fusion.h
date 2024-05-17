#define VTS_NUM_MAX (1024*1600) //maximum number of vertices we allocate memory for, this is largely dependent on config file fusion_voxel_res and fusion_ed_nodes_res. 
#define VT_DIM 9

namespace VolumetricFusionCuda{
	
#ifndef VNL_CUDA_DEFS
#define VNL_CUDA_DEFS
	//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
	template <>
	class VNL_EXPORT vnl_numeric_traits<float4> : public vnl_numeric_traits<int>
	{
	public:
		//: Additive identity
		static constexpr float zero = 0.0F;
		//: Multiplicative identity
		static constexpr float one = 1.0F;
		//: Maximum value which this type can assume
		static constexpr float maxval = 3.40282346638528860e+38F;
		//: Return value of abs()
		typedef float abs_t;
		//: Name of a type twice as long as this one for accumulators and products.
		typedef double double_t;
		//: Name of type which results from multiplying this type with a double
		typedef double real_t;
	};


	//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
	template <>
	class VNL_EXPORT vnl_numeric_traits<int2> : public vnl_numeric_traits<int>
	{
	public:
		//: Additive identity
		static constexpr float zero = 0.0F;
		//: Multiplicative identity
		static constexpr float one = 1.0F;
		//: Maximum value which this type can assume
		static constexpr float maxval = 3.40282346638528860e+38F;
		//: Return value of abs()
		typedef float abs_t;
		//: Name of a type twice as long as this one for accumulators and products.
		typedef double double_t;
		//: Name of type which results from multiplying this type with a double
		typedef double real_t;
	};
#endif
class VolumetricFusionHelperCuda : public VolumetricFusionHelperCudaImpl
{
public:
	VolumetricFusionHelperCuda(CudaGlobalMemory *cuda_global_memory_key, int depth_num, int depth_width, int depth_height, bool bAllocateDepthCudaArray)
	{
		this->cuda_global_memory_key_ = cuda_global_memory_key;
		//allocate mipmap array and depth image array
		this->depth_height_ = depth_height;
		this->depth_width_ = depth_width;
		if (depth_num > 0)
		{
			if (bAllocateDepthCudaArray)
				allocate_cuda_arrays_depths(depth_num, depth_width, depth_height);
			this->allocate_cuda_arrays_normals(depth_num, depth_width, depth_height);

			//TODO: assume depth and color is aligned and have the same resolution
			this->allocate_cuda_arrays_colors(depth_num, depth_width, depth_height);
		}

		this->allocate_vts_buf(VTS_NUM_MAX, VT_DIM);
		this->allocate_triangles_buf(VTS_NUM_MAX * 2);
		this->setup_marching_cubes_vMesh();

		setup_color_map(&(CLRMAP_JET[0][0]));
	}
	~VolumetricFusionHelperCuda()
	{}
public:
	void feed_depth_textures(std::vector<cv::Mat> &depthImgs);
	void feed_depth_texturesToArray(std::vector<cv::Mat> &depthImgs, cudaArray * target, cudaStream_t * cs = NULL);
	void feed_depth_texturesToArray(unsigned short* combinedDepthImg, int sizeInBytes, cudaArray * target, cudaStream_t * cs = NULL);

	void feed_color_textures(std::vector<cv::Mat> colorImgs);

	void label_fg_and_tighten_bbox(BoundingBox3D bbox, bool bUseDepthTopBitAsSeg, float granularity = -1.0f);

	bool setup_cameras(std::vector<GCameraView*> cams);

	void setup_color_map(float const* color_map);

	void allocate_vts_buf(int vts_buf_size, int vts_dim);
	void allocate_triangles_buf(int tris_buf_size);


	bool setup_volume(BoundingBox3D bbox, float coarse_cube_res, int cube_size_in_voxel, float mu);

	//must be called if setup_volume is called. because setup_volume will take away the memory allocation
	bool setup_volume_cur(BoundingBox3D bbox, float coarse_cube_res, int cube_size_in_voxel);


public:
	//dev_cubes_num, dev_cubes_offset, dev_occupied_cubes_count-->...
	void update_volume_dim_from_gpu(VolumeTwoLevelHierachy &volume_gpu);
	//gpu-->cpu
	bool readout_volume_data(VolumeTwoLevelHierachy &volume_gpu, VolumeTwoLevelHierachy &volume_cpu, bool bReadGPUDimension=false, bool bReadVoxelData = false);
	bool readout_volume_data(VolumeTwoLevelHierachy &volume, bool bReadVoxelData = false);
	bool readout_volume_cur_data(VolumeTwoLevelHierachy &volume, bool bReadVoxelData = false);
	//assume volume_gpu have enough memory allocated
	void feed_gpu_volume(VolumeTwoLevelHierachy const&volume_cpu, VolumeTwoLevelHierachy &volume_gpu);
	//allocate GPU volume, and copy data from CPU to GPU
	//depreciated
	bool setup_GPUVolume_data(VolumeTwoLevelHierachy &volume);
	bool HierachyVolume_to_TSDF(VolumeTwoLevelHierachy const&volume, TSDF &sdf);
	void readout_mipmaps(std::vector<cv::Mat> &mipmaps);
	void readout_normalMaps(std::vector<vnl_matrix<vnl_vector_fixed<float, 3>>> &normMaps);
	static void readout_depthImgs(cudaArray *cu_3dArr_depth, std::vector<cv::Mat> &depthImgs, int depth_num, int depth_width, int depth_height);
	void readout_depthImgs_f(std::vector<cv::Mat> &depthImgs_f);
	void readout_depthImgs(std::vector<cv::Mat> &depthImgs);
	void readout_bbox_cur(BoundingBox3D &bbox_cur);

	void readout_surface(float const* dev_vts, cuda::gpu_size_data vts_num, int vts_dim, CSurface<float> &surface);
	void readout_point_cloud(CPointCloud<float> &pcd);
	void readout_surface_t(CSurface<float> &surface);
	void readout_surface(CSurface<float> &surface);
	void readout_surface_cur(CSurface<float> &surface);

	void readout_surface_mesh(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vts_dim, 
							  int3 const* dev_triangles, cuda::gpu_size_data tris_num_gpu,
							CSurface<float> &surface);

	void readout_cube_ngns_info(vnl_vector<int> &ngn_indices, vector<vnl_vector_fixed<int, 2>> &ngn_indices_range);

	void readout_weights_interp(vnl_vector<float> &weights);

	void readout_vtIdx_per_edge(vnl_vector<int> &vtIdx_per_edge);
	void readout_surfCubes_list(vnl_vector<int> &surface_cubes_list);
	void readout_surfCubes_offset(vnl_vector<int> &surface_cubes_offset);


};

}
