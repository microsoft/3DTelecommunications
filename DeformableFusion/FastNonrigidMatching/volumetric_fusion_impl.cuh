namespace VolumetricFusionCuda{


#define THRES_WEIGHT_MATCHING_CUBES 0.01f


#define MIPMAP_MAX_LEVEL 7 

#define VXL_NEIGHBOR_EDNODE_NUM NEIGHBOR_EDNODE_NUM

#define TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL 8
#define TWO_LEVEL_VOLUME_VXLS_PER_CUBE (TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
#define TWO_LEVEL_VOLUME_CUBES_DIM_MAX 100
#define TWO_LEVEL_VOLUME_CUBES_NUM_MAX (TWO_LEVEL_VOLUME_CUBES_DIM_MAX * TWO_LEVEL_VOLUME_CUBES_DIM_MAX * TWO_LEVEL_VOLUME_CUBES_DIM_MAX)
#define TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX (TWO_LEVEL_VOLUME_CUBES_DIM_MAX*TWO_LEVEL_VOLUME_CUBES_DIM_MAX*6)
#define TWO_LEVEL_VOLUME_VXL_NUM_MAX (TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX*TWO_LEVEL_VOLUME_VXLS_PER_CUBE) //~50M vxls
#define TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX        TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX


//this defines the stride in which we sample (not how we load) the voxel data in marching cube.
// given 8x8x8 dim per block, the actual amount of work done changes based on how ResoFactor is defined:
// 1 = 8x8x8; 2= 4x4x4; 4 = 2x2x2 
const int cDefaultResoFactor = 2;
// using marching cube resofactor = 2, max should be 60k 30k, respectively. but using slightly higher numbers for safety
//with high quality, marching cube resofactor = 1, max can hit 500k ith a 200x200x200 setup
const int cMaxTri = 500000;
const int cMaxVert = 400000;


class VolumetricFusionHelperCudaImpl : public CudaGlobalMemoryStatic
{
public:
	VolumetricFusionHelperCudaImpl()
		: dev_color_map_(NULL),
		dev_vts_buf_(NULL),
		dev_vts_t_buf_(NULL),
		dev_vts_half_buf_(NULL),
		dev_vts_cur_buf_(NULL)
	{		
		checkCudaErrors(cudaMalloc(&dev_bbox_fg_cur_, sizeof(BoundingBox3DCuda)));

		checkCudaErrors(cudaMalloc(&(dev_weights_interp_), sizeof(float)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
		checkCudaErrors(cudaMalloc(&(dev_counts_interp_), sizeof(int)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));

		allocate_volume(dev_volume_, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL);
		allocate_volume(dev_volume_cur_, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL);
		allocate_volume(dev_volume_static_, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL);
		dev_volume_static_->cubes_occpied_count = 0;

		checkCudaErrors(cudaMalloc(&dev_vxl_data_f_, sizeof(float)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));

		//allocate memory for volume warping
		cube_ngn_indices_buf_size_ = TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX * 200;
		checkCudaErrors(cudaMalloc(&(dev_cube_ngns_indices_), sizeof(int)*cube_ngn_indices_buf_size_));
		checkCudaErrors(cudaMalloc(&(dev_cube_ngn_indices_range_), sizeof(int2)*TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX));

		checkCudaErrors(cudaMalloc(&dev_occu_cubes_count_pre_, sizeof(int)));	

		//allocate memory for volume_warp_f2f2
		checkCudaErrors(cudaMalloc(&dev_vxl_pos_t_, sizeof(ushort3)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
		checkCudaErrors(cudaMalloc(&dev_vxl_min_sdf_srcIdx_dst_, sizeof(int)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));
		checkCudaErrors(cudaMalloc(&dev_vxl_min_sdf_dst_, sizeof(float)*TWO_LEVEL_VOLUME_VXL_NUM_MAX));

		meshSimplifier = new McMeshProcessing();

		meshSimplifier->initialize(McMeshProcessing::MemoryModel::MEMORY_MODEL_HOST, cMaxTri, cMaxVert);
	}

public:
	//allocate mipmap array and depth image array
	bool allocate_cuda_arrays_depths(int tex_num, int width, int height);
	bool allocate_cuda_arrays_normals(int view_num, int width, int height);
	bool allocate_cuda_arrays_colors(int view_num, int width, int height);

//frame to frame nonrigid alignment and fusion (at data frame)
public:
	//set up cubes offset and cubes dim
	void setup_volume_info(VolumeTwoLevelHierachy* volume, BoundingBox3DCuda const*dev_bbox, float vxl_res, float fusion_mu);
	//if dev_bbox_cur is invalid, use rigidly transformed dev_bbox_prev as the new bounding box
	void setup_volume_info(VolumeTwoLevelHierachy* volume, 
						   BoundingBox3DCuda const*dev_bbox_cur, 
						   BoundingBox3DCuda const*dev_bbox_prev, RigidTransformCuda const* dev_rigid_transf, 
						   float vxl_res, float fusion_mu);

	//Note: will reset volume's occupied cubes' number
	void coarse_cube_prune_f2f(VolumeTwoLevelHierachy* volume, float const* vts_t, cuda::gpu_size_data vts_num_gpu, int vts_dim);
	void init_volume_to_zero(VolumeTwoLevelHierachy *volume);
	void init_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, bool bUseVisualHull = true, bool bUseNormal = true, bool bWeightBasedOnDepth = true);
	void update_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, bool bUseVisualHull = true, bool bUseNormal = true, bool bWeightBasedOnDepth = true);
	void update_volume_with_curr_depth(VolumeTwoLevelHierachy *volume, 
									   float const* dev_depth_align_residual, int width_proj, int height_prj,
									   bool bUseVisualHull = true, bool bUseNormal = true, bool bWeightBasedOnDepth = true);

	//volume_cur must be inited to zero before hand
	void warp_and_update_volume_f2f(VolumeTwoLevelHierachy *volume_prev, VolumeTwoLevelHierachy *volume_cur,
									DeformGraphNodeCuda const* dev_ed_nodes, float const* dev_ed_nodes_align_residual, cuda::gpu_size_data ed_nodes_num_gpu, float sigma_vxl_node_dist,
									int3 const* dev_ed_cubes_dims, float3 const* ed_cubes_offsets, float ed_cube_res, RigidTransformCuda* dev_rigid_transf, 
									float thres_align_residual);
	//handle voxel collision. volume_cur must be inited to zero before hand
	void warp_and_update_volume_f2f2(VolumeTwoLevelHierachy *volume_prev, VolumeTwoLevelHierachy *volume_cur,
									 DeformGraphNodeCuda const* dev_ed_nodes, float const* dev_ed_nodes_align_residual, cuda::gpu_size_data ed_nodes_num_gpu, float sigma_vxl_node_dist,
									 int3 const* dev_ed_cubes_dims, float3 const* ed_cubes_offsets, float ed_cube_res, RigidTransformCuda* dev_rigid_transf,
									 float thres_align_residual);
	//void normalize_sdf_f2f(VolumeTwoLevelHierachy *volume);
	void swap_volume_pointer(VolumeTwoLevelHierachy* &volume_prev, VolumeTwoLevelHierachy* &volume_curr){
		VolumeTwoLevelHierachy *tmp = volume_prev;
		volume_prev = volume_curr;
		volume_curr = tmp;
	}
	void extract_volume_bbox(VolumeTwoLevelHierachy const*volume, BoundingBox3DCuda *dev_bbox);

	void smooth_volume(VolumeTwoLevelHierachy* volume, float *dev_vxl_data_out, float thres_weight = 10.0f);
	float* dev_vxl_data_f(){ return dev_vxl_data_f_; }
public:
	void allocate_volume_f2f(int cube_size_in_vxl);
	VolumeTwoLevelHierachy *volume_f2f_prev_;
	VolumeTwoLevelHierachy *volume_f2f_curr_;

public:
	void marching_cubes(VolumeTwoLevelHierachy *volume, float* dev_vts_buf, cuda::gpu_size_data vts_num, int vts_dim);

	void convert_to_half_floats(float* old_mesh, short* mesh, int* count, int max_count);
	short* get_mesh_gpu() { return mesh_gpu_; }

public:

	void setup_marching_cubes_vMesh();
	void marching_cubes_vMesh(VolumeTwoLevelHierachy *volume, float* dev_vts_buf, cuda::gpu_size_data vts_num_gpu, int vts_dim, 
							  int3* dev_triangles_buf, cuda::gpu_size_data tris_num_gpu, float iso_level, int reso_factor = cDefaultResoFactor);

public:
	//the following four: lookup table from edgeIdx to vtIdx. 
	//only save the data from cubes that have surface points
	int3* dev_vxl_vtIdx_per_edge_;
	int* dev_surfCubeIds_;
	int* dev_surfCubes_offset_;
	cuda::gpu_size_data surf_cubes_count_gpu_;
	char* dev_triTable_; //256 X 16. each line: [triNum, vtIdx1, vtIdx2, ...]
	int* dev_edgeTable_;//256
	char* dev_edgeIdx_to_lcEdgeIdx_LookUp_;

public:
	void compute_mipmap();
	//void coarse_cube_prune();
	void update_volume();
	void marching_cubes();
	void bind_cuda_array_to_texture_depth(cudaArray *cu_3dArr_depth);

	void label_fg_and_tighten_bbox(BoundingBox3DCuda bbox, bool bUseDepthTopBitAsSeg = false, float granularity = -1.0f);
	void estimate_normal_for_depth_maps(bool bUpdateDepthToFilteredForFusion = false, bool bBilaterialFiltering = true);

	void volumetric_fusion_on_curr_depths();
	void marching_cubes_on_curr_depths();

public:
	void allocate_volume(VolumeTwoLevelHierachy* &volume, int cube_size_in_vxl);



public:
	void sortTriangles();
	void simplifyTriangles(bool doQuads);

public:

	float* dev_vts_cur(){ return this->dev_vts_cur_buf_; }
	float* dev_vts_t() { return this->dev_vts_t_buf_; }
	short* dev_vts_half() { return this->dev_vts_half_buf_; }

	float* dev_vts() { return this->dev_vts_buf_; }
	float* dev_vts_prev() { return this->dev_vts_prev_buf_; }
	int3* dev_triangles() { return this->dev_triangles_buf_; }
	int vts_buf_size() { return this->vts_buf_size_; }

	int vt_dim() { return this->vts_dim_; }
	cuda::gpu_size_data vts_num_gpu() { return this->vts_num_gpu_; }
	cuda::gpu_size_data vts_prev_num_gpu() { return this->vts_prev_num_gpu_; }
	cuda::gpu_size_data vts_t_num_gpu() { return this->vts_t_num_gpu_; }
	cuda::gpu_size_data vts_half_num_gpu() { return this->vts_half_num_gpu_; }
	cuda::gpu_size_data vts_cur_num_gpu() { return this->vts_cur_num_gpu_; }
	cuda::gpu_size_data tris_num_gpu() { return this->tris_num_gpu_; }
	
	//swith dev_vts with dev_vts_prev
	void switch_vts_buf(){
		float *tmp = dev_vts_buf_;
		dev_vts_buf_ = dev_vts_prev_buf_;
		dev_vts_prev_buf_ = tmp;

		cuda::gpu_size_data size_tmp = vts_num_gpu_;
		vts_num_gpu_ = vts_prev_num_gpu_;
		vts_prev_num_gpu_ = size_tmp;
	}

	cudaArray* cu_3dArr_depth() { return this->cu_3dArr_depth_; }
	cudaArray* cu_3dArr_depth_f() { return this->cu_3dArr_depth_f_; }
	cudaArray* cu_3dArr_color() { return this->cu_3dArr_color_; }
	cudaArray* cu_3dArr_normal() { return this->cu_3dArr_normal_; }
	BoundingBox3DCuda const* dev_bbox_fg_cur(){ return this->dev_bbox_fg_cur_; }

	VolumeTwoLevelHierachy*& dev_volume() { return dev_volume_; }
	VolumeTwoLevelHierachy*& dev_volume_cur() { return dev_volume_cur_; }
	VolumeTwoLevelHierachy*& dev_volume_static() { return dev_volume_static_; }

	//warp the voxels before fusion
	void bind_tex_ndId(cudaArray* cu_3dArr_ndIds);
	//Note: does not reset the occupied cubes count
	void coarse_cube_prune_vWarp(VolumeTwoLevelHierachy *volume,
								 DeformGraphNodeCuda const*dev_ed_nodes, int const* dev_ed_nodes_num, float sigma_vxl_node_dist,
								 int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res,
								 RigidTransformCuda const* dev_rigid_transf);
	void update_volume_vWarp(VolumeTwoLevelHierachy *volume,
							 DeformGraphNodeCuda const*dev_ed_nodes, float const* dev_ed_nodes_align_residual, 
							 int const* dev_ed_nodes_num, float sigma_vxl_node_dist,
							 int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, 
							 float ed_cube_res, RigidTransformCuda const* dev_rigid_transf,
							 int const* vts_ngn_indices,
							 int const* dev_depth_maps_proj, int const* dev_depth_maps_corr_vtIdx, float const* dev_depth_align_residual,
							 int depth_width_prj, int depth_height_prj);

	void copy_some_gpu_volume_data(VolumeTwoLevelHierachy *volumeDst, VolumeTwoLevelHierachy *volumeSrc);

	//TODO: colorChannelMode: 0--none; 1-copy color; 2-copy normal to color;
	void copy_vts_to_dev_buf(float* dev_buf, int stride, cuda::gpu_size_data buf_vt_size_gpu, int colorMode, bool bFlipNormal);
	void copy_vts_t_to_dev_buf(float* dev_buf, int stride, cuda::gpu_size_data buf_vt_size_gpu, int colorMode, bool bFlipNormal);
	void copy_vts_cur_to_dev_buf(float* dev_buf, int stride, cuda::gpu_size_data buf_vt_size_gpu, int colorMode, bool bFlipNormal);
	
	cudaError_t sync_copy_vts_to_cpu_buf_sync_as_half(short* buf_out, int vts_num_cpu_out, bool bFlipNormal);
	void sync_copy_vts_to_dev_buf(float* dev_vts_buf, int stride, int& vts_num_in_and_out, int colorMode, bool bFlipNormal){
		sync_copy_vts_buf(dev_vts_buf_, vts_dim_, vts_num_gpu_, dev_vts_buf, stride, vts_num_in_and_out, colorMode, bFlipNormal);
	}
	void sync_copy_vts_prev_to_dev_buf(float* dev_vts_buf, int stride, int& vts_num_in_and_out, int colorMode, bool bFlipNormal){
		sync_copy_vts_buf(dev_vts_prev_buf_, vts_dim_, vts_prev_num_gpu_, dev_vts_buf, stride, vts_num_in_and_out, colorMode, bFlipNormal);
	}
	void sync_copy_vts_t_to_dev_buf(float* dev_vts_buf, int stride, int& vts_num_in_and_out, int colorMode, bool bFlipNormal){
		sync_copy_vts_buf(dev_vts_t_buf_, vts_dim_, vts_t_num_gpu_, dev_vts_buf, stride, vts_num_in_and_out, colorMode, bFlipNormal);
	}
	void sync_copy_vts_cur_to_dev_buf(float* dev_vts_buf, int stride, int& vts_num_in_and_out, int colorMode, bool bFlipNormal){
		sync_copy_vts_buf(dev_vts_cur_buf_, vts_dim_, vts_cur_num_gpu_, dev_vts_buf, stride, vts_num_in_and_out, colorMode, bFlipNormal);
	}

	void sync_copy_tris_to_dev_buf(int* dev_triangles_buf_out, int& tris_num_in_and_out){
		sync_copy_triangles_buf(dev_triangles_buf_, tris_num_gpu_, dev_triangles_buf_out, tris_num_in_and_out);
	}

	static void sync_copy_vts_buf( float const* dev_vts_in, int stride_in, cuda::gpu_size_data vts_num_gpu_in,
							       float* dev_buf_out, int stride_out, int &buf_size_in_and_out, //number of vertices
							       int colorMode, bool bFlipNormal);

	static void sync_feed_vts_buf(float const* vts_data, int vt_dim, int vt_num,
									float *dev_vts_buf, int vt_dim_gpu, cuda::gpu_size_data vts_num_gpu);

	static void sync_copy_triangles_buf(int3 const* dev_triangles_buf, cuda::gpu_size_data tris_num_gpu, 
										int* dev_triangles_buf_out, int &buf_size_in_and_out);

	void set_vt_color_as_residual(float *dev_vts, float const*dev_per_vertex_align_residual, cuda::gpu_size_data vts_num_gpu);

	void feed_camera_view(CameraViewCuda* cam_views, int view_num);
	void init_dev_global_cubes_count();

protected:
	float *dev_vts_buf_;
	float *dev_vts_t_buf_;
	short *dev_vts_half_buf_;
	float *dev_vts_cur_buf_;
	float *dev_vts_prev_buf_;//for debug visualization
	cuda::gpu_size_data vts_num_gpu_;
	cuda::gpu_size_data vts_prev_num_gpu_;
	cuda::gpu_size_data vts_t_num_gpu_;
	cuda::gpu_size_data vts_half_num_gpu_;
	cuda::gpu_size_data vts_cur_num_gpu_;
	int vts_dim_;
	int vts_buf_size_;

	int3 *dev_triangles_buf_;
	cuda::gpu_size_data tris_num_gpu_;

	McMeshProcessing* meshSimplifier;


//for visualization of ED nodes in GUI
public:
	void dev_copy_ed_nodes(DeformGraphNodeCuda const* dev_ed_nodes_src, cuda::gpu_size_data ed_nodes_num_src_gpu, RigidTransformCuda* dev_rigid_transf_src){
		checkCudaErrors(cudaMemcpyAsync(dev_ed_nodes_, dev_ed_nodes_src, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX, cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(ed_nodes_num_gpu_.dev_ptr, ed_nodes_num_src_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
		checkCudaErrors(cudaMemcpyAsync(dev_rigid_transf_, dev_rigid_transf_src, sizeof(RigidTransformCuda), cudaMemcpyDeviceToDevice));
	}
	DeformGraphNodeCuda const* dev_ed_nodes() const { return dev_ed_nodes_; }
	RigidTransformCuda const* dev_rigid_transf() const { return dev_rigid_transf_; }
	int ed_nodes_num() const { return ed_nodes_num_gpu_.sync_read(); }
	cuda::gpu_size_data ed_nodes_num_gpu_;
	DeformGraphNodeCuda *dev_ed_nodes_;
	RigidTransformCuda* dev_rigid_transf_;

public:

	float* dev_color_map_; //256*3: JET color map

	BoundingBox3DCuda *dev_bbox_fg_cur_; //bbox on the depth map

	int cube_ngn_indices_buf_size_;
	int* dev_cube_ngns_indices_;
	int2* dev_cube_ngn_indices_range_;
	float* dev_weights_interp_;// for warping the volume data
	int* dev_counts_interp_; //for warping the volume data

	VolumeTwoLevelHierachy *dev_volume_; //accumualted volume
	float* dev_vxl_data_f_;
	short* mesh_gpu_;
	VolumeTwoLevelHierachy *dev_volume_cur_; //volume for current frame
	VolumeTwoLevelHierachy *dev_volume_static_; //volume for static frame
	cudaArray *cu_3dArr_depth_;
	cudaArray *cu_3dArr_depth_f_;
	cudaArray *cu_3dArr_color_;
	cudaArray *cu_3dArr_mipmap_;
	cudaArray *cu_3dArr_normal_;
	int num_depthmaps_;
	int depth_height_;
	int depth_width_;

	CudaGlobalMemory *cuda_global_memory_key_;

private:
	int* dev_occu_cubes_count_pre_;

	//volume warp f2f2
	ushort3* dev_vxl_pos_t_;
	int* dev_vxl_min_sdf_srcIdx_dst_;
	float* dev_vxl_min_sdf_dst_;
};

}
