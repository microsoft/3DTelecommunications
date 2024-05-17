namespace VolumetricFusionCuda{
void VolumetricFusionHelperCuda::feed_depth_textures(std::vector<cv::Mat> & depthImgs)
{
	feed_depth_texturesToArray(depthImgs, cu_3dArr_depth_);
	this->bind_cuda_array_to_texture_depth(cu_3dArr_depth_);
}

void VolumetricFusionHelperCuda::feed_depth_texturesToArray(std::vector<cv::Mat> &depthImgs, cudaArray * target, cudaStream_t * cs)
{
	if (depthImgs.size() != DEPTH_CAMERAS_NUM)
	{
		LOGGER()->error("VolumetricFusionHelperCuda::feed_depth_textures", "depthImgs.size()!=DEPTH_CAMERAS_NUM! (%d vs %d)",
			depthImgs.size(), DEPTH_CAMERAS_NUM);
	}

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(unsigned short), depth_width_, depth_height_);
	paras.dstArray = target;
	paras.extent = make_cudaExtent(depth_width_, depth_height_, 1);
	paras.kind = cudaMemcpyHostToDevice;
	for (int i = 0; i < depthImgs.size(); i++)
	{
		assert(depth_height_ == depthImgs[i].rows  &&	depth_width_ == depthImgs[i].cols);
		paras.dstPos = make_cudaPos(0, 0, i);
		paras.srcPtr = make_cudaPitchedPtr(depthImgs[i].ptr<unsigned short>(), depthImgs[i].step, depth_width_, depth_height_);

		if (cs)
		{
			checkCudaErrors(cudaMemcpy3DAsync(&paras, *cs));
		}
		else
		{
			checkCudaErrors(cudaMemcpy3DAsync(&paras));
		}
	}
}

void VolumetricFusionHelperCuda::feed_depth_texturesToArray(unsigned short* combinedDepthImg, int sizeInBytes, cudaArray * target, cudaStream_t * cs)
{
	if (sizeInBytes != sizeof(unsigned short)* DEPTH_CAMERAS_NUM *depth_width_*depth_height_)
	{
		LOGGER()->error("VolumetricFusionHelperCuda::feed_depth_textures", "depthImgs.size()!=DEPTH_CAMERAS_NUM*depth_width_*depth_height_! (%d vs %d)",
			sizeInBytes, DEPTH_CAMERAS_NUM *depth_width_*depth_height_);
	}

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(unsigned short), depth_width_, depth_height_);
	paras.dstArray = target;
	paras.extent = make_cudaExtent(depth_width_, depth_height_, DEPTH_CAMERAS_NUM);
	paras.kind = cudaMemcpyHostToDevice;

	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcPtr = make_cudaPitchedPtr(combinedDepthImg, depth_width_*sizeof(unsigned short), depth_width_, depth_height_);
	if (cs)
	{
		checkCudaErrors(cudaMemcpy3DAsync(&paras, *cs));
	}
	else
	{
		checkCudaErrors(cudaMemcpy3DAsync(&paras));
	}
}

void VolumetricFusionHelperCuda::feed_color_textures(std::vector<cv::Mat> colorImgs)
{
	if (colorImgs.size() != DEPTH_CAMERAS_NUM)
	{
		LOGGER()->error("VolumetricFusionHelperCuda::feed_color_textures", "depthImgs.size()!=DEPTH_CAMERAS_NUM! (%d vs %d)",
			colorImgs.size(), DEPTH_CAMERAS_NUM);
	}

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(uchar4), depth_width_, depth_height_);
	paras.dstArray = cu_3dArr_color_;
	paras.extent = make_cudaExtent(depth_width_, depth_height_, 1);
	paras.kind = cudaMemcpyHostToDevice;
	for (int i = 0; i < colorImgs.size(); i++)
	{
		cv::Mat img_with_alpha = pad_alpha_for_color_CvMat(colorImgs[i]);
		assert(depth_height_ == img_with_alpha.rows &&
			depth_width_ == img_with_alpha.cols);
		paras.dstPos = make_cudaPos(0, 0, i);
		paras.srcPtr = make_cudaPitchedPtr(img_with_alpha.ptr<uchar4>(), img_with_alpha.step, depth_width_, depth_height_);

		checkCudaErrors(cudaMemcpy3DAsync(&paras));
		img_with_alpha.release();
	}
}

void VolumetricFusionHelperCuda::label_fg_and_tighten_bbox(BoundingBox3D bbox, bool bUseDepthTopBitAsSeg, float granularity)
{
	BoundingBox3DCuda bbox_cuda;
	bbox_cuda.x_s = bbox.x_s;
	bbox_cuda.x_e = bbox.x_e;
	bbox_cuda.y_s = bbox.y_s;
	bbox_cuda.y_e = bbox.y_e;
	bbox_cuda.z_s = bbox.z_s;
	bbox_cuda.z_e = bbox.z_e;
	VolumetricFusionHelperCudaImpl::label_fg_and_tighten_bbox(bbox_cuda, bUseDepthTopBitAsSeg, granularity);
}

bool VolumetricFusionHelperCuda::setup_cameras(std::vector<GCameraView*> cams)
{
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

void VolumetricFusionHelperCuda::setup_color_map(float const* color_map)
{
	if (dev_color_map_ == NULL)
		checkCudaErrors(cudaMalloc(&dev_color_map_, sizeof(float)* 3 * 256));
	checkCudaErrors(cudaMemcpy(dev_color_map_, color_map, sizeof(float)* 3 * 256, cudaMemcpyHostToDevice));
}

void VolumetricFusionHelperCuda::allocate_triangles_buf(int tris_buf_size)
{
	tris_num_gpu_.max_size = tris_buf_size;
	checkCudaErrors(cudaMalloc(&(tris_num_gpu_.dev_ptr), sizeof(int)));

	checkCudaErrors(cudaMalloc(&dev_triangles_buf_, sizeof(int3)*tris_buf_size));
}


void VolumetricFusionHelperCuda::allocate_vts_buf(int vts_buf_size, int vts_dim)
{
	this->vts_buf_size_ = vts_buf_size;
	this->vts_dim_ = vts_dim;
	checkCudaErrors(cudaMalloc(&dev_vts_buf_, sizeof(float)*vts_dim*vts_buf_size));
	checkCudaErrors(cudaMalloc(&dev_vts_cur_buf_, sizeof(float)*vts_dim*vts_buf_size));
	checkCudaErrors(cudaMalloc(&dev_vts_t_buf_, sizeof(float)*vts_dim*vts_buf_size));
	checkCudaErrors(cudaMalloc(&dev_vts_half_buf_, sizeof(short)*6*vts_buf_size));
	checkCudaErrors(cudaMalloc(&dev_vts_prev_buf_, sizeof(float)*vts_dim*vts_buf_size));

	vts_num_gpu_.max_size = vts_buf_size;
	checkCudaErrors(cudaMalloc(&(vts_num_gpu_.dev_ptr), sizeof(int)));
	vts_t_num_gpu_.max_size = vts_buf_size;
	checkCudaErrors(cudaMalloc(&(vts_t_num_gpu_.dev_ptr), sizeof(int)));
	vts_half_num_gpu_.max_size = vts_buf_size;
	checkCudaErrors(cudaMalloc(&(vts_half_num_gpu_.dev_ptr), sizeof(int)));
	vts_cur_num_gpu_.max_size = vts_buf_size;
	checkCudaErrors(cudaMalloc(&(vts_cur_num_gpu_.dev_ptr), sizeof(int)));
	vts_prev_num_gpu_.max_size = vts_buf_size;
	checkCudaErrors(cudaMalloc(&(vts_prev_num_gpu_.dev_ptr), sizeof(int)));

	checkCudaErrors(cudaMemsetAsync(vts_num_gpu_.dev_ptr, 0, sizeof(int)));
	checkCudaErrors(cudaMemsetAsync(vts_prev_num_gpu_.dev_ptr, 0, sizeof(int)));
	checkCudaErrors(cudaMemsetAsync(vts_t_num_gpu_.dev_ptr, 0, sizeof(int)));
	checkCudaErrors(cudaMemsetAsync(vts_half_num_gpu_.dev_ptr, 0, sizeof(int)));
	checkCudaErrors(cudaMemsetAsync(vts_cur_num_gpu_.dev_ptr, 0, sizeof(int)));

	checkCudaErrors(cudaMalloc(&dev_ed_nodes_, sizeof(DeformGraphNodeCuda)*ED_NODES_NUM_MAX));
	ed_nodes_num_gpu_.max_size = ED_NODES_NUM_MAX;
	checkCudaErrors(cudaMalloc(&(ed_nodes_num_gpu_.dev_ptr), sizeof(int)));
	checkCudaErrors(cudaMalloc(&dev_rigid_transf_, sizeof(RigidTransformCuda)));

	checkCudaErrors(cudaMalloc(&mesh_gpu_, sizeof(short)* 6 * vts_t_num_gpu().max_size));
}


void VolumetricFusionHelperCuda::
update_volume_dim_from_gpu(VolumeTwoLevelHierachy &volume_gpu)
{
	checkCudaErrors(cudaMemcpy(&(volume_gpu.cubes_num), volume_gpu.ptr_cubes_num, sizeof(int3), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&(volume_gpu.cubes_offset), volume_gpu.ptr_cubes_offset, sizeof(float3), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&(volume_gpu.cubes_occpied_count), volume_gpu.gpu_cubes_occpied_count.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));

	if (volume_gpu.cubes_occpied_count > TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX)
	{
		LOGGER()->warning("VolumetricFusionHelperCuda::update_volume_dim_from_gpu", "volume_gpu.cubes_occpied_count > TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX (%d v.s %d)",
			volume_gpu.cubes_occpied_count, TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX);
		volume_gpu.cubes_occpied_count = TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX;
	}
}

bool
VolumetricFusionHelperCuda::readout_volume_data(VolumeTwoLevelHierachy &volume_gpu, VolumeTwoLevelHierachy &volume_cpu, bool bReadGPUDimension, bool bReadVoxelData)
{
	if (bReadGPUDimension)
		update_volume_dim_from_gpu(volume_gpu);

	int cubes_occpied_count = volume_gpu.cubes_occpied_count;
	int3 cubes_num = volume_gpu.cubes_num;

	volume_cpu = volume_gpu;
	int size = cubes_num.x*cubes_num.y*cubes_num.z;
	LOGGER()->info("Cube Occupancy: %f", cubes_occpied_count / (double)size * 100.0);

	volume_cpu.cubes = new OccupcyCube[size];
	checkCudaErrors(cudaMemcpy(volume_cpu.cubes, volume_gpu.cubes, sizeof(OccupcyCube)*size, cudaMemcpyDeviceToHost));

	volume_cpu.buf_occupied_cube_ids = new int[cubes_occpied_count];
	checkCudaErrors(cudaMemcpy(volume_cpu.buf_occupied_cube_ids, volume_gpu.buf_occupied_cube_ids,
		sizeof(int)*cubes_occpied_count, cudaMemcpyDeviceToHost));

	int vxls_per_cube = volume_gpu.vxls_per_cube;
	volume_cpu.data = NULL;
	volume_cpu.weights = NULL;
	if (bReadVoxelData)
	{
		volume_cpu.data = new float[cubes_occpied_count*vxls_per_cube];
		volume_cpu.weights = new float[cubes_occpied_count*vxls_per_cube];

		checkCudaErrors(cudaMemcpy(volume_cpu.data, volume_gpu.data,
			cubes_occpied_count*vxls_per_cube*sizeof(float), cudaMemcpyDeviceToHost));
		checkCudaErrors(cudaMemcpy(volume_cpu.weights, volume_gpu.weights,
			cubes_occpied_count*vxls_per_cube*sizeof(float), cudaMemcpyDeviceToHost));
	}

	return true;
}

void VolumetricFusionHelperCuda::
feed_gpu_volume(VolumeTwoLevelHierachy const&volume_cpu, VolumeTwoLevelHierachy &volume_gpu)
{
	volume_gpu.mu = volume_cpu.mu;
	volume_gpu.vxl_res = volume_cpu.vxl_res;
	volume_gpu.cube_res = volume_cpu.cube_res;
	volume_gpu.cube_size_in_voxel = volume_cpu.cube_size_in_voxel;
	volume_gpu.vxls_per_cube = volume_cpu.vxls_per_cube;
	volume_gpu.cubes_occpied_capacity = volume_cpu.cubes_occpied_capacity;
	volume_gpu.cubes_offset = volume_cpu.cubes_offset;
	volume_gpu.cubes_num = volume_cpu.cubes_num;
	volume_gpu.cubes_occpied_count = volume_cpu.cubes_occpied_count;

	//set gpu dimension
	checkCudaErrors(cudaMemcpy(volume_gpu.ptr_cubes_num, &(volume_cpu.cubes_num), sizeof(int3), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(volume_gpu.ptr_cubes_offset, &(volume_cpu.cubes_offset), sizeof(float3), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(volume_gpu.gpu_cubes_occpied_count.dev_ptr, &(volume_cpu.cubes_occpied_count), sizeof(int), cudaMemcpyHostToDevice));
	volume_gpu.gpu_cubes_occpied_count.max_size = volume_cpu.cubes_occpied_capacity;

	int cubes_occpied_count = volume_cpu.cubes_occpied_count;
	int vxls_per_cube = volume_cpu.vxls_per_cube;
	int cubes_count = volume_cpu.cubes_num.x*volume_cpu.cubes_num.y*volume_cpu.cubes_num.z;
	//set gpu volume data
	if (volume_cpu.data != NULL)
		checkCudaErrors(cudaMemcpy(volume_gpu.data, volume_cpu.data, sizeof(float)*vxls_per_cube*cubes_occpied_count, cudaMemcpyHostToDevice));
	if (volume_cpu.weights != NULL)
		checkCudaErrors(cudaMemcpy(volume_gpu.weights, volume_cpu.weights, sizeof(float)*vxls_per_cube*cubes_occpied_count, cudaMemcpyHostToDevice));

	if (volume_cpu.cubes != NULL)
		checkCudaErrors(cudaMemcpy(volume_gpu.cubes, volume_cpu.cubes, sizeof(OccupcyCube)*cubes_count, cudaMemcpyHostToDevice));

	if (volume_cpu.buf_occupied_cube_ids != NULL)
		checkCudaErrors(cudaMemcpy(volume_gpu.buf_occupied_cube_ids, volume_cpu.buf_occupied_cube_ids, sizeof(int)*cubes_occpied_count, cudaMemcpyHostToDevice));
}


bool
VolumetricFusionHelperCuda::readout_volume_data(VolumeTwoLevelHierachy &volume, bool bReadVoxelData)
{
	return readout_volume_data(*dev_volume_, volume, true, bReadVoxelData);
}


bool
VolumetricFusionHelperCuda::readout_volume_cur_data(VolumeTwoLevelHierachy &volume, bool bReadVoxelData)
{
	return readout_volume_data(*dev_volume_cur_, volume, true, bReadVoxelData);
}

bool
VolumetricFusionHelperCuda::HierachyVolume_to_TSDF(VolumeTwoLevelHierachy const&volume, TSDF &sdf)
{
	BoundingBox3D bbox;
	bbox.x_s = volume.cubes_offset.x;
	bbox.y_s = volume.cubes_offset.y;
	bbox.z_s = volume.cubes_offset.z;
	bbox.x_e = volume.cubes_offset.x + volume.cubes_num.x* volume.cube_res;
	bbox.y_e = volume.cubes_offset.y + volume.cubes_num.y* volume.cube_res;
	bbox.z_e = volume.cubes_offset.z + volume.cubes_num.z* volume.cube_res;

	double vxl_res = volume.vxl_res;
	sdf.Init(bbox, vxl_res, vxl_res, vxl_res, volume.mu, false, false, false);

	int cube_size_in_voxel = volume.cube_size_in_voxel;
	int vxls_per_cube = cube_size_in_voxel*cube_size_in_voxel*cube_size_in_voxel;

	for (int zCubeId = 0; zCubeId < volume.cubes_num.z; zCubeId++)
	for (int yCubeId = 0; yCubeId < volume.cubes_num.y; yCubeId++)
	for (int xCubeId = 0; xCubeId < volume.cubes_num.x; xCubeId++)
	{
		int cubeId = zCubeId*volume.cubes_num.x*volume.cubes_num.y + yCubeId*volume.cubes_num.x + xCubeId;

		int offset = volume.cubes[cubeId].offset;

		if (offset >= 0)
		{
			int data_offset = offset * vxls_per_cube;
			int count = 0;
			for (int k = 0; k < cube_size_in_voxel; k++)
			for (int j = 0; j < cube_size_in_voxel; j++)
			for (int i = 0; i < cube_size_in_voxel; i++)
			{
				int xVxlId = xCubeId*cube_size_in_voxel + i;
				int yVxlId = yCubeId*cube_size_in_voxel + j;
				int zVxlId = zCubeId*cube_size_in_voxel + k;

				sdf.func_val(xVxlId, yVxlId, zVxlId) = volume.data[data_offset + count];
				sdf.weights(xVxlId, yVxlId, zVxlId) = volume.weights[data_offset + count];
				//correct sdf
				if (sdf.weights(xVxlId, yVxlId, zVxlId) == 0.0f)
					sdf.func_val(xVxlId, yVxlId, zVxlId) = -2.0f;

				count++;
			}
		}
	}
	return true;
}

void
VolumetricFusionHelperCuda::readout_point_cloud(CPointCloud<float> &pcd)
{
	int vts_num = vts_num_gpu_.sync_read();
	pcd.points_num = vts_num;
	pcd.colors = NULL;
	pcd.normals = new float[3 * vts_num];
	pcd.points = new float[3 * vts_num];

	float *buf = new float[this->vts_dim_*vts_num];
	checkCudaErrors(cudaMemcpy(buf, this->dev_vts_buf_, sizeof(float)* vts_dim_ * vts_num, cudaMemcpyDeviceToHost));

	for (int i = 0; i < vts_num; i++)
	{
		pcd.points[3 * i] = buf[i*vts_dim_];
		pcd.points[3 * i + 1] = buf[i*vts_dim_ + 1];
		pcd.points[3 * i + 2] = buf[i*vts_dim_ + 2];
		pcd.normals[3 * i] = buf[i*vts_dim_ + 3];
		pcd.normals[3 * i + 1] = buf[i*vts_dim_ + 4];
		pcd.normals[3 * i + 2] = buf[i*vts_dim_ + 5];
	}
	delete[] buf;
}

void VolumetricFusionHelperCuda::
readout_surface(float const* dev_vts, cuda::gpu_size_data vts_num, int vts_dim, CSurface<float> &surface)
{
	surface.freeMemory();
	int vts_num_cpu = 0;
	checkCudaErrors(cudaMemcpy(&vts_num_cpu, vts_num.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
	vts_num_cpu = MIN(vts_num_cpu, vts_num.max_size);

	surface.vtNum = vts_num_cpu;
	surface.vtDim = vts_dim;
	surface.color = (vts_dim == 9);
	surface.normal = true;

	surface.vtData = new float[surface.vtDim*vts_num_cpu];
	checkCudaErrors(cudaMemcpy(surface.vtData, dev_vts, sizeof(float)* surface.vtDim * vts_num_cpu, cudaMemcpyDeviceToHost));
}


void VolumetricFusionHelperCuda::
readout_surface_mesh(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vts_dim,
					 int3 const* dev_triangles, cuda::gpu_size_data tris_num_gpu,
					 CSurface<float> &surface)
{
	surface.freeMemory();
	int vts_num_cpu = 0;
	checkCudaErrors(cudaMemcpy(&vts_num_cpu, vts_num_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
	vts_num_cpu = MIN(vts_num_cpu, vts_num_gpu.max_size);

	surface.vtNum = vts_num_cpu;
	surface.vtDim = vts_dim;
	surface.color = (vts_dim == 9);
	surface.normal = true;

	surface.vtData = new float[surface.vtDim*vts_num_cpu];
	checkCudaErrors(cudaMemcpy(surface.vtData, dev_vts, sizeof(float)* surface.vtDim * vts_num_cpu, cudaMemcpyDeviceToHost));

	int tris_num_cpu = 0;
	checkCudaErrors(cudaMemcpy(&tris_num_cpu, tris_num_gpu.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
	tris_num_cpu = MIN(tris_num_cpu, tris_num_gpu.max_size);

	surface.triangles = new int[tris_num_cpu * 3];
	surface.triNum = tris_num_cpu;
	checkCudaErrors(cudaMemcpy(surface.triangles, dev_triangles, sizeof(int)* tris_num_cpu * 3, cudaMemcpyDeviceToHost));

}

void
VolumetricFusionHelperCuda::readout_surface_t(CSurface<float> &surface)
{
	surface.freeMemory();
	surface.vtNum = vts_t_num_gpu_.sync_read();
	surface.vtDim = vts_dim_;
	surface.color = (vts_dim_ == 9);
	surface.normal = true;

	surface.vtData = new float[surface.vtDim*surface.vtNum];
	checkCudaErrors(cudaMemcpy(surface.vtData, dev_vts_t_buf_, sizeof(float)* surface.vtDim * surface.vtNum, cudaMemcpyDeviceToHost));
}

void
VolumetricFusionHelperCuda::readout_surface(CSurface<float> &surface)
{
	surface.freeMemory();
	surface.vtNum = vts_num_gpu_.sync_read();
	surface.vtDim = vts_dim_;
	surface.color = (vts_dim_==9);
	surface.normal = true;

	surface.vtData = new float[surface.vtDim*surface.vtNum];
	checkCudaErrors(cudaMemcpy(surface.vtData, this->dev_vts_buf_, sizeof(float)* surface.vtDim * surface.vtNum, cudaMemcpyDeviceToHost));
}

void
VolumetricFusionHelperCuda::readout_surface_cur(CSurface<float> &surface)
{
	surface.freeMemory();
	surface.vtNum = vts_cur_num_gpu_.sync_read();
	surface.vtDim = vts_dim_;
	surface.color = (vts_dim_ == 9);
	surface.normal = true;

	surface.vtData = new float[surface.vtDim*surface.vtNum];
	checkCudaErrors(cudaMemcpy(surface.vtData, this->dev_vts_cur_buf_, sizeof(float)* surface.vtDim * surface.vtNum, cudaMemcpyDeviceToHost));
}


void
VolumetricFusionHelperCuda::readout_mipmaps(std::vector<cv::Mat> &mipmaps)
{
	mipmaps.clear();

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcArray = cu_3dArr_mipmap_;
	paras.dstPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(unsigned char), depth_width_, depth_height_);
	paras.extent = make_cudaExtent(depth_width_ / 2, depth_height_, 1);
	paras.kind = cudaMemcpyDeviceToHost;
	for (int i = 0; i < this->num_depthmaps_; i++)
	{
		cv::Mat mipmap = cv::Mat(this->depth_height_, this->depth_width_, CV_MAKETYPE(CV_16U, 1));
		paras.srcPos = make_cudaPos(0, 0, i);
		paras.dstPtr = make_cudaPitchedPtr(mipmap.ptr<unsigned char>(), mipmap.step, depth_width_ / 2, depth_height_);
		checkCudaErrors(cudaMemcpy3D(&paras));
		mipmaps.push_back(mipmap);
	}
}


void 
VolumetricFusionHelperCuda::readout_normalMaps(std::vector<vnl_matrix<vnl_vector_fixed<float, 3>>> &normMaps)
{
	normMaps.resize(this->num_depthmaps_);

	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcArray = cu_3dArr_normal_;
	paras.dstPtr = make_cudaPitchedPtr(NULL, depth_width_ * sizeof(float4), depth_width_, depth_height_);
	paras.extent = make_cudaExtent(depth_width_, depth_height_, 1);
	paras.kind = cudaMemcpyDeviceToHost;
	for (int i = 0; i < this->num_depthmaps_; i++)
	{
		vnl_matrix<float4> normMap(this->depth_height_, this->depth_width_);
		paras.srcPos = make_cudaPos(0, 0, i);
		paras.dstPtr = make_cudaPitchedPtr(normMap.data_block(), depth_width_ * sizeof(float4), depth_width_, depth_height_);
		checkCudaErrors(cudaMemcpy3D(&paras));

		normMaps[i].set_size(depth_height_, depth_width_);
		for (int m = 0; m < depth_height_; m++)
		for (int n = 0; n < depth_width_; n++)
		{
			float4 const&norm = normMap(m, n);
			normMaps[i](m, n) = vnl_vector_fixed<float, 3>(norm.x, norm.y, norm.z);
		}
	}
}

void VolumetricFusionHelperCuda::readout_depthImgs_f(std::vector<cv::Mat> &depthImgs_f)
{
	readout_depthImgs(cu_3dArr_depth_f_, depthImgs_f, num_depthmaps_, depth_width_, depth_height_);
}

void VolumetricFusionHelperCuda::readout_depthImgs(std::vector<cv::Mat> &depthImgs)
{
	readout_depthImgs(cu_3dArr_depth_, depthImgs, num_depthmaps_, depth_width_, depth_height_);
}

void VolumetricFusionHelperCuda::readout_depthImgs(cudaArray *cu_3dArr_depth, std::vector<cv::Mat> &depthImgs, int depth_num, int depth_width, int depth_height)
{
	releaseCvMats(depthImgs);
	cudaMemcpy3DParms paras = { 0 };
	paras.srcPos = make_cudaPos(0, 0, 0);
	paras.dstPos = make_cudaPos(0, 0, 0);
	paras.srcArray = cu_3dArr_depth;
	paras.dstPtr = make_cudaPitchedPtr(NULL, depth_width * sizeof(unsigned short), depth_width, depth_height);
	paras.extent = make_cudaExtent(depth_width, depth_height, 1);
	paras.kind = cudaMemcpyDeviceToHost;
	for (int i = 0; i < depth_num; i++)
	{
		cv::Mat depthImg = cv::Mat(depth_height, depth_width, CV_MAKETYPE(CV_16U, 1));
		paras.srcPos = make_cudaPos(0, 0, i);
		paras.dstPtr = make_cudaPitchedPtr(depthImg.ptr<unsigned short>(), depthImg.step, depth_width, depth_height);
		checkCudaErrors(cudaMemcpy3D(&paras));

		depthImgs.push_back(depthImg);
	}
}

void VolumetricFusionHelperCuda::readout_bbox_cur(BoundingBox3D &bbox_cur)
{
	BoundingBox3DCuda bbox_cuda;
	checkCudaErrors(cudaMemcpy(&bbox_cuda, dev_bbox_fg_cur_, sizeof(BoundingBox3DCuda), cudaMemcpyDeviceToHost));
	bbox_cur.x_s = bbox_cuda.x_s;
	bbox_cur.x_e = bbox_cuda.x_e;
	bbox_cur.y_s = bbox_cuda.y_s;
	bbox_cur.y_e = bbox_cuda.y_e;
	bbox_cur.z_s = bbox_cuda.z_s;
	bbox_cur.z_e = bbox_cuda.z_e;
}

void VolumetricFusionHelperCuda::readout_cube_ngns_info(vnl_vector<int> &ngn_indices, vector<vnl_vector_fixed<int, 2>> &ngn_indices_range)
{
	ngn_indices.set_size(this->cube_ngn_indices_buf_size_);
	checkCudaErrors(cudaMemcpy(ngn_indices.data_block(), this->dev_cube_ngns_indices_, sizeof(int)* this->cube_ngn_indices_buf_size_, cudaMemcpyDeviceToHost));

	vnl_vector<int2> ngn_indices_range_;
	ngn_indices_range_.set_size(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX);
	checkCudaErrors(cudaMemcpy(ngn_indices_range_.data_block(), this->dev_cube_ngn_indices_range_, sizeof(int2)* TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, cudaMemcpyDeviceToHost));
	ngn_indices_range.resize(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX);
	for (int i = 0; i < TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX; i++)
	{
		ngn_indices_range[i][0] = ngn_indices_range_[i].x;
		ngn_indices_range[i][1] = ngn_indices_range_[i].y;
	}
}


void VolumetricFusionHelperCuda::readout_weights_interp(vnl_vector<float>& weights)
{
	weights.set_size(TWO_LEVEL_VOLUME_VXL_NUM_MAX);
	checkCudaErrors(cudaMemcpy(weights.data_block(), this->dev_weights_interp_,
		sizeof(float)* TWO_LEVEL_VOLUME_VXL_NUM_MAX, cudaMemcpyDeviceToHost));
}


void VolumetricFusionHelperCuda::
readout_vtIdx_per_edge(vnl_vector<int> &vtIdx_per_edge)
{
	int surfCubes_num = this->surf_cubes_count_gpu_.sync_read();
	vtIdx_per_edge.set_size(surfCubes_num*TWO_LEVEL_VOLUME_VXLS_PER_CUBE * 3);

	checkCudaErrors(cudaMemcpy(vtIdx_per_edge.data_block(), this->dev_vxl_vtIdx_per_edge_,
		sizeof(int3)* surfCubes_num*TWO_LEVEL_VOLUME_VXLS_PER_CUBE, cudaMemcpyDeviceToHost));
}

void VolumetricFusionHelperCuda::
readout_surfCubes_list(vnl_vector<int> &surface_cubes_list)
{
	int surfCubes_num = this->surf_cubes_count_gpu_.sync_read();
	surface_cubes_list.set_size(surfCubes_num);

	checkCudaErrors(cudaMemcpy(surface_cubes_list.data_block(), this->dev_surfCubeIds_,
		sizeof(int)* surfCubes_num, cudaMemcpyDeviceToHost));
}

void VolumetricFusionHelperCuda::
readout_surfCubes_offset(vnl_vector<int> &surface_cubes_offset)
{
	surface_cubes_offset.set_size(TWO_LEVEL_VOLUME_CUBES_NUM_MAX);
	checkCudaErrors(cudaMemcpy(surface_cubes_offset.data_block(), this->dev_surfCubes_offset_,
		sizeof(int)* TWO_LEVEL_VOLUME_CUBES_NUM_MAX, cudaMemcpyDeviceToHost));
}


}