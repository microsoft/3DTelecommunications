namespace VolumetricFusionCuda{

	//512 threads per cuda block
	//one sparse cube per cuda block
	//each cuda block handles 8x8x8 grid within for-loop, there might be 16x16x16 grid
	__global__
		void marching_cubes_kernel_vMesh_stepVertex(VolumeConstDataGPU volume,
		float cube_res, int cube_size_in_voxel, float vxl_res, int vxls_per_cube,
		float* __restrict__ dev_vts_buf, int stride, int vt_buf_size,
		int* __restrict__ dev_global_vts_count,
		int3* dev_vxl_vtIdx_per_edge,
		int* dev_surfCubeIds, //list of cubes that contains zero crossing
		int* dev_surfCubes_offset, //should be initialized as -1
		int* dev_global_surfCube_count, //count of cubes that contain zero crossing
		int const* edgeTable,
		int resoFactor = 1.0f,
		float iso_level = 0.0f
		)
	{

		__shared__ float sh_volume[9 * 9 * 9]; //sdf values
		__shared__ float3 sh_grad[9 * 9 * 9]; //gradient
		__shared__ uchar4 sh_color[9 * 9 * 9];

		//__shared__ int3 vtIds_per_edge[8 * 8 * 8]; // each grid point has three edges
		__shared__ int shPtCount;
		__shared__ int sh_surfCubeOffset;
		__shared__ int sh_bPtsInsideCube;//include the boundary


		__shared__ int cubeId;
		__shared__ float x_corner; //cube corner
		__shared__ float y_corner;
		__shared__ float z_corner;
		__shared__ int3 cubes_num;
		__shared__ float3 cubes_offset;



		int const count_occupied_cube = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);

		for (int blkId = blockIdx.x; blkId < count_occupied_cube; blkId += gridDim.x)
		{
			if (threadIdx.x == 0)
			{
				cubes_num = volume.cubes_num[0];
				cubes_offset = volume.cubes_offset[0];
				cubeId = volume.occupied_cube_ids[blkId];

				int xCubeId = cubeId;
				int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
				xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
				int yCubeId = xCubeId / cubes_num.x;
				xCubeId -= yCubeId*cubes_num.x;

				//corner of the cube
				x_corner = xCubeId*cube_res + cubes_offset.x;
				y_corner = yCubeId*cube_res + cubes_offset.y;
				z_corner = zCubeId*cube_res + cubes_offset.z;
			}
			__syncthreads();


			if (threadIdx.x == 0)
			{
				shPtCount = 0; //no need to syncthreads. it is synced later
				sh_bPtsInsideCube = 0;
				sh_surfCubeOffset = 0;
			}

			//load volume data to shared memory
			for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
			{
				//compute 3D idx for the source
				int xId = idx; //original 3d index on sh_volume
				int zId = idx / (9 * 9);
				xId -= zId * 9 * 9;
				int yId = xId / 9;
				xId -= yId * 9;

				if (xId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					yId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					zId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
				{
					int pos = vxls_per_cube*blkId +
						zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
						yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
					float weight = volume.weights[pos];
					sh_color[idx] = volume.colors[pos];
					if (weight > THRES_WEIGHT_MATCHING_CUBES)
						sh_volume[idx] = volume.data[pos];
					else
						sh_volume[idx] = -2.0f;
				}
				else
				{
					int cubeId_src = cubeId;
					if (xId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += 1;
						xId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (yId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x;
						yId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (zId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x*cubes_num.y;
						zId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}

					if (cubeId_src >= cubes_num.x*cubes_num.y*cubes_num.z)
					{
						sh_volume[idx] = -2.0f;
						sh_color[idx] = make_uchar4(0, 0, 0, 0);
					}
					else
					{
						int offset = volume.cubes[cubeId_src].offset;
						if (offset < 0){
							sh_volume[idx] = -2.0f;
							sh_color[idx] = make_uchar4(0, 0, 0, 0);
						}
						else{
							int pos = vxls_per_cube*offset +
								zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
								yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
							float weight = volume.weights[pos];
							sh_color[idx] = volume.colors[pos];
							if (weight > THRES_WEIGHT_MATCHING_CUBES)
								sh_volume[idx] = volume.data[pos];
							else
								sh_volume[idx] = -2.0f;
						}
					}
				}
			}
			__syncthreads();

			//compute the gradient
			for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
			{
				float val_1 = sh_volume[idx];
				float val_0, val_2;
				int bValidComputation = 0;
				if (-1.0f < val_1 && val_1 < 1.0f)
				{
					bValidComputation = 1;

					int xId = idx;
					int zId = idx / (9 * 9);
					xId -= zId * 9 * 9;
					int yId = xId / 9;
					xId -= yId * 9;

					//compute dx
					val_0 = xId > 0 ? sh_volume[idx - 1] : -2.0f;
					val_2 = xId < 8 ? sh_volume[idx + 1] : -2.0f;
					if (val_0 > -1.0f && val_2 > -1.0f)
						sh_grad[idx].x = (val_0 - val_2) / 2.0f;
					else if (val_0 > -1.0f && val_2 <= -1.0f)
						sh_grad[idx].x = val_0 - val_1;
					else if (val_0 <= -1.0f && val_2 > -1.0f)
						sh_grad[idx].x = val_1 - val_2;
					else
						bValidComputation = 0;

					//compute dy
					val_0 = yId > 0 ? sh_volume[idx - 9] : -2.0f;
					val_2 = yId < 8 ? sh_volume[idx + 9] : -2.0f;
					if (val_0 > -1.0f && val_2 > -1.0f)
						sh_grad[idx].y = (val_0 - val_2) / 2.0f;
					else if (val_0 > -1.0f && val_2 <= -1.0f)
						sh_grad[idx].y = val_0 - val_1;
					else if (val_0 <= -1.0f && val_2 > -1.0f)
						sh_grad[idx].y = val_1 - val_2;
					else
						bValidComputation = 0;

					//compute dz
					val_0 = zId > 0 ? sh_volume[idx - 81] : -2.0f;
					val_2 = zId < 8 ? sh_volume[idx + 81] : -2.0f;
					if (val_0 > -1.0f && val_2 > -1.0f)
						sh_grad[idx].z = (val_0 - val_2) / 2.0f;
					else if (val_0 > -1.0f && val_2 <= -1.0f)
						sh_grad[idx].z = val_0 - val_1;
					else if (val_0 <= -1.0f && val_2 > -1.0f)
						sh_grad[idx].z = val_1 - val_2;
					else
						bValidComputation = 0;
				}

				if (bValidComputation == 0)
				{
					sh_grad[idx].x = 0.0;
					sh_grad[idx].y = 0.0;
					sh_grad[idx].z = 0.0;
				}
			}
			__syncthreads();


			//grid id
			int xId = threadIdx.x;
			int zId = xId / (8 * 8);
			xId -= zId * 8 * 8;
			int yId = xId / 8;
			xId -= yId * 8;



			int idx_sh = zId * 9 * 9 + yId * 9 + xId; //idx in shared memory
			float val_cur = sh_volume[idx_sh];
			float vals[3];
			vals[0] = sh_volume[idx_sh + (resoFactor * 1)]; //x+1, y, z
			vals[1] = sh_volume[idx_sh + (resoFactor * 9)]; //x, y+1, z
			vals[2] = sh_volume[idx_sh + (resoFactor * 81)];//x, y, z+1
			int lcPtOffset = 0;
			int count = 0;
			int lc_bPtsInsideCube = 0;
			if (xId % resoFactor == 0 && yId % resoFactor == 0 && zId % resoFactor == 0)
			{
				if (-1.0f < val_cur && val_cur < 1.0f)
				{
					for (int i = 0; i < 3; i++)
						if (-1.0f< vals[i] && vals[i] < 1.0f && (val_cur - iso_level)*(vals[i] - iso_level) <= 0.0f)
							count++;
					lcPtOffset = atomicAdd(&shPtCount, count);
					lc_bPtsInsideCube = atomicAdd(&sh_bPtsInsideCube, 1);
				}
			}
				const int lastRow = 8 - resoFactor;
				if (lc_bPtsInsideCube == 0 && count == 0 && (xId == lastRow || yId == lastRow || zId == lastRow))
				{
					int tableIdx = 0;
					if (-1.0f < sh_volume[idx_sh] && sh_volume[idx_sh] < iso_level) tableIdx |= 1;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 9)] && sh_volume[idx_sh + (resoFactor * 9)] < iso_level) tableIdx |= 2;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 9) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 9) + (resoFactor * 1)] < iso_level) tableIdx |= 4;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 1)] < iso_level) tableIdx |= 8;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 81)] && sh_volume[idx_sh + (resoFactor * 81)] < iso_level) tableIdx |= 16;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9)] < iso_level) tableIdx |= 32;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9) + (resoFactor * 1)] < iso_level) tableIdx |= 64;
					if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 1)] < iso_level) tableIdx |= 128;

					if (edgeTable[tableIdx])
						atomicAdd(&sh_bPtsInsideCube, 1);
				}
			
			__syncthreads();
			
				if (sh_bPtsInsideCube > 0)
				{
					if (threadIdx.x == 0)
					{
						sh_surfCubeOffset = atomicAdd(dev_global_surfCube_count, 1);
						if (sh_surfCubeOffset < TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX)
						{
							dev_surfCubeIds[sh_surfCubeOffset] = cubeId;
							dev_surfCubes_offset[cubeId] = sh_surfCubeOffset;
						}
					}

					if (threadIdx.x == 0)
					{
						shPtCount = atomicAdd(dev_global_vts_count, shPtCount);
					}
					__syncthreads();
					int3 vtIndices_for_three_edges = make_int3(-1, -1, -1);
					if (xId % resoFactor == 0 && yId % resoFactor == 0 && zId % resoFactor == 0)
					{
						
						if (-1.0f < val_cur && val_cur < 1.0f)
						{
							float3 pt;
							pt.x = xId*vxl_res + x_corner;
							pt.y = yId*vxl_res + y_corner;
							pt.z = zId*vxl_res + z_corner;

							lcPtOffset += shPtCount;
							if (-1.0f < vals[0] && vals[0] < 1.0f && (val_cur - iso_level) * (vals[0] - iso_level) <= 0.0f)
							{
								int lcCount = lcPtOffset++;

								if (lcCount < vt_buf_size)
								{
									vtIndices_for_three_edges.x = lcCount;

									float mu = 0.5f;
									if (fabsf(val_cur - vals[0]) > M_EPS)
										mu = (val_cur - iso_level) / (val_cur - vals[0]);
									dev_vts_buf[stride * lcCount] = pt.x + mu * vxl_res * resoFactor;
									dev_vts_buf[stride * lcCount + 1] = pt.y;
									dev_vts_buf[stride * lcCount + 2] = pt.z;

									float3 n;
									float3 grads = make_float3(0, 0, 0);
									for (int i = 1; i < resoFactor; ++i)
									{
										grads.x += sh_grad[idx_sh + i * 1].x;
										grads.y += sh_grad[idx_sh + i * 1].y;
										grads.z += sh_grad[idx_sh + i * 1].z;
									}
									n.x = sh_grad[idx_sh].x *(1.0f - mu) + grads.x * mu;
									n.y = sh_grad[idx_sh].y *(1.0f - mu) + grads.y * mu;
									n.z = sh_grad[idx_sh].z *(1.0f - mu) + grads.z * mu;
									normalize(n);
									dev_vts_buf[stride * lcCount + 3] = n.x;
									dev_vts_buf[stride * lcCount + 4] = n.y;
									dev_vts_buf[stride * lcCount + 5] = n.z;

									float3 clr;
									clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 1].x*mu;
									clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 1].y*mu;
									clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 1].z*mu;
									clr.x /= 255.0f;
									clr.y /= 255.0f;
									clr.z /= 255.0f;
									dev_vts_buf[stride*lcCount + 6] = clr.x;
									dev_vts_buf[stride*lcCount + 7] = clr.y;
									dev_vts_buf[stride*lcCount + 8] = clr.z;
								}
							}

							if (-1.0f < vals[1] && vals[1] < 1.0f && (val_cur - iso_level)*(vals[1] - iso_level) <= 0.0f)
							{
								int lcCount = lcPtOffset++;

								if (lcCount < vt_buf_size)
								{
									vtIndices_for_three_edges.y = lcCount;

									float mu = 0.5f;
									if (fabsf(val_cur - vals[1]) > M_EPS)
										mu = (val_cur - iso_level) / (val_cur - vals[1]);
									dev_vts_buf[stride * lcCount] = pt.x;
									dev_vts_buf[stride * lcCount + 1] = pt.y + mu * vxl_res * resoFactor;
									dev_vts_buf[stride * lcCount + 2] = pt.z;

									float3 n;
									float3 grads = make_float3(0, 0, 0);
									for (int i = 1; i < resoFactor; ++i)
									{
										grads.x += sh_grad[idx_sh + i * 9 ].x;
										grads.y += sh_grad[idx_sh + i * 9 ].y;
										grads.z += sh_grad[idx_sh + i * 9].z;
									}
									n.x = sh_grad[idx_sh].x *(1.0f - mu) + grads.x * mu;
									n.y = sh_grad[idx_sh].y *(1.0f - mu) + grads.y * mu;
									n.z = sh_grad[idx_sh].z *(1.0f - mu) + grads.z * mu;
									normalize(n);
									dev_vts_buf[stride * lcCount + 3] = n.x;
									dev_vts_buf[stride * lcCount + 4] = n.y;
									dev_vts_buf[stride * lcCount + 5] = n.z;

									float3 clr;
									clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 9].x*mu;
									clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 9].y*mu;
									clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 9].z*mu;
									clr.x /= 255.0f;
									clr.y /= 255.0f;
									clr.z /= 255.0f;
									dev_vts_buf[stride*lcCount + 6] = clr.x;
									dev_vts_buf[stride*lcCount + 7] = clr.y;
									dev_vts_buf[stride*lcCount + 8] = clr.z;
								}
							}

							if (-1.0f < vals[2] && vals[2] < 1.0f && (val_cur - iso_level)*(vals[2] - iso_level) <= 0.0f)
							{
								int lcCount = lcPtOffset++;

								if (lcCount < vt_buf_size)
								{
									vtIndices_for_three_edges.z = lcCount;

									float mu = 0.5f;
									if (fabsf(val_cur - vals[2]) > M_EPS)
										mu = (val_cur - iso_level) / (val_cur - vals[2]);
									dev_vts_buf[stride * lcCount] = pt.x;
									dev_vts_buf[stride * lcCount + 1] = pt.y;
									dev_vts_buf[stride * lcCount + 2] = pt.z + mu * vxl_res * resoFactor;

									float3 n;
									float3 grads = make_float3(0, 0, 0);
									for (int i = 1; i < resoFactor; ++i)
									{
										grads.x += sh_grad[idx_sh + i * 81].x;
										grads.y += sh_grad[idx_sh + i * 81].y;
										grads.z += sh_grad[idx_sh + i * 81].z;
									}
									n.x = sh_grad[idx_sh].x *(1.0f - mu) + grads.x * mu;
									n.y = sh_grad[idx_sh].y *(1.0f - mu) + grads.y * mu;
									n.z = sh_grad[idx_sh].z *(1.0f - mu) + grads.z * mu;
									normalize(n);
									dev_vts_buf[stride * lcCount + 3] = n.x;
									dev_vts_buf[stride * lcCount + 4] = n.y;
									dev_vts_buf[stride * lcCount + 5] = n.z;

									float3 clr;
									clr.x = sh_color[idx_sh].x * (1.0f - mu) + sh_color[idx_sh + 81].x*mu;
									clr.y = sh_color[idx_sh].y * (1.0f - mu) + sh_color[idx_sh + 81].y*mu;
									clr.z = sh_color[idx_sh].z * (1.0f - mu) + sh_color[idx_sh + 81].z*mu;
									clr.x /= 255.0f;
									clr.y /= 255.0f;
									clr.z /= 255.0f;
									dev_vts_buf[stride*lcCount + 6] = clr.x;
									dev_vts_buf[stride*lcCount + 7] = clr.y;
									dev_vts_buf[stride*lcCount + 8] = clr.z;
								}
							}
						} // end if val -1 ~ 1
					} // end if (reso factor %  =0 )

					dev_vxl_vtIdx_per_edge[sh_surfCubeOffset*vxls_per_cube + threadIdx.x] = vtIndices_for_three_edges;
				
			}   // end if pt inside cube > 0 
			__syncthreads();

		} // end for block
	}

	__global__
		void marching_cubes_kernel_vMesh_stepTriangle(VolumeConstDataGPU volume,
		int cube_size_in_voxel, int vxls_per_cube,
		int3 const* __restrict__ dev_vxl_vtIdx_per_edge,
		int const* __restrict__ dev_surfCubeIds, //list of cube Ids
		int const* __restrict__ dev_surfCubes_offset, //array
		int const* __restrict__ dev_global_surfCube_count, //count of cubes with surface points 
		char const* __restrict__ triTable,
		char const* __restrict__ edgeIdx_to_lcEdgeIdx_LookUp, //edgeIdx(0~11)-->lcEdgeIdx(0, 1, 2)
		int3* dev_triangles,
		int triangle_buf_size,
		int *dev_global_tri_count,
		int resoFactor = 1.0f,
		float iso_level = 0.0f
		)
	{
		__shared__ float sh_volume[9 * 9 * 9];
		__shared__ int3 sh_vtIdx_per_edge[9 * 9 * 9];
		__shared__ int cubeId;
		__shared__ int cubeOffset; //data offset
		__shared__ int3 cubes_num;

		__shared__ int sh_triCount;

		if (threadIdx.x == 0)
		{
			cubes_num = volume.cubes_num[0];
		}
		__syncthreads();

		const int count_surf_cubes = MIN(TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX, *dev_global_surfCube_count);
		for (int blkId = blockIdx.x; blkId < count_surf_cubes; blkId += gridDim.x)
		{
			if (threadIdx.x == 0)
			{
				cubeId = dev_surfCubeIds[blkId];
				cubeOffset = volume.cubes[cubeId].offset;
				sh_triCount = 0;
			}
			__syncthreads();

			//load volume data to shared memory
			for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
			{
				//compute 3D idx for the source
				int xId = idx; //original 3d index on sh_volume
				int zId = idx / (9 * 9);
				xId -= zId * 9 * 9;
				int yId = xId / 9;
				xId -= yId * 9;

				if (xId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					yId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL &&
					zId < TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
				{
					int pos = vxls_per_cube*cubeOffset +
						zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
						yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
					float weight = volume.weights[pos];
					if (weight > THRES_WEIGHT_MATCHING_CUBES)
						sh_volume[idx] = volume.data[pos];
					else
						sh_volume[idx] = -2.0f;

					pos = vxls_per_cube*blkId +
						zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
						yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
					sh_vtIdx_per_edge[idx] = dev_vxl_vtIdx_per_edge[pos];
				}
				else
				{
					int cubeId_src = cubeId;
					if (xId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += 1;
						xId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (yId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x;
						yId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}
					if (zId >= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL)
					{
						cubeId_src += cubes_num.x*cubes_num.y;
						zId -= TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL;
					}

					if (cubeId_src >= cubes_num.x*cubes_num.y*cubes_num.z)
					{
						sh_volume[idx] = -2.0f;
						sh_vtIdx_per_edge[idx] = make_int3(-1, -1, -1);
					}
					else
					{
						int offset = volume.cubes[cubeId_src].offset;

						if (offset < 0){
							sh_volume[idx] = -2.0f;
							sh_vtIdx_per_edge[idx] = make_int3(-1, -1, -1);
						}
						else{
							int pos = vxls_per_cube*offset +
								zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
								yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
							float weight = volume.weights[pos];
							if (weight > THRES_WEIGHT_MATCHING_CUBES)
								sh_volume[idx] = volume.data[pos];
							else
								sh_volume[idx] = -2.0f;

							int offset_surfCubes = dev_surfCubes_offset[cubeId_src];
							if (offset_surfCubes >= 0)
							{
								pos = vxls_per_cube*offset_surfCubes +
									zId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL +
									yId*TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL + xId;
								sh_vtIdx_per_edge[idx] = dev_vxl_vtIdx_per_edge[pos];
							}
							else
								sh_vtIdx_per_edge[idx] = make_int3(-1, -1, -1);

						}
					}
				}

			}
			__syncthreads();

			//process each vxl, find number of triangles
			int vxlId = threadIdx.x;
			//grid id
			int xId = vxlId;
			int zId = xId / (8 * 8);
			xId -= zId * 8 * 8;
			int yId = xId / 8;
			xId -= yId * 8;

			

				int idx_sh = zId * 9 * 9 + yId * 9 + xId; //idx in shared memory

				int tableIdx = 0;
			if (xId % resoFactor == 0  % resoFactor && yId % resoFactor == 0 && zId % resoFactor == 0)
			{
				if (-1.0f < sh_volume[idx_sh] && sh_volume[idx_sh] < iso_level) tableIdx |= 1;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 9)] && sh_volume[idx_sh + (resoFactor * 9)] < iso_level) tableIdx |= 2;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 9) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 9) + (resoFactor * 1)] < iso_level) tableIdx |= 4;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 1)] < iso_level) tableIdx |= 8;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 81)] && sh_volume[idx_sh + (resoFactor * 81)] < iso_level) tableIdx |= 16;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9)] < iso_level) tableIdx |= 32;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 9) + (resoFactor * 1)] < iso_level) tableIdx |= 64;
				if (-1.0f < sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 1)] && sh_volume[idx_sh + (resoFactor * 81) + (resoFactor * 1)] < iso_level) tableIdx |= 128;

			} // end if (resofactor % = 0) 
				int lc_triCount = 0;
				int vxl_triCount = triTable[tableIdx * 16];
				int vxl_triCount_valid = 0;
				int nonValid = 0;
				for (int i = 0; i < vxl_triCount; i++)
				{
					int3 tri;
					int edgeIdx = triTable[tableIdx * 16 + 1 + i * 3];
					int xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					int yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					int zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					int lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					int vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.x = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);

					edgeIdx = triTable[tableIdx * 16 + 2 + i * 3];
					xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.y = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);

					edgeIdx = triTable[tableIdx * 16 + 3 + i * 3];
					xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.z = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);
					if (tri.x >= 0 && tri.y >= 0 && tri.z >= 0)
						vxl_triCount_valid++;
					else
						nonValid++;
				}

				if (vxl_triCount_valid > 0)
				{
					lc_triCount = atomicAdd(&sh_triCount, vxl_triCount_valid);
				}
			
				__syncthreads();

				if (threadIdx.x == 0)
				{
					sh_triCount = atomicAdd(dev_global_tri_count, sh_triCount);
				}
				__syncthreads();

				lc_triCount += sh_triCount;
				for (int i = 0; i < vxl_triCount; i++)
				{
					int3 tri;
					int edgeIdx = triTable[tableIdx * 16 + 1 + i * 3];
					int xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					int yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					int zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					int lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					int vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.x = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);

					edgeIdx = triTable[tableIdx * 16 + 2 + i * 3];
					xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.y = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);

					edgeIdx = triTable[tableIdx * 16 + 3 + i * 3];
					xVxlId_n = xId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx];
					yVxlId_n = yId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 1];
					zVxlId_n = zId + resoFactor * edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 2];
					lcEdgeId = edgeIdx_to_lcEdgeIdx_LookUp[4 * edgeIdx + 3];
					vxlId_n = zVxlId_n * 81 + yVxlId_n * 9 + xVxlId_n;
					tri.z = (lcEdgeId == 0) ? sh_vtIdx_per_edge[vxlId_n].x : ((lcEdgeId == 1) ? sh_vtIdx_per_edge[vxlId_n].y : sh_vtIdx_per_edge[vxlId_n].z);

					if (tri.x >= 0 && tri.y >= 0 && tri.z >= 0 &&
						lc_triCount < triangle_buf_size)
					{
						dev_triangles[lc_triCount] = tri;
						lc_triCount++;
					}
				}
			
			__syncthreads();
		}
	}

	template<class T>
	__global__
		void init_buf(T* dev_buf, T val, int buf_size)
	{
		int id = threadIdx.x + blockIdx.x*blockDim.x;
		if (id < buf_size)
			dev_buf[id] = val;
	}

	void VolumetricFusionHelperCudaImpl::
		marching_cubes_vMesh(VolumeTwoLevelHierachy *volume, float* dev_vts_buf, cuda::gpu_size_data vts_num_gpu, int vts_dim,
		int3* dev_triangles_buf, cuda::gpu_size_data tris_num_gpu, float iso_level, int resoFactor)
	{
		if (volume->vxls_per_cube != 512)
		{
			LOGGER()->warning("VolumetricFusionHelperCudaImpl::marching_cubes_vMesh", "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
				"Warning<marching_cubes_vMesh>: only 8x8x8 cube is supported for now.\n"
				"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			return;
		}
		checkCudaErrors(cudaMemsetAsync(vts_num_gpu.dev_ptr, 0, sizeof(int)));
		checkCudaErrors(cudaMemsetAsync(surf_cubes_count_gpu_.dev_ptr, 0, sizeof(int)));
		int threads_per_block = 64;
		int blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_NUM_MAX + threads_per_block - 1) / threads_per_block;
		init_buf<int> << <blocks_per_grid, threads_per_block >> >(dev_surfCubes_offset_, -1, TWO_LEVEL_VOLUME_CUBES_NUM_MAX);
		m_checkCudaErrors();
		
		threads_per_block = 512;
		blocks_per_grid = 1024;
		VolumeConstDataGPU volume_gpu(*volume);

		marching_cubes_kernel_vMesh_stepVertex << <blocks_per_grid, threads_per_block >> >(volume_gpu,
			volume->cube_res, volume->cube_size_in_voxel, volume->vxl_res, volume->vxls_per_cube,
			dev_vts_buf, vts_dim, vts_num_gpu.max_size, vts_num_gpu.dev_ptr,
			dev_vxl_vtIdx_per_edge_, dev_surfCubeIds_, dev_surfCubes_offset_, surf_cubes_count_gpu_.dev_ptr, dev_edgeTable_, resoFactor, iso_level );
		m_checkCudaErrors();

		// This check is necessary because .sync_read() performs implicit synchronization and we don't want
		// that to be done outside debug mode
		if (LOGGER()->check_verbosity(Logger::Debug))
		{
			LOGGER()->debug(">>>>>>>>>>>>>>surface cubes num = %d", surf_cubes_count_gpu_.sync_read());
			LOGGER()->debug(">>>>>>>>>>>>>>surface vertex num = %d", vts_num_gpu.sync_read());
		}

		checkCudaErrors(cudaMemsetAsync(tris_num_gpu.dev_ptr, 0, sizeof(int)));
		threads_per_block = 512;
		blocks_per_grid = 512;
		marching_cubes_kernel_vMesh_stepTriangle << <blocks_per_grid, threads_per_block >> >(volume_gpu, volume->cube_size_in_voxel, volume->vxls_per_cube,
			dev_vxl_vtIdx_per_edge_, dev_surfCubeIds_, dev_surfCubes_offset_, surf_cubes_count_gpu_.dev_ptr,
			dev_triTable_, dev_edgeIdx_to_lcEdgeIdx_LookUp_, dev_triangles_buf, tris_num_gpu.max_size, tris_num_gpu.dev_ptr, resoFactor, iso_level );
		m_checkCudaErrors();

		LOGGER()->debug(">>>>>>>>>>>>>>triangles num = %d", tris_num_gpu.sync_read());

		vts_num_gpu.cap_gpu_size(vts_num_gpu.max_size);
		tris_num_gpu.cap_gpu_size(tris_num_gpu.max_size);
	}


	void VolumetricFusionHelperCudaImpl::setup_marching_cubes_vMesh()
	{
		surf_cubes_count_gpu_.max_size = TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX;
		checkCudaErrors(cudaMalloc(&(surf_cubes_count_gpu_.dev_ptr), sizeof(int)));
		checkCudaErrors(cudaMalloc(&dev_surfCubeIds_, sizeof(int)*TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX));
		checkCudaErrors(cudaMalloc(&dev_surfCubes_offset_, sizeof(int)*TWO_LEVEL_VOLUME_CUBES_NUM_MAX));
		checkCudaErrors(cudaMalloc(&dev_vxl_vtIdx_per_edge_, sizeof(int3)*TWO_LEVEL_VOLUME_SURFACE_CUBES_OCCUPIED_MAX*TWO_LEVEL_VOLUME_VXLS_PER_CUBE));

		checkCudaErrors(cudaMalloc(&dev_triTable_, sizeof(char) * 256 * 16));
		checkCudaErrors(cudaMalloc(&dev_edgeTable_, sizeof(int) * 256));
		checkCudaErrors(cudaMemcpy(dev_triTable_, &(TRI_TABLE[0][0]), sizeof(char) * 16 * 256, cudaMemcpyHostToDevice));
		checkCudaErrors(cudaMemcpy(dev_edgeTable_, &(EDGE_TABLE[0]), sizeof(int) * 256, cudaMemcpyHostToDevice));

		checkCudaErrors(cudaMalloc(&dev_edgeIdx_to_lcEdgeIdx_LookUp_, sizeof(char) * 12 * 4));
		checkCudaErrors(cudaMemcpy(dev_edgeIdx_to_lcEdgeIdx_LookUp_, &(EdgeIdx_to_lcEdgeIdx_LookUpTable[0][0]), sizeof(char) * 12 * 4, cudaMemcpyHostToDevice));
	}

}