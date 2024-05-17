#include "utility.h"

namespace VolumetricFusionCuda{

	__device__ __forceinline__ float atomicMin(float *address, float val)
	{
		int ret = __float_as_int(*address);
		while (val < __int_as_float(ret))
		{
			int old = ret;
			if ((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
				break;
		}
		return __int_as_float(ret);
	}

	__device__ __forceinline__ float atomicAbsMin_FavorNeg(float *address, float val)
	{
		float ret_f = *address;
		int ret = __float_as_int(ret_f);
		while ((ret_f>0 && val < ret_f) || (ret_f<0 && val<0 && val>ret_f))
		{
			int old = ret;
			if ((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
				break;
			ret_f = __int_as_float(ret);
		}
		return __int_as_float(ret);
	}

	__device__ __forceinline__ float atomicMax(float *address, float val)
	{
		int ret = __float_as_int(*address);
		while (val > __int_as_float(ret))
		{
			int old = ret;
			if ((ret = atomicCAS((int *)address, old, __float_as_int(val))) == old)
				break;
		}
		return __int_as_float(ret);
	}

	//32X32 kernels per cuda block
	__global__
		void label_fg_and_tighten_bbox_depth_maps_kernel(BoundingBox3DCuda *dev_bbox_out, BoundingBox3DCuda bbox_in, int depth_width, int depth_height, int bUseDepthTopBitAsSeg, float granularity)
	{
		//points positions in world space
		__shared__ float sh_pts_x[MAX_THREADS_PER_BLOCK];
		__shared__ float sh_pts_y[MAX_THREADS_PER_BLOCK];
		__shared__ float sh_pts_z[MAX_THREADS_PER_BLOCK];
		__shared__ char sh_flag[MAX_THREADS_PER_BLOCK];

		int tid = threadIdx.y*blockDim.x + threadIdx.x;
		sh_flag[tid] = 0;

		int u_ = threadIdx.x + blockIdx.x*blockDim.x;
		int v = threadIdx.y + blockIdx.y*blockDim.y;
		int vId = u_ / depth_width;
		int u = u_%depth_width;

		if (u < depth_width && v < depth_height && vId < dev_num_cam_views)
		{
			unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);

			if (bUseDepthTopBitAsSeg)
				depth_extract_fg(d);
			else
				depth_remove_top_bit(d);

			//bUseDepthTopBitAsSeg is used. for those invalid pixels labeled as fg, will not change its label 
			if (d > 0)
			{
				cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
				float fx = K[0][0];
				float fy = K[1][1];
				float cx = K[0][2];
				float cy = K[1][2];

				//to camera space
				cuda_vector_fixed<float, 3> p;
				p[2] = d / 10.0f;
				p[0] = (u - cx)*p[2] / fx;
				p[1] = (v - cy)*p[2] / fy;

				//to world space
				p -= dev_cam_views[vId].cam_pose.T;
				cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

				//lable foreground
				if (bbox_in.x_s < p_wld[0] && p_wld[0] < bbox_in.x_e &&
					bbox_in.y_s < p_wld[1] && p_wld[1] < bbox_in.y_e &&
					bbox_in.z_s < p_wld[2] && p_wld[2] < bbox_in.z_e)
				{
					d |= 0x8000;
					sh_pts_x[tid] = p_wld[0];
					sh_pts_y[tid] = p_wld[1];
					sh_pts_z[tid] = p_wld[2];
					sh_flag[tid] = 1;
				}
				surf2DLayeredwrite(d, surf_depthImgs, u * sizeof(unsigned short), v, vId, cudaBoundaryModeClamp);
			}
		}
		int hasFgPt = __syncthreads_or(sh_flag[tid]);

		//reduction to get min and max
		if (hasFgPt)
		{
			if (tid < MAX_THREADS_PER_BLOCK / 2)
			{
				char flag0 = sh_flag[tid];
				char flag1 = sh_flag[tid + MAX_THREADS_PER_BLOCK / 2];

				float x0 = sh_pts_x[tid];
				float x1 = sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2];
				float y0 = sh_pts_y[tid];
				float y1 = sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2];
				float z0 = sh_pts_z[tid];
				float z1 = sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2];

				if (flag0 && flag1)
				{
					sh_pts_x[tid] = MIN(x0, x1);
					sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(x0, x1);

					sh_pts_y[tid] = MIN(y0, y1);
					sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(y0, y1);

					sh_pts_z[tid] = MIN(z0, z1);
					sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(z0, z1);
				}
				else if (flag0 && !flag1)
				{
					sh_pts_x[tid] = x0;
					sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2] = x0;

					sh_pts_y[tid] = y0;
					sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2] = y0;

					sh_pts_z[tid] = z0;
					sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2] = z0;
				}
				else if (!flag0 && flag1)
				{
					sh_pts_x[tid] = x1;
					sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2] = x1;

					sh_pts_y[tid] = y1;
					sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2] = y1;

					sh_pts_z[tid] = z1;
					sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2] = z1;
				}
				else
				{
					sh_pts_x[tid] = 1.0e+10f;
					sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2] = -1.0e+10f;

					sh_pts_y[tid] = 1.0e+10f;
					sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2] = -1.0e+10f;

					sh_pts_z[tid] = 1.0e+10f;
					sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2] = -1.0e+10f;
				}
			}
			__syncthreads();

			for (int s = MAX_THREADS_PER_BLOCK / 4; s > 0; s >>= 1)
			{
				if (tid < s)
				{
					sh_pts_x[tid] = MIN(sh_pts_x[tid], sh_pts_x[tid + s]);
					sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2],
						sh_pts_x[tid + MAX_THREADS_PER_BLOCK / 2 + s]);

					sh_pts_y[tid] = MIN(sh_pts_y[tid], sh_pts_y[tid + s]);
					sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2],
						sh_pts_y[tid + MAX_THREADS_PER_BLOCK / 2 + s]);

					sh_pts_z[tid] = MIN(sh_pts_z[tid], sh_pts_z[tid + s]);
					sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2] = MAX(sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2],
						sh_pts_z[tid + MAX_THREADS_PER_BLOCK / 2 + s]);
				}
				__syncthreads();
			}

			//note: extend the bounding box by 3cm
			const float extension = 0.0f;// 3.0f;
			if (tid == 0)
			{
				if (sh_pts_x[0] < 1.0e+10f)
				{
					float vx = sh_pts_x[0] - extension;
					if (granularity > 0)
					{
						int x = (int)(vx / granularity);
						vx = x*granularity;
					}
					atomicMin(&(dev_bbox_out[0].x_s), vx);
				}
				if (sh_pts_y[0] < 1.0e+10f)
				{
					float vy = sh_pts_y[0] - extension;
					if (granularity > 0)
					{
						int y = (int)(vy / granularity);
						vy = y*granularity;
					}
					atomicMin(&(dev_bbox_out[0].y_s), vy);
				}
				if (sh_pts_z[0] < 1.0e+10f)
				{
					float vz = sh_pts_z[0] - extension;
					if (granularity > 0)
					{
						int z = (int)(vz / granularity);
						vz = z*granularity;
					}
					atomicMin(&(dev_bbox_out[0].z_s), vz);
				}
			}
			else if (tid == MAX_THREADS_PER_BLOCK / 2)
			{
				if (sh_pts_x[tid] > -1.0e+10f)
				{
					float vx = sh_pts_x[tid] + extension;
					atomicMax(&(dev_bbox_out[0].x_e), vx);
				}
				if (sh_pts_y[tid] > -1.0e+10f)
				{
					float vy = sh_pts_y[tid] + extension;
					atomicMax(&(dev_bbox_out[0].y_e), vy);
				}
				if (sh_pts_z[tid] > -1.0e+10f)
				{
					float vz = sh_pts_z[tid] + extension;
					atomicMax(&(dev_bbox_out[0].z_e), vz);
				}
			}
		}
	}

	__global__
		void init_bbox_kernel(BoundingBox3DCuda * dev_bbox_fg_cur)
	{
		dev_bbox_fg_cur[0].x_s = 1.0e+10f;
		dev_bbox_fg_cur[0].y_s = 1.0e+10f;
		dev_bbox_fg_cur[0].z_s = 1.0e+10f;

		dev_bbox_fg_cur[0].x_e = -1.0e+10f;
		dev_bbox_fg_cur[0].y_e = -1.0e+10f;
		dev_bbox_fg_cur[0].z_e = -1.0e+10f;
	}

	void VolumetricFusionHelperCudaImpl::label_fg_and_tighten_bbox(BoundingBox3DCuda bbox, bool bUseDepthTopBitAsSeg, float granularity)
	{
		init_bbox_kernel << <1, 1 >> >(dev_bbox_fg_cur_);

		dim3 threads_per_block(32, 32);
		dim3 blocks_per_grid((depth_width_* DEPTH_CAMERAS_NUM + 31) / 32, (depth_height_ + 31) / 32);
		label_fg_and_tighten_bbox_depth_maps_kernel << <blocks_per_grid, threads_per_block >> >(dev_bbox_fg_cur_, bbox, depth_width_, depth_height_, bUseDepthTopBitAsSeg, granularity);
		m_checkCudaErrors()
	}

	//1024 threads per cuda block (32*32)
	__global__
		void depth_map_calcNormal_kernel(int depth_width, int depth_height, int vId)
	{
		extern __shared__ char sh_mem[];
		float* sh_depth_subArr = (float*)sh_mem;

		int dim_subArr = blockDim.x + 2; //read 1 more pixels at each side
		//load depth to shared memory
		for (int j = threadIdx.y; j < dim_subArr; j += blockDim.y)
		{
			for (int i = threadIdx.x; i < dim_subArr; i += blockDim.x)
			{
				int x = blockIdx.x*blockDim.x + i - 1; //x in depth map
				int y = blockIdx.y*blockDim.y + j - 1;
				if (0 <= x && x < depth_width &&
					0 <= y && y < depth_height)
				{
					unsigned short d = tex2DLayered(tex_depthImgs, x, y, vId);
					depth_extract_fg(d);

					sh_depth_subArr[j*dim_subArr + i] = d / 10.0f;
				}
				else
				{
					sh_depth_subArr[j*dim_subArr + i] = 0.0f;
				}
			}
		}
		__syncthreads();


		cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
		float const&fx = K[0][0];
		float const&fy = K[1][1];
		float const&cx = K[0][2];
		float const&cy = K[1][2];

		//position in the image coordinate
		int i = blockIdx.x*blockDim.x + threadIdx.x;
		int j = blockIdx.y*blockDim.y + threadIdx.y;

		//cross production
		int x = threadIdx.x + 1; //location in sh_subArr;
		int y = threadIdx.y + 1;
		cuda_vector_fixed<float, 3> dx(0.0f);
		cuda_vector_fixed<float, 3> dy(0.0f);
		cuda_vector_fixed<float, 3> normal(0.0f);
		float d = sh_depth_subArr[y*dim_subArr + x];
		if (d > 0.0f)
		{
			float d_l = sh_depth_subArr[y*dim_subArr + x - 1]; //left
			float d_r = sh_depth_subArr[y*dim_subArr + x + 1]; //right
			if (d_l > 0.0f && d_r > 0.0f && fabsf(d_l - d_r) < 4.0f)
			{
				dx[0] = ((i + 1) - cx)*d_r / fx - ((i - 1) - cx)*d_l / fx;
				dx[1] = (j - cy)*d_r / fy - (j - cy)*d_l / fy;
				dx[2] = d_r - d_l;
			}
			else if (d_r > 0.0f && d > 0.0f && fabsf(d_r - d) < 2.0f)
			{
				dx[0] = ((i + 1) - cx)*d_r / fx - (i - cx)*d / fx;
				dx[1] = (j - cy)*d_r / fy - (j - cy)*d / fy;
				dx[2] = d_r - d;
			}
			else if (d_l > 0.0f && d > 0.0f && fabsf(d_l - d) < 2.0f)
			{
				dx[0] = (i - cx)*d / fx - ((i - 1) - cx)*d_l / fx;
				dx[1] = (j - cy)*d / fy - (j - cy)*d_l / fy;
				dx[2] = d - d_l;
			}

			float d_t = sh_depth_subArr[(y - 1)*dim_subArr + x]; //top
			float d_b = sh_depth_subArr[(y + 1)*dim_subArr + x]; //bottom
			if (d_t > 0.0f && d_b > 0.0f && fabsf(d_t - d_b) < 4.0f)
			{
				dy[0] = (i - cx)*d_b / fx - (i - cx)*d_t / fx;
				dy[1] = ((j + 1) - cy)*d_b / fy - ((j - 1) - cy)*d_t / fy;
				dy[2] = d_b - d_t;
			}
			else if (d_t > 0.0f && d > 0.0f && fabsf(d_t - d) < 2.0f)
			{
				dy[0] = (i - cx)*d / fx - (i - cx)*d_t / fx;
				dy[1] = (j - cy)*d / fy - ((j - 1) - cy)*d_t / fy;
				dy[2] = d - d_t;
			}
			else if (d > 0.0f && d_b > 0.0f && fabsf(d_b - d) < 2.0f)
			{
				dy[0] = (i - cx)*d_b / fx - (i - cx)*d / fx;
				dy[1] = ((j + 1) - cy)*d_b / fy - (j - cy)*d / fy;
				dy[2] = d_b - d;
			}

			normal = cross_product(dx, dy);
			normal.normalize();
		}

		if (i < depth_width && j < depth_height)
		{
			float4 n = make_float4(normal[0], normal[1], normal[2], 0.0);
			surf2DLayeredwrite(n, surf_normalMaps, i*sizeof(float4), j, vId, cudaBoundaryModeClamp);
		}
	}

	//filtering and then cross-product to get normal
	//TODO: filtering only inside a bounding box
	__global__
		void depth_map_filter_calcNormal_kernel(int depth_width, int depth_height, int vId, float sigma_d, float sigma_s)
	{
		extern __shared__ char sh_mem[];
		float* sh_depth_subArr = (float*)sh_mem;
		float* sh_depth_subArr_f = (float*)(sh_mem + (blockDim.x + 6)*(blockDim.y + 6)*sizeof(float));

		int dim_subArr = blockDim.x + 6; //read 3 more pixels at each side
		//load depth to shared memory
		for (int j = threadIdx.y; j < dim_subArr; j += blockDim.y)
		{
			for (int i = threadIdx.x; i < dim_subArr; i += blockDim.x)
			{
				int x = blockIdx.x*blockDim.x + i - 3; //x in depth map
				int y = blockIdx.y*blockDim.y + j - 3;
				if (0 <= x && x < depth_width &&
					0 <= y && y < depth_height)
				{
					unsigned short d = tex2DLayered(tex_depthImgs, x, y, vId);
					depth_extract_fg(d);

					sh_depth_subArr[j*dim_subArr + i] = d / 10.0f;
				}
				else
				{
					sh_depth_subArr[j*dim_subArr + i] = 0.0f;
				}
			}
		}
		__syncthreads();

		//bilateral filtering
		int dim_subArr_f = blockDim.x + 2;
		for (int j = threadIdx.y; j < blockDim.y + 2; j += blockDim.y)
		{
			for (int i = threadIdx.x; i < blockDim.x + 2; i += blockDim.x)
			{
				sh_depth_subArr_f[j*dim_subArr_f + i] = 0.0f;

				int x = i + 2; //x in subArr
				int y = j + 2;

				float d = sh_depth_subArr[y*dim_subArr + x];
				if (d > 0.0f)
				{
					float w_sum = 0.0f;
					float val = 0.0f;
#pragma unroll
					for (int n = 0; n < 5; n++)
#pragma unroll
						for (int m = 0; m < 5; m++)
						{
							int xx = x + m - 2;
							int yy = y + n - 2;
							float d_n = sh_depth_subArr[yy*dim_subArr + xx];

							if (d_n > 0.0f && fabs(d - d_n) < 4.0f)
							{
								float delta_d = d - d_n;
								delta_d = delta_d*delta_d;

								float dist2 = (m - 2)*(m - 2) + (n - 2)*(n - 2);

								float w = __expf(-delta_d / (sigma_d*sigma_d) - dist2 / (sigma_s*sigma_s));
								val += w * d_n;
								w_sum += w;
							}
						}
					if (w_sum > 0.0f)
						sh_depth_subArr_f[j*dim_subArr_f + i] = val / w_sum;
				}
			}
		}
		__syncthreads();

		cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
		float const&fx = K[0][0];
		float const&fy = K[1][1];
		float const&cx = K[0][2];
		float const&cy = K[1][2];

		//position in the image coordinate
		int i = blockIdx.x*blockDim.x + threadIdx.x;
		int j = blockIdx.y*blockDim.y + threadIdx.y;

		//cross production
		int x = threadIdx.x + 1; //location in subArr_f;
		int y = threadIdx.y + 1;
		cuda_vector_fixed<float, 3> dx(0.0f);
		cuda_vector_fixed<float, 3> dy(0.0f);
		cuda_vector_fixed<float, 3> normal(0.0f);
		float d = sh_depth_subArr_f[y*dim_subArr_f + x];
		if (d > 0.0f)
		{
			float d_l = sh_depth_subArr_f[y*dim_subArr_f + x - 1]; //left
			float d_r = sh_depth_subArr_f[y*dim_subArr_f + x + 1]; //right
			if (d_l > 0.0f && d_r > 0.0f && fabsf(d_l - d_r) < 4.0f)
			{
				dx[0] = ((i + 1) - cx)*d_r / fx - ((i - 1) - cx)*d_l / fx;
				dx[1] = (j - cy)*d_r / fy - (j - cy)*d_l / fy;
				dx[2] = d_r - d_l;
			}
			else if (d_r > 0.0f && d > 0.0f && fabsf(d_r - d) < 2.0f)
			{
				dx[0] = ((i + 1) - cx)*d_r / fx - (i - cx)*d / fx;
				dx[1] = (j - cy)*d_r / fy - (j - cy)*d / fy;
				dx[2] = d_r - d;
			}
			else if (d_l > 0.0f && d > 0.0f && fabsf(d_l - d) < 2.0f)
			{
				dx[0] = (i - cx)*d / fx - ((i - 1) - cx)*d_l / fx;
				dx[1] = (j - cy)*d / fy - (j - cy)*d_l / fy;
				dx[2] = d - d_l;
			}

			float d_t = sh_depth_subArr_f[(y - 1)*dim_subArr_f + x]; //top
			float d_b = sh_depth_subArr_f[(y + 1)*dim_subArr_f + x]; //bottom
			if (d_t > 0.0f && d_b > 0.0f && fabsf(d_t - d_b) < 4.0f)
			{
				dy[0] = (i - cx)*d_b / fx - (i - cx)*d_t / fx;
				dy[1] = ((j + 1) - cy)*d_b / fy - ((j - 1) - cy)*d_t / fy;
				dy[2] = d_b - d_t;
			}
			else if (d_t > 0.0f && d > 0.0f && fabsf(d_t - d) < 2.0f)
			{
				dy[0] = (i - cx)*d / fx - (i - cx)*d_t / fx;
				dy[1] = (j - cy)*d / fy - ((j - 1) - cy)*d_t / fy;
				dy[2] = d - d_t;
			}
			else if (d > 0.0f && d_b > 0.0f && fabsf(d_b - d) < 2.0f)
			{
				dy[0] = (i - cx)*d_b / fx - (i - cx)*d / fx;
				dy[1] = ((j + 1) - cy)*d_b / fy - (j - cy)*d / fy;
				dy[2] = d_b - d;
			}

			normal = cross_product(dx, dy);
			normal.normalize();
		}

		if (i < depth_width && j < depth_height)
		{
			float4 n = make_float4(normal[0], normal[1], normal[2], 0.0);
			surf2DLayeredwrite(n, surf_normalMaps, i*sizeof(float4), j, vId, cudaBoundaryModeClamp);
		}

		unsigned short d_f_short = ROUND(d * 10.0f);
		unsigned short d_ori = tex2DLayered(tex_depthImgs, i, j, vId);
		if (d_ori >= 0x8000)
			d_f_short |= 0x8000;
		surf2DLayeredwrite(d_f_short, surf_depthImgs_f, i * sizeof(unsigned short), j, vId, cudaBoundaryModeClamp);
	}

	void VolumetricFusionHelperCudaImpl::estimate_normal_for_depth_maps(bool bUpdateDepthToFilteredForFusion, bool bBilaterialFiltering)
	{
		for (int vId = 0; vId < this->num_depthmaps_; vId++)
		{
			int blk_size = 32;
			dim3 blkDim(blk_size, blk_size);
			dim3 gridDim((depth_width_ + blk_size - 1) / blk_size, (depth_height_ + blk_size - 1) / blk_size);
			if (bBilaterialFiltering)
			{
				float sigma_d = 2.0; //depth in cm
				float sigma_s = 2.0;
				int sh_mem_size = (blk_size + 6) * (blk_size + 6) * sizeof(float) * 2;
				depth_map_filter_calcNormal_kernel << <gridDim, blkDim, sh_mem_size >> >(depth_width_, depth_height_, vId, sigma_d, sigma_s);
			}
			else
			{
				int sh_mem_size = (blk_size + 2) * (blk_size + 2) * sizeof(float);
				depth_map_calcNormal_kernel << <gridDim, blkDim, sh_mem_size >> >(depth_width_, depth_height_, vId);
			}

			m_checkCudaErrors();
		}

		if (bUpdateDepthToFilteredForFusion && bBilaterialFiltering)
		{
			//bind the filtered data to depth texture
			bind_cuda_array_to_texture_depth(cu_3dArr_depth_f_);
		}
	}


	__global__ void UpdateVolumeKernel_Bayesian_WO_Deform(float *dev_vxl_data,
		int vxl_buf_size_,
		OccupcyCube const* dev_cubes,
		float x_offset, float y_offset, float z_offset,
		int cubes_num_x, int cubes_num_y, int cubes_num_z,
		float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
		int depth_width, int depth_height, float mu)
	{
		int cubeId = blockIdx.z * cubes_num_x*cubes_num_y + blockIdx.y*cubes_num_x + blockIdx.x;
		int data_offset = dev_cubes[cubeId].offset*vxls_per_cube;

		if (data_offset >= 0)
		{
			//voxel location
			float x_corner = blockIdx.x*cube_res + x_offset;
			float y_corner = blockIdx.y*cube_res + y_offset;
			float z_corner = blockIdx.z*cube_res + z_offset;

			int threads_per_block = blockDim.x*blockDim.y*blockDim.z;
			int chunk_size = vxls_per_cube / threads_per_block;
			for (int li = 0; li < chunk_size; li++)
			{
				cuda_vector_fixed<float, 3> V; // voxel location
				V[0] = x_corner + threadIdx.x*vxl_res;
				V[1] = y_corner + threadIdx.y*vxl_res;
				V[2] = z_corner + threadIdx.z*vxl_res + li*cube_res / chunk_size;
				int idx = (threadIdx.z + li*cube_size_in_vxl / chunk_size)*cube_size_in_vxl*cube_size_in_vxl +
					threadIdx.y * cube_size_in_vxl + threadIdx.x;
				float Ok = dev_vxl_data[data_offset + idx];

				for (int i = 0; i < dev_num_cam_views; i++)
				{
					cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V + dev_cam_views[i].cam_pose.T;

					if (X[2] > 0.1f)
					{
						cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
						float fx = K[0][0];
						float fy = K[1][1];
						float cx = K[0][2];
						float cy = K[1][2];

						int u = ROUND(fx * X[0] / X[2] + cx);
						int v = ROUND(fy * X[1] / X[2] + cy);

						if (u >= 0 && u < depth_width &&
							v >= 0 && v < depth_height)
						{
							unsigned short d = tex2DLayered(tex_depthImgs, u, v, i);
							depth_remove_top_bit(d);

							if (d > 0)
							{
								cuda_vector_fixed<float, 3> Pt;
								Pt[2] = d / 10.0f;
								Pt[0] = (u - cx)*Pt[2] / fx;
								Pt[1] = (v - cy)*Pt[2] / fy;

								float Mr = Pt.magnitude();
								float dxr = X.magnitude();

								float rho = mu / 2.0f;
								float Pk = 0.5f*erfcf((Mr - dxr) / sqrtf(2.0f) / rho) - 0.25f*erfcf((Mr - dxr + mu) / sqrtf(2.0f) / rho);
								if (Ok > 0.0f)
									Ok = Ok*Pk / (Ok*Pk + (1.0f - Ok)*(1.0f - Pk));
								else
									Ok = Pk;
							}
						}
					}
				}
				dev_vxl_data[data_offset + idx] = Ok;
			}
		}
	}


	//one sparse cube per cuda block
	//512 threads per cuda block
	//lunch a cuda block for all cubes including empty cubes
	__global__ void UpdateVolumeKernel_TSDF_WO_Deform_V1(float *dev_vxl_data,
		float *dev_vxl_weights,
		int vxl_buf_size_,
		OccupcyCube const* dev_cubes,
		float x_offset, float y_offset, float z_offset,
		int cubes_num_x, int cubes_num_y, int cubes_num_z,
		float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
		int depth_width, int depth_height, float mu)
	{
		__shared__ int offset;
		if (threadIdx.x == 0)
		{
			int cubeId = blockIdx.z * cubes_num_x*cubes_num_y + blockIdx.y*cubes_num_x + blockIdx.x;
			offset = dev_cubes[cubeId].offset;
		}
		__syncthreads();

		if (offset >= 0)
		{
			int data_offset = offset * vxls_per_cube;
			//corner of the cube
			float x_corner = blockIdx.x*cube_res + x_offset;
			float y_corner = blockIdx.y*cube_res + y_offset;
			float z_corner = blockIdx.z*cube_res + z_offset;

			for (int idx = threadIdx.x; idx < vxls_per_cube; idx += blockDim.x)
			{
				int xId = idx;
				int zId = xId / (cube_size_in_vxl*cube_size_in_vxl);
				xId -= zId*(cube_size_in_vxl*cube_size_in_vxl);
				int yId = xId / cube_size_in_vxl;
				xId -= yId*cube_size_in_vxl;

				cuda_vector_fixed<float, 3> V; // voxel location
				V[0] = x_corner + xId*vxl_res;
				V[1] = y_corner + yId*vxl_res;
				V[2] = z_corner + zId*vxl_res;

				float sdf_old = dev_vxl_data[data_offset + idx];
				float weight_old = dev_vxl_weights[data_offset + idx];

				float weight_new = 0.0;
				float sdf_new = 0.0;
				for (int i = 0; i < dev_num_cam_views; i++)
				{
					cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V + dev_cam_views[i].cam_pose.T;

					if (X[2] > 0.1f)
					{
						cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
						float const&fx = K[0][0];
						float const&fy = K[1][1];
						float const&cx = K[0][2];
						float const&cy = K[1][2];

						int u = ROUND(fx * X[0] / X[2] + cx);
						int v = ROUND(fy * X[1] / X[2] + cy);

						if (u >= 0 && u < depth_width &&
							v >= 0 && v < depth_height)
						{
							unsigned short ds = tex2DLayered(tex_depthImgs, u, v, i);
							depth_remove_top_bit(ds);

							if (ds > 0)
							{
								float sdf_cur = ds / 10.0f - X[2];

								if (-mu < sdf_cur && sdf_cur < mu)
								{
									float weight_cur = 1.0f;
									sdf_cur /= mu;
									sdf_new += sdf_cur * weight_cur;
									weight_new += weight_cur;
								}
							}
						}
					}
				}

				weight_new += weight_old;
				if (weight_new > 0.0f)
				{
					dev_vxl_data[data_offset + idx] = (sdf_old * weight_old + sdf_new) / weight_new;
					dev_vxl_weights[data_offset + idx] = weight_new;
				}
			}
		}
	}

	//one sparse cube per cuda block
	//512 threads per cuda block
	//only lunch cuda block for occupied cubes
	__global__ void UpdateVolumeKernel_TSDF_WO_Deform(float *dev_vxl_data, float *dev_vxl_weights, uchar4 *dev_vxl_colors, int vxl_buf_size_,
		int const* dev_buf_occupied_cube_ids, int buf_occupied_cube_ids_size,
		float3 cubes_offset,
		int3 cubes_num,
		float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
		int depth_width, int depth_height, float mu)
	{
		__shared__ float x_corner;
		__shared__ float y_corner;
		__shared__ float z_corner;
		__shared__ int cubeId;
		__shared__ int data_offset;

		if (threadIdx.x == 0 && blockIdx.x < buf_occupied_cube_ids_size)
		{
			cubeId = dev_buf_occupied_cube_ids[blockIdx.x];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_num.x*cubes_num.y);
			xCubeId -= zCubeId*cubes_num.x*cubes_num.y;
			int yCubeId = xCubeId / cubes_num.x;
			xCubeId -= yCubeId*cubes_num.x;

			data_offset = blockIdx.x * vxls_per_cube;
			//corner of the cube
			x_corner = xCubeId*cube_res + cubes_offset.x;
			y_corner = yCubeId*cube_res + cubes_offset.y;
			z_corner = zCubeId*cube_res + cubes_offset.z;
		}
		__syncthreads();

		if (blockIdx.x < buf_occupied_cube_ids_size)
		{
			if (0 <= cubeId && cubeId < cubes_num.x*cubes_num.y*cubes_num.z)
			{
				for (int idx = threadIdx.x; idx < vxls_per_cube; idx += blockDim.x)
				{
					int xId = idx;
					int zId = xId / (cube_size_in_vxl*cube_size_in_vxl);
					xId -= zId*(cube_size_in_vxl*cube_size_in_vxl);
					int yId = xId / cube_size_in_vxl;
					xId -= yId*cube_size_in_vxl;

					cuda_vector_fixed<float, 3> V; // voxel location
					V[0] = x_corner + xId*vxl_res;
					V[1] = y_corner + yId*vxl_res;
					V[2] = z_corner + zId*vxl_res;

					float sdf_old = dev_vxl_data[data_offset + idx];
					float weight_old = dev_vxl_weights[data_offset + idx];

					float weight_new = 0.0;
					float sdf_new = 0.0;
					float3 color_new = make_float3(0, 0, 0);
					float weight_color_new = 0.0;

#pragma unroll
					for (int i = 0; i < dev_num_cam_views; i++)
					{
						cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V + dev_cam_views[i].cam_pose.T;

						if (X[2] > 0.1f)
						{
							cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
							float const&fx = K[0][0];
							float const&fy = K[1][1];
							float const&cx = K[0][2];
							float const&cy = K[1][2];

							int u = ROUND(fx * X[0] / X[2] + cx);
							int v = ROUND(fy * X[1] / X[2] + cy);

							if (u >= 0 && u < depth_width &&
								v >= 0 && v < depth_height)
							{
								unsigned short ds = tex2DLayered(tex_depthImgs, u, v, i);
								depth_remove_top_bit(ds);

								float4 nd_ = tex2DLayered(tex_normalMaps, u, v, i);
								cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);


								if (ds > 0)
								{
									float sdf_cur = ds / 10.0f - X[2];

									if (-mu < sdf_cur && sdf_cur < mu)
									{
										cuda_vector_fixed<float, 3> p;
										p[2] = ds / 10.0;
										p[0] = (u - cx)*p[2] / fx;
										p[1] = (v - cy)*p[2] / fy;
										p.normalize();
										float weight_cur = MAX(0.0f, dot_product(p, nd));//1.0;
										sdf_cur /= mu;
										sdf_new += sdf_cur * weight_cur;
										weight_new += weight_cur;

										//pickup color
										uchar4 clr = tex2DLayered(tex_colorImgs, u, v, i);
										color_new.x += clr.z*weight_cur;
										color_new.y += clr.y*weight_cur;
										color_new.z += clr.x*weight_cur;
										weight_color_new += weight_cur;
									}
									else if (sdf_cur > mu)
									{
										float weight_cur = 0.5f;
										sdf_cur = 1.0f;
										sdf_new += sdf_cur * weight_cur;
										weight_new += weight_cur;
									}
								}
							}
						}
					}

					weight_new += weight_old;
					if (weight_new > M_EPS)
					{
						dev_vxl_data[data_offset + idx] = (sdf_old * weight_old + sdf_new) / weight_new;
						dev_vxl_weights[data_offset + idx] = weight_new;

						if (weight_color_new > M_EPS)
						{
							uchar4 clr = make_uchar4(color_new.x / weight_color_new, color_new.y / weight_color_new, color_new.z / weight_color_new, 0);
							dev_vxl_colors[data_offset + idx] = clr;
						}
					}
				}
			}
		}
	}


	//one cube per thread
	//find the neighboring ed nodes for each cube
	//save the indices
	__global__
		void UpdateVolumeKenerl_TSDF_vDeform_ngns(int* cube_ngns_indices,
		int2 *cube_ngn_indices_range,//starting point, num
		int const*dev_buf_occupied_cube_ids, int const* dev_occupied_cubes_count,
		float3 const* dev_cubes_offset, int3 const* dev_cubes_dim, float cube_res,
		int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets,
		float ed_cube_res, float ed_search_radius)
	{
		const int occupied_cubes_count = *dev_occupied_cubes_count;
		if (blockDim.x*blockIdx.x > occupied_cubes_count)
			return;

		__shared__ int sh_ngns_count;
		if (threadIdx.x == 0)
			sh_ngns_count = 0;
		__syncthreads();

		int idx = threadIdx.x + blockDim.x*blockIdx.x;
		int ngns_count = 0;
		int lc_offset = 0;

		int xEdCubeIdSt = 0;
		int yEdCubeIdSt = 0;
		int zEdCubeIdSt = 0;
		int xEdCubeIdEnd = 0;
		int yEdCubeIdEnd = 0;
		int zEdCubeIdEnd = 0;
		if (idx < occupied_cubes_count)
		{
			const float3 cubes_offset = *dev_cubes_offset;
			const int3 cubes_dim = *dev_cubes_dim;
			const int3 ed_cubes_dims = *dev_ed_cubes_dims;
			const float3 ed_cubes_offsets = *dev_ed_cubes_offsets;

			int cubeId = dev_buf_occupied_cube_ids[idx];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_dim.x*cubes_dim.y);
			xCubeId -= zCubeId*cubes_dim.x*cubes_dim.y;
			int yCubeId = xCubeId / cubes_dim.x;
			xCubeId -= yCubeId*cubes_dim.x;

			//cube center
			float pt_cen[3];
			pt_cen[0] = cubes_offset.x + (xCubeId + 0.5f)*cube_res;
			pt_cen[1] = cubes_offset.y + (yCubeId + 0.5f)*cube_res;
			pt_cen[2] = cubes_offset.z + (zCubeId + 0.5f)*cube_res;

			xEdCubeIdSt = MAX(0, (pt_cen[0] - ed_search_radius - ed_cubes_offsets.x) / ed_cube_res);
			yEdCubeIdSt = MAX(0, (pt_cen[1] - ed_search_radius - ed_cubes_offsets.y) / ed_cube_res);
			zEdCubeIdSt = MAX(0, (pt_cen[2] - ed_search_radius - ed_cubes_offsets.z) / ed_cube_res);
			xEdCubeIdEnd = MIN(ed_cubes_dims.x - 1, (pt_cen[0] + ed_search_radius - ed_cubes_offsets.x) / ed_cube_res);
			yEdCubeIdEnd = MIN(ed_cubes_dims.y - 1, (pt_cen[1] + ed_search_radius - ed_cubes_offsets.y) / ed_cube_res);
			zEdCubeIdEnd = MIN(ed_cubes_dims.z - 1, (pt_cen[2] + ed_search_radius - ed_cubes_offsets.z) / ed_cube_res);

			for (int k = zEdCubeIdSt; k <= zEdCubeIdEnd; k++)
				for (int j = yEdCubeIdSt; j <= yEdCubeIdEnd; j++)
					for (int i = xEdCubeIdSt; i <= xEdCubeIdEnd; i++)
					{
						short ndId = tex3D(tex_ndIds, i, j, k);
						if (ndId >= 0)
							ngns_count++;
					}
			if (ngns_count > 0)
				lc_offset = atomicAdd(&sh_ngns_count, ngns_count);
		}
		__syncthreads();

		if (threadIdx.x == 0)
			sh_ngns_count = atomicAdd(&dev_global_cube_ngns_count_sum, sh_ngns_count);
		__syncthreads();

		if (idx < occupied_cubes_count)
		{
			int offset = sh_ngns_count + lc_offset;
			int2 range = make_int2(offset, ngns_count);
			cube_ngn_indices_range[idx] = range;

			for (int k = zEdCubeIdSt; k <= zEdCubeIdEnd; k++)
				for (int j = yEdCubeIdSt; j <= yEdCubeIdEnd; j++)
					for (int i = xEdCubeIdSt; i <= xEdCubeIdEnd; i++)
					{
						short ndId = tex3D(tex_ndIds, i, j, k);
						if (ndId >= 0)
							cube_ngns_indices[offset++] = ndId;
					}
		}
	}

	//one sparse cube per cuda block
	//512 threads per cuda block
	//only lunch cuda block for occupied cubes
	//occupancy 100%: no gradient check
	__global__
		void UpdateVolumeKenerl_TSDF_vDeform_fusing(float* __restrict__ dev_vxl_data, float* __restrict__ dev_vxl_weights, int vxl_buf_size_,
		int const* __restrict__ dev_buf_occupied_cube_ids, int buf_occupied_cube_ids_size,
		float3 cubes_offset, int3 cube_dim,
		float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
		int const* __restrict__ cube_ngns_indices, int2 const* __restrict__ cube_ngn_indices_range,//starting point, num
		DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int ed_nodes_num, float sigma_vxl_node_dist,
		RigidTransformCuda const* __restrict__ dev_rigid_transf,
		int depth_width, int depth_height, float mu)
	{
		__shared__ float x_corner;
		__shared__ float y_corner;
		__shared__ float z_corner;
		__shared__ int cubeId;
		__shared__ int data_offset;
		__shared__ RigidTransformCuda sh_rigid_transf;

#define MAX_ED_NODES_PER_CUBE 390
		__shared__ int ngn_idx_range_st;
		__shared__ int sh_ed_nodes_num;
		__shared__ DeformGraphNodeCoreCuda sh_ed_nodes[MAX_ED_NODES_PER_CUBE];

		if (threadIdx.x == 0)
		{
			sh_rigid_transf = dev_rigid_transf[0];
		}

		if (threadIdx.x == 0 && blockIdx.x < buf_occupied_cube_ids_size)
		{
			cubeId = dev_buf_occupied_cube_ids[blockIdx.x];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cube_dim.x*cube_dim.y);
			xCubeId -= zCubeId*cube_dim.x*cube_dim.y;
			int yCubeId = xCubeId / cube_dim.x;
			xCubeId -= yCubeId*cube_dim.x;

			data_offset = blockIdx.x * vxls_per_cube;
			//corner of the cube
			x_corner = xCubeId*cube_res + cubes_offset.x;
			y_corner = yCubeId*cube_res + cubes_offset.y;
			z_corner = zCubeId*cube_res + cubes_offset.z;

			int2 ngn_idx_range = cube_ngn_indices_range[blockIdx.x];
			ngn_idx_range_st = ngn_idx_range.x;
			sh_ed_nodes_num = MIN(MAX_ED_NODES_PER_CUBE, ngn_idx_range.y);
		}
		__syncthreads();

		//load ed nodes
		for (int i = threadIdx.x; i < sh_ed_nodes_num * 9; i += blockDim.x)
		{
			int idx = i / 9;
			int ele_idx = i % 9;

			int ndId = cube_ngns_indices[ngn_idx_range_st + idx];
			sh_ed_nodes[idx].A(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A(ele_idx / 3, ele_idx % 3);

			if (ele_idx < 3)
			{
				sh_ed_nodes[idx].t[ele_idx] = dev_ed_nodes[ndId].t[ele_idx];
				sh_ed_nodes[idx].g[ele_idx] = dev_ed_nodes[ndId].g[ele_idx];
			}
		}
		__syncthreads();

		if (blockIdx.x < buf_occupied_cube_ids_size && sh_ed_nodes_num > 0)
		{
			if (0 <= cubeId && cubeId < cube_dim.x*cube_dim.y*cube_dim.z)
			{
				for (int idx = threadIdx.x; idx < vxls_per_cube; idx += blockDim.x)
				{
					int xId = idx;
					int zId = xId / (cube_size_in_vxl*cube_size_in_vxl);
					xId -= zId*(cube_size_in_vxl*cube_size_in_vxl);
					int yId = xId / cube_size_in_vxl;
					xId -= yId*cube_size_in_vxl;

					cuda_vector_fixed<float, 3> V; // voxel location
					V[0] = x_corner + xId*vxl_res;
					V[1] = y_corner + yId*vxl_res;
					V[2] = z_corner + zId*vxl_res;

					//find neighboring ed nodes
					float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
					int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						dists_sq[i] = 1.0e+10;
						ngn_idx[i] = -1;
					}

					for (int i = 0; i < sh_ed_nodes_num; i++)
					{
						float dist_sq = dist_square<3>(V.data_block(), sh_ed_nodes[i].g.data_block());
						if (dist_sq < 4.0f*sigma_vxl_node_dist*sigma_vxl_node_dist + mu*mu)
						{
							if (dist_sq < dists_sq[0])
							{
								dists_sq[0] = dist_sq;
								ngn_idx[0] = i;
							}

#pragma unroll
							for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
							{
								if (dist_sq < dists_sq[c])
								{
									dists_sq[c - 1] = dists_sq[c];
									ngn_idx[c - 1] = ngn_idx[c];
									dists_sq[c] = dist_sq;
									ngn_idx[c] = i;
								}
							}
						}
					}

					float w_sum = 0;
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						if (ngn_idx[i] != -1)
						{
							dists_sq[i] = expf(-dists_sq[i] / (2.0f*sigma_vxl_node_dist*sigma_vxl_node_dist));
							w_sum += dists_sq[i];
						}
					}

					//warp V
					if (w_sum > 1.0e-4)
					{
						float sdf_old = dev_vxl_data[data_offset + idx];
						float weight_old = dev_vxl_weights[data_offset + idx];

						//TODO: compute the gradient from the sdf field

						cuda_vector_fixed<float, 3> V_t(0.0f);
#pragma unroll
						for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
						{
							int &ndIdx_k = ngn_idx[k];
							if (ndIdx_k >= 0)
							{
								DeformGraphNodeCoreCuda const&nd = sh_ed_nodes[ndIdx_k];
								cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
								cuda_vector_fixed<float, 3> const&g = nd.g;
								cuda_vector_fixed<float, 3> const&t = nd.t;

								float &w_k = dists_sq[k];
								V_t += (w_k / w_sum)*(A*(V - g) + g + t);
							}
						}
						V_t = sh_rigid_transf.R*V_t + sh_rigid_transf.T;

						float weight_new = 0.0;
						float sdf_new = 0.0;

#pragma unroll
						for (int i = 0; i < dev_num_cam_views; i++)
						{
							cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*V_t + dev_cam_views[i].cam_pose.T;

							if (X[2] > 1.0e-5f)
							{
								cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
								float const&fx = K[0][0];
								float const&fy = K[1][1];
								float const&cx = K[0][2];
								float const&cy = K[1][2];

								float u = fx * X[0] / X[2] + cx;
								float v = fy * X[1] / X[2] + cy;

								if (u >= 0 && u < depth_width &&
									v >= 0 && v < depth_height)
								{
									unsigned short ds = tex2DLayered(tex_depthImgs, u, v, i);
									depth_remove_top_bit(ds);

									if (ds > 0)
									{
										float sdf_cur = ds / 10.0f - X[2];

										if (sdf_cur > -mu)
										{
											float weight_cur = 1.0f;
											sdf_cur = sdf_cur / mu;

											if (sdf_cur > 1.0f)
											{
												sdf_cur = 1.0f;
												weight_cur = 1.0f;
											}

											sdf_new += sdf_cur * weight_cur;
											weight_new += weight_cur;
										}

									}
								}
							}
						}

						weight_new += weight_old;
						if (weight_new > 1.0e-4f)
						{
							dev_vxl_data[data_offset + idx] = (sdf_old * weight_old + sdf_new) / weight_new;
							dev_vxl_weights[data_offset + idx] = MIN(SDF_WEIGHT_MAX, weight_new);
						}
					}
				}
			}
		}
	}

	//one sparse cube per cuda block
	//512 threads per cuda block
	//only lunch cuda block for occupied cubes
	//occupancy 50%: no gradient check
#define THRES_SDF_WEIGHTS_FOR_NORMAL 0.01f
	__global__
		void UpdateVolumeKenerl_TSDF_vDeform_fusing_vGradient(float* __restrict__ dev_vxl_data, float* __restrict__ dev_vxl_weights, int vxl_buf_size_,
		int const* __restrict__ dev_buf_occupied_cube_ids, int buf_occupied_cube_ids_size,
		OccupcyCube const* __restrict__ dev_cubes,
		float3 cubes_offset, int3 cube_dim,
		float cube_res, int cube_size_in_vxl, float vxl_res, int vxls_per_cube,
		int const* __restrict__ cube_ngns_indices, int2 const* __restrict__ cube_ngn_indices_range,//starting point, num
		DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int ed_nodes_num, float sigma_vxl_node_dist,
		RigidTransformCuda const* __restrict__ dev_rigid_transf,
		int const* __restrict__ dev_depth_maps_proj, int depth_width_prj, int depth_height_prj,
		int depth_width, int depth_height, float mu)
	{
		__shared__ float x_corner;
		__shared__ float y_corner;
		__shared__ float z_corner;
		__shared__ int cubeId;
		__shared__ int data_offset;
		__shared__ RigidTransformCuda sh_rigid_transf;

		__shared__ float sh_volume[9 * 9 * 9];

		__shared__ int ngn_idx_range_st;
		__shared__ int sh_ed_nodes_num;
		__shared__ DeformGraphNodeCoreCudaL2 sh_ed_nodes[MAX_ED_NODES_PER_CUBE];

		if (threadIdx.x == 0)
		{
			sh_rigid_transf = dev_rigid_transf[0];
		}

		if (threadIdx.x == 0 && blockIdx.x < buf_occupied_cube_ids_size)
		{
			cubeId = dev_buf_occupied_cube_ids[blockIdx.x];
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cube_dim.x*cube_dim.y);
			xCubeId -= zCubeId*cube_dim.x*cube_dim.y;
			int yCubeId = xCubeId / cube_dim.x;
			xCubeId -= yCubeId*cube_dim.x;

			data_offset = blockIdx.x * vxls_per_cube;
			//corner of the cube
			x_corner = xCubeId*cube_res + cubes_offset.x;
			y_corner = yCubeId*cube_res + cubes_offset.y;
			z_corner = zCubeId*cube_res + cubes_offset.z;

			int2 ngn_idx_range = cube_ngn_indices_range[blockIdx.x];
			ngn_idx_range_st = ngn_idx_range.x;
			sh_ed_nodes_num = MIN(MAX_ED_NODES_PER_CUBE, ngn_idx_range.y);

		}
		__syncthreads();

		//load ed nodes
		for (int i = threadIdx.x; i < sh_ed_nodes_num * 9; i += blockDim.x)
		{
			int idx = i / 9;
			int ele_idx = i % 9;

			int ndId = cube_ngns_indices[ngn_idx_range_st + idx];
			sh_ed_nodes[idx].A(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A(ele_idx / 3, ele_idx % 3);
			sh_ed_nodes[idx].A_inv_t(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A_inv_t(ele_idx / 3, ele_idx % 3);


			if (ele_idx < 3)
			{
				sh_ed_nodes[idx].t[ele_idx] = dev_ed_nodes[ndId].t[ele_idx];
				sh_ed_nodes[idx].g[ele_idx] = dev_ed_nodes[ndId].g[ele_idx];
				sh_ed_nodes[idx].n[ele_idx] = dev_ed_nodes[ndId].n[ele_idx];
			}
		}
		__syncthreads();

		if (blockIdx.x < buf_occupied_cube_ids_size && sh_ed_nodes_num > 0)
		{
			if (0 <= cubeId && cubeId < cube_dim.x*cube_dim.y*cube_dim.z)
			{
				//process different sub blocks 
				for (int xOffset = 0; xOffset < cube_size_in_vxl; xOffset += 8)
					for (int yOffset = 0; yOffset < cube_size_in_vxl; yOffset += 8)
						for (int zOffset = 0; zOffset < cube_size_in_vxl; zOffset += 8)
						{
							//load volume data to shared memory
							for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
							{
								//compute 3D idx for the source
								int xId = idx; //original 3d index on sh_volume
								int zId = idx / (9 * 9);
								xId -= zId * 9 * 9;
								int yId = xId / 9;
								xId -= yId * 9;

								xId += xOffset; //offset 
								yId += yOffset;
								zId += zOffset;

								if (xId < cube_size_in_vxl &&
									yId < cube_size_in_vxl &&
									zId < cube_size_in_vxl)
								{
									int pos = vxls_per_cube*blockIdx.x +
										zId*cube_size_in_vxl*cube_size_in_vxl +
										yId*cube_size_in_vxl + xId;
									float weight = dev_vxl_weights[pos];
									if (weight > THRES_SDF_WEIGHTS_FOR_NORMAL)
										sh_volume[idx] = dev_vxl_data[pos];
									else
										sh_volume[idx] = SDF_NULL_VALUE;
								}
								else
								{
									int cubeId_src = cubeId;
									if (xId >= cube_size_in_vxl)
									{
										cubeId_src += 1;
										xId -= cube_size_in_vxl;
									}
									if (yId >= cube_size_in_vxl)
									{
										cubeId_src += cube_dim.x;
										yId -= cube_size_in_vxl;
									}
									if (zId >= cube_size_in_vxl)
									{
										cubeId_src += cube_dim.x*cube_dim.y;
										zId -= cube_size_in_vxl;
									}
									if (cubeId_src >= cube_dim.x*cube_dim.y*cube_dim.z)
									{
										sh_volume[idx] = SDF_NULL_VALUE;
									}
									else
									{
										int offset = dev_cubes[cubeId_src].offset;

										if (offset < 0){
											sh_volume[idx] = SDF_NULL_VALUE;
										}
										else{
											int pos = vxls_per_cube*offset +
												zId*cube_size_in_vxl*cube_size_in_vxl +
												yId*cube_size_in_vxl + xId;
											float weight = dev_vxl_weights[pos];
											if (weight > THRES_SDF_WEIGHTS_FOR_NORMAL)
												sh_volume[idx] = dev_vxl_data[pos];
											else
												sh_volume[idx] = SDF_NULL_VALUE;
										}
									}
								}
							}
							__syncthreads();

							//process one voxel: find neighboring ed nodes; update sdf
							//grid id in the current block of the current cube
							int xId = threadIdx.x;
							int zId = xId / (8 * 8);
							xId -= zId*(8 * 8);
							int yId = xId / 8;
							xId -= yId * 8;

							cuda_vector_fixed<float, 3> V; // voxel location
							V[0] = x_corner + (xId + xOffset)*vxl_res;
							V[1] = y_corner + (yId + yOffset)*vxl_res;
							V[2] = z_corner + (zId + zOffset)*vxl_res;

							//find neighboring ed nodes
							float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
							int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
							{
								dists_sq[i] = 1.0e+10;
								ngn_idx[i] = -1;
							}

							for (int i = 0; i < sh_ed_nodes_num; i++)
							{
								float dist_sq = dist_square<3>(V.data_block(), sh_ed_nodes[i].g.data_block());
								if (dist_sq < 4.0f*sigma_vxl_node_dist*sigma_vxl_node_dist + mu*mu)
								{
									if (dist_sq < dists_sq[0])
									{
										dists_sq[0] = dist_sq;
										ngn_idx[0] = i;
									}

#pragma unroll
									for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
									{
										if (dist_sq < dists_sq[c])
										{
											dists_sq[c - 1] = dists_sq[c];
											ngn_idx[c - 1] = ngn_idx[c];
											dists_sq[c] = dist_sq;
											ngn_idx[c] = i;
										}
									}
								}
							}

							float w_sum = 0;
							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
							{
								if (ngn_idx[i] != -1)
								{
									dists_sq[i] = expf(-dists_sq[i] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
									w_sum += dists_sq[i];
								}
							}
							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
								if (ngn_idx[i] != -1)  dists_sq[i] /= w_sum;

							//warp V
							if (w_sum > 1.0e-4)
							{
								//idx in shared memory
								int idx_sh = zId * 9 * 9 + yId * 9 + xId;

								//idx in the current cube
								int idx = (zId + zOffset)*cube_size_in_vxl*cube_size_in_vxl + (yId + yOffset)*cube_size_in_vxl + (xId + xOffset);
								float sdf_old = dev_vxl_data[data_offset + idx];
								float weight_old = dev_vxl_weights[data_offset + idx];

								//compute the gradient from the sdf field
								float val_1 = sdf_old;
								float val_0, val_2;
								cuda_vector_fixed<float, 3> grad(0.0);
								int bValidNormal = 1;
								//compute dx
								val_2 = sh_volume[idx_sh + 1];
								if (xId > 0)
								{
									val_0 = sh_volume[idx_sh - 1];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[0] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[0] = val_1 - val_2;
									else
										bValidNormal = 0;
								}

								//compute dy
								val_2 = sh_volume[idx_sh + 9];
								if (yId > 0)
								{
									val_0 = sh_volume[idx_sh - 9];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[1] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[1] = val_1 - val_2;
									else
										bValidNormal = 0;
								}

								//compute dz
								val_2 = sh_volume[idx_sh + 81];
								if (zId > 0)
								{
									val_0 = sh_volume[idx_sh - 81];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[2] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[2] = val_1 - val_2;
									else
										bValidNormal = 0;
								}

								//get the normal from the ED node as an approximation
								if (bValidNormal == 0)
								{
									grad.fill(0.0);
									for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
									{
										int &ndIdx_k = ngn_idx[k];
										if (ndIdx_k >= 0)
										{
											DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
											float &w_k = dists_sq[k];
											grad += nd.n * w_k;
										}
									}
								}
								grad.normalize();

								cuda_vector_fixed<float, 3> V_t(0.0f);
								cuda_vector_fixed<float, 3> n_t(0.0f);
#pragma unroll
								for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
								{
									int &ndIdx_k = ngn_idx[k];
									if (ndIdx_k >= 0)
									{
										DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
										cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
										cuda_matrix_fixed<float, 3, 3> const&A_inv_t = nd.A_inv_t;
										cuda_vector_fixed<float, 3> const&g = nd.g;
										cuda_vector_fixed<float, 3> const&t = nd.t;

										float &w_k = dists_sq[k];
										V_t += w_k*(A*(V - g) + g + t);
										n_t += w_k*(A_inv_t*grad);
									}
								}
								V_t = sh_rigid_transf.R*V_t + sh_rigid_transf.T;
								n_t = sh_rigid_transf.R*n_t;
								n_t.normalize();

								float weight_new = 0.0;
								float sdf_new = 0.0;

#pragma unroll
								for (int vId = 0; vId < dev_num_cam_views; vId++)
								{
									cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*V_t + dev_cam_views[vId].cam_pose.T;

									if (X[2] > 0.1f)
									{
										cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
										float const&fx = K[0][0];
										float const&fy = K[1][1];
										float const&cx = K[0][2];
										float const&cy = K[1][2];

										float u_ = fx * X[0] / X[2] + cx;
										float v_ = fy * X[1] / X[2] + cy;
										int u = ROUND(u_);
										int v = ROUND(v_);

										if (u >= 0 && u < depth_width &&
											v >= 0 && v < depth_height)
										{
											unsigned short ds = tex2DLayered(tex_depthImgs, u, v, vId);
											depth_remove_top_bit(ds);


											if (ds > 0)
											{
												float sdf_cur = ds / 10.0f - X[2];
												float weight_cur = 0.0f;

												if (-mu < sdf_cur && sdf_cur < mu)
												{
													sdf_cur = sdf_cur / mu;
													weight_cur = 1.0f;

													//pick up the normal
													float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
													cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
													cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);

													{
														weight_cur = dot_product(n_t, nd_t);
														weight_cur = MAX(1.0f, (weight_cur + 1.0f) / 1.5f);
													}
												}
												else if (sdf_cur > mu)
												{
													sdf_cur = 1.0f;
													weight_cur = 1.0f;
												}
												sdf_new += sdf_cur * weight_cur;
												weight_new += weight_cur;
											}
										}
									}
								}

								if (weight_new > 1.0e-4f)
								{
									weight_new += weight_old;
									dev_vxl_data[data_offset + idx] = (sdf_old * weight_old + sdf_new) / weight_new;
									dev_vxl_weights[data_offset + idx] = MIN(SDF_WEIGHT_MAX, weight_new);
								}
							}
						}
			}
		}
	}

	template<int N>
	__device__ bool is_indices_overlapping(int const* indices_1, int const* lc_indices, int const *indices_lut)
	{
		int num = 0;
#pragma unroll
		for (int i = 0; i < N; i++)
		{
			int idx1 = indices_1[i];
#pragma unroll
			for (int j = 0; j < N; j++)
			{
				int lc_idx2 = lc_indices[j];
				if (lc_idx2 != -1)
				{
					int idx2 = indices_lut[lc_idx2];
					if (idx1 != -1 && idx1 == idx2)
						num++;
				}
			}
		}
		if (num >= 3)
			return true;
		else
			return false;
	}

	__global__
		__launch_bounds__(512, 2) //tell compiler that I want only 50% occupancy so that I can use 64 registers rather than 32
		void UpdateVolumeKenerl_TSDF_vDeform_fusing_vCollision(VolumeDataGPU volume,
		float cube_res, float vxl_res, int vxls_per_cube, int cube_size_in_vxl,
		int const* __restrict__ cube_ngns_indices, int2 const* __restrict__ cube_ngn_indices_range,//starting point, num
		DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, float const* __restrict__ dev_ed_nodes_align_residual,
		int const* dev_ed_nodes_num, float sigma_vxl_node_dist,
		RigidTransformCuda const* __restrict__ dev_rigid_transf,
		float const* __restrict__ dev_vts, int vt_dim,
		int const* __restrict__ vts_ngn_indices,
		int const* __restrict__ dev_depth_maps_proj, int const* __restrict__ dev_depth_maps_corr_vtIdx,
		float const* __restrict__ dev_depth_align_residual,
		int depth_width_prj, int depth_height_prj,
		int depth_width, int depth_height, float mu)
	{
		__shared__ float x_corner;
		__shared__ float y_corner;
		__shared__ float z_corner;
		__shared__ int cubeId;
		__shared__ int data_offset;
		__shared__ RigidTransformCuda sh_rigid_transf;

		__shared__ float sh_volume[9 * 9 * 9];

#define MAX_ED_NODES_PER_CUBE_V2 380
		__shared__ int ngn_idx_range_st;
		__shared__ int sh_ed_nodes_num;
		__shared__ DeformGraphNodeCoreCudaL2 sh_ed_nodes[MAX_ED_NODES_PER_CUBE_V2];
		__shared__ int sh_ed_node_indices[MAX_ED_NODES_PER_CUBE_V2];
		__shared__ float sh_ed_node_align_residual[MAX_ED_NODES_PER_CUBE_V2];

		if (threadIdx.x == 0)
		{
			sh_rigid_transf = dev_rigid_transf[0];
		}

		const int occupied_cubes_num = MIN(TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX, volume.count_occu_cubes[0]);
		const int3 cube_dim = *(volume.cubes_num);

		for (int blkId = blockIdx.x; blkId < occupied_cubes_num; blkId += gridDim.x)
		{
			if (threadIdx.x == 0)
			{
				cubeId = volume.occupied_cube_ids[blkId];
				const float3 cubes_offset = *(volume.cubes_offset);

				int xCubeId = cubeId;
				int zCubeId = xCubeId / (cube_dim.x*cube_dim.y);
				xCubeId -= zCubeId*cube_dim.x*cube_dim.y;
				int yCubeId = xCubeId / cube_dim.x;
				xCubeId -= yCubeId*cube_dim.x;

				data_offset = blkId * vxls_per_cube;
				//corner of the cube
				x_corner = xCubeId*cube_res + cubes_offset.x;
				y_corner = yCubeId*cube_res + cubes_offset.y;
				z_corner = zCubeId*cube_res + cubes_offset.z;

				int2 ngn_idx_range = cube_ngn_indices_range[blkId];
				ngn_idx_range_st = ngn_idx_range.x;
				sh_ed_nodes_num = MIN(MAX_ED_NODES_PER_CUBE_V2, ngn_idx_range.y);
			}
			__syncthreads();

			//load ed nodes
			for (int i = threadIdx.x; i < sh_ed_nodes_num * 9; i += blockDim.x)
			{
				int idx = i / 9;
				int ele_idx = i % 9;

				int ndId = cube_ngns_indices[ngn_idx_range_st + idx];
				sh_ed_nodes[idx].A(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A(ele_idx / 3, ele_idx % 3);
				sh_ed_nodes[idx].A_inv_t(ele_idx / 3, ele_idx % 3) = dev_ed_nodes[ndId].A_inv_t(ele_idx / 3, ele_idx % 3);

				if (ele_idx == 0)
				{
					sh_ed_node_indices[idx] = ndId;
					sh_ed_node_align_residual[idx] = dev_ed_nodes_align_residual[ndId];
				}

				if (ele_idx < 3)
				{
					sh_ed_nodes[idx].t[ele_idx] = dev_ed_nodes[ndId].t[ele_idx];
					sh_ed_nodes[idx].g[ele_idx] = dev_ed_nodes[ndId].g[ele_idx];
					sh_ed_nodes[idx].n[ele_idx] = dev_ed_nodes[ndId].n[ele_idx];
				}
			}
			__syncthreads();

			if (sh_ed_nodes_num > 0 &&
				0 <= cubeId && cubeId < cube_dim.x*cube_dim.y*cube_dim.z)
			{
				//process different sub blocks 
				for (int xOffset = 0; xOffset < cube_size_in_vxl; xOffset += 8)
					for (int yOffset = 0; yOffset < cube_size_in_vxl; yOffset += 8)
						for (int zOffset = 0; zOffset < cube_size_in_vxl; zOffset += 8)
						{
							//load volume data to shared memory
							for (int idx = threadIdx.x; idx < 9 * 9 * 9; idx += blockDim.x)
							{
								//compute 3D idx for the source
								int xId = idx; //original 3d index on sh_volume
								int zId = idx / (9 * 9);
								xId -= zId * 9 * 9;
								int yId = xId / 9;
								xId -= yId * 9;

								xId += xOffset; //offset 
								yId += yOffset;
								zId += zOffset;

								if (xId < cube_size_in_vxl &&
									yId < cube_size_in_vxl &&
									zId < cube_size_in_vxl)
								{
									int pos = vxls_per_cube*blkId +
										zId*cube_size_in_vxl*cube_size_in_vxl +
										yId*cube_size_in_vxl + xId;
									float weight = volume.weights[pos];
									if (weight > THRES_SDF_WEIGHTS_FOR_NORMAL)
										sh_volume[idx] = volume.data[pos];
									else
										sh_volume[idx] = SDF_NULL_VALUE;
								}
								else
								{
									int cubeId_src = cubeId;
									if (xId >= cube_size_in_vxl)
									{
										cubeId_src += 1;
										xId -= cube_size_in_vxl;
									}
									if (yId >= cube_size_in_vxl)
									{
										cubeId_src += cube_dim.x;
										yId -= cube_size_in_vxl;
									}
									if (zId >= cube_size_in_vxl)
									{
										cubeId_src += cube_dim.x*cube_dim.y;
										zId -= cube_size_in_vxl;
									}
									if (cubeId_src >= cube_dim.x*cube_dim.y*cube_dim.z)
									{
										sh_volume[idx] = SDF_NULL_VALUE;
									}
									else
									{
										int offset = volume.cubes[cubeId_src].offset;

										if (offset < 0){
											sh_volume[idx] = SDF_NULL_VALUE;
										}
										else{
											int pos = vxls_per_cube*offset +
												zId*cube_size_in_vxl*cube_size_in_vxl +
												yId*cube_size_in_vxl + xId;
											float weight = volume.weights[pos];
											if (weight > THRES_SDF_WEIGHTS_FOR_NORMAL)
												sh_volume[idx] = volume.data[pos];
											else
												sh_volume[idx] = SDF_NULL_VALUE;
										}
									}
								}
							}
							__syncthreads();

							//process one voxel: find neighboring ed nodes; update sdf
							//grid id in the current block of the current cube
							int xId = threadIdx.x;
							int zId = xId / (8 * 8);
							xId -= zId*(8 * 8);
							int yId = xId / 8;
							xId -= yId * 8;

							cuda_vector_fixed<float, 3> V; // voxel location
							V[0] = x_corner + (xId + xOffset)*vxl_res;
							V[1] = y_corner + (yId + yOffset)*vxl_res;
							V[2] = z_corner + (zId + zOffset)*vxl_res;

							//find neighboring ed nodes
							float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
							int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
							{
								dists_sq[i] = 1.0e+10;
								ngn_idx[i] = -1;
							}

							for (int i = 0; i < sh_ed_nodes_num; i++)
							{
								float dist_sq = dist_square<3>(V.data_block(), sh_ed_nodes[i].g.data_block());
								if (dist_sq < 4.0f*sigma_vxl_node_dist*sigma_vxl_node_dist + mu*mu)
								{
									if (dist_sq < dists_sq[0])
									{
										dists_sq[0] = dist_sq;
										ngn_idx[0] = i;
									}

#pragma unroll
									for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
									{
										if (dist_sq < dists_sq[c])
										{
											dists_sq[c - 1] = dists_sq[c];
											ngn_idx[c - 1] = ngn_idx[c];
											dists_sq[c] = dist_sq;
											ngn_idx[c] = i;
										}
									}
								}
							}

							float w_sum = 0;
							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
							{
								if (ngn_idx[i] != -1)
								{
									dists_sq[i] = expf(-dists_sq[i] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
									w_sum += dists_sq[i];
								}
							}

							for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
								if (ngn_idx[i] != -1)  dists_sq[i] /= w_sum;

							//warp V
							if (w_sum > 1.0e-4f)
							{
								//idx in shared memory
								int idx_sh = zId * 9 * 9 + yId * 9 + xId;

								//idx in the current cube
								int idx = (zId + zOffset)*cube_size_in_vxl*cube_size_in_vxl + (yId + yOffset)*cube_size_in_vxl + (xId + xOffset);
								float sdf_old = volume.data[data_offset + idx];
								float weight_old = volume.weights[data_offset + idx];

								//compute the gradient from the sdf field
								float val_1 = sdf_old;
								float val_0, val_2;
								cuda_vector_fixed<float, 3> grad(0.0);
								int bValidNormal = 1;
								//compute dx
								val_2 = sh_volume[idx_sh + 1];
								if (xId > 0)
								{
									val_0 = sh_volume[idx_sh - 1];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[0] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[0] = val_1 - val_2;
									else
										bValidNormal = 0;
								}

								//compute dy
								val_2 = sh_volume[idx_sh + 9];
								if (yId > 0)
								{
									val_0 = sh_volume[idx_sh - 9];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[1] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[1] = val_1 - val_2;
									else
										bValidNormal = 0;
								}

								//compute dz
								val_2 = sh_volume[idx_sh + 81];
								if (zId > 0)
								{
									val_0 = sh_volume[idx_sh - 81];
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[2] = (val_0 - val_2) / 2.0f;
									else
										bValidNormal = 0;
								}
								else
								{
									if (-0.95f < val_0 && val_0 < 0.95f && -0.95f < val_2 && val_2 < 0.95f)
										grad[2] = val_1 - val_2;
									else
										bValidNormal = 0;
								}
								grad.normalize();


								cuda_vector_fixed<float, 3> cen_eds(0.0f);
								cuda_vector_fixed<float, 3> n_eds(0.0f);
								cuda_vector_fixed<float, 3> cen_eds_t(0.0f);
								cuda_vector_fixed<float, 3> n_eds_t(0.0f);
#pragma unroll
								for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
								{
									int &ndIdx_k = ngn_idx[k];
									if (ndIdx_k >= 0)
									{
										DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
										float w_k = dists_sq[k];
										n_eds += nd.n * w_k;
										cen_eds += nd.g * w_k;
										n_eds_t += nd.A_inv_t*nd.n*w_k;
										cen_eds_t += (nd.g + nd.t)*w_k;
									}
								}
								n_eds.normalize();
								n_eds_t.normalize();

								cuda_vector_fixed<float, 3> V_t(0.0f);
								cuda_vector_fixed<float, 3> n_t(0.0f);
#pragma unroll
								for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
								{
									int &ndIdx_k = ngn_idx[k];
									if (ndIdx_k >= 0)
									{
										DeformGraphNodeCoreCudaL2 const&nd = sh_ed_nodes[ndIdx_k];
										cuda_matrix_fixed<float, 3, 3> const&A = nd.A;
										cuda_matrix_fixed<float, 3, 3> const&A_inv_t = nd.A_inv_t;
										cuda_vector_fixed<float, 3> const&g = nd.g;
										cuda_vector_fixed<float, 3> const&t = nd.t;

										float &w_k = dists_sq[k];
										V_t += w_k*(A*(V - g) + g + t);
										n_t += w_k*(A_inv_t*grad);
									}
								}
								V_t = sh_rigid_transf.R*V_t + sh_rigid_transf.T;
								n_t = sh_rigid_transf.R*n_t;
								n_t.normalize();

								if (!bValidNormal)
									n_t = n_eds_t;

								float vxl_align_residual = 0.0f;
#pragma unroll
								for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
								{
									int &ndIdx_k = ngn_idx[k];
									if (ndIdx_k >= 0)
									{
										float &w_k = dists_sq[k];
										vxl_align_residual = MAX(sh_ed_node_align_residual[ndIdx_k], vxl_align_residual);
									}
								}

								float weight_new = 0.0;
								float sdf_new = 0.0;
								float align_residual_sum = 0.0;
								int align_residual_count = 0;
								float3 color_new = make_float3(0, 0, 0);
								float weight_color_new = 0;

#pragma unroll
								for (int vId = 0; vId < dev_num_cam_views; vId++)
								{
									cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*V_t + dev_cam_views[vId].cam_pose.T;

									if (X[2] > 0.1f)
									{
										cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
										float const&fx = K[0][0];
										float const&fy = K[1][1];
										float const&cx = K[0][2];
										float const&cy = K[1][2];

										float u_ = fx * X[0] / X[2] + cx;
										float v_ = fy * X[1] / X[2] + cy;
										int u = ROUND(u_);
										int v = ROUND(v_);

										if (u >= 0 && u < depth_width &&
											v >= 0 && v < depth_height)
										{
											unsigned short ds = tex2DLayered(tex_depthImgs, u, v, vId);
											depth_remove_top_bit(ds);


											if (ds > 0)
											{
												float sdf_cur = ds / 10.0f - X[2];
												float weight_cur = 0.0f;

												if (-mu < sdf_cur && sdf_cur < mu)
												{
													sdf_cur = sdf_cur / mu;
													//weight_cur = 1.0f;
													//pick up the normal
													float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
													cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
													cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);

													weight_cur = (dot_product(n_t, nd_t) + 0.5f) / 1.5f;

													//check if the associated depth being picked up by a surface point on model
													int u_proj = MAX(0, MIN(depth_width_prj - 1, ROUND(u_ * depth_width_prj / depth_width)));
													int v_proj = MAX(0, MIN(depth_height_prj - 1, ROUND(v_ * depth_height_prj / depth_height)));
													int d_proj = dev_depth_maps_proj[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
													float residual = dev_depth_align_residual[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
													if (residual >= 0)
													{
														align_residual_sum += residual;
														align_residual_count++;
													}
													if (fabsf(d_proj - ds) < (mu + 0.3f)*10.0f) //TODO: check normal
													{
														//the depth pixel is taken
														int vtIdx = dev_depth_maps_corr_vtIdx[vId*depth_height_prj*depth_width_prj + v_proj*depth_width_prj + u_proj];
														float const* vt_ref_1 = dev_vts + vtIdx*vt_dim;

														//TODO: move outside of for loop
														cuda_vector_fixed<float, 3> vt_ref_2;
														if (bValidNormal && -1.0f < sdf_old && sdf_old < 1.0f)
															vt_ref_2 = V + grad*sdf_old*mu;
														else
															vt_ref_2 = V + dot_product(n_eds, cen_eds - V)*n_eds;


														if (!is_indices_overlapping<VXL_NEIGHBOR_EDNODE_NUM>(&(vts_ngn_indices[vtIdx*VXL_NEIGHBOR_EDNODE_NUM]),
															&(ngn_idx[0]), sh_ed_node_indices))
														{
															weight_cur = 0.0f;
														}
													}
													else
													{
														//the depth pixel is new
														cuda_vector_fixed<float, 3> vt_t_depth;
														vt_t_depth[2] = ds / 10.0f;
														vt_t_depth[0] = (u - cx)*vt_t_depth[2] / fx;
														vt_t_depth[1] = (v - cy)*vt_t_depth[2] / fy;

														cuda_vector_fixed<float, 3> vt_t_vxl; //surface point associated with the voxel
														if (bValidNormal && -1.0f < sdf_old && sdf_old < 1.0f)
														{
															vt_t_vxl = V_t + n_t*sdf_old*mu;

															if (dist_square<3>(vt_t_depth.data_block(), vt_t_vxl.data_block()) > 1.0f*1.0f)
															{
																weight_cur = 0.0f;
															}
														}
														else
														{
															vt_t_vxl = V_t + dot_product(n_eds_t, cen_eds_t - V_t)*n_eds_t;
															float dist2 = dist_square<3>(vt_t_depth.data_block(), vt_t_vxl.data_block());

															if (dist2 > 4.0f*4.0f)
															{
																weight_cur = 0.0f;
															}
														}

													}

													//pickup color
													uchar4 clr = tex2DLayered(tex_colorImgs, u, v, vId);
													color_new.x += clr.z*weight_cur;
													color_new.y += clr.y*weight_cur;
													color_new.z += clr.x*weight_cur;
													weight_color_new += weight_cur;

												}
												else if (sdf_cur > mu)
												{
													sdf_cur = 1.0f;
													weight_cur = 0.8f;
												}

												sdf_new += sdf_cur * weight_cur;
												weight_new += weight_cur;
											}
										}
									}
								}

								if (weight_new > 1.0e-4f)
								{
									if (vxl_align_residual > 0.7f)
									{
										volume.data[data_offset + idx] = sdf_new / weight_new;
										volume.weights[data_offset + idx] = weight_new;

										uchar4 clr = make_uchar4(0, 0, 0, 0);
										if (weight_color_new > M_EPS)
										{
											clr.x = color_new.x / weight_color_new;
											clr.y = color_new.y / weight_color_new;
											clr.z = color_new.z / weight_color_new;
										}
										volume.colors[data_offset + idx] = clr;
									}
									else
									{
										weight_new += weight_old;
										volume.data[data_offset + idx] = (sdf_old * weight_old + sdf_new) / weight_new;
										volume.weights[data_offset + idx] = MIN(SDF_WEIGHT_MAX, weight_new);
										if (weight_color_new > M_EPS)
										{
											uchar4 clr_old = volume.colors[data_offset + idx];
											uchar4 clr;
											clr.x = (clr_old.x * weight_old * 0.1f + color_new.x) / (weight_old*0.1f + weight_color_new);
											clr.y = (clr_old.y * weight_old * 0.1f + color_new.y) / (weight_old*0.1f + weight_color_new);
											clr.z = (clr_old.z * weight_old * 0.1f + color_new.z) / (weight_old*0.1f + weight_color_new);
											clr.w = 0;
											volume.colors[data_offset + idx] = clr;
										}
									}
								}
							}
						}
			}
			__syncthreads();
		}
	}

	void VolumetricFusionHelperCudaImpl::
		update_volume_vWarp(VolumeTwoLevelHierachy *volume,
		DeformGraphNodeCuda const*dev_ed_nodes, float const* dev_ed_nodes_align_residual, int const* dev_ed_nodes_num, float sigma_vxl_node_dist,
		int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res, RigidTransformCuda const* dev_rigid_transf,
		int const* vts_ngn_indices,
		int const* dev_depth_maps_proj, int const* dev_depth_maps_corr_vtIdx, float const* dev_depth_align_residual,
		int depth_width_prj, int depth_height_prj)
	{
		float cube_res = volume->cube_res;
		float ed_search_radius = sqrt((volume->mu + sqrt(3.0)*cube_res / 2.0)*(volume->mu + sqrt(3.0)*cube_res / 2.0) +
			(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)*(2.0*sigma_vxl_node_dist + sqrt(3.0)*cube_res / 2.0)
			);
		int ed_nodes_num_per_cube_est = 2.0*(ed_search_radius*2.0 / ed_cube_res)*(ed_search_radius*2.0 / ed_cube_res);

		static cuda::PinnedMemory<int> count(0);
		checkCudaErrors(cudaMemcpyToSymbolAsync(dev_global_cube_ngns_count_sum, count.memory, sizeof(int)));

		//group the ed nodes for each occupied cube
		int threads_per_block = MAX_THREADS_PER_BLOCK;
		int blocks_per_grid = (TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX + threads_per_block - 1) / threads_per_block;
		UpdateVolumeKenerl_TSDF_vDeform_ngns << <blocks_per_grid, threads_per_block >> >(dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
			volume->buf_occupied_cube_ids, volume->gpu_cubes_occpied_count.dev_ptr,
			volume->ptr_cubes_offset,
			volume->ptr_cubes_num,
			volume->cube_res,
			dev_ed_cubes_dims, dev_ed_cubes_offsets, ed_cube_res, ed_search_radius);
		m_checkCudaErrors();

		if (LOGGER()->check_verbosity(Logger::Debug))
		{
			int cube_ngn_indices_num = 0;
			cudaMemcpyFromSymbol(&cube_ngn_indices_num, dev_global_cube_ngns_count_sum, sizeof(int));
			LOGGER()->debug("<<<<<<<<<<<<< cube_ngn_indices_num = %d/%d", cube_ngn_indices_num, cube_ngn_indices_buf_size_);
		}


		//update the volume
		threads_per_block = 512;
		blocks_per_grid = 256; //any number you like
		VolumeDataGPU volume_gpu(*volume);
		UpdateVolumeKenerl_TSDF_vDeform_fusing_vCollision << <blocks_per_grid, threads_per_block >> >(volume_gpu,
			volume->cube_res, volume->vxl_res, volume->vxls_per_cube, volume->cube_size_in_voxel,
			dev_cube_ngns_indices_, dev_cube_ngn_indices_range_,
			dev_ed_nodes, dev_ed_nodes_align_residual, dev_ed_nodes_num, sigma_vxl_node_dist, dev_rigid_transf,
			dev_vts_buf_, vts_dim_, vts_ngn_indices,
			dev_depth_maps_proj, dev_depth_maps_corr_vtIdx, dev_depth_align_residual, depth_width_prj, depth_height_prj,
			depth_width_, depth_height_, volume->mu);
		m_checkCudaErrors();
	}

	__device__ bool isCubeOverlappingDepth(float x, float y, float r_h, float r_v, int texId, int z_max, int z_min)
	{
		for (int i = x - r_h; i < x + r_h; i += 1)
		{
			for (int j = y - r_h; j < y + r_h; j += 1)
			{
				unsigned short d = tex2DLayered(tex_depthImgs, i, j, texId);
				depth_extract_fg(d);

				if (d>z_min &&d < z_max)
				{
					return true;
				}
			}
		}

		return false;
	}



#define THREADS_PER_BLOCK_CUBEPRUNE 256
#define CUBEPRUNE_NDIMS 3 //3D data

	//each thread handles a cube: warp 8 vertices, project them, and find the occupancy
	//only check a cube that neighbors an occupied cube
	__global__
		void OccupcyCubePruneKernel_vDeform(OccupcyCube* dev_cubes,
		int * dev_buf_occupied_cube_ids,
		int *dev_global_count_occu_cubes,
		float3 cubes_offset, int3 cubes_dim, float cube_res,
		DeformGraphNodeCuda const*dev_ed_nodes, int ed_nodes_num, float sigma_vxl_node_dist,
		int3 ed_cubes_dims, float3 ed_cubes_offsets, float ed_cube_res, int ed_search_radius,
		RigidTransformCuda const* dev_rigid_transf,
		int width_depth, int height_depth, float mu)
	{
		__shared__  cuda_vector_fixed<float, 3> sh_cube_vts[THREADS_PER_BLOCK_CUBEPRUNE * 8];
		__shared__ int sh_offset;
		__shared__ RigidTransformCuda sh_rigid_transf;
		int cubeId = threadIdx.x + blockDim.x*blockIdx.x;

		if (threadIdx.x == 0)
		{
			sh_rigid_transf = dev_rigid_transf[0];
		}
		__syncthreads();

		bool bOccupied = false;
		if (dev_cubes[cubeId].offset < 0)
		{
			int xCubeId = cubeId;
			int zCubeId = xCubeId / (cubes_dim.x*cubes_dim.y);
			xCubeId -= zCubeId*cubes_dim.x*cubes_dim.y;
			int yCubeId = xCubeId / cubes_dim.x;
			xCubeId -= yCubeId*cubes_dim.x;

			bool bProcess = false;
			//check neighboring cell
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					for (int k = -1; k <= 1; k++)
					{
						int xId = xCubeId + i;
						int yId = yCubeId + j;
						int zId = zCubeId + k;
						if (xId >= 0 && yId >= 0 && zId >= 0 &&
							xId < cubes_dim.x && yId < cubes_dim.y && zId < cubes_dim.z)
						{
							int id = zId*cubes_dim.x*cubes_dim.y + yId*cubes_dim.x + xId;
							if (dev_cubes[id].offset >= 0)
								bProcess = true;
						}
					}

			const int dx[] = { 0, 0, 0, 0, 1, 1, 1, 1 };
			const int dy[] = { 0, 0, 1, 1, 0, 0, 1, 1 };
			const int dz[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

			if (bProcess)
			{
				//warp the corner
				for (int cId = 0; cId < 8; cId++) //corner id
				{
					cuda_vector_fixed<float, 3> vt;
					vt[0] = cubes_offset.x + (xCubeId + dx[cId]) * cube_res;
					vt[1] = cubes_offset.y + (yCubeId + dy[cId]) * cube_res;
					vt[2] = cubes_offset.z + (zCubeId + dz[cId]) * cube_res;

					int xEdCubeId = (vt[0] - ed_cubes_offsets.x) / ed_cube_res;
					int yEdCubeId = (vt[1] - ed_cubes_offsets.y) / ed_cube_res;
					int zEdCubeId = (vt[2] - ed_cubes_offsets.z) / ed_cube_res;

					float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
					int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
					for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
					{
						dists_sq[i] = 1.0e+10f;
						ngn_idx[i] = -1;
					}

					for (int k = -ed_search_radius; k <= ed_search_radius; k++)
						for (int j = -ed_search_radius; j <= ed_search_radius; j++)
							for (int i = -ed_search_radius; i <= ed_search_radius; i++)
							{
								int xId = xEdCubeId + i;
								int yId = yEdCubeId + j;
								int zId = zEdCubeId + k;

								if (xId >= 0 && yId >= 0 && zId >= 0 &&
									xId < ed_cubes_dims.x && yId < ed_cubes_dims.y && zId < ed_cubes_dims.z)
								{
									short ndId = tex3D(tex_ndIds, xId, yId, zId);
									if (ndId >= 0)
									{
										cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndId].g;
										float dist_sq = dist_square<3>(vt.data_block(), g.data_block());

										if (dist_sq < dists_sq[0])
										{
											dists_sq[0] = dist_sq;
											ngn_idx[0] = ndId;
										}

#pragma unroll
										for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
										{
											if (dist_sq < dists_sq[c])
											{
												dists_sq[c - 1] = dists_sq[c];
												ngn_idx[c - 1] = ngn_idx[c];
												dists_sq[c] = dist_sq;
												ngn_idx[c] = ndId;
											}
										}
									}
								}
							}

					float w_sum = 0;
#pragma unroll
					for (int c = 0; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
					{
						if (ngn_idx[c] != -1)
						{
							dists_sq[c] = expf(-dists_sq[c] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
							w_sum += dists_sq[c];
						}
					}

					cuda_vector_fixed<float, 3> vt_t(0.0);
#pragma unroll
					for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
					{
						int &ndIdx_k = ngn_idx[k];
						float &w_k = dists_sq[k];
						if (ndIdx_k >= 0)
						{
							DeformGraphNodeCuda nd = dev_ed_nodes[ndIdx_k];
							cuda_matrix_fixed<float, 3, 3> &A = nd.A;
							cuda_vector_fixed<float, 3> &g = nd.g;
							cuda_vector_fixed<float, 3> &t = nd.t;

							vt_t += w_k / w_sum*(A*(vt - g) + g + t);
						}
					}
					vt_t = sh_rigid_transf.R*vt_t + sh_rigid_transf.T;


					sh_cube_vts[threadIdx.x * 8 + cId] = vt_t;
				}

				float u_max, u_min;
				float v_max, v_min;
				float w_max, w_min;
				for (int vId = 0; vId < dev_num_cam_views; vId++)
				{
					u_max = 0.0f;
					u_min = 1.0e+10f;
					v_max = 0.0f;
					v_min = 1.0e+10f;
					w_max = 0.0f;
					w_min = 1.0e+10f;
					for (int cId = 0; cId < 8; cId++)
					{
						cuda_vector_fixed<float, 3> &P_wld = sh_cube_vts[threadIdx.x * 8 + cId];

						cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*P_wld + dev_cam_views[vId].cam_pose.T;
						cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
						float const&fx = K[0][0];
						float const&fy = K[1][1];
						float const&cx = K[0][2];
						float const&cy = K[1][2];
						float u = fx * X[0] / X[2] + cx;
						float v = fy * X[1] / X[2] + cy;
						float w = X[2];

						if (u > u_max) u_max = u;
						if (u < u_min) u_min = u;
						if (v > v_max) v_max = v;
						if (v < v_min) v_min = v;
						if (w > w_max) w_max = w;
						if (w < w_min) w_min = w;
					}

					w_max *= 10.0f;
					w_min *= 10.0f; //cm to mm
					float r_h = u_max - u_min;
					float r_v = v_max - v_min;
					float level = MAX(r_h, r_v);
					level = MAX(2.0f, floorf(log2f(level)));
					r_h /= 2.0f;
					r_v /= 2.0f;
					ushort2 mipmap = pick_mipmap9((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level, width_depth, height_depth/*, bPrint*/);
					if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
					{
						ushort2 mipmap = pick_mipmap25((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level - 1.0f, width_depth, height_depth/*, bPrint*/);
						if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
						{
							bOccupied = true;
						}
					}
				}//end of for-vId

			}//end of bProcess
		}

		if (bOccupied)
		{
			int offset = atomicAdd(dev_global_count_occu_cubes, 1);
			dev_buf_occupied_cube_ids[offset] = cubeId;
		}
	}


	__global__
		void OccupcyCubePruneKernel_vDeform2(OccupcyCube* dev_cubes,
		int *dev_buf_occupied_cube_ids,
		int *dev_global_count_occu_cubes,
		float3 const* dev_cubes_offset, int3 const* dev_cubes_dim, float cube_res,
		DeformGraphNodeCuda const*dev_ed_nodes, float sigma_vxl_node_dist,
		int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res, int ed_search_radius,
		RigidTransformCuda const* dev_rigid_transf,
		int width_depth, int height_depth, float mu)
	{
		const int3 cubes_dim = *dev_cubes_dim;
		if ((blockDim.x - 1)*blockIdx.x > cubes_dim.x ||
			(blockDim.y - 1)*blockIdx.y > cubes_dim.y ||
			(blockDim.z - 1)*blockIdx.z > cubes_dim.z)
			return;

		__shared__ float x_prjs[MAX_NUM_DEPTH_CAMERAS * CUBEPRUNE_NDIMS * THREADS_PER_BLOCK_CUBEPRUNE];
		__shared__ int sh_occu_cubes_count;
		__shared__ RigidTransformCuda sh_rigid_transf;

		int xId = threadIdx.x + (blockDim.x - 1)*blockIdx.x; //cube Ids
		int yId = threadIdx.y + (blockDim.y - 1)*blockIdx.y;
		int zId = threadIdx.z + (blockDim.z - 1)*blockIdx.z;
		int cubeId = zId*cubes_dim.x*cubes_dim.y + yId*cubes_dim.x + xId;
		int tid = threadIdx.z*blockDim.x*blockDim.y + threadIdx.y*blockDim.x + threadIdx.x;
		if (tid == 0)
		{
			sh_occu_cubes_count = 0;
			sh_rigid_transf = dev_rigid_transf[0];
		}
		__syncthreads();

		int lc_offset = 0;
		if (xId <= cubes_dim.x && yId <= cubes_dim.y && zId <= cubes_dim.z)
		{
			const float3 cubes_offset = *dev_cubes_offset;
			const float3 ed_cubes_offsets = *dev_ed_cubes_offsets;
			const int3 ed_cubes_dims = *dev_ed_cubes_dims;

			cuda_vector_fixed<float, 3> vt;
			vt[0] = cubes_offset.x + xId * cube_res;
			vt[1] = cubes_offset.y + yId * cube_res;
			vt[2] = cubes_offset.z + zId * cube_res;

			int xEdCubeId = (vt[0] - ed_cubes_offsets.x) / ed_cube_res;
			int yEdCubeId = (vt[1] - ed_cubes_offsets.y) / ed_cube_res;
			int zEdCubeId = (vt[2] - ed_cubes_offsets.z) / ed_cube_res;

			float dists_sq[VXL_NEIGHBOR_EDNODE_NUM];
			int ngn_idx[VXL_NEIGHBOR_EDNODE_NUM];
			for (int i = 0; i < VXL_NEIGHBOR_EDNODE_NUM; i++)
			{
				dists_sq[i] = 1.0e+10f;
				ngn_idx[i] = -1;
			}

			for (int k = -ed_search_radius; k <= ed_search_radius; k++)
				for (int j = -ed_search_radius; j <= ed_search_radius; j++)
					for (int i = -ed_search_radius; i <= ed_search_radius; i++)
					{
						int xEdId = xEdCubeId + i;
						int yEdId = yEdCubeId + j;
						int zEdId = zEdCubeId + k;

						if (xEdId >= 0 && yEdId >= 0 && zEdId >= 0 &&
							xEdId < ed_cubes_dims.x && yEdId < ed_cubes_dims.y && zEdId < ed_cubes_dims.z)
						{
							short ndId = tex3D(tex_ndIds, xEdId, yEdId, zEdId);
							if (ndId >= 0)
							{
								cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndId].g;
								float dist_sq = dist_square<3>(vt.data_block(), g.data_block());

								if (dist_sq < dists_sq[0])
								{
									dists_sq[0] = dist_sq;
									ngn_idx[0] = ndId;
								}
								for (int c = 1; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
								{
									if (dist_sq < dists_sq[c])
									{
										dists_sq[c - 1] = dists_sq[c];
										ngn_idx[c - 1] = ngn_idx[c];
										dists_sq[c] = dist_sq;
										ngn_idx[c] = ndId;
									}
								}
							}
						}
					}

			float w_sum = 0;
			for (int c = 0; c < VXL_NEIGHBOR_EDNODE_NUM; c++)
			{
				if (ngn_idx[c] != -1)
				{
					dists_sq[c] = expf(-dists_sq[c] / (sigma_vxl_node_dist*sigma_vxl_node_dist * 2.0f));
					w_sum += dists_sq[c];
				}
			}

			if (w_sum > 1.0e-6)
			{
				cuda_vector_fixed<float, 3> vt_t(0.0);
				for (int k = 0; k < VXL_NEIGHBOR_EDNODE_NUM; k++)
				{
					int &ndIdx_k = ngn_idx[k];
					float &w_k = dists_sq[k];
					if (ndIdx_k >= 0)
					{
						cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[ndIdx_k].A;
						cuda_vector_fixed<float, 3> g = dev_ed_nodes[ndIdx_k].g;
						cuda_vector_fixed<float, 3> t = dev_ed_nodes[ndIdx_k].t;

						vt_t += w_k / w_sum*(A*(vt - g) + g + t);
					}
				}
				vt_t = sh_rigid_transf.R*vt_t + sh_rigid_transf.T;

				for (int i = 0; i < dev_num_cam_views; i++)
				{
					cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*vt_t + dev_cam_views[i].cam_pose.T;

					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
					float const&fx = K[0][0];
					float const&fy = K[1][1];
					float const&cx = K[0][2];
					float const&cy = K[1][2];

					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS] = fx * X[0] / X[2] + cx;
					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 1] = fy * X[1] / X[2] + cy;
					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 2] = X[2];
				}
			}
			else
			{
				for (int i = 0; i < dev_num_cam_views; i++)
				{
					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS] = 0.0f;
					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 1] = 0.0f;
					x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 2] = 0.0f;
				}
			}
		}
		__syncthreads();

		const int dx[] = { 0, 0, 0, 0, 1, 1, 1, 1 };
		const int dy[] = { 0, 0, 1, 1, 0, 0, 1, 1 };
		const int dz[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

		bool bOccupied = false;
		if (xId < cubes_dim.x && yId < cubes_dim.y && zId < cubes_dim.z &&
			threadIdx.x < blockDim.x - 1 &&
			threadIdx.y < blockDim.y - 1 &&
			threadIdx.z < blockDim.z - 1)
		{
			if (dev_cubes[cubeId].offset < 0)
			{
				bool bProcess = false;
				//check neighboring cell
				for (int i = -1; i <= 1; i++)
					for (int j = -1; j <= 1; j++)
						for (int k = -1; k <= 1; k++)
						{
							int xId_n = xId + i;
							int yId_n = yId + j;
							int zId_n = zId + k;
							if (xId_n >= 0 && yId_n >= 0 && zId_n >= 0 &&
								xId_n < cubes_dim.x && yId_n < cubes_dim.y && zId_n < cubes_dim.z)
							{
								int id_n = zId_n*cubes_dim.x*cubes_dim.y + yId_n*cubes_dim.x + xId_n;
								if (dev_cubes[id_n].offset >= 0)
									bProcess = true;
							}
						}

				if (bProcess)
				{
					for (int vId = 0; vId < dev_num_cam_views; vId++)
					{
						float u_max = 0.0f;
						float u_min = 1.0e+10f;
						float v_max = 0.0f;
						float v_min = 1.0e+10f;
						float w_max = 0.0f;
						float w_min = 1.0e+10f;
						for (int i = 0; i < 8; i++)
						{
							int idx = (threadIdx.z + dz[i])*blockDim.x*blockDim.y +
								(threadIdx.y + dy[i])*blockDim.x + threadIdx.x + dx[i];
							float const& u = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId];
							float const& v = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId + 1];
							float const& w = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId + 2];

							if (u > u_max) u_max = u;
							if (u < u_min) u_min = u;
							if (v > v_max) v_max = v;
							if (v < v_min) v_min = v;
							if (w > w_max) w_max = w;
							if (w < w_min) w_min = w;
						}
						if (w_min > 1.0e-6f) //corner point might not be deformed
						{
							w_max *= 10.0f;
							w_min *= 10.0f; //cm to mm
							float r_h = u_max - u_min;
							float r_v = v_max - v_min;
							float level = MAX(r_h, r_v);
							level = MAX(2.0f, floorf(log2f(level)));
							r_h /= 2.0f;
							r_v /= 2.0f;
							ushort2 mipmap = pick_mipmap9((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level, width_depth, height_depth/*, bPrint*/);
							if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
							{
								ushort2 mipmap = pick_mipmap25((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level - 1.0f, width_depth, height_depth/*, bPrint*/);
								if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
								{
									bOccupied = true;
								}
							}
						}
					}
				}//end of if(bProcess)
			}//end of if (dev_cubes[cubeId].offset < 0)
		}

		int lc_count = 0;
		if (bOccupied)
			lc_count = atomicAdd(&sh_occu_cubes_count, 1);
		__syncthreads();

		if (tid == 0)
			sh_occu_cubes_count = atomicAdd(dev_global_count_occu_cubes, sh_occu_cubes_count);
		__syncthreads();

		if (bOccupied)
		{
			lc_count += sh_occu_cubes_count;
			if (lc_count < TWO_LEVEL_VOLUME_CUBES_OCCUPIED_MAX)
			{
				dev_buf_occupied_cube_ids[lc_count] = cubeId;
			}
		}
	}

	__global__ void init_newly_occupied_cubes_kernel(OccupcyCube* dev_cubes,
		float* vxl_data,
		float* vxl_weights,
		uchar4* vxl_colors,
		int const*dev_buf_occupied_cube_ids,
		int const* dev_occu_cubes_id_buf_stIdx, //inclusive
		int const* dev_occu_cubes_id_buf_endIdx, //exclusive
		int vxls_per_cube
		)
	{
		const int occu_cubes_id_buf_stIdx = *dev_occu_cubes_id_buf_stIdx;
		const int occu_cubes_id_buf_endIdx = *dev_occu_cubes_id_buf_endIdx;

		for (int idx = blockIdx.x + occu_cubes_id_buf_stIdx; idx < occu_cubes_id_buf_endIdx; idx += gridDim.x)
		{
			int cubeId = dev_buf_occupied_cube_ids[idx];
			if (threadIdx.x == 0)
				dev_cubes[cubeId].offset = idx;

			int data_offset = idx * vxls_per_cube;
			for (int i = threadIdx.x; i < vxls_per_cube; i += blockDim.x)
			{
				vxl_data[data_offset + i] = SDF_NULL_VALUE;
				vxl_weights[data_offset + i] = 0;
				vxl_colors[data_offset + i] = make_uchar4(0, 0, 0, 0);
			}
		}
	}



	void VolumetricFusionHelperCudaImpl::
		coarse_cube_prune_vWarp(VolumeTwoLevelHierachy *volume,
		DeformGraphNodeCuda const*dev_ed_nodes, int const* dev_ed_nodes_num, float sigma_vxl_node_dist,
		int3 const* dev_ed_cubes_dims, float3 const* dev_ed_cubes_offsets, float ed_cube_res,
		RigidTransformCuda const* dev_rigid_transf)
	{
		//save the occupied cubes count before prune
		checkCudaErrors(cudaMemcpyAsync(dev_occu_cubes_count_pre_, volume->gpu_cubes_occpied_count.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));

		float cube_res = volume->cube_res;
		float r = sqrt((2.0*sigma_vxl_node_dist)*(2.0*sigma_vxl_node_dist) +
			(volume->mu + sqrt(3.0)*cube_res)*(volume->mu + sqrt(3.0)*cube_res)
			);

		int ed_search_radius = MAX(ceil(r / ed_cube_res), 2);
		LOGGER()->debug("coarse cube prune vWarp: ed_search_radius=<%f, %d>", r, ed_search_radius);
		
		dim3 blockDim = dim3(8, 8, 4);
		//each cube block processing 8x8x4 grids points (==7x7x3 cubes)
		dim3 gridDim = dim3((TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 6) / 7, (TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 6) / 7, (TWO_LEVEL_VOLUME_CUBES_DIM_MAX + 2) / 3);
		OccupcyCubePruneKernel_vDeform2 << <gridDim, blockDim >> >(volume->cubes,
			volume->buf_occupied_cube_ids,
			volume->gpu_cubes_occpied_count.dev_ptr,
			volume->ptr_cubes_offset,
			volume->ptr_cubes_num,
			volume->cube_res,
			dev_ed_nodes, sigma_vxl_node_dist,
			dev_ed_cubes_dims, dev_ed_cubes_offsets, ed_cube_res, ed_search_radius,
			dev_rigid_transf,
			this->depth_width_, this->depth_height_, volume->mu);
		m_checkCudaErrors();


		if (LOGGER()->check_verbosity(Logger::Debug))
		{
			int count_pre = 0;
			checkCudaErrors(cudaMemcpy(&count_pre, dev_occu_cubes_count_pre_, sizeof(int), cudaMemcpyDeviceToHost));
			int count_post = volume->gpu_cubes_occpied_count.sync_read();
			LOGGER()->debug(">>>>>>>>>>cubes prune vWarp<pre vs post>: %d / %d", count_pre, count_post);
		}

		int threads_per_block = 512;
		int blocks_per_grid = 24;
		init_newly_occupied_cubes_kernel << <blocks_per_grid, threads_per_block >> >(volume->cubes, volume->data, volume->weights, volume->colors,
			volume->buf_occupied_cube_ids,
			dev_occu_cubes_count_pre_, volume->gpu_cubes_occpied_count.dev_ptr,
			volume->vxls_per_cube);
		m_checkCudaErrors();
	}

	__global__ void OccupcyCubePruneKernel_WO_Deform(OccupcyCube* dev_cubes,
		int *dev_buf_occupied_cube_ids,
		int *dev_global_count_occu_cubes,
		float3 cubes_offset,
		int3 cubes_num,
		float cube_res,
		int width_depth, int height_depth, float mu)
	{
		__shared__ float x_prjs[MAX_NUM_DEPTH_CAMERAS * CUBEPRUNE_NDIMS * THREADS_PER_BLOCK_CUBEPRUNE];
		__shared__ int sh_offset;
		int xId = threadIdx.x + (blockDim.x - 1)*blockIdx.x; //cube Ids
		int yId = threadIdx.y + (blockDim.y - 1)*blockIdx.y;
		int zId = threadIdx.z + (blockDim.z - 1)*blockIdx.z;

		int tid = threadIdx.z*blockDim.x*blockDim.y + threadIdx.y*blockDim.x + threadIdx.x;
		if (tid == 0)
			sh_offset = 0;
		__syncthreads();

		int lc_offset = 0;
		if (xId < cubes_num.x + 1 && yId < cubes_num.y + 1 && zId < cubes_num.z + 1)
		{
			cuda_vector_fixed<float, 3> P_wld;
			P_wld[0] = cubes_offset.x + xId * cube_res;
			P_wld[1] = cubes_offset.y + yId * cube_res;
			P_wld[2] = cubes_offset.z + zId * cube_res;

			for (int i = 0; i < dev_num_cam_views; i++)
			{
				cuda_vector_fixed<float, 3> X = dev_cam_views[i].cam_pose.R*P_wld + dev_cam_views[i].cam_pose.T;

				cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[i].K;
				float const&fx = K[0][0];
				float const&fy = K[1][1];
				float const&cx = K[0][2];
				float const&cy = K[1][2];

				x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS] = fx * X[0] / X[2] + cx;
				x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 1] = fy * X[1] / X[2] + cy;
				x_prjs[tid * dev_num_cam_views * CUBEPRUNE_NDIMS + i * CUBEPRUNE_NDIMS + 2] = X[2];

			}
		}
		__syncthreads();

		const int dx[] = { 0, 0, 0, 0, 1, 1, 1, 1 };
		const int dy[] = { 0, 0, 1, 1, 0, 0, 1, 1 };
		const int dz[] = { 0, 1, 0, 1, 0, 1, 0, 1 };

		bool bOccupied = false;
		if (xId < cubes_num.x && yId < cubes_num.y && zId < cubes_num.z &&
			threadIdx.x < blockDim.x - 1 &&
			threadIdx.y < blockDim.y - 1 &&
			threadIdx.z < blockDim.z - 1)
		{
			for (int vId = 0; vId < dev_num_cam_views; vId++)
			{
				float u_max = 0.0f;
				float u_min = 1.0e+10f;
				float v_max = 0.0f;
				float v_min = 1.0e+10f;
				float w_max = 0.0f;
				float w_min = 1.0e+10f;
				for (int i = 0; i < 8; i++)
				{
					int idx = (threadIdx.z + dz[i])*blockDim.x*blockDim.y +
						(threadIdx.y + dy[i])*blockDim.x + threadIdx.x + dx[i];
					float const& u = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId];
					float const& v = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId + 1];
					float const& w = x_prjs[dev_num_cam_views * CUBEPRUNE_NDIMS * idx + CUBEPRUNE_NDIMS * vId + 2];

					if (u > u_max) u_max = u;
					if (u < u_min) u_min = u;
					if (v > v_max) v_max = v;
					if (v < v_min) v_min = v;
					if (w > w_max) w_max = w;
					if (w < w_min) w_min = w;
				}

				w_max *= 10.0f;
				w_min *= 10.0f; //cm to mm
				float r_h = u_max - u_min;
				float r_v = v_max - v_min;
				float level = MAX(r_h, r_v);
				level = MAX(2.0f, floorf(log2f(level)));
				r_h /= 2.0f;
				r_v /= 2.0f;
				ushort2 mipmap = pick_mipmap9((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level, width_depth, height_depth/*, bPrint*/);
				if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
				{
					ushort2 mipmap = pick_mipmap25((u_max + u_min) / 2.0f, (v_max + v_min) / 2.0f, r_h, r_v, vId, level - 1.0f, width_depth, height_depth/*, bPrint*/);
					if (!(w_max< mipmap.y - mu*10.0f || w_min > mipmap.x + mu*10.0f))
					{
						bOccupied = true;
					}
				}
			}
		}

		if (bOccupied)
			lc_offset = atomicAdd(&sh_offset, 1);
		__syncthreads();

		if (tid == 0)
			sh_offset = atomicAdd(dev_global_count_occu_cubes, sh_offset);
		__syncthreads();

		if (xId < cubes_num.x && yId < cubes_num.y && zId < cubes_num.z &&
			threadIdx.x < blockDim.x - 1 &&
			threadIdx.y < blockDim.y - 1 &&
			threadIdx.z < blockDim.z - 1)
		{
			int cubeId = zId * cubes_num.x*cubes_num.y + yId*cubes_num.x + xId;
			if (bOccupied)
			{
				int offset = sh_offset + lc_offset;
				dev_cubes[cubeId].offset = offset;
				dev_buf_occupied_cube_ids[offset] = cubeId;
			}
			else
				dev_cubes[cubeId].offset = -1;
		}
	}


	__global__ void ComputeTextureMipmapKernel_L0(int depth_num, int width_trg, int height_trg)
	{
		int col = blockIdx.x*blockDim.x + threadIdx.x;
		unsigned char texId = col / width_trg;
		col = col - texId*width_trg;
		int const& row = threadIdx.y + blockIdx.y*blockDim.y;

		if (texId < depth_num && row < height_trg)
		{
			unsigned short d1 = tex2DLayered(tex_depthImgs, col * 2, row * 2, texId);
			depth_extract_fg(d1);
			unsigned short d2 = tex2DLayered(tex_depthImgs, col * 2, row * 2 + 1, texId);
			depth_extract_fg(d2);
			unsigned short d3 = tex2DLayered(tex_depthImgs, col * 2 + 1, row * 2, texId);
			depth_extract_fg(d3);
			unsigned short d4 = tex2DLayered(tex_depthImgs, col * 2 + 1, row * 2 + 1, texId);
			depth_extract_fg(d4);

			unsigned short d_max = d1;
			if (d2 > d_max)
				d_max = d2;
			if (d3 > d_max)
				d_max = d3;
			if (d4 > d_max)
				d_max = d4;
			unsigned short d_min = d_max;
			if (d1 != 0 && d1 < d_min)
				d_min = d1;
			if (d2 != 0 && d2 < d_min)
				d_min = d2;
			if (d3 != 0 && d3 < d_min)
				d_min = d3;
			if (d4 != 0 && d4 < d_min)
				d_min = d4;
			ushort2 tmp;
			tmp.x = d_max;
			tmp.y = d_min;
			surf2DLayeredwrite(tmp, surf_mipmaps, col*sizeof(ushort2), row, texId, cudaBoundaryModeClamp);
		}
	}

	__global__ void ComputeTextureMipmapKernel_Surf2Surf(int depth_num, int width_trg, int height_trg, short mipmap_level,
		int height_offset_0, int height_offset_1)
	{
		int col = blockIdx.x*blockDim.x + threadIdx.x;
		int texId = col / width_trg;
		col = col - texId*width_trg;
		int const& row = threadIdx.y + blockIdx.y*blockDim.y;


		if (texId < depth_num && row < height_trg)
		{
			ushort2 d1 = surf2DLayeredread<ushort2>(surf_mipmaps, col * 2 * sizeof(ushort2), height_offset_0 + row * 2, texId, cudaBoundaryModeClamp);
			ushort2 d2 = surf2DLayeredread<ushort2>(surf_mipmaps, col * 2 * sizeof(ushort2), height_offset_0 + row * 2 + 1, texId, cudaBoundaryModeClamp);
			ushort2 d3 = surf2DLayeredread<ushort2>(surf_mipmaps, (col * 2 + 1)*sizeof(ushort2), height_offset_0 + row * 2, texId, cudaBoundaryModeClamp);
			ushort2 d4 = surf2DLayeredread<ushort2>(surf_mipmaps, (col * 2 + 1)*sizeof(ushort2), height_offset_0 + row * 2 + 1, texId, cudaBoundaryModeClamp);

			ushort2 mipmap = d1;
			if (d2.x > mipmap.x)
				mipmap.x = d2.x;
			if (d3.x > mipmap.x)
				mipmap.x = d3.x;
			if (d4.x > mipmap.x)
				mipmap.x = d4.x;

			mipmap.y = mipmap.x;
			if (d1.y != 0 && d1.y < mipmap.y)
				mipmap.y = d1.y;
			if (d2.y != 0 && d2.y < mipmap.y)
				mipmap.y = d2.y;
			if (d3.y != 0 && d3.y < mipmap.y)
				mipmap.y = d3.y;
			if (d4.y != 0 && d4.y < mipmap.y)
				mipmap.y = d4.y;

			surf2DLayeredwrite(mipmap, surf_mipmaps, col*sizeof(ushort2), height_offset_1 + row, texId, cudaBoundaryModeClamp);
		}
	}

	void VolumetricFusionHelperCudaImpl::compute_mipmap()
	{
		int width_trg = this->depth_width_ / 2;
		int height_trg = this->depth_height_ / 2;

		dim3 blockDim = dim3(8, 8, 1);
		dim3 gridDim = dim3((width_trg *this->num_depthmaps_ + 7) / 8, (height_trg + 7) / 8, 1);
		ComputeTextureMipmapKernel_L0 << <gridDim, blockDim >> >(this->num_depthmaps_, width_trg, height_trg);
		m_checkCudaErrors();

		int level_max = ROUND(std::log(MAX(this->depth_height_, this->depth_width_)) / log(2.0));
		for (int level = 1; level <= MIN(level_max, MIPMAP_MAX_LEVEL); level++)
		{
			int width_trg = ROUND(this->depth_width_ / pow(2, level + 1));
			int height_trg = this->depth_height_ / pow(2, level + 1); //TODO: might run into problems when h != 2^n
			int h = this->depth_height_;
			short h_offset_0 = h - h / std::pow(2, level - 1);
			short h_offset_1 = h - h / std::pow(2, level);
			blockDim = dim3(8, 8, 1);
			gridDim = dim3((width_trg *this->num_depthmaps_ + 7) / 8, (height_trg + 7) / 8, 1);
			ComputeTextureMipmapKernel_Surf2Surf << <gridDim, blockDim >> >(this->num_depthmaps_, width_trg, height_trg, level, h_offset_0, h_offset_1);
			m_checkCudaErrors()
		}

	}

	void VolumetricFusionHelperCudaImpl::feed_camera_view(CameraViewCuda* host_cam_views, int view_num)
	{
		LOGGER()->error("VolumetricFusionHelperCudaImpl::feed_camera_view", "View num %d", view_num);

		checkCudaErrors(cudaMemcpyToSymbol(dev_num_cam_views, &view_num, sizeof(int)));
		checkCudaErrors(cudaMemcpyToSymbolAsync(dev_cam_views, host_cam_views, sizeof(CameraViewCuda)*view_num));
	}

	__global__
		void set_vt_color_as_residual_kernel(float *dev_vts, float const* dev_per_vertex_align_residual, float const* dev_color_map, int const* dev_vts_num)
	{
		const int vts_num = *dev_vts_num;
		int vtIdx = threadIdx.x + blockIdx.x*blockDim.x;
		if (vtIdx < vts_num)
		{
			float residual = dev_per_vertex_align_residual[vtIdx];
			int idx = MAX(0, MIN(255, residual * 255.0f));
			dev_vts[vtIdx * 9 + 6] = dev_color_map[3 * idx];
			dev_vts[vtIdx * 9 + 7] = dev_color_map[3 * idx + 1];
			dev_vts[vtIdx * 9 + 8] = dev_color_map[3 * idx + 2];
		}
	}

	void VolumetricFusionHelperCudaImpl::
		set_vt_color_as_residual(float *dev_vts, float const*dev_per_vertex_align_residual, cuda::gpu_size_data vts_num_gpu)
	{
		int threads_per_block = 256;
		int blocks_per_grid = (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
		set_vt_color_as_residual_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts, dev_per_vertex_align_residual, dev_color_map_, vts_num_gpu.dev_ptr);
		m_checkCudaErrors();
	}

	__global__ void copy_vts_as_half_kernel(float const*dev_vts, int const* dev_vts_num, int vts_dim,
		short *dev_vts_out, int out_buf_size, //size in vt num 
		int bFlipNormal)
	{
		int threads_num = gridDim.x*blockDim.x;
		const int vts_num = MIN(out_buf_size, dev_vts_num[0]);
		for (int idx = threadIdx.x + blockIdx.x*blockDim.x; idx < vts_num; idx += threads_num)
		{
			if (idx < vts_num && idx < out_buf_size)
			{
				float n[3];
				for (int i = 0; i < 3; i++)
					dev_vts_out[idx * 6 + i] = __float2half_rn(dev_vts[idx*vts_dim + i]);
				for (int i = 3; i < 6; i++){
					n[i - 3] = dev_vts[idx*vts_dim + i];
					if (bFlipNormal)
						dev_vts_out[idx * 6 + i] = __float2half_rn(-n[i - 3]);
					else
						dev_vts_out[idx * 6 + i] = __float2half_rn(n[i - 3]);
				}
			}
		}
	}
	cudaError_t VolumetricFusionHelperCudaImpl::sync_copy_vts_to_cpu_buf_sync_as_half(short* buf_out, int vts_num_cpu_out, bool bFlipNormal)
	{
		int threads_per_block = 64;
		int blocks_per_grid = 256;
		copy_vts_as_half_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts_cur_buf_, vts_cur_num_gpu_.dev_ptr, vts_dim_,
			dev_vts_half_buf_, vts_num_cpu_out, bFlipNormal);
		m_checkCudaErrors();
		return cudaMemcpy(buf_out, dev_vts_half_buf_, sizeof(short) * 6 * vts_num_cpu_out, cudaMemcpyDeviceToHost);
	}

	//one vertex per thread
	//TODO: colorChannelMode: 0--none; 1-copy color; 2-copy normal to color
	__global__ void copy_vts_kernel(float const*dev_vts, int const* dev_vts_num, int vts_dim,
		float *dev_vts_out, int out_buf_size, //size in vt num 
		int vts_dim_out, int colorChannelMode, int bFlipNormal)
	{
		int threads_num = gridDim.x*blockDim.x;
		const int vts_num = MIN(out_buf_size, dev_vts_num[0]);
		for (int idx = threadIdx.x + blockIdx.x*blockDim.x; idx < vts_num; idx += threads_num)
		{
			if (idx < vts_num && idx < out_buf_size)
			{
				float n[3];
				for (int i = 0; i < 3; i++)
					dev_vts_out[idx*vts_dim_out + i] = dev_vts[idx*vts_dim + i];
				for (int i = 3; i < 6; i++){
					n[i - 3] = dev_vts[idx*vts_dim + i];
					if (bFlipNormal)
						dev_vts_out[idx*vts_dim_out + i] = -n[i - 3];
					else
						dev_vts_out[idx*vts_dim_out + i] = n[i - 3];
				}

				//copy color 
				if (vts_dim == 9 && vts_dim_out == 9)
				{
					dev_vts_out[idx * 9 + 6] = dev_vts[idx * 9 + 6];
					dev_vts_out[idx * 9 + 7] = dev_vts[idx * 9 + 7];
					dev_vts_out[idx * 9 + 8] = dev_vts[idx * 9 + 8];
				}

				if (colorChannelMode == 2)
				{
					//normal to color
					dev_vts_out[idx*vts_dim_out + 6] = (-n[0] + 1.0f) / 2.0;
					dev_vts_out[idx*vts_dim_out + 7] = (n[1] + 1.0f) / 2.0;
					dev_vts_out[idx*vts_dim_out + 8] = (n[2] + 1.0f) / 2.0;

				}
			}
		}
	}

	void VolumetricFusionHelperCudaImpl::copy_vts_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data vts_num_gpu_out, int colorMode, bool bFlipNormal)
	{
		int threads_per_block = 64;
		int blocks_per_grid = 256;
		copy_vts_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts_buf_, vts_num_gpu_.dev_ptr, vts_dim_,
			dev_buf_out, vts_num_gpu_out.max_size, stride_out, colorMode, bFlipNormal);
		m_checkCudaErrors();
		checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_out.dev_ptr, vts_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	}

	void VolumetricFusionHelperCudaImpl::copy_vts_t_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data vts_num_gpu_out, int colorMode, bool bFlipNormal)
	{
		int threads_per_block = 64;
		int blocks_per_grid = 256;
		copy_vts_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts_t_buf_, vts_t_num_gpu_.dev_ptr, vts_dim_,
			dev_buf_out, vts_num_gpu_out.max_size, stride_out, colorMode, bFlipNormal);
		m_checkCudaErrors();

		checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_out.dev_ptr, vts_t_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	}

	void VolumetricFusionHelperCudaImpl::copy_vts_cur_to_dev_buf(float* dev_buf_out, int stride_out, cuda::gpu_size_data vts_num_gpu_out, int colorMode, bool bFlipNormal)
	{
		int threads_per_block = 64;
		int blocks_per_grid = 256;
		copy_vts_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts_cur_buf_, vts_cur_num_gpu_.dev_ptr, vts_dim_,
			dev_buf_out, vts_num_gpu_out.max_size, stride_out, colorMode, bFlipNormal);
		m_checkCudaErrors();

		checkCudaErrors(cudaMemcpyAsync(vts_num_gpu_out.dev_ptr, vts_cur_num_gpu_.dev_ptr, sizeof(int), cudaMemcpyDeviceToDevice));
	}

	void VolumetricFusionHelperCudaImpl::
		sync_copy_vts_buf(float const* dev_vts_in, int stride_in, cuda::gpu_size_data vts_num_gpu_in,
		float* dev_buf_out, int stride_out, int &buf_size_in_and_out,
		int colorMode, bool bFlipNormal)
	{
		int threads_per_block = 64;
		int blocks_per_grid = 256;
		copy_vts_kernel << <blocks_per_grid, threads_per_block >> >(dev_vts_in, vts_num_gpu_in.dev_ptr, stride_in,
			dev_buf_out, buf_size_in_and_out, stride_out, colorMode, bFlipNormal);
		m_checkCudaErrors();

		int vts_num = 0;
		checkCudaErrors(cudaMemcpy(&vts_num, vts_num_gpu_in.dev_ptr, sizeof(int), cudaMemcpyDeviceToHost));
		buf_size_in_and_out = MIN(buf_size_in_and_out, vts_num);
	}

	void VolumetricFusionHelperCudaImpl::
		sync_copy_triangles_buf(int3 const* dev_triangles_buf, cuda::gpu_size_data tris_num_gpu,
		int* dev_triangles_buf_out, int &buf_size_in_and_out)
	{
		int tris_num = tris_num_gpu.sync_read();
		checkCudaErrors(cudaMemcpy(dev_triangles_buf_out, dev_triangles_buf, sizeof(int3) * tris_num, cudaMemcpyDeviceToDevice));
		buf_size_in_and_out = MIN(buf_size_in_and_out, tris_num);
	}

	void VolumetricFusionHelperCudaImpl::
		sync_feed_vts_buf(float const* vts_data, int vt_dim, int vt_num,
		float *dev_vts_buf, int vt_dim_gpu, cuda::gpu_size_data vts_num_gpu)
	{
		if (vt_dim != vt_dim_gpu)
		{
			LOGGER()->error("VolumetricFusionHelperCudaImpl::sync_feed_vts_buf", "vt_dim != vt_dim_gpu (%d vs %d)", vt_dim, vt_dim_gpu);
			return;
		}

		checkCudaErrors(cudaMemcpy(dev_vts_buf, vts_data, sizeof(float)*vt_dim*vt_num, cudaMemcpyHostToDevice));
		checkCudaErrors(cudaMemcpy(vts_num_gpu.dev_ptr, &vt_num, sizeof(int), cudaMemcpyHostToDevice));
	}


}
