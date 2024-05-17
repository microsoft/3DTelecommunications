#ifndef __EDMATCHINGHELPERCUDAIMPL_RIGID_CU__
#define __EDMATCHINGHELPERCUDAIMPL_RIGID_CU__


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//             RIGID ALIGNMENT
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<int vtStep>
__global__
void evaluate_cost_rigid_alignment_kernel(float *dev_cost_global_rigid,
									 float const* dev_R, float const* dev_T,
									 float const* dev_vts, int vts_num, int vt_dim,
									 int depth_width, int depth_height, float mu)
{
	__shared__ cuda_matrix_fixed<float, 3, 3> R;
	__shared__ cuda_vector_fixed<float, 3> T;
	__shared__ float costs[MAX_THREADS_PER_BLOCK];

	int id = threadIdx.x;
	if (id < 9)
	{
		float *p_dst = R.data_block();
		p_dst[id] = dev_R[id];
	}
	if (id < 3) T[id] = dev_T[id];
	__syncthreads();

	float cost = 0.0f;
	int vtIdx = (threadIdx.x + blockDim.x*blockIdx.x)*vtStep;
	if (vtIdx < vts_num)
	{
		float const* ptr_vt = dev_vts + vt_dim * vtIdx;
		cuda_vector_fixed<float, 3> vt(ptr_vt);
		cuda_vector_fixed<float, 3> n(ptr_vt + 3);
		vt = R*vt + T;
		n = R*n;

		#pragma unroll
		for (int vId = 0; vId< dev_num_cam_views; vId++)
		{
			cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vt + dev_cam_views[vId].cam_pose.T;

			//if the point is behind the camera, then skip it
			if (X[2] > 0.1f)
			{
				cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
				float const&fx = K[0][0];
				float const&fy = K[1][1];
				float const&cx = K[0][2];
				float const&cy = K[1][2];

				int u = ROUND(fx*X[0] / X[2] + cx);
				int v = ROUND(fy*X[1] / X[2] + cy);

				// if the point cannot be observed at the current camera pose
				if (u >= 0 && u < depth_width &&
					v >= 0 && v < depth_height)
				{
					unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
					depth_remove_top_bit(d);

					if (d > 0)
					{
						cuda_vector_fixed<float, 3> p;
						p[2] = d / 10.0f;
						p[0] = (u - cx)*p[2] / fx;
						p[1] = (v - cy)*p[2] / fy;

						//to world space
						p -= dev_cam_views[vId].cam_pose.T;
						cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

						if (dist_square<3>(p_wld.data_block(), vt.data_block()) < mu*mu)
						{
							float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
							cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
							cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);
							if (dot_product<float, 3>(n, nd_t) > NORMAL_CHECK_THRES)
							{
								 float f = dot_product(n, p_wld - vt);
								 cost += f*f;
							}
						}
					}
				}
			}
		}
	}
	costs[threadIdx.x] = cost;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
			costs[threadIdx.x] += costs[threadIdx.x + s];
		__syncthreads();
	}

	if (threadIdx.x == 0)
		atomicAdd(dev_cost_global_rigid, costs[0]);
}


__global__
void rodrigues_kernel(float const* dev_rod, float *dev_R, float *dev_dR_drod)
{
	__shared__ float sh_R[9];
	__shared__ float sh_dR_drod[27];

#define DR(i, j) sh_dR_drod[3*(i) + (j)]
#define R(i, j)  sh_R[3*(i)+(j)]

	if (threadIdx.x == 0)
	{	
		float x = dev_rod[0];
		float y = dev_rod[1];
		float z = dev_rod[2];

		double th = sqrtf(x*x + y*y + z*z);
		if (th < 1.0e-6)
		{
			R(0, 0) = 1.0f; R(0, 1) = 0.0f; R(0, 2) = 0.0f;
			R(1, 0) = 0.0f; R(1, 1) = 1.0f; R(1, 2) = 0.0f;
			R(2, 0) = 0.0f; R(2, 1) = 0.0f; R(2, 2) = 1.0f;

			if (dev_dR_drod)
			{
				DR(0, 0) = 0.0f; DR(0, 1) = 0.0f; DR(0, 2) = 0.0f;
				DR(1, 0) = 0.0f; DR(1, 1) = 0.0f; DR(1, 2) = -1.0f;
				DR(2, 0) = 0.0f; DR(2, 1) = 1.0f; DR(2, 2) = 0.0f;

				DR(3, 0) = 0.0f; DR(3, 1) = 0.0f; DR(3, 2) = 1.0f;
				DR(4, 0) = 0.0f; DR(4, 1) = 0.0f; DR(4, 2) = 0.0f;
				DR(5, 0) = -1.0f; DR(5, 1) = 0.0f; DR(5, 2) = 0.0f;

				DR(6, 0) = 0.0f; DR(6, 1) = -1.0f; DR(6, 2) = 0.0f;
				DR(7, 0) = 1.0f; DR(7, 1) = 0.0f; DR(7, 2) = 0.0f;
				DR(8, 0) = 0.0f; DR(8, 1) = 0.0f; DR(8, 2) = 0.0f;
			}
		}
		else
		{
			x /= th;
			y /= th;
			z /= th;

			float xx = x*x;
			float xy = x*y;
			float xz = x*z;
			float yy = y*y;
			float yz = y*z;
			float zz = z*z;

			float &yx = xy;
			float &zx = xz;
			float &zy = yz;

			float sth = sinf(th);
			float cth = cosf(th);
			float mcth = 1.0f - cth;

			R(0, 0) = 1.0f - mcth * (yy + zz);
			R(0, 1) = -sth*z + mcth * yx;
			R(0, 2) = sth*y + mcth * xz;

			R(1, 0) = sth*z + mcth * xy;
			R(1, 1) = 1 - mcth * (zz + xx);
			R(1, 2) = -sth*x + mcth * yz;

			R(2, 0) = -sth*y + mcth * xz;
			R(2, 1) = sth*x + mcth * yz;
			R(2, 2) = 1 - mcth * (xx + yy);

			if (dev_dR_drod)
			{
				float a = sth / th;
				float b = mcth / th;
				float c = cth - a;
				float d = sth - 2.0f * b;

				DR(0, 0) = -d * (yy + zz) * x;
				DR(0, 1) = -2.0f * b*y - d * (yy + zz) * y;
				DR(0, 2) = -2.0f * b*z - d * (yy + zz) * z;

				DR(1, 0) = b*y - c * zx + d * xy * x;
				DR(1, 1) = b*x - c * zy + d * xy * y;
				DR(1, 2) = -a - c * zz + d * xy * z;
				
				DR(2, 0) = b*z + c * yx + d * zx * x;
				DR(2, 1) = a + c * yy + d * zx * y;
				DR(2, 2) = b*x + c * yz + d * zx * z;

				DR(3, 0) = b*y + c * zx + d * xy * x;
				DR(3, 1) = b*x + c * zy + d * xy * y;
				DR(3, 2) = a + c * zz + d * xy * z;
				
				DR(4, 0) = -2.0f * b*x - d * (zz + xx) * x;
				DR(4, 1) = -d * (zz + xx) * y;
				DR(4, 2) = -2.0f * b*z - d * (zz + xx) * z;

				DR(5, 0) = -a - c * xx + d * zy * x;
				DR(5, 1) = b*z - c * xy + d * zy * y;
				DR(5, 2) = b*y - c * xz + d * zy * z;

				DR(6, 0) = b*z - c * yx + d * xz * x;
				DR(6, 1) = -a - c * yy + d * xz * y;
				DR(6, 2) = b*x - c * yz + d * xz * z;
				
				DR(7, 0) = a + c * xx + d * yz * x;
				DR(7, 1) = b*z + c * xy + d * yz * y;
				DR(7, 2) = b*y + c * xz + d * yz * z;
				
				DR(8, 0) = -2.0f * b*x - d * (yy + xx) * x;
				DR(8, 1) = -2.0f * b*y - d * (yy + xx) * y;
				DR(8, 2) = -d * (yy + xx) * z;
			}
		}
	}
	__syncthreads();
#undef R
#undef DR


	if (threadIdx.x < 27 && dev_dR_drod)
		dev_dR_drod[threadIdx.x] = sh_dR_drod[threadIdx.x];

	if (threadIdx.x < 9)
		dev_R[threadIdx.x] = sh_R[threadIdx.x];	
}


#define CHUNK_NUM_RIGIDJTJ 16 //16 or 8
#define	THREADS_PER_BLOCK_RIGIDJTJ (36*CHUNK_NUM_RIGIDJTJ) //576
//the number of threads can be divided by both 32 and 36
template<int vtStep>
__global__
void evaluate_jtj_jtf_rigid_align_kernel(float* __restrict__ dev_partial_JtJs, float* __restrict__ dev_partial_JtFs,
										float const* dev_R, float const* dev_T,
										float const* dev_dR_drod,
										float const* dev_vts, int* dev_vts_num, int vt_dim,
										int depth_width, int depth_height, float mu
										)
{
	__shared__ cuda_matrix_fixed<float, 3, 3> R;
	__shared__ cuda_vector_fixed<float, 3> T;
	__shared__ cuda_matrix_fixed<float, 9, 3> dR_drod;
	__shared__ float sh_Js[THREADS_PER_BLOCK_RIGIDJTJ * 6];
	__shared__ float sh_JtFs[THREADS_PER_BLOCK_RIGIDJTJ * 6];

	int vts_num = *dev_vts_num;
	int blks_max = (vts_num + THREADS_PER_BLOCK_RIGIDJTJ - 1) / THREADS_PER_BLOCK_RIGIDJTJ;

	for (int blkIdx = blockIdx.x; blkIdx < blks_max; blkIdx += gridDim.x) //stride loop trick: each cuda block handle multiple blocks
	{
		int id = threadIdx.x;
		if (id < 9)
		{
			float *p_dst = R.data_block();
			p_dst[id] = dev_R[id];
			if (id < 3) T[id] = dev_T[id];
		}
		else if (32 <= id && id < 32 + 27)
		{
			float* p_dst = dR_drod.data_block();
			p_dst[id - 32] = dev_dR_drod[id - 32];
		}
		__syncthreads();

		for (int i = 0; i < 6; i++)
		{
			sh_Js[6 * threadIdx.x + i] = 0.0f;
			sh_JtFs[6 * threadIdx.x + i] = 0.0f;
		}

		int vtIdx = (threadIdx.x + blockDim.x*blkIdx)*vtStep;
		if (vtIdx < vts_num)
		{
			float const* ptr_vt = dev_vts + vt_dim * vtIdx;
			cuda_vector_fixed<float, 3> vt_ori(ptr_vt);
			cuda_vector_fixed<float, 3> n_ori(ptr_vt + 3);
			cuda_vector_fixed<float, 3> vt = R*vt_ori + T;
			cuda_vector_fixed<float, 3> n = R*n_ori;

			int vis_cam_count = 0;
			float f_sum = 0.0f;
			#pragma unroll
			for (int vId = 0; vId< dev_num_cam_views; vId++)
			{
				cuda_vector_fixed<float, 3> X = dev_cam_views[vId].cam_pose.R*vt + dev_cam_views[vId].cam_pose.T;

				//if the point is behind the camera, then skip it
				if (X[2] > 0.1f)
				{
					cuda_matrix_fixed<float, 3, 3> const& K = dev_cam_views[vId].K;
					float fx = K[0][0];
					float fy = K[1][1];
					float cx = K[0][2];
					float cy = K[1][2];

					int u = ROUND(fx*X[0] / X[2] + cx);
					int v = ROUND(fy*X[1] / X[2] + cy);

					// if the point cannot be observed at the current camera pose
					if (u >= 0 && u < depth_width &&
						v >= 0 && v < depth_height)
					{
						//float d_s = tex2D(tex_depthMap_, u, v);
						unsigned short d = tex2DLayered(tex_depthImgs, u, v, vId);
						depth_remove_top_bit(d);

						if (d > 0)
						{
							cuda_vector_fixed<float, 3> p;
							p[2] = d / 10.0f;
							p[0] = (u - cx)*p[2] / fx;
							p[1] = (v - cy)*p[2] / fy;

							//to world space
							p -= dev_cam_views[vId].cam_pose.T;
							cuda_vector_fixed<float, 3> p_wld = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(p);

							if (dist_square<3>(p_wld.data_block(), vt.data_block()) < mu*mu)
							{
								float4 nd_ = tex2DLayered(tex_normalMaps, u, v, vId);
								cuda_vector_fixed<float, 3> nd(nd_.x, nd_.y, nd_.z);
								cuda_vector_fixed<float, 3> nd_t = dev_cam_views[vId].cam_pose.R.transpose_and_multiply(nd);
								if (dot_product<float, 3>(n, nd_t) > NORMAL_CHECK_THRES)
								{
									f_sum += dot_product(n, p_wld - vt);
									vis_cam_count++;
								}

							}
						}
					}
				}
			}

			if (vis_cam_count > 0)
			{
				float df_dR[9];
				float w = sqrtf(vis_cam_count);
				for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					df_dR[3 * i + j] = -n[i] * vt_ori[j];// / 100.0f; //convert center meters to meters

				//df_drod
				#pragma unroll
				for (int i = 0; i < 3; i++)
				{
					float val = 0.0;
					#pragma unroll
					for (int j = 0; j < 9; j++)
						val += df_dR[j] * dR_drod[j][i];
					sh_Js[6 * threadIdx.x + i] = val * w;
					sh_JtFs[6 * threadIdx.x + i] = val * f_sum;// / 100.0f; //convert center meters to meters
				}

				//df_dT
				for (int i = 0; i < 3; i++)
				{
					sh_Js[6 * threadIdx.x + i + 3] = -n[i] * w;
					sh_JtFs[6 * threadIdx.x + i + 3] = -n[i] * f_sum;// / 100.0f; //convert center meters to meters
				}
			}
		}
		__syncthreads();

		//JtJ: divede threads into chunks of 36. 
		//Each chunk of threads is responsible for JtJ of 36 examples
		//each threads in a chunk handles on jtj element
		int chunkId = threadIdx.x / 36;
		int tmp = threadIdx.x % 36;
		int pi = tmp / 6;
		int pj = tmp % 6;

		float jtj_val = 0.0; //jtj at <pi, pj>
		for (int i = 0; i < 36; i++)
		{
			int idx = i + chunkId * 36;
			jtj_val += sh_Js[idx * 6 + pi] * sh_Js[idx * 6 + pj];
		}
		__syncthreads();

		//write jtj to shared memory and perform reduction
		sh_Js[threadIdx.x] = jtj_val;
		__syncthreads();

		for (int s = CHUNK_NUM_RIGIDJTJ / 2; s > 0; s >>= 1)
		{
			if (threadIdx.x < s * 36)
				sh_Js[threadIdx.x] += sh_Js[threadIdx.x + s * 36];
			__syncthreads();
		}
		if (threadIdx.x < 36)
			dev_partial_JtJs[blkIdx * 36 + threadIdx.x] = sh_Js[threadIdx.x];


		//reduction on jtf
		for (int s = blockDim.x / 2; s > 0; s >>= 1)
		{
			if (threadIdx.x < s)
			{
				for (int i = 0; i < 6; i++)
					sh_JtFs[6 * threadIdx.x + i] += sh_JtFs[6 * (threadIdx.x + s) + i];
			}
			__syncthreads();
		}

		//missing 8-th section during the above reduction
		if (threadIdx.x < 6)
			dev_partial_JtFs[blkIdx * 6 + threadIdx.x] = sh_JtFs[threadIdx.x] + sh_JtFs[threadIdx.x + 6 * 8];

		__syncthreads();
	}
}

//each cuda block handle reduction on one element (36 in total)
//assume: partial_JtJs_count < 1024
__global__
void jtj_rigid_align_reduction( float* dev_JtJ, float const* dev_partial_JtJs, int partial_JtJs_count)
{
	__shared__ float vals[MAX_THREADS_PER_BLOCK];

	int eleId = blockIdx.x;
	if (threadIdx.x < partial_JtJs_count)
		vals[threadIdx.x] = dev_partial_JtJs[36 * threadIdx.x + eleId];
	else
		vals[threadIdx.x] = 0.0f;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
			vals[threadIdx.x] += vals[threadIdx.x + s];
		__syncthreads();
	}

	if (threadIdx.x == 0)
		atomicAdd(&(dev_JtJ[eleId]), vals[0]);
}

//each cuda block handle reduction on one element (6 in total)
//assume: partial_JtFs_count < 1024
__global__
void jtf_rigid_align_reduction(float* dev_JtF, float const* dev_partial_JtFs, int partial_JtFs_count)
{
	__shared__ float vals[MAX_THREADS_PER_BLOCK];

	int eleId = blockIdx.x;
	if (threadIdx.x < partial_JtFs_count)
		vals[threadIdx.x] = dev_partial_JtFs[6 * threadIdx.x + eleId];
	else
		vals[threadIdx.x] = 0.0f;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
			vals[threadIdx.x] += vals[threadIdx.x + s];
		__syncthreads();
	}

	if (threadIdx.x == 0)
		atomicAdd(&(dev_JtF[eleId]), vals[0]);
}

//one cuda block with 32 threads
//rigid_transf.R = R * rigid_transf.R
//rigid_transf.T = R*rigid_transf.T + T
__global__
void apply_rigid_alignment_to_RigidTransf_kernel(float const* dev_R, float const*dev_T, RigidTransformCuda *dev_rigid_transf)
{
	__shared__ RigidTransformCuda sh_rigid_transf;
	__shared__ cuda_matrix_fixed<float, 3, 3> sh_R;
	__shared__ cuda_vector_fixed<float, 3> sh_T;

	int i = threadIdx.x;
	float* p_sh_R = sh_R.data_block();
	if (i < 9)
		p_sh_R[i] = dev_R[i];
	float *p_sh_T = sh_T.data_block();
	if (i < 3)
		p_sh_T[i] = dev_T[i];

	if (i==0)
		sh_rigid_transf = *dev_rigid_transf;
	__syncthreads();

	if (i == 0)
	{
		sh_rigid_transf.R = sh_R * sh_rigid_transf.R;
		sh_rigid_transf.T = sh_R*sh_rigid_transf.T + sh_T;

		dev_rigid_transf[0] = sh_rigid_transf;
	}
}

//use only one cuda block with 36 threads
__global__
void solve_and_update_R_T_rigid_align_kernel(float *dev_rod, float *dev_T, float const*dev_JtJ, float const*dev_JtF)
{
#define	JTJ_BLOCK_DIM 6
	__shared__ float sh_L[JTJ_BLOCK_DIM*JTJ_BLOCK_DIM];
	__shared__ float sh_x[JTJ_BLOCK_DIM];

	int i = threadIdx.x;
	int r = i / JTJ_BLOCK_DIM;
	int c = i % JTJ_BLOCK_DIM;

	//cholesky
	if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM)
	{
		sh_L[i] = dev_JtJ[i];
		if (r == c)
			sh_L[i] += 1.0e-6f; //to deal with the siutation of JtJ is all 0s.
	}
	__syncthreads();

	for (int k = 0; k < JTJ_BLOCK_DIM; k++)
	{
		if (k == c && c == r) // diagonal element for current iteration
			sh_L[i] = sqrtf(sh_L[i]);
		__syncthreads();

		if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM && 
			k == c && c < r) // elements in its lower column
			sh_L[i] /= sh_L[k * JTJ_BLOCK_DIM + k];
		__syncthreads();

		if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM && 
			k < c && c <= r) // elements in the remaining lower triangular matrix
			sh_L[i] -= sh_L[r*JTJ_BLOCK_DIM + k] * sh_L[c * JTJ_BLOCK_DIM + k];
		__syncthreads();
	}
	

	float diag = 0.0f;
	//solve
	if (i < JTJ_BLOCK_DIM)
	{
		diag = sh_L[i*JTJ_BLOCK_DIM + i];
		sh_x[i] = -dev_JtF[i] / diag;
	}

	for (int j = 0; j < JTJ_BLOCK_DIM - 1; j++)
	{
		__syncthreads();

		if (i < JTJ_BLOCK_DIM &&
			i > j)
			sh_x[threadIdx.x] -= sh_L[i*JTJ_BLOCK_DIM + j] / diag * sh_x[j];
	}
	// sh_x now is result of L y = b, solve for LT x = y next

	if (i < JTJ_BLOCK_DIM)
		sh_x[i] /= diag;

	for (int j = JTJ_BLOCK_DIM - 1; j >= 1; j--)
	{
		__syncthreads();

		if (i < j)
			sh_x[i] -= sh_L[j*JTJ_BLOCK_DIM + i] / diag * sh_x[j];
	}
	__syncthreads();


	//update rod and T
	if (i < 3)
	{
		dev_rod[i] += sh_x[i];
		dev_T[i] += sh_x[i + 3];
	}
}

//use only one cuda block with 36 threads
__global__
void solve_rigid_align_kernel(float *dev_rod_dx, float *dev_T_dx, float const*dev_JtJ, float const*dev_JtF, float opt_mu)
{
#define	JTJ_BLOCK_DIM 6
	__shared__ float sh_L[JTJ_BLOCK_DIM*JTJ_BLOCK_DIM];
	__shared__ float sh_x[JTJ_BLOCK_DIM];

	int i = threadIdx.x;
	int r = i / JTJ_BLOCK_DIM;
	int c = i % JTJ_BLOCK_DIM;

	//cholesky
	if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM)
	{
		sh_L[i] = dev_JtJ[i];
		if (r == c)
			sh_L[i] *= (1.0 + opt_mu);
		__syncthreads();
	}

	for (int k = 0; k < JTJ_BLOCK_DIM; k++)
	{
		if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM &&
			k == c && c == r) // diagonal element for current iteration
			sh_L[i] = sqrtf(sh_L[i]);
		__syncthreads();

		if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM &&
			k == c && c < r) // elements in its lower column
			sh_L[i] /= sh_L[k * JTJ_BLOCK_DIM + k];
		__syncthreads();

		if (i < JTJ_BLOCK_DIM*JTJ_BLOCK_DIM &&
			k < c && c <= r) // elements in the remaining lower triangular matrix
			sh_L[i] -= sh_L[r*JTJ_BLOCK_DIM + k] * sh_L[c * JTJ_BLOCK_DIM + k];
		__syncthreads();
	}

	//solve
	float diag = 0.0f;
	if (i < JTJ_BLOCK_DIM)
	{
		diag = sh_L[i*JTJ_BLOCK_DIM + i];
		sh_x[i] = -dev_JtF[i] / diag;
	}

	for (int j = 0; j < JTJ_BLOCK_DIM - 1; j++)
	{
		__syncthreads();

		if (i < JTJ_BLOCK_DIM && 
			i > j)
			sh_x[threadIdx.x] -= sh_L[i*JTJ_BLOCK_DIM + j] / diag * sh_x[j];
	}
	// sh_x now is result of L y = b, solve for LT x = y next
	if( i < JTJ_BLOCK_DIM)
		sh_x[i] /= diag;

	for (int j = JTJ_BLOCK_DIM - 1; j >= 1; j--)
	{
		__syncthreads();

		if (i < j)
			sh_x[i] -= sh_L[j*JTJ_BLOCK_DIM + i] / diag * sh_x[j];
	}
	__syncthreads();

	if (i < 3)
	{
		dev_rod_dx[i] = sh_x[i];
		dev_T_dx[i] = sh_x[i + 3];// *100.0f; //convert meters to center meters
	}
}

void EDMatchingHelperCudaImpl::solve_rigid_alignment(float opt_mu)
{
	solve_rigid_align_kernel<<<1, 36 >>>(dev_rod_dx_, dev_T_dx_, dev_JtJ_rigid_align_, dev_JtF_rigid_align_, opt_mu);
	m_checkCudaErrors();
}

void EDMatchingHelperCudaImpl::solve_and_update_rod_T_rigid_alignment()
{
	solve_and_update_R_T_rigid_align_kernel<<<1, 36>>>(dev_rod_, dev_T_, dev_JtJ_rigid_align_, dev_JtF_rigid_align_);
	m_checkCudaErrors();
}


void EDMatchingHelperCudaImpl::
allocate_memory_rigid_alignment(int vts_num_max)
{
	checkCudaErrors(cudaMalloc(&dev_rod_, sizeof(float)* 3));
	checkCudaErrors(cudaMalloc(&dev_R_, sizeof(float)* 9));
	checkCudaErrors(cudaMalloc(&dev_dR_drod_, sizeof(float)* 27));
	checkCudaErrors(cudaMalloc(&dev_T_, sizeof(float)* 3));
	checkCudaErrors(cudaMalloc(&dev_cost_rigid_align_, sizeof(float)));
	checkCudaErrors(cudaMalloc(&dev_JtJ_rigid_align_, sizeof(float)* 36));
	checkCudaErrors(cudaMalloc(&dev_JtF_rigid_align_, sizeof(float)* 6));

	checkCudaErrors(cudaMalloc(&dev_rod_dx_, sizeof(float)* 3));
	checkCudaErrors(cudaMalloc(&dev_T_dx_, sizeof(float)* 3));

	partial_JtJs_rigid_align_count_ = (vts_num_max + THREADS_PER_BLOCK_RIGIDJTJ - 1) / THREADS_PER_BLOCK_RIGIDJTJ;
	checkCudaErrors(cudaMalloc(&dev_partial_JtJs_rigid_align_, sizeof(float)* partial_JtJs_rigid_align_count_ * 36));
	checkCudaErrors(cudaMalloc(&dev_partial_JtFs_rigid_align_, sizeof(float)* partial_JtJs_rigid_align_count_ * 6));
}

void EDMatchingHelperCudaImpl::setup_rigid_alignment(cuda::gpu_size_data vts_num_gpu)
{
	checkCudaErrors(cudaMemsetAsync(dev_partial_JtJs_rigid_align_, 0, sizeof(float)*partial_JtJs_rigid_align_count_ * 36));
	checkCudaErrors(cudaMemsetAsync(dev_partial_JtFs_rigid_align_, 0, sizeof(float)*partial_JtJs_rigid_align_count_ * 6));
}

void EDMatchingHelperCudaImpl::init_rigid_alignment_to_identity()
{
	//TODO: Async
	checkCudaErrors(cudaMemset(dev_rod_, 0, sizeof(float)* 3));
	checkCudaErrors(cudaMemset(dev_T_, 0, sizeof(float)* 3));
}

void EDMatchingHelperCudaImpl::set_rigid_alignment(float rod[], float T[])
{
	//TODO: Async
	checkCudaErrors(cudaMemcpy(dev_rod_, rod, sizeof(float)* 3, cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(dev_T_, T, sizeof(float)* 3, cudaMemcpyHostToDevice));
}

void EDMatchingHelperCudaImpl::run_rigid_alignment(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vt_dim, float mu, int iter_max)
{
	init_rigid_alignment_to_identity();
	for (int i = 0; i < iter_max; i++)
	{
		compute_jtj_jtf_rigid_alignment(dev_vts, vts_num_gpu, vt_dim, mu);

		solve_and_update_rod_T_rigid_alignment();
	}

	//convert final rod to R
	rodrigues_kernel<<<1, 32>>>(dev_rod_, dev_R_, NULL);
}

void EDMatchingHelperCudaImpl::
apply_rigid_alignment_to_RigidTransf(RigidTransformCuda *dev_rigid_transf)
{
	apply_rigid_alignment_to_RigidTransf_kernel<<<1, 32>>>(dev_R_, dev_T_, dev_rigid_transf);
	m_checkCudaErrors();
}


void EDMatchingHelperCudaImpl::
compute_jtj_jtf_rigid_alignment(float const* dev_vts, cuda::gpu_size_data vts_num_gpu, int vt_dim, float mu)
{
	rodrigues_kernel<<<1, 32>>>(dev_rod_, dev_R_, dev_dR_drod_);
	m_checkCudaErrors();

	int threads_per_block = THREADS_PER_BLOCK_RIGIDJTJ;
	// TODO: evaluate if using (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block breaks code
	int blocks_per_grid = 256;//stride-loop trick// (vts_num_gpu.max_size + threads_per_block - 1) / threads_per_block;
	evaluate_jtj_jtf_rigid_align_kernel<1><<<blocks_per_grid, threads_per_block>>>(dev_partial_JtJs_rigid_align_, dev_partial_JtFs_rigid_align_, 
										dev_R_, dev_T_, dev_dR_drod_, 
										dev_vts, vts_num_gpu.dev_ptr, vt_dim, depth_width_, depth_height_, mu);
	m_checkCudaErrors();

	checkCudaErrors(cudaMemsetAsync(dev_JtJ_rigid_align_, 0, sizeof(float)* 36));
	checkCudaErrors(cudaMemsetAsync(dev_JtF_rigid_align_, 0, sizeof(float)* 6));
	jtj_rigid_align_reduction<<<36, MAX_THREADS_PER_BLOCK>>>(dev_JtJ_rigid_align_, dev_partial_JtJs_rigid_align_, partial_JtJs_rigid_align_count_);
	m_checkCudaErrors();

	jtf_rigid_align_reduction<<<6, MAX_THREADS_PER_BLOCK >>>(dev_JtF_rigid_align_, dev_partial_JtFs_rigid_align_, partial_JtJs_rigid_align_count_);
	m_checkCudaErrors();

}

void EDMatchingHelperCudaImpl::
evaluate_cost_rigid_alignment(float const* dev_vts, int vts_num, int vt_dim, float mu)
{
	rodrigues_kernel<<<1, 32>>>(dev_rod_, dev_R_, NULL);
	m_checkCudaErrors();

	//TODO: Async
	checkCudaErrors(cudaMemset(dev_cost_rigid_align_, 0, sizeof(float)));
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (vts_num + threads_per_block - 1) / threads_per_block;
	evaluate_cost_rigid_alignment_kernel<1><<<blocks_per_grid, threads_per_block>>>(dev_cost_rigid_align_, dev_R_, dev_T_,
										dev_vts, vts_num, vt_dim, depth_width_, depth_height_, mu);
	m_checkCudaErrors();

}

void EDMatchingHelperCudaImpl::
readout_R_T_rigid_alignment(float rod[3], float R[9], float dR_drod[27], float T[3])
{
	if (rod)
		checkCudaErrors(cudaMemcpy(rod, dev_rod_, sizeof(float)* 3, cudaMemcpyDeviceToHost));
	if (R)
		checkCudaErrors(cudaMemcpy(R, dev_R_, sizeof(float)* 9, cudaMemcpyDeviceToHost));
	if (dR_drod)
		checkCudaErrors(cudaMemcpy(dR_drod, dev_dR_drod_, sizeof(float)* 27, cudaMemcpyDeviceToHost));
	if (T)
		checkCudaErrors(cudaMemcpy(T, dev_T_, sizeof(float)* 3, cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCudaImpl::
readout_drod_dT_rigid_alignment(float drod[3], float dT[3])
{
	checkCudaErrors(cudaMemcpy(drod, dev_rod_dx_, sizeof(float)* 3, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(dT, dev_T_dx_, sizeof(float)* 3, cudaMemcpyDeviceToHost));
}


void EDMatchingHelperCudaImpl::
readout_JtJ_JtF_rigid_alignment(float jtj[36], float jtf[6])
{
	checkCudaErrors(cudaMemcpy(jtj, dev_JtJ_rigid_align_, sizeof(float)* 36, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(jtf, dev_JtF_rigid_align_, sizeof(float)* 6, cudaMemcpyDeviceToHost));
}

float EDMatchingHelperCudaImpl::readout_cost_rigid_alignment()
{
	float cost = 0.0f;
	checkCudaErrors(cudaMemcpy(&cost, dev_cost_rigid_align_, sizeof(float), cudaMemcpyDeviceToHost));
	return cost;
}

void EDMatchingHelperCudaImpl::
readout_partial_JtJs_JtFs(float *partial_jtjs, float *partial_jtfs)
{
	checkCudaErrors(cudaMemcpy(partial_jtjs, dev_partial_JtJs_rigid_align_, sizeof(float)* 36 * this->partial_JtJs_rigid_align_count_, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(partial_jtfs, dev_partial_JtFs_rigid_align_, sizeof(float)* 6 * this->partial_JtJs_rigid_align_count_, cudaMemcpyDeviceToHost));
}



#endif