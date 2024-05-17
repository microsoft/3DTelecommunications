//helper double based atomicAdd
__device__ double atomicAddDoubleREG(double* address, double val)
{
	unsigned long long int* address_as_ull =
		(unsigned long long int*)address;
	unsigned long long int old = *address_as_ull, assumed;

	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed,
			__double_as_longlong(val +
				__longlong_as_double(assumed)));

		// Note: uses integer comparison to avoid hang in case of NaN (since NaN != NaN)
	} while (assumed != old);

	return __longlong_as_double(old);
}


//one ed nodes per thread
__global__
void EveluateCostTemporalTerm_Kernel_vRobust(float *dev_global_cost_temporal,
											DeformGraphNodeCuda const* dev_ed_nodes, 
											DeformGraphNodeCoreCuda const*dev_ed_nodes_initial_,
											RigidTransformCuda const* dev_rigid_transf, 
											RigidTransformCuda const* dev_rigid_transf_prev,
											int const* dev_ed_nodes_num, float tau_A, float tau_t, float w_temporal)
{
	const int ed_nodes_num = *dev_ed_nodes_num;

	if (blockIdx.x*blockDim.x > ed_nodes_num)
		return;

	__shared__ float costs[MAX_THREADS_PER_BLOCK];
	__shared__ RigidTransformCuda rigid_transf;
	__shared__ RigidTransformCuda rigid_transf_prev;
	if (threadIdx.x == 0)
	{
		rigid_transf = *dev_rigid_transf;
		rigid_transf_prev = *dev_rigid_transf_prev;
	}
	__syncthreads();

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	float cost = 0.0;
	if (id < ed_nodes_num)
	{
		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[id].A;
		cuda_matrix_fixed<float, 3, 3> A0 = dev_ed_nodes_initial_[id].A;
		A = rigid_transf.R * A;
		A0 = rigid_transf_prev.R * A0;
		float rr_A = 0.0f;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			float f = A(i, j) - A0(i, j);
			rr_A += f*f;
		}
		cost += phi(rr_A, tau_A);

		cuda_vector_fixed<float, 3> t = dev_ed_nodes[id].t;
		cuda_vector_fixed<float, 3> t0 = dev_ed_nodes_initial_[id].t;
		cuda_vector_fixed<float, 3> g = dev_ed_nodes[id].g;
		cuda_vector_fixed<float, 3> dt = rigid_transf.R*(g + t) + rigid_transf.T - (rigid_transf_prev.R*(g + t0) + rigid_transf_prev.T);
		float rr_t = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			float f = dt[i];
			rr_t += f*f;
		}
		cost += phi(rr_t, tau_t);
	}
	costs[threadIdx.x] = cost*w_temporal;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_temporal, costs[0]);
	}
}

//one ed nodes per cuda block
__global__
void ComputeJtJJtFTemporalTerm_Kernel_vRobust(float* dev_jtj, float *dev_jtf,
											  DeformGraphNodeCuda const* dev_ed_nodes,
											  DeformGraphNodeCoreCuda const*dev_ed_nodes_initial_,
											  RigidTransformCuda const* dev_rigid_transf,
											  RigidTransformCuda const* dev_rigid_transf_prev,
											  int const* dev_ed_nodes_num, float tau_A, float tau_t, float w_temporal
											  )
{
	const int ed_nodes_num = *dev_ed_nodes_num;
	if (blockIdx.x >= ed_nodes_num)
		return;

	int id = blockIdx.x;

	__shared__ RigidTransformCuda rigid_transf;
	__shared__ RigidTransformCuda rigid_transf_prev;
	__shared__ cuda_matrix_fixed<float, 3, 3> C; // C = R^T*R
	__shared__ cuda_matrix_fixed<float, 3, 3> A_;
	__shared__ cuda_matrix_fixed<float, 3, 3> A0_;
	__shared__ cuda_matrix_fixed<float, 3, 3> A;
	__shared__ cuda_matrix_fixed<float, 3, 3> A0;
	__shared__ float f[9];
	__shared__ float jtf[9];
	__shared__ float c, d, e;
	if (threadIdx.x == 0)
	{
		rigid_transf = *dev_rigid_transf;
		rigid_transf_prev = *dev_rigid_transf_prev;
	}
	if (threadIdx.x < 9)
	{
		int i = threadIdx.x / 3;
		int j = threadIdx.x % 3;
		A_(i, j) = dev_ed_nodes[id].A(i, j);
		A0_(i, j) = dev_ed_nodes_initial_[id].A(i, j);
	}
	__syncthreads();

	if (threadIdx.x < 9)
	{
		int i = threadIdx.x / 3;
		int j = threadIdx.x % 3;

		//A = R*A
		A(i, j) = rigid_transf.R(i, 0)*A_(0, j) +
				  rigid_transf.R(i, 1)*A_(1, j) +
				  rigid_transf.R(i, 2)*A_(2, j);
		A0(i, j) = rigid_transf_prev.R(i, 0)*A0_(0, j) +
				   rigid_transf_prev.R(i, 1)*A0_(1, j) +
				   rigid_transf_prev.R(i, 2)*A0_(2, j);

		f[threadIdx.x] = A(i, j) - A0(i, j);

		C(i, j) = rigid_transf.R(0, i)*rigid_transf.R(0, j) +
				  rigid_transf.R(1, i)*rigid_transf.R(1, j) +
				  rigid_transf.R(2, i)*rigid_transf.R(2, j);
	}
	__syncthreads();

	float *p_data_jtj = dev_jtj + id * 144;
	float *p_data_jtf = dev_jtf + id * 12;

	if (threadIdx.x < 9)
	{
		int i = threadIdx.x / 3;
		int j = threadIdx.x % 3;

		jtf[threadIdx.x] = rigid_transf.R(0, i)*f[j] +
						   rigid_transf.R(1, i)*f[j + 3] +
						   rigid_transf.R(2, i)*f[j + 6];
	}
	__syncthreads();

	if (threadIdx.x == 0)
	{
		float rr_A = 0.0f;
		for (int i = 0; i < 9; i++)
		{
			rr_A += f[i] * f[i];
		}
		float r_A = sqrtf(rr_A);
		robust_kernel_paras(r_A, tau_A, c, d, e);
	}
	__syncthreads();

	//for A
	if (threadIdx.x < 9)
	{
		int i = threadIdx.x / 3;
		int j = threadIdx.x % 3;

		p_data_jtj[(3 * i) * 12 + 3 * j] += C(i, j)*w_temporal*c;
		p_data_jtj[(3 * i + 1) * 12 + 3 * j + 1] += C(i, j)*w_temporal*c;
		p_data_jtj[(3 * i + 2) * 12 + 3 * j + 2] += C(i, j)*w_temporal*c;

		p_data_jtf[threadIdx.x] += jtf[threadIdx.x] * e*w_temporal;
	}
	__syncthreads();

	for (int idx = threadIdx.x; idx < 81; idx += blockDim.x)
	{
		int i = idx / 9;
		int j = idx % 9;
		p_data_jtj[i * 12 + j] += jtf[i] * jtf[j] * d * w_temporal;
	}
	__syncthreads();

	//for t
	cuda_vector_fixed<float, 3> t = dev_ed_nodes[id].t;
	cuda_vector_fixed<float, 3> t0 = dev_ed_nodes_initial_[id].t;
	cuda_vector_fixed<float, 3> g = dev_ed_nodes[id].g;
	cuda_vector_fixed<float, 3> dt = rigid_transf.R*(g + t) + rigid_transf.T - (rigid_transf_prev.R*(g + t0) + rigid_transf_prev.T);
	if (threadIdx.x == 0)
	{
		float rr_t = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			rr_t += dt[i]*dt[i];
		}
		float r_t = sqrtf(rr_t);
		robust_kernel_paras(r_t, tau_t, c, d, e);
	}
	__syncthreads();

	// J = R
	if (threadIdx.x < 3)
	{
		int i = threadIdx.x;
		float val = rigid_transf.R(0, i)*dt[0] + rigid_transf.R(1, i)*dt[1] + rigid_transf.R(2, i)*dt[2];
		p_data_jtf[9 + i] += val*e*w_temporal;
		jtf[i] = val;
	}
	__syncthreads();

	if (threadIdx.x < 9)
	{
		int i = threadIdx.x / 3;
		int j = threadIdx.x % 3;

		p_data_jtj[(9 + i) * 12 + 9 + j] += (C(i, j)*c + (jtf[i] * jtf[j])*d)*w_temporal;
	}
}

//one ed nodes per thread
__global__
void EveluateCostTemporalTerm_Kernel(float *dev_global_cost_temporal, 
									 DeformGraphNodeCuda const* dev_ed_nodes, DeformGraphNodeCoreCuda const*dev_ed_nodes_initial_, 
									 RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
									 int const* dev_ed_nodes_num, float w_temporal)
{
	const int ed_nodes_num = *dev_ed_nodes_num;
	if (blockDim.x*blockIdx.x > ed_nodes_num)
		return;

	__shared__ float costs[MAX_THREADS_PER_BLOCK];
	__shared__ RigidTransformCuda rigid_transf;
	__shared__ RigidTransformCuda rigid_transf_prev;
	if (threadIdx.x == 0)
	{
		rigid_transf = *dev_rigid_transf;
		rigid_transf_prev = *dev_rigid_transf_prev;
	}
	__syncthreads();

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	float cost = 0.0;
	if (id < ed_nodes_num)
	{
		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[id].A;
		cuda_matrix_fixed<float, 3, 3> A0 = dev_ed_nodes_initial_[id].A;
		A = rigid_transf.R * A;
		A0 = rigid_transf_prev.R * A0;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			float dif = A(i, j) - A0(i, j);
			cost += dif*dif;
		}

		cuda_vector_fixed<float, 3> t = dev_ed_nodes[id].t;
		cuda_vector_fixed<float, 3> t0 = dev_ed_nodes_initial_[id].t;
		cuda_vector_fixed<float, 3> g = dev_ed_nodes[id].g;
		cuda_vector_fixed<float, 3> dt = rigid_transf.R*(g + t) + rigid_transf.T - (rigid_transf_prev.R*(g + t0) + rigid_transf_prev.T);
		for (int i = 0; i<3; i++)
			cost += dt[i] * dt[i];
	}
	costs[threadIdx.x] = cost*w_temporal;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_temporal, costs[0]);
	}
}

__global__
void ComputeJtJJtFTemporalTerm_Kernel(float* dev_jtj, float *dev_jtf,
									  DeformGraphNodeCuda const* dev_ed_nodes, 
									  DeformGraphNodeCoreCuda const*dev_ed_nodes_initial_,
									  RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
									  int const* dev_ed_nodes_num, float w_temporal)
{
	const int ed_nodes_num = *dev_ed_nodes_num;

	if (blockDim.x*blockIdx.x > ed_nodes_num)
		return;

	int id = threadIdx.x + blockIdx.x*blockDim.x;

	__shared__ RigidTransformCuda rigid_transf;
	__shared__ RigidTransformCuda rigid_transf_prev;
	if (threadIdx.x == 0)
	{
		rigid_transf = *dev_rigid_transf;
		rigid_transf_prev = *dev_rigid_transf_prev;
	}
	__syncthreads();

	if (id < ed_nodes_num)
	{
		float *p_data_jtj = dev_jtj + id * 144;
		float *p_data_jtf = dev_jtf + id * 12;

		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[id].A;
		cuda_matrix_fixed<float, 3, 3> A0 = dev_ed_nodes_initial_[id].A;
		A = rigid_transf.R * A;
		A0 = rigid_transf_prev.R * A0;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			//f_ij = dot(r_i, a_j): r_i--ith row on R, a_j--j-th column on A
			float f = A(i, j) - A0(i, j);			
			//non-zeros of Jac_ij:  j + 3*m (m=0, 1, 2)
			//Jac_ij(j+3*m) = r_i(m) = R(i, m)

			//J^tJ
			for (int m = 0; m < 3; m++)
			{
				int r = j + 3 * m;
				for (int n = 0; n < 3; n++)
				{
					int c = j + 3 * n;
					p_data_jtj[r * 12 + c] += rigid_transf.R(i, m)*rigid_transf.R(i, n) * w_temporal;
				}
			}

			//J^tF
			for (int m = 0; m < 3; m++)
			{
				p_data_jtf[j + 3 * m] += rigid_transf.R(i, m)*f*w_temporal;
			}
		}

		cuda_vector_fixed<float, 3> t = dev_ed_nodes[id].t;
		cuda_vector_fixed<float, 3> t0 = dev_ed_nodes_initial_[id].t;
		cuda_vector_fixed<float, 3> g = dev_ed_nodes[id].g;
		cuda_vector_fixed<float, 3> dt = rigid_transf.R*(g + t) + rigid_transf.T - (rigid_transf_prev.R*(g + t0) + rigid_transf_prev.T);
		for (int i = 0; i < 3; i++)
		{
			float f = dt[i];
			//Jac_i = r_i

			//JtJ
			for (int m = 0; m < 3; m++)
			for (int n = 0; n < 3; n++)
				p_data_jtj[(m + 9) * 12 + n + 9] += rigid_transf.R(i, m)*rigid_transf.R(i, n)*w_temporal;

			//JtF
			for (int m = 0; m < 3; m++)
				p_data_jtf[m + 9] += rigid_transf.R(i, m)*f*w_temporal;
		}
	}
}

void EDMatchingHelperCudaImpl::
evaluate_cost_temporal_vRobust(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
							   RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
							   int const* dev_ed_nodes_num, float tau_A, float tau_t, float w_temporal, float* cost)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_temporal_, 0, sizeof(float)));
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	EveluateCostTemporalTerm_Kernel_vRobust<<<blocks_per_grid, threads_per_block>>>(dev_global_cost_temporal_, dev_ed_nodes, dev_ed_nodes_initial, 
																	dev_rigid_transf, dev_rigid_transf_prev,
																	dev_ed_nodes_num, tau_A, tau_t, w_temporal);

	m_checkCudaErrors();
	
	if (cost != NULL)
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_temporal_, sizeof(float), cudaMemcpyDeviceToHost));
}

void EDMatchingHelperCudaImpl::
compute_jtj_jtf_temporal_vRobust(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
								 RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
								 int const* dev_ed_nodes_num, 
								 float tau_A, float tau_t, float w_temporal)
{
	int threads_per_block = 32;
	int blocks_per_grid = ED_NODES_NUM_MAX;
	ComputeJtJJtFTemporalTerm_Kernel_vRobust<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtf_,
																					dev_ed_nodes, dev_ed_nodes_initial, 
																					dev_rigid_transf, dev_rigid_transf_prev,
																					dev_ed_nodes_num, tau_A, tau_t, w_temporal);
	m_checkCudaErrors();

}

void EDMatchingHelperCudaImpl::
evaluate_cost_temporal(DeformGraphNodeCuda const* dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
						RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
						int const* dev_ed_nodes_num, float w_temporal, float *cost)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_temporal_, 0, sizeof(float)));
	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	EveluateCostTemporalTerm_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_global_cost_temporal_, dev_ed_nodes, dev_ed_nodes_initial, 
																			dev_rigid_transf, dev_rigid_transf_prev,
																			dev_ed_nodes_num, w_temporal);

	m_checkCudaErrors();

	if (cost != NULL)
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_temporal_, sizeof(float), cudaMemcpyDeviceToHost));	
}

void EDMatchingHelperCudaImpl::
compute_jtj_jtf_temporal(DeformGraphNodeCuda const*dev_ed_nodes, DeformGraphNodeCoreCuda const* dev_ed_nodes_initial, 
						 RigidTransformCuda const* dev_rigid_transf, RigidTransformCuda const* dev_rigid_transf_prev,
						 int const* dev_ed_nodes_num, float w_temporal)
{
	int threads_per_block = 512;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	ComputeJtJJtFTemporalTerm_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtf_,
																dev_ed_nodes, dev_ed_nodes_initial, 
																dev_rigid_transf, dev_rigid_transf_prev,
																dev_ed_nodes_num, w_temporal);
	m_checkCudaErrors();

}

__global__
void EveluateCostRotTerm_Kernel(float *dev_global_cost_rot, 
								DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int const* dev_ed_nodes_num, float w_rot)
{
	extern __shared__ float costs[];

	const int ed_nodes_num = *dev_ed_nodes_num;
	if (blockDim.x*blockIdx.x > ed_nodes_num)
		return;

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	float cost = 0.0;
	if (id < ed_nodes_num)
	{
		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[id].A;
		for (int k = 0; k < 3; k++)
		{
			cuda_vector_fixed<float, 3> r(A[k][0], A[k][1], A[k][2]);

			//cost
			float f = dot_product(r, r) - 1.0f;
			cost += f*f*w_rot;
		}
		for (int i = 0; i < 3; i++)
		{
			cuda_vector_fixed<float, 3> r_i(A[i][0], A[i][1], A[i][2]);
			for (int j = i + 1; j < 3; j++)
			{
				cuda_vector_fixed<float, 3> r_j(A[j][0], A[j][1], A[j][2]);

				//cost
				float f = dot_product(r_i, r_j);
				cost += f*f*w_rot;
			}
		}
		//det(A)
		float w_detA = w_rot * 16.0f;
		float f = cuda_det(A) - 1.0f;
		cost += f*f*w_detA;

	}
	costs[threadIdx.x] = cost;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_rot, costs[0]);
	}
}

__global__
__launch_bounds__(64, 2)
void ComputeJtJJtFRotTerm_Kernel(float* dev_jtj, float *dev_jtf, DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, int const* dev_ed_nodes_num, float w_rot_sqrt)
{
	const int ed_nodes_num = *dev_ed_nodes_num;

	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if (id < ed_nodes_num)
	{
		cuda_matrix_fixed<float, 3, 3> A = dev_ed_nodes[id].A;
		float *p_data_jtj = dev_jtj + id * 144;
		float *p_data_jtf = dev_jtf + id * 12;
		// ||r||^2-1
		for (int k = 0; k < 3; k++)
		{
			cuda_vector_fixed<float, 3> r(A[k][0], A[k][1], A[k][2]);

			//cost
			float f = (dot_product(r, r) - 1.0f)*w_rot_sqrt;

			//jacobian
			cuda_vector_fixed<float, 3> df_dr;
			df_dr[0] = 2.0f*r[0] * w_rot_sqrt;
			df_dr[1] = 2.0f*r[1] * w_rot_sqrt;
			df_dr[2] = 2.0f*r[2] * w_rot_sqrt;

			//gradient
			for (int m = 0; m < 3; m++)
				p_data_jtf[3 * k + m] += df_dr[m] * f;

			//hessian
			for (int m = 0; m < 3; m++)
			for (int n = 0; n < 3; n++)
				p_data_jtj[(3 * k + m) * 12 + 3 * k + n] += df_dr[m] * df_dr[n];
		}
		// r_i^T r_j 
		for (int i = 0; i < 3; i++)
		{
			cuda_vector_fixed<float, 3> r_i(A[i][0], A[i][1], A[i][2]);
			for (int j = i + 1; j < 3; j++)
			{
				cuda_vector_fixed<float, 3> r_j(A[j][0], A[j][1], A[j][2]);

				//cost
				float f = dot_product(r_i, r_j) * w_rot_sqrt;

				//jacobian
				cuda_vector_fixed<float, 3> df_dr_i = r_j * w_rot_sqrt;
				cuda_vector_fixed<float, 3> df_dr_j = r_i * w_rot_sqrt;

				//gradient
				for (int m = 0; m < 3; m++)
				{
					p_data_jtf[3 * i + m] += df_dr_i[m] * f;
					p_data_jtf[3 * j + m] += df_dr_j[m] * f;
				}

				//hessian
				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					p_data_jtj[(3 * i + m)*12 + 3 * i + n] += df_dr_i[m] * df_dr_i[n];

				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					p_data_jtj[(3 * j + m)*12 + 3 * j + n] += df_dr_j[m] * df_dr_j[n];

				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					p_data_jtj[(3 * i + m)*12 + 3 * j + n] += df_dr_i[m] * df_dr_j[n];

				for (int m = 0; m < 3; m++)
				for (int n = 0; n < 3; n++)
					p_data_jtj[(3 * j + m)*12 + 3 * i + n] += df_dr_j[m] * df_dr_i[n];
			}
		}

		//det(A)
		float w_detA = w_rot_sqrt * 4.0f;
		float f = (cuda_det(A) - 1.0f) * w_detA;

		//jacobian
		cuda_vector_fixed<float, 9> df_dA;
		df_dA[0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * w_detA;
		df_dA[1] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * w_detA;
		df_dA[2] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * w_detA;
		df_dA[3] = (A[2][1] * A[0][2] - A[0][1] * A[2][2]) * w_detA;
		df_dA[4] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * w_detA;
		df_dA[5] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * w_detA;
		df_dA[6] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * w_detA;
		df_dA[7] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * w_detA;
		df_dA[8] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * w_detA;

		//gradient
		for (int m = 0; m < 9; m++)
			p_data_jtf[m] += df_dA[m] * f;

		//hessian
		for (int m = 0; m < 9; m++)
		for (int n = 0; n < 9; n++)
			p_data_jtj[m*12+n] += df_dA[m] * df_dA[n];
	}
}


bool EDMatchingHelperCudaImpl::compute_jtj_jtf_rot(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num, double w_rot)
{
	float w_rot_sqrt = std::sqrt(w_rot);
	int threads_per_block = 64;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	ComputeJtJJtFRotTerm_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtf_, dev_ed_nodes, dev_ed_nodes_num, w_rot_sqrt);
	m_checkCudaErrors();

	return true;
}

void EDMatchingHelperCudaImpl::evaluate_cost_rot(DeformGraphNodeCuda *dev_ed_nodes, int const* dev_ed_nodes_num, double w_rot, float *cost)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_rot_, 0, sizeof(float)));

	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	EveluateCostRotTerm_Kernel<<<blocks_per_grid, threads_per_block, threads_per_block * 4>>>(dev_global_cost_rot_, dev_ed_nodes, dev_ed_nodes_num, w_rot);
	m_checkCudaErrors();

	if (cost != NULL)
	{
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_rot_, sizeof(float), cudaMemcpyDeviceToHost));
	}
}

//each thread handles one reg constraint: one node pair
__global__
void ComputeJtJRegTermOffDiag_Kernel(float* dev_jtj,
							 short2 const* __restrict__ dev_jtj_2d_infos,
							  DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
							  int const* dev_ed_nodes_num_all_levels,
							  ushort2 const* __restrict__ dev_reg_node_pairs_list_buf, int const* dev_ed_reg_pairs_num,
							  float sigma_nodes_dist, float w_reg
							  )
{
	const int node_pairs_num = *dev_ed_reg_pairs_num;

	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < node_pairs_num)
	{
		const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

		ushort2 ndId_pair = dev_reg_node_pairs_list_buf[id];
		int ndIdx_i = ndId_pair.x; //ndIdx_i > ndIdx_j
		int ndIdx_j = ndId_pair.y;
		
		int data_offset = dev_jtj_2d_infos[ndIdx_i*ed_nodes_num + ndIdx_j].x * 144;

		cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_i].g - dev_ed_nodes[ndIdx_j].g; //gi-gj

		//dist of ed nodes after warp
		cuda_vector_fixed<float, 3> dg = tmp + dev_ed_nodes_initial[ndIdx_i].t - dev_ed_nodes_initial[ndIdx_j].t;
		float dist2 = dot_product(dg, dg);
		float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));
		w_reg *= w_dg;

		float* p_data = dev_jtj + data_offset;
		for (int i = 0; i < 3; i++)
		{
			float val = w_reg*tmp[i];
			p_data[12 * 9 + i] -= val;
			p_data[12 * 10 + 3 + i] -= val;
			p_data[12 * 11 + 6 + i] -= val;

			p_data[i * 12 + 9] += val;
			p_data[(i + 3) * 12 + 10] += val;
			p_data[(i + 6) * 12 + 11] += val;

			p_data[(9 + i) * 12 + (9 + i)] -= 2.0*w_reg;
		}
	}
}

//each thread handles one ed node
__global__
void ComputeJtJRegTermDiag_Kernel(float* dev_jtj, DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
									int const*dev_ed_nodes_num_all_levels, float sigma_nodes_dist, float w_reg)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

	int ndIdx_i = threadIdx.x + blockDim.x*blockIdx.x;
	if (ndIdx_i < ed_nodes_num)
	{
		for (int k = 0; k < EDNODE_NN_MAX; k++)
		{
			int ndIdx_j = dev_ed_nodes[ndIdx_i].neighbors[k];
			if (ndIdx_j >= 0)
			{
				cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_j].g - dev_ed_nodes[ndIdx_i].g; //gj-gi

				//dist of ed nodes after warp
				cuda_vector_fixed<float, 3> dg = tmp + dev_ed_nodes_initial[ndIdx_j].t - dev_ed_nodes_initial[ndIdx_i].t;
				float dist2 = dot_product(dg, dg);
				float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));
				float w = w_dg*w_reg;

				float *p_data = dev_jtj + ndIdx_i * 144; //diagnal jtj blocks are in order and go first before off-diagnal blocks

				for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
				{
					float val = tmp[i] * tmp[j] * w;
					p_data[i * 12 + j] += val;
					p_data[(i + 3) * 12 + j + 3] += val;
					p_data[(i + 6) * 12 + j + 6] += val;
				}

				for (int i = 0; i < 3; i++)
				{
					float val = tmp[i] * w;
					p_data[12 * 9 + i] += val;
					p_data[12 * 10 + 3 + i] += val;
					p_data[12 * 11 + 6 + i] += val;

					p_data[i * 12 + 9] += val;
					p_data[(i + 3) * 12 + 10] += val;
					p_data[(i + 6) * 12 + 11] += val;

					p_data[(9 + i) * 12 + (9 + i)] += 2.0*w;
				}
			}
		}
	}
}

//each thread handles one ed node
__global__
void ComputeJtFRegTerm_Kernel(float* dev_jtf, DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
								int const* dev_ed_nodes_num_all_levels, float sigma_nodes_dist, float w_reg)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

	int ndIdx_i = threadIdx.x + blockDim.x*blockIdx.x;
	if (ndIdx_i < ed_nodes_num)
	{
		for (int k = 0; k < EDNODE_NN_MAX; k++)
		{
			int ndIdx_j = dev_ed_nodes[ndIdx_i].neighbors[k];
			if (ndIdx_j >= 0)
			{
				cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_j].g - dev_ed_nodes[ndIdx_i].g; //gj-gi

				//dist of ed nodes after warp
				cuda_vector_fixed<float, 3> dg = tmp + dev_ed_nodes_initial[ndIdx_j].t - dev_ed_nodes_initial[ndIdx_i].t;
				float dist2 = dot_product(dg, dg);
				float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));
				float w = w_dg*w_reg;

				cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
				cuda_vector_fixed<float, 3> f1 = dev_ed_nodes[ndIdx_i].A*tmp - tmp + ti_m_tj;

				float *p_data = dev_jtf + ndIdx_i * 12; //diagnal jtj blocks are in order and go first before off-diagnal blocks

				for (int i = 0; i < 3; i++)
				{
					p_data[i] += tmp[i] * f1[0] * w;
					p_data[3 + i] += tmp[i] * f1[1] * w;
					p_data[6 + i] += tmp[i] * f1[2] * w;
					p_data[9 + i] += f1[i] * w;
				}

				cuda_vector_fixed<float, 3> f2 = -dev_ed_nodes[ndIdx_j].A*tmp + tmp - ti_m_tj;
				for (int i = 0; i < 3; i++)
					p_data[9 + i] -= f2[i] * w;
			}
		}
	}
}

__global__
void EveluateCostRegTerm_Kernel(float* dev_global_cost_reg,
								DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
								int const* dev_ed_nodes_num_all_levels,
								ushort2 const* __restrict__ dev_reg_node_pairs_list_buf, int const* dev_ed_reg_pairs_num,
								float sigma_nodes_dist, float w_reg)
{
	const int node_pairs_num = *dev_ed_reg_pairs_num;

	extern __shared__ float costs[];
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	float cost = 0.0f;
	if (id < node_pairs_num)
	{
		ushort2 ndId_pair = dev_reg_node_pairs_list_buf[id];
		int ndIdx_i = ndId_pair.x;
		int ndIdx_j = ndId_pair.y;

		cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_j].g - dev_ed_nodes[ndIdx_i].g; //gj-gi

		//dist of ed nodes after warp
		cuda_vector_fixed<float, 3> dg = tmp + dev_ed_nodes_initial[ndIdx_j].t - dev_ed_nodes_initial[ndIdx_i].t;
		float dist2 = dot_product(dg, dg);
		float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));

		cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
		cuda_vector_fixed<float, 3> f1 = dev_ed_nodes[ndIdx_i].A*tmp - tmp + ti_m_tj;
		cost = dot_product(f1, f1);

		cuda_vector_fixed<float, 3> f2 = -dev_ed_nodes[ndIdx_j].A*tmp + tmp - ti_m_tj;
		cost += dot_product(f2, f2);

		cost *= (w_reg*w_dg);
	}
	costs[threadIdx.x] = cost;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_reg, costs[0]);
	}
}

bool EDMatchingHelperCudaImpl::compute_jtj_jtf_reg(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
													int const* dev_ed_nodes_num_all_levels, float sigma_nodes_dist, float w_reg)
{
	int threads_per_block = 64;
	int blocks_per_grid = (ed_reg_pairs_count_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	ComputeJtJRegTermOffDiag_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtj_2d_infos_,
																			dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels,
																			dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.dev_ptr,
																			sigma_nodes_dist, w_reg);
	m_checkCudaErrors();

	threads_per_block = 64;
	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	ComputeJtJRegTermDiag_Kernel<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels, sigma_nodes_dist, w_reg);
	m_checkCudaErrors();

	ComputeJtFRegTerm_Kernel<<<blocks_per_grid, threads_per_block >>>(dev_jtf_, dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels, sigma_nodes_dist, w_reg);
	m_checkCudaErrors();

	return true;
}

void EDMatchingHelperCudaImpl::evaluate_cost_reg(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
												int const* dev_ed_nodes_num_all_levels, float sigma_nodes_dist, float w_reg, float* cost)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_reg_, 0, sizeof(float)));

	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ed_reg_pairs_count_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	EveluateCostRegTerm_Kernel<<<blocks_per_grid, threads_per_block, threads_per_block * 4>>>(dev_global_cost_reg_, 
			dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels,
			dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.dev_ptr, sigma_nodes_dist, w_reg);
	m_checkCudaErrors();

	if (cost != NULL)
	{
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_reg_, sizeof(float), cudaMemcpyDeviceToHost));
	}
}

//each thread handles one reg constraint: one node pair
__global__
void ComputeJtJRegTermOffDiag_Kernel_vRobust(float* dev_jtj,
											short2 const* __restrict__ dev_jtj_2d_infos,
											DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial,
											int const* dev_ed_nodes_num_all_levels,
											ushort2 const* __restrict__ dev_reg_node_pairs_list_buf, int const *dev_ed_reg_pairs_num,
											float sigma_nodes_dist, float tau, float w_reg
											)
{
	const int node_pairs_num = *dev_ed_reg_pairs_num;
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	if (id < node_pairs_num)
	{
		const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

		ushort2 ndId_pair = dev_reg_node_pairs_list_buf[id];
		int ndIdx_i = ndId_pair.x; //ndIdx_i > ndIdx_j
		int ndIdx_j = ndId_pair.y;

		int data_offset = dev_jtj_2d_infos[ndIdx_i*ed_nodes_num + ndIdx_j].x * 144;

		cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_i].g - dev_ed_nodes[ndIdx_j].g; //gi-gj
		cuda_vector_fixed<float, 3> ti_m_tj_init = dev_ed_nodes_initial[ndIdx_i].t - dev_ed_nodes_initial[ndIdx_j].t;//ti-tj

		//dist of ed nodes after warp
		cuda_vector_fixed<float, 3> dg = tmp + ti_m_tj_init;
		float dist2 = dot_product(dg, dg);
		float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));
		float w = w_reg * w_dg;

		float* p_data = dev_jtj + data_offset;

		cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
		cuda_vector_fixed<float, 3> f1 = -(dev_ed_nodes[ndIdx_i].A*tmp) + tmp + ti_m_tj;
		float rr1 = dot_product(f1, f1);
		float r1 = sqrtf(rr1);
		float c, d, e;
		robust_kernel_paras(r1, tau, c, d, e);

		cuda_matrix_fixed<float, 3, 3> K;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K(i, j) = (i == j) ? c : 0.0f + d*f1[i] * f1[j];

		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			p_data[(i)* 12 + 9 + j] += K(0, j)*tmp[i] * w;
			p_data[(i + 3) * 12 + 9 + j] += K(1, j)*tmp[i] * w;
			p_data[(i + 6) * 12 + 9 + j] += K(2, j)*tmp[i] * w;

			p_data[(9 + i) * 12 + (9 + j)] -= K(i, j)*w;
		}


		cuda_vector_fixed<float, 3> f2 = dev_ed_nodes[ndIdx_j].A*tmp - tmp - ti_m_tj;
		float rr2 = dot_product(f2, f2);
		float r2 = sqrtf(rr2);
		robust_kernel_paras(r2, tau, c, d, e);

		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			K(i, j) = (i == j) ? c : 0.0f + d*f2[i] * f2[j];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			p_data[(9 + i) * 12 + j] -= K(i, 0)*tmp[j] * w;
			p_data[(9 + i) * 12 + j + 3] -= K(i, 1)*tmp[j] * w;
			p_data[(9 + i) * 12 + j + 6] -= K(i, 2)*tmp[j] * w;

			p_data[(9 + i) * 12 + (9 + j)] -= K(i, j)*w;
		}
	}
}


//64 threads per cuda block
//each block handles one ed node
__global__
__launch_bounds__(64, 5)
void ComputeJtJRegTermDiag_Kernel_vRobust(float* dev_jtj, DeformGraphNodeCuda const* __restrict__ dev_ed_nodes, DeformGraphNodeCoreCuda* dev_ed_nodes_initial,
	int const* dev_ed_nodes_num_all_levels,
	float sigma_nodes_dist, float tau, float w_reg)
{
	__shared__ float JtK[12][3]; //J^T*K
	__shared__ float K[3][3];

	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;

	int ndIdx_i = blockIdx.x;
	int tid = threadIdx.x;
	if (ndIdx_i < ed_nodes_num)
	{
		float* p_data = dev_jtj + ndIdx_i * 144; //diagnal jtj blocks are in order and go first before off-diagnal blocks
		for (int k = 0; k < EDNODE_NN_MAX; k++)
		{
			int ndIdx_j = dev_ed_nodes[ndIdx_i].neighbors[k];
			if (ndIdx_j >= 0)
			{
				cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_i].g - dev_ed_nodes[ndIdx_j].g; //gi-gj
				cuda_vector_fixed<float, 3> ti_m_tj_init = dev_ed_nodes_initial[ndIdx_i].t - dev_ed_nodes_initial[ndIdx_j].t;//ti-tj

				//dist of ed nodes after warp
				cuda_vector_fixed<float, 3> dg = tmp + ti_m_tj_init;
				float dist2 = dot_product(dg, dg);
				float w_dg = expf(-dist2 / (2.0f * sigma_nodes_dist * sigma_nodes_dist));
				float w = w_dg * w_reg;

				

				cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
				cuda_vector_fixed<float, 3> f1 = -(dev_ed_nodes[ndIdx_i].A * tmp) + tmp + ti_m_tj;
				float rr1 = dot_product(f1, f1);
				float r1 = sqrtf(rr1);
				float c, d, e;
				robust_kernel_paras(r1, tau, c, d, e);

				if (tid < 9)
				{
					int i = tid / 3;
					int j = tid % 3;
					K[i][j] = (i == j) ? c : 0.0f + d * f1[i] * f1[j];
				}

				if (tid < 9)
				{
					int i = tid / 3;
					int j = tid % 3;

					JtK[i][j] = -tmp[i] * K[0][j];
					JtK[i + 3][j] = -tmp[i] * K[1][j];
					JtK[i + 6][j] = -tmp[i] * K[2][j];
					JtK[i + 9][j] = K[i][j];
				}			

				__syncthreads();

				cuda_vector_fixed<float, 3> f2 = dev_ed_nodes[ndIdx_j].A * tmp - tmp - ti_m_tj;
				float rr2 = dot_product(f2, f2);
				float r2 = sqrtf(rr2);
				robust_kernel_paras(r2, tau, c, d, e);

				if (tid < 36)
				{
					int i = tid / 3;
					int j = tid % 3;

					//NOTE: this fixes concurrent memory access, but nature of floating point atomicAdd introduces nondeterminism in calculated results!
					atomicAdd(&p_data[i * 12 + j]    , -JtK[i][0] * tmp[j] * w);
					atomicAdd(&p_data[i * 12 + j + 3], -JtK[i][1] * tmp[j] * w);
					atomicAdd(&p_data[i * 12 + j + 6], -JtK[i][2] * tmp[j] * w);
					atomicAdd(&p_data[i * 12 + j + 9], +JtK[i][j] * w);

					if (tid < 9)
					{
						atomicAdd(&p_data[(i + 9) * 12 + (j + 9)], ((i == j) ? c : 0.0f + d * f2[i] * f2[j]) * w);
					}
				}		
			}
		} // end for neighbor loop
	}
}

//each thread handles one ed node
__global__
void ComputeJtFRegTerm_Kernel_vRobust(float* dev_jtf, DeformGraphNodeCuda const* dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
									 int const* dev_ed_nodes_num_all_levels,
										float sigma_nodes_dist, float tau, float w_reg)
{
	const int ed_nodes_num = *dev_ed_nodes_num_all_levels;
	int ndIdx_i = threadIdx.x + blockDim.x*blockIdx.x;
	if (ndIdx_i < ed_nodes_num)
	{
		for (int k = 0; k < EDNODE_NN_MAX; k++)
		{
			int ndIdx_j = dev_ed_nodes[ndIdx_i].neighbors[k];
			if (ndIdx_j >= 0)
			{
				cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_i].g - dev_ed_nodes[ndIdx_j].g; //gi-gj
				cuda_vector_fixed<float, 3> ti_m_tj_init = dev_ed_nodes_initial[ndIdx_i].t - dev_ed_nodes_initial[ndIdx_j].t;//ti-tj

				//dist of ed nodes after warp
				cuda_vector_fixed<float, 3> dg = tmp + ti_m_tj_init;
				float dist2 = dot_product(dg, dg);
				float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));
				float w = w_dg*w_reg;

				cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
				cuda_vector_fixed<float, 3> f1 = -(dev_ed_nodes[ndIdx_i].A*tmp) + tmp + ti_m_tj;
				float rr1 = dot_product(f1, f1);
				float r1 = sqrtf(rr1);
				float c, d, e;
				robust_kernel_paras(r1, tau, c, d, e);

				float *p_data = dev_jtf + ndIdx_i * 12; //diagnal jtj blocks are in order and go first before off-diagnal blocks

				for (int i = 0; i < 3; i++)
				{
					p_data[i] -= tmp[i] * f1[0] * w * e;
					p_data[3 + i] -= tmp[i] * f1[1] * w * e;
					p_data[6 + i] -= tmp[i] * f1[2] * w * e;
					p_data[9 + i] += f1[i] * w * e;
				}

				cuda_vector_fixed<float, 3> f2 = dev_ed_nodes[ndIdx_j].A*tmp - tmp - ti_m_tj;
				float rr2 = dot_product(f2, f2);
				float r2 = sqrtf(rr2);
				robust_kernel_paras(r2, tau, c, d, e);
				for (int i = 0; i < 3; i++)
					p_data[9 + i] -= f2[i] * w * e;
			}
		}
	}
}

__global__
void EveluateCostRegTerm_Kernel_vRobust(float *dev_global_cost_reg,
										DeformGraphNodeCuda const* dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
										int const*dev_ed_nodes_num_all_levels,
										ushort2 const* dev_reg_node_pairs_list_buf, int const* dev_ed_reg_pairs_num,
										float sigma_nodes_dist, float tau, float w_reg)
{
	const int node_pairs_num = *dev_ed_reg_pairs_num;

	extern __shared__ float costs[];
	int id = threadIdx.x + blockDim.x*blockIdx.x;
	float cost = 0.0f;
	if (id < node_pairs_num)
	{
		ushort2 ndId_pair = dev_reg_node_pairs_list_buf[id];
		int ndIdx_i = ndId_pair.x;
		int ndIdx_j = ndId_pair.y;

		cuda_vector_fixed<float, 3> tmp = dev_ed_nodes[ndIdx_i].g - dev_ed_nodes[ndIdx_j].g; //gi-gj
		cuda_vector_fixed<float, 3> ti_m_tj_init = dev_ed_nodes_initial[ndIdx_i].t - dev_ed_nodes_initial[ndIdx_j].t;//ti-tj

		//dist of ed nodes after warp
		cuda_vector_fixed<float, 3> dg = tmp + ti_m_tj_init;
		float dist2 = dot_product(dg, dg);
		float w_dg = expf(-dist2 / (2.0f*sigma_nodes_dist*sigma_nodes_dist));

		cuda_vector_fixed<float, 3> ti_m_tj = dev_ed_nodes[ndIdx_i].t - dev_ed_nodes[ndIdx_j].t;//ti-tj
		cuda_vector_fixed<float, 3> f1 = -(dev_ed_nodes[ndIdx_i].A*tmp) + tmp + ti_m_tj;
		float rr = dot_product(f1, f1);
		cost += phi(rr, tau);

		cuda_vector_fixed<float, 3> f2 = dev_ed_nodes[ndIdx_j].A*tmp - tmp - ti_m_tj;
		rr = dot_product(f2, f2);
		cost += phi(rr, tau);

		cost *= (w_reg*w_dg);
	}
	costs[threadIdx.x] = cost;
	__syncthreads();

	for (int s = blockDim.x / 2; s > 0; s >>= 1)
	{
		if (threadIdx.x < s)
		{
			costs[threadIdx.x] += costs[threadIdx.x + s];
		}
		__syncthreads();
	}

	if (threadIdx.x == 0)
	{
		atomicAdd(dev_global_cost_reg, costs[0]);
	}
}


bool EDMatchingHelperCudaImpl::
compute_jtj_jtf_reg_vRobust(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, 
							int const* dev_ed_nodes_num_all_levels,
							float sigma_nodes_dist, float tau, float w_reg)
{
	int threads_per_block = 64;
	int blocks_per_grid = (ed_reg_pairs_count_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	ComputeJtJRegTermOffDiag_Kernel_vRobust<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_jtj_2d_infos_, 
															dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels, 
															dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.dev_ptr,
															sigma_nodes_dist, tau, w_reg);
	m_checkCudaErrors();

	threads_per_block = 64;
	blocks_per_grid = ED_NODES_NUM_MAX;
	ComputeJtJRegTermDiag_Kernel_vRobust<<<blocks_per_grid, threads_per_block>>>(dev_jtj_, dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels, sigma_nodes_dist, tau, w_reg);
	m_checkCudaErrors();

	blocks_per_grid = (ED_NODES_NUM_MAX + threads_per_block - 1) / threads_per_block;
	ComputeJtFRegTerm_Kernel_vRobust<<<blocks_per_grid, threads_per_block>>>(dev_jtf_, dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels, sigma_nodes_dist, tau, w_reg);
	m_checkCudaErrors();

	return true;
}

void EDMatchingHelperCudaImpl::
evaluate_cost_reg_vRobust(DeformGraphNodeCuda *dev_ed_nodes, DeformGraphNodeCoreCuda *dev_ed_nodes_initial, int const* dev_ed_nodes_num_all_levels, 
													float sigma_nodes_dist, float tau, float w_reg, float *cost)
{
	checkCudaErrors(cudaMemsetAsync(dev_global_cost_reg_, 0, sizeof(float)));

	int threads_per_block = MAX_THREADS_PER_BLOCK;
	int blocks_per_grid = (ed_reg_pairs_count_gpu_.max_size + threads_per_block - 1) / threads_per_block;
	EveluateCostRegTerm_Kernel_vRobust<<<blocks_per_grid, threads_per_block, threads_per_block * 4>>>(dev_global_cost_reg_, dev_ed_nodes, dev_ed_nodes_initial, dev_ed_nodes_num_all_levels,
																			dev_reg_node_pairs_list_buf_, ed_reg_pairs_count_gpu_.dev_ptr, sigma_nodes_dist, tau, w_reg);
	m_checkCudaErrors();

	if (cost != NULL)
	{
		checkCudaErrors(cudaMemcpy(cost, dev_global_cost_reg_, sizeof(float), cudaMemcpyDeviceToHost));
	}
}

