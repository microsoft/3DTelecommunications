#include "lmsolver_gpu.h"
#include "matrix_types_cuda_io.h"
#include "cuda_utility.h"
#include "cuda_math_common.cuh"

#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

__global__
void adjust_trust_region_kernel(float *dev_cost_prev, float *dev_cost_curr, float *dev_cost_predicted_decrease, 
								int *dev_failed_steps_count, float *dev_mu,
								bool bSilent)
{
	const float cost_curr = *dev_cost_curr;
	const float cost_prev = *dev_cost_prev;
	const float cost_decrease_predicted = *dev_cost_predicted_decrease;
	float mu = *dev_mu;
	int failed_steps_count = *dev_failed_steps_count;
	float cost_decease_actual = cost_curr - cost_prev;

	float rho = cost_decease_actual / (cost_decrease_predicted+M_EPS);
	if (!bSilent) printf("cost_prev=%f; cost_new=%f; cost_decease_acutal=%f\n rho = %f\n", cost_prev, cost_curr, cost_decease_actual, rho);

	if (rho > 0)
	{
		*dev_failed_steps_count = 0;
		//rho  = 0.0    ---> mu *= 2.0;
		//rho  = 0.5    ---> mu *= 1.0;
		//rho >= 0.9368 ---> mu *= 1.0/3.0
		mu *= MAX(1.0f / 3.0f, 1.0f - powf(2.0f * rho - 1.0f, 3.0f));
		*dev_cost_prev = cost_curr;

		if (!bSilent) printf("OK<%g>!\n", mu);
	}
	else
	{
		*dev_failed_steps_count = failed_steps_count+1;
		mu *= powf(2.0f, failed_steps_count);

		if (!bSilent) printf("failed<%g>!\n", mu);
	}

	*dev_mu = mu;
}

__global__
void init_lm_solver_variables_kernel(float* dev_mu, int *dev_failed_steps_count, float mu)
{
	*dev_mu = mu;
	*dev_failed_steps_count = 0;
}

void LMSolverGPU::solve_impl(int frmIdx, bool bContinueLastSolving, int iters_max)
{
	problem_->gpu_return_internal_JtJ_JtF(jtj, jtf);
	if (!bContinueLastSolving)
		normal_equation_solver_->setup_solver(jtj, jtf);

	BlockMatrixFullCR const*A_bscr_gpu = normal_equation_solver_->JtJ_full();

	int failed_steps_count = 0;
	int iter = 0;
	problem_->gpu_evaluate_cost(dev_cost_prev, frmIdx, iter);
	init_lm_solver_variables_kernel<<<1,1>>>(dev_mu, dev_failed_steps_count, options_.mu_init);


	if (iters_max == -1)
		iters_max = options_.iter_max;

	while (iter < iters_max)
	{
		iter++;
		LOGGER()->debug("iter: %03d", iter);

		problem_->gpu_evaluate_JtJ_JtF(frmIdx, iter, options_.bUseOffDiagJtJData);

		normal_equation_solver_->update_JtJ_JtF(jtj, jtf);

		//change the diagonal term
		//jtj += mu*D
		normal_equation_solver_->regularize_JtJ(dev_mu);

		//solve the problem
		float* dev_dx = normal_equation_solver_->solve();

		//compute the predicated cost_decease;
		// dx'*(jtf-mu*D*dx)
		normal_equation_solver_->predicted_cost_decrease(dev_cost_decease_predicted, jtf.jtf, dev_dx, dev_mu);

		//update attentatively and compute the actual cost_decease
		problem_->gpu_updateX(dev_dx);
		problem_->gpu_evaluate_cost(dev_cost_curr, frmIdx, iter);

		adjust_trust_region_kernel<<<1,1>>>(dev_cost_prev, dev_cost_curr, dev_cost_decease_predicted, dev_failed_steps_count, dev_mu, !LOGGER()->check_verbosity(Logger::Trace));
	}
}
