#include "stdafx.h"
#include "lmsolver_gpu.h"
#include "utility.h"
#include "matrix_types_cuda_io.h"
#include "cuda_utility.h"
#include <vnl/vnl_vector.h>


bool LMSolverGPU::Solve(int frmIdx, bool bContinueLastSolving, int iters_max)
{
	problem_->gpu_return_internal_JtJ_JtF(jtj, jtf);
	if (!bContinueLastSolving)
		normal_equation_solver_->setup_solver(jtj, jtf);

	BlockMatrixFullCR const*A_bscr_gpu = normal_equation_solver_->JtJ_full();

	int failed_steps_count = 0;
	int iter = 0;
	double cost_prev = problem_->gpu_evaluate_cost(frmIdx, iter);
	double mu = options_.mu_init; //inverse of the trust region

	bool last_failed = false;

	char name[500];

	if (iters_max == -1)
		iters_max = options_.iter_max;

	while (iter < iters_max)
	{
		iter++;
		LOGGER()->debug("iter: %03d", iter);

		if (!last_failed)
			problem_->gpu_evaluate_JtJ_JtF(frmIdx, iter, options_.bUseOffDiagJtJData);

		//debug
		if (options_.bDebugMode && options_.dump_dir!= NULL)
		{
			printf("saving ori jtj jtf...");
			BlockedHessianMatrix jtj_ltri_cpu;
			readout_JtJBlockMatrixLTri_from_GPU(jtj, jtj_ltri_cpu);
			cv::Mat jtj_mat_ori = jtj_ltri_cpu.to_dense();
			sprintf(name, "%s/A_ori_%03d.txt", options_.dump_dir, iter);
			saveMatrixAscii(name, jtj_mat_ori);
			jtj_mat_ori.release();
			vnl_vector<float> jtf_ori;
			readout_global_memory_as_vector(jtf.jtf, jtf.n.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), jtf_ori);
			sprintf(name, "%s/jtf_ori_%03d.txt", options_.dump_dir, iter);
			saveVNLVectorASCAII(name, jtf_ori);
			printf("finished!\n");
			TerminateProcess(GetCurrentProcess(), 0);
		}

		problem_->gpu_backup_currentX();

		if (!last_failed)
			normal_equation_solver_->update_JtJ_JtF(jtj, jtf);

		//change the diagonal term
		//jtj += mu*D
		normal_equation_solver_->regularize_JtJ(mu);

		//solve the problem
		float* dev_dx = normal_equation_solver_->solve();

		//debug
		if (options_.bDebugMode && options_.dump_dir != NULL)
		{
			vnl_vector<float> dx;
			readout_global_memory_as_vector(dev_dx, jtf.n.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace)), dx);
			sprintf(name, "%s/dx_%03d.txt", options_.dump_dir, iter);
			saveVNLVectorASCAII(name, dx);
		}

		//compute the predicated cost_decease;
		// dx'*(jtf-mu*D*dx)
		double cost_decease_predicted = normal_equation_solver_->predicted_cost_decrease(jtf.jtf, dev_dx, mu);
		LOGGER()->debug("cost_decease_predicted=%f", cost_decease_predicted);

		//update attentatively and compute the actual cost_decease
		problem_->gpu_updateX(dev_dx);
		double cost_new = problem_->gpu_evaluate_cost(frmIdx, iter);
		double cost_decease_actual = cost_new - cost_prev;

		double rho = cost_decease_actual / cost_decease_predicted;
		LOGGER()->debug("cost_prev=%f; cost_new=%f; cost_decease_acutal=%f rho = %f", cost_prev, cost_new, cost_decease_actual, rho);

		if (abs(cost_decease_predicted) <= 1.0e-6f)
		{
			LOGGER()->debug("cost_decease_predicted == 0.0f!");
			return true;
		}

		if (rho > 0)
		{
			last_failed = false;
			failed_steps_count = 0;
			//rho  = 0.0    ---> mu *= 2.0;
			//rho  = 0.5    ---> mu *= 1.0;
			//rho >= 0.9368 ---> mu *= 1.0/3.0
			mu *= MAX(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
			cost_prev = cost_new;

			//check function tolerance
			if (-cost_decease_actual / cost_new < options_.function_tolerance)
			{
				LOGGER()->debug("function tolerance reached!");
				return true;
			}

			LOGGER()->debug("OK<%g>!", mu);
		}
		else
		{
			last_failed = true;
			failed_steps_count++;
			mu *= std::pow(2, failed_steps_count);

			//recover X
			problem_->gpu_recover_backupX();
			LOGGER()->debug("failed<%g>!", mu);
		}

	}

	return false;
}
