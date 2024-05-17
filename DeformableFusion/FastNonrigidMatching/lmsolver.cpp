#include "stdafx.h"
#include "lmsolver.h"
#include "utility.h"


bool LMSolver::Solve(int frmIdx, bool bDummy, int iters_max)
{
	//clean up the leftovers of the last solve
	this->jtj_.free_space();
	this->jtf_.clear();
	this->D.clear();

	problem_->init(jtj_, jtf_);

	int failed_steps_count = 0;
	int iter = 0;
	double cost_prev = problem_->evaluateCurrentCost(frmIdx, iter);
	double mu = options_.mu_init; //inverse of the trust region

	char name[500];

	if (iters_max == -1)
		iters_max = options_.iter_max;

	while (iter < iters_max)
	{
		iter++;
		LOGGER()->debug("\niter: %03d", iter);

		problem_->evaluate(jtj_, jtf_, options_.bUseOffDiagJtJData, frmIdx, iter);

		//check graident tolerance
		if (jtf_.max_value() <= options_.gradient_tolerance &&
			jtf_.min_value() >= -options_.gradient_tolerance)
		{
			printf("graident tolerance reached!\n");
			return true;
		}

		problem_->backup_currentX();


		this->regularize_jtj_vAddition(jtj_, mu);

		//analyze non-zero pattern if necessay, only meaningful for cholesky solver
		if (iter == 1 || options_.bDynamicSparsity)
			normal_equation_solver_->Analyze(jtj_);

		//solve the problem
		vnl_vector<float> dx;
		__tic__();
		normal_equation_solver_->SolveImpl(jtj_, -jtf_, dx);
		LOGGER()->debug("solve: %f", __toc__());

		//compute the predicated cost_decease;
		// dx'*(jtf-mu*D*dx)
		double cost_decease_predicted = this->compute_predicted_cost_decease_vAddition(jtj_, jtf_, dx, mu);
		LOGGER()->debug("cost_decease_predicted=%f", cost_decease_predicted);

		//update attentatively and compute the actual cost_decease
		problem_->updateX(dx);
		double cost_new = problem_->evaluateCurrentCost(frmIdx, iter);
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
			failed_steps_count++;
			mu *= std::pow(2, failed_steps_count);

			//recover X
			problem_->recover_backupX();
			LOGGER()->debug("failed<%g>!", mu);

		}
		
	}

	return false;
}
