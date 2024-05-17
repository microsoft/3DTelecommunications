#ifndef __LMSOLVER_GPU_H__
#define __LMSOLVER_GPU_H__
#include <opencv2\opencv.hpp>
#include "matrix_types_cuda.h"
#include <helper_cuda.h>

#include "lmsolver.h"

class NormalEquationSolverGPU
{
public:
	virtual bool setup_solver(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf) = 0; //assume the data structure of jtj and jtf are constant through out the LM optimization
	virtual bool update_JtJ_JtF(JtJBlockMatrixLTri const&jtj, JtFVector const&jtf)=0;
	virtual bool regularize_JtJ(float mu) = 0;
	virtual bool regularize_JtJ(float const* dev_mu) = 0;
	virtual float* solve() = 0;
	virtual float predicted_cost_decrease(float const* dev_jtf, float const* dev_dx, float mu)=0;
	virtual void predicted_cost_decrease(float* dev_cost_decrease, float const* dev_jtf, float const* dev_dx, float const* dev_mu) = 0;
	virtual BlockMatrixFullCR const* JtJ_full() const = 0;
	virtual float const* dev_b() const = 0;
};

class LSQProblemGPU
{
public:
	virtual bool gpu_return_internal_JtJ_JtF(JtJBlockMatrixLTri &jtj, JtFVector &jtf) = 0;
	virtual bool gpu_evaluate_JtJ_JtF(int frmIdx, int iter, bool bUseOffDiagJtJData) = 0;
	virtual double gpu_evaluate_cost(int frmIdx, int iter) = 0; // ||F(x)||^2
	virtual void gpu_evaluate_cost(float* dev_cost, int frmIdx, int iter) = 0;
	virtual bool gpu_updateX(float const* dev_dx) = 0;//update state variables
	virtual bool gpu_backup_currentX() = 0; 
	virtual bool gpu_recover_backupX() = 0;

	virtual bool initX()=0;
};

class LMSolverGPU
{
public:
	LMSolverGPU()
	{
		checkCudaErrors(cudaMalloc(&dev_cost_prev, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_cost_curr, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_cost_decease_predicted, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_mu, sizeof(float)));
		checkCudaErrors(cudaMalloc(&dev_failed_steps_count, sizeof(int)));

		jtf.n.allocate_once();
	}
	~LMSolverGPU()
	{
		checkCudaErrors(cudaFree(dev_cost_prev));
		checkCudaErrors(cudaFree(dev_cost_curr));
		checkCudaErrors(cudaFree(dev_cost_decease_predicted));
		checkCudaErrors(cudaFree(dev_mu));
		checkCudaErrors(cudaFree(dev_failed_steps_count));
	}

public:
	bool Solve(int frmIdx = -1, bool bContinueLastSolving = false, int iters_max = -1); //iter_max==-1: use the ones in the option
	void solve_impl(int frmIdx, bool bContinueLastSolving, int iters_max);
	void feed_linear_solver(NormalEquationSolverGPU *linear_solver) {
		this->normal_equation_solver_ = linear_solver;
	}
	void feed_problem(LSQProblemGPU *problem){
		this->problem_ = problem;
	}
	void set_options(LMSolverOptions option){
		this->options_ = option;
	}

private:
	LSQProblemGPU *problem_;
	NormalEquationSolverGPU *normal_equation_solver_;

private:
	LMSolverOptions options_;
	float *dev_cost_prev;
	float *dev_cost_curr;
	float *dev_cost_decease_predicted;
	float *dev_mu;
	int *dev_failed_steps_count;

	JtFVector jtf;
	JtJBlockMatrixLTri jtj;
};


#endif