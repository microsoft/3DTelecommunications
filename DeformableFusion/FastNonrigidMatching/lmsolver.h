#ifndef __LMSOLVER_H__
#define __LMSOLVER_H__

#include <opencv2\opencv.hpp>
#include <vnl/vnl_vector.h>
#include "BlockedHessianMatrix.h"

class NormalEquationSolver
{
public:
	virtual bool Analyze(BlockedHessianMatrix const&A) = 0;
	virtual bool SolveImpl(BlockedHessianMatrix const&A, vnl_vector<float> const&b, vnl_vector<float> &x) = 0;
};

class LSQProblem
{
public:
	virtual bool init(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf) = 0;
	virtual bool evaluate(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf, bool bUseOffDiagJtJData, int frmIdx, int iter) = 0;
	virtual double evaluateCurrentCost(int frmIdx, int iter) = 0; // ||F(x)||^2
	virtual bool updateX(vnl_vector<float> const&dx) = 0;//update state variables
	virtual bool backup_currentX() = 0; //return state variables as a vector
	virtual bool recover_backupX() = 0;
	virtual void saveIntermediate(int idx) = 0;
};

struct LMSolverOptions
{
public:
	LMSolverOptions()
		:iter_max(20),
		parameter_tolerance(1.0e-6),
		gradient_tolerance(1.0e-6),
		function_tolerance(1.0e-6),
		mu_init(1.0e-3),
		bDynamicSparsity(false),
		bDebugMode(false),
		bUseOffDiagJtJData(true),
		dump_dir(NULL)
	{}
public:
	int iter_max;
	double parameter_tolerance;
	double gradient_tolerance;
	double function_tolerance;
	double mu_init;// inverse of the trust region
	bool bDynamicSparsity;
	bool bDebugMode;
	bool bUseOffDiagJtJData;
	char const* dump_dir;
};

class LMSolver
{
public:
	LMSolver(){}

public:
	bool Solve(int frmIdx = -1, bool bDummy = false, int iters_max = 5);
	void feed_linear_solver(NormalEquationSolver *linear_solver) {
		this->normal_equation_solver_ = linear_solver;
	}
	void feed_problem(LSQProblem *problem){
		this->problem_ = problem;
	}
	void set_options(LMSolverOptions option){
		this->options_ = option;
	}

private:
	LSQProblem *problem_;
	NormalEquationSolver *normal_equation_solver_;
	vnl_vector<double> D; // the conditioner matrix added to the diagonal matrix

private:
	void regularize_jtj(BlockedHessianMatrix &jtj, double mu){
		for (int blkIdx = 0; blkIdx < jtj.para_blk_num; blkIdx++)
			for (int i = 0; i < jtj.blocks[blkIdx].para_count; i++)
				jtj.blocks[blkIdx].at(i, i) *= (float)(1.0 + mu);
	}

	//assume all blocks are of the same size
	void regularize_jtj_vAddition(BlockedHessianMatrix &jtj, double mu)
	{
		int para_blk_size = jtj.blocks[0].para_count;
		D.set_size(para_blk_size);
		D.fill(0.0);
		for (int i = 0; i < jtj.para_blk_num; i++)
			for (int j = 0; j < para_blk_size; j++)
				D[j] += jtj.blocks[i].at(j, j);

		for (int j = 0; j < para_blk_size; j++)
			D[j] /= jtj.para_blk_num;

		for (int blkIdx = 0; blkIdx < jtj.para_blk_num; blkIdx++)
			for (int j = 0; j < para_blk_size; j++)
				jtj.blocks[blkIdx].at(j, j) += mu*D[j];
	}


	void unregularize_jtj(BlockedHessianMatrix &jtj, double mu){
		for (int blkIdx = 0; blkIdx < jtj.para_blk_num; blkIdx++)
			for (int i = 0; i < jtj.blocks[blkIdx].para_count; i++)
				jtj.blocks[blkIdx].at(i, i) /= (1.0 + mu);
	}

	// dx'*(jtf-mu*D*dx) : where mu*D is the addition to the diagnoal
	//jtj_p: regualized jtj
	double compute_predicted_cost_decease(BlockedHessianMatrix const&jtj_p, vnl_vector<float> const&jtf, vnl_vector<float> const& dx, double mu){
		double ret = 0.0;
		int count = 0;
		for (int blkIdx = 0; blkIdx < jtj_p.para_blk_num; blkIdx++)
		{
			for (int i = 0; i < jtj_p.blocks[blkIdx].para_count; i++)
			{
				ret += dx[count] * (jtf[count] - dx[count] * mu*jtj_p.blocks[blkIdx].at(i, i) / (1.0 + mu));
				count++;
			}
		}
		return ret;
	}

	// dx'*(jtf-mu*D*dx) : where mu*D is the addition to the diagnoal
	//jtj_p: regualized jtj
	double compute_predicted_cost_decease_vAddition(BlockedHessianMatrix const&jtj_p, vnl_vector<float> const&jtf, vnl_vector<float> const& dx, double mu){
		double ret = 0.0;
		int count = 0;
		for (int blkIdx = 0; blkIdx < jtj_p.para_blk_num; blkIdx++)
		{
			for (int i = 0; i < jtj_p.blocks[blkIdx].para_count; i++)
			{
				ret += dx[count] * (jtf[count] - dx[count] * mu*D[i]);
				count++;
			}
		}
		return ret;
	}

private:
	BlockedHessianMatrix jtj_;
	vnl_vector<float> jtf_;
	LMSolverOptions options_;
};


#endif