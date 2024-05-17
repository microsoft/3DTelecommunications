#ifndef __NORMALEQUATIONSOLVERSUITESPARSE_H__
#define __NORMALEQUATIONSOLVERSUITESPARSE_H__
#include "lmsolver.h"
#include "suitesparse.h"
#include "UtilVnlMatrix.hpp"
#include <chrono>
#include "LinearSolverCuSolver.h"

class NormalEquationSolverSuiteSparse : public NormalEquationSolver
{
public:
	NormalEquationSolverSuiteSparse()
		: factor_(NULL) {}

	~NormalEquationSolverSuiteSparse()
	{
		if (factor_ != NULL)
			ss_.FreeFactor(&factor_);
	}
public:
	bool Analyze(BlockedHessianMatrix const&A)
	{
		cholmod_sparse *A_ccs = ss_.CreateSparseMatrix(A);
		if (factor_ != NULL)
			ss_.FreeFactor(&factor_);

		factor_ = ss_.AnalyzeCholesky(A_ccs);
		ss_.FreeSparse(&A_ccs);

		return true;
	}

	bool SolveImpl(BlockedHessianMatrix const&A, vnl_vector<float> const&b, vnl_vector<float> &x)
	{
		cholmod_sparse *A_ccs = ss_.CreateSparseMatrix(A);

		auto tic = std::chrono::high_resolution_clock::now();
		ss_.Cholesky(A_ccs, factor_);

		//float-->double
		vnl_vector<double> b_(b.size());
		for (unsigned int i = 0; i < b.size(); i++)
			b_[i] = b[i];

		vnl_vector<double> x_d;
		ss_.Solve(factor_, b_, x_d);
		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time = toc - tic;
		LOGGER()->info("cholesky & solve: %lfms", time.count() * 1000);

		//double-->float
		x.set_size(x_d.size());
		for (unsigned int i = 0; i < x.size(); i++)
			x[i] = x_d[i];

		ss_.FreeSparse(&A_ccs);

		return true;
	}

private:
	SuiteSparse ss_;
	cholmod_factor *factor_;
};


#endif