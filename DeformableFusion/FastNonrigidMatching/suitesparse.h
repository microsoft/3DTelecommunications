#ifndef __SUITESPARSE_H__
#define __SUITESPARSE_H__
#include "cholmod.h"
#include "UtilMatrix.h"
#include "lmsolver.h"

class SuiteSparse
{
public:
	SuiteSparse() { cholmod_start(&cc_); }
	~SuiteSparse(){	cholmod_finish(&cc_);}

public:
	cholmod_sparse* CreateSparseMatrix(BlockedHessianMatrix const&H);
	cholmod_factor* AnalyzeCholesky(cholmod_sparse* A);
	cholmod_factor* AnalyzeCholeskyWithNaturalOrdering(cholmod_sparse* A);
	bool Cholesky(cholmod_sparse* A, cholmod_factor* L);
	cholmod_dense* CreateDenseVector(const double* x,
		int in_size,
		int out_size);
	bool Solve(cholmod_factor* L, vnl_vector<double> const& b, vnl_vector<double> &x);

	void FreeFactor(cholmod_factor** L) { cholmod_free_factor(L, &cc_); }
	void FreeSparse(cholmod_sparse** A) { cholmod_free_sparse(A, &cc_); }
	void FreeDense(cholmod_dense** x) { cholmod_free_dense(x, &cc_); }

	cholmod_sparse* Factor2SpraseMatrix(cholmod_factor* L)
	{
		cholmod_sparse* ret = cholmod_factor_to_sparse(L, &cc_);
		if (cc_.status != CHOLMOD_OK)
		{
			printf("cholmod_factor_to_sparse failed. error code: %d", cc_.status);
			return NULL;
		}
		return ret;
	}

	bool get_permutation(cholmod_factor* L, vnl_vector<int> &P)
	{
		if (L == NULL)
			return false;

		P.set_size(L->n);
		P.set((int const*)L->Perm);

		return true;
	}

public:
	static cv::Mat* cholmod_sparse_to_dense(cholmod_sparse* A);
	static bool save_cholmod_sparse_to_ascii(char const* file_name, cholmod_sparse* A);
private:
	cholmod_common cc_;
};


#endif