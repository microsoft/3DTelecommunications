#include "stdafx.h"
#include "suitesparse.h"
#include "cholmod_internal.h"
#include "UtilVnlMatrix.h"

//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<HessianBlock const*> : public vnl_numeric_traits<int>
{
public:
	//: Additive identity
	static constexpr double zero = 0.0;
	//: Multiplicative identity
	static constexpr double one = 1.0;
	//: Maximum value which this type can assume
	static constexpr double maxval = 1.7976931348623157E+308;
	//: Return value of abs()
	typedef double abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef long double double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};

cholmod_sparse* SuiteSparse::
CreateSparseMatrix(BlockedHessianMatrix const&H)
{
	int blk_num = H.blk_num;
	int para_blk_num = H.para_blk_num;

	vnl_matrix<HessianBlock const*> block_pointer_array(para_blk_num, para_blk_num);
	block_pointer_array.fill(NULL);
	for (int i = 0; i < H.blk_num; i++)
	{
		int row = H.blocks[i].row;
		int col = H.blocks[i].col;
		block_pointer_array[row][col] = H.blocks + i;
	}

	//column-compressed matrix
	int nz_num = blk_num * 12 * 12 - para_blk_num * 66;
	cholmod_sparse *A = cholmod_allocate_sparse(para_blk_num*12, para_blk_num*12, nz_num, 
							TRUE, TRUE, -1, CHOLMOD_REAL, &cc_);

	int nz_count = 0;
	for (int c = 0; c < para_blk_num; c++) //c: paarmeter block column
	{
		int blks_num_per_col = 0; //number of parameter blocks in current column
		for (int r = c; r < para_blk_num; r++)
		{
			if (block_pointer_array[r][c] != NULL)
				blks_num_per_col++;
		}

		//block on diagonal
		HessianBlock const* blk = block_pointer_array[c][c];
		assert(blk->col == c && blk->row == c);
		for (int i = 0; i < 12; i++) //col
		{
			//idx of the first non-zero element of the current column
			int idx = nz_count + blks_num_per_col*12*i - ((i*(i-1))/2);

			((Int*)(A->p))[c * 12 + i] = idx;
			
			for (int j = i; j < 12; j++) //row
			{
				((Int*)(A->i))[idx + j - i] = c * 12 + j;
				((double*)(A->x))[idx + j - i] = blk->at(j, i);
			}
		}

		//block off diagonal
		int blk_idx_per_c = 1;
		for (int r = c + 1; r < para_blk_num; r++)
		{
			if (block_pointer_array[r][c] == NULL)
				continue;

			HessianBlock const* blk = block_pointer_array[r][c];

			for (int i = 0; i < 12; i++) //col
			{
				//idx of the first non-zero element of the current column
				int idx = nz_count + blks_num_per_col * 12 * i - ((i*(i - 1)) / 2);

				for (int j = 0; j < 12; j++)//row
				{
					int idx_cur = idx + blk_idx_per_c * 12 + j - i; //idx of the current non-zero element

					((Int*)(A->i))[idx_cur] = r * 12 + j;
					((double*)(A->x))[idx_cur] = blk->at(j, i);
				}
			}
			blk_idx_per_c++;
		}

		nz_count += blk_idx_per_c * 144 - 66;

	}

	((Int*)(A->p))[para_blk_num*12] = nz_count;

	return A;
}

cholmod_factor* SuiteSparse::AnalyzeCholesky(cholmod_sparse* A) {
	// Cholmod can try multiple re-ordering strategies to find a fill
	// reducing ordering. Here we just tell it use AMD with automatic
	// matrix dependence choice of supernodal versus simplicial
	// factorization.
	cc_.nmethods = 1;
	cc_.method[0].ordering = CHOLMOD_COLAMD;
	cc_.supernodal = CHOLMOD_AUTO;

	cholmod_factor* factor = cholmod_analyze(A, &cc_);
	cholmod_print_common(const_cast<char*>("Symbolic Analysis"), &cc_);

	if (cc_.status != CHOLMOD_OK) 
	{
		printf("cholmod_analyze failed. error code: %d", cc_.status);
		return NULL;
	}

	return factor;
}

cholmod_factor* SuiteSparse::AnalyzeCholeskyWithNaturalOrdering(cholmod_sparse * A) 
{
	cc_.nmethods = 1;
	cc_.method[0].ordering = CHOLMOD_NATURAL;
	cc_.postorder = 0;

	cholmod_factor* factor = cholmod_analyze(A, &cc_);
	
	cholmod_print_common(const_cast<char*>("Symbolic Analysis"), &cc_);
	if (cc_.status != CHOLMOD_OK) 
	{
		printf("cholmod_analyze failed. error code: %d", cc_.status);
		return NULL;
	}

	return factor;
}

bool SuiteSparse::Cholesky(cholmod_sparse* A, cholmod_factor* L) 
{
	assert(A != NULL);
	assert(L != NULL);

	// Save the current print level and silence CHOLMOD, otherwise
	// CHOLMOD is prone to dumping stuff to stderr, which can be
	// distracting when the error (matrix is indefinite) is not a fatal
	// failure.
	const int old_print_level = cc_.print;
	cc_.print = 0;

	cc_.quick_return_if_not_posdef = 1;
	int cholmod_status = cholmod_factorize(A, L, &cc_);
	cc_.print = old_print_level;

	// TODO(sameeragarwal): This switch statement is not consistent. It
	// treats all kinds of CHOLMOD failures as warnings. Some of these
	// like out of memory are definitely not warnings. The problem is
	// that the return value Cholesky is two valued, but the state of
	// the linear solver is really three valued. SUCCESS,
	// NON_FATAL_FAILURE (e.g., indefinite matrix) and FATAL_FAILURE
	// (e.g. out of memory).
	switch (cc_.status) {
	case CHOLMOD_NOT_INSTALLED:
		printf("CHOLMOD failure: Method not installed.");
		return false;
	case CHOLMOD_OUT_OF_MEMORY:
		printf("CHOLMOD failure: Out of memory.");
		return false;
	case CHOLMOD_TOO_LARGE:
		printf("CHOLMOD failure: Integer overflow occured.");
		return false;
	case CHOLMOD_INVALID:
		printf("CHOLMOD failure: Invalid input.");
		return false;
	case CHOLMOD_NOT_POSDEF:
		printf("CHOLMOD warning: Matrix not positive definite.");
		return false;
	case CHOLMOD_DSMALL:
		printf("CHOLMOD warning: D for LDL' or diag(L) or "
			"LL' has tiny absolute value.");
		return false;
	case CHOLMOD_OK:
		if (cholmod_status != 0) {
			return true;
		}

		printf("CHOLMOD failure: cholmod_factorize returned false "
			"but cholmod_common::status is CHOLMOD_OK."
			"Please report this to ceres-solver@googlegroups.com.");
		return false;
	default:
		printf("Unknown cholmod return code: %d. "
			"Please report this to ceres-solver@googlegroups.com.",
			cc_.status);
		return false;
	}

	return false;
}

cholmod_dense* SuiteSparse::CreateDenseVector(const double* x, int in_size, int out_size) 
{
	if (in_size > out_size)
		return false;

	cholmod_dense* v = cholmod_zeros(out_size, 1, CHOLMOD_REAL, &cc_);
	if (x != NULL) {
		memcpy(v->x, x, in_size*sizeof(*x));
	}
	return v;
}

bool SuiteSparse::Solve(cholmod_factor* L, vnl_vector<double> const& b, vnl_vector<double> &x)
{
	if (cc_.status != CHOLMOD_OK) {
		printf("cholmod_solve failed. CHOLMOD status is not CHOLMOD_OK");
		return false;
	}

	cholmod_dense *b_ = CreateDenseVector(b.data_block(), b.size(), b.size());
	cholmod_dense *x_ = cholmod_solve(CHOLMOD_A, L, b_, &cc_);

	x.set_size(b.size());
	memcpy(x.data_block(), x_->x, sizeof(double)*b.size());

	cholmod_free_dense(&b_, &cc_);
	cholmod_free_dense(&x_, &cc_);

	return true;
}


cv::Mat* SuiteSparse::
cholmod_sparse_to_dense(cholmod_sparse* A)
{
	int row = A->nrow;
	int col = A->ncol;

	if (!A->packed)
	{
		printf("TODO---A: non packed!\n");
		return NULL;
	}

	cv::Mat* ret = &cv::Mat(row, col, CV_64F, cv::Scalar(0));

	for (int j = 0; j < col; j++)
	{
		int idx_s = ((Int*)(A->p))[j];
		int idx_e = ((Int*)(A->p))[j + 1];

		for (int k = idx_s; k < idx_e; k++)
		{
			int i = ((Int*)(A->i))[k];
			ret->at<double>(i, j) = ((double*)(A->x))[k];
		}
	}

	return ret;
}

bool SuiteSparse::
save_cholmod_sparse_to_ascii(char const* file_name, cholmod_sparse* A)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if (!fp)
	{
		printf("Error<save_cholmod_sparse_to_ascii>: Cannot open the file <%s>.\n", file_name);
		return false;
	}

	fprintf(fp, "Choldmod Compressed Column Sparse Matrix.\n");
	fprintf(fp, "nrow=%d\n", A->nrow);
	fprintf(fp, "ncol=%d\n", A->ncol);
	fprintf(fp, "nzmax=%d\n", A->nzmax);
	fprintf(fp, "stype=%d\n", A->stype); //-1: ignore upper triangle entries
	fprintf(fp, "\n");

	fprintf(fp, "column pointers:\n");
	for (int i = 0; i <= A->ncol; i++)
	{
		fprintf(fp, "%04d\n", ((Int*)(A->p))[i]);
	}
	fprintf(fp, "\n\n");

	fprintf(fp, "rowIdx, val:\n");
	for (int i = 0; i < A->nzmax; i++)
	{
		fprintf(fp, "%d, %f\n", ((Int*)(A->i))[i], ((double*)(A->x))[i]);
	}
	fprintf(fp, "\n\n");

	fclose(fp);
	return true;
}
