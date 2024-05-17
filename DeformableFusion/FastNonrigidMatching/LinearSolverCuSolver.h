#ifndef __LINEARSOLVERCUSOLVER_H__
#define __LINEARSOLVERCUSOLVER_H__
#include "lmsolver.h"

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

//compressed row sparse matrix
class CRSMat
{
public:
	CRSMat()
		: rows(0), cols(0), nz_num(0), rowPtr(NULL), vals(NULL), colInd(NULL)
	{}
	~CRSMat(){ free_space();}

public:
	void allocate_space(int rows_, int cols_, int nz_num_)
	{
		rows = rows_;
		cols = cols_;
		nz_num = nz_num_;
		rowPtr = new int[rows + 1];
		colInd = new int[nz_num];
		vals = new float[nz_num];
	}

	void free_space()
	{
		if (rowPtr)
		{
			delete[] rowPtr;
			rowPtr = NULL;
		}
		if (vals)
		{
			delete[] vals;
			vals = NULL;
		}
		if (colInd)
		{
			delete[] colInd;
			colInd = NULL;
		}
		rows = 0;
		cols = 0;
		nz_num = 0;
	}

	bool save_to_ascii(char const* filename);
	cv::Mat *to_dense();

public:
	int rows;
	int cols;
	int nz_num;
	int* rowPtr; //size: rows+1
	float* vals; //size: nz_num
	int* colInd; //size: nz_num
};

class LinearSolverCuSolver : public NormalEquationSolver
{
public:
	//only lower triangular part of hessian is used
	//assume each hessian block is 12X12
	static CRSMat* create_sparse_matrix(BlockedHessianMatrix const& hessian);

public:
	bool Analyze(BlockedHessianMatrix const&A){}
	bool SolveImpl(BlockedHessianMatrix const&A, vnl_vector<float> const&b, vnl_vector<float> &x) {}
};

#endif