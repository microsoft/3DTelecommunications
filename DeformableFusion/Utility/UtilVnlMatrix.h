//===============================================
//			UtilVnlMatrix.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __UTILVNLMATRIX_H__
#define __UTILVNLMATRIX_H__
//!!! it seems there will be compling errors if include "windows.h" before here
#include <vector>
#include <iostream>
#include <ios>
#include <vcl_compiler.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_det.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matrix_ref.h>
#include <vnl/vnl_matrix.hxx>

#include <vnl/vnl_c_vector.hxx>
#include <vnl/vnl_matrix_fixed.h>
#include "vnl/vnl_matrix_fixed_ref.h"
#include <vnl/vnl_matrix_fixed.hxx>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_vector.hxx> 
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_vector_fixed.hxx>

#include <vnl/algo/vnl_svd.h>
#include <vnl/algo/vnl_symmetric_eigensystem.h> 
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/algo/vnl_matrix_inverse.hxx>
#include <vnl/vnl_sparse_matrix.h>
#include <vnl/vnl_sparse_matrix_linear_system.h>
#include <vnl/algo/vnl_lsqr.h>
#include <vnl/vnl_inverse.h>

#include <math.h>
#include "UtilMatrix.h"
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include "CDepthBias.h"

template <class T>
inline T pickAMatElement(vnl_matrix<T> const&mat, int x, int y);
template <class T>
inline T pickAMatElement(vnl_matrix<T> const&mat, double x, double y);

/* if dir == 1, return the row mean vector
 * if dir == 2, return the colum mean vector
 */
vnl_vector<double> get_mean_vector(vnl_matrix<double> mat, int dir);
vnl_vector<double> get_mean_vector(vector< vnl_vector<double> > &vecs );
template <int N>
vnl_vector_fixed<double, N> get_mean_vector(vector< vnl_vector_fixed<double, N> > const&vecs);

/* if dir == 1, treat vec as a row vector
 * if dir == 2, treat vec as a column vector
 * if dir == -1, automatically decide the direction of the vector to finish the operation; if failed, return false
 */
bool sub_vec_from_mat(vnl_matrix<double> &mat, vnl_vector<double> vec, int dir=-1);

/* return a permutation of [0, ..., n-1];
 */
void permutation_vector( vnl_vector<int> &perm_vec, int num);

/* dist between two 3-d vectors
 */
template <class T>
T dist_3d(vnl_vector_fixed<T, 3> const&p, vnl_vector_fixed<T, 3> const&q);
template <class T, int N>
T dist_nd(vnl_vector_fixed<T, N> const&p, vnl_vector_fixed<T, N> const&q);

/* I/O operations
 */
template <class T>
bool loadVNLMatrixBin(const char* filename, vnl_matrix<T> &mat);
template <class T>
bool saveVNLMatrixBin(const char* filename, vnl_matrix<T> &mat);
template <class T>
bool loadVNLMatrixASCAII(const char* filename, vnl_matrix<T> &mat);
template <class T>
bool saveVNLMatrixASCAII(const char* filename, vnl_matrix<T> const&mat);
template <class T>
bool saveVNLMatrix2IMG(const char* filename, vnl_matrix<T> const&mat);
template <class T>
bool loadVNLVectorASCAII(const char* filename, vnl_vector<T> &vec);
template <class T>
bool saveVNLVectorASCAII(const char* filename, vnl_vector<T> const& vec, char dir = 'c');
template <class T>
bool saveDataBlock(const char* filename, T* data_block, int size);
//data_block must be pre-allocated
template <class T>
bool loadDataBlock(const char* filename, T* data_block, int &size);
template<class T>
bool saveVnlVectorSetASCAII(const char* filename, vector< vnl_vector<T> > const&vecs);

template<class T, int N>
bool saveVnlVectorSetASCAII(const char* filename, vector< vnl_vector_fixed<T, N> > const&vecs);
template<class T, int N>
bool loadVnlVectorSetASCAII(const char* filename, vector< vnl_vector_fixed<T, N> > &vecs);

/* assume that the unit of each depth value in PGN is milimeter, while the value in vnl_matrix is in centimeter
 * the bias on the depth will be corrected
 */
bool loadDepthMatFromPNG( const char* name, vnl_matrix<double> &depthMat, bool bUseMostSignificantBits = false, 
						  double bias_a = 1.0, double bias_b = 0.0);
bool loadDepthMatFromPNG( cv::Mat* img, vnl_matrix<double> &depthMat, bool bUseMostSignificantBits = false,
						  double bias_a = 1.0, double bias_b = 0.0);
bool saveDepthMatToPNG(const char* name, vnl_matrix<double> const&depthMat, bool bUseMostSignificantBits = false);
void applyDepthMatBias(vnl_matrix<double> &depthMat, CDepthBias const& depth_bias);

/* delete items with corresponding true value in bDeleteFlag */
template <class T>
bool delete_vector_items( vector<T> &vec, vector<bool> &bDeleteFlag);

/* get the maximum/minimum value in a matrix, and the corresponding element location
 */
template <class T>
void matrix_max(vnl_matrix<T> const& mat, T &max_val, int &row_idx, int &col_idx); 
template <class T>
void matrix_min(vnl_matrix<T> const& mat, T &min_val, int &row_idx, int &col_idx); 


/* find i-th value, if i<0 or i>=vec.size(), return 0;
 */
template<class T>
inline T value_at(vnl_vector<T> const&vec, int i);

/* find value mat(i, j), if (i, j) out of boundary, return 0;
 */
template<class T>
inline T value_at(vnl_matrix<T> const&mat, int i, int j);
template<class T>
inline T value_at(vnl_matrix<T> const&mat, double i, double j);

template<class T>
vnl_vector_fixed<T, 3> cross_product(vnl_vector_fixed<T, 3> const&a, vnl_vector_fixed<T, 3> const&b);

// --- Vector-scalar operators ----------------------------------------

//:
// \relatesalso vnl_vector_fixed
template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const vnl_vector_fixed<T, n>& v, float s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const vnl_vector_fixed<T, n>& v, double s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const vnl_vector_fixed<T, n>& v, int s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed
template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const float& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const double& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator+(const int& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed
template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const vnl_vector_fixed<T, n>& v, float s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const vnl_vector_fixed<T, n>& v, double s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(v.data_block(), s, r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const vnl_vector_fixed<T, n>& v, int s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed
template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const float& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(s, v.data_block(), r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const double& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(s, v.data_block(), r.data_block());
	return r;
}

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator-(const int& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::sub(s, v.data_block(), r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed


template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator*(const vnl_vector_fixed<T, n>& v, int s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::mul(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator*(const int& s,
	const vnl_vector_fixed<T, n>& v)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::mul(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso vnl_vector_fixed

template<class T, unsigned int n>
inline vnl_vector_fixed<T, n> operator/(const vnl_vector_fixed<T, n>& v, int s)
{
	vnl_vector_fixed<T, n> r;
	vnl_vector_fixed<T, n>::div(v.data_block(), s, r.data_block());
	return r;
}



//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<vnl_vector_fixed<double, 3>> : public vnl_numeric_traits<double>
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

//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<vnl_vector_fixed<double, 2>> : public vnl_numeric_traits<double>
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

//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<vnl_vector_fixed<float, 3>> : public vnl_numeric_traits<float>
{
public:
	//: Additive identity
	static constexpr float zero = 0.0F;
	//: Multiplicative identity
	static constexpr float one = 1.0F;
	//: Maximum value which this type can assume
	static constexpr float maxval = 3.40282346638528860e+38F;
	//: Return value of abs()
	typedef float abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef double double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};
#include "UtilVnlMatrix.hpp"
#endif