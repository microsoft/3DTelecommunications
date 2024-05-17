#pragma once
#ifndef __CUDA_MATRIX_FIXED_CUH__
#define __CUDA_MATRIX_FIXED_CUH__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"

#include "cuda_vector_fixed.cuh"

template <class T, unsigned int num_rows, unsigned int num_cols>
class cuda_matrix_fixed
{
	T data_[num_rows][num_cols]; // Local storage

public:
	typedef cuda_matrix_fixed<T, num_rows, num_cols> self;
	typedef unsigned int size_type;

	//: Construct an empty num_rows*num_cols matrix
	__host__ __device__ cuda_matrix_fixed() {}

	//: Construct an m*n matrix and fill with value
	__host__ __device__ explicit cuda_matrix_fixed(T value)
	{
		T* p = data_[0];
		unsigned int n = num_rows * num_cols;
		while (n--)
			*p++ = value;
	}

	//: Construct an m*n Matrix and copy data into it row-wise.
	__host__ __device__ explicit cuda_matrix_fixed(const T* datablck)
	{
		cuda_copy(datablck, data_[0], num_rows*num_cols);
	}

	//: Construct an m*n Matrix and copy rhs into it.
	//  Abort if rhs is not the same size.
	__host__ __device__ cuda_matrix_fixed(const cuda_matrix_fixed& rhs)
	{
		cuda_copy(rhs.data_block(), data_[0], num_rows*num_cols);
	}

	//--------------------------------------------------------------------------------

	//: Access the contiguous block storing the elements in the matrix row-wise. O(1).
	// 1d array, row-major order.
	__host__ __device__ T const* data_block() const { return data_[0]; }

	//: Access the contiguous block storing the elements in the matrix row-wise. O(1).
	// 1d array, row-major order.
	__host__ __device__ T      * data_block() { return data_[0]; }

	//: Set all elements to value v
	// Complexity $O(r.c)$
	__host__ __device__ cuda_matrix_fixed& operator= (T const&v) { fill(v); return *this; }

	//: Copy another cuda_matrix_fixed<T,m,n> into this.
	__host__ __device__ cuda_matrix_fixed& operator=(const cuda_matrix_fixed& rhs)
	{
		cuda_copy(rhs.data_block(), data_[0], num_rows*num_cols);
		return *this;
	}

	// Basic 2D-Array functionality-------------------------------------------

	//: Return number of rows
	__host__ __device__ unsigned rows()    const { return num_rows; }

	//: Return number of columns
	// A synonym for cols()
	__host__ __device__ unsigned columns()  const { return num_cols; }

	//: Return number of columns
	// A synonym for columns()
	__host__ __device__ unsigned cols()    const { return num_cols; }

	//: Return number of elements
	// This equals rows() * cols()
	__host__ __device__ unsigned size()    const { return num_rows*num_cols; }

	//: return pointer to given row
	// No boundary checking here.
	__host__ __device__ T       * operator[] (unsigned r) { return data_[r]; }

	//: return pointer to given row
	// No boundary checking here.
	__host__ __device__ T const * operator[] (unsigned r) const { return data_[r]; }

	//: Access an element for reading or writing
	__host__ __device__ T       & operator() (unsigned r, unsigned c)
	{
		return this->data_[r][c];
	}

	//: Access an element for reading
	__host__ __device__ T const & operator() (unsigned r, unsigned c) const
	{
		return this->data_[r][c];
	}

	// ----------------------- Filling and copying -----------------------

	//: Sets all elements of matrix to specified value, and returns "*this".
	//  Complexity $O(r.c)$
	//  Returning "*this" allows "chaining" two or more operations:
	__host__ __device__ cuda_matrix_fixed& fill(T value)
	{
		for (unsigned int i = 0; i < num_rows; ++i)
		for (unsigned int j = 0; j < num_cols; ++j)
			this->data_[i][j] = value;
		return *this;
	}

	//: Fills (laminates) this matrix with the given data, then returns it.
	__host__ __device__ cuda_matrix_fixed& copy_in(T const *p)
	{
		T* dp = this->data_block();
		#pragma unroll
		for (int i = 0; i < num_rows * num_cols; i++)
			dp[i] = p[i];
		return *this;
	}

	//: Fills (laminates) this matrix with the given data, then returns it.
	//  A synonym for copy_in()
	__host__ __device__ cuda_matrix_fixed& set(T const *d) { return copy_in(d); }

	//: Fills the given array with this matrix.
	//  We assume that the argument points to a contiguous rows*cols array, stored rowwise.
	//  No bounds checking on the array.
	__host__ __device__ void copy_out(T * p) const
	{
		T const* dp = this->data_block();
		unsigned int i = num_rows*num_cols;
		while (i--)
			*p++ = *dp++;
	}

	//: Transposes this matrix efficiently, if it is square, and returns it.
	//  Returning "*this" allows "chaining" two or more operations:
	__host__ __device__ cuda_matrix_fixed& inplace_transpose()
	{
#ifndef __CUDA_ARCH__
		assert(num_rows == num_cols); // cannot inplace_transpose non-square fixed size matrix
#endif

		for (unsigned i = 0; i < nrows; ++i)
		for (unsigned j = i + 1; j < ncols; ++j)
		{
			T t = this->data_[i][j];
			this->data_[i][j] = this->data_[j][i];
			this->data_[j][i] = t;
		}
		return *this;
	}

	// ----------------------- Arithmetic --------------------------------
	// note that these functions should not pass scalar as a const&.
	// Look what would happen to A /= A(0,0).

	//: Add \a s to each element of lhs matrix in situ
	__host__ __device__ cuda_matrix_fixed& operator+= (T s)
	{
		self::add(data_block(), s, data_block()); return *this;
	}

	//: Subtract \a s from each element of lhs matrix in situ
	__host__ __device__ cuda_matrix_fixed& operator-= (T s)
	{
		self::sub(data_block(), s, data_block()); return *this;
	}

	//:
	__host__ __device__ cuda_matrix_fixed& operator*= (T s)
	{
		self::mul(data_block(), s, data_block()); return *this;
	}

	//:
	__host__ __device__ cuda_matrix_fixed& operator/= (T s)
	{
		self::div(data_block(), s, data_block()); return *this;
	}

	//:
	__host__ __device__ cuda_matrix_fixed& operator+= (cuda_matrix_fixed const& m)
	{
		self::add(data_block(), m.data_block(), data_block()); return *this;
	}

	//:
	__host__ __device__ cuda_matrix_fixed& operator-= (cuda_matrix_fixed const& m)
	{
		self::sub(data_block(), m.data_block(), data_block()); return *this;
	}

	//: Negate all elements of matrix
	__host__ __device__ cuda_matrix_fixed operator- () const
	{
		cuda_matrix_fixed r;
		self::sub(T(0), data_block(), r.data_block());
		return r;
	}

	//:
	__host__ __device__ cuda_matrix_fixed& operator*= (cuda_matrix_fixed<T, num_cols, num_cols> const& s)
	{
		cuda_matrix_fixed<T, num_rows, num_cols> out;
		for (unsigned i = 0; i < num_rows; ++i)
		for (unsigned j = 0; j < num_cols; ++j)
		{
			T accum = this->data_[i][0] * s(0, j);
			for (unsigned k = 1; k < num_cols; ++k)
				accum += this->data_[i][k] * s(k, j);
			out(i, j) = accum;
		}
		return *this = out;
	}

	//: Return transpose
	__host__ __device__ cuda_matrix_fixed<T, num_cols, num_rows> transpose() const
	{
		cuda_matrix_fixed<T, num_cols, num_rows> result;
		for (unsigned int i = 0; i < cols(); ++i)
		for (unsigned int j = 0; j < rows(); ++j)
			result(i, j) = this->data_[j][i];
		return result;
	}

__forceinline__	__host__ __device__ cuda_vector_fixed<T, num_cols> transpose_and_multiply(cuda_vector_fixed<T, num_rows> const& b) const
	{
		cuda_vector_fixed<T, num_cols> out;
		for (int i = 0; i < num_cols; ++i)
		{
			T accum = (*this)(0, i) * b(0);
			for (int k = 1; k < num_rows; ++k)
				accum += (*this)(k, i) * b(k);
			out(i) = accum;
		}
		return out;
	}

	//--------------------------------------------------------------------------------

	//: Return true if *this == rhs
	__host__ __device__ bool operator_eq(cuda_matrix_fixed const & rhs) const
	{
		return equal(this->data_block(), rhs.data_block());
	}

	//: Equality operator
	__host__ __device__ bool operator==(cuda_matrix_fixed const &that) const { return  this->operator_eq(that); }

	//: Inequality operator
	__host__ __device__ bool operator!=(cuda_matrix_fixed const &that) const { return !this->operator_eq(that); }

	//--------------------------------------------------------------------------------


	// Helper routines for arithmetic. These routines know the size from
	// the template parameters. The vector-vector operations are
	// element-wise.

	__host__ __device__ static void add(const T* a, const T* b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) + *(b++);
	}

	__host__ __device__ static void add(const T* a, T b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) + b;
	}

	__host__ __device__ static void sub(const T* a, const T* b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) - *(b++);
	}

	__host__ __device__ static void sub(const T* a, T b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) - b;
	}

	__host__ __device__ static void sub(T a, const T* b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = a - *(b++);
	}
	__host__ __device__ static void mul(const T* a, const T* b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) * *(b++);
	}

	__host__ __device__ static void mul(const T* a, T b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) * b;
	}

	__host__ __device__ static void div(const T* a, const T* b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) / *(b++);
	}

	__host__ __device__ static void div(const T* a, T b, T* r)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
			*(r++) = *(a++) / b;
	}

	__host__ __device__ static bool equal(const T* a, const T* b)
	{
		unsigned int count = num_rows*num_cols;
		while (count--)
		if (*(a++) != *(b++))  return false;
		return true;
	}
};


// --- Matrix-scalar -------------------------------------------------------------

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator+(const cuda_matrix_fixed<T, m, n>& mat1, const cuda_matrix_fixed<T, m, n>& mat2)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::add(mat1.data_block(), mat2.data_block(), r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator+(const cuda_matrix_fixed<T, m, n>& mat, T s)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::add(mat.data_block(), s, r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator+(const T& s,
const cuda_matrix_fixed<T, m, n>& mat)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::add(mat.data_block(), s, r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator-(const cuda_matrix_fixed<T, m, n>& mat1, const cuda_matrix_fixed<T, m, n>& mat2)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::sub(mat1.data_block(), mat2.data_block(), r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator-(const cuda_matrix_fixed<T, m, n>& mat, T s)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::sub(mat.data_block(), s, r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator-(const T& s,
const cuda_matrix_fixed<T, m, n>& mat)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::sub(s, mat.data_block(), r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator*(const cuda_matrix_fixed<T, m, n>& mat, T s)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::mul(mat.data_block(), s, r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator*(const T& s,
const cuda_matrix_fixed<T, m, n>& mat)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::mul(mat.data_block(), s, r.data_block());
	return r;
}

template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> operator/(const cuda_matrix_fixed<T, m, n>& mat, T s)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::div(mat.data_block(), s, r.data_block());
	return r;
}


template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> element_product(const cuda_matrix_fixed<T, m, n>& mat1,
const cuda_matrix_fixed<T, m, n>& mat2)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::mul(mat1.data_block(), mat2.data_block(), r.data_block());
	return r;
}


template <class T, unsigned m, unsigned n>
__host__ __device__ inline
cuda_matrix_fixed<T, m, n> element_quotient(const cuda_matrix_fixed<T, m, n>& mat1,
const cuda_matrix_fixed<T, m, n>& mat2)
{
	cuda_matrix_fixed<T, m, n> r;
	cuda_matrix_fixed<T, m, n>::div(mat1.data_block(), mat2.data_block(), r.data_block());
	return r;
}


// The following two functions are helper functions keep the
// matrix-matrix and matrix-vector multiplication code in one place,
// so that bug fixes, etc, can be localized.
template <class T, unsigned M, unsigned N>
__host__ __device__ inline
cuda_vector_fixed<T, M>
cuda_matrix_fixed_mat_vec_mult(const cuda_matrix_fixed<T, M, N>& a,
const cuda_vector_fixed<T, N>& b)
{
	cuda_vector_fixed<T, M> out;
#pragma unroll
	for (int i = 0; i < M; ++i)
	{
		T accum = a(i, 0) * b(0);
#pragma unroll
		for (int k = 1; k < N; ++k)
			accum += a(i, k) * b(k);
		out(i) = accum;
	}
	return out;
}

template <class T, unsigned M, unsigned N>
__host__ __device__ __forceinline__
cuda_vector_fixed<T, N>
cuda_matrix_fixed_vec_mat_mult(const cuda_vector_fixed<T, M>& a,
const cuda_matrix_fixed<T, M, N>& b)
{
	cuda_vector_fixed<T, N> out;

	for (int i = 0; i < N; ++i)
	{
		out(i) = a(0) * b(0, i);

		for (int k = 1; k < M; ++k)
			out(i) += a(k) * b(k, i);
	}
	return out;
}

template <class T, unsigned M, unsigned N>
__host__ __device__ __forceinline__
void cuda_matrix_fixed_vec_mat_mult(const cuda_vector_fixed<T, M>& a,
const cuda_matrix_fixed<T, M, N>& b, cuda_vector_fixed<T, N>& out)
{
	#pragma unroll
	for (int i = 0; i < N; ++i)
	{
		out[i] = a(0) * b(0, i);
		#pragma unroll
		for (int k = 1; k < M; ++k)
			out[i] += a(k) * b(k, i);
	}
}

// see comment above
template <class T, unsigned M, unsigned N, unsigned O>
__host__ __device__ inline
cuda_matrix_fixed<T, M, O>
cuda_matrix_fixed_mat_mat_mult(const cuda_matrix_fixed<T, M, N>& a,
const cuda_matrix_fixed<T, N, O>& b)
{
	cuda_matrix_fixed<T, M, O> out;
	for (unsigned i = 0; i < M; ++i)
	for (unsigned j = 0; j < O; ++j)
	{
		T accum = a(i, 0) * b(0, j);
		for (unsigned k = 1; k < N; ++k)
			accum += a(i, k) * b(k, j);
		out(i, j) = accum;
	}
	return out;
}

//: Multiply  conformant cuda_matrix_fixed (M x N) and vector_fixed (N)
template <class T, unsigned M, unsigned N>
__host__ __device__ inline
cuda_vector_fixed<T, M> operator*(const cuda_matrix_fixed<T, M, N>& a, const cuda_vector_fixed<T, N>& b)
{
	return cuda_matrix_fixed_mat_vec_mult(a, b);
}

//: Multiply  conformant vector_fixed (M) and cuda_matrix_fixed (M x N)
// \relatesalso cuda_vector_fixed
// \relatesalso cuda_matrix_fixed
template <class T, unsigned M, unsigned N>
__host__ __device__ inline
cuda_vector_fixed<T, N> operator*(const cuda_vector_fixed<T, M>& a, const cuda_matrix_fixed<T, M, N>& b)
{
	return cuda_matrix_fixed_vec_mat_mult(a, b);
}

//: Multiply two conformant cuda_matrix_fixed (M x N) times (N x O)
// \relatesalso cuda_matrix_fixed
template <class T, unsigned M, unsigned N, unsigned O>
__host__ __device__ inline
cuda_matrix_fixed<T, M, O> operator*(const cuda_matrix_fixed<T, M, N>& a, const cuda_matrix_fixed<T, N, O>& b)
{
	return cuda_matrix_fixed_mat_mat_mult(a, b);
}

//:
// \relatesalso cuda_vector_fixed
template <class T, unsigned m, unsigned n>
__host__ __device__ cuda_matrix_fixed<T, m, n> outer_product(cuda_vector_fixed<T, m> const& a, cuda_vector_fixed<T, n> const& b)
{
	cuda_matrix_fixed<T, m, n> out; // = a.column() * b.row()
	for (unsigned int i = 0; i < m; ++i)
	for (unsigned int j = 0; j < n; ++j)
		out[i][j] = a[i] * b[j];
	return out;
}


template <class T>
__host__ __device__ T cuda_det(T const *row0, T const *row1, T const *row2)
{
	return  
		  row0[0] * row1[1] * row2[2]
		- row0[0] * row2[1] * row1[2]
		- row1[0] * row0[1] * row2[2]
		+ row1[0] * row2[1] * row0[2]
		+ row2[0] * row0[1] * row1[2]
		- row2[0] * row1[1] * row0[2];
}

//: Determinant of small size matrices
// \relatesalso vnl_matrix_fixed
template <class T>
__host__ __device__ T cuda_det(cuda_matrix_fixed<T, 3, 3> const& m) { return cuda_det(m[0], m[1], m[2]); }


template <class T>
cuda_matrix_fixed<T, 3, 3> vnl_inverse(cuda_matrix_fixed<T, 3, 3> const& m)
{
	T det = cuda_det(m);
	if (det == 0) {
		return cuda_matrix_fixed<T, 3, 3>();
	}
	det = T(1) / det;
	T d[9];
	d[0] = (m(1, 1)*m(2, 2) - m(1, 2)*m(2, 1))*det;
	d[1] = (m(2, 1)*m(0, 2) - m(2, 2)*m(0, 1))*det;
	d[2] = (m(0, 1)*m(1, 2) - m(0, 2)*m(1, 1))*det;
	d[3] = (m(1, 2)*m(2, 0) - m(1, 0)*m(2, 2))*det;
	d[4] = (m(0, 0)*m(2, 2) - m(0, 2)*m(2, 0))*det;
	d[5] = (m(1, 0)*m(0, 2) - m(1, 2)*m(0, 0))*det;
	d[6] = (m(1, 0)*m(2, 1) - m(1, 1)*m(2, 0))*det;
	d[7] = (m(0, 1)*m(2, 0) - m(0, 0)*m(2, 1))*det;
	d[8] = (m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0))*det;
	return cuda_matrix_fixed<T, 3, 3>(d);
}

#endif