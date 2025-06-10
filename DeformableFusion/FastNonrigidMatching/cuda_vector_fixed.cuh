// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __CUDA_VECTOR_FIXED_CUH__
#define __CUDA_VECTOR_FIXED_CUH__
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"

template <class T, unsigned int n>
class cuda_vector_fixed
{
protected:
	T data_[n];

public:
	typedef cuda_vector_fixed<T, n> self;
	typedef unsigned int size_type;
	// Compile-time accessible attribute to get the dimensionality of the vector.
	enum { SIZE = n };

	// Don't out-of-line the constructors, as extra the function call
	// adds a significant overhead. (memcpy is often implemented with a
	// couple of assembly instructions.)

	//: Construct an uninitialized n-vector
	__host__ __device__ cuda_vector_fixed() {}

	//: Copy constructor
	//  The dimensions must match.
	__host__ __device__ cuda_vector_fixed(const cuda_vector_fixed<T, n>& rhs)
	{

#pragma unroll
		for (int i = 0; i < n; i++)
			data_[i] = rhs[i];
	}

	//: Constructs n-vector with all elements initialised to \a v
	__host__ __device__ explicit cuda_vector_fixed(const T& v) { fill(v); }

	//: Construct a fixed-n-vector initialized from \a datablck
	//  The data \e must have enough data. No checks performed.
	__host__ __device__ explicit cuda_vector_fixed(const T* datablck)
	{
		//cuda_copy(datablck, data_, n);
	#pragma unroll
		for (int i = 0; i < n; i++)
			data_[i] = datablck[i];
	}

	//: Convenience constructor for 2-D vectors
	// While this constructor is sometimes useful, consider using
	// vnl_double_2 or vnl_float_2 instead.
	__host__ __device__ cuda_vector_fixed(const T& x0, const T& x1)
	{
		data_[0] = x0; data_[1] = x1;
	}

	//: Convenience constructor for 3-D vectors
	// While this constructor is sometimes useful, consider using
	// vnl_double_3 or vnl_float_3 instead.
	__host__ __device__ cuda_vector_fixed(const T& x0, const T& x1, const T& x2)
	{
		data_[0] = x0; data_[1] = x1; data_[2] = x2;
	}

	//: Convenience constructor for 4-D vectors
	__host__ __device__ cuda_vector_fixed(const T& x0, const T& x1, const T& x2, const T& x3)
	{
		data_[0] = x0; data_[1] = x1; data_[2] = x2; data_[3] = x3;
	}

	//: Copy operator
	__host__ __device__ cuda_vector_fixed<T, n>& operator=(const cuda_vector_fixed<T, n>& rhs) {
		cuda_copy(rhs.data_, data_, n);
		return *this;
	}

	//: Length of the vector.
	// This is always \a n.
	__host__ __device__ unsigned size() const { return n; }

	//: Put value at given position in vector.
	__host__ __device__ void put(unsigned int i, T const& v) { data_[i] = v; }

	//: Get value at element i
	__host__ __device__ T get(unsigned int i) const { return data_[i]; }

	//: Set all values to v
	__host__ __device__ cuda_vector_fixed& fill(T const& v)
	{
		for (size_type i = 0; i < n; ++i)
			data_[i] = v;
		return *this;
	}

	//: Sets elements to ptr[i]
	//  Note: ptr[i] must be valid for i=0..size()-1
	__host__ __device__ cuda_vector_fixed& copy_in(T const * ptr)
	{
		for (size_type i = 0; i < n; ++i)
			data_[i] = ptr[i];
		return *this;
	}

	//: Copy elements to ptr[i]
	//  Note: ptr[i] must be valid for i=0..size()-1
	__host__ __device__ void copy_out(T* ptr) const
	{
		for (size_type i = 0; i < n; ++i)
			ptr[i] = data_[i];
	}

	//: Sets elements to ptr[i]
	//  Note: ptr[i] must be valid for i=0..size()-1
	__host__ __device__ cuda_vector_fixed& set(T const *ptr) { return copy_in(ptr); }

	//: Return reference to the element at specified index.
	// There are assert style boundary checks - #define NDEBUG to turn them off.
	__host__ __device__ T       & operator() (unsigned int i){
		return data_[i];
	}

	//: Return reference to the element at specified index.
	// There are assert style boundary checks - #define NDEBUG to turn them off.
	__host__ __device__ T const & operator() (unsigned int i) const{
		return data_[i];
	}

	//: Return the i-th element
	__host__ __device__ T& operator[] (unsigned int i) { return data_[i]; }

	//: Return the i-th element
	__host__ __device__ const T& operator[] (unsigned int i) const { return data_[i]; }

	//: Access the contiguous block storing the elements in the vector.
	//  O(1).
	//  data_block()[0] is the first element of the vector
	__host__ __device__ T const* data_block() const { return data_; }

	//: Access the contiguous block storing the elements in the vector.
	//  O(1).
	//  data_block()[0] is the first element of the vector
	__host__ __device__ T      * data_block() { return data_; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator+=(T s) { self::add(data_, s, data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator-=(T s) { self::sub(data_, s, data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator*=(T s) { self::mul(data_, s, data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator/=(T s) { self::div(data_, s, data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator+=(const cuda_vector_fixed<T, n>& v) { self::add(data_, v.data_block(), data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n>& operator-=(const cuda_vector_fixed<T, n>& v) { self::sub(data_, v.data_block(), data_); return *this; }

	//:
	__host__ __device__ cuda_vector_fixed<T, n> operator-() const
	{
		cuda_vector_fixed<T, n> result;
		self::sub((T)0, data_, result.data_);
		return result;
	}


	//: Return true iff the size is zero.
	__host__ __device__ bool empty() const { return n == 0; }

	//: Return true if *this == v
	__host__ __device__ bool operator_eq(cuda_vector_fixed<T, n> const& v) const
	{
		for (size_type i = 0; i < n; ++i)
		if ((*this)[i] != v[i])
			return false;
		return true;
	}

	//: Return magnitude (length) of vector
	__host__ __device__ T magnitude() const { return two_norm(); }

	//: Return sqrt of sum of squares of values of elements
	__host__ __device__ T two_norm() const
	{ 
		T ret = 0;
#pragma unroll
		for (int i = 0; i < n; i++)
			ret += data_[i] * data_[i];		
		return sqrtf(ret);
	}

	//: Normalise by dividing through by the magnitude
	__host__ __device__ cuda_vector_fixed<T, n>& normalize() { 
		T mag = magnitude();
		if (mag > M_EPS)
		{
			#pragma unroll
			for (int i = 0; i < n; i++)
				data_[i] /= mag;
		}

		return *this; 
	}

public:
	// Helper routines for arithmetic. n is the size, and is the
	// template parameter.

	__host__ __device__ inline static void add(const T* a, const T* b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a, ++b)
			*r = *a + *b;
	}

	__host__ __device__ inline static void add(const T* a, T b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a)
			*r = *a + b;
	}

	__host__ __device__ inline static void sub(const T* a, const T* b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a, ++b)
			*r = *a - *b;
	}

	__host__ __device__ inline static void sub(const T* a, T b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a)
			*r = *a - b;
	}

	__host__ __device__ inline static void sub(T a, const T* b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++b)
			*r = a - *b;
	}

	__host__ __device__ inline static void mul(const T* a, const T* b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a, ++b)
			*r = *a * *b;
	}

	__host__ __device__ inline static void mul(const T* a, T b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a)
			*r = *a * b;
	}

	__host__ __device__ inline static void div(const T* a, const T* b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a, ++b)
			*r = *a / *b;
	}

	__host__ __device__ inline static void div(const T* a, T b, T* r)
	{
		for (unsigned int i = 0; i < n; ++i, ++r, ++a)
			*r = *a / b;
	}
};


// --- Vector-scalar operators ----------------------------------------

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator+(const cuda_vector_fixed<T, n>& v, T s)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator+(const T& s,
	const cuda_vector_fixed<T, n>& v)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::add(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator-(const cuda_vector_fixed<T, n>& v, T s)
{
	cuda_vector_fixed<T,n> r;
	cuda_vector_fixed<T,n>::sub( v.data_block(), s, r.data_block() );
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator-(const T& s,
	const cuda_vector_fixed<T, n>& v)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::sub(s, v.data_block(), r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator*(const cuda_vector_fixed<T, n>& v, T s)
{
	cuda_vector_fixed<T,n> r;
	cuda_vector_fixed<T,n>::mul( v.data_block(), s, r.data_block() );
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator*(const T& s,
	const cuda_vector_fixed<T, n>& v)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::mul(v.data_block(), s, r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator/(const cuda_vector_fixed<T, n>& v, T s)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::div(v.data_block(), s, r.data_block());
	return r;
}


// --- Vector-vector operators ----------------------------------------
//
// Includes overloads for the common case of mixing a fixed with a
// non-fixed. Because the operators are templated, the fixed will not
// be automatically converted to a non-fixed-ref. These do it for you.

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator+(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::add(a.data_block(), b.data_block(), r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> operator-(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::sub(a.data_block(), b.data_block(), r.data_block());
	return r;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> element_product(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::mul(a.data_block(), b.data_block(), r.data_block());
	return r;
}


//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline cuda_vector_fixed<T, n> element_quotient(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	cuda_vector_fixed<T, n> r;
	cuda_vector_fixed<T, n>::div(a.data_block(), b.data_block(), r.data_block());
	return r;
}


//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned n>
__host__ __device__ inline T dot_product(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	T ret = 0.0;
#pragma unroll
	for (int i = 0; i < n; i++)
		ret += a[i] * b[i];
	return ret;
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline bool operator==(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	return a.operator_eq(b);
}

//:
// \relatesalso cuda_vector_fixed
template<class T, unsigned int n>
__host__ __device__ inline bool operator!=(const cuda_vector_fixed<T, n>& a, const cuda_vector_fixed<T, n>& b)
{
	return !a.operator_eq(b);
}

template<class T>
__host__ __device__ inline cuda_vector_fixed<T, 3> cross_product(cuda_vector_fixed<T, 3> const&a, cuda_vector_fixed<T, 3> const&b)
{
	return cuda_vector_fixed<T, 3>(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

#endif