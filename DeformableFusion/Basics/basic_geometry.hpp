#pragma once
#ifndef __BASIC_GEOMETRY_HPP__
#define __BASIC_GEOMETRY_HPP__
#include "basic_geometry.h"
#include "VecOperation.h"

template<class T>
inline bool is_clockwise_tri(T lookat[], T v1[], T v2[], T v3[])
{
	T e12[3];
	T e13[3];
	T n[3];
	VecOperation<T>::VecSub(v2, v1, e12, 3);
	VecOperation<T>::VecSub(v3, v1, e13, 3);
	VecOperation<T>::Unitalization(e12, 3);
	VecOperation<T>::Unitalization(e13, 3);
	VecOperation<T>::CrossProdVec3(e12, e13, n);
	T order = VecOperation<T>::DotProd(lookat, n, 3);
	if( order < 0.0 ) //counter clockwise
		return false;
	else
		return true;
}

template<class T, int N>
double angle_btw_two_vec(vnl_vector_fixed<T, N> normal1, vnl_vector_fixed<T, N> normal2)
{
	normal1.normalize();
	normal2.normalize();

	double a = MAX(-1.0, MIN(1.0, dot_product(normal1, normal2)));
	//double ret = vcl_acos(a);
	return std::acos(a);
}


#endif