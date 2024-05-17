//===============================================
//			VecOperation.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __VECOPERATION_H__
#define __VECOPERATION_H__
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <functional> 

#include "time.h"
using namespace std;

template<class T>
class VecOperation
{
/******************* Print Vector *****************************/
public:
	/* direct==0: print vector in a row; direct==1: print vector in a column*/
	static void printVec(T *vec, int len, int direct=0);
	static void SaveVectorAscii(const char* fileanme, T* vec, int len);

/*************  Statistical Operations ****************/
public:
	static T GetMax(T *vec, int len);
	static T GetMax(T *vec, int len, int* idx_max);
	static T GetMin(T *vec, int len, int* idx_min);

	static T GetMean(T* vec, int len);

	static T GetStd(T* vec, int len, int mod=0);

	static T GetMoment(T* vec, int len);

	static T GetMedian(T* vec, int len);

	/* Normalize the vector to make the standard deviation be 1, and mean 0*/
	static void Normalize(T* vec, int len); 

	static void VecSetRandom(T *vec, int len);

	static T InnerProd(T* vec1, T* vec2, int len);

	/* to compute the correlation coefficient between vec1 and vec2
	 * Note that the values in vec1 and vec2 are changed after invocation
	 */
	static T NormalizedCorrelation(T* vec1, T* vec2, int len);

	static T Correlation(T* vec1, T* vec2, int len);
	static T CorrelationImagePatch(T *vec1, T *vec2, int len);

	static T Distance(T const*vec1, T const*vec2, int len);


/************************ Geometry Operations ********************************/
public:
	static inline T DotProd(T const*vec1, T const*vec2, int len);
	static T Norm(T *vec, int len);
	static inline void CrossProdVec3(T* a, T* b, T* c);
	static void Unitalization(T *vec, int len);
	static inline void VecSub(T const*vec1, T const*vec2, T *res, int len);
	static inline void VecAdd(T const*vec1, T const*vec2, T *res, int len);
	static void VecAddWeighted( T const*vec1, T alpha, 
								T const*vec2, T beta,
								T gamma, T *res, int len);
	static inline void VecCopy(T const*src, T *dst, int len);
	static inline void VecScale(T *vec, T scalar, int len);

	// return angle in radius, it should lie in (0, pi)
	static T AngleBtwVecs(T const* vec1, T const* vec2, int len=3);

/********************** Others *************************************************/
public:
	/* the maxima which is larger than thres are returned*/
	static vector<int> FindPeaks(T *vec, int len, int max_suppr_radius, double thres);
	//exact suppression
	static vector<int> FindPeaks2(T *vec, int len, int max_suppr_radius, double thres);
	static void Sort(T *vec, int len, int order=0); //order=0: sort in increasing order, 1: decreasing order

public:
	static T GetStd(T* vec, T mean, int len, int mod=0);

	static void Normalize(T* vec, T mean, T std, int len);
};

#include "VecOperation.inl"


#endif