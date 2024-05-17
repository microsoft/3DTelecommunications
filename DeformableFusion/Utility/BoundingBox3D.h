//===============================================
//			BoundingBox3D.h
//			Mingsong Dou (doums@cs.unc.edu)
//			August, 2010
//===============================================

#ifndef __BOUNDINGBOX3D_H__
#define __BOUNDINGBOX3D_H__
//#include "UtilMatrix.h"
#include <algorithm>

struct BoundingBox3D
{
public:
	BoundingBox3D(double x_s_=0.0, double x_e_=0.0, double y_s_=0.0, double y_e_=0.0, double z_s_=0.0, double z_e_=0.0)
		:x_s(x_s_),
		 x_e(x_e_),
		 y_s(y_s_),
		 y_e(y_e_),
		 z_s(z_s_),
		 z_e(z_e_)
	{;}
	double x_s;
	double x_e;
	double y_s;
	double y_e;
	double z_s;
	double z_e;

public:
	BoundingBox3D& extend(double d)
	{
		double v = fabs(d);
		x_s -= v;
		y_s -= v;
		z_s -= v;
		x_e += v;
		y_e += v;
		z_e += v;

		return (*this);
	}

	BoundingBox3D& shrink(double d)
	{
		double v = fabs(d);
		x_s += v;
		y_s += v;
		z_s += v;
		x_e -= v;
		y_e -= v;
		z_e -= v;

		return (*this);
	}

	template<class T>
	bool isInsideBox(T* p)
	{
		if( p[0]>=x_s && p[0]<=x_e &&
			p[1]>=y_s && p[1]<=y_e &&
			p[2]>=z_s && p[2]<=z_e )
			return true;
		else
			return false;
	}

	void print()
	{
		printf("Bounding box<%f, %f, %f, %f, %f, %f>\n", x_s, x_e, y_s, y_e, z_s, z_e);
	}
};

#ifndef MAX
#define MAX(x, y) (((x)>(y))?(x):(y))
#endif
#ifndef MIN
#define MIN(x, y) (((x)<(y))?(x):(y))
#endif

inline BoundingBox3D bbox_max(BoundingBox3D const&bbox1, BoundingBox3D const&bbox2)
{
	return BoundingBox3D( MIN(bbox1.x_s, bbox2.x_s), MAX(bbox1.x_e, bbox2.x_e), 
						  MIN(bbox1.y_s, bbox2.y_s), MAX(bbox1.y_e, bbox2.y_e), 
						  MIN(bbox1.z_s, bbox2.z_s), MAX(bbox1.z_e, bbox2.z_e) );
};

//overlap of two bounding box
inline BoundingBox3D bbox_min(BoundingBox3D const&bbox1, BoundingBox3D const&bbox2)
{
	return BoundingBox3D( MAX(bbox1.x_s, bbox2.x_s), MIN(bbox1.x_e, bbox2.x_e), 
						  MAX(bbox1.y_s, bbox2.y_s), MIN(bbox1.y_e, bbox2.y_e), 
						  MAX(bbox1.z_s, bbox2.z_s), MIN(bbox1.z_e, bbox2.z_e) );
};


#endif