//===============================================
//			VoxelMatrix.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#pragma once
#ifndef __VOXELMATRIX_H__
#define __VOXELMATRIX_H__
//A 3-Dimension Matrix
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <type_traits>

template<class T>
class VoxelMatrix
{
public:
	VoxelMatrix()
		: nx(0), ny(0), nz(0), data(NULL)
	{
	}
	VoxelMatrix(unsigned int nx_, unsigned int ny_, unsigned nz_, T outer_val_, T null_val_)
		: nx(nx_), ny(ny_), nz(nz_), outer_val(outer_val_), null_val(null_val_)
	{
		if (nx*ny*nz > 0)
		{
			data = new T[nx*ny*nz];
			assert(data != NULL);
		}
		else
			data = NULL;
	}

	VoxelMatrix(VoxelMatrix const &other)
	{
		this->nx = other.nx;
		this->ny = other.ny;
		this->nz = other.nz;
		this->outer_val = other.outer_val;
		this->null_val = other.null_val;
		if (nx*ny*nz > 0)
		{
			data = new T[nx*ny*nz];
			assert(data != NULL);
			memcpy(data, other.data, nx*ny*nz*sizeof(T));
		}
		else
			data = NULL;
	}

	~VoxelMatrix()
	{
		if (data != NULL)
			delete[] data;
	}

public:
	void bilateral_filter(double window_r = 3, double sigma_s = 2.0, double sigma_r = 0.02);

	void print_dim()
	{
		printf("Dim: %d, %d, %d\n", this->ni(), this->nj(), this->nk());
	}

public:
	VoxelMatrix& operator=(VoxelMatrix const &rhs)
	{
		this->nx = rhs.nx;
		this->ny = rhs.ny;
		this->nz = rhs.nz;
		this->outer_val = rhs.outer_val;
		this->null_val = rhs.null_val;
		if (this->data != NULL)
			delete[] this->data;
		this->data = new T[nx*ny*nz];
		assert(data != NULL);
		memcpy(data, rhs.data, nx*ny*nz*sizeof(T));

		return (*this);
	}

	inline T & operator[](unsigned int i)
	{
		assert(i < nx*ny*nz);
		return this->data[i];
	}

	inline T const& operator[](unsigned int i) const
	{
		assert(i < nx*ny*nz);
		return this->data[i];
	}

	inline T & operator()(unsigned int x, unsigned int y, unsigned int z)
	{
		return this->data[z*nx*ny + y*nx + x];
	}

	inline T const& operator()(unsigned int x, unsigned int y, unsigned int z) const
	{
		return this->data[z*nx*ny + y*nx + x];
	}

	inline bool isNull(unsigned int x, unsigned int y, unsigned int z) const
	{
		return (*this)(x, y, z) == this->null_val;
	}

	bool mid_der_at(unsigned int x, unsigned int y, unsigned int z, double* der) const
	{
		T P1 = (*this)(x, y, z);
		if (P1 == this->null_val || isnan(P1))
			return false;

		//compute dx
		T P0 = (x <= 0) ? this->null_val : (*this)(x - 1, y, z);
		T P2 = (x >= nx-1) ? this->null_val : (*this)(x + 1, y, z);
		if (P0 != this->null_val && !isnan(P0) && P2 != this->null_val && !isnan(P2))
			der[0] = (P2 - P0) / 2.0;
		else if (P0 != this->null_val && !isnan(P0) && (P2 == this->null_val || isnan(P2)))
			der[0] = P1 - P0;
		else if ((P0 == this->null_val || isnan(P0)) && P2 != this->null_val && !isnan(P2))
			der[0] = P2 - P1;
		else
			return false;

		//compute dy
		P0 = (y <= 0) ? this->null_val : (*this)(x, y - 1, z);
		P2 = (y >= ny - 1) ? this->null_val : (*this)(x, y + 1, z);
		if (P0 != this->null_val && !isnan(P0) && P2 != this->null_val && !isnan(P2))
			der[1] = (P2 - P0) / 2.0;
		else if (P0 != this->null_val && !isnan(P0) && (P2 == this->null_val || isnan(P2)))
			der[1] = P1 - P0;
		else if ((P0 == this->null_val || isnan(P0)) && P2 != this->null_val && !isnan(P2))
			der[1] = P2 - P1;
		else
			return false;

		//compute dz
		P0 = (z <= 0) ? this->null_val : (*this)(x, y, z - 1);
		P2 = (z >= nz - 1) ? this->null_val : (*this)(x, y, z + 1);
		if (P0 != this->null_val && !isnan(P0) && P2 != this->null_val && !isnan(P2))
			der[2] = (P2 - P0) / 2.0;
		else if (P0 != this->null_val && !isnan(P0) && (P2 == this->null_val || isnan(P2)))
			der[2] = P1 - P0;
		else if ((P0 == this->null_val || isnan(P0)) && P2 != this->null_val && !isnan(P2))
			der[2] = P2 - P1;
		else
			return false;

		return true;
	}

	bool isNull(double x, double y, double z) const
	{
		unsigned int x0 = floor(x);
		unsigned int y0 = floor(y);
		unsigned int z0 = floor(z);
		unsigned int x1 = ceil(x);
		unsigned int y1 = ceil(y);
		unsigned int z1 = ceil(z);

		if (isNull(x0, y0, z0))   return true;
		if (isNull(x1, y0, z0))   return true;
		if (isNull(x0, y1, z0))   return true;
		if (isNull(x1, y1, z0))   return true;
		if (isNull(x0, y0, z1))   return true;
		if (isNull(x1, y0, z1))   return true;
		if (isNull(x0, y1, z1))   return true;
		if (isNull(x1, y1, z1))   return true;

		return false;
	}

	inline T val_at(int x, int y, int z) const
	{
		if (x < nx && y < ny && z < nz &&
			x >= 0 && y >= 0 && z >= 0)
			return this->data[z*nx*ny + y*nx + x];
		else
			return this->null_val;
	}

	// tri-linear interpolation
	//return SDF_NULL_VALUE for NULL voxel
	T val_at(double x, double y, double z) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at(x0, y0, z0);
		T P001 = val_at(x1, y0, z0);
		T P010 = val_at(x0, y1, z0);
		T P011 = val_at(x1, y1, z0);
		T P100 = val_at(x0, y0, z1);
		T P101 = val_at(x1, y0, z1);
		T P110 = val_at(x0, y1, z1);
		T P111 = val_at(x1, y1, z1);

		if (P000 == this->null_val ||
			P001 == this->null_val ||
			P010 == this->null_val ||
			P011 == this->null_val ||
			P100 == this->null_val ||
			P101 == this->null_val ||
			P110 == this->null_val ||
			P111 == this->null_val)
			return this->null_val;
		else
			return (T)(((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
					   ((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c);

	}

	void der_at(double x, double y, double z, T* der) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at(x0, y0, z0);
		T P001 = val_at(x1, y0, z0);
		T P010 = val_at(x0, y1, z0);
		T P011 = val_at(x1, y1, z0);
		T P100 = val_at(x0, y0, z1);
		T P101 = val_at(x1, y0, z1);
		T P110 = val_at(x0, y1, z1);
		T P111 = val_at(x1, y1, z1);

		if (P000 == this->null_val ||
			P001 == this->null_val ||
			P010 == this->null_val ||
			P011 == this->null_val ||
			P100 == this->null_val ||
			P101 == this->null_val ||
			P110 == this->null_val ||
			P111 == this->null_val)
		{
			der[0] = 0.0;
			der[1] = 0.0;
			der[2] = 0.0;
		}
		else
		{
			der[0] = (P001 - P000)*(1.0 - b)*(1.0 - c) + (P011 - P010)*b*(1.0 - c) + (P101 - P100)*(1.0 - b)*c + (P111 - P110)*b*c;
			der[1] = (P010 - P000)*(1.0 - a)*(1.0 - c) + (P011 - P001)*a*(1.0 - c) + (P110 - P100)*(1.0 - a)*c + (P111 - P101)*a*c;
			der[2] = (P100 - P000)*(1.0 - a)*(1.0 - b) + (P101 - P001)*a*(1.0 - b) + (P110 - P010)*(1.0 - a)*b + (P111 - P011)*a*b;
		}
	}

	void der2nd_at(double x, double y, double z, T* der2nd) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at(x0, y0, z0);
		T P001 = val_at(x1, y0, z0);
		T P010 = val_at(x0, y1, z0);
		T P011 = val_at(x1, y1, z0);
		T P100 = val_at(x0, y0, z1);
		T P101 = val_at(x1, y0, z1);
		T P110 = val_at(x0, y1, z1);
		T P111 = val_at(x1, y1, z1);

		if (P000 == this->null_val ||
			P001 == this->null_val ||
			P010 == this->null_val ||
			P011 == this->null_val ||
			P100 == this->null_val ||
			P101 == this->null_val ||
			P110 == this->null_val ||
			P111 == this->null_val)
		{
			memset(der2nd, 0, sizeof(T) * 9);
		}
		else
		{
			der2nd[0] = 0.0;
			der2nd[1] = -(P001 - P000)*(1.0 - c) + (P011 - P010)*(1.0 - c) - (P101 - P100)*c + (P111 - P110)*c;
			der2nd[2] = -(P001 - P000)*(1.0 - b) - (P011 - P010)*b + (P101 - P100)*(1.0 - b) + (P111 - P110)*b;

			der2nd[3] = -(P010 - P000)*(1.0 - c) + (P011 - P001)*(1.0 - c) - (P110 - P100)*c + (P111 - P101)*c;
			der2nd[4] = 0.0;
			der2nd[5] = -(P010 - P000)*(1.0 - a) - (P011 - P001)*a + (P110 - P100)*(1.0 - a) + (P111 - P101)*a;

			der2nd[6] = -(P100 - P000)*(1.0 - b) + (P101 - P001)*(1.0 - b) - (P110 - P010)*b + (P111 - P011)*b;
			der2nd[7] = -(P100 - P000)*(1.0 - a) - (P101 - P001)*a + (P110 - P010)*(1.0 - a) + (P111 - P011)*a;
			der2nd[8] = 0.0;
		}
	}

	//return 1.0 for NULL cell
	T val_at_awf(int x, int y, int z) const
	{
		if (x < nx && y < ny && z < nz &&
			x >= 0 && y >= 0 && z >= 0)
		{
			T val = this->data[z*nx*ny + y*nx + x];
			return (val == this->null_val) ? 1.0 : val;
		}
		else
			return 1.0;
	}

	//return 1.0 for NULL cell
	T val_at_awf(double x, double y, double z) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at_awf(x0, y0, z0);
		T P001 = val_at_awf(x1, y0, z0);
		T P010 = val_at_awf(x0, y1, z0);
		T P011 = val_at_awf(x1, y1, z0);
		T P100 = val_at_awf(x0, y0, z1);
		T P101 = val_at_awf(x1, y0, z1);
		T P110 = val_at_awf(x0, y1, z1);
		T P111 = val_at_awf(x1, y1, z1);

		return (T)(((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
				   ((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c);
	}

	void der_at_awf(double x, double y, double z, T* der) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at_awf(x0, y0, z0);
		T P001 = val_at_awf(x1, y0, z0);
		T P010 = val_at_awf(x0, y1, z0);
		T P011 = val_at_awf(x1, y1, z0);
		T P100 = val_at_awf(x0, y0, z1);
		T P101 = val_at_awf(x1, y0, z1);
		T P110 = val_at_awf(x0, y1, z1);
		T P111 = val_at_awf(x1, y1, z1);

		der[0] = (P001 - P000)*(1.0 - b)*(1.0 - c) + (P011 - P010)*b*(1.0 - c) + (P101 - P100)*(1.0 - b)*c + (P111 - P110)*b*c;
		der[1] = (P010 - P000)*(1.0 - a)*(1.0 - c) + (P011 - P001)*a*(1.0 - c) + (P110 - P100)*(1.0 - a)*c + (P111 - P101)*a*c;
		der[2] = (P100 - P000)*(1.0 - a)*(1.0 - b) + (P101 - P001)*a*(1.0 - b) + (P110 - P010)*(1.0 - a)*b + (P111 - P011)*a*b;
	}

	//second order derivative
	void der2nd_at_awf(double x, double y, double z, T* der2nd) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = val_at_awf(x0, y0, z0);
		T P001 = val_at_awf(x1, y0, z0);
		T P010 = val_at_awf(x0, y1, z0);
		T P011 = val_at_awf(x1, y1, z0);
		T P100 = val_at_awf(x0, y0, z1);
		T P101 = val_at_awf(x1, y0, z1);
		T P110 = val_at_awf(x0, y1, z1);
		T P111 = val_at_awf(x1, y1, z1);

		der2nd[0] = 0.0;
		der2nd[1] = -(P001 - P000)*(1.0 - c) + (P011 - P010)*(1.0 - c) - (P101 - P100)*c + (P111 - P110)*c;
		der2nd[2] = -(P001 - P000)*(1.0 - b) - (P011 - P010)*b + (P101 - P100)*(1.0 - b) + (P111 - P110)*b;

		der2nd[3] = -(P010 - P000)*(1.0 - c) + (P011 - P001)*(1.0 - c) - (P110 - P100)*c + (P111 - P101)*c;
		der2nd[4] = 0.0;
		der2nd[5] = -(P010 - P000)*(1.0 - a) - (P011 - P001)*a + (P110 - P100)*(1.0 - a) + (P111 - P101)*a;

		der2nd[6] = -(P100 - P000)*(1.0 - b) + (P101 - P001)*(1.0 - b) - (P110 - P010)*b + (P111 - P011)*b;
		der2nd[7] = -(P100 - P000)*(1.0 - a) - (P101 - P001)*a + (P110 - P010)*(1.0 - a) + (P111 - P011)*a;
		der2nd[8] = 0.0;
	}

	T abs_val_at_awf(int x, int y, int z) const
	{
		if (x < nx && y < ny && z < nz &&
			x >= 0 && y >= 0 && z >= 0)
		{
			T val = this->data[z*nx*ny + y*nx + x];
			return (val == this->null_val) ? 1.0 : std::abs(val);
		}
		else
			return 1.0;
	}

	T abs_val_at_awf(double x, double y, double z) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0+1;
		int y1 = y0+1;
		int z1 = z0+1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = abs_val_at_awf(x0, y0, z0);
		T P001 = abs_val_at_awf(x1, y0, z0);
		T P010 = abs_val_at_awf(x0, y1, z0);
		T P011 = abs_val_at_awf(x1, y1, z0);
		T P100 = abs_val_at_awf(x0, y0, z1);
		T P101 = abs_val_at_awf(x1, y0, z1);
		T P110 = abs_val_at_awf(x0, y1, z1);
		T P111 = abs_val_at_awf(x1, y1, z1);

		return (T)(((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
			((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c);
	}


	//der of the abs val field
	void abs_der_at_awf(double x, double y, double z, T* der) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000 = abs_val_at_awf(x0, y0, z0);
		T P001 = abs_val_at_awf(x1, y0, z0);
		T P010 = abs_val_at_awf(x0, y1, z0);
		T P011 = abs_val_at_awf(x1, y1, z0);
		T P100 = abs_val_at_awf(x0, y0, z1);
		T P101 = abs_val_at_awf(x1, y0, z1);
		T P110 = abs_val_at_awf(x0, y1, z1);
		T P111 = abs_val_at_awf(x1, y1, z1);

		der[0] = (P001 - P000)*(1.0 - b)*(1.0 - c) + (P011 - P010)*b*(1.0 - c) + (P101 - P100)*(1.0 - b)*c + (P111 - P110)*b*c;
		der[1] = (P010 - P000)*(1.0 - a)*(1.0 - c) + (P011 - P001)*a*(1.0 - c) + (P110 - P100)*(1.0 - a)*c + (P111 - P101)*a*c;
		der[2] = (P100 - P000)*(1.0 - a)*(1.0 - b) + (P101 - P001)*a*(1.0 - b) + (P110 - P010)*(1.0 - a)*b + (P111 - P011)*a*b;
	}

	inline bool sobel_x(int x, int y, int z, T& Sx) const
	{
		T P0 = this->val_at(x + 1, y, z);
		T P1 = this->val_at(x + 1, y + 1, z);
		T P2 = this->val_at(x + 1, y - 1, z);
		T P3 = this->val_at(x + 1, y, z + 1);
		T P4 = this->val_at(x + 1, y, z - 1);
		T P5 = this->val_at(x + 1, y + 1, z + 1);
		T P6 = this->val_at(x + 1, y + 1, z - 1);
		T P7 = this->val_at(x + 1, y - 1, z + 1);
		T P8 = this->val_at(x + 1, y - 1, z - 1);

		T M0 = this->val_at(x - 1, y, z);
		T M1 = this->val_at(x - 1, y + 1, z);
		T M2 = this->val_at(x - 1, y - 1, z);
		T M3 = this->val_at(x - 1, y, z + 1);
		T M4 = this->val_at(x - 1, y, z - 1);
		T M5 = this->val_at(x - 1, y + 1, z + 1);
		T M6 = this->val_at(x - 1, y + 1, z - 1);
		T M7 = this->val_at(x - 1, y - 1, z + 1);
		T M8 = this->val_at(x - 1, y - 1, z - 1);

		if (P0 == this->null_val || P1 == this->null_val ||
			P2 == this->null_val || P3 == this->null_val ||
			P4 == this->null_val || P5 == this->null_val ||
			P6 == this->null_val || P7 == this->null_val ||
			P8 == this->null_val || M0 == this->null_val ||
			M1 == this->null_val || M2 == this->null_val ||
			M3 == this->null_val || M4 == this->null_val ||
			M5 == this->null_val || M6 == this->null_val ||
			M7 == this->null_val || M8 == this->null_val)
			return false;

		Sx = 4 * P0 + 2 * (P1 + P2 + P3 + P4) + (P5 + P6 + P7 + P8) -
			 4 * M0 - 2 * (M1 + M2 + M3 + M4) - (M5 + M6 + M7 + M8);
		Sx /= 32.0;

		return true;
	}

	inline bool sobel_y(int x, int y, int z, T& Sy) const
	{
		T P0 = this->val_at(x,     y + 1, z);
		T P1 = this->val_at(x + 1, y + 1, z);
		T P2 = this->val_at(x - 1, y + 1, z);
		T P3 = this->val_at(x,     y + 1, z + 1);
		T P4 = this->val_at(x,     y + 1, z - 1);
		T P5 = this->val_at(x + 1, y + 1, z + 1);
		T P6 = this->val_at(x + 1, y + 1, z - 1);
		T P7 = this->val_at(x - 1, y + 1, z + 1);
		T P8 = this->val_at(x - 1, y + 1, z - 1);

		T M0 = this->val_at(x,     y - 1, z);
		T M1 = this->val_at(x + 1, y - 1, z);
		T M2 = this->val_at(x - 1, y - 1, z);
		T M3 = this->val_at(x,     y - 1, z + 1);
		T M4 = this->val_at(x,     y - 1, z - 1);
		T M5 = this->val_at(x + 1, y - 1, z + 1);
		T M6 = this->val_at(x + 1, y - 1, z - 1);
		T M7 = this->val_at(x - 1, y - 1, z + 1);
		T M8 = this->val_at(x - 1, y - 1, z - 1);

		if (P0 == this->null_val || P1 == this->null_val ||
			P2 == this->null_val || P3 == this->null_val ||
			P4 == this->null_val || P5 == this->null_val ||
			P6 == this->null_val || P7 == this->null_val ||
			P8 == this->null_val || M0 == this->null_val ||
			M1 == this->null_val || M2 == this->null_val ||
			M3 == this->null_val || M4 == this->null_val ||
			M5 == this->null_val || M6 == this->null_val ||
			M7 == this->null_val || M8 == this->null_val)
			return false;

		Sy = 4 * P0 + 2 * (P1 + P2 + P3 + P4) + (P5 + P6 + P7 + P8) -
			4 * M0 - 2 * (M1 + M2 + M3 + M4) - (M5 + M6 + M7 + M8);
		Sy /= 32;

		return true;
	}

	inline bool sobel_z(int x, int y, int z, T& Sz) const
	{
		T P0 = this->val_at(x,     y,     z + 1);
		T P1 = this->val_at(x,     y + 1, z + 1);
		T P2 = this->val_at(x,     y - 1, z + 1);
		T P3 = this->val_at(x + 1, y,     z + 1);
		T P4 = this->val_at(x - 1, y,     z + 1);
		T P5 = this->val_at(x + 1, y + 1, z + 1);
		T P6 = this->val_at(x - 1, y + 1, z + 1);
		T P7 = this->val_at(x + 1, y - 1, z + 1);
		T P8 = this->val_at(x - 1, y - 1, z + 1);

		T M0 = this->val_at(x,     y,     z - 1);
		T M1 = this->val_at(x,     y + 1, z - 1);
		T M2 = this->val_at(x,     y - 1, z - 1);
		T M3 = this->val_at(x + 1, y,     z - 1);
		T M4 = this->val_at(x - 1, y,     z - 1);
		T M5 = this->val_at(x + 1, y + 1, z - 1);
		T M6 = this->val_at(x - 1, y + 1, z - 1);
		T M7 = this->val_at(x + 1, y - 1, z - 1);
		T M8 = this->val_at(x - 1, y - 1, z - 1);

		if (P0 == this->null_val || P1 == this->null_val ||
			P2 == this->null_val || P3 == this->null_val ||
			P4 == this->null_val || P5 == this->null_val ||
			P6 == this->null_val || P7 == this->null_val ||
			P8 == this->null_val || M0 == this->null_val ||
			M1 == this->null_val || M2 == this->null_val ||
			M3 == this->null_val || M4 == this->null_val ||
			M5 == this->null_val || M6 == this->null_val ||
			M7 == this->null_val || M8 == this->null_val)
			return false;

		Sz = 4 * P0 + 2 * (P1 + P2 + P3 + P4) + (P5 + P6 + P7 + P8) -
			4 * M0 - 2 * (M1 + M2 + M3 + M4) - (M5 + M6 + M7 + M8);
		Sz /= 32;

		return true;
	}

	inline bool sobel_x(double x, double y, double z, T& Sx) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000;
		if (!sobel_x(x0, y0, z0, P000))
			return false;
		T P001;
		if (!sobel_x(x1, y0, z0, P001))
			return false;
		T P010;
		if (!sobel_x(x0, y1, z0, P010))
			return false;
		T P011;
		if (!sobel_x(x1, y1, z0, P011))
			return false;
		T P100;
		if (!sobel_x(x0, y0, z1, P100))
			return false;
		T P101;
		if (!sobel_x(x1, y0, z1, P101))
			return false;
		T P110;
		if (!sobel_x(x0, y1, z1, P110))
			return false;
		T P111;
		if (!sobel_x(x1, y1, z1, P111))
			return false;

		Sx = (T)(((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
			((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c);

		return true;
	}

	inline bool sobel_y(double x, double y, double z, T& Sy) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000;
		if (!sobel_y(x0, y0, z0, P000))
			return false;
		T P001;
		if (!sobel_y(x1, y0, z0, P001))
			return false;
		T P010;
		if (!sobel_y(x0, y1, z0, P010))
			return false;
		T P011;
		if (!sobel_y(x1, y1, z0, P011))
			return false;
		T P100;
		if (!sobel_y(x0, y0, z1, P100))
			return false;
		T P101;
		if (!sobel_y(x1, y0, z1, P101))
			return false;
		T P110;
		if (!sobel_y(x0, y1, z1, P110))
			return false;
		T P111;
		if (!sobel_y(x1, y1, z1, P111))
			return false;

		Sy = (T)(((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
			     ((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c);

		return true;
	}

	inline bool sobel_z(double x, double y, double z, T& Sz) const
	{
		int x0 = floor(x);
		int y0 = floor(y);
		int z0 = floor(z);
		int x1 = x0 + 1;
		int y1 = y0 + 1;
		int z1 = z0 + 1;
		double a = x - x0;
		double b = y - y0;
		double c = z - z0;
		T P000;
		if (!sobel_z(x0, y0, z0, P000))
			return false;
		T P001;
		if (!sobel_z(x1, y0, z0, P001))
			return false;
		T P010;
		if (!sobel_z(x0, y1, z0, P010))
			return false;
		T P011;
		if (!sobel_z(x1, y1, z0, P011))
			return false;
		T P100;
		if (!sobel_z(x0, y0, z1, P100))
			return false;
		T P101;
		if( !sobel_z(x1, y0, z1, P101) )
			return false;
		T P110;
		if (!sobel_z(x0, y1, z1, P110))
			return false;
		T P111;
		if (!sobel_z(x1, y1, z1, P111))
			return false;

		Sz = (T)( ((P000*(1.0 - a) + P001*a)*(1.0 - b) + (P010*(1.0 - a) + P011*a)*b)*(1.0 - c) +
				  ((P100*(1.0 - a) + P101*a)*(1.0 - b) + (P110*(1.0 - a) + P111*a)*b)*c );

		return true;
	}

	template<class T2>
	inline bool dif_x(double x, double y, double z, T2& Gx) const
	{
		T val1 = this->val_at(x+0.5, y, z);
		T val2 = this->val_at(x-0.5, y, z);
		if( val1 == this->null_val ||
			val2 == this->null_val )
			return false;
			
		Gx = val1-val2;
		return true;
	}

	template<class T2>
	inline bool dif_y(double x, double y, double z, T2& Gy) const
	{
		T val1 = this->val_at(x, y+0.5, z);
		T val2 = this->val_at(x, y-0.5, z);
		if( val1 == this->null_val ||
			val2 == this->null_val )
			return false;
	
		Gy = val1-val2;
		return true;
	}

	template<class T2>
	inline bool dif_z(double x, double y, double z, T2& Gz) const
	{
		T val1 = this->val_at(x, y, z+0.5);
		T val2 = this->val_at(x, y, z-0.5);
		if (val1 == this->null_val ||
			val2 == this->null_val)
			return false;

		Gz = val1 - val2;
		return true;
	}

	void set_size(unsigned int nx_=0, unsigned int ny_=0, unsigned nz_ = 0)
	{
		if( data != NULL )
		{
			delete [] data;
			nx = 0; ny = 0; nz = 0;
		}

		if( nx_*ny_*nz_ > 0)
		{
			nx = nx_; ny = ny_; nz = nz_;
			data = new T[nx*ny*nz];
			assert(data != NULL);
		}
	}

	void fill(T val)
	{
		for(int i=0; i<nx*ny*nz; i++)
			data[i] = val;
	}

	void set_outer_val(T val)
	{
		this->outer_val = val;
	}

	void set_null_val(T val)
	{
		this->null_val = val;
	}

	T null_value() const
	{
		return this->null_val;
	}

	T* data_block() {return this->data;}
	T const* data_block() const {return this->data;}
	int ni() const {return this->nx;}
	int nj() const {return this->ny;}
	int nk() const {return this->nz;}

//I/O
public:
	bool save_to_file_ascii(const char* filename);
	bool load_from_file_ascii(const char* filename);
	bool save_to_file_bin(const char* filename);
	bool load_from_file_bin(const char* filename);

	bool save_to_rawbin(const char* filename); //could be loaded with paraview

private:
	T* data;
	T outer_val;// return this value when out of boundary
	T null_val; // the voxel with null_val is a null voxel
private:
	int nx;
	int ny;
	int nz;

};
template<class T>
void VoxelMatrix<T>::bilateral_filter(double window_r, double sigma_s, double sigma_r)
{

	VoxelMatrix<T> vxl_mat_bak = *this;

	for (int k = 0; k < nz; k++)
	{
#pragma omp parallel for schedule(dynamic)
		for (int j = 0; j < ny; j++)
		{
			for (int i = 0; i < nx; i++)
			{
				T val_cur = vxl_mat_bak.val_at(i, j, k);
				if (val_cur == this->null_val)
					continue;

				double weights = 0.0;
				double vals = 0.0;
				for (int kk = k - window_r; kk <= k + window_r; kk++)
				{
					for (int jj = j - window_r; jj <= j + window_r; jj++)
					{
						for (int ii = i - window_r; ii <= i + window_r; ii++)
						{
							T val = vxl_mat_bak.val_at(ii, jj, kk);
							if (val == this->null_val)
								continue;

							double weight = std::exp(-((kk-k)*(kk-k) + (jj-j)*(jj-j) + (ii-i)*(ii-i)) / (2 * sigma_s*sigma_s) - 
												  (val-val_cur)*(val-val_cur) / (2 * sigma_r*sigma_r));
							vals += val * weight;
							weights += weight;
						}
					}
				}

				if (weights > 0.0)
					(*this)(i, j, k) = vals / weights;
			}
		}
	}
}


template<class T>
bool VoxelMatrix<T>::save_to_file_ascii(const char* filename)
{
	std::ofstream fp(filename);
	if (fp.fail())
	{
		printf("Error<VoxelMatrix<T>::save_to_file_asci>: Cannot open the file <%s>.\n", filename);
		return false;
	}
	

	int count = 0;
	for (int k = 0; k < nz; k++)
	{
		for (int j = 0; j < ny; j++)
		{
			for (int i = 0; i < nx; i++)
			{
				if (data[count] == -2.0f)
				{
					fp << i << " " << j << " " << k << " " << endl;
				}
				count++;
			}
		}
	}

	fp.close();
	return true;
}

template<class T>
bool VoxelMatrix<T>::load_from_file_ascii(const char* filename)
{
	std::ifstream fp(filename);
	if (fp.fail())
	{
		printf("Error<VoxelMatrix<T>::load_from_file_ascii(>: cannot open the file %s!\n", filename);
		return false;
	}

	fp >> nx >> ny >> nz;
	if (nx*ny*nz > 0)
	{
		data = new T[nx*ny*nz];
		assert(data != NULL);
	}
	else
	{
		data = NULL;
		return false;
	}

	int count = 0;
	for (int k = 0; k < nz; k++)
	{
		for (int j = 0; j < ny; j++)
		{
			for (int i = 0; i < nx; i++)
				fp >> data[count++];
		}
	}

	fp.close();
	return true;
}

template<class T>
bool VoxelMatrix<T>::save_to_file_bin(const char* filename)
{
	if( this->data == NULL || this->nx*this->ny*this->nz <= 0 )
		return false;

	FILE *fp = NULL;
	fopen_s(&fp, filename, "wb");
	if(!fp)
	{
		printf("Cannot open the file <%s> for saving VoxelMatrix.\n", filename);
		return false;
	}

	int count = fwrite(&(this->nx), sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when writing file %s!\n", filename);
		fclose(fp);
		return false;
	}
	count = fwrite(&(this->ny), sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when writing file %s!\n", filename);
		fclose(fp);
		return false;
	}
	count = fwrite(&(this->nz), sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when writing file %s!\n", filename);
		fclose(fp);
		return false;
	}

	count = fwrite(this->data, sizeof(T), nx*ny*nz, fp);
	if( count != nx*ny*nz )
	{
		printf("Error when writing file %s, %d elements written while %d needed to be writen \n", filename, count, nx*ny*nz);
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}


template<class T>
bool VoxelMatrix<T>::load_from_file_bin(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "rb");
	if(!fp)
	{
		printf("Cannot open the file <%s> for reading VoxelMatrix.\n", filename);
		return false;
	}

	int nx_ = 0;
	int ny_ = 0;
	int nz_ = 0;
	int count = fread(&nx_, sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when reading file %s!\n", filename);
		fclose(fp);
		return false;
	}
	count = fread(&ny_, sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when reading file %s!\n", filename);
		fclose(fp);
		return false;
	}
	count = fread(&nz_, sizeof(int), 1, fp);
	if( count != 1 )
	{
		printf("Error when reading file %s!\n", filename);
		fclose(fp);
		return false;
	}

	this->set_size(nx_, ny_, nz_);

	count = fread(this->data, sizeof(T), nx_*ny_*nz_, fp);
	if( count != nx*ny*nz )
	{
		printf("Error when reading file %s, %d elements read while %d needed to be read \n", filename, count, nx*ny*nz);
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}

template<class T>
bool VoxelMatrix<T>::save_to_rawbin(const char* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "wb");
	if(!fp)
	{
		printf("Cannot open the file <%s> for saving VoxelMatrix.\n", filename);
		return false;
	}

	int count = fwrite(this->data, sizeof(T), nx*ny*nz, fp);
	if( count != nx*ny*nz )
	{
		printf("Error when writing file %s, %d elements written while %d needed to be writen \n", filename, count, nx*ny*nz);
		fclose(fp);
		return false;
	}

	fclose(fp);
	return true;
}


#endif