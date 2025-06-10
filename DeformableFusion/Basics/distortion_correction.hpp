// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __DISTORTION_CORRECTION_HPP__
#define __DISTORTION_CORRECTION_HPP__

#include "distortion_correction.h"

template <class T>
bool undistort( vnl_matrix<T> &mat_in,
				vnl_matrix<T> &mat_out,
				vnl_matrix_fixed<double, 3, 3> &K, 
				vnl_vector<double> &dist_coef,
				T fill_value,
				interpolation_method method)
{
	vnl_matrix<double> MapX, MapY;
	vnl_vector_fixed<int, 2> image_size(mat_in.cols(), mat_in.rows());
	bool ret = build_undistortion_map(MapX, MapY, K, dist_coef, image_size);
	if(!ret)
		return false;

	return remap<T>(mat_in, mat_out, MapX, MapY, fill_value, method);
}

template <class T>
bool undistort( vil_image_view<T> &img_in,
				vil_image_view<T> &img_out,
				vnl_matrix_fixed<double, 3, 3> &K, 
				vnl_vector<double> &dist_coef,
				T fill_value,
				interpolation_method method)
{
	vnl_matrix<double> MapX, MapY;
	vnl_vector_fixed<int, 2> image_size(img_in.ni(), img_in.nj());
	bool ret = build_undistortion_map(MapX, MapY, K, dist_coef, image_size);
	if(!ret)
		return false;

	return remap<T>(img_in, img_out, MapX, MapY, fill_value, method);
}

template <class T>
bool remap( vnl_matrix<T> &mat_in, 
			vnl_matrix<T> &mat_out, 
			vnl_matrix<double> &MapX, 
			vnl_matrix<double> &MapY, 
			T fill_value,
			interpolation_method method)
{
	if( mat_in.rows() != MapX.rows() ||
		mat_in.cols() != MapY.cols() )
		return false;

	mat_out.set_size(mat_in.rows(), mat_in.columns());

	int h = mat_in.rows();
	int w = mat_in.cols();

	if( method == inter_nearest_neighborhood )
	{
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				int x_src = ROUND(MapX[i][j]);
				int y_src = ROUND(MapY[i][j]);

				if( x_src >=0 && x_src < w &&
					y_src >=0 && y_src < h)
				{
					mat_out[i][j] = mat_in[y_src][x_src];
				}
				else
					mat_out[i][j] = fill_value;
			}
		}
	}
	else
	{
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				double x_src = MapX[i][j];
				double y_src = MapY[i][j];

				if( x_src >=0 && x_src < w &&
					y_src >=0 && y_src < h)
				{
					mat_out[i][j] = pickAMatElement(mat_in, x_src, y_src);
				}
				else
					mat_out[i][j] = fill_value;
			}
		}
	}

	return true;
}


template <class T>
bool remap( vil_image_view<T> &img_in, 
			vil_image_view<T> &img_out, 
			vnl_matrix<double> &MapX, 
			vnl_matrix<double> &MapY, 
			T fill_value,
			interpolation_method method)
{
	if( img_in.nj() != MapX.rows() ||
		img_in.ni() != MapY.cols() )
		return false;

	int h = img_in.nj();
	int w = img_in.ni();
	int nplanes = img_in.nplanes();

	img_out.set_size(w, h, nplanes);

	if( method == inter_nearest_neighborhood )
	{
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				int x_src = ROUND(MapX[i][j]);
				int y_src = ROUND(MapY[i][j]);

				if( x_src >=0 && x_src < w &&
					y_src >=0 && y_src < h)
				{
					for(int c=0; c<nplanes; c++)
						img_out(j, i, c) = img_in(x_src, y_src, c);
				}
				else
				{
					for(int c=0; c<nplanes; c++)
						img_out(j, i, c) = fill_value;
				}
			}
		}
	}
	else
	{
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				double x_src = MapX[i][j];
				double y_src = MapY[i][j];

				if( x_src >=0 && x_src <= w-1 &&
					y_src >=0 && y_src <= h-1)
				{
					for(int c=0; c<nplanes; c++)
						img_out(j, i, c) = vil_bilin_interp(img_in, x_src, y_src, c);
				}
				else
				{
					for(int c=0; c<nplanes; c++)
						img_out(j, i, c) = fill_value;
				}
			}
		}
	}
}

#endif