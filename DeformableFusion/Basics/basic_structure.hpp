// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once
#ifndef __BASIC_STRUCTURE_HPP__
#define __BASIC_STRUCTURE_HPP__
#include "basic_structure.h"

template <class T>
void rgb_to_hsv( vnl_vector_fixed<T, 3> const&rgb, vnl_vector_fixed<T, 3> &hsv )
{
	double hue, sat, val;
	double min_v;
	if( rgb[0] == rgb[1] && rgb[0] == rgb[2] )
	{	
		hue = 0.0;
		sat = 0.0;
		val = rgb[0];
	}
	else if( rgb[0] >= rgb[1] && rgb[0] >= rgb[2])
	{
		min_v = min<T>(rgb[1], rgb[2]); 
		sat = 1.0 - min_v/rgb[0];
		hue = double(rgb[1]-rgb[2])/(rgb[0]-min_v)/6.0;
		val = rgb[0];
	}
	else if( rgb[1] >= rgb[0] && rgb[1] >= rgb[2] )
	{
		min_v = min<T>(rgb[0], rgb[2]);
		sat = 1.0 - min_v/rgb[1];
		hue = (60.0*(rgb[2]-rgb[0])/(rgb[1]-min_v) + 120.0)/360.0;
		val = rgb[1];
	}
	else if( rgb[2] >= rgb[0] && rgb[2] >= rgb[1] )
	{
		min_v = min<T>(rgb[0], rgb[1]);
		sat = 1.0 - min_v/rgb[2];
		hue = (60.0*(rgb[0]-rgb[1])/(rgb[2]-min_v) + 240.0)/360.0;
		val = rgb[2];
	}
	
	if( hue < 0)
		hue += 1.0;

	hsv[0] = (T) (hue*255.0);
	hsv[1] = (T) (sat*255.0);
	hsv[2] = val;
}


#endif