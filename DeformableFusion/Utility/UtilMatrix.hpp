//===============================================
//			UtilMatrix.hpp
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __UTILMATRIX_HPP_
#define __UTILMATRIX_HPP_
#include "UtilMatrix.h"

template<class T>
void pickAColorPixel(cv::Mat const& img, int x, int y, T pixel[])
{
	if (x<0 || x >= img.cols || y<0 || y >= img.rows)
	{
		pixel[0] = 0.0;
		pixel[1] = 0.0;
		pixel[2] = 0.0;
		return;
	}
	cv::Vec3b src = img.ptr<cv::Vec3b>(y)[x];
	pixel[0] = src[0];
	pixel[1] = src[1];
	pixel[2] = src[2];
}

template<class T>
void pickAColorPixel(cv::Mat const& img, double x, double y, T pixel[])
{
	int x0 = (int)(floor(x));
	int y0 = (int)(floor(y));
	int x1 = (int)(ceil(x));
	int y1 = (int)(ceil(y));

	double a = x - x0;
	double b = y - y0;

	T P00[3];
	T P01[3];
	T P10[3];
	T P11[3];

	pickAColorPixel(img, x0, y0, P00);
	pickAColorPixel(img, x1, y0, P01);
	pickAColorPixel(img, x0, y1, P10);
	pickAColorPixel(img, x1, y1, P11);

	pixel[0] = (P00[0] * (1.0 - a) + P01[0] * a)*(1.0 - b) + (P10[0] * (1.0 - a) + P11[0] * a)*b;
	pixel[1] = (P00[1] * (1.0 - a) + P01[1] * a)*(1.0 - b) + (P10[1] * (1.0 - a) + P11[1] * a)*b;
	pixel[2] = (P00[2] * (1.0 - a) + P01[2] * a)*(1.0 - b) + (P10[2] * (1.0 - a) + P11[2] * a)*b;
}


template<class T>
bool saveStdVectorASCII(char const*filename, std::vector<T> const&vec)
{
	FILE *file = NULL;
	if ((fopen_s(&file, filename, "w")) != 0)
	{
		printf("Error<saveStdVectorASCII> when writing file<%s>\n", filename);
		return false;
	}

	fprintf(file, "Length<%zd>\n", vec.size());
	for (int i = 0; i < vec.size(); i++)
	{
		fprintf(file, "%f\n", vec[i]);
	}
	fclose(file);
	return true;
}

template<class T>
bool loadStdVectorASCII(char const*filename, std::vector<T> &vec)
{
	FILE *file = NULL;
	if ((fopen_s(&file, filename, "r")) != 0)
	{
		printf("Error<loadStdVectorASCII> when reading file<%s>\n", filename);
		return false;
	}

	int len = 0;
	fscanf(file, "Length<%d>\n", &len);
	vec.resize(len);
	for (int i = 0; i < vec.size(); i++)
	{
		double tmp = 0;
		fscanf(file, "%lf\n", &tmp);
		vec[i] = (T)tmp;
	}

	fclose(file);
	return true;
}

template<class T>
bool saveVectorList(char const*filename, std::vector< std::vector<T> > const&vec_list)
{
	std::ofstream fp(filename);
	if (fp.fail())
	{
		cout << "Error<saveVectorList>: cannot open file: " << filename<<endl;
		return false;
	}

	fp << "vector list: " << vec_list.size() << " lines" << endl;
	for (int i = 0; i<vec_list.size(); i++)
	{
		fp << "Line " << i << "<" << vec_list[i].size() << ">:";
		for (int j = 0; j < vec_list[i].size(); j++)
			fp << " " << vec_list[i][j];
		fp << endl;
	}

	fp.close();
	return true;
}

template<class T>
bool saveVectorList(char const*filename, std::vector< std::vector<T>* > const&vec_list)
{
	std::ofstream fp(filename);
	if (fp.fail())
	{
		cout << "Error<saveVectorList>: cannot open file: " << filename << endl;
		return false;
	}

	fp << "vector list: " << vec_list.size() << " lines" << endl;
	for (int i = 0; i<vec_list.size(); i++)
	{
		fp << "Line " << i << "<" << vec_list[i]->size() << ">:";
		for (int j = 0; j < vec_list[i]->size(); j++)
			fp << " " << vec_list[i]->at(j);
		fp << endl;
	}

	fp.close();
	return true;
}

#endif