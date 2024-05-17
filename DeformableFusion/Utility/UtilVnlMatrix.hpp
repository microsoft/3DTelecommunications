//===============================================
//			UtilVnlMatrix.hpp
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __MYVNLMATRIX_HPP__
#define __MYVNLMATRIX_HPP__
#include "UtilVnlMatrix.h"
#include "vgl/vgl_vector_3d.h"

template <class T>
T pickAMatElement(vnl_matrix<T> const&mat, int x, int y)
{
	if( x<0 || x >= mat.cols() || y<0 || y>=mat.rows())
		return 0;
	else
		return mat[y][x];
}

template <class T>
T pickAMatElement(vnl_matrix<T> const&mat, double x, double y)
{
	int x0 = (int)( floor(x) );
	int y0 = (int)( floor(y) );
	int x1 = (int)( ceil(x) );
	int y1 = (int)( ceil(y) );

	double a = x-x0;
	double b = y-y0;

	T P00 = pickAMatElement(mat, x0, y0);
	T P01 = pickAMatElement(mat, x1, y0);
	T P10 = pickAMatElement(mat, x0, y1);
	T P11 = pickAMatElement(mat, x1, y1);

	return (P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b;
}

template <class T>
T dist_3d(vnl_vector_fixed<T, 3> const&p, vnl_vector_fixed<T, 3> const&q)
{
	vnl_vector_fixed<T, 3> d = p-q;
	return d.two_norm();
}

template <class T, int N>
T dist_nd(vnl_vector_fixed<T, N> const&p, vnl_vector_fixed<T, N> const&q)
{
	vnl_vector_fixed<T, N> d = p - q;
	return d.two_norm();
}

template<class T>
T value_at(vnl_vector<T> const&vec, int i)
{
	if( i < 0 || i >= (int) vec.size() )
		return T(0.0);
	else
		return vec[i];
}

template<class T>
T value_at(vnl_matrix<T> const&mat, int i, int j)
{
	if( i<0 || i>=mat.rows() ||
		j<0 || j>=mat.cols() )
		return T(0.0);
	else
		return mat[i][j];
}

template<class T>
T value_at(vnl_matrix<T> const&mat, double i, double j)
{
	int i0 = (int)( floor(i) );
	int j0 = (int)( floor(j) );
	int i1 = (int)( ceil(i) );
	int j1 = (int)( ceil(j) );

	double a = j-j0;
	double b = i-i0;

	T P00 = value_at(mat, i0, j0);
	T P01 = value_at(mat, i0, j1);
	T P10 = value_at(mat, i1, j0);
	T P11 = value_at(mat, i1, j1);

	return (T)((P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b);
}

template <class T>
void matrix_max(vnl_matrix<T> const& mat, T &max_val, int &row_idx, int &col_idx)
{
	if( mat.rows() == 0 || mat.cols() == 0)
	{
		max_val = 0;
		row_idx = -1;
		col_idx = -1;
		return;
	}

	row_idx = 0;
	col_idx = 0;
	max_val = mat[0][0];
	for(unsigned int i=0; i<mat.rows(); i++)
	{
		for(unsigned int j=0; j<mat.cols(); j++)
		{
			if( mat[i][j] > max_val )
			{
				row_idx = i;
				col_idx = j;
				max_val = mat[i][j];
			}
		}
	}
}

template <class T>
void matrix_min(vnl_matrix<T> const& mat, T &min_val, int &row_idx, int &col_idx)
{
	if( mat.rows() == 0 || mat.cols() == 0)
	{
		min_val = 0;
		row_idx = -1;
		col_idx = -1;
		return;
	}

	row_idx = 0;
	col_idx = 0;
	min_val = mat[0][0];
	for(int i=0; i<mat.rows(); i++)
	{
		for(int j=0; j<mat.cols(); j++)
		{
			if( mat[i][j] < min_val )
			{
				row_idx = i;
				col_idx = j;
				min_val = mat[i][j];
			}
		}
	}
}

template <int N>
vnl_vector_fixed<double, N> get_mean_vector(vector< vnl_vector_fixed<double, N> > const&vecs)
{
	vnl_vector_fixed<double, N> ret;
	ret.fill(0.0);
	if( vecs.size() == 0)
		return ret;
	
	for(int i=0; i<vecs.size(); i++)
	{
		ret += vecs[i];
	}
	ret = ret / double(vecs.size());

	return ret;
}

template <class T>
bool delete_vector_items( vector<T> &vec, vector<bool> &bDeleteFlag)
{
	if( vec.size() != bDeleteFlag.size() )
		return false;

	int num_deleted = 0;
	vector<T> vec_bak;
	for(int i=0; i<bDeleteFlag.size(); i++)
	{
		if( !bDeleteFlag[i])
			vec_bak.push_back(vec[i]);
	}
	vec = vec_bak;
	return true;
}


template <class T>
bool loadVNLMatrixBin(const char* filename, vnl_matrix<T> &mat)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File\n");
		return false;
	}

	int rows, cols;
	numread = (long) fread(&rows, sizeof(int), 1, file);
	if( numread != 1 )
	{
		//printf("Error when Reading File\n");
		fclose(file);
		return false;
	}
	numread = (long) fread(&cols, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		fclose(file);
		return false;
	}

	mat.set_size(rows, cols);
	
	numread = (long) fread( mat.data_block(), sizeof(T), mat.rows() * mat.cols(), file);
	if( numread != mat.rows() * mat.cols() )
	{
		printf("Error when Reading File\n");
		mat.set_size(0, 0);
		fclose(file);
		return false;
	}

	fclose(file);

	return true;
}

template <class T>
bool saveVNLMatrixBin(const char* filename, vnl_matrix<T> &mat)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing File\n");
		return false;
	}

	int height = mat.rows();
	int numwritten = (int) fwrite(&height, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		fclose(file);
		return false;
	}

	int width = mat.cols();
	numwritten = (int) fwrite(&width, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		fclose(file);
		return false;
	}

	numwritten = (int) fwrite(mat.data_block(), sizeof(T), mat.rows() * mat.cols(), file);
	if( numwritten != mat.rows() * mat.cols() )
	{
		printf("Error when writing data\n");
		fclose(file);
		return false;
	}

	fclose(file);
	return true;
}

template <class T>
bool saveDataBlock(const char* filename, T* data_block, int size)
{
	ofstream data_file(filename);

	for(int i=0; i<size; i++)
		data_file<<data_block[i]<<vcl_endl;
	data_file.close();

	return true;
}

template <class T>
bool loadDataBlock(const char* filename, T* data_block, int size)
{
	ifstream data_file(filename);

	for(int i=0; i<size; i++)
		data_file>>data_block[i];
	data_file.close();

	return true;
}

template <class T>
bool saveVNLMatrixASCAII(const char* filename, vnl_matrix<T> const&mat)
{
	ofstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Error: cannot open file for saving <%s>\n", filename);
		return false;
	}

	data_file<<mat;
	
	data_file.close();

	return true;
}

template <class T>
bool saveVNLMatrix2IMG(const char* filename, vnl_matrix<T> const&mat)
{
	T max_val = mat.max_value();
	T min_val = mat.min_value();

	int h = mat.rows();
	int w = mat.cols();
	if( h<=0||w<=0||
		max_val==min_val )
	{
		printf("Error in saveVNLMatrix2IMG<%s>: all zeros!\n", filename);
		return false;
	}
	cv::Mat* img = &cv::Mat(h, w, CV_8UC1);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			img->at<uchar>(i, j) = (double)(mat[i][j]-min_val)/(double)(max_val-min_val) * 255.0;
		}
	}
	cv::imwrite(filename, *img);
	img->release();
	return true;
}

template <class T>
bool loadVNLMatrixASCAII(const char* filename, vnl_matrix<T> &mat)
{
	if(filename == NULL)
		return false;
	ifstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Error: cannot open file for reading <%s>\n", filename);
		return false;
	}

	data_file>>mat;
	
	data_file.close();

	return true;
}

template <class T>
bool saveVNLVectorASCAII(const char* filename, vnl_vector<T> const&vec, char dir)
{
	ofstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Error: cannot open file for saving <%s>\n", filename);
		return false;
	}

	for (unsigned int i = 0; i < vec.size(); i++)
	{
		data_file<<vec[i];
		if( dir == 'c')
			data_file<<endl;
		else
			data_file<<"\t";
	}
	
	data_file.close();

	return true;
}

template <class T>
bool loadVNLVectorASCAII(const char* filename, vnl_vector<T> &vec)
{
	if(filename == NULL)
		return false;
	ifstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Error: cannot open file for reading <%s>\n", filename);
		return false;
	}

	data_file>>vec;
	
	data_file.close();

	return true;
}

template<class T>
bool saveVnlVectorSetASCAII(const char* filename, vector< vnl_vector<T> > const&vecs)
{
	ofstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Cannot open the file <%s> for saving VnlVector Set.\n", filename);
		return false;
	}
	
	for(int i=0; i<vecs.size(); i++)
		data_file<<vecs[i]<<endl;

	data_file.close();
	return true;
}

template<class T, int N>
bool saveVnlVectorSetASCAII(const char* filename, vector< vnl_vector_fixed<T, N> > const&vecs)
{
	ofstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Cannot open the file <%s> for saving VnlVector Set.\n", filename);
		return false;
	}
	
	for(int i=0; i<vecs.size(); i++)
		data_file<<vecs[i]<<endl;

	data_file.close();
	return true;
}


template<class T, int N>
bool loadVnlVectorSetASCAII(const char* filename, vector< vnl_vector_fixed<T, N> > &vecs)
{
	ifstream data_file(filename);

	if( !data_file.is_open() )
	{
		printf("Cannot open the file <%s> for loading VnlVector Set.\n", filename);
		return false;
	}
	
	vecs.clear();
	while(true)
	{
		vnl_vector_fixed<T, N> vec;
		data_file>>vec;
		if( data_file.good() )
			vecs.push_back(vec);
		else
			break;
	}

	data_file.close();
	return true;
}

template<class T>
vnl_vector_fixed<T, 3> cross_product(vnl_vector_fixed<T, 3> const&a, vnl_vector_fixed<T, 3> const&b)
{
	return vnl_vector_fixed<T, 3>(a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]);
}

#endif