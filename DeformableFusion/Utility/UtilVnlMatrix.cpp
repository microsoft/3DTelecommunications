#include "stdafx.h"
#include "UtilVnlMatrix.h"

vnl_vector<double> get_mean_vector(vnl_matrix<double> mat, int dir)
{
	int h = mat.rows();
	int w = mat.cols();

	if( dir == 1 )
	{
		vnl_vector<double> mean_vec(w, 0.0);

		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				mean_vec[j] += mat[i][j];
			}
		}

		for(int j=0; j<w; j++)
			mean_vec[j] /= h;

		return mean_vec;
	}
	else 
	{
		vnl_vector<double> mean_vec(h, 0.0);

		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				mean_vec[i] += mat[i][j];
			}
			mean_vec[i] /= w;
		}

		return mean_vec;
	}
}

vnl_vector<double> get_mean_vector(vector< vnl_vector<double> > &vecs )
{
	vnl_vector<double> ret;
	if( vecs.size() == 0)
		return ret;

	int vec_len = vecs[0].size();
	ret.set_size(vec_len);
	ret.fill(0.0);

	for(int i=0; i<vecs.size(); i++)
	{
		ret += vecs[i];
	}

	ret = ret / vecs.size();

	return ret;
}

bool sub_vec_from_mat(vnl_matrix<double> &mat, vnl_vector<double> vec, int dir)
{
	if( dir == 1 )
	{
		if( mat.cols() != vec.size() )
		{
			printf("sub_vec_from_mat: operation cannot be finished (size does not match)!\n");
			return false;
		}
	}
	else if( dir == 2 )
	{
		if( mat.rows() != vec.size() )
		{
			printf("sub_vec_from_mat: operation cannot be finished (size does not match)!\n");
			return false;
		}
	}
	else
	{
		if( mat.rows() == vec.size() )
			dir = 2;
		else if( mat.cols() == vec.size() )
			dir = 1;
		else
		{
			printf("sub_vec_from_mat: operation cannot be finished (size does not match)!\n");
			return false;
		}
	}

	if( dir == 1)
	{
		for(unsigned int i=0; i<mat.rows(); i++)
		{
			for(unsigned int j=0; j<mat.cols(); j++)
			{
				mat[i][j] -= vec[j];
			}
		}
	}
	else if(dir == 2)
	{
		for(unsigned int i=0; i<mat.rows(); i++)
		{
			for(unsigned int j=0; j<mat.cols(); j++)
			{
				mat[i][j] -= vec[i];
			}
		}
	}

	return true;
}

void permutation_vector( vnl_vector<int> &perm_vec, int num)
{
	perm_vec.set_size(num);

	vector<int> nums;
	for(int i=0; i<num; i++)
		nums.push_back(i);

	double t = RANDOM;

	for(int i=0; i<num; i++)
	{
		int n = int(RANDOM * num);

		n = n % nums.size();

		perm_vec[i] = nums[n];
		nums.erase(nums.begin() + n);
	}
}


bool loadDepthMatFromPNG( const char* name, vnl_matrix<double> &depthMat, bool bUseMostSignificantBits,
						  double bias_a, double bias_b )
{
	cv::Mat* img = &cv::imread(name, cv::IMREAD_UNCHANGED);
	if( img == NULL || img->depth() != CV_16UC1)
	{
		printf("Error<loadDepthMatFromPNG>: file not found or with wrong format(16U only)!\n");
		return false;
	}

	int w = img->cols;
	int h = img->rows;
	depthMat.set_size(h, w);	

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned short ori_depth = img->at<unsigned short>(i, j);
			if( bUseMostSignificantBits )
				depthMat[i][j] = (ori_depth>>3)/10.0;
			else
				depthMat[i][j] = ori_depth/10.0;

			if( depthMat[i][j] > 0.001 )
			{
				depthMat[i][j] *= bias_a;
				depthMat[i][j] += bias_b;
			}
		}
	}

	img->release();
	return true;
}

bool loadDepthMatFromPNG( cv::Mat* img, vnl_matrix<double> &depthMat, bool bUseMostSignificantBits,
						  double bias_a, double bias_b )
{
	if( img == NULL || img->depth() != CV_16UC1 )
	{
		printf("Error<loadDepthMatFromPNG>: file not found or with wrong format(16U only)!\n");
		return false;
	}

	int w = img->cols;
	int h = img->rows;
	depthMat.set_size(h, w);	

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned short ori_depth = img->at<unsigned short>(i, j);
			if( bUseMostSignificantBits )
				depthMat[i][j] = (ori_depth>>3)/10.0;
			else
				depthMat[i][j] = ori_depth/10.0;

			if( depthMat[i][j] > 0.001 )
			{
				depthMat[i][j] *= bias_a;
				depthMat[i][j] += bias_b;
			}
		}
	}
	return true;
}

void applyDepthMatBias(vnl_matrix<double> &depthMat, CDepthBias const& depth_bias)
{
	for(int i=0; i<depthMat.rows(); i++)
	{
		for(int j=0; j<depthMat.cols(); j++)
		{
			if( depthMat[i][j] > 0.001)
				depthMat[i][j] = depth_bias.correct_depth_bias(i, j, depthMat[i][j], false);
		}
	}
}

bool saveDepthMatToPNG(const char* name, vnl_matrix<double> const&depthMat, bool bUseMostSignificantBits)
{
	//check the file name
	int len = strlen(name);
	if( (name[len-3] != 'p' && name[len-3] != 'P') ||
		(name[len-2] != 'n' && name[len-2] != 'N') ||
		(name[len-1] != 'g' && name[len-1] != 'G') )
	{
		printf("Error: the extension of the file should be PNG.\n");
		return false;
	}

	int w = depthMat.cols();
	int h = depthMat.rows();
	cv::Mat* img = &cv::Mat(h, w, CV_16UC1);
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned short ori_depth = unsigned short(depthMat[i][j]*10.0);
			if( bUseMostSignificantBits )
				img->at<unsigned short>(i, j) = ori_depth<<3;
			else
				img->at<unsigned short>(i, j) = ori_depth;
		}
	}
	cv::imwrite(name, *img);

	img->release();
	return true;
}
