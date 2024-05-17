#ifndef __BLOCKEDHESSIANMATRIX_H__
#define __BLOCKEDHESSIANMATRIX_H__

#include "opencv2\opencv.hpp"
#include "UtilMatrix.h"

struct HessianBlock
{
public:
	HessianBlock()
		:row(-1), col(-1), vals(NULL), para_count(0),
		vtii_first(0), vts_num(0), bGraphEdge(false) 
	{}
	~HessianBlock(){}
public:

public:
	int row;
	int col; // row >= col--lower triangle matrix
	float *vals; //pointer to jtj block, does not own data
	int para_count; //12 right now

	int vtii_first;
	int vts_num;
	bool bGraphEdge; //use for cpu regularization term

public:
	inline float& at(int row, int col)
	{
		return vals[row * para_count + col];
	}
	inline float at(int row, int col) const
	{
		return vals[row * para_count + col];
	}
};

class BlockedHessianMatrix
{
public:
	BlockedHessianMatrix()
		: blk_num(0),
		para_blk_num(0),
		blocks(NULL),
		data(NULL)
	{}
	~BlockedHessianMatrix()
	{
		free_space();
	}

public:
	void allocate_space_equal_block_size(int para_blk_size = 12)
	{
		if (blk_num > 0 && para_blk_size > 0)
		{
			data = new float[para_blk_size*para_blk_size*blk_num];
			for (int i = 0; i < blk_num; i++)
			{
				blocks[i].vals = data + para_blk_size*para_blk_size*i;
				blocks[i].para_count = para_blk_size;
			}
		}
	}

	void free_space()
	{
		if (data)
		{
			delete[] data;
			data = NULL;
		}
		if (blocks != NULL)
		{
			delete[] blocks;
			blocks = NULL;
		}
		blk_num = 0;
		para_blk_num = 0;
	}

	cv::Mat to_dense() const
	{
		if (blocks == NULL ||
			para_blk_num == 0)
			return cv::Mat();

		cv::Mat ret = cv::Mat(para_blk_num * 12, para_blk_num * 12, CV_64F, 0);

		for (int i = 0; i < blk_num; i++)
		{
			int row = blocks[i].row;
			int col = blocks[i].col;

			for (int m = 0; m < 12; m++)
			for (int n = 0; n < 12; n++)
			{
				ret.at<double>(row * 12 + m, col * 12 + n) = blocks[i].at(m, n);
			}
		}

		return ret;
	}

public:
	int blk_num;
	int para_blk_num;
	HessianBlock *blocks;
	float* data;
};

#endif