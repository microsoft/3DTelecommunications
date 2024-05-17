#ifndef __CCLCUDA_H__
#define __CCLCUDA_H__
#include <vector>
#include "CCLCudaImpl.h"
#include "UtilMatrix.h"
#include "UtilVnlMatrix.h"

class CCLCuda : public CCLCudaImpl
{
public:
	CCLCuda(int width, int height, int view_num)
		:CCLCudaImpl(width, height, view_num)
	{}
	~CCLCuda(){}

public:
	void readout_labels(std::vector<vnl_matrix<int>> &mat_labels);
};

#endif