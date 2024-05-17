#include "stdafx.h"
#include "CCLCuda.h"
#include <helper_cuda.h>


//void CCLCuda::
//readout_labels(std::vector<vnl_matrix<int>> &mat_labels)
//{
//	mat_labels.clear();
//	mat_labels.resize(view_num_);
//
//	cudaMemcpy3DParms paras = { 0 };
//	paras.srcPos = make_cudaPos(0, 0, 0);
//	paras.dstPos = make_cudaPos(0, 0, 0);
//	paras.srcArray = cu_3dArr_labels_;
//	paras.dstPtr = make_cudaPitchedPtr(NULL, width_ * sizeof(int),width_, height_);
//	paras.extent = make_cudaExtent(width_, height_, 1);
//	paras.kind = cudaMemcpyDeviceToHost;
//	for (int i = 0; i < view_num_; i++)
//	{
//		mat_labels[i].set_size(height_, width_);
//		paras.srcPos = make_cudaPos(0, 0, i);
//		paras.dstPtr = make_cudaPitchedPtr(mat_labels[i].data_block(), width_ * sizeof(int), width_, height_);
//		checkCudaErrors(cudaMemcpy3D(&paras));
//	}
//}

void CCLCuda::
readout_labels(std::vector<vnl_matrix<int>> &mat_labels)
{
	mat_labels.clear();
	mat_labels.resize(view_num_);

	int *host_labels = new int[width_*height_*view_num_];
	checkCudaErrors(cudaMemcpy(host_labels, dev_labels_all_views_, width_*height_*view_num_*sizeof(int), cudaMemcpyDeviceToHost));
	for (int i = 0; i < view_num_; i++)
	{
		mat_labels[i].set_size(height_, width_);
		//mat_labels[i].copy_in(&(host_labels[i*width_*height_]));
		memcpy(mat_labels[i].data_block(), &(host_labels[i*width_*height_]), width_*height_*sizeof(int));
	}

	delete[] host_labels;
}


