#include "stdafx.h"
#include "matrix_types_cuda_io.h"

bool save_BlockMatrixFullCR_ascii(char const* filename, BlockMatrixFullCR const&A_bcsr)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if (!fp)
	{
		printf("Error<save_BlockMatrixFullCR_ascii>: Cannot open the file <%s>.\n", filename);
		return false;
	}

	int blks_num = A_bcsr.blks_num.max_size;
	int para_blks_num = A_bcsr.para_blks_num.max_size;
	int para_blk_dim = A_bcsr.para_blk_dim;

	fprintf(fp, "BlockMatrix Full Compressed Row: <blks_num=%d, para_blks_num=%d, para_blk_dim=%d>\n", 
		blks_num, para_blks_num, para_blk_dim);

	fprintf(fp, "row_ptr:\n");
	for (int i = 0; i < para_blks_num + 1; i++)
		fprintf(fp, "%d ", A_bcsr.brow_ptr[i]);
	fprintf(fp, "\n");

	fprintf(fp, "col_ind:\n");
	for (int i = 0; i < blks_num; i++)
		fprintf(fp, "%d ", A_bcsr.bcolind[i]);
	fprintf(fp, "\n");

	fprintf(fp, "data: \n");
	for (int i = 0; i < blks_num * para_blk_dim*para_blk_dim; i++)
		fprintf(fp, "%f ", A_bcsr.data[i]);
	fprintf(fp, "\n");

	fprintf(fp, "row_ind:\n");
	for (int i = 0; i < blks_num; i++)
		fprintf(fp, "%d ", A_bcsr.browind[i]);
	fprintf(fp, "\n");

	fprintf(fp, "extern data offset:\n");
	for (int i = 0; i < blks_num; i++)
		fprintf(fp, "%d ", A_bcsr.extern_data_offset[i]);
	fprintf(fp, "\n");

	fprintf(fp, "diag blk pos:\n");
	for (int i = 0; i < para_blks_num; i++)
		fprintf(fp, "%d ", A_bcsr.diag_blk_pos[i]);
	fprintf(fp, "\n");

	fclose(fp);

	return true;
}


bool load_BlockMatrixFullCR_ascii(char const* filename, BlockMatrixFullCR &A_bcsr)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "r");
	if (!fp)
	{
		printf("Error<load_BlockMatrixFullCR_ascii>: Cannot open the file <%s>.\n", filename);
		return false;
	}

	int blks_num = 0;
	int para_blks_num = 0;
	int para_blk_dim = 0;
	fscanf(fp, "BlockMatrix Full Compressed Row: <blks_num=%d, para_blks_num=%d, para_blk_dim=%d>\n",
		&blks_num, &para_blks_num, &para_blk_dim);
	A_bcsr.blks_num.max_size = blks_num;
	A_bcsr.para_blks_num.max_size = para_blks_num;
	A_bcsr.para_blk_dim = para_blk_dim;

	A_bcsr.brow_ptr = new int[para_blks_num + 1];
	fscanf(fp, "row_ptr:\n");
	for (int i = 0; i < para_blks_num + 1; i++)
		fscanf(fp, "%d ", &(A_bcsr.brow_ptr[i]));
	fscanf(fp, "\n");

	A_bcsr.bcolind = new int[blks_num];
	fscanf(fp, "col_ind:\n");
	for (int i = 0; i < blks_num; i++)
		fscanf(fp, "%d ", &(A_bcsr.bcolind[i]));
	fscanf(fp, "\n");

	int blk_size = para_blk_dim * para_blk_dim;
	A_bcsr.data = new float[blks_num * blk_size];
	fscanf(fp, "data: \n");
	for (int i = 0; i < blks_num * blk_size; i++)
		fscanf(fp, "%f ", &(A_bcsr.data[i]));
	fscanf(fp, "\n");

	A_bcsr.browind = new int[blks_num];
	fscanf(fp, "row_ind:\n");
	for (int i = 0; i < blks_num; i++)
		fscanf(fp, "%d ", &(A_bcsr.browind[i]));
	fscanf(fp, "\n");
	
	A_bcsr.extern_data_offset = new int[blks_num];
	fscanf(fp, "extern data offset:\n");
	for (int i = 0; i < blks_num; i++)
		fscanf(fp, "%d ", &(A_bcsr.extern_data_offset[i]));
	fscanf(fp, "\n");

	A_bcsr.diag_blk_pos = new int[para_blks_num];
	fscanf(fp, "diag blk pos:\n");
	for (int i = 0; i < para_blks_num; i++)
		fscanf(fp, "%d ", &(A_bcsr.diag_blk_pos[i]));
	fscanf(fp, "\n");

	fclose(fp);
	return true;
}


bool save_HessianBlockInfoCuda_buf_ASCII(char const* file_name, HessianBlockInfoCuda const* hessian_block_info, int count)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if (!fp)
	{
		printf("Error<save_HessianBlockInfoCuda_buf_ASCII>: Cannot open the file <%s>.\n", file_name);
		return false;
	}

	fprintf(fp, "HessianBlock Info.\n");
	fprintf(fp, "blk num: %d\n\n", count);

	for (int i = 0; i < count; i++)
	{
		fprintf(fp, "blk %d:\n", i);
		fprintf(fp, "pi_idx = %d\n", hessian_block_info[i].pi_idx);
		fprintf(fp, "pj_idx = %d\n", hessian_block_info[i].pj_idx);

		fprintf(fp, "pi_dim = %d\n", hessian_block_info[i].pi_dim);
		fprintf(fp, "pj_dim = %d\n", hessian_block_info[i].pj_dim);

		fprintf(fp, "vtii_first = %d\n", hessian_block_info[i].vtii_first);
		fprintf(fp, "vts_num = %d\n", hessian_block_info[i].vts_num);

		fprintf(fp, "data_offset = %d\n", hessian_block_info[i].data_offset);

		fprintf(fp, "\n\n");
	}

	fclose(fp);
	return true;
}


bool readout_JtJBlockMatrixLTri_from_GPU(JtJBlockMatrixLTri const&jtj_gpu, BlockedHessianMatrix &jtj_cpu)
{
	int diag_blk_count = jtj_gpu.para_blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	int blks_num = jtj_gpu.blks_num.debug_sync_read(LOGGER()->check_verbosity(Logger::Trace));
	int para_blk_dim = jtj_gpu.para_blk_dim;

	HessianBlockInfoCuda *jtj_blk_info = new HessianBlockInfoCuda[blks_num];
	checkCudaErrors(cudaMemcpy(jtj_blk_info, jtj_gpu.blk_info, sizeof(HessianBlockInfoCuda)*blks_num, cudaMemcpyDeviceToHost));

	jtj_cpu.blk_num = blks_num;
	jtj_cpu.para_blk_num = diag_blk_count;
	jtj_cpu.blocks = new HessianBlock[blks_num];
	jtj_cpu.data = new float[blks_num*para_blk_dim*para_blk_dim];
	checkCudaErrors(cudaMemcpy(jtj_cpu.data, jtj_gpu.data, sizeof(float)*para_blk_dim*para_blk_dim*blks_num, cudaMemcpyDeviceToHost));

	for (int i = 0; i < blks_num; i++)
	{
		HessianBlockInfoCuda const& blk_cuda = jtj_blk_info[i];

		HessianBlock &blk = jtj_cpu.blocks[i];
		blk.row = blk_cuda.pi_idx;
		blk.col = blk_cuda.pj_idx;
		blk.vtii_first = blk_cuda.vtii_first;
		blk.vts_num = blk_cuda.vts_num;
		blk.para_count = para_blk_dim;
		blk.bGraphEdge = false;
		blk.vals = jtj_cpu.data + blk_cuda.data_offset;
	}
	delete[] jtj_blk_info;
	return true;
}


bool free_BlockMatrixFullCR_host(BlockMatrixFullCR &bcsr_mat)
{
	delete[] bcsr_mat.brow_ptr;
	delete[] bcsr_mat.bcolind;
	delete[] bcsr_mat.browind;
	delete[] bcsr_mat.diag_blk_pos;
	delete[] bcsr_mat.extern_data_offset;
	delete[] bcsr_mat.data;
	bcsr_mat.para_blks_num.max_size = 0;
	//bcsr_mat.para_blks_num.copy_transformed(bcsr_mat.para_blks_num, 0, 0);
	bcsr_mat.para_blk_dim = 0;
	bcsr_mat.blks_num.max_size = 0;
	//bcsr_mat.blks_num.copy_transformed(bcsr_mat.blks_num, 0, 0);
	return true;
}

// Note: if bcsr_dst is on the host, it does not need to allocate memory beforehand. 
//       otherwise the gpu memory should be setup beforehand.
bool BlockMatrixFullCR_cudaMemcpy(BlockMatrixFullCR const& bcsr_src, BlockMatrixFullCR &bcsr_dst, cudaMemcpyKind kind)
{
	abort(); // NOT IMPLEMENTED
	/*
	int rows = bcsr_src.para_blks_num;
	int blk_num = bcsr_src.blks_num;
	int para_blk_dim = bcsr_src.para_blk_dim;
	int blk_size = para_blk_dim*para_blk_dim;

	bcsr_dst.para_blks_num = bcsr_src.para_blks_num;
	bcsr_dst.para_blk_dim = bcsr_src.para_blk_dim;
	bcsr_dst.blks_num = bcsr_src.blks_num;

	if (kind == cudaMemcpyDeviceToHost)
	{
		bcsr_dst.brow_ptr = new int[rows + 1];
		bcsr_dst.bcolind = new int[blk_num];
		bcsr_dst.browind = new int[blk_num];
		bcsr_dst.diag_blk_pos = new int[rows];
		bcsr_dst.extern_data_offset = new int[blk_num];
		bcsr_dst.data = new float[blk_num * blk_size];
	}

	checkCudaErrors(cudaMemcpy(bcsr_dst.brow_ptr, bcsr_src.brow_ptr, sizeof(int)*(rows + 1), kind));
	checkCudaErrors(cudaMemcpy(bcsr_dst.bcolind, bcsr_src.bcolind, sizeof(int)*blk_num, kind));
	checkCudaErrors(cudaMemcpy(bcsr_dst.browind, bcsr_src.browind, sizeof(int)*blk_num, kind));
	checkCudaErrors(cudaMemcpy(bcsr_dst.diag_blk_pos, bcsr_src.diag_blk_pos, sizeof(int)*rows, kind));
	checkCudaErrors(cudaMemcpy(bcsr_dst.extern_data_offset, bcsr_src.extern_data_offset, sizeof(int)*blk_num, kind));
	checkCudaErrors(cudaMemcpy(bcsr_dst.data, bcsr_src.data, sizeof(float)*blk_num*blk_size, kind));
	*/
	return true;
}

cv::Mat* BlockMatrixFullCR_to_dense(BlockMatrixFullCR const&bcsr_mat_host)
{
	int para_blk_dim = bcsr_mat_host.para_blk_dim;
	int blk_rows = bcsr_mat_host.para_blks_num.max_size;
	int w = blk_rows*bcsr_mat_host.para_blk_dim;
	int h = w;
	cv::Mat* A = &cv::Mat(h, w, CV_64F);

	for (int i = 0; i < blk_rows; i++)
	{
		int pi = i;
		for (int j = bcsr_mat_host.brow_ptr[i]; j < bcsr_mat_host.brow_ptr[i + 1]; j++)
		{
			int pj = bcsr_mat_host.bcolind[j];
			int data_offset = j*para_blk_dim*para_blk_dim;

			float* p_blk_data = bcsr_mat_host.data + data_offset;

			for (int m = 0; m < 12; m++)
			{
				for (int n = 0; n < 12; n++)
				{
					int row = pi * 12 + m;
					int col = pj * 12 + n;
					A->at<double>(row, col) = p_blk_data[m * 12 + n];
				}
			}
		}
	}

	return A;
}
