#include "stdafx.h"
#include "LinearSolverCuSolver.h"
#include "UtilVnlMatrix.h"


bool CRSMat::save_to_ascii(char const* filename)
{
	FILE *fp = NULL;
	fopen_s(&fp, filename, "w");
	if (!fp)
	{
		printf("Error<CRSMat::save_to_ascii>: Cannot open the file <%s>.\n", filename);
		return false;
	}

	fprintf(fp, "Compressed Row Sparse Matrix.\n");
	fprintf(fp, "nrow=%d\n", this->rows);
	fprintf(fp, "ncol=%d\n", this->cols);
	fprintf(fp, "nzmax=%d\n", this->nz_num);
	fprintf(fp, "\n");

	fprintf(fp, "row pointers:\n");
	for (int i = 0; i <= this->rows; i++)
	{
		fprintf(fp, "%04d\n", this->rowPtr[i]);
	}
	fprintf(fp, "\n\n");

	fprintf(fp, "colIdx, val:\n");
	for (int i = 0; i < this->nz_num; i++)
	{
		fprintf(fp, "%d, %f\n", this->colInd[i], this->vals[i]);
	}
	fprintf(fp, "\n\n");

	fclose(fp);
	return true;
}

cv::Mat *CRSMat::to_dense()
{
	cv::Mat *A = &cv::Mat(this->rows, this->cols, CV_64F, cv::Scalar(0));

	for (int i = 0; i < rows; i++)
	{
		int idxSt = this->rowPtr[i];
		int idxEnd = this->rowPtr[i+1];

		for (int j = idxSt; j < idxEnd; j++)
		{
			int col = this->colInd[j];

			A->at<double>(i, col) = this->vals[j];
		}
	}

	return A;
}

CRSMat* LinearSolverCuSolver::
create_sparse_matrix(BlockedHessianMatrix const& hessian)
{
	int rows = hessian.para_blk_num * 12;
	int cols = rows;
	//only the lower triangle matrix is saved
	int nz_num = hessian.blk_num * 144 - hessian.para_blk_num * 66;;

	CRSMat *crs = new CRSMat();
	crs->allocate_space(rows, cols, nz_num);

	vnl_matrix<HessianBlock const*> block_pointer_array(hessian.para_blk_num, hessian.para_blk_num);;
	block_pointer_array.fill(NULL);
	for (int i = 0; i < hessian.blk_num; i++)
	{
		int row = hessian.blocks[i].row;
		int col = hessian.blocks[i].col;
		block_pointer_array[row][col] = &(hessian.blocks[i]);
	}

	int para_blk_num = hessian.para_blk_num;
	int nz_count = 0;
	for (int r = 0; r < para_blk_num; r++) //r: parameter block row
	{
		int blks_num_per_row = 0; //number of parameter blocks in current row
		for (int c = 0; c<=r; c++)
		{
			if (block_pointer_array[r][c])
				blks_num_per_row++;
		}

		//off-diag term
		int blk_idx_per_r = 0;
		for (int c = 0; c < r; c++) //c: parameter bock col
		{
			if (!block_pointer_array[r][c])
				continue;

			HessianBlock const* blk = block_pointer_array[r][c];

			for (int i = 0; i < 12; i++) //row
			{
				//idx of the first non-zero element of the current row
				int idx = nz_count + (blks_num_per_row-1) * 12 * i + ((i*(i + 1)) / 2);

				for (int j = 0; j < 12; j++)//col
				{
					int idx_cur = idx + blk_idx_per_r * 12+j; //idx of the current non-zero element

					crs->colInd[idx_cur] = c * 12 + j;
					crs->vals[idx_cur] = blk->at(i, j);
				}
			}

			blk_idx_per_r++;
		}

		//diag block
		HessianBlock const* blk = block_pointer_array[r][r];
		assert(blk->col == r && blk->row == r);
		for (int i = 0; i < 12; i++) //row
		{
			//idx of the first non-zero element of the current row
			int idx = nz_count + (blks_num_per_row-1) * 12 * i + ((i*(i + 1)) / 2);
			//idx of the first element of current row on the diag block
			int idx_d = idx + (blks_num_per_row-1)*12;

			for (int j = 0; j <= i; j++) //col
			{
				int idx_cur = idx_d + j;
				crs->colInd[idx_cur] = r * 12 + j; //r==c
				crs->vals[idx_cur] = blk->at(i, j);
			}

			crs->rowPtr[r * 12 + i] = idx; //start of the current row
		}

		nz_count += blks_num_per_row * 144 - 66;
	}

	crs->rowPtr[para_blk_num*12] = nz_count;

	return crs;
}