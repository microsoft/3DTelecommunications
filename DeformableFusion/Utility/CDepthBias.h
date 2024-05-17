//===============================================
//			CDepthBias.h
//			Mingsong Dou (doums@cs.unc.edu)
//			Sept., 2013
//===============================================
#ifndef __CDEPTHBIAS_H__
#define __CDEPTHBIAS_H__
#include <vector>
#include <stdio.h>

class CDepthBias
{
public:
	CDepthBias() {};
	~CDepthBias()
	{
		this->freeData();
	};

public:
inline double correct_depth_bias(int row, int col, double depth, bool bUseGlobal = false) const
	{
		if( bUseGlobal )
		 return depth*depth*a[0] + depth*a[1] + a[2];

		int rowBlkIdx = row / block_size;
		int colBlkIdx = col / block_size;
		if( rowBlkIdx < block_rows && colBlkIdx < block_cols )
		{
			double *b = depth_bias[rowBlkIdx][colBlkIdx];
			if( (b[0] == 0 && b[1] == 0 && b[2] == 0) ||
				b[2] > 15 )
				return depth*depth*a[0] + depth*a[1] + a[2];
			else
				return depth*depth*b[0] + depth*b[1] + b[2];
		}
		else
			return 0.0;		
	}

public:
	bool load_from_txt(char const* file_name)
	{
		FILE *fp = NULL;
		fopen_s(&fp, file_name, "r");
		if(!fp)
		{
			printf("Cannot open the file <%s> for reading DepthBais.\n", file_name);
			return false;
		}

		fscanf_s(fp, "#Depth Bias Correction File, v1.0\n");
		fscanf_s(fp, "Blocks<%d, %d>; Block Size<%d>\n", &this->block_rows, &this->block_cols, &this->block_size);
		fscanf_s(fp, "Overall Bias<%lf %lf %lf>\n\n", &a[0], &a[1], &a[2]);

		this->freeData();
		depth_bias.resize(this->block_rows);
		for(int i=0; i<this->block_rows; i++)
		{
			depth_bias[i].resize(this->block_cols);
			for(int j=0; j<this->block_cols; j++)
			{
				double *b = new double[3];
				fscanf_s(fp, "%lf %lf %lf\n", &b[0], &b[1], &b[2]);
				depth_bias[i][j] = b;
			}
		}

		fclose(fp);

		return true;
	}

private:
	int block_size;
	int block_cols;
	int block_rows;
	std::vector< std::vector< double*> > depth_bias;
	double a[3]; //global parameter

private:
	void freeData()
	{
		for(int i=0; i<this->depth_bias.size(); i++)
		{
			for(int j=0; j<this->depth_bias[i].size(); j++)
			{
				if( depth_bias[i][j] != NULL )
					delete [] depth_bias[i][j];
			}
		}
		this->depth_bias.clear();
	}
};



#endif