#include "stdafx.h"
#include "opencv2\opencv.hpp"
#include "opencv2\highgui.hpp"
#include <iostream>
#include <fstream>
#include <vector>
//#include "f2c.h"
//#include "clapack.h"
#include "UtilMatrix.h"
#include "VecOperation.h"
using namespace std;

bool saveIntPairVectorASCII(char const* filename, std::vector<pair<int, int>> const& pairs)
{
	FILE *file = NULL;
	if ((fopen_s(&file, filename, "w")) != 0)
	{
		printf("Error<saveIntPairVectorASCII> when writing file<%s>\n", filename);
		return false;
	}

	fprintf(file, "PairNum<%zd>\n", pairs.size());
	for (int i = 0; i < pairs.size(); i++)
	{
		fprintf(file, "%d  %d\n", pairs[i].first, pairs[i].second);
	}

	fclose(file);
	return true;
}

bool loadIntPairVectorASCII(char const* filename, std::vector<pair<int, int>> &pairs)
{
	FILE *file = NULL;
	if ((fopen_s(&file, filename, "r")) != 0)
	{
		printf("Error<loadIntPairVectorASCII> when reading file<%s>\n", filename);
		return false;
	}

	int len = 0;
	fscanf(file, "PairNum<%d>\n", &len);
	pairs.resize(len);
	for (int i = 0; i < pairs.size(); i++)
	{
		fscanf(file, "%d  %d\n", &(pairs[i].first), &(pairs[i].second));
	}

	fclose(file);
	return true;
}

void loadVectorList( char const*filename, std::vector< std::vector<int> > &vec_list)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "r")) != 0 )
	{
		printf("Error<saveVectorList> when reading file<%s>\n", filename);
		return;
	}

	int line_num = 0;
	fscanf(file, "vector list: %d lines\n", &line_num);
	vec_list.clear();
	vec_list.resize(line_num);
	for(int i=0; i<line_num; i++)
	{
		int idx = 0;
		int itemNum = 0;
		fscanf(file, "Line %d<%d>:", &idx, &itemNum );
		for(int j=0; j<itemNum; j++)
		{
			int val = 0;
			fscanf(file, " %d", &val);
			vec_list[i].push_back(val);
		}
		fscanf(file, "\n");
	}

	fclose(file);
}

void saveVector( char const*filename, std::vector<int> const&vec)
{
	vector< vector<int> > vec_list;
	vec_list.push_back(vec);
	saveVectorList(filename, vec_list);
}
void loadVector( char const*filename, std::vector<int> &vec)
{
	vector< vector<int> > vec_list;
	loadVectorList(filename, vec_list);
	vec = vec_list[0];
}


void saveMatrixAscii(const char* filename, cv::Mat const& mat)
{
	if( mat.empty())
	{	
		printf("Warning: mat is NULL, file<%s> not created!\n", filename);
		return;
	}

	int step = mat.cols;
	double* data= (double*)mat.ptr<double>();


	std::ofstream matFile(filename);
	if(matFile.fail())
	{
		cout<<"Error<v>: write matrix file:"<<filename<<endl;
		return;
	}

	for(int i=0; i < mat.rows; i++, data += step)
	{
		for(int j=0;j < mat.cols; j++)
		{
			matFile.width(25);
			matFile.precision(15);
			matFile<<data[j];
		}
		matFile<<endl;
	}
	matFile.close();
}



void saveMatrixListAscii(const char* filename, std::vector<cv::Mat>& mat_list)
{
	std::ofstream matFile(filename);
	if(matFile.fail())
	{
		cout<<"error write matrix file"<<endl;
		return;
	}

	cv::Mat* mat = NULL;
	int step;
	double *data;
	for(int c=0; c<mat_list.size(); c++)
	{
		step = mat_list[c].cols;
		data=mat_list[c].ptr<double>();

		for(int i=0; i < mat_list[c].rows; i++, data += step)
		{
			for(int j=0;j < mat_list[c].cols; j++)
			{
				matFile.width(25);
				matFile.precision(15);
				matFile<<data[j];
			}
			matFile<<endl;
		}
		matFile<<endl;
	}
	matFile.close();
}

void printMat(cv::Mat const& mat, const char* title, int order)
{
	if( title!=NULL)
		printf("====== %s ======\n", title);
	if( order == 0 )
	{
		for( int i=0; i<mat.rows; i++ )
		{
			for(int j=0; j<mat.cols; j++)
			{
				printf("%f  ", mat.at<double>(i, j));
			}
			printf("\n");
		}
	}
	else
	{
		for(int j=0; j<mat.cols; j++)
		{
			for( int i=0; i<mat.rows; i++ )
			{
				printf("%f  ", mat.at<double>(i, j));
			}
			printf("\n");
		}
	}
}
void printMatFloat(cv::Mat const& mat, const char* title, int order)
{
	if( title!=NULL)
		printf("====== %s ======\n", title);
	if( order == 0 )
	{
		for( int i=0; i<mat.rows; i++ )
		{
			for(int j=0; j<mat.cols; j++)
			{
				printf("%f  ", mat.at<float>(i, j));
			}
			printf("\n");
		}
	}
	else
	{
		for(int j=0; j<mat.cols; j++)
		{
			for( int i=0; i<mat.rows; i++ )
			{
				printf("%f  ", mat.at<float>(i, j));
			}
			printf("\n");
		}
	}
}

void saveMatrix2IMG(const char *filename, cv::Mat const& mat)
{
	cv::Mat img = cv::Mat(mat.rows,mat.cols, CV_8UC1);
	cv::normalize(mat, img, 255, 0, cv::NORM_MINMAX);

	cv::imwrite(filename, img);

	img.release();
}

int saveMatrixListBIN(const char* filename, std::vector<cv::Mat>& mat_list)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	int mat_num = mat_list.size();
	int numwritten = (int) fwrite(&mat_num, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	cv::Mat mat = cv::Mat();
	for(int i=0; i<mat_num; i++)
	{
		mat = mat_list[i];
		numwritten = (int) fwrite(&mat.rows, sizeof(int), 1, file);
		if( numwritten != 1 )
		{
			printf("Error when Writing File\n");
			return -1;
		}

		numwritten = (int) fwrite(&mat.cols, sizeof(int), 1, file);
		if( numwritten != 1 )
		{
			printf("Error when Writing File\n");
			return -1;
		}

		numwritten = (int) fwrite(mat.ptr<double>(), sizeof(double), mat.rows * mat.cols, file);
		if( numwritten != mat.rows * mat.cols )
		{
			printf("Error when writing vertex data\n");
			return -1;
		}
	}

	fclose(file);
	return 1;
}

int loadMatrixListBIN(const char* filename, std::vector<cv::Mat>& mat_list)
{
	mat_list.clear();
	
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File\n");
		return -1;
	}

	int mat_num=0; 
	numread = (long) fread(&mat_num, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		return -1;
	}

	for(int i=0; i<mat_num; i++)
	{
		int rows, cols;
		numread = (long) fread(&rows, sizeof(int), 1, file);
		if( numread != 1 )
		{
			printf("Error when Reading File\n");
			return -1;
		}
		numread = (long) fread(&cols, sizeof(int), 1, file);
		if( numread != 1 )
		{
			printf("Error when Reading File\n");
			return -1;
		}

		cv::Mat mat = cv::Mat(rows, cols, CV_64F);
		
		numread = (long) fread( mat.ptr<double>(), sizeof(double), mat.rows * mat.cols, file);
		if( numread != mat.rows * mat.cols )
		{
			printf("Error when Reading File\n");
			mat.release();
			return -1;
		}
		mat_list.push_back(mat);
	}
	fclose(file);

	return 1;
}

int saveMatrixBIN(const char* filename, cv::Mat& mat)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	if( mat.empty() )
	{
		fclose(file);
		return -1;
	}

	int numwritten = (int) fwrite(&mat.rows, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		fclose(file);
		return -1;
	}

	numwritten = (int) fwrite(&mat.cols, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		fclose(file);
		return -1;
	}

	numwritten = (int) fwrite(mat.ptr<double>(), sizeof(double), mat.rows * mat.cols, file);
	if( numwritten != mat.rows * mat.cols )
	{
		printf("Error when writing data\n");
		fclose(file);
		return -1;
	}

	fclose(file);
	return 1;
}

double *ParseStringToDoubleArray(char* str, int &len)
{
	if(str == NULL)
	{
		len = 0;
		return NULL;
	}

	vector<double> double_list;
	char tbuf[1000];
	memset(tbuf, 0, 1000);
	bool empty = true;
	int j=0;
	int i=0;
	while(str[i] != '\0')
	{
		if( str[i] == ' '||
			str[i] == '\t' ||
			str[i] == '\n'
		   )
		{
			if(!empty)
			{
				tbuf[j] = '\0';
				double tmp = atof(tbuf);
				double_list.push_back(tmp);
				empty = true;
				j=0;
			}
			i++;
			continue;
		}

		tbuf[j] = str[i];
		empty = false;
		j++;
		i++;
	}
	if( !empty )
	{
		tbuf[j] = '\0';
		double tmp = atof(tbuf);
		double_list.push_back(tmp);
		empty = true;
		j=0;
	}

	len = double_list.size();
	double* ret = new double[len];
	for(int i=0; i<len; i++)
	{
		ret[i] = double_list[i];
	}

	return ret;
}

int* ParseStringToIntArray(char* str, int &len)
{
	if(str == NULL)
	{
		len = 0;
		return NULL;
	}

	vector<int> int_list;
	char tbuf[1000];
	memset(tbuf, 0, 1000);
	bool empty = true;
	int j=0;
	int i=0;
	while(str[i] != '\0')
	{
		if( str[i] == ' '||
			str[i] == '\t' ||
			str[i] == '\n'
		   )
		{
			if(!empty)
			{
				tbuf[j] = '\0';
				int tmp = atoi(tbuf);
				int_list.push_back(tmp);
				empty = true;
				j=0;
			}
			i++;
			continue;
		}

		tbuf[j] = str[i];
		empty = false;
		j++;
		i++;
	}
	if(!empty)
	{
		tbuf[j] = '\0';
		int tmp = atoi(tbuf);
		int_list.push_back(tmp);
		empty = true;
		j=0;
	}

	len = int_list.size();
	int* ret = new int[len];
	for(int i=0; i<len; i++)
	{
		ret[i] = int_list[i];
	}
	return ret;
}

cv::Mat loadMatrixAscii(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "r")) != 0 )
	{
		printf("Error when Reading File\n");
		return cv::Mat();
	}
	
	//======decide the width====
	int width;
	char line[200000];
	if( fgets(line, 200000, file) == NULL)
	{
		printf("Error when read a line\n");
		return cv::Mat();
	}	
	double *data = ParseStringToDoubleArray(line, width);
	if( data == NULL)
	{
		printf("Error when parsing a line.\n");
		return cv::Mat();
	}
	delete [] data;

	//=======decide the height======
	int height=0;
	rewind(file);
	while( !feof(file) )
	{
		if( fgets(line, 200000, file) != NULL)
			height++;
		else
			break;
	}
	fclose(file);

	return loadMatrixAscii(filename, width, height);
		
}

cv::Mat loadMatrixAscii(const char* filename, int width, int height)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "r")) != 0 )
	{
		printf("Error when Open File!\n");
		return cv::Mat();
	}

	cv::Mat mat = cv::Mat(height, width, CV_64F);
	double data;
	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			fscanf(file, "%lf", &data);
			if( feof(file)!=0 && (i!=height-1 && j!=width-1))
			{
				mat.release();
				printf("Error<loadMatrixAscii>: cols and rows do not match!\n");
				fclose(file);
				return cv::Mat();
			}

			mat.at<double>(i, j) = data;
		}
	}
	fclose(file);
	return mat;
}

cv::Mat loadMatrixBIN(const char* filename, bool *bFileFound)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		if( bFileFound != NULL)
			*bFileFound = false;
		return cv::Mat();
	}

	if( bFileFound != NULL)
		*bFileFound = true;
	int rows, cols;
	numread = (long) fread(&rows, sizeof(int), 1, file);
	if( numread != 1 )
	{
		fclose(file);
		return cv::Mat();
	}
	numread = (long) fread(&cols, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		fclose(file);
		return cv::Mat();
	}

	cv::Mat mat = cv::Mat(rows, cols, CV_64F);
	
	numread = (long) fread( mat.ptr<double>(), sizeof(double), mat.rows * mat.cols, file);
	if( numread != mat.rows * mat.cols )
	{
		printf("Error when Reading File\n");
		mat.release();
		fclose(file);
		return cv::Mat();
	}

	fclose(file);

	return mat;
}

int saveCvMatBIN(const char *filename, cv::Mat& iplImg)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing cv::Mat File\n");
		return -1;
	}

	try
	{
		int d = iplImg.depth();
		int c = iplImg.channels();
		fwrite(&(iplImg.cols), sizeof(int), 1, file);
		fwrite(&(iplImg.rows), sizeof(int), 1, file);
		fwrite(&(d), sizeof(int), 1, file);
		fwrite(&(c), sizeof(int), 1, file);

		fwrite(iplImg.ptr<uchar>(), 1, iplImg.cols*iplImg.rows, file);		
	}
	catch(...)//System::FieldAccessException ^ e)
	{
		fclose(file);		
		return -1;
	}
	fclose(file);
	return 1;
}

int loadCvMatBIN(const char* filename, cv::Mat& iplImg)
{

	FILE *file = NULL;
	cv::Mat *ret = NULL;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File\n");
		return -1;
	}

	try
	{
		int width, height, depth, nChannels;
		fread(&width, sizeof(int), 1, file);
		fread(&height, sizeof(int), 1, file);
		fread(&depth, sizeof(int), 1, file);
		fread(&nChannels, sizeof(int), 1, file);

		iplImg = cv::Mat(height, width, CV_MAKETYPE(depth, nChannels));

		fread(iplImg.ptr<uchar>(), 1, width*height, file);
	}
	catch(...)//System::FieldAccessException ^ e)
	{
		fclose(file);
		return -1;
	}
	fclose(file);
	return 1;

}


cv::Mat loadDepthMatFromPNG( cv::Mat& img, bool bUseMostSignificantBits,
							CDepthBias const* depth_bias )
{
	if( img.empty() || img.depth() != CV_16UC1 )
	{
		printf("Error<loadDepthMatFromPNG>: file not found or with wrong format(16U only)!\n");
		return cv::Mat();
	}

	int w = img.cols;
	int h = img.rows;
	cv::Mat mat = cv::Mat(h, w, CV_64F);

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned short ori_depth = img.at<unsigned short>(i, j);
			if( bUseMostSignificantBits )
				mat.at<double>(i, j) = (ori_depth>>3)/10.0;
			else
				mat.at<double>(i, j) = ori_depth/10.0;

			if( mat.at<double>(i, j) > 0.001 && depth_bias != NULL)
			{
				mat.at<double>(i, j) = depth_bias->correct_depth_bias(i, j, mat.at<double>(i, j));
			}
		}
	}
	return mat;
}


cv::Mat loadDepthImageFromPNG(const char* filename, bool bUseMostSignificantBits)
{
	cv::Mat img = cv::imread(filename, cv::IMREAD_UNCHANGED);
	if (img.empty())
	{
		printf("Error<loadDepthImageFromPNG>: cannot open file %s!\n", filename);
		return cv::Mat();
	}

	if (bUseMostSignificantBits)
	{
		int w = img.cols;
		int h = img.rows;
		for (int i = 0; i < h; i++)
		{
			for (int j = 0; j < w; j++)
			{
				img.at<unsigned short>(i, j) >>= 3;
			}
		}
	}
	return img;
}

void saveDepthImageToPNG(cv::Mat const& depthImg, const char* filename, bool bUseMostSignificantBits)
{
	if (bUseMostSignificantBits)
	{
		cv::Mat img = depthImg.clone();
		for (int i = 0; i < depthImg.rows; i++)
		for (int j = 0; j < depthImg.cols; j++)
			img.at<unsigned short>(i, j) <<= 3;
		cv::imwrite(filename, img);
		img.release();
	}
	else
	{
		cv::imwrite(filename, depthImg);
	}
}

cv::Mat loadDepthMatFromPNG( const char* name, bool bUseMostSignificantBits,
							double bias_a, double bias_b )
{
	cv::Mat img = cv::imread(name, cv::IMREAD_UNCHANGED);
	if( img.empty() || img.depth() != CV_16UC1 )
	{
		printf("Error<loadDepthMatFromPNG>: file<%s> not found or with wrong format(16U only)!\n", name);
		return cv::Mat();
	}

	cv::Mat ret = loadDepthMatFromPNG(img, bUseMostSignificantBits, bias_a, bias_b);

	img.release();
	return ret;
}

cv::Mat loadDepthMatFromPNG( cv::Mat const& img, bool bUseMostSignificantBits,
							double bias_a, double bias_b )
{
	if( img.empty() || img.depth() != CV_16UC1)
	{
		printf("Error<loadDepthMatFromPNG>: file not found or with wrong format(16U only)!\n");
		return cv::Mat();
	}

	if( bias_a == 0.0 && bias_b == 0.0)
	{
		bias_a = 1.0; 
		bias_b = 0.0;
		printf("Warning: linear depthbias are set to <1.0, 0.0>");
	}

	int w = img.cols;
	int h = img.rows;
	cv::Mat mat = cv::Mat(h, w, CV_64F);

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned short ori_depth = img.at<unsigned short>(i, j);
			if( bUseMostSignificantBits )
				mat.at<double>(i, j) = (ori_depth>>3)/10.0;
			else
				mat.at<double>(i, j) = ori_depth/10.0;

			if( mat.at<double>(i, j) > 0.001 )
			{
				mat.at<double>(i, j) *= bias_a;
				mat.at<double>(i, j) += bias_b;
			}
		}
	}
	return mat;
}

cv::Mat loadDepthMatFromPNG( const char* name, bool bUseMostSignificantBits,
							CDepthBias const* depth_bias )
{
	cv::Mat img = cv::imread(name, cv::IMREAD_UNCHANGED);
	
	cv::Mat ret = loadDepthMatFromPNG(img, bUseMostSignificantBits, depth_bias);

	img.release();
	return ret;
}


void applyDepthMatBias(cv::Mat& depthMat, double bias_a, double bias_b)
{
	for(int i=0; i<depthMat.rows; i++)
	{
		for(int j=0; j<depthMat.cols; j++)
		{
			if( depthMat.at<double>(i, j) > 0.001 )
			{
				depthMat.at<double>(i, j) *= bias_a;
				depthMat.at<double>(i, j) += bias_b;
			}
		}
	}
}

void applyDepthMatBias(cv::Mat& depthMat, CDepthBias const*depth_bias, bool bUseGlobalPara)
{
	for(int i=0; i<depthMat.rows; i++)
	{
		for(int j=0; j<depthMat.cols; j++)
		{			
			if( depthMat.at<double>(i, j) > 0.001 && depth_bias != NULL)
			{
				depthMat.at<double>(i, j) = depth_bias->correct_depth_bias(i, j, depthMat.at<double>(i, j), bUseGlobalPara);
			}
		}
	}
}

cv::Mat DepthMat2DepthImg(cv::Mat const& mat, bool bUseMostSignificantBits)
{
	if (mat.empty())
	{
		printf("Warning<DepthMat2DepthImg>: mat is NULL!\n");
		return cv::Mat();
	}
	int w = mat.cols;
	int h = mat.rows;

	cv::Mat img = cv::Mat(h, w, CV_16UC1);
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			unsigned short ori_depth = unsigned short(MAX(0.0, mat.at<double>(i, j)*10.0));
			if (bUseMostSignificantBits)
				img.at<unsigned short>(i, j) = ori_depth << 3;
			else
				img.at<unsigned short>(i, j) = ori_depth;
		}
	}
	return img;
}

bool saveDepthMatToPNG(const char* name, cv::Mat& mat, bool bUseMostSignificantBits)
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

	cv::Mat img = DepthMat2DepthImg(mat, bUseMostSignificantBits);
	imwrite(name, img);

	img.release();
	return true;
}

double pickAMatElement(cv::Mat const& mat, int x, int y)
{
	if( x<0 || x >= mat.cols || y<0 || y>=mat.rows)
		return 0;
	else
		return mat.at<double>(y, x);;
}

double pickAMatElement(cv::Mat const& mat, double x, double y)
{
	int x0 = (int)( floor(x) );
	int y0 = (int)( floor(y) );
	int x1 = (int)( ceil(x) );
	int y1 = (int)( ceil(y) );

	double a = x-x0;
	double b = y-y0;

	double P00 = pickAMatElement(mat, x0, y0);
	double P01 = pickAMatElement(mat, x1, y0);
	double P10 = pickAMatElement(mat, x0, y1);
	double P11 = pickAMatElement(mat, x1, y1);

	return (P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b;
}

double pickADepthElement(cv::Mat const& depthMat, double x, double y)
{
	int x0 = (int)(floor(x));
	int y0 = (int)(floor(y));
	int x1 = (int)(ceil(x));
	int y1 = (int)(ceil(y));

	double a = x - x0;
	double b = y - y0;

	double P00 = pickAMatElement(depthMat, x0, y0);
	double P01 = pickAMatElement(depthMat, x1, y0);
	double P10 = pickAMatElement(depthMat, x0, y1);
	double P11 = pickAMatElement(depthMat, x1, y1);

	if (P00 <= 0.0 || P01 <= 0.0 ||
		P10 <= 0.0 || P11 <= 0.0)
		return 0.0;
	else
		return (P00*(1 - a) + P01*a)*(1 - b) + (P10*(1 - a) + P11*a)*b;
}


double* pickColorPixelsInRegularGrid(cv::Mat const& img, double x, double y, int window_r)
{
	int cols = window_r*2+1;
	double *pixels = new double[cols*cols*3];
	pickColorPixelsInRegularGrid(img, x, y, window_r, pixels);
	return pixels;
}

double* pickColorPixelsInRegularGrid(cv::Mat const& img, int x, int y, int window_r)
{
	int cols = window_r*2+1;
	double *pixels = new double[cols*cols*3];
	pickColorPixelsInRegularGrid(img, x, y, window_r, pixels);
	return pixels;
}

void pickColorPixelsInRegularGrid(cv::Mat const& img, double x, double y, int window_radius, double* pixels)
{
	for(int i=-window_radius; i<=window_radius; i++) //y
	{
		for(int j=-window_radius; j<=window_radius; j++)//x
		{
			pickAColorPixel(img, x+j, y+i, pixels);
			pixels += 3;
		}
	}
}

void pickColorPixelsInRegularGrid(cv::Mat const& img, int x, int y, int window_radius, double* pixels)
{
	for(int i=-window_radius; i<=window_radius; i++) //y
	{
		for(int j=-window_radius; j<=window_radius; j++)//x
		{
			pickAColorPixel(img, x+j, y+i, pixels);
			pixels += 3;
		}
	}
}

//======= Gray (UInt16) Pixel Operation ==========
double* pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, double x, double y, int window_r)
{
	int cols = window_r*2+1;
	double *pixels = new double[cols*cols];
	pickGrayPixelsInRegularGridUInt16(img, x, y, window_r, pixels);
	return pixels;
}

double* pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, int x, int y, int window_r)
{
	int cols = window_r*2+1;
	double *pixels = new double[cols*cols];
	pickGrayPixelsInRegularGridUInt16(img, x, y, window_r, pixels);
	return pixels;
}

void pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, double x, double y, int window_radius, double* pixels)
{
	for(int i=-window_radius; i<=window_radius; i++) //y
	{
		for(int j=-window_radius; j<=window_radius; j++)//x
		{
			pickAGrayPixelUInt16(img, x+j, y+i, pixels);
			pixels++;
		}
	}
}

void pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, int x, int y, int window_radius, double* pixels)
{
	for(int i=-window_radius; i<=window_radius; i++) //y
	{
		for(int j=-window_radius; j<=window_radius; j++)//x
		{
			pickAGrayPixelUInt16(img, x+j, y+i, pixels);
			pixels++;
		}
	}
}

void pickAGrayPixelUInt16(cv::Mat const& img, int x, int y, double *p_pixel)
{
	if( x<0 || x >= img.cols || y<0 || y>=img.rows)
	{
		*p_pixel = 0.0;
		return;
	}
	unsigned short* src = (unsigned short*)(img.ptr<unsigned short>(y)) + x;
	*p_pixel = (double)(*src);
}

void pickAGrayPixelUInt16(cv::Mat const& img, double x, double y, double *p_pixel)
{
	int x0 = (int)( floor(x) );
	int y0 = (int)( floor(y) );
	int x1 = (int)( ceil(x) );
	int y1 = (int)( ceil(y) );

	double a = x-x0;
	double b = y-y0;

	double P00;
	double P01;
	double P10;
	double P11;

	pickAGrayPixelUInt16(img, x0, y0, &P00);
	pickAGrayPixelUInt16(img, x1, y0, &P01);
	pickAGrayPixelUInt16(img, x0, y1, &P10);
	pickAGrayPixelUInt16(img, x1, y1, &P11);

	*p_pixel = (P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b;
}

short pickAGrayPixelInt16(cv::Mat const& img, int x, int y)
{
	if( x<0 || x >= img.cols || y<0 || y>=img.rows)
	{
		return 0;
	}
	short* src = (short*)(img.ptr<short>(y)) + x;
	return *src;
}

double pickAGrayPixelInt16(cv::Mat const& img, double x, double y)
{
	int x0 = (int)( floor(x) );
	int y0 = (int)( floor(y) );
	int x1 = (int)( ceil(x) );
	int y1 = (int)( ceil(y) );

	double a = x-x0;
	double b = y-y0;

	short P00 = pickAGrayPixelInt16(img, x0, y0);
	short P01 = pickAGrayPixelInt16(img, x1, y0);
	short P10 = pickAGrayPixelInt16(img, x0, y1);
	short P11 = pickAGrayPixelInt16(img, x1, y1);

	return (P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b;
}

unsigned char pickAGrayPixelUChar(cv::Mat const& img, int x, int y)
{
	if( x<0 || x >= img.cols || y<0 || y>=img.rows)
	{
		return 0;
	}
	unsigned char* src = (unsigned char*)(img.ptr<unsigned char>(y)) + x;
	return *src;
}

double pickAGrayPixelUChar(cv::Mat const& img, double x, double y)
{
	int x0 = (int)( floor(x) );
	int y0 = (int)( floor(y) );
	int x1 = (int)( ceil(x) );
	int y1 = (int)( ceil(y) );

	double a = x-x0;
	double b = y-y0;

	unsigned char P00 = pickAGrayPixelUChar(img, x0, y0);
	unsigned char P01 = pickAGrayPixelUChar(img, x1, y0);
	unsigned char P10 = pickAGrayPixelUChar(img, x0, y1);
	unsigned char P11 = pickAGrayPixelUChar(img, x1, y1);

	return (P00*(1-a) + P01*a)*(1-b) + (P10*(1-a) + P11*a)*b;
}

void copyMatrix( cv::Mat& mat_dest, int srow_dest, int scol_dest,
				cv::Mat& mat_sour, int srow_sour, int scol_sour, int height, int width)
{
	assert( (srow_dest + height) <= mat_dest.rows && (scol_dest + width) <= mat_dest.cols && 
		(srow_sour + height) <= mat_sour.rows && (scol_sour + width) <= mat_sour.cols);
	int i,j;
	int s1 = mat_sour.cols;
	int s2 = mat_dest.cols;
	double* src = mat_sour.ptr<double>() + srow_sour * s1;
	double* dst = mat_dest.ptr<double>() + srow_dest * s2;

	for( i = 0; i < height; i++,src += s1, dst += s2 )
	{
		for( j = 0; j < width; j++)
		{
			dst[j+scol_dest] = src[j+scol_sour];
		}
	}
}

void scalarMatrixMultiply(cv::Mat& mat,double scalar)
{
	double* src = mat.ptr<double>();
	int s = mat.cols;
	for(int i=0;i<mat.rows;i++, src += s)
	{
		for(int j=0;j<mat.cols;j++)
		{
			src[j] = scalar * src[j];
		}
	}
}

void scalarMatrixOper(cv::Mat& mat, double multiply, double addition, cv::Mat const& mask)
{
	double* src = mat.ptr<double>();
	int s = mat.cols;
	for (int i = 0; i<mat.rows; i++, src += s)
	{
		for (int j = 0; j<mat.cols; j++)
		{
			if (mask.empty() || mask.at<uchar>(i, j) > 0)
				src[j] = multiply * src[j] + addition;
		}
	}
}

void subMatrixVec(cv::Mat& A, cv::Mat& v, int dim) //dim=1: along the colum; dim=2: along the row
{
	double* src = A.ptr<double>();
	int s = A.cols;
	double scalar;
	double* dst = v.ptr<double>();
	if (dim==1)
	{
		for(int i=0;i<A.rows;i++, src += s)
		{
			scalar = v.at<double>(i,0);
			for(int j=0;j<A.cols;j++)
			{
				src[j] -= scalar;
			}
		}
	}
	else if(dim==2)
	{
		for(int i=0;i<A.rows;i++, src += s)
		{
			for(int j=0;j<A.cols;j++)
			{
				src[j] -= dst[j];
			}
		}
	}
}

void normalizeEachRow(cv::Mat& mat)
{
	int h = mat.rows;
	int w = mat.cols;
	for(int i=0; i<h; i++)
	{
		double *line = &(mat.at<double>(i, 0));
		VecOperation<double>::Normalize(line, w);
	}
}

void freeMatrixArray(vector<cv::Mat>& mats)
{
	if(!mats.empty())
	{
		for(int i=0; i<mats.size(); i++)
		{
			if(!mats[i].empty())
			{
				mats[i].release();
			}
		}
		mats.clear();
	}
}

cv::Mat pad_alpha_for_color_CvMat(cv::Mat const& img)
{
	if (img.empty())
		return cv::Mat();

	if (img.channels() == 4)
	{
		cv::Mat ret = img.clone();
		return ret;
	}

	int w = img.cols;
	int h = img.rows;
	cv::Mat ret = cv::Mat(h, w, CV_MAKETYPE(img.depth(), 4));

	for (int i = 0; i < h; i++)
		for (int j = 0; j < w; j++)
		{		
			ret.at<cv::Vec<uchar,4>>(i, j)[0] = img.at<cv::Vec3b>(i, j)[0];
			ret.at<cv::Vec<uchar,4>>(i, j)[1] = img.at<cv::Vec3b>(i, j)[1];
			ret.at<cv::Vec<uchar,4>>(i, j)[2] = img.at<cv::Vec3b>(i, j)[2];
			ret.at<cv::Vec<uchar,4>>(i, j)[3] = 0;
		}
	return ret;
}

