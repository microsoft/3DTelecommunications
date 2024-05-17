//===============================================
//			UtilMatrix.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __MYMATRIX_H__
#define __MYMATRIX_H__
#include "opencv2\opencv.hpp"
#include "opencv2\calib3d.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include "CDepthBias.h"

using namespace std;

const double EPS = 1.0e-6;

#define cvMatMulTransL( src1, src2, dst )  cvGEMM( src1, src2, 1, NULL, 0, dst, CV_GEMM_A_T )
#define cvMatMulTransR( src1, src2, dst )  cvGEMM( src1, src2, 1, NULL, 0, dst, CV_GEMM_B_T )
#define cvMatMulTransLR( src1, src2, dst )  cvGEMM( src1, src2, 1, NULL, 0, dst, CV_GEMM_A_T + CV_GEMM_B_T )

#ifndef _MAX_MIN_ROUND_IMAGEELE_MATELE_
#define _MAX_MIN_ROUND_IMAGEELE_MATELE_

#define ROUND(x)  (((x)>0)?((int)((x)+0.5f)):((int)((x)-0.5f))) //note: (int)(-1.8) = -1
#define RANDOM   (fabs(((double)rand())/RAND_MAX)) //generate a rand number lying inside [0, 1]
#define SIGN(x) (((x) > 0) - ((x) < 0)) //return 1 if x>0; -1 if x<0; 0 if x=0

#endif

struct IntPair
{
public:
	IntPair(int i1=-1, int i2=-1)
		: fst(i1),
		  scd(i2)
	{;}

public:
	int fst;
	int scd;
};

typedef pair<double,int> DoubleIndexPair;
inline bool less_comparator_DoubleIndexPair( const DoubleIndexPair& l, //or DoubleIndexPair const& l
											 const DoubleIndexPair& r)
{ 
	return l.first < r.first; 
}

inline bool greater_comparator_DoubleIndexPair( const DoubleIndexPair& l, //or DoubleIndexPair const& l
												const DoubleIndexPair& r)
{ 
	return l.first > r.first; 
}

typedef pair<int, double> IndexDoublePair;
inline bool less_comparator_IndexDoublePair( const IndexDoublePair& l, //or DoubleIndexPair const& l
											 const IndexDoublePair& r)
{ 
	return l.second < r.second; 
}

inline bool greater_comparator_IndexDoublePair( const IndexDoublePair& l, //or DoubleIndexPair const& l
												const IndexDoublePair& r)
{ 
	return l.second > r.second; 
}

inline int find_in_IndexDoublePair_list( vector< IndexDoublePair > const& list, int key)
{
	for(int i=0; i<list.size(); i++)
	{
		if( list[i].first == key )
			return i;
	}
	return -1;
}
inline int find_maxVal_in_IndexDoublePair_list(vector< IndexDoublePair > const& list)
{
	if( list.size() == 0)
		return -1;

	int idx_max = 0;
	double val_max = list[0].second;
	for(int i=1; i<list.size(); i++)
	{
		if( list[i].second > val_max )
		{
			val_max = list[i].second;
			idx_max = i;
		}
	}

	return idx_max;
}

double *ParseStringToDoubleArray(char* str, int &len);
int* ParseStringToIntArray(char* str, int &len);

inline void releaseCvMats(std::vector<cv::Mat> &mats)
{
	for(int i=0; i<mats.size(); i++)
		if (!mats[i].empty())
			mats[i].release();
	mats.clear();
}


inline void fillMatrix(cv::Mat& mat, double val)
{
	if( mat.empty() )
		return;
	for(int i=0; i<mat.rows; i++)
		for(int j=0; j<mat.cols; j++)
			mat.at<double>(i, j) = val;
}

void saveMatrixAscii(const char* filename, cv::Mat const& mat);
void printMat(cv::Mat const& mat, const char* title="", int order = 0);//order: 0--direct print, 1--transpose and print
void printMatFloat(cv::Mat const& mat, const char* title = "", int order = 0); //order: 0--direct print, 1--transpose and print

void saveMatrix2IMG(const char* filename, cv::Mat const& mat);

int saveMatrixBIN(const char* filename, cv::Mat& mat);
cv::Mat loadMatrixBIN(const char* filename, bool *bFileFound=NULL);
cv::Mat loadMatrixAscii(const char* filename, int width, int height);
cv::Mat loadMatrixAscii(const char* filename);

template<class T>
bool saveVectorList(char const*filename, std::vector< std::vector<T> > const&vec_list);
template<class T>
bool saveVectorList(char const*filename, std::vector< std::vector<T>* > const&vec_list);
void loadVectorList( char const*filename, std::vector< std::vector<int> > &vec_list);
void saveVector( char const*filename, std::vector<int> const&vec);
void loadVector( char const*filename, std::vector<int> &vec);

template<class T>
bool saveStdVectorASCII(char const*filename, std::vector<T> const&vec);
template<class T>
bool loadStdVectorASCII(char const*filename, std::vector<T> &vec);

bool saveIntPairVectorASCII(char const* filename, std::vector<pair<int, int>> const& pairs);
bool loadIntPairVectorASCII(char const* filename, std::vector<pair<int, int>> &pairs);


/* the depth value of cv::Mat is in centimeter,
 * the depth of each pixel in PNG file should be 16U and in milimeter
 * the bias on the depth value will be corrected z = a*z+b;
 */
cv::Mat loadDepthMatFromPNG( const char* filename, bool bUseMostSignificantBits = false,
							double bias_a = 1.0, double bias_b = 0.0);
cv::Mat loadDepthMatFromPNG( cv::Mat const& img, bool bUseMostSignificantBits = false,
							double bias_a = 1.0, double bias_b = 0.0);
cv::Mat loadDepthMatFromPNG( const char* filename, bool bUseMostSignificantBits,
							CDepthBias const*depth_bias);
cv::Mat loadDepthMatFromPNG( cv::Mat& img, bool bUseMostSignificantBits,
							CDepthBias const*depth_bias);
cv::Mat loadDepthImageFromPNG(const char* filename, bool bUseMostSignificantBits = false);
void saveDepthImageToPNG(cv::Mat const& img, const char* filename, bool bUseMostSignificantBits = false);
void applyDepthMatBias(cv::Mat& depthMat, double bias_a = 1.0, double bias_b = 0.0);
void applyDepthMatBias(cv::Mat& depthMat, CDepthBias const*depth_bias, bool bUseGlobalPara = false);

cv::Mat DepthMat2DepthImg(cv::Mat const& mat, bool bUseMostSignificantBits);
bool saveDepthMatToPNG(const char* filename, cv::Mat& mat, bool bUseMostSignificantBits = false);

void saveMatrixListAscii(const char* filename, std::vector<cv::Mat&> mat_list);
int saveMatrixListBIN(const char* filename, std::vector<cv::Mat&> mat_list);
int loadMatrixListBIN(const char* filename, std::vector<cv::Mat&> *mat_list);

int saveCvMatBIN(const char* filename, cv::Mat& iplImg);
cv::Mat loadCvMatBIN(const char* filename);
int loadCvMatBIN(const char* filename, cv::Mat& iplImg);


//pick a cv::Mat element
double pickAMatElement(cv::Mat const& mat, int x, int y);
double pickAMatElement(cv::Mat const& mat, double x, double y);
double pickADepthElement(cv::Mat const& depthMat, double x, double y);

//color pixel operations
double* pickColorPixelsInRegularGrid(cv::Mat const& img, double x, double y, int window_r);
double* pickColorPixelsInRegularGrid(cv::Mat const& img, int x, int y, int window_r);
void pickColorPixelsInRegularGrid(cv::Mat const& img, double x, double y, int window_r, double *pixels);
void pickColorPixelsInRegularGrid(cv::Mat const& img, int x, int y, int window_r, double *pixels);

template<class T>
void pickAColorPixel(cv::Mat const& img, int x, int y, T pixels[3]);
template<class T>
void pickAColorPixel(cv::Mat const& img, double x, double y, T pixel[3]);


//gray (UInt16) pixel operations
double* pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, double x, double y, int window_r);
double* pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, int x, int y, int window_r);
void pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, double x, double y, int window_r, double *pixels);
void pickGrayPixelsInRegularGridUInt16(cv::Mat const& img, int x, int y, int window_r, double *pixels);
void pickAGrayPixelUInt16(cv::Mat const& img, int x, int y, double *p_pixel);
void pickAGrayPixelUInt16(cv::Mat const& img, double x, double y, double *p_pixel);

//gray(Int16) pixel operations
short pickAGrayPixelInt16(cv::Mat const& img, int x, int y);
double pickAGrayPixelInt16(cv::Mat const& img, double x, double y);

//gray(Uchar) pixel operations
unsigned char pickAGrayPixelUChar(cv::Mat const& img, int x, int y);
double pickAGrayPixelUChar(cv::Mat const& img, double x, double y);

void copyMatrix( cv::Mat& mat_dest, int srow_dest, int scol_dest, cv::Mat& mat_sour,
				int srow_sour, int scol_sour, int height, int width);
void scalarMatrixMultiply(cv::Mat& mat,double scalar);
/*mat = mat*multiply + addition*/
void scalarMatrixOper(cv::Mat& mat, double multiply, double addition, cv::Mat const& mask=cv::Mat());
void subMatrixVec(cv::Mat& A, cv::Mat& v, int dim);//A = A - repmat(v,?);
void normalizeEachRow(cv::Mat& mat);

void freeMatrixArray(vector<cv::Mat &>mats, int num);

//resize and then put the source image to the designated place on the destine image
bool copyCvMat( cv::Mat& img_dst, cv::Mat const& mat_src,
				   int x, int y, int width, int height );

cv::Mat pad_alpha_for_color_CvMat(cv::Mat const& img);

// save data into a file with 'height' rows and 'width' columes
template<class T> 
void save2DArrayAscii(const char* filename, T* data, int height, int width);

template<class T> 
int save2DArrayBIN(const char* filename, T* data, int height, int width);

template<class T>
T* load2DArrayBIN(const char* filename, int *height, int *width);

//file format: {data[0] data[1] data[2] ...}
template<class T> 
bool saveMemoryBlockASCII(const char* filename, T* data, int len);
//the memory for data must be pre-allocated, len is the size of memory; 
//return the number of elements read in
template<class T> 
int loadMemoryBlockASCII(const char* filename, T* data, int len);

//file format: {len data[0] data[1] ...}
template<class T> 
int saveMemoryBlockBIN(const char* filename, T* data, int len);
template<class T>
T* loadMemoryBlockBIN(const char* filename, int *len);


/******************************************************************************
 *
 * Template function implementation
 *
 ******************************************************************************/
template<class T> 
void save2DArrayAscii(const char* filename, T* data, int height, int width)
{
	std::ofstream matFile(filename);
	if(matFile.fail())
	{
		cout<<"error write matrix file"<<endl;
		return;
	}

	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			matFile.width(25);
			matFile.precision(15);
			matFile<<data[i*width+j];
		}
		matFile<<endl;
	}
	matFile.close();
}

template<class T> 
int save2DArrayBIN(const char* filename, T* data, int height, int width)
{
	int numwritten = 0;
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing File\n");
		return -1;
	}
	
	numwritten = (int) fwrite(&height, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	numwritten = (int) fwrite(&width, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	numwritten = (int) fwrite(data, sizeof(T), height*width, file);
	if( numwritten != height*width )
	{
		printf("Error when writing data\n");
		return -1;
	}

	fclose(file);
	return 1;
}

template<class T>
T* load2DArrayBIN(const char* filename, int *height, int *width)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File\n");
		return NULL;
	}

	numread = (long) fread(height, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		return NULL;
	}

	numread = (long) fread(width, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		return NULL;
	}

	T* data = new T[(*height)*(*width)];
	
	numread = (long) fread( data, sizeof(T), (*height)*(*width), file);
	if( numread != (*height)*(*width) )
	{
		printf("Error when Reading File\n");
		delete [] data;
		return NULL;
	}

	fclose(file);

	return data;
}

template<class T> 
bool saveMemoryBlockASCII(const char* filename, T* data_block, int size)
{
	ofstream data_file(filename);

	for(int i=0; i<size; i++)
		data_file<<data_block[i]<<endl;
	data_file.close();

	return true;
}

template<class T> 
int loadMemoryBlockASCII(const char* filename, T* data, int len)
{
	ifstream data_file;
	data_file.open(filename);
	if( !data_file.is_open() )
	{
		printf("Error: cannot open the file %s for reading!\n", filename);
		return 0;
	}

	int i = 0;
	while( !data_file.eof() && i < len )
	{
		data_file>>data[i];
		i++;
	}

	return i;
}

template<class T> 
int saveMemoryBlockBIN(const char* filename, T* data, int len)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	int numwritten = (int) fwrite(&len, sizeof(int), 1, file);
	if( numwritten != 1 )
	{
		printf("Error when Writing File\n");
		return -1;
	}

	numwritten = (int) fwrite(data, sizeof(T), len, file);
	if( numwritten != len )
	{
		printf("Error when writing data\n");
		return -1;
	}

	fclose(file);
	return 1;
}

template<class T>
T* loadMemoryBlockBIN(const char* filename, int *len)
{
	FILE *file = NULL;
	long numread=0;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when Reading File\n");
		return NULL;
	}

	numread = (long) fread(len, sizeof(int), 1, file);
	if( numread != 1 )
	{
		printf("Error when Reading File\n");
		return NULL;
	}

	T* data = new T[*len];
	
	numread = (long) fread( data, sizeof(T), *len, file);
	if( numread != *len )
	{
		printf("Error when Reading File\n");
		delete [] data;
		return NULL;
	}

	fclose(file);

	return data;
}

#include "UtilMatrix.hpp"

#endif