//===============================================
//			CConnectedComponent.h
//			Mingsong Dou (doums@cs.unc.edu)
//			August, 2010
//===============================================

#ifndef __CCONNECTEDCOMPONENT_H__
#define __CCONNECTEDCOMPONENT_H__
#include "opencv2\opencv.hpp"
#include "opencv2\core.hpp"
#include <vector>
using namespace std;

class CComponent
{
public:
	CComponent()
	{
		label = 0;
		depth = 0.0;
		rect = cv::Rect(0, 0, 0, 0);
		x_centroid = 0.0;
		y_centroid = 0.0;
		pixelCount = 0;
	}
	CComponent(int l, double d, cv::Rect r)
	{
		label = l;
		depth = d;
		rect = r;
		x_centroid = 0.0;
		y_centroid = 0.0;
		pixelCount = 0;
	}
	CComponent(int l, double d, cv::Rect r, double x_c, double y_c, int pCount = 0)
	{
		label = l;
		depth = d;
		rect = r;
		x_centroid = x_c;
		y_centroid = y_c;
		pixelCount = pCount;
	}
public:
	int label;
	double depth;
	double x_centroid;
	double y_centroid;
	cv::Rect rect;
	int pixelCount;

public:
	static void get_color_code(double depth, int color[3]);

};


class FGComponent
{
public:
	FGComponent(int label = -1, double x = 0.0, double y = 0.0, double z = 0.0, int pixelCount = 0)
	{
		this->label = label;
		this->x = x;
		this->y = y;
		this->z = z;
		this->pixelCount = pixelCount;
	}

public:
	int label;
	double x;
	double y;
	double z;
	int pixelCount;

public:
	//First check whether there is a similar object in the list. If it is, merge it with the existing one; 
	//otherwise push it back
	static bool addFGComponentToList(vector<FGComponent> *fgList, FGComponent &fg)
	{
		if( fgList == NULL)
		{
			printf("No input list!\n");
			return false;
		}

		bool bSimilarFGFound = false;
		for(int i=0; i<fgList->size(); i++)
		{
			if( std::abs((*fgList)[i].x - fg.x) < 15.0 &&
				std::abs((*fgList)[i].y - fg.y) < 15.0 &&
				std::abs((*fgList)[i].z - fg.z) < 100.0 )
			{
				(*fgList)[i].x = ( (*fgList)[i].x * (*fgList)[i].pixelCount + fg.x * fg.pixelCount) / double((*fgList)[i].pixelCount + fg.pixelCount);
				(*fgList)[i].y = ( (*fgList)[i].y * (*fgList)[i].pixelCount + fg.y * fg.pixelCount) / double((*fgList)[i].pixelCount + fg.pixelCount);
				(*fgList)[i].z = ( (*fgList)[i].z * (*fgList)[i].pixelCount + fg.z * fg.pixelCount) / double((*fgList)[i].pixelCount + fg.pixelCount);
				(*fgList)[i].pixelCount = (*fgList)[i].pixelCount + fg.pixelCount;

				bSimilarFGFound = true;
				break;
			}
		}

		if( !bSimilarFGFound )
			fgList->push_back(fg);

		return true;
	}

	static bool addFGComponentToList(vector<FGComponent> *fgList, vector<FGComponent> *fgList_new)
	{
		if( fgList_new->empty() )
		{
			printf("No input list!\n");
			return false;
		}

		for(int i=0; i<fgList_new->size(); i++)
		{
			addFGComponentToList(fgList, (*fgList_new)[i]);
		}
		return true;
	}
};

class CConnectedComponent
{
public:
	CConnectedComponent()
	{
		m_ccomps = new vector<CComponent*>();
	};

	~CConnectedComponent()
	{
		freeComponents();
		delete this->m_ccomps;
	}

public:

	/* in--depthMap, mask (mask is used to filter out unwanted areas, these area will have a label 0 in labelImg together with other small noise blobs)
	 * out -- labelImg (and, of course, m_ccomps)
	 */
	void FindDConnectedComps(cv::Mat& depthMap, cv::Mat& mask, cv::Mat& labelImg, double d = 5.0, int area_thres=1000);
	
	/*labelImg -- in
	 *depthMap -- in
	 * out -- m_ccomps
	 */
	void FindCComps(cv::Mat& labelImg, cv::Mat& depthMap);

	void Cvt2ColorLabelImage(cv::Mat& labelImg, cv::Mat& color_labelImg);
	void DrawSilhouetteOnImage(cv::Mat& labelImg, cv::Mat& img);

	static void imFillHole(cv::Mat& bImg, vector<cv::Rect> rects = vector<cv::Rect>());
	static bool isSame(cv::Mat& a, cv::Mat& b);
	static bool isZero(cv::Mat& a);

	/* mask--in, out. the small blobs in mask willbe set to 0; could be NULL, which indicates performing operations on all pixels
	 * labelImg--out. background is 0. foregound starts from 1. Could be NULL.
	 */
	static void FindDConnectedComps( cv::Mat const& depthMap, cv::Mat const& mask,
									 cv::Mat& labelImg, vector<CComponent> &m_ccomp,
									 double d = 5.0, int area_thres=1000);

	/* mask--in, out. the small blobs in mask willbe set to 0; could be NULL, which indicates performing operations on all pixels
	* labelImg--out. background is 0. foregound starts from 1. Could be NULL.
	*/
	static void FindConnectedComps(cv::Mat& mask, cv::Mat& labelImg, vector<CComponent> &m_ccomp, int area_thres = 1000);

public:
	vector<CComponent*> *m_ccomps;

private:
	void freeComponents()
	{
		for(unsigned int i=0; i<m_ccomps->size(); i++)
			delete (*m_ccomps)[i];
		m_ccomps->clear();
	}
};




#endif