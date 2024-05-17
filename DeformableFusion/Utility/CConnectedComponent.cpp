#include "stdafx.h"
#include "CConnectedComponent.h"
#include "UtilMatrix.h"
#include "utility.h"

void CComponent::get_color_code(double val, int color[])
{
	int low_bound = 80;
	int up_bound = 500;

	val = MIN(val, 500.0);
	val = MAX(val, 80);

	val = (val-80)/(500-80); //val in [0, 1]

	int r = (int) (val*255.0);
	int b = (int)(255 - r);
	int g = (val < 0.5) ? val*2.0*255 : (255- val*2.0*255);
	color[0] = r;
	color[1] = g;
	color[2] = b;
}

#ifndef P_LABEL
#define P_LABEL
#define P_Label(x,y)   (p_label[(y)*(width)+(x)])
#endif

void CConnectedComponent::FindDConnectedComps(cv::Mat& depthMap, cv::Mat& mask, cv::Mat& labelImg, double d, int area_thres)
{
	freeComponents();

	if( depthMap.empty() || labelImg.empty() )
	{
		printf("Error: input valid!\n");
		return;
	}

	int height = depthMap.rows;
	int width = depthMap.cols;

	bool mask_own_data = false;
	if( mask.empty() )
	{
		mask_own_data = true;
		mask = cv::Mat(height, width, CV_8UC1, 255);		
	}

	if( mask.rows != height || mask.cols != width ||
		labelImg.rows != height || labelImg.cols != width)
	{
		printf("Error: the size input arrays do not match with each other!\n");
		return;
	}

	int *p_label = new int[width*height];
	int *p_x = new int[width*height];
	int *p_y = new int[width*height];
	memset(p_label,0,width*height*sizeof(int));

	int cur_pos=0;//start from 0
	int tail_pos=0;//point to the empty cell
	int cur_x, cur_y;
	double cur_depth;
	int left_pos, right_pos, up_pos, bottom_pos;
	int xdir[8]={-1,0,0,1,1,1,-1,-1};
	int ydir[8]={0,-1,1,0,1,-1,-1,1};
	int label_count=0;

	for(int y=0; y<height; y++)//y
	{
		for(int x=0; x<width; x++)//x
		{
			if( p_label[y*width+x]!=0 || mask.at<uchar>(y,x) == 0 || depthMap.at<double>(y, x) <= 0.0)
				continue;
			label_count++;	//begin a new component
			if(label_count >= 255)
				LOGGER()->warning("CConnectedComponent::FindDConnectedComps","label count is greater than or equal to 255!");

			P_Label(x,y) = label_count;
			cur_pos = 0;
			tail_pos = 0; 
			p_x[tail_pos] = x;
			p_y[tail_pos] = y;
			tail_pos++;
			left_pos = x; right_pos = x;
			up_pos = y; bottom_pos = y;

			while(cur_pos<tail_pos)
			{
				cur_x = p_x[cur_pos];
				cur_y = p_y[cur_pos];
				cur_depth = depthMap.at<double>(cur_y, cur_x);;
				cur_pos++;
				for(int m=0; m<8; m++)
				{
					int x_m = cur_x+xdir[m];
					int y_m = cur_y+ydir[m];
					if( y_m>=0 && y_m<height &&
						x_m>=0 && x_m<width &&
						P_Label(x_m,y_m)==0 &&
						mask.at<uchar>(y_m, x_m)!=0 &&
						depthMap.at<double>(y_m, x_m)>0.0 &&
						abs(depthMap.at<double>(y_m, x_m) - cur_depth)<=d )
					{
						p_x[tail_pos] = x_m;
						p_y[tail_pos] = y_m;
						tail_pos++;
						P_Label(x_m, y_m) = label_count;

						if(xdir[m]==1 && cur_x+1 > right_pos )
							right_pos = cur_x+1;
						if(xdir[m]==-1 && cur_x-1 < left_pos )
							left_pos = cur_x-1;
						if(ydir[m]==1 && cur_y+1 > bottom_pos)
							bottom_pos = cur_y+1;
						if(ydir[m]==-1 && cur_y-1 < up_pos)
							up_pos = cur_y -1;							
					}
				}
			}

			if(cur_pos<area_thres)
			{
				label_count--;
				for(int i=0; i<tail_pos; i++)//clear the small component from mask
				{
					mask.at<uchar>(p_y[i], p_x[i]) = 0;
					P_Label(p_x[i], p_y[i]) = 0;
				}
			}
			else
			{
				CComponent *comp = new CComponent();
				comp->rect = cv::Rect(left_pos,up_pos,right_pos-left_pos+1,bottom_pos-up_pos+1);
				comp->label = label_count;
				double depth_total = 0.0;
				long x_total = 0;
				long y_total = 0;
				for(int i=0; i<tail_pos; i++)
				{
					depth_total += depthMap.at<double>(p_y[i], p_x[i]);
					x_total += p_x[i];
					y_total += p_y[i];
				}
				comp->depth = depth_total/tail_pos;	
				comp->x_centroid = double(x_total)/tail_pos;
				comp->y_centroid = double(y_total)/tail_pos;
				comp->pixelCount = tail_pos;
				this->m_ccomps->push_back(comp);
			}

		}
	}

	//copy the content of p_label
	for(int i=0; i<height; i++)
	{
		for(int j=0; j<width; j++)
		{
			labelImg.at<uchar>(i, j) = P_Label(j, i);
		}
	}

	if (mask_own_data)
		mask.release();

	delete [] p_label;
	delete [] p_x;
	delete [] p_y;
}


void CConnectedComponent::FindDConnectedComps( cv::Mat const& depthMap, cv::Mat const& mask_in,
											   cv::Mat& labelImg, vector<CComponent> &m_ccomp,
											   double d, int area_thres)
{
	m_ccomp.clear();

	if( depthMap.empty() )
	{
		printf("Error: input valid!\n");
		return;
	}

	int height = depthMap.rows;
	int width = depthMap.cols;

	cv::Mat mask = (cv::Mat) mask_in;
	bool mask_own_data = false;
	if( mask.empty())
	{
		mask_own_data = true;
		mask = cv::Mat(height, width, CV_8UC1, 255);
	}

	if( mask.rows != height || mask.cols != width ||
		!labelImg.empty() && labelImg.rows != height ||
		!labelImg.empty() && labelImg.cols != width)
	{
		printf("Error: the size input arrays do not match with each other!\n");
		return;
	}

	int *p_label = new int[width*height];
	int *p_x = new int[width*height];
	int *p_y = new int[width*height];
	memset(p_label,0,width*height*sizeof(int));

	int cur_pos=0;//start from 0
	int tail_pos=0;//point to the empty cell
	int cur_x, cur_y;
	double cur_depth;
	int left_pos, right_pos, up_pos, bottom_pos;
	int xdir[8]={-1,0,0,1,1,1,-1,-1};
	int ydir[8]={0,-1,1,0,1,-1,-1,1};
	int label_count=0;

	for(int y=0; y<height; y++)//y
	{
		for(int x=0; x<width; x++)//x
		{
			if( p_label[y*width+x]!=0 || mask.at<uchar>(y,x) == 0 || depthMap.at<double>(y, x) <= 0.0)
				continue;
			label_count++;	//begin a new component
			if(label_count >= 255)
				LOGGER()->warning("CConnectedComponent::FindDConnectedComps","label count is greater than or equal to 255!");

			P_Label(x,y) = label_count;
			cur_pos = 0;
			tail_pos = 0; 
			p_x[tail_pos] = x;
			p_y[tail_pos] = y;
			tail_pos++;
			left_pos = x; right_pos = x;
			up_pos = y; bottom_pos = y;

			while(cur_pos<tail_pos)
			{
				cur_x = p_x[cur_pos];
				cur_y = p_y[cur_pos];
				cur_depth = depthMap.at<double>(cur_y, cur_x);
				cur_pos++;
				for(int m=0; m<4; m++)
				{
					int x_m = cur_x+xdir[m];
					int y_m = cur_y+ydir[m];
					if( y_m>=0 && y_m<height &&
						x_m>=0 && x_m<width &&
						P_Label(x_m,y_m)==0 &&
						mask.at<uchar>(y_m, x_m)!=0 &&
						depthMap.at<double>(y_m, x_m)>0.0 &&
						abs(depthMap.at<double>(y_m, x_m) - cur_depth)<=d )
					{
						p_x[tail_pos] = x_m;
						p_y[tail_pos] = y_m;
						tail_pos++;
						P_Label(x_m, y_m) = label_count;

						if(xdir[m]==1 && cur_x+1 > right_pos )
							right_pos = cur_x+1;
						if(xdir[m]==-1 && cur_x-1 < left_pos )
							left_pos = cur_x-1;
						if(ydir[m]==1 && cur_y+1 > bottom_pos)
							bottom_pos = cur_y+1;
						if(ydir[m]==-1 && cur_y-1 < up_pos)
							up_pos = cur_y -1;							
					}
				}
			}

			if(cur_pos<area_thres)
			{
				label_count--;
				for(int i=0; i<tail_pos; i++)//clear the small component from mask
				{
					mask.at<uchar>(p_y[i], p_x[i]) = 0;
					P_Label(p_x[i], p_y[i]) = 0;
				}
			}
			else
			{
				CComponent comp;
				comp.rect = cv::Rect(left_pos,up_pos,right_pos-left_pos+1,bottom_pos-up_pos+1);
				comp.label = label_count;
				double depth_total = 0.0;
				long x_total = 0;
				long y_total = 0;
				for(int i=0; i<tail_pos; i++)
				{
					depth_total += depthMap.at<double>(p_y[i], p_x[i]);
					x_total += p_x[i];
					y_total += p_y[i];
				}
				comp.depth = depth_total/tail_pos;	
				comp.x_centroid = double(x_total)/tail_pos;
				comp.y_centroid = double(y_total)/tail_pos;
				comp.pixelCount = tail_pos;
				m_ccomp.push_back(comp);
			}

		}
	}

	//copy the content of p_label
	if( !labelImg.empty() )
	{
		for(int i=0; i<height; i++)
		{
			for(int j=0; j<width; j++)
			{
				labelImg.at<uchar>(i, j) = P_Label(j, i);
			}
		}
	}
	if (mask_own_data)
		mask.release();
	
	delete [] p_label;
	delete [] p_x;
	delete [] p_y;
}


void CConnectedComponent::FindConnectedComps(cv::Mat& mask, cv::Mat& labelImg, vector<CComponent> &m_ccomp, int area_thres)
{
	m_ccomp.clear();

	if (mask.empty())
	{
		printf("Error<FindConnectedComps>: input mask is valid!\n");
		return;
	}

	int height = mask.rows;
	int width = mask.cols;

	if (!labelImg.empty() && labelImg.rows != height ||
		!labelImg.empty() && labelImg.cols != width)
	{
		printf("Error<FindConnectedComps>: the size of input arrays do not match!\n");
		return;
	}

	int *p_label = new int[width*height];
	int *p_x = new int[width*height];
	int *p_y = new int[width*height];
	memset(p_label, 0, width*height*sizeof(int));

	int cur_pos = 0;//start from 0
	int tail_pos = 0;//point to the empty cell
	int cur_x, cur_y;
	double cur_depth;
	int left_pos, right_pos, up_pos, bottom_pos;
	int xdir[8] = { -1, 0, 0, 1, 1, 1, -1, -1 };
	int ydir[8] = { 0, -1, 1, 0, 1, -1, -1, 1 };
	int label_count = 0;

	for (int y = 0; y<height; y++)//y
	{
		for (int x = 0; x<width; x++)//x
		{
			//if visisted continue
			if (p_label[y*width + x] != 0 || mask.at<uchar>(y, x) == 0)
				continue;

			label_count++;	//begin a new component
			if (label_count >= 255)
				LOGGER()->warning(" CConnectedComponent::FindConnectedComps", "label count is greater than or equal to 255!");

			P_Label(x, y) = label_count;
			cur_pos = 0;
			tail_pos = 0;
			p_x[tail_pos] = x;
			p_y[tail_pos] = y;
			tail_pos++;
			left_pos = x; right_pos = x;
			up_pos = y; bottom_pos = y;

			while (cur_pos<tail_pos)
			{
				cur_x = p_x[cur_pos];
				cur_y = p_y[cur_pos];
				cur_pos++;
				for (int m = 0; m<8; m++)
				{
					int x_m = cur_x + xdir[m];
					int y_m = cur_y + ydir[m];
					if (y_m >= 0 && y_m<height &&
						x_m >= 0 && x_m<width &&
						P_Label(x_m, y_m) == 0 &&
						mask.at<uchar>(y_m, x_m) != 0)
					{
						p_x[tail_pos] = x_m;
						p_y[tail_pos] = y_m;
						tail_pos++;
						P_Label(x_m, y_m) = label_count;

						if (xdir[m] == 1 && cur_x + 1 > right_pos)
							right_pos = cur_x + 1;
						if (xdir[m] == -1 && cur_x - 1 < left_pos)
							left_pos = cur_x - 1;
						if (ydir[m] == 1 && cur_y + 1 > bottom_pos)
							bottom_pos = cur_y + 1;
						if (ydir[m] == -1 && cur_y - 1 < up_pos)
							up_pos = cur_y - 1;
					}
				}
			}

			if (cur_pos<area_thres)
			{
				label_count--;
				for (int i = 0; i<tail_pos; i++)//clear the small component from mask
				{
					mask.at<uchar>(p_y[i], p_x[i]) = 0;
					P_Label(p_x[i], p_y[i]) = 0;
				}
			}
			else
			{
				CComponent comp;
				comp.rect = cv::Rect(left_pos, up_pos, right_pos - left_pos + 1, bottom_pos - up_pos + 1);
				comp.label = label_count;
				long x_total = 0;
				long y_total = 0;
				for (int i = 0; i<tail_pos; i++)
				{
					x_total += p_x[i];
					y_total += p_y[i];
				}
				comp.depth = -1.0;
				comp.x_centroid = double(x_total) / tail_pos;
				comp.y_centroid = double(y_total) / tail_pos;
				comp.pixelCount = tail_pos;
				m_ccomp.push_back(comp);
			}

		}
	}

	//copy the content of p_label
	if (!labelImg.empty())
	{
		for (int i = 0; i<height; i++)
		{
			for (int j = 0; j<width; j++)
			{
				labelImg.at<uchar>(i, j) = P_Label(j, i);
			}
		}
	}

	delete[] p_label;
	delete[] p_x;
	delete[] p_y;
}


void CConnectedComponent::FindCComps(cv::Mat& labelImg, cv::Mat& depthMap)
{
	this->freeComponents();
	int h = labelImg.rows;
	int w = labelImg.cols;
	bool flags[256];
	for(int i=0; i<256; i++)
		flags[i] = false;

	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			int val = labelImg.at<uchar>(i, j);
			flags[val] = true;
		}
	}

	//ignore the label 0
	flags[0] = false;

	for(int c=0; c<256; c++)
	{
		if( !flags[c] )
			continue;

		CComponent *comp = new CComponent();
	
		int min_x = w;
		int min_y = h;
		int max_x = -1;
		int max_y = -1;
		double total_depth = 0;
		double num = 0;
		for(int i=0; i<h; i++)
		{
			for(int j=0; j<w; j++)
			{
				int val = labelImg.at<uchar>(i, j);
				if(val != c)
					continue;
				total_depth += depthMap.at<double>(i, j);;
				num ++;
				if( j < min_x )
					min_x = j;
				if( j > max_x )
					max_x = j;
				if( i < min_y )
					min_y = i;
				if( i > max_y )
					max_y = i;
			}
		}

		comp->rect = cv::Rect(min_x, min_y, max_x-min_x+1, max_y-min_y+1);
		comp->label = c;
		comp->depth = total_depth/num;	
		m_ccomps->push_back(comp);
	}
}

void CConnectedComponent::Cvt2ColorLabelImage(cv::Mat& labelImg, cv::Mat& color_labelImg)
{
	if( labelImg.empty() || color_labelImg.empty() )
	{
		printf("Error: one of the input image is NULL!\n");
		return;
	}

	if( labelImg.rows != color_labelImg.rows ||
		labelImg.cols != color_labelImg.cols ||
		color_labelImg.channels() != 3 ||
		labelImg.channels() != 1
	  )
	{
		printf("Error: the input format is wrong!\n");
		return;
	}

	color_labelImg = 0;

	for(unsigned int c=0; c<m_ccomps->size(); c++)
	{
		cv::Rect r = (*m_ccomps)[c]->rect;
		int label = (*m_ccomps)[c]->label;
		double depth = (*m_ccomps)[c]->depth;
		int color[3];
		CComponent::get_color_code(depth, color);
		for(int i=r.y; i<=r.y+r.height; i++)
		{
			for(int j=r.x; j<=r.x+r.width; j++)
			{
				if( labelImg.at<uchar>(i, j) != label)
					continue;
				cv::Vec3b dst = color_labelImg.at<cv::Vec3b>(i,j);
				dst[0] = color[0];
				dst[1] = color[1];
				dst[2] = color[2];
			}
		}
	}
}

void CConnectedComponent::DrawSilhouetteOnImage(cv::Mat& labelImg, cv::Mat& img)
{
	if( labelImg.empty() || img.empty())
	{
		printf("Error: one of the input image is NULL!\n");
		return;
	}

	if( labelImg.rows != img.rows ||
		labelImg.cols != img.cols ||
		img.channels() != 3 ||
		labelImg.channels() != 1
	  )
	{
		printf("Error: the input format is wrong!\n");
		return;
	}

	int height = labelImg.rows;
	int width = labelImg.cols;

	for(unsigned int c=0; c<m_ccomps->size(); c++)
	{
		cv::Rect r = (*m_ccomps)[c]->rect;
		int label = (*m_ccomps)[c]->label;
		double depth = (*m_ccomps)[c]->depth;
		int color[3];
		CComponent::get_color_code(depth, color);
		for(int i=r.y; i<=r.y+r.height; i++)
		{
			for(int j=r.x; j<=r.x+r.width; j++)
			{
				if( labelImg.at<uchar>(i, j) != label )
					continue;
				if( i!=height-1 && labelImg.at<uchar>(i+1, j) != label ||
					j!=width-1  && labelImg.at<uchar>(i, j+1) != label ||
					i!=0        && labelImg.at<uchar>(i-1, j) != label ||
					j!=0        && labelImg.at<uchar>(i, j-1) != label ||
					i!=height-1 && j!=width-1 && labelImg.at<uchar>(i+1, j+1) != label ||
					i!=0 && j!=0 && labelImg.at<uchar>(i-1, j-1) != label ||
					i!=height-1 && j!=0 && labelImg.at<uchar>(i+1, j-1) != label ||
					i!=0 && j!=width-1 && labelImg.at<uchar>(i-1, j+1) != label
					)
				{
					img.at<cv::Vec3b>(i, j)[0] = color[0];
					img.at<cv::Vec3b>(i, j)[1] = color[1];
					img.at<cv::Vec3b>(i, j)[2] = color[2];				
				}
			}
		}
	}
}

void CConnectedComponent::imFillHole(cv::Mat& bImg, vector<cv::Rect> rects)
{
	int i;
	int r, c;
	uchar* src, *dst;

	if(isZero(bImg))
		return;

	int h = bImg.rows;
	int w = bImg.cols;
	cv::Mat fm = cv::Mat(bImg.rows, bImg.cols, CV_8UC1, 255); //maker image
	cv::Mat fm_b = cv::Mat(bImg.rows, bImg.cols, CV_8UC1); //backup of the maker image;
	cv::Mat em = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));

	if(rects.size() == 0)
	{
		for(i=0; i<fm.cols; i++)
		{
			src = (uchar*)bImg.ptr<uchar>() + i;
			dst = (uchar*)fm.ptr<uchar>() + i;
			dst[0] = src[0];
		}
		for(i=0; i<fm.rows; i++)
		{
			src = (uchar*)bImg.ptr<uchar>(bImg.rows - 1) + i;
			dst = (uchar*)fm.ptr<uchar>(fm.rows - 1) + i;
			dst[0] = src[0];
		}
		for(i=0; i<fm.cols; i++)
		{
			src = (uchar*)bImg.ptr<uchar>(i);
			dst = (uchar*)fm.ptr<uchar>(i);
			dst[0] = src[0];
		}
		for(i=0; i<fm.cols; i++)
		{
			src = (uchar*)bImg.ptr<uchar>(i) + bImg.cols -1;
			dst = (uchar*)fm.ptr<uchar>(i) + fm.cols -1;
			dst[0] = src[0];
		}
	}
	else
	{
		fm = cv::Mat::zeros(fm.size(), fm.type());
		
		for(unsigned i=0;i<rects.size();i++)
		{
			for(r=rects[i].y; r<=MIN(rects[i].y+rects[i].height, h-1); r++)
			{
				for(c=rects[i].x; c<=MIN(rects[i].x+rects[i].width, w-1); c++)
				{
					src = fm.ptr<uchar>(r) + c;
					src[0] = 255;				
				}
			}
		}

	}

	do{
		fm.copyTo(fm_b);
		cv::erode(fm, fm, em);
		cv::max(fm, bImg, fm);		
	}while( !isSame(fm, fm_b) );

	fm.copyTo(bImg);
	if(bImg.empty())
	{
		//throw gcnew System::Exception(L"This should never happen!");
		printf("This should never happen!");
	}
	fm.release();
	fm_b.release();
	em.release();
}

//check if image a equals to image b (gray image)
bool CConnectedComponent::isSame(cv::Mat& a, cv::Mat& b)
{
	int i,j;
	uchar* src, *dst;
	assert(a.rows==b.rows && a.step==b.step);
	for(i=0; i<a.rows; i++)
	{
		src = (uchar*) a.ptr<uchar>(i);
		dst = (uchar*) b.ptr<uchar>(i);
		for(j=0; j<a.cols; j++)
		{
			if( src[j] != dst[j])
				return 0;
		}
	}
	return 1;
}
bool CConnectedComponent::isZero(cv::Mat& a)
{
	int i, j;
	uchar* src;
	for(i=0; i<a.rows; i++)
	{
		src = (uchar*) a.ptr<uchar>(i);
		for(j=0; j<a.cols; j++)
		{
			if( src[j] != 0)
				return 0;
		}
	}
	return 1;
}
