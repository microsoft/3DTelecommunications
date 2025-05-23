// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef __TRACK_EXTRACTION_BASIC_H__
#define __TRACK_EXTRACTION_BASIC_H__
#include "UtilMatrix.h"
#include <vector>

struct PairwiseMatchIdxSet
{
public:
	PairwiseMatchIdxSet(int frm_i1=-1, int frm_i2=-1)
		: frm_idx1(frm_i1),
		  frm_idx2(frm_i2)
	{;}

public:
	int frm_idx1;
	int frm_idx2;
	std::vector<IntPair> match;
};

struct XGlobalLabel
{
public:
	XGlobalLabel(int frm=-1, int item=-1)
			: frm_idx(frm),
			  item_idx(item)
	{;}
public:
	int frm_idx;
	int item_idx;
};

//========================== track extractions =====================
void track_extraction( std::vector< PairwiseMatchIdxSet > const&all_matches,
					   std::vector< int > const&item_nums_at_each_frame,
					   std::vector< std::vector<XGlobalLabel> > &item_tracks,
					   std::vector< std::vector<int> > &track_label_on_items,
					   int thres_item_num_in_track);

//first find the connected graph including the reference frame, and then only find tracks 
// for the frames inside this graph
void track_extraction2( std::vector< PairwiseMatchIdxSet > const&all_matches,
					    std::vector< int > const&item_nums_at_each_frame,
					    std::vector< std::vector<XGlobalLabel> > &item_tracks,
					    std::vector< std::vector<int> > &track_label_on_items,
					    int thres_item_num_in_track,
					    int ref_frame = 0);


#endif