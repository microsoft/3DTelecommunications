#include "stdafx.h"
#include "track_extraction_basic.h"
using namespace std;

void track_extraction( vector< PairwiseMatchIdxSet > const&all_matches,
					   vector< int > const&item_nums_at_each_frame,
					   vector< vector<XGlobalLabel> > &item_tracks,
					   vector< vector<int> > &track_label_on_items,
					   int thres_item_num_in_track)
{
	int frame_node_num = item_nums_at_each_frame.size();

	//build the frame node connectivity dynamic array
	vector< vector<PairwiseMatchIdxSet*> > frame_node_connectivity(frame_node_num);
	for(int i=0; i<all_matches.size(); i++)
	{
		int frame_fst = all_matches[i].frm_idx1;
		int frame_scd = all_matches[i].frm_idx2;

		PairwiseMatchIdxSet *p_one_match = (PairwiseMatchIdxSet *)&(all_matches[i]);
		frame_node_connectivity[frame_fst].push_back(p_one_match);
		frame_node_connectivity[frame_scd].push_back(p_one_match);
	}

	//initialize the visit flag
	vector< vector<int> > item_node_track_label(frame_node_num);
	for(int i=0; i<frame_node_num; i++)
	{
		int item_num = item_nums_at_each_frame[i];
		item_node_track_label[i].resize(item_num, -1);
	}

	//find all the tracks
	int track_count = 0;
	for(int frmIdx=0; frmIdx<frame_node_num; frmIdx++)
	{
		for(int itemIdx=0; itemIdx<item_node_track_label[frmIdx].size(); itemIdx++)
		{
			if( item_node_track_label[frmIdx][itemIdx] != -1)
				continue;

			//begin a new track
			printf("find track %d\r", track_count);
			vector<XGlobalLabel> one_track;
			one_track.push_back(XGlobalLabel(frmIdx, itemIdx));
			item_node_track_label[frmIdx][itemIdx] = track_count;
			int cur_pos = 0;
			while( cur_pos < one_track.size() )
			{
				XGlobalLabel cur_item_node = one_track[cur_pos];
				int cur_frmIdx = cur_item_node.frm_idx;
				int cur_itemIdx = cur_item_node.item_idx;
				for(int i=0; i<frame_node_connectivity[cur_frmIdx].size(); i++)
				{
					PairwiseMatchIdxSet *p_match = frame_node_connectivity[cur_frmIdx][i];
					if( cur_frmIdx == p_match->frm_idx1 )
					{
						for(int j=0; j<p_match->match.size(); j++)
						{
							if( cur_itemIdx == p_match->match[j].fst )
							{
								int matched_frmIdx = p_match->frm_idx2;
								int matched_itemIdx = p_match->match[j].scd;
								if( item_node_track_label[matched_frmIdx][matched_itemIdx] == -1)
								{
									item_node_track_label[matched_frmIdx][matched_itemIdx] = track_count;
									one_track.push_back(XGlobalLabel(matched_frmIdx, matched_itemIdx));
								}
								break;
							}
						}
					}
					else
					{
						for(int j=0; j<p_match->match.size(); j++)
						{
							if( cur_itemIdx == p_match->match[j].scd )
							{
								int matched_frmIdx = p_match->frm_idx1;
								int matched_itemIdx = p_match->match[j].fst;
								if( item_node_track_label[matched_frmIdx][matched_itemIdx] == -1)
								{
									item_node_track_label[matched_frmIdx][matched_itemIdx] = track_count;
									one_track.push_back(XGlobalLabel(matched_frmIdx, matched_itemIdx));
								}
								break;
							}
						}
					}
				}
				cur_pos++;
			}

			//push back the track just found if there is enough plane in it, otherwise reset item_node_track_label
			if( one_track.size() >= thres_item_num_in_track)
			{
				item_tracks.push_back(one_track);
				track_count++;
			}
			else
			{
				for(int i=0; i<one_track.size(); i++)
				{
					int frm_idx = one_track[i].frm_idx;
					int pln_idx = one_track[i].item_idx;
					item_node_track_label[frm_idx][pln_idx] = -1;
				}
			}
		}
	}
	printf("\n");

	track_label_on_items = item_node_track_label;

	//if( thres_item_num_in_track >= 1 )
	//{
	//	vector<bool> bDeleteFlag;
	//	for(int i=0; i<item_tracks.size(); i++)
	//	{
	//		if( item_tracks[i].size() <= thres_item_num_in_track )
	//			bDeleteFlag.push_back(true);
	//		else
	//			bDeleteFlag.push_back(false);
	//	}

	//	delete_vector_items(item_tracks, bDeleteFlag);
	//}
}

void track_extraction2( vector< PairwiseMatchIdxSet > const&all_matches,
					    vector< int > const&item_nums_at_each_frame,
					    vector< vector<XGlobalLabel> > &item_tracks,
					    vector< vector<int> > &track_label_on_items,
					    int thres_item_num_in_track,
						int ref_frmIdx)
{
	int frame_node_num = item_nums_at_each_frame.size();

	//build the frame node connectivity dynamic array
	vector< vector<PairwiseMatchIdxSet*> > frame_node_connectivity(frame_node_num);
	for(int i=0; i<all_matches.size(); i++)
	{
		int frame_fst = all_matches[i].frm_idx1;
		int frame_scd = all_matches[i].frm_idx2;

		PairwiseMatchIdxSet *p_one_match = (PairwiseMatchIdxSet *)&(all_matches[i]);
		frame_node_connectivity[frame_fst].push_back(p_one_match);
		frame_node_connectivity[frame_scd].push_back(p_one_match);
	}

	//find the connected graph including the reference frame
	vector<bool> graph_node_flag(frame_node_num, false);
	vector<int> connected_graph_node;
	connected_graph_node.push_back(ref_frmIdx);
	graph_node_flag[ref_frmIdx] = true;
	int cur_pos = 0;
	while( cur_pos < connected_graph_node.size() )
	{
		int cur_frmIdx = connected_graph_node[cur_pos];
		for(int i=0; i<frame_node_connectivity[cur_frmIdx].size(); i++)
		{		
			PairwiseMatchIdxSet *p_match = frame_node_connectivity[cur_frmIdx][i];
			if( cur_frmIdx == p_match->frm_idx1 )
			{
				if( !graph_node_flag[p_match->frm_idx2] )
				{
					graph_node_flag[p_match->frm_idx2] = true;
					connected_graph_node.push_back(p_match->frm_idx2);
				}
			}
			else if( cur_frmIdx == p_match->frm_idx2 )
			{
				if( !graph_node_flag[p_match->frm_idx1] )
				{
					graph_node_flag[p_match->frm_idx1] = true;
					connected_graph_node.push_back(p_match->frm_idx1);
				}
			}
		}
		cur_pos++;
	}

	printf("Graph Node:\n");
	for(int i=0; i<connected_graph_node.size(); i++)
		printf("%d, ", connected_graph_node[i]);
	printf("\n\n");

	//initialize the visit flag
	vector< vector<int> > item_node_track_label(frame_node_num);
	for(int i=0; i<frame_node_num; i++)
	{
		int plane_num = item_nums_at_each_frame[i];
		item_node_track_label[i].resize(plane_num, -1);
	}

	//find all the tracks
	int track_count = 0;
	for(int frmIdx=0; frmIdx<frame_node_num; frmIdx++)
	{
		//skip the frame if it is not connected with the reference frame
		if( !graph_node_flag[frmIdx] )	continue;

		for(int itemIdx=0; itemIdx<item_node_track_label[frmIdx].size(); itemIdx++)
		{
			if( item_node_track_label[frmIdx][itemIdx] != -1)
				continue;

			//begin a new track
			printf("find track %d\r", track_count);
			vector<XGlobalLabel> one_track;
			one_track.push_back(XGlobalLabel(frmIdx, itemIdx));
			item_node_track_label[frmIdx][itemIdx] = track_count;
			int cur_pos = 0;
			while( cur_pos < one_track.size() )
			{
				XGlobalLabel cur_item_node = one_track[cur_pos];
				int cur_frmIdx = cur_item_node.frm_idx;
				int cur_itemIdx = cur_item_node.item_idx;
				for(int i=0; i<frame_node_connectivity[cur_frmIdx].size(); i++)
				{
					PairwiseMatchIdxSet *p_match = frame_node_connectivity[cur_frmIdx][i];
					if( cur_frmIdx == p_match->frm_idx1 )
					{
						for(int j=0; j<p_match->match.size(); j++)
						{
							if( cur_itemIdx == p_match->match[j].fst )
							{
								int matched_frmIdx = p_match->frm_idx2;
								int matched_itemIdx = p_match->match[j].scd;
								if( item_node_track_label[matched_frmIdx][matched_itemIdx] == -1)
								{
									item_node_track_label[matched_frmIdx][matched_itemIdx] = track_count;
									one_track.push_back(XGlobalLabel(matched_frmIdx, matched_itemIdx));
								}
								break;
							}
						}
					}
					else
					{
						for(int j=0; j<p_match->match.size(); j++)
						{
							if( cur_itemIdx == p_match->match[j].scd )
							{
								int matched_frmIdx = p_match->frm_idx1;
								int matched_itemIdx = p_match->match[j].fst;
								if( item_node_track_label[matched_frmIdx][matched_itemIdx] == -1)
								{
									item_node_track_label[matched_frmIdx][matched_itemIdx] = track_count;
									one_track.push_back(XGlobalLabel(matched_frmIdx, matched_itemIdx));
								}
								break;
							}
						}
					}
				}
				cur_pos++;
			}

			//push back the track just found if there is enough plane in it, otherwise reset item_node_track_label
			if( one_track.size() >= thres_item_num_in_track)
			{
				item_tracks.push_back(one_track);
				track_count++;
			}
			else
			{
				for(int i=0; i<one_track.size(); i++)
				{
					int frm_idx = one_track[i].frm_idx;
					int pln_idx = one_track[i].item_idx;
					item_node_track_label[frm_idx][pln_idx] = -1;
				}
			}
		}
	}
	printf("\n");

	track_label_on_items = item_node_track_label;

	//if( thres_item_num_in_track >= 1 )
	//{
	//	vector<bool> bDeleteFlag;
	//	for(int i=0; i<item_tracks.size(); i++)
	//	{
	//		if( item_tracks[i].size() <= thres_item_num_in_track )
	//			bDeleteFlag.push_back(true);
	//		else
	//			bDeleteFlag.push_back(false);
	//	}

	//	delete_vector_items(item_tracks, bDeleteFlag);
	//}
}
