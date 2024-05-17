#ifndef __SURFACE_GEODESIC_DISTANCE_H__
#define __SURFACE_GEODESIC_DISTANCE_H__
#include "CSurface.h"
#include <map>
#include <vector>
#include "basic_structure.h"
#include "DeformGraph.h"
using namespace std;

class CGeodesicDistanceOnSurface
{
public:
	CGeodesicDistanceOnSurface()
	{
		this->bAssociatedWithSurface = false;
	}
	CGeodesicDistanceOnSurface(CSurface<float> const& surface)
	{
		this->associate_with_surface(surface);
	}
	~CGeodesicDistanceOnSurface()
	{
		free_geo_dists(this->geo_dists);
		free_geo_dists(this->geo_dists_hash);
		free_geo_dists(this->geo_dists_drt_cnt);
	}

public:
	CGeodesicDistanceOnSurface& operator=(CGeodesicDistanceOnSurface const& rhs)
	{
		free_geo_dists(geo_dists);
		free_geo_dists(geo_dists_hash);
		free_geo_dists(geo_dists_drt_cnt);

		this->ptr_cur_visited = rhs.ptr_cur_visited;
		for(int i=0; i<rhs.geo_dists.size(); i++)
		{
			IndexDistList *list = new IndexDistList;
			*list = *(rhs.geo_dists[i]);
			this->geo_dists.push_back(list);
		}

		for(int i=0; i<rhs.geo_dists_drt_cnt.size(); i++)
		{
			IndexDistList *list = new IndexDistList;
			*list = *(rhs.geo_dists_drt_cnt[i]);
			this->geo_dists_drt_cnt.push_back(list);
		}

		for(int i=0; i<rhs.geo_dists_hash.size(); i++)
		{
			IndexDistHashTable *list = new IndexDistHashTable;
			*list = *(rhs.geo_dists_hash[i]);
			this->geo_dists_hash.push_back(list);
		}

		return (*this);
	}

public:
	bool associate_with_surface( CSurface<float> const& surface);
	bool associate_with_graph(NonrigidMatching::DeformGraph const& graph);
	//end the searching if the visited node has a distance greater than thres_dist
	IndexDistList& calc_dijkstra_dist_for_one_vertex(int idxSrc, double thres_dist);
	//end the searching if the visited node is idxDest
	IndexDistList& calc_dijkstra_dist_for_one_vertex(int idxSrc, int idxDest);

	IndexDistList const& dist_list_for_one_vertex(int vtIdx) const
	{
		return *(this->geo_dists[vtIdx]);
	}

	IndexDistList const& neighbors_for_one_vertex(int vtIdx) const
	{
		return *(this->geo_dists_drt_cnt[vtIdx]);
	}

	IndexDistHashTable const& hash_table_for_one_vertex(int vtIdx) const
	{
		return *(this->geo_dists_hash[vtIdx]);
	}

	double operator()(int vert_idx1, int vert_idx2) const;

//I/O
public:
	bool write_to_file_bin(const char* filename);
	bool load_from_file_bin(const char* filename);

public:
	bool bAssociatedWithSurface;

private:
	vector< IndexDistHashTable* > geo_dists_hash;
	vector< IndexDistList* > geo_dists; //will be sorted based on dist
	vector<int> ptr_cur_visited;
	vector< IndexDistList* > geo_dists_drt_cnt;//list of vertice that are direct connected to current one

private:
	void copy_geo_dists(vector< IndexDistList* > const& geo_dists_src, vector< IndexDistList* > &geo_dists_dst);
	void free_geo_dists(vector< IndexDistList* > &geo_dists);
	void free_geo_dists(vector< IndexDistHashTable* > &geo_dists_hashed);
	//order_idx: the position in the list
	IndexDistList::iterator find_in_list( IndexDistList &list, int idx, int *order_idx = NULL)
	{
		int count = 0;
		IndexDistList::iterator iter;
		for(iter = list.begin(); iter != list.end(); iter++)
		{			
			if( iter->first == idx )
				return iter;
			count++;
		}
		if( order_idx != NULL )
			*order_idx = count;
		return iter;
	}
};



#endif