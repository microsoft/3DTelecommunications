#include "surface_geodesic_distance.h"
#include "VecOperation.h"
#include <assert.h>

bool CGeodesicDistanceOnSurface::
associate_with_surface( CSurface<float> const& surface)
{
	float *vtData = surface.vtData;
	int vtDim = surface.vtDim;
	int* triangles = surface.triangles;

	free_geo_dists(this->geo_dists);
	free_geo_dists(this->geo_dists_hash);
	this->geo_dists.clear();
	this->geo_dists_hash.clear();
	this->ptr_cur_visited.clear();
	//build the intial distance table
	for(int i=0; i<surface.vtNum; i++)
	{
		IndexDistList *p_list = new IndexDistList();
		geo_dists.push_back(p_list);
		ptr_cur_visited.push_back(0);
		IndexDistHashTable *p_hashTbl = new IndexDistHashTable();
		geo_dists_hash.push_back(p_hashTbl);
	}
	for(int i=0; i<surface.triNum; i++)
	{
		int triIdx1 = triangles[i*3];
		int triIdx2 = triangles[i*3+1];
		int triIdx3 = triangles[i*3+2];

		float *vt1 = &vtData[triIdx1*vtDim];
		float *vt2 = &vtData[triIdx2*vtDim];
		float *vt3 = &vtData[triIdx3*vtDim];

		double dst_12 = VecOperation<float>::Distance(vt1, vt2, 3);
		double dst_13 = VecOperation<float>::Distance(vt1, vt3, 3);
		double dst_23 = VecOperation<float>::Distance(vt2, vt3, 3);

		IndexDistList::iterator iter = find_in_list(*geo_dists[triIdx1], triIdx2);
		if( iter == geo_dists[triIdx1]->end() )
			geo_dists[triIdx1]->push_back(make_pair(triIdx2, dst_12));

		iter = find_in_list(*geo_dists[triIdx1], triIdx3);
		if( iter == geo_dists[triIdx1]->end() )
			geo_dists[triIdx1]->push_back(make_pair(triIdx3, dst_13));
		
		iter = find_in_list(*geo_dists[triIdx2], triIdx1);
		if( iter == geo_dists[triIdx2]->end() )
			geo_dists[triIdx2]->push_back(make_pair(triIdx1, dst_12));
		
		iter = find_in_list(*geo_dists[triIdx2], triIdx3);
		if( iter == geo_dists[triIdx2]->end() )
			geo_dists[triIdx2]->push_back(make_pair(triIdx3, dst_23));
		
		iter = find_in_list(*geo_dists[triIdx3], triIdx1);
		if( iter == geo_dists[triIdx3]->end() )
			geo_dists[triIdx3]->push_back(make_pair(triIdx1, dst_13));
		
		iter = find_in_list(*geo_dists[triIdx3], triIdx2);
		if( iter == geo_dists[triIdx3]->end() )
			geo_dists[triIdx3]->push_back(make_pair(triIdx2, dst_23));
	}

	copy_geo_dists(geo_dists, geo_dists_drt_cnt);

	this->bAssociatedWithSurface = true;
	
	return true;
}

bool CGeodesicDistanceOnSurface::
associate_with_graph(NonrigidMatching::DeformGraph const& graph)
{
	int ndNum = graph.nodes.size();
	free_geo_dists(this->geo_dists);
	free_geo_dists(this->geo_dists_hash);
	this->geo_dists.clear();
	this->geo_dists_hash.clear();
	this->ptr_cur_visited.clear();
	//build the intial distance table
	for (int i = 0; i<ndNum; i++)
	{
		IndexDistList *p_list = new IndexDistList();
		geo_dists.push_back(p_list);
		ptr_cur_visited.push_back(0);
		IndexDistHashTable *p_hashTbl = new IndexDistHashTable();
		geo_dists_hash.push_back(p_hashTbl);
	}

	for (int ndIdx_i = 0; ndIdx_i<ndNum; ndIdx_i++)
	{
		NonrigidMatching::DeformGraphNode const& nd_i = graph.nodes[ndIdx_i];
		for (int j = 0; j < nd_i.neighborIndices.size(); j++)
		{
			int ndIdx_j = nd_i.neighborIndices[j];
			NonrigidMatching::DeformGraphNode const& nd_j = graph.nodes[ndIdx_j];

			double dist_ij = dist_3d(nd_i.g, nd_j.g);

			IndexDistList::iterator iter = find_in_list(*geo_dists[ndIdx_i], ndIdx_j);
			if (iter == geo_dists[ndIdx_i]->end())
				geo_dists[ndIdx_i]->push_back(make_pair(ndIdx_j, dist_ij));

			iter = find_in_list(*geo_dists[ndIdx_j], ndIdx_i);
			if (iter == geo_dists[ndIdx_j]->end())
				geo_dists[ndIdx_j]->push_back(make_pair(ndIdx_i, dist_ij));
		}
	}

	copy_geo_dists(geo_dists, geo_dists_drt_cnt);

	this->bAssociatedWithSurface = true;

	return true;
}


//end the searching if the visited node has a distance greater than thres_dist
IndexDistList& CGeodesicDistanceOnSurface::calc_dijkstra_dist_for_one_vertex(int vtIdxSrc, double thres_dist)
{
	IndexDistList &node_dist_list = *(this->geo_dists[vtIdxSrc]);
	if( node_dist_list.size() == 0)
	{
		return node_dist_list;
	}

	int& ptr_next_visited = this->ptr_cur_visited[vtIdxSrc]; //pick up where it left last time
	if( node_dist_list.size() == ptr_next_visited )
	{

		return node_dist_list;
	}

	//the hash table saves all the visited nodes which are ready to be retrieved
	IndexDistHashTable &node_dist_hash = *(this->geo_dists_hash[vtIdxSrc]);

	//originally node_dist_list is not sorted
	if( ptr_next_visited == 0)
		std::sort(node_dist_list.begin(), node_dist_list.end(), less_comparator_IndexDistPair);

	while(true)
	{
		IndexDistPair &cur_node = node_dist_list[ptr_next_visited];
		int vtIdx_cur = cur_node.first;
		double dist_cur = cur_node.second;

		if( dist_cur > thres_dist )
			break;

		//add or edit the neigbors of the cur_node in the list
		IndexDistList &neighbors_list = *(this->geo_dists_drt_cnt[vtIdx_cur]);
		for(int i=0; i<neighbors_list.size(); i++)
		{
			int vtIdx_neighbor = neighbors_list[i].first;
			double dist_neighbor = neighbors_list[i].second;
			//skip the visited nodes--the nodes in the hash table (vtIdxSrc is not in the hash table, so check it separately)
			if( vtIdx_neighbor == vtIdxSrc )	continue;
			if (node_dist_hash.find(vtIdx_neighbor) != node_dist_hash.end())	continue;
			assert(dist_neighbor >= 0);
			double dist_neighbor_to_src = dist_neighbor + dist_cur;
			IndexDistList::iterator iter = find_in_list(node_dist_list, vtIdx_neighbor);
			if( iter != node_dist_list.end() )
			{
				double dist_old = iter->second;
				if( dist_old > dist_neighbor_to_src )
					iter->second = dist_neighbor_to_src;
			}
			else
			{
				node_dist_list.push_back(make_pair(vtIdx_neighbor, dist_neighbor_to_src));
			}
		}

		//resort the list
		std::sort(node_dist_list.begin()+ptr_next_visited+1, node_dist_list.end(), less_comparator_IndexDistPair);

		//push the current IndexDist Pair to hash table.
		node_dist_hash.insert(make_pair(vtIdx_cur, dist_cur));

		ptr_next_visited++;
		if( ptr_next_visited == node_dist_list.size() )
		{
			break;
		}
	}
	return node_dist_list;
}

//end the searching if the visited node is idxDest
IndexDistList&  CGeodesicDistanceOnSurface::calc_dijkstra_dist_for_one_vertex(int vtIdxSrc, int idxDest)
{
	IndexDistList &node_dist_list = *(this->geo_dists[vtIdxSrc]);
	if( node_dist_list.size() == 0)
	{
		printf("\n---Warning: isolated node found!\n");
		return node_dist_list;
	}

	int& ptr_next_visited = this->ptr_cur_visited[vtIdxSrc]; //pick up where it left last time
	if( node_dist_list.size() == ptr_next_visited )
	{
		printf("\n---Warning: all connected nodes are visited!\n");
		return node_dist_list;
	}

	IndexDistHashTable &node_dist_hash = *(this->geo_dists_hash[vtIdxSrc]);

	//originally node_dist_list is not sorted
	if( ptr_next_visited == 0)
		std::sort(node_dist_list.begin(), node_dist_list.end(), less_comparator_IndexDistPair);

	while(true)
	{
		IndexDistPair &cur_node = node_dist_list[ptr_next_visited];
		int vtIdx_cur = cur_node.first;
		double dist_cur = cur_node.second;

		if( vtIdx_cur == idxDest )
			break;

		//add or edit the neigbors of the cur_node in the list
		IndexDistList &neighbors_list = *(this->geo_dists_drt_cnt[vtIdx_cur]);
		for(int i=0; i<neighbors_list.size(); i++)
		{
			int vtIdx_neighbor = neighbors_list[i].first;
			double dist_neighbor = neighbors_list[i].second;
			//skip the visited nodes--the nodes in the hash table (vtIdxSrc is not in the hash table, so check it separately)
			if( vtIdx_neighbor == vtIdxSrc )	continue;
			if (node_dist_hash.find(vtIdx_neighbor) != node_dist_hash.end())	continue;
			assert(dist_neighbor >= 0);
			double dist_neighbor_to_src = dist_neighbor + dist_cur;
			IndexDistList::iterator iter = find_in_list(node_dist_list, vtIdx_neighbor);
			if( iter != node_dist_list.end() )
			{
				double dist_old = iter->second;
				if( dist_old > dist_neighbor_to_src )
					iter->second = dist_neighbor_to_src;
			}
			else
			{
				node_dist_list.push_back(make_pair(vtIdx_neighbor, dist_neighbor_to_src));
			}
		}

		//resort the list
		std::sort(node_dist_list.begin()+ptr_next_visited+1, node_dist_list.end(), less_comparator_IndexDistPair);
		assert(node_dist_list[ptr_next_visited].first == vtIdx_cur);

		//push the current IndexDist Pair to hash table
		node_dist_hash.insert(make_pair(vtIdx_cur, dist_cur));

		ptr_next_visited++;
		if( ptr_next_visited == node_dist_list.size() )
		{
			printf("\nWarning: all connected nodes are visited\n");
			break;
		}
	}
	return node_dist_list;
}

double CGeodesicDistanceOnSurface::operator()(int vert_idx1, int vert_idx2) const
{
	if( vert_idx1 == vert_idx2 )
		return 0.0;

	IndexDistHashTable::iterator iter = geo_dists_hash[vert_idx1]->find(vert_idx2);
	if( iter != geo_dists_hash[vert_idx1]->end() )
		return iter->second;
	else
	{
		iter = geo_dists_hash[vert_idx2]->find(vert_idx1);
		if(iter != geo_dists_hash[vert_idx2]->end() )
			return iter->second;
		else
			return -1.0;
	}
}

void CGeodesicDistanceOnSurface::copy_geo_dists(vector< IndexDistList* > const& geo_dists_src, vector< IndexDistList* > &geo_dists_dst)
{
	free_geo_dists(geo_dists_dst);
	geo_dists_dst.clear();
	for(int i=0; i<geo_dists_src.size(); i++)
	{
		IndexDistList *list_cpy = new IndexDistList();
		*list_cpy = *(geo_dists_src[i]);
		geo_dists_dst.push_back(list_cpy);
	}
}

void CGeodesicDistanceOnSurface::free_geo_dists( vector< IndexDistList* > &geo_dists )
{
	for(int i=0; i<geo_dists.size(); i++)
	{
		if( geo_dists[i] != NULL )
			delete geo_dists[i];
		geo_dists[i] = NULL;
	}
	geo_dists.clear();
}
void CGeodesicDistanceOnSurface::free_geo_dists( vector< IndexDistHashTable* > &geo_dists )
{
	for(int i=0; i<geo_dists.size(); i++)
	{
		if( geo_dists[i] != NULL )
			delete geo_dists[i];
		geo_dists[i] = NULL;
	}
	geo_dists.clear();
}

bool CGeodesicDistanceOnSurface::write_to_file_bin(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "wb")) != 0 )
	{
		printf("Error when Writing CGeodesicDistanceOnSurface File<%s>\n", filename);
		return false;
	}

	int vtNum = this->geo_dists.size();
	int numwritten = fwrite(&vtNum, sizeof(int), 1, file);

	for(int i=0; i<vtNum; i++)
	{
		IndexDistList* dist_list = geo_dists[i];
		int num = dist_list->size();
		fwrite(&num, sizeof(int), 1, file);
		for(int i=0; i<num; i++)
		{
			fwrite(&((*dist_list)[i].first), sizeof(int), 1, file);
			fwrite(&((*dist_list)[i].second), sizeof(double), 1, file);
		}
	}

	for(int i=0; i<vtNum; i++)
	{
		IndexDistHashTable &dist_hash = *(geo_dists_hash[i]);
		int num = dist_hash.size();
		fwrite(&num, sizeof(int), 1, file);
		for(IndexDistHashTable::iterator iter = dist_hash.begin(); iter != dist_hash.end(); iter++)
		{
			fwrite(&(iter->first), sizeof(int), 1, file);
			fwrite(&(iter->second), sizeof(double), 1, file);
		}
	}

	for(int i=0; i<vtNum; i++)
	{
		IndexDistList* dist_list = geo_dists_drt_cnt[i];
		int num = dist_list->size();
		fwrite(&num, sizeof(int), 1, file);
		for(int i=0; i<num; i++)
		{
			fwrite(&((*dist_list)[i].first), sizeof(int), 1, file);
			fwrite(&((*dist_list)[i].second), sizeof(double), 1, file);
		}
	}

	for(int i=0; i<vtNum; i++)
	{
		fwrite(&(this->ptr_cur_visited[i]), sizeof(int), 1, file);
	}
	fclose(file);
	return true;
}

bool CGeodesicDistanceOnSurface::load_from_file_bin(const char* filename)
{
	FILE *file = NULL;
	if( (fopen_s(&file, filename, "rb")) != 0 )
	{
		printf("Error when reading CGeodesicDistanceOnSurface File<%s>\n", filename);
		return false;
	}

	int vtNum;
	int numread = fread(&vtNum, sizeof(int), 1, file);

	free_geo_dists(this->geo_dists);
	for(int i=0; i<vtNum; i++)
	{
		IndexDistList* dist_list = new IndexDistList();
		int num;
		fread(&num, sizeof(int), 1, file);
		for(int i=0; i<num; i++)
		{
			int idx;
			double dist;
			fread(&idx, sizeof(int), 1, file);
			fread(&dist, sizeof(double), 1, file);
			dist_list->push_back(make_pair(idx, dist));
		}
		this->geo_dists.push_back(dist_list);
	}

	free_geo_dists(this->geo_dists_hash);
	for(int i=0; i<vtNum; i++)
	{
		IndexDistHashTable* dist_hash = new IndexDistHashTable();
		int num;
		fread(&num, sizeof(int), 1, file);
		for(int i=0; i<num; i++)
		{
			int idx;
			double dist;
			fread(&idx, sizeof(int), 1, file);
			fread(&dist, sizeof(double), 1, file);
			dist_hash->insert(make_pair(idx, dist));
		}
		this->geo_dists_hash.push_back(dist_hash);
	}

	free_geo_dists(this->geo_dists_drt_cnt);
	for(int i=0; i<vtNum; i++)
	{
		IndexDistList* dist_list = new IndexDistList();
		int num;
		fread(&num, sizeof(int), 1, file);
		for(int i=0; i<num; i++)
		{
			int idx;
			double dist;
			fread(&idx, sizeof(int), 1, file);
			fread(&dist, sizeof(double), 1, file);
			dist_list->push_back(make_pair(idx, dist));
		}
		this->geo_dists_drt_cnt.push_back(dist_list);
	}

	this->ptr_cur_visited.clear();
	for(int i=0; i<vtNum; i++)
	{
		int ptr;
		fread(&ptr, sizeof(int), 1, file);
		ptr_cur_visited.push_back(ptr);
	}
	fclose(file);
	return true;
}