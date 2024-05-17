#include "nonrigid_matching_surface.h"
#include "utility.h"
#include "DeformationGraphOptBasic.h"
#include "affines_nonrigid_pw_opt.h"
namespace NonrigidMatching{

bool generate_deform_graph_from_surface( CSurface<float> const& surface,
										 CGeodesicDistanceOnSurface &geo_dist,
										 DeformGraph &graph,
										 double min_node_dist,
										 int neighbor_nodes_num,
										 int thres_num_isolated_vts)
{
	if( surface.vtNum == 0 || surface.triNum == 0 )		
	{
		LOGGER()->error("generate_deform_graph_from_surface","the surface is invalid!\n");
		return false;
	}
	
	LOGGER()->trace("Points num: %d\n", surface.vtNum);

	//initialize CGeodesicDistanceOnSurface
	geo_dist.associate_with_surface(surface);

	vnl_vector<int> bNode(surface.vtNum); //1-unvisited. 0-deleted, 2-randomly selected
	bNode.fill(1);
	int loops = 0;
	int NumOfLoopsWithoutDeletion = 0;
	int count_remain = surface.vtNum;
	while(true)
	{
		LOGGER()->trace("point deletion loops: %06d\r", loops);
		int idxSelected = ROUND(RANDOM * (count_remain-1));

		//find the point with the index of idxSelected
		int nonvisited_points_count = 0;
		int vtIdx=0;
		for(; vtIdx<bNode.size(); vtIdx++)
		{
			if( bNode[vtIdx]==1 )
			{
				if( nonvisited_points_count == idxSelected )
					break;
				nonvisited_points_count++;
			}
		}
		assert(vtIdx >=0 && vtIdx < surface.vtNum);

		bNode[vtIdx] = 2;
		count_remain--;
		if( count_remain == 0 )
			break;

		//delete its nearby points
		IndexDistList& dist_list_cur = geo_dist.calc_dijkstra_dist_for_one_vertex(vtIdx, min_node_dist);
		for(int i=0; i<dist_list_cur.size(); i++)
		{
			int idx_nearby = dist_list_cur[i].first;
			double dist_nearby = dist_list_cur[i].second;

			if( bNode[idx_nearby] == 0 )
				continue;

			if(dist_nearby <= min_node_dist )
			{
				if( bNode[idx_nearby] == 2 )
				{
					LOGGER()->trace("Info: impossible situation!\n");
					continue;
				}
				bNode[idx_nearby] = 0;
				count_remain--;
			}
		}
		if( count_remain <= 0 )
			break;
		loops++;		
	} //end of while
	

//=======Generate Graph==========
	float *vtData = surface.vtData;
	int vtDim = surface.vtDim;
	vector< vnl_vector_fixed<double, 3> > graph_nodes_vec;
	vector< int > graph_nodes_vtIdx_vec;

	for(int i=0; i<bNode.size(); i++)
	{
		if( bNode[i]==2 )
		{
			float* pVt = &(vtData[vtDim*i]);
			graph_nodes_vec.push_back(vnl_vector_fixed<double, 3>(pVt[0], pVt[1], pVt[2]));
			graph_nodes_vtIdx_vec.push_back(i);
		}
	}

	//build geo_dist
#pragma omp parallel for
	for(int i=0; i<graph_nodes_vec.size(); i++)
	{
		int vtIdx = graph_nodes_vtIdx_vec[i];
		IndexDistList& dist_list_all = geo_dist.calc_dijkstra_dist_for_one_vertex(vtIdx, 2.5*min_node_dist);
	}

	//delete the isolated graph nodes from the list
	for(int i=0; i<graph_nodes_vec.size(); i++)
	{
		LOGGER()->trace("Checking Nearest nodes at %4d\r", i);
		int vtIdx = graph_nodes_vtIdx_vec[i];

		IndexDistList const& dist_list_all = geo_dist.dist_list_for_one_vertex(vtIdx);
		vector<DoubleIndexPair> dist_vec;
		for(int j=0; j<graph_nodes_vec.size(); j++)
		{
			int vtIdx_j = graph_nodes_vtIdx_vec[j];
			double d = geo_dist(vtIdx, vtIdx_j);
			if( d < 0.0)
				continue;
			assert( d >= min_node_dist || d == 0.0 );
			dist_vec.push_back(DoubleIndexPair(d, j));
		}
		
		//if a node is not connected with other graph nodes and has very few dense points neighbors, delete it
		if( dist_vec.size() <= 1 && dist_list_all.size() <= thres_num_isolated_vts )
		{
			graph_nodes_vec.erase(graph_nodes_vec.begin()+i);
			graph_nodes_vtIdx_vec.erase(graph_nodes_vtIdx_vec.begin()+i);
			i--;
		}
	}

	//find the neighbors to build the graph
	graph.nodes.clear();
	int graph_nodes_num = graph_nodes_vec.size();
	graph.nodes.resize(graph_nodes_num);
#pragma omp parallel for
	for(int i=0; i<graph_nodes_vec.size(); i++)
	{
		LOGGER()->trace("Checking Nearest nodes at %4d\r", i);
		DeformGraphNode node;
		node.idx = i;
		node.g = graph_nodes_vec[i];
		node.vtIdx = graph_nodes_vtIdx_vec[i];

		vector<DoubleIndexPair> dist_vec;
		for(int j=0; j<graph_nodes_vec.size(); j++)
		{
			int vtIdx_j = graph_nodes_vtIdx_vec[j];
			double d = geo_dist(node.vtIdx, vtIdx_j);
			if( d < 0.0)
				continue;

			assert( d >= min_node_dist || d == 0.0 );
			dist_vec.push_back(DoubleIndexPair(d, j));
		}
		sort(dist_vec.begin(), dist_vec.end(), less_comparator_DoubleIndexPair);
		assert( dist_vec[0].second == i);

		for(int j=0; j<MIN(dist_vec.size()-1, neighbor_nodes_num); j++)
		{
			int node_idx = dist_vec[j+1].second;
			node.neighborIndices.push_back(node_idx);
		}

		if( node.neighborIndices.size() == 0)
			LOGGER()->trace("Warning: isolated node found!\n");

		graph.nodes[i] = node;
	}

	return true;
}

// compute the graph_r that align surface_t to surface
bool reverse_DeformGraph2( DeformGraph const& graph, 
						   CSurface<float> const& surface, //template
						   vector< NeighborGraphNodesOfPoint > const& ngns, //on surface (template)
						   DeformGraph &graph_r  //out						
						   )
{
	CSurface<float> surface_t = surface;
	transform_Surface_with_DeformGraph(surface_t, graph, ngns, false);

	graph_r = graph;
	graph_r.reverse();

	S3DPointMatchSet match_set;
	vector< NeighborGraphNodesOfPoint > ngns_matched_points;
	for(int i=0; i<surface.vtNum; i++)
	{
		if( ngns[i].neighborIndices.size() == 0 )
			continue;

		float const* vt = surface.vt_data_block(i);
		float const* vt_t = surface_t.vt_data_block(i);
		
		vnl_vector_fixed<double, 3> p( vt[0], vt[1], vt[2]);
		vnl_vector_fixed<double, 3> q( vt_t[0], vt_t[1], vt_t[2]);
		match_set.push_back(q, p);
		ngns_matched_points.push_back(ngns[i]);
	}

//=======RUN deform graph optimization
	DeformGraphOptimizationData data;
	data.bFixGlobalRigid = false;
	data.bFixNonrigid = false;
	data.graph = graph_r;
	data.matched_points_3d = match_set;
	data.ngns_key_points = ngns_matched_points;
	data.w_rot = 300.0; //3000
	data.w_reg = 0.3; //30.0
	data.w_constr = 10.0;	//50.0
	data.w_nodes_dist = 0.0;
	data.itmax = 30;
	deform_graph_optimization(data);
	graph_r = data.graph;

	return true;
}

//for each graph node, transform it and then find its nearest point on the target surface
bool generate_reverse_deform_graph_for_target_surface( DeformGraph const &graph,
													   CSurface<float> const& surface_t,//the target surface
													   DeformGraph &graph_r,
													   CGeodesicDistanceOnSurface &geo_dist_t,
													   double thres_dist_node_to_surf,
													   bool bReSample,													   										 
													   double min_node_dist,
													   int neighbor_nodes_num)
{
	geo_dist_t.associate_with_surface(surface_t);

	DeformGraph graph_r_t = graph;
	graph_r_t.reverse();
	force_graph_on_surface(graph_r_t, graph_r, surface_t, thres_dist_node_to_surf);

	//resample the node
	if( bReSample )
	{
		DeformGraph graph_t;
		generate_deform_graph_from_surface(surface_t, geo_dist_t, graph_t, min_node_dist, neighbor_nodes_num);

		deform_grpah_initialization_Geodesic(graph_t, graph_r, surface_t, geo_dist_t);

		graph_r = graph_t;

	}

	return true;
}

//generate reverse graph by find the matched dense points
bool generate_reverse_deform_graph( DeformGraph const& graph, 
									CSurface<float> const& surface, CSurface<float> const& surface_dst,
									vector< NeighborGraphNodesOfPoint > const& ngns, //on surface (template)
									DeformGraph &graph_r,  //out
									CGeodesicDistanceOnSurface &geo_dist_dst, //out; unnecessary to initialize it	
									vector< NeighborGraphNodesOfPoint > &ngns_dst,
									bool bResample,
									double min_node_dist,
									int neighbor_nodes_num,
									bool bUseIntPutGraphAndNgns,
									double angle_thres,
									double dist_thres,
									int downsample_rate_for_opt,
									bool bOptNgns)
{
	if (!bUseIntPutGraphAndNgns)
	{
		LOGGER()->info("generate graph...");
		generate_reverse_deform_graph_for_target_surface(graph, surface_dst, graph_r, geo_dist_dst, 3.0, bResample, min_node_dist, neighbor_nodes_num);
		LOGGER()->info("end<%d>!", graph.nodes.size());
		LOGGER()->info("calc ngns...");
		calc_NeighborGraphNodesOfPoint_Surface_all_vts(graph_r, surface_dst, geo_dist_dst, ngns_dst, neighbor_nodes_num - 1, 2.2*min_node_dist);
		LOGGER()->info("end<%d>", ngns_dst.size());
	}
	else
	{
		DeformGraph graph_r_t = graph;
		graph_r_t.reverse();
		graph_r.reset(true, true);
		graph_r.global_rigid = graph_r_t.global_rigid;
	}

	CSurface<float> surface_t = surface;
	transform_Surface_with_DeformGraph(surface_t, graph, ngns, false);
	surface_t.extract_boundary();

	int vtNum_dst = surface_dst.vtNum;
	vector< vnl_vector_fixed<double, 3> > points_1(vtNum_dst);
	vector< vnl_vector_fixed<double, 3> > points_2(vtNum_dst);
	vector<int> vt_indices_corr(vtNum_dst);
	vector<bool> bEmpty(vtNum_dst, true);
#pragma omp parallel for
	for(int vtIdx=0; vtIdx<surface_dst.vtNum; vtIdx+=downsample_rate_for_opt)
	{
		float const*pVt = surface_dst.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> p(pVt[0], pVt[1], pVt[2]);
		vnl_vector_fixed<double, 3> q;
		int index = 0;
		closest_point_on_surface(surface_t, p, q, index, DistanceMode_Point2Point);

		float angle = M_PI;
		if( surface_t.haveNormalInfo() && surface_dst.haveNormalInfo() )
		{
			float const* n1 = surface_t.vt_normal(index);
			float const* n2 = surface_dst.vt_normal(vtIdx);
			angle = angle_btw_two_vec(vnl_vector_fixed<float, 3>(n1[0], n1[1], n1[2]), vnl_vector_fixed<float, 3>(n2[0], n2[1], n2[2]));
		}

		double dist = dist_3d(p, q);
		//assert(dist == 0);
		if( dist < dist_thres && angle*180/M_PI < angle_thres && !surface_t.bBoundary[index])
		{
			bEmpty[vtIdx] = false;
			float const*pVt_ori = surface.vt_data_block(index);
			vnl_vector_fixed<double, 3> q_ori(pVt_ori[0], pVt_ori[1], pVt_ori[2]);
			points_1[vtIdx] = p;
			points_2[vtIdx] = q_ori;
			vt_indices_corr[vtIdx] = index;
		}
		else
		{
			bEmpty[vtIdx] = true;
		}
	}
	
	//S3DPointMatchSet match_set;
	S3DPointMatchSetIndexed match_set;
	vector< NeighborGraphNodesOfPoint > ngns_matched_points;
	for(int i=0; i<bEmpty.size(); i++)
	{
		if( !bEmpty[i] )
		{
			match_set.push_back( points_1[i], points_2[i]);
			match_set.indices_1.push_back(i);
			match_set.indices_2.push_back(vt_indices_corr[i]);
			ngns_matched_points.push_back(ngns_dst[i]);
		}
	}

//=======RUN deform graph optimization
	DeformGraphOptimizationData data;
	data.bFixGlobalRigid = false;
	data.bFixNonrigid = false;
	data.graph = graph_r;
	data.matched_points_3d = match_set;
	data.ngns_key_points = ngns_matched_points;
	data.w_rot = 3000.0; //3000
	data.w_reg = 500; //30.0
	data.w_zero = 100;
	data.w_constr = 50.0*downsample_rate_for_opt;	//50.0
	data.w_nodes_dist = 0.0;
	data.itmax = 50;
	data.bFixNonrigid = true;
	deform_graph_optimization(data);
	data.itmax = 25;
	data.bFixNonrigid = false;
	deform_graph_optimization(data);
	graph_r = data.graph;

	if (bOptNgns)
	{
		NonrigidMatchingPara para;
		para.bUseColorField = true;
		para.bFixEDWeights = false;
		para.bFixGlobalRigid = false;
		para.bFixNonrigid = false;
		para.bUseZollhofer = true;
		para.w_rot = 5000;
		para.w_reg = 300.0;
		para.w_weights_sum_one = 5000;
		para.w_zero = 0;
		para.w_weights_smooth = 30;
		para.w_intrinsic = 0;
		para.w_weights_polar = 10;
		para.w_sdf = 0;// 300;
		para.w_clr = 0;
		para.w_constr = 50;// 50.0;
		para.itmax = 30;
		para.w_intersect_init = 0.0;//2000; //1000 seems to not work
		para.itmax = 10;
		para.vt_clr_downsample_rate = 1;
		para.vt_sdf_downsample_rate = 1;
		para.pt_constr_thres = 20.0;
		para.w_weights_smooth = 0.5;
		affine_set_nonrigid_pw_opt(surface_dst, &graph_r, &ngns_dst, NULL, vector<cv::Mat>(), vector<GCameraView*>(), match_set, vector<double>(), para, NULL);
		para.pt_constr_thres = 2.0; 
		para.w_weights_smooth = 0.2;
		affine_set_nonrigid_pw_opt(surface_dst, &graph_r, &ngns_dst, NULL, vector<cv::Mat>(), vector<GCameraView*>(), match_set, vector<double>(), para, NULL);
	}

	return true;
}


bool generate_relative_deform_graph( CSurface<float> const& surface,
									 DeformGraph const& graph_1, // deform surface to surface_1
									 DeformGraph const& graph_2, // deform surface to surface_2
									 vector< NeighborGraphNodesOfPoint > const& ngns_1,
									 vector< NeighborGraphNodesOfPoint > const& ngns_2,
									 DeformGraph &graph_relative // deform surface_1 to surface_2
									 )
{
	CSurface<float> surface_1 = surface;
	transform_Surface_with_DeformGraph( surface_1, graph_1, ngns_1, false);
	CSurface<float> surface_2 = surface;
	transform_Surface_with_DeformGraph( surface_2, graph_2, ngns_2, false);

	S3DPointMatchSet match_set;
	for(int i=0; i<surface.vtNum; i++)
	{
		float *p1 = surface_1.vt_data_block(i);
		float *p2 = surface_2.vt_data_block(i);
		match_set.push_back(vnl_vector_fixed<double, 3>(p1[0], p1[1], p1[2]), vnl_vector_fixed<double, 3>(p2[0], p2[1], p2[2]));
	}

	graph_relative = graph_1;
	graph_relative.bAinvFilled = false;
	for(int i=0; i<graph_relative.nodes.size(); i++)
	{
		DeformGraphNode &node = graph_relative.nodes[i];
		node.g = node.g+node.t;
		node.A.set_identity();
		node.t.fill(0.0);
	}

//=======RUN deform graph optimization
	DeformGraphOptimizationData data;
	data.graph = graph_relative;
	data.matched_points_3d = match_set;
	data.ngns_key_points = ngns_1;
	data.w_rot = 300.0; //3000
	data.w_reg = 0.3; //30.0
	data.w_constr = 10.0;	//50.0
	deform_graph_optimization(data);
	graph_relative = data.graph;

	return true;	
}

bool force_graph_on_surface( DeformGraph const&graph, DeformGraph &graph_new, CSurface<float> const&surface, 
							 double thres_dist_node_to_surf, 
							 vector<int> *indices_old, //new graph node idx to old graph index
							 bool bUpdateG
							 )
{
	graph_new.nodes.clear();
	int node_num = graph.nodes.size();
	vector<bool> bFlag(node_num);//false if node should be deleted;

	if( indices_old!=NULL)	indices_old->clear();

	//find the nearest node on the surface
	int count = 0;
	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode node = graph.nodes[i];

		vnl_vector_fixed<double, 3> const&g = node.g;
		int vtIdx_nearest = -1;
		double dist_min = 1.0e+20;
		float p[3];
		p[0] = g[0]; p[1] = g[1]; p[2] = g[2];
		for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
		{
			float const* vt = surface.vt_data_block(vtIdx);
			if( abs(p[0]-vt[0]) > thres_dist_node_to_surf ||
				abs(p[1]-vt[1]) > thres_dist_node_to_surf ||
				abs(p[2]-vt[2]) > thres_dist_node_to_surf )
				continue;
			double dist = VecOperation<float>::Distance(p, vt, 3);
			if( dist < dist_min )
			{
				dist_min = dist;
				vtIdx_nearest = vtIdx;
			}
		}
		if( vtIdx_nearest == -1 || dist_min > thres_dist_node_to_surf)
		{
			bFlag[i] = false;
		}
		else
		{
			bFlag[i] = true;
			node.vtIdx = vtIdx_nearest;
			float const*vt = surface.vt_data_block(vtIdx_nearest);
			if(bUpdateG)	
				node.g = vnl_vector_fixed<double, 3>(vt[0], vt[1], vt[2]);
			node.idx = count;
			graph_new.nodes.push_back(node);
			count++;
			if( indices_old != NULL)
				indices_old->push_back(i);
		}
	}

	//if some of nodes are deleted, then change the node indices of the neighbors
	if( count != node_num )
	{
	//====change the node indices of the neighbors===
		//build the look up table
		LOGGER()->trace("warning<force_graph_on_surface>: delete nodes!\n");
		vector<int> idx_LUT(node_num); //old graph index to new index

		count = 0;
		for(int i=0; i<node_num; i++)
		{
			if( bFlag[i] )
			{
				idx_LUT[i] = count;
				count++;
			}
			else
				idx_LUT[i] = -1;
		}
		assert(count == graph_new.nodes.size());

		//change the neighbor indices
		for(int i=0; i<graph_new.nodes.size(); i++)
		{
			vector<int> &indices = graph_new.nodes[i].neighborIndices;
			for(int k=0; k<indices.size(); k++)
			{
				int idx = indices[k];
				if( idx_LUT[idx] == -1 )
				{
					indices.erase(indices.begin()+k);
					k--;
				}
				else
				{
					indices[k] = idx_LUT[idx];
				}
			}
		}
	}

	graph_new.global_rigid = graph.global_rigid;

	return true;
}

bool deform_grpah_initialization_Geodesic( DeformGraph &graph, DeformGraph const& graph_prev, 
										   CSurface<float> const&surface, 
										   CGeodesicDistanceOnSurface &geo_dist)
{										   
	if( graph_prev.nodes.size() == 0 )
	{
		LOGGER()->trace("Warning: NO previous graph nodes!\n");
		graph.global_rigid = graph_prev.global_rigid;
		return false;
	}

	DeformGraph graph_prev_on_surface;
	force_graph_on_surface(graph_prev, graph_prev_on_surface, surface, 1.0);

	//calc geo_dist for each node on graph
	if( !geo_dist.bAssociatedWithSurface )
		geo_dist.associate_with_surface(surface);
	for(int i=0; i<graph.nodes.size(); i++)
	{
		geo_dist.calc_dijkstra_dist_for_one_vertex(graph.nodes[i].vtIdx, 1.6*7.0);
	}

	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode &node = graph.nodes[i];
		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_Surface(node.g, graph_prev_on_surface, surface, geo_dist, ngn, 5, 1.6*7.0);
		if( ngn.neighborIndices.size() == 0)
			calc_NeighborGraphNodesOfPoint_by_dist(node.g, graph_prev_on_surface, ngn, 5);
		assert(ngn.neighborIndices.size() > 0);

		if( ngn.neighborIndices.size() != 0)
		{
			vnl_matrix_fixed<double, 3, 3> A;
			A.fill(0.0);
			for(int k=0; k<ngn.neighborIndices.size(); k++)
			{
				int idx = ngn.neighborIndices[k];
				assert(idx < graph_prev_on_surface.nodes.size() );
				double w = ngn.weights[k];
				A += graph_prev_on_surface.nodes[idx].A * w;
			}
			node.A = A;

			vnl_vector_fixed<double, 3> v = node.g;
			vnl_vector_fixed<double, 3> v_;
			transform_one_point_with_DeformGraph(v, v_, graph_prev_on_surface, ngn, false);
			node.t = v_-v;
		}		
	}

	graph.global_rigid = graph_prev.global_rigid;
	return true;
}

void deform_graph_from_graph_chain( CSurface<float> const& surface_1st, 
									vector< DeformGraph* > const& graphs,
									NeighborGraphNodesOfPointList const& ngns,
									DeformGraph &graph_1st_to_last)
{
	CSurface<float> surface_t = surface_1st;

	vector< DeformGraph* > graphs_with_same_nodes;
	graphs_with_same_nodes.push_back((DeformGraph*)graphs[0]);
	for (int i = 0; i < graphs.size()-1; i++)
	{
		LOGGER()->trace("%03d/%03zd\n", i, graphs.size());
		transform_Surface_with_DeformGraph(surface_t, *(graphs_with_same_nodes[i]), ngns, false);
		
		DeformGraph *graph_tmp = new DeformGraph(); // from i+1 to i+2
		*graph_tmp = *(graphs_with_same_nodes[i]);
		for (int k = 0; k < graph_tmp->nodes.size(); k++)
		{
			DeformGraphNode &nd = graph_tmp->nodes[k];
			nd.g = nd.g + nd.t;
		}

		deform_grpah_initialization_Geodesic(*graph_tmp, *(graphs[i + 1]), surface_t);
		graphs_with_same_nodes.push_back(graph_tmp);
	}

	//apply the deformation graph chair
	graph_1st_to_last = *(graphs[0]);
	for (int ndIdx = 0; ndIdx < graph_1st_to_last.nodes.size(); ndIdx++)
	{
		DeformGraphNode &nd = graph_1st_to_last.nodes[ndIdx];
		for (int i = 1; i < graphs_with_same_nodes.size(); i++)
		{
			DeformGraphNode const& nd_tmp = graphs_with_same_nodes[i]->nodes[ndIdx];
			nd.A = nd_tmp.A * nd.A;
			nd.t += nd_tmp.t;
		}
	}

	//release temporary deformation graphs
	for (int i = 1; i < graphs_with_same_nodes.size(); i++)
	{
		delete graphs_with_same_nodes[i];
		graphs_with_same_nodes[i] = NULL;
	}

}

bool deform_graph_verification(CSurface<float> const& surface,
								DeformGraph &graph,
								NeighborGraphNodesOfPointList const& ngns,
								CSurface<float> const& surface_dst,
								bool bInitBadNodeWithNN)
{
	CSurface<float> surface_t = surface;
	transform_Surface_with_DeformGraph(surface_t, graph, ngns, false);

	GraphNodesTerritory node_territory;
	calc_deform_graph_node_territory(graph, ngns, node_territory, true);

	int nodes_num = graph.nodes.size();
	vector<bool> bNodeHasMatch(nodes_num, false);
#pragma omp parallel for
	for (int ndIdx = 0; ndIdx < graph.nodes.size(); ndIdx++)
	{
		vector<int> const& vt_indices = node_territory[ndIdx];
		int count_vt_matched = 0;
		for (int i = 0; i < vt_indices.size(); i++)
		{
			int vtIdx = vt_indices[i];
			float const* vt_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<float, 3> vt_t(vt_t_);

			int vtIdx_dst;
			vnl_vector_fixed<float, 3> q;
			closest_point_on_surface(surface_dst, vt_t, q, vtIdx_dst, DistanceMode_Point2Plane);
			double dist = dist_3d(vt_t, q);
			if (dist < 0.5)
				count_vt_matched++;
		}
		if (count_vt_matched > 10 && count_vt_matched / double(vt_indices.size()) > 0.1)
			bNodeHasMatch[ndIdx] = true;
		else
			bNodeHasMatch[ndIdx] = false;
	}

	int count_nodes_wo_match = 0;
	for (int i = 0; i < bNodeHasMatch.size(); i++)
		if (!bNodeHasMatch[i])
			count_nodes_wo_match++;
	if (count_nodes_wo_match == 0)
		return true;

	// distribute the flags over the graph, so the bad node turns good if it connects to a good node
	bool bNodesFlagChanged = true;
	while (bNodesFlagChanged)
	{
		bNodesFlagChanged = false;
		for (int i = 0; i<nodes_num; i++)
		{
			vector<int> const& neighbors = graph.nodes[i].neighborIndices;
			if (bNodeHasMatch[i])
			{
				for (int j = 0; j<neighbors.size(); j++)
				{
					int idx = neighbors[j];
					if (bNodeHasMatch[idx])
						continue;

					bNodeHasMatch[idx] = true;
					bNodesFlagChanged = true;
				}
			}
			else
			{
				for (int j = 0; j<neighbors.size(); j++)
				{
					int idx = neighbors[j];
					if (bNodeHasMatch[idx])
					{
						bNodeHasMatch[i] = true;
						bNodesFlagChanged = true;
						break;
					}
				}
			}
		}
	}

	count_nodes_wo_match = 0;
	for (int i = 0; i < bNodeHasMatch.size(); i++)
		if (!bNodeHasMatch[i])
			count_nodes_wo_match++;
	if (count_nodes_wo_match == 0)
		return true;

	LOGGER()->warning("deform_graph_verification","%d nodes without matches!", count_nodes_wo_match);

	if (bInitBadNodeWithNN)
	{
		for (int ndIdx = 0; ndIdx < nodes_num; ndIdx++)
		{
			if (bNodeHasMatch[ndIdx])	continue;

			DeformGraphNode &node = graph.nodes[ndIdx];

			//find the nearest good node
			double dist_min = 1.0e+10;
			int ndIdx_min = -1;
			for (int i = 0; i < graph.nodes.size(); i++)
			{
				if (!bNodeHasMatch[i]) continue;

				vnl_vector_fixed<double, 3> const &g = graph.nodes[i].g;
				double d = dist_3d(node.g, g);
				if (d < dist_min)
				{
					dist_min = d;
					ndIdx_min = i;
				}
			}
			assert(ndIdx_min != -1);
			NeighborGraphNodesOfPoint ngn;
			ngn.neighborIndices.push_back(ndIdx_min);
			ngn.weights.push_back(1.0);

			//apply the affine transformation parameter of nearest nodes to current nodes
			if (ngn.neighborIndices.size() != 0)
			{
				vnl_matrix_fixed<double, 3, 3> A;
				A.fill(0.0);
				for (int k = 0; k < ngn.neighborIndices.size(); k++)
				{
					int idx = ngn.neighborIndices[k];
					assert(idx < graph.nodes.size());
					double w = ngn.weights[k];
					A += graph.nodes[idx].A * w;
				}
				node.A = A;

				vnl_vector_fixed<double, 3> v = node.g;
				vnl_vector_fixed<double, 3> v_;
				transform_one_point_with_DeformGraph(v, v_, graph, ngn);
				node.t = v_ - v;
			}

		}
	}
	else
	{
		;//delete node
	}

	return false;
}


bool reverse_transform_Surface_with_DeformGraph( DeformGraph const &graph,
												 CSurface<float> &surface,
												 CGeodesicDistanceOnSurface &geo_dist,
												 int num_neighbor_nodes,
												 double thres_dist_to_node)
{
	DeformGraph graph_r;
	generate_reverse_deform_graph_for_target_surface(graph, surface, graph_r, geo_dist, 2.0, true, 7.0, 5);

	//geo_dist.associate_with_surface(surface);
	vector< NeighborGraphNodesOfPoint > ngns;
	LOGGER()->trace("computing ngns...");
	calc_NeighborGraphNodesOfPoint_Surface_all_vts(graph_r, surface, geo_dist, ngns, num_neighbor_nodes, thres_dist_to_node);
	LOGGER()->trace("end!\n");
	transform_Surface_with_DeformGraph(surface, graph_r, ngns);
	return true;
}


bool identify_possible_collision_on_deformGraph( DeformGraph const& graph,
												 vector<int> &collided_nodes_list,
												 double thres_collision_dist)
{
// nodes to be put into the list must be fulfilled the following requirements: 
//    there exists at leat one node other than its neighbors that is closer than thres_collision_dist
//	  and 40% their original dist in the reference pose

	int nodes_num = graph.nodes.size();
	vnl_vector<bool> bCollided(nodes_num);
	bCollided.fill(false);
	for(int i=0; i<nodes_num; i++)
	{
		if(bCollided[i])	continue;

		DeformGraphNode const& node_i = graph.nodes[i];
		for(int j=i+1; j<nodes_num; j++)
		{
			if( search_val_list(graph.nodes[i].neighborIndices, j) >= 0 )
				continue;
			
			DeformGraphNode const& node_j = graph.nodes[j];
			vnl_vector_fixed<double, 3> p1 = node_i.g + node_i.t;
			vnl_vector_fixed<double, 3> p2 = node_j.g + node_j.t;

			double dist_cur = dist_3d(p1, p2);
			
			if( dist_cur > thres_collision_dist )
				continue;
			
			double dist_ref = dist_3d(node_i.g, node_j.g);
			if( dist_cur < dist_ref*0.4 )
			{
				bCollided[i] = true;
				bCollided[j] = true;
				break;				
			}
		}
	}

	for(int i=0; i<nodes_num; i++)
		if( bCollided[i] )
			collided_nodes_list.push_back(i);

	return true;
}

bool identify_collision_on_surface( CSurface<float> const& surface_ref, 
									CSurface<float> const& surface_t,
									DeformGraph const& graph, //graph on the reference surface, transformed graph will be on "surface"
									vector< NeighborGraphNodesOfPoint > const& ngns,
									vector< vector<int> > const& graph_node_territory,
									S3DPointMatchSet &match_set_collsion,
									double thres_collision_dist)
{
	match_set_collsion.clear();

	vector<int> collided_nodes_list;
	identify_possible_collision_on_deformGraph( graph, collided_nodes_list, 10.0);
	for(int i=0; i<collided_nodes_list.size(); i++)
	{
		int ndIdx = collided_nodes_list[i];
		vector<int> const& vt_list = graph_node_territory[ndIdx];
		for(int j=0; j<vt_list.size(); j++)
		{
			int vtIdx = vt_list[j];
			NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];

			float const*q = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vq(q[0], q[1], q[2]);
			
			float const*p = surface_ref.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vp(p[0], p[1], p[2]);
			float const*np_ = surface_ref.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> np(np_[0], np_[1], np_[2]);
			
			//find graph nodes that the current vertex is going to collide
			vector<int> nodes_list_candidate;
			for(int k=0; k<graph.nodes.size(); k++)
			{
				//skip its neighbors
				if( search_val_list(ngn.neighborIndices, k) >= 0 )
					continue;

				DeformGraphNode const& node = graph.nodes[k];
				double dist_cur = dist_3d( vq, node.g+node.t );
				double dist_ref = dist_3d( vp, node.g );

				if( dist_cur < thres_collision_dist &&
					dist_cur < 0.4*dist_ref)
					nodes_list_candidate.push_back(k);
			}

						
			int vtIdx_t_min = -1;//nearest vertex that is going to collide
			double dist_min = 1.0e+10;
			for(int c=0; c<nodes_list_candidate.size(); c++)
			{
				int ndIdx_t = nodes_list_candidate[c];
				vector<int> const& vt_list_t = (graph_node_territory)[ndIdx_t];
				for(int k=0; k<vt_list_t.size(); k++)
				{
					int vtIdx_nr = vt_list_t[k];
					float const*v_nr_ = surface_t.vt_data_block(vtIdx_nr);
					vnl_vector_fixed<double, 3> v_nr(v_nr_[0], v_nr_[1], v_nr_[2]);
					float const*vp_nr_ = surface_ref.vt_data_block(vtIdx_nr);
					vnl_vector_fixed<double, 3> vp_nr(vp_nr_[0], vp_nr_[1], vp_nr_[2]);

					double dist_ref = dist_3d(vp, vp_nr);
					double dist_cur = dist_3d(vq, v_nr);
					if( dist_cur < dist_min &&
						dist_cur < 0.4*dist_ref)
					{
						dist_min = dist_cur;
						vtIdx_t_min = vtIdx_nr;
					}
				}
			}

			if( vtIdx_t_min == -1 )
				continue;

			float const*v_nr_ = surface_t.vt_data_block(vtIdx_t_min);
			vnl_vector_fixed<double, 3> v_nr(v_nr_[0], v_nr_[1], v_nr_[2]);
			float const*n_nr_ = surface_t.vt_normal(vtIdx_t_min);
			vnl_vector_fixed<double, 3> n_nr(n_nr_[0], n_nr_[1], n_nr_[2]);
			
			float const*vp_nr_ = surface_ref.vt_data_block(vtIdx_t_min);
			vnl_vector_fixed<double, 3> vp_nr(vp_nr_[0], vp_nr_[1], vp_nr_[2]);
			float const*np_nr_ = surface_ref.vt_normal(vtIdx_t_min);
			vnl_vector_fixed<double, 3> np_nr(np_nr_[0], np_nr_[1], np_nr_[2]);

			if( dot_product(n_nr, vq-v_nr) > 0 ) //outwards normal
				continue;

			if( dot_product(vp-vp_nr, np_nr) < 0 &&
				dist_3d(vp, vp_nr)<15.0 )
				continue;

			match_set_collsion.push_back(vq, v_nr);
		}

	}

	return true;
}

/* graph will transform sdf to sdf_dst
 */
void transform_sdf_with_DeformGraph( TSDF const&sdf, TSDF &sdf_dst, 
									 DeformGraph const&graph, 
									 CSurface<float> const&surface, 
									 vector< NeighborGraphNodesOfPoint > const&ngns,											 											 
									 BoundingBox3D bbox,
									 double x_res, double y_res, double z_res)

{
	double mu = sdf.mu;
	sdf_dst.Init(bbox, x_res, y_res, z_res, sdf.mu, false, false, false);

	int nx = sdf.nx;
	int ny = sdf.ny;
	int nz = sdf.nz;
	double xres = sdf.xres;
	double yres = sdf.yres;
	double zres = sdf.zres;
	double xoffset = sdf.xoffset;
	double yoffset = sdf.yoffset;
	double zoffset = sdf.zoffset;

	float *vtData = surface.vtData;
	int vtDim = surface.vtDim;
	int vtNum = surface.vtNum;

	double dist_thres = 2.0*mu;
	LOGGER()->trace("dist_to_surf=%f\n", dist_thres);
	
	VoxelMatrix<int> vtIdxVol_n(nx, ny, nz, -1, -1);
	vtIdxVol_n.fill(-1);
	VoxelMatrix<float> distsVol_n(nx, ny, nz, -1.0, -1.0);
	distsVol_n.fill(1.0e+20);
	for(int vtIdx=0; vtIdx<vtNum; vtIdx++)
	{
		NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];
		if(ngn.neighborIndices.size() == 0)
			continue;

		float *pVt = &(vtData[vtIdx*vtDim]);
		int x1 = MAX(0, (int)((pVt[0]-dist_thres-xoffset)/xres));
		int x2 = MIN(nx-1, ROUND((pVt[0]+dist_thres-xoffset)/xres));
		int y1 = MAX(0, (int)((pVt[1]-dist_thres-yoffset)/yres));
		int y2 = MIN(ny-1, ROUND((pVt[1]+dist_thres-yoffset)/yres));
		int z1 = MAX(0, (int)((pVt[2]-dist_thres-zoffset)/zres));
		int z2 = MIN(nz-1, ROUND((pVt[2]+dist_thres-zoffset)/zres));
		for(int k=z1; k<=z2; k++)
		{
			double z = k*zres + zoffset;
			for(int j=y1; j<=y2; j++)
			{
				double y = j*yres + yoffset;			
				for(int i=x1; i<=x2; i++)
				{
					double x = i*xres + xoffset;

					double dist_2 = (x-pVt[0])*(x-pVt[0]) + (y-pVt[1])*(y-pVt[1]) + (z-pVt[2])*(z-pVt[2]);
					if( dist_2 < distsVol_n(i, j, k) )
					{
						distsVol_n(i, j, k) = dist_2;
						vtIdxVol_n(i, j, k) = vtIdx;
					}
				}
			}
		}
	}

	int nx_d = sdf_dst.nx;
	int ny_d = sdf_dst.ny;
	int nz_d = sdf_dst.nz;
	double xres_d = sdf_dst.xres;
	double yres_d = sdf_dst.yres;
	double zres_d = sdf_dst.zres;
	double xoffset_d = sdf_dst.xoffset;
	double yoffset_d = sdf_dst.yoffset;
	double zoffset_d = sdf_dst.zoffset;

	VoxelMatrix<float> weights_max(nx_d, ny_d, nz_d, -1.0, -1.0);
	weights_max.fill(0.0);
	// transform a source point (x, y, z)
	for(unsigned int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for(unsigned int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;			
			for(unsigned int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;

				int vtIdx_n = vtIdxVol_n(i, j, k);//nearest vertex
				if( vtIdx_n == -1 )
					continue;

				vnl_vector_fixed<double, 3> p(x, y, z);
				NeighborGraphNodesOfPoint ngn = ngns[vtIdx_n];

				assert(ngn.neighborIndices.size() > 0);

				vnl_vector_fixed<double, 3> q;
				transform_one_point_with_DeformGraph(p, q, graph, ngn);

				float radius = 2.0*xres/xres_d;
				float i_d = (q[0]-xoffset_d)/xres_d;
				float j_d = (q[1]-yoffset_d)/yres_d;
				float k_d = (q[2]-zoffset_d)/zres_d;

				float val = sdf.func_val(i, j, k);
				float weight = sdf.func_val(i, j, k);

				for(int k2 = ceill(k_d-radius); k2<=floorl(k_d+radius); k2++)
				for(int j2 = ceill(j_d-radius); j2<=floorl(j_d+radius); j2++)
				for(int i2 = ceill(i_d-radius); i2<=floorl(i_d+radius); i2++)
				{
					if( k2 < 0 || k2 >= nz_d ||
						j2 < 0 || j2 >= ny_d ||
						i2 < 0 || i2 >= nx_d )
						continue;

					float w = 1.0/((k2-k_d)*(k2-k_d) + (j2-j_d)*(j2-j_d) + (i2-i_d)*(i2-i_d));

					//for each voxel in the destination sdf, choose the closest one
					if( w > weights_max(i2, j2, k2) )
					{
						weights_max(i2, j2, k2) = w;
						sdf_dst.weights(i2, j2, k2) = weight;
						sdf_dst.func_val(i2, j2, k2) = val;
					}
				}

			}//end of for-i
		}//end of for-j
	}//end of for-k
}

/* graph will transform ddf to ddf_dst
 */
void transform_ddf_with_DeformGraph( DDF const&ddf, DDF &ddf_dst, 
									 DeformGraph const&graph, 
									 CSurface<float> const&surface, 
									 vector< NeighborGraphNodesOfPoint > const&ngns,											 											 
									 BoundingBox3D bbox,
									 double x_res, double y_res, double z_res,
									 SDFTransfInterpMethod interp_method)

{
	double mu = ddf.mu;
	ddf_dst.Init(bbox, x_res, y_res, z_res, ddf.mu, false, false, ddf.bBuildColorField);

	int nx = ddf.nx;
	int ny = ddf.ny;
	int nz = ddf.nz;
	double xres = ddf.xres;
	double yres = ddf.yres;
	double zres = ddf.zres;
	double xoffset = ddf.xoffset;
	double yoffset = ddf.yoffset;
	double zoffset = ddf.zoffset;

	float *vtData = surface.vtData;
	int vtDim = surface.vtDim;
	int vtNum = surface.vtNum;

	double dist_thres = 1.4*mu;
	LOGGER()->trace("dist_to_surf=%f\n", dist_thres);
	
	__tic__();
	VoxelMatrix<int> vtIdxVol_n(nx, ny, nz,-1,-1);
	vtIdxVol_n.fill(-1);
	VoxelMatrix<float> distsVol_n(nx, ny, nz,-1.0,-1.0);
	distsVol_n.fill(1.0e+20);
	for(int vtIdx=0; vtIdx<vtNum; vtIdx++)
	{
		NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];
		if(ngn.neighborIndices.size() == 0)
			continue;

		float *pVt = &(vtData[vtIdx*vtDim]);
		int x1 = MAX(0, (int)((pVt[0]-dist_thres-xoffset)/xres));
		int x2 = MIN(nx-1, ROUND((pVt[0]+dist_thres-xoffset)/xres));
		int y1 = MAX(0, (int)((pVt[1]-dist_thres-yoffset)/yres));
		int y2 = MIN(ny-1, ROUND((pVt[1]+dist_thres-yoffset)/yres));
		int z1 = MAX(0, (int)((pVt[2]-dist_thres-zoffset)/zres));
		int z2 = MIN(nz-1, ROUND((pVt[2]+dist_thres-zoffset)/zres));
#pragma omp parallel for
		for(int k=z1; k<=z2; k++)
		{
			double z = k*zres + zoffset;
			for(int j=y1; j<=y2; j++)
			{
				double y = j*yres + yoffset;			
				for(int i=x1; i<=x2; i++)
				{
					double x = i*xres + xoffset;

					double dist_2 = (x-pVt[0])*(x-pVt[0]) + (y-pVt[1])*(y-pVt[1]) + (z-pVt[2])*(z-pVt[2]);
					if( dist_2 < distsVol_n(i, j, k) )
					{
						distsVol_n(i, j, k) = dist_2;
						vtIdxVol_n(i, j, k) = vtIdx;
					}
				}
			}
		}
	}
	LOGGER()->trace("nn search = %f\n", __toc__());
	
	int nx_d = ddf_dst.nx;
	int ny_d = ddf_dst.ny;
	int nz_d = ddf_dst.nz;
	double xres_d = ddf_dst.xres;
	double yres_d = ddf_dst.yres;
	double zres_d = ddf_dst.zres;
	double xoffset_d = ddf_dst.xoffset;
	double yoffset_d = ddf_dst.yoffset;
	double zoffset_d = ddf_dst.zoffset;

	__tic__();
	TransformedVoxelList tsfmedVoxelList_all;
	VoxelMatrix<TransformedVoxelList> tsfmedVoxelList_Mat(nx_d, ny_d, nz_d, TransformedVoxelList(0), TransformedVoxelList(0));
	// transform a source point (x, y, z)
	for(unsigned int k=0; k<nz; k++)
	{
		double z = k*zres + zoffset;
		for(unsigned int j=0; j<ny; j++)
		{
			double y = j*yres + yoffset;			
			for(unsigned int i=0; i<nx; i++)
			{
				double x = i*xres + xoffset;

				int vtIdx_n = vtIdxVol_n(i, j, k);//nearest vertex
				if( vtIdx_n == -1 )
					continue;

				if( ddf.weights(i, j, k) == 0 || ddf.func_val(i, j, k) == SDF_NULL_VALUE)
					continue;

				vnl_vector_fixed<double, 3> p(x, y, z);
				NeighborGraphNodesOfPoint const&ngn = ngns[vtIdx_n];

				if( ngn.neighborIndices.size() == 0)
					continue;
				assert(ngn.neighborIndices.size() > 0);

				vnl_vector_fixed<double, 3> q;
				transform_one_point_with_DeformGraph(p, q, graph, ngn);

				float radius = 2.0*xres/xres_d;
				float i_d = (q[0]-xoffset_d)/xres_d;
				float j_d = (q[1]-yoffset_d)/yres_d;
				float k_d = (q[2]-zoffset_d)/zres_d;
				float val = ddf.func_val(i, j, k);

				TransformedVoxel *tsfmed_voxel = new TransformedVoxel();
				tsfmed_voxel->coord_ori[0] = i;
				tsfmed_voxel->coord_ori[1] = j;
				tsfmed_voxel->coord_ori[2] = k;
				tsfmed_voxel->coord_t[0] = i_d;
				tsfmed_voxel->coord_t[1] = j_d;
				tsfmed_voxel->coord_t[2] = k_d;
				tsfmed_voxel->val = val;
				if( ddf.bBuildColorField )
					tsfmed_voxel->clr = ddf.clr_field(i, j, k);
				vnl_vector_fixed<float, 3> drt_src = ddf.drts(i, j,k);
				float drt_mag = drt_src.magnitude();
				drt_src.normalize();
				transform_one_normal_with_DeformGraph(drt_src, tsfmed_voxel->drt, graph, ngn);
				tsfmed_voxel->drt *= drt_mag;

				tsfmedVoxelList_all.push_back(tsfmed_voxel);

				for(int k2 = ceill(k_d-radius); k2<=floorl(k_d+radius); k2++)
				for(int j2 = ceill(j_d-radius); j2<=floorl(j_d+radius); j2++)
				for(int i2 = ceill(i_d-radius); i2<=floorl(i_d+radius); i2++)
				{
					if( k2 < 0 || k2 >= nz_d ||
						j2 < 0 || j2 >= ny_d ||
						i2 < 0 || i2 >= nx_d )
						continue;

					tsfmedVoxelList_Mat(i2, j2, k2).push_back(tsfmed_voxel);
				}

			}//end of for-i
		}//end of for-j
	}//end of for-k

	LOGGER()->trace("src->dst = %f\n", __toc__());

	__tic__();
	float thres_dist_nn = xres/xres_d;
	//for each voxel in the destination, check its source voxels
	for(int k=2; k<nz_d-2; k++)
	{
		for(int j=2; j<ny_d-2; j++)
		{			
#pragma omp parallel for schedule(dynamic)
			for(int i=2; i<nx_d-2; i++)
			{
				if( tsfmedVoxelList_Mat(i, j, k).size() == 0 )
					continue;

				TransformedVoxelList &tsfmed_voxel_list = tsfmedVoxelList_Mat(i, j, k);

				//select the one with the smaller absolute sdf value, but prefer negative value
				int idx_selected = -1;
				float value_selected = 100.0;
				for(int c=0; c<tsfmed_voxel_list.size(); c++)
				{
					float val_cur = tsfmed_voxel_list[c]->val;

					if( abs(val_cur) < abs(value_selected) )
					{
						value_selected = val_cur;
						idx_selected = c;
					}
				}

				if( idx_selected == -1 )
					continue;

				int* coord_ori_slcted = tsfmed_voxel_list[idx_selected]->coord_ori;
				//find all the source voxels around the selected voxel
				vnl_vector_fixed<float, 3> drt_dst(0.0, 0.0, 0.0);
				vnl_vector_fixed<float, 3> clr_dst(0.0, 0.0, 0.0);
				float weight_dst = 0;
				float val_dst = 0;
				float coef_total = 0;

				int idx_min_dist = -1;
				float min_dist = 1.0e+10;
				for(int c=0; c<tsfmed_voxel_list.size(); c++)
				{					
					TransformedVoxel *tsfmed_voxel = tsfmed_voxel_list[c];
					if( abs(tsfmed_voxel->coord_ori[0]-coord_ori_slcted[0]) > 4 ||
						abs(tsfmed_voxel->coord_ori[1]-coord_ori_slcted[1]) > 4 ||
						abs(tsfmed_voxel->coord_ori[2]-coord_ori_slcted[2]) > 4 )
						continue;

					float dist = (tsfmed_voxel->coord_t[0]-i)*((tsfmed_voxel->coord_t[0]-i)) + 
									(tsfmed_voxel->coord_t[1]-j)*((tsfmed_voxel->coord_t[1]-j)) +
									(tsfmed_voxel->coord_t[2]-k)*((tsfmed_voxel->coord_t[2]-k));

					if( dist < min_dist )
					{
						min_dist = dist;
						idx_min_dist = c;
					}
						
					float coef = exp(-dist*3 );

					drt_dst += coef * tsfmed_voxel->drt;
					weight_dst += coef * ddf.weights(tsfmed_voxel->coord_ori[0], tsfmed_voxel->coord_ori[1], tsfmed_voxel->coord_ori[2]);
					val_dst += coef * tsfmed_voxel->val;	
					clr_dst += coef * tsfmed_voxel->clr;
					coef_total += coef;
				}

				if( idx_min_dist == -1 )
					continue;
				//earlier: if( min_dist/3 > 0.52*0.52*thres_dist_nn*thres_dist_nn) // to avoid the expansion of the surface at open boundaries
				if( min_dist > 0.56*thres_dist_nn*thres_dist_nn ) // 0.56 to avoid the expansion of the surface at open boundaries					
					continue;

				if( interp_method == SDFTransfInterp_WeightedAvg )
				{
					drt_dst /= coef_total;
					weight_dst /= coef_total;
					val_dst /= coef_total;
					clr_dst /= coef_total;
					ddf_dst.func_val(i, j, k) = val_dst;
					ddf_dst.weights(i, j, k) = weight_dst;
					ddf_dst.drts(i, j, k) = drt_dst;
					if( ddf_dst.bBuildColorField)
						ddf_dst.clr_field(i, j, k) = tsfmed_voxel_list[idx_min_dist]->clr; //donot blur color
				
				}
				else if( interp_method == SDFTransfInterp_NN )
				{
					int* voxel_coord_src = tsfmed_voxel_list[idx_min_dist]->coord_ori;
					ddf_dst.func_val(i, j, k) = tsfmed_voxel_list[idx_min_dist]->val;
					ddf_dst.weights(i, j, k) = ddf.weights(voxel_coord_src[0], voxel_coord_src[1], voxel_coord_src[2]);
					ddf_dst.drts(i, j, k) = tsfmed_voxel_list[idx_min_dist]->drt;
					if( ddf_dst.bBuildColorField)
						ddf_dst.clr_field(i, j, k) = tsfmed_voxel_list[idx_min_dist]->clr;
				}
				else
				{
					throw exception("unsupported SDF Transformation Type!");
				}				

			}//end of for-i
		}//end of for-j
	}//end of for-k

	LOGGER()->trace("dst calc = %f\n", __toc__());

	for(int i=0; i<tsfmedVoxelList_all.size(); i++)
	{
		delete tsfmedVoxelList_all[i];
	}

}

bool calc_NeighborGraphNodesOfPoint_Surface( vnl_vector_fixed<double, 3> const &p, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface const&geo_dist,
											 NeighborGraphNodesOfPoint &ngn,
											 int k, double max_dist)
{
	float *vtData = surface.vtData;
	int vtDim = surface.vtDim;

	float pt[3];
	pt[0] = p[0]; pt[1] = p[1]; pt[2] = p[2];
	//find the nearest vertex on the surface
	int vtIdx_nearest = -1;
	double dist_to_near_vert = 1.0e+20;
	for(int i=0; i<surface.vtNum; i++)
	{
		float *pVt = &(vtData[i*vtDim]);
		if( pVt[0] < p[0]-max_dist || pVt[0] > p[0]+max_dist ||
			pVt[1] < p[1]-max_dist || pVt[1] > p[1]+max_dist ||
			pVt[2] < p[2]-max_dist || pVt[2] > p[2]+max_dist )
			continue;

		double dist = VecOperation<float>::Distance(pt, pVt, 3);
		if( dist < dist_to_near_vert )
		{
			dist_to_near_vert = dist;
			vtIdx_nearest = i;
		}
	}

	if( vtIdx_nearest == -1 )
	{
		ngn.neighborIndices.clear();
		ngn.weights.clear();
		LOGGER()->warning("calc_NeighborGraphNodesOfPoint_Surface","no neighbor found for ngn!");
		return false;
	}

	return calc_NeighborGraphNodesOfPoint_Surface( vtIdx_nearest, graph, 
												   surface, geo_dist, ngn, k, max_dist);
}

bool calc_NeighborGraphNodesOfPoint_Surface( int vtIdx, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface const&geo_dist,
											 NeighborGraphNodesOfPoint &ngn,
											 int k, double max_dist)
{
	ngn.neighborIndices.clear();
	ngn.weights.clear();

	//find the nearest graph nodes
	vector<DoubleIndexPair> dist_vec;
	for(int i = 0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode const& node = graph.nodes[i];
		int vtIdx_dgn = node.vtIdx;
		assert(vtIdx_dgn >= 0);

		double dist = geo_dist(vtIdx_dgn, vtIdx);
		if( dist < 0.0 || dist > max_dist)
			continue;

		dist_vec.push_back(make_pair(dist, i));
	}
	if( dist_vec.size() <= 0)
		return false;

	//sort the node in the ascending order of their distance
	sort(dist_vec.begin(), dist_vec.end(), less_comparator_DoubleIndexPair);

	//pick the distance of k+1-th node
	double d_max = 0.0;
	if(dist_vec.size() > k)
		d_max = dist_vec[k].first;
	else
		d_max = dist_vec[dist_vec.size()-1].first + 5.0;

	double weights_all = 0;
	for(int i=0; i<MIN(k, dist_vec.size()); i++)
	{
		int idx_dgn = dist_vec[i].second;
		
		double w_dgn = 1.0-dist_vec[i].first/d_max;
		w_dgn = w_dgn*w_dgn;

		ngn.neighborIndices.push_back(idx_dgn);
		ngn.weights.push_back(w_dgn);
		weights_all += w_dgn;
	}

	assert(weights_all > 0.0);

	for(int i=0; i<ngn.weights.size(); i++)
	{
		ngn.weights[i] /= weights_all;
	}

	return true;
}

bool calc_NeighborGraphNodesOfPoint_Surface( vector< vnl_vector_fixed<double, 3> > const& points, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface &geo_dist,
											 vector< NeighborGraphNodesOfPoint > &ngns,
											 int k, double max_dist)
{
	if( !geo_dist.bAssociatedWithSurface )
		geo_dist.associate_with_surface(surface);

	//first calc the Dijkstra distance for nodes on the deformation graph
	for(int i=0; i<graph.nodes.size(); i++)
	{
		int vtIdx = graph.nodes[i].vtIdx;
		assert(vtIdx >= 0);

		geo_dist.calc_dijkstra_dist_for_one_vertex(vtIdx, max_dist);
	}

	ngns.clear();
	for(int i=0; i<points.size(); i++)
	{
		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_Surface(points[i], graph, surface, geo_dist, ngn, k, max_dist);

		ngns.push_back(ngn);
	}

	return true;
}
//calc NeighborGraphNodesOfPoint for all surface vertices
bool calc_NeighborGraphNodesOfPoint_Surface_all_vts( DeformGraph const &graph, 
													 CSurface<float> const& surface,
													 CGeodesicDistanceOnSurface &geo_dist,
													 vector< NeighborGraphNodesOfPoint > &ngns,
													 int k, double max_dist)
{
	if( !geo_dist.bAssociatedWithSurface )
		geo_dist.associate_with_surface(surface);

	//calc the Dijkstra distance for nodes on the deformation graph
	
#pragma omp parallel for
	for(int i=0; i<graph.nodes.size(); i++)
	{
		LOGGER()->trace("compute CGeodist on graph %04d\r", i);
		int vtIdx = graph.nodes[i].vtIdx;
		assert(vtIdx >= 0);

		geo_dist.calc_dijkstra_dist_for_one_vertex(vtIdx, max_dist);
	}

	ngns.clear();
	ngns.resize(surface.vtNum);
#pragma omp parallel for
	for(int i=0; i<surface.vtNum; i++)
	{
		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_Surface(i, graph, surface, geo_dist, ngn, k, max_dist);

		ngns[i] = ngn;
	}

	return true;
}


void calc_NeighborGraphNodesOfPoint_vertex_matching( CSurface<float> const& surface_template,
														  NeighborGraphNodesOfPointList const& ngns_template, 
														  CSurface<float> const& surface,
														  NeighborGraphNodesOfPointList &ngns,
														  double thres_vert_dist)
{
	ngns.resize(surface.vtNum);

#pragma omp parallel for
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		float const* vt = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<float, 3> p(vt);
		vnl_vector_fixed<float, 3> q;
		int index=0;
		closest_point_on_surface(surface_template, p, q, index); 
		if( dist_3d(p, q) < thres_vert_dist )
			ngns[vtIdx] = ngns_template[index];
		else
			ngns[vtIdx] = NeighborGraphNodesOfPoint();
	}

}

void calc_NeighborGraphNodesOfPoint_vertex_matching2( CSurface<float> const& surface_template,
													  NeighborGraphNodesOfPointList const& ngns_template, 
													  DeformGraph const& graph,
													  CSurface<float> const& surface,
													  NeighborGraphNodesOfPointList &ngns,
													  double thres_vert_dist)
{
	GraphNodesTerritory nodes_territories;
	calc_deform_graph_node_territory(graph, ngns_template, nodes_territories, true);

	ngns.resize(surface.vtNum);

#pragma omp parallel for
	for(int vtIdx=0; vtIdx<surface.vtNum; vtIdx++)
	{
		float const* vt_ = surface.vt_data_block(vtIdx);
		vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

		//find closest graph node index
		int ndIdx_min=-1;
		double dist_min = 100;
		for(int ndIdx=0; ndIdx<graph.nodes.size(); ndIdx++)
		{
			double dist = dist_3d(graph.nodes[ndIdx].g, vt);
			if( dist < dist_min )
			{
				ndIdx_min = ndIdx;
				dist_min = dist;
			}
		}	
		if( ndIdx_min == -1)
		{
			ngns[vtIdx] = NeighborGraphNodesOfPoint();
			continue;
		}

		//find nearby graph nodes
		vector<int> nearby_nodes_indices;
		nearby_nodes_indices.push_back(ndIdx_min);
		for(int ndIdx=0; ndIdx<graph.nodes.size(); ndIdx++)
		{
			double dist = dist_3d(graph.nodes[ndIdx].g, vt);
			if( dist < dist_min*1.5 )
				nearby_nodes_indices.push_back(ndIdx);
		}
		
		//find closest vertex indices
		int vtIdx_min = -1;
		double vtDist_min = thres_vert_dist*thres_vert_dist;
		for(int i=0; i<nearby_nodes_indices.size(); i++)
		{
			int ndIdx = nearby_nodes_indices[i];
			vector<int> const&vt_indices = nodes_territories[ndIdx];
			for(int j=0; j<vt_indices.size(); j++)
			{
				int vtIdx_t = vt_indices[j];
				float const* vt_t = surface_template.vt_data_block(vtIdx_t);
				double dist = (vt[0]-vt_t[0])*(vt[0]-vt_t[0]) + 
							  (vt[1]-vt_t[1])*(vt[1]-vt_t[1]) +
							  (vt[2]-vt_t[2])*(vt[2]-vt_t[2]);
				if( dist < vtDist_min )
				{
					vtDist_min = dist;
					vtIdx_min = vtIdx_t;
				}
			}
		}
		if( vtIdx_min == -1)
			ngns[vtIdx] = NeighborGraphNodesOfPoint();			
		else
			ngns[vtIdx] = ngns_template[vtIdx_min];
	}

}





























}