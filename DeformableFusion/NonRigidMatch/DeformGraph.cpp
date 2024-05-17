#include "DeformGraph.h"
namespace NonrigidMatching{

//compute incremental vectors
void calc_deform_graph_inc_vector( DeformGraph const& graph1, DeformGraph const& graph2,
								   vector< vnl_vector_fixed<double, 3> > & inc_vecs)
{
	assert( graph1.nodes.size() == graph2.nodes.size() );
	inc_vecs.clear();

	for(int i=0; i<graph1.nodes.size(); i++)
	{
		vnl_vector_fixed<double, 3> vec = graph2.nodes[i].t - graph1.nodes[i].t;
		inc_vecs.push_back(vec);
	}
}

void add_inc_vector_on_deform_graph(DeformGraph &graph, vector< vnl_vector_fixed<double, 3> > const& inc_vecs, double c)
{
	assert( graph.nodes.size() == inc_vecs.size() );

	for(int i=0; i<inc_vecs.size(); i++)
		graph.nodes[i].t += inc_vecs[i]*c;
}

//initialize the current graph based on previous graph
bool deform_grpah_initialization_DIST(DeformGraph &graph, DeformGraph const& graph_prev)
{
	if( graph_prev.nodes.size() <= 0)
		return false;

	graph.global_rigid = graph_prev.global_rigid;
	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode &node = graph.nodes[i];
		vnl_vector_fixed<double, 3> &g = node.g;

		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_by_dist(g, graph_prev, ngn, 4);
		
		vnl_matrix_fixed<double, 3, 3> A;
		A.fill(0.0);
		vnl_vector_fixed<double, 3> t;
		t.fill(0.0);
		for(int j=0; j<ngn.weights.size(); j++)
		{
			int idx = ngn.neighborIndices[j];
			double w = ngn.weights[j];
			A += graph_prev.nodes[idx].A * w;
			t += graph_prev.nodes[idx].t * w;
		}
		matrix_orthogonalize(A);
		node.A = A;
		node.t = t;
	}
	return true;
}


bool identify_illConditioned_DeformGraphNode( DeformGraph const& graph, 
											   vector< NeighborGraphNodesOfPoint > const& ngns_key_points, 
											   vector<int> &bad_nodes_list)
{
	int nodes_num = graph.nodes.size();
	vector<bool> bFlag(nodes_num);
	for(int i=0; i<nodes_num; i++)
		bFlag[i] = false;
	
	//set nodes with directly connection to key points as good nodes
	for(int i=0; i<ngns_key_points.size(); i++)
	{
		vector<int> const& neighbor_indices = ngns_key_points[i].neighborIndices;
		for(int j=0; j<neighbor_indices.size(); j++)
		{
			int idx = neighbor_indices[j];
			assert(idx < nodes_num );
			bFlag[idx] = true;
		}
	}

	bool bNodesFlagChanged = true;
	while( bNodesFlagChanged )
	{
		bNodesFlagChanged = false;
		for(int i=0; i<nodes_num; i++)
		{
			vector<int> const& neighbors = graph.nodes[i].neighborIndices;
			if( bFlag[i] )
			{
				for(int j=0; j<neighbors.size(); j++)
				{
					int idx = neighbors[j];
					if(bFlag[idx])
						continue;

					bFlag[idx] = true;
					bNodesFlagChanged = true;
				}
			}
			else
			{
				for(int j=0; j<neighbors.size(); j++)
				{
					int idx = neighbors[j];
					if( bFlag[idx] )
					{
						bFlag[i] = true;
						bNodesFlagChanged = true;
						break;
					}
				}
			}
		}
	}

	bad_nodes_list.clear();
	for(int i=0; i<nodes_num; i++)
	{
		if( !bFlag[i] )
			bad_nodes_list.push_back(i);
	}

	return true;
}

bool add_connections_for_illConditioned_DeformGraphNode(DeformGraph &graph, vector<int> const&bad_nodes_list)
{
	if(graph.nodes.size() == 0 || bad_nodes_list.size() == 0)
		return false;

	int nodes_num = graph.nodes.size();
	vector<bool> bFlag(nodes_num);
	for(int i=0; i<nodes_num; i++)
		bFlag[i] = true;
	for(int i=0; i<bad_nodes_list.size(); i++)
	{
		int idx = bad_nodes_list[i];
		bFlag[idx] = false;
	}

	for(int i=0; i<bad_nodes_list.size(); i++)
	{
		int idx = bad_nodes_list[i];
		DeformGraphNode &node = graph.nodes[idx];
		vnl_vector_fixed<double, 3> &p = node.g;

		//let the bad node connect with the nearest good node
		int nodeIdx_nearest = -1;
		double dist_min = 1.0e+100;
		for(int j=0; j<nodes_num; j++)
		{
			if( !bFlag[j] )
				continue;
			vnl_vector_fixed<double, 3> &q = graph.nodes[j].g;
			double dist = dist_3d(p, q);
			if( dist < dist_min)
			{
				dist_min = dist;
				nodeIdx_nearest = j;
			}
		}

		if( nodeIdx_nearest != -1 )
			node.neighborIndices.push_back(nodeIdx_nearest);
	}

	return true;
}

bool remove_connections_for_illConditioned_DeformGraphNode(DeformGraph &graph, vector<int> const&bad_nodes_list)
{
	if(graph.nodes.size() == 0 || bad_nodes_list.size() == 0)
		return false;

	int nodes_num = graph.nodes.size();
	vector<bool> bFlag(nodes_num);
	for(int i=0; i<nodes_num; i++)
		bFlag[i] = true;
	for(int i=0; i<bad_nodes_list.size(); i++)
	{
		int idx = bad_nodes_list[i];
		bFlag[idx] = false;
	}

	for(int i=0; i<bad_nodes_list.size(); i++)
	{
		int idx = bad_nodes_list[i];
		DeformGraphNode &node = graph.nodes[idx];

		for(int j=0; j<node.neighborIndices.size(); j++)
		{
			//bad nodes can only connected to bad nodes
			int idx = node.neighborIndices[j];
			if( bFlag[idx] )
			{
				node.neighborIndices.erase(node.neighborIndices.begin() + j);
				j--;
			}
		}
	}

	return true;
}


bool saveDeformGraphASCII(DeformGraph const &graph, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if(!fp)
	{
		LOGGER()->error("saveDeformGraphASCII", "Cannot open the file <%s>.", file_name);
		return false;
	}

	fprintf(fp, "Deform Graph Data(#version 3.0)\n");
	fprintf(fp, "Basic Info: node dist=%f, nn_num=%d\n", graph.nodes_dist, graph.nn_max);
	fprintf(fp, "Global Rigid: <%f, %f, %f>, <%f, %f, %f>\n", graph.global_rigid.rod[0], graph.global_rigid.rod[1], graph.global_rigid.rod[2],
															  graph.global_rigid.t[0], graph.global_rigid.t[1], graph.global_rigid.t[2]);
	fprintf(fp, "Number of graph nodes %zd\n", graph.nodes.size());

	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode const &node = graph.nodes[i];
		fprintf(fp, "Graph node %d<%d>: g = <%f %f %f>, n = <%f %f %f>, A = <%.15f %.15f %.15f; %.15f %.15f %.15f; %.15f %.15f %.15f>, t = <%.15f %.15f %.15f>\n",
			node.idx, node.vtIdx, node.g[0], node.g[1], node.g[2], node.n[0], node.n[1], node.n[2],
			node.A[0][0], node.A[0][1], node.A[0][2], node.A[1][0], node.A[1][1], node.A[1][2],
			node.A[2][0], node.A[2][1], node.A[2][2], node.t[0], node.t[1], node.t[2]);
		
		fprintf(fp, "\t\t neighbor indices <%zd>:", node.neighborIndices.size() );
		for(int j=0; j<node.neighborIndices.size(); j++)
			fprintf(fp, " %d", node.neighborIndices[j]);
		fprintf(fp, "\n\n");
	}

	fclose(fp);

	return true;
}


bool loadDeformGraphASCII(DeformGraph &graph, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "r");
	if(!fp)
	{
		LOGGER()->error("loadDeformGraphASCII", "Cannot open the file <%s>.", file_name);
		return false;
	}

	double version = 0.0;
	fscanf(fp, "Deform Graph Data(#version %lf)\n", &version);
	if (version >= 2.0)
	{
		fscanf(fp, "Basic Info: node dist=%lf, nn_num=%lf\n", &(graph.nodes_dist), &(graph.nn_max));
		fscanf(fp, "Global Rigid: <%lf, %lf, %lf>, <%lf, %lf, %lf>\n", &(graph.global_rigid.rod[0]), &(graph.global_rigid.rod[1]), &(graph.global_rigid.rod[2]),
			&(graph.global_rigid.t[0]), &(graph.global_rigid.t[1]), &(graph.global_rigid.t[2]));
	}
	bool bWithNormal = false;
	if (version >= 3.0)
		bWithNormal = true;

	int node_num = 0;
	fscanf(fp, "Number of graph nodes %d\n", &node_num);

	graph.nodes.clear();
	for(int i=0; i<node_num; i++)
	{
		DeformGraphNode node;
		if (bWithNormal)
			fscanf(fp, "Graph node %d<%d>: g = <%lf %lf %lf>, n = <%lf %lf %lf>, A = <%lf %lf %lf; %lf %lf %lf; %lf %lf %lf>, t = <%lf %lf %lf>\n",
			&(node.idx), &(node.vtIdx), &(node.g[0]), &(node.g[1]), &(node.g[2]), &(node.n[0]), &(node.n[1]), &(node.n[2]),
			&(node.A[0][0]), &(node.A[0][1]), &(node.A[0][2]), &(node.A[1][0]), &(node.A[1][1]), &(node.A[1][2]),
			&(node.A[2][0]), &(node.A[2][1]), &(node.A[2][2]), &(node.t[0]), &(node.t[1]), &(node.t[2]));
		else
			fscanf(fp, "Graph node %d<%d>: g = <%lf %lf %lf>, A = <%lf %lf %lf; %lf %lf %lf; %lf %lf %lf>, t = <%lf %lf %lf>\n",
						&(node.idx), &(node.vtIdx), &(node.g[0]), &(node.g[1]), &(node.g[2]),
						&(node.A[0][0]), &(node.A[0][1]), &(node.A[0][2]), &(node.A[1][0]), &(node.A[1][1]), &(node.A[1][2]),
						&(node.A[2][0]), &(node.A[2][1]), &(node.A[2][2]), &(node.t[0]), &(node.t[1]), &(node.t[2]));
		int neibors_num = 0;
		fscanf(fp, "\t\t neighbor indices <%d>:", &neibors_num );
		for(int j=0; j<neibors_num; j++)
		{
			int neighborIdx = 0;
			fscanf(fp, " %d", &neighborIdx);
			node.neighborIndices.push_back(neighborIdx);
		}
		fscanf(fp, "\n\n");

		graph.nodes.push_back(node);
	}

	fclose(fp);
	return true;
}


bool saveDeformGraphBIN(DeformGraph const &graph, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "wb");
	if(!fp)
	{
		LOGGER()->error("saveDeformGraphBIN", "Cannot open the file <%s>.", file_name);
		return false;
	}

	fwrite("#DeformGraph#", 1, 13, fp);
	fwrite(&graph.nodes_dist, sizeof(double), 1, fp);
	fwrite(&graph.nn_max, sizeof(int), 1, fp);
	fwrite(graph.global_rigid.rod.data_block(), sizeof(double), 3, fp);
	fwrite(graph.global_rigid.t.data_block(), sizeof(double), 3, fp);
	int node_num = graph.nodes.size();
	fwrite(&node_num, sizeof(int), 1, fp);
	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode const &node = graph.nodes[i];
		fwrite(&node.idx, sizeof(int), 1, fp);
		fwrite(&node.vtIdx, sizeof(int), 1, fp);
		fwrite(node.g.data_block(), sizeof(double), 3, fp);
		fwrite(node.n.data_block(), sizeof(double), 3, fp);
		fwrite(node.A.data_block(), sizeof(double), 9, fp);
		fwrite(node.t.data_block(), sizeof(double), 3, fp);
		int neightbor_num = node.neighborIndices.size();
		fwrite(&neightbor_num, sizeof(int), 1, fp);
		for(int k=0; k<neightbor_num; k++)
		{
			fwrite(&node.neighborIndices[k], sizeof(int), 1, fp);
		}
	}

	fclose(fp);
	return true;
}

bool loadDeformGraphBIN(DeformGraph &graph, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if(!fp)
	{
		LOGGER()->error("loadDeformGraphBIN", "Cannot open the file <%s>.", file_name);
		return false;
	}
	char buf[14];
	memset(buf, 0, 14);
	fread(buf, 1, 13, fp);
	if( strcmp(buf, "#DeformGraph#") != 0 )
	{
		LOGGER()->error("loadDeformGraphBIN", "File format doesnot match!\n");
		return false;
	}

	fread(&graph.nodes_dist, sizeof(double), 1, fp);
	fread(&graph.nn_max, sizeof(int), 1, fp);
	fread(graph.global_rigid.rod.data_block(), sizeof(double), 3, fp);
	fread(graph.global_rigid.t.data_block(), sizeof(double), 3, fp);

	graph.nodes.clear();
	int node_num;
	fread(&node_num, sizeof(int), 1, fp);
	for(int i=0; i<node_num; i++)
	{
		DeformGraphNode node;
		fread(&node.idx, sizeof(int), 1, fp);
		fread(&node.vtIdx, sizeof(int), 1, fp);
		fread(node.g.data_block(), sizeof(double), 3, fp);
		fread(node.n.data_block(), sizeof(double), 3, fp);
		fread(node.A.data_block(), sizeof(double), 9, fp);
		fread(node.t.data_block(), sizeof(double), 3, fp);
		int neightbor_num;
		fread(&neightbor_num, sizeof(int), 1, fp);
		for(int k=0; k<neightbor_num; k++)
		{
			int idx;
			fread(&idx, sizeof(int), 1, fp);
			node.neighborIndices.push_back(idx);
		}
		graph.nodes.push_back(node);
	}

	fclose(fp);
	return true;
}

bool texture_surface_from_ngn(CSurface<float> &surface, NeighborGraphNodesOfPointList const& ngns, std::vector<vnl_vector_fixed<double, 3>> const& clrs_ed_nodes)
{
	if (surface.vtNum != ngns.size())
	{
		LOGGER()->warning("texture_surface_from_ngn", "vertex number does not match with ngns!\n");
		return false;
	}

	surface.expand_data(true, false);
	for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
	{
		NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];

		vnl_vector_fixed<double, 3> clr_t(0.0);
		for (int k = 0; k < ngn.neighborIndices.size(); k++)
		{
			int ndIdx = ngn.neighborIndices[k];
			clr_t += ngn.weights[k] * clrs_ed_nodes[ndIdx];
		}

		float *clr = surface.vt_color(vtIdx);
		clr[0] = clr_t[0];
		clr[1] = clr_t[1];
		clr[2] = clr_t[2];
	}
	return true;
}

bool calc_NeighborGraphNodesOfPoint_by_dist( vnl_vector_fixed<double, 3> const &p, DeformGraph const &graph, 
											 NeighborGraphNodesOfPoint &ngn,
											 int k)
{
	ngn.neighborIndices.clear();
	ngn.weights.clear();

	assert(k<graph.nodes.size());
	vector<DoubleIndexPair> dist_vec;
	for(int i=0; i<graph.nodes.size(); i++)
	{
		vnl_vector_fixed<double, 3> const &g = graph.nodes[i].g;
		double d = dist_3d(p, g);
		if (d > MIN(30.0, MAX(20.0, 4.0*graph.nodes_dist)))
			continue;

		dist_vec.push_back(DoubleIndexPair(d, i));
	}
	sort(dist_vec.begin(), dist_vec.end(), less_comparator_DoubleIndexPair);
	
	ngn.neighborIndices.clear();
	ngn.weights.clear();

	if (dist_vec.size() > 0)
	{
		k = MIN(k, dist_vec.size());

		double w_sum = 0;
		for (int i = 0; i < k; i++)
		{
			int idx = dist_vec[i].second;

			double w = exp(-dist_vec[i].first*dist_vec[i].first / (graph.nodes_dist*graph.nodes_dist/4.0f));
			ngn.neighborIndices.push_back(idx);
			ngn.weights.push_back(w);
			w_sum += w;
		}
		for (int i = 0; i < k; i++)
			ngn.weights[i] /= w_sum;
	}

	return true;
}

bool calc_NeighborGraphNodesOfPoint_by_dist( vector< vnl_vector_fixed<double, 3> > const &points, DeformGraph const &graph, 
											 vector< NeighborGraphNodesOfPoint > &ngns,
											 int k )
{
	ngns.clear();
	for(int i=0; i<points.size(); i++)
	{
		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_by_dist(points[i], graph, ngn, k);
		ngns.push_back(ngn);
	}

	return true;
}

bool save_NeighborGraphNodesOfPoint_ASCII(vector< NeighborGraphNodesOfPoint > const &ngns, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "w");
	if(!fp)
	{
		LOGGER()->error("save_NeighborGraphNodesOfPoint_ASCII", "Cannot open the file <%s>.", file_name);
		return false;
	}

	fprintf(fp, "Neighbor Graph Nodes Of Points(#version 1.0)\n");
	fprintf(fp, "Number of items %zd\n", ngns.size());

	for(int i=0; i<ngns.size(); i++)
	{
		NeighborGraphNodesOfPoint const &ngn = ngns[i];
		fprintf(fp, "Item %d<%zd>:", i, ngn.neighborIndices.size());
		
		fprintf(fp, " neighbors = <");
		for(int j=0; j<ngn.neighborIndices.size(); j++)
			fprintf(fp, "%d ", ngn.neighborIndices[j]);
		fprintf(fp, ">\n");
		
		fprintf(fp, "\t\t weights = <");
		for(int j=0; j<ngn.weights.size(); j++)
			fprintf(fp, "%f ", ngn.weights[j]);
		fprintf(fp, ">\n\n");
	}

	fclose(fp);
	return true;
}
bool load_NeighborGraphNodesOfPoint_ASCII(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "r");
	if(!fp)
	{
		LOGGER()->error("load_NeighborGraphNodesOfPoint_ASCII", "Cannot open the file <%s>.", file_name);
		return false;
	}
	
	double version = 0;
	fscanf(fp, "Neighbor Graph Nodes Of Points(#version %lf)\n", &version);
	int num = 0;
	fscanf(fp, "Number of items %d\n", &num);

	ngns.clear();
	for(int i=0; i<num; i++)
	{
		NeighborGraphNodesOfPoint ngn;
		int junk = 0;
		int neighbor_num = 0;
		fscanf(fp, "Item %d<%d>:", &junk, &neighbor_num);
		
		fscanf(fp, " neighbors = <");
		for(int j=0; j<neighbor_num; j++)
		{
			int idx=0;
			fscanf(fp, "%d ", &idx);
			ngn.neighborIndices.push_back(idx);
		}
		fscanf(fp, ">\n");
		
		fscanf(fp, "\t\t weights = <");
		for(int j=0; j<neighbor_num; j++)
		{
			double w = 0;
			fscanf(fp, "%lf ", &w);
			ngn.weights.push_back(w);
		}
		fscanf(fp, ">\n\n");

		ngns.push_back(ngn);
	}

	fclose(fp);
	return true;
}

bool save_NeighborGraphNodesOfPoint_BIN(vector< NeighborGraphNodesOfPoint > const &ngns, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "wb");
	if(!fp)
	{
		LOGGER()->error("save_NeighborGraphNodesOfPoint_BIN", "Cannot open the file <%s>.", file_name);
		return false;
	}

	fwrite("#NeighborGraphNodesOfPointList#", 1, 31, fp);

	int ngns_num = ngns.size();
	fwrite(&ngns_num, sizeof(int), 1, fp);

	for(int i=0; i<ngns.size(); i++)
	{
		NeighborGraphNodesOfPoint const &ngn = ngns[i];
		int neighbor_num = ngn.neighborIndices.size();
		fwrite(&neighbor_num, sizeof(int), 1, fp);
		for(int k=0; k<neighbor_num; k++)
			fwrite(&ngn.neighborIndices[k], sizeof(int), 1, fp);
		for(int k=0; k<neighbor_num; k++)
			fwrite(&ngn.weights[k], sizeof(double), 1, fp);
	}

	fclose(fp);
	return true;
}

bool load_NeighborGraphNodesOfPoint_BIN(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if(!fp)
	{
		LOGGER()->error("load_NeighborGraphNodesOfPoint_BIN", "Cannot open the file <%s>.", file_name);
		return false;
	}

	char buf[32];
	memset(buf, 0, 32);
	fread(buf, 1, 31, fp);
	if( strcmp(buf, "#NeighborGraphNodesOfPointList#") != 0 )
	{
		LOGGER()->error("load_NeighborGraphNodesOfPoint_BIN", "file format doesnot match!\n");
		return false;
	}

	ngns.clear();

	int ngns_num;
	fread(&ngns_num, sizeof(int), 1, fp);
	ngns.clear();
	ngns.resize(ngns_num);

	for(int i=0; i<ngns_num; i++)
	{
		NeighborGraphNodesOfPoint ngn;
		int neighbor_num;
		fread(&neighbor_num, sizeof(int), 1, fp);
		for(int k=0; k<neighbor_num; k++)
		{
			int idx;
			fread(&idx, sizeof(int), 1, fp);
			ngn.neighborIndices.push_back(idx);
		}
		for(int k=0; k<neighbor_num; k++)
		{
			double weight;
			fread(&weight, sizeof(double), 1, fp);
			ngn.weights.push_back(weight);
		}
		ngns[i] = ngn;
	}

	fclose(fp);
	return true;
}


bool load_NeighborGraphNodesOfPoint_BIN_Float(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name)
{
	FILE *fp = NULL;
	fopen_s(&fp, file_name, "rb");
	if (!fp)
	{
		LOGGER()->error("load_NeighborGraphNodesOfPoint_BIN", "Cannot open the file <%s>.", file_name);
		return false;
	}

	char buf[32];
	memset(buf, 0, 32);
	fread(buf, 1, 31, fp);
	if (strcmp(buf, "#NeighborGraphNodesOfPointList#") != 0)
	{
		LOGGER()->error("load_NeighborGraphNodesOfPoint_BIN", "file format doesnot match!\n");
		return false;
	}

	ngns.clear();

	int ngns_num;
	fread(&ngns_num, sizeof(int), 1, fp);
	ngns.clear();
	ngns.resize(ngns_num);

	//#pragma omp parallel for
	for (int i = 0; i<ngns_num; i++)
	{
		NeighborGraphNodesOfPoint ngn;
		int neighbor_num;
		fread(&neighbor_num, sizeof(int), 1, fp);
		for (int k = 0; k<neighbor_num; k++)
		{
			int idx;
			fread(&idx, sizeof(int), 1, fp);
			ngn.neighborIndices.push_back(idx);
		}
		for (int k = 0; k<neighbor_num; k++)
		{
			float weight;
			fread(&weight, sizeof(float), 1, fp);
			ngn.weights.push_back(weight);
		}
		ngns[i] = ngn;
	}

	fclose(fp);
	return true;
}

/* for each graph node, find the surface patch which is dominated by this node
*/
bool calc_deform_graph_node_territory(DeformGraph const& graph,
	NeighborGraphNodesOfPointList const& ngns,
	GraphNodesTerritory &nodes_terriotry, //index of the vertices
	bool bNgnOrdered //indicate whether neighboring graph nodes are ordered by their weights for each vertex
	)
{
	if (graph.nodes.size() == 0)
	{
		LOGGER()->warning("calc_deform_graph_node_territory", "graph is empty!\n");
		return false;
	}
	if (ngns.size() == 0)
	{
		LOGGER()->warning("calc_deform_graph_node_territory", "ngns is empty!\n");
		return false;
	}

	int nodes_num = graph.nodes.size();
	nodes_terriotry.clear();
	nodes_terriotry.resize(nodes_num);
	for (int vtIdx = 0; vtIdx<ngns.size(); vtIdx++)
	{
		NeighborGraphNodesOfPoint const& ngn = ngns[vtIdx];
		if (ngn.neighborIndices.size() == 0)
			continue;

		if (bNgnOrdered)
		{
			int nodeIdx = ngn.neighborIndices[0];
			nodes_terriotry[nodeIdx].push_back(vtIdx);
		}
		else
		{
			//find the neighboring nodes with the biggest weight
			int idx_max = 0;
			double w_max = ngn.weights[0];
			for (int i = 1; i<ngn.neighborIndices.size(); i++)
			{
				if (ngn.weights[i] > w_max)
				{
					w_max = ngn.weights[i];
					idx_max = i;
				}
			}

			int nodeIdx = ngn.neighborIndices[idx_max];
			nodes_terriotry[nodeIdx].push_back(vtIdx);
		}
	}

	return true;
}

bool transform_one_point_with_DeformGraph( vnl_vector_fixed<double, 3> const &p, vnl_vector_fixed<double, 3> &q,
											DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn, bool bApplyGlobalRigid)
{
	if( graph.nodes.size() == 0 || ngn.neighborIndices.size() == 0)
	{
		q = p;
		return false;
	}

	q.fill(0.0);
	for(int i=0; i<ngn.neighborIndices.size(); i++)
	{
		int idx = ngn.neighborIndices[i];
		double w = ngn.weights[i];
		DeformGraphNode const &node = graph.nodes[idx];

		q += w*(node.A*(p-node.g) + node.g + node.t);
	}

	if ( bApplyGlobalRigid )
		q = graph.global_rigid.rotation()*q + graph.global_rigid.t;

	return true;
}

bool transform_one_normal_with_DeformGraph( vnl_vector_fixed<double, 3> const &n, vnl_vector_fixed<double, 3> &n_,
										    DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn)
{
	if( graph.nodes.size() == 0 || ngn.neighborIndices.size() == 0 )
	{
		n_ = n;
		return false;
	}

	n_.fill(0.0);
	for(int i=0; i<ngn.neighborIndices.size(); i++)
	{
		int idx = ngn.neighborIndices[i];
		double w = ngn.weights[i];
		DeformGraphNode const &node = graph.nodes[idx];
		if( !graph.bAinvFilled )
			n_ += w* (vnl_inverse(node.A).transpose())*n;
		else
			n_ += w* node.A_inv.transpose() * n;
	}
	n_ = graph.global_rigid.rotation()*n_;

	n_.normalize();

	return true;
}

bool transform_one_normal_with_DeformGraph( vnl_vector_fixed<float, 3> const &n, vnl_vector_fixed<float, 3> &n_,
										    DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn)
{
	vnl_vector_fixed<double, 3> n_d(n[0], n[1], n[2]);
	vnl_vector_fixed<double, 3> n_t_d;
	bool ret = transform_one_normal_with_DeformGraph(n_d, n_t_d, graph, ngn);
	n_[0] = n_t_d[0];
	n_[1] = n_t_d[1];
	n_[2] = n_t_d[2];

	return ret;
}

bool transform_points_with_DeformGraph( vector< vnl_vector_fixed<double, 3> > const &points_t, 
										vector< vnl_vector_fixed<double, 3> > &points_d,
										DeformGraph const &graph, 
										vector< NeighborGraphNodesOfPoint > const &ngn)
{
	if (ngn.size() != points_t.size())
	{
		LOGGER()->warning("transform_points_with_DeformGraph","ngns.size!=points.size()!\n");
		points_d = points_t;
		return false;
	}
	points_d.clear();
	for(int i=0; i<points_t.size(); i++)
	{
		vnl_vector_fixed<double, 3> q;
		transform_one_point_with_DeformGraph(points_t[i], q, graph, ngn[i]);
		points_d.push_back(q);
	}
	return true;
}

void transform_S3DPointMatchSetIndexed_with_DeformGraph(S3DPointMatchSetIndexed &match_set,
														DeformGraph const*graph_1, //will fill_A_inv; could be NULL
														vector< NeighborGraphNodesOfPoint > const* ngns_1,
														DeformGraph const*graph_2, //will fill_A_inv, could be NULL
														vector< NeighborGraphNodesOfPoint > const* ngns_2,
														bool bDeleteExtraVert)
{
	vector< bool > bTransformed_1(match_set.indices_1.size(), false);
	if (graph_1 != NULL && ngns_1 != NULL)
	{
		for (int i = 0; i < match_set.indices_1.size(); i++)
		{
			int vtIdx1 = match_set.indices_1[i];
			assert(vtIdx1 < ngns_1->size());
			vnl_vector_fixed<double, 3> vq;
			if (transform_one_point_with_DeformGraph(match_set.points_1[i], vq, *graph_1, (*ngns_1)[vtIdx1]))
			{
				match_set.points_1[i] = vq;
				bTransformed_1[i] = true;
			}
		}		
	}

	vector< bool > bTransformed_2(match_set.indices_2.size(), false);
	if (graph_2 != NULL && ngns_2 != NULL)
	{
		for (int i = 0; i < match_set.indices_2.size(); i++)
		{
			int vtIdx2 = match_set.indices_2[i];
			assert(vtIdx2 < ngns_2->size());
			vnl_vector_fixed<double, 3> vq;
			if (transform_one_point_with_DeformGraph(match_set.points_2[i], vq, *graph_2, (*ngns_2)[vtIdx2]))
			{
				match_set.points_2[i] = vq;
				bTransformed_2[i] = true;
			}
		}
	}

	if (bDeleteExtraVert)
	{
		S3DPointMatchSetIndexed match_set_new;
		for (int i = 0; i < match_set.size(); i++)
		{
			if (bTransformed_1[i] && bTransformed_2[2])
			{
				match_set_new.points_1.push_back(match_set.points_1[i]);
				match_set_new.points_2.push_back(match_set.points_2[i]);
				match_set_new.indices_1.push_back(match_set.indices_1[i]);
				match_set_new.indices_2.push_back(match_set.indices_2[i]);
			}
			else
			{
				printf("!");
			}
		}
		match_set = match_set_new;
	}
}


}