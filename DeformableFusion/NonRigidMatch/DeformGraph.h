#ifndef __DEFORMGRAPH_H__
#define __DEFORMGRAPH_H__
//===============================================
//			DeformGraph.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================

#include "UtilVnlMatrix.h"
#include "basic_structure.h"
#include "basic_geometry.h"
#include "TSDF.h"
#include "CSurface.h"
#include <vector>
#include "RigidTransformModel.h"
#include "CPointCloud.h"
#include "DeformGraphCore.h"


namespace NonrigidMatching{


struct NonrigidMatchingPara
{
public:
	NonrigidMatchingPara()
		: w_rot(4000),
		w_reg(200),
		w_zero(0),
		w_constr(0),
		w_constr_manual(0),
		w_intrinsic(0),
		w_sdf(0),
		w_clr(0),
		w_node_dist(0),
		w_intersect_init(0),
		itmax(10),
		vt_clr_downsample_rate(1),
		vt_sdf_downsample_rate(1),
		vt_intrinsic_downsample_rate(1),
		w_zero_surface_update(0.0),
		bUseColorField(false),
		bFixGlobalRigid(false),
		bFixNonrigid(false),
		bUseJacHack(false),
		bLimitSurfaceUpdateAlongNormal(false),
		max_traverse_step(5),
		bPutAllVerticesIntoOneParaBlock(false),
		w_normal(0),
		matched_points_downsample_rate(1),
		w_laplacian(0.0),
		bWeightVertexOnDistance(false),
		w_color_smoothness(0.0),
		bRecordResidual(false),
		w_weights_polar(0.0),
		w_weights_sum_one(0.0),
		w_weights_smooth(0.0),
		bFixEDWeights(true),
		bFixLatentMesh(false),
		w_temporal(0.0),
		w_elastic_isometric(0.0),
		bUseZollhofer(false),
		pt_constr_thres(5.0),
		w_intrinsic_dst(0.0),
		tau_len_ratio(10.0),
		tau_weight_smoothness(0.5),
		vt_keyPts_downsample_rate(1),
		bFixEdgeAlphas(false),
		bUseJacApprox(false)
	{};

public:
	double w_rot;
	double w_reg;
	double w_zero; //rigidness
	double w_sdf;
	double w_clr;
	double w_constr;
	double w_intrinsic;
	double w_node_dist;
	double w_intersect_init;
	int itmax;
	int vt_clr_downsample_rate;
	int vt_sdf_downsample_rate;
	int vt_intrinsic_downsample_rate;
	bool bUseColorField; // use the color information stored in the volumetric data
	bool bUseJacHack;
	bool bFixGlobalRigid;
	bool bFixNonrigid;

	bool bUseJacApprox; //jacobian approximation by make J^T*J be a diagonal block matrix

public:
	//for AWF bundle adjustment
	bool bLimitSurfaceUpdateAlongNormal;
	bool bPutAllVerticesIntoOneParaBlock;
	double w_zero_surface_update;
	double w_normal;
	double max_traverse_step;
	double w_laplacian;
	bool bWeightVertexOnDistance;
	double w_color_smoothness;
	bool bRecordResidual;

	bool bUseZollhofer;

	//for error redistribution
	int matched_points_downsample_rate;
	double w_constr_manual;

	//for affine sets
	double w_weights_polar;
	double w_weights_sum_one;
	double w_weights_smooth;

	//for polymesh-reverse
	bool bFixEDWeights;
	bool bFixLatentMesh;
	double w_temporal;
	double w_elastic_isometric;
	double w_intrinsic_dst;
	double tau_len_ratio; //threshold for isometric term: tau_len_ratio*len_avg 
	double tau_weight_smoothness;
	bool bFixEdgeAlphas;

	int vt_keyPts_downsample_rate;
	double pt_constr_thres;

};

inline void releaseDeformGraphs( vector<DeformGraph*> &graphs)
{
	for(int i=0; i<graphs.size(); i++)
	{
		if( graphs[i] != NULL )
			delete graphs[i];
	}
	graphs.clear();
}

bool saveDeformGraphASCII(DeformGraph const &graph, const char* filename);
bool saveDeformGraphBIN(DeformGraph const &graph, const char* filename);
bool loadDeformGraphASCII(DeformGraph &graph, const char* filename);
bool loadDeformGraphBIN(DeformGraph &graph, const char* filename);

inline void releaseNeighborGraphNodesOfPointList( std::vector<NeighborGraphNodesOfPointList*> &ngns)
{
	for(int i=0; i<ngns.size(); i++)
	{
		if( ngns[i] != NULL )
			delete ngns[i];
	}
	ngns.clear();
}

bool texture_surface_from_ngn(CSurface<float> &surface, NeighborGraphNodesOfPointList const& ngns, std::vector<vnl_vector_fixed<double, 3>> const& clrs_ed_nodes);

bool calc_NeighborGraphNodesOfPoint_by_dist( vnl_vector_fixed<double, 3> const &p, DeformGraph const &graph, 
											 NeighborGraphNodesOfPoint &ngn,
											 int k = 4);
bool calc_NeighborGraphNodesOfPoint_by_dist( vector< vnl_vector_fixed<double, 3> > const &points, DeformGraph const &graph, 
											 vector< NeighborGraphNodesOfPoint > &ngns,
											 int k = 4);
bool save_NeighborGraphNodesOfPoint_ASCII(vector< NeighborGraphNodesOfPoint > const &ngns, const char* file_name);
bool load_NeighborGraphNodesOfPoint_ASCII(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name);
bool save_NeighborGraphNodesOfPoint_BIN(vector< NeighborGraphNodesOfPoint > const &ngns, const char* file_name);
bool load_NeighborGraphNodesOfPoint_BIN(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name);
//ngns BIN data saved for CVPR 2015 paper have float type of weights
bool load_NeighborGraphNodesOfPoint_BIN_Float(vector< NeighborGraphNodesOfPoint > &ngns, const char* file_name);


typedef vector< vector<int> > GraphNodesTerritory;
inline void releaseGraphNodesTerritories(std::vector<GraphNodesTerritory*> &nodes_territories)
{
	for (int i = 0; i<nodes_territories.size(); i++)
	{
		if (nodes_territories[i] != NULL)
			delete nodes_territories[i];
	}
	nodes_territories.clear();
}
/* for each graph node, find the surface patch which is dominated by this node
*/
bool calc_deform_graph_node_territory(DeformGraph const& graph,
	NeighborGraphNodesOfPointList const& ngns,
	GraphNodesTerritory &nodes_terriotry, //index of the vertices
	bool bNgnOrdered = true //indicate whether neighboring graph nodes are ordered by their weights for each vertex
	);

template<class T1, class T2>
bool surface_closest_point_fast(vnl_vector_fixed<T1, 3> const&vt_, vnl_vector_fixed<T1, 3> const&n_,
								CSurface<T2> const& surface, DeformGraph const& graph, GraphNodesTerritory const& nodes_territory,
								int &vtIdx_corr,
								double thres_vt_dist = 10.0,
								bool bCheckNormal = true,
								double thres_normal_dif = 45)
{
	const double nn_dist = 6.0;
	const int node_num_choosed = 6;
	double thres_normal_dotprod_dif = std::cos(thres_normal_dif*M_PI / 180.0);

	vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);
	vnl_vector_fixed<double, 3> n(n_[0], n_[1], n_[2]);

	IndexDistList vt_node_dists; //first: node index; second: vt to node dist
	for (int i = 0; i < graph.nodes.size(); i++)
	{
		double dist = dist_3d(vt, graph.nodes[i].g);
		if (dist < thres_vt_dist*2.0 + nn_dist*2.2)
			vt_node_dists.push_back(make_pair(i, dist));
	}
	std::sort(vt_node_dists.begin(), vt_node_dists.end(), less_comparator_IndexDistPair);

	vtIdx_corr = -1;
	double dist_min = 1.0e+10;
	for (int i = 0; i < MIN(node_num_choosed, vt_node_dists.size()); i++)
	{
		int ndIdx = vt_node_dists[i].first;
		vector<int> const vt_list = nodes_territory[ndIdx];
		for (int k = 0; k < vt_list.size(); k++)
		{
			int vtIdx = vt_list[k];
			T2 const* vt2_ = surface.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt2(vt2_[0], vt2_[1], vt2_[2]);

			if (bCheckNormal)
			{
				T2 const* n2_ = surface.vt_normal(vtIdx);
				vnl_vector_fixed<double, 3> n2(n2_[0], n2_[1], n2_[2]);
				if (dot_product(n, n2) < thres_normal_dotprod_dif)
					continue;
			}

			double dist = dist_3d(vt, vt2);
			if (dist < dist_min)
			{
				dist_min = dist;
				vtIdx_corr = vtIdx;
			}
		}
	}

	if (dist_min < thres_vt_dist)
	{
		return true;
	}
	else
	{
		vtIdx_corr = -1;
		return false;
	}
}

struct IntPair
{
public:
	IntPair(int i1_=-1, int i2_=-1)
		: i1(i1_),
		  i2(i2_)
	{;}
public:
	int i1;
	int i2;
};

//compute incremental vectors
void calc_deform_graph_inc_vector( DeformGraph const& graph1, DeformGraph const& graph2,
								   vector< vnl_vector_fixed<double, 3> > & inc_vecs);
void add_inc_vector_on_deform_graph(DeformGraph &graph1, vector< vnl_vector_fixed<double, 3> > const& inc_vecs, double c=1.0);

//initialize the current graph based on previous graph
bool deform_grpah_initialization_DIST(DeformGraph &graph, DeformGraph const& graph_prev);

/* ngns_key_points are the neighboring graph nodes of extracted key points
 */
bool identify_illConditioned_DeformGraphNode( DeformGraph const& graph, 
											   vector< NeighborGraphNodesOfPoint > const& ngns_key_points, 
											   vector<int> &bad_nodes_list);
bool add_connections_for_illConditioned_DeformGraphNode(DeformGraph &graph, vector<int> const&bad_nodes_list);
bool remove_connections_for_illConditioned_DeformGraphNode(DeformGraph &graph, vector<int> const&bad_nodes_list);


bool transform_one_point_with_DeformGraph( vnl_vector_fixed<double, 3> const &p, vnl_vector_fixed<double, 3> &q,
										   DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn, bool bApplyGlobalRigid = true);
bool transform_one_normal_with_DeformGraph( vnl_vector_fixed<double, 3> const &n, vnl_vector_fixed<double, 3> &n_,
										   DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn);
bool transform_one_normal_with_DeformGraph( vnl_vector_fixed<float, 3> const &n, vnl_vector_fixed<float, 3> &n_,
										   DeformGraph const &graph, NeighborGraphNodesOfPoint const &ngn);
bool transform_points_with_DeformGraph( vector< vnl_vector_fixed<double, 3> > const &points_t, 
										vector< vnl_vector_fixed<double, 3> > &points_d,
										DeformGraph const &graph, 
										vector< NeighborGraphNodesOfPoint > const &ngns);

/*
 */
void transform_S3DPointMatchSetIndexed_with_DeformGraph(S3DPointMatchSetIndexed &match_set, 
														DeformGraph const*graph_1, //will fill_A_inv; could be NULL
														vector< NeighborGraphNodesOfPoint > const* ngns_1,
														DeformGraph const*graph_2, //will fill_A_inv, could be NULL
														vector< NeighborGraphNodesOfPoint > const* ngns_2,
														bool bDeleteExtraVert = false);


template<class T>
void transform_PointCloud_with_DeformGraph(CPointCloud<T> &pcd, DeformGraph &graph)
{
	int points_num = pcd.points_num;

#pragma omp parallel for
	for (int i = 0; i < points_num; i++)
	{
		printf("%04d\r", i);
		T* p_ = pcd.vt_data(i);
		vnl_vector_fixed<double, 3> p(p_[0], p_[1], p_[2]);
		NeighborGraphNodesOfPoint ngn;
		calc_NeighborGraphNodesOfPoint_by_dist(p, graph, ngn, 4);
		vnl_vector_fixed<double, 3> q;
		transform_one_point_with_DeformGraph(p, q, graph, ngn, true);
		p_[0] = q[0]; p_[1] = q[1]; p_[2] = q[2];

		T* n_ = pcd.vt_normal(i);
		if (n_)
		{
			vnl_vector_fixed<double, 3> n(n_[0], n_[1], n_[2]);
			vnl_vector_fixed<double, 3> nt;
			transform_one_normal_with_DeformGraph(n, nt, graph, ngn);
			n_[0] = nt[0]; n_[1] = nt[1], n_[2] = nt[2];
		}
	}
}

/* the connectivity remains, only points and normals are transformed.
 * the vertex with NULL ngns will be set to 0, and the triangles deleted
 */
template <class T>
void transform_Surface_with_DeformGraph( CSurface<T> &surface, 
										 DeformGraph graph, //will fill_A_inv
										 vector< NeighborGraphNodesOfPoint > const &ngns,
										 bool bDeleteExtraVert = false)
{
	assert( surface.vtNum == ngns.size() );
	if( graph.nodes.size() == 0)
	{
		LOGGER()->warning("transform_Surface_with_DeformGraph","graph nodes is empty!!!");
		return;
	}
	if( surface.vtNum != ngns.size() || surface.vtNum == 0)
	{
		LOGGER()->error("transform_Surface_with_DeformGraph", "data sizes do not match for surface<%d> and ngns<%d>!", surface.vtNum, ngns.size());
		return;
	}

	//compute A_inv for deformation graph
	graph.fill_A_inv();

	vnl_vector<bool> bFlag(surface.vtNum);
	T* vtData = surface.vtData;
	int vtDim = surface.vtDim;
#pragma omp parallel for schedule(static)
	for(int i=0; i<surface.vtNum; i++)
	{
		vnl_vector_fixed<double, 3> p(vtData[vtDim*i], vtData[vtDim*i+1], vtData[vtDim*i+2]);
		vnl_vector_fixed<double, 3> q;
		bFlag[i] = transform_one_point_with_DeformGraph(p, q, graph, ngns[i]);
		if( !bFlag[i] )		
			q.fill(0.0);
		vtData[vtDim*i] = (T)q[0];
		vtData[vtDim*i+1] = (T)q[1];
		vtData[vtDim*i+2] = (T)q[2];
		if( surface.haveNormalInfo() )
		{
			vnl_vector_fixed<double, 3> n(vtData[vtDim*i+3], vtData[vtDim*i+4], vtData[vtDim*i+5]);
			vnl_vector_fixed<double, 3> n_;
			transform_one_normal_with_DeformGraph(n, n_, graph, ngns[i]);
			vtData[vtDim*i+3] = n_[0];
			vtData[vtDim*i+4] = n_[1];
			vtData[vtDim*i+5] = n_[2];
		}
	}

	//delete triangle with untransformed points
	vector<int> triangles_new;
	int *triangles = surface.triangles;
	for(int i=0; i<surface.triNum; i++)
	{
		int idx1 = triangles[3*i];
		int idx2 = triangles[3*i+1];
		int idx3 = triangles[3*i+2];
		if( !bFlag[idx1] || !bFlag[idx2] || !bFlag[idx3] )
			continue;
		triangles_new.push_back(idx1);
		triangles_new.push_back(idx2);
		triangles_new.push_back(idx3);
	}
	delete [] surface.triangles;
	surface.triangles = new int[triangles_new.size()];
	surface.triNum = triangles_new.size()/3;
	for(int i=0; i<triangles_new.size(); i++)
		surface.triangles[i] = triangles_new[i];

	if( bDeleteExtraVert )
		surface.delete_extra_vertice();
}

}
#endif