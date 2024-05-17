#ifndef __NONRIGID_MATCHING_SURFACE_H__
#define __NONRIGID_MATCHING_SURFACE_H__
#include "DeformGraph.h"
#include "CSurface.h"
#include "surface_geodesic_distance.h"
#include "TSDF.h"
#include "DDF.h"
#include <utility>
#include <vector>
using namespace std;

namespace NonrigidMatching{

/* NOTE: upon returned, geo_dist is associate with surface, and the shortest paths from 
 *		 the graph nodes are calculated. Note that the previous associated will be cleared
 */
bool generate_deform_graph_from_surface( CSurface<float> const& surface,
										 CGeodesicDistanceOnSurface &geo_dist, //out only
										 DeformGraph &graph,
										 double min_node_dist = 7.0,
										 int neighbor_nodes_num = 5,
										 int thres_num_isolated_vts = 50);


bool identify_possible_collision_on_deformGraph( DeformGraph const& graph,
												 vector<int> &collided_nodes_list,
												 double thres_collision_dist );

/* identify the collision areas on the surface
 */
bool identify_collision_on_surface( CSurface<float> const& surface_ref, 
									CSurface<float> const& surface_t,
									DeformGraph const& graph, //graph on the reference surface, transformed graph will be on "surface"
									vector< NeighborGraphNodesOfPoint > const& ngns,
									vector< vector<int> > const& graph_node_territory,
									S3DPointMatchSet &match_set_collsion,
									double thres_collision_dist);
									   

/* for each graph node, transform it and then find its nearest point on the target surface
 * if bReSample == true, will resample the nodes;
 * upon return, geo_dist_t will be associated with surface_t
 */
bool generate_reverse_deform_graph_for_target_surface( DeformGraph const &graph,
													   CSurface<float> const& surface_dst, //surface at destination
													   DeformGraph &graph_r,
													   CGeodesicDistanceOnSurface &geo_dist_dst, //out; unnecessary to initialize it
													   double thres_dist_node_to_surf = 4.0,
													   bool bReSample = true,													   										 
													   double min_node_dist = 7.0,
													   int neighbor_nodes_num = 5);

// compute the graph_r that align surface_t to surface with optimization
bool reverse_DeformGraph2( DeformGraph const& graph, //graph of template surface
						   CSurface<float> const& surface, //template
						   vector< NeighborGraphNodesOfPoint > const& ngns, //on surface (template)
						   DeformGraph &graph_r  //out						
						   );

//generate reverse graph by find the matched dense points with optimization
// the graph topology is based on surface_t
bool generate_reverse_deform_graph( DeformGraph const& graph, 
									CSurface<float> const& surface, //template
									CSurface<float> const& surface_t, //target
									vector< NeighborGraphNodesOfPoint > const& ngns, //on surface (template)
									DeformGraph &graph_r,  //out
									CGeodesicDistanceOnSurface &geo_dist_t, //out; unnecessary to initialize it
									vector< NeighborGraphNodesOfPoint > &ngns_dst, //out
									bool bResample = true,
									double min_node_dist = 7.0,
									int neighbor_nodes_num = 5,
									bool bUseIntPutGraphAndNgns = false,
									double angle_thres = 80, //degree
									double dist_thres = 3.0,
									int downsample_rate_for_opt = 2,
									bool bOptNgns = false
									);

bool generate_relative_deform_graph( CSurface<float> const& surface,
									 DeformGraph const& graph_1, // deform surface to surface_1
									 DeformGraph const& graph_2, // deform surface to surface_2
									 vector< NeighborGraphNodesOfPoint > const& ngns_1,
									 vector< NeighborGraphNodesOfPoint > const& ngns_2,
									 DeformGraph &graph_relative // deform surface_1 to surface_2 using ngns_1
									 ); 
/* NOTE: some nodes might be deleted upon return even when bReSample == false
 * if bReSample == true, upon return, geo_dist will be associated with surface.
 * if geo_dist is associate with surface before hand, the associated surface MUST be "surface",
 *  because this function will not clear old association
 */
bool force_graph_on_surface( DeformGraph const&graph, DeformGraph &graph_new, CSurface<float> const&surface, 
							 double thres_dist_node_to_surf = 4.0, 
							 vector<int> *indices_old = NULL, //new to old graph index. store the index of the corresponding old graph node
							 bool bUpdateG = true
							 );

//graph is on the surface, and graph_prev should be close to surface
//upon return, geo_dist will be associated with "surface". However if geo_dist is associate with surface before hand, 
//the associated surface MUST be "surface", because this function will not clear old association
bool deform_grpah_initialization_Geodesic( DeformGraph &graph, DeformGraph const& graph_prev, 
										   CSurface<float> const&surface,
										   CGeodesicDistanceOnSurface &geo_dist = CGeodesicDistanceOnSurface());


// graphs: <in> the deformation graphs in the list will deform surface_1st consecutively from 1st to last
//         it is not necessary to require these graphs have corresponding nodes
// surface_1st: <in> graphs[0] must lie on this surface
// ngns: <in> NeighborGraphNodesOfPointList corresponds to graphs[0] for surface_1st
// graph_1st_to_last: <out> has the same nodes as graphs[0], but will deform surface_1st to the last surface as defined in graphs.
void deform_graph_from_graph_chain( CSurface<float> const& surface_1st, 
									vector< DeformGraph* > const& graphs,
									NeighborGraphNodesOfPointList const& ngns,
									DeformGraph &graph_1st_to_last);


// verify the deformation graph model
// for each node on the EG, checking whether the vertcies within its territory has corresponding points in the destiny surface
// If not, check if it is connected with other nodes
// If not, reinialize its affine transformation with its nearest neighbors or delete the nodes and modify ngns as well
bool deform_graph_verification( CSurface<float> const& surface,
								DeformGraph &graph,
								NeighborGraphNodesOfPointList const&ngns,
								CSurface<float> const& surface_dst,
								bool bInitBadNodeWithNN = true);

//surface: in and out
// geo_dist: out
bool reverse_transform_Surface_with_DeformGraph( DeformGraph const &graph,
												 CSurface<float> &surface,
												 CGeodesicDistanceOnSurface &geo_dist,
												 int num_neighbor_nodes,
												 double thres_dist_to_node);

struct TransformedVoxel
{
	int coord_ori[3];
	float coord_t[3]; //coordinate after transformation on the grid
	float val; // the value it carries
	vnl_vector_fixed<float, 3> drt; //the drt after transformation
	vnl_vector_fixed<float, 3> clr; // the color it carries
};

enum SDFTransfInterpMethod
{
	SDFTransfInterp_NN,
	SDFTransfInterp_WeightedAvg
};

typedef vector<TransformedVoxel*> TransformedVoxelList;

/* graph will transform sdf to sdf_dst
 */
void transform_sdf_with_DeformGraph( TSDF const&sdf, TSDF &sdf_dst, 
									 DeformGraph const&graph, 
									 CSurface<float> const&surface, 
									 vector< NeighborGraphNodesOfPoint > const&ngns,											 											 
									 BoundingBox3D bbox,
									 double x_res, double y_res, double z_res);

//color field is handled as well
void transform_ddf_with_DeformGraph( DDF const&ddf, DDF &ddf_dst, 
									 DeformGraph const&graph, 
									 CSurface<float> const&surface, 
									 vector< NeighborGraphNodesOfPoint > const&ngns,											 											 
									 BoundingBox3D bbox, //at dst
									 double x_res, double y_res, double z_res,
									 SDFTransfInterpMethod interp_method = SDFTransfInterp_WeightedAvg);

//==============Calculate NeighborGraphNodesOfPoint=========

//will NOT update CGeodesicDistanceOnSurface
// the nodes of graph must lie on the surface (i.e., has a corresponding vertex ID)
bool calc_NeighborGraphNodesOfPoint_Surface( vnl_vector_fixed<double, 3> const &p, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface const&geo_dist,
											 NeighborGraphNodesOfPoint &ngn,
											 int k, double max_dist);
//will NOT update CGeodesicDistanceOnSurface
bool calc_NeighborGraphNodesOfPoint_Surface( int vtIdx, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface const&geo_dist,
											 NeighborGraphNodesOfPoint &ngn,
											 int k, double max_dist);

/* note: the geo_dist must be the one returned by generate_deform_graph_from_surface;
 *       otherwise it must be associated with according surface
 */
//might update CGeodesicDistanceOnSurface
bool calc_NeighborGraphNodesOfPoint_Surface( vector< vnl_vector_fixed<double, 3> > const& points, 
											 DeformGraph const &graph, 
											 CSurface<float> const& surface,
											 CGeodesicDistanceOnSurface &geo_dist,
											 vector< NeighborGraphNodesOfPoint > &ngns,
											 int k, double max_dist);

//calc NeighborGraphNodesOfPoint for all surface vertices
// NOTE: 1. the graph must be generated for the surface, i.e. the vtIdx must
//			be the index of the surface vertex.			
//		 2. might update CGeodesicDistanceOnSurface; geo_dist can be empty.
bool calc_NeighborGraphNodesOfPoint_Surface_all_vts( DeformGraph const &graph, 
													 CSurface<float> const& surface,
													 CGeodesicDistanceOnSurface &geo_dist,
													 vector< NeighborGraphNodesOfPoint > &ngns,
													 int k, double max_dist);

void calc_NeighborGraphNodesOfPoint_vertex_matching( CSurface<float> const& surface_template,
													 NeighborGraphNodesOfPointList const& ngns_template, 
													 CSurface<float> const& surface,
													 NeighborGraphNodesOfPointList &ngns,
													 double thres_vert_dist = 3.0);

//compared with earlier method, it is much faster. 
void calc_NeighborGraphNodesOfPoint_vertex_matching2( CSurface<float> const& surface_template,
													  NeighborGraphNodesOfPointList const& ngns_template, 
													  DeformGraph const& graph, //graph for surface_template
													  CSurface<float> const& surface,
													  NeighborGraphNodesOfPointList &ngns,
													  double thres_vert_dist = 3.0);


}
#endif