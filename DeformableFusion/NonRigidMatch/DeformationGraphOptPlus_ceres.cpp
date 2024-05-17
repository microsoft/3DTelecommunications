#include "DeformationGraphOptPlus.h"
#include "nonrigid_matching_surface.h"
#include <omp.h>

namespace NonrigidMatching{

#ifdef USE_CERES_SOLVER

vector< CSurface<float>* > DeformGraphOptMultiDataPlusStatic::surfaces = vector< CSurface<float>* >();
vector< CSurface<float>* > DeformGraphOptMultiDataPlusStatic::rigid_surfaces = vector< CSurface<float>* >();
vector< NeighborGraphNodesOfPointList* > DeformGraphOptMultiDataPlusStatic::ngns_dense = vector< NeighborGraphNodesOfPointList* >();
SignedDistanceFunc* DeformGraphOptMultiDataPlusStatic::sdf_target = NULL;
double DeformGraphOptMultiDataPlusStatic::w_sdf = 0.0;
double DeformGraphOptMultiDataPlusStatic::w_intrinsic = 0.0;
double DeformGraphOptMultiDataPlusStatic::w_intersect = 0.0;
vector<CSurface<float>> DeformGraphOptMultiDataPlusStatic::surfaces_t = vector< CSurface<float> >();
vector<CSurface<float>> DeformGraphOptMultiDataPlusStatic::rigid_surfaces_t = vector< CSurface<float> >();
vector<GraphNodesTerritory*> DeformGraphOptMultiDataPlusStatic::graph_node_territory = vector<GraphNodesTerritory*>();
vector<GraphNodesTerritory*> DeformGraphOptMultiDataPlusStatic::rigid_graph_node_territory = vector<GraphNodesTerritory*>();

vector< DeformGraph* > DeformGraphOptMultiDataPlusStatic::rigid_graphs = vector< DeformGraph* >();
vector< CSurface<float>*> DeformGraphOptMultiDataPlusStatic::surfaces_t_all = vector< CSurface<float>*>();
vector< DeformGraph* > DeformGraphOptMultiDataPlusStatic::graphs_all = vector< DeformGraph* >();	
vector< vector< vector<int> >* > DeformGraphOptMultiDataPlusStatic::graph_node_territory_all = vector< vector< vector<int> >* >();
vector<double*> DeformGraphOptMultiDataPlusStatic::sdf_vals_last_retrieved_rigid_surfs = vector<double*>();
vector<double*> DeformGraphOptMultiDataPlusStatic::sdf_vals_last_retrieved_dynamic_surfs = vector<double*>();

void deform_graph_optimization_plus_ceres(DeformGraphOptMultiDataPlus &data)
{
	__tic__();
	DeformGraphOptMultiDataPlusStatic::rigid_transfs = data.rigid_transfs;
	DeformGraphOptMultiDataPlusStatic::rigid_surfaces = data.rigid_surfaces;
	DeformGraphOptMultiDataPlusStatic::graphs = data.graphs;
	DeformGraphOptMultiDataPlusStatic::matched_points_3d = data.matched_points_3d;
	DeformGraphOptMultiDataPlusStatic::ngns_key_points = data.ngns_key_points;
	DeformGraphOptMultiDataPlusStatic::surfaces = data.surfaces;
	DeformGraphOptMultiDataPlusStatic::ngns_dense = data.ngns_dense;
	for(int i=0; i<data.graphs.size(); i++)
	{
		assert( data.graphs[i] != NULL );
		if( i < data.matched_points_3d.size() )
			assert( data.matched_points_3d[i] != NULL );
		if( i < data.ngns_key_points.size() )
			assert( data.ngns_key_points[i] != NULL );
		assert( data.surfaces[i] != NULL );
		assert( data.ngns_dense[i] != NULL );

		assert( data.surfaces[i]->haveNormalInfo());
	}
	DeformGraphOptMultiDataPlusStatic::sdf_target = data.sdf_target;

	DeformGraphOptMultiDataPlusStatic::w_rot = std::sqrt(data.w_rot);
	DeformGraphOptMultiDataPlusStatic::w_reg = std::sqrt(data.w_reg);
	DeformGraphOptMultiDataPlusStatic::w_nodes_dist = std::sqrt(data.w_nodes_dist);
	DeformGraphOptMultiDataPlusStatic::w_key = std::sqrt(data.w_key);
	DeformGraphOptMultiDataPlusStatic::w_sdf = std::sqrt(data.w_sdf);
	DeformGraphOptMultiDataPlusStatic::w_intrinsic = std::sqrt(data.w_intrinsic);
	DeformGraphOptMultiDataPlusStatic::w_intersect = std::sqrt(data.w_intersect);

	ceres::Problem problem;
	
	//add rotation constraints
	printf("Adding...");
	if( data.w_rot > 0)
	{
		LossFunction *loss_fun_orthogonal = new ScaledLoss( new TrivialLoss(), data.w_rot, TAKE_OWNERSHIP );
		LossFunction *loss_fun_unitary = new ScaledLoss( new TrivialLoss(), data.w_rot, TAKE_OWNERSHIP );
		LossFunction *loss_fun_det = new ScaledLoss( new TrivialLoss(), data.w_rot*100.0, TAKE_OWNERSHIP );

		int count=0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph & graph = *(data.graphs[graphIdx]);
			int nodes_num = graph.nodes.size();
			for(int i=0; i<nodes_num; i++)
			{
				vnl_matrix_fixed<double, 3, 3> &A = graph.nodes[i].A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);

				problem.AddResidualBlock(new RotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r1);
				problem.AddResidualBlock(new RotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r2);
				problem.AddResidualBlock(new RotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r1, r2);
				problem.AddResidualBlock(new RotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r0);
				problem.AddResidualBlock(new RotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r1);
				problem.AddResidualBlock(new RotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r2);
				problem.AddResidualBlock(new RotationConstrTerm::DetCostFunction(), loss_fun_det, r0, r1, r2);
				count++;
			}
		}
		printf("Rot<%d>...", count);
	}

	//add regularization constraints
	if( data.w_reg > 0 )
	{
		LossFunction *loss_fun_reg = new ScaledLoss( new TrivialLoss(), data.w_reg, TAKE_OWNERSHIP );

		int count = 0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph & graph = *(data.graphs[graphIdx]);
			int nodes_num = graph.nodes.size();
			for(int i=0; i<nodes_num; i++)
			{
				DeformGraphNode &node = graph.nodes[i];
				vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
				double *r0_i = &(Ai[0][0]);
				double *r1_i = &(Ai[1][0]);
				double *r2_i = &(Ai[2][0]);
				double *t_i = node.t.data_block();
				vector<int> &nb_indices = node.neighborIndices;
				for(int j=0; j<nb_indices.size(); j++)
				{
					int ndIdx_j = nb_indices[j];
					double *t_j = graph.nodes[ndIdx_j].t.data_block();
					problem.AddResidualBlock(new RegularizationTerm::F0(graphIdx, i, ndIdx_j), loss_fun_reg, r0_i, &t_i[0], &t_j[0]);
					problem.AddResidualBlock(new RegularizationTerm::F1(graphIdx, i, ndIdx_j), loss_fun_reg, r1_i, &t_i[1], &t_j[1]); 
					problem.AddResidualBlock(new RegularizationTerm::F2(graphIdx, i, ndIdx_j), loss_fun_reg, r2_i, &t_i[2], &t_j[2]); 
					count++;
				}
			}
		}
		printf("Reg<%d>...", count);
	}

	//add nodes distance preservation term
	if( data.w_nodes_dist > 0 )
	{
		int count = 0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph & graph = *(data.graphs[graphIdx]);
			int nodes_num = graph.nodes.size();
			for(int i=0; i<nodes_num; i++)
			{
				DeformGraphNode &node_i = graph.nodes[i];
				vnl_matrix_fixed<double, 3, 3> &Ai = node_i.A;
				double *t_i = node_i.t.data_block();
				vector<int> &nb_indices = node_i.neighborIndices;
				for(int j=0; j<nb_indices.size(); j++)
				{
					int ndIdx_j = nb_indices[j];
					DeformGraphNode &node_j = graph.nodes[ndIdx_j];

					//loss function
					double dist_ij = dist_3d(node_i.g, node_j.g);
					LossFunction *loss_fun_nodes_dist = new ScaledLoss( new HuberLoss(0.2*dist_ij), data.w_nodes_dist, TAKE_OWNERSHIP );
					
					double *t_j = node_j.t.data_block();
					problem.AddResidualBlock(new NodesDistPreserveTerm::F(graphIdx, i, ndIdx_j), loss_fun_nodes_dist, &t_i[0], &t_i[1], &t_i[2], &t_j[0], &t_j[1], &t_j[2]);
					count++;
				}
			}
		}
		printf("NodeDist<%d>...", count);
	}

	//add key point constraints
	if( data.w_key > 0 )
	{
		LossFunction *loss_fun_key = new ScaledLoss( new CauchyLoss(5.0), data.w_key, TAKE_OWNERSHIP );

		int count = 0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph &graph = *(data.graphs[graphIdx]);
			S3DPointMatchSet const& mached_points_3d_s = *(data.matched_points_3d[graphIdx]); //_s means single
			NeighborGraphNodesOfPointList const& ngns_key_points_s = *(data.ngns_key_points[graphIdx]);
			for(int i=0; i<mached_points_3d_s.size(); i++)
			{
				NeighborGraphNodesOfPoint const& ngn = ngns_key_points_s[i];
				if( ngn.neighborIndices.size() == 0 )
					continue;

				vector< double* > paras_0;
				vector< double* > paras_1;
				vector< double* > paras_2;
				for(int k=0; k<ngn.neighborIndices.size(); k++)
				{
					int ndIdx = ngn.neighborIndices[k];
					DeformGraphNode &node = graph.nodes[ndIdx];
					vnl_matrix_fixed<double, 3, 3> &A = node.A;
					double *r0 = &(A[0][0]);
					double *r1 = &(A[1][0]);
					double *r2 = &(A[2][0]);			
					double *t = node.t.data_block();
					paras_0.push_back(r0);
					paras_0.push_back(&t[0]);
					paras_1.push_back(r1);
					paras_1.push_back(&t[1]);
					paras_2.push_back(r2);
					paras_2.push_back(&t[2]);
				}
				problem.AddResidualBlock(new KeyPointsTerm::F(graphIdx, i, 0), loss_fun_key, paras_0);
				problem.AddResidualBlock(new KeyPointsTerm::F(graphIdx, i, 1), loss_fun_key, paras_1);
				problem.AddResidualBlock(new KeyPointsTerm::F(graphIdx, i, 2), loss_fun_key, paras_2);
				count++;
			}
		}
		printf("KeyMatch<%d>...", count);
	}

	//add dense points constraints
	if( data.w_sdf > 0 )
	{
		LossFunction *loss_fun_sdf = new ScaledLoss( new TrivialLoss(), data.w_sdf, TAKE_OWNERSHIP );

		int count = 0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph &graph = *(data.graphs[graphIdx]);
			NeighborGraphNodesOfPointList const& ngns_dense_s = *(data.ngns_dense[graphIdx]);
			for(int i=0; i<ngns_dense_s.size(); i++)
			{
				NeighborGraphNodesOfPoint const& ngn = ngns_dense_s[i];
				if( ngn.neighborIndices.size() == 0 )
					continue;

				vector< double* > paras;
				for(int k=0; k<ngn.neighborIndices.size(); k++)
				{
					int ndIdx = ngn.neighborIndices[k];
					DeformGraphNode &node = graph.nodes[ndIdx];
					vnl_matrix_fixed<double, 3, 3> &A = node.A;
					double *r0 = &(A[0][0]);
					double *r1 = &(A[1][0]);
					double *r2 = &(A[2][0]);			
					double *t = node.t.data_block();
					paras.push_back(r0);
					paras.push_back(r1);
					paras.push_back(r2);
					paras.push_back(&t[0]);
					paras.push_back(&t[1]);
					paras.push_back(&t[2]);
				}
				problem.AddResidualBlock(new DensePtsTerm::F(graphIdx, i), loss_fun_sdf, paras);
				count++;
			}
		}

		for(int robjIdx=0; robjIdx<data.rigid_surfaces.size(); robjIdx++)
		{
			CSurface<float>* surface = data.rigid_surfaces[robjIdx];
			RigidTransformModel *rigid_transf = data.rigid_transfs[robjIdx];
			vector<double*> paras;
			paras.push_back(rigid_transf->rod.data_block());
			paras.push_back(rigid_transf->t.data_block());
			for(int vtIdx=0; vtIdx<surface->vtNum; vtIdx++)
			{
				problem.AddResidualBlock(new RigidDensePtsTerm::F(robjIdx, vtIdx), loss_fun_sdf, paras);
				count++;
			}
		}

		printf("DnsPts<%d>...", count);
	}


	if( data.w_intersect > 0 )
	{
		LossFunction *loss_fun_self_interect = new ScaledLoss( new TrivialLoss(), data.w_intersect, TAKE_OWNERSHIP );

		int count=0;
		//self intersection
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph &graph = *(data.graphs[graphIdx]);
			NeighborGraphNodesOfPointList const& ngns_dense_s = *(data.ngns_dense[graphIdx]);
			vector< vector<int> > const& graph_node_territory_s = *(DeformGraphOptMultiDataPlusStatic::graph_node_territory[graphIdx]);

			double thres_collision_dist = 10.0;
			vector<int> nodes_collided;
			identify_possible_collision_on_deformGraph(graph, nodes_collided, thres_collision_dist);
			for(int i=0; i<nodes_collided.size(); i++)
			{
				int nodeIdx = nodes_collided[i];
				vector<int> const& vt_list = graph_node_territory_s[nodeIdx];
				for(int j=0; j<vt_list.size(); j++)
				{
					int vtIdx = vt_list[j];

					NeighborGraphNodesOfPoint const& ngn = ngns_dense_s[vtIdx];
					if( ngn.neighborIndices.size() == 0 )
						continue;

					vector< double* > paras;
					for(int k=0; k<ngn.neighborIndices.size(); k++)
					{
						int ndIdx = ngn.neighborIndices[k];
						DeformGraphNode &node = graph.nodes[ndIdx];
						vnl_matrix_fixed<double, 3, 3> &A = node.A;
						double *r0 = &(A[0][0]);
						double *r1 = &(A[1][0]);
						double *r2 = &(A[2][0]);			
						double *t = node.t.data_block();
						paras.push_back(r0);
						paras.push_back(r1);
						paras.push_back(r2);
						paras.push_back(&t[0]);
						paras.push_back(&t[1]);
						paras.push_back(&t[2]);
					}
					problem.AddResidualBlock(new AntiSelfInterSectionTerm::F(graphIdx, vtIdx), loss_fun_self_interect, paras);
					count++;
				}//end of for-j
			}//end of for-i
		}//end of for-graphIdx
		printf("Anti-Self-InterSection<%d>...", count);
	}


	//add intrinsic term F3	
	if( data.w_intrinsic > 0 )
	{
		int count = 0;
		for(int graphIdx=0; graphIdx<data.graphs.size(); graphIdx++)
		{
			DeformGraph &graph = *(data.graphs[graphIdx]);
			CSurface<float> const& surface = *(data.surfaces[graphIdx]);
			NeighborGraphNodesOfPointList const& ngns_dense_s = *(data.ngns_dense[graphIdx]);

			for(int i=0; i<surface.triNum; i++)
			{
				int vtIdx1 = surface.triangles[3*i];
				int vtIdx2 = surface.triangles[3*i+1];
				int vtIdx3 = surface.triangles[3*i+2];
				if( ngns_dense_s[vtIdx1].neighborIndices.size() == 0 ||
					ngns_dense_s[vtIdx2].neighborIndices.size() == 0 ||
					ngns_dense_s[vtIdx3].neighborIndices.size() == 0 )
					continue;

				//compute the inner product of the edges
				float *p1 = data.surfaces[graphIdx]->vt_data_block(vtIdx1);
				float *p2 = data.surfaces[graphIdx]->vt_data_block(vtIdx2);
				float *p3 = data.surfaces[graphIdx]->vt_data_block(vtIdx3);
				vnl_vector_fixed<double, 3> vt1 = vnl_vector_fixed<double, 3>(p1[0], p1[1], p1[2]);
				vnl_vector_fixed<double, 3> vt2 = vnl_vector_fixed<double, 3>(p2[0], p2[1], p2[2]);
				vnl_vector_fixed<double, 3> vt3 = vnl_vector_fixed<double, 3>(p3[0], p3[1], p3[2]);
				float *n1_ = data.surfaces[graphIdx]->vt_normal(vtIdx1);
				float *n2_ = data.surfaces[graphIdx]->vt_normal(vtIdx2);
				float *n3_ = data.surfaces[graphIdx]->vt_normal(vtIdx3);
				vnl_vector_fixed<double, 3> n1 = vnl_vector_fixed<double, 3>(n1_[0], n1_[1], n1_[2]);
				vnl_vector_fixed<double, 3> n2 = vnl_vector_fixed<double, 3>(n2_[0], n2_[1], n2_[2]);
				vnl_vector_fixed<double, 3> n3 = vnl_vector_fixed<double, 3>(n3_[0], n3_[1], n3_[2]);
				vnl_vector_fixed<double, 3> e12 = vt2-vt1;
				vnl_vector_fixed<double, 3> e13 = vt3-vt1;
				vnl_vector_fixed<double, 3> e21 = vt1-vt2;
				vnl_vector_fixed<double, 3> e23 = vt3-vt2;
				vnl_vector_fixed<double, 3> e31 = vt1-vt3;
				vnl_vector_fixed<double, 3> e32 = vt2-vt3;
				double inner_prod_1 = dot_product(e12, e13) - dot_product(e12, n1)*dot_product(e13, n1);
				double inner_prod_2 = dot_product(e21, e23) - dot_product(e21, n2)*dot_product(e23, n2);
				double inner_prod_3 = dot_product(e31, e32) - dot_product(e31, n3)*dot_product(e32, n3);

				//loss function
				double s = 0.2*(std::fabs(inner_prod_1) + std::fabs(inner_prod_1) + std::fabs(inner_prod_1))/3.0;
				if( s == 0 )
					continue;
				LossFunction *loss_fun_intrinsic = new ScaledLoss( new HuberLoss(s), data.w_intrinsic, TAKE_OWNERSHIP);

				//get the uion of the neighboring graph nodes of three vertices
				vector<int> ngn_indices = ngns_dense_s[vtIdx1].neighborIndices;
				for(int k=0; k<ngns_dense_s[vtIdx2].neighborIndices.size(); k++)
				{
					int ndIdx = ngns_dense_s[vtIdx2].neighborIndices[k];
					if( search_val_list(ngn_indices, ndIdx) < 0 )
						ngn_indices.push_back(ndIdx);				
				}
				for(int k=0; k<ngns_dense_s[vtIdx3].neighborIndices.size(); k++)
				{
					int ndIdx = ngns_dense_s[vtIdx3].neighborIndices[k];
					if( search_val_list(ngn_indices, ndIdx) < 0 )
						ngn_indices.push_back(ndIdx);				
				}

				vector< double* > paras;
				for(int k=0; k<ngn_indices.size(); k++)
				{
					int ndIdx = ngn_indices[k];
					DeformGraphNode &node = graph.nodes[ndIdx];
					vnl_matrix_fixed<double, 3, 3> &A = node.A;
					double *r0 = &(A[0][0]);
					double *r1 = &(A[1][0]);
					double *r2 = &(A[2][0]);			
					double *t = node.t.data_block();
					paras.push_back(r0);
					paras.push_back(r1);
					paras.push_back(r2);
					paras.push_back(&t[0]);
					paras.push_back(&t[1]);
					paras.push_back(&t[2]);
				}
				problem.AddResidualBlock(new IsometricTerm::F3(graphIdx, vtIdx1, vtIdx2, vtIdx3, ngn_indices), loss_fun_intrinsic, paras);
				count++;
			}
		}
		printf("Intrinsics<%d>...", count);
	}


	Solver::Options options;
	options.max_num_iterations = 30;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.function_tolerance = 1e-8;
    options.gradient_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = omp_get_num_procs();
	options.num_linear_solver_threads = omp_get_num_procs();
	options.use_nonmonotonic_steps = false;
	options.update_state_every_iteration = false;

	options.callbacks.push_back(new IterCallBackTransSurface());
	Solver::Summary summary;
	int graph_nodes_num = 0;
	int matches_key_points_num = 0;
	int dense_pts_num = 0;
	for(int i=0; i<data.graphs.size(); i++)
	{
		graph_nodes_num += data.graphs[i]->nodes.size();
		if( data.matched_points_3d.size() > i )
			matches_key_points_num += data.matched_points_3d[i]->size();
		dense_pts_num += data.surfaces[i]->vtNum;
	}
	printf("\nCeres Optimization Plus: nodes=%d, key pts=%d, dense pts=%d\n", graph_nodes_num, matches_key_points_num, dense_pts_num);
	Solve(options, &problem, &summary);

	std::cout<<summary.FullReport()<<endl;
	printf("time OptPlus: %f\n", __toc__());
}
#endif

};