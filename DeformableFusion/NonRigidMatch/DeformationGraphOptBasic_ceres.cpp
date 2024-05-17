#include "DeformationGraphOptBasic.h"
#include <omp.h>

namespace NonrigidMatching{

#ifdef USE_CERES_SOLVER
vector<DeformGraph*> DeformGraphOptMultiDataStatic::graphs = vector<DeformGraph*>();
vector<RigidTransformModel*> DeformGraphOptMultiDataStatic::rigid_transfs = vector<RigidTransformModel*>();
vector<S3DPointMatchSet*> DeformGraphOptMultiDataStatic::matched_points_3d = vector<S3DPointMatchSet*>(); //Key Points(or constrain points): points_1 is the template, points_2 is the target
vector<NeighborGraphNodesOfPointList*> DeformGraphOptMultiDataStatic::ngns_key_points = vector<NeighborGraphNodesOfPointList*>(); // neighboring graphs node of key points
double DeformGraphOptMultiDataStatic::w_rot = 0.0;//rotation
double DeformGraphOptMultiDataStatic::w_reg = 0.0; //regularization
double DeformGraphOptMultiDataStatic::w_key = 0.0; //key points
double DeformGraphOptMultiDataStatic::w_nodes_dist = 0.0; //nodes distance prevervation

void deform_graph_optimization_ceres(DeformGraphOptMultiData &data)
{
	__tic__();
	DeformGraphOptMultiDataStatic::graphs = data.graphs;
	DeformGraphOptMultiDataStatic::matched_points_3d = data.matched_points_3d;
	DeformGraphOptMultiDataStatic::ngns_key_points = data.ngns_key_points;
	for(int i=0; i<data.graphs.size(); i++)
	{
		assert( data.graphs[i] != NULL );
		if( i < data.matched_points_3d.size() )
			assert( data.matched_points_3d[i] != NULL );
		if( i < data.ngns_key_points.size() )
			assert( data.ngns_key_points[i] != NULL );
	}

	DeformGraphOptMultiDataStatic::w_rot = std::sqrt(data.w_rot);
	DeformGraphOptMultiDataStatic::w_reg = std::sqrt(data.w_reg);
	DeformGraphOptMultiDataStatic::w_nodes_dist = std::sqrt(data.w_nodes_dist);
	DeformGraphOptMultiDataStatic::w_key = std::sqrt(data.w_key);

	ceres::Problem problem;
	
	//add rotation constraints
	LOGGER()->trace("Adding...");
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
		LOGGER()->trace("Rot<%d>...", count);
	}

	if (data.w_zero > 0 && !data.bFixGlobalRigid && !data.bFixNonrigid)
	{
		LossFunction *loss_func_r = new ScaledLoss(new CauchyLoss(0.2), data.w_zero, TAKE_OWNERSHIP);
		LossFunction *loss_func_t = new ScaledLoss(new CauchyLoss(1.5), data.w_zero, TAKE_OWNERSHIP);

		int count = 0;
		for (int graphIdx = 0; graphIdx < data.graphs.size(); graphIdx++)
		{
			DeformGraph & graph = *(data.graphs[graphIdx]);
			int nodes_num = graph.nodes.size();
			for (int i = 0; i < nodes_num; i++)
			{
				DeformGraphNode &node = graph.nodes[i];
				vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
				double *r0_i = &(Ai[0][0]);
				double *r1_i = &(Ai[1][0]);
				double *r2_i = &(Ai[2][0]);
				double *t_i = node.t.data_block();
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::IdentityCostFunctionR0(), loss_func_r, r0_i);
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::IdentityCostFunctionR1(), loss_func_r, r1_i);
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::IdentityCostFunctionR2(), loss_func_r, r2_i);
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::ZeroTranslation(), loss_func_t, &t_i[0]);
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::ZeroTranslation(), loss_func_t, &t_i[1]);
				problem.AddResidualBlock(new DeformGraphIdentityRotationZeroTranslation::ZeroTranslation(), loss_func_t, &t_i[2]);
				count++;
			}
		}
		LOGGER()->trace("Zero<%d>...", count);
	}

	//add regularization constraints
	if( data.w_reg > 0)
	{
		LossFunction *loss_fun_reg = new ScaledLoss( new ArctanLoss(2.0), data.w_reg, TAKE_OWNERSHIP );

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
		LOGGER()->trace("Reg<%d>...", count);
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
		LOGGER()->trace("NodeDist<%d>...", count);
	}

	//add key point constraints
	if( data.w_key > 0 )
	{

		LossFunction *loss_fun_key = new ScaledLoss(new TrivialLoss(), data.w_key, TAKE_OWNERSHIP);

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
				paras.push_back(graph.global_rigid.rod.data_block());
				paras.push_back(graph.global_rigid.t.data_block());
				problem.AddResidualBlock(new KeyPointsTerm::F2(graphIdx, i), loss_fun_key, paras);

				count++;
			}
		}
		LOGGER()->trace("KeyMatch<%d>...", count);
	}
	LOGGER()->trace("End\n");

	if (data.bFixGlobalRigid)
	{
		for (int i = 0; i < data.graphs.size(); i++)
		{
			DeformGraph *graph = data.graphs[i];
			if (problem.HasParameterBlock(graph->global_rigid.rod.data_block()))
				problem.SetParameterBlockConstant(graph->global_rigid.rod.data_block());
			if (problem.HasParameterBlock(graph->global_rigid.t.data_block()))
				problem.SetParameterBlockConstant(graph->global_rigid.t.data_block());
		}
	}

	if (data.bFixNonrigid)
	{
		for (int i = 0; i < data.graphs.size(); i++)
		{
			DeformGraph *graph = data.graphs[i];
			for (int ndIdx = 0; ndIdx < graph->nodes.size(); ndIdx++)
			{
				DeformGraphNode &nd = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = nd.A;
				vnl_vector_fixed<double, 3> &t = nd.t;

				if (problem.HasParameterBlock(&(A[0][0])))
					problem.SetParameterBlockConstant(&(A[0][0]));
				if (problem.HasParameterBlock(&(A[1][0])))
					problem.SetParameterBlockConstant(&(A[1][0]));
				if (problem.HasParameterBlock(&(A[2][0])))
					problem.SetParameterBlockConstant(&(A[2][0]));
				if (problem.HasParameterBlock(&(t[0])))
					problem.SetParameterBlockConstant(&(t[0]));
				if (problem.HasParameterBlock(&(t[1])))
					problem.SetParameterBlockConstant(&(t[1]));
				if (problem.HasParameterBlock(&(t[2])))
					problem.SetParameterBlockConstant(&(t[2]));
			}
		}
	}

	Solver::Options options;
	options.max_num_iterations = data.itmax;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-6;
    options.parameter_tolerance = 1e-6;
	options.minimizer_progress_to_stdout = false;
	options.logging_type = ceres::PER_MINIMIZER_ITERATION;
	options.num_threads = omp_get_num_procs();
	options.num_linear_solver_threads = omp_get_num_procs();
	options.callbacks.push_back(new IterCallBackBasic());

	Solver::Summary summary;
	int graph_nodes_num = 0;
	int matches_key_points_num = 0;
	for(int i=0; i<data.graphs.size(); i++)
	{
		graph_nodes_num += data.graphs[i]->nodes.size();
		if( data.matched_points_3d.size() > i )
			matches_key_points_num += data.matched_points_3d[i]->size();
	}
	LOGGER()->trace("\nCeres Optimization Basic: nodes=%d, key pts=%d\n", graph_nodes_num, matches_key_points_num);
	Solve(options, &problem, &summary);
	std::cout<<summary.BriefReport()<<endl;

	LOGGER()->trace("time OptBasic: %f\n", __toc__());
}

#endif


};
