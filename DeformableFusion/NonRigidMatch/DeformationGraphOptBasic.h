//===============================================
//			DeformGraphOptBasic.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __DEFORMATIONGRAPHOPTBASIC_H__
#define __DEFORMATIONGRAPHOPTBASIC_H__
#ifdef USE_SPARSELM
#include "splm.h"
#endif

#include "DeformGraph.h"
#include "basic_geometry.h"
#include "surface_misc.h"
#include "RigidTransformModel.h"

#ifdef USE_CERES_SOLVER
#include "ceres\ceres.h"
using namespace ceres;
#endif


namespace NonrigidMatching{

inline void assign_rigid_movement_to_DeformGraph(DeformGraph &graph, RigidTransformModel const& rigid_model)
{
	vnl_matrix_fixed<double, 3, 3> R;
	rodrigues_to_matrix(rigid_model.rod, R);
	vnl_vector_fixed<double, 3> T = rigid_model.t;
	for(int i=0; i<graph.nodes.size(); i++)
	{
		DeformGraphNode &node = graph.nodes[i];
		node.A = R;
		node.t = R*node.g + T - node.g;
	}
}

template<class T>
void transform_Surface_with_RigidModel(CSurface<T> &surface, RigidTransformModel const& rigid_model)
{
	vnl_matrix_fixed<double, 3, 3> R;
	rodrigues_to_matrix(rigid_model.rod, R);
	vnl_vector_fixed<double, 3> t = rigid_model.t;

	transform_Surface_with_RigidModel(surface, R, t);
};

class DeformGraphOptimizationData
{
public:
	DeformGraphOptimizationData()
		:w_rot(0.0),
		 w_reg(0.0),
		 w_zero(0.0),
		 w_nodes_dist(0.0),
		 w_constr(0.0),
		 pt_constr_thres(500),
		 bFixGlobalRigid(false),
		 bFixNonrigid(false),
		 itmax(30)
	{}

public:
	DeformGraph graph;
	S3DPointMatchSet matched_points_3d;//Key Points(or constrain points): points_1 is the template, points_2 is the target
	vector< NeighborGraphNodesOfPoint > ngns_key_points; // neighboring graphs node of key points

public:
	double w_rot;//rotation
	double w_reg; //regularization
	double w_zero; //non-rigidness
	double w_nodes_dist; //nodes distance preversation term
	double w_constr; //key points
	double pt_constr_thres;
	int itmax;

public:
	bool bFixGlobalRigid;
	bool bFixNonrigid;

public: //will be initialized automatically before running optization
	int constr_num_reg; //the number of constraint for smoothness (reguliation) constraint
	int constr_num_key_points; //the number of constraint from matched key points
	int key_points_ngn_num_total; //the total number of ngns for key points
};

class DeformGraphOptMultiData
{
public:
	DeformGraphOptMultiData() 
		: bFixGlobalRigid(false),
		  bFixNonrigid(false),
		  pt_constr_thres(500),
		  itmax(30)
	{}
public:
	vector<RigidTransformModel*> rigid_transfs;
	vector<DeformGraph*> graphs;
	vector<S3DPointMatchSet*> matched_points_3d;//Key Points(or constrain points): points_1 is the template, points_2 is the target
	vector<NeighborGraphNodesOfPointList*> ngns_key_points; // neighboring graphs node of key points

public:
	double w_rot;//rotation
	double w_reg; //regularization
	double w_zero; //non-rigidness
	double w_nodes_dist; // nodes distance preservation term
	double w_key; //key points
	double pt_constr_thres;

public:
	bool bFixGlobalRigid;
	bool bFixNonrigid;

public:
	int frmIdx;
	int itmax;
};

void deform_graph_optimization(DeformGraphOptimizationData &data);
void deform_graph_multiObj_opt(DeformGraphOptMultiData &multi_data);

#ifdef USE_SPARSELM
bool deform_graph_optimization_sparselm(DeformGraphOptimizationData& data);
#endif

#ifdef USE_CERES_SOLVER
class DeformGraphOptMultiDataStatic
{
public:	
	static vector<RigidTransformModel*> rigid_transfs;
	static vector<DeformGraph*> graphs;
	static vector<S3DPointMatchSet*> matched_points_3d;//Key Points(or constrain points): points_1 is the template, points_2 is the target
	static vector<NeighborGraphNodesOfPointList*> ngns_key_points; // neighboring graphs node of key points

public:
	static double w_rot;//rotation
	static double w_reg; //regularization
	static double w_nodes_dist; // nodes distance preservation term
	static double w_key; //key points

public:
	static bool bEstimateGlobalRigidForDeformGraph;

};

void deform_graph_optimization_ceres(DeformGraphOptMultiData &data);

//transform the surface
class IterCallBackBasic : public ceres::IterationCallback, public DeformGraphOptMultiDataStatic
{
public:
	virtual CallbackReturnType operator()(const IterationSummary& summary)
	{
		printf(".");
		return SOLVER_CONTINUE;
	}
};

namespace RotationConstrTerm
{
	//orthogonal constraint on rows: r1.*r2=0; r1.*r3=0; r2.*r3=0
	class OrthogonalCostFunction : public SizedCostFunction<1, 3, 3>, public DeformGraphOptMultiDataStatic
	{
	public: 
		OrthogonalCostFunction() {};
		virtual ~OrthogonalCostFunction() {};

	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const* r1 = parameters[0];
		  double const* r2 = parameters[1];
		  residuals[0] = (r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2]);

		  if( jacobians != NULL )
		  {
			  if( jacobians[0] != NULL )
			  {
				  jacobians[0][0] = r2[0];
				  jacobians[0][1] = r2[1];
				  jacobians[0][2] = r2[2];
			  }

			  if( jacobians[1] != NULL )
			  {
				  jacobians[1][0] = r1[0];
				  jacobians[1][1] = r1[1];
				  jacobians[1][2] = r1[2];
			  }
		  }
		  return true;
	  }
	};

	//unitary constraint: r.*r - 1.0 = 0
	class UnitaryCostFunction : public SizedCostFunction<1, 3>, public DeformGraphOptMultiDataStatic
	{
	public:
		UnitaryCostFunction() {};
		~UnitaryCostFunction() {};
	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const* r = parameters[0];
		  residuals[0] = (r[0]*r[0] + r[1]*r[1] + r[2]*r[2] - 1.0);
		  if( jacobians != NULL && jacobians[0] != NULL)
		  {
			  jacobians[0][0] = 2.0 * r[0];
			  jacobians[0][1] = 2.0 * r[1];
			  jacobians[0][2] = 2.0 * r[2];
		  }
		  return true;
	  }
	};

	class DetCostFunction : public SizedCostFunction<1, 3, 3, 3>, public DeformGraphOptMultiDataStatic
	{
	public: 
		DetCostFunction() {};
		~DetCostFunction() {};
	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const* r0 = parameters[0];
		  double const* r1 = parameters[1];
		  double const* r2 = parameters[2];

		  // double w_rot_adj = 10.0*w_rot;
		  residuals[0] = (r0[0]*r1[1]*r2[2] + r0[1]*r1[2]*r2[0] + r1[0]*r2[1]*r0[2] - r0[2]*r1[1]*r2[0] - r0[1]*r1[0]*r2[2] - r0[0]*r1[2]*r2[1] - 1.0);// * w_rot_adj;
		  if( jacobians != NULL )
		  {
			  if( jacobians[0] != NULL )
			  {
				  jacobians[0][0] = (r1[1]*r2[2] - r1[2]*r2[1]);//*w_rot_adj;
				  jacobians[0][1] = (r1[2]*r2[0] - r1[0]*r2[2]);//*w_rot_adj;
				  jacobians[0][2] = (r1[0]*r2[1] - r1[1]*r2[0]);//*w_rot_adj;
			  }
			  if( jacobians[1] != NULL )
			  {
				  jacobians[1][0] = (r2[1]*r0[2] - r0[1]*r2[2]);//*w_rot_adj;
				  jacobians[1][1] = (r0[0]*r2[2] - r0[2]*r2[0]);//*w_rot_adj;
				  jacobians[1][2] = (r0[1]*r2[0] - r0[0]*r2[1]);//*w_rot_adj;
			  }
			  if( jacobians[2] != NULL )
			  {
				  jacobians[2][0] = (r0[1]*r1[2] - r0[2]*r1[1]);//*w_rot_adj;
				  jacobians[2][1] = (r1[0]*r0[2] - r0[0]*r1[2]);//*w_rot_adj;
				  jacobians[2][2] = (r0[0]*r1[1] - r0[1]*r1[0]);//*w_rot_adj;
			  }
		  }
		  return true;
	  }
	};
};


namespace DeformGraphIdentityRotationZeroTranslation
{
	class IdentityCostFunctionR0 : public SizedCostFunction<3, 3>, public DeformGraphOptMultiDataStatic
	{
	public:
		IdentityCostFunctionR0() {};
		~IdentityCostFunctionR0() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const* r0 = parameters[0];

			residuals[0] = r0[0] - 1.0;
			residuals[1] = r0[1] - 0.0;
			residuals[2] = r0[2] - 0.0;

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					//dres0/ dr0
					jacobians[0][0] = 1.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;


					//dres1/ dr0
					jacobians[0][3] = 0.0;
					jacobians[0][4] = 1.0;
					jacobians[0][5] = 0.0;

					//dres2/ dr0
					jacobians[0][6] = 0.0;
					jacobians[0][7] = 0.0;
					jacobians[0][8] = 1.0;

				}
			}
			return true;
		}
	};

	class IdentityCostFunctionR1 : public SizedCostFunction<3, 3>, public DeformGraphOptMultiDataStatic
	{
	public:
		IdentityCostFunctionR1() {};
		~IdentityCostFunctionR1() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const* r1 = parameters[0];

			residuals[0] = r1[0] - 0.0;
			residuals[1] = r1[1] - 1.0;
			residuals[2] = r1[2] - 0.0;

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					//dres0/ dr0
					jacobians[0][0] = 1.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;


					//dres1/ dr0
					jacobians[0][3] = 0.0;
					jacobians[0][4] = 1.0;
					jacobians[0][5] = 0.0;

					//dres2/ dr0
					jacobians[0][6] = 0.0;
					jacobians[0][7] = 0.0;
					jacobians[0][8] = 1.0;

				}
			}
			return true;
		}
	};

	class IdentityCostFunctionR2 : public SizedCostFunction<3, 3>, public DeformGraphOptMultiDataStatic
	{
	public:
		IdentityCostFunctionR2() {};
		~IdentityCostFunctionR2() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const* r2 = parameters[0];

			residuals[0] = r2[0] - 0.0;
			residuals[1] = r2[1] - 0.0;
			residuals[2] = r2[2] - 1.0;

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					//dres0/ dr0
					jacobians[0][0] = 1.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;


					//dres1/ dr0
					jacobians[0][3] = 0.0;
					jacobians[0][4] = 1.0;
					jacobians[0][5] = 0.0;

					//dres2/ dr0
					jacobians[0][6] = 0.0;
					jacobians[0][7] = 0.0;
					jacobians[0][8] = 1.0;

				}
			}
			return true;
		}
	};

	class ZeroTranslation : public SizedCostFunction<1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		ZeroTranslation() {};
		~ZeroTranslation() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const* t = parameters[0];

			residuals[0] = t[0];

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					//dres0/ dt
					jacobians[0][0] = 1.0;
				}
			}
			return true;
		}
	};
}

namespace RegularizationTerm
{
	//input parameters: r0, t_i[0], t_j[0]
	class F0 : public SizedCostFunction<1, 3, 1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F0( int graphId, int ndIdx_i, int ndIdx_j)
		{
			
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F0() {};
	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const*r = parameters[0];
		  double t_i = parameters[1][0];
		  double t_j = parameters[2][0];

		  residuals[0] = (r[0]*tmp[0]+r[1]*tmp[1]+r[2]*tmp[2] + t_i - t_j - tmp[0]);
		  if( jacobians != NULL )
		  {
			  if( jacobians[0] != NULL )
			  {
				  jacobians[0][0] = tmp[0];
				  jacobians[0][1] = tmp[1];
				  jacobians[0][2] = tmp[2];
			  }
			  if( jacobians[1] != NULL )
				jacobians[1][0] =  1.0;
			  if( jacobians[2] != NULL )
				jacobians[2][0] = -1.0;
		  }
		  return true;
	  }

	private:
		vnl_vector_fixed<double, 3> tmp; 
	};

	//input parameters:  r1, t_i[1], t_j[1]		
	class F1 : public SizedCostFunction<1, 3, 1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F1(int graphId, int ndIdx_i, int ndIdx_j)
		{
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F1() {};
	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const*r = parameters[0];
		  double t_i = parameters[1][0];
		  double t_j = parameters[2][0];

		  residuals[0] = (r[0]*tmp[0]+r[1]*tmp[1]+r[2]*tmp[2] + t_i - t_j - tmp[1]);
		  if( jacobians != NULL )
		  {
			  if( jacobians[0] != NULL )
			  {
				  jacobians[0][0] = tmp[0];
				  jacobians[0][1] = tmp[1];
				  jacobians[0][2] = tmp[2];
			  }
			  if( jacobians[1] != NULL )
				jacobians[1][0] =  1.0;
			  if( jacobians[2] != NULL )
				jacobians[2][0] = -1.0;
		  }
		  return true;
	  }

	private:
		vnl_vector_fixed<double, 3> tmp; 
	};

	//input parameters:  r2, t_i[2], t_j[2]
	class F2 : public SizedCostFunction<1, 3, 1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F2(int graphId, int ndIdx_i, int ndIdx_j)
		{
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F2() {};
	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  double const*r = parameters[0];
		  double t_i = parameters[1][0];
		  double t_j = parameters[2][0];

		  residuals[0] = (r[0]*tmp[0]+r[1]*tmp[1]+r[2]*tmp[2] + t_i - t_j - tmp[2]);
		  if( jacobians != NULL )
		  {
			  if( jacobians[0] != NULL )
			  {
				  jacobians[0][0] = tmp[0];
				  jacobians[0][1] = tmp[1];
				  jacobians[0][2] = tmp[2];
			  }
			  if( jacobians[1] != NULL )
				jacobians[1][0] =  1.0;
			  if( jacobians[2] != NULL )
				jacobians[2][0] = -1.0;
		  }
		  return true;
	  }

	private:
		vnl_vector_fixed<double, 3> tmp;
	};
};

namespace RegularizationApproxTerm
{
	//input parameters: r0, t_i[0]
	class F0 : public SizedCostFunction<1, 3, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F0(int graphId, int ndIdx_i, int ndIdx_j, double t_j)
		{
			
			this->t_j = t_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F0() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const*r = parameters[0];
			double t_i = parameters[1][0];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[0]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = tmp[0];
					jacobians[0][1] = tmp[1];
					jacobians[0][2] = tmp[2];
				}
				if (jacobians[1] != NULL)
					jacobians[1][0] = 1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp;
		double t_j;
	};

	class F0_ : public SizedCostFunction<1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F0_(int graphId, int ndIdx_i, int ndIdx_j)
		{
			this->graphId = graphId;
			this->ndIdx_i = ndIdx_i;
			this->ndIdx_j = ndIdx_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F0_() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			DeformGraphNode const& nd_i = graphs[graphId]->nodes[ndIdx_i];
			DeformGraphNode const& nd_j = graphs[graphId]->nodes[ndIdx_j];

			double const* r = &(nd_i.A[0][0]);
			double t_i = nd_i.t[0];
			double t_j = nd_j.t[0];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[0]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = -1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp;
		int graphId;
		int ndIdx_i;
		int ndIdx_j;
	};

	//input parameters:  r1, t_i[1], t_j[1]		
	class F1 : public SizedCostFunction<1, 3, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F1(int graphId, int ndIdx_i, int ndIdx_j, double t_j)
		{
			
			this->t_j = t_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F1() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const*r = parameters[0];
			double t_i = parameters[1][0];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[1]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = tmp[0];
					jacobians[0][1] = tmp[1];
					jacobians[0][2] = tmp[2];
				}
				if (jacobians[1] != NULL)
					jacobians[1][0] = 1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp;
		double t_j;
	};

	class F1_ : public SizedCostFunction<1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F1_(int graphId, int ndIdx_i, int ndIdx_j)
		{
			this->graphId = graphId;
			this->ndIdx_i = ndIdx_i;
			this->ndIdx_j = ndIdx_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F1_() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			DeformGraphNode const& nd_i = graphs[graphId]->nodes[ndIdx_i];
			DeformGraphNode const& nd_j = graphs[graphId]->nodes[ndIdx_j];

			double const* r = &(nd_i.A[1][0]);
			double t_i = nd_i.t[1];
			double t_j = nd_j.t[1];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[1]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = -1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp;
		int graphId;
		int ndIdx_i;
		int ndIdx_j;
	};

	//input parameters:  r2, t_i[2], t_j[2]
	class F2 : public SizedCostFunction<1, 3, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F2(int graphId, int ndIdx_i, int ndIdx_j, double t_j)
		{
			
			this->t_j = t_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F2() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double const*r = parameters[0];
			double t_i = parameters[1][0];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[2]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = tmp[0];
					jacobians[0][1] = tmp[1];
					jacobians[0][2] = tmp[2];
				}
				if (jacobians[1] != NULL)
					jacobians[1][0] = 1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp;
		double t_j;
	};

	class F2_ : public SizedCostFunction<1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F2_(int graphId, int ndIdx_i, int ndIdx_j)
		{
			this->graphId = graphId;
			this->ndIdx_i = ndIdx_i;
			this->ndIdx_j = ndIdx_j;
			this->tmp = graphs[graphId]->nodes[ndIdx_j].g - graphs[graphId]->nodes[ndIdx_i].g;
		}
		~F2_() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			DeformGraphNode const& nd_i = graphs[graphId]->nodes[ndIdx_i];
			DeformGraphNode const& nd_j = graphs[graphId]->nodes[ndIdx_j];

			double const* r = &(nd_i.A[2][0]);
			double t_i = nd_i.t[2];
			double t_j = nd_j.t[2];

			residuals[0] = (r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2] + t_i - t_j - tmp[2]);
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = -1.0;
			}
			return true;
		}

	private:
		vnl_vector_fixed<double, 3> tmp; 
		int graphId;
		int ndIdx_i;
		int ndIdx_j;
	};
};

namespace NodesDistPreserveTerm
{
//input parameters: t_i, t_j
	class F : public SizedCostFunction<1, 1, 1, 1, 1, 1, 1>, public DeformGraphOptMultiDataStatic
	{
	public:
		F(int graphIdx_, int ndIdx_i_, int ndIdx_j_)
		{
			assert( graphIdx_ < graphs.size() );
			this->graphIdx = graphIdx_;

			assert( ndIdx_i_ < graphs[graphIdx]->nodes.size() );
			assert( ndIdx_j_ < graphs[graphIdx]->nodes.size() );
			this->ndIdx_i = ndIdx_i_;
			this->ndIdx_j = ndIdx_j_;
		}
		~F() {};

	public:
	  virtual bool Evaluate(double const* const* parameters,
							double* residuals,
							double** jacobians) const
	  {
		  vnl_vector_fixed<double, 3> const& g_i = graphs[graphIdx]->nodes[ndIdx_i].g;
		  vnl_vector_fixed<double, 3> const& g_j = graphs[graphIdx]->nodes[ndIdx_j].g;
		  
		  vnl_vector_fixed<double, 3> const& t_i = graphs[graphIdx]->nodes[ndIdx_i].t;
		  vnl_vector_fixed<double, 3> const& t_j = graphs[graphIdx]->nodes[ndIdx_j].t;
		  assert( t_i[0] == parameters[0][0]);
		  assert( t_i[1] == parameters[1][0]);
		  assert( t_i[2] == parameters[2][0]);
		  assert( t_j[0] == parameters[3][0]);
		  assert( t_j[1] == parameters[4][0]);
		  assert( t_j[2] == parameters[5][0]);

		  vnl_vector_fixed<double, 3> tmp1 = g_i - g_j;
		  vnl_vector_fixed<double, 3> tmp2 = t_i - t_j + tmp1;
		  double tmp1_norm = std::sqrt(dot_product(tmp1, tmp1));
		  double tmp2_norm = std::sqrt(dot_product(tmp2, tmp2));
		  
		  residuals[0] = tmp2_norm-tmp1_norm;
		  
		  if( jacobians != NULL )
		  {
				  if( jacobians[0] != NULL )
					  jacobians[0][0] = tmp2[0]/tmp2_norm;
				  if( jacobians[1] != NULL )
					  jacobians[1][0] = tmp2[1]/tmp2_norm;
				  if( jacobians[2] != NULL )
					  jacobians[2][0] = tmp2[2]/tmp2_norm;
				  if( jacobians[3] != NULL )
					  jacobians[3][0] = -tmp2[0]/tmp2_norm;
				  if( jacobians[4] != NULL )
					  jacobians[4][0] = -tmp2[1]/tmp2_norm;
				  if( jacobians[5] != NULL )
					  jacobians[5][0] = -tmp2[2]/tmp2_norm;
		  }
		  return true;
	  }

	private:
		int graphIdx;
		int ndIdx_i;
		int ndIdx_j;
	};
};

namespace KeyPointsTerm
{
	//parameters: [r, t; r, t; ...]
	class F : public CostFunction, public DeformGraphOptMultiDataStatic
	{
	public:
		F(int graphIdx_, int keyPairIdx, int cpntIdx_)
		{
			assert( graphIdx_ < graphs.size() );
			this->graphIdx = graphIdx_;
			
			NeighborGraphNodesOfPointList* ngns_key_list = ngns_key_points[graphIdx];
			this->ngn = &((*ngns_key_list)[keyPairIdx]);
			assert(this->ngn->neighborIndices.size() > 0);
			
			this->p = matched_points_3d[graphIdx]->points_1[keyPairIdx].data_block();
			this->q = matched_points_3d[graphIdx]->points_2[keyPairIdx].data_block();

			assert(cpntIdx_ < 3);
			this->cpntIdx = cpntIdx_;

			this->set_num_residuals(1);
			int ngn_num = this->ngn->neighborIndices.size();
			for(int i=0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(1); //t
			}
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
					double* residuals,
					double** jacobians) const
		{
			int ngn_num = this->ngn->neighborIndices.size();
			DeformGraph const* graph = this->graphs[graphIdx];

			vnl_vector_fixed<double, 3> vp(p);

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix<double> dR_drod(9, 3);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			residuals[0] = 0.0;
			vnl_matrix<double> df_dR_global(1, 9);
			df_dR_global.fill(0.0);
			vnl_vector_fixed<double, 3> df_dT_global(0.0);
			for(int i=0; i<ngn_num; i++)
			{
				double const* r_k = parameters[2*i];
				double t_k = parameters[2*i+1][0];
				int ndIdx = this->ngn->neighborIndices[i];
				double w_k = this->ngn->weights[i];
				//vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
				vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;

				double tmp[3];
				tmp[0] = vp[0]-g_k[0];
				tmp[1] = vp[1]-g_k[1];
				tmp[2] = vp[2]-g_k[2];

				residuals[0] += w_k * ( r_k[0]*tmp[0]+r_k[1]*tmp[1]+r_k[2]*tmp[2] + g_k[cpntIdx] + t_k);

				if( jacobians != NULL )
				{
					double w = w_k;// * w_key;
					if( jacobians[2*i] != NULL )
					{
						jacobians[2*i][0] = -tmp[0] * w;
						jacobians[2*i][1] = -tmp[1] * w;
						jacobians[2*i][2] = -tmp[2] * w;
					}
					if( jacobians[2*i+1] != NULL )
					{
						jacobians[2*i+1][0] = -w;
					}
				}
			}
			residuals[0] = ( q[cpntIdx] - residuals[0] );

			return true;
		}
	private:
		NeighborGraphNodesOfPoint *ngn;
		double* p;
		double* q;
		int graphIdx;
		int cpntIdx;
	};

	//parameters: [r0, r1, r2; t0, t1, t2 ...] & [rod_global, t_global]
	//residuals: vp_t - vq
	class F2 : public CostFunction, public DeformGraphOptMultiDataStatic
	{
	public:
		F2(int graphIdx_, int keyPairIdx)
		{
			assert(graphIdx_ < graphs.size());
			this->graphIdx = graphIdx_;

			NeighborGraphNodesOfPointList* ngns_key_list = ngns_key_points[graphIdx];
			this->ngn = &((*ngns_key_list)[keyPairIdx]);
			assert(this->ngn->neighborIndices.size() > 0);

			this->p = matched_points_3d[graphIdx]->points_1[keyPairIdx].data_block();
			this->q = matched_points_3d[graphIdx]->points_2[keyPairIdx].data_block();

			this->set_num_residuals(3);
			int ngn_num = this->ngn->neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(1); //t
				this->mutable_parameter_block_sizes()->push_back(1); //t
				this->mutable_parameter_block_sizes()->push_back(1); //t
			}
			this->mutable_parameter_block_sizes()->push_back(3);
			this->mutable_parameter_block_sizes()->push_back(3);
		}
		~F2() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			int ngn_num = this->ngn->neighborIndices.size();
			DeformGraph const* graph = this->graphs[graphIdx];

			vnl_vector_fixed<double, 3> vp(p);

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix<double> dR_drod(9, 3);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			vnl_matrix_fixed<double, 3, 3> const&df_dvptn = R_global;

			vnl_vector_fixed<double, 3> vp_tn(0.0); //nonrigidly transformed point
			for (int i = 0; i<ngn_num; i++)
			{
				double const* r_k = parameters[2 * i];
				int ndIdx = this->ngn->neighborIndices[i];
				double w_k = this->ngn->weights[i];
				vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
				vnl_vector_fixed<double, 3> const&t_k = graph->nodes[ndIdx].t;
				vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;

				vnl_vector_fixed<double, 3> tmp;
				tmp[0] = vp[0] - g_k[0];
				tmp[1] = vp[1] - g_k[1];
				tmp[2] = vp[2] - g_k[2];

				vp_tn += (A_k *tmp + g_k + t_k)*w_k;								

				if (jacobians != NULL)
				{
					double w = w_k;
					if (jacobians[6 * i] != NULL)
					{
						//df0/dr0
						jacobians[6 * i][0] = R_global(0, 0)*tmp[0]*w_k;
						jacobians[6 * i][1] = R_global(0, 0)*tmp[1]*w_k;
						jacobians[6 * i][2] = R_global(0, 0)*tmp[2]*w_k;

						//df1/dr0
						jacobians[6 * i][3] = R_global(1, 0)*tmp[0]*w_k;
						jacobians[6 * i][4] = R_global(1, 0)*tmp[1]*w_k;
						jacobians[6 * i][5] = R_global(1, 0)*tmp[2]*w_k;

						//df2/dr0
						jacobians[6 * i][6] = R_global(2, 0)*tmp[0]*w_k;
						jacobians[6 * i][7] = R_global(2, 0)*tmp[1]*w_k;
						jacobians[6 * i][8] = R_global(2, 0)*tmp[2]*w_k;
					}
					if (jacobians[6 * i+1] != NULL)
					{
						//df0/dr1
						jacobians[6 * i+1][0] = R_global(0, 1)*tmp[0]*w_k;
						jacobians[6 * i+1][1] = R_global(0, 1)*tmp[1]*w_k;
						jacobians[6 * i+1][2] = R_global(0, 1)*tmp[2]*w_k;
									   
						//df1/dr1	   
						jacobians[6 * i+1][3] = R_global(1, 1)*tmp[0]*w_k;
						jacobians[6 * i+1][4] = R_global(1, 1)*tmp[1]*w_k;
						jacobians[6 * i+1][5] = R_global(1, 1)*tmp[2]*w_k;
									   
						//df2/dr1	   
						jacobians[6 * i+1][6] = R_global(2, 1)*tmp[0]*w_k;
						jacobians[6 * i+1][7] = R_global(2, 1)*tmp[1]*w_k;
						jacobians[6 * i+1][8] = R_global(2, 1)*tmp[2]*w_k;
					}
					if (jacobians[6 * i + 2] != NULL)
					{
						//df0/dr2
						jacobians[6 * i + 2][0] = R_global(0, 2)*tmp[0]*w_k;
						jacobians[6 * i + 2][1] = R_global(0, 2)*tmp[1]*w_k;
						jacobians[6 * i + 2][2] = R_global(0, 2)*tmp[2]*w_k;

						//df1/dr2	   
						jacobians[6 * i + 2][3] = R_global(1, 2)*tmp[0]*w_k;
						jacobians[6 * i + 2][4] = R_global(1, 2)*tmp[1]*w_k;
						jacobians[6 * i + 2][5] = R_global(1, 2)*tmp[2]*w_k;

						//df2/dr2	   
						jacobians[6 * i + 2][6] = R_global(2, 2)*tmp[0]*w_k;
						jacobians[6 * i + 2][7] = R_global(2, 2)*tmp[1]*w_k;
						jacobians[6 * i + 2][8] = R_global(2, 2)*tmp[2]*w_k;
					}

					if (jacobians[6 * i + 3] != NULL)
					{
						//df0/dt0
						jacobians[6 * i + 3][0] = R_global(0, 0)*w_k;

						//df1/dt0
						jacobians[6 * i + 3][1] = R_global(1, 0)*w_k;

						//df2/dt0
						jacobians[6 * i + 3][2] = R_global(2, 0)*w_k;
					}
					if (jacobians[6 * i + 4] != NULL)
					{
						//df0/dt1
						jacobians[6 * i + 4][0] = R_global(0, 1)*w_k;

						//df1/dt1
						jacobians[6 * i + 4][1] = R_global(1, 1)*w_k;

						//df2/dt1
						jacobians[6 * i + 4][2] = R_global(2, 1)*w_k;
					}
					if (jacobians[6 * i + 5] != NULL)
					{
						//df0/dt2
						jacobians[6 * i + 5][0] = R_global(0, 2)*w_k;

						//df1/dt2
						jacobians[6 * i + 5][1] = R_global(1, 2)*w_k;

						//df2/dt2
						jacobians[6 * i + 5][2] = R_global(2, 2)*w_k;
					}
				}
			}

			if (jacobians != NULL)
			{
				if (jacobians[6 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> df_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						df_dR(0, i) = vp_tn[i];
						df_dR(1, 3+i) = vp_tn[i];
						df_dR(2, 6+i) = vp_tn[i];
					}
					vnl_matrix_fixed<double, 3, 3> df_drod = df_dR * dR_drod;

					memcpy(jacobians[6 * ngn_num], df_drod.data_block(), 9 * sizeof(double));
				}

				if (jacobians[6 * ngn_num+1] != NULL)
				{
					//df0/dt
					jacobians[6 * ngn_num + 1][0] = 1.0;
					jacobians[6 * ngn_num + 1][1] = 0.0;
					jacobians[6 * ngn_num + 1][2] = 0.0;

					//df1/dt
					jacobians[6 * ngn_num + 1][3] = 0.0;
					jacobians[6 * ngn_num + 1][4] = 1.0;
					jacobians[6 * ngn_num + 1][5] = 0.0;

					//df2/dt
					jacobians[6 * ngn_num + 1][6] = 0.0;
					jacobians[6 * ngn_num + 1][7] = 0.0;
					jacobians[6 * ngn_num + 1][8] = 1.0;
				}
			}

			vnl_vector_fixed<double, 3> vp_t = R_global*vp_tn + T_global;
			residuals[0] = vp_t[0] - q[0];
			residuals[1] = vp_t[1] - q[1];
			residuals[2] = vp_t[2] - q[2];

			return true;
		}
	private:
		NeighborGraphNodesOfPoint *ngn;
		double* p;
		double* q;
		int graphIdx;
	};

};
#endif


//===============local functions=============
// the M is stored in row major order
inline void mat3x3_mult_vec3(double const*M, double const*a, double *b)
{
	b[0] = M[0]*a[0] + M[1]*a[1] + M[2]*a[2];
	b[1] = M[3]*a[0] + M[4]*a[1] + M[5]*a[2];
	b[2] = M[6]*a[0] + M[7]*a[1] + M[8]*a[2];
}
inline void mat3x3_trans_mult_vec3(double const*M, double const*a, double *b)
{
	b[0] = M[0]*a[0] + M[3]*a[1] + M[6]*a[2];
	b[1] = M[1]*a[0] + M[4]*a[1] + M[7]*a[2];
	b[2] = M[2]*a[0] + M[5]*a[1] + M[8]*a[2];
}
//the M is stored in colomn major order
inline void mat3x3_colmj_mult_vec3(double const*M, double const*a, double *b)
{
	b[0] = M[0]*a[0] + M[3]*a[1] + M[6]*a[2];
	b[1] = M[1]*a[0] + M[4]*a[1] + M[7]*a[2];
	b[2] = M[2]*a[0] + M[5]*a[1] + M[8]*a[2];
}
inline void mat3x3_inverse(double const*M, double *M_inv)
{
	vnl_matrix_fixed<double, 3, 3> Mat(M);
	vnl_matrix_fixed<double, 3, 3> Mat_inv = vnl_inverse(Mat);
	memcpy(M_inv, Mat_inv.data_block(), sizeof(double)*9);
}
inline void mat3x3_colmj_inverse(double const*M_colmj, double *M_inv_rowmj)
{
	double M[9];
	M[0] = M_colmj[0]; M[1] = M_colmj[3]; M[2] = M_colmj[6];
	M[3] = M_colmj[1]; M[4] = M_colmj[4]; M[5] = M_colmj[7];
	M[6] = M_colmj[2]; M[7] = M_colmj[5]; M[8] = M_colmj[8];
	mat3x3_inverse(M, M_inv_rowmj);
}
/* q = A * (p-g) + g + t
 * A--column major
 */
inline void deformation_around_graph_node(double *A, double *t, double *g, double *p, double *q)
{
	double tmp1[3];
	double tmp2[3];
	VecOperation<double>::VecSub(p, g, tmp1, 3);//p-g
	mat3x3_colmj_mult_vec3(A, tmp1, tmp2);
	
	q[0] = tmp2[0]+g[0]+t[0];
	q[1] = tmp2[1]+g[1]+t[1];
	q[2] = tmp2[2]+g[2]+t[2];
}
/* A^t * n  */
inline void deform_normal_around_graph_node(double* A_inv, double* n_in, double* n_out)
{
	mat3x3_trans_mult_vec3(A_inv, n_in, n_out);
}

#ifdef USE_SPARSELM
	inline void push_back_stm_val(splm_stm & stm, int row_idx, int col_idx, double val)
	{
		stm.rowidx[stm.nnz] = row_idx;
		stm.colidx[stm.nnz] = col_idx;
		stm.val[stm.nnz] = val;
		stm.nnz++;
	}
#endif

void DeformGraphPara_to_array(DeformGraph const& graph, vnl_vector<double> &p);
bool array_to_DeformGraphPara(vnl_vector<double> &p, DeformGraph &graph);
}

#endif