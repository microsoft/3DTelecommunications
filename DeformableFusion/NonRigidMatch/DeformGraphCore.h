#ifndef __DEFORMGRAPHCORE_H__
#define __DEFORMGRAPHCORE_H__
#include <vnl\vnl_vector_fixed.h>
#include <vnl\vnl_matrix_fixed.h>
#include <vector>
#include "RigidTransformModel.h"

namespace NonrigidMatching{

	class DeformGraphNode
	{
	public:
		DeformGraphNode()
			: idx(-1),
			vtIdx(-1)
		{
			this->A.set_identity();
			this->t.fill(0.0);
			this->g.fill(0.0);
			this->n.fill(0.0);
		}

		~DeformGraphNode()
		{
			;
		}

	public:
		int idx;
		std::vector<int> neighborIndices;

		vnl_matrix_fixed<double, 3, 3> A;//affine matrix
		vnl_vector_fixed<double, 3> t; //traslation vector
		vnl_vector_fixed<double, 3> g; //node 3d position
		vnl_vector_fixed<double, 3> n;//the normal vector at a node
		int vtIdx;//vertex index on the surface

	public:
		vnl_matrix_fixed<double, 3, 3> A_inv;
	};

	class DeformGraph
	{
	public:
		DeformGraph(double nodes_dist_ = 7.0, int nn_max_ = 5)
			: nodes_dist(nodes_dist_),
			nn_max(nn_max_),
			bAinvFilled(false)
		{
			;
		}
	public:
		std::vector<DeformGraphNode> nodes; // the nodes are positioned based on their indices

	public:
		RigidTransformModel global_rigid;
		double nodes_dist;
		int nn_max;
		bool bAinvFilled;
	public:
		void transform_nodes_to_target()
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				DeformGraphNode &node = nodes[i];
				node.g = global_rigid.rotation()*(node.g + node.t) + global_rigid.t;
			}
		}

		void reverse()
		{
			vnl_matrix_fixed<double, 3, 3> R = global_rigid.rotation();
			vnl_vector_fixed<double, 3> T = global_rigid.t;
			vnl_matrix_fixed<double, 3, 3> R_n = R.transpose();
			vnl_vector_fixed<double, 3> T_n = -R.transpose() * T;

			for (int i = 0; i < nodes.size(); i++)
			{
				vnl_matrix_fixed<double, 3, 3> const& A = this->nodes[i].A;
				vnl_vector_fixed<double, 3> const& t = this->nodes[i].t;
				vnl_vector_fixed<double, 3> const& g = this->nodes[i].g;

				vnl_matrix_fixed<double, 3, 3> A_n = R*vnl_inverse(A)*R.transpose();
				vnl_vector_fixed<double, 3> g_n = R*(g + t) + T;
				vnl_vector_fixed<double, 3> t_n = R*(g - T_n) - g_n;

				this->nodes[i].A = A_n;
				this->nodes[i].t = t_n;
				this->nodes[i].g = g_n;
			}

			this->global_rigid = RigidTransformModel(R_n, T_n);
			this->bAinvFilled = false;
		}

		void fill_A_inv()
		{
			for (int i = 0; i < nodes.size(); i++)
			{
				DeformGraphNode &node = nodes[i];
				node.A_inv = vnl_inverse(node.A);
			}
			bAinvFilled = true;
		}

		void reset(bool bResetNonrigid = true, bool bResetRigid = true)
		{
			if (bResetRigid)
			{
				global_rigid.rod.fill(0.0);
				global_rigid.t.fill(0.0);
			}

			if (bResetNonrigid)
			{
				for (int i = 0; i < nodes.size(); i++)
				{
					DeformGraphNode &node = nodes[i];
					node.A.set_identity();
					node.t.fill(0.0);
				}
				bAinvFilled = false;
			}
		}
	};

	class NeighborGraphNodesOfPoint
	{
	public:
		std::vector<double> weights;
		std::vector<int> neighborIndices;
	};

	typedef std::vector<NeighborGraphNodesOfPoint> NeighborGraphNodesOfPointList;

}
#endif