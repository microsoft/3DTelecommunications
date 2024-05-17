#ifndef __EDMATCHINGJACOBIANHELPER_H__
#define __EDMATCHINGJACOBIANHELPER_H__
#include "CSurface.h"
#include "UtilVnlMatrix.h"
#include "DeformGraphCore.h"
#include "lmsolver.h"
#include "DDF.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"

struct WeightsPair
{
	float w1;
	float w2;
};

struct HessianBlockRelatedData
{
public:
	HessianBlockRelatedData()
		:bGraphEdge(false),
		vt_indices(NULL),
		weights(NULL)
	{}
	~HessianBlockRelatedData()
	{
		if (vt_indices != NULL)
		{
			delete vt_indices;
			vt_indices = NULL;
		}
		if (weights != NULL)
		{
			delete weights;
			weights = NULL;
		}
	}
public:
	bool bGraphEdge;
	std::vector<int> *vt_indices;
	std::vector<std::pair<float, float>> *weights; //weights of each associated vertices to the two ED nodes
};
//depdendency from vnl matrix -->vnl_c_vector -->vnl_numeric_traits
template <>
class VNL_EXPORT vnl_numeric_traits<HessianBlockRelatedData *> : public vnl_numeric_traits<int>
{
public:
	//: Additive identity
	static constexpr bool zero = false;
	//: Multiplicative identity
	static constexpr bool one = true;
	//: Maximum value which this type can assume
	static constexpr bool maxval = true;
	//: Return value of abs()
	typedef unsigned int abs_t;
	//: Name of a type twice as long as this one for accumulators and products.
	typedef unsigned int double_t;
	//: Name of type which results from multiplying this type with a double
	typedef double real_t;
};


struct Triplet
{
public:
	Triplet(int r = -1, int c = -1, double v = 0)
		:row(r), col(c), val(v) {}
	int row;
	int col;
	double val;
};

class EDMatchingJacobianHelper
{
public:

	static bool volumetric_fusion(DDF &ddf, NonrigidMatching::DeformGraph const&graph,
		vector<cv::Mat> depthMats, vector<GCameraView*> cams_d,
		vector<cv::Mat> imgs, vector<GCameraView*> cams_clr);

	static bool allocate_vertices_and_graph_edges(NonrigidMatching::DeformGraph const& graph,
		NonrigidMatching::NeighborGraphNodesOfPointList const& ngns,
		vnl_matrix<HessianBlockRelatedData*> &hessian_related_data
		);
	static bool init_hessian_blocks(vnl_matrix<HessianBlockRelatedData*> &hessian_related_data,
		BlockedHessianMatrix &hessian,
		vnl_vector<float> & b,
		int* &vt_indices, //vt indices
		WeightsPair *&vt_node_weights, //
		int &vt_indices_count
		);

	static double evaluate_data_term(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K,
		float w_data);

	static double evaluate_data_term(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
		float w_data, float mu);

	static double evaluate_data_term2(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		vector<int2> &vts_mask,
		vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
		float w_data, float mu, int frmIdx = -1, int iter = -1);


	static double evaluate_data_term3(CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
									  CSurface<float> const& surface_t,
									  vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams,
									  float w_data, float mu);

	static bool fill_jacobian_data(vector<Triplet> &jac, CSurface<float> const& surface,
		NonrigidMatching::DeformGraph const& graph, NonrigidMatching::NeighborGraphNodesOfPointList const& ngns,
		CSurface<float> const& surface_t,
		cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K,
		float w_data);

	static bool fill_hessians_data(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		int const* vt_indices, WeightsPair const* vt_node_weights,
		CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		cv::Mat const& depthMat_trg, vnl_matrix_fixed<float, 3, 3> const& K,
		float w_data);

	static bool fill_hessians_data(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		int const* vt_indices, WeightsPair const* vt_node_weights,
		CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		vector<cv::Mat> depthMat_trg, vector<GCameraView*> cams,
		float w_data, float mu);

	static bool fill_hessians_data2(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		vector<int2> &vts_mask,
		int const* vt_indices, WeightsPair const* vt_node_weights,
		CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams, 
		float w_data, float mu,
		int frmIdx=-1, int iter=-1 );

	static bool fill_hessians_data3(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		int const* vt_indices, WeightsPair const* vt_node_weights,
		CSurface<float> const& surface, NonrigidMatching::DeformGraph const& graph,
		CSurface<float> const& surface_t,
		vector<cv::Mat> depthMats_trg, vector<GCameraView*> cams, float w_data,
		float mu);

	static double evaluate_rot_term(NonrigidMatching::DeformGraph const& graph, float w_rot);
	static bool fill_hessians_rot(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		NonrigidMatching::DeformGraph const& graph, float w_rot);

	static double evaluate_reg_term(NonrigidMatching::DeformGraph const& graph, vnl_matrix<HessianBlockRelatedData*> const& hessian_related_data,
									float w_reg);
	static double evaluate_reg_term(NonrigidMatching::DeformGraph const& graph,
									vnl_matrix<bool> const& reg_mat, float w_reg);
	static bool fill_hessians_reg(BlockedHessianMatrix &hessian, vnl_vector<float> &b, double *cost,
		NonrigidMatching::DeformGraph const& graph, float w_reg);

	static bool save_triplets_matlab(vector<Triplet> const&triplets, char const* file_name);
};

#endif