#include "stdafx.h"
#include "EDMatchingLSQProblem.h"
#include "DeformGraph.h"
#include <omp.h>
#include "utility.h"
#include "cholmod.h"

using namespace NonrigidMatching;

void EDMatchingLSQProblem::free_memory()
{
	if (this->vt_indices_)
	{
		delete[] vt_indices_;
		vt_indices_ = NULL;
	}
	if (this->vt_node_weights_)
	{
		delete[] vt_node_weights_;
		vt_node_weights_ = NULL;
	}

	for (int i = 0; i < hessian_related_data_.rows(); i++)
	for (int j = 0; j < hessian_related_data_.cols(); j++)
	{
		delete hessian_related_data_[i][j];
		hessian_related_data_[i][j] = NULL;
	}
}

void EDMatchingLSQProblem::
set_problem(CSurface<float> *surface,
			NonrigidMatching::DeformGraph *graph,
			NonrigidMatching::NeighborGraphNodesOfPointList *ngns,
			vector<cv::Mat> depthMats, vector<GCameraView*> cams,
			double w_sdf, double w_rot, double w_reg, double mu)
{
	this->surface_src = surface;
	this->graph = graph;
	this->ngns = ngns;
	this->depthMats_trg = depthMats;
	this->cams = cams;
	this->w_sdf = w_sdf;
	this->w_reg = w_reg;
	this->w_rot = w_rot;
	this->mu = mu;

	this->vt_indices_ = NULL;
	this->vt_node_weights_ = NULL;
	this->vts_mask.clear();
	EDMatchingJacobianHelper::allocate_vertices_and_graph_edges(*graph, *ngns, hessian_related_data_);
}

bool EDMatchingLSQProblem::
init(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf)
{
	EDMatchingJacobianHelper::init_hessian_blocks(hessian_related_data_, jtj, jtf, vt_indices_, vt_node_weights_, vt_indices_count_);
	return true;
}

bool EDMatchingLSQProblem::
evaluate(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf, int frmIdx, int iter)
{
	CSurface<float> surface_src_t = *surface_src;
	__tic__();
	NonrigidMatching::transform_Surface_with_DeformGraph(surface_src_t, *graph, *ngns, false);
	LOGGER()->debug("surface warp: %f", __toc__());

	if (jtj.blk_num == 0 || jtf.size() == 0)
	{
		init(jtj, jtf);
	}

	for (int i = 0; i < jtj.blk_num; i++)
	{
		memset(jtj.blocks[i].vals, 0, sizeof(float)*jtj.blocks[i].para_count*jtj.blocks[i].para_count);
	}
	jtf.fill(0.0);

	__tic__();
	double cost_data = 0.0;
	EDMatchingJacobianHelper::fill_hessians_data2(jtj, jtf, &cost_data, this->vts_mask, vt_indices_, vt_node_weights_, 
													*surface_src, *graph, surface_src_t, 
													depthMats_trg, cams, 
													w_sdf, mu, frmIdx, iter);
	double cost_rot = 0.0;
	EDMatchingJacobianHelper::fill_hessians_rot(jtj, jtf, &cost_rot, *graph, w_rot);
	double cost_reg = 0.0;
	EDMatchingJacobianHelper::fill_hessians_reg(jtj, jtf, &cost_reg, *graph, w_reg);

	LOGGER()->info("cost when evaluate jtj: data<%f>; rot<%f>; reg<%f>\n", cost_data, cost_rot, cost_reg);

	LOGGER()->debug("jacobian %f", __toc__());

	return true;
}

// ||F(x)||^2
double EDMatchingLSQProblem::evaluateCurrentCost(int frmIdx, int iter) 
{
	__tic__();
	CSurface<float> surface_src_t = *surface_src;
	NonrigidMatching::transform_Surface_with_DeformGraph(surface_src_t, *graph, *ngns, false);
	LOGGER()->debug("surface warp: %f", __toc__());

	if (dir_debug_)
	{
		char name[500];
		sprintf(name, "%s/surface_t_iter%03d.bin", dir_debug_, iter);
		surface_src_t.writeToFileBIN(name);
	}

	__tic__();
	__tic__();
	double cost_data = EDMatchingJacobianHelper::evaluate_data_term2(*surface_src, *graph, surface_src_t, this->vts_mask, depthMats_trg, cams, 
																	w_sdf, mu, frmIdx, iter);
	LOGGER()->debug("data cost eveluation %f", __toc__());

	double cost_rot = EDMatchingJacobianHelper::evaluate_rot_term(*graph, w_rot);

	double cost_reg = EDMatchingJacobianHelper::evaluate_reg_term(*graph, hessian_related_data_, w_reg);

	LOGGER()->debug("cost: data<%f>; rot<%f>; reg<%f>", cost_data, cost_rot, cost_reg);
	LOGGER()->debug("cost eveluation %f", __toc__());


	return cost_data + cost_rot + cost_reg;
}

//update state variables
bool EDMatchingLSQProblem::updateX(vnl_vector<float> const&dx)
{
	if (dx.size() != graph->nodes.size() * 12)
	{
		LOGGER()->error("PairwiseEDMatchingLSQProblem::updateX", "dimension does not agree (%d vs %zd)\n", dx.size(), graph->nodes.size()*12);
		return false;
	}

	for (int ndIdx = 0; ndIdx < graph->nodes.size(); ndIdx++)
	{
		float const* dx_p = dx.data_block() + 12 * ndIdx;
		DeformGraphNode &nd = graph->nodes[ndIdx];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			nd.A[i][j] += dx_p[i * 3 + j];
		nd.t[0] += dx_p[9];
		nd.t[1] += dx_p[10];
		nd.t[2] += dx_p[11];
	}

	return true;
}

//return state variables as a vector
bool EDMatchingLSQProblem::getCurrentX(vnl_vector<float> &x)
{
	int para_num = graph->nodes.size() * 12;
	x.set_size(para_num);
	for (int ndIdx = 0; ndIdx < graph->nodes.size(); ndIdx++)
	{
		DeformGraphNode &nd = graph->nodes[ndIdx];
		float *x_p = x.data_block() + 12 * ndIdx;
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			x_p[3*i+j] = nd.A[i][j];
		x_p[9] = nd.t[0];
		x_p[10] = nd.t[1];
		x_p[11] = nd.t[2];
	}

	return true;
}
bool EDMatchingLSQProblem::setCurrentX(vnl_vector<float> const&x)
{
	if (x.size() != graph->nodes.size() * 12)
	{
		LOGGER()->error("PairwiseEDMatchingLSQProblem::updateX", "dimension does not agree (%d vs %zd)\n", x.size(), graph->nodes.size() * 12);
		return false;
	}

	for (int ndIdx = 0; ndIdx < graph->nodes.size(); ndIdx++)
	{
		float const* x_p = x.data_block() + 12 * ndIdx;
		DeformGraphNode &nd = graph->nodes[ndIdx];
		for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			nd.A[i][j] = x_p[i * 3 + j];
		nd.t[0] = x_p[9];
		nd.t[1] = x_p[10];
		nd.t[2] = x_p[11];
	}

	return true;
}

void EDMatchingLSQProblem::saveIntermediate(int frmIdx)
{
	if (dir_debug_)
	{
		char name[500];
		sprintf(name, "%s/surface_t_iter%03d.bin", dir_debug_, frmIdx);

		CSurface<float> surface_src_t = *surface_src;
		NonrigidMatching::transform_Surface_with_DeformGraph(surface_src_t, *graph, *ngns, false);

		surface_src_t.writeToFileBIN(name);
	}

}