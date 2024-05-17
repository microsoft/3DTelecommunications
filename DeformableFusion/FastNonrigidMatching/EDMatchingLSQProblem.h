#ifndef __EDMATCHINGLSQPROBLEM_H__
#define __EDMATCHINGLSQPROBLEM_H__
#include "CSurface.h"
#include "UtilVnlMatrix.h"
#include "DeformGraphCore.h"
#include "lmsolver.h"

#include "EDMatchingJacobianHelper.h"

class EDMatchingLSQProblem : public LSQProblem
{
public:
	EDMatchingLSQProblem(char const* dir_debug = NULL)
		: vt_indices_(NULL), vt_node_weights_(NULL), dir_debug_(dir_debug) {}

	~EDMatchingLSQProblem()
	{
		free_memory();
	}

	void set_problem(CSurface<float> *surface, NonrigidMatching::DeformGraph *graph, NonrigidMatching::NeighborGraphNodesOfPointList *ngns,
					 vector<cv::Mat> depthMats, vector<GCameraView*> cams,
						double w_sdf, double w_rot, double w_reg, double mu);
	void free_memory();

public:
	bool init(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf);
	bool evaluate(BlockedHessianMatrix &jtj, vnl_vector<float> &jtf, int frmIdx=-1, int iter=-1);
	double evaluateCurrentCost(int frmIdx=-1, int iter=-1); // ||F(x)||^2
	bool updateX(vnl_vector<float> const&dx);//update state variables
	bool getCurrentX(vnl_vector<float> &x); //return state variables as a vector
	bool setCurrentX(vnl_vector<float> const&x);
	void saveIntermediate(int frmIdx);

public:
	CSurface<float>* get_surface_src() { return this->surface_src; }
	vector<cv::Mat> get_depthMats_trg() { return this->depthMats_trg; }
	NonrigidMatching::NeighborGraphNodesOfPointList* get_ngns() { return this->ngns; }
	int const* get_vt_indices() const { return this->vt_indices_; }
	WeightsPair const* get_vt_node_weights() const { return vt_node_weights_; };
	int get_vt_indices_count() const { return this->vt_indices_count_; }

private:
	CSurface<float> *surface_src;
	NonrigidMatching::DeformGraph *graph; //state variable X is here
	NonrigidMatching::NeighborGraphNodesOfPointList *ngns;
	vector<cv::Mat> depthMats_trg; //pointer to external memory
	vector<GCameraView*> cams; //pointer to external memory
	double w_sdf;
	double w_rot;
	double w_reg;
	double mu;

private:
	vector<int2> vts_mask;
	char const* dir_debug_;
	int vt_indices_count_;
	int *vt_indices_;
	WeightsPair *vt_node_weights_;
	vnl_matrix<HessianBlockRelatedData*> hessian_related_data_;
};


#endif