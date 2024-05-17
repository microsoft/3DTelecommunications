//===============================================
//			affines_nonrigid_pw_opt.h
//			Nov.27, 2014. by Mingsong Dou (doums@cs.unc.edu)
//===============================================
#ifndef __AFFINES_NONRIGID_PW_OPT_H__
#define __AFFINES_NONRIGID_PW_OPT_H__
#include "DeformGraph.h"

#include "TSDF.h"
#include "DDF.h"
#include "CSurface.h"
#include "surface_geodesic_distance.h"
#undef NDEBUG
#include <assert.h>
#include "ceres\ceres.h"
using namespace ceres;

namespace NonrigidMatching{

class AffineSetNonrigidPWDataStatic
{
public:
	static bool bUseColorField;

	//parameters
public:
	static DeformGraph *graph; //node the neighboring relations are not used
	static NeighborGraphNodesOfPointList *ngns;
	static std::vector<double> *edge_alphas;

	//constants
public:
	static TSDF const*sdf_target;
	static CSurface<float> const* surface;
	static S3DPointMatchSetIndexed const* match_set;

	//internal temporary variable
public:
	static CSurface<float> surface_t;
	static int iter;
	static char const* work_dir;
	static std::vector<GCameraView*> cams; //at target
	static std::vector< vnl_matrix_fixed<double, 3, 4> > cam_prj_mats; //camera projection matrix
	static std::vector< cv::Mat > imgs; //without lens distortion
	static std::vector< cv::Mat > mats_Rx;
	static std::vector< cv::Mat > mats_Ry;
	static std::vector< cv::Mat > mats_Gx;
	static std::vector< cv::Mat > mats_Gy;
	static std::vector< cv::Mat > mats_Bx;
	static std::vector< cv::Mat > mats_By;
	static std::vector< cv::Mat > depthMats_prj; //project surface_t onto various camera spaces
	static std::vector< cv::Mat > imgs_proj; // for debug only

};

void affine_set_nonrigid_pw_opt(CSurface<float> const& surface_in, 
								DeformGraph *graph, 
								NeighborGraphNodesOfPointList *ngns,
								NeighborGraphNodesOfPointList const& ngns_base, 
								DDF const* sdf_target, vector<cv::Mat> imgs, vector<GCameraView*> cams, 
								S3DPointMatchSetIndexed const& match_set, //only the index of the points_1, and vertices of points_2 is used
								vector<double> &edge_alphas_in_out,
								NonrigidMatchingPara para,
								char const* work_dir = NULL);

void affine_set_nonrigid_pw_opt(CSurface<float> const&surface, DeformGraph *graph, NeighborGraphNodesOfPointList *ngns,
								DDF const* sdf_target, 
								vector<cv::Mat> imgs, vector<GCameraView*> cams,
								S3DPointMatchSetIndexed const& match_set,
								vector<double> &edge_alphas,
								NonrigidMatchingPara paras, 
								char const* work_dir = NULL
								);

class AffineSetNonrigidPWIterCallBack : public ceres::IterationCallback, public AffineSetNonrigidPWDataStatic
{
public:
	virtual CallbackReturnType operator()(const IterationSummary& summary)
	{
		printf("-");
		surface_t = *surface;
		transform_Surface_with_DeformGraph(surface_t, *graph, *ngns, false);

		if (!bUseColorField)
		{
			if (imgs.size() > 0 && cams.size() > 0)
			{
				for (int i = 0; i < depthMats_prj.size(); i++)
				{
					if (!depthMats_prj[i].empty())
						depthMats_prj[i].release();
				}
				depthMats_prj.resize(imgs.size());
#pragma omp parallel for
				for (int i = 0; i < imgs.size(); i++)
				{
					cv::Mat depthMat = ComputeDepthMap<float>(&surface_t, cams[i], remove_clockwise_tri, false, 5.0);
					depthMats_prj[i] = depthMat;
				}
			}
		}

		if (summary.step_is_successful)
			printf("-");

			if (work_dir != NULL)
			{
				char name[500];
				sprintf(name, "%s/surface_t_iter%03d.bin", work_dir, iter);
				surface_t.writeToFileBIN(name);
				sprintf(name, "%s/graph_iter%03d.txt", work_dir, iter);
				saveDeformGraphASCII(*graph, name);
				sprintf(name, "%s/ngns_iter%03d.txt", work_dir, iter);
				save_NeighborGraphNodesOfPoint_ASCII(*ngns, name);

				sprintf(name, "%s/zollhofer_iter%03d.txt", work_dir, iter);
				saveStdVectorASCII(name, *edge_alphas);

				static std::vector<vnl_vector_fixed<double, 3>> clrs_ed_nodes;
				if (clrs_ed_nodes.size() == 0)
				{
					for (int k = 0; k < graph->nodes.size(); k++)
					{
						clrs_ed_nodes.push_back(vnl_vector_fixed<double, 3>(RANDOM, RANDOM, RANDOM));
					}
				}

				texture_surface_from_ngn(surface_t, *ngns, clrs_ed_nodes);
				sprintf(name, "%s/surface_t_clr_iter%03d.bin", work_dir, iter);
				surface_t.writeToFileBIN(name);
			}

			iter++;
		printf(">");
		return SOLVER_CONTINUE;
	}
};

namespace AffineSetNonrigidPWDensePtsTerm
{
	//parameters: [r0, r1, r2, t, w; r0, r1, r2, t, w; ...] & [rod_global, t_global]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int vtIdx_)
		{
			assert(vtIdx_<surface->vtNum);
			this->ngn = &((*ngns)[vtIdx_]);
			this->vtIdx = vtIdx_;

			this->set_num_residuals(1);
			int ngn_num = this->ngn->neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(3); //t
				this->mutable_parameter_block_sizes()->push_back(1); //w
			}
			this->mutable_parameter_block_sizes()->push_back(3); // rod_global
			this->mutable_parameter_block_sizes()->push_back(3); // t_global
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			int ngn_num = this->ngn->neighborIndices.size();

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			float const* v = surface->vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(v[0], v[1], v[2]);
			float const* vt_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt_t(vt_t_[0], vt_t_[1], vt_t_[2]);
			float const* n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);

			residuals[0] = sdf_target->val_at(vt_t.data_block(), NULL, 0.0);

			vnl_vector_fixed<double, 3> df_dvtt = sdf_target->der_at(vt_t.data_block(), n_t.data_block(), 75);
			vnl_vector_fixed<double, 3> df_dvttn = df_dvtt * R_global; //vttn: nonrigid deformed vt

			if (df_dvtt[0] == 0.0 && df_dvtt[1] == 0.0 && df_dvtt[2] == 0.0)
				residuals[0] = 0.0;

			vnl_vector_fixed<double, 3> vp_tn(0.0); //nonrigidly transformed point
			for (int i = 0; i<ngn_num; i++)
			{
				int ndIdx = this->ngn->neighborIndices[i];
				double w_k = this->ngn->weights[i];
				assert(w_k == parameters[5 * i + 4][0]);
				vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
				vnl_vector_fixed<double, 3> const&t_k = graph->nodes[ndIdx].t;
				vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;

				vnl_vector_fixed<double, 3> tmp;
				tmp[0] = vt[0] - g_k[0];
				tmp[1] = vt[1] - g_k[1];
				tmp[2] = vt[2] - g_k[2];

				vnl_vector_fixed<double, 3> tmp2 = A_k*tmp+g_k+t_k;
				vp_tn += tmp2*w_k;

				if (jacobians != NULL)
				{
					double w = w_k;
					if (jacobians[5 * i] != NULL)
					{
						//df0/dr0
						jacobians[5 * i][0] = df_dvttn[0] * tmp[0] * w_k;
						jacobians[5 * i][1] = df_dvttn[0] * tmp[1] * w_k;
						jacobians[5 * i][2] = df_dvttn[0] * tmp[2] * w_k;
					}
					if (jacobians[5 * i + 1] != NULL)
					{
						//df0/dr1
						jacobians[5 * i + 1][0] = df_dvttn[1] * tmp[0] * w_k;
						jacobians[5 * i + 1][1] = df_dvttn[1] * tmp[1] * w_k;
						jacobians[5 * i + 1][2] = df_dvttn[1] * tmp[2] * w_k;
					}
					if (jacobians[5 * i + 2] != NULL)
					{
						//df0/dr2
						jacobians[5 * i + 2][0] = df_dvttn[2] * tmp[0] * w_k;
						jacobians[5 * i + 2][1] = df_dvttn[2] * tmp[1] * w_k;
						jacobians[5 * i + 2][2] = df_dvttn[2] * tmp[2] * w_k;
					}

					if (jacobians[5 * i + 3] != NULL)
					{
						//df0/dt
						jacobians[5 * i + 3][0] = df_dvttn[0] * w_k;
						jacobians[5 * i + 3][1] = df_dvttn[1] * w_k;
						jacobians[5 * i + 3][2] = df_dvttn[2] * w_k;
					}
					if (jacobians[5 * i + 4] != NULL)
					{
						//df0/dw
						jacobians[5 * i + 4][0] = dot_product(tmp2, df_dvttn);
					}
				}
			}

			if (jacobians != NULL)
			{
				if (jacobians[5 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						dvtt_dR(0, i) = vp_tn[i];
						dvtt_dR(1, 3 + i) = vp_tn[i];
						dvtt_dR(2, 6 + i) = vp_tn[i];
					}
					vnl_vector_fixed<double, 9> df_dR = df_dvtt * dvtt_dR;
					vnl_vector_fixed<double, 3> df_drod = df_dR * dR_drod;

					memcpy(jacobians[5 * ngn_num], df_drod.data_block(), 3 * sizeof(double));
				}

				if (jacobians[5 * ngn_num + 1] != NULL)
				{
					//df0/dt
					jacobians[5 * ngn_num + 1][0] = df_dvtt[0];
					jacobians[5 * ngn_num + 1][1] = df_dvtt[1];
					jacobians[5 * ngn_num + 1][2] = df_dvtt[2];
				}
			}
			return true;
		}

	private:
		NeighborGraphNodesOfPoint *ngn;
		int vtIdx;
	};
}

namespace AffineSetNonrigidPWNormalTerm
{
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int vtIdx_)
		{
			assert(vtIdx_<surface->vtNum);
			this->ngn = &((*ngns)[vtIdx_]);
			this->vtIdx = vtIdx_;

			this->set_num_residuals(3);
			int ngn_num = this->ngn->neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(1); //w
			}
			this->mutable_parameter_block_sizes()->push_back(3); // rod_global
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			int ngn_num = this->ngn->neighborIndices.size();

			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			const float *vt = surface->vt_data_block(vtIdx);
			const float *vt_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt_t(vt_t_[0], vt_t_[1], vt_t_[2]);
			const float *n_ = surface->vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n(n_[0], n_[1], n_[2]);
			const float *n_t_ = surface_t.vt_normal(vtIdx);
			vnl_vector_fixed<double, 3> n_t(n_t_[0], n_t_[1], n_t_[2]);

			//asserts
			for (int i = 0; i<ngn_num; i++)
			{
				int ndIdx = this->ngn->neighborIndices[i];
				DeformGraphNode const& node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> A_para;
				A_para.set_row(0, parameters[4 * i]);
				A_para.set_row(1, parameters[4 * i + 1]);
				A_para.set_row(2, parameters[4 * i + 2]);
				assert(A_para == node.A);
				assert(this->ngn->weights[i] == parameters[4 * i + 3][0]);
			}
			vnl_vector_fixed<double, 3> rod_para(parameters[4 * ngn_num]);
			assert(rod_global == rod_para);

			vnl_vector_fixed<double, 3> der = sdf_target->der_at(vt_t.data_block(), n_t.data_block(), 75);
			der.normalize();

			if (der[0] == 0.0 && der[1] == 0.0 && der[2] != 0.0)
			{
				residuals[0] = 0.0;
				residuals[1] = 0.0;
				residuals[2] = 0.0;
				if (jacobians != NULL)
				{
					for (int i = 0; i < ngn_num; i++)
					{
						if (jacobians[4 * i] != NULL)
							memset(jacobians[4 * i], 0, sizeof(double)* 9);
						if (jacobians[4 * i + 1] != NULL)
							memset(jacobians[4 * i + 1], 0, sizeof(double)* 9);
						if (jacobians[4 * i + 2] != NULL)
							memset(jacobians[4 * i + 2], 0, sizeof(double)* 9);
						if (jacobians[4 * i + 3] != NULL)
							memset(jacobians[4 * i + 3], 0, sizeof(double)* 3);
					}
					if (jacobians[4 * ngn_num] != NULL)
						memset(jacobians[4 * ngn_num], 0, sizeof(double)* 9);
				}

				return true;
			}


			residuals[0] = n_t[0] + der[0];
			residuals[1] = n_t[1] + der[1];
			residuals[2] = n_t[2] + der[2];

			if (jacobians != NULL)
			{
				vnl_vector_fixed<double, 3> n_tnu(0.0); //nonrigidly transformed point
				for (int k = 0; k < ngn_num; k++)
				{
					int ndIdx = this->ngn->neighborIndices[k];
					double w_k = this->ngn->weights[k];
					vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
					n_tnu += w_k*vnl_inverse(A_k).transpose()*n;
				}
				vnl_vector_fixed<double, 3> n_tn;
				vnl_matrix_fixed<double, 3, 3> T; // d(n_tn)/d(n_tnu)
				Jac_normalization(n_tnu, n_tn, T);

				vnl_matrix_fixed<double, 3, 3> dres_dntnu = R_global * T;

				for (int i = 0; i < ngn_num; i++)
				{
					int ndIdx = this->ngn->neighborIndices[i];
					double w_k = this->ngn->weights[i];
					vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;

					vnl_matrix_fixed<double, 3, 9> dntnu_dAi;
					Jac_affine_on_normal(A_k, w_k, n, dntnu_dAi);

					vnl_matrix_fixed<double, 3, 9> dres_dAi = dres_dntnu * dntnu_dAi;

					if (jacobians[4 * i] != NULL)
					{
						//dres0/dr0
						jacobians[4 * i][0] = dres_dAi(0, 0);
						jacobians[4 * i][1] = dres_dAi(0, 1);
						jacobians[4 * i][2] = dres_dAi(0, 2);

						//dres1/dr0
						jacobians[4 * i][3] = dres_dAi(1, 0);
						jacobians[4 * i][4] = dres_dAi(1, 1);
						jacobians[4 * i][5] = dres_dAi(1, 2);

						//dres2/dr0
						jacobians[4 * i][6] = dres_dAi(2, 0);
						jacobians[4 * i][7] = dres_dAi(2, 1);
						jacobians[4 * i][8] = dres_dAi(2, 2);
					}

					if (jacobians[4 * i + 1] != NULL)
					{
						//dres0/dr1
						jacobians[4 * i + 1][0] = dres_dAi(0, 3);
						jacobians[4 * i + 1][1] = dres_dAi(0, 4);
						jacobians[4 * i + 1][2] = dres_dAi(0, 5);

						//dres1/dr1
						jacobians[4 * i + 1][3] = dres_dAi(1, 3);
						jacobians[4 * i + 1][4] = dres_dAi(1, 4);
						jacobians[4 * i + 1][5] = dres_dAi(1, 5);

						//dres2/dr1
						jacobians[4 * i + 1][6] = dres_dAi(2, 3);
						jacobians[4 * i + 1][7] = dres_dAi(2, 4);
						jacobians[4 * i + 1][8] = dres_dAi(2, 5);
					}

					if (jacobians[4 * i + 2] != NULL)
					{
						//dres0/dr2
						jacobians[4 * i + 2][0] = dres_dAi(0, 6);
						jacobians[4 * i + 2][1] = dres_dAi(0, 7);
						jacobians[4 * i + 2][2] = dres_dAi(0, 8);

						//dres1/dr2
						jacobians[4 * i + 2][3] = dres_dAi(1, 6);
						jacobians[4 * i + 2][4] = dres_dAi(1, 7);
						jacobians[4 * i + 2][5] = dres_dAi(1, 8);

						//dres2/dr2
						jacobians[4 * i + 2][6] = dres_dAi(2, 6);
						jacobians[4 * i + 2][7] = dres_dAi(2, 7);
						jacobians[4 * i + 2][8] = dres_dAi(2, 8);
					}

					if (jacobians[4 * i + 3] != NULL)
					{
						vnl_vector_fixed<double, 3> dntnu_dw = vnl_inverse(A_k).transpose() * n;
						vnl_vector_fixed<double, 3> dres_dw = dres_dntnu * dntnu_dw;

						//dres/dw
						memcpy(jacobians[4 * i + 3], dres_dw.data_block(), sizeof(double)* 3);
					}
				}

				if (jacobians[4 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> dnt_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						dnt_dR(0, i) = n_tn[i];
						dnt_dR(1, i + 3) = n_tn[i];
						dnt_dR(2, i + 6) = n_tn[i];
					}
					//dres/drod
					vnl_matrix_fixed<double, 3, 3> dres_drod = dnt_dR * dR_drod;
					memcpy(jacobians[4 * ngn_num], dres_drod.data_block(), 9 * sizeof(double));
				}
			}

			return true;
		}

	private:
		NeighborGraphNodesOfPoint *ngn;
		int vtIdx;
	};
}

namespace AffineSetNonrigidPWDenseClrTerm{
	//parameters: [r0, r1, r2, t, w; r0, r1, r2, t, w; ...] & [rod_global, t_global]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int vtIdx_, int camIdx_)
		{
			assert(vtIdx_ < surface->vtNum);
			this->vtIdx = vtIdx_;

			assert(camIdx_ < cams.size());
			this->camIdx = camIdx_;

			this->ngn = &((*ngns)[vtIdx]);
			this->p = surface->vt_data_block(vtIdx);
			if (surface->colors_of_view.size() > camIdx &&
				surface->colors_of_view[camIdx] != NULL)
			{
				this->clr = &(surface->colors_of_view[camIdx_][3 * vtIdx_]);
				if (this->clr[0] == 0 &&
					this->clr[1] == 0 &&
					this->clr[2] == 0)
					this->clr = surface->vt_color(vtIdx);
			}
			else
				this->clr = surface->vt_color(vtIdx);

			if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
			{
				float *clr_t = const_cast<float*>(this->clr);
				clr_t[0] = 0;
				clr_t[1] = 0;
				clr_t[2] = 0;
			}

			this->set_num_residuals(3);
			int ngn_num = this->ngn->neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(3); //t

				this->mutable_parameter_block_sizes()->push_back(1); //w
			}
			this->mutable_parameter_block_sizes()->push_back(3);
			this->mutable_parameter_block_sizes()->push_back(3);
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			float const*q = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 4> q_h(q[0], q[1], q[2], 1.0); //homogeous coord at target surface

			cv::Mat img = imgs[camIdx];
			cv::Mat mat_Rx = mats_Rx[camIdx];
			cv::Mat mat_Ry = mats_Ry[camIdx];
			cv::Mat mat_Gx = mats_Gx[camIdx];
			cv::Mat mat_Gy = mats_Gy[camIdx];
			cv::Mat mat_Bx = mats_Bx[camIdx];
			cv::Mat mat_By = mats_By[camIdx];

			vnl_matrix_fixed<double, 3, 4> const& cam_proj = cam_prj_mats[camIdx];
			vnl_vector_fixed<double, 3> X_h = cam_proj * q_h; //project to image space
			double u = X_h[0] / X_h[2];
			double v = X_h[1] / X_h[2];

			double Ux = 0.0;
			double Uy = 0.0;
			double Uz = 0.0;
			double Vx = 0.0;
			double Vy = 0.0;
			double Vz = 0.0;
			if (jacobians != NULL)
			{
				Ux = cam_proj(0, 0) / X_h[2] - X_h[0] * cam_proj(2, 0) / (X_h[2] * X_h[2]);
				Uy = cam_proj(0, 1) / X_h[2] - X_h[0] * cam_proj(2, 1) / (X_h[2] * X_h[2]);
				Uz = cam_proj(0, 2) / X_h[2] - X_h[0] * cam_proj(2, 2) / (X_h[2] * X_h[2]);
				Vx = cam_proj(1, 0) / X_h[2] - X_h[1] * cam_proj(2, 0) / (X_h[2] * X_h[2]);
				Vy = cam_proj(1, 1) / X_h[2] - X_h[1] * cam_proj(2, 1) / (X_h[2] * X_h[2]);
				Vz = cam_proj(1, 2) / X_h[2] - X_h[1] * cam_proj(2, 2) / (X_h[2] * X_h[2]);
			}

			double Ru = 0.0;
			double Rv = 0.0;
			double Gu = 0.0;
			double Gv = 0.0;
			double Bu = 0.0;
			double Bv = 0.0;
			residuals[0] = 0.0; // for some unknown reason, the optimization does not work with loss function (even trivia loss function) if residual is set to 1.0;
			residuals[1] = 0.0;
			residuals[2] = 0.0;
			if (ROUND(u) < img.cols - 2 && ROUND(v) < img.rows - 2 &&
				ROUND(u) > 2 && ROUND(v) > 2 &&
				(clr[0] != 0 || clr[1] != 0 || clr[2] != 0))
			{
				//check visibility
				double depth = depthMats_prj[camIdx].at<double>(ROUND(v), ROUND(u));
				if (depth > 0.1 && fabs(X_h[2] - depth) < 0.2)
				{
					//int u_i = ROUND(u);
					//int v_i = ROUND(v);
					double clr_img[3];
					pickAColorPixel(img, u, v, clr_img);
					if (clr_img[0] != 0 || clr_img[1] != 0 || clr_img[2] != 0)
					{
						residuals[0] = (clr_img[2] / 255.0 - clr[0]);
						residuals[1] = (clr_img[1] / 255.0 - clr[1]);
						residuals[2] = (clr_img[0] / 255.0 - clr[2]);

						//the gradient on image
						if (jacobians != NULL)
						{
							Ru = pickAMatElement(mat_Rx, u, v) / 255.0;
							Rv = pickAMatElement(mat_Ry, u, v) / 255.0;
							Gu = pickAMatElement(mat_Gx, u, v) / 255.0;
							Gv = pickAMatElement(mat_Gy, u, v) / 255.0;
							Bu = pickAMatElement(mat_Bx, u, v) / 255.0;
							Bv = pickAMatElement(mat_By, u, v) / 255.0;
						}

#ifdef WRITE_INTERMEDIATE
						if ((clr_img[0] != 0 ||
							clr_img[1] != 0 ||
							clr_img[2] != 0))
						{
							int u_i = ROUND(u);
							int v_i = ROUND(v);
							CIE(imgs_proj[camIdx], v_i, u_i, 0) = clr[2] * 255;
							CIE(imgs_proj[camIdx], v_i, u_i, 1) = clr[1] * 255;
							CIE(imgs_proj[camIdx], v_i, u_i, 2) = clr[0] * 255;
						}
						else
						{
							int u_i = ROUND(u);
							int v_i = ROUND(v);
							CIE(imgs_proj[camIdx], v_i, u_i, 0) = clr[2] * 30;
							CIE(imgs_proj[camIdx], v_i, u_i, 1) = clr[1] * 30;
							CIE(imgs_proj[camIdx], v_i, u_i, 2) = clr[0] * 30;
						}
#endif
					}
				}
			}

			if (jacobians != NULL)
			{
				vnl_matrix_fixed<double, 3, 3> dI_dvtt(0.0);
				dI_dvtt(0, 0) = Ru*Ux + Rv*Vx;
				dI_dvtt(0, 1) = Ru*Uy + Rv*Vy;
				dI_dvtt(0, 2) = Ru*Uz + Rv*Vz;
				dI_dvtt(1, 0) = Gu*Ux + Gv*Vx;
				dI_dvtt(1, 1) = Gu*Uy + Gv*Vy;
				dI_dvtt(1, 2) = Gu*Uz + Gv*Vz;
				dI_dvtt(2, 0) = Bu*Ux + Bv*Vx;
				dI_dvtt(2, 1) = Bu*Uy + Bv*Vy;
				dI_dvtt(2, 2) = Bu*Uz + Bv*Vz;

				vnl_matrix_fixed<double, 3, 3> const& dvtt_dvttn = R_global;
				vnl_matrix_fixed<double, 3, 3> dI_dvttn = dI_dvtt*dvtt_dvttn;

				vnl_vector_fixed<double, 3> vt_tn(0.0); //nonrigidly deformed point
				int ngn_num = ngn->neighborIndices.size();
				for (int j = 0; j<ngn_num; j++)
				{
					int k = ngn->neighborIndices[j];
					double w_k = ngn->weights[j];////*w_color;
					DeformGraphNode const&node = graph->nodes[k];
					vnl_vector_fixed<double, 3> const&g_k = node.g;
					vnl_vector_fixed<double, 3> const&t_k = node.t;
					vnl_matrix_fixed<double, 3, 3> const&A_k = node.A;

					vnl_vector_fixed<double, 3> tmp;
					tmp[0] = p[0] - g_k[0];
					tmp[1] = p[1] - g_k[1];
					tmp[2] = p[2] - g_k[2];
					vnl_vector_fixed<double, 3> tmp2 = A_k*tmp + g_k + t_k;
					vt_tn += tmp2*w_k;

					if (jacobians[5 * j] != NULL)
					{
						//d(clr_R)/d(r0)
						jacobians[5 * j][0] = tmp[0] * dI_dvttn(0, 0)*w_k;
						jacobians[5 * j][1] = tmp[1] * dI_dvttn(0, 0)*w_k;
						jacobians[5 * j][2] = tmp[2] * dI_dvttn(0, 0)*w_k;

						//d(clr_G)/d(r0)
						jacobians[5 * j][3] = tmp[0] * dI_dvttn(1, 0)*w_k;
						jacobians[5 * j][4] = tmp[1] * dI_dvttn(1, 0)*w_k;
						jacobians[5 * j][5] = tmp[2] * dI_dvttn(1, 0)*w_k;

						//d(clr_B)/d(r0)
						jacobians[5 * j][6] = tmp[0] * dI_dvttn(2, 0)*w_k;
						jacobians[5 * j][7] = tmp[1] * dI_dvttn(2, 0)*w_k;
						jacobians[5 * j][8] = tmp[2] * dI_dvttn(2, 0)*w_k;
					}
					if (jacobians[5 * j + 1] != NULL)
					{
						//d(clr_R)/d(r1)
						jacobians[5 * j + 1][0] = tmp[0] * dI_dvttn(0, 1)*w_k;
						jacobians[5 * j + 1][1] = tmp[1] * dI_dvttn(0, 1)*w_k;
						jacobians[5 * j + 1][2] = tmp[2] * dI_dvttn(0, 1)*w_k;

						//d(clr_G)/d(r1)
						jacobians[5 * j + 1][3] = tmp[0] * dI_dvttn(1, 1)*w_k;
						jacobians[5 * j + 1][4] = tmp[1] * dI_dvttn(1, 1)*w_k;
						jacobians[5 * j + 1][5] = tmp[2] * dI_dvttn(1, 1)*w_k;

						//d(clr_B)/d(r1)
						jacobians[5 * j + 1][6] = tmp[0] * dI_dvttn(2, 1)*w_k;
						jacobians[5 * j + 1][7] = tmp[1] * dI_dvttn(2, 1)*w_k;
						jacobians[5 * j + 1][8] = tmp[2] * dI_dvttn(2, 1)*w_k;
					}
					if (jacobians[5 * j + 2] != NULL)
					{
						//d(clr_R)/d(r2)
						jacobians[5 * j + 2][0] = tmp[0] * dI_dvttn(0, 2)*w_k;
						jacobians[5 * j + 2][1] = tmp[1] * dI_dvttn(0, 2)*w_k;
						jacobians[5 * j + 2][2] = tmp[2] * dI_dvttn(0, 2)*w_k;

						//d(clr_G)/d(r2)
						jacobians[5 * j + 2][3] = tmp[0] * dI_dvttn(1, 2)*w_k;
						jacobians[5 * j + 2][4] = tmp[1] * dI_dvttn(1, 2)*w_k;
						jacobians[5 * j + 2][5] = tmp[2] * dI_dvttn(1, 2)*w_k;

						//d(clr_B)/d(r2)
						jacobians[5 * j + 2][6] = tmp[0] * dI_dvttn(2, 2)*w_k;
						jacobians[5 * j + 2][7] = tmp[1] * dI_dvttn(2, 2)*w_k;
						jacobians[5 * j + 2][8] = tmp[2] * dI_dvttn(2, 2)*w_k;
					}
					if (jacobians[5 * j + 3] != NULL)
					{
						jacobians[5 * j + 3][0] = dI_dvttn(0, 0)*w_k; //d(clr_R)/d(t0)
						jacobians[5 * j + 3][1] = dI_dvttn(0, 1)*w_k; //d(clr_R)/d(t1)
						jacobians[5 * j + 3][2] = dI_dvttn(0, 2)*w_k; //d(clr_R)/d(t2)

						jacobians[5 * j + 3][3] = dI_dvttn(1, 0)*w_k; //d(clr_G)/d(t0)
						jacobians[5 * j + 3][4] = dI_dvttn(1, 1)*w_k; //d(clr_G)/d(t1)
						jacobians[5 * j + 3][5] = dI_dvttn(1, 2)*w_k; //d(clr_G)/d(t2)

						jacobians[5 * j + 3][6] = dI_dvttn(2, 0)*w_k; //d(clr_B)/d(t0)
						jacobians[5 * j + 3][7] = dI_dvttn(2, 1)*w_k; //d(clr_B)/d(t1)
						jacobians[5 * j + 3][8] = dI_dvttn(2, 2)*w_k; //d(clr_B)/d(t2)
					}
					if (jacobians[5 * j + 4] != NULL)
					{
						vnl_vector_fixed<double, 3> dI_dw = dI_dvttn*tmp2;
						memcpy(jacobians[5 * j + 4], dI_dw.data_block(), sizeof(double)* 3);
					}

				}//end of for-j
				if (jacobians[5 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						dvtt_dR(0, i) = vt_tn[i];
						dvtt_dR(1, 3 + i) = vt_tn[i];
						dvtt_dR(2, 6 + i) = vt_tn[i];
					}
					vnl_matrix_fixed<double, 3, 3> dClr_drod = dI_dvtt * (dvtt_dR * dR_drod);

					memcpy(jacobians[5 * ngn_num], dClr_drod.data_block(), 9 * sizeof(double));
				}

				if (jacobians[5 * ngn_num + 1] != NULL)
				{
					//dclr/dt == dclr/dvt_t
					memcpy(jacobians[5 * ngn_num + 1], dI_dvtt.data_block(), 9 * sizeof(double));
				}
			}//end of if(jacobians != NULL )
			return true;
		}

	private:
		int vtIdx;
		int camIdx;
		NeighborGraphNodesOfPoint *ngn;
		float const* p;
		float const* clr;
	};
};

namespace AffineSetNonrigidPWDenseClrFldTerm{
	//parameters: [r0, r1, r2, t, w; r0, r1, r2, t, w; ...] & [rod_global, t_global]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int vtIdx_)
		{
			assert(vtIdx_<surface->vtNum);
			this->vtIdx = vtIdx_;

			this->set_num_residuals(3);
			int ngn_num = ((*ngns)[vtIdx]).neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(3); //r
				this->mutable_parameter_block_sizes()->push_back(3); //t
				this->mutable_parameter_block_sizes()->push_back(1); //w
			}
			this->mutable_parameter_block_sizes()->push_back(3);
			this->mutable_parameter_block_sizes()->push_back(3);
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			NeighborGraphNodesOfPoint const* ngn = &((*ngns)[vtIdx]);
			int ngn_num = ngn->neighborIndices.size();

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			float const*clr = surface->vt_color(vtIdx);

			float const* v = surface->vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(v[0], v[1], v[2]);
			const float *vt_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt_t(vt_t_[0], vt_t_[1], vt_t_[2]);

			vnl_vector_fixed<float, 3> clr_t = sdf_target->clr_at(vt_t[0], vt_t[1], vt_t[2]);

			// residuals
			if ((clr_t[0] == 0.0 && clr_t[1] == 0.0 && clr_t[2] == 0.0) ||
				_isnan(clr[0]) || _isnan(clr[1]) || _isnan(clr[2]) ||
				(clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0)
				 )
			{
				residuals[0] = 0.0;
				residuals[1] = 0.0;
				residuals[2] = 0.0;
			}
			else
			{
				residuals[0] = clr_t[0] - clr[0];
				residuals[1] = clr_t[1] - clr[1];
				residuals[2] = clr_t[2] - clr[2];
			}

			vnl_vector_fixed<double, 3> der_x(0.0);
			vnl_vector_fixed<double, 3> der_y(0.0);
			vnl_vector_fixed<double, 3> der_z(0.0);
			if (!sdf_target->clr_fld_der_x_at(vt_t.data_block(), der_x.data_block()) ||
				!sdf_target->clr_fld_der_y_at(vt_t.data_block(), der_y.data_block()) ||
				!sdf_target->clr_fld_der_z_at(vt_t.data_block(), der_z.data_block()))
			{
				residuals[0] = 0.0;
				residuals[1] = 0.0;
				residuals[2] = 0.0;
			}

			if (jacobians != NULL)
			{
				vnl_matrix_fixed<double, 3, 3> dClr_dvtt;
				dClr_dvtt.set_column(0, der_x);
				dClr_dvtt.set_column(1, der_y);
				dClr_dvtt.set_column(2, der_z);

				vnl_matrix_fixed<double, 3, 3> dClr_dvttn = dClr_dvtt * R_global;

				vnl_vector_fixed<double, 3> vp_tn(0.0); //nonrigidly transformed point
				for (int i = 0; i<ngn_num; i++)
				{
					int ndIdx = ngn->neighborIndices[i];
					double w_k = ngn->weights[i];//*w_sdf;
					vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;
					vnl_vector_fixed<double, 3> const& t_k = graph->nodes[ndIdx].t;
					vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
					vnl_vector_fixed<double, 3> tmp;
					tmp[0] = vt[0] - g_k[0];
					tmp[1] = vt[1] - g_k[1];
					tmp[2] = vt[2] - g_k[2];
					vnl_vector_fixed<double, 3> tmp2 = A_k *tmp + g_k + t_k;
					vp_tn += tmp2*w_k;

					if (jacobians[5 * i] != NULL)
					{
						// d (clr_r) / dr0
						jacobians[5 * i][1] = tmp[1] * dClr_dvttn[0][0] * w_k;
						jacobians[5 * i][2] = tmp[2] * dClr_dvttn[0][0] * w_k;
						jacobians[5 * i][0] = tmp[0] * dClr_dvttn[0][0] * w_k;

						// d (clr_g) / dr0
						jacobians[5 * i][3] = tmp[0] * dClr_dvttn[1][0] * w_k;
						jacobians[5 * i][4] = tmp[1] * dClr_dvttn[1][0] * w_k;
						jacobians[5 * i][5] = tmp[2] * dClr_dvttn[1][0] * w_k;

						// d (clr_b) / dr0
						jacobians[5 * i][6] = tmp[0] * dClr_dvttn[2][0] * w_k;
						jacobians[5 * i][7] = tmp[1] * dClr_dvttn[2][0] * w_k;
						jacobians[5 * i][8] = tmp[2] * dClr_dvttn[2][0] * w_k;
					}
					if (jacobians[5 * i + 1] != NULL)
					{
						jacobians[5 * i + 1][0] = tmp[0] * dClr_dvttn[0][1] * w_k;
						jacobians[5 * i + 1][1] = tmp[1] * dClr_dvttn[0][1] * w_k;
						jacobians[5 * i + 1][2] = tmp[2] * dClr_dvttn[0][1] * w_k;

						jacobians[5 * i + 1][3] = tmp[0] * dClr_dvttn[1][1] * w_k;
						jacobians[5 * i + 1][4] = tmp[1] * dClr_dvttn[1][1] * w_k;
						jacobians[5 * i + 1][5] = tmp[2] * dClr_dvttn[1][1] * w_k;

						jacobians[5 * i + 1][6] = tmp[0] * dClr_dvttn[2][1] * w_k;
						jacobians[5 * i + 1][7] = tmp[1] * dClr_dvttn[2][1] * w_k;
						jacobians[5 * i + 1][8] = tmp[2] * dClr_dvttn[2][1] * w_k;
					}
					if (jacobians[5 * i + 2] != NULL)
					{
						jacobians[5 * i + 2][0] = tmp[0] * dClr_dvttn[0][2] * w_k;
						jacobians[5 * i + 2][1] = tmp[1] * dClr_dvttn[0][2] * w_k;
						jacobians[5 * i + 2][2] = tmp[2] * dClr_dvttn[0][2] * w_k;

						jacobians[5 * i + 2][3] = tmp[0] * dClr_dvttn[1][2] * w_k;
						jacobians[5 * i + 2][4] = tmp[1] * dClr_dvttn[1][2] * w_k;
						jacobians[5 * i + 2][5] = tmp[2] * dClr_dvttn[1][2] * w_k;

						jacobians[5 * i + 2][6] = tmp[0] * dClr_dvttn[2][2] * w_k;
						jacobians[5 * i + 2][7] = tmp[1] * dClr_dvttn[2][2] * w_k;
						jacobians[5 * i + 2][8] = tmp[2] * dClr_dvttn[2][2] * w_k;
					}

					if (jacobians[5 * i + 3] != NULL)
					{
						jacobians[5 * i + 3][0] = dClr_dvttn[0][0] * w_k;
						jacobians[5 * i + 3][1] = dClr_dvttn[0][1] * w_k;
						jacobians[5 * i + 3][2] = dClr_dvttn[0][2] * w_k;

						jacobians[5 * i + 3][3] = dClr_dvttn[1][0] * w_k;
						jacobians[5 * i + 3][4] = dClr_dvttn[1][1] * w_k;
						jacobians[5 * i + 3][5] = dClr_dvttn[1][2] * w_k;
						
						jacobians[5 * i + 3][6] = dClr_dvttn[2][0] * w_k;
						jacobians[5 * i + 3][7] = dClr_dvttn[2][1] * w_k;
						jacobians[5 * i + 3][8] = dClr_dvttn[2][2] * w_k;
					}
					if (jacobians[5 * i + 4] != NULL)
					{
						vnl_vector_fixed<double, 3> dI_dw = dClr_dvttn*tmp2;
						memcpy(jacobians[5 * i + 4], dI_dw.data_block(), sizeof(double)* 3);
					}
				}

				if (jacobians[5 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						dvtt_dR(0, i) = vp_tn[i];
						dvtt_dR(1, 3 + i) = vp_tn[i];
						dvtt_dR(2, 6 + i) = vp_tn[i];
					}
					vnl_matrix_fixed<double, 3, 3> dClr_drod = dClr_dvtt * (dvtt_dR * dR_drod);

					memcpy(jacobians[5 * ngn_num], dClr_drod.data_block(), 9 * sizeof(double));
				}

				if (jacobians[5 * ngn_num + 1] != NULL)
				{
					//dclr/dt == dclr/dvt_t
					memcpy(jacobians[5 * ngn_num + 1], dClr_dvtt.data_block(), 9 * sizeof(double));
				}
			}

			return true;
		}

	private:
		int vtIdx;
	};

}

namespace AffineSetNonrigidPWKeyPtsTerm
{
	//parameters: [r0, r1, r2, t, w; r0, r1, r2, t, w; ...] & [rod_global, t_global]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int pairIdx_)
		{
			assert(pairIdx_ < match_set->points_1.size());
			this->pairIdx = pairIdx_;

			int vtIdx = match_set->indices_1[pairIdx];
			assert(vtIdx<surface->vtNum);
			this->ngn = &((*ngns)[vtIdx]);

			this->set_num_residuals(3);
			int ngn_num = this->ngn->neighborIndices.size();
			for (int i = 0; i<ngn_num; i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(3); //t
				this->mutable_parameter_block_sizes()->push_back(1); //w
			}
			this->mutable_parameter_block_sizes()->push_back(3); // rod_global
			this->mutable_parameter_block_sizes()->push_back(3); // t_global
		}
		~F() {};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			int ngn_num = this->ngn->neighborIndices.size();

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
			vnl_matrix_fixed<double, 3, 3> R_global(0.0);
			//d(R)/d(rod)
			vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
			cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
			cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
			cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
			cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

			int vtIdx = match_set->indices_1[pairIdx];
			float const* v = surface->vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt(v[0], v[1], v[2]);
			float const* vt_t_ = surface_t.vt_data_block(vtIdx);
			vnl_vector_fixed<double, 3> vt_t(vt_t_[0], vt_t_[1], vt_t_[2]);

			vnl_vector_fixed<double, 3> const& q = match_set->points_2[pairIdx];

			residuals[0] = vt_t[0] - q[0];
			residuals[1] = vt_t[1] - q[1];
			residuals[2] = vt_t[2] - q[2];

			vnl_vector_fixed<double, 3> vp_tn(0.0); //nonrigidly transformed point
			for (int i = 0; i<ngn_num; i++)
			{
				int ndIdx = this->ngn->neighborIndices[i];
				double w_k = this->ngn->weights[i];
				assert(w_k == parameters[5 * i + 4][0]);
				vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
				vnl_vector_fixed<double, 3> const&t_k = graph->nodes[ndIdx].t;
				vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;

				vnl_vector_fixed<double, 3> tmp;
				tmp[0] = vt[0] - g_k[0];
				tmp[1] = vt[1] - g_k[1];
				tmp[2] = vt[2] - g_k[2];

				vnl_vector_fixed<double, 3> tmp2 = A_k*tmp + g_k + t_k;
				vp_tn += tmp2*w_k;

				if (jacobians != NULL)
				{
					double w = w_k;
					if (jacobians[5 * i] != NULL)
					{
						//df0/dr0
						jacobians[5 * i][0] = R_global(0, 0)*tmp[0] * w_k;
						jacobians[5 * i][1] = R_global(0, 0)*tmp[1] * w_k;
						jacobians[5 * i][2] = R_global(0, 0)*tmp[2] * w_k;

						//df1/dr0
						jacobians[5 * i][3] = R_global(1, 0)*tmp[0] * w_k;
						jacobians[5 * i][4] = R_global(1, 0)*tmp[1] * w_k;
						jacobians[5 * i][5] = R_global(1, 0)*tmp[2] * w_k;

						//df2/dr0
						jacobians[5 * i][6] = R_global(2, 0)*tmp[0] * w_k;
						jacobians[5 * i][7] = R_global(2, 0)*tmp[1] * w_k;
						jacobians[5 * i][8] = R_global(2, 0)*tmp[2] * w_k;
					}
					if (jacobians[5 * i + 1] != NULL)
					{
						//df0/dr0
						jacobians[5 * i + 1][0] = R_global(0, 1)*tmp[0] * w_k;
						jacobians[5 * i + 1][1] = R_global(0, 1)*tmp[1] * w_k;
						jacobians[5 * i + 1][2] = R_global(0, 1)*tmp[2] * w_k;

						//df1/dr0
						jacobians[5 * i + 1][3] = R_global(1, 1)*tmp[0] * w_k;
						jacobians[5 * i + 1][4] = R_global(1, 1)*tmp[1] * w_k;
						jacobians[5 * i + 1][5] = R_global(1, 1)*tmp[2] * w_k;

						//df2/dr0
						jacobians[5 * i + 1][6] = R_global(2, 1)*tmp[0] * w_k;
						jacobians[5 * i + 1][7] = R_global(2, 1)*tmp[1] * w_k;
						jacobians[5 * i + 1][8] = R_global(2, 1)*tmp[2] * w_k;
					}
					if (jacobians[5 * i + 2] != NULL)
					{
						//df0/dr0
						jacobians[5 * i + 2][0] = R_global(0, 2)*tmp[0] * w_k;
						jacobians[5 * i + 2][1] = R_global(0, 2)*tmp[1] * w_k;
						jacobians[5 * i + 2][2] = R_global(0, 2)*tmp[2] * w_k;

						//df1/dr0
						jacobians[5 * i + 2][3] = R_global(1, 2)*tmp[0] * w_k;
						jacobians[5 * i + 2][4] = R_global(1, 2)*tmp[1] * w_k;
						jacobians[5 * i + 2][5] = R_global(1, 2)*tmp[2] * w_k;

						//df2/dr0
						jacobians[5 * i + 2][6] = R_global(2, 2)*tmp[0] * w_k;
						jacobians[5 * i + 2][7] = R_global(2, 2)*tmp[1] * w_k;
						jacobians[5 * i + 2][8] = R_global(2, 2)*tmp[2] * w_k;
					}

					if (jacobians[5 * i + 3] != NULL)
					{
						jacobians[5 * i + 3][0] = R_global(0, 0)*w_k;
						jacobians[5 * i + 3][1] = R_global(0, 1)*w_k;
						jacobians[5 * i + 3][2] = R_global(0, 2)*w_k;

						jacobians[5 * i + 3][3] = R_global(1, 0)*w_k;
						jacobians[5 * i + 3][4] = R_global(1, 1)*w_k;
						jacobians[5 * i + 3][5] = R_global(1, 2)*w_k;
						
						jacobians[5 * i + 3][6] = R_global(2, 0)*w_k;
						jacobians[5 * i + 3][7] = R_global(2, 1)*w_k;
						jacobians[5 * i + 3][8] = R_global(2, 2)*w_k;
					}
					if (jacobians[5 * i + 4] != NULL)
					{
						//df/dw
						vnl_vector_fixed<double, 3> df_dw = R_global*tmp2;
						memcpy(jacobians[5 * i + 4], df_dw.data_block(), 3 * sizeof(double));
					}
				}
			}

			if (jacobians != NULL)
			{
				if (jacobians[5 * ngn_num] != NULL)
				{
					vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
					for (int i = 0; i < 3; i++)
					{
						dvtt_dR(0, i) = vp_tn[i];
						dvtt_dR(1, 3 + i) = vp_tn[i];
						dvtt_dR(2, 6 + i) = vp_tn[i];
					}
					vnl_matrix_fixed<double, 3, 3> df_drod = dvtt_dR * dR_drod;

					memcpy(jacobians[5 * ngn_num], df_drod.data_block(), 9 * sizeof(double));
				}

				if (jacobians[5 * ngn_num + 1] != NULL)
				{
					//df0/dt
					jacobians[5 * ngn_num + 1][0] = 1.0;
					jacobians[5 * ngn_num + 1][1] = 0.0;
					jacobians[5 * ngn_num + 1][2] = 0.0;

					jacobians[5 * ngn_num + 1][3] = 0.0;
					jacobians[5 * ngn_num + 1][4] = 1.0;
					jacobians[5 * ngn_num + 1][5] = 0.0;

					jacobians[5 * ngn_num + 1][6] = 0.0;
					jacobians[5 * ngn_num + 1][7] = 0.0;
					jacobians[5 * ngn_num + 1][8] = 1.0;
				}
			}
			return true;
		}

	private:
		NeighborGraphNodesOfPoint *ngn;
		int pairIdx;
	};
}

namespace AffineSetNonrigidPWIsometricTerm{
	//parameters: [r0, r1, r2, t; r0, r1, r2, t; ...] & [ws] & [ws] & [ws]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		//ngn_indices_: the neighboring graph nodes of all three vertices
		F(int vtIdx1_, int vtIdx2_, int vtIdx3_, vector<int> const& ngn_indices_)
		{
			assert(vtIdx1_ < surface->vtNum);
			assert(vtIdx2_ < surface->vtNum);
			assert(vtIdx3_ < surface->vtNum);
			this->vtIdx1 = vtIdx1_;
			this->vtIdx2 = vtIdx2_;
			this->vtIdx3 = vtIdx3_;
			this->ngn_indices = ngn_indices_;

			float const*p1 = surface->vt_data_block(vtIdx1);
			float const*p2 = surface->vt_data_block(vtIdx2);
			float const*p3 = surface->vt_data_block(vtIdx3);
			this->vt1 = vnl_vector_fixed<double, 3>(p1[0], p1[1], p1[2]);
			this->vt2 = vnl_vector_fixed<double, 3>(p2[0], p2[1], p2[2]);
			this->vt3 = vnl_vector_fixed<double, 3>(p3[0], p3[1], p3[2]);

			this->inner_prod_1 = dot_product(vt2 - vt1, vt3 - vt1);
			this->inner_prod_2 = dot_product(vt3 - vt2, vt1 - vt2);
			this->inner_prod_3 = dot_product(vt1 - vt3, vt2 - vt3);

			vector<int> const& ngn_list1 = (*ngns)[vtIdx1].neighborIndices;
			vector<double> const& ngn_weights1 = (*ngns)[vtIdx1].weights;
			vector<int> const& ngn_list2 = (*ngns)[vtIdx2].neighborIndices;
			vector<double> const& ngn_weights2 = (*ngns)[vtIdx2].weights;
			vector<int> const& ngn_list3 = (*ngns)[vtIdx3].neighborIndices;
			vector<double> const& ngn_weights3 = (*ngns)[vtIdx3].weights;

			this->ngnWeights_vt1.resize(ngn_indices.size());
			this->ngnWeights_vt2.resize(ngn_indices.size());
			this->ngnWeights_vt3.resize(ngn_indices.size());
			for (int i = 0; i < ngn_indices.size(); i++)
			{
				int ngnIdx = ngn_indices[i];
				assert(ngnIdx >= 0 && ngnIdx < graph->nodes.size());

				int index1 = search_val_list(ngn_list1, ngnIdx);
				if (index1 == -1)
					ngnWeights_vt1[i] = 0.0;
				else
					ngnWeights_vt1[i] = ngn_weights1[index1];

				int index2 = search_val_list(ngn_list2, ngnIdx);
				if (index2 == -1)
					ngnWeights_vt2[i] = 0.0;
				else
					ngnWeights_vt2[i] = ngn_weights2[index2];

				int index3 = search_val_list(ngn_list3, ngnIdx);
				if (index3 == -1)
					ngnWeights_vt3[i] = 0.0;
				else
					ngnWeights_vt3[i] = ngn_weights3[index3];
			}

			this->set_num_residuals(3);
			for (int i = 0; i < ngn_indices.size(); i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(3); //t
			}
			for (int i = 0; i < ngn_list1.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
			for (int i = 0; i < ngn_list2.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
			for (int i = 0; i < ngn_list3.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
		}
		~F(){};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			const float* q1 = surface_t.vt_data_block(this->vtIdx1);
			const float* q2 = surface_t.vt_data_block(this->vtIdx2);
			const float* q3 = surface_t.vt_data_block(this->vtIdx3);
			vnl_vector_fixed<double, 3> vq1(q1[0], q1[1], q1[2]);
			vnl_vector_fixed<double, 3> vq2(q2[0], q2[1], q2[2]);
			vnl_vector_fixed<double, 3> vq3(q3[0], q3[1], q3[2]);

			//remove the global rigid movement
			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_matrix_fixed<double, 3, 3> R_global = graph->global_rigid.rotation();
			vq1 = R_global.transpose() *(vq1 - T_global);
			vq2 = R_global.transpose() *(vq2 - T_global);
			vq3 = R_global.transpose() *(vq3 - T_global);

			residuals[0] = dot_product(vq2 - vq1, vq3 - vq1) - inner_prod_1;
			residuals[1] = dot_product(vq3 - vq2, vq1 - vq2) - inner_prod_2;
			residuals[2] = dot_product(vq1 - vq3, vq2 - vq3) - inner_prod_3;

			if (jacobians != NULL)
			{
				vnl_vector_fixed<double, 3> f1_vq1 = 2.0*vq1 - vq2 - vq3;
				vnl_vector_fixed<double, 3> f1_vq2 = vq3 - vq1;
				vnl_vector_fixed<double, 3> f1_vq3 = vq2 - vq1;

				vnl_vector_fixed<double, 3> f2_vq1 = vq3 - vq2;
				vnl_vector_fixed<double, 3> f2_vq2 = 2.0*vq2 - vq3 - vq1;
				vnl_vector_fixed<double, 3> f2_vq3 = vq1 - vq2;

				vnl_vector_fixed<double, 3> f3_vq1 = vq2 - vq3;
				vnl_vector_fixed<double, 3> f3_vq2 = vq1 - vq3;
				vnl_vector_fixed<double, 3> f3_vq3 = 2.0*vq3 - vq2 - vq1;

				for (int i = 0; i < ngn_indices.size(); i++)
				{
					int ndIdx = ngn_indices[i];
					vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;
					double w1 = this->ngnWeights_vt1[i];
					double w2 = this->ngnWeights_vt2[i];
					double w3 = this->ngnWeights_vt3[i];
					vnl_vector_fixed<double, 3> tmp1 = this->vt1 - g_k;
					vnl_vector_fixed<double, 3> tmp2 = this->vt2 - g_k;
					vnl_vector_fixed<double, 3> tmp3 = this->vt3 - g_k;

					if (jacobians[4 * i] != NULL)
					{
						jacobians[4 * i][0] = w1*f1_vq1[0] * tmp1[0] + w2*f1_vq2[0] * tmp2[0] + w3*f1_vq3[0] * tmp3[0];
						jacobians[4 * i][1] = w1*f1_vq1[0] * tmp1[1] + w2*f1_vq2[0] * tmp2[1] + w3*f1_vq3[0] * tmp3[1];
						jacobians[4 * i][2] = w1*f1_vq1[0] * tmp1[2] + w2*f1_vq2[0] * tmp2[2] + w3*f1_vq3[0] * tmp3[2];

						jacobians[4 * i][3] = w1*f2_vq1[0] * tmp1[0] + w2*f2_vq2[0] * tmp2[0] + w3*f2_vq3[0] * tmp3[0];
						jacobians[4 * i][4] = w1*f2_vq1[0] * tmp1[1] + w2*f2_vq2[0] * tmp2[1] + w3*f2_vq3[0] * tmp3[1];
						jacobians[4 * i][5] = w1*f2_vq1[0] * tmp1[2] + w2*f2_vq2[0] * tmp2[2] + w3*f2_vq3[0] * tmp3[2];

						jacobians[4 * i][6] = w1*f3_vq1[0] * tmp1[0] + w2*f3_vq2[0] * tmp2[0] + w3*f3_vq3[0] * tmp3[0];
						jacobians[4 * i][7] = w1*f3_vq1[0] * tmp1[1] + w2*f3_vq2[0] * tmp2[1] + w3*f3_vq3[0] * tmp3[1];
						jacobians[4 * i][8] = w1*f3_vq1[0] * tmp1[2] + w2*f3_vq2[0] * tmp2[2] + w3*f3_vq3[0] * tmp3[2];
					}

					if (jacobians[4 * i + 1] != NULL)
					{
						jacobians[4 * i + 1][0] = w1*f1_vq1[1] * tmp1[0] + w2*f1_vq2[1] * tmp2[0] + w3*f1_vq3[1] * tmp3[0];
						jacobians[4 * i + 1][1] = w1*f1_vq1[1] * tmp1[1] + w2*f1_vq2[1] * tmp2[1] + w3*f1_vq3[1] * tmp3[1];
						jacobians[4 * i + 1][2] = w1*f1_vq1[1] * tmp1[2] + w2*f1_vq2[1] * tmp2[2] + w3*f1_vq3[1] * tmp3[2];

						jacobians[4 * i + 1][3] = w1*f2_vq1[1] * tmp1[0] + w2*f2_vq2[1] * tmp2[0] + w3*f2_vq3[1] * tmp3[0];
						jacobians[4 * i + 1][4] = w1*f2_vq1[1] * tmp1[1] + w2*f2_vq2[1] * tmp2[1] + w3*f2_vq3[1] * tmp3[1];
						jacobians[4 * i + 1][5] = w1*f2_vq1[1] * tmp1[2] + w2*f2_vq2[1] * tmp2[2] + w3*f2_vq3[1] * tmp3[2];

						jacobians[4 * i + 1][6] = w1*f3_vq1[1] * tmp1[0] + w2*f3_vq2[1] * tmp2[0] + w3*f3_vq3[1] * tmp3[0];
						jacobians[4 * i + 1][7] = w1*f3_vq1[1] * tmp1[1] + w2*f3_vq2[1] * tmp2[1] + w3*f3_vq3[1] * tmp3[1];
						jacobians[4 * i + 1][8] = w1*f3_vq1[1] * tmp1[2] + w2*f3_vq2[1] * tmp2[2] + w3*f3_vq3[1] * tmp3[2];
					}

					if (jacobians[4 * i + 2] != NULL)
					{
						jacobians[4 * i + 2][0] = w1*f1_vq1[2] * tmp1[0] + w2*f1_vq2[2] * tmp2[0] + w3*f1_vq3[2] * tmp3[0];
						jacobians[4 * i + 2][1] = w1*f1_vq1[2] * tmp1[1] + w2*f1_vq2[2] * tmp2[1] + w3*f1_vq3[2] * tmp3[1];
						jacobians[4 * i + 2][2] = w1*f1_vq1[2] * tmp1[2] + w2*f1_vq2[2] * tmp2[2] + w3*f1_vq3[2] * tmp3[2];

						jacobians[4 * i + 2][3] = w1*f2_vq1[2] * tmp1[0] + w2*f2_vq2[2] * tmp2[0] + w3*f2_vq3[2] * tmp3[0];
						jacobians[4 * i + 2][4] = w1*f2_vq1[2] * tmp1[1] + w2*f2_vq2[2] * tmp2[1] + w3*f2_vq3[2] * tmp3[1];
						jacobians[4 * i + 2][5] = w1*f2_vq1[2] * tmp1[2] + w2*f2_vq2[2] * tmp2[2] + w3*f2_vq3[2] * tmp3[2];

						jacobians[4 * i + 2][6] = w1*f3_vq1[2] * tmp1[0] + w2*f3_vq2[2] * tmp2[0] + w3*f3_vq3[2] * tmp3[0];
						jacobians[4 * i + 2][7] = w1*f3_vq1[2] * tmp1[1] + w2*f3_vq2[2] * tmp2[1] + w3*f3_vq3[2] * tmp3[1];
						jacobians[4 * i + 2][8] = w1*f3_vq1[2] * tmp1[2] + w2*f3_vq2[2] * tmp2[2] + w3*f3_vq3[2] * tmp3[2];
					}

					if (jacobians[4 * i + 3] != NULL)
					{
						jacobians[4 * i + 3][0] = w1*f1_vq1[0] + w2*f1_vq2[0] + w3*f1_vq3[0];
						jacobians[4 * i + 3][1] = w1*f1_vq1[1] + w2*f1_vq2[1] + w3*f1_vq3[1];
						jacobians[4 * i + 3][2] = w1*f1_vq1[2] + w2*f1_vq2[2] + w3*f1_vq3[2];

						jacobians[4 * i + 3][3] = w1*f2_vq1[0] + w2*f2_vq2[0] + w3*f2_vq3[0];
						jacobians[4 * i + 3][4] = w1*f2_vq1[1] + w2*f2_vq2[1] + w3*f2_vq3[1];
						jacobians[4 * i + 3][5] = w1*f2_vq1[2] + w2*f2_vq2[2] + w3*f2_vq3[2];

						jacobians[4 * i + 3][6] = w1*f3_vq1[0] + w2*f3_vq2[0] + w3*f3_vq3[0];
						jacobians[4 * i + 3][7] = w1*f3_vq1[1] + w2*f3_vq2[1] + w3*f3_vq3[1];
						jacobians[4 * i + 3][8] = w1*f3_vq1[2] + w2*f3_vq2[2] + w3*f3_vq3[2];
					}
				}

				NeighborGraphNodesOfPoint const& ngn_1 = (*ngns)[vtIdx1];
				NeighborGraphNodesOfPoint const& ngn_2 = (*ngns)[vtIdx2];
				NeighborGraphNodesOfPoint const& ngn_3 = (*ngns)[vtIdx3];
				int ngns_num_uion = this->ngn_indices.size();
				int ngns_num_1 = ngn_1.neighborIndices.size();
				int ngns_num_2 = ngn_2.neighborIndices.size();
				int ngns_num_3 = ngn_3.neighborIndices.size();

				for (int k = 0; k < ngns_num_1; k++)
				{
					if (jacobians[4 * ngns_num_uion + k] != NULL)
					{
						int ndIdx = ngn_1.neighborIndices[k];
						double w_1k = ngn_1.weights[k];
						assert(w_1k == parameters[4 * ngns_num_uion + k][0]);
						DeformGraphNode const& nd = graph->nodes[ndIdx];
						vnl_vector_fixed<double, 3> const& g_1k = nd.g;
						vnl_vector_fixed<double, 3> const& t_1k = nd.t;
						vnl_matrix_fixed<double, 3, 3> const& A_1k = nd.A;
						
						vnl_vector_fixed<double, 3> tmp = A_1k*(vt1 - g_1k) + g_1k + t_1k;
						//d(f1)/d(w1k)
						jacobians[4 * ngns_num_uion + k][0] = dot_product(f1_vq1, tmp);
						//d(f2)/d(w1k)
						jacobians[4 * ngns_num_uion + k][1] = dot_product(f2_vq1, tmp);
						//d(f3)/d(w1k)
						jacobians[4 * ngns_num_uion + k][2] = dot_product(f3_vq1, tmp);
					}
				}
				for (int k = 0; k < ngns_num_2; k++)
				{
					if (jacobians[4 * ngns_num_uion + ngns_num_1 + k] != NULL)
					{
						int ndIdx = ngn_2.neighborIndices[k];
						double w_2k = ngn_2.weights[k];
						assert(w_2k == parameters[4 * ngns_num_uion + ngns_num_1 + k][0]);
						DeformGraphNode const& nd = graph->nodes[ndIdx];
						vnl_vector_fixed<double, 3> const& g_2k = nd.g;
						vnl_vector_fixed<double, 3> const& t_2k = nd.t;
						vnl_matrix_fixed<double, 3, 3> const& A_2k = nd.A;

						vnl_vector_fixed<double, 3> tmp = A_2k*(vt2 - g_2k) + g_2k + t_2k;
						//d(f1)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + k][0] = dot_product(f1_vq2, tmp);
						//d(f2)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + k][1] = dot_product(f2_vq2, tmp);
						//d(f3)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + k][2] = dot_product(f3_vq2, tmp);
					}
				}
				for (int k = 0; k < ngns_num_3; k++)
				{
					if (jacobians[4 * ngns_num_uion + ngns_num_1 + ngns_num_2 + k] != NULL)
					{
						int ndIdx = ngn_3.neighborIndices[k];
						double w_3k = ngn_3.weights[k];
						assert(w_3k == parameters[4 * ngns_num_uion + ngns_num_1 + ngns_num_2 + k][0]);
						DeformGraphNode const& nd = graph->nodes[ndIdx];
						vnl_vector_fixed<double, 3> const& g_3k = nd.g;
						vnl_vector_fixed<double, 3> const& t_3k = nd.t;
						vnl_matrix_fixed<double, 3, 3> const& A_3k = nd.A;

						vnl_vector_fixed<double, 3> tmp = A_3k*(vt3 - g_3k) + g_3k + t_3k;
						//d(f1)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + ngns_num_2 + k][0] = dot_product(f1_vq3, tmp);
						//d(f2)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + ngns_num_2 + k][1] = dot_product(f2_vq3, tmp);
						//d(f3)/d(w1k)
						jacobians[4 * ngns_num_uion + ngns_num_1 + ngns_num_2 + k][2] = dot_product(f3_vq3, tmp);
					}
				}
			}

			return true;
		}

	private:
		int vtIdx1;
		int vtIdx2;
		int vtIdx3;

		vnl_vector_fixed<double, 3> vt1;
		vnl_vector_fixed<double, 3> vt2;
		vnl_vector_fixed<double, 3> vt3;

		vector<int> ngn_indices;
		vector<float> ngnWeights_vt1;
		vector<float> ngnWeights_vt2;
		vector<float> ngnWeights_vt3;

		//inner product of two edges of a triangular on the tangent plane
		double inner_prod_1;// two edges connected by vt1
		double inner_prod_2;// two edges connected by vt2
		double inner_prod_3;// two edges connected by vt3		

	};
}

namespace AffineSetNonrigidPWEDRegularizationTerm
{
	//input parameters: [r0, r1, r2] t_i, t_j
	class F : public SizedCostFunction<3, 3, 3, 3, 3, 3>, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int ndIdx_i, int ndIdx_j)
		{
			assert(ndIdx_i < graph->nodes.size());
			assert(ndIdx_j < graph->nodes.size());
			this->ndIdx_i = ndIdx_i;
			this->ndIdx_j = ndIdx_j;
		}
		~F() {};
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			vnl_vector_fixed<double, 3> r0_i(parameters[0]);
			vnl_vector_fixed<double, 3> r1_i(parameters[1]);
			vnl_vector_fixed<double, 3> r2_i(parameters[2]);
			vnl_vector_fixed<double, 3> t_i(parameters[3]);
			vnl_vector_fixed<double, 3> t_j(parameters[4]);

			vnl_vector_fixed<double, 3> const& g_i = graph->nodes[ndIdx_i].g;
			vnl_vector_fixed<double, 3> const& g_j = graph->nodes[ndIdx_j].g;
			vnl_vector_fixed<double, 3> tmp = g_j - g_i;

			residuals[0] = dot_product(r0_i, tmp) + g_i[0] + t_i[0] - g_j[0] - t_j[0];
			residuals[1] = dot_product(r1_i, tmp) + g_i[1] + t_i[1] - g_j[1] - t_j[1];
			residuals[2] = dot_product(r2_i, tmp) + g_i[2] + t_i[2] - g_j[2] - t_j[2];

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = tmp[0];
					jacobians[0][1] = tmp[1];
					jacobians[0][2] = tmp[2];
					jacobians[0][3] = 0.0;
					jacobians[0][4] = 0.0;
					jacobians[0][5] = 0.0;
					jacobians[0][6] = 0.0;
					jacobians[0][7] = 0.0;
					jacobians[0][8] = 0.0;
				}
				if (jacobians[1] != NULL)
				{
					jacobians[1][0] = 0.0;
					jacobians[1][1] = 0.0;
					jacobians[1][2] = 0.0;
					jacobians[1][3] = tmp[0];
					jacobians[1][4] = tmp[1];
					jacobians[1][5] = tmp[2];
					jacobians[1][6] = 0.0;
					jacobians[1][7] = 0.0;
					jacobians[1][8] = 0.0;
				}
				if (jacobians[2] != NULL)
				{
					jacobians[2][0] = 0.0;
					jacobians[2][1] = 0.0;
					jacobians[2][2] = 0.0;
					jacobians[2][3] = 0.0;
					jacobians[2][4] = 0.0;
					jacobians[2][5] = 0.0;
					jacobians[2][6] = tmp[0];
					jacobians[2][7] = tmp[1];
					jacobians[2][8] = tmp[2];
				}
				if (jacobians[3] != NULL)
				{
					jacobians[3][0] = 1.0;
					jacobians[3][1] = 0.0;
					jacobians[3][2] = 0.0;
					jacobians[3][3] = 0.0;
					jacobians[3][4] = 1.0;
					jacobians[3][5] = 0.0;
					jacobians[3][6] = 0.0;
					jacobians[3][7] = 0.0;
					jacobians[3][8] = 1.0;
				}
				if (jacobians[4] != NULL)
				{
					jacobians[4][0] = -1.0;
					jacobians[4][1] = 0.0;
					jacobians[4][2] = 0.0;
					jacobians[4][3] = 0.0;
					jacobians[4][4] = -1.0;
					jacobians[4][5] = 0.0;
					jacobians[4][6] = 0.0;
					jacobians[4][7] = 0.0;
					jacobians[4][8] = -1.0;
				}
			}
			return true;
		}

	private:
		int ndIdx_i;
		int ndIdx_j;
	};
};

namespace AffineSetNonrigidPWWeightSmoothness
{
	//paras [ngn_i.neighbor_node_weights, ngn_j.neighbor_nodes_weights]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int idx_i_, int idx_j_)
		{
			assert(idx_i_ < surface->vtNum);
			this->idx_i = idx_i_;
			assert(idx_j_ < surface->vtNum);
			this->idx_j = idx_j_;

			NeighborGraphNodesOfPoint const& ngn_i = (*ngns)[idx_i];
			NeighborGraphNodesOfPoint const& ngn_j = (*ngns)[idx_j];
			vector<int> indices_cmb = ngn_i.neighborIndices;
			this->ngnIdx_cmb_at_i.resize(ngn_i.neighborIndices.size(), -1);
			for (int i = 0; i < this->ngnIdx_cmb_at_i.size(); i++)
				this->ngnIdx_cmb_at_i[i] = i;

			this->ngnIdx_cmb_at_j.resize(ngn_i.neighborIndices.size(), -1);
			for (int j = 0; j < ngn_j.neighborIndices.size(); j++)
			{
				int idx_j_at_cmb = search_val_list(indices_cmb, ngn_j.neighborIndices[j]); //-1: not found
				if (idx_j_at_cmb != -1)
				{
					this->ngnIdx_cmb_at_j[idx_j_at_cmb] = j;
				}
				else
				{
					indices_cmb.push_back(ngn_j.neighborIndices[j]);
					this->ngnIdx_cmb_at_i.push_back(-1);
					this->ngnIdx_cmb_at_j.push_back(j);
				}
			}

			this->set_num_residuals(this->ngnIdx_cmb_at_i.size());
			for (int i = 0; i < ngn_i.neighborIndices.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1);
			for (int i = 0; i < ngn_j.neighborIndices.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1);
		}
		~F(){}
	
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			return true;
		}

	private:
		int idx_i;
		int idx_j;
		std::vector<int> ngnIdx_cmb_at_i;
		std::vector<int> ngnIdx_cmb_at_j;
	};

	//paras: [w1, w2]
	//min || w1 - w2 ||^2
	class F1 : public SizedCostFunction<1, 1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w1 = parameters[0][0];
			double w2 = parameters[1][0];
			residuals[0] = w1 - w2;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = 1.0;
				if (jacobians[1] != NULL)
					jacobians[1][0] = -1.0;
			}
			return true;
		}
	};

	// paras: [w1]
	// min || w1 ||^2
	class F2 : public SizedCostFunction<1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];
			residuals[0] = w;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = 1.0;			
			}
			return true;
		}
	};
}

namespace AffineSetNonrigidPWZollhofer
{
	class F : public SizedCostFunction<1, 1>
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double alpha = parameters[0][0];
			residuals[0] = 1.0 - alpha*alpha;
			if (jacobians != NULL && jacobians[0] != NULL)
			{
				jacobians[0][0] = -2.0*alpha;
			}
			return true;
		}
	};
}

namespace AffineSetNonrigidPWWeightSmoothnessZollhofer
{
	//paras: [w1, w2]
	//min || w1 - w2 ||^2
	class F1 : public SizedCostFunction<1, 1, 1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		F1(int edgeIdx_)
		{
			assert(edgeIdx_ < edge_alphas->size());
			this->edgeIdx = edgeIdx_;
		}
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w1 = parameters[0][0];
			double w2 = parameters[1][0];
			double alpha = parameters[2][0];
			assert(alpha == (*edge_alphas)[edgeIdx]);
			residuals[0] = (w1 - w2)*alpha;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = alpha;
				if (jacobians[1] != NULL)
					jacobians[1][0] = -alpha;
				if (jacobians[2] != NULL)
					jacobians[2][0] = (w1 - w2);
			}
			return true;
		}
	private:
		int edgeIdx;
	};

	// paras: [w1]
	// min || w1 ||^2
	class F2 : public SizedCostFunction<1, 1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		F2(int edgeIdx_)
		{
			assert(edgeIdx_ < edge_alphas->size());
			this->edgeIdx = edgeIdx_;
		}
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];
			double alpha = parameters[1][0];
			assert(alpha == (*edge_alphas)[edgeIdx]);
			residuals[0] = w*alpha;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = alpha;
				if (jacobians[1] != NULL)
					jacobians[1][0] = w;
			}
			return true;
		}
	private:
		int edgeIdx;
	};
}

namespace AffineSetNonrigidPWWeightIsometricZollhofer
{
	//paras: [w1, w2]
	//min || w1 - w2 ||^2
	class F1 : public SizedCostFunction<1, 1, 1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		F1(int edgeIdx_, double dw_)
		{
			assert(edgeIdx_ < edge_alphas->size());
			this->edgeIdx = edgeIdx_;
			this->dw = dw_;
		}
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w1 = parameters[0][0];
			double w2 = parameters[1][0];
			double alpha = parameters[2][0];
			assert(alpha == (*edge_alphas)[edgeIdx]);

			residuals[0] = (w1 - w2 - dw)*alpha;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = alpha;
				if (jacobians[1] != NULL)
					jacobians[1][0] = -alpha;
				if (jacobians[2] != NULL)
					jacobians[2][0] = w1 - w2 - dw;
			}
			return true;
		}
	private:
		int edgeIdx;
		double dw;
	};

	// paras: [w1]
	// min || w1 ||^2
	class F2 : public SizedCostFunction<1, 1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		F2(int edgeIdx_, double dw_)
		{
			assert(edgeIdx_ < edge_alphas->size());
			this->edgeIdx = edgeIdx_;
			this->dw = dw_;
		}
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];
			double alpha = parameters[1][0];
			assert(alpha == (*edge_alphas)[edgeIdx]);
			residuals[0] = (w - dw)*alpha;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
					jacobians[0][0] = alpha;
				if (jacobians[1] != NULL)
					jacobians[1][0] = w - dw;
			}
			return true;
		}
	private:
		int edgeIdx;
		double dw;
	};
}

namespace AffineSetNonrigidPWDstIsometricZollhofer{
	//preserve length. parameters: [alpha, [shared ED node paras], [ws for vt1] [ws for vt2]]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		//ngn_indices_: the neighboring graph nodes of both vertices
		F(int vtIdx1_, int vtIdx2_, vector<int> const& ngn_indices_, double len_sqr_)
		{
			assert(vtIdx1_ < surface->vtNum);
			assert(vtIdx2_ < surface->vtNum);
			this->vtIdx1 = vtIdx1_;
			this->vtIdx2 = vtIdx2_;
			this->ngn_indices = ngn_indices_;

			vnl_vector_fixed<float, 3> vt1 = vnl_vector_fixed<float, 3>(surface->vt_data_block(vtIdx1));
			vnl_vector_fixed<float, 3> vt2 = vnl_vector_fixed<float, 3>(surface->vt_data_block(vtIdx2));

			if (len_sqr_>0.0)
				this->len_sqr = len_sqr_;
			else
				this->len_sqr = dot_product(vt1 - vt2, vt1 - vt2);

			vector<int> const& ngn_list1 = (*ngns)[vtIdx1].neighborIndices;
			vector<double> const& ngn_weights1 = (*ngns)[vtIdx1].weights;
			vector<int> const& ngn_list2 = (*ngns)[vtIdx2].neighborIndices;
			vector<double> const& ngn_weights2 = (*ngns)[vtIdx2].weights;

			this->ngnWeights_vt1.resize(ngn_indices.size());
			this->ngnWeights_vt2.resize(ngn_indices.size());
			for (int i = 0; i < ngn_indices.size(); i++)
			{
				int ngnIdx = ngn_indices[i];
				assert(ngnIdx >= 0 && ngnIdx < graph->nodes.size());

				int index1 = search_val_list(ngn_list1, ngnIdx);
				if (index1 == -1)
					ngnWeights_vt1[i] = 0.0;
				else
					ngnWeights_vt1[i] = ngn_weights1[index1];

				int index2 = search_val_list(ngn_list2, ngnIdx);
				if (index2 == -1)
					ngnWeights_vt2[i] = 0.0;
				else
					ngnWeights_vt2[i] = ngn_weights2[index2];
			}

			this->set_num_residuals(1);
			this->mutable_parameter_block_sizes()->push_back(1); //alpha
			for (int i = 0; i < ngn_indices.size(); i++)
			{
				this->mutable_parameter_block_sizes()->push_back(3); //r0
				this->mutable_parameter_block_sizes()->push_back(3); //r1
				this->mutable_parameter_block_sizes()->push_back(3); //r2
				this->mutable_parameter_block_sizes()->push_back(3); //t
			}
			for (int i = 0; i < ngn_list1.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
			for (int i = 0; i < ngn_list2.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
		}
		~F(){};

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			int A_offset = 1;
			int Ws_offset = A_offset + 4 * this->ngn_indices.size();

			float const* v1 = surface->vt_data_block(vtIdx1);
			float const* v2 = surface->vt_data_block(vtIdx2);
			vnl_vector_fixed<double, 3> vt1 = vnl_vector_fixed<double, 3>(v1[0], v1[1], v1[2]);
			vnl_vector_fixed<double, 3> vt2 = vnl_vector_fixed<double, 3>(v2[0], v2[1], v2[2]);

			float const* q1 = surface_t.vt_data_block(vtIdx1);
			float const* q2 = surface_t.vt_data_block(vtIdx2);
			vnl_vector_fixed<double, 3> vt1_t = vnl_vector_fixed<double, 3>(q1[0], q1[1], q1[2]);
			vnl_vector_fixed<double, 3> vt2_t = vnl_vector_fixed<double, 3>(q2[0], q2[1], q2[2]);

			vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
			vnl_matrix_fixed<double, 3, 3> R_global = graph->global_rigid.rotation();
			vt1_t = R_global.transpose() *(vt1_t - T_global);
			vt2_t = R_global.transpose() *(vt2_t - T_global);

			double alpha = parameters[0][0];

			double ori_res = dot_product(vt1_t - vt2_t, vt1_t - vt2_t) - len_sqr;
			residuals[0] = alpha*ori_res;

			if (jacobians != NULL)
			{
				vnl_vector_fixed<double, 3> f_vq1 = (vt1_t - vt2_t)*2.0*alpha;
				vnl_vector_fixed<double, 3> f_vq2 = -f_vq1;

				NeighborGraphNodesOfPoint const& ngn_1 = (*ngns)[vtIdx1];
				NeighborGraphNodesOfPoint const& ngn_2 = (*ngns)[vtIdx2];
				int ngns_num_1 = ngn_1.neighborIndices.size();
				int ngns_num_2 = ngn_2.neighborIndices.size();
				
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = ori_res;
				}

				for (int i = 0; i < ngn_indices.size(); i++)
				{
					int ndIdx = ngn_indices[i];
					vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;
					double w1 = this->ngnWeights_vt1[i];
					double w2 = this->ngnWeights_vt2[i];
					vnl_vector_fixed<double, 3> tmp1 = vt1 - g_k;
					vnl_vector_fixed<double, 3> tmp2 = vt2 - g_k;

					if (jacobians[A_offset + 4 * i] != NULL)
					{
						jacobians[A_offset + 4 * i][0] = w1*f_vq1[0] * tmp1[0] + w2*f_vq2[0] * tmp2[0];
						jacobians[A_offset + 4 * i][1] = w1*f_vq1[0] * tmp1[1] + w2*f_vq2[0] * tmp2[1];
						jacobians[A_offset + 4 * i][2] = w1*f_vq1[0] * tmp1[2] + w2*f_vq2[0] * tmp2[2];
					}

					if (jacobians[A_offset + 4 * i + 1] != NULL)
					{
						jacobians[A_offset + 4 * i + 1][0] = w1*f_vq1[1] * tmp1[0] + w2*f_vq2[1] * tmp2[0];
						jacobians[A_offset + 4 * i + 1][1] = w1*f_vq1[1] * tmp1[1] + w2*f_vq2[1] * tmp2[1];
						jacobians[A_offset + 4 * i + 1][2] = w1*f_vq1[1] * tmp1[2] + w2*f_vq2[1] * tmp2[2];
					}

					if (jacobians[A_offset + 4 * i + 2] != NULL)
					{
						jacobians[A_offset + 4 * i + 2][0] = w1*f_vq1[2] * tmp1[0] + w2*f_vq2[2] * tmp2[0];
						jacobians[A_offset + 4 * i + 2][1] = w1*f_vq1[2] * tmp1[1] + w2*f_vq2[2] * tmp2[1];
						jacobians[A_offset + 4 * i + 2][2] = w1*f_vq1[2] * tmp1[2] + w2*f_vq2[2] * tmp2[2];
					}

					if (jacobians[A_offset + 4 * i + 3] != NULL)
					{
						jacobians[A_offset + 4 * i + 3][0] = w1*f_vq1[0] + w2*f_vq2[0];
						jacobians[A_offset + 4 * i + 3][1] = w1*f_vq1[1] + w2*f_vq2[1];
						jacobians[A_offset + 4 * i + 3][2] = w1*f_vq1[2] + w2*f_vq2[2];
					}
				}

				for (int k = 0; k < ngns_num_1; k++)
				{
					if (jacobians[Ws_offset + k] != NULL)
					{
						int ndIdx = ngn_1.neighborIndices[k];
						double w_1k = ngn_1.weights[k];
						assert(w_1k == parameters[Ws_offset + k][0]);
						DeformGraphNode const& nd = graph->nodes[ndIdx];
						vnl_vector_fixed<double, 3> const& g_1k = nd.g;
						vnl_vector_fixed<double, 3> const& t_1k = nd.t;
						vnl_matrix_fixed<double, 3, 3> const& A_1k = nd.A;

						vnl_vector_fixed<double, 3> tmp = A_1k*(vt1 - g_1k) + g_1k + t_1k;
						//d(f)/d(w1k)
						jacobians[Ws_offset + k][0] = dot_product(f_vq1, tmp);
					}
				}
				for (int k = 0; k < ngns_num_2; k++)
				{
					if (jacobians[Ws_offset + ngns_num_1 + k] != NULL)
					{
						int ndIdx = ngn_2.neighborIndices[k];
						double w_2k = ngn_2.weights[k];
						assert(w_2k == parameters[Ws_offset + ngns_num_1 + k][0]);
						DeformGraphNode const& nd = graph->nodes[ndIdx];
						vnl_vector_fixed<double, 3> const& g_2k = nd.g;
						vnl_vector_fixed<double, 3> const& t_2k = nd.t;
						vnl_matrix_fixed<double, 3, 3> const& A_2k = nd.A;

						vnl_vector_fixed<double, 3> tmp = A_2k*(vt2 - g_2k) + g_2k + t_2k;
						//d(f)/d(w1k)
						jacobians[Ws_offset + ngns_num_1 + k][0] = dot_product(f_vq2, tmp);
					}
				}
			}

			return true;
		}

	private:
		int vtIdx1;
		int vtIdx2;

		vector<int> ngn_indices;
		vector<float> ngnWeights_vt1;
		vector<float> ngnWeights_vt2;

		double len_sqr;
	};
}

namespace AffineSetNonrigidPWWeightSumOne
{
	//parameters: [ngn.neighbor_nodes_weights]
	class F : public CostFunction, public AffineSetNonrigidPWDataStatic
	{
	public:
		F(int vtIdx_)
		{
			assert(vtIdx_ < surface->vtNum);
			this->vtIdx = vtIdx_;

			NeighborGraphNodesOfPoint const& ngn = (*ngns)[vtIdx];
			this->set_num_residuals(1);
			for (int i = 0; i < ngn.neighborIndices.size(); i++)
				this->mutable_parameter_block_sizes()->push_back(1); //w
		}

	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			NeighborGraphNodesOfPoint const& ngn = (*ngns)[vtIdx];
			residuals[0] = -1.0;
			for (int i = 0; i < ngn.neighborIndices.size(); i++)
			{
				assert(ngn.weights[i] == parameters[i][0]);
				residuals[0] += parameters[i][0];
			}
			if (jacobians != NULL)
			{
				for (int i = 0; i < ngn.neighborIndices.size(); i++)
				{
					if (jacobians[i] != NULL)
					{
						jacobians[i][0] = 1.0;
					}
				}
			}
			return true;
		}
	private:
		int vtIdx;
	};

	//penalize negative w
	class F2 : public SizedCostFunction<1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];

			const double lamda = 200;
			double t = std::exp(-lamda*w);
			if (w < -1.0)
				residuals[0] = -1.0;
			else if (w>1.0)
				residuals[0] = 0.0;
			else
				residuals[0] = 1.0 / (1 + t) - 1.0;

			if (jacobians != NULL && jacobians[0] != NULL)
			{
				if (w < -1.0||w>1.0)
					jacobians[0][0] = 0.0;
				else
					jacobians[0][0] = lamda*t/((1+t)*(1+t));
			}
			return true;
		}
	};

	//penalize negative w
	class F3 : public SizedCostFunction<1, 1>
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];

			residuals[0] = w >= 0.0 ? 0.0 : w;

			if (jacobians != NULL && jacobians[0] != NULL)
			{
				jacobians[0][0] = w >= 0.0 ? 0.0 : 1.0;
			}
			return true;
		}
	};
}

namespace AffineSetNonrigidPWWeightPolarity
{
	//parmeter [w]
	class F : public SizedCostFunction<1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];
			residuals[0] = 0.25 - (w - 0.5)*(w - 0.5);
			if (jacobians != NULL && jacobians[0] != NULL)
			{
				jacobians[0][0] = -2.0*(w - 0.5);
			}
			return true;
		}
	};

	//parmeter [w]
	class F1 : public SizedCostFunction<1, 1>, public AffineSetNonrigidPWDataStatic
	{
	public:
		virtual bool Evaluate(double const* const* parameters,
			double* residuals,
			double** jacobians) const
		{
			double w = parameters[0][0];
			residuals[0] = w > 0.5 ? 1.0 - w : w;
			if (jacobians != NULL && jacobians[0] != NULL)
			{
				jacobians[0][0] = w > 0.5? -1.0 : 1.0;
			}
			return true;
		}
	};
}

namespace AffineSetNonrigidPWRotationConstrTerm
{
	//orthogonal constraint on rows: r1.*r2=0; r1.*r3=0; r2.*r3=0
	class OrthogonalCostFunction : public SizedCostFunction<1, 3, 3>, public AffineSetNonrigidPWDataStatic
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
			residuals[0] = (r1[0] * r2[0] + r1[1] * r2[1] + r1[2] * r2[2]);// * w_rot;

			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = r2[0];// * w_rot;
					jacobians[0][1] = r2[1];// * w_rot;
					jacobians[0][2] = r2[2];// * w_rot;
				}

				if (jacobians[1] != NULL)
				{
					jacobians[1][0] = r1[0];// * w_rot;
					jacobians[1][1] = r1[1];// * w_rot;
					jacobians[1][2] = r1[2];// * w_rot;
				}
			}
			return true;
		}
	};

	//unitary constraint: r.*r - 1.0 = 0
	class UnitaryCostFunction : public SizedCostFunction<1, 3>, public AffineSetNonrigidPWDataStatic
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
			residuals[0] = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - 1.0);// * w_rot;
			if (jacobians != NULL && jacobians[0] != NULL)
			{
				jacobians[0][0] = 2.0 * r[0];// * w_rot;
				jacobians[0][1] = 2.0 * r[1];// * w_rot;
				jacobians[0][2] = 2.0 * r[2];// * w_rot;
			}
			return true;
		}
	};

	class DetCostFunction : public SizedCostFunction<1, 3, 3, 3>, public AffineSetNonrigidPWDataStatic
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
			residuals[0] = (r0[0] * r1[1] * r2[2] + r0[1] * r1[2] * r2[0] + r1[0] * r2[1] * r0[2] - r0[2] * r1[1] * r2[0] - r0[1] * r1[0] * r2[2] - r0[0] * r1[2] * r2[1] - 1.0);// * w_rot_adj;
			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					jacobians[0][0] = (r1[1] * r2[2] - r1[2] * r2[1]);//*w_rot_adj;
					jacobians[0][1] = (r1[2] * r2[0] - r1[0] * r2[2]);//*w_rot_adj;
					jacobians[0][2] = (r1[0] * r2[1] - r1[1] * r2[0]);//*w_rot_adj;
				}
				if (jacobians[1] != NULL)
				{
					jacobians[1][0] = (r2[1] * r0[2] - r0[1] * r2[2]);//*w_rot_adj;
					jacobians[1][1] = (r0[0] * r2[2] - r0[2] * r2[0]);//*w_rot_adj;
					jacobians[1][2] = (r0[1] * r2[0] - r0[0] * r2[1]);//*w_rot_adj;
				}
				if (jacobians[2] != NULL)
				{
					jacobians[2][0] = (r0[1] * r1[2] - r0[2] * r1[1]);//*w_rot_adj;
					jacobians[2][1] = (r1[0] * r0[2] - r0[0] * r1[2]);//*w_rot_adj;
					jacobians[2][2] = (r0[0] * r1[1] - r0[1] * r1[0]);//*w_rot_adj;
				}
			}
			return true;
		}
	};
};

namespace AffineSetNonrigidPWRigidity
{
	class IdentityCostFunctionR0 : public SizedCostFunction<3, 3>, public AffineSetNonrigidPWDataStatic
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

			// double w_rot_adj = 10.0*w_rot;
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

	class IdentityCostFunctionR1 : public SizedCostFunction<3, 3>, public AffineSetNonrigidPWDataStatic
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

			// double w_rot_adj = 10.0*w_rot;
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

	class IdentityCostFunctionR2 : public SizedCostFunction<3, 3>, public AffineSetNonrigidPWDataStatic
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

			// double w_rot_adj = 10.0*w_rot;
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

	class ZeroTranslation : public SizedCostFunction<3, 3>, public AffineSetNonrigidPWDataStatic
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
			residuals[1] = t[1];
			residuals[2] = t[2];


			if (jacobians != NULL)
			{
				if (jacobians[0] != NULL)
				{
					//dres0/ dt
					jacobians[0][0] = 1.0;
					jacobians[0][1] = 0.0;
					jacobians[0][2] = 0.0;

					//dres0/ dt
					jacobians[0][3] = 0.0;
					jacobians[0][4] = 1.0;
					jacobians[0][5] = 0.0;

					//dres0/ dt
					jacobians[0][6] = 0.0;
					jacobians[0][7] = 0.0;
					jacobians[0][8] = 1.0;
				}
			}
			return true;
		}
	};
}

}
#endif