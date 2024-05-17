//===============================================
//			DeformGraphOptDensePtsAndClr.h
//			Mingsong Dou (doums@cs.unc.edu)
//===============================================

#ifndef __DEFORMGRAPHOPTDENSEPTSANDCLR_H__
#define __DEFORMGRAPHOPTDENSEPTSANDCLR_H__
#include "DeformationGraphOptPlus.h"
#include "color_map.h"

namespace NonrigidMatching {

	void nonrigid_alignment_multiObj_densePtsAndClr(vector<RigidTransformModel*>& rigid_transfs, //in and out
		vector<CSurface<float>*> const& rigid_surfaces_ref,
		vector<DeformGraph*> const& rigid_graphs, //for intersection detection
		vector< GraphNodesTerritory* > const& rigid_graph_nodes_territories, //for intersection detection
		vector<DeformGraph*>& graphs_out, //in and out
		vector<CSurface<float>*> const& surfaces_ref,
		vector< NeighborGraphNodesOfPointList* > const& ngns_ref,
		vector< GraphNodesTerritory* > const& graph_nodes_territories,
		vector<S3DPointMatchSetIndexed*> const& match_set_3d,
		SignedDistanceFunc const* sdf_target,
		vector< cv::Mat > const& imgs_dst, //full image without distortion
		vector< cv::Mat > const& masks_dst,
		vector< GCameraView* > const& cams_dst,
		int round = 3,
		int frmIdx = 0,
		NonrigidMatchingPara para = NonrigidMatchingPara(),
		char const* debug_dir = NULL);

	void nonrigid_alignment_multiObj_densePtsAndClr(vector<RigidTransformModel*>& rigid_transfs, //in and out
		vector<CSurface<float>*> const& rigid_surfaces_ref,
		vector<DeformGraph*>& graphs_out, //in and out
		vector<CSurface<float>*> const& surfaces_ref,
		vector< NeighborGraphNodesOfPointList* > const& ngns_ref,
		vector<S3DPointMatchSetIndexed*> const& match_set_3d,
		SignedDistanceFunc const* sdf_target,
		vector< cv::Mat > const& imgs_dst, //full image without distortion
		vector< cv::Mat > const& masks_dst,
		vector< GCameraView* > const& cams_dst,
		int round = 3,
		int frmIdx = 0,
		NonrigidMatchingPara para = NonrigidMatchingPara(),
		char const* debug_dir = NULL);

	void nonrigid_alignment_densePtsAndClr(DeformGraph& graph_out, //in and out
		CSurface<float> const& surface_ref,
		vector< NeighborGraphNodesOfPoint > const& ngns_ref,
		S3DPointMatchSetIndexed const& match_set_3d,
		SignedDistanceFunc const* sdf_target,
		vector< cv::Mat > const& imgs_dst, //full image without distortion
		vector< cv::Mat > const& masks_dst,
		vector< GCameraView* > const& cams_dst,
		int round = 3,
		int frmIdx = 0,
		NonrigidMatchingPara para = NonrigidMatchingPara(),
		char const* debug_dir = NULL);

	//extern int rdIdx;
	class DeformGraphOptDataDensePtsAndClr : public DeformGraphOptimizationDataPlus
	{
	public:
		DeformGraphOptDataDensePtsAndClr()
			:vt_clr_downsample_rate(1)
		{};
	public:
		//int view_num;
		vector<GCameraView*> cams; //at target
		vector< cv::Mat > imgs; //without lens distortion
		vector< cv::Mat > mats_Rx;
		vector< cv::Mat > mats_Ry;
		vector< cv::Mat > mats_Gx;
		vector< cv::Mat > mats_Gy;
		vector< cv::Mat > mats_Bx;
		vector< cv::Mat > mats_By;
		vector< vector<int> > visible_cams; // the visible cameras for each point in the surface
		bool bUseColorField;

	public:
		double w_color; //weight for the color
		int vt_clr_downsample_rate;

		//internal data, will be computed automatically 
	public: //will be initialized automatically before running optization
		int constr_num_color;
		int Jnnz_num_color;
		vector< vnl_matrix_fixed<double, 3, 4> > cam_prj_mats; //camera projection matrix
	};

	class DeformGraphOptMultiDataDensePtsAndClr : public DeformGraphOptMultiDataPlus
	{
	public:
		DeformGraphOptMultiDataDensePtsAndClr()
			:vt_clr_downsample_rate(1),
			bUseColorField(false)
		{};
	public:
		//int view_num;
		vector<GCameraView*> cams; //at target
		vector< cv::Mat > imgs; //without lens distortion
		vector< cv::Mat > mats_Rx;
		vector< cv::Mat > mats_Ry;
		vector< cv::Mat > mats_Gx;
		vector< cv::Mat > mats_Gy;
		vector< cv::Mat > mats_Bx;
		vector< cv::Mat > mats_By;

		bool bUseColorField;

	public:
		double w_color; //weight for the color
		int vt_clr_downsample_rate;
	};

	void deform_graph_optimization_DensePtsAndClr(DeformGraphOptDataDensePtsAndClr& data, NonrigidMatchingPara para = NonrigidMatchingPara());
	void deform_graph_multiObj_opt_DensePtsAndClr(DeformGraphOptMultiDataDensePtsAndClr& multi_data, NonrigidMatchingPara para = NonrigidMatchingPara());

#ifdef USE_SPARSELM
	bool deform_graph_optimization_DensePtsAndClr_sparselm(DeformGraphOptDataDensePtsAndClr& data);
#endif

#ifdef USE_CERES_SOLVER
	class DeformGraphOptMultiDataDensePtsAndClrStatic : public DeformGraphOptMultiDataPlusStatic
	{
	public:
		static vector<GCameraView*> cams; //at target
		static vector< vnl_matrix_fixed<double, 3, 4> > cam_prj_mats; //at target
		static vector< cv::Mat > imgs;
		static vector< cv::Mat > mats_Rx;
		static vector< cv::Mat > mats_Ry;
		static vector< cv::Mat > mats_Gx;
		static vector< cv::Mat > mats_Gy;
		static vector< cv::Mat > mats_Bx;
		static vector< cv::Mat > mats_By;

	public:
		static double w_color; //weight for the color

	public:
		//generated by IterCallBackTransSurface
		static vector< cv::Mat > depthMats_prj; //project surface_t onto various camera spaces
		static int iterIdx;
		static int frmIdx;
		static vector< cv::Mat > imgs_proj; // for debug only
		static char const* tmp_dir;
	};

	//transform the surface and project the surface to images 
	class IterCallBackTransAndPrjSurface : public ceres::IterationCallback, public DeformGraphOptMultiDataDensePtsAndClrStatic
	{
	public:
		virtual CallbackReturnType operator()(const IterationSummary& summary)
		{
			char name[500];
			printf("-");
			surfaces_t_all.clear();
			//transform rigid surface
			rigid_surfaces_t.resize(rigid_surfaces.size());
			for (int i = 0; i < rigid_transfs.size(); i++)
			{
				rigid_surfaces_t[i] = *(rigid_surfaces[i]);
				transform_Surface_with_RigidModel(rigid_surfaces_t[i], *(rigid_transfs[i]));
				surfaces_t_all.push_back(&rigid_surfaces_t[i]);
			}
			//transform dynamic surface		
			surfaces_t.resize(surfaces.size());
			for (int i = 0; i < graphs.size(); i++)
			{
				surfaces_t[i] = *(surfaces[i]);
				transform_Surface_with_DeformGraph(surfaces_t[i], *(graphs[i]), *(ngns_dense[i]), false);
				surfaces_t_all.push_back(&surfaces_t[i]);

				if (tmp_dir != NULL)
				{
					sprintf(name, "%s/surface_t_d%d_iter%03d.bin", tmp_dir, i, iterIdx);
					surfaces_t[i].writeToFileBIN(name);
				}
			}

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
					cv::Mat depthMat = ComputeDepthMap<float>(surfaces_t_all, cams[i], remove_clockwise_tri, false, 5.0);
					depthMats_prj[i] = depthMat;
				}
			}

			if (summary.step_is_successful)
			{
				printf("-");
			}

			printf(">");

			iterIdx++;

			return SOLVER_CONTINUE;
		}
	};

	namespace RigidDenseColorTerm {
		//parameters [rod, t]
		class F : public SizedCostFunction<3, 3, 3>, public DeformGraphOptMultiDataDensePtsAndClrStatic
		{
		public:
			F(int robjIdx_, int vtIdx_, int camIdx_)
			{
				assert(robjIdx_ < rigid_transfs.size());
				this->robjIdx = robjIdx_;
				assert(vtIdx_ < rigid_surfaces[robjIdx]->vtNum);
				this->vtIdx = vtIdx_;
				assert(camIdx_ < cams.size());
				this->camIdx = camIdx_;

				this->p = rigid_surfaces[robjIdx]->vt_data_block(vtIdx);
				if (rigid_surfaces[robjIdx]->colors_of_view.size() > camIdx &&
					rigid_surfaces[robjIdx]->colors_of_view[camIdx] != NULL)
				{
					this->clr = &(rigid_surfaces[robjIdx]->colors_of_view[camIdx][3 * vtIdx]);
					if (this->clr[0] == 0 &&
						this->clr[1] == 0 &&
						this->clr[2] == 0)
						this->clr = rigid_surfaces[robjIdx]->vt_color(vtIdx);
				}
				else
					this->clr = rigid_surfaces[robjIdx]->vt_color(vtIdx);

				if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
				{
					this->clr[0] = 0;
					this->clr[1] = 0;
					this->clr[2] = 0;
				}

			};
			~F() {};

		public:
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				vnl_vector_fixed<double, 3>& rod = rigid_transfs[robjIdx]->rod;

				float const* q = rigid_surfaces_t[robjIdx].vt_data_block(vtIdx);
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
				assert(X_h[2] != 0);
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
						double clr_img[3];
						pickAColorPixel(img, u, v, clr_img);
						if (clr_img[0] != 0 || clr_img[1] != 0 || clr_img[2] != 0)
						{
							residuals[0] = (clr_img[2] / 255.0 - clr[0]);
							residuals[1] = (clr_img[1] / 255.0 - clr[1]);
							residuals[2] = (clr_img[0] / 255.0 - clr[2]);

							//debug
							if (_isnan(residuals[0]) || _isnan(residuals[1]) || _isnan(residuals[0]))
							{
								printf("<u, v>=<%f, %f>\n", u, v);
								printf("<%f, %f, %f>", clr_img[0], clr_img[1], clr_img[2]);
								printf("<%f, %f, %f>", clr[0], clr[1], clr[2]);
							}
							assert(!_isnan(residuals[0]));
							assert(!_isnan(residuals[1]));
							assert(!_isnan(residuals[2]));

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
						}
					}
				}

				if (jacobians != NULL)
				{
					double Rx = Ru * Ux + Rv * Vx;
					double Ry = Ru * Uy + Rv * Vy;
					double Rz = Ru * Uz + Rv * Vz;
					double Gx = Gu * Ux + Gv * Vx;
					double Gy = Gu * Uy + Gv * Vy;
					double Gz = Gu * Uz + Gv * Vz;
					double Bx = Bu * Ux + Bv * Vx;
					double By = Bu * Uy + Bv * Vy;
					double Bz = Bu * Uz + Bv * Vz;
					vnl_vector_fixed<double, 3> dR_dvq(Rx, Ry, Rz);
					vnl_vector_fixed<double, 3> dG_dvq(Gx, Gy, Gz);
					vnl_vector_fixed<double, 3> dB_dvq(Bx, By, Bz);

					//d(vq)/d(R)
					vnl_matrix<double> dvq_dR(3, 9);
					dvq_dR.fill(0.0);
					dvq_dR[0][0] = p[0]; dvq_dR[0][1] = p[1]; dvq_dR[0][2] = p[2];
					dvq_dR[1][3] = p[0]; dvq_dR[1][4] = p[1]; dvq_dR[1][5] = p[2];
					dvq_dR[2][6] = p[0]; dvq_dR[2][7] = p[1]; dvq_dR[2][8] = p[2];

					//d(R)/d(rod)
					vnl_matrix<double> dR_drod(9, 3);
					//dR_drod.fill(0.0);
					cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
					cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, rod.data_block());
					vnl_matrix_fixed<double, 3, 3> R;
					cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R.data_block());
					cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

					//d(vq)/d(rod)
					vnl_matrix_fixed<double, 3, 3> dvq_drod = dvq_dR * dR_drod;


					if (jacobians[0] != NULL)
					{
						vnl_vector_fixed<double, 3> jac_R = dR_dvq * dvq_drod;
						vnl_vector_fixed<double, 3> jac_G = dG_dvq * dvq_drod;
						vnl_vector_fixed<double, 3> jac_B = dB_dvq * dvq_drod;
						jacobians[0][0] = jac_R[0]; jacobians[0][1] = jac_R[1]; jacobians[0][2] = jac_R[2];
						jacobians[0][3] = jac_G[0]; jacobians[0][4] = jac_G[1]; jacobians[0][5] = jac_G[2];
						jacobians[0][6] = jac_B[0]; jacobians[0][7] = jac_B[1]; jacobians[0][8] = jac_B[2];
					}
					if (jacobians[1] != NULL)
					{
						jacobians[1][0] = dR_dvq[0]; jacobians[1][1] = dR_dvq[1]; jacobians[1][2] = dR_dvq[2];
						jacobians[1][3] = dG_dvq[0]; jacobians[1][4] = dG_dvq[1]; jacobians[1][5] = dG_dvq[2];
						jacobians[1][6] = dB_dvq[0]; jacobians[1][7] = dB_dvq[1]; jacobians[1][8] = dB_dvq[2];
					}

				}//end of if(jacobians != NULL )

				return true;
			}

		private:
			int robjIdx;
			int vtIdx;
			int camIdx;
			float* p;
			float* clr;
		};
	};

	namespace DenseColorTerm {
		//parameters: [r0, r1, r2, t0, t1, t2; r0, r1, r2, t0, t1, t2; ...]
		class F : public CostFunction, public DeformGraphOptMultiDataDensePtsAndClrStatic
		{
		public:
			F(int graphIdx_, int vtIdx_, int camIdx_)
			{
				assert(graphIdx_ < graphs.size());
				this->graphIdx = graphIdx_;

				assert(vtIdx_ < surfaces[graphIdx]->vtNum);
				this->vtIdx = vtIdx_;

				assert(camIdx_ < cams.size());
				this->camIdx = camIdx_;

				this->ngn = &((*(ngns_dense[graphIdx]))[vtIdx]);
				this->p = surfaces[graphIdx]->vt_data_block(vtIdx);
				if (surfaces[graphIdx]->colors_of_view.size() > camIdx &&
					surfaces[graphIdx]->colors_of_view[camIdx] != NULL)
				{
					this->clr = &(surfaces[graphIdx]->colors_of_view[camIdx_][3 * vtIdx_]);
					if (this->clr[0] == 0 &&
						this->clr[1] == 0 &&
						this->clr[2] == 0)
						this->clr = surfaces[graphIdx]->vt_color(vtIdx);
				}
				else
					this->clr = surfaces[graphIdx]->vt_color(vtIdx);

				if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
				{
					this->clr[0] = 0;
					this->clr[1] = 0;
					this->clr[2] = 0;
				}

				this->set_num_residuals(3);
				int ngn_num = this->ngn->neighborIndices.size();
				for (int i = 0; i < ngn_num; i++)
				{
					this->mutable_parameter_block_sizes()->push_back(3); //r0
					this->mutable_parameter_block_sizes()->push_back(3); //r1
					this->mutable_parameter_block_sizes()->push_back(3); //r2
					this->mutable_parameter_block_sizes()->push_back(1); //t0
					this->mutable_parameter_block_sizes()->push_back(1); //t1
					this->mutable_parameter_block_sizes()->push_back(1); //t2
				}
			}
			~F() {};

		public:
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				CSurface<float> const& surface_t = surfaces_t[graphIdx];
				DeformGraph* graph = graphs[graphIdx];

				float const* q = surface_t.vt_data_block(vtIdx);
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
						}
					}
				}

				if (jacobians != NULL)
				{
					double Rx = Ru * Ux + Rv * Vx;
					double Ry = Ru * Uy + Rv * Vy;
					double Rz = Ru * Uz + Rv * Vz;
					double Gx = Gu * Ux + Gv * Vx;
					double Gy = Gu * Uy + Gv * Vy;
					double Gz = Gu * Uz + Gv * Vz;
					double Bx = Bu * Ux + Bv * Vx;
					double By = Bu * Uy + Bv * Vy;
					double Bz = Bu * Uz + Bv * Vz;
					for (int j = 0; j < ngn->neighborIndices.size(); j++)
					{
						double w = ngn->weights[j];
						int k = ngn->neighborIndices[j];
						DeformGraphNode const& node = graph->nodes[k];
						double const* g = node.g.data_block();

						double tmp[3];
						tmp[0] = p[0] - g[0];
						tmp[1] = p[1] - g[1];
						tmp[2] = p[2] - g[2];
						if (jacobians[6 * j] != NULL)
						{
							//d(clr_R)/d(r0)
							jacobians[6 * j][0] = tmp[0] * Rx * w;
							jacobians[6 * j][1] = tmp[1] * Rx * w;
							jacobians[6 * j][2] = tmp[2] * Rx * w;

							//d(clr_G)/d(r0)
							jacobians[6 * j][3] = tmp[0] * Gx * w;
							jacobians[6 * j][4] = tmp[1] * Gx * w;
							jacobians[6 * j][5] = tmp[2] * Gx * w;

							//d(clr_B)/d(r0)
							jacobians[6 * j][6] = tmp[0] * Bx * w;
							jacobians[6 * j][7] = tmp[1] * Bx * w;
							jacobians[6 * j][8] = tmp[2] * Bx * w;
						}
						if (jacobians[6 * j + 1] != NULL)
						{
							//d(clr_R)/d(r1)
							jacobians[6 * j + 1][0] = tmp[0] * Ry * w;
							jacobians[6 * j + 1][1] = tmp[1] * Ry * w;
							jacobians[6 * j + 1][2] = tmp[2] * Ry * w;

							//d(clr_G)/d(r1)
							jacobians[6 * j + 1][3] = tmp[0] * Gy * w;
							jacobians[6 * j + 1][4] = tmp[1] * Gy * w;
							jacobians[6 * j + 1][5] = tmp[2] * Gy * w;

							//d(clr_B)/d(r1)
							jacobians[6 * j + 1][6] = tmp[0] * By * w;
							jacobians[6 * j + 1][7] = tmp[1] * By * w;
							jacobians[6 * j + 1][8] = tmp[2] * By * w;
						}
						if (jacobians[6 * j + 2] != NULL)
						{
							//d(clr_R)/d(r2)
							jacobians[6 * j + 2][0] = tmp[0] * Rz * w;
							jacobians[6 * j + 2][1] = tmp[1] * Rz * w;
							jacobians[6 * j + 2][2] = tmp[2] * Rz * w;

							//d(clr_G)/d(r2)
							jacobians[6 * j + 2][3] = tmp[0] * Gz * w;
							jacobians[6 * j + 2][4] = tmp[1] * Gz * w;
							jacobians[6 * j + 2][5] = tmp[2] * Gz * w;

							//d(clr_B)/d(r2)
							jacobians[6 * j + 2][6] = tmp[0] * Bz * w;
							jacobians[6 * j + 2][7] = tmp[1] * Bz * w;
							jacobians[6 * j + 2][8] = tmp[2] * Bz * w;
						}
						if (jacobians[6 * j + 3] != NULL)
						{
							jacobians[6 * j + 3][0] = Rx * w; //d(clr_R)/d(t0)
							jacobians[6 * j + 3][1] = Gx * w; //d(clr_G)/d(t0)
							jacobians[6 * j + 3][2] = Bx * w; //d(clr_B)/d(t0)
						}
						if (jacobians[6 * j + 4] != NULL)
						{
							jacobians[6 * j + 4][0] = Ry * w; //d(clr_R)/d(t1)
							jacobians[6 * j + 4][1] = Gy * w; //d(clr_G)/d(t1)
							jacobians[6 * j + 4][2] = By * w; //d(clr_B)/d(t1)
						}
						if (jacobians[6 * j + 5] != NULL)
						{
							jacobians[6 * j + 5][0] = Rz * w; //d(clr_R)/d(t2)
							jacobians[6 * j + 5][1] = Gz * w; //d(clr_G)/d(t2)
							jacobians[6 * j + 5][2] = Bz * w; //d(clr_B)/d(t2)
						}
					}//end of for-j
				}//end of if(jacobians != NULL )
				return true;
			}

		private:
			int graphIdx;
			int vtIdx;
			int camIdx;
			NeighborGraphNodesOfPoint* ngn;
			float* p;
			float* clr;
		};

		//parameters: [r0, r1, r2, t0, t1, t2; r0, r1, r2, t0, t1, t2; ...] & [rod_global, t_global]
		class F2 : public CostFunction, public DeformGraphOptMultiDataDensePtsAndClrStatic
		{
		public:
			F2(int graphIdx_, int vtIdx_, int camIdx_)
			{
				assert(graphIdx_ < graphs.size());
				this->graphIdx = graphIdx_;

				assert(vtIdx_ < surfaces[graphIdx]->vtNum);
				this->vtIdx = vtIdx_;

				assert(camIdx_ < cams.size());
				this->camIdx = camIdx_;

				this->ngn = &((*(ngns_dense[graphIdx]))[vtIdx]);
				this->p = surfaces[graphIdx]->vt_data_block(vtIdx);
				if (surfaces[graphIdx]->colors_of_view.size() > camIdx &&
					surfaces[graphIdx]->colors_of_view[camIdx] != NULL)
				{
					this->clr = &(surfaces[graphIdx]->colors_of_view[camIdx_][3 * vtIdx_]);
					if (this->clr[0] == 0 &&
						this->clr[1] == 0 &&
						this->clr[2] == 0)
						this->clr = surfaces[graphIdx]->vt_color(vtIdx);
				}
				else
					this->clr = surfaces[graphIdx]->vt_color(vtIdx);

				if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
				{
					this->clr[0] = 0;
					this->clr[1] = 0;
					this->clr[2] = 0;
				}

				this->set_num_residuals(3);
				int ngn_num = this->ngn->neighborIndices.size();
				for (int i = 0; i < ngn_num; i++)
				{
					this->mutable_parameter_block_sizes()->push_back(3); //r0
					this->mutable_parameter_block_sizes()->push_back(3); //r1
					this->mutable_parameter_block_sizes()->push_back(3); //r2
					this->mutable_parameter_block_sizes()->push_back(1); //t0
					this->mutable_parameter_block_sizes()->push_back(1); //t1
					this->mutable_parameter_block_sizes()->push_back(1); //t2
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
				CSurface<float> const& surface_t = surfaces_t[graphIdx];
				DeformGraph* graph = graphs[graphIdx];

				vnl_vector_fixed<double, 3> const& T_global = graph->global_rigid.t;
				vnl_vector_fixed<double, 3> rod_global = graph->global_rigid.rod;
				vnl_matrix_fixed<double, 3, 3> R_global(0.0);
				//d(R)/d(rod)
				vnl_matrix_fixed<double, 9, 3> dR_drod(0.0);
				cv::Mat dR_drod_cv = cv::Mat(9, 3, CV_64F, dR_drod.data_block());
				cv::Mat rod_cv = cv::Mat(1, 3, CV_64F, &rod_global[0]);
				cv::Mat R_cv = cv::Mat(3, 3, CV_64F, R_global.data_block());
				cv::Rodrigues(rod_cv, R_cv, dR_drod_cv);

				float const* q = surface_t.vt_data_block(vtIdx);
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
						}
					}
				}

				if (jacobians != NULL)
				{
					vnl_matrix_fixed<double, 3, 3> dI_dvtt(0.0);
					dI_dvtt(0, 0) = Ru * Ux + Rv * Vx;
					dI_dvtt(0, 1) = Ru * Uy + Rv * Vy;
					dI_dvtt(0, 2) = Ru * Uz + Rv * Vz;
					dI_dvtt(1, 0) = Gu * Ux + Gv * Vx;
					dI_dvtt(1, 1) = Gu * Uy + Gv * Vy;
					dI_dvtt(1, 2) = Gu * Uz + Gv * Vz;
					dI_dvtt(2, 0) = Bu * Ux + Bv * Vx;
					dI_dvtt(2, 1) = Bu * Uy + Bv * Vy;
					dI_dvtt(2, 2) = Bu * Uz + Bv * Vz;

					vnl_matrix_fixed<double, 3, 3> const& dvtt_dvttn = R_global;
					vnl_matrix_fixed<double, 3, 3> dI_dvttn = dI_dvtt * dvtt_dvttn;

					vnl_vector_fixed<double, 3> vt_tn(0.0); //nonrigidly deformed point
					int ngn_num = ngn->neighborIndices.size();
					for (int j = 0; j < ngn_num; j++)
					{
						int k = ngn->neighborIndices[j];
						double w_k = ngn->weights[j];////*w_color;
						DeformGraphNode const& node = graph->nodes[k];
						vnl_vector_fixed<double, 3> const& g_k = node.g;
						vnl_vector_fixed<double, 3> const& t_k = node.t;
						vnl_matrix_fixed<double, 3, 3> const& A_k = node.A;

						vnl_vector_fixed<double, 3> tmp;
						tmp[0] = p[0] - g_k[0];
						tmp[1] = p[1] - g_k[1];
						tmp[2] = p[2] - g_k[2];
						vt_tn += (A_k * tmp + g_k + t_k) * w_k;

						if (jacobians[6 * j] != NULL)
						{
							//d(clr_R)/d(r0)
							jacobians[6 * j][0] = tmp[0] * dI_dvttn(0, 0) * w_k;
							jacobians[6 * j][1] = tmp[1] * dI_dvttn(0, 0) * w_k;
							jacobians[6 * j][2] = tmp[2] * dI_dvttn(0, 0) * w_k;

							//d(clr_G)/d(r0)
							jacobians[6 * j][3] = tmp[0] * dI_dvttn(1, 0) * w_k;
							jacobians[6 * j][4] = tmp[1] * dI_dvttn(1, 0) * w_k;
							jacobians[6 * j][5] = tmp[2] * dI_dvttn(1, 0) * w_k;

							//d(clr_B)/d(r0)
							jacobians[6 * j][6] = tmp[0] * dI_dvttn(2, 0) * w_k;
							jacobians[6 * j][7] = tmp[1] * dI_dvttn(2, 0) * w_k;
							jacobians[6 * j][8] = tmp[2] * dI_dvttn(2, 0) * w_k;
						}
						if (jacobians[6 * j + 1] != NULL)
						{
							//d(clr_R)/d(r1)
							jacobians[6 * j + 1][0] = tmp[0] * dI_dvttn(0, 1) * w_k;
							jacobians[6 * j + 1][1] = tmp[1] * dI_dvttn(0, 1) * w_k;
							jacobians[6 * j + 1][2] = tmp[2] * dI_dvttn(0, 1) * w_k;

							//d(clr_G)/d(r1)
							jacobians[6 * j + 1][3] = tmp[0] * dI_dvttn(1, 1) * w_k;
							jacobians[6 * j + 1][4] = tmp[1] * dI_dvttn(1, 1) * w_k;
							jacobians[6 * j + 1][5] = tmp[2] * dI_dvttn(1, 1) * w_k;

							//d(clr_B)/d(r1)
							jacobians[6 * j + 1][6] = tmp[0] * dI_dvttn(2, 1) * w_k;
							jacobians[6 * j + 1][7] = tmp[1] * dI_dvttn(2, 1) * w_k;
							jacobians[6 * j + 1][8] = tmp[2] * dI_dvttn(2, 1) * w_k;
						}
						if (jacobians[6 * j + 2] != NULL)
						{
							//d(clr_R)/d(r2)
							jacobians[6 * j + 2][0] = tmp[0] * dI_dvttn(0, 2) * w_k;
							jacobians[6 * j + 2][1] = tmp[1] * dI_dvttn(0, 2) * w_k;
							jacobians[6 * j + 2][2] = tmp[2] * dI_dvttn(0, 2) * w_k;

							//d(clr_G)/d(r2)
							jacobians[6 * j + 2][3] = tmp[0] * dI_dvttn(1, 2) * w_k;
							jacobians[6 * j + 2][4] = tmp[1] * dI_dvttn(1, 2) * w_k;
							jacobians[6 * j + 2][5] = tmp[2] * dI_dvttn(1, 2) * w_k;

							//d(clr_B)/d(r2)
							jacobians[6 * j + 2][6] = tmp[0] * dI_dvttn(2, 2) * w_k;
							jacobians[6 * j + 2][7] = tmp[1] * dI_dvttn(2, 2) * w_k;
							jacobians[6 * j + 2][8] = tmp[2] * dI_dvttn(2, 2) * w_k;
						}
						if (jacobians[6 * j + 3] != NULL)
						{
							jacobians[6 * j + 3][0] = dI_dvttn(0, 0) * w_k; //d(clr_R)/d(t0)
							jacobians[6 * j + 3][1] = dI_dvttn(1, 0) * w_k; //d(clr_G)/d(t0)
							jacobians[6 * j + 3][2] = dI_dvttn(2, 0) * w_k; //d(clr_B)/d(t0)
						}
						if (jacobians[6 * j + 4] != NULL)
						{
							jacobians[6 * j + 4][0] = dI_dvttn(0, 1) * w_k; //d(clr_R)/d(t1)
							jacobians[6 * j + 4][1] = dI_dvttn(1, 1) * w_k; //d(clr_G)/d(t1)
							jacobians[6 * j + 4][2] = dI_dvttn(2, 1) * w_k; //d(clr_B)/d(t1)
						}
						if (jacobians[6 * j + 5] != NULL)
						{
							jacobians[6 * j + 5][0] = dI_dvttn(0, 2) * w_k; //d(clr_R)/d(t2)
							jacobians[6 * j + 5][1] = dI_dvttn(1, 2) * w_k; //d(clr_G)/d(t2)
							jacobians[6 * j + 5][2] = dI_dvttn(2, 2) * w_k; //d(clr_B)/d(t2)
						}
					}//end of for-j
					if (jacobians[6 * ngn_num] != NULL)
					{
						vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
						for (int i = 0; i < 3; i++)
						{
							dvtt_dR(0, i) = vt_tn[i];
							dvtt_dR(1, 3 + i) = vt_tn[i];
							dvtt_dR(2, 6 + i) = vt_tn[i];
						}
						vnl_matrix_fixed<double, 3, 3> dClr_drod = dI_dvtt * (dvtt_dR * dR_drod);

						memcpy(jacobians[6 * ngn_num], dClr_drod.data_block(), 9 * sizeof(double));
					}

					if (jacobians[6 * ngn_num + 1] != NULL)
					{
						//dclr/dt == dclr/dvt_t
						memcpy(jacobians[6 * ngn_num + 1], dI_dvtt.data_block(), 9 * sizeof(double));
					}

				}//end of if(jacobians != NULL )
				return true;
			}

		private:
			int graphIdx;
			int vtIdx;
			int camIdx;
			NeighborGraphNodesOfPoint* ngn;
			float* p;
			float* clr;
		};
	};

	namespace DenseColorFieldTerm {
		//parameters: [r0, r1, r2, t0, t1, t2; r0, r1, r2, t0, t1, t2; ...]
		class F : public CostFunction, public DeformGraphOptMultiDataDensePtsAndClrStatic
		{
		public:
			F(int graphIdx_, int vtIdx_)
			{
				assert(graphIdx_ < graphs.size());
				this->graphIdx = graphIdx_;

				assert(vtIdx_ < surfaces[graphIdx]->vtNum);
				this->ngn = &((*(ngns_dense[graphIdx]))[vtIdx_]);
				this->v = surfaces[graphIdx]->vt_data_block(vtIdx_);
				//this->n = surface->vt_normal(vtIdx_);
				this->vtIdx = vtIdx_;

				this->clr = surfaces[graphIdx]->vt_color(vtIdx);

				if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
				{
					this->clr[0] = 0;
					this->clr[1] = 0;
					this->clr[2] = 0;
				}

				this->set_num_residuals(3);
				int ngn_num = this->ngn->neighborIndices.size();
				for (int i = 0; i < ngn_num; i++)
				{
					this->mutable_parameter_block_sizes()->push_back(3); //r
					this->mutable_parameter_block_sizes()->push_back(3); //r
					this->mutable_parameter_block_sizes()->push_back(3); //r
					this->mutable_parameter_block_sizes()->push_back(1); //t
					this->mutable_parameter_block_sizes()->push_back(1); //t
					this->mutable_parameter_block_sizes()->push_back(1); //t
				}
			}
			~F() {};

		public:
			virtual bool Evaluate(double const* const* parameters,
				double* residuals,
				double** jacobians) const
			{
				CSurface<float> const& surface_t = surfaces_t[graphIdx];
				DeformGraph* graph = graphs[graphIdx];
				int ngn_num = this->ngn->neighborIndices.size();

				const float* vt_ = surface_t.vt_data_block(vtIdx);
				vnl_vector_fixed<double, 3> vt(vt_[0], vt_[1], vt_[2]);

				vnl_vector_fixed<float, 3> clr_t = sdf_target->clr_at(vt[0], vt[1], vt[2]);

				// residuals
				if ((clr_t[0] == 0.0 && clr_t[1] == 0.0 && clr_t[2] == 0.0) ||
					(clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0))
				{
					residuals[0] = 0.0;
					residuals[1] = 0.0;
					residuals[2] = 0.0;
				}
				else
				{
					residuals[0] = clr_t[0] - this->clr[0];
					residuals[1] = clr_t[1] - this->clr[1];
					residuals[2] = clr_t[2] - this->clr[2];
				}

				vnl_vector_fixed<double, 3> der_x(0.0);
				vnl_vector_fixed<double, 3> der_y(0.0);
				vnl_vector_fixed<double, 3> der_z(0.0);
				if (!sdf_target->clr_fld_der_x_at(vt.data_block(), der_x.data_block()) ||
					!sdf_target->clr_fld_der_y_at(vt.data_block(), der_y.data_block()) ||
					!sdf_target->clr_fld_der_z_at(vt.data_block(), der_z.data_block()))
				{
					residuals[0] = 0.0;
					residuals[1] = 0.0;
					residuals[2] = 0.0;
				}

				if (jacobians != NULL)
				{
					for (int i = 0; i < ngn_num; i++)
					{
						int ndIdx = this->ngn->neighborIndices[i];
						double w = this->ngn->weights[i];
						vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;
						vnl_vector_fixed<double, 3> tmp(v[0] - g_k[0],
							v[1] - g_k[1],
							v[2] - g_k[2]);

						if (jacobians[6 * i] != NULL)
						{
							// d (clr_r) / dp
							jacobians[6 * i][0] = tmp[0] * der_x[0] * w;
							jacobians[6 * i][1] = tmp[1] * der_x[0] * w;
							jacobians[6 * i][2] = tmp[2] * der_x[0] * w;

							// d (clr_g) / dp
							jacobians[6 * i][3] = tmp[0] * der_x[1] * w;
							jacobians[6 * i][4] = tmp[1] * der_x[1] * w;
							jacobians[6 * i][5] = tmp[2] * der_x[1] * w;

							// d (clr_b) / dp
							jacobians[6 * i][6] = tmp[0] * der_x[2] * w;
							jacobians[6 * i][7] = tmp[1] * der_x[2] * w;
							jacobians[6 * i][8] = tmp[2] * der_x[2] * w;
						}
						if (jacobians[6 * i + 1] != NULL)
						{
							jacobians[6 * i + 1][0] = tmp[0] * der_y[0] * w;
							jacobians[6 * i + 1][1] = tmp[1] * der_y[0] * w;
							jacobians[6 * i + 1][2] = tmp[2] * der_y[0] * w;

							jacobians[6 * i + 1][3] = tmp[0] * der_y[1] * w;
							jacobians[6 * i + 1][4] = tmp[1] * der_y[1] * w;
							jacobians[6 * i + 1][5] = tmp[2] * der_y[1] * w;

							jacobians[6 * i + 1][6] = tmp[0] * der_y[2] * w;
							jacobians[6 * i + 1][7] = tmp[1] * der_y[2] * w;
							jacobians[6 * i + 1][8] = tmp[2] * der_y[2] * w;
						}
						if (jacobians[6 * i + 2] != NULL)
						{
							jacobians[6 * i + 2][0] = tmp[0] * der_z[0] * w;
							jacobians[6 * i + 2][1] = tmp[1] * der_z[0] * w;
							jacobians[6 * i + 2][2] = tmp[2] * der_z[0] * w;

							jacobians[6 * i + 2][3] = tmp[0] * der_z[1] * w;
							jacobians[6 * i + 2][4] = tmp[1] * der_z[1] * w;
							jacobians[6 * i + 2][5] = tmp[2] * der_z[1] * w;

							jacobians[6 * i + 2][6] = tmp[0] * der_z[2] * w;
							jacobians[6 * i + 2][7] = tmp[1] * der_z[2] * w;
							jacobians[6 * i + 2][8] = tmp[2] * der_z[2] * w;
						}

						if (jacobians[6 * i + 3] != NULL)
						{
							jacobians[6 * i + 3][0] = der_x[0] * w;
							jacobians[6 * i + 3][1] = der_x[1] * w;
							jacobians[6 * i + 3][2] = der_x[2] * w;
						}
						if (jacobians[6 * i + 4] != NULL)
						{
							jacobians[6 * i + 4][0] = der_y[0] * w;
							jacobians[6 * i + 4][1] = der_y[1] * w;
							jacobians[6 * i + 4][2] = der_y[2] * w;
						}
						if (jacobians[6 * i + 5] != NULL)
						{
							jacobians[6 * i + 5][0] = der_z[0] * w;
							jacobians[6 * i + 5][1] = der_z[1] * w;
							jacobians[6 * i + 5][2] = der_z[2] * w;
						}
					}
				}

				return true;
			}

		private:
			NeighborGraphNodesOfPoint* ngn;
			float* v;
			//float* n;
			int graphIdx;
			int vtIdx;
			float* clr;
		};

		//parameters: [r0, r1, r2, t0, t1, t2; r0, r1, r2, t0, t1, t2; ...] & [rod_global, t_global]
		class F2 : public CostFunction, public DeformGraphOptMultiDataDensePtsAndClrStatic
		{
		public:
			F2(int graphIdx_, int vtIdx_)
			{
				assert(graphIdx_ < graphs.size());
				this->graphIdx = graphIdx_;

				assert(vtIdx_ < surfaces[graphIdx]->vtNum);
				this->ngn = &((*(ngns_dense[graphIdx]))[vtIdx_]);
				this->v = surfaces[graphIdx]->vt_data_block(vtIdx_);
				//this->n = surface->vt_normal(vtIdx_);
				this->vtIdx = vtIdx_;

				this->clr = surfaces[graphIdx]->vt_color(vtIdx);

				if (_isnan(this->clr[0]) || _isnan(this->clr[1]) || _isnan(this->clr[2]))
				{
					this->clr[0] = 0;
					this->clr[1] = 0;
					this->clr[2] = 0;
				}

				this->set_num_residuals(3);
				int ngn_num = this->ngn->neighborIndices.size();
				for (int i = 0; i < ngn_num; i++)
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
				CSurface<float> const& surface_t = surfaces_t[graphIdx];
				DeformGraph* graph = graphs[graphIdx];
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

				vnl_vector_fixed<double, 3> vt(v[0], v[1], v[2]);
				const float* vt_t_ = surface_t.vt_data_block(vtIdx);
				vnl_vector_fixed<double, 3> vt_t(vt_t_[0], vt_t_[1], vt_t_[2]);

				vnl_vector_fixed<float, 3> clr_t = sdf_target->clr_at(vt_t[0], vt_t[1], vt_t[2]);

				// residuals
				if ((clr_t[0] == 0.0 && clr_t[1] == 0.0 && clr_t[2] == 0.0) ||
					(clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0))
				{
					residuals[0] = 0.0;
					residuals[1] = 0.0;
					residuals[2] = 0.0;
				}
				else
				{
					residuals[0] = clr_t[0] - this->clr[0];
					residuals[1] = clr_t[1] - this->clr[1];
					residuals[2] = clr_t[2] - this->clr[2];
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
					for (int i = 0; i < ngn_num; i++)
					{
						int ndIdx = this->ngn->neighborIndices[i];
						double w_k = this->ngn->weights[i];//*w_sdf;
						vnl_vector_fixed<double, 3> const& g_k = graph->nodes[ndIdx].g;
						vnl_vector_fixed<double, 3> const& t_k = graph->nodes[ndIdx].t;
						vnl_matrix_fixed<double, 3, 3> const& A_k = graph->nodes[ndIdx].A;
						vnl_vector_fixed<double, 3> tmp;
						tmp[0] = vt[0] - g_k[0];
						tmp[1] = vt[1] - g_k[1];
						tmp[2] = vt[2] - g_k[2];
						vp_tn += (A_k * tmp + g_k + t_k) * w_k;

						if (jacobians[6 * i] != NULL)
						{
							// d (clr_r) / dr0
							jacobians[6 * i][1] = tmp[1] * dClr_dvttn[0][0] * w_k;
							jacobians[6 * i][2] = tmp[2] * dClr_dvttn[0][0] * w_k;
							jacobians[6 * i][0] = tmp[0] * dClr_dvttn[0][0] * w_k;

							// d (clr_g) / dr0
							jacobians[6 * i][3] = tmp[0] * dClr_dvttn[1][0] * w_k;
							jacobians[6 * i][4] = tmp[1] * dClr_dvttn[1][0] * w_k;
							jacobians[6 * i][5] = tmp[2] * dClr_dvttn[1][0] * w_k;

							// d (clr_b) / dr0
							jacobians[6 * i][6] = tmp[0] * dClr_dvttn[2][0] * w_k;
							jacobians[6 * i][7] = tmp[1] * dClr_dvttn[2][0] * w_k;
							jacobians[6 * i][8] = tmp[2] * dClr_dvttn[2][0] * w_k;
						}
						if (jacobians[6 * i + 1] != NULL)
						{
							jacobians[6 * i + 1][0] = tmp[0] * dClr_dvttn[0][1] * w_k;
							jacobians[6 * i + 1][1] = tmp[1] * dClr_dvttn[0][1] * w_k;
							jacobians[6 * i + 1][2] = tmp[2] * dClr_dvttn[0][1] * w_k;

							jacobians[6 * i + 1][3] = tmp[0] * dClr_dvttn[1][1] * w_k;
							jacobians[6 * i + 1][4] = tmp[1] * dClr_dvttn[1][1] * w_k;
							jacobians[6 * i + 1][5] = tmp[2] * dClr_dvttn[1][1] * w_k;

							jacobians[6 * i + 1][6] = tmp[0] * dClr_dvttn[2][1] * w_k;
							jacobians[6 * i + 1][7] = tmp[1] * dClr_dvttn[2][1] * w_k;
							jacobians[6 * i + 1][8] = tmp[2] * dClr_dvttn[2][1] * w_k;
						}
						if (jacobians[6 * i + 2] != NULL)
						{
							jacobians[6 * i + 2][0] = tmp[0] * dClr_dvttn[0][2] * w_k;
							jacobians[6 * i + 2][1] = tmp[1] * dClr_dvttn[0][2] * w_k;
							jacobians[6 * i + 2][2] = tmp[2] * dClr_dvttn[0][2] * w_k;

							jacobians[6 * i + 2][3] = tmp[0] * dClr_dvttn[1][2] * w_k;
							jacobians[6 * i + 2][4] = tmp[1] * dClr_dvttn[1][2] * w_k;
							jacobians[6 * i + 2][5] = tmp[2] * dClr_dvttn[1][2] * w_k;

							jacobians[6 * i + 2][6] = tmp[0] * dClr_dvttn[2][2] * w_k;
							jacobians[6 * i + 2][7] = tmp[1] * dClr_dvttn[2][2] * w_k;
							jacobians[6 * i + 2][8] = tmp[2] * dClr_dvttn[2][2] * w_k;
						}

						if (jacobians[6 * i + 3] != NULL)
						{
							jacobians[6 * i + 3][0] = dClr_dvttn[0][0] * w_k;
							jacobians[6 * i + 3][1] = dClr_dvttn[1][0] * w_k;
							jacobians[6 * i + 3][2] = dClr_dvttn[2][0] * w_k;
						}
						if (jacobians[6 * i + 4] != NULL)
						{
							jacobians[6 * i + 4][0] = dClr_dvttn[0][1] * w_k;
							jacobians[6 * i + 4][1] = dClr_dvttn[1][1] * w_k;
							jacobians[6 * i + 4][2] = dClr_dvttn[2][1] * w_k;
						}
						if (jacobians[6 * i + 5] != NULL)
						{
							jacobians[6 * i + 5][0] = dClr_dvttn[0][2] * w_k;
							jacobians[6 * i + 5][1] = dClr_dvttn[1][2] * w_k;
							jacobians[6 * i + 5][2] = dClr_dvttn[2][2] * w_k;
						}
					}

					if (jacobians[6 * ngn_num] != NULL)
					{
						vnl_matrix_fixed<double, 3, 9> dvtt_dR(0.0);
						for (int i = 0; i < 3; i++)
						{
							dvtt_dR(0, i) = vp_tn[i];
							dvtt_dR(1, 3 + i) = vp_tn[i];
							dvtt_dR(2, 6 + i) = vp_tn[i];
						}
						vnl_matrix_fixed<double, 3, 3> dClr_drod = dClr_dvtt * (dvtt_dR * dR_drod);

						memcpy(jacobians[6 * ngn_num], dClr_drod.data_block(), 9 * sizeof(double));
					}

					if (jacobians[6 * ngn_num + 1] != NULL)
					{
						//dclr/dt == dclr/dvt_t
						memcpy(jacobians[6 * ngn_num + 1], dClr_dvtt.data_block(), 9 * sizeof(double));
					}
				}

				return true;
			}

		private:
			NeighborGraphNodesOfPoint* ngn;
			float* v;
			//float* n;
			int graphIdx;
			int vtIdx;
			float* clr;
		};

	}

#endif

	bool calc_imgs_gradient(vector<cv::Mat> const& imgs_ori,
		vector< cv::Mat >& imgs_f,
		vector< cv::Mat >& mats_Rx,
		vector< cv::Mat >& mats_Ry,
		vector< cv::Mat >& mats_Gx,
		vector< cv::Mat >& mats_Gy,
		vector< cv::Mat >& mats_Bx,
		vector< cv::Mat >& mats_By,
		int kernal_radius = 2,
		vector<cv::Mat> const& masks = vector<cv::Mat>() //input
	);
	//private
	void extract_prj_mats(vector<GCameraView*> const& cams,
		vector< vnl_matrix_fixed<double, 3, 4> >& cam_prj_mats);


	void save_DeformGraphOptDataDensePtsAndClr_imgs_and_gradients(DeformGraphOptDataDensePtsAndClr const& data, char const* out_dir);
}
#endif