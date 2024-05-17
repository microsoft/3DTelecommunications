#include "affines_nonrigid_pw_opt.h"
#include "DeformGraphOptDensePtsAndClr.h"

namespace NonrigidMatching{

bool AffineSetNonrigidPWDataStatic::bUseColorField = false;

//parameters
DeformGraph * AffineSetNonrigidPWDataStatic::graph = NULL; //node the neighboring relations are not used
NeighborGraphNodesOfPointList* AffineSetNonrigidPWDataStatic::ngns = NULL;
vector<double> * AffineSetNonrigidPWDataStatic::edge_alphas = NULL;

//constants
TSDF const* AffineSetNonrigidPWDataStatic::sdf_target = NULL;
CSurface<float> const* AffineSetNonrigidPWDataStatic::surface = NULL;
S3DPointMatchSetIndexed const* AffineSetNonrigidPWDataStatic::match_set = NULL;

//internal temporary variable
CSurface<float> AffineSetNonrigidPWDataStatic::surface_t = CSurface<float>();
int AffineSetNonrigidPWDataStatic::iter = 0;
char const* AffineSetNonrigidPWDataStatic::work_dir = NULL;
vector<GCameraView*> AffineSetNonrigidPWDataStatic::cams = vector<GCameraView*>(); //at target
vector< vnl_matrix_fixed<double, 3, 4> > AffineSetNonrigidPWDataStatic::cam_prj_mats = vector< vnl_matrix_fixed<double, 3, 4> >(); //camera projection matrix
vector< cv::Mat > AffineSetNonrigidPWDataStatic::imgs = vector< cv::Mat >(); //without lens distortion
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_Rx = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_Ry = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_Gx = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_Gy = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_Bx = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::mats_By = vector< cv::Mat >();
vector< cv::Mat > AffineSetNonrigidPWDataStatic::depthMats_prj = vector< cv::Mat >(); //project surface_t onto various camera spaces
vector< cv::Mat > AffineSetNonrigidPWDataStatic::imgs_proj = vector<cv::Mat>(); // for debug only


void affine_set_nonrigid_pw_opt(CSurface<float> const& surface_in, DeformGraph *graph, NeighborGraphNodesOfPointList *ngns,
	NeighborGraphNodesOfPointList const& ngns_base,
	DDF const* sdf_target, vector<cv::Mat> imgs, vector<GCameraView*> cams, S3DPointMatchSetIndexed const& match_set, 
	vector<double> &edge_alphas_in_out,
	NonrigidMatchingPara para, 	
	char const* work_dir)
{
	CSurface<float> surface = surface_in; //the edge extraction operation requires the surface to be mutable
	AffineSetNonrigidPWDataStatic::graph = graph;
	AffineSetNonrigidPWDataStatic::ngns = ngns;
	AffineSetNonrigidPWDataStatic::surface = &surface;
	AffineSetNonrigidPWDataStatic::sdf_target = (TSDF const*)sdf_target;
	AffineSetNonrigidPWDataStatic::match_set = &match_set;
	AffineSetNonrigidPWDataStatic::work_dir = work_dir;
	AffineSetNonrigidPWDataStatic::bUseColorField = para.bUseColorField;

	if (surface.edges.size()==0)
		surface.build_edges();
	vector<double> edge_alphas(surface.edges.size(), 1.0);
	if (edge_alphas.size() == edge_alphas_in_out.size())
		edge_alphas = edge_alphas_in_out;
	AffineSetNonrigidPWDataStatic::edge_alphas = &edge_alphas;

	double len_avg = 0.0;
	for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx++)
	{
		int vtIdx_i = surface.edges[edgeIdx].first;
		int vtIdx_j = surface.edges[edgeIdx].second;

		vnl_vector_fixed<float, 3> vt_i(surface.vt_data_block(vtIdx_i));
		vnl_vector_fixed<float, 3> vt_j(surface.vt_data_block(vtIdx_j));
		double len = std::sqrt(dot_product(vt_i - vt_j, vt_i - vt_j));
		len_avg += len;
	}
	len_avg /= surface.edges.size();
	LOGGER()->trace("average edge length<%f>...", len_avg);
	
	ceres::Problem problem;
	__tic__();

	//add rotation constraints
	LOGGER()->trace("Adding...");
	if (para.w_rot > 0 && !para.bFixNonrigid)
	{
		LossFunction *loss_fun_orthogonal = new ScaledLoss(new TrivialLoss(), para.w_rot, TAKE_OWNERSHIP);
		LossFunction *loss_fun_unitary = new ScaledLoss(new TrivialLoss(), para.w_rot, TAKE_OWNERSHIP);
		LossFunction *loss_fun_det = new ScaledLoss(new TrivialLoss(), para.w_rot*5.0, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i < nodes_num; i++)
		{
			vnl_matrix_fixed<double, 3, 3> &A = graph->nodes[i].A;
			double *r0 = &(A[0][0]);
			double *r1 = &(A[1][0]);
			double *r2 = &(A[2][0]);

			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r1);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r1, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r0);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r1);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::DetCostFunction(), loss_fun_det, r0, r1, r2);
			count++;
		}
		LOGGER()->trace("Rot<%d>...", count);
	}

	if (para.w_reg > 0 && !para.bFixNonrigid)
	{
		LossFunction *loss_fun_reg = new ScaledLoss(new ArctanLoss(1.5), para.w_reg, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i<nodes_num; i++)
		{
			DeformGraphNode &node = graph->nodes[i];
			vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
			double *r0_i = &(Ai[0][0]);
			double *r1_i = &(Ai[1][0]);
			double *r2_i = &(Ai[2][0]);
			double *t_i = node.t.data_block();
			vector<int> &nb_indices = node.neighborIndices;
			for (int j = 0; j<nb_indices.size(); j++)
			{
				int ndIdx_j = nb_indices[j];
				double *t_j = graph->nodes[ndIdx_j].t.data_block();
				problem.AddResidualBlock(new AffineSetNonrigidPWEDRegularizationTerm::F(i, ndIdx_j), loss_fun_reg, r0_i, r1_i, r2_i, t_i, t_j);
				count++;
			}
		}
		LOGGER()->trace("Reg<%d>...", count);
	}

	if (para.w_zero > 0 && !para.bFixGlobalRigid && !para.bFixNonrigid)
	{
		LossFunction *loss_func_r = new ScaledLoss(new ArctanLoss(0.15), para.w_zero, TAKE_OWNERSHIP);
		LossFunction *loss_func_t = new ScaledLoss(new ArctanLoss(1.5), para.w_zero, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i < nodes_num; i++)
		{
			DeformGraphNode &node = graph->nodes[i];
			vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
			double *r0_i = &(Ai[0][0]);
			double *r1_i = &(Ai[1][0]);
			double *r2_i = &(Ai[2][0]);
			double *t_i = node.t.data_block();
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR0(), loss_func_r, r0_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR1(), loss_func_r, r1_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR2(), loss_func_r, r2_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::ZeroTranslation(), loss_func_t, t_i);
			count++;
		}
		LOGGER()->trace("Zero<%d>...", count);
	}

	if (para.w_sdf > 0 && sdf_target != NULL)
	{
		LossFunction *loss_fun_sdf = new ScaledLoss(new TrivialLoss(), para.w_sdf * para.vt_sdf_downsample_rate, TAKE_OWNERSHIP);

		int count = 0;
		for (int i = 0; i < ngns->size(); i+=para.vt_sdf_downsample_rate)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[i];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector< double* > paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());
			problem.AddResidualBlock(new AffineSetNonrigidPWDensePtsTerm::F(i), loss_fun_sdf, paras);
			count++;
		}
		LOGGER()->trace("DnsDynPts<%f, %d>...", para.w_sdf, count);
	}

	if (para.w_normal > 0 && sdf_target != NULL)
	{
		LossFunction *loss_fun_norm = new ScaledLoss(new TrivialLoss(), para.w_normal*para.vt_sdf_downsample_rate, TAKE_OWNERSHIP);

		int count = 0;
		for (int i = 0; i < ngns->size(); i += para.vt_sdf_downsample_rate)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[i];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector< double* > paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *w = &(ngn.weights[k]);
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			problem.AddResidualBlock(new AffineSetNonrigidPWNormalTerm::F(i), loss_fun_norm, paras);
			count++;
		}
		LOGGER()->trace("Normal<%f, %d>...", para.w_normal, count);
	}

	if (para.w_clr > 0)
	{
		LossFunction *loss_fun_color = new ScaledLoss(new TrivialLoss(), para.w_clr*para.vt_clr_downsample_rate, TAKE_OWNERSHIP);
		
		//get the camera visibility from the initial transformation parameter
		vector<vector<vector<int>>> visible_cams;
		if (!para.bUseColorField)
		{
			CSurface<float> surface_t = surface;
			transform_Surface_with_DeformGraph(surface_t, *graph, *ngns, true);
			vector< CSurface<float>*> surfaces_t;
			surfaces_t.push_back(&surface_t);
			calc_camera_visibility(surfaces_t, cams, visible_cams);

			const int kernal_radius = 3;
			vector< cv::Mat > imgs_f;
			vector< cv::Mat > mats_Rx;
			vector< cv::Mat > mats_Ry;
			vector< cv::Mat > mats_Gx;
			vector< cv::Mat > mats_Gy;
			vector< cv::Mat > mats_Bx;
			vector< cv::Mat > mats_By;
			LOGGER()->trace("filtering...");
			calc_imgs_gradient(imgs, imgs_f, mats_Rx, mats_Ry, mats_Gx, mats_Gy, mats_Bx, mats_By, kernal_radius);
			LOGGER()->trace("end!\n");

			AffineSetNonrigidPWDataStatic::imgs = imgs_f;
			AffineSetNonrigidPWDataStatic::mats_Rx = mats_Rx;
			AffineSetNonrigidPWDataStatic::mats_Gx = mats_Gx;
			AffineSetNonrigidPWDataStatic::mats_Bx = mats_Bx;
			AffineSetNonrigidPWDataStatic::mats_Ry = mats_Ry;
			AffineSetNonrigidPWDataStatic::mats_Gy = mats_Gy;
			AffineSetNonrigidPWDataStatic::mats_By = mats_By;
			extract_prj_mats(cams, AffineSetNonrigidPWDataStatic::cam_prj_mats);
		}

		int count = 0;
		for (int vtIdx = 0; vtIdx<surface.vtNum; vtIdx += para.vt_clr_downsample_rate)
		{
			NeighborGraphNodesOfPoint & ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			//if no color attached at a vertex, skip
			float const* clr = surface.vt_color(vtIdx);
			if (clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0)
				continue;

			vector< double* > paras;
			for (int k = 0; k<ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);

				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());

			if (!para.bUseColorField)
			{
				vector<int> const&visible_cams_vt = visible_cams[0][vtIdx];
				for (int j = 0; j < visible_cams_vt.size(); j++)
				{
					int camIdx = visible_cams_vt[j];
					problem.AddResidualBlock(new AffineSetNonrigidPWDenseClrTerm::F(vtIdx, camIdx), loss_fun_color, paras);
					count++;
				}
			}
			else
			{
				problem.AddResidualBlock(new AffineSetNonrigidPWDenseClrFldTerm::F(vtIdx), loss_fun_color, paras);
				count++;
			}
		}//end for-vtIdx
		LOGGER()->trace("DnsCLR<%f, %d>...", para.w_clr, count);

	}

	if (para.w_constr > 0)
	{
		LossFunction *loss_fun = new ScaledLoss(new ArctanLoss(para.pt_constr_thres), para.w_constr, TAKE_OWNERSHIP);

		int count = 0;
		for (int i = 0; i<match_set.indices_1.size(); i++)
		{
			int vtIdx = match_set.indices_1[i];
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector< double* > paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());
			problem.AddResidualBlock(new AffineSetNonrigidPWKeyPtsTerm::F(i), loss_fun, paras);
			count++;
		}
		LOGGER()->trace("KeyPts<%f, %d>...", para.w_constr, count);
	}

	if (para.bUseZollhofer && ( (!para.bFixEDWeights && para.w_weights_smooth > 0) || (!para.bFixNonrigid && para.w_intrinsic_dst>0)) )
	{
		int count = 0;
		double weights_zollhofer_avg = para.w_weights_smooth*std::pow(para.tau_weight_smoothness, 2) +
									   para.w_intrinsic_dst*std::pow(para.tau_len_ratio, 4);

		for (int edgeIdx = 0; edgeIdx<edge_alphas.size(); edgeIdx++)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;
			vnl_vector_fixed<float, 3> vt1 = vnl_vector_fixed<float, 3>(surface.vt_data_block(vtIdx_i));
			vnl_vector_fixed<float, 3> vt2 = vnl_vector_fixed<float, 3>(surface.vt_data_block(vtIdx_j));
			double len_sq = dot_product(vt1 - vt2, vt1 - vt2);

			double weights_zollhofer = para.w_weights_smooth*std::pow(para.tau_weight_smoothness, 2) +
				para.w_intrinsic_dst*std::pow(para.tau_len_ratio, 4)*len_sq*len_sq/std::pow(len_avg, 4);
			LossFunction *loss_fun = new ScaledLoss(new TrivialLoss(), weights_zollhofer, TAKE_OWNERSHIP);

			problem.AddResidualBlock(new AffineSetNonrigidPWZollhofer::F(), loss_fun, &(edge_alphas[edgeIdx]));
			count++;
		}
		LOGGER()->trace("Zollhofer<%f, %d>...", weights_zollhofer_avg, count);
	}

	if (para.w_weights_smooth > 0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = NULL;
		if (para.bUseZollhofer)
			loss_fun = new ScaledLoss(new TrivialLoss(), para.w_weights_smooth*2.0, TAKE_OWNERSHIP);
		else
			loss_fun = new ScaledLoss(new ArctanLoss(0.2), para.w_weights_smooth, TAKE_OWNERSHIP);

		int count = 0;
		for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx++)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;

			NeighborGraphNodesOfPoint &ngn_i = (*ngns)[vtIdx_i];
			NeighborGraphNodesOfPoint &ngn_j = (*ngns)[vtIdx_j];
			if (ngn_i.neighborIndices.size() == 0 ||
				ngn_j.neighborIndices.size() == 0)
				continue;

			for (int i = 0; i < ngn_i.neighborIndices.size(); i++)
			{
				int i2 = search_val_list(ngn_j.neighborIndices, ngn_i.neighborIndices[i]);
				if (i2 == -1)
				{
					if (para.bUseZollhofer)
					{
						double dw = ngns_base[vtIdx_i].weights[i];
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightIsometricZollhofer::F2(edgeIdx, dw), 
							loss_fun,
							&(ngn_i.weights[i]), &(edge_alphas[edgeIdx]));
					}
					else
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F2(), loss_fun, &(ngn_i.weights[i]));
				}
				else
				{
					if (para.bUseZollhofer)
					{
						double dw = ngns_base[vtIdx_i].weights[i] - ngns_base[vtIdx_j].weights[i2];
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightIsometricZollhofer::F1(edgeIdx, dw),
							loss_fun,
							&(ngn_i.weights[i]), &(ngn_j.weights[i2]), &(edge_alphas[edgeIdx]));
					}
					else
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F1(), loss_fun, &(ngn_i.weights[i]), &(ngn_j.weights[i2]));
				}
			}
			for (int i = 0; i < ngn_j.neighborIndices.size(); i++)
			{
				int i2 = search_val_list(ngn_i.neighborIndices, ngn_j.neighborIndices[i]);
				if (i2 == -1)
				{
					if (para.bUseZollhofer)
					{
						double dw = ngns_base[vtIdx_j].weights[i];
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightIsometricZollhofer::F2(edgeIdx, dw),
							loss_fun,
							&(ngn_j.weights[i]), &(edge_alphas[edgeIdx]));
					}
					else
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F2(), loss_fun, &(ngn_j.weights[i]));
				}
			}

			count++;
		}
		LOGGER()->trace("WeightsSmoothness<%f, %d>...", para.w_weights_smooth, count);
	}

	if (para.w_intrinsic_dst > 0.0)
	{
		LossFunction *loss_fun = NULL;
		if (para.bUseZollhofer)
			loss_fun = new ScaledLoss(new TrivialLoss(), para.w_intrinsic_dst*2.0 / std::pow(len_avg, 4), TAKE_OWNERSHIP);
		else
			loss_fun = new ScaledLoss(new ArctanLoss(0.5*len_avg*len_avg), para.w_intrinsic_dst*para.vt_intrinsic_downsample_rate, TAKE_OWNERSHIP);

		int count = 0;
		for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx+=para.vt_intrinsic_downsample_rate)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;

			NeighborGraphNodesOfPoint const& ngn_i = (*ngns)[vtIdx_i];
			NeighborGraphNodesOfPoint const& ngn_j = (*ngns)[vtIdx_j];
			if (ngn_i.neighborIndices.size() == 0 || ngn_j.neighborIndices.size() == 0)
				continue;

			vector<int> ngn_indices = ngn_i.neighborIndices;
			for (int k = 0; k<ngn_j.neighborIndices.size(); k++)
			{
				int ndIdx = ngn_j.neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}

			vector<double*> paras;
			paras.push_back(&(edge_alphas[edgeIdx]));
			for (int k = 0; k<ngn_indices.size(); k++)
			{
				int ndIdx = ngn_indices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
			}
			for (int k = 0; k < (*ngns)[vtIdx_i].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx_i].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx_j].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx_j].weights[k]));

			if (para.bUseZollhofer)
			{
				problem.AddResidualBlock(new AffineSetNonrigidPWDstIsometricZollhofer::F(vtIdx_i, vtIdx_j, ngn_indices, -1.0),
										loss_fun, paras);
				count++;
			}			
		}
		LOGGER()->trace("DstIntrinsics<%f, %d>...", para.w_intrinsic_dst, count);
	}

	if (para.w_weights_sum_one > 0.0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = new ScaledLoss(new TrivialLoss(), para.w_weights_sum_one, TAKE_OWNERSHIP);

		int count = 0;
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector<double*> paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				paras.push_back(&(ngn.weights[k]));
				problem.AddResidualBlock(new AffineSetNonrigidPWWeightSumOne::F3(), loss_fun, &(ngn.weights[k]));
			}

			problem.AddResidualBlock(new AffineSetNonrigidPWWeightSumOne::F(vtIdx), loss_fun, paras);
			count++;
		}
		LOGGER()->trace("WeightsSumOne<%f, %d>...", para.w_weights_sum_one, count);
	}

	if (para.w_weights_polar > 0.0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = new ScaledLoss(new ArctanLoss(0.1), para.w_weights_polar, TAKE_OWNERSHIP);
		int count = 0;
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			for (int k = 0; k < ngn.neighborIndices.size(); k++)
				problem.AddResidualBlock(new AffineSetNonrigidPWWeightPolarity::F1(), loss_fun, &(ngn.weights[k]));
			count++;
		}
		LOGGER()->trace("WeightsPolar<%f, %d>...", para.w_weights_polar, count);
	}

	if (para.w_intrinsic > 0)
	{
		int count = 0;
		for (int i = 0; i<surface.triNum; i++)
		{
			int vtIdx1 = surface.triangles[3 * i];
			int vtIdx2 = surface.triangles[3 * i + 1];
			int vtIdx3 = surface.triangles[3 * i + 2];
			if ((*ngns)[vtIdx1].neighborIndices.size() == 0 ||
				(*ngns)[vtIdx2].neighborIndices.size() == 0 ||
				(*ngns)[vtIdx3].neighborIndices.size() == 0)
				continue;			

			vnl_vector_fixed<float, 3> vt1(surface.vt_data_block(vtIdx1));
			vnl_vector_fixed<float, 3> vt2(surface.vt_data_block(vtIdx2));
			vnl_vector_fixed<float, 3> vt3(surface.vt_data_block(vtIdx3));
			double inner_prod_1 = abs(dot_product(vt2 - vt1, vt3 - vt1));
			double inner_prod_2 = abs(dot_product(vt3 - vt2, vt1 - vt2));
			double inner_prod_3 = abs(dot_product(vt1 - vt3, vt2 - vt3));

			if (inner_prod_1 == 0 &&
				inner_prod_2 == 0 &&
				inner_prod_3 == 0)
				continue;

			double clr_sim_weight = 1.0;
			LossFunction *loss_fun_intrinsic = new ScaledLoss(new ArctanLoss((inner_prod_1+inner_prod_2+inner_prod_3)*0.3), para.w_intrinsic*clr_sim_weight, TAKE_OWNERSHIP);

			//get the uion of the neighboring graph nodes of three vertices
			vector<int> ngn_indices = (*ngns)[vtIdx1].neighborIndices;
			for (int k = 0; k<(*ngns)[vtIdx2].neighborIndices.size(); k++)
			{
				int ndIdx = (*ngns)[vtIdx2].neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}
			for (int k = 0; k<(*ngns)[vtIdx3].neighborIndices.size(); k++)
			{
				int ndIdx = (*ngns)[vtIdx3].neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}

			vector< double* > paras;
			for (int k = 0; k<ngn_indices.size(); k++)
			{
				int ndIdx = ngn_indices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
			}
			for (int k = 0; k < (*ngns)[vtIdx1].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx1].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx2].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx2].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx3].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx3].weights[k]));
			problem.AddResidualBlock(new AffineSetNonrigidPWIsometricTerm::F(vtIdx1, vtIdx2, vtIdx3, ngn_indices), loss_fun_intrinsic, paras);
			count++;
		}
		LOGGER()->trace("Intrinsics<%f, %d>...", para.w_intrinsic, count);
	}

	if (para.bFixNonrigid)
	{
		//fix ED graph
		for (int j = 0; j < graph->nodes.size(); j++)
		{
			DeformGraphNode &node = graph->nodes[j];
			vnl_matrix_fixed<double, 3, 3> &A = node.A;
			double *r0 = &(A[0][0]);
			double *r1 = &(A[1][0]);
			double *r2 = &(A[2][0]);
			double *t = node.t.data_block();

			if (problem.HasParameterBlock(r0))
				problem.SetParameterBlockConstant(r0);
			if (problem.HasParameterBlock(r1))
				problem.SetParameterBlockConstant(r1);
			if (problem.HasParameterBlock(r2))
				problem.SetParameterBlockConstant(r2);
			if (problem.HasParameterBlock(t))
				problem.SetParameterBlockConstant(t);
		}
	}

	if (para.bFixEDWeights)
	{
		//fix weights
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				double *w = &(ngn.weights[k]);
				if (problem.HasParameterBlock(w))
					problem.SetParameterBlockConstant(w);
			}
		}
	}

	if (para.bFixGlobalRigid)
	{
		if (problem.HasParameterBlock(graph->global_rigid.rod.data_block()))
			problem.SetParameterBlockConstant(graph->global_rigid.rod.data_block());
		if (problem.HasParameterBlock(graph->global_rigid.t.data_block()))
			problem.SetParameterBlockConstant(graph->global_rigid.t.data_block());
	}

	if (para.bFixEdgeAlphas)
	{
		for (int i = 0; i < edge_alphas.size(); i++)
		{
			if (problem.HasParameterBlock(&(edge_alphas[i])))
				problem.SetParameterBlockConstant(&(edge_alphas[i]));
		}
	}


	Solver::Options options;
	options.max_num_iterations = para.itmax;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.jacobi_scaling = false;

	options.function_tolerance = 1e-6;
	options.gradient_tolerance = 1e-6;
	options.parameter_tolerance = 1e-6;
	options.logging_type = SILENT;// PER_MINIMIZER_ITERATION;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = omp_get_num_procs();
	options.num_linear_solver_threads =  omp_get_num_procs();
	options.dynamic_sparsity = false;
	options.callbacks.push_back(new AffineSetNonrigidPWIterCallBack());

	Solver::Summary summary;
	LOGGER()->trace("\nCeres Optimization Basic: nodes=%zd\n", graph->nodes.size());
	Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << endl;

	//clean the tempory memory
	releaseCvMats(AffineSetNonrigidPWDataStatic::imgs);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Rx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Ry);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Gx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Gy);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Bx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_By);
	releaseCvMats(AffineSetNonrigidPWDataStatic::depthMats_prj);
	releaseCvMats(AffineSetNonrigidPWDataStatic::imgs_proj);	

	edge_alphas_in_out = edge_alphas;

	LOGGER()->trace("time OptBasic: %f\n", __toc__());
}

void affine_set_nonrigid_pw_opt(CSurface<float> const& surface_in, DeformGraph *graph, NeighborGraphNodesOfPointList *ngns,
	DDF const* sdf_target, vector<cv::Mat> imgs, vector<GCameraView*> cams, S3DPointMatchSetIndexed const& match_set,
	vector<double> &edge_alphas_in_out,
	NonrigidMatchingPara para,
	char const* work_dir)
{
	CSurface<float> surface = surface_in; //the edge extraction operation requires the surface to be mutable
	AffineSetNonrigidPWDataStatic::graph = graph;
	AffineSetNonrigidPWDataStatic::ngns = ngns;
	AffineSetNonrigidPWDataStatic::surface = &surface;
	AffineSetNonrigidPWDataStatic::sdf_target = (TSDF const*)sdf_target;
	AffineSetNonrigidPWDataStatic::match_set = &match_set;
	AffineSetNonrigidPWDataStatic::work_dir = work_dir;
	AffineSetNonrigidPWDataStatic::bUseColorField = para.bUseColorField;

	if (surface.edges.size() == 0)
		surface.build_edges();
	vector<double> edge_alphas(surface.edges.size(), 1.0);
	if (edge_alphas.size() == edge_alphas_in_out.size())
		edge_alphas = edge_alphas_in_out;
	AffineSetNonrigidPWDataStatic::edge_alphas = &edge_alphas;

	double len_avg = 0.0;
	for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx++)
	{
		int vtIdx_i = surface.edges[edgeIdx].first;
		int vtIdx_j = surface.edges[edgeIdx].second;

		vnl_vector_fixed<float, 3> vt_i(surface.vt_data_block(vtIdx_i));
		vnl_vector_fixed<float, 3> vt_j(surface.vt_data_block(vtIdx_j));
		double len = std::sqrt(dot_product(vt_i - vt_j, vt_i - vt_j));
		len_avg += len;
	}
	len_avg /= surface.edges.size();
	LOGGER()->info("average edge length<%f>...", len_avg);

	ceres::Problem problem;
	__tic__();

	//add rotation constraints
	LOGGER()->info("Adding...");
	if (para.w_rot > 0 && !para.bFixNonrigid)
	{
		LossFunction *loss_fun_orthogonal = new ScaledLoss(new TrivialLoss(), para.w_rot, TAKE_OWNERSHIP);
		LossFunction *loss_fun_unitary = new ScaledLoss(new TrivialLoss(), para.w_rot, TAKE_OWNERSHIP);
		LossFunction *loss_fun_det = new ScaledLoss(new TrivialLoss(), para.w_rot*5.0, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i < nodes_num; i++)
		{
			vnl_matrix_fixed<double, 3, 3> &A = graph->nodes[i].A;
			double *r0 = &(A[0][0]);
			double *r1 = &(A[1][0]);
			double *r2 = &(A[2][0]);

			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r1);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r0, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::OrthogonalCostFunction(), loss_fun_orthogonal, r1, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r0);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r1);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::UnitaryCostFunction(), loss_fun_unitary, r2);
			problem.AddResidualBlock(new AffineSetNonrigidPWRotationConstrTerm::DetCostFunction(), loss_fun_det, r0, r1, r2);
			count++;
		}
		LOGGER()->info("Rot<%d>...", count);
	}

	if (para.w_reg > 0 && !para.bFixNonrigid)
	{
		LossFunction *loss_fun_reg = new ScaledLoss(new ArctanLoss(1.5), para.w_reg, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i<nodes_num; i++)
		{
			DeformGraphNode &node = graph->nodes[i];
			vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
			double *r0_i = &(Ai[0][0]);
			double *r1_i = &(Ai[1][0]);
			double *r2_i = &(Ai[2][0]);
			double *t_i = node.t.data_block();
			vector<int> &nb_indices = node.neighborIndices;
			for (int j = 0; j<nb_indices.size(); j++)
			{
				int ndIdx_j = nb_indices[j];
				double *t_j = graph->nodes[ndIdx_j].t.data_block();
				problem.AddResidualBlock(new AffineSetNonrigidPWEDRegularizationTerm::F(i, ndIdx_j), loss_fun_reg, r0_i, r1_i, r2_i, t_i, t_j);
				count++;
			}
		}
		LOGGER()->info("Reg<%d>...", count);
	}

	if (para.w_zero > 0 && !para.bFixGlobalRigid && !para.bFixNonrigid)
	{
		LossFunction *loss_func_r = new ScaledLoss(new ArctanLoss(0.15), para.w_zero, TAKE_OWNERSHIP);
		LossFunction *loss_func_t = new ScaledLoss(new ArctanLoss(1.5), para.w_zero, TAKE_OWNERSHIP);

		int count = 0;
		int nodes_num = graph->nodes.size();
		for (int i = 0; i < nodes_num; i++)
		{
			DeformGraphNode &node = graph->nodes[i];
			vnl_matrix_fixed<double, 3, 3> &Ai = node.A;
			double *r0_i = &(Ai[0][0]);
			double *r1_i = &(Ai[1][0]);
			double *r2_i = &(Ai[2][0]);
			double *t_i = node.t.data_block();
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR0(), loss_func_r, r0_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR1(), loss_func_r, r1_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::IdentityCostFunctionR2(), loss_func_r, r2_i);
			problem.AddResidualBlock(new AffineSetNonrigidPWRigidity::ZeroTranslation(), loss_func_t, t_i);
			count++;
		}
		LOGGER()->info("Zero<%d>...", count);
	}

	if (para.w_sdf > 0 && sdf_target != NULL)
	{
		LossFunction *loss_fun_sdf = new ScaledLoss(new TrivialLoss(), para.w_sdf, TAKE_OWNERSHIP);

		int count = 0;
		for (int i = 0; i < ngns->size(); i += para.vt_sdf_downsample_rate)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[i];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector< double* > paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());
			problem.AddResidualBlock(new AffineSetNonrigidPWDensePtsTerm::F(i), loss_fun_sdf, paras);
			count++;
		}
		LOGGER()->info("DnsDynPts<%f, %d>...", para.w_sdf, count);
	}

	if (para.w_clr > 0)
	{
		LossFunction *loss_fun_color = new ScaledLoss(new TrivialLoss(), para.w_clr, TAKE_OWNERSHIP);

		//get the camera visibility from the initial transformation parameter
		vector<vector<vector<int>>> visible_cams;
		if (!para.bUseColorField)
		{
			CSurface<float> surface_t = surface;
			transform_Surface_with_DeformGraph(surface_t, *graph, *ngns, true);
			vector< CSurface<float>*> surfaces_t;
			surfaces_t.push_back(&surface_t);
			calc_camera_visibility(surfaces_t, cams, visible_cams);

			const int kernal_radius = 3;
			vector< cv::Mat > imgs_f;
			vector< cv::Mat > mats_Rx;
			vector< cv::Mat > mats_Ry;
			vector< cv::Mat > mats_Gx;
			vector< cv::Mat > mats_Gy;
			vector< cv::Mat > mats_Bx;
			vector< cv::Mat > mats_By;
			LOGGER()->trace("filtering...");
			calc_imgs_gradient(imgs, imgs_f, mats_Rx, mats_Ry, mats_Gx, mats_Gy, mats_Bx, mats_By, kernal_radius);
			LOGGER()->trace("end!\n");

			AffineSetNonrigidPWDataStatic::imgs = imgs_f;
			AffineSetNonrigidPWDataStatic::mats_Rx = mats_Rx;
			AffineSetNonrigidPWDataStatic::mats_Gx = mats_Gx;
			AffineSetNonrigidPWDataStatic::mats_Bx = mats_Bx;
			AffineSetNonrigidPWDataStatic::mats_Ry = mats_Ry;
			AffineSetNonrigidPWDataStatic::mats_Gy = mats_Gy;
			AffineSetNonrigidPWDataStatic::mats_By = mats_By;
			extract_prj_mats(cams, AffineSetNonrigidPWDataStatic::cam_prj_mats);
		}

		int count = 0;
		for (int vtIdx = 0; vtIdx<surface.vtNum; vtIdx += para.vt_clr_downsample_rate)
		{
			NeighborGraphNodesOfPoint & ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			//if no color attached at a vertex, skip
			float const* clr = surface.vt_color(vtIdx);
			if (clr[0] == 0.0 && clr[1] == 0.0 && clr[2] == 0.0)
				continue;

			vector< double* > paras;
			for (int k = 0; k<ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);

				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());

			if (!para.bUseColorField)
			{
				vector<int> const&visible_cams_vt = visible_cams[0][vtIdx];
				for (int j = 0; j < visible_cams_vt.size(); j++)
				{
					int camIdx = visible_cams_vt[j];
					problem.AddResidualBlock(new AffineSetNonrigidPWDenseClrTerm::F(vtIdx, camIdx), loss_fun_color, paras);
					count++;
				}
			}
			else
			{
				problem.AddResidualBlock(new AffineSetNonrigidPWDenseClrFldTerm::F(vtIdx), loss_fun_color, paras);
				count++;
			}
		}//end for-vtIdx
		LOGGER()->info("DnsCLR<%f, %d>...", para.w_clr, count);

	}

	if (para.w_constr > 0)
	{
		LossFunction *loss_fun = new ScaledLoss(new ArctanLoss(para.pt_constr_thres), para.w_constr, TAKE_OWNERSHIP);

		int count = 0;
		for (int i = 0; i<match_set.indices_1.size(); i++)
		{
			int vtIdx = match_set.indices_1[i];
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector< double* > paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				int ndIdx = ngn.neighborIndices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				double *w = &(ngn.weights[k]);
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
				paras.push_back(w);
			}
			paras.push_back(graph->global_rigid.rod.data_block());
			paras.push_back(graph->global_rigid.t.data_block());
			problem.AddResidualBlock(new AffineSetNonrigidPWKeyPtsTerm::F(i), loss_fun, paras);
			count++;
		}
		LOGGER()->info("KeyPts<%f, %d>...", para.w_constr, count);
	}

	if (para.bUseZollhofer && (para.w_weights_smooth > 0 || para.w_intrinsic_dst>0))
	{
		int count = 0;
		double weights_zollhofer_avg = para.w_weights_smooth*std::pow(para.tau_weight_smoothness, 2) +
			para.w_intrinsic_dst*std::pow(para.tau_len_ratio, 4);

		for (int edgeIdx = 0; edgeIdx<edge_alphas.size(); edgeIdx++)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;
			vnl_vector_fixed<float, 3> vt1 = vnl_vector_fixed<float, 3>(surface.vt_data_block(vtIdx_i));
			vnl_vector_fixed<float, 3> vt2 = vnl_vector_fixed<float, 3>(surface.vt_data_block(vtIdx_j));
			double len_sq = dot_product(vt1 - vt2, vt1 - vt2);

			double weights_zollhofer = para.w_weights_smooth*std::pow(para.tau_weight_smoothness, 2) +
				para.w_intrinsic_dst*std::pow(para.tau_len_ratio, 4)*len_sq*len_sq / std::pow(len_avg, 4);
			LossFunction *loss_fun = new ScaledLoss(new TrivialLoss(), weights_zollhofer, TAKE_OWNERSHIP);

			problem.AddResidualBlock(new AffineSetNonrigidPWZollhofer::F(), loss_fun, &(edge_alphas[edgeIdx]));
			count++;
		}
		LOGGER()->info("Zollhofer<%f, %d>...", weights_zollhofer_avg, count);
	}

	if (para.w_weights_smooth > 0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = NULL;
		if (para.bUseZollhofer)
			loss_fun = new ScaledLoss(new TrivialLoss(), para.w_weights_smooth*2.0, TAKE_OWNERSHIP);
		else
			loss_fun = new ScaledLoss(new ArctanLoss(0.2), para.w_weights_smooth, TAKE_OWNERSHIP);

		int count = 0;
		for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx++)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;

			NeighborGraphNodesOfPoint &ngn_i = (*ngns)[vtIdx_i];
			NeighborGraphNodesOfPoint &ngn_j = (*ngns)[vtIdx_j];
			if (ngn_i.neighborIndices.size() == 0 ||
				ngn_j.neighborIndices.size() == 0)
				continue;

			//prepare the data in orders to find the weights for the same ED nodes
			vector<int> ngnIndices_cmb = ngn_i.neighborIndices;
			vector<int> ngnIdx_cmb_at_i(ngn_i.neighborIndices.size(), -1);
			for (int i = 0; i < ngnIdx_cmb_at_i.size(); i++)
				ngnIdx_cmb_at_i[i] = i;
			vector<int> ngnIdx_cmb_at_j(ngn_i.neighborIndices.size(), -1);
			for (int j = 0; j < ngn_j.neighborIndices.size(); j++)
			{
				int idx_j_at_cmb = search_val_list(ngnIndices_cmb, ngn_j.neighborIndices[j]); //-1: not found
				if (idx_j_at_cmb != -1)
				{
					ngnIdx_cmb_at_j[idx_j_at_cmb] = j;
				}
				else
				{
					ngnIndices_cmb.push_back(ngn_j.neighborIndices[j]);
					ngnIdx_cmb_at_i.push_back(-1);
					ngnIdx_cmb_at_j.push_back(j);
				}
			}

			for (int k = 0; k < ngnIndices_cmb.size(); k++)
			{
				int ngnIdx_i_k = ngnIdx_cmb_at_i[k];
				int ngnIdx_j_k = ngnIdx_cmb_at_j[k];

				double *w_i = NULL;
				if (ngnIdx_i_k != -1)
					w_i = &(ngn_i.weights[ngnIdx_i_k]);
				double *w_j = NULL;
				if (ngnIdx_j_k != -1)
					w_j = &(ngn_j.weights[ngnIdx_j_k]);

				if (para.bUseZollhofer)
				{
					if (w_i != NULL && w_j != NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothnessZollhofer::F1(edgeIdx), loss_fun, w_i, w_j, &(edge_alphas[edgeIdx]));
					else if (w_i != NULL && w_j == NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothnessZollhofer::F2(edgeIdx), loss_fun, w_i, &(edge_alphas[edgeIdx]));
					else if (w_i == NULL && w_j != NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothnessZollhofer::F2(edgeIdx), loss_fun, w_j, &(edge_alphas[edgeIdx]));
					else
					{
						LOGGER()->error("affine_set_nonrigid_pw_opt", "This should not happen!\n");
					}
				}
				else
				{
					if (w_i != NULL && w_j != NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F1(), loss_fun, w_i, w_j);
					else if (w_i != NULL && w_j == NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F2(), loss_fun, w_i);
					else if (w_i == NULL && w_j != NULL)
						problem.AddResidualBlock(new AffineSetNonrigidPWWeightSmoothness::F2(), loss_fun, w_j);
					else
					{
						LOGGER()->error("affine_set_nonrigid_pw_opt", "This should not happen!\n");
					}
				}
			}
			count++;
		}
		LOGGER()->info("WeightsSmoothness<%f, %d>...", para.w_weights_smooth, count);
	}

	if (para.w_intrinsic_dst > 0.0)
	{
		LossFunction *loss_fun = NULL;
		if (para.bUseZollhofer)
			loss_fun = new ScaledLoss(new TrivialLoss(), para.w_intrinsic_dst*2.0 / std::pow(len_avg, 4), TAKE_OWNERSHIP);
		else
			loss_fun = new ScaledLoss(new ArctanLoss(0.5*len_avg*len_avg), para.w_intrinsic_dst, TAKE_OWNERSHIP);

		int count = 0;
		for (int edgeIdx = 0; edgeIdx < surface.edges.size(); edgeIdx++)
		{
			int vtIdx_i = surface.edges[edgeIdx].first;
			int vtIdx_j = surface.edges[edgeIdx].second;

			NeighborGraphNodesOfPoint const& ngn_i = (*ngns)[vtIdx_i];
			NeighborGraphNodesOfPoint const& ngn_j = (*ngns)[vtIdx_j];
			if (ngn_i.neighborIndices.size() == 0 || ngn_j.neighborIndices.size() == 0)
				continue;

			vector<int> ngn_indices = ngn_i.neighborIndices;
			for (int k = 0; k<ngn_j.neighborIndices.size(); k++)
			{
				int ndIdx = ngn_j.neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}

			vector<double*> paras;
			paras.push_back(&(edge_alphas[edgeIdx]));
			for (int k = 0; k<ngn_indices.size(); k++)
			{
				int ndIdx = ngn_indices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
			}
			for (int k = 0; k < (*ngns)[vtIdx_i].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx_i].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx_j].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx_j].weights[k]));

			if (para.bUseZollhofer)
			{
				problem.AddResidualBlock(new AffineSetNonrigidPWDstIsometricZollhofer::F(vtIdx_i, vtIdx_j, ngn_indices, -1.0),
					loss_fun, paras);
				count++;
			}
		}
		LOGGER()->info("DstIntrinsics<%f, %d>...", para.w_intrinsic_dst, count);
	}

	if (para.w_weights_sum_one > 0.0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = new ScaledLoss(new TrivialLoss(), para.w_weights_sum_one, TAKE_OWNERSHIP);

		int count = 0;
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			vector<double*> paras;
			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				paras.push_back(&(ngn.weights[k]));
				problem.AddResidualBlock(new AffineSetNonrigidPWWeightSumOne::F3(), loss_fun, &(ngn.weights[k]));
			}

			problem.AddResidualBlock(new AffineSetNonrigidPWWeightSumOne::F(vtIdx), loss_fun, paras);
			count++;
		}
		LOGGER()->info("WeightsSumOne<%f, %d>...", para.w_weights_sum_one, count);
	}

	if (para.w_weights_polar > 0.0 && !para.bFixEDWeights)
	{
		LossFunction *loss_fun = new ScaledLoss(new ArctanLoss(0.1), para.w_weights_polar, TAKE_OWNERSHIP);
		int count = 0;
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			for (int k = 0; k < ngn.neighborIndices.size(); k++)
				problem.AddResidualBlock(new AffineSetNonrigidPWWeightPolarity::F1(), loss_fun, &(ngn.weights[k]));
			count++;
		}
		LOGGER()->info("WeightsPolar<%f, %d>...", para.w_weights_polar, count);
	}

	if (para.w_intrinsic > 0)
	{
		int count = 0;
		for (int i = 0; i<surface.triNum; i++)
		{
			int vtIdx1 = surface.triangles[3 * i];
			int vtIdx2 = surface.triangles[3 * i + 1];
			int vtIdx3 = surface.triangles[3 * i + 2];
			if ((*ngns)[vtIdx1].neighborIndices.size() == 0 ||
				(*ngns)[vtIdx2].neighborIndices.size() == 0 ||
				(*ngns)[vtIdx3].neighborIndices.size() == 0)
				continue;

			vnl_vector_fixed<float, 3> vt1(surface.vt_data_block(vtIdx1));
			vnl_vector_fixed<float, 3> vt2(surface.vt_data_block(vtIdx2));
			vnl_vector_fixed<float, 3> vt3(surface.vt_data_block(vtIdx3));
			double inner_prod_1 = abs(dot_product(vt2 - vt1, vt3 - vt1));
			double inner_prod_2 = abs(dot_product(vt3 - vt2, vt1 - vt2));
			double inner_prod_3 = abs(dot_product(vt1 - vt3, vt2 - vt3));

			if (inner_prod_1 == 0 &&
				inner_prod_2 == 0 &&
				inner_prod_3 == 0)
				continue;

			double clr_sim_weight = 1.0;
			LossFunction *loss_fun_intrinsic = new ScaledLoss(new ArctanLoss((inner_prod_1 + inner_prod_2 + inner_prod_3)*0.3), para.w_intrinsic*clr_sim_weight, TAKE_OWNERSHIP);

			//get the uion of the neighboring graph nodes of three vertices
			vector<int> ngn_indices = (*ngns)[vtIdx1].neighborIndices;
			for (int k = 0; k<(*ngns)[vtIdx2].neighborIndices.size(); k++)
			{
				int ndIdx = (*ngns)[vtIdx2].neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}
			for (int k = 0; k<(*ngns)[vtIdx3].neighborIndices.size(); k++)
			{
				int ndIdx = (*ngns)[vtIdx3].neighborIndices[k];
				if (search_val_list(ngn_indices, ndIdx) < 0)
					ngn_indices.push_back(ndIdx);
			}

			vector< double* > paras;
			for (int k = 0; k<ngn_indices.size(); k++)
			{
				int ndIdx = ngn_indices[k];
				DeformGraphNode &node = graph->nodes[ndIdx];
				vnl_matrix_fixed<double, 3, 3> &A = node.A;
				double *r0 = &(A[0][0]);
				double *r1 = &(A[1][0]);
				double *r2 = &(A[2][0]);
				double *t = node.t.data_block();
				paras.push_back(r0);
				paras.push_back(r1);
				paras.push_back(r2);
				paras.push_back(t);
			}
			for (int k = 0; k < (*ngns)[vtIdx1].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx1].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx2].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx2].weights[k]));
			for (int k = 0; k < (*ngns)[vtIdx3].weights.size(); k++)
				paras.push_back(&((*ngns)[vtIdx3].weights[k]));
			problem.AddResidualBlock(new AffineSetNonrigidPWIsometricTerm::F(vtIdx1, vtIdx2, vtIdx3, ngn_indices), loss_fun_intrinsic, paras);
			count++;
		}
		LOGGER()->info("Intrinsics<%f, %d>...", para.w_intrinsic, count);
	}

	if (para.bFixNonrigid)
	{
		//fix ED graph
		for (int j = 0; j < graph->nodes.size(); j++)
		{
			DeformGraphNode &node = graph->nodes[j];
			vnl_matrix_fixed<double, 3, 3> &A = node.A;
			double *r0 = &(A[0][0]);
			double *r1 = &(A[1][0]);
			double *r2 = &(A[2][0]);
			double *t = node.t.data_block();

			if (problem.HasParameterBlock(r0))
				problem.SetParameterBlockConstant(r0);
			if (problem.HasParameterBlock(r1))
				problem.SetParameterBlockConstant(r1);
			if (problem.HasParameterBlock(r2))
				problem.SetParameterBlockConstant(r2);
			if (problem.HasParameterBlock(t))
				problem.SetParameterBlockConstant(t);
		}
	}

	if (para.bFixEDWeights)
	{
		//fix weights
		for (int vtIdx = 0; vtIdx < surface.vtNum; vtIdx++)
		{
			NeighborGraphNodesOfPoint &ngn = (*ngns)[vtIdx];
			if (ngn.neighborIndices.size() == 0)
				continue;

			for (int k = 0; k < ngn.neighborIndices.size(); k++)
			{
				double *w = &(ngn.weights[k]);
				if (problem.HasParameterBlock(w))
					problem.SetParameterBlockConstant(w);
			}
		}
	}

	if (para.bFixGlobalRigid)
	{
		if (problem.HasParameterBlock(graph->global_rigid.rod.data_block()))
			problem.SetParameterBlockConstant(graph->global_rigid.rod.data_block());
		if (problem.HasParameterBlock(graph->global_rigid.t.data_block()))
			problem.SetParameterBlockConstant(graph->global_rigid.t.data_block());
	}

	if (para.bFixEdgeAlphas)
	{
		for (int i = 0; i < edge_alphas.size(); i++)
		{
			if (problem.HasParameterBlock(&(edge_alphas[i])))
				problem.SetParameterBlockConstant(&(edge_alphas[i]));
		}
	}

	Solver::Options options;
	options.max_num_iterations = para.itmax;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.jacobi_scaling = false;

	options.function_tolerance = 1e-6;
	options.gradient_tolerance = 1e-6;
	options.parameter_tolerance = 1e-6;
	options.logging_type = SILENT;// PER_MINIMIZER_ITERATION;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = omp_get_num_procs();
	options.num_linear_solver_threads = omp_get_num_procs();
	options.dynamic_sparsity = false;
	options.callbacks.push_back(new AffineSetNonrigidPWIterCallBack());

	Solver::Summary summary;
	LOGGER()->info("Ceres Optimization Basic: nodes=%zd", graph->nodes.size());
	Solve(options, &problem, &summary);

	*LOGGER() << Logger::Info << summary.FullReport() << Logger::endl;

	//clean the tempory memory
	releaseCvMats(AffineSetNonrigidPWDataStatic::imgs);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Rx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Ry);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Gx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Gy);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_Bx);
	releaseCvMats(AffineSetNonrigidPWDataStatic::mats_By);
	releaseCvMats(AffineSetNonrigidPWDataStatic::depthMats_prj);
	releaseCvMats(AffineSetNonrigidPWDataStatic::imgs_proj);

	edge_alphas_in_out = edge_alphas;

	LOGGER()->info("time OptBasic: %f\n", __toc__());
}


}