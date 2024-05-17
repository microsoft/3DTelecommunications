#include "stdafx.h"
#include "CMotionFieldEstimationF2F.h"
#include "DeformGraph.h"
#include "nonrigid_matching_surface.h"
#include "DeformGraphOptDensePtsAndClr.h"
#include "DDF.h"
#include "KL_features.h"
#include "visual_feature_matching.h"

#include "suitesparse.h"
#include "lmsolver.h"
#include "NormalEquationSolverSuiteSparse.h"
#include "EDMatchingLSQProblem.h"
#include "EDMatchingLSQProblemCuda.h"

#include "normal_estimation.h"

#include "surface_misc.h"
#include "LMICP.h"

using namespace Fusion4D_GPU;
using namespace VolumetricFusionCuda;
using namespace NonrigidMatching;


const int FRAME_TO_DEBUG = -1;

void CMotionFieldEstimationF2F::
set_up_1st_frame(std::vector<cv::Mat> &depthImgs,
			BoundingBox3D bbox,
			char const* tmp_dir, int frmIdx)
{
	vol_fusion.feed_depth_textures(depthImgs);
	set_up_1st_frame(NULL, bbox, tmp_dir, frmIdx);
}

void CMotionFieldEstimationF2F::
set_up_1st_frame(cudaArray *cu_3dArr_depth,
				BoundingBox3D bbox,
				char const* tmp_dir, int frmIdx)
{
	char name[500];

	this->bbox = bbox;

	vol_fusion.bind_cuda_array_to_texture_depth(cu_3dArr_depth);
	vol_fusion.label_fg_and_tighten_bbox(bbox, bUseDepthTopBitAsSeg, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * vxl_res);
	vol_fusion.estimate_normal_for_depth_maps();
	
	vol_fusion.setup_volume_info(vol_fusion.dev_volume(), vol_fusion.dev_bbox_fg_cur(), vxl_res, fusion_mu);
	vol_fusion.compute_mipmap();
	vol_fusion.coarse_cube_prune_f2f(vol_fusion.dev_volume(), NULL, cuda::gpu_size_data(), 0);

	if (tmp_dir != NULL)
	{
		VolumeTwoLevelHierachy volume;
		vol_fusion.readout_volume_data(volume, false);
		sprintf(name, "%s/volume_%04d.txt", tmp_dir, frmIdx);
		volume.save_to_txt(name);

		vector<cv::Mat> mipmaps;
		vol_fusion.readout_mipmaps(mipmaps);
		for (int i = 0; i < mipmaps.size(); i++)
		{
			sprintf(name, "%s/mipmap_%04d_p%d.png", tmp_dir, frmIdx, i);
			saveDepthImageToPNG(mipmaps[i], name, true);
		}

		vector<vnl_matrix<vnl_vector_fixed<float, 3>>> normMaps;
		vol_fusion.readout_normalMaps(normMaps);
		for (int i = 0; i < normMaps.size(); i++)
		{
			cv::Mat norm_img = normalMap_to_image(normMaps[i]);
			sprintf(name, "%s/normal_%04d_p%d.bmp", tmp_dir, frmIdx, i);
			imwrite(name, norm_img);
			norm_img.release();
		}
	}

	vol_fusion.init_volume_with_curr_depth(vol_fusion.dev_volume(), true);
	vol_fusion.marching_cubes(vol_fusion.dev_volume(), vol_fusion.dev_vts(), vol_fusion.vts_num_gpu(), vol_fusion.vt_dim());

	//set vts_t as vts
	vol_fusion.copy_vts_to_dev_buf(vol_fusion.dev_vts_t(), vol_fusion.vt_dim(), vol_fusion.vts_t_num_gpu(), 0, false);
	//set vts_cur as vts
	vol_fusion.copy_vts_to_dev_buf(vol_fusion.dev_vts_cur(), vol_fusion.vt_dim(), vol_fusion.vts_cur_num_gpu(), 0, false);

	if (tmp_dir != NULL)
	{
		CSurface<float> accu_surface;
		vol_fusion.readout_surface(accu_surface);
		sprintf(name, "%s/accu_surface_%04d.bin", tmp_dir, frmIdx);
		accu_surface.writeToFileBIN(name);
		sprintf(name, "%s/accu_surface_t_%04d.bin", tmp_dir, frmIdx);
		accu_surface.writeToFileBIN(name);

		CPointCloud<float> pcd;
		vol_fusion.readout_point_cloud(pcd);
		sprintf(name, "%s/pcd_%04d.txt", tmp_dir, frmIdx);
		pcd.writeToFileASCII(name);
	}

	frm_count = 1;
}

void CMotionFieldEstimationF2F::
add_a_frame(std::vector<cv::Mat> &depthImgs,
			char const* tmp_dir,
			int frmIdx,
			bool bInitBackground)
{
	vol_fusion.feed_depth_textures(depthImgs);
	add_a_frame(NULL, tmp_dir, frmIdx, bInitBackground);
}

void CMotionFieldEstimationF2F::
add_a_frame(cudaArray *cu_3dArr_depth,
			char const* tmp_dir,
			int frmIdx,
			bool bInitBackground)
{
	// 1. generate the deformation graph, and deform the reference frame to the next frame
	// 2. accumulate the surface

	char name[500];

	//==== load depth to GPU
	vol_fusion.bind_cuda_array_to_texture_depth(cu_3dArr_depth);
	vol_fusion.label_fg_and_tighten_bbox(bbox, bUseDepthTopBitAsSeg, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * vxl_res);

	if (LOGGER()->check_verbosity(Logger::Debug))
	{
		BoundingBox3D bbox_cur;
		vol_fusion.readout_bbox_cur(bbox_cur);
		bbox_cur.print();
	}

	vol_fusion.estimate_normal_for_depth_maps(bUseFilteredDepthForFusion, bBilateralFilter);

	//==============Set up the problem
	double w_data = 0.2 / DEPTH_CAMERAS_NUM *(vxl_res*vxl_res / (0.4f*0.4f));// 0.03*accu_surface.vtNum*depthImgs.size();// / depthImgs.size();
	double w_vis_hull = 0.2 / DEPTH_CAMERAS_NUM *(vxl_res*vxl_res / (0.4f*0.4f));
	if (!bUseVisualHullForMatching)
		w_vis_hull = 0.0f;
	double w_rot = 100.0; //100.0
	double w_reg = 10.0;// 1.0;// 1.0;
	double w_temporal = 0.0f;
	double w_keypts = 0.0;
	int ed_nodes_hier_levels = 1;
	float sigma_nodes_dist = 10.0;//in cm
	float sigma_vt_node_dist = ed_nodes_res / 2.5f;
	float sigma_vxl_node_dist = ed_nodes_res / 2.5f;
	float sigma_vt_node_dist_low_res = ed_nodes_res_low / 2.5f;
	float sigma_vxl_node_dist_low_res = ed_nodes_res_low / 2.5f;
	bool switch_to_low_res = false;

	vol_fusion.extract_volume_bbox(vol_fusion.dev_volume(), dev_bbox_accu_volume);
	problem.gpu_set_problem(vol_fusion.dev_vts(), vol_fusion.vt_dim(), vol_fusion.vts_num_gpu(), 
							dev_bbox_accu_volume, vol_fusion.dev_bbox_fg_cur(),
							bBilateralFilter ? vol_fusion.cu_3dArr_depth_f() : cu_3dArr_depth,
							vol_fusion.cu_3dArr_normal(),
							DEPTH_CAMERAS_NUM, depth_width, depth_height, vector<GCameraView*>(),
							ed_nodes_res, ed_nodes_res_low, w_data, w_vis_hull, w_rot, w_reg, w_temporal, w_keypts,
							align_mu, vxl_res, ed_nodes_hier_levels, sigma_nodes_dist, sigma_vt_node_dist, sigma_vt_node_dist_low_res,
							bComputeVisualHull, bUseDepthTopBitAsSeg, switch_to_low_res, bInitBackground);

	if (switch_to_low_res) {
		sigma_vt_node_dist = sigma_vt_node_dist_low_res;
		sigma_vxl_node_dist = sigma_vxl_node_dist_low_res;
	}

	problem.init_ed_graph_to_identy();

	DeformGraphHierachy graph_cur;
	if (tmp_dir != NULL)
	{
		problem.graph_cuda.readout_DeformGraph_all_levels(graph_cur);
		sprintf(name, "%s/graph_in_%04d.txt", tmp_dir, frmIdx);
		graph_cur.save_to_txt(name);

		S3DPointMatchSet matches_3d;
		problem.jtj_helper_cuda.readout_keypts_3d(matches_3d);
		sprintf(name, "%s/matches_3d_%04d_%04d.txt", tmp_dir, frmIdx - 1, frmIdx);
		save_3D_match_set(matches_3d, name);
	}

	//============rigid matching=====
	if (bRigidAlignment)
	{
		problem.rigid_alignment(8.0, 6, frmIdx);
	}

	//========NonRigid Matching=========
	//solving
	problem.w_sdf = w_data;
	problem.w_vis_hull = w_vis_hull;
	problem.w_rot = w_rot;
	problem.w_reg = w_reg;
	problem.w_temporal = w_temporal;
	problem.w_keypts = 0.0f;

	lm_solver.solve_impl(frmIdx, false, -1);

	//TODO: optimization
	problem.graph_cuda.transform_surface();
	vol_fusion.dev_copy_ed_nodes(problem.graph_cuda.dev_ed_nodes(), problem.graph_cuda.ed_nodes_num_gpu(), problem.graph_cuda.dev_rigid_transf());
	
	if (tmp_dir != NULL)
	{
		problem.graph_cuda.readout_DeformGraph_all_levels(graph_cur);
		sprintf(name, "%s/graph_%04d.txt", tmp_dir, frmIdx);
		graph_cur.save_to_txt(name);

		if (frmIdx == FRAME_TO_DEBUG)
		{
			VolumeTwoLevelHierachy volume;
			vol_fusion.readout_volume_data(volume, true);
			sprintf(name, "%s/volume_%04d_pre.txt", tmp_dir, frmIdx);
			volume.save_to_txt(name);

			TSDF sdf;
			vol_fusion.HierachyVolume_to_TSDF(volume, sdf);
			sprintf(name, "%s/volume_%04d_pre(%d-%d-%d).raw", tmp_dir, frmIdx, sdf.func_val.ni(), sdf.func_val.nj(), sdf.func_val.nk());
			sdf.func_val.save_to_rawbin(name);

			CSurface<float> surface_ref;
			sdf.generate_surface(surface_ref, 0, false);
			removeSurfaceSmallUnconnectedBlobs(surface_ref, 1000);
			surface_ref.reverse_normal();
			sprintf(name, "%s/surface_ref_%04d.ply", tmp_dir, frmIdx);
			surface_ref.writeToFilePLY(name);

			char graph_name[500];
			sprintf(graph_name, "%s/graph_dmp_%04d.txt", tmp_dir, frmIdx);
			char arr_ndIds_name[500];
			sprintf(arr_ndIds_name, "%s/arr_ndIds_dmp_%04d.txt", tmp_dir, frmIdx);
			char ed_info_name[500];
			sprintf(ed_info_name, "%s/ed_info_dmp_%04d.txt", tmp_dir, frmIdx); 
			problem.dump_ed_nodes_infos(graph_name, arr_ndIds_name, ed_info_name);
		}
	}

	//================= Fusion ========
	vol_fusion.bind_tex_ndId(problem.cu_3dArr_ndIds());
	vol_fusion.compute_mipmap();

	//fuse only current depth maps	
	vol_fusion.setup_volume_info(vol_fusion.dev_volume_cur(), vol_fusion.dev_bbox_fg_cur(), dev_bbox_accu_volume, 
		problem.graph_cuda.dev_rigid_transf(), vxl_res, fusion_mu);
	vol_fusion.coarse_cube_prune_f2f(vol_fusion.dev_volume_cur(), NULL, 
									problem.graph_cuda.vts_num_gpu(), problem.graph_cuda.vt_dim());
	vol_fusion.init_volume_with_curr_depth(vol_fusion.dev_volume_cur(), bUseVisualHullForFusingCurrentDepth&bUseDepthTopBitAsSeg);


	//compute alignment residual
	float *dev_vts = problem.graph_cuda.dev_vts();
	float* dev_vts_t = problem.graph_cuda.dev_vts_t();
	cuda::gpu_size_data vts_num_gpu = problem.graph_cuda.vts_num_gpu();
	int vt_dim = problem.graph_cuda.surface_vt_dim();
	problem.jtj_helper_cuda.project_points_to_depth_map(dev_vts_t, vts_num_gpu, vt_dim, vxl_res, true);
	
	problem.jtj_helper_cuda.calc_per_vertex_residual(dev_vts_t, problem.graph_cuda.vts_num_gpu(), 
												vt_dim, vol_fusion.dev_volume_cur(), bUseVisualHullForAlignmentMetric);

	problem.jtj_helper_cuda.aggregate_ed_nodes_residual(problem.graph_cuda.ed_nodes_num_gpu());
	
	int const* dev_ngn_indices = problem.graph_cuda.dev_vts_ngn_indices();
	float const* dev_ngn_weights = problem.graph_cuda.dev_vts_ngn_weights();

	if (tmp_dir != NULL)
	{
		CSurface<float> accu_surface_t;

		//color the surface with the residual
		float const* dev_per_vertex_residual = problem.jtj_helper_cuda.dev_per_vertex_align_residual();
		vol_fusion.set_vt_color_as_residual(dev_vts_t, dev_per_vertex_residual, problem.graph_cuda.vts_num_gpu());
		problem.getCurrentSurfaceT(accu_surface_t);
		sprintf(name, "%s/accu_surface_t0_%04d.bin", tmp_dir, frmIdx);
		accu_surface_t.writeToFileBIN(name);
	}

	//warp key volume and fuse it with current data

	vol_fusion.marching_cubes_vMesh(vol_fusion.dev_volume_cur(), vol_fusion.dev_vts_t(), vol_fusion.vts_t_num_gpu(), vol_fusion.vt_dim(),
		vol_fusion.dev_triangles(), vol_fusion.tris_num_gpu(), iso_surface_level, cDefaultResoFactor);

	vol_fusion.copy_vts_t_to_dev_buf(vol_fusion.dev_vts(), vol_fusion.vt_dim(), vol_fusion.vts_num_gpu(), 0, false);
	vol_fusion.copy_vts_t_to_dev_buf(vol_fusion.dev_vts_cur(), vol_fusion.vt_dim(), vol_fusion.vts_cur_num_gpu(), 0, false);

	problem.copy_vts_t_to_dev_buf(vol_fusion.dev_vts_prev(), vol_fusion.vt_dim(), vol_fusion.vts_prev_num_gpu());
	//color dev_vts_prev
	float const* dev_per_vertex_residual = problem.jtj_helper_cuda.dev_per_vertex_align_residual();
	vol_fusion.set_vt_color_as_residual(vol_fusion.dev_vts_prev(), dev_per_vertex_residual, vol_fusion.vts_prev_num_gpu());

	if (bInitBackground)
	{
		vol_fusion.copy_some_gpu_volume_data(vol_fusion.dev_volume_static(), vol_fusion.dev_volume());
		vol_fusion.init_volume_to_zero(vol_fusion.dev_volume());
	}

	if (tmp_dir != NULL)
	{
		CSurface<float> accu_surface_t;
		vol_fusion.readout_surface_t(accu_surface_t);
		sprintf(name, "%s/accu_surface_t2_%04d.bin", tmp_dir, frmIdx);
		accu_surface_t.writeToFileBIN(name);

		CSurface<float> accu_surface;
		vol_fusion.readout_surface(accu_surface);
		sprintf(name, "%s/accu_surface_%04d.bin", tmp_dir, frmIdx);
		accu_surface.writeToFileBIN(name);

		CSurface<float> surface_cur;
		vol_fusion.readout_surface_cur(surface_cur);
		sprintf(name, "%s/surface_cur_%04d.bin", tmp_dir, frmIdx);
		surface_cur.writeToFileBIN(name);
	}

	frm_count++;
}

void CMotionFieldEstimationF2F::
add_a_frame(cudaArray *cu_3dArr_depth,
	char* dev_buf_init_ed_nodes, //results for second gpu to use
	int buf_size,
	char const* tmp_dir,
	int frmIdx,
	bool bInitBackground)
{
	add_a_frame(cu_3dArr_depth, tmp_dir, frmIdx, bInitBackground);
	EDNodesParasGPU ed_nodes_init = ed_nodes_for_init();
	pack_EDNodesParasGPU_to_dev_buf(ed_nodes_init, dev_buf_init_ed_nodes, buf_size);
}