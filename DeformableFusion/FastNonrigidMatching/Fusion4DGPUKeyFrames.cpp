#include "stdafx.h"
#include "Fusion4DGPUKeyFrames.h"
#include "DeformGraph.h"
#include "nonrigid_matching_surface.h"
#include "DeformGraphOptDensePtsAndClr.h"
#include "DDF.h"
#include "KL_features.h"
#include "visual_feature_matching.h"

#include "suitesparse.h"
#include "EDMatchingLSQProblem.h"
#include "lmsolver.h"
#include "normal_estimation.h"

#include "surface_misc.h"
#include "LMICP.h"
#include "../FusionDemo-MultiView/GlobalDataStatic.h"

//using namespace Fusion4D_GPU1;
using namespace Fusion4D_GPU;
using namespace VolumetricFusionCuda;
using namespace NonrigidMatching;


const int FRAME_TO_DEBUG = -1;
const bool FUSION_USE_QUAD = true;

void Fusion4DKeyFrames::
set_up_1st_frame(cudaArray *cu_3dArr_depth,
BoundingBox3D bbox,
char const* tmp_dir, int frmIdx)
{
	char name[500];

	this->bbox = bbox;
	vol_fusion.bind_cuda_array_to_texture_depth(cu_3dArr_depth);
	vol_fusion.label_fg_and_tighten_bbox(bbox, bUseDepthTopBitAsSeg, TWO_LEVEL_VOLUME_CUBE_SIZE_IN_VXL * vxl_res);

	vol_fusion.setup_volume_info(vol_fusion.dev_volume(), vol_fusion.dev_bbox_fg_cur(), vxl_res, fusion_mu);


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

void Fusion4DKeyFrames::
set_up_1st_frame(std::vector<cv::Mat> &depthImgs,
BoundingBox3D bbox,
char const* tmp_dir, int frmIdx)
{
	vol_fusion.feed_depth_textures(depthImgs);

	set_up_1st_frame(NULL, bbox, tmp_dir, frmIdx);
}

bool g_fusion_draw_current_depth = true;


void Fusion4DKeyFrames::
add_a_frame(cudaArray* cu_3dArr_depth,
	EDNodesParasGPU* ed_nodes_init,
	char const* tmp_dir,
	int frmIdx,
	bool bInitBackground)
{
	// 1. generate the deformation graph, and deform the reference frame to the next frame
	// 2. accumulate the surface

	char name[500];
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
	double w_data = 0.2 / DEPTH_CAMERAS_NUM * (vxl_res * vxl_res / (0.4f * 0.4f));
	double w_vis_hull = 0.2 / DEPTH_CAMERAS_NUM * (vxl_res * vxl_res / (0.4f * 0.4f));
	if (!bUseVisualHullForMatching)
		w_vis_hull = 0.0f;
	double w_rot = 50.0; //100.0
	double w_reg = 4.0;// 1.0;// 1.0;
	double w_temporal = 0.0f;
	double w_keypts = 0.0;
	int ed_nodes_hier_levels = 1;
	float sigma_nodes_dist = 5.0;//in cm
	float sigma_vt_node_dist = ed_nodes_res / 2.5f;
	float sigma_vxl_node_dist = ed_nodes_res / 2.5f;
	float sigma_vt_node_dist_low_res = ed_nodes_res_low / 2.5f;
	float sigma_vxl_node_dist_low_res = ed_nodes_res_low / 2.5f;
	bool switch_to_low_res = false;

	vol_fusion.extract_volume_bbox(vol_fusion.dev_volume(), dev_bbox_accu_volume);

	problem.gpu_set_problem(vol_fusion.dev_vts(), vol_fusion.vt_dim(), vol_fusion.vts_num_gpu(), dev_bbox_accu_volume, vol_fusion.dev_bbox_fg_cur(),
		bBilateralFilter ? vol_fusion.cu_3dArr_depth_f() : cu_3dArr_depth,
		vol_fusion.cu_3dArr_normal(), DEPTH_CAMERAS_NUM, depth_width, depth_height, vector<GCameraView*>(),
		ed_nodes_res, ed_nodes_res_low, w_data, w_vis_hull, w_rot, w_reg, w_temporal, w_keypts,
		align_mu, vxl_res, ed_nodes_hier_levels, sigma_nodes_dist, sigma_vt_node_dist, sigma_vt_node_dist_low_res,
		bComputeVisualHull, bUseDepthTopBitAsSeg, switch_to_low_res, bInitBackground);

	if (switch_to_low_res) {
		sigma_vt_node_dist = sigma_vt_node_dist_low_res;
		sigma_vxl_node_dist = sigma_vxl_node_dist_low_res;
	}

	//TODO: test. initialize with fast match parameters
	LOGGER()->info("init ed nodes...");
	if (ed_nodes_init)
		problem.init_ed_graph(*ed_nodes_init);
	else
	{
		if (frm_count > 1)
			problem.init_ed_graph_with_old();
		else
			problem.init_ed_graph_to_identy();
	}
	LOGGER()->info("finished!");

	DeformGraphHierachy graph_cur;
	if (tmp_dir != NULL)
	{
		if (ed_nodes_init != NULL)
		{
			DeformGraphHierachy graph_input;
			DeformGraphCuda::readout_DeformGraph_all_levels(*ed_nodes_init, graph_input);
			sprintf(name, "%s/graph_input_%04d.txt", tmp_dir, frmIdx);
			graph_input.save_to_txt(name);
		}

		problem.graph_cuda.transform_surface();
		char name[500];
		sprintf(name, "%s/accu_surface_init_%04d.bin", tmp_dir, frmIdx);
		CSurface<float> surface_in;
		problem.graph_cuda.read_out_surface_t(surface_in);
		surface_in.writeToFileBIN(name);

		S3DPointMatchSet matches_3d;
		problem.jtj_helper_cuda.readout_keypts_3d(matches_3d);
		sprintf(name, "%s/matches_3d_%04d_%04d.txt", tmp_dir, frmIdx - 1, frmIdx);
		save_3D_match_set(matches_3d, name);
	}
	
	//============rigid matching=====
	if (bRigidAlignment)
	{
		problem.rigid_alignment(8.0, 10, frmIdx);
	}
	
	//========NonRigid Matching=========
	//solving
	lm_solver.solve_impl(frmIdx, false, -1);

	//TODO: optimization
	problem.graph_cuda.transform_surface();
	vol_fusion.dev_copy_ed_nodes(problem.graph_cuda.dev_ed_nodes(), problem.graph_cuda.ed_nodes_num_gpu(), problem.graph_cuda.dev_rigid_transf());

	if (tmp_dir != NULL)
	{
		if (frmIdx == FRAME_TO_DEBUG)
		{
			VolumeTwoLevelHierachy volume;
			vol_fusion.readout_volume_data(volume, true);
			sprintf(name, "%s/volume_%04d_pre.txt", tmp_dir, frmIdx);
			volume.save_to_txt(name);

			TSDF sdf;
			vol_fusion.HierachyVolume_to_TSDF(volume, sdf);
			sprintf(name, "%s/volume_%04d_pre(%d-%d-%d).rawascii", tmp_dir, frmIdx, sdf.func_val.ni(), sdf.func_val.nj(), sdf.func_val.nk());
			sdf.func_val.save_to_file_ascii(name);

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

	vol_fusion.setup_volume_info(vol_fusion.dev_volume_cur(), vol_fusion.dev_bbox_fg_cur(), vxl_res, fusion_mu);
	vol_fusion.coarse_cube_prune_f2f(vol_fusion.dev_volume_cur(), problem.graph_cuda.dev_vts_t(),
		problem.graph_cuda.vts_num_gpu(), problem.graph_cuda.vt_dim());
	vol_fusion.init_volume_with_curr_depth(vol_fusion.dev_volume_cur(), bUseVisualHullForFusingCurrentDepth&bUseDepthTopBitAsSeg);
	vol_fusion.marching_cubes(vol_fusion.dev_volume_cur(), vol_fusion.dev_vts_cur(), vol_fusion.vts_cur_num_gpu(), vol_fusion.vt_dim());

	if (tmp_dir != NULL)
	{
		char name[500];
		CSurface<float> surface_cur;
		vol_fusion.readout_surface_cur(surface_cur);
		sprintf(name, "%s/surface_cur_%04d.bin", tmp_dir, frmIdx);
		surface_cur.writeToFileBIN(name);
	}


	//compute alignment residual
	float *dev_vts = problem.graph_cuda.dev_vts();
	float* dev_vts_t = problem.graph_cuda.dev_vts_t();
	//int vts_num = problem.graph_cuda.surface_vt_num();
	cuda::gpu_size_data vts_num_gpu = problem.graph_cuda.vts_num_gpu();
	int vt_dim = problem.graph_cuda.surface_vt_dim();

	if (GlobalDataStatic::HighQualityFrameNumber != frmIdx)
	{
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

			//color the surface with the residual
			dev_per_vertex_residual = problem.jtj_helper_cuda.dev_per_vertex_align_residual_ed_interp();
			vol_fusion.set_vt_color_as_residual(dev_vts_t, dev_per_vertex_residual, problem.graph_cuda.vts_num_gpu());
			problem.getCurrentSurfaceT(accu_surface_t);
			sprintf(name, "%s/accu_surface_t1_%04d.bin", tmp_dir, frmIdx);
			accu_surface_t.writeToFileBIN(name);
		}

		//warp key volume and fuse it with current data
		vol_fusion.init_volume_to_zero(vol_fusion.dev_volume_cur());
		float const* dev_ed_nodes_align_residual = problem.jtj_helper_cuda.dev_ed_nodes_align_residual();
		vol_fusion.warp_and_update_volume_f2f(vol_fusion.dev_volume(), vol_fusion.dev_volume_cur(),
			problem.graph_cuda.dev_ed_nodes(), dev_ed_nodes_align_residual, problem.graph_cuda.ed_nodes_num_gpu(),
			sigma_vxl_node_dist, problem.graph_cuda.dev_ed_cubes_dim(), problem.graph_cuda.dev_ed_cubes_offset(),
			problem.graph_cuda.ed_cubes_res(), problem.graph_cuda.dev_rigid_transf(), thres_align_residual_for_voxel_pruning);
		if (tmp_dir != NULL)
		{

			if (frmIdx == FRAME_TO_DEBUG)
			{
				VolumeTwoLevelHierachy volume;
				vol_fusion.readout_volume_data(*(vol_fusion.dev_volume_cur()), volume, true, true);
				sprintf(name, "%s/volume_warped_wo_cur_%04d.txt", tmp_dir, frmIdx);
				volume.save_to_txt(name);

				TSDF sdf;
				vol_fusion.HierachyVolume_to_TSDF(volume, sdf);
				sprintf(name, "%s/volume_warped_wo_cur_%04d(%d-%d-%d).raw", tmp_dir, frmIdx, sdf.func_val.ni(), sdf.func_val.nj(), sdf.func_val.nk());
				sdf.func_val.save_to_rawbin(name);

				CSurface<float> surface_warped_wo_cur;
				sdf.generate_surface(surface_warped_wo_cur, 0, false);
				removeSurfaceSmallUnconnectedBlobs(surface_warped_wo_cur, 1000);
				surface_warped_wo_cur.reverse_normal();
				sprintf(name, "%s/surface_warped_wo_cur_%04d.ply", tmp_dir, frmIdx);
				surface_warped_wo_cur.writeToFilePLY(name);


				vnl_vector<float> per_vertex_residuals;
				problem.jtj_helper_cuda.readout_per_vertex_align_residual(per_vertex_residuals); sprintf(name, "%s/per_vertex_residuals_%04d.txt", tmp_dir, frmIdx);
				saveVNLVectorASCAII(name, per_vertex_residuals);
				vnl_vector<float> per_vertex_residuals_interp;
				problem.jtj_helper_cuda.readout_per_vertex_align_residual_interp(per_vertex_residuals_interp);
				sprintf(name, "%s/per_vertex_residuals_interp_%04d.txt", tmp_dir, frmIdx);
				saveVNLVectorASCAII(name, per_vertex_residuals_interp);
				vnl_vector<float> ed_nodes_residuals;
				problem.jtj_helper_cuda.readout_ed_nodes_align_residual(ed_nodes_residuals);
				sprintf(name, "%s/ed_nodes_residuals_%04d.txt", tmp_dir, frmIdx);
				saveVNLVectorASCAII(name, ed_nodes_residuals);
			}
		}

		//get the alignment residual on the depth pixel
		problem.jtj_helper_cuda.calc_alignment_residual_on_depth(vol_fusion.dev_volume_cur(), 3);
	}


	//update the volume with current depth
	vol_fusion.update_volume_with_curr_depth(vol_fusion.dev_volume_cur(),
		problem.jtj_helper_cuda.dev_depth_align_residual_f(),
		problem.jtj_helper_cuda.depth_width_proj(), problem.jtj_helper_cuda.depth_height_proj(),
		bUseDepthTopBitAsSeg&bUseVisualHullForFusingCurrentDepth, true, true);


	float* vxl_data_raw = vol_fusion.dev_volume_cur()->data;
	if (bSmoothVolume)
	{
		vol_fusion.smooth_volume(vol_fusion.dev_volume_cur(), vol_fusion.dev_vxl_data_f(), 10.0f);
		vol_fusion.dev_volume_cur()->data = vol_fusion.dev_vxl_data_f();
	}
	if (bSmoothVolume)
		vol_fusion.dev_volume_cur()->data = vxl_data_raw;

	//determines marching cube size
	//default size
	int resoFactor = cDefaultResoFactor;
	if (GlobalDataStatic::HighQualityFrameNumber == frmIdx)
	{
		resoFactor = 1;
	}

	vol_fusion.marching_cubes_vMesh(vol_fusion.dev_volume_cur(), vol_fusion.dev_vts_t(), vol_fusion.vts_t_num_gpu(), vol_fusion.vt_dim(),
		vol_fusion.dev_triangles(), vol_fusion.tris_num_gpu(), iso_surface_level, resoFactor);

	size_t free_mem, total_mem;
	cudaMemGetInfo(&free_mem, &total_mem);

	const int inputTriangles = vol_fusion.tris_num_gpu().sync_read();

	const int inputVertices = vol_fusion.vts_t_num_gpu().sync_read();

	//// Post-process geometry.
	if(GlobalDataStatic::HighQualityFrameNumber != frmIdx)
	{		
		vol_fusion.simplifyTriangles(FUSION_USE_QUAD);		
	}

	const int outputTriangles = vol_fusion.tris_num_gpu().sync_read();
	const int outputVertices = vol_fusion.vts_t_num_gpu().sync_read();

	LOGGER()->debug("%d Triangles: %d => %d", frmIdx,  inputTriangles, outputTriangles);
	LOGGER()->debug("Vertices: %d => %d", inputVertices, outputVertices);

	//run the half-float conversion into mesh_gpu
	vol_fusion.convert_to_half_floats(vol_fusion.dev_vts_t(), vol_fusion.get_mesh_gpu(), vol_fusion.vts_t_num_gpu().dev_ptr, vol_fusion.vts_t_num_gpu().max_size);

	//TODO: marching cubes data needs to propagate to the mesh server

	if (tmp_dir != NULL)
	{
		vector<cv::Mat> residual_imgs;
		problem.jtj_helper_cuda.readout_depth_mats_align_residual(residual_imgs, true);
		for (int vId = 0; vId < residual_imgs.size(); vId++)
		{
			sprintf(name, "%s/residual_v%d_%04d.bmp", tmp_dir, vId, frmIdx);
			cv::imwrite(name, residual_imgs[vId]);
		}

		if (frmIdx == FRAME_TO_DEBUG)
		{
			VolumeTwoLevelHierachy volume;
			vol_fusion.readout_volume_data(*(vol_fusion.dev_volume_cur()), volume, true, true);
			sprintf(name, "%s/volume_warped_w_cur_%04d.txt", tmp_dir, frmIdx);
			volume.save_to_txt(name);

			TSDF sdf;
			vol_fusion.HierachyVolume_to_TSDF(volume, sdf);
			sprintf(name, "%s/volume_warped_w_cur_%04d(%d-%d-%d).raw", tmp_dir, frmIdx, sdf.func_val.ni(), sdf.func_val.nj(), sdf.func_val.nk());
			sdf.func_val.save_to_rawbin(name);

			CSurface<float> surface_warped_w_cur;
			sdf.generate_surface(surface_warped_w_cur, 0, false);
			removeSurfaceSmallUnconnectedBlobs(surface_warped_w_cur, 1000);
			surface_warped_w_cur.reverse_normal();
			sprintf(name, "%s/surface_warped_w_cur_%04d.ply", tmp_dir, frmIdx);
			surface_warped_w_cur.writeToFilePLY(name);
		}
	}

	if (bInitBackground)
	{
		vol_fusion.copy_some_gpu_volume_data(vol_fusion.dev_volume_static(), vol_fusion.dev_volume());
		vol_fusion.init_volume_to_zero(vol_fusion.dev_volume());
	}

	//============Update Reference
	if (frm_count == frame_count_key_volume)
	{
		//extract the point in reference: get p for next nonrigid matching
		float3* dev_keypts_3d_p = problem.jtj_helper_cuda.dev_keypts_3d_p();
		ushort3* dev_keypts_2d_p = problem.jtj_helper_cuda.dev_keypts_2d_p();
		problem.jtj_helper_cuda.calc_3d_keypts_in_curr(dev_keypts_3d_p, dev_keypts_2d_p);

		vol_fusion.swap_volume_pointer(vol_fusion.dev_volume(), vol_fusion.dev_volume_cur());

		vol_fusion.switch_vts_buf();

		// if you want to play with single marching cubes for matching and rendering comment vol_fusion.marching_cubes amd uncomment vol_fusion.copy_vts_t_to_dev_buf

		vol_fusion.marching_cubes(vol_fusion.dev_volume(), vol_fusion.dev_vts(), vol_fusion.vts_num_gpu(), vol_fusion.vt_dim());

		LOGGER()->debug("Set Key Volumes!");
		frm_count = 0;
	}
	else
	{
		if (!bNoFusion)
		{
			int const* dev_depth_mats_proj = problem.jtj_helper_cuda.dev_depth_mats_proj();
			int const* dev_depth_mats_corr_vtIdx = problem.jtj_helper_cuda.dev_depth_mats_corr_vtIdx();
			int depth_width_proj = problem.jtj_helper_cuda.depth_width_proj();
			int depth_height_proj = problem.jtj_helper_cuda.depth_height_proj();

			//extract the point in reference: get p for next nonrigid matching
			float3* dev_keypts_3d_p = problem.jtj_helper_cuda.dev_keypts_3d_p();
			ushort3* dev_keypts_2d_p = problem.jtj_helper_cuda.dev_keypts_2d_p();
			problem.jtj_helper_cuda.calc_3d_keypts_in_reference(dev_keypts_3d_p, dev_keypts_2d_p, dev_vts, dev_vts_t, vt_dim,
				dev_depth_mats_proj, dev_depth_mats_corr_vtIdx, depth_width_proj, depth_height_proj);

			vol_fusion.coarse_cube_prune_vWarp(vol_fusion.dev_volume(), problem.dev_ed_nodes(),
				problem.graph_cuda.ed_nodes_num_gpu().dev_ptr, sigma_vxl_node_dist,
				problem.graph_cuda.dev_ed_cubes_dim(), problem.graph_cuda.dev_ed_cubes_offset(),
				problem.ed_cubes_res(), problem.dev_rigid_transf());

			float const* dev_depth_align_residual = problem.jtj_helper_cuda.dev_depth_align_residual();
			float const* dev_ed_nodes_align_residual = problem.jtj_helper_cuda.dev_ed_nodes_align_residual();

			vol_fusion.update_volume_vWarp(vol_fusion.dev_volume(),
				problem.dev_ed_nodes(), dev_ed_nodes_align_residual, problem.graph_cuda.ed_nodes_num_gpu().dev_ptr, sigma_vxl_node_dist,
				problem.graph_cuda.dev_ed_cubes_dim(), problem.graph_cuda.dev_ed_cubes_offset(), problem.ed_cubes_res(), problem.dev_rigid_transf(),
				problem.graph_cuda.dev_vts_ngn_indices(),
				dev_depth_mats_proj, dev_depth_mats_corr_vtIdx, dev_depth_align_residual, depth_width_proj, depth_height_proj);

			vol_fusion.switch_vts_buf();
			vol_fusion.marching_cubes(vol_fusion.dev_volume(), vol_fusion.dev_vts(), vol_fusion.vts_num_gpu(), vol_fusion.vt_dim());
		}
	}

	//color dev_vts_prev
	float const* dev_per_vertex_residual = problem.jtj_helper_cuda.dev_per_vertex_align_residual();
	vol_fusion.set_vt_color_as_residual(vol_fusion.dev_vts_prev(), dev_per_vertex_residual, vol_fusion.vts_prev_num_gpu());


	if (tmp_dir != NULL)
	{
		CSurface<float> accu_surface_t;
		vol_fusion.readout_surface_mesh(vol_fusion.dev_vts_t(), vol_fusion.vts_t_num_gpu(), vol_fusion.vt_dim(),
			vol_fusion.dev_triangles(), vol_fusion.tris_num_gpu(),
			accu_surface_t);
		sprintf(name, "%s/accu_surface_t2_%04d.bin", tmp_dir, frmIdx);
		accu_surface_t.writeToFileBIN(name);

		CSurface<float> accu_surface;
		vol_fusion.readout_surface(accu_surface);
		sprintf(name, "%s/accu_surface_%04d.bin", tmp_dir, frmIdx);
		accu_surface.writeToFileBIN(name);
	}

	frm_count++;
}

void Fusion4DKeyFrames::
add_a_frame(std::vector<cv::Mat> &depthImgs,
EDNodesParasGPU * ed_nodes_init,
char const* tmp_dir,
int frmIdx,
bool bInitBackground)
{
	vol_fusion.feed_depth_textures(depthImgs);
	add_a_frame(NULL, ed_nodes_init, tmp_dir, frmIdx, bInitBackground);
}

void Fusion4DKeyFrames::
add_a_frame(cudaArray *cu_3dArr_depth,
char* dev_buf_init_ed_paras, //initial parameters to use
float ed_cubes_res_init,
char const* tmp_dir,
int frmIdx,
bool bInitBackground)
{
	unpack_EDNodesParasGPU_from_dev_buf(dev_buf_init_ed_paras, ed_nodes_paras_init);

	ed_nodes_paras_init.ed_cubes_res = ed_cubes_res_init;

	add_a_frame(cu_3dArr_depth, &ed_nodes_paras_init,
		tmp_dir, frmIdx, bInitBackground);

}