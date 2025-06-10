// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once

#include "BoundingBox3D.h"
#include "DDF.h"
#include "CameraView.h"
#include "DeformGraph.h"
#include "CSurface.h"
#include "UtilMatrix.h"

#include "suitesparse.h"
#include "lmsolver.h"
#include "NormalEquationSolverSuiteSparse.h"
#include "EDMatchingLSQProblem.h"
#include "EDMatchingLSQProblemCuda.h"

#include "lmsolver_gpu.h"
#include "NormalEquationSolverPCGCuda_GPU.h"

#include "volumetric_fusion_GPU.h"

#include "color_map.h"

#include "..\FusionDemo-MultiView\FusionConfig.h"

class CMotionFieldEstimationF2F
{
public:
	CMotionFieldEstimationF2F(CudaGlobalMemory* cuda_memory_fusion, int depth_num, int depth_width, int depth_height, int frame_count_key_volume = 1)
		: problem(NULL, VTS_NUM_MAX, VT_DIM, depth_num, depth_width, depth_height, false),
		linear_solver(JTJ_BLKS_NUM_MAX, ED_NODES_NUM_MAX, 12),
		vol_fusion(cuda_memory_fusion, depth_num, depth_width, depth_height, true),
		frame_count_key_volume(frame_count_key_volume),
		bUseFilteredDepthForFusion(true),
		bUseVisualHullForFusingCurrentDepth(true),
		bUseVisualHullForAlignmentMetric(true),
		bSmoothVolume(true),
		bUseDepthTopBitAsSeg(true),
		thres_align_residual_for_voxel_pruning(0.7f),
		bBilateralFilter(true),
		bUseVisualHullForMatching(true),
		bComputeVisualHull(true),
		bRigidAlignment(true),
		align_mu(5.0),
		ed_nodes_res(4.0),
		vxl_res(0.5),
		fusion_mu(5.0),
		iso_surface_level(0.0f)
	{
		lm_solver.feed_linear_solver(&linear_solver);
		lm_solver.feed_problem(&problem);

		LMSolverOptions options;
		options.bUseOffDiagJtJData = true;
		options.iter_max = 5;					// D2D LM iterations 

		auto conf = FusionConfig::current();
		if (conf) {
			options.iter_max = conf->GetValueWithDefault<int>("Fusion", "d2d_LM_iterations", options.iter_max);
		}

		lm_solver.set_options(options);

		checkCudaErrors(cudaMalloc(&dev_bbox_accu_volume, sizeof(BoundingBox3DCuda)));

		this->depth_width = depth_width;
		this->depth_height = depth_height;
		frm_count = 0;
	};

	~CMotionFieldEstimationF2F()
	{
		CudaGlobalMemoryStatic::cleanup();
	};

public:
	DeformGraphNodeCuda* dev_ed_nodes(){ return problem.dev_ed_nodes(); }
	cuda::gpu_size_data ed_nodes_num_gpu() { return problem.ed_nodes_num_gpu(); }
	int3 const* dev_ed_cubes_dim() { return problem.dev_ed_cubes_dim(); }
	float3 const* dev_ed_cubes_offset() { return problem.dev_ed_cubes_offset(); }
	float ed_cubes_res() { return problem.ed_cubes_res(); }
	cudaArray* cu_3dArr_ndIds() { return problem.cu_3dArr_ndIds(); }
	RigidTransformCuda* dev_rigid_transf() { return problem.dev_rigid_transf(); }

	//ed nodes data
	EDNodesParasGPU ed_nodes_for_init()
	{
		EDNodesParasGPU ret;
		ret.dev_ed_nodes = dev_ed_nodes();
		ret.ed_nodes_num_gpu = ed_nodes_num_gpu();
		ret.dev_ed_cubes_dim = (int3*) dev_ed_cubes_dim();
		ret.dev_ed_cubes_offset = (float3*)dev_ed_cubes_offset();
		ret.ed_cubes_res = ed_cubes_res();
		ret.cu_3dArr_ndIds = cu_3dArr_ndIds();
		ret.dev_rigid_transf = dev_rigid_transf();
		return ret;
	}

	//visual hull data

	void setup_cameras(vector<GCameraView*> cams_d)
	{
		vol_fusion.setup_cameras(cams_d);
		problem.jtj_helper_cuda.setup_cameras(cams_d);
	}

	void set_up_1st_frame(cudaArray *cu_3dArr_depth,
						BoundingBox3D bbox,
						char const* tmp_dir, int frmIdx);

	void add_a_frame(cudaArray *cu_3dArr_depth,
		char const* tmp_dir = NULL,
		int frmIdx = 0,
		bool bInitBackground = false);

	void add_a_frame(cudaArray *cu_3dArr_depth,
		char* dev_buf_init_ed_nodes, //results for second gpu to use
		int buf_size,
		char const* tmp_dir = NULL,
		int frmIdx = 0,
		bool bInitBackground = false);

	void set_up_1st_frame(std::vector<cv::Mat> &depthImgs,
						  BoundingBox3D bbox,
						  char const* tmp_dir, int frmIdx);

	void add_a_frame(std::vector<cv::Mat> &depthImgs,
					char const* tmp_dir = NULL,
					int frmIdx = 0,
					bool bInitBackground = false);


	void load_2d_matches_batch(char const* file_basename, int views_num)
	{
		abort();
	}

	void load_2d_matches_batch(char const* file_basename, vector<int> cam_indices)
	{
		abort();
	}

public:
	BoundingBox3D bbox;
	Fusion4D_GPU::EDMatchingLSQProblemCuda problem;

	LMSolverGPU lm_solver;
	Fusion4D_GPU::NormalEquationSolverPCGCuda linear_solver;

	Fusion4D_GPU::VolumetricFusionCuda::VolumetricFusionHelperCuda vol_fusion;
	float vxl_res;
	float vxl_res_low; // Low voxel resolution
	float fusion_mu;
	float align_mu; // mu for alignment only
	float ed_nodes_res;
	float ed_nodes_res_low; // Low resolution of ED nodes

	int frm_count; //frame counts acculuated
	int frame_count_key_volume;
	BoundingBox3DCuda *dev_bbox_accu_volume;
	S3DPointMatchSet matches_3d;
	int depth_width;
	int depth_height;
	float iso_surface_level;

public:
	bool bUseFilteredDepthForFusion;
	bool bBilateralFilter;
	bool bUseVisualHullForFusingCurrentDepth;
	bool bUseVisualHullForAlignmentMetric;
	bool bUseVisualHullForMatching;
	bool bComputeVisualHull;
	bool bSmoothVolume;
	bool bUseDepthTopBitAsSeg;
	bool bRigidAlignment;
	float thres_align_residual_for_voxel_pruning;
};
