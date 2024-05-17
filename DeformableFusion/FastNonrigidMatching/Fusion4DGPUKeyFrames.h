#ifndef __FUSION4DKEYFRAMES_H__
#define __FUSION4DKEYFRAMES_H__
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

class Fusion4DKeyFrames
{
public:
	Fusion4DKeyFrames(CudaGlobalMemory* cuda_memory_fusion, int depth_num, int depth_width, int depth_height, int frame_count_key_volume = 1)
		: problem(NULL, VTS_NUM_MAX, VT_DIM, depth_num, depth_width, depth_height, false),
		linear_solver(JTJ_BLKS_NUM_MAX, ED_NODES_NUM_MAX, 12),
		vol_fusion(cuda_memory_fusion, depth_num, depth_width, depth_height, true),
		frame_count_key_volume(frame_count_key_volume),
		bUseFilteredDepthForFusion(true),
		bUseVisualHullForFusingCurrentDepth(true),
		bUseVisualHullForAlignmentMetric(true),
		bSmoothVolume(true),
		thres_align_residual_for_voxel_pruning(0.7f),
		bBilateralFilter(true),
		bUseVisualHullForMatching(true),
		bUseDepthTopBitAsSeg(true),
		bNoFusion(false),
		bComputeVisualHull(true),
		bRigidAlignment(true),
		fusion_mu(2.5),
		ed_nodes_res(4.0),
		vxl_res(0.5),
		align_mu(5.0),
		iso_surface_level(0.0f)
	{
		lm_solver.feed_linear_solver(&linear_solver);
		lm_solver.feed_problem(&problem);

		LMSolverOptions options;
		options.bUseOffDiagJtJData = true;
		options.iter_max = 2;				// M2D LM ITERATIONS

		auto conf = FusionConfig::current();
		if (conf) {
			options.iter_max = conf->GetValueWithDefault<int>("Fusion", "m2d_LM_iterations", options.iter_max);
		}

		lm_solver.set_options(options);

		checkCudaErrors(cudaMalloc(&dev_bbox_accu_volume, sizeof(BoundingBox3DCuda)));
		allocate_EDNodesParasGPU(ed_nodes_paras_init);

		this->depth_height = depth_height;
		this->depth_width = depth_width;
		frm_count = 0;
	};

	~Fusion4DKeyFrames()
	{
		CudaGlobalMemoryStatic::cleanup();
	};

public:
	void setup_cameras(vector<GCameraView*> cams_d)
	{
		vol_fusion.setup_cameras(cams_d);
		problem.jtj_helper_cuda.setup_cameras(cams_d);
	}

	EDNodesParasGPU* internel_EDNodesParasGPU_for_init(){
		return &ed_nodes_paras_init;
	}

	void set_up_1st_frame(cudaArray *cu_3dArr_depth,
		BoundingBox3D bbox,
		char const* tmp_dir, int frmIdx);

	void add_a_frame(cudaArray *cu_3dArr_depth,
		EDNodesParasGPU * ed_nodes_init,
		char const* tmp_dir = NULL,
		int frmIdx = 0,
		bool bInitBackground = false);

	void add_a_frame(cudaArray *cu_3dArr_depth,
		char* dev_buf_init_ed_paras, //initial parameters to use
		float ed_cubes_res_init,
		char const* tmp_dir = NULL,
		int frmIdx = 0,
		bool bInitBackground = false);

	void set_up_1st_frame(std::vector<cv::Mat> &depthImgs,
						  BoundingBox3D bbox,
						  char const* tmp_dir, int frmIdx);

	void add_a_frame(std::vector<cv::Mat> &depthImgs,
					EDNodesParasGPU * ed_nodes_init,
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
	float vxl_res_low;

	float fusion_mu;
	float align_mu; // mu for alignment only
	float ed_nodes_res;
	float ed_nodes_res_low;

	int frm_count; //frame counts acculuated
	int frame_count_key_volume;
	BoundingBox3DCuda *dev_bbox_accu_volume;
	S3DPointMatchSet matches_3d;
	int depth_width;
	int depth_height;
	float iso_surface_level;
	FILE* residual_error_experiment_file;

public:
	bool bUseFilteredDepthForFusion;
	bool bBilateralFilter;
	bool bUseVisualHullForFusingCurrentDepth;
	bool bUseVisualHullForAlignmentMetric;
	bool bUseVisualHullForMatching;
	bool bComputeVisualHull;
	bool bSmoothVolume;
	bool bUseDepthTopBitAsSeg;
	bool bNoFusion; //no fusion on the key frames
	bool bRigidAlignment;
	float thres_align_residual_for_voxel_pruning;
	EDNodesParasGPU ed_nodes_paras_init;
	
	vector<vector<S2DPointMatchSet>> matches_all;
};


#endif