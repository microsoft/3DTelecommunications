#ifndef __TEMPORALFUSIONTHREAD_H__
#define __TEMPORALFUSIONTHREAD_H__
#include <queue>
#include <mutex>
#include <vector>
#include <algorithm>
#include <ppltasks.h>
#include <memoryapi.h>
#include <limits>
#include "omp.h"

#include <opencv2\opencv.hpp>
#include "opencv2\highgui.hpp"

#include "UtilMatrix.h"

#include "DataCaptureThread.h"
#include "TofinoMeshServer.h"

#include "../Common/debug.h"
#include "Peabody.h"

#include "CMotionFieldEstimationF2F.h"
#include "Fusion4DGPUKeyFrames.h"

// When fusion starts before data is present, it will crash if it tries to process a fully empty volume.  So we populate an initial image with 
// "blank" data, but this blank data must have 0x8000 bit set to indicate there is foreground data, and then just some generic depth value
// somewhere inside the volume (100 in this case) so there is something to process
const int FUSION_GENERIC_DEPTH_PIXEL_VALUE = 100 | 0x8000;

#define MAXFRAMESINPROCESSING 7
struct frameProcessingState
{
	volatile int inUse;
	volatile int frameId;
	volatile int depthuploadedGPU0; //started upload
	volatile int depthuploadedGPU1; //started upload
	volatile int GPU0ComputeFinished;
	volatile int coarseEDHostCopyStarted;
	volatile int coarseEDonHost;
	volatile int coarseEDGPU1CopyStarted;
	volatile int GPU1ComputeFinished;
	volatile int FrameCopiedFromGPU1;
};

//matching strategy
const int VERTEX_SIZE = TofinoFormat::GetSize(TofinoFormat::FORMAT_HALF_FLOAT6_P_N);

#define	LOAD_ALL_BEFORE_COMPUTE 0

bool data_use_network_loader = true;
bool run_mesh_server = false;

bool run_mock_server = false;
std::string server_mock_file;
int server_mock_begin = 0;
int server_mock_end = 0;

std::string controlPanelIP = "127.0.0.1";
std::string myInterfaceIP = "127.0.0.1";
std::string eventPort = "14520";
std::string statusPort = "14512";

// 0 - normal, temporal fusion
// 1-8 - show camera N only
// 9 - all cameras, no fusion

int g_select_fusion_input_view = 0;
int* g_select_fusion_input_views = NULL;

float voxel_res = DEFAULT_BEST_VOXEL_RES;
float ed_nodes_res = DEFAULT_BEST_ED_NODES_RES;
float voxel_res_low = DEFAULT_BEST_VOXEL_RES;
float ed_nodes_res_low = DEFAULT_BEST_ED_NODES_RES;

extern bool in_the_first_few_frames;
extern bool renderingEnabled();

const int initial_frames_num = 20;

static bool load_all_before_compute()
{
	return !data_use_network_loader && LOAD_ALL_BEFORE_COMPUTE;
}

struct FrameData {
	std::vector<cv::Mat> depthImgs;
	std::vector<cv::Mat> colorImgs;
	std::vector<int> colorSizeToCopy;

	//audio related
	std::vector<char*> audioData;
	std::vector<int> audioSizeToCopy;
};

class TemporalFusion : public GlobalDataStatic
{
public:
	TemporalFusion()
		:frame_count(0),
		frame_grab_count(0),
		debug_dir(),
		ddf_mu(DEFAULT_FUSION_DDF_MU),
		ddf_res(voxel_res), //in cm 0.30--400k; 0.40--200K
		ed_nodes_min_dist(ed_nodes_res),
		align_mu(DEFAULT_ALIGN_MU), //maximum distance when detecting ICP correspondence
		dual_gpu(true),
		m_thread_grab_frame(NULL)
	{
		auto conf = FusionConfig::current();
		if (!conf)
		{
			cerr << "ERROR.  NO CONFIG FILE" << endl;
			return;
		}

		depth_num = conf->GetValueWithDefault<int>("DepthGeneration", "DepthCameraCount", DEPTH_CAMERAS_NUM);
		depth_width = conf->GetValueWithDefault<int>("DepthGeneration", "DepthImageWidth", DEFAULT_DEPTH_WIDTH);
		depth_height = conf->GetValueWithDefault<int>("DepthGeneration", "DepthImageHeight", DEFAULT_DEPTH_HEIGHT);
		std::string calib_dir = conf->GetValueWithDefault<std::string>("Fusion", "CalibrationDirectory", DEFAULT_CALIBRATION_FOLDER);

		g_select_fusion_input_views = (int*)calloc(depth_num, sizeof(int));
		for (int i = 0; i < depth_num; i++) {
			g_select_fusion_input_views[i] = 1;
		}

		depthBufferSize = depth_height * depth_width * depth_num * sizeof(short);
		EDdataBufferSize = 6 * depth_width * depth_height;
		newestFrame = 0;
		oldestFrame = 0;
		dual_gpu = conf->GetValueWithDefault<bool>("Fusion", "dual_gpu", dual_gpu);

		this->bbox = BoundingBox3D(-100, 100, -200, 0, -100, 100);//for the room //-1.0 on z//BoundingBox3D(0, 100, -170, -40, -110, 110);//

		this->debug_dir_fusion4d = NULL;

		if (conf)
		{
			voxel_res = conf->GetValueWithDefault<float>("Fusion", "voxel_resolution", voxel_res);
			ed_nodes_res = conf->GetValueWithDefault<float>("Fusion", "ed_nodes_resolution", ed_nodes_res);
			voxel_res_low = conf->GetValueWithDefault<float>("Fusion", "voxel_resolution_low", voxel_res_low);
			ed_nodes_res_low = conf->GetValueWithDefault<float>("Fusion", "ed_nodes_resolution_low", ed_nodes_res_low);

			float min_coords[3], max_coords[3];

			if (!LoadVolumeBoundingBoxFromCalibration(conf, calib_dir, min_coords, max_coords))
				return;

			LOGGER()->info("TemporalFusion: computed volume bounding box min %f %f %f max %f %f %f", min_coords[0], min_coords[1], min_coords[2], max_coords[0], max_coords[1], max_coords[2]);

			this->bbox = BoundingBox3D(min_coords[0], max_coords[0], min_coords[1], max_coords[1], min_coords[2], max_coords[2]);

			ddf_mu = conf->GetValueWithDefault<float>("Fusion", "ddf_mu", ddf_mu);
			ddf_res = conf->GetValueWithDefault<float>("Fusion", "ddf_res", voxel_res);
			align_mu = conf->GetValueWithDefault<float>("Fusion", "align_mu", align_mu);
			ed_nodes_min_dist = conf->GetValueWithDefault<float>("Fusion", "ed_nodes_min_dist", ed_nodes_res);
			debug_dir = conf->GetValueWithDefault<std::string>("Fusion", "DebugOutputDirectory", debug_dir);
		}

		cuda_mem_fusion = NULL;
		InitGPUFusion();
		fusion4d = new Fusion4DKeyFrames(NULL, depth_num, depth_width, depth_height, FrameCountKeyVolume);
		fusion4d->bUseFilteredDepthForFusion = false; //turn it on to use bilaterial filtered depth for fusion
		fusion4d->bUseVisualHullForFusingCurrentDepth = true; //turn it off to make current depth looks bad. But it affects voxel-pruning. 
		fusion4d->bUseVisualHullForAlignmentMetric = true; // turn it on to get more complete surface. Warning: may increase tracking failure
		fusion4d->bUseVisualHullForMatching = true; //use visual hull constrain in LM solver
		fusion4d->thres_align_residual_for_voxel_pruning = 0.9f;
		fusion4d->bSmoothVolume = false;
		fusion4d->bBilateralFilter = true; //run bilateral filtering on depth if true, and nornigid matching will also use filtered depth
		fusion4d->bUseDepthTopBitAsSeg = true;
		fusion4d->bComputeVisualHull = true;
		fusion4d->bRigidAlignment = false;
		fusion4d->align_mu = align_mu;
		fusion4d->ed_nodes_res = ed_nodes_res;
		fusion4d->vxl_res = voxel_res;
		fusion4d->ed_nodes_res_low = ed_nodes_res_low;
		fusion4d->vxl_res_low = voxel_res_low;
		fusion4d->fusion_mu = ddf_mu;
		fusion4d->iso_surface_level = DEFAULT_FUSION_ISOS_LEVEL;

		if (conf)
		{
			conf->SetErrorIfNameNotFound(false);
			fusion4d->bUseFilteredDepthForFusion = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bUseFilteredDepthForFusion", fusion4d->bUseFilteredDepthForFusion);
			fusion4d->bUseVisualHullForFusingCurrentDepth = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bUseVisualHullForFusingCurrentDepth", fusion4d->bUseVisualHullForFusingCurrentDepth);
			fusion4d->bUseVisualHullForAlignmentMetric = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bUseVisualHullForAlignmentMetric", fusion4d->bUseVisualHullForAlignmentMetric);
			fusion4d->bUseVisualHullForMatching = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bUseVisualHullForMatching", fusion4d->bUseVisualHullForMatching);
			fusion4d->thres_align_residual_for_voxel_pruning = conf->GetValueWithDefault<float>("Fusion", "fusion4d_thres_align_residual_for_voxel_pruning", fusion4d->thres_align_residual_for_voxel_pruning);
			fusion4d->bSmoothVolume = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bSmoothVolume", fusion4d->bSmoothVolume);
			fusion4d->bBilateralFilter = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bBilateralFilter", fusion4d->bBilateralFilter);
			fusion4d->bUseDepthTopBitAsSeg = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bUseDepthTopBitAsSeg", fusion4d->bUseDepthTopBitAsSeg);
			fusion4d->bComputeVisualHull = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bComputeVisualHull", fusion4d->bComputeVisualHull);
			fusion4d->bRigidAlignment = conf->GetValueWithDefault<bool>("Fusion", "fusion4d_bRigidAlignment", fusion4d->bRigidAlignment);
			fusion4d->align_mu = conf->GetValueWithDefault<float>("Fusion", "fusion4d_align_mu", fusion4d->align_mu);
			fusion4d->ed_nodes_res = conf->GetValueWithDefault<float>("Fusion", "fusion4d_ed_nodes_res", fusion4d->ed_nodes_res);
			fusion4d->vxl_res = conf->GetValueWithDefault<float>("Fusion", "fusion4d_vxl_res", fusion4d->vxl_res);
			fusion4d->ed_nodes_res_low = conf->GetValueWithDefault<float>("Fusion", "fusion4d_ed_nodes_res_low", fusion4d->ed_nodes_res_low);
			fusion4d->vxl_res_low = conf->GetValueWithDefault<float>("Fusion", "fusion4d_vxl_res_low", fusion4d->vxl_res_low);
			fusion4d->fusion_mu = conf->GetValueWithDefault<float>("Fusion", "fusion4d_fusion_mu", fusion4d->fusion_mu);
			fusion4d->iso_surface_level = conf->GetValueWithDefault<float>("Fusion", "fusion4d_iso_surface_level", fusion4d->iso_surface_level);
		}

		if (dual_gpu) {
			cuCtxPopCurrent(&ctxFusion);

			cudaSetDevice(1);
		}

		InitGPUF2f();
		f2f_motion = new CMotionFieldEstimationF2F(NULL, depth_num, depth_width, depth_height, FrameCountKeyVolume);
		f2f_motion->bUseFilteredDepthForFusion = false; //turn it on to use bilaterial filtered depth for fusion
		f2f_motion->bUseVisualHullForFusingCurrentDepth = true; //turn it off to make current depth looks bad. But it affects voxel-pruning. 
		f2f_motion->bUseVisualHullForAlignmentMetric = false; // turn it on to get more complete surface. Warning: may increase tracking failure
		f2f_motion->bUseVisualHullForMatching = false; //use visual hull constrain in LM solver
		f2f_motion->thres_align_residual_for_voxel_pruning = 0.80f; //not relevant here
		f2f_motion->bSmoothVolume = false;
		f2f_motion->bBilateralFilter = true;
		f2f_motion->bUseDepthTopBitAsSeg = true;
		f2f_motion->bComputeVisualHull = false;
		f2f_motion->bRigidAlignment = true;
		f2f_motion->align_mu = align_mu;
		f2f_motion->ed_nodes_res = ed_nodes_res;
		f2f_motion->vxl_res = voxel_res;
		f2f_motion->ed_nodes_res_low = ed_nodes_res_low;
		f2f_motion->vxl_res_low = voxel_res_low;

		f2f_motion->fusion_mu = ddf_mu;
		f2f_motion->iso_surface_level = 0.0f;

		if (conf)
		{
			conf->SetErrorIfNameNotFound(false);
			f2f_motion->bUseFilteredDepthForFusion = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bUseFilteredDepthForFusion", f2f_motion->bUseFilteredDepthForFusion);
			f2f_motion->bUseVisualHullForFusingCurrentDepth = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bUseVisualHullForFusingCurrentDepth", f2f_motion->bUseVisualHullForFusingCurrentDepth);
			f2f_motion->bUseVisualHullForAlignmentMetric = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bUseVisualHullForAlignmentMetric", f2f_motion->bUseVisualHullForAlignmentMetric);
			f2f_motion->bUseVisualHullForMatching = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bUseVisualHullForMatching", f2f_motion->bUseVisualHullForMatching);
			f2f_motion->thres_align_residual_for_voxel_pruning = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_thres_align_residual_for_voxel_pruning", f2f_motion->thres_align_residual_for_voxel_pruning);
			f2f_motion->bSmoothVolume = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bSmoothVolume", f2f_motion->bSmoothVolume);
			f2f_motion->bBilateralFilter = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bBilateralFilter", f2f_motion->bBilateralFilter);
			f2f_motion->bUseDepthTopBitAsSeg = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bUseDepthTopBitAsSeg", f2f_motion->bUseDepthTopBitAsSeg);
			f2f_motion->bComputeVisualHull = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bComputeVisualHull", f2f_motion->bComputeVisualHull);
			f2f_motion->bRigidAlignment = conf->GetValueWithDefault<bool>("Fusion", "f2f_motion_bRigidAlignment", f2f_motion->bRigidAlignment);
			f2f_motion->align_mu = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_align_mu", f2f_motion->align_mu);
			f2f_motion->ed_nodes_res = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_ed_nodes_res", f2f_motion->ed_nodes_res);
			f2f_motion->vxl_res = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_vxl_res", f2f_motion->vxl_res);
			f2f_motion->ed_nodes_res_low = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_ed_nodes_res_low", f2f_motion->ed_nodes_res_low);
			f2f_motion->vxl_res_low = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_vxl_res_low", f2f_motion->vxl_res_low);
			f2f_motion->fusion_mu = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_fusion_mu", f2f_motion->fusion_mu);
			f2f_motion->iso_surface_level = conf->GetValueWithDefault<float>("Fusion", "f2f_motion_iso_surface_level", f2f_motion->iso_surface_level);
		}
		if (dual_gpu) {
			cuCtxPopCurrent(&ctxF2f);

			cuCtxPushCurrent(ctxFusion);
		}

	}

public:
	void Shutdown()
	{

	}
	double GetPreviousFrameFPS()
	{
		return previousFrameRate;
	}

	void LoadCameras()
	{
		for (int i = 0; i < cams_d.size(); i++)
			delete cams_d[i];
		cams_d.clear();

		for (int i = 0; i < cams_clr.size(); i++)
			delete cams_clr[i];
		cams_clr.clear();


		std::string calib_dir = DEFAULT_CALIBRATION_FOLDER;

		auto conf = FusionConfig::current();
		if (conf && conf->HasKey("Fusion", "CalibrationDirectory"))
		{
			calib_dir = conf->GetValue<std::string>("Fusion", "CalibrationDirectory");
		}
		else
		{
			*LOGGER() << Logger::Warning << "[Fusion][WARN] calib_dir is set to default folder in debug.h" << Logger::endl;
		}

		int depth_num = DEPTH_CAMERAS_NUM;
		int depth_width = DEFAULT_DEPTH_WIDTH;
		int depth_height = DEFAULT_DEPTH_HEIGHT;

		LoadCalibrationFromFile(conf, depth_num, depth_height, depth_width, calib_dir);

		fusion4d->setup_cameras(cams_d);
		if (dual_gpu) {
			cuCtxPopCurrent(&ctxFusion);
			cuCtxPushCurrent(ctxF2f);
		}
		f2f_motion->setup_cameras(cams_d);
		if (dual_gpu) {
			cuCtxPopCurrent(&ctxF2f);
			cuCtxPushCurrent(ctxFusion);
		}
	}

	void LoadCalibrationFromFile(CConfig* conf, int depth_num, int depth_height, int depth_width, std::string& calib_dir)
	{
		char name[8192];

		if (conf) {
			sprintf(name, "%s\\%s", calib_dir.c_str(), DEFAULT_CALIBRATION_FILENAME);

			LOGGER()->info("Loading calibration from 3DTM Format Json in %s", name);

			for (int i = 0; i < depth_num; i++)
			{
				GCameraView* cam_depth = new GCameraView(depth_height, depth_width);
				if (!cam_depth->LoadCalibrationFrom3DTMFormat(name, i, "depth", -1.0)) {
					LOGGER()->error("TemporalFusionThread::LoadCalibrationFromFile", "Unable to load calibration in 3DTM Format for \"depth\" camera %02d from file %s. Invalid GCameraView object will be used.", i, name);
				}

				cams_d.push_back(cam_depth);

				GCameraView* cam_color = new GCameraView();
				if (!cam_color->LoadCalibrationFrom3DTMFormat(name, i, "rgb", -1.0)) {
					LOGGER()->error("TemporalFusionThread::LoadCalibrationFromFile", "Unable to load calibration in 3DTM Format for \"rgb\" camera %02d from file %s. Invalid GCameraView object will be used.", i, name);
				}
				cams_clr.push_back(cam_color);
			}
		}
	}

	bool LoadVolumeBoundingBoxFromCalibration(CConfig* conf, const std::string& calib_dir, float min_coords[3], float max_coords[3]) {
		char calib_filename[8192];

		if (conf) {
			float min_coords_adjustment[3] = { 0.0f, 0.0f, 0.0f };
			float max_coords_adjustment[3] = { 0.0f, 0.0f, 0.0f };

			// Zeroing coords
			memset(min_coords, 0.0f, sizeof(float) * 3);
			memset(max_coords, 0.0f, sizeof(float) * 3);

			min_coords_adjustment[0] = conf->GetValueWithDefault("Fusion", "xmin_adjustment", 0.0f);
			min_coords_adjustment[1] = conf->GetValueWithDefault("Fusion", "ymin_adjustment", 0.0f);
			min_coords_adjustment[2] = conf->GetValueWithDefault("Fusion", "zmin_adjustment", 0.0f);

			max_coords_adjustment[0] = conf->GetValueWithDefault("Fusion", "xmax_adjustment", 0.0f);
			max_coords_adjustment[1] = conf->GetValueWithDefault("Fusion", "ymax_adjustment", 0.0f);
			max_coords_adjustment[2] = conf->GetValueWithDefault("Fusion", "zmax_adjustment", 0.0f);

			sprintf(calib_filename, "%s\\%s", calib_dir.c_str(), DEFAULT_CALIBRATION_FILENAME);

			std::ifstream calibIFS(calib_filename);
			if (!calibIFS.is_open())
			{
				LOGGER()->error("LoadVolumeBoundingBoxFromCalibration", "Can't open 3DTM Json calibration file %s", calib_filename);
				return false;
			}

			Json::Value root;

			// Trying to parse Json format
			try {
				calibIFS >> root;
			}
			catch (const std::exception& e) {
				LOGGER()->error("LoadVolumeBoundingBoxFromCalibration", "Invalid 3DTM Format ine %s", calib_filename);
				return false;
			}

			calibIFS.close();

			// Validating basic information about the file
			const char* bbox_keys[] = { "xmin", "xmax", "ymin", "ymax", "zmin", "zmax" };

			if (ValidateKey("volumeInformation", root) && ValidateKey("boundingBox", root["volumeInformation"]) &&
				ValidateParameterList(root["volumeInformation"]["boundingBox"], bbox_keys, 6)) {
				min_coords[0] = root["volumeInformation"]["boundingBox"]["xmin"].asFloat();
				min_coords[1] = root["volumeInformation"]["boundingBox"]["ymin"].asFloat();
				min_coords[2] = root["volumeInformation"]["boundingBox"]["zmin"].asFloat();
				max_coords[0] = root["volumeInformation"]["boundingBox"]["xmax"].asFloat();
				max_coords[1] = root["volumeInformation"]["boundingBox"]["ymax"].asFloat();
				max_coords[2] = root["volumeInformation"]["boundingBox"]["zmax"].asFloat();
			}
			else {
				LOGGER()->error("LoadVolumeBoundingBoxFromCalibration", "Unable to find volumeInformation and/or boundingBox values in calibration file %s", calib_filename);
				return false;
			}

			// Cropping the volume by user-selected values and converting to Fusion metric system
			for (int t = 0; t < 3; t++) {
				min_coords[t] = Convert3DTMMetricSystemToFusion(min_coords[t] + min_coords_adjustment[t]);
				max_coords[t] = Convert3DTMMetricSystemToFusion(max_coords[t] + max_coords_adjustment[t]);
			}

			return true;
		}
	}

	void operator()()
	{
		LOGGER()->info("[Fusion] Starting cuda computation thread!");
		frame_count = 0;

		int frames_in_sequence = DataCaptureFileMultiView::frmIdx_end - DataCaptureFileMultiView::frmIdx_1st;
		if (data_use_network_loader) {
			frames_in_sequence = INT_MAX;
			setup_pinned_memory();
			setup_blank_depth_buffer();
		}

		//setup thread should set m_mesh_server before calling
		if (run_mesh_server && m_mesh_server != nullptr) {
			setup_mesh_server_pinned_memory();
		}

		if (load_all_before_compute())
		{
			while (!DataCaptureFileMultiView::load_complete())
			{
			}

			if (!DataCaptureFileMultiView::depthImgs_queue.empty())
			{
				size_t num_frames = DataCaptureFileMultiView::depthImgs_queue.size();
				size_t num_depth_images = DataCaptureFileMultiView::depthImgs_queue[0].size();
				size_t mem_size_of_depth_image = DataCaptureFileMultiView::depthImgs_queue[0][0].rows * DataCaptureFileMultiView::depthImgs_queue[0][0].cols * sizeof(DataCaptureFileMultiView::depthImgs_queue[0][0].type());

				char* pinned_buffer;
				cudaMallocHost((void**)&pinned_buffer, num_frames * num_depth_images * mem_size_of_depth_image, cudaHostAllocPortable);
				char* current_free_pointer = pinned_buffer;
				for (auto&& depth_imgs : DataCaptureFileMultiView::depthImgs_queue)
				{
					for (auto&& depth_img : depth_imgs)
					{
						memcpy(current_free_pointer, depth_img.ptr<unsigned char>(), mem_size_of_depth_image);
						int rows = depth_img.rows;
						int cols = depth_img.cols;
						int depth = depth_img.depth();
						int channels = depth_img.channels();
						depth_img.release();
						depth_img = cv::Mat(rows, cols, CV_MAKETYPE(depth, channels), current_free_pointer);
						current_free_pointer += mem_size_of_depth_image;
					}
				}
			}
		}

		LoadCameras();

		__tic__();
		auto fusionStart = std::chrono::high_resolution_clock::now();
		auto fusionEnd = std::chrono::high_resolution_clock::now();

		cv::Mat randomDepth = cv::Mat(data_format::DEPTH_HEIGHT, data_format::DEPTH_WIDTH, CV_MAKETYPE(GetCVDepthMaskFromBits(data_format::DEPTH_BYTES_PER_PIXEL * CHAR_BIT), 1), FUSION_GENERIC_DEPTH_PIXEL_VALUE);
		cv::Mat randomColor = cv::Mat(data_format::COLOR_HEIGHT, data_format::COLOR_WIDTH, CV_MAKETYPE(GetCVDepthMaskFromBits((data_format::COLOR_BYTES_PER_PIXEL + data_format::COLOR_BYTES_PER_PIXEL % 2) * CHAR_BIT), 1)); // the image depth has to be an even number, so allocate extra if it's odd


		const int cFrameTimeQueueWindowSize = 15;
		float frameTimeCumulative = 0;
		std::queue<float> frameTimeQueue;

		auto conf = FusionConfig::current();

		bool writeDisk = conf->GetValueWithDefault<bool>("Fusion", "interceptFusionWriteDisk", false);

		if (!conf->HasKey("Fusion", "DepthOffsetAmount") || conf->TryParseValue<int>("Fusion", "DepthOffsetAmount")) {
			int DepthOffset = conf->GetValueWithDefault("Fusion", "DepthOffsetAmount", 0);
			GlobalDataStatic::DepthOffset.clear();

			LOGGER()->info("Parsing config option \"Fusion\"->\"DepthOffsetAmount\" as int");

			for (size_t i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				GlobalDataStatic::DepthOffset.push_back(DepthOffset);
			}
		}
		else {
			LOGGER()->info("Parsing config option \"Fusion\"->\"DepthOffsetAmount\" as int failed. Trying to parse as array");

			std::vector<int> DepthOffsets = conf->GetValueArray<int>("Fusion", "DepthOffsetAmount");
			GlobalDataStatic::DepthOffset.clear();

			if (DepthOffsets.size() < DEPTH_CAMERAS_NUM) {
				LOGGER()->fatal("TemporalFusionThread()", "The number of depth offsets in the config file (%lu) is less than the number of available cameras (%lu)! Please use a single offset for all cameras, or an array of values with the correct dimension.",
					DepthOffsets.size(), DEPTH_CAMERAS_NUM);
				exit(-1);
			}
			for (size_t i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				GlobalDataStatic::DepthOffset.push_back(DepthOffsets[i]);
			}
		}

		while (true)
		{

			std::vector<cv::Mat> depthImgs;
			std::vector<cv::Mat> colorImgs;
			std::vector<int> colorSizeToCopy;

			//audio related
			std::vector<char*> audioData;
			std::vector<int> audioSizeToCopy;

			TimingTable timing_table;
			char timing_table_prefix[64];
			sprintf_s(timing_table_prefix, 64, "FusionMainThread_Frame_%06d", frame_count);

			timing_table.SetPrefix(timing_table_prefix);

			fusionEnd = std::chrono::high_resolution_clock::now();

			timing_table.Start("fusion_full", "Fusion full computation");
			timing_table.Start("grab_frame", "Fusion grab frame main thread");

			bool preloadNonEmptyFrames = frame_count < 20;

			GrabNextFrame(conf, preloadNonEmptyFrames, randomDepth, randomColor, depthImgs, colorImgs, colorSizeToCopy, audioData, audioSizeToCopy);

			timing_table.End("grab_frame");
			timing_table.Start("fusion_pre_processing", "Fusion pre-processing");

			if (writeDisk)
			{
				char filename1[256];
				char fullFileName1[256];
				// temp File save disk
				for (int i = 0; i < depth_num; ++i)
				{
					//COLOR
					sprintf(filename1, "%s\\cam%02d", conf->GetValue<std::string>("Fusion", "interceptFusionWriteDirectory").c_str(), i);
					CreateDirectory(filename1, NULL);
					sprintf(filename1, "%s\\frames", filename1);
					CreateDirectory(filename1, NULL);
					sprintf(fullFileName1, "%s\\%04d.colorBin", filename1, frame_count);
					std::fstream file1;
					file1.open(fullFileName1, std::ios::out | std::ios::app | std::ios::binary);
					if (file1) // make sure file opened properly
					{
						try {
							file1.write(reinterpret_cast<char*>(colorImgs[i].data), colorSizeToCopy[i]);
							file1.close();
						}
						catch (std::ifstream::failure e)
						{
							LOGGER()->error("Failed writing file to %s", fullFileName1);
						}
					}

					//DEPTH
					sprintf(filename1, "%s\\cam%02d_depth", conf->GetValue<std::string>("Fusion", "interceptFusionWriteDirectory").c_str(), i);
					CreateDirectory(filename1, NULL);
					sprintf(filename1, "%s\\frames", filename1);
					CreateDirectory(filename1, NULL);;

					sprintf(fullFileName1, "%s\\%04d.bin", filename1, frame_count);
					file1.open(fullFileName1, std::ios::out | std::ios::app | std::ios::binary);
					if (file1) // make sure file opened properly
					{
						try
						{

							file1.write(reinterpret_cast<char*>(depthImgs[i].data), 1024 * 1024 * 2);
							file1.close();
						}
						catch (std::ifstream::failure e)
						{
							*LOGGER() << Logger::Info << "Failed writing file to " << fullFileName1 << Logger::endl;
						}
					}

				}
			}

			bool useDepthOffset = conf->GetValueWithDefault<bool>("Fusion", "UseDepthOffset", false) && !(data_use_network_loader && preloadNonEmptyFrames);
			if (useDepthOffset)
			{
				auto t0 = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
				for (int i = 0; i < depth_num; ++i)
				{
					if (GlobalDataStatic::DepthOffset[i] > 0)
						cv::add(depthImgs[i], GlobalDataStatic::DepthOffset[i], depthImgs[i]);
					else if (GlobalDataStatic::DepthOffset[i] < 0)
						cv::subtract(depthImgs[i], -1 * GlobalDataStatic::DepthOffset[i], depthImgs[i]);
				}
				auto t1 = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double> time = t1 - t0;
				LOGGER()->info("Fusion", "Depth Setting: %lfms", time.count() * 1000);
			}

			//check for early out after fetching data
			if (GlobalDataStatic::bStopFusion)
				continue;
			if (m_mesh_server && !m_mesh_server->is_next_frame_ready())
			{
				LOGGER()->warning("TemporalFusionThread()", "Skipping fusing frame due to mesh_server buffer not ready");
				continue;
			}

			fusionStart = std::chrono::high_resolution_clock::now();

			blankImageHeader = cv::Mat(data_format::DEPTH_HEIGHT, data_format::DEPTH_WIDTH, CV_16UC1, cv::Scalar(0x8000));
			std::vector<int> disabledCameras;
			if (conf->HasKey("Fusion", "DisabledCameraList"))
			{
				disabledCameras = conf->GetValueArray<int>("Fusion", "DisabledCameraList");
			}

			for (int i = 0; i < disabledCameras.size(); ++i)
			{
				int disablingIndex = disabledCameras[i] - 1; // user-side is 1-indexed.
				if (disablingIndex >= 0)
				{
					depthImgs[disablingIndex] = blankImageHeader;
				}
			}

			// Disabling interactively selected cameras 
			for (int i = 0; i < depth_num; ++i)
			{
				if (g_select_fusion_input_views[i] == 0)
				{
					depthImgs[i] = blankImageHeader;
				}
			}

			int frmIdx = frame_count;

			if (GlobalDataStatic::bNextFrameHighQuality)
			{
				GlobalDataStatic::HighQualityFrameNumber = frmIdx;
				LOGGER()->debug("HQ frame = %d", frmIdx);
			}

			timing_table.End("fusion_pre_processing");

			timing_table.Start("fusion_computation_and_data_transfer", "Data transfer and Fusion computation");
			timing_table.Start("fusion_color_transfer", "Color and audio data transfer to Renderer");

			concurrency::task<void> frame_color_copy_task;
			int mesh_server_frame_id = -1;
			if (m_mesh_server) {
				mesh_server_frame_id = m_mesh_server->start_next_frame(frame_count);

				//transfer color datam_mesh_server
				std::vector<char*> image_ptrs(colorImgs.size());
				std::transform(
					colorImgs.begin(),
					colorImgs.end(),
					image_ptrs.begin(), [](cv::Mat a) {return a.ptr<char>(); });

				auto ptr = tfutil::vec_data_ptr(image_ptrs);
				m_mesh_server->set_frame_color_data(mesh_server_frame_id, ptr, colorSizeToCopy, colorImgs.size());
				m_mesh_server->set_frame_audio_data(mesh_server_frame_id, audioData, audioSizeToCopy, audioData.size());
				auto srv = m_mesh_server;
				frame_color_copy_task = concurrency::create_task([srv, mesh_server_frame_id]()
					{
						srv->copy_frame_color_data(mesh_server_frame_id);
						srv->copy_frame_audio_data(mesh_server_frame_id);
					});
			}

			{
				lock_guard<std::mutex> lock_cuda_res(mutex_cuda);
				bInitBackgroundThisFrame = bInitbackgroundNextFrame;
				bInitbackgroundNextFrame = false;
			}



			LOGGER()->info("[cuda]: frame <%d, %d> loaded", frame_count, frmIdx);

			if (m_mesh_server) {
				frame_color_copy_task.wait(); //wait for the frame data to be copied before we exit this.
			}

			std::vector<cv::Mat> colorImgsForGPU = colorImgs;

			colorImgsForGPU = std::vector<cv::Mat>(); // TODO : does the cuda thread ever need this?

			timing_table.End("fusion_color_transfer");

			timing_table.Start("fusion_computation", "Fusion computation");

			char const* debug_dir_str = debug_dir.empty() ? nullptr : debug_dir.c_str();
			if (frame_count == 0) //only need to skip frames on the first setup 
			{
				in_the_first_few_frames = true;
				newestFrame = 0;
				oldestFrame = 0;

				mesh_server_gpuframe_id[newestFrame] = -1;
				fusion4d->set_up_1st_frame(depthImgs, bbox, debug_dir_fusion4d, frmIdx);
				if (dual_gpu) {
					cuCtxPopCurrent(&ctxFusion);
					cuCtxPushCurrent(ctxF2f);
				}

				f2f_motion->set_up_1st_frame(depthImgs, bbox, debug_dir_str, frmIdx);
				if (dual_gpu) {
					cuCtxPopCurrent(&ctxF2f);
					cuCtxPushCurrent(ctxFusion);
				}
				GlobalDataStatic::bResetKeyFrame = false;

				datacapture_front_id[newestFrame] = frame_count;

				timing_table.End("fusion_computation");
			}
			else
			{
				in_the_first_few_frames = (frame_count < initial_frames_num);
				newestFrame++;
				if (newestFrame >= MAXFRAMESINPROCESSING) newestFrame = 0;

				mesh_server_gpuframe_id[newestFrame] = mesh_server_frame_id;

				datacapture_front_id[newestFrame] = frame_count;

				if (!data_use_network_loader)
				{
					int offset = 0;
					int step = depth_width * depth_height;
					unsigned short* target = _depthOnCPUPinned[newestFrame];
					for (int i = 0; i < depth_num; i++)
					{
						memcpy(target + step * i, depthImgs[i].ptr<short>(), step * sizeof(short));
					}
				}
				// add gpu0-gpu1 explicit sync!

				if (dual_gpu) {
					cuCtxPopCurrent(&ctxFusion);
				}
#pragma omp parallel for num_threads(2), schedule(dynamic)
				for (int i = 0; i < 2; i++)
				{

					if (i == 0)
					{
						if (dual_gpu) {
							cuCtxPushCurrent(ctxF2f);
						}

						if (!data_use_network_loader)
							RunThreadGPUF2f(frmIdx, NULL);
						else
							RunThreadGPUF2f(frmIdx, &depthImgs);
						if (dual_gpu) {
							cuCtxPopCurrent(&ctxF2f);
						}
					}
					if (i == 1)
					{
						if (dual_gpu) {
							cuCtxPushCurrent(ctxFusion);
						}

						if (!data_use_network_loader)
							RunThreadGPUFusion(frmIdx, NULL);
						else
							RunThreadGPUFusion(frmIdx, &depthImgs);
						if (dual_gpu) {
							cuCtxPopCurrent(&ctxFusion);
						}
					}

				}
				if (dual_gpu) {
					cuCtxPushCurrent(ctxFusion);
				}

				timing_table.End("fusion_computation");
				timing_table.Start("mesh_transfer", "Mesh data transfer to Renderer");

				int prev_frame = (newestFrame + MAXFRAMESINPROCESSING - 1) % MAXFRAMESINPROCESSING;

				int activeFrames = newestFrame - oldestFrame;
				if (activeFrames < 0) activeFrames += MAXFRAMESINPROCESSING;
				if (activeFrames >= 5)
				{
					int mesh_server_copy_readyFrame = oldestFrame;
					oldestFrame++;
					if (oldestFrame >= MAXFRAMESINPROCESSING) oldestFrame = 0;

					//tell the rendering thread the new data is ready
					if (m_mesh_server)
					{
						if (mesh_server_gpuframe_id[oldestFrame] != -1)
						{
							auto gp_vertices = fusion4d->vol_fusion.get_mesh_gpu();
							auto gp_triangles = fusion4d->vol_fusion.dev_triangles();

							char* pinned_vert = 0;
							char* pinned_tri = 0;

							m_mesh_server->get_frame_data_ptrs(mesh_server_gpuframe_id[oldestFrame], &pinned_vert, &pinned_tri);

							int* pinned_vert_size_ptr = reinterpret_cast<int*>(pinned_vert);
							char* pinned_vert_data_ptr = pinned_vert + sizeof(int);
							int* pinned_ind_size_ptr = reinterpret_cast<int*>(pinned_tri);
							char* pinned_ind_data_ptr = pinned_tri + sizeof(int);


							bool copy_to_host_successful = false;
							//synchronous copy
							if (pinned_vert && pinned_tri)
							{
								*pinned_vert_size_ptr = 0;
								*pinned_ind_size_ptr = 0;

								cudaError_t res_vert_size_copy;
								cudaError_t res_ind_size_copy;
								cudaError_t res_vert_data_copy;
								cudaError_t res_ind_data_copy;
								bool bSendRightViewThroughMeshServer = GlobalDataStatic::bSendRightViewThroughMeshServer;
								if (bSendRightViewThroughMeshServer)
								{
									*pinned_ind_size_ptr = 0;
									res_vert_size_copy = cudaMemcpy(pinned_vert_size_ptr, fusion4d->vol_fusion.vts_cur_num_gpu().dev_ptr, sizeof(int), cudaMemcpyDeviceToHost);
									if (res_vert_size_copy == cudaSuccess)
									{
										res_vert_data_copy = fusion4d->vol_fusion.sync_copy_vts_to_cpu_buf_sync_as_half((short*)pinned_vert_data_ptr, *pinned_vert_size_ptr, false);
										if (res_vert_data_copy == cudaSuccess) {
											copy_to_host_successful = true;
										}
										else {
											*LOGGER() << Logger::Error << "failed to copy vertex data from device" << Logger::endl;
										}
									}
									else
									{
										*LOGGER() << Logger::Error << "failed to copy vertex data size from device" << Logger::endl;
									}
								}
								else
								{
									res_vert_size_copy = cudaMemcpy(pinned_vert_size_ptr, fusion4d->vol_fusion.vts_t_num_gpu().dev_ptr, sizeof(int), cudaMemcpyDeviceToHost);
									if (res_vert_size_copy == cudaSuccess)
									{
										res_vert_data_copy = cudaMemcpy(pinned_vert_data_ptr, gp_vertices, (*pinned_vert_size_ptr) * VERTEX_SIZE, cudaMemcpyDeviceToHost);
									}
									else
									{
										*LOGGER() << Logger::Error << "failed to copy vertex data size from device" << Logger::endl;
									}
									res_ind_size_copy = cudaMemcpy(pinned_ind_size_ptr, fusion4d->vol_fusion.tris_num_gpu().dev_ptr, sizeof(int), cudaMemcpyDeviceToHost);
									if ((res_ind_size_copy == cudaSuccess))
									{
										res_ind_data_copy = cudaMemcpy(pinned_ind_data_ptr, gp_triangles, (*pinned_ind_size_ptr) * sizeof(int) * 3, cudaMemcpyDeviceToHost);
									}
									else
									{
										*LOGGER() << Logger::Error << "failed to copy index data size from device" << Logger::endl;
									}
									if ((res_vert_data_copy == cudaSuccess) && (res_ind_data_copy == cudaSuccess))
									{
										copy_to_host_successful = true;
									}
									else
									{
										*LOGGER() << Logger::Error << "failed to copy vertex/index data from device" << Logger::endl;
									}
								}
							}

							if (!copy_to_host_successful) {
								//report an empty frame.  
								//TODO: we should add the option to skip this frame
								*LOGGER() << Logger::Error << "cuda memory transfer to host failed... reporting empty frame" << Logger::endl;
								*pinned_vert_size_ptr = 0;
								*pinned_ind_size_ptr = 0;
							}

							GlobalDataStatic::FrameStartEndOffset = (newestFrame - oldestFrame + MAXFRAMESINPROCESSING) % MAXFRAMESINPROCESSING;
							m_mesh_server->notify_frame_data_ptrs_ready(mesh_server_gpuframe_id[oldestFrame], pinned_vert, pinned_tri);
							m_mesh_server->dispatch_frame(datacapture_front_id[oldestFrame]);

						}
					}
				}

				timing_table.End("mesh_transfer");
			}

			if (renderingEnabled())
			{
				lock_guard<std::mutex> lock_cuda_res(mutex_cuda);
				bNewSurface = true;
			}

			if (!data_use_network_loader) {
				releaseCvMats(depthImgs);
				releaseCvMats(colorImgs);
			}

			frame_count++;

			timing_table.End("fusion_computation_and_data_transfer");
			timing_table.End("fusion_full");

			timing_table.LogTimes(Logger::Info);

			{


				float frame_time = __toc__();
				if (frameTimeQueue.size() >= cFrameTimeQueueWindowSize)
				{
					float oldest = frameTimeQueue.front();
					frameTimeCumulative -= oldest;

					frameTimeQueue.pop();

				}
				frameTimeCumulative += frame_time;
				frameTimeQueue.push(frame_time);

				float windowFrameTime = frameTimeCumulative / frameTimeQueue.size();
				previousFrameRate = 1000.0 / (double)windowFrameTime;

				__tic__();

				if (frame_count % 2 == 0) {

					char mesh_server_string[512];
					char windowTitle[512];

					if (m_mesh_server)
					{
						if (m_mesh_server && m_mesh_server->m_show_stats)
						{
							sprintf(mesh_server_string, "/ meshserver on frame %05d clients:%02d %4.1lf:fps %4.1lf:Gbps %3.2lf:kvpf  latency:%4.1lfms prep:%4.1lfms",
								m_mesh_server->get_current_frame(),
								m_mesh_server->get_current_connection_count(),
								m_mesh_server->m_fps,
								m_mesh_server->m_gbps,
								m_mesh_server->m_vtxps * 0.001,
								m_mesh_server->m_avg_latency_ms,
								m_mesh_server->m_avg_prep_ms);
						}
						else
						{
							sprintf(mesh_server_string, "/ meshserver on frame %05d clients:%02d",
								m_mesh_server->get_current_frame(),
								m_mesh_server->get_current_connection_count());
						}
					}

					sprintf(windowTitle, "Live Fusion Demo @ %5.1f ms, %5.1f FPS // cudaDeviceSync is %s // %s",
						windowFrameTime, 1000.f / windowFrameTime, renderingEnabled() ? "on" : "off",
						m_mesh_server ? mesh_server_string : "meshserver off");

					glutSetWindowTitle(windowTitle);
				}
			}
		} //end forever loop

		if (data_use_network_loader) {
			clear_pinned_memory();
			clear_blank_buffer();
		}
	}

	void GrabNextFrame(CConfig* conf, bool preloadNonEmptyFrames, cv::Mat& randomDepth, cv::Mat& randomColor, std::vector<cv::Mat>& depthImgs, std::vector<cv::Mat>& colorImgs, std::vector<int>& colorSizeToCopy, std::vector<char*>& audioData, std::vector<int>& audioSizeToCopy)
	{
		if (data_use_network_loader)
		{
			if (preloadNonEmptyFrames)
			{
				for (int i = 0; i < depth_num; ++i)
				{
					depthImgs.push_back(randomDepth);
					colorImgs.push_back(randomColor);
					colorSizeToCopy.push_back(COLOR_SIZE);
				}
			}
			else
			{
				if (!conf->GetValueWithDefault("Fusion", "ThreadedGrabFrame", false)) {
					ResyncFrameBuffersIfNecessary(conf);

					if (terminate_fusion) {
						LOGGER()->fatal("TemporalFusionThread()", "Fusion termination requested! Exiting...");
						exit(-1);
					}

					//dont actually grab until fusion is actually setup
					while (!m_DataCaptureNetworkView->grab_next_frame(depthImgs, colorImgs, colorSizeToCopy))
					{
						Sleep(2); // dont burn the CPU down
					}

					// grab audio too
					m_DataCaptureNetworkView->grab_next_audio_frame(audioData, audioSizeToCopy);

					frame_grab_count++;
				}
				else {

					if (m_thread_grab_frame == NULL) {
						m_thread_grab_frame = new std::thread(
							[=]() {
								while (keep_grabbing_frames) {
									std::vector<cv::Mat> depthImgs_local;
									std::vector<cv::Mat> colorImgs_local;
									std::vector<int> colorSizeToCopy_local;

									//audio related
									std::vector<char*> audioData_local;
									std::vector<int> audioSizeToCopy_local;

									ResyncFrameBuffersIfNecessary(conf);

									if (terminate_fusion)
										return;

									LOGGER()->debug("GrabNextFrame()->Thread", "Grab frame");

									//dont actually grab until fusion is actually setup
									while (!m_DataCaptureNetworkView->grab_next_frame(depthImgs_local, colorImgs_local, colorSizeToCopy_local))
									{
										Sleep(1); // dont burn the CPU down
									}

									LOGGER()->debug("GrabNextFrame()->Thread", "Frame grabbed!!");
									// grab audio too
									m_DataCaptureNetworkView->grab_next_audio_frame(audioData_local, audioSizeToCopy_local);

									// Copying latest acquired frame to main thread
									std::lock_guard guard(m_latest_frame_mutex);

									m_latest_frame.depthImgs.clear();
									m_latest_frame.colorImgs.clear();
									m_latest_frame.colorSizeToCopy.clear();

									for (size_t i = 0; i < depthImgs_local.size(); i++) {
										m_latest_frame.depthImgs.push_back(depthImgs_local[i]);
										m_latest_frame.colorImgs.push_back(colorImgs_local[i]);
										m_latest_frame.colorSizeToCopy.push_back(colorSizeToCopy_local[i]);
									}

									LOGGER()->debug("GrabNextFrame()->Thread", "Copied color and depth!!", depthImgs_local.size(), colorImgs_local.size(), audioData_local.size());

									m_latest_frame.audioData = audioData_local;

									m_latest_frame.audioSizeToCopy = audioSizeToCopy_local;

									LOGGER()->debug("GrabNextFrame()->Thread", "Frame copied. Updating frame_grabbing_started!!");

									frame_grab_count++;
								}
							}
						);
					}

					while (frame_grab_count == 0) {
						LOGGER()->debug("GrabNextFrame", "Waiting for frame grabbing to start");
						Sleep(2);
					}

					{
						LOGGER()->debug("GrabNextFrame", "[Fusion side] Frame grabbing started, waiting to copy latest frame");

						std::lock_guard guard(m_latest_frame_mutex);

						if (terminate_fusion) {
							LOGGER()->fatal("TemporalFusionThread()", "Fusion termination requested! Exiting...");
							exit(-1);
						}

						LOGGER()->debug("GrabNextFrame", "[Fusion side] Going to copy latest frames depth #%lu color #%lu audio #%lu", m_latest_frame.depthImgs.size(), m_latest_frame.colorImgs.size(), m_latest_frame.audioData.size());

						for (size_t i = 0; i < m_latest_frame.depthImgs.size(); i++) {
							depthImgs.push_back(m_latest_frame.depthImgs[i]);
							colorImgs.push_back(m_latest_frame.colorImgs[i]);
							colorSizeToCopy.push_back(m_latest_frame.colorSizeToCopy[i]);
						}

						LOGGER()->debug("GrabNextFrame", "[Fusion side] Copying grabbed framed depth #%lu color #%lu audio #%lu.", depthImgs.size(), colorImgs.size(), audioData.size());

						audioSizeToCopy = m_latest_frame.audioSizeToCopy;
						audioData = m_latest_frame.audioData;
					}
				}
			}
		}
		else
		{
			while (!DataCaptureFileMultiView::grab_next_frame(depthImgs, colorImgs, colorSizeToCopy))
			{
				Sleep(2); // dont burn the CPU down
			}
			LOGGER()->info("Done grabbing frames from file!");
		}
	}

	void ResyncFrameBuffersIfNecessary(CConfig* conf) {
		std::string action_if_out_of_sync = conf->GetValueWithDefault("Fusion", "ActionIfBuffersOutOfSync", "");
		long frame_count_reset = conf->GetValueWithDefault("Fusion", "ResyncBuffersEveryGivenNumberOfFrames", 0);
		bool buffers_out_of_sync = m_DataCaptureNetworkView->AreBuffersOutOfSync();
		size_t max_number_of_resync_attempts = conf->GetValueWithDefault("Fusion", "MaximumNumberOfResyncAttempts", 3);

		LOGGER()->warning("TemporalFusionThread::ResyncFrameBuffersIfNecessary", "buffers_out_of_sync %d, action_if_out_of_sync %s", buffers_out_of_sync, action_if_out_of_sync.c_str());

		if (action_if_out_of_sync == "terminate") {
			terminate_fusion = m_DataCaptureNetworkView->AreBuffersOutOfSync();
			LOGGER()->error("ResyncFrameBuffersIfNecessary", "Buffers out of sync!! Requesting Fusion to terminate");
		}
		else {
			bool resync = false;

			if ((action_if_out_of_sync == "try_resync" || action_if_out_of_sync == "try_resync_and_terminate") && buffers_out_of_sync) {
				if (action_if_out_of_sync == "try_resync_and_terminate" && resync_attempt >= max_number_of_resync_attempts) {
					LOGGER()->warning("ResyncFrameBuffersIfNecessary", "Maximum number of resync attempts (%lu) reached. Terminating Fusion. Current processed frame: %d. Current grabbed frame: %d", max_number_of_resync_attempts, frame_count, frame_grab_count);
					terminate_fusion = true;
				}
				else {
					LOGGER()->warning("ResyncFrameBuffersIfNecessary", "Buffers out of sync! Resetting buffers and forcing frame synchronization. Current processed frame: %d. Current grabbed frame: %d", frame_count, frame_grab_count);
					resync = true;
				}
				resync_attempt++;
			}
			else if (frame_count_reset > 0 && frame_grab_count > 0 && frame_grab_count % frame_count_reset == 0) {
				LOGGER()->warning("ResyncFrameBuffersIfNecessary", "Forcing frame synchronization every %d frames. Current processed frame: %d. Current grabbed frame: %d", frame_count_reset, frame_count, frame_grab_count);
				resync = true;
			}

			if (resync) {
				m_DataCaptureNetworkView->ResetPackets();
			}
		}
	}

public:
	long frame_count;
	long frame_grab_count;
	bool bInitBackgroundThisFrame;
	CudaGlobalMemory* cuda_mem_fusion;

	CMotionFieldEstimationF2F* f2f_motion;
	Fusion4DKeyFrames* fusion4d;

	std::vector<GCameraView*> cams_d;
	std::vector<GCameraView*> cams_clr;

	float ddf_res;
	float ddf_mu;
	float align_mu;
	float ed_nodes_min_dist;

	BoundingBox3D bbox;

	std::string debug_dir;
	char const* debug_dir_fusion4d;

	double previousFrameRate;
	// GPU state
	CUcontext ctxFusion;
	CUcontext ctxF2f;

	bool dual_gpu;

	//frame0
	// upload d0 to f2f and fus

	//frame1
	// upload d1 to f2f and fus
	// start processing on d0 on f2f

	//frame2
	// upload d2 to f2f and fus
	// download d0result from f2f
	// start processing on d1 on f2f

	//frame3
	// upload d3 to f2f and fus
	// download d1result from f2f
	// upload d0result to fus
	// start processing on d2 on f2f

	//frame4
	// upload d4 to f2f and fus
	// download d2result from f2f
	// upload d1result to fus
	// start processing on d3 on f2f
	// start processing on d0 on fus

	//frame5
	// upload d5 to f2f and fus
	// download d3result from f2f
	// upload d2result to fus
	// donload finald0 from fus
	// start processing on d4 on f2f
	// start processing on d1 on fus

	// dispatch final d0 result
	// 6 frames latency...

	int depthBufferSize;
	int EDdataBufferSize;
	int ResultdataBufferSize;
	int depth_num;
	int depth_width;
	int depth_height;

	int oldestFrame;
	int newestFrame;

	cudaArray* _gpuF2fDepth[MAXFRAMESINPROCESSING];
	char* _gpuF2fEdData[MAXFRAMESINPROCESSING];

	cudaArray* _gpuFusionDepth[MAXFRAMESINPROCESSING];
	char* _gpuFusionEdDataCoarse[MAXFRAMESINPROCESSING];

	int datacapture_front_id[MAXFRAMESINPROCESSING];

	char* _EDcoarsePinned[MAXFRAMESINPROCESSING];// on CPU-pinned
	unsigned short* _depthOnCPUPinned[MAXFRAMESINPROCESSING];// on CPU - pinned
	int mesh_server_gpuframe_id[MAXFRAMESINPROCESSING];

	cudaStream_t _sGPUF2fMemory;
	cudaStream_t _sGPUFusionMemory;


	cudaEvent_t _depthToGpuF2f[MAXFRAMESINPROCESSING];
	cudaEvent_t _computeFinishedGpuF2f[MAXFRAMESINPROCESSING];
	cudaEvent_t _coarseEDDownloadedGpuF2f[MAXFRAMESINPROCESSING];
	cudaEvent_t _depthToGpuFusion[MAXFRAMESINPROCESSING];
	cudaEvent_t _coarseEDUploadedGpuFusion[MAXFRAMESINPROCESSING];
	cudaEvent_t _computeFinishedGpuFusion[MAXFRAMESINPROCESSING];
	cudaEvent_t _resultDownloadedGpuFusion[MAXFRAMESINPROCESSING];

	void RunThreadGPUF2f(int frmIdx, std::vector<cv::Mat>* depthImgs)
	{
		int activeFrames = newestFrame - oldestFrame;
		if (activeFrames < 0) activeFrames += MAXFRAMESINPROCESSING;
		int depthFrame = newestFrame;
		int f2fprocessingFrame = newestFrame - 1;
		if (f2fprocessingFrame < 0) f2fprocessingFrame += MAXFRAMESINPROCESSING;
		int coarseEDresultFrame = newestFrame - 2;
		if (coarseEDresultFrame < 0) coarseEDresultFrame += MAXFRAMESINPROCESSING;

		char const* debug_dir_str = debug_dir.empty() ? nullptr : debug_dir.c_str();

		// download d3result from f2f
		if (activeFrames > 2)
		{
			checkCudaErrors(cudaStreamWaitEvent(_sGPUF2fMemory, _computeFinishedGpuF2f[coarseEDresultFrame], 0));
			checkCudaErrors(cudaMemcpyAsync(_EDcoarsePinned[coarseEDresultFrame], _gpuF2fEdData[coarseEDresultFrame], EDdataBufferSize, cudaMemcpyDeviceToHost, _sGPUF2fMemory));
			checkCudaErrors(cudaEventRecord(_coarseEDDownloadedGpuF2f[coarseEDresultFrame], _sGPUF2fMemory));
		}
		// start processing on d4 on f2f
		if (activeFrames > 1)
		{
			checkCudaErrors(cudaStreamWaitEvent(0, _depthToGpuF2f[f2fprocessingFrame], 0));
			f2f_motion->add_a_frame(_gpuF2fDepth[f2fprocessingFrame], _gpuF2fEdData[f2fprocessingFrame], EDdataBufferSize, debug_dir_str, frmIdx, false);
			checkCudaErrors(cudaEventRecord(_computeFinishedGpuF2f[f2fprocessingFrame], 0));
			cudaStreamSynchronize(_sGPUF2fMemory);
		}
		// upload d5 to f2f 
		{
			if (depthImgs)
				f2f_motion->vol_fusion.feed_depth_texturesToArray(*depthImgs, _gpuF2fDepth[depthFrame], &_sGPUF2fMemory);
			else
				f2f_motion->vol_fusion.feed_depth_texturesToArray(_depthOnCPUPinned[depthFrame], sizeof(short) * depth_num * depth_width * depth_height, _gpuF2fDepth[depthFrame], &_sGPUF2fMemory);

			checkCudaErrors(cudaEventRecord(_depthToGpuF2f[depthFrame], _sGPUF2fMemory));
		}
	};


	void RunThreadGPUFusion(int frmIdx, std::vector<cv::Mat>* depthImgs)
	{

		// upload d5 to fus
		// upload d2result to fus
		// donload finald0 from fus
		// start processing on d1 on fus
		int activeFrames = newestFrame - oldestFrame;
		if (activeFrames < 0) activeFrames += MAXFRAMESINPROCESSING;
		int depthFrame = newestFrame;
		int fusprocessingFrame = newestFrame - 4;
		if (fusprocessingFrame < 0) fusprocessingFrame += MAXFRAMESINPROCESSING;
		int fusDownloadFrame = newestFrame - 5;
		if (fusDownloadFrame < 0) fusDownloadFrame += MAXFRAMESINPROCESSING;
		int coarseEDUploadFrame = newestFrame - 3;
		if (coarseEDUploadFrame < 0) coarseEDUploadFrame += MAXFRAMESINPROCESSING;

		char const* debug_dir_str = debug_dir.empty() ? nullptr : debug_dir.c_str();

		// start processing on d1 on fus
		if (activeFrames > 4)
		{
			cudaStreamSynchronize(_sGPUFusionMemory);
			fusion4d->add_a_frame(_gpuFusionDepth[fusprocessingFrame], _gpuFusionEdDataCoarse[fusprocessingFrame], f2f_motion->ed_cubes_res(), //ed_nodes_init, 
				debug_dir_fusion4d, frmIdx, bInitBackgroundThisFrame);
			checkCudaErrors(cudaEventRecord(_computeFinishedGpuFusion[fusprocessingFrame], 0));
			// 
			// schedule async copy for meshserver here
			// 
		}
		// upload d2result to fus
		if (activeFrames > 3)
		{
			checkCudaErrors(cudaMemcpyAsync(_gpuFusionEdDataCoarse[coarseEDUploadFrame], _EDcoarsePinned[coarseEDUploadFrame], EDdataBufferSize, cudaMemcpyHostToDevice, _sGPUFusionMemory));
			checkCudaErrors(cudaEventRecord(_coarseEDUploadedGpuFusion[coarseEDUploadFrame], _sGPUFusionMemory));
		}
		// upload d5 to fus
		{
			if (depthImgs)
				fusion4d->vol_fusion.feed_depth_texturesToArray(*depthImgs, _gpuFusionDepth[depthFrame], &_sGPUFusionMemory);
			else
				fusion4d->vol_fusion.feed_depth_texturesToArray(_depthOnCPUPinned[depthFrame], sizeof(short) * depth_num * depth_width * depth_height, _gpuFusionDepth[depthFrame], &_sGPUFusionMemory);

			checkCudaErrors(cudaEventRecord(_depthToGpuFusion[depthFrame], _sGPUFusionMemory));
		}
	}


	void InitGPUF2f()
	{
		cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			checkCudaErrors(cudaMalloc3DArray(&_gpuF2fDepth[i], &channelDesc_tex, make_cudaExtent(depth_width, depth_height, depth_num),
				cudaArraySurfaceLoadStore | cudaArrayLayered));
			cudaMalloc((void**)&_gpuF2fEdData[i], EDdataBufferSize);

			cudaHostAlloc((void**)&_EDcoarsePinned[i], EDdataBufferSize, cudaHostAllocPortable);
			cudaHostAlloc((void**)&_depthOnCPUPinned[i], depthBufferSize, cudaHostAllocPortable);

			cudaEventCreateWithFlags(&_depthToGpuF2f[i], cudaEventDisableTiming);
			cudaEventCreateWithFlags(&_computeFinishedGpuF2f[i], cudaEventDisableTiming);
			cudaEventCreateWithFlags(&_coarseEDDownloadedGpuF2f[i], cudaEventDisableTiming);
		}

		//start by uploading depth frame 0 to GPU0 - we don't process it in any form - only used to process frame 1
		cudaStreamCreateWithFlags(&_sGPUF2fMemory, cudaStreamNonBlocking);
	}

	void InitGPUFusion()
	{
		cudaChannelFormatDesc channelDesc_tex = cudaCreateChannelDesc(16, 0, 0, 0, cudaChannelFormatKindUnsigned);
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			checkCudaErrors(cudaMalloc3DArray(&_gpuFusionDepth[i], &channelDesc_tex, make_cudaExtent(depth_width, depth_height, depth_num),
				cudaArraySurfaceLoadStore | cudaArrayLayered));
			cudaMalloc((void**)&_gpuFusionEdDataCoarse[i], EDdataBufferSize);

			cudaEventCreateWithFlags(&_depthToGpuFusion[i], cudaEventDisableTiming);
			cudaEventCreateWithFlags(&_coarseEDUploadedGpuFusion[i], cudaEventDisableTiming);
			cudaEventCreateWithFlags(&_computeFinishedGpuFusion[i], cudaEventDisableTiming);
			cudaEventCreateWithFlags(&_resultDownloadedGpuFusion[i], cudaEventDisableTiming);
		}

		//start by uploading depth frame 0 to GPU0 - we don't process it in any form - only used to process frame 1
		cudaStreamCreateWithFlags(&_sGPUFusionMemory, cudaStreamNonBlocking);
	}


	size_t blankDepthBufferSize = 0;
	cv::Mat blankImageHeader = cv::Mat();

	void setup_blank_depth_buffer()
	{
		blankImageHeader = cv::Mat(data_format::DEPTH_HEIGHT, data_format::DEPTH_WIDTH, CV_16UC1, cv::Scalar(0x8000));
	}

	std::vector<char*> pinnedHostBuffers;
	std::vector<size_t> pinnedHostSizes;

	void setup_pinned_memory() {

		if (!data_use_network_loader) return;

		auto bufferSize = m_DataCaptureNetworkView->get_buffer_required_size();

		auto bufferCount = m_DataCaptureNetworkView->get_buffer_required_count();
		auto bufferCountFront = m_DataCaptureNetworkView->get_front_buffer_required_count();
		auto bufferCountTotal = bufferCount + bufferCountFront;

		if (bufferSize == 0 || bufferCountTotal == 0) return;

		pinnedHostBuffers.resize(bufferCountTotal);
		pinnedHostSizes = std::vector<size_t>(bufferCountTotal, bufferSize);

		for (auto& b : pinnedHostBuffers) {
			checkCudaErrors(cudaHostAlloc(&b, bufferSize, cudaHostAllocPortable));
		}

		if (bufferCount > 0)
		{
			m_DataCaptureNetworkView->set_buffer_storage(bufferCount, &pinnedHostBuffers.front(), &pinnedHostSizes.front());
		}

		if (bufferCountFront > 0)
		{
			m_DataCaptureNetworkView->set_front_buffer_storage(bufferCountFront, &pinnedHostBuffers.front() + bufferCount, &pinnedHostSizes.front() + bufferCount);
		}

	}

	void clear_pinned_memory() {
		if (!pinnedHostBuffers.empty()) {
			for (auto& b : pinnedHostBuffers) {
				cudaFreeHost(b);
			}
			pinnedHostBuffers.clear();
		}
	}

	void clear_blank_buffer()
	{
		if (!blankImageHeader.empty())
		{
			blankImageHeader.release();
		}
	}



public:
	void set_mesh_server(TofinoMeshServer* server) {
		m_mesh_server = server;
	}

	void set_network_capture_view(DataCaptureNetworkMultiView* view)
	{
		m_DataCaptureNetworkView = view;
	}

protected:
	DataCaptureNetworkMultiView* m_DataCaptureNetworkView;

	TofinoMeshServer* m_mesh_server = nullptr;
	std::vector<char*>  mesh_server_pinned_vtx_buffers;
	std::vector<char*>  mesh_server_pinned_idx_buffers;

	std::thread* m_thread_grab_frame;
	std::mutex m_latest_frame_mutex;
	FrameData m_latest_frame;
	bool keep_grabbing_frames = true;
	bool terminate_fusion = false;
	size_t resync_attempt = 0;

	void setup_mesh_server_pinned_memory()
	{
		if (!m_mesh_server) return;

		int ring_size_needed = m_mesh_server->get_pinned_buffer_count_needed();

		if (ring_size_needed == 0) return;

		mesh_server_pinned_vtx_buffers.resize(ring_size_needed);
		mesh_server_pinned_idx_buffers.resize(ring_size_needed);

		const int max_vertex_count = fusion4d->vol_fusion.vts_t_num_gpu().max_size + 4;
		const int max_index_count = fusion4d->vol_fusion.tris_num_gpu().max_size + 4;
		int max_vertex_size_needed = max_vertex_count * VERTEX_SIZE;
		int max_index_size_needed = max_index_count * 3 * sizeof(int);

		for (int i = 0; i < ring_size_needed; ++i) {
			cudaMallocHost(&mesh_server_pinned_vtx_buffers[i], max_vertex_size_needed);
			cudaMallocHost(&mesh_server_pinned_idx_buffers[i], max_index_size_needed);
		}

		m_mesh_server->set_pinned_buffer_pointers(
			&mesh_server_pinned_vtx_buffers.front(),
			&mesh_server_pinned_idx_buffers.front());
	}


};
#endif