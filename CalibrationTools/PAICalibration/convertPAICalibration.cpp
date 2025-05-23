// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
// C++17 defines std::byte and it causes numerous compile errors for ambiguous byte types without this flag
#define _HAS_STD_BYTE 0  
#include <filesystem>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>
#include "3dtm_version.h"  //built at runtime by Peabody.x64.buildversion.props build event
#include "ConvertPAICalibration.h"
#include "Logger.h"


bool ValidateParameterList(const Json::Value& json, const char* keys[], size_t nkeys) {

	for (size_t i = 0; i < nkeys; i++) {
		if (!ValidateKey(keys[i], json))
			return false;
	}

	return true;
}

double MillimetersToCentimeters(double v) {
	return v / 10.0f;
}

double MillimetersToFusionMetricSystem(double v) {
	return MillimetersToCentimeters(v);
}


double MetersToCentimeters(double v) {
	return v * 100.0f;
}

double MetersToMillimeters(double v) {
	return v * 1000.0f;
}

double MetersTo3DTMMetricSystem(double v) {
	return MetersToMillimeters(v);
}

bool Validate3DTMFormatForCamera(const Json::Value& camera) {
	const char* intrinsics_req[] = { "resolution", "codx", "cody", "fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2" };
	const char* extrinsics_req[] = { "rotation", "translation" };

	bool valid = true;

	valid = valid && ValidateParameterList(camera["intrinsics"], intrinsics_req, 12);
	valid = valid && ValidateParameterList(camera["extrinsics"], extrinsics_req, 2);

	if (valid) {
		valid = valid && camera["extrinsics"]["rotation"].size() == 3 && camera["extrinsics"]["rotation"][0].size() == 3 && camera["extrinsics"]["rotation"][1].size() == 3 && camera["extrinsics"]["rotation"][2].size() == 3;
		valid = valid && camera["extrinsics"]["translation"].size() == 3;
	}

	return valid;
}


bool CopyIntrinsics(const Json::Value& json_input_camera, Json::Value& json_converted_calib_camera)
{
	const size_t nkeys = 7;
	const char* intrinsics_names[nkeys] = { "width", "height", "principalPoint", "focalLength", "radialParams", "distortionCenter", "tangentialParams" };

	if (!ValidateParameterList(json_input_camera["intrinsics"], intrinsics_names, nkeys)) {
		return false;
	}

	// Reading the resolution values here, but they'll be overridden by the values obtained from the config file since the 
	// image is undistorted by the PODs before sending them to Fusion
	json_converted_calib_camera["intrinsics"]["resolution"]["width"] = json_input_camera["intrinsics"]["width"];
	json_converted_calib_camera["intrinsics"]["resolution"]["height"] = json_input_camera["intrinsics"]["height"];
	json_converted_calib_camera["intrinsics"]["cx"] = json_input_camera["intrinsics"]["principalPoint"][0];
	json_converted_calib_camera["intrinsics"]["cy"] = json_input_camera["intrinsics"]["principalPoint"][1];
	json_converted_calib_camera["intrinsics"]["fx"] = json_input_camera["intrinsics"]["focalLength"][0];
	json_converted_calib_camera["intrinsics"]["fy"] = json_input_camera["intrinsics"]["focalLength"][1];
	json_converted_calib_camera["intrinsics"]["k1"] = json_input_camera["intrinsics"]["radialParams"][0];
	json_converted_calib_camera["intrinsics"]["k2"] = json_input_camera["intrinsics"]["radialParams"][1];
	json_converted_calib_camera["intrinsics"]["k3"] = json_input_camera["intrinsics"]["radialParams"][2];
	json_converted_calib_camera["intrinsics"]["k4"] = json_input_camera["intrinsics"]["radialParams"][3];
	json_converted_calib_camera["intrinsics"]["k5"] = json_input_camera["intrinsics"]["radialParams"][4];
	json_converted_calib_camera["intrinsics"]["k6"] = json_input_camera["intrinsics"]["radialParams"][5];
	json_converted_calib_camera["intrinsics"]["codx"] = json_input_camera["intrinsics"]["distortionCenter"][0];
	json_converted_calib_camera["intrinsics"]["cody"] = json_input_camera["intrinsics"]["distortionCenter"][1];
	json_converted_calib_camera["intrinsics"]["p2"] = json_input_camera["intrinsics"]["tangentialParams"][0]; // Tangential distortion coefficients [tx, ty],
	json_converted_calib_camera["intrinsics"]["p1"] = json_input_camera["intrinsics"]["tangentialParams"][1]; // equivalent to OpenCV's [p2, p1] in that order

	// Copying depth scale and offset if cam_type == "depth"
	if (ValidateKey("scale", json_input_camera)) {
		json_converted_calib_camera["scale"] = json_input_camera["scale"];
	}

	if (ValidateKey("offset", json_input_camera)) {
		json_converted_calib_camera["offset"] = MetersTo3DTMMetricSystem(json_input_camera["offset"].asFloat());
	}

	return true;
}

bool CopyExtrinsics(const Json::Value& json_input_camera, Json::Value& json_converted_calib_camera)
{
	if (!ValidateKey("parentToView", json_input_camera)) {
		return false;
	}

	int parentToViewNcols = 4;
	// Extracting rotation matrix from 3x4 parentToView matrix
	json_converted_calib_camera["extrinsics"]["rotation"] = Json::arrayValue;
	for (int y = 0; y < 3; y++) {
		json_converted_calib_camera["extrinsics"]["rotation"][y] = Json::arrayValue;
		for (int x = 0; x < 3; x++) {
			float r = json_input_camera["parentToView"][y][x].asFloat();
			json_converted_calib_camera["extrinsics"]["rotation"][y].append(r);
		}
	}

	// Extracting XYZ translation vector from 3x4 parentToView matrix (XYZ translation is 4th column)
	int translationCol = 3;
	for (int y = 0; y < 3; y++) {
		float t = json_input_camera["parentToView"][y][translationCol].asFloat();
		// Converting to centimeters, since Fusion uses it instead of meters or millimeters
		json_converted_calib_camera["extrinsics"]["translation"].append(MetersTo3DTMMetricSystem(t));
	}

	return true;
}

void CreateEmptyColorCalibrationInfo(Json::Value& rgb_camera) {
	rgb_camera["colorCalibration"]["colorScale"] = Json::arrayValue;
	rgb_camera["colorCalibration"]["colorBias"] = Json::arrayValue;

	rgb_camera["colorCalibration"]["colorScale"].append(1.0f);
	rgb_camera["colorCalibration"]["colorScale"].append(1.0f);
	rgb_camera["colorCalibration"]["colorScale"].append(1.0f);

	rgb_camera["colorCalibration"]["colorBias"].append(0.0f);
	rgb_camera["colorCalibration"]["colorBias"].append(0.0f);
	rgb_camera["colorCalibration"]["colorBias"].append(0.0f);

}

bool AddInformationFromMKV(const char* mkv_path, Json::Value& json_camera) {
	k4a_result_t result;
	k4a_playback_t playback_handle;
	// Print the serial number of the device used to record
	size_t serial_number_size = 256;
	char serial_number[256];
	bool success = false;

	result = k4a_playback_open(mkv_path, &playback_handle);

	if (result == K4A_RESULT_FAILED) {
		LOGGER()->fatal("AddInformationFromMKV", "Unable to open file %s", mkv_path);
		return success;
	}

	k4a_buffer_result_t buffer_result = k4a_playback_get_tag(playback_handle, "K4A_DEVICE_SERIAL_NUMBER", serial_number, &serial_number_size);

	k4a_playback_close(playback_handle);

	if (buffer_result == K4A_BUFFER_RESULT_SUCCEEDED)
	{
		LOGGER()->info("AddInformationFromMKVs: Device serial number: %s. File: %s", serial_number, mkv_path);
		success = true;
	}
	else if (buffer_result == K4A_BUFFER_RESULT_TOO_SMALL)
	{
		LOGGER()->error("AddInformationFromMKVs", "Device serial number too long.");
	}
	else
	{
		LOGGER()->error("AddInformationFromMKVs", "Tag does not exist. Device serial number was not recorded.");
	}

	if (success) {
		json_camera["device"]["serial"] = serial_number;
	}

	return success;
}

bool AddVolumeInformationTo3DTM(Json::Value& json_converted_calib) {
	std::string camera_names[2];
	float min_coords[3] = { 1000000000.0f, 1000000000.0f,1000000000.0f };
	float max_coords[3] = { -1000000000.0f, -1000000000.0f, -1000000000.0f };
	cv::Mat worldToView = cv::Mat::eye(4, 4, CV_32F);
	cv::Mat worldToViewInv;

	camera_names[0] = "rgb";
	camera_names[1] = "depth";

	// Copying common parameters for RGB and Depth cameras
	for (int cam_type_idx = 0; cam_type_idx < 2; cam_type_idx++) {
		const char* cam_type = camera_names[cam_type_idx].c_str();

		for (int cam_idx = 0; cam_idx < (int)json_converted_calib["inputCameras"].size(); cam_idx++)
		{
			// Creating world to view matrix from camera [ rotation	|	translation	]
			//											 [	0		|	1			]
			for (int y = 0; y < worldToView.rows - 1; y++) {
				for (int x = 0; x < worldToView.cols - 1; x++) {
					worldToView.at<float>(y, x) = json_converted_calib["inputCameras"][cam_idx][cam_type]["extrinsics"]["rotation"][y][x].asFloat();
				}
			}

			for (int t = 0; t < 3; t++) {
				float coord = json_converted_calib["inputCameras"][cam_idx][cam_type]["extrinsics"]["translation"][t].asFloat();
				worldToView.at<float>(t, 3) = coord;
			}

			// Extracting translation from the inverted world to view matrix, which refers to the position of the camera
			// in world space
			worldToViewInv = worldToView.inv();

			for (int t = 0; t < 3; t++) {
				float coord = worldToViewInv.at<float>(t, 3);
				min_coords[t] = min(min_coords[t], coord);
				max_coords[t] = max(max_coords[t], coord);
			}

		}
	}

	json_converted_calib["volumeInformation"]["boundingBox"]["xmin"] = min_coords[0];
	json_converted_calib["volumeInformation"]["boundingBox"]["ymin"] = min_coords[1];
	json_converted_calib["volumeInformation"]["boundingBox"]["zmin"] = min_coords[2];
	json_converted_calib["volumeInformation"]["boundingBox"]["xmax"] = max_coords[0];
	json_converted_calib["volumeInformation"]["boundingBox"]["ymax"] = max_coords[1];
	json_converted_calib["volumeInformation"]["boundingBox"]["zmax"] = max_coords[2];

	return true;
}

bool AddPAICalibrationSessionInfoTo3DTM(const Json::Value calib_params, const Json::Value metrics, int depth_num, Json::Value& json_converted_calib)
{
	std::string mkvsDir = calib_params["mkvsDir"].asString();
	bool success = true;

	json_converted_calib["calibrationSession"]["algorithm"] = "PAI";
	json_converted_calib["calibrationSession"]["date"] = Logger::current_data_time();

	for (auto p = calib_params.begin(); p != calib_params.end(); p++) {
		json_converted_calib["calibrationSession"]["parameters"][p.key().asString()] = (*p);
	}

	json_converted_calib["calibrationSession"]["inputData"]["videos"] = Json::arrayValue;
	json_converted_calib["calibrationSession"]["metrics"] = metrics;

	for (int i = 0; i < depth_num && success; i++) {
		const size_t size = 32;
		char video_name[size];
		std::string video_path;

		sprintf_s(video_name, size, "output-%d.mkv", i);
		video_path = mkvsDir + "\\" + video_name;

		if (AddInformationFromMKV(video_path.c_str(), json_converted_calib["inputCameras"][i])) {
			json_converted_calib["calibrationSession"]["inputData"]["videos"].append(video_path);
		}
		else
		{
			LOGGER()->fatal("AddPAICalibrationSessionInfoTo3DTM", "Unable to load information for %s", video_path.c_str());
			success = false;
		}
	}

	return success;
}


int ConvertPAICalibration(const CConfig& config, const Json::Value& calib_params, const Json::Value& metrics) {
	int depth_num = 10;
	int depth_height = -1;
	int depth_width = -1;
	std::string calib_dir;

	calib_dir = config.GetValue<std::string>("Calibration", "CalibrationWorkingDirectory");
	depth_num = config.GetValueWithDefault("DepthGeneration", "DepthCameraCount", depth_num);
	depth_height = config.GetValueWithDefault("DepthGeneration", "DepthImageHeight", depth_height);
	depth_width = config.GetValueWithDefault("DepthGeneration", "DepthImageWidth", depth_width);

	Json::Value json_input_cameras;
	Json::Value json_converted_calib;

	// Reading PAI Json calibration format
	if (!LoadPAICalibFormat(calib_dir, depth_num, json_input_cameras))
		return -1;

	if (!ConvertPAICalibFormatTo3DTM(json_input_cameras, depth_width, depth_height, json_converted_calib))
		return -1;

	if (!AddPAICalibrationSessionInfoTo3DTM(calib_params, metrics, depth_num, json_converted_calib))
		return -1;

	if (!AddVolumeInformationTo3DTM(json_converted_calib))
		return -1;

	// TODO: adjust world coordinates to align with floor. ADO feature #9669

	Write3DTMCalibFormat(calib_dir, json_converted_calib);

	return 0;
}

bool ConvertPAICalibFormatTo3DTM(const Json::Value& json_input_cameras, int output_depth_width, int output_depth_height, Json::Value& json_converted_calib)
{
	json_converted_calib["inputCameras"] = Json::arrayValue;

	std::string camera_names[2];
	camera_names[0] = "rgb";
	camera_names[1] = "depth";

	LOGGER()->info("Converting PAI format to 3DTM. Number of cameras: %lu", json_input_cameras["inputCameras"].size());

	for (int cam_idx = 0; cam_idx < (int)json_input_cameras["inputCameras"].size(); cam_idx++)
	{
		// TODO: add the device serial number from the input video files or stream
		json_converted_calib["inputCameras"][cam_idx]["device"]["serial"] = "";

		// Copying common parameters for RGB and Depth cameras
		for (int cam_type_idx = 0; cam_type_idx < 2; cam_type_idx++) {
			const char* cam_type = camera_names[cam_type_idx].c_str();

			if (!CopyIntrinsics(json_input_cameras["inputCameras"][cam_idx][cam_type], json_converted_calib["inputCameras"][cam_idx][cam_type])) {
				LOGGER()->error("CopyIntrinsics", "Failed to identify intrinsics parameters in Json for camera %d, type %s", cam_idx, cam_type);
				return false;
			}

			if (!CopyExtrinsics(json_input_cameras["inputCameras"][cam_idx][cam_type], json_converted_calib["inputCameras"][cam_idx][cam_type])) {
				LOGGER()->error("CopyExtrinsics", "Failed to identify extrinsics parameters in Json for camera %d, type %s", cam_idx, cam_type);
				return false;
			}
		}

		// Overriding depth width and height with the values passed by the config file and updating the principal point accordingly
		if (output_depth_width > 0 && output_depth_height > 0) {
			Json::Value& json_converted_calib_camera_depth = json_converted_calib["inputCameras"][cam_idx]["depth"];
			int depth_width_unpadded, depth_height_unpadded;
			depth_width_unpadded = json_converted_calib_camera_depth["intrinsics"]["resolution"]["width"].asInt();
			depth_height_unpadded = json_converted_calib_camera_depth["intrinsics"]["resolution"]["height"].asInt();

			json_converted_calib_camera_depth["intrinsics"]["resolution"]["width"] = output_depth_width;
			json_converted_calib_camera_depth["intrinsics"]["resolution"]["height"] = output_depth_height;

			float cx = json_converted_calib_camera_depth["intrinsics"]["cx"].asFloat() + (output_depth_width - depth_width_unpadded) / 2;
			float cy = json_converted_calib_camera_depth["intrinsics"]["cy"].asFloat() + (output_depth_height - depth_height_unpadded) / 2;

			json_converted_calib_camera_depth["intrinsics"]["cx"] = cx;
			json_converted_calib_camera_depth["intrinsics"]["cy"] = cy;
		}

		// Creating a field with default/empty color calibration information
		CreateEmptyColorCalibrationInfo(json_converted_calib["inputCameras"][cam_idx]["rgb"]);
	}

	return true;
}

void Write3DTMCalibFormat(const std::string& calib_dir, const Json::Value& json_converted_calib)
{
	std::string json_converted_calib_filename = calib_dir + "\\calibCameras3DTM.json";

	LOGGER()->info("Writing converted Json format to file %s", json_converted_calib_filename.c_str());

	Json::StreamWriterBuilder wbuilder;
	std::string document = Json::writeString(wbuilder, json_converted_calib);
	std::ofstream ofs(json_converted_calib_filename, std::ios::trunc);
	ofs << std::fixed;
	ofs.precision(6);
	ofs << document << std::endl;
}

bool LoadPAICalibFormat(const std::string& calib_dir, int depth_num, Json::Value& json_input_cameras)
{
	std::string input_cameras_filename = calib_dir + "\\inputCameras.json";

	LOGGER()->info("Processing %s\n", calib_dir.c_str());

	std::ifstream input_cameras_file(input_cameras_filename);
	if (!input_cameras_file.is_open())
	{
		LOGGER()->error("ConvertPAICalibration", "Can't open K4A calibration file %s\n", input_cameras_filename.c_str());
		return false;
	}

	input_cameras_file >> json_input_cameras;

	if (depth_num != (int)json_input_cameras["inputCameras"].size()) {
		LOGGER()->error("ConvertPAICalibration", "Number of cameras in config file (%d) differs from the number of input cameras in the PAI calibration file %lu", depth_num, json_input_cameras["inputCameras"].size());
		return false;
	}

	return true;
}
