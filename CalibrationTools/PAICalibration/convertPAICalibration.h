// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#ifndef CONVERT_PAI_CALIBRATION_H
#define CONVERT_PAI_CALIBRATION_H

#include <json/json.h>
#include <config.h>

#ifndef ValidateKey
#define ValidateKey(keyStr, value) ((value).isMember((keyStr)))
#endif

bool ValidateParameterList(const Json::Value& json, const char* keys[], size_t nkeys);

double MillimetersToCentimeters(double v);
double MillimetersToFusionMetricSystem(double v);

double MetersToCentimeters(double v);
double MetersToMillimeters(double v);
double MetersTo3DTMMetricSystem(double v);

bool Validate3DTMFormatForCamera(const Json::Value& camera);

bool CopyIntrinsics(const Json::Value& json_input_camera, Json::Value& json_converted_calib_camera);

bool CopyExtrinsics(const Json::Value& json_input_camera, Json::Value& json_converted_calib_camera);

bool ConvertPAICalibFormatTo3DTM(const Json::Value& json_input_cameras, int output_depth_width, int output_depth_height, Json::Value& json_converted_calib);

void Write3DTMCalibFormat(const std::string& calib_dir, const Json::Value& json_converted_calib);

bool LoadPAICalibFormat(const std::string& calib_dir, int depth_num, Json::Value& json_input_cameras);

void SetupLogs(const CConfig& config);

// Optinally receives the calibration parameters used when invoking the PAI algorithm
int ConvertPAICalibration(const CConfig& config, const Json::Value& calib_params, const Json::Value& metrics);

#endif