#ifndef PEABODY_H
#define PEABODY_H

/// Project Peabody 
///   - Header file used across Peabody Solutions to consolidate global variables
///   - TODO: replace #defines with const's 
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

///////////////////////////////////////
/// Global Defines
///////////////////////////////////////

// FOR ENCODER
const int TRANSMITTED_RAW_DEPTH_BYTES_PER_PIXEL = 2;

// FOR FUSION, still used in CUDA files
extern int DEPTH_CAMERAS_NUM;
const int MAX_NUM_DEPTH_CAMERAS = 15;
const int DEFAULT_PODS_PER_HOST = 1;

const int COLOR_WIDTH = 1920;// 2048;
const int COLOR_HEIGHT = 1080;// 2048;

const int cDepthWidth = 1024;// 2048;
const int cDepthHeight = 1024;// 2048;

// Number of frames after which a new Fusion Key Volume is set. Set 1 to update the key volume at each frame
extern int FrameCountKeyVolume;



//////////////////////////////
/// CalibrationSoftware Defines
//////////////////////////////

#define DEFAULT_CALIBRATION_FILENAME "calibCameras3DTM.json"

#endif