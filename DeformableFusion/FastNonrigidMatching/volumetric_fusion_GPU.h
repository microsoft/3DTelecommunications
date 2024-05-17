#ifndef __VOLUMETRIC_FUSION_GPU_CUH__
#define __VOLUMETRIC_FUSION_GPU_CUH__
#include "volumetric_fusion_impl_GPU.cuh"
#include <vector>
#include <opencv2\opencv.hpp>
#include "BoundingBox3D.h"
#include "UtilMatrix.h"
#include "CameraView.h"
#include "UtilVnlMatrix.h"
#include "utility.h"

#include "TSDF.h"
#include "CPointCloud.h"

#include "color_map.h"

namespace Fusion4D_GPU{
#include "volumetric_fusion.h"
}

#endif