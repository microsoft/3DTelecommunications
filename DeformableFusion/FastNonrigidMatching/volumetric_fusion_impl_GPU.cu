#define USING_THRUST
#define WHICH_GPU 0
#ifdef USING_THRUST
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/device_vector.h>
#include <thrust/system/cuda/execution_policy.h>
#include <thrust/system/omp/execution_policy.h> 
#include <thrust/system/cpp/execution_policy.h>
#endif

#include "volumetric_fusion_impl_GPU.cuh"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_math_common.cuh"
#include "geometry_types_cuda.h"
#include "cuda_vector_fixed.cuh"
#include "DeformGraphCudaImpl_GPU.cuh"
#include "triangle_table.h"
#include "utility.h"

#include <vector>
#include <stdio.h>
#include <algorithm>
#include <future>
#include <assert.h>

namespace Fusion4D_GPU{
#include "volumetric_fusion_impl.cu"
#include "volumetric_fusion_marchingcubes.cu"
#include "volumetric_fusion_marchingcubes_vMesh.cu"
#include "mc_mesh_processing.cu"
}