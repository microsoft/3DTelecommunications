#include "stdafx.h"
#include "DeformGraphCuda_GPU.h"
#include "DeformGraph.h"
#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

using namespace NonrigidMatching;

namespace Fusion4D_GPU{
#include "DeformGraphCuda.cpp"
}