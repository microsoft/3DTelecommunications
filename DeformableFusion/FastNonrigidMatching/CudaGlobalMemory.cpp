// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#include "stdafx.h"
#include "CudaGlobalMemory.h"

long long CudaGlobalMemoryStatic::bytes_allocated = 0;
long long CudaGlobalMemoryStatic::bytes_max = 0;
char* CudaGlobalMemoryStatic::dev_global_mem = NULL;
std::vector<void*> CudaGlobalMemoryStatic::dev_pts = std::vector<void*>();
std::vector<long long> CudaGlobalMemoryStatic::size_mem_segments = std::vector<long long>();

