// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.
#pragma once

#include <functional>

int RunCalibrationApi(int argc, const char** argv,
    void* progressState,
    std::function<void(void*, uint32_t, uint32_t)> ProgressCallback,
    std::function<void(void*, float, float, float)> CalibrationCompletedCallback);