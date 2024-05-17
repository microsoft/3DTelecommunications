#pragma once

#ifdef _WIN32
#include <Windows.h>
#else
#include "Win2Linux.h"
#endif

#include <iostream>
#include <stdio.h>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <string.h>
#include <vector>
#include <queue>
#include <map>
#include <fstream>

#include <k4a/k4a.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"