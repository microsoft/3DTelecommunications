#pragma once

// Windows Header Files:
#include <winsock2.h>
#include <mswsock.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#pragma comment(lib, "ws2_32.lib")

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

//// D3D
//#include <d3d9.h>
//#include <d3dx9.h>

// local
#include "WorkerThreadT.h"