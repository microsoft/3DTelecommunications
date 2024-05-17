#ifndef __GLOBALDATASTATIC_H__
#define __GLOBALDATASTATIC_H__
#include <Windows.h>
#include <mutex>
#include <vector>

class GlobalDataStatic
{
public:
	static int vts_num;

	static bool bNewSurface;
	static bool bResetKeyFrame;
	static bool bStopFusion;
	static bool bSendRightViewThroughMeshServer;
	static bool bInitbackgroundNextFrame;
	static bool bExit;
	static bool bSave;
	static bool bNextFrameHighQuality;
	static int HighQualityFrameNumber;
	static int FrameStartEndOffset;
	static std::vector<int> DepthOffset;


public:
	static std::mutex mutex_cuda;
};

#endif