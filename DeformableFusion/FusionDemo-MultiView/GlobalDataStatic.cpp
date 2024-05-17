#include "GlobalDataStatic.h"

std::mutex GlobalDataStatic::mutex_cuda;
int GlobalDataStatic::vts_num = -1;
bool GlobalDataStatic::bNewSurface = false;
bool GlobalDataStatic::bResetKeyFrame = false;
bool GlobalDataStatic::bExit = false;
bool GlobalDataStatic::bSave = false;
bool GlobalDataStatic::bStopFusion = false;
bool GlobalDataStatic::bSendRightViewThroughMeshServer = false;
bool GlobalDataStatic::bInitbackgroundNextFrame = false;
bool GlobalDataStatic::bNextFrameHighQuality = false;
int GlobalDataStatic::HighQualityFrameNumber = -1;
int GlobalDataStatic::FrameStartEndOffset = 0;
std::vector<int> GlobalDataStatic::DepthOffset = std::vector<int>();

