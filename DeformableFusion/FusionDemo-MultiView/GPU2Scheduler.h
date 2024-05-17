#pragma once
#include "cuda_runtime.h"
#include "helper_cuda.h"
#include "cuda.h"
#include <mutex>
#include <vector>
#include <algorithm>


#define MAXFRAMESINPROCESSING 5

struct frameProcessingState
{
	volatile int inUse;
	volatile int frameId;
	volatile int depthuploadedGPU0; //started upload
	volatile int depthuploadedGPU1; //started upload
	volatile int GPU0ComputeFinished;
	volatile int coarseEDHostCopyStarted;
	volatile int coarseEDonHost;
	volatile int coarseEDonGPU1;
	volatile int GPU1ComputeFinished;
	volatile int FrameCopiedFromGPU1;
};

class GPU2Scheduler
{
public:
	GPU2Scheduler(int depthFrameSize, int edCoarseSize, int fusionSize);
	~GPU2Scheduler();

	void InitScheduler();

	void DeInitScheduler();

//	void DoSimulation();

protected:
	void InitGPU();
	void DeInitGPU();

	void InitGPU0();
	void InitGPU1();

	void DeInitGPU0();
	void DeInitGPU1();


	void RunThreadInput();
	void RunThreadGPU0();
	void RunThreadGPU1();

	volatile int _inputFrame;
	(volatile frameProcessingState)* _frameStates;
	volatile int _outputFrame;

	void InitState(int frameStateId, int cpuFrameId);
	void StartFrameOnGPUs(int frameStateId);
	void StartProcessingOnGPU1(int frameStateId);
	void OutputFrame(int frameStateId);
	void CleanupFrame(int frameStateId);

	// issue copy of completed frame from GPU 0 to CPU
	void CopyCoarseEDToCPU();

	// if a slot and new depth frame available - get the slot and issue copy of new depth frame to GPU
	void StartNewFrame();

	// try to continue processing depth frame on GPU0
	void ProcessNextFrameGPU0();
	// issue copy of completed frame from CPU to GPU
	void CopyCoarseEDToGPU1();
	// issue copy of new depth frame to GPU
	void CopyDepthFrameToGPU1();

	// cleanup completed frame slots
	void CleanupComplettedFrames();

	// try to continue processing depth frame on GPU1
	void ProcessNextFrameGPU1();

	int CountUsedGPUFrames();

	//state shows frameid that has finished processing on the device

	int _depthFrameSize;
	int _edCoarseSize;
	int _edFineSize;
	int _fusionSize;
	int _framesOnCPU;

	std::vector<cpuFrameData2> _frames;

	// GPU state
	CUcontext _ctx0;
	CUcontext _ctx1;
	// GPU data
	float * _gpu0depth[MAXFRAMESINPROCESSING];
	float * _gpu0edData[MAXFRAMESINPROCESSING];

	float *  _gpu1depth[MAXFRAMESINPROCESSING];
	float *  _gpu1edDataCoarse[MAXFRAMESINPROCESSING];
	float *  _gpu1edDataFine[MAXFRAMESINPROCESSING];
	float *  _gpu1FusionData[MAXFRAMESINPROCESSING];
	float *  _gpu1Output[MAXFRAMESINPROCESSING];

	float * _EDcoarsePinned[MAXFRAMESINPROCESSING];// on CPU-pinned
	float * _depthOnCPUPinned[MAXFRAMESINPROCESSING];// on CPU - pinned

	cudaStream_t _sGPU0compute;
	cudaStream_t _sGPU0memoryDepth;
	cudaStream_t _sGPU0memoryCoarse;
	cudaStream_t _sGPU1compute;
	cudaStream_t _sGPU1memoryDepth;
	cudaStream_t _sGPU1memoryOutput;

	// depth -> gpu 0 transfer unblocks coarse LM
	cudaEvent_t _depthToGpu0[MAXFRAMESINPROCESSING];

	// depth -> gpu 1 transfer preceeds coarse LM upload - no sync needed
	// coarse LM finished unblocks coarse ED -> CPU transfer
	cudaEvent_t _coarseLMFinishedGpu0[MAXFRAMESINPROCESSING];

	// coarse ED -> CPU transfer finished unblocks 
	cudaEvent_t _coarseLMToCPUFinished[MAXFRAMESINPROCESSING];

	cudaEvent_t _coarseLMToGPU1Finished[MAXFRAMESINPROCESSING];

	cudaEvent_t _depthToGpu1[MAXFRAMESINPROCESSING];
	// fineLM finished - no need, used by blocking fusiuon call
	cudaEvent_t _fusionFinished[MAXFRAMESINPROCESSING];

	// need this just to make it more realistic
	cudaEvent_t _outputFinished[MAXFRAMESINPROCESSING];

};

