#define CUDA_API_PER_THREAD_DEFAULT_STREAM 
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "Windows.h"

#include "GPU2Scheduler.h"

GPU2Scheduler::GPU2Scheduler(int depthFrameSize, int edCoarseSize, int fusionSize)
{
	_depthFrameSize = depthFrameSize;
	_edCoarseSize = edCoarseSize;
	_edFineSize = edFineSize;
	_fusionSize = fusionSize;
	_framesOnCPU = framesOnCPU;

	_inputFrame = 0;

	_outputFrame = -1;

	_frames.resize(framesOnCPU);
	for (int i = 0; i < (int)_frames.size(); i++)
	{
		_frames[i].depth.resize(_depthFrameSize);
	}
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		_frameStates[i].inUse = 0;
	}
}

void GPU2Scheduler::InitGPU()
{
	cudaSetDevice(1);
	InitGPU1();
	cuCtxPopCurrent(&_ctx1);

	cudaSetDevice(0);
	InitGPU0();
	cuCtxPopCurrent(&_ctx0);
}

void GPU2Scheduler::DeInitGPU()
{
	DeInitGPU0();

	DeInitGPU1();
}

void GPU2Scheduler::InitGPU0()
{
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		cudaMalloc((void**)&_gpu0depth[i], _depthFrameSize * sizeof(float));
		cudaMalloc((void**)&_gpu0edData[i], _edCoarseSize * sizeof(float));

		cudaHostAlloc((void **)&_EDcoarsePinned[i], _edCoarseSize * sizeof(float), cudaHostAllocPortable);
		cudaHostAlloc((void **)&_depthOnCPUPinned[i], _depthFrameSize * sizeof(float), cudaHostAllocPortable);
		cudaHostAlloc((void **)&_frameStates, sizeof(frameProcessingState)*MAXFRAMESINPROCESSING, cudaHostAllocPortable);

		cudaEventCreateWithFlags(&_depthToGpu0[i], cudaEventDisableTiming);
		cudaEventCreateWithFlags(&_coarseLMFinishedGpu0[i], cudaEventDisableTiming);
		cudaEventCreateWithFlags(&_coarseLMToCPUFinished[i], cudaEventDisableTiming);
	}

	//start by uploading depth frame 0 to GPU0 - we don't process it in any form - only used to process frame 1
	cudaStreamCreate(&_sGPU0compute);
	cudaStreamCreateWithFlags(&_sGPU0memoryDepth, cudaStreamNonBlocking);
	cudaStreamCreate(&_sGPU0memoryCoarse);

	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		_frameStates[i].inUse = 0;
	}
	_inputFrame = 0;
	_outputFrame = 0;
}

void GPU2Scheduler::InitGPU1()
{
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		cudaMalloc((void**)&_gpu1depth[i], _depthFrameSize * sizeof(float));
		cudaMalloc((void**)&_gpu1edDataCoarse[i], _edCoarseSize * sizeof(float));
		cudaMalloc((void**)&_gpu1edDataFine[i], _edFineSize * sizeof(float));
		cudaMalloc((void**)&_gpu1FusionData[i], _fusionSize * sizeof(float));
		cudaMalloc((void**)&_gpu1Output[i], _fusionSize * sizeof(float));


		cudaEventCreateWithFlags(&_coarseLMToGPU1Finished[i], cudaEventDisableTiming);
		cudaEventCreateWithFlags(&_depthToGpu1[i], cudaEventDisableTiming);
		cudaEventCreateWithFlags(&_fusionFinished[i], cudaEventDisableTiming);
		cudaEventCreateWithFlags(&_outputFinished[i], cudaEventDisableTiming);
	}

	cudaStreamCreate(&_sGPU1compute);
	cudaStreamCreateWithFlags(&_sGPU1memoryDepth, cudaStreamNonBlocking);
	cudaStreamCreate(&_sGPU1memoryOutput);
}

void GPU2Scheduler::DeInitGPU0()
{
	cuCtxPushCurrent(_ctx0);
	cudaDeviceSynchronize();
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		cudaFree(_gpu0depth[i]);
		cudaFree(_gpu0edData[i]);
		cudaFreeHost(_EDcoarsePinned[i]);
		cudaFreeHost(_depthOnCPUPinned[i]);

		cudaEventDestroy(_depthToGpu0[i]);
		cudaEventDestroy(_coarseLMFinishedGpu0[i]);
		cudaEventDestroy(_coarseLMToCPUFinished[i]);
	}


	cudaStreamDestroy(_sGPU0compute);
	cudaStreamDestroy(_sGPU0memoryDepth);
	cudaStreamDestroy(_sGPU1memoryOutput);
	cuCtxPopCurrent(&_ctx0);
}

void GPU2Scheduler::DeInitGPU1()
{
	cuCtxPushCurrent(_ctx1);
	cudaDeviceSynchronize();
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{

		cudaFree(_gpu1depth[i]);
		cudaFree(_gpu1edDataCoarse[i]);
		cudaFree(_gpu1edDataFine[i]);
		cudaFree(_gpu1FusionData[i]);
		cudaFree(_gpu1Output[i]);


		cudaEventDestroy(_coarseLMToGPU1Finished[i]);
		cudaEventDestroy(_depthToGpu1[i]);
		cudaEventDestroy(_fusionFinished[i]);
		cudaEventDestroy(_outputFinished[i]);
	}


	cudaStreamDestroy(_sGPU1compute);
	cudaStreamDestroy(_sGPU1memoryDepth);
	cuCtxPopCurrent(&_ctx1);
}

GPU2Scheduler::~GPU2Scheduler()
{
}

void GPU2Scheduler::DoSimulation()
{
	InitGPU();

#pragma omp parallel for num_threads(3), schedule(dynamic)
	for (int i = 0; i < 3; i++)
	{
		if (i == 0) RunThreadGPU0();
		if (i == 1) RunThreadGPU1();
		if (i == 2) RunThreadInput();
	}

	DeInitGPU();
}


void GPU2Scheduler::RunThreadInput()
{
	for (int frameId = 0; frameId < (int)_frames.size(); frameId++)
	{
		Sleep(25);
		_inputFrame++;
	}
	_inputFrame = -1;
}

void GPU2Scheduler::RunThreadGPU0()
{
	while (true)
	{
		//generic logic: issue all data copy work that's outstanding, then try to process new frame

		CopyCoarseEDToCPU();

		// if a slot and new depth frame available - get the slot and issue copy of new depth frame to GPU
		StartNewFrame();

		// try to continue processing depth frame on GPU0
		ProcessNextFrameGPU0();

		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (_frameStates[i].inUse == 0) continue;
			if (_gpu0doneState[i] && !_frameStates[i].coarseEDonCPUProcessed)
			{
				StartProcessingOnGPU1(i);
				_frameStates[i].coarseEDonCPUProcessed = 1;
			}
		}
		int usedGPUFrames = CountUsedGPUFrames();
		// check if all work is done
		if (_inputFrame == -1 && usedGPUFrames == 0) break;


		// try to output a frame that is ready
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (_frameStates[i].inUse == 0) continue;
			if (_gpu1doneState[i] && !_frameStates[i].outputFrameProcessed)
			{
				OutputFrame(i); //deinit
			}
		}
		// try to cleanup a frame that was processed and is not needed for current/future iteration
		usedGPUFrames = CountUsedGPUFrames();
		if (usedGPUFrames > 1 || (usedGPUFrames == 1 && _inputFrame == -1))
		{
			int oldestFrameInProcess = 0x7fffffff;
			int oldestFrameStateId = -1;
			for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
			{
				if (_frameStates[i].inUse == 0) continue;
				if (_frameStates[i].frameId < oldestFrameInProcess)
				{
					oldestFrameInProcess = _frameStates[i].frameId;
					oldestFrameStateId = i;
				}
			}
			if (oldestFrameStateId != -1)
			{
				if ((_frameStates[oldestFrameStateId].inUse != 0) &&
					_gpu1doneState[oldestFrameStateId] &&
					_frameStates[oldestFrameStateId].outputFrameProcessed)
					CleanupFrame(oldestFrameStateId); //deinit
			}
		}

	}
}


void GPU2Scheduler::RunThreadGPU0()
{
	while (true)
	{
		//generic logic: issue all data copy work that's outstanding, then try to process new frame

		// issue copy of completed frame from CPU to GPU
		CopyCoarseEDToGPU1();
		// issue copy of new depth frame to GPU
		CopyDepthFrameToGPU1();

		// cleanup completed frame slots
		CleanupComplettedFrames();

		// try to continue processing depth frame on GPU1
		ProcessNextFrameGPU1();

		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (_frameStates[i].inUse == 0) continue;
			if (_gpu0doneState[i] && !_frameStates[i].coarseEDonCPUProcessed)
			{
				StartProcessingOnGPU1(i);
				_frameStates[i].coarseEDonCPUProcessed = 1;
			}
		}

		int usedGPUFrames = CountUsedGPUFrames();
		// check if all work is done
		if (_inputFrame == -1 && usedGPUFrames == 0) break;
		// try to start processing new depth frame if available
		if (usedGPUFrames < MAXFRAMESINPROCESSING)
		{
			int maxFrameInProcess = -1;
			int previousFrameStateId = -1;
			for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
			{
				if (_frameStates[i].inUse == 0) continue;
				if (_frameStates[i].frameId > maxFrameInProcess)
				{
					maxFrameInProcess = _frameStates[i].frameId;
					previousFrameStateId = i;
				}
			}
			if (maxFrameInProcess < _inputFrame)
			{
				for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
				{
					if (_frameStates[i].inUse == 1) continue;
					InitState(i, previousFrameStateId, _inputFrame);
					StartFrameOnGPUs(i);
					break;
				}
			}
		}

		// try to output a frame that is ready
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (_frameStates[i].inUse == 0) continue;
			if (_gpu1doneState[i] && !_frameStates[i].outputFrameProcessed)
			{
				OutputFrame(i); //deinit
			}
		}
		// try to cleanup a frame that was processed and is not needed for current/future iteration
		usedGPUFrames = CountUsedGPUFrames();
		if (usedGPUFrames > 1 || (usedGPUFrames == 1 && _inputFrame == -1))
		{
			int oldestFrameInProcess = 0x7fffffff;
			int oldestFrameStateId = -1;
			for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
			{
				if (_frameStates[i].inUse == 0) continue;
				if (_frameStates[i].frameId < oldestFrameInProcess)
				{
					oldestFrameInProcess = _frameStates[i].frameId;
					oldestFrameStateId = i;
				}
			}
			if (oldestFrameStateId != -1)
			{
				if ((_frameStates[oldestFrameStateId].inUse != 0) &&
					_gpu1doneState[oldestFrameStateId] &&
					_frameStates[oldestFrameStateId].outputFrameProcessed)
					CleanupFrame(oldestFrameStateId); //deinit
			}
		}

	}
}


void GPU2Scheduler::InitState(int frameStateId, int cpuFrameId)
{
	memset((void *)&_frameStates[frameStateId], 0, sizeof(frameProcessingState));

	_frameStates[frameStateId].frameId = cpuFrameId;
	_frameStates[frameStateId].inUse = 1;
}

void GPU2Scheduler::StartFrameOnGPUs(int frameStateId)
{
	memcpy(_depthOnCPUPinned[frameStateId], &_frames[_frameStates[frameStateId].frameId].depth[0], _depthFrameSize * sizeof(float));
	cuCtxPushCurrent(_ctx1);
	checkCudaErrors(cudaMemcpyAsync(_gpu1depth[frameStateId], _depthOnCPUPinned[frameStateId], _depthFrameSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU1memoryDepth));
	checkCudaErrors(cudaEventRecord(_depthToGpu1[frameStateId], _sGPU1memoryDepth));
	cuCtxPopCurrent(&_ctx1);

	cuCtxPushCurrent(_ctx0);
	checkCudaErrors(cudaMemcpyAsync(_gpu0depth[frameStateId], _depthOnCPUPinned[frameStateId], _depthFrameSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU0memoryDepth));
	checkCudaErrors(cudaEventRecord(_depthToGpu0[frameStateId], _sGPU0memoryDepth));

	// launch coarse;
	checkCudaErrors(cudaStreamWaitEvent(0, _depthToGpu0[frameStateId], 0));
	doCoarseLM(_frameStates[frameStateId].frameId, _depthFrameSize, _edCoarseSize, _gpu0depth[frameStateId], _gpu0edData[frameStateId], 0);
	cudaEventRecord(_coarseLMFinishedGpu0[frameStateId], _sGPU0compute);

	// copy coarseED to memory
	//	cudaStreamWaitEvent(_sGPU0memoryCoarse, _coarseLMFinishedGpu0[frameStateId], 0);
	checkCudaErrors(cudaMemcpyAsync(_EDcoarsePinned[frameStateId], _gpu0edData[frameStateId], _edCoarseSize * sizeof(float), cudaMemcpyDeviceToHost, 0));
	checkCudaErrors(cudaMemcpyAsync((void *)&(_gpu0doneState[frameStateId]), _gpu0edData[frameStateId], sizeof(float), cudaMemcpyDeviceToHost, 0));
	cuCtxPopCurrent(&_ctx0);
}

void GPU2Scheduler::StartProcessingOnGPU1(int frameStateId)
{
	//  memcpy(EDcoarseGPU1[frameStateId], EDcoarseGPU0[frameStateId], _edCoarseSize * sizeof(float));
	cuCtxPushCurrent(_ctx1);
	checkCudaErrors(cudaMemcpyAsync(_gpu1edDataCoarse[frameStateId], _EDcoarsePinned[frameStateId], _edCoarseSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU1memoryDepth));
	checkCudaErrors(cudaEventRecord(_coarseLMToGPU1Finished[frameStateId], _sGPU1memoryDepth));

	//launch fine LM
	checkCudaErrors(cudaStreamWaitEvent(0, _coarseLMToGPU1Finished[frameStateId], 0));
	doFineLM(_frameStates[frameStateId].frameId, _depthFrameSize, _edCoarseSize, _fusionSize, _edFineSize, _gpu1depth[frameStateId], _gpu1edDataCoarse[frameStateId], _gpu1edDataFine[frameStateId], 0);

	checkCudaErrors(cudaMemcpyAsync(_gpu1Output[frameStateId], _gpu1FusionData[frameStateId], _fusionSize * sizeof(float), cudaMemcpyDeviceToDevice, 0));
	//launch fusion
	//	cudaStreamWaitEvent(_sGPU1compute, _outputFinished[_frameStates[frameStateId].previousFrameStateId], 0); //wait for previous final fused frame data to be copied away to output
	doFusion(_frameStates[frameStateId].frameId, _depthFrameSize, _edFineSize, _fusionSize, _gpu1depth[frameStateId], _gpu1edDataFine[frameStateId], _gpu1FusionData[frameStateId], 0);
	//	cudaEventRecord(_fusionFinished[frameStateId], _sGPU1compute);

	//output
	//	cudaStreamWaitEvent(_sGPU1memoryOutput, _fusionFinished[frameStateId], 0);
	checkCudaErrors(cudaMemcpyAsync(_gpu1Output[frameStateId], _gpu1FusionData[frameStateId], _fusionSize * sizeof(float), cudaMemcpyDeviceToDevice, 0));
	checkCudaErrors(cudaMemcpyAsync((void *)&(_gpu1doneState[frameStateId]), _gpu1Output[frameStateId], sizeof(float), cudaMemcpyDeviceToHost, 0));
	//	cudaEventRecord(_outputFinished[frameStateId], _sGPU1memoryOutput);
	cuCtxPopCurrent(&_ctx1);
}

void GPU2Scheduler::OutputFrame(int frameStateId)
{
	_frameStates[frameStateId].outputFrameProcessed = 1;
}

void GPU2Scheduler::CleanupFrame(int frameStateId)
{
	_gpu0doneState[frameStateId] = 0;
	_frameStates[frameStateId].coarseEDonCPUProcessed = 0;
	_frameStates[frameStateId].frameId = -1;
	_frameStates[frameStateId].inUse = 0;
	_gpu1doneState[frameStateId] = 0;
}


int GPU2Scheduler::CountUsedGPUFrames()
{
	int cnt = 0;
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (_frameStates[i].inUse) cnt++;
	}
	return cnt;
}

void GPU2Scheduler::CopyCoarseEDToCPU()
{
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (!_frameStates[i].inUse) continue;
		if (!_frameStates[i].GPU0ComputeFinished) continue;
		if (_frameStates[i].coarseEDHostCopyStarted) continue;
		_frameStates[i].coarseEDHostCopyStarted = 1;
		cudaStreamWaitEvent(_sGPU0memoryCoarse, _coarseLMFinishedGpu0[i], 0);
		checkCudaErrors(cudaMemcpyAsync(_EDcoarsePinned[i], _gpu0edData[i], _edCoarseSize * sizeof(float), cudaMemcpyDeviceToHost, _sGPU0memoryCoarse));
		checkCudaErrors(cudaMemcpyAsync((void *)&(_frameStates[i].coarseEDonHost), _gpu0edData[i], sizeof(float), cudaMemcpyDeviceToHost, _sGPU0memoryCoarse));
	}

};

void GPU2Scheduler::StartNewFrame()
{
	int usedGPUFrames = CountUsedGPUFrames();
	// try to start processing new depth frame if available
	if (usedGPUFrames < MAXFRAMESINPROCESSING)
	{
		int maxFrameInProcess = -1;
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (!_frameStates[i].inUse) continue;
			if (_frameStates[i].frameId > maxFrameInProcess)
			{
				maxFrameInProcess = _frameStates[i].frameId;
			}
		}
		if (maxFrameInProcess < _inputFrame)
		{
			for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
			{
				if (_frameStates[i].inUse == 1) continue;
				int grabbedframe = _inputFrame;
				InitState(i, grabbedframe);
				memcpy(_depthOnCPUPinned[i], &_frames[_frameStates[i].frameId].depth[0], _depthFrameSize * sizeof(float));
				_frameStates[i].depthuploadedGPU0 = 1;
				checkCudaErrors(cudaMemcpyAsync(_gpu0depth[i], _depthOnCPUPinned[i], _depthFrameSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU0memoryDepth));
				checkCudaErrors(cudaEventRecord(_depthToGpu0[i], _sGPU0memoryDepth));
				break;
			}
		}
	}
};

void GPU2Scheduler::ProcessNextFrameGPU0()
{
	int frameToProcess = 0x7fffffff;
	int frameToProcessStateId = -1;
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (!_frameStates[i].inUse) continue;
		if (!_frameStates[i].depthuploadedGPU0) continue;
		if (_frameStates[i].GPU0ComputeFinished) continue;
		if (frameToProcess < _frameStates[i].frameId)
		{
			frameToProcess = _frameStates[i].frameId;
			frameToProcessStateId = i;
		}
	}
	if (frameToProcessStateId != -1)
	{
		_frameStates[frameToProcessStateId].GPU0ComputeFinished = 1;
	}

	if (frameToProcessStateId != -1)
	{
		checkCudaErrors(cudaStreamWaitEvent(0, _depthToGpu0[frameToProcessStateId], 0));
		doCoarseLM(_frameStates[frameToProcessStateId].frameId, _depthFrameSize, _edCoarseSize, _gpu0depth[frameToProcessStateId], _gpu0edData[frameToProcessStateId], 0);
		cudaEventRecord(_coarseLMFinishedGpu0[frameToProcessStateId], 0);
	}
};

void GPU2Scheduler::CopyCoarseEDToGPU1()
{
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (!_frameStates[i].inUse) continue;
		if (!_frameStates[i].coarseEDonHost) continue;
		if (_frameStates[i].coarseEDonGPU1) continue;
		_frameStates[i].coarseEDonGPU1 = 1;
		checkCudaErrors(cudaMemcpyAsync(_gpu1edDataCoarse[i], _EDcoarsePinned[i], _edCoarseSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU1memoryDepth));
		checkCudaErrors(cudaEventRecord(_coarseLMToGPU1Finished[i], _sGPU1memoryDepth));
	}
};

void GPU2Scheduler::CopyDepthFrameToGPU1()
{
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (!_frameStates[i].inUse) continue;
		if (_frameStates[i].depthuploadedGPU1) continue;
		_frameStates[i].depthuploadedGPU1 = 1;
		checkCudaErrors(cudaMemcpyAsync(_gpu1depth[i], _depthOnCPUPinned[i], _depthFrameSize * sizeof(float), cudaMemcpyHostToDevice, _sGPU1memoryDepth));
		checkCudaErrors(cudaEventRecord(_depthToGpu1[i], _sGPU1memoryDepth));
	}
};

void GPU2Scheduler::CleanupComplettedFrames()
{
	int usedGPUFrames = CountUsedGPUFrames();
	if (usedGPUFrames > 1 || (usedGPUFrames == 1 && _inputFrame == -1))
	{
		int oldestFrameInProcess = 0x7fffffff;
		int oldestFrameStateId = -1;
		for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
		{
			if (_frameStates[i].inUse == 0) continue;
			if (_frameStates[i].frameId < oldestFrameInProcess)
			{
				oldestFrameInProcess = _frameStates[i].frameId;
				oldestFrameStateId = i;
			}
		}
		if (oldestFrameStateId != -1)
		{
			if ((_frameStates[oldestFrameStateId].inUse != 0) &&
				_gpu1doneState[oldestFrameStateId] &&
				_frameStates[oldestFrameStateId].outputFrameProcessed)
				CleanupFrame(oldestFrameStateId); //deinit
		}
	}
};

void GPU2Scheduler::ProcessNextFrameGPU1()
{
	int frameToProcess = 0x7fffffff;
	int frameToProcessStateId = -1;
	for (int i = 0; i < MAXFRAMESINPROCESSING; i++)
	{
		if (!_frameStates[i].inUse) continue;
		if (!_frameStates[i].depthuploadedGPU1) continue;
		if (!_frameStates[i].coarseEDonGPU1) continue;
		if (_frameStates[i].GPU1ComputeFinished) continue;
		if (frameToProcess < _frameStates[i].frameId)
		{
			frameToProcess = _frameStates[i].frameId;
			frameToProcessStateId = i;
		}
	}
	if (frameToProcessStateId != -1)
	{
		_frameStates[frameToProcessStateId].GPU1ComputeFinished = 1;
	}

	if (frameToProcessStateId != -1)
	{
		checkCudaErrors(cudaStreamWaitEvent(0, _depthToGpu1[frameToProcessStateId], 0));
		checkCudaErrors(cudaStreamWaitEvent(0, _coarseLMToGPU1Finished[frameToProcessStateId], 0));
		doFineLM(_frameStates[frameToProcessStateId].frameId, _depthFrameSize, _edCoarseSize, _fusionSize, _edFineSize, _gpu1depth[frameToProcessStateId], _gpu1edDataCoarse[frameToProcessStateId], _gpu1edDataFine[frameToProcessStateId], 0);

		//launch fusion
		//	cudaStreamWaitEvent(_sGPU1compute, _outputFinished[_frameStates[frameStateId].previousFrameStateId], 0); //wait for previous final fused frame data to be copied away to output
		doFusion(_frameStates[frameToProcessStateId].frameId, _depthFrameSize, _edFineSize, _fusionSize, _gpu1depth[frameToProcessStateId], _gpu1edDataFine[frameToProcessStateId], _gpu1FusionData[frameToProcessStateId], 0);
		checkCudaErrors(cudaMemcpyAsync((void *)&(_frameStates[frameToProcessStateId].FrameCopiedFromGPU1), _gpu1Output[frameToProcessStateId], sizeof(float), cudaMemcpyDeviceToHost, 0));
		cudaEventRecord(_fusionFinished[frameToProcessStateId], 0);
	}
};