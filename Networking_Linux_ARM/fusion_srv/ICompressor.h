///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#include "../Win2Linux.h"

#pragma once

interface ICompressor
{
	virtual DWORD GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) PURE;

	virtual void ReleaseLastFetchedPacket(DWORD swapBufferIndex) PURE; 
};
