///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#include <Unknwn.h>

#pragma once

interface ICompressor
{
	virtual HRESULT GetLatestPacket(const BYTE** ppData, DWORD* pcbData, DWORD* pdFrameNumber) PURE;

	virtual void ReleaseLastFetchedPacket() {}; // not always needed unless 
};
