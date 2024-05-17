///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once

#include "combaseapi.h"
#include "LinkedList.h"
#include <iostream>

interface IInternalIO
{
    // Caller passes in results from GetQueuedCompletionStatus
    virtual HRESULT STDMETHODCALLTYPE Process
        (
            BOOL result,
            void* context,  // reinterpret_cast<LPOVERLAPPED>
            ULONG cbBytes
        )  = 0;
};

class CConnectionManager
{
public:
    friend class CTcpListener;
    CConnectionManager ();
    virtual ~CConnectionManager ();
    DWORD GetNumConnections();

public:
	HRESULT SetInput(interface ICompressor* compressor);
    HRESULT STDMETHODCALLTYPE Startup (USHORT port);
    HRESULT STDMETHODCALLTYPE Shutdown ();

protected:
    enum { eStopped, eStarted, eStopping } m_eState;
	interface ICompressor* m_compressor;
    HRESULT m_lastError;
    HANDLE m_hIocp;
    HANDLE m_hWorkerThread;
    class CTcpListener* m_listener;
    CLinkEntry m_connections;

protected:
    HRESULT WINAPI StartAndRegisterConnection
        (
            class CTcpConnection* pConnection
        );

    //
    // worker thread
    //
protected:
    HRESULT StartWorkerThread ();
    HRESULT StopWorkerThread ();
    static DWORD WINAPI ThreadStart(void* param);
    HRESULT ThreadMain();
};
