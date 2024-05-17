///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#include "sr_stdafx.h"
#include "ConnectionManager.h"
#include "TCPListener.h"
#include "TCPConnection.h"
#include "TimeUtil.h"

// --------------------------------------------------------------------------
// 
// ctor/init
// 

CConnectionManager::CConnectionManager () : 
    m_eState(eStopped),
    m_lastError(S_OK),
    m_hIocp(NULL),
    m_hWorkerThread(NULL),
    m_listener(NULL)
{
}

CConnectionManager::~CConnectionManager ()
{
    if ( NULL != m_listener )
    {
        delete m_listener;
    }

    while ( !m_connections.IsEmpty() )
    {
        CTcpConnection* pConnection = LinkEntryContainer(m_connections.RemoveHead(), CTcpConnection, LinkEntry);
        delete pConnection;
    }
}

DWORD CConnectionManager::GetNumConnections()
{
    return CTcpConnection::GetConnectionCount();
}

HRESULT CConnectionManager::SetInput(interface ICompressor* compressor)
{
    if ( NULL == compressor )
    {
        return E_POINTER;
    }

    m_compressor = compressor;
    return S_OK;
}

HRESULT STDMETHODCALLTYPE 
CConnectionManager::Startup (USHORT port)
{
    HRESULT hr;

    if ( eStopped != m_eState )
    {
        hr = E_ABORT;
        goto bailout;
    }

    WSADATA wsadata;
    if ( 0 != ::WSAStartup(MAKEWORD(2,2), &wsadata) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    hr = StartWorkerThread();
    if ( SUCCEEDED(hr) )
    {
        m_eState = eStarted;
    }

    hr = CTcpListener::CreateInstance(this, port, m_hIocp, &m_listener);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_listener->Startup();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

bailout:
    return hr;
}

HRESULT STDMETHODCALLTYPE 
CConnectionManager::Shutdown ()
{
    std::cout << "Connection Manager shutting down." << std::endl;
    HRESULT hr;
    if ( eStarted != m_eState )
    {
        hr = E_ABORT;
        goto bailout;
    }
    m_eState = eStopping;

    if ( NULL != m_listener )
    {
        std::cout << "Shutting down TCP listener...";
        m_listener->Shutdown();
        std::cout << "Done.\nShutting down TCP connections...";
    }

    for ( CLinkEntry* entry = m_connections.Next();
            entry != &m_connections;
            entry = entry->Next() )
    {
        CTcpConnection* pConnection = LinkEntryContainer(entry, CTcpConnection, LinkEntry);
        pConnection->Shutdown();
    }
    std::cout << "Done.\nStopping worker thread...";
    hr = StopWorkerThread();
    if ( SUCCEEDED(hr) )
    {
        m_eState = eStopped;
    }
    std::cout << "Done.\nCleaning up WSA...";

    ::WSACleanup();

    std::cout << "Done." << std::endl;

bailout:
    return hr;
}

// --------------------------------------------------------------------------
//
// worker thread
//

HRESULT 
CConnectionManager::StartWorkerThread ()
{
    HRESULT hr = S_OK;

    // Allow only one thread though the completion port
    m_hIocp = ::CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 1);
    if ( NULL == m_hIocp )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

    DWORD threadId;
    m_hWorkerThread = ::CreateThread
                        (
                            NULL, 0, 
                            ThreadStart,
                            this,
                            0,
                            &threadId
                        );
    if ( NULL == m_hWorkerThread )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

bailout:
    if ( FAILED(hr) )
        StopWorkerThread();
    return hr;
}


HRESULT 
CConnectionManager::StopWorkerThread ()
{
    HRESULT hr = S_OK;

    if ( NULL != m_hWorkerThread )
    {
        // Signal thread and wait for it to exit
        if ( !::PostQueuedCompletionStatus(m_hIocp, 0, NULL, NULL)
             || WAIT_FAILED == ::WaitForSingleObject(m_hWorkerThread, INFINITE) )
            hr = HRESULT_FROM_WIN32(::GetLastError());

        if ( !::CloseHandle(m_hWorkerThread) )
            hr = HRESULT_FROM_WIN32(::GetLastError());
        m_hWorkerThread = NULL;
    }

    if ( NULL != m_hIocp )
    {
        if ( !::CloseHandle(m_hIocp) )
            hr = HRESULT_FROM_WIN32(::GetLastError());
        m_hIocp = NULL;
    }

    return hr;
}


DWORD WINAPI 
CConnectionManager::ThreadStart(void* param)
{
    return ((CConnectionManager*)param)->ThreadMain();
}


HRESULT 
CConnectionManager::ThreadMain()
{
    m_lastError = S_OK;

    // processing loop
    while ( true )
    {
        BOOL result = FALSE;
        DWORD dwIoBytes = 0;
        IInternalIO* pIoProcessor = NULL;
        LPOVERLAPPED pIoContext = NULL;
        TimingTable timing_table("connection_manager");

        timing_table.Start("connection_manager_processing_loop", "ConnectionManager Processing Loop");
        timing_table.Start("connection_manager_processing_loop_get_queued", "ConnectionManager Processing Loop Get Queued Completion Status");

        // dequeue next job
        result = ::GetQueuedCompletionStatus
                    (
                        m_hIocp,
                        &dwIoBytes,
                        reinterpret_cast<ULONG_PTR*>(&pIoProcessor),    // completion key
                        &pIoContext,
                        INFINITE
                    );
        timing_table.End("connection_manager_processing_loop_get_queued");

        // check for thread exit signal
        if ( NULL == pIoProcessor )
            break;

        // process
        HRESULT hr = pIoProcessor->Process(result, pIoContext, dwIoBytes);
        if ( FAILED(hr) )
            m_lastError = hr;

        timing_table.End("connection_manager_processing_loop");
        timing_table.LogTimes(Logger::Debug);

    } // end processing loop

    // cancel any queued jobs
    while ( true )
    {
        BOOL result = FALSE;
        DWORD dwIoBytes = 0;
        IInternalIO* pIoProcessor = NULL;
        LPOVERLAPPED pIoContext = NULL;
        TimingTable timing_table("connection_manager");

        timing_table.Start("connection_manager_cancelling_loop", "ConnectionManager Job cancelling Loop");

        // dequeue next job
        result = ::GetQueuedCompletionStatus
                    (
                        m_hIocp,
                        &dwIoBytes,
                        reinterpret_cast<ULONG_PTR*>(&pIoProcessor),    // completion key
                        &pIoContext,
                        0 // time out immediately
                    );

        if ( NULL == pIoContext )
            break;  // no jobs left on the completion port

        HRESULT hr = pIoProcessor->Process(false, pIoContext, dwIoBytes);
        if ( FAILED(hr) )
            m_lastError = hr;

        timing_table.End("connection_manager_cancelling_loop");
        timing_table.LogTimes(Logger::Debug);

    } // end cancellation loop

    return m_lastError;
}

HRESULT WINAPI 
CConnectionManager::StartAndRegisterConnection
    (
        class CTcpConnection* pConnection
    )
{
    if ( NULL == pConnection )
        return E_POINTER;

    HRESULT hr;

    if ( eStarted != m_eState )
    {
        hr = E_ABORT;
        goto bailout;
    }

    hr = pConnection->Startup();
    if ( SUCCEEDED(hr) )
    {
        m_connections.InsertTail(&pConnection->LinkEntry);
    }

bailout:
    return hr;
}
