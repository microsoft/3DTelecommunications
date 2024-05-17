///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///


#include "stdafx.h"
#include "HoloNet.h"
#include "HoloNetP.h"
#include "HNManager.h"
#include "ATLUtil.h"

using namespace HoloNet;

// --------------------------------------------------------------------------
// 
// ctor/init
//

HRESULT WINAPI FactoryP::CreateNetManager
    (
        INetManager** managerOut
    )
{
    HRESULT hr = S_OK;
    CHoloNetManager* pManager = NULL;

    if ( NULL == managerOut )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    pManager = new CHoloNetManager();
    if ( NULL == pManager )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    hr = pManager->QueryInterface(__uuidof(INetManager), (void**)managerOut);

bailout:
    if ( NULL != pManager )
    {
        pManager->Release();
    }
    return hr;
}

CHoloNetManager::CHoloNetManager () : 
    m_cRef(1),
    m_eState(eStopped),
    m_bSocketsInitialized(false),
    m_hIocp(NULL),
    m_cThreads(0)
{
    for ( DWORD i = 0; i < s_dwConcurrentThreads; i++ )
    {
        m_hWorkerThread[i] = NULL;
        m_lastError[i] = S_OK;
    }
}

CHoloNetManager::~CHoloNetManager ()
{
    Shutdown();
}


// --------------------------------------------------------------------------
//
// IUnknown
//

STDMETHODIMP CHoloNetManager::QueryInterface
    (
        REFIID riid,
        __deref_out void **ppv
    )
{
    HRESULT hr = S_OK;

    if ( NULL == ppv )
    {
        hr = E_POINTER;
    }
    else
    if ( __uuidof(IUnknown) == riid )
    {
        *ppv = static_cast<INetManager*>(this);
        AddRef();
    }
    else
    if ( __uuidof(INetManager) == riid )
    {
        *ppv = static_cast<INetManager*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CHoloNetManager::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CHoloNetManager::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
    if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}


// --------------------------------------------------------------------------
//
// INetManager
//

HRESULT STDMETHODCALLTYPE 
CHoloNetManager::Startup ()
{
    HRESULT hr = S_OK;

    if ( eStopped != m_eState )
    {
        hr = E_ABORT;
        goto bailout;
    }

    if ( !m_bSocketsInitialized )
    {
        WSADATA wsadata;
        if ( 0 != ::WSAStartup(MAKEWORD(2,2), &wsadata) )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
            goto bailout;
        }
        m_bSocketsInitialized = true;
    }

    hr = StartWorkerThreads();
    if ( SUCCEEDED(hr) )
    {
        m_eState = eStarted;
    }

bailout:
    return hr;
}


HRESULT STDMETHODCALLTYPE
CHoloNetManager::CreateConnection
    (
        LPCWSTR url,
        INetConnection** connectionOut
    )
{
    HRESULT hr = S_OK;
    CUrl decodedUrl;

    if ( NULL == url
         || NULL == connectionOut )
    {
        hr = E_POINTER;
        goto bailout;
    }

    if ( !decodedUrl.CrackUrl(url) )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( 0 == wcscmp(L"multicast", decodedUrl.GetSchemeName()) )
    {
        hr = FactoryP::CreateMulticastConnection(decodedUrl.GetHostName(), decodedUrl.GetPortNumber(), m_hIocp, connectionOut);
        if ( FAILED(hr) )
        {
            goto bailout;
        }
    }
    else
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

bailout:
    return hr;
}


HRESULT STDMETHODCALLTYPE 
CHoloNetManager::Shutdown ()
{
    HRESULT hr = S_OK;

    if ( eStarted != m_eState )
    {
        hr = E_ABORT;
    }
    else
    {
        m_eState = eStopping;

        hr = StopWorkerThreads();
        if ( SUCCEEDED(hr) )
        {
            m_eState = eStopped;
        }
    }

    if ( m_bSocketsInitialized )
    {
        ::WSACleanup();
        m_bSocketsInitialized = false;
    }

    return hr;
}

// --------------------------------------------------------------------------
//
// worker thread
//

HRESULT 
CHoloNetManager::StartWorkerThreads ()
{
    HRESULT hr = S_OK;

    // Create the completion port for concurrent access by the worker threads
    m_hIocp = ::CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, NULL, s_dwConcurrentThreads);
    if ( NULL == m_hIocp )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

    m_cThreads = 0;
    for ( DWORD i = 0; i < s_dwConcurrentThreads; i++ )
    {
        DWORD threadId;
        m_hWorkerThread[i] = ::CreateThread
                            (
                                NULL, 0, 
                                ThreadStart,
                                this,
                                0,
                                &threadId
                            );
        if ( NULL == m_hWorkerThread[i] )
        {
            hr = HRESULT_FROM_WIN32(::GetLastError());
            goto bailout;
        }
    }

bailout:
    if ( FAILED(hr) )
    {
        // race condition here if all threads have not incremented m_cThreads
        // TODO: add active wait to match m_cThreads with last successfully started thread
        StopWorkerThreads();
    }
    return hr;
}


HRESULT 
CHoloNetManager::StopWorkerThreads ()
{
    HRESULT hr = S_OK;

    for ( DWORD i = 0; i < s_dwConcurrentThreads; i++ )
    {
        if ( NULL != m_hWorkerThread[i] )
        {
            // Signal thread to exit
            if ( !::PostQueuedCompletionStatus(m_hIocp, 0, NULL, NULL) )
            {
                hr = HRESULT_FROM_WIN32(::GetLastError());
                goto bailout;
            }
        }
    }

    for ( DWORD i = 0; i < s_dwConcurrentThreads; i++ )
    {
        if ( NULL != m_hWorkerThread[i] )
        {
            // Wait for thread to exit
            if ( WAIT_FAILED == ::WaitForSingleObject(m_hWorkerThread[i], INFINITE) )
            {
                hr = HRESULT_FROM_WIN32(::GetLastError());
                goto bailout;
            }
            if ( !::CloseHandle(m_hWorkerThread[i]) )
            {
                hr = HRESULT_FROM_WIN32(::GetLastError());
                goto bailout;
            }
            m_hWorkerThread[i] = NULL;
        }
    }

    if ( NULL != m_hIocp )
    {
        if ( !::CloseHandle(m_hIocp) )
        {
            hr = HRESULT_FROM_WIN32(::GetLastError());
        }
        m_hIocp = NULL;
    }

bailout:
    return hr;
}


DWORD WINAPI 
CHoloNetManager::ThreadStart(void* param)
{
    return ((CHoloNetManager*)param)->ThreadMain();
}


HRESULT 
CHoloNetManager::ThreadMain()
{
    HRESULT hr = S_OK;
    LONG threadIndex = InterlockedIncrement(&m_cThreads) - 1;
    m_lastError[threadIndex] = hr;

    // processing loop
    while ( true )
    {
        BOOL result = FALSE;
        DWORD dwIoBytes = 0;
        IInternalIO* pIoProcessor = NULL;
        LPOVERLAPPED pIoContext = NULL;

        // dequeue next job
        result = ::GetQueuedCompletionStatus
                    (
                        m_hIocp,
                        &dwIoBytes,
                        reinterpret_cast<ULONG_PTR*>(&pIoProcessor),    // completion key
                        &pIoContext,
                        INFINITE
                    );

        // check for thread exit signal
        if ( NULL == pIoProcessor )
        {
            break;
        }

        // process
        hr = pIoProcessor->Process(result, pIoContext, dwIoBytes);
        if ( FAILED(hr) )
        {
            m_lastError[threadIndex] = hr;
        }
    } // end processing loop

    if ( 0 == InterlockedDecrement(&m_cThreads) )
    {
        // The last thread cancels any queued jobs
        while ( true )
        {
            BOOL result = FALSE;
            DWORD dwIoBytes = 0;
            IInternalIO* pIoProcessor = NULL;
            LPOVERLAPPED pIoContext = NULL;

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
            {
                break;  // no jobs left on the completion port
            }

            hr = pIoProcessor->Process(false, pIoContext, dwIoBytes);
            if ( FAILED(hr) )
            {
                m_lastError[threadIndex] = hr;
            }
        } // end cancellation loop
    }

    return hr;
}
