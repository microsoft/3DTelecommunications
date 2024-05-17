///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once

namespace HoloNet
{
    class CHoloNetManager :
        public INetManager
    {
    public:
        static const DWORD s_dwConcurrentThreads = 4;

    protected:
        friend FactoryP;
        CHoloNetManager ();
    private:
        virtual ~CHoloNetManager ();

    //
    // IUnknown
    //
    public:
        STDMETHODIMP QueryInterface
            (
                REFIID riid,
                __deref_out void **ppv
            );
        STDMETHODIMP_(ULONG) AddRef ();
        STDMETHODIMP_(ULONG) Release ();

    //
    // INetManager
    //
    public:
        HRESULT STDMETHODCALLTYPE Startup ();
        HRESULT STDMETHODCALLTYPE CreateConnection
            (
                LPCWSTR url,
                INetConnection** connectionOut
            );
        HRESULT STDMETHODCALLTYPE Shutdown ();

    protected:
        LONG m_cRef;
        enum { eStopped, eStarted, eStopping } m_eState;
        bool m_bSocketsInitialized;
        HANDLE m_hIocp;
        LONG m_cThreads;
        HANDLE m_hWorkerThread[s_dwConcurrentThreads];
        HRESULT m_lastError[s_dwConcurrentThreads];

        //
        // worker threads
        //
    protected:
        HRESULT StartWorkerThreads ();
        HRESULT StopWorkerThreads ();
        static DWORD WINAPI ThreadStart(void* param);
        HRESULT ThreadMain();
    };
}
