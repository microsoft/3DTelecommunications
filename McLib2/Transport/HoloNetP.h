///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

//
// Internal/private interfaces for HoloNet
//

#pragma once

namespace HoloNet
{

    [uuid(ab74f9f2-0cc9-427f-be59-18e0891e7749)]
    interface INetDatagram :
        public IUnknown
    {
        virtual ULONG STDMETHODCALLTYPE GetSize () PURE;
        virtual BYTE* STDMETHODCALLTYPE GetBuffer () PURE;
        virtual HRESULT STDMETHODCALLTYPE SetLength (ULONG length ) PURE;
        virtual ULONG STDMETHODCALLTYPE GetLength () PURE;
    };

    [uuid(d5cd475e-9c45-4819-b091-dd7391a5fc6c)]
    interface INetSink :
        public IUnknown
    {
        virtual void STDMETHODCALLTYPE Received (INetDatagram* datagram) PURE;
        virtual void STDMETHODCALLTYPE Notify (HRESULT result) PURE;
    };


    [uuid(39cece0e-6698-4376-ab56-7c92476f4608)]
    interface INetConnection :
        public IUnknown
    {
        virtual HRESULT STDMETHODCALLTYPE AddSink (INetSink* sink) PURE;
        virtual HRESULT STDMETHODCALLTYPE Start () PURE;
        virtual HRESULT STDMETHODCALLTYPE CreateDatagram (INetDatagram** datagramOut) PURE;
        virtual HRESULT STDMETHODCALLTYPE SendDatagram (INetDatagram* datagram) PURE;
        virtual HRESULT STDMETHODCALLTYPE Stop () PURE;
    };

    [uuid(e5c90de4-bdcd-4ab7-b18e-2e4d0d5af441)]
    interface INetManager :
        public IUnknown
    {
        virtual HRESULT STDMETHODCALLTYPE Startup () PURE;
        virtual HRESULT STDMETHODCALLTYPE CreateConnection
            (
                LPCWSTR url,
                INetConnection** connectionOut
            ) PURE;
        virtual HRESULT STDMETHODCALLTYPE Shutdown () PURE;
    };

    [uuid(41816ec6-b381-4b3d-b60d-1d6d171d1c3c)]
    interface IInternalIO :
        public IUnknown
    {
        // Caller passes in results from GetQueuedCompletionStatus
        virtual HRESULT Process
            (
                BOOL result,
                void* context,  // reinterpret_cast<LPOVERLAPPED>
                ULONG cbBytes
            ) PURE;
    };

    class FactoryP
    {
    public:
        static HRESULT WINAPI CreateNetManager
            (
                INetManager** managerOut
            );

        static HRESULT WINAPI CreateMulticastConnection
            (
                LPCWSTR address,
                USHORT port,
                HANDLE hIocp,
                INetConnection** connectionOut
            );
    };

}
