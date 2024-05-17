///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

//
// Public interfaces for HoloNet
//

#pragma once

namespace HoloNet
{

    [uuid(dbb3501e-2cc0-4d24-b8b9-68c8a9999c63)]
    interface INetSessionSink :
        public IUnknown
    {
        virtual void STDMETHODCALLTYPE NodeAppeared () PURE;
        virtual void STDMETHODCALLTYPE PacketReceived
            (
                REFIID from,
                BYTE packetType,
                const BYTE* p,
                ULONG cb
            ) PURE;
        virtual void STDMETHODCALLTYPE MessageReceived
            (
                REFIID from,
                BYTE messageType,
                const BYTE* p,
                ULONG cb
            ) PURE;
    };

    [uuid(ef934c7f-f2c3-4f05-937b-f7f61053a3a8)]
    interface INetSession :
        public IUnknown
    {
        virtual HRESULT STDMETHODCALLTYPE AddSink (INetSessionSink* sink) PURE;
        virtual HRESULT STDMETHODCALLTYPE Startup () PURE;
        virtual BOOL STDMETHODCALLTYPE IsConnected () PURE;
        virtual BOOL STDMETHODCALLTYPE IsPresent (REFIID id) PURE;
        virtual HRESULT STDMETHODCALLTYPE SendPacket
            (
                BYTE packetType,
                const BYTE* pHeader,
                ULONG cbHeader,
                const BYTE* pPayload,
                ULONG cbPayload
            ) PURE;
        virtual HRESULT STDMETHODCALLTYPE SendMessage
            (
                BYTE messageType,
                const BYTE* pHeader,
                ULONG cbHeader,
                const BYTE* pPayload,
                ULONG cbPayload
            ) PURE;
        virtual HRESULT STDMETHODCALLTYPE Shutdown () PURE;
    };

    class Factory
    {
    public:
        static HRESULT WINAPI CreateNetSession
            (
                LPCWSTR url,
                REFIID id,
                BOOL allowLoopback,
                DWORD aliveTime,
                INetSession** sessionOut
            );
    };

}
