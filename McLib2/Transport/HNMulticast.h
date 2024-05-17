///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once

namespace HoloNet
{
    class CMulticastTransport :
        public INetConnection,
        public IInternalIO
    {
    protected:
        friend FactoryP;
        CMulticastTransport ();
    private:
        virtual ~CMulticastTransport ();

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
    // INetConnection
    //
    public:
        HRESULT STDMETHODCALLTYPE AddSink (INetSink* sink);
        HRESULT STDMETHODCALLTYPE Start ();
        HRESULT STDMETHODCALLTYPE CreateDatagram (INetDatagram** datagramOut);
        HRESULT STDMETHODCALLTYPE SendDatagram (INetDatagram* datagram);
        HRESULT STDMETHODCALLTYPE Stop ();

    //
    // IInternalIO
    //
    public:
    HRESULT Process
        (
            BOOL result,
            void* context,  // reinterpret_cast<LPOVERLAPPED>
            ULONG cbBytes
        );

    protected:
        HRESULT Setup
                (
                    LPCWSTR address,
                    USHORT port,
                    HANDLE hIocp
                );
        HRESULT PostReceive ();

    protected:
        LONG m_cRef;
        HANDLE m_hIocp;
        SOCKET m_txSocket;
        SOCKET m_rxSocket;
        SOCKADDR_STORAGE m_localTxAddress;
        SOCKADDR_STORAGE m_localRxAddress;
        SOCKADDR_STORAGE m_multicastAddress;
        INetSink* m_sink;
    };

    [uuid(7bf04a37-42af-4737-b290-e0baba7b3fd5)]
    class CMulticastTransportOverlapped :
        public WSAOVERLAPPED,
        public INetDatagram
    {
        //
        // member variables
        //
    protected:
        LONG m_cRef;
        bool m_receivedElseSent;
        // single datagram
        WSABUF m_wsabuf;
        BYTE m_payload[1472]; // use 1448 for ipv6 packets
        DWORD m_cbLength;

        //
        // ctor
        //
    public:
        CMulticastTransportOverlapped (bool receivedElseSent) :
            m_cRef(1),
            m_receivedElseSent(receivedElseSent)
        {
            Reset();
        }
    private:
        ~CMulticastTransportOverlapped ()
        {
        }

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
    // INetDatagram
    //
    public:
        virtual ULONG STDMETHODCALLTYPE GetSize ();
        virtual BYTE* STDMETHODCALLTYPE GetBuffer ();
        virtual HRESULT STDMETHODCALLTYPE SetLength (ULONG length );
        virtual ULONG STDMETHODCALLTYPE GetLength ();

    //
    // public interface
    //
    public:
        inline void Reset ()
        {
            Internal = 0;
            InternalHigh = 0;
            Offset = 0;
            OffsetHigh = 0;
            hEvent = NULL;
            m_cbLength = 0;
            m_wsabuf.len = 0;
            m_wsabuf.buf = NULL;
        }
        inline bool Received ()
        {
            return m_receivedElseSent;
        }
        inline bool Sent ()
        {
            return !m_receivedElseSent;
        }
        inline LPWSABUF GetWsaBuffers ()
        {
            m_wsabuf.buf = (char*)&m_payload;
            m_wsabuf.len = GetLength();
            return &m_wsabuf;
        }
        inline DWORD GetWsaBufferCount ()
        {
            return 1;
        }
    };

}
