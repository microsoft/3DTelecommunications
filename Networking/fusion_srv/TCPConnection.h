///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once
#include "LinkedList.h"

///////////////////////////////////////////////////////////////////////////////

// Overlapped wrapper

class CTcpConnectionContext : public WSAOVERLAPPED
{
    //
    // member variables
    //
protected:
    bool m_receivedElseSent;
    ULONG m_cbSize;
    ULONG m_cbRead;
    WSABUF m_wsabuf;
    DWORD m_payload;

    //
    // ctor
    //
public:
    CTcpConnectionContext (bool receivedElseSent) :
        m_receivedElseSent(receivedElseSent)
    {
        Reset();
    }

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
        m_cbSize = 0;
        m_cbRead = 0;
        m_wsabuf.len = sizeof(m_payload);
        m_wsabuf.buf = (char*)&m_payload;
        m_payload = 0;
    }
    inline bool ReceivedElseSent ()
    {
        return m_receivedElseSent;
    }
    inline LPWSABUF GetWsaBuffers ()
    {
        return &m_wsabuf;
    } // WsaBuffers
    inline DWORD GetWsaBufferCount ()
    {
        return 1;
    } // WsaBufferCount
    inline LPWSAOVERLAPPED GetPOverlapped ()
    {
        return static_cast<WSAOVERLAPPED*>(this);
    } // POverlapped

    //
    // receive
    //
public:
    HRESULT PrepareRecv ();
    HRESULT RecvResult
        (
            ULONG cbBytes
        );
    inline DWORD GetRequestedFrame ()
    {
        return m_payload;
    }

    //
    // send
    //
public:
    HRESULT PrepareSend
        (
            BYTE* pData,
            DWORD cbData
        );
    HRESULT SendResult
        (
            ULONG cbBytes
        );
};

///////////////////////////////////////////////////////////////////////////////

class CTcpConnection :
    public IInternalIO
{
    //
    // ctor
    //
public:
    static HRESULT WINAPI CreateInstance
        (
			interface ICompressor* compressor,
            HANDLE hIocp,
            SOCKET sock,
            CTcpConnection** connectionOut
        );
    virtual ~CTcpConnection ();
protected:
    CTcpConnection
        (
			interface ICompressor* compressor,
            HANDLE hIocp,
            SOCKET sock
        );
    HRESULT InitInstance ();

public:
    static LONG GetConnectionCount();
    HRESULT STDMETHODCALLTYPE Startup ();
    HRESULT STDMETHODCALLTYPE Shutdown ();

public:
    CLinkEntry LinkEntry;

    //
    // IInternalIO
    //
public:
    HRESULT STDMETHODCALLTYPE Process
        (
            BOOL result,
            void* context,
            ULONG cbBytes
        );

    //
    // member variables
    //
protected:
    ULONG m_cRef;
	interface ICompressor* m_compressor;
    HANDLE m_hIocp;
    SOCKET m_sock;
    CTcpConnectionContext m_inboundContext;
    CTcpConnectionContext m_outboundContext;
    DWORD m_lastFrameSent;
	const BYTE* m_zero_buffer;

public:

    //
    // utility
    //
protected:
    HRESULT StartAsyncRecv (bool newTransaction);
    HRESULT ProcessRequest
        (
            DWORD dwRequestedFrame
        );
    HRESULT ProcessRecv
        (
            BOOL result,
            CTcpConnectionContext* context,
            ULONG cbBytes
        );
    HRESULT ProcessSend
        (
            BOOL result,
            CTcpConnectionContext* context,
            ULONG cbBytes
        );
};
