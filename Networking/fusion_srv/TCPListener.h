///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mswsock.h>

///////////////////////////////////////////////////////////////////////////////

// Overlapped wrapper for AcceptEx

class CTcpListenerContext : public WSAOVERLAPPED
{
    //
    // member variables
    //
protected:
    static const size_t c_addrLen = sizeof(SOCKADDR_STORAGE)+16; // see AcceptEx
    SOCKET m_sock;
    BYTE m_buffer[2*c_addrLen];

    //
    // ctor
    //
public:
    CTcpListenerContext ()
    {
        Reset(INVALID_SOCKET);
    }

public:
    inline void Reset
        (
            SOCKET sock
        )
    {
        Internal = 0;
        InternalHigh = 0;
        Offset = 0;
        OffsetHigh = 0;
        hEvent = NULL;
        m_sock = sock;
        ::ZeroMemory(m_buffer, sizeof(m_buffer));
    }

    //
    // accessors for AcceptEx
    //
public:
    inline SOCKET GetAcceptSocket ()
    {
        return m_sock;
    } // AcceptSocket
    inline PVOID GetOutputBuffer ()
    {
        return m_buffer;
    } // OutputBuffer
    inline DWORD GetReceiveDataLength ()
    {
        return 0;
    } // ReceiveDataLength
    inline DWORD GetLocalAddressLength ()
    {
        return c_addrLen;
    } // LocalAddressLength
    inline DWORD GetRemoteAddressLength ()
    {
        return c_addrLen;
    } // RemoteAddressLength
    inline LPWSAOVERLAPPED GetPOverlapped ()
    {
        return static_cast<WSAOVERLAPPED*>(this);
    } // POverlapped
};


///////////////////////////////////////////////////////////////////////////////

class CTcpListener : public IInternalIO
{
    //
    // ctor
    //
public:
    static HRESULT WINAPI CreateInstance
        (
            class CConnectionManager* connectionManager,
            USHORT port,
            HANDLE hIocp,
            CTcpListener** listenerOut
        );
    virtual ~CTcpListener ();
protected:
    CTcpListener
        (
            CConnectionManager* connectionManager,
            USHORT port,
            HANDLE hIocp
        );
    HRESULT InitInstance ();

public:
    HRESULT STDMETHODCALLTYPE Startup ();
    HRESULT STDMETHODCALLTYPE Shutdown ();

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
    CConnectionManager* m_connectionManager;
    USHORT m_port;
    HANDLE m_hIocp;
    SOCKET m_sock;
    LPFN_ACCEPTEX m_lpfnAcceptEx;
    LPFN_GETACCEPTEXSOCKADDRS m_lpfnGetAcceptExSockaddrs;
    CTcpListenerContext m_context;

    //
    // utility
    //
protected:
    HRESULT CreateSocket
        (
            SOCKET& sock
        );
    HRESULT StartAsyncAccept ();

};
