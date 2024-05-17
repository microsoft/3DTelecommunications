///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#include "sr_stdafx.h"
#include "ConnectionManager.h"
#include "TCPListener.h"
#include "TCPConnection.h"

// 
// ctor/init
// 

HRESULT WINAPI 
CTcpListener::CreateInstance
    (
        class CConnectionManager* connectionManager,
        USHORT port,
        HANDLE hIocp,
        CTcpListener** listenerOut
    )
{
    if ( NULL == listenerOut )
        return E_POINTER;
    *listenerOut = NULL;

    CTcpListener* listener = new CTcpListener
                                    (
                                        connectionManager,
                                        port,
                                        hIocp
                                    );
    if ( NULL == listener )
        return E_OUTOFMEMORY;

    HRESULT hr = listener->InitInstance();
    if ( FAILED(hr) )
    {
        delete listener;
    }
    else
    {
      *listenerOut = listener;
    }

    return hr;
}

CTcpListener::CTcpListener
    (
        CConnectionManager* connectionManager,
        USHORT port,
        HANDLE hIocp
    ) :
    m_connectionManager(connectionManager),
    m_port(port),
    m_hIocp(hIocp),
    m_sock(INVALID_SOCKET),
    m_lpfnAcceptEx(NULL),
    m_lpfnGetAcceptExSockaddrs(NULL)
{
}


HRESULT 
CTcpListener::InitInstance ()
{
    GUID GuidAcceptEx = WSAID_ACCEPTEX;
    GUID GuidGetAcceptExSockaddrs = WSAID_GETACCEPTEXSOCKADDRS;
    DWORD dwBytes = 0;

    HRESULT hr = E_INVALIDARG;

    // Validate args
    if ( NULL == m_hIocp || INVALID_HANDLE_VALUE == m_hIocp )
        goto bailout;
    hr = S_OK;  // valid args

    // Init socket
    hr = CreateSocket(m_sock);
    if ( FAILED(hr) )
        goto bailout;

    // load the AcceptEx function

    if ( SOCKET_ERROR == ::WSAIoctl
                            (
                                m_sock,
                                SIO_GET_EXTENSION_FUNCTION_POINTER,
                                &GuidAcceptEx, 
                                sizeof(GuidAcceptEx),
                                &m_lpfnAcceptEx, 
                                sizeof(m_lpfnAcceptEx),
                                &dwBytes,
                                NULL,
                                NULL
                            ) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // load the GetAcceptExSockaddrs function

    if ( SOCKET_ERROR == ::WSAIoctl
                            (
                                m_sock,
                                SIO_GET_EXTENSION_FUNCTION_POINTER,
                                &GuidGetAcceptExSockaddrs,
                                sizeof(GuidGetAcceptExSockaddrs),
                                &m_lpfnGetAcceptExSockaddrs,
                                sizeof(m_lpfnGetAcceptExSockaddrs),
                                &dwBytes,
                                NULL,
                                NULL
                            ) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }
	{
		// add thyself to the iocp
		HANDLE newIocp = ::CreateIoCompletionPort
		(
			(HANDLE)m_sock,
			m_hIocp,
			reinterpret_cast<ULONG_PTR>(static_cast<IInternalIO*>(this)),
			0
		);
		if (NULL == newIocp)
		{
			hr = HRESULT_FROM_WIN32(::GetLastError());
			goto bailout;
		}
	}
bailout:
    return hr;
}


CTcpListener::~CTcpListener ()
{
    if ( INVALID_SOCKET != m_sock )
        ::closesocket(m_sock);
}

// --------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE 
CTcpListener::Startup ()
{
    HRESULT hr = S_OK;

    // bind
    SOCKADDR_STORAGE sockname;
    int namelen = sizeof(sockname);
    ((SOCKADDR_IN*)&sockname)->sin_family = AF_INET;
    ((SOCKADDR_IN*)&sockname)->sin_port = htons(m_port);
    ((SOCKADDR_IN*)&sockname)->sin_addr.S_un.S_addr = INADDR_ANY;

    if ( SOCKET_ERROR == ::bind
                            (
                                m_sock,
                                (SOCKADDR*)&sockname,
                                namelen
                            ) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // listen
    if ( SOCKET_ERROR == ::listen(m_sock, SOMAXCONN) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // retrieve our port number
    if ( SOCKET_ERROR == ::getsockname(m_sock, (SOCKADDR*)&sockname, &namelen) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // post the first accept
    hr = StartAsyncAccept();
    if ( FAILED(hr) )
        goto bailout;

bailout:
    return hr;
}

HRESULT STDMETHODCALLTYPE 
CTcpListener::Shutdown ()
{
    if ( INVALID_SOCKET == m_sock )
        return E_ABORT;

    ::closesocket(m_sock);
    m_sock = INVALID_SOCKET;

    return S_OK;
}

// --------------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE 
CTcpListener::Process
    (
        BOOL result,
        void* context,
        ULONG cbBytes
    )
{
    HRESULT hr = S_OK;
    CTcpListenerContext* pAcceptContext = 
        static_cast<CTcpListenerContext*>(reinterpret_cast<WSAOVERLAPPED*>(context)); 
    context = NULL; // ensure it won't get used again
    CTcpConnection* pConnection = NULL;
    UNREFERENCED_PARAMETER(cbBytes);

    if ( !result )
    // acceptEx failed, clean up and retry
    {
        hr = HRESULT_FROM_WIN32(WSAENOTCONN);
        ::closesocket(pAcceptContext->GetAcceptSocket());
        goto next;
    }
    else
    // acceptEx succeeded
    {
        // retrieve addresses
        LPSOCKADDR pLocalSockaddr, pRemoteSockaddr;
        INT localAddrlen, remoteAddrlen;
        m_lpfnGetAcceptExSockaddrs // match AcceptEx args!
            (
                pAcceptContext->GetOutputBuffer(),
                pAcceptContext->GetReceiveDataLength(),
                pAcceptContext->GetLocalAddressLength(),
                pAcceptContext->GetRemoteAddressLength(),
                &pLocalSockaddr,
                &localAddrlen,
                &pRemoteSockaddr,
                &remoteAddrlen
            );

        // inherit properties from the listening socket
        if ( SOCKET_ERROR == ::setsockopt
                                (
                                    pAcceptContext->GetAcceptSocket(),
                                    SOL_SOCKET,
                                    SO_UPDATE_ACCEPT_CONTEXT,
                                    (char*)&m_sock,
                                    sizeof(m_sock)
                                ) )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
            ::closesocket(pAcceptContext->GetAcceptSocket());
            goto next;
        }

        // create a connection
        hr = CTcpConnection::CreateInstance(m_connectionManager->m_compressor, m_hIocp, pAcceptContext->GetAcceptSocket(), &pConnection);
        if ( FAILED(hr) )
        {
            ::closesocket(pAcceptContext->GetAcceptSocket());
            goto next;
        }
        // the connection object owns the accept socket after this line

        // start and register the connection object
        hr = m_connectionManager->StartAndRegisterConnection(pConnection);
        if ( FAILED(hr) )
        {
            goto next;
        }

        pConnection = NULL;
    }

next:
    // post subsequent accept
    HRESULT hr1 = StartAsyncAccept();
    hr = FAILED(hr) ? hr : hr1;
    if ( NULL != pConnection )
    {
        delete pConnection;
    }
    return hr;
} // CTcpListener::Process


// --------------------------------------------------------------------------
//
// utility
//

HRESULT 
CTcpListener::CreateSocket
    (
        SOCKET& sock
    )
{
    HRESULT hr = S_OK;

    SHORT family = AF_INET;

    // create socket
    sock = ::WSASocketW
            (
                family,
                SOCK_STREAM,
                IPPROTO_IP,
                NULL,
                0,
                WSA_FLAG_OVERLAPPED
            );
    if ( INVALID_SOCKET == sock )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }
	{
		// send buffering
		DWORD dwSndBuf = 1024 * 1024;
		if ( SOCKET_ERROR == ::setsockopt
								(
									sock,
									SOL_SOCKET,
									SO_SNDBUF,
									(char*)&dwSndBuf,
									sizeof(dwSndBuf)
								) )
		{
			hr = HRESULT_FROM_WIN32(::WSAGetLastError());
			goto bailout;
		}
	
		// disable Nagle
		BOOL bSetOption = 1;
		if (SOCKET_ERROR == ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)& bSetOption, sizeof(bSetOption)))
		{
			hr = HRESULT_FROM_WIN32(::WSAGetLastError());
			goto bailout;
		}
	}

bailout:
    return hr;
}


HRESULT 
CTcpListener::StartAsyncAccept ()
{
    HRESULT hr = S_OK;
    SOCKET acceptSock = INVALID_SOCKET;

    // create an accept socket
    hr = CreateSocket(acceptSock);
    if ( FAILED(hr) )
        goto bailout;
	{
		// prep the overlapped context for AcceptEx
		m_context.Reset(acceptSock);

		// accept
		DWORD dwBytes = 0;
		if (!m_lpfnAcceptEx
		(
			m_sock,
			m_context.GetAcceptSocket(),
			m_context.GetOutputBuffer(),
			m_context.GetReceiveDataLength(),
			m_context.GetLocalAddressLength(),
			m_context.GetRemoteAddressLength(),
			&dwBytes,
			m_context.GetPOverlapped()
		)
			&& ERROR_IO_PENDING != ::WSAGetLastError())
		{
			hr = HRESULT_FROM_WIN32(::WSAGetLastError());
			::closesocket(acceptSock);
			goto bailout;
		}
	}
bailout:
    return hr;
}
