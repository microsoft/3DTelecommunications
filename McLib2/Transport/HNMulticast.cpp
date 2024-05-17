///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///


#include "stdafx.h"
#include "HoloNet.h"
#include "HoloNetP.h"
#include "HNMulticast.h"

using namespace HoloNet;


#define TX_SOCKET_BUFFER (4 * 1024 * 1024)
#define RX_SOCKET_BUFFER (32 * 1024 * 1024)

// --------------------------------------------------------------------------
// 
// ctor/init
//

HRESULT WINAPI FactoryP::CreateMulticastConnection
    (
        LPCWSTR address,
        USHORT port,
        HANDLE hIocp,
        INetConnection** connectionOut
    )
{
    HRESULT hr = S_OK;
    CMulticastTransport* pConnection = NULL;

    if ( NULL == connectionOut )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    pConnection = new CMulticastTransport();
    if ( NULL == pConnection )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    hr = pConnection->Setup(address, port, hIocp);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = pConnection->QueryInterface(__uuidof(INetConnection), (void**)connectionOut);

bailout:
    if ( NULL != pConnection )
    {
        pConnection->Release();
    }
    return hr;
}

CMulticastTransport::CMulticastTransport () : 
    m_cRef(1),
    m_hIocp(NULL),
    m_txSocket(INVALID_SOCKET),
    m_rxSocket(INVALID_SOCKET),
    m_sink(NULL)
{
}

CMulticastTransport::~CMulticastTransport ()
{
    Stop();
}


// --------------------------------------------------------------------------
//
// IUnknown
//

STDMETHODIMP CMulticastTransport::QueryInterface
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
        *ppv = static_cast<INetConnection*>(this);
        AddRef();
    }
    else
    if ( __uuidof(INetConnection) == riid )
    {
        *ppv = static_cast<INetConnection*>(this);
        AddRef();
    }
    else
    if ( __uuidof(IInternalIO) == riid )
    {
        *ppv = static_cast<IInternalIO*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CMulticastTransport::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CMulticastTransport::Release ()
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
// INetConnection
//

HRESULT STDMETHODCALLTYPE CMulticastTransport::AddSink (INetSink* sink)
{
    if ( NULL != sink )
    {
        sink->AddRef();
    }
    if ( NULL != m_sink )
    {
        m_sink->Release();
    }

    m_sink = sink;

    return S_OK;
}

HRESULT STDMETHODCALLTYPE CMulticastTransport::Start ()
{
    HRESULT hr = PostReceive();
    return hr;
}

HRESULT STDMETHODCALLTYPE CMulticastTransport::CreateDatagram (INetDatagram** datagramOut)
{
    HRESULT hr = S_OK;

    if ( NULL == datagramOut )
    {
        hr = E_POINTER;
        goto bailout;
    }

    CMulticastTransportOverlapped* pOverlapped = new CMulticastTransportOverlapped(false);
    if ( NULL == pOverlapped )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    hr = pOverlapped->QueryInterface(__uuidof(INetDatagram), (void**)datagramOut);
    pOverlapped->Release();

bailout:
    return hr;
}

HRESULT STDMETHODCALLTYPE CMulticastTransport::SendDatagram (INetDatagram* datagram)
{
    HRESULT hr = S_OK;
    CMulticastTransportOverlapped* pOverlapped = NULL;

    if ( NULL == datagram )
    {
        hr = E_POINTER;
        goto bailout;
    }

    // retrieve back the object pointer
    hr = datagram->QueryInterface(__uuidof(CMulticastTransportOverlapped), (void**)&pOverlapped);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    if ( !pOverlapped->Sent() )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( SOCKET_ERROR == ::WSASendTo
                            (
                                m_txSocket,
                                pOverlapped->GetWsaBuffers(),
                                pOverlapped->GetWsaBufferCount(),
                                NULL,
                                0,
                                (SOCKADDR*)&m_multicastAddress,
                                sizeof(m_multicastAddress),
                                static_cast<WSAOVERLAPPED*>(pOverlapped),
                                NULL
                            )
            && WSA_IO_PENDING != ::WSAGetLastError() )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // keep the reference until the I/O completes
    pOverlapped = NULL;

bailout:
    if ( NULL != pOverlapped )
    {
        pOverlapped->Release();
    }
    return hr;
}

HRESULT STDMETHODCALLTYPE CMulticastTransport::Stop ()
{
    if ( INVALID_SOCKET != m_rxSocket )
    {
        int result;
        result = ::shutdown(m_rxSocket, SD_BOTH);
        result = ::closesocket(m_rxSocket);
        m_rxSocket = INVALID_SOCKET;
    }

    if ( INVALID_SOCKET != m_txSocket )
    {
        int result;
        result = ::shutdown(m_txSocket, SD_BOTH);
        result = ::closesocket(m_txSocket);
        m_txSocket = INVALID_SOCKET;
    }

    if ( NULL != m_sink )
    {
        m_sink->Release();
        m_sink = NULL;
    }

    return S_OK;
}


// --------------------------------------------------------------------------
//
// IInternalIO
//

HRESULT CMulticastTransport::Process
    (
        BOOL result,
        void* context,  // reinterpret_cast<LPOVERLAPPED>
        ULONG cbBytes
    )
{
    HRESULT hr = S_OK;
    CMulticastTransportOverlapped* pOverlapped = static_cast<CMulticastTransportOverlapped*>(reinterpret_cast<WSAOVERLAPPED*>(context));

    if ( !result || 0 == cbBytes )
    {
        hr = HRESULT_FROM_WIN32(WSAECONNABORTED);
        goto bailout;
    }

    if ( NULL == pOverlapped )
    {
        hr = E_UNEXPECTED;
        goto bailout;
    }

    if ( pOverlapped->Received() )
    {
        hr = pOverlapped->SetLength(cbBytes);
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        // dispatch
        if ( NULL != m_sink )
        {
            m_sink->Received(static_cast<INetDatagram*>(pOverlapped));
        }

        // post the next one
        hr = PostReceive();
    }
    else
    {
        if ( cbBytes == pOverlapped->GetLength() )
        {
            hr = S_OK;
        }
        else
        {
            hr = STG_E_WRITEFAULT;
        }
    }

bailout:
    if ( FAILED(hr) && NULL != m_sink )
    {
        // dispatch the error
        m_sink->Notify(hr);
    }
    if ( NULL != pOverlapped )
    {
        pOverlapped->Release();
    }
    return hr;
}


// --------------------------------------------------------------------------
//
// Utility
//

HRESULT CMulticastTransport::Setup
        (
            LPCWSTR address,
            USHORT port,
            HANDLE hIocp
        )
{
    HRESULT hr = S_OK;

    if ( NULL == address
         || NULL == hIocp )
    {
        hr = E_POINTER;
        goto bailout;
    }

    if ( 0 == port )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    //
    // Resolve the multicast address
    //

    // try IPv4
    INT addrLength = sizeof(m_multicastAddress);
    m_multicastAddress.ss_family = AF_INET;
    if ( 0 != WSAStringToAddress(const_cast<LPWSTR>(address), AF_INET, NULL, reinterpret_cast<LPSOCKADDR>(&m_multicastAddress), &addrLength) )
    {
        // then try IPv6
        addrLength = sizeof(m_multicastAddress);
        m_multicastAddress.ss_family = AF_INET6;
        if ( 0 != WSAStringToAddress(const_cast<LPWSTR>(address), AF_INET6, NULL, reinterpret_cast<LPSOCKADDR>(&m_multicastAddress), &addrLength) )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
            goto bailout;
        }
    }

    ADDRESS_FAMILY family = m_multicastAddress.ss_family;
    if ( AF_INET == family )
    {
        ((SOCKADDR_IN*)&m_multicastAddress)->sin_port = htons(port);

        ((SOCKADDR_IN*)&m_localTxAddress)->sin_family = AF_INET;
        ((SOCKADDR_IN*)&m_localTxAddress)->sin_port = 0; // let winsock pick the outbound port
        ((SOCKADDR_IN*)&m_localTxAddress)->sin_addr.S_un.S_addr = ADDR_ANY;

        ((SOCKADDR_IN*)&m_localRxAddress)->sin_family = AF_INET;
        ((SOCKADDR_IN*)&m_localRxAddress)->sin_port = htons(port);
        ((SOCKADDR_IN*)&m_localRxAddress)->sin_addr.S_un.S_addr = ADDR_ANY;
    }
    else
    if ( AF_INET6 == family )
    {
        ((SOCKADDR_IN6*)&m_multicastAddress)->sin6_port = htons(port);
        ((SOCKADDR_IN6*)&m_multicastAddress)->sin6_flowinfo = 0;
        ((SOCKADDR_IN6*)&m_multicastAddress)->sin6_scope_id = 0;

        ((SOCKADDR_IN6*)&m_localTxAddress)->sin6_family = AF_INET6;
        ((SOCKADDR_IN6*)&m_localTxAddress)->sin6_port = 0; // let winsock pick the outbound port
        ((SOCKADDR_IN6*)&m_localTxAddress)->sin6_flowinfo = 0;
        ((SOCKADDR_IN6*)&m_localTxAddress)->sin6_addr = in6addr_any;
        ((SOCKADDR_IN6*)&m_localTxAddress)->sin6_scope_id = 0;

        ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_family = AF_INET6;
        ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_port = htons(port);
        ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_flowinfo = 0;
        ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_addr = in6addr_any;
        ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_scope_id = 0;
    }
    else
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    //
    // Create and set up the transmit socket
    //

    m_txSocket = ::WSASocket(family, SOCK_DGRAM, IPPROTO_UDP, NULL, 0, WSA_FLAG_OVERLAPPED);
	if ( INVALID_SOCKET == m_txSocket )
	{
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
	}

    DWORD dwBuffer = TX_SOCKET_BUFFER;
    if ( SOCKET_ERROR == ::setsockopt(m_txSocket, SOL_SOCKET, SO_SNDBUF, (char*)&dwBuffer, sizeof(dwBuffer)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    DWORD multicastHops = 1;
    if ( SOCKET_ERROR == ::setsockopt
                            (
                                m_txSocket,
                                AF_INET == family ? IPPROTO_IP : IPPROTO_IPV6,
                                AF_INET == family ? IP_MULTICAST_TTL : IPV6_MULTICAST_HOPS,
                                (char*)&multicastHops,
                                sizeof(multicastHops)
                            ) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    if ( SOCKET_ERROR == ::bind(m_txSocket, (SOCKADDR*)&m_localTxAddress, sizeof(m_localTxAddress)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // query back the socket information
    int namelen = sizeof(m_localTxAddress);
    if ( SOCKET_ERROR == ::getsockname(m_txSocket, (SOCKADDR*)&m_localTxAddress, &namelen) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    //
    // Create and set up the multicast receive socket
    //

    m_rxSocket = ::WSASocket(family, SOCK_DGRAM, IPPROTO_UDP, NULL, 0, WSA_FLAG_OVERLAPPED);
	if ( INVALID_SOCKET == m_rxSocket )
	{
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
	}

    BOOL bReuseAddr = 1;
    if ( SOCKET_ERROR == ::setsockopt(m_rxSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&bReuseAddr, sizeof(bReuseAddr)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    dwBuffer = RX_SOCKET_BUFFER;
    if ( SOCKET_ERROR == ::setsockopt(m_rxSocket, SOL_SOCKET, SO_RCVBUF, (char*)&dwBuffer, sizeof(dwBuffer)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    DWORD dwMulticastLoop = 1;
    if ( SOCKET_ERROR == ::setsockopt
                            (
                                m_rxSocket,
                                AF_INET == family ? IPPROTO_IP : IPPROTO_IPV6,
                                AF_INET == family ? IP_MULTICAST_LOOP : IPV6_MULTICAST_LOOP,
                                (char*)&multicastHops,
                                sizeof(multicastHops)
                            ) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    if ( SOCKET_ERROR == ::bind(m_rxSocket, (SOCKADDR*)&m_localRxAddress, sizeof(m_localRxAddress)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // query back the socket information
    namelen = sizeof(m_localRxAddress);
    if ( SOCKET_ERROR == ::getsockname(m_rxSocket, (SOCKADDR*)&m_localRxAddress, &namelen) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    //
    // Join the multicast group
    //

    if ( AF_INET == family )
    {
        ip_mreq imr4;
        imr4.imr_multiaddr = ((SOCKADDR_IN*)&m_multicastAddress)->sin_addr;
        imr4.imr_interface = ((SOCKADDR_IN*)&m_localRxAddress)->sin_addr;

        if ( SOCKET_ERROR == ::setsockopt(m_rxSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&imr4, sizeof(imr4)) )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
            goto bailout;
        }
    }
    else
    if ( AF_INET6 == family )
    {
        ipv6_mreq imr6;
        imr6.ipv6mr_multiaddr = ((SOCKADDR_IN6*)&m_multicastAddress)->sin6_addr;
        imr6.ipv6mr_interface = ((SOCKADDR_IN6*)&m_localRxAddress)->sin6_scope_id;
        if ( SOCKET_ERROR == ::setsockopt(m_rxSocket, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, (char*)&imr6, sizeof(imr6)) )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
            goto bailout;
        }
    }

    //
    // Attach to the completion port
    //

    m_hIocp = ::CreateIoCompletionPort((HANDLE)m_txSocket, hIocp, reinterpret_cast<ULONG_PTR>(static_cast<IInternalIO*>(this)), 0);
    if ( m_hIocp != hIocp )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

    m_hIocp = ::CreateIoCompletionPort((HANDLE)m_rxSocket, hIocp, reinterpret_cast<ULONG_PTR>(static_cast<IInternalIO*>(this)), 0);
    if ( m_hIocp != hIocp )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

bailout:
    return hr;
}

HRESULT CMulticastTransport::PostReceive ()
{
    HRESULT hr = S_OK;

    CMulticastTransportOverlapped* pOverlapped = new CMulticastTransportOverlapped(true);
    if ( NULL == pOverlapped )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    hr = pOverlapped->SetLength(pOverlapped->GetSize());
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    DWORD dwFlags = 0;
    if ( SOCKET_ERROR == ::WSARecv
                            (
                                m_rxSocket,
                                pOverlapped->GetWsaBuffers(),
                                pOverlapped->GetWsaBufferCount(),
                                NULL,
                                &dwFlags,
                                static_cast<WSAOVERLAPPED*>(pOverlapped),
                                NULL
                            )
            && WSA_IO_PENDING != ::WSAGetLastError() )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // keep the reference until the I/O completes
    pOverlapped = NULL;

bailout:
    if ( NULL != pOverlapped )
    {
        pOverlapped->Release();
    }
    return hr;
}


// --------------------------------------------------------------------------
//
// CMulticastTransportOverlapped
//

//
// IUnknown
//

STDMETHODIMP CMulticastTransportOverlapped::QueryInterface
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
        *ppv = static_cast<INetDatagram*>(this);
        AddRef();
    }
    else
    if ( __uuidof(INetDatagram) == riid )
    {
        *ppv = static_cast<INetDatagram*>(this);
        AddRef();
    }
    else
    if ( __uuidof(CMulticastTransportOverlapped) == riid )
    {
        // trick to retrieve the object pointer
        *ppv = static_cast<CMulticastTransportOverlapped*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CMulticastTransportOverlapped::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CMulticastTransportOverlapped::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
    if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}

//
// INetDatagram
//

ULONG STDMETHODCALLTYPE CMulticastTransportOverlapped::GetSize ()
{
    return sizeof(m_payload);
}

BYTE* STDMETHODCALLTYPE CMulticastTransportOverlapped::GetBuffer ()
{
    return m_payload;
}

HRESULT STDMETHODCALLTYPE CMulticastTransportOverlapped::SetLength (ULONG length )
{
    if ( length > sizeof(m_payload) )
    {
        return E_INVALIDARG;
    }

    m_cbLength = length;
    return S_OK;
}

ULONG STDMETHODCALLTYPE CMulticastTransportOverlapped::GetLength ()
{
    return m_cbLength;
}
