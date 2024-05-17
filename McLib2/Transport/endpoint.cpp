#include "stdafx.h"
#include "endpoint.h"
#include <Ws2ipdef.h>
#include <iphlpapi.h>
#include <ws2tcpip.h>

using namespace HolochatNetworking;

CEndpoint::CEndpoint() :
    m_bWinsockInit(false),
    m_socket(INVALID_SOCKET)
{
    Close();
}

CEndpoint::~CEndpoint()
{
}

HRESULT CEndpoint::Initialize()
{
    HRESULT hr = S_OK;

    // WSAStartup

    WSADATA wsadata;
    if ( 0 != ::WSAStartup(MAKEWORD(2,2), &wsadata) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }
    m_bWinsockInit = true;

bailout:
    return hr;
}

void CEndpoint::Close()
{
    if ( INVALID_SOCKET != m_socket )
    {
        ::closesocket(m_socket);
        m_socket = INVALID_SOCKET;
    }
    if ( m_bWinsockInit )
    {
        ::WSACleanup();
        m_bWinsockInit = false;
    }
}

static HRESULT ResolveTarget(PCTSTR targetName, SOCKADDR_STORAGE* targetAddrOut)
{
    HRESULT hr = E_UNEXPECTED;
    ADDRINFOT* pAddrInfo = NULL;
    ADDRINFOT hints = {0};

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    // Resolve the target
    INT wsResult = GetAddrInfo(targetName, NULL, &hints, &pAddrInfo);
    if ( 0 != wsResult )
    {
        hr = HRESULT_FROM_WIN32(wsResult);
        goto bailout;
    }

    // Out of the solutions, pick the best metric
    ULONG ulBestMetric = MAXINT;
    ADDRINFOT* pSelectedAddr = NULL;
    for ( ADDRINFOT* pTargetAddr  = pAddrInfo; NULL != pTargetAddr; pTargetAddr = pTargetAddr->ai_next )
    {
        ADDRESS_FAMILY family = pTargetAddr->ai_family;

        SOCKADDR_INET targetAddr = {0};
        targetAddr.si_family = family;
        if ( AF_INET == family )
        {
            targetAddr.Ipv4.sin_addr = ((SOCKADDR_IN*)pTargetAddr->ai_addr)->sin_addr;
        }
        else
        if ( AF_INET6 == family )
        {
            targetAddr.Ipv6.sin6_addr = ((SOCKADDR_IN6*)pTargetAddr->ai_addr)->sin6_addr;
            continue; // ensure that both sides pick a v4 address
        }
        else
        {
            continue;
        }

        DWORD dwBestIfIndex = 0;
        DWORD dwResult = ::GetBestInterfaceEx(pTargetAddr->ai_addr, &dwBestIfIndex);
        if ( NO_ERROR != dwResult )
        {
            continue;
        }

        MIB_IPINTERFACE_ROW ipInterfaceRow = {0};
        ipInterfaceRow.InterfaceIndex = dwBestIfIndex;
        ipInterfaceRow.Family = family;
        dwResult = ::GetIpInterfaceEntry(&ipInterfaceRow);
        if ( NO_ERROR != dwResult )
        {
            continue;
        }

        MIB_IPFORWARD_ROW2 ipForwardRow2 = {0};
        SOCKADDR_INET hostAddr;
        dwResult = GetBestRoute2( NULL, dwBestIfIndex, NULL, &targetAddr, 0, &ipForwardRow2, &hostAddr);
        if( NO_ERROR != dwResult )
        {
            continue;
        }

        ULONG ulMetric = ipInterfaceRow.Metric + ipForwardRow2.Metric;
        if ( 0 == ipForwardRow2.Metric )
        {
            // wireless adapters sometimes give back a bogus result,
            // set it relatively high
            ulMetric = ipInterfaceRow.Metric + 512;
        }
        if ( ulMetric < ulBestMetric )
        {
            // found a better match
            ulBestMetric = ulMetric;
            pSelectedAddr = pTargetAddr;
        }
    }

    if ( NULL == pSelectedAddr )
    {
        // Was not found
        hr = HRESULT_FROM_WIN32(ERROR_BAD_NETPATH);
        goto bailout;
    }

    if ( AF_INET == pSelectedAddr->ai_family)
    {
        // copy the v4 address
        *(SOCKADDR_IN*)targetAddrOut = *(SOCKADDR_IN*)pSelectedAddr->ai_addr;
        hr = S_OK;
    }
    else
    if ( AF_INET6 == pSelectedAddr->ai_family)
    {
        // copy the v6 address
        *(SOCKADDR_IN6*)targetAddrOut = *(SOCKADDR_IN6*)pSelectedAddr->ai_addr;
        hr = S_OK;
    }

bailout:
    if ( NULL != pAddrInfo )
    {
        FreeAddrInfo(pAddrInfo);
    }
    return hr;
}


//
// UDP
//

CUdpEndpoint::CUdpEndpoint() :
    m_hIOCP(NULL)
{
}

CUdpEndpoint::~CUdpEndpoint()
{
    Close();
}

HRESULT CUdpEndpoint::Initialize(USHORT localPort, PCTSTR targetName,  USHORT targetPort, DWORD cbBuffer, HANDLE hIOCP, ULONG_PTR key)
{
    HRESULT hr = S_OK;

    if ( NULL == hIOCP
         || INVALID_HANDLE_VALUE == hIOCP )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    hr = __super::Initialize();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    // Resolve

    hr = ResolveTarget(targetName, &m_remoteAddr);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    ADDRESS_FAMILY family = m_remoteAddr.ss_family;
    if ( AF_INET == family )
    {
        ((SOCKADDR_IN*)&m_localAddr)->sin_family = AF_INET;
        ((SOCKADDR_IN*)&m_localAddr)->sin_port = htons(localPort);
        ((SOCKADDR_IN*)&m_localAddr)->sin_addr.S_un.S_addr = ADDR_ANY;
        ((SOCKADDR_IN*)&m_remoteAddr)->sin_port = htons(targetPort);
    }
    else
    if ( AF_INET6 == family )
    {
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_family = AF_INET6;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_port = htons(localPort);
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_flowinfo = 0;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_addr = in6addr_any;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_scope_id = 0;
        ((SOCKADDR_IN6*)&m_remoteAddr)->sin6_port = htons(targetPort);
    }
    else
    {
        hr = E_UNEXPECTED;
        goto bailout;
    }

    // Set up the UDP socket

    m_socket = ::WSASocket(m_localAddr.ss_family, SOCK_DGRAM, IPPROTO_UDP, NULL, 0, WSA_FLAG_OVERLAPPED);
	if ( INVALID_SOCKET == m_socket )
	{
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
	}

    BOOL bSetOption = 1;
    if ( SOCKET_ERROR == ::setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&bSetOption, sizeof(bSetOption)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    if ( SOCKET_ERROR == ::setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, (char*)&cbBuffer, sizeof(cbBuffer)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    if ( SOCKET_ERROR == ::setsockopt(m_socket, SOL_SOCKET, SO_SNDBUF, (char*)&cbBuffer, sizeof(cbBuffer)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    if ( SOCKET_ERROR == ::bind(m_socket, (SOCKADDR*)&m_localAddr, sizeof(m_localAddr)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

#ifdef _DEBUG
    SOCKADDR_STORAGE localname;
    int localnamelen = sizeof(localname);
    ::getsockname(m_socket, (SOCKADDR*)&localname, &localnamelen);
    TCHAR localnamestring[128];
    DWORD localnamestringlen = sizeof(localnamestring);
    ::WSAAddressToString((SOCKADDR*)&localname, localnamelen, NULL, localnamestring, &localnamestringlen );

    TCHAR peernamestring[128];
    DWORD peernamestringlen = sizeof(peernamestring);
    int peernamelen = sizeof(m_remoteAddr);
    ::WSAAddressToString((SOCKADDR*)&m_remoteAddr, peernamelen, NULL, peernamestring, &peernamestringlen);
    OutputDebugString(peernamestring);
#endif

    // Attach to the IOCP

    m_hIOCP = ::CreateIoCompletionPort((HANDLE)m_socket, hIOCP, key, 1);
    if ( m_hIOCP != hIOCP )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

bailout:
    return hr;
}

void CUdpEndpoint::Close()
{
    __super::Close();
}

HRESULT CUdpEndpoint::PostAsyncSend(TransportPacket* pPacket)
{
    HRESULT hr = S_OK;
    DWORD cbSent = 0;

    if ( SOCKET_ERROR == ::WSASendTo
                            (
                                m_socket,
                                &pPacket->Wsabuf,
                                1,
                                &cbSent,
                                0,
                                (SOCKADDR*)&m_remoteAddr,
                                sizeof(m_remoteAddr),
                                pPacket,
                                NULL
                            )
            && WSA_IO_PENDING != ::WSAGetLastError() )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
    }

    return hr;
}

HRESULT CUdpEndpoint::PostAsyncReceive(TransportPacket* pPacket)
{
    HRESULT hr = S_OK;
    DWORD dwBytesRecvd = 0;
    DWORD dwFlags = 0;

    if ( SOCKET_ERROR == ::WSARecv
                        (
                            m_socket,
                            &pPacket->Wsabuf,
                            1,
                            &dwBytesRecvd,
                            &dwFlags,
                            pPacket,
                            NULL
                        )
        && WSA_IO_PENDING != ::WSAGetLastError() )
        {
            hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        }

    return hr;
}

//
// TCP
//

HRESULT WINAPI NetworkFactory::CreateTcpSocket
    (
        PCTSTR targetName,
        USHORT targetPort,
        ITcpSocket** socketOut
    )
{
    if ( NULL == socketOut )
    {
        return E_INVALIDARG;
    }

    CTcpEndpoint* endpoint = new CTcpEndpoint();
    if ( NULL == endpoint )
    {
        return E_OUTOFMEMORY;
    }

    HRESULT hr = endpoint->Initialize(targetName, targetPort);
    if ( FAILED(hr) )
        goto bailout;

    hr = endpoint->QueryInterface(__uuidof(ITcpSocket), (void**)socketOut);

bailout:
    endpoint->Release();
    return hr;
}

CTcpEndpoint::CTcpEndpoint() :
    m_cRef(1)
{
}

CTcpEndpoint::~CTcpEndpoint()
{
    Close();
}

HRESULT CTcpEndpoint::Initialize(PCTSTR targetName,  USHORT targetPort)
{
    HRESULT hr = S_OK;

    hr = __super::Initialize();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    // Resolve

    hr = ResolveTarget(targetName, &m_remoteAddr);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    ADDRESS_FAMILY family = m_remoteAddr.ss_family;
    if ( AF_INET == family )
    {
        ((SOCKADDR_IN*)&m_localAddr)->sin_family = AF_INET;
        ((SOCKADDR_IN*)&m_localAddr)->sin_port = 0;
        ((SOCKADDR_IN*)&m_localAddr)->sin_addr.S_un.S_addr = ADDR_ANY;
        ((SOCKADDR_IN*)&m_remoteAddr)->sin_port = htons(targetPort);
    }
    else
    if ( AF_INET6 == family )
    {
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_family = AF_INET6;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_port = 0;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_flowinfo = 0;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_addr = in6addr_any;
        ((SOCKADDR_IN6*)&m_localAddr)->sin6_scope_id = 0;
        ((SOCKADDR_IN6*)&m_remoteAddr)->sin6_port = htons(targetPort);
    }
    else
    {
        hr = E_UNEXPECTED;
        goto bailout;
    }

    // Set up the TCP socket

    m_socket = ::WSASocket(m_localAddr.ss_family, SOCK_STREAM, IPPROTO_TCP, NULL, 0, 0);
	if ( INVALID_SOCKET == m_socket )
	{
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
	}


    BOOL bSetOption = 1;
    if ( SOCKET_ERROR == ::setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, (char*)&bSetOption, sizeof(bSetOption)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    DWORD bufferSize = 1024 * 1024;
    if ( SOCKET_ERROR == ::setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, (char*)&bufferSize, sizeof(bufferSize)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    // Connect to the remote host

    if ( SOCKET_ERROR == ::connect(m_socket, (SOCKADDR*)&m_remoteAddr, sizeof(m_remoteAddr)) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

#ifdef _DEBUG
    SOCKADDR_STORAGE localname;
    int localnamelen = sizeof(localname);
    ::getsockname(m_socket, (SOCKADDR*)&localname, &localnamelen);
    TCHAR localnamestring[128];
    DWORD localnamestringlen = sizeof(localnamestring);
    ::WSAAddressToString((SOCKADDR*)&localname, localnamelen, NULL, localnamestring, &localnamestringlen );

    TCHAR peernamestring[128];
    DWORD peernamestringlen = sizeof(peernamestring);
    int peernamelen = sizeof(m_remoteAddr);
    ::WSAAddressToString((SOCKADDR*)&m_remoteAddr, peernamelen, NULL, peernamestring, &peernamestringlen);
    OutputDebugString(peernamestring);
#endif

bailout:
    return hr;
}

STDMETHODIMP CTcpEndpoint::QueryInterface
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
        *ppv = static_cast<ITcpSocket*>(this);
        AddRef();
    }
    else
    if ( __uuidof(ITcpSocket) == riid )
    {
        *ppv = static_cast<ITcpSocket*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CTcpEndpoint::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CTcpEndpoint::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
   if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP CTcpEndpoint::Send
    (
        const BYTE* payload,
        DWORD cbPayload
    )
{
    HRESULT hr = S_OK;

    if ( SOCKET_ERROR == ::send(m_socket, reinterpret_cast<const char*>(payload), cbPayload, 0) )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

bailout:
    return hr;
}

STDMETHODIMP CTcpEndpoint::Receive
    (
        BYTE* payload,
        DWORD* pcbPayload
    )
{
    HRESULT hr = S_OK;

    int received = ::recv(m_socket, reinterpret_cast<char*>(payload), *pcbPayload, 0);

    if ( SOCKET_ERROR == received )
    {
        hr = HRESULT_FROM_WIN32(::WSAGetLastError());
        goto bailout;
    }

    *pcbPayload = received;

bailout:
    return hr;
}

STDMETHODIMP CTcpEndpoint::Close()
{
    CEndpoint::Close();
    return S_OK;
}
