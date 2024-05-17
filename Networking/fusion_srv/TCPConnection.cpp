///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///


#include "sr_stdafx.h"
//#include "../includes/VoxelGrid.h"
//#include "../includes/VoxelCompress.h"
#include "ConnectionManager.h"
#include "TCPConnection.h"
#include "ICompressor.h"
#include "TimeUtil.h"
#include <iostream>
#include <mutex>

LONG g_connections = 0;
LONG g_requests = 0;
LONG g_responses = 0;
LONG g_payloads = 0;

// --------------------------------------------------------------------------
// 
// ctor/init
// 

HRESULT WINAPI 
CTcpConnection::CreateInstance
    (
		interface ICompressor* compressor,
        HANDLE hIocp,
        SOCKET sock,
        CTcpConnection** connectionOut
    )
{
    if ( NULL == connectionOut )
        return E_POINTER;
    *connectionOut = NULL;

    CTcpConnection* connection = new CTcpConnection(compressor, hIocp, sock);
    if ( NULL == connection )
        return E_OUTOFMEMORY;

    HRESULT hr = connection->InitInstance();

    if ( FAILED(hr) )
    {
        delete connection;
    }
    else
    {
      *connectionOut = connection;
    }

    return hr;
}


CTcpConnection::CTcpConnection 
    (
		interface ICompressor* compressor,
        HANDLE hIocp,
        SOCKET sock
    ) :
    m_cRef(1),
    m_compressor(compressor),
    m_hIocp(hIocp),
    m_sock(sock),
    m_inboundContext(true),
    m_outboundContext(false),
    m_lastFrameSent(0)
{
	m_zero_buffer = (BYTE*)calloc(12, sizeof(BYTE));
}

LONG CTcpConnection::GetConnectionCount()
{
    return g_connections;
}

HRESULT 
CTcpConnection::InitInstance ()
{
    HRESULT hr = E_INVALIDARG;
    if ( NULL == m_hIocp || INVALID_HANDLE_VALUE == m_hIocp )
        goto bailout;
    if ( NULL == m_sock || INVALID_SOCKET == m_sock )
        goto bailout;
    hr = S_OK;  // valid args
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


CTcpConnection::~CTcpConnection ()
{
    Shutdown();
}

HRESULT STDMETHODCALLTYPE 
CTcpConnection::Startup ()
{
    HRESULT hr = S_OK;

    // post the first read
    hr = StartAsyncRecv(true);
    if ( FAILED(hr) )
        goto bailout;

	g_connections++;

bailout:
    return hr;
}

HRESULT STDMETHODCALLTYPE 
CTcpConnection::Shutdown ()
{
    HRESULT hr = S_OK;

    if ( INVALID_SOCKET != m_sock )
    {
        ::closesocket(m_sock);
        m_sock = INVALID_SOCKET;
		g_connections--;
    }
	free((void*)m_zero_buffer);

    return hr;
}

// --------------------------------------------------------------------------
//
// Public interface
//

HRESULT STDMETHODCALLTYPE 
CTcpConnection::Process
    (
        BOOL result,
        void* context,
        ULONG cbBytes
    )
{
    HRESULT hr = S_OK;
    CTcpConnectionContext* ioContext = static_cast<CTcpConnectionContext*>(reinterpret_cast<WSAOVERLAPPED*>(context)); 
    context = NULL; // ensure it won't get used again
    TimingTable timing_table("TCP_connection_process");

    if ( ioContext->ReceivedElseSent() )
    // ----------------------------------------
    // WSARecv completed
    // ----------------------------------------
    {
        timing_table.Start("TCP_connection_process_if", "TCP connection received");

        hr = ProcessRecv(result, ioContext, cbBytes);
        timing_table.End("TCP_connection_process_if");

        if ( FAILED(hr) )
            goto bailout;
    }
    else
    // ----------------------------------------
    // WSASend completed
    // ----------------------------------------
    {
        timing_table.Start("TCP_connection_process_if", "TCP connection sent");

        hr = ProcessSend(result, ioContext, cbBytes);

        timing_table.End("TCP_connection_process_if");

        if ( FAILED(hr) )
            goto bailout;

        timing_table.Start("TCP_connection_process_if2", "TCP connection StartAsyncRecv");

        // Post the receive for the next transaction
        hr = StartAsyncRecv(true);

        timing_table.End("TCP_connection_process_if2");

        if ( FAILED(hr) )
            goto bailout;
    }

    timing_table.LogTimes(Logger::Debug);

bailout:
    return hr;
}


// --------------------------------------------------------------------------
//
// utility
//

HRESULT 
CTcpConnection::StartAsyncRecv (bool newTransaction)
{
    HRESULT hr = S_OK;
    DWORD dwBytes = 0;

    if ( newTransaction )
    {
        hr = m_inboundContext.PrepareRecv();
        if ( FAILED(hr) )
        {
            goto bailout;
        }
    }
	{
		DWORD dwFlags = 0;
		if (SOCKET_ERROR == ::WSARecv
		(
			m_sock,
			m_inboundContext.GetWsaBuffers(),
			m_inboundContext.GetWsaBufferCount(),
			&dwBytes,
			&dwFlags,
			m_inboundContext.GetPOverlapped(),
			NULL
		)
			&& ERROR_IO_PENDING != ::WSAGetLastError())
		{
			hr = HRESULT_FROM_WIN32(::WSAGetLastError());
			goto bailout;
		}
	}
bailout:
    return hr;
}

HRESULT 
CTcpConnection::ProcessRequest
    (
        DWORD dwRequestedFrame
    )
{
    TimingTable timing_table("TCP_connection");

    HRESULT hr = S_OK;
    const BYTE* pData = NULL;
    DWORD cbData = 0;
    DWORD dwFrameNumber = m_lastFrameSent;
	
    timing_table.Start("process_request", "TCP connection process request");

    while (dwFrameNumber == m_lastFrameSent)
    {
        hr = m_compressor->GetLatestPacket(&pData, &cbData, &dwFrameNumber);
    }

   if ( FAILED(hr) )
    {
		printf("GetLatestPacket returned FAIL with error %d\n", hr);
        goto bailout;
    }

    // Prep the scatter-gather buffers and transmit
    if (dwFrameNumber < dwRequestedFrame)
    {
		//TODO: check fusion and DG network both work. if there are issues look here!
		//NOTE: also used between fusion and renderer! look here.

  //      // no new frames, send size of zero
		hr = m_outboundContext.PrepareSend(const_cast<BYTE*>(m_zero_buffer), 12);
		// no new frames
		//hr = m_outboundContext.PrepareSend(NULL, 0);
		if (FAILED(hr))
		{
            std::cout << "Failed PrepareSend. Error " << hr << std::endl;            
			goto bailout;
		}
    }
    else
    {
        m_lastFrameSent = dwFrameNumber;

        hr = m_outboundContext.PrepareSend(const_cast<BYTE*>(pData), cbData);
	/*	unsigned long encodedSize = reinterpret_cast<const unsigned long *>(pData)[0];
		if (encodedSize == 0)
		{
			std::cout << "how?" << std::endl;
		}
		std::cout << "Sent" << encodedSize << std::endl;*/
		unsigned long* dataStart = reinterpret_cast<unsigned long *>(m_outboundContext.GetWsaBuffers()->buf);
	//	std::cout << "wsaSending " << dataStart[0] << " " << dataStart[1] << std::endl;
        if ( FAILED(hr) )
            goto bailout;
    }
	{
		DWORD dwBytes = 0;
		DWORD dwFlags = 0;

		if (SOCKET_ERROR == ::WSASend
		(
			m_sock,
			m_outboundContext.GetWsaBuffers(),
			m_outboundContext.GetWsaBufferCount(),
			&dwBytes,
			dwFlags,
			m_outboundContext.GetPOverlapped(),
			NULL
		)
			&& ERROR_IO_PENDING != ::WSAGetLastError())
		{
			hr = HRESULT_FROM_WIN32(::WSAGetLastError());
			goto bailout;
		}
		m_compressor->ReleaseLastFetchedPacket();
		g_responses++;
		if (dwFrameNumber != dwRequestedFrame)
		{
			g_payloads++;
		}
	}
    timing_table.End("process_request");
    timing_table.LogTimes(Logger::Debug);

bailout:
    return hr;
}

HRESULT 
CTcpConnection::ProcessRecv
    (
        BOOL result,
        CTcpConnectionContext* context,
        ULONG cbBytes
    )
{
    HRESULT hr = S_OK;

    if ( !result || 0 == cbBytes )
    {
        hr = HRESULT_FROM_WIN32(WSAECONNABORTED);
        goto bailout;
    }

    // process the I/O
    hr = context->RecvResult(cbBytes);
    if ( FAILED(hr) )
        goto bailout;

    if ( S_FALSE == hr )
    // incomplete receive, post another one
    {
        // start the next i/o (recycle the inbound context)
        hr = StartAsyncRecv(false);
        if ( FAILED(hr) )
            goto bailout;
    }
    else
    if ( SUCCEEDED(hr) )
    // complete receive
    {
		g_requests++;
        hr = ProcessRequest(context->GetRequestedFrame());
        if ( FAILED(hr) )
            goto bailout;
    }

bailout:
    if ( FAILED(hr) )
    {
        Shutdown();
    }
    return hr;
}


HRESULT 
CTcpConnection::ProcessSend
    (
        BOOL result,
        CTcpConnectionContext* context,
        ULONG cbBytes
    )
{
    HRESULT hr = S_OK;

    if ( !result || 0 == cbBytes )
    {
        hr = HRESULT_FROM_WIN32(WSAECONNABORTED);
        goto bailout;
    }

    // process the I/O
    hr = context->SendResult(cbBytes);
    if ( FAILED(hr) )
        goto bailout;

bailout:
    return hr;
}

// --------------------------------------------------------------------------

HRESULT 
CTcpConnectionContext::PrepareRecv ()
{
    Reset();
    return S_OK;
}

HRESULT 
CTcpConnectionContext::RecvResult
    (
        ULONG cbBytes
    )
{
    HRESULT hr = E_UNEXPECTED;

    if ( cbBytes > m_wsabuf.len )
    // more bytes than the receive buffer??
    {
        goto bailout;
    }

    if ( cbBytes < m_wsabuf.len )
    // incomplete read, set up for more
    {
        m_wsabuf.len -= cbBytes;
        m_wsabuf.buf += cbBytes;
        hr = S_FALSE; // incomplete read
    }
    else
    if ( cbBytes == m_wsabuf.len )
    // header received
    {
        hr = S_OK; // message complete
    }

bailout:
    return hr;
}

HRESULT 
CTcpConnectionContext::PrepareSend
    (
        BYTE* pData,
        DWORD cbData
    )
{
    Reset();
    if ( NULL == pData )
    {
        m_payload = sizeof(DWORD);
    }
    else
    {
        m_wsabuf.len = cbData;
        m_wsabuf.buf = reinterpret_cast<CHAR*>(pData);
    }
    return S_OK;
}


HRESULT 
CTcpConnectionContext::SendResult
    (
        ULONG cbBytes
    )
{
    HRESULT hr;

    if ( cbBytes == m_wsabuf.len )
    {
        // we expect to send all of the message in one scatter/gather async I/O
        hr = S_OK;
    }
    else
    {
        hr = STG_E_WRITEFAULT;
    }

    return hr;
}
