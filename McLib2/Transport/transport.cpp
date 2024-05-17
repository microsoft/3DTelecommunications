#include "stdafx.h"
#include "endpoint.h"
#include "transport.h"

using namespace HolochatNetworking;

#define ENDPOINT_BUFFER_SIZE (256*1024)

#define IOCP_KEY_EXIT   1
#define IOCP_KEY_SOCKET 2

const DWORD CTransport::HeaderSize = 
              sizeof(BYTE)                  // messageType
            + sizeof(DWORD)                 // messageId
            + sizeof(WORD) + sizeof(BYTE)   // messageSize
            + sizeof(WORD);                 // sequence

HRESULT WINAPI NetworkFactory::CreateTransport
    (
        USHORT localPort,
        PCTSTR targetName,
        USHORT targetPort,
        ITransportCallback* callback,
        ITransport** transportOut
    )
{
    if ( NULL == transportOut)
    {
        return E_INVALIDARG;
    }

    CTransport* transport = new CTransport();
    if ( NULL == transport )
    {
        return E_OUTOFMEMORY;
    }

    HRESULT hr = transport->Initialize(localPort, targetName, targetPort, callback);
    if ( FAILED(hr) )
        goto bailout;

    hr = transport->QueryInterface(__uuidof(ITransport), (void**)transportOut);

bailout:
    transport->Release();
    return hr;
}

CTransport::CTransport() :
    m_cRef(1),
    m_hIOCP(NULL),
    m_hThread(NULL)
{
}

CTransport::~CTransport()
{
    if ( NULL != m_hThread )
    {
        ::CloseHandle(m_hThread);
    }
    if ( NULL != m_hIOCP )
    {
        ::CloseHandle(m_hIOCP);
    }
    if ( NULL != m_callback )
    {
        m_callback->Release();
    }
}


HRESULT CTransport::Initialize(USHORT localPort, PCTSTR targetName,  USHORT targetPort, ITransportCallback* callback)
{
    HRESULT hr = S_OK;

    if ( NULL == callback )
    {
        hr = E_POINTER;
        goto bailout;
    }

    m_callback = callback;
    m_callback->AddRef();

    m_hIOCP = ::CreateIoCompletionPort(INVALID_HANDLE_VALUE, NULL, 0, 1);
    if ( NULL == m_hIOCP )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

    hr = m_endpoint.Initialize(localPort, targetName, targetPort, ENDPOINT_BUFFER_SIZE, m_hIOCP, IOCP_KEY_SOCKET);
    if ( FAILED(hr) )
        goto bailout;

    DWORD threadId;
    m_hThread = ::CreateThread
                        (
                            NULL, 0, 
                            ThreadStart,
                            this,
                            0, &threadId
                        );
    if ( NULL == m_hThread )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

bailout:
    if ( FAILED(hr) && NULL != m_callback )
    {
        m_callback->Release();
        m_callback = NULL;
    }
    return hr;
}

STDMETHODIMP CTransport::QueryInterface
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
        *ppv = static_cast<IUnknown*>(this);
        AddRef();
    }
    else
    if ( __uuidof(ITransport) == riid )
    {
        *ppv = static_cast<ITransport*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CTransport::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CTransport::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
   if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP_(DWORD) CTransport::MaxPayloadOctets ()
{
    return TransportPacket::PacketSize - CTransport::HeaderSize;
}

STDMETHODIMP CTransport::Send
    (
        BYTE messageType,
        DWORD messageId,
        DWORD messageSize,
        WORD sequence,
        const BYTE* payload,
        DWORD cbPayload
    )
{
    HRESULT hr = S_OK;

    if ( NULL == payload && 0 != cbPayload )
    {
        hr = E_POINTER;
        goto bailout;
    }

    if ( messageSize > 0x00ffffff )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }
    
    TransportPacket* pPacket = new TransportPacket();
    if ( NULL == pPacket )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    DWORD index = 0;
    pPacket->Data[index++] = (BYTE)(messageType);
    pPacket->Data[index++] = (BYTE)(messageId >> 24);
    pPacket->Data[index++] = (BYTE)(messageId >> 16);
    pPacket->Data[index++] = (BYTE)(messageId >> 8);
    pPacket->Data[index++] = (BYTE)(messageId >> 0);
    pPacket->Data[index++] = (BYTE)(messageSize >> 16);
    pPacket->Data[index++] = (BYTE)(messageSize >> 8);
    pPacket->Data[index++] = (BYTE)(messageSize >> 0);
    pPacket->Data[index++] = (BYTE)(sequence >> 8);
    pPacket->Data[index++] = (BYTE)(sequence >> 0);

    if ( 0 != cbPayload )
    {
        if ( index + cbPayload > pPacket->PacketSize )
        {
            hr = E_INVALIDARG;
            goto bailout;
        }
        memcpy(&pPacket->Data[index], payload, cbPayload);
        index += cbPayload;
    }

    pPacket->Wsabuf.len = index;

    hr = m_endpoint.PostAsyncSend(pPacket);
    if ( FAILED(hr) )
    {
        goto bailout;
    }
    pPacket = NULL; // will be freed once the transmit is complete

bailout:
    if ( NULL != pPacket)
    {
        delete pPacket;
    }
    return hr;
}

STDMETHODIMP CTransport::Close()
{
    if ( NULL != m_hThread )
    {
        if ( ::PostQueuedCompletionStatus
                (
                    m_hIOCP,
                    0,
                    IOCP_KEY_EXIT,
                    (LPOVERLAPPED)NULL
                ) )
        {
            ::WaitForSingleObject(m_hThread, INFINITE);
        }
    }

    m_endpoint.Close();

    if ( NULL != m_callback )
    {
        m_callback->Release();
        m_callback = NULL;
    }

    return S_OK;
}

DWORD WINAPI
CTransport::ThreadStart( void* param )
{
    return ((CTransport*)param)->ThreadMain();
}

HRESULT CTransport::ThreadMain()
{
    HRESULT hr = S_OK;
    TransportPacket readPacket;
    bool bPostReceive = true;

    hr = ::CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if ( FAILED(hr) )
        goto bailout;

    while ( true)
    {
        if ( bPostReceive )
        {
            readPacket.Reset();

            hr = m_endpoint.PostAsyncReceive(&readPacket);
            if ( SUCCEEDED(hr) )
            {
                bPostReceive = false;
            }
        }

        DWORD cbBytes = 0;
        TransportPacket* pOverlapped = NULL;
        ULONG_PTR ulCompletionKey = 0;
        if ( !::GetQueuedCompletionStatus
                    (
                        m_hIOCP,
                        &cbBytes,
                        &ulCompletionKey,
                        (LPOVERLAPPED*) &pOverlapped,
                        ClockTicks
                    ) )
        {
            hr = HRESULT_FROM_WIN32(::GetLastError());

            if ( HRESULT_FROM_WIN32(WAIT_TIMEOUT) == hr )
            {
                // tis just a tick
                Dispatch(NULL, 0);
            }
            else
            {
                continue;
            }
        }

        if ( IOCP_KEY_EXIT == ulCompletionKey )
        {
            hr = S_FALSE;
            break;
        }
        else
        if ( IOCP_KEY_SOCKET == ulCompletionKey )
        {
            if ( &readPacket == pOverlapped )
            {
                // Receive
                Dispatch(readPacket.Data, cbBytes);
                bPostReceive = true;
            }
            else
            {
                // Transmit complete
                delete pOverlapped;
            }
        }
    }

bailout:
    ::CoUninitialize();
    return hr;
}

void CTransport::Dispatch(BYTE* pMsg, DWORD cbMsg)
{
    if ( NULL == pMsg || cbMsg < HeaderSize )
    {
        m_callback->Received(0, 0, 0, 0, NULL, cbMsg);
        return;
    }

    BYTE messageType = 0;
    messageType = *pMsg++;
    cbMsg -= sizeof(BYTE);

    DWORD messageId = 0;
    messageId |= *pMsg++; messageId <<= 8;
    messageId |= *pMsg++; messageId <<= 8;
    messageId |= *pMsg++; messageId <<= 8;
    messageId |= *pMsg++;
    cbMsg -= sizeof(DWORD);

    DWORD messageSize = 0;
    messageSize |= *pMsg++; messageSize <<= 8;
    messageSize |= *pMsg++; messageSize <<= 8;
    messageSize |= *pMsg++;
    cbMsg -= sizeof(WORD) + sizeof(BYTE);

    WORD sequence = 0;
    sequence |= *pMsg++; sequence <<= 8;
    sequence |= *pMsg++;
    cbMsg -= sizeof(WORD);

    m_callback->Received(messageType, messageId, messageSize, sequence, pMsg, cbMsg);
}
