#include "stdafx.h"
#include "framebuffer.h"

using namespace HolochatNetworking;

HRESULT WINAPI NetworkFactory::CreateMessageBuffer
    (
        SIZE_T maxBufferSize,
        IMessageBuffer** messageBufferOut
    )
{
    if ( NULL == messageBufferOut )
    {
        return E_POINTER;
    }

    CFrameBuffer* messageBuffer = new CFrameBuffer();
    if ( NULL == messageBuffer )
    {
        return E_OUTOFMEMORY;
    }

    HRESULT hr = messageBuffer->Initialize(maxBufferSize);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = messageBuffer->QueryInterface(__uuidof(IMessageBuffer), (void**)messageBufferOut);

bailout:
    messageBuffer->Release();
    return hr;
}

CFrameBuffer::CFrameBuffer() :
    m_cRef(1),
    m_readIndex(-1)
{
    InitializeCriticalSection(&m_lock);
    m_cbMax = 0;
    m_lastMessageID = 0;
    for ( int index = 0; index < 2; index++ )
    {
        m_bufferSizes[index] = 0;
        m_buffers[index] = NULL;
    }
}

CFrameBuffer::~CFrameBuffer()
{
    for ( int index = 0; index < 2; index++ )
    {
        if ( NULL != m_buffers[index] )
        {
            delete [] m_buffers[index];
        }
    }
    DeleteCriticalSection(&m_lock);
}

HRESULT CFrameBuffer::Initialize(SIZE_T maxBufferSize)
{
    m_cbMax = maxBufferSize;
    m_buffers[0] = new BYTE[maxBufferSize];
    m_buffers[1] = new BYTE[maxBufferSize];

    if ( NULL == m_buffers[0]
         || NULL == m_buffers[1] )
    {
        return E_OUTOFMEMORY;
    }

    return S_OK;
}

STDMETHODIMP CFrameBuffer::QueryInterface
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
        *ppv = static_cast<IMessageCallback*>(this);
        AddRef();
    }
    else
    if ( __uuidof(IMessageCallback) == riid )
    {
        *ppv = static_cast<IMessageCallback*>(this);
        AddRef();
    }
    else
    if ( __uuidof(IMessageBuffer) == riid )
    {
        *ppv = static_cast<IMessageBuffer*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CFrameBuffer::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CFrameBuffer::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
   if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP_(void) CFrameBuffer::MessageReceived
    (
        DWORD messageId,
        const BYTE* payload,
        DWORD cbPayload
    )
{
    if ( cbPayload > m_cbMax)
    {
        return;
    }

    EnterCriticalSection(&m_lock);
    {
        // verify that the messages are in order
        if ( (int)(messageId - m_lastMessageID) > 0 )
        {
            int index = 0;
            if ( m_readIndex >= 0 )
            {
                // one buffer is locked
                index = ~m_readIndex & 1;
            }

            CopyMemory(m_buffers[index], payload, cbPayload);
            m_bufferSizes[index] = cbPayload;
            m_lastMessageID = messageId;

            if ( m_readIndex < 0 )
            {
                // clear out the other buffer (older)
                int flipIndex = ~index& 1;
                m_bufferSizes[flipIndex] = 0;
            }
        }
    }
    LeaveCriticalSection(&m_lock);
}


STDMETHODIMP CFrameBuffer::AcquireBuffer
    (
        const BYTE** bufferOut,
        SIZE_T* lengthOut
    )
{
    if ( NULL == bufferOut
         || NULL == lengthOut )
    {
        return E_POINTER;
    }

    HRESULT hr = S_OK;

    *bufferOut = NULL;
    *lengthOut = 0;

    EnterCriticalSection(&m_lock);
    {
        if ( m_readIndex >= 0 )
        {
            // already acquired
            hr = E_ABORT;
        }
        else
        {
            for ( int index = 0; index < 2; index++ )
            {
                if ( m_bufferSizes[index] > 0 )
                {
                    *bufferOut = m_buffers[index];
                    *lengthOut = m_bufferSizes[index];
                    m_readIndex = index;
                    break;
                }
            }
            if ( m_readIndex < 0 )
            {
                // no data available
                hr = E_FAIL;
            }
        }
    }
    LeaveCriticalSection(&m_lock);

    return hr;
}

STDMETHODIMP CFrameBuffer::ReleaseBuffer
    (
        const BYTE* buffer
    )
{
    if ( NULL == buffer )
    {
        return E_POINTER;
    }

    HRESULT hr = S_OK;

    EnterCriticalSection(&m_lock);
    {
        if ( m_readIndex < 0 )
        {
            // buffer not acquired
            hr = E_ABORT;
        }
        else
        if ( buffer != m_buffers[m_readIndex] )
        {
            // wrong buffer?
            hr = E_INVALIDARG;
        }
        else
        {
            m_bufferSizes[m_readIndex] = 0; // resets the buffer entry
            m_readIndex = -1;
        }
    }
    LeaveCriticalSection(&m_lock);

    return hr;
}
