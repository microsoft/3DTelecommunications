///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///


#include "stdafx.h"
#include "HoloNet.h"
#include "HoloNetP.h"
#include "HNSession.h"
#include "FecMatrix.h"

using namespace HoloNet;
using namespace VideoTransmission;

static const BYTE s_DatagramPing = 0;
static const BYTE s_DatagramPacketMin = 1;
static const BYTE s_DatagramPacketMax = 127;
static const BYTE s_DatagramMessageMin = 128;
static const BYTE s_DatagramMessageMax = 255;

//
// Datagram format:
//
// Octets 0-15  : Sender GUID
// Octet 16     : Datagram type (Ping, Packet or Message)
// For datagram type Ping
//              : End of datagram
// For datagram type Packet:
// Octet 17-    : Packet payload
// For datagram type Message:
// Octet 17-20  : Message ID
// Octet 21-24  : Total message size
// Octet 25-26  : Message sequence number
// Octet 27-    : Message payload
//

//
// Connection protocol:
//
// Send pings at least every s_dwPingTime.
// Switch to "Connected" if Ping, Packet or Message received.
// Switch to "Disconnected" and stop transmitting (other than pings) if nothing received in s_dwAliveTime.
//

// --------------------------------------------------------------------------
// 
// ctor/init
//

HRESULT WINAPI Factory::CreateNetSession
    (
        LPCWSTR url,
        REFIID id,
        BOOL allowLoopback,
        DWORD aliveTime,
        INetSession** sessionOut
    )
{
    HRESULT hr = S_OK;
    CHoloNetSession* pSession = NULL;

    if ( NULL == sessionOut )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    pSession = new CHoloNetSession();
    if ( NULL == pSession )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }

    hr = pSession->Initialize(url, id, FALSE != allowLoopback, aliveTime);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = pSession->QueryInterface(__uuidof(INetSession), (void**)sessionOut);

bailout:
    if ( NULL != pSession )
    {
        pSession->Release();
    }
    return hr;
}

CHoloNetSession::CHoloNetSession () : 
    m_cRef(1),
    m_bAllowLoopback(false),
    m_hEvent(NULL),
    m_hThread(NULL),
    m_state(Stopped),
    m_dwAliveTime(s_dwDefaultAliveTime),
    m_lastMessageId(0),
    m_lastAllocatedAssembly(AssemblyBuffers-1),
    m_notifyAppeared(false),
    m_manager(NULL),
    m_connection(NULL),
    m_sink(NULL)
{
    m_id = __uuidof(NULL);
    InitializeCriticalSection(&m_lock);

    for ( ULONG i = 0; i < PresenceSlots; i++ )
    {
        m_presenceTable[i].nodeId = __uuidof(NULL);
    }
}

CHoloNetSession::~CHoloNetSession ()
{
    Shutdown();
    if ( NULL == m_hEvent )
    {
        CloseHandle(m_hEvent);
    }
    DeleteCriticalSection(&m_lock);
}

// --------------------------------------------------------------------------
//
// IUnknown
//

STDMETHODIMP CHoloNetSession::QueryInterface
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
        *ppv = static_cast<INetSession*>(this);
        AddRef();
    }
    else
    if ( __uuidof(INetSession) == riid )
    {
        *ppv = static_cast<INetSession*>(this);
        AddRef();
    }
    else
    if ( __uuidof(INetSink) == riid )
    {
        *ppv = static_cast<INetSink*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CHoloNetSession::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CHoloNetSession::Release ()
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
// INetSession
//

HRESULT STDMETHODCALLTYPE CHoloNetSession::AddSink (INetSessionSink* sink)
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

HRESULT STDMETHODCALLTYPE CHoloNetSession::Startup ()
{
    HRESULT hr = S_OK;

    hr = m_connection->Start();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = StartWorkerThread();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

bailout:
    return hr;
}

BOOL STDMETHODCALLTYPE CHoloNetSession::IsConnected ()
{
    return m_state == Connected;
}

BOOL STDMETHODCALLTYPE CHoloNetSession::IsPresent (REFIID id)
{
    BOOL present = FALSE;

    EnterCriticalSection(&m_lock);
    {
        ULONGLONG ticks = ::GetTickCount64();

        for ( ULONG i = 0; i < PresenceSlots; i++ )
        {
            if ( m_presenceTable[i].nodeId == id )
            {
                ULONGLONG elapsed = ticks - m_presenceTable[i].lastReceivedTick;
                if ( elapsed <= (ULONGLONG)m_dwAliveTime )
                {
                    present = TRUE;
                }
                break;
            }
        }
    }
    LeaveCriticalSection(&m_lock);

    return present;
}

HRESULT STDMETHODCALLTYPE CHoloNetSession::SendPacket
    (
        BYTE packetType,
        const BYTE* pHeader,
        ULONG cbHeader,
        const BYTE* pPayload,
        ULONG cbPayload
    )
{
    HRESULT hr = S_OK;
    INetDatagram* datagram = NULL;

    if ( packetType > (s_DatagramPacketMax - s_DatagramPacketMin) )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( m_state != Connected )
    {
        hr = S_FALSE;
        goto bailout;
    }

    hr = m_connection->CreateDatagram(&datagram);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    BYTE* p = datagram->GetBuffer();
    ULONG size = datagram->GetSize();
    ULONG cb = 0;

    if ( size < (sizeof(GUID) + sizeof(BYTE) + cbHeader + cbPayload) )
    {
        // not enough room
        hr = E_UNEXPECTED;
        goto bailout;
    }

    *(UNALIGNED GUID*)p = m_id;
    p += sizeof(GUID);
    cb += sizeof(GUID);

    *p = s_DatagramPacketMin + packetType;
    p += sizeof(BYTE);
    cb += sizeof(BYTE);

    if ( 0 != cbHeader )
    {
        memcpy(p, pHeader, cbHeader);
        p += cbHeader;
        cb += cbHeader;
    }

    if ( 0 != cbPayload )
    {
        memcpy(p, pPayload, cbPayload);
        p += cbPayload;
        cb += cbPayload;
    }

    hr = datagram->SetLength(cb);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_connection->SendDatagram(datagram);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    EnterCriticalSection(&m_lock);
    {
        m_lastSentTicks = ::GetTickCount();
    }
    LeaveCriticalSection(&m_lock);

bailout:
    if ( NULL != datagram )
    {
        datagram->Release();
    }
    return hr;
}

HRESULT STDMETHODCALLTYPE CHoloNetSession::SendMessage
    (
        BYTE messageType,
        const BYTE* pHeader,
        ULONG cbHeader,
        const BYTE* pPayload,
        ULONG cbPayload
    )
{
    HRESULT hr = S_OK;
    INetDatagram* datagram = NULL;
    BYTE* redundancyBytes = NULL;

    if ( messageType > (s_DatagramMessageMax - s_DatagramMessageMin) )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( m_state != Connected )
    {
        hr = S_FALSE;
        goto bailout;
    }

    hr = m_connection->CreateDatagram(&datagram);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    ULONG datagramSize = datagram->GetSize();
    if ( datagramSize < (sizeof(GUID) + sizeof(BYTE) + sizeof(DWORD) + sizeof(DWORD) + sizeof(USHORT)) )
    {
        // not enough room for header
        hr = E_UNEXPECTED;
        goto bailout;
    }

    ULONG payloadSize = datagramSize - (sizeof(GUID) + sizeof(BYTE) + sizeof(DWORD) + sizeof(DWORD) + sizeof(USHORT));
    ULONG messageSize = (cbHeader + cbPayload);
    ULONG packetCount = (messageSize + payloadSize - 1 ) / payloadSize;
    ULONG redundancyCount = (packetCount * s_redundancyPerThousand + 1000 - 1)/ 1000;
    ULONG messageId = InterlockedIncrement(&m_lastMessageId);
    USHORT messageSequence = 0;

    const BYTE* pSrc1 = pHeader;
    ULONG cbSrc1 = cbHeader;
    const BYTE *pSrc2 = pPayload;
    ULONG cbSrc2 = cbPayload;
    for ( ULONG i = 0; i < packetCount; i++ )
    {
        if ( NULL == datagram )
        {
            hr = m_connection->CreateDatagram(&datagram);
            if ( FAILED(hr) )
            {
                goto bailout;
            }
        }

        BYTE* p = datagram->GetBuffer();
        ULONG cb = 0;

        *(UNALIGNED GUID*)p = m_id;
        p += sizeof(GUID);
        cb += sizeof(GUID);

        *p = s_DatagramMessageMin + messageType;
        p += sizeof(BYTE);
        cb += sizeof(BYTE);

        *(UNALIGNED ULONG*)p = messageId;
        p += sizeof(ULONG);
        cb += sizeof(ULONG);

        *(UNALIGNED ULONG*)p = messageSize;
        p += sizeof(ULONG);
        cb += sizeof(ULONG);

        *(UNALIGNED USHORT*)p = messageSequence++;
        p += sizeof(USHORT);
        cb += sizeof(USHORT);

        // bytes from header
        if ( 0 != cbSrc1 )
        {
            ULONG k = __min(cbSrc1, datagramSize - cb);
            memcpy(p, pSrc1, k);
            p += k;
            cb += k;
            pSrc1 += k;
            cbSrc1 -= k;
        }

        // bytes from payload
        if ( 0 != cbSrc2 && cb < datagramSize )
        {
            ULONG k = __min(cbSrc2, datagramSize - cb);
            memcpy(p, pSrc2, k);
            p += k;
            cb += k;
            pSrc2 += k;
            cbSrc2 -= k;
        }

        hr = datagram->SetLength(cb);
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        hr = m_connection->SendDatagram(datagram);
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        EnterCriticalSection(&m_lock);
        {
            m_lastSentTicks = ::GetTickCount();
        }
        LeaveCriticalSection(&m_lock);

        datagram->Release();
        datagram = NULL;
    }

    ULONG redundancySize = payloadSize * redundancyCount;
    if ( 0 != redundancyCount )
    {
        redundancyBytes = new BYTE[redundancySize];
        if ( NULL == redundancyBytes )
        {
            hr = E_OUTOFMEMORY;
            goto bailout;
        }

	    FecMatrix * senderMatrix = new FecMatrix(packetCount, redundancyCount);
        if ( NULL == senderMatrix )
        {
            hr = E_OUTOFMEMORY;
            goto bailout;
        }
	    senderMatrix->ComputeRedundancy2(pHeader, cbHeader, pPayload, cbPayload, redundancyBytes, payloadSize);
	    delete senderMatrix;
    }

    pSrc1 = redundancyBytes;
    cbSrc1 = redundancySize;
    for ( ULONG i = 0; i < redundancyCount; i++ )
    {
        if ( NULL == datagram )
        {
            hr = m_connection->CreateDatagram(&datagram);
            if ( FAILED(hr) )
            {
                goto bailout;
            }
        }

        BYTE* p = datagram->GetBuffer();
        ULONG cb = 0;

        *(UNALIGNED GUID*)p = m_id;
        p += sizeof(GUID);
        cb += sizeof(GUID);

        *p = s_DatagramMessageMin + messageType;
        p += sizeof(BYTE);
        cb += sizeof(BYTE);

        *(UNALIGNED ULONG*)p = messageId;
        p += sizeof(ULONG);
        cb += sizeof(ULONG);

        *(UNALIGNED ULONG*)p = messageSize;
        p += sizeof(ULONG);
        cb += sizeof(ULONG);

        *(UNALIGNED USHORT*)p = messageSequence++;
        p += sizeof(USHORT);
        cb += sizeof(USHORT);

        ULONG k = __min(cbSrc1, datagramSize - cb);
        memcpy(p, pSrc1, k);
        p += k;
        cb += k;
        pSrc1 += k;
        cbSrc1 -= k;

        hr = datagram->SetLength(cb);
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        hr = m_connection->SendDatagram(datagram);
        if ( FAILED(hr) )
        {
            goto bailout;
        }

        EnterCriticalSection(&m_lock);
        {
            m_lastSentTicks = ::GetTickCount();
        }
        LeaveCriticalSection(&m_lock);

        datagram->Release();
        datagram = NULL;
    }

bailout:
    if ( NULL != redundancyBytes )
    {
        delete [] redundancyBytes;
    }
    if ( NULL != datagram )
    {
        datagram->Release();
    }
    return hr;
}

HRESULT STDMETHODCALLTYPE CHoloNetSession::Shutdown ()
{
    HRESULT hr = S_OK;

    hr = StopWorkerThread();

    if ( NULL != m_connection )
    {
        hr = m_connection->Stop();
    }
    if ( NULL != m_manager )
    {
        hr = m_manager->Shutdown();
    }

    if ( NULL != m_sink )
    {
        m_sink->Release();
        m_sink = NULL;
    }
    if ( NULL != m_connection )
    {
        m_connection->Release();
        m_connection = NULL;
    }
    if ( NULL != m_manager )
    {
        m_manager->Release();
        m_manager = NULL;
    }

    return hr;
}

// --------------------------------------------------------------------------
//
// INetSink
//

void STDMETHODCALLTYPE CHoloNetSession::Received (INetDatagram* datagram)
{
    const BYTE* p = datagram->GetBuffer();
    ULONG cb = datagram->GetLength();
    if ( cb < (sizeof(GUID) + sizeof(BYTE)) )
    {
        // invalid header
        goto bailout;
    }

    GUID sender;
    sender = *(UNALIGNED GUID*)p;
    p += sizeof(GUID);
    cb -= sizeof(GUID);

    if ( !m_bAllowLoopback
         && sender == m_id )
    {
        // drop loopback datagrams
        goto bailout;
    }

    BYTE datagramType;
    datagramType = *p;
    p += sizeof(BYTE);
    cb -= sizeof(BYTE);

    HRESULT hr = S_OK;

    if ( s_DatagramPing == datagramType )
    {
        if ( 0 != cb )
        {
            // invalid ping
            goto bailout;
        }
        hr = DispatchPing(sender);
    }
    else
    if ( s_DatagramPacketMax >= datagramType )
    {
        hr = DispatchPacket(sender, datagramType - s_DatagramPacketMin, p, cb);
    }
    else
    if ( s_DatagramMessageMax >= datagramType )
    {
        if ( cb < (sizeof(ULONG) + sizeof(ULONG) + sizeof(USHORT)) )
        {
            // invalid message header
            goto bailout;
        }

        ULONG messageId;
        messageId = *(UNALIGNED ULONG*)p;
        p += sizeof(ULONG);
        cb -= sizeof(ULONG);
        ULONG messageSize;
        messageSize = *(UNALIGNED ULONG*)p;
        p += sizeof(ULONG);
        cb -= sizeof(ULONG);
        USHORT sequenceNumber;
        sequenceNumber = *(UNALIGNED USHORT*)p;
        p += sizeof(USHORT);
        cb -= sizeof(USHORT);

        hr = DispatchMessageFragment
                (
                    sender,
                    datagramType - s_DatagramMessageMin,
                    datagram->GetSize() - (sizeof(GUID) + sizeof(BYTE) + sizeof(DWORD) + sizeof(DWORD) + sizeof(USHORT)),
                    messageId,
                    messageSize,
                    sequenceNumber,
                    p,
                    cb
                );
    }

    if ( S_FALSE == hr )
    {
        hr = SendPing();
    }

bailout:
    return;
}

void STDMETHODCALLTYPE CHoloNetSession::Notify (HRESULT result)
{
    WCHAR text[128];
    wsprintf(text, L"Session error notify %x\n", result);
    OutputDebugString(text);
}


// --------------------------------------------------------------------------
//
// utility
//

HRESULT CHoloNetSession::Initialize
    (
        LPCWSTR url,
        REFIID id,
        bool allowLoopback,
        DWORD dwAliveTime
    )
{
    HRESULT hr = S_OK;

    m_id = id;
    m_bAllowLoopback = allowLoopback;
    m_dwAliveTime = dwAliveTime;

    m_hEvent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
    if ( NULL == m_hEvent )
    {
        hr = HRESULT_FROM_WIN32(GetLastError());
        goto bailout;
    }

    hr = FactoryP::CreateNetManager(&m_manager);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_manager->Startup();
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_manager->CreateConnection(url, &m_connection);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_connection->AddSink(static_cast<INetSink*>(this));
    if ( FAILED(hr) )
    {
        goto bailout;
    }

bailout:
    return hr;
}

void CHoloNetSession::Tick ()
{
    HRESULT hr = S_OK;
    bool sendPing = false;
    bool bNotifyAppeared = false;

    EnterCriticalSection(&m_lock);
    {
        DWORD ticks = ::GetTickCount();

        switch ( m_state )
        {
        case Stopped:
            break;

        case Connected:
            if ( m_notifyAppeared 
				 || (ticks - m_lastAppearedTicks) > s_dwAppearedTime )
            {
				bNotifyAppeared = true;
				m_notifyAppeared = false;
				m_lastAppearedTicks = ticks;
            }
            if ( (ticks - m_lastReceivedTicks) > m_dwAliveTime )
            {
                m_state = Disconnected;
            }
            // fall through
        case Disconnected:
            if ( (ticks - m_lastSentTicks) > s_dwPingTime )
            {
                sendPing = true;
            }
            break;
        }

    }
    LeaveCriticalSection(&m_lock);

    if ( sendPing )
    {
        hr = SendPing();
    }

    if ( bNotifyAppeared
         && NULL != m_sink )
    {
        m_sink->NodeAppeared();
    }

    return;
}

HRESULT CHoloNetSession::SendPing ()
{
    HRESULT hr = S_OK;
    INetDatagram* datagram = NULL;

    hr = m_connection->CreateDatagram(&datagram);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    BYTE* p = datagram->GetBuffer();
    ULONG size = datagram->GetSize();
    ULONG cb = 0;

    if ( size < (sizeof(GUID) + sizeof(BYTE)) )
    {
        // not enough room for header
        hr = E_UNEXPECTED;
        goto bailout;
    }

    *(UNALIGNED GUID*)p = m_id;
    p += sizeof(GUID);
    cb += sizeof(GUID);

    *p = s_DatagramPing;
    p += sizeof(BYTE);
    cb += sizeof(BYTE);

    hr = datagram->SetLength(cb);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = m_connection->SendDatagram(datagram);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    EnterCriticalSection(&m_lock);
    {
        m_lastSentTicks = ::GetTickCount();
    }
    LeaveCriticalSection(&m_lock);

bailout:
    if ( NULL != datagram )
    {
        datagram->Release();
    }
    return hr;
}

HRESULT CHoloNetSession::DispatchPing (REFIID sender)
{
    HRESULT hr = S_OK;
    bool sendPing = false;

    EnterCriticalSection(&m_lock);
    {
        m_lastReceivedTicks = ::GetTickCount();
        if ( m_state == Disconnected )
        {
            m_state = Connected;
            sendPing = true;
        }
        RegisterPresent(sender);
    }
    LeaveCriticalSection(&m_lock);

    if ( sendPing && SUCCEEDED(hr) )
    {
        hr = S_FALSE;
    }
    return hr;
}

HRESULT CHoloNetSession::DispatchPacket
    (
        REFIID sender,
        BYTE packetType,
        const BYTE* payload,
        DWORD cbPayload
    )
{
    HRESULT hr = S_OK;
    bool sendPing = false;

    EnterCriticalSection(&m_lock);
    {
        m_lastReceivedTicks = ::GetTickCount();
        if ( m_state == Disconnected )
        {
            m_state = Connected;
            sendPing = true;
        }
        RegisterPresent(sender);
    }
    LeaveCriticalSection(&m_lock);

    if ( NULL != m_sink )
    {
        m_sink->PacketReceived(sender, packetType, payload, cbPayload);
    }

    if ( sendPing && SUCCEEDED(hr) )
    {
        hr = S_FALSE;
    }
    return hr;
}

HRESULT CHoloNetSession::DispatchMessageFragment
    (
        REFIID sender,
        BYTE messageType,
        ULONG packetSize,
        ULONG messageId,
        ULONG messageSize,
        USHORT sequenceNumber,
        const BYTE* payload,
        DWORD cbPayload
    )
{
    HRESULT hr = S_OK;
    bool sendPing = false;
    CMessageAssembly* pMessageAssembly = NULL;

    EnterCriticalSection(&m_lock);
    {
        DWORD ticks = ::GetTickCount();

        m_lastReceivedTicks = ticks;
        if ( m_state == Disconnected )
        {
            m_state = Connected;
            sendPing = true;
        }

        // search for existing assembly buffers
        for ( ULONG i = 0; i < AssemblyBuffers; i++ )
        {
            ULONG index = (m_lastAllocatedAssembly - i) % AssemblyBuffers;

            if ( m_messageAssemblies[index].MessageId == messageId
                && m_messageAssemblies[index].Sender == sender )
            {
                // found
                pMessageAssembly = &m_messageAssemblies[index];
                break;
            }
            //else
            //if( (ticks - m_messageAssemblies[index].LastUpdateTicks) > MessageExpiryTicks )
            //{
            //    m_messageAssemblies[index].Initialize(__uuidof(NULL), 0, 0, 0, 0, 0);
            //}
        }

        // not found, pick a new one
        if ( NULL == pMessageAssembly )
        {
            m_lastAllocatedAssembly = (m_lastAllocatedAssembly + 1) % AssemblyBuffers;
            pMessageAssembly = &m_messageAssemblies[m_lastAllocatedAssembly];

            ULONG packetCount = (messageSize + packetSize - 1 ) / packetSize;
            ULONG redundancyCount = (packetCount * s_redundancyPerThousand + 1000 - 1)/ 1000;

            hr = pMessageAssembly->Initialize
                (
                    sender,
                    messageId,
                    ticks,
                    messageSize,
                    packetSize,
                    packetCount,
                    redundancyCount
                );
        }

        if ( SUCCEEDED(hr) )
        {
            hr = pMessageAssembly->NewData(sequenceNumber, payload, cbPayload);
        }

        if ( SUCCEEDED(hr) )
        {
            RegisterPresent(sender);
        }

    }
    LeaveCriticalSection(&m_lock);

    if ( FAILED(hr) )
    {
        goto bailout;
    }
    if ( S_OK == hr
         && NULL != m_sink )
    {
        m_sink->MessageReceived(sender, messageType, pMessageAssembly->GetMessageBytes(), pMessageAssembly->GetMessageSize());
    }

    hr = S_OK;

bailout:
    if ( sendPing && SUCCEEDED(hr) )
    {
        hr = S_FALSE;
    }
    return hr;
}

bool CHoloNetSession::RegisterPresent
    (
        REFIID sender
    )
{
    bool bAppeared = false;
    // Must be called within the lock!
    ULONG found = PresenceSlots;
    ULONG oldest = PresenceSlots;
    ULONGLONG ticks = GetTickCount64();
    ULONGLONG oldestAge = 0;
    // find either an existing entry, or override an empty entry or override the oldest one
    for ( ULONG i = 0; i < PresenceSlots; i++ )
    {
        if ( m_presenceTable[i].nodeId == sender )
        {
            found = i;
            break;
        }
        if ( m_presenceTable[i].nodeId == __uuidof(NULL) )
        {
            oldest = i;
            oldestAge = (ULONGLONG)-1;
        }
        else
        {
            ULONGLONG elapsed = ticks - m_presenceTable[i].lastReceivedTick;
            if ( elapsed > oldestAge )
            {
                oldest = i;
                oldestAge = elapsed;
            }
        }
    }

    if ( found < PresenceSlots )
    {
        ULONGLONG elapsed = ticks - m_presenceTable[found].lastReceivedTick;
        m_presenceTable[found].lastReceivedTick = ticks;

        if ( elapsed > (DWORD)s_dwAppearTime )
        {
            bAppeared = true;
        }
    }
    else
    {
        m_presenceTable[oldest].nodeId = sender;
        m_presenceTable[oldest].lastReceivedTick = ticks;
        bAppeared = true;
    }

    if ( bAppeared )
    {
        m_notifyAppeared = true;
    }
    return bAppeared;
}

// --------------------------------------------------------------------------
//
// worker thread
//

HRESULT 
CHoloNetSession::StartWorkerThread ()
{
    HRESULT hr = S_OK;

    m_state = Disconnected;
    DWORD ticks = ::GetTickCount();
    m_lastSentTicks = ticks - s_dwPingTime;
    m_lastReceivedTicks = ticks - m_dwAliveTime;
	m_lastAppearedTicks = ticks;

    DWORD threadId = 0;
    m_hThread = ::CreateThread
                (
                    NULL,
                    0,
                    ThreadStart,
                    this,
                    0,
                    &threadId
                );
    if ( NULL == m_hThread )
    {
        hr = HRESULT_FROM_WIN32(::GetLastError());
        goto bailout;
    }

bailout:
    return hr;
}


HRESULT 
CHoloNetSession::StopWorkerThread ()
{
    HRESULT hr = S_OK;

    m_state = Stopped;

    if ( NULL != m_hThread )
    {
        ::SetEvent(m_hEvent);

        if ( WAIT_FAILED == ::WaitForSingleObject(m_hThread, INFINITE) )
        {
            hr = HRESULT_FROM_WIN32(::GetLastError());
        }

        CloseHandle(m_hThread);
        m_hThread = NULL;
    }

    return hr;
}


DWORD WINAPI 
CHoloNetSession::ThreadStart(void* param)
{
    return ((CHoloNetSession*)param)->ThreadMain();
}


HRESULT 
CHoloNetSession::ThreadMain()
{
    HRESULT hr = S_OK;

    hr = ::CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    // Main thread loop
    while( SUCCEEDED(hr) )
    {
        DWORD dwWaitResult = WaitForSingleObject(m_hEvent, s_dwPollTime);

        if ( WAIT_OBJECT_0 == dwWaitResult )
        {
            break;
        }
        else
        if ( WAIT_TIMEOUT == dwWaitResult )
        {
            Tick();
        }
        else
        {
            hr = HRESULT_FROM_WIN32(GetLastError());
        }
    }

bailout:
    CoUninitialize();
    return hr;
}

// --------------------------------------------------------------------------
//
// CMessageAssembly
//

CMessageAssembly::CMessageAssembly() :
    Sender(__uuidof(NULL)),
    MessageId(0),
    m_messageSize(0),
    m_packetSize(0),
    m_packets(0),
    m_redundancy(0),
    m_receivedPackets(0),
    m_receivedRedundancy(0),
    m_sizeAllocated(0),
    m_data(NULL),
    m_packetsAllocated(0),
    m_packetsPresent(NULL)
{
    LastUpdateTicks = ::GetTickCount();
}

CMessageAssembly::~CMessageAssembly()
{
    if ( NULL != m_data )
    {
        delete [] m_data;
    }
    if ( NULL != m_packetsPresent )
    {
        delete [] m_packetsPresent;
    }
}

HRESULT CMessageAssembly::Initialize
    (
        REFIID sender,
        ULONG messageId,
        DWORD ticks,
        ULONG messageSize,
        ULONG packetSize,
        ULONG packets,
        ULONG redundancy
    )
{
    HRESULT hr = Realloc(packetSize * (packets + redundancy), packets + redundancy);
    if ( FAILED(hr) )
    {
        return hr;
    }

    Sender = sender;
    MessageId = messageId;
    LastUpdateTicks = ticks;

    m_messageSize = messageSize;
    m_packetSize = packetSize;
    m_packets = packets;
    m_redundancy = redundancy;
    m_receivedPackets = 0;
    m_receivedRedundancy = 0;
    memset(m_packetsPresent, false, sizeof(bool) * (packets + redundancy));

    return hr;
}

HRESULT CMessageAssembly::NewData(USHORT sequenceNumber, const BYTE* p, ULONG cb)
{
    HRESULT hr = S_FALSE; // not completed by default

	if ( sequenceNumber  >= (m_packets + m_redundancy) )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( m_packetsPresent[sequenceNumber] )
    {
        // dupe packet
        goto bailout;
    }
    if ( m_receivedPackets >= m_packets )
    {
        // completed message
        goto bailout;
    }

    // Copy the data
	ULONG position = sequenceNumber * m_packetSize;
	// Copy the packet, add zeroes if needed.
	memcpy(&m_data[position], p, cb);
	if ( cb < m_packetSize )
	{
		memset(&m_data[position+cb], 0, m_packetSize-cb);
	}

    // Mark the segment as received 
    m_packetsPresent[sequenceNumber] = true;

    m_receivedPackets++;
    if ( sequenceNumber >= m_packets )
    {
        // this is a redundant packet
        ULONG receivedRedundancy = sequenceNumber-m_packets+1;
        if ( receivedRedundancy > m_receivedRedundancy )
        {
            m_receivedRedundancy = receivedRedundancy;
        }
    }

    // message completed?
    if ( m_receivedPackets >= m_packets )
    {
        if ( m_receivedRedundancy > 0 )
        {
            // we need to apply one or more redundancy packets
			FecMatrix* receiveMatrix = new FecMatrix(m_packets, m_redundancy);
            if ( NULL == receiveMatrix )
            {
                hr = E_OUTOFMEMORY;
                goto bailout;
            }
			bool result = receiveMatrix->ReconstructBuffer(m_data, m_messageSize, &m_data[m_packetSize * m_packets], m_packetSize, m_packetsPresent);
            delete receiveMatrix;
            if ( !result )
            {
                hr = E_FAIL;
                goto bailout;
            }
            hr = S_OK;
        }
        else
        {
            // No need to compute FEC
            hr = S_OK;
        }
    }

bailout:
    return hr;
}

HRESULT CMessageAssembly::Realloc(ULONG size, ULONG packets)
{
    if ( size > m_sizeAllocated )
    {
        if ( NULL != m_data )
        {
            delete [] m_data;
        }

        m_data = new BYTE[size];
        if ( NULL == m_data )
        {
            m_sizeAllocated = 0;
            return E_OUTOFMEMORY;
        }
        m_sizeAllocated = size;
    }

    if ( packets > m_packetsAllocated )
    {
        if ( NULL != m_packetsPresent )
        {
            delete [] m_packetsPresent;
        }

        m_packetsPresent = new bool[size];
        if ( NULL == m_packetsPresent )
        {
            m_packetsAllocated = 0;
            return E_OUTOFMEMORY;
        }
        m_packetsAllocated = packets;
    }

    return S_OK;
}
