#include "stdafx.h"
#include "GaloisField256.h"
#include "FecMatrix.h"
#include "assembly.h"
#include "session.h"

using namespace HolochatNetworking;
using namespace VideoTransmission;

const DWORD CSession::TickInterval = 200;

const BYTE CSession::PacketTypePing = 0;
const BYTE CSession::PacketTypeMessage = 1;
const BYTE CSession::PacketTypeAudio = 2;

HRESULT WINAPI NetworkFactory::CreateSession
    (
        USHORT localPort,
        PCTSTR targetName,
        USHORT targetPort,
        IMessageCallback* messageCallback,
        IAudioCallback* audioCallback,
        ISession** sessionOut
    )
{
    if ( NULL == messageCallback
         || NULL == sessionOut)
    {
        return E_POINTER;
    }

    CSession* session = new CSession();
    if ( NULL == session )
    {
        return E_OUTOFMEMORY;
    }

    HRESULT hr = session->Initialize(localPort, targetName, targetPort, messageCallback, audioCallback);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    hr = session->QueryInterface(__uuidof(ISession), (void**)sessionOut);

bailout:
    session->Release();
    return hr;
}

CSession::CSession() :
    m_cRef(1),
    m_messageCallback(NULL),
    m_audioCallback(NULL),
    m_transport(NULL),
    m_packetSize(0),
    m_redundancyPerThousand(0),
    m_state(Reset),
    m_dwTxMessageId(0),
    m_dwRxMessageId(0)
{
    ZeroMemory(&m_stats, sizeof(m_stats));
    InitializeCriticalSection(&m_lock);
    m_state = Reset;
    m_lastUpdateTicks = m_startTicks = GetTickCount();
}

CSession::~CSession()
{
    if ( NULL != m_transport)
    {
        m_transport->Release();
    }
    if ( NULL != m_messageCallback)
    {
        m_messageCallback->Release();
    }
    if ( NULL != m_audioCallback)
    {
        m_audioCallback->Release();
    }
    DeleteCriticalSection(&m_lock);
}


HRESULT CSession::Initialize(USHORT localPort, PCTSTR targetName, USHORT targetPort, IMessageCallback* messageCallback, IAudioCallback* audioCallback)
{
    HRESULT hr = S_OK;

    m_messageCallback = messageCallback;
    m_messageCallback->AddRef();
    if ( NULL != audioCallback )
    {
        m_audioCallback = audioCallback;
        m_audioCallback->AddRef();
    }

    hr = NetworkFactory::CreateTransport(localPort, targetName, targetPort, static_cast<ITransportCallback*>(this), &m_transport);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    m_packetSize = m_transport->MaxPayloadOctets();
    m_redundancyPerThousand = 40; // per thousands

bailout:
    return hr;
}

STDMETHODIMP CSession::QueryInterface
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
        *ppv = static_cast<ISession*>(this);
        AddRef();
    }
    else
    if ( __uuidof(ISession) == riid )
    {
        *ppv = static_cast<ISession*>(this);
        AddRef();
    }
    else
    if ( __uuidof(ITransportCallback) == riid )
    {
        *ppv = static_cast<ISession*>(this);
        AddRef();
    }
    else
    {
        *ppv = NULL;
        hr = E_NOINTERFACE;
    }

    return hr;
}

STDMETHODIMP_(ULONG) CSession::AddRef ()
{
    return (ULONG)::InterlockedIncrement(&m_cRef);
}

STDMETHODIMP_(ULONG) CSession::Release ()
{
    ULONG cRef = (ULONG)::InterlockedDecrement(&m_cRef);
   if ( 0 == cRef )
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP_(BOOL) CSession::IsConnected()
{
    BOOL isConnected = FALSE;
    EnterCriticalSection(&m_lock);
    {
        if ( Connected == m_state )
        {
            isConnected = TRUE;
        }
    }
    LeaveCriticalSection(&m_lock);

    return isConnected;
}

STDMETHODIMP CSession::Close()
{
    if ( NULL != m_transport )
    {
        m_transport->Close();
    }

    // Update stats
    EnterCriticalSection(&m_lock);
    {
        for ( DWORD i = 0; i < AssemblyBuffers; i++ )
        {
            if ( m_messageAssemblyBuffers[i].IsInitialized() && !m_messageAssemblyBuffers[i].IsComplete() )
            {
                m_stats.IncompleteMessages++;
            }
        }
    }
    LeaveCriticalSection(&m_lock);

    return S_OK;
}

STDMETHODIMP_(void) CSession::Received
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
    bool tick = false;
    DWORD receivedMessages = 0;
    DWORD incompleteMessages = 0;
    DWORD receivedAudio = 0;
    DWORD malformedPackets = 0;

    EnterCriticalSection(&m_lock);
    {

        DWORD elapsed = GetTickCount() - m_lastUpdateTicks;
        if ( elapsed > TickInterval )
        {
            tick = true;
        }

        if ( NULL != payload )
        {
            // this is a real message, not a tick
            m_stats.ReceivedPackets++;
            m_stats.ReceivedPacketsBytes += cbPayload;
            if ( PacketTypePing  == messageType )
            {
                m_stats.ReceivedPings++;
            }
            else
            if ( PacketTypeMessage == messageType )
            {
                m_dwRxMessageId = messageId;
            }

			switch ( m_state )
			{
			case Disconnected:
				m_state = Connecting;
				break;
            }
        }
    }
    LeaveCriticalSection(&m_lock);

    //
    // Process a tick
    //

    if ( tick )
    {
        Tick();
    }
    if ( NULL == payload )
    {
        goto bailout;
    }

    //
    // Process a ping message
    //

    if ( PacketTypePing  == messageType )
    {
        // Ping only, no further action
        if ( 0 != cbPayload )
        {
            malformedPackets++;
        }
        goto bailout;
    }

    //
    // Process a frame message
    //

    if ( PacketTypeMessage == messageType )
    {
        // Reassemble
        DWORD payloadCount = (messageSize + m_packetSize - 1) / m_packetSize;
        DWORD redundancyCount = (payloadCount * m_redundancyPerThousand + 1000 - 1)/ 1000;
        if ( sequence >= payloadCount + redundancyCount )
        {
            // bogus
            malformedPackets++;
            goto bailout;
        }

        // Pick (round robin) a message assembly buffer
        CMessageAssembly* pAssembler = &m_messageAssemblyBuffers[messageId%AssemblyBuffers];

        // Is this packet part of an existing message?
        bool resetAssembly = false;
        if ( pAssembler->IsInitialized() )
        {
            if ( GetTickCount() - pAssembler->LastUpdateTicks() > MessageExpiryTicks )
            {
                // Old message in slot, recycle
                if ( !pAssembler->IsComplete() )
                {
                    // Old message was never completed
                    incompleteMessages++;
                }
                resetAssembly = true;
            }
            else
            if ( pAssembler->MessageId == messageId )
            {
                // this message is current
                if ( pAssembler->IsComplete() )
                {
                    // already completed, this packet is redundant
                    goto bailout;
                }
            }
            else
            {
                if ( !pAssembler->IsComplete() )
                {
                    // the message in this slot is not that old, but needs to be recycled
                    // this condition should not happen
                    incompleteMessages++;
                }
                resetAssembly = true;
            }
        }
        else
        {
            // Message falls in new slot
            resetAssembly = true;
        }

        if ( resetAssembly )
        {
            hr = pAssembler->Initialize(messageSize, m_packetSize, payloadCount, redundancyCount);
            if ( FAILED(hr) )
            {
                malformedPackets++;
                goto bailout;
            }
            pAssembler->MessageId = messageId;
        }

        if ( messageSize != pAssembler->MessageLength() )
        {
            malformedPackets++;
            goto bailout;
        }

        hr = pAssembler->NewPacket(sequence, payload, cbPayload);
        if ( FAILED(hr) )
        {
            malformedPackets++;
        }
        else
        if ( S_OK == hr )
        {
            receivedMessages++;
            // dispatch
            m_messageCallback->MessageReceived(pAssembler->MessageId, pAssembler->MessageBytes(), pAssembler->MessageLength());
        }

        // Done processing MessageTypeFrame
        goto bailout;
    }

    //
    // Process an audio packet
    //

    if ( PacketTypeAudio == messageType )
    {
        // dispatch
        ULONGLONG timestamp = (WORD)sequence;
        timestamp <<= 16;
        timestamp |= (WORD)messageSize;
        timestamp <<= 32;
        timestamp |= (DWORD)messageId;

        if ( NULL != m_audioCallback )
        {
            m_audioCallback->AudioReceived(timestamp, payload, cbPayload);
        }
        receivedAudio++;
        goto bailout;
    }

    //
    // Unknown message type
    //

    malformedPackets++;

bailout:
    if ( 0 != receivedMessages
         || 0 != incompleteMessages
         || 0 != receivedAudio
         || 0 != malformedPackets )
    {
        EnterCriticalSection(&m_lock);
        {
            m_stats.ReceivedMessages += receivedMessages;
            m_stats.IncompleteMessages += incompleteMessages;
            m_stats.ReceivedAudio += receivedAudio;
            m_stats.MalformedPackets += malformedPackets;
        }
        LeaveCriticalSection(&m_lock);
    }
    return;
}

void CSession::Tick()
{
    bool ping = false;

    DWORD timestamp = 0;

    EnterCriticalSection(&m_lock);
    {
        switch ( m_state )
        {
        case Reset:
            ping = true;
            m_state = Disconnected;
            break;
        case Disconnected:
            ping = true;
            break;
        case Connecting:
            // Transmit one last ping and switch state
            ping = true;
            m_state = Connected;
            break;
        }
    }
    LeaveCriticalSection(&m_lock);

    if ( ping )
    {
        HRESULT hr = m_transport->Send(PacketTypePing, 0, 0, 0, NULL, 0);
        EnterCriticalSection(&m_lock);
        {
            if ( SUCCEEDED(hr) )
            {
                m_stats.SentPacketsSuccess++;
                m_stats.SentPings++;
            }
            else
            {
                m_stats.SentPacketsFail++;
            }

        }
        LeaveCriticalSection(&m_lock);
    }
}


STDMETHODIMP CSession::SendMessage
    (
        const BYTE* data,
        DWORD cbData
    )
{
    HRESULT hr = S_OK;
    BYTE messageType = PacketTypeMessage;
    DWORD messageId;
    DWORD sentPacketsFail = 0;
    DWORD sentPacketsSuccess = 0;
    DWORD sentPacketsBytes = 0;

    BYTE* redundancy = NULL;

    if ( NULL == data )
    {
        hr = E_POINTER;
        goto bailout;
    }
    if ( 0 == cbData )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    // Verify that we are connected and get a message number

    EnterCriticalSection(&m_lock);
    {
        switch ( m_state )
        {
        case Connected:
            messageId = ++m_dwTxMessageId;
            break;
        default:
            hr = E_ABORT;
            break;
        }
    }
    LeaveCriticalSection(&m_lock);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    // Compute the redundancy packets

    DWORD packetCount = (cbData + m_packetSize - 1 ) / m_packetSize;
    DWORD redundancyCount = (packetCount * m_redundancyPerThousand + 1000 - 1)/ 1000;
    redundancy = new BYTE[m_packetSize*redundancyCount];
    if ( NULL == redundancy )
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
	senderMatrix->ComputeRedundancy(const_cast<BYTE*>(data), cbData, redundancy, m_packetSize);
	delete senderMatrix;

    // Transmit the data packets

    WORD sequence;
    const BYTE* packetData = data;
    DWORD cbPacketDataRemaining = cbData;
    for ( sequence = 0 ; sequence < packetCount; sequence++ )
    {
        DWORD cbPacket = cbPacketDataRemaining > m_packetSize ? m_packetSize : cbPacketDataRemaining;
        hr = m_transport->Send(messageType, messageId, cbData, sequence, packetData, cbPacket);
        if ( FAILED(hr) )
        {
            sentPacketsFail++;
            goto bailout;
        }
        sentPacketsSuccess++;
        sentPacketsBytes += cbPacket;
        packetData += cbPacket;
        cbPacketDataRemaining -= cbPacket;
    }

    // Transmit the redundancy packets

    packetData = redundancy;
    for ( DWORD i = 0 ; i < redundancyCount; i ++ )
    {
        hr = m_transport->Send(messageType, messageId, cbData, sequence, packetData, m_packetSize);
        if ( FAILED(hr) )
        {
            sentPacketsFail++;
            goto bailout;
        }
        sentPacketsSuccess++;
        sentPacketsBytes += m_packetSize;
        sequence++;
        packetData += m_packetSize;
    }

bailout:
    EnterCriticalSection(&m_lock);
    {
        m_stats.SentPacketsFail += sentPacketsFail;
        m_stats.SentPacketsSuccess += sentPacketsSuccess;
        m_stats.SentPacketsBytes += sentPacketsBytes;
        if ( FAILED(hr) )
        {
            m_stats.SentMessagesFail++;
        }
        else
        {
            m_stats.SentMessagesSuccess++;
        }
    }
    LeaveCriticalSection(&m_lock);
    if ( NULL != redundancy )
    {
        delete [] redundancy;
    }
    return hr;
}


STDMETHODIMP CSession::QueryStats (Statistics* statsOut)
{
    if ( NULL == statsOut )
    {
        return E_POINTER;
    }

    EnterCriticalSection(&m_lock);
    {
        *statsOut = m_stats;
    }
    LeaveCriticalSection(&m_lock);

    return S_OK;
}

STDMETHODIMP CSession::SendAudio
    (
        LONGLONG timestamp,
        const BYTE* data,
        DWORD cbData
    )
{
    HRESULT hr = S_OK;
    BYTE messageType = PacketTypeAudio;
    DWORD sentPacketsFail = 0;
    DWORD sentPacketsSuccess = 0;

    if ( NULL == data )
    {
        hr = E_POINTER;
        goto bailout;
    }
    if ( 0 == cbData )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    // Verify that we are connected

    EnterCriticalSection(&m_lock);
    {
        switch ( m_state )
        {
        case Connected:
            break;
        default:
            hr = E_ABORT;
            break;
        }
    }
    LeaveCriticalSection(&m_lock);
    if ( FAILED(hr) )
    {
        goto bailout;
    }

    // Transmit the data packets

    DWORD messageId = (DWORD)(timestamp);
    timestamp >>= 32;
    DWORD messageSize = (WORD)(timestamp);
    timestamp >>= 16;
    WORD sequence = (WORD)(timestamp);
    hr = m_transport->Send(messageType, messageId, messageSize, sequence, data, cbData);
    if ( FAILED(hr) )
    {
        sentPacketsFail++;
        goto bailout;
    }
    sentPacketsSuccess++;

bailout:
    EnterCriticalSection(&m_lock);
    {
        m_stats.SentPacketsFail += sentPacketsFail;
        m_stats.SentPacketsSuccess += sentPacketsSuccess;
        if ( FAILED(hr) )
        {
            m_stats.SentAudioFail++;
        }
        else
        {
            m_stats.SentAudioSuccess++;
            m_stats.SentPacketsBytes += cbData;
        }
    }
    LeaveCriticalSection(&m_lock);
    return hr;
}
