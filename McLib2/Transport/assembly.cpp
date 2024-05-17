#include "stdafx.h"
#include "assembly.h"
#include "FecMatrix.h"

using namespace HolochatNetworking;
using namespace VideoTransmission;

CMessageAssembly::CMessageAssembly() :
    m_complete(false),
    m_length(0),
    m_bytes(NULL),
    m_packetLength(0),
    m_packets(0),
    m_redundancy(0),
    m_packetPresent(NULL),
    m_receivedPackets(0),
    m_redundancyRank(0),
    MessageId(0)
{
    m_lastUpdateTicks = GetTickCount();
}

CMessageAssembly::~CMessageAssembly()
{
    if ( NULL != m_bytes )
    {
        delete [] m_bytes;
    }
    if ( NULL != m_packetPresent )
    {
        delete [] m_packetPresent;
    }
}

HRESULT CMessageAssembly::Initialize(DWORD messageLength, DWORD packetLength, DWORD packets, DWORD redundancy)
{
    HRESULT hr = S_OK;

    m_lastUpdateTicks = GetTickCount();

    if ( NULL != m_bytes )
    {
        delete [] m_bytes;
        m_bytes = NULL;
    }
    if ( NULL != m_packetPresent )
    {
        delete [] m_packetPresent;
        m_packetPresent = NULL;
    }

    m_complete = false;
    m_length = messageLength;

    m_packetPresent = new bool[(packets+redundancy)];
    if ( NULL == m_packetPresent )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }
	for ( DWORD i=0; i < (packets+redundancy); i++ )
	{
		m_packetPresent[i] = false;
	}

    m_bytes = new BYTE [packetLength * (packets+redundancy)];
    if ( NULL == m_bytes )
    {
        hr = E_OUTOFMEMORY;
        goto bailout;
    }
    m_packetLength = packetLength;
    m_packets = packets;
    m_redundancy = redundancy;

    m_receivedPackets = 0;
    m_redundancyRank = 0;

bailout:
    return hr;
}

HRESULT CMessageAssembly::NewPacket(WORD sequenceNumber, const BYTE* data, DWORD cbData)
{
    HRESULT hr = S_FALSE; // not completed by default

    m_lastUpdateTicks = GetTickCount();

    if ( m_complete )
    {
        // already completed
        hr = S_OK;
        goto bailout;
    }

	if ( sequenceNumber  >= (m_packets+m_redundancy) )
    {
        hr = E_INVALIDARG;
        goto bailout;
    }

    if ( m_packetPresent[sequenceNumber] )
    {
        // dupe packet
        goto bailout;
    }

    // Copy the data
	DWORD position = sequenceNumber * m_packetLength;
	// Copy the packet, add zeroes if needed.
	memcpy(m_bytes+position, data, cbData);
	if ( cbData < m_packetLength )
	{
		memset(m_bytes+position+cbData, 0, m_packetLength-cbData);
	}

    // Mark the segment as received 
    m_packetPresent[sequenceNumber] = TRUE;
    m_receivedPackets++;
    if ( sequenceNumber >= m_packets )
    {
        // this is a redundant packet
        WORD redundancyRank = sequenceNumber-(WORD)m_packets+1;
        if ( redundancyRank > m_redundancyRank )
        {
            m_redundancyRank = redundancyRank;
        }
    }

    // message completed?
    if ( m_receivedPackets >= m_packets )
    {
        if ( m_redundancyRank > 0 )
        {
            // we need to apply one or more redundancy packets
			FecMatrix * receiveMatrix = new FecMatrix(m_packets, m_redundancy);
			receiveMatrix->ReconstructBuffer(m_bytes, m_length, m_bytes + (m_packets*m_packetLength), m_packetLength, m_packetPresent);
			delete receiveMatrix;
        }

        m_complete  = true;
        hr = S_OK;
    }

bailout:
    return hr;
}

