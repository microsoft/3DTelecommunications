namespace HolochatNetworking
{
    class CMessageAssembly
    {
    public:
        CMessageAssembly();
        virtual ~CMessageAssembly();
    public:
        HRESULT Initialize(DWORD messageLength, DWORD packetLength, DWORD packets, DWORD redundancy);
        HRESULT NewPacket(WORD sequenceNumber, const BYTE* data, DWORD cbData);
    public:
        DWORD LastUpdateTicks() { return m_lastUpdateTicks; }
        bool IsInitialized() { return NULL != m_bytes; }
        bool IsComplete() { return m_complete; }
        DWORD MessageLength() { return m_length; }
        const BYTE* MessageBytes() { return m_bytes; }
    public:
        DWORD MessageId;

    protected:
        DWORD m_lastUpdateTicks;
        bool m_complete;
        DWORD m_length;
        BYTE* m_bytes;
        DWORD m_packetLength;
        DWORD m_packets;
        DWORD m_redundancy;
        bool* m_packetPresent;
        DWORD m_receivedPackets;
        WORD m_redundancyRank;
    };
}
