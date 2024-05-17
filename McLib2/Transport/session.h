namespace HolochatNetworking
{
    class CSession :
        public ISession,
        public ITransportCallback
    {
    protected:
        friend NetworkFactory;
        CSession();
        virtual ~CSession();
        HRESULT Initialize(USHORT localPort, PCTSTR targetName, USHORT targetPort, IMessageCallback* messageCallback, IAudioCallback* audioCallback);

        //
        // IUnknown
        //
    public:
        STDMETHODIMP QueryInterface
            (
                REFIID riid,
                __deref_out void **ppv
            );
        STDMETHODIMP_(ULONG) AddRef ();
        STDMETHODIMP_(ULONG) Release ();

        //
        // ISession
        //
    public:
        STDMETHODIMP_(BOOL) IsConnected ();
        STDMETHODIMP SendMessage
            (
                const BYTE* data,
                DWORD cbData
            );
        STDMETHODIMP SendAudio
            (
                LONGLONG timestamp,
                const BYTE* data,
                DWORD cbData
            );
        STDMETHODIMP Close ();
        STDMETHODIMP QueryStats (Statistics* statsOut);

        //
        // ITransportCallback
        //
    public:
        STDMETHODIMP_(void) Received
            (
                BYTE messageType,
                DWORD messageId,
                DWORD messageSize,
                WORD sequence,
                const BYTE* payload,
                DWORD cbPayload
            );

    public:
        static const DWORD TickInterval;

    protected:
        static const BYTE PacketTypePing;
        static const BYTE PacketTypeMessage;
        static const BYTE PacketTypeAudio;
        void Tick();
        void Reassemble
                (
                    DWORD packetCount,
                    DWORD redundancyRank,
                    DWORD sequence,
                    const BYTE* payload,
                    DWORD cbPayload
                );

    protected:
        LONG m_cRef;
        IMessageCallback* m_messageCallback;
        IAudioCallback* m_audioCallback;
        ITransport* m_transport;
        CRITICAL_SECTION m_lock;
        DWORD m_startTicks;
        DWORD m_packetSize;
        DWORD m_redundancyPerThousand;
        enum State
        {
            Reset,
            Disconnected,
            Connecting,
            Connected
        } m_state;
        DWORD m_lastUpdateTicks;
        Statistics m_stats;

    protected:
        DWORD m_dwTxMessageId;
        DWORD m_dwRxMessageId;

    protected:
        static const DWORD AssemblyBuffers = 16;
        static const DWORD MessageExpiryTicks = 3500;
        CMessageAssembly m_messageAssemblyBuffers[AssemblyBuffers];
    };
}
