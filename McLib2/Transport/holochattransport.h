namespace HolochatNetworking
{
    [uuid(4b7517fb-cf5d-4d7a-8c9b-09226457146a)]
    interface IMessageBuffer :
        public IUnknown
    {
        STDMETHOD(AcquireBuffer)
            (
                const BYTE** bufferOut,
                SIZE_T* lengthOut
            ) PURE;
        STDMETHOD(ReleaseBuffer)
            (
                const BYTE* buffer
            ) PURE;
    };

    [uuid(dd764c9a-406e-415e-9a79-a997ca4b90a9)]
    interface IMessageCallback :
        public IUnknown
    {
        STDMETHOD_(void, MessageReceived)
            (
                DWORD messageId,
                const BYTE* payload,
                DWORD cbPayload
            ) PURE;
    };

    [uuid(94221565-08c3-4925-a01b-62cd6f1538b9)]
    interface IAudioCallback :
        public IUnknown
    {
        STDMETHOD_(void, AudioReceived)
            (
                LONGLONG timestamp,
                const BYTE* payload,
                DWORD cbPayload
            ) PURE;
    };

    [uuid(998639f2-47ab-475f-9c16-97cad1564bb8)]
    interface ISession :
        public IUnknown
    {
        struct Statistics
        {
            DWORD SentMessagesSuccess;
            DWORD SentMessagesFail;
            DWORD SentAudioSuccess;
            DWORD SentAudioFail;
            DWORD SentPacketsSuccess;
            DWORD SentPacketsFail;
            DWORD SentPacketsBytes;
            DWORD SentPings;
            DWORD ReceivedMessages;
            DWORD IncompleteMessages;
            DWORD ReceivedAudio;
            DWORD ReceivedPackets;
            DWORD ReceivedPacketsBytes;
            DWORD MalformedPackets;
            DWORD ReceivedPings;
        };

        STDMETHOD_(BOOL, IsConnected) () PURE;
        STDMETHOD(SendMessage)
            (
                const BYTE* data,
                DWORD cbData
            ) PURE;
        STDMETHOD(SendAudio)
            (
                LONGLONG timestamp,
                const BYTE* data,
                DWORD cbData
            ) PURE;
        STDMETHOD(Close)() PURE;
        STDMETHOD(QueryStats)(Statistics* statsOut) PURE;
    };

    [uuid(8f04210d-7748-4ed1-89a3-3891264c10ba)]
    interface ITransportCallback :
        public IUnknown
    {
        STDMETHOD_(void, Received)
            (
                BYTE messageType,
                DWORD messageId,
                DWORD messageSize,
                WORD sequence,
                const BYTE* payload,
                DWORD cbPayload
            ) PURE;
    };

    [uuid(8f04210d-7748-4ed1-89a3-3891264c10ba)]
    interface ITransport :
        public IUnknown
    {
        STDMETHOD_(DWORD, MaxPayloadOctets) () PURE;
        STDMETHOD(Send)
            (
                BYTE messageType,
                DWORD messageId,
                DWORD messageSize,
                WORD sequence,
                const BYTE* payload,
                DWORD cbPayload
            ) PURE;
        STDMETHOD(Close)() PURE;
    };

    [uuid(dc82e930-7145-4336-a949-a500abe049d2)]
    interface ITcpSocket :
        public IUnknown
    {
        STDMETHOD(Send)
            (
                const BYTE* payload,
                DWORD cbPayload
            ) PURE;
        STDMETHOD(Receive)
            (
                BYTE* payload,
                DWORD* pcbPayload
            ) PURE;
        STDMETHOD(Close)() PURE;
    };

    class NetworkFactory
    {
    public:
        static HRESULT WINAPI CreateMessageBuffer
            (
                SIZE_T maxBufferSize,
                IMessageBuffer** messageBufferOut
            );

        static HRESULT WINAPI CreateSession
            (
                USHORT localPort,
                PCTSTR targetName,
                USHORT targetPort,
                IMessageCallback* messageCallback,
                IAudioCallback* audioCallback,
                ISession** sessionOut
            );

        static HRESULT WINAPI CreateTransport ( USHORT localPort, PCTSTR targetName, USHORT targetPort, ITransportCallback* callback, ITransport** transportOut );

        static HRESULT WINAPI CreateTcpSocket
        (
            PCTSTR targetName,
            USHORT targetPort,
            ITcpSocket** socketOut
        );
};
}
