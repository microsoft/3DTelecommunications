namespace HolochatNetworking
{
    struct TransportPacket :
        public WSAOVERLAPPED
    {
        static const DWORD PacketSize = 1448;
        WSABUF Wsabuf;
        BYTE Data[PacketSize];

        TransportPacket()
        {
            Reset();
        }

        void Reset ()
        {
            Internal = 0;
            InternalHigh = 0;
            Offset = 0;
            OffsetHigh = 0;
            hEvent = NULL;
            Wsabuf.buf = (char*)Data;
            Wsabuf.len = PacketSize;
        }
    };

    class CEndpoint
    {
    public:
        CEndpoint();
        virtual ~CEndpoint();

    protected:
        HRESULT Initialize();
        void Close();

    protected:
        bool m_bWinsockInit;
        SOCKADDR_STORAGE m_localAddr;
        SOCKADDR_STORAGE m_remoteAddr;
	    SOCKET m_socket;
    };

    class CUdpEndpoint :
        public CEndpoint
    {
    public:
        CUdpEndpoint();
        virtual ~CUdpEndpoint();
    public:
        HRESULT Initialize(USHORT localPort, PCTSTR targetName,  USHORT targetPort, DWORD cbBuffer, HANDLE hIOCP, ULONG_PTR key);
        HRESULT PostAsyncSend(TransportPacket* pPacket);
        HRESULT PostAsyncReceive(TransportPacket* pPacket);
        void Close();

    protected:
        HANDLE m_hIOCP;
    };


    class CTcpEndpoint :
        public CEndpoint,
        public ITcpSocket
    {
    public:
        CTcpEndpoint();
        virtual ~CTcpEndpoint();

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
        // ITcpSocket
        //
    public:
        STDMETHODIMP Send
            (
                const BYTE* payload,
                DWORD cbPayload
            );
        STDMETHODIMP Receive
            (
                BYTE* payload,
                DWORD* pcbPayload
            );
        STDMETHODIMP Close();

    public:
        HRESULT Initialize(PCTSTR targetName,  USHORT targetPort);

    protected:
        LONG m_cRef;
    };

}
