namespace HolochatNetworking
{
    class CTransport :
        public ITransport
    {
    protected:
        friend NetworkFactory;
        CTransport();
        virtual ~CTransport();
        HRESULT Initialize(USHORT localPort, PCTSTR targetName, USHORT targetPort, ITransportCallback* callback);

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
        // ITransport
        //
    public:
        STDMETHODIMP_(DWORD) MaxPayloadOctets ();
        STDMETHODIMP Send
            (
                BYTE messageType,
                DWORD messageId,
                DWORD messageSize,
                WORD sequence,
                const BYTE* payload,
                DWORD cbPayload
            );
        STDMETHODIMP Close();

    protected:
        static DWORD WINAPI ThreadStart( void* param );
        HRESULT ThreadMain();
        static const DWORD ClockTicks = 107;

    protected:
        static const DWORD HeaderSize;
        void Dispatch(BYTE* pMsg, DWORD cbMsg);

    protected:
        LONG m_cRef;
        HANDLE m_hIOCP;
        CUdpEndpoint m_endpoint;
        HANDLE m_hThread;
        ITransportCallback* m_callback;
    };
}
