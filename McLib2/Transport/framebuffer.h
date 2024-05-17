namespace HolochatNetworking
{
    class CFrameBuffer :
        public IMessageCallback,
        public IMessageBuffer
    {
    protected:
        friend NetworkFactory;
        CFrameBuffer();
        virtual ~CFrameBuffer();
        HRESULT Initialize(SIZE_T maxBufferSize);

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
        STDMETHODIMP_(void) MessageReceived
            (
                DWORD messageId,
                const BYTE* payload,
                DWORD cbPayload
            );

        //
        // IMessageBuffer
        //
    public:
        STDMETHODIMP AcquireBuffer
            (
                const BYTE** bufferOut,
                SIZE_T* lengthOut
            );
        STDMETHODIMP ReleaseBuffer
            (
                const BYTE* buffer
            );

    protected:
        LONG m_cRef;

    protected:
        SIZE_T m_cbMax;
        CRITICAL_SECTION m_lock;
        int m_readIndex;
        DWORD m_lastMessageID;
        SIZE_T m_bufferSizes[2];
        BYTE* m_buffers[2];
    };
}
