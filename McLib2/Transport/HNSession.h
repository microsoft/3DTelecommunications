///
/// Copyright (c) Microsoft Corporation. All Rights Reserved
///

#pragma once

namespace HoloNet
{

    class CMessageAssembly
    {
    public:
        CMessageAssembly();
        ~CMessageAssembly();
    public:
        HRESULT Initialize
            (
                REFIID sender,
                ULONG messageId,
                DWORD ticks,
                ULONG messageSize,
                ULONG packetSize,
                ULONG packets,
                ULONG redundancy
            );
        HRESULT NewData(USHORT sequenceNumber, const BYTE* p, ULONG cb);
        const BYTE* GetMessageBytes ()
        {
            return m_data;
        }
        ULONG GetMessageSize ()
        {
            return m_messageSize;
        }

    protected:
        HRESULT Realloc(ULONG size, ULONG packets);

    public:
        GUID Sender;
        ULONG MessageId;
        DWORD LastUpdateTicks;
    protected:
        ULONG m_messageSize;
        ULONG m_packetSize;
        ULONG m_packets;
        ULONG m_redundancy;
        ULONG m_receivedPackets;
        ULONG m_receivedRedundancy;
        ULONG m_sizeAllocated;
        BYTE* m_data;
        ULONG m_packetsAllocated;
        bool* m_packetsPresent;
    };

    struct PresenceEntry
    {
        GUID nodeId;
        ULONGLONG lastReceivedTick;
    };

    class CHoloNetSession :
        public INetSession,
        public INetSink
    {
    public:
        static const ULONG s_redundancyPerThousand = 0; // turned off, use 40 // per thousands

    protected:
        friend Factory;
        CHoloNetSession ();
    private:
        virtual ~CHoloNetSession ();

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
    // INetSession
    //
    public:
        HRESULT STDMETHODCALLTYPE AddSink (INetSessionSink* sink);
        HRESULT STDMETHODCALLTYPE Startup ();
        BOOL STDMETHODCALLTYPE IsConnected ();
        BOOL STDMETHODCALLTYPE IsPresent (REFIID id);
        HRESULT STDMETHODCALLTYPE SendPacket
            (
                BYTE packetType,
                const BYTE* pHeader,
                ULONG cbHeader,
                const BYTE* pPayload,
                ULONG cbPayload
            );
        HRESULT STDMETHODCALLTYPE SendMessage
            (
                BYTE messageType,
                const BYTE* pHeader,
                ULONG cbHeader,
                const BYTE* pPayload,
                ULONG cbPayload
            );
        HRESULT STDMETHODCALLTYPE Shutdown ();

    //
    // INetSink
    //
    public:
        void STDMETHODCALLTYPE Received (INetDatagram* datagram);
        void STDMETHODCALLTYPE Notify (HRESULT result);

    protected:
        LONG m_cRef;
        GUID m_id;
        CRITICAL_SECTION m_lock;
        bool m_bAllowLoopback;
        HANDLE m_hEvent;
        HANDLE m_hThread;
        enum State
        {
            Stopped,
            Disconnected,
            Connected,
        } m_state;
        DWORD m_lastSentTicks;
        DWORD m_lastReceivedTicks;
		DWORD m_lastAppearedTicks;
        DWORD m_dwAliveTime;
        ULONG m_lastMessageId;
        INetManager* m_manager;
        INetConnection* m_connection;
        INetSessionSink* m_sink;

        //
        // message reassembly
        //
    protected:
        static const ULONG AssemblyBuffers = 16;
        static const DWORD MessageExpiryTicks = 1000;
        CMessageAssembly m_messageAssemblies[AssemblyBuffers];
        ULONG m_lastAllocatedAssembly;

        //
        // presence tracking
        //
    protected:
        static const ULONG PresenceSlots = 8;
        PresenceEntry m_presenceTable[PresenceSlots];
        bool m_notifyAppeared;

        //
        // utility
        //
    protected:
        HRESULT Initialize
            (
                LPCWSTR url,
                REFIID id,
                bool allowLoopback,
                DWORD dwAliveTime
            );
        void Tick ();
        HRESULT SendPing ();
        HRESULT DispatchPing (REFIID sender);
        HRESULT DispatchPacket
            (
                REFIID sender,
                BYTE packetType,
                const BYTE* payload,
                DWORD cbPayload
            );
        HRESULT DispatchMessageFragment
            (
                REFIID sender,
                BYTE messageType,
                ULONG packetSize,
                ULONG messageId,
                ULONG messageSize,
                USHORT sequenceNumber,
                const BYTE* payload,
                DWORD cbPayload
            );
        bool RegisterPresent
            (
                REFIID sender
            );

        //
        // worker thread
        //
    protected:
        static const DWORD s_dwPollTime = 211;
        static const DWORD s_dwDefaultAliveTime = 1000;
        static const DWORD s_dwAppearTime = s_dwDefaultAliveTime;
        static const DWORD s_dwPingTime = 397;
        static const DWORD s_dwAppearedTime = 1555;
        HRESULT StartWorkerThread ();
        HRESULT StopWorkerThread ();
        static DWORD WINAPI ThreadStart (void* param);
        HRESULT ThreadMain ();
    };
}
