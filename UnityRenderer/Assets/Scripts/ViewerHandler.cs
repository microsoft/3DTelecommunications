using UnityEngine;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Threading;
using System.Collections.Concurrent;
using System.Net.Sockets;
using System.Net;
using System.Threading.Tasks;
using System.IO;
using Holoportation;
using Assets.Scripts;

public enum PacketType
{
    Renderer_Get2DTextures,
    Fusion_ToggleQuality,
    Heartbeat
}

public class MJPEGFrame
{
    public byte[] mjpegData;
    public int mjpegColorSize;
}

public class ViewerHandler : MonoBehaviour
{
    public bool useViewHandler = false;
    public HoloPortScript holoportScriptToTransmit;
    public bool isRunning = false;
    public int responsePort = 7777;

    private HoloportControlPanelConnector holoCPC;

    private int mjpegDataBufferSizePerPod;
    private MJPEGFrame[] allMjpegData;

    private TimeSpan sendMessageTimeout = TimeSpan.FromMilliseconds(100);

    /// <summary>
    /// A mapping of when the last 3d mesh data was sent for a client
    /// </summary>
    private ConcurrentDictionary<string, DateTimeOffset> last3DSend = new ConcurrentDictionary<string, DateTimeOffset>();

    private TcpListener server;
    private Thread networkThread;
    private CancellationTokenSource networkThreadTokenSource;

    /// <summary>
    /// Temporary reference to hold vertex data before being sent on a stream
    /// </summary>
    private byte[] rawVertexData;

    /// <summary>
    /// Temporary reference to hold index data before being sent on a stream
    /// </summary>
    private byte[] rawIndexData;

    /// <summary>
    /// A list of clients and their last heartbeat
    /// </summary>
    private Dictionary<string, ViewerClient> clients =
        new Dictionary<string, ViewerClient>();

    //SETTINGS
    public double cNetworkTimeoutSeconds = 3.0;
    private int num3DPacketsSentSinceUpdate = 0;
    private DateTime lastFPSUpdate = DateTime.Now;

    /// <summary>
    /// The timeout before marking a client "offline"
    /// </summary>
    public readonly int clientOfflineTimeoutSeconds = 5 * 60 /* 5 minutes in seconds */;

    private Color32 unusedBackgroundColor = Color.clear;
    private byte[] unusedBackgroundColorBytes = new byte[] { 0, 0, 0, 0 };

    /// <summary>
    /// The number of clients currently connected
    /// </summary>
    public int ClientCount
    {
        get
        {
            return clients.Count;
        }
    }

    /// <summary>
    /// Gets the transmission fps for the 3d data
    /// </summary>
    public float Network3DTransmitFPS
    {
        get;
        private set;
    }

    /// <summary>
    /// Gets the transmission fps for the 2d data
    /// </summary>
    public float Network2DTransmitFPS
    {
        get;
        private set;
    }

    /// <summary>
    /// Color used to indicate unused / background color of the mesh texture data
    /// Defined in Unity Editor in the textureCam/BackgroundColor
    /// </summary>
    public Color32 UnusedBackgroundColor
    {
        get { return unusedBackgroundColor; }
        set
        {
            unusedBackgroundColor = value;
            unusedBackgroundColorBytes = new byte[] {
                value.r,
                value.g,
                value.b,
                value.a
            };
        }
    }

    /// <summary>
    /// Update loop
    /// </summary>
    private void Update()
    {
        if (clients.Count > 0)
        {
            Network3DTransmitFPS = Utils.ComputeFPS(lastFPSUpdate, DateTimeOffset.Now, (float)num3DPacketsSentSinceUpdate / clients.Count, Network3DTransmitFPS, 0.01f);
            lastFPSUpdate = DateTime.Now;
            num3DPacketsSentSinceUpdate = 0;
        }
    }

    // Start is called before the first frame update
    private void Start()
    {
        if (useViewHandler)
        {
            SetupServerAndStart();
        }
    }

    /// <summary>
    /// Behavior has been disabled
    /// </summary>

    private void OnDisable()
    {
        if (networkThread != null)
        {
            networkThreadTokenSource.Cancel();
            try 
            { 
                server.Stop();
            } 
            catch (SocketException) { }

            // Wait for it to finish
            networkThread.Join();

            server = null;
            networkThread = null;
        }

        clients.Clear();
        isRunning = false;
    }

    /// <summary>
    /// Sets up the networking server and starts
    /// </summary>
    private void SetupServerAndStart()
    {
        if (holoportScriptToTransmit != null)
        {
            mjpegDataBufferSizePerPod =
                holoportScriptToTransmit.RigConfiguration.ColorWidth *
                holoportScriptToTransmit.RigConfiguration.ColorHeight *
                holoportScriptToTransmit.RigConfiguration.ColorPixelBytes;

            allMjpegData = new MJPEGFrame[holoportScriptToTransmit.RigConfiguration.NumPods];
            for (int i = 0; i < allMjpegData.Length; ++i)
            {
                allMjpegData[i] = new MJPEGFrame();
                allMjpegData[i].mjpegData = new byte[mjpegDataBufferSizePerPod];
                allMjpegData[i].mjpegColorSize = 0;
            }

            holoportScriptToTransmit.onDataReadyCallback += OnHoloportDataReady;
            holoCPC = holoportScriptToTransmit.GetComponent<HoloportControlPanelConnector>();
        }
        else
        {
            Debug.LogError("Missing HoloportRenderer for ViewHandler!");
            return;
        }

        networkThreadTokenSource = new CancellationTokenSource();
        networkThread = new Thread(new ParameterizedThreadStart(ViewerNetworkingThread));
        networkThread.Start(networkThreadTokenSource.Token);

        isRunning = true;
    }

    /// <summary>
    /// Listener for when new holoportation data is ready
    /// </summary>
    /// <param name="mjpegColor">All the camera images</param>
    /// <param name="mjpegColorSizes">The byte sizes for each camera image</param>
    private void OnHoloportDataReady(byte[] mjpegColor, int[] mjpegColorSizes)
    {
        int frameSize = holoportScriptToTransmit.RigConfiguration.ColorWidth * holoportScriptToTransmit.RigConfiguration.ColorHeight * holoportScriptToTransmit.RigConfiguration.ColorPixelBytes;
        for (int i = 0; i < holoportScriptToTransmit.RigConfiguration.NumPods; ++i)
        {
            lock (allMjpegData[i])
            {
                Buffer.BlockCopy(mjpegColor, i * frameSize, allMjpegData[i].mjpegData, 0, mjpegColorSizes[i]);

                allMjpegData[i].mjpegColorSize = mjpegColorSizes[i];
            }
        }
    }

    /// <summary>
    /// Handles the networking between the renderer and the viewer
    /// </summary>
    private void ViewerNetworkingThread(object objToken)
    {
        //IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Any, 11000);

        // Create a TCP/IP socket.  
        //Socket socket = new Socket(localEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
        server = new TcpListener(IPAddress.Any, responsePort);
        server.Start();

        CancellationToken token = (CancellationToken)objToken;
        int id = 0;
        while (!token.IsCancellationRequested)
        {
            try
            {
                TcpClient client = server.AcceptTcpClient();
                Task.Run(() => ClientThread($"{id++}", client, token), token);
            } catch (SocketException) { }
        }
    }

    /// <summary>
    /// Thread to handle an individual client
    /// </summary>
    /// <param name="ob"></param>
    private async void ClientThread(string clientId, TcpClient tcpClient, CancellationToken token)
    {
        Debug.Log($"Client connected: {clientId}");

        using (NetworkStream stream = tcpClient.GetStream())
        {
            ViewerClient client = new ViewerClient()
            {
                Id = clientId,
                Outbox = new ConcurrentQueue<ViewerMessage>(),
                LastMessageTimestamp = DateTime.Now,
                Stream = stream,
                Tcp = tcpClient,
            };

            clients[clientId] = client;
            try
            {
                stream.WriteTimeout = 100;
                while (!token.IsCancellationRequested && tcpClient.Connected)
                {
                    await ProcessNextOutgoingMessage(client, stream, token);

                    if (!token.IsCancellationRequested && tcpClient.Connected)
                    {
                        ProcessNextIncomingMessage(client, stream, token);
                        Thread.Sleep(10);
                    }
                }
            }
            catch (Exception e)
            {
                if (!IsServerDisconnectException(e))
                {
                    Debug.LogError($"ViewerNetworkingThread send message exception: {e.Message}\n{e.StackTrace}");
                    //Application.Quit(1);
                    return;
                }
            }
            finally
            {
                stream.Dispose();
                tcpClient.Dispose();
                clients.Remove(clientId);
            }
        }
    }

    /// <summary>
    /// Processes the outgoing messages on the given socket
    /// </summary>
    /// <param name="socket">The socket to process</param>
    /// <param name="token">The cancellation token for when the processing should stop</param>
    private async Task ProcessNextOutgoingMessage(ViewerClient client, NetworkStream clientStream, CancellationToken token)
    {
        // Just go through the messages that we started with
        // don't worry about the ones added while we're sending messages
        if (clientStream.CanWrite && !token.IsCancellationRequested && client.Outbox.Count > 0)
        {
            ViewerMessage message;
            if (!client.Outbox.TryDequeue(out message))
            {
                Debug.LogWarning($"Couldn't process message from message queue!");
            }
            else
            {
                message.Position = 0;
                await message.CopyToAsync(clientStream, 1024 * 1024, token);
                message.Dispose();

                /// The 3D data
                if (message.Type == PacketType.Fusion_ToggleQuality)
                {
                    num3DPacketsSentSinceUpdate++;
                    last3DSend[client.Id] = DateTimeOffset.Now;
                }
            }
        }
        clientStream.Flush();
    }


    /// <summary>
    /// Reads a message from the viewer from the given socket
    /// </summary>
    /// <param name="socket">The socket to read the message from</param>
    private bool ProcessNextIncomingMessage(ViewerClient client, NetworkStream clientStream, CancellationToken token)
    {
        // Can we even read from the stream and there is data available
        if (clientStream.CanRead && clientStream.DataAvailable)
        {
            // If so, then try to read from it
            byte msgId;
            if (clientStream.TryReadByte(out msgId, "Message Id"))
            {
                // Update the last time we've heard from this client
                client.LastMessageTimestamp = DateTimeOffset.Now;

                //Actually PROCESS the REQUEST
                PacketType packetType = (PacketType)msgId;
                if (packetType == PacketType.Renderer_Get2DTextures)
                {
                    // The indexes requested are individual bytes, this assumes there will not be more than 255 cameras available
                    // [1, 2, 6] = Camera index 1, 2 and 6 are requested
                    byte[] cameraIdxData;
                    if (!clientStream.TryReadArrayWithLength(out cameraIdxData))
                    {
                        Debug.LogWarning("ViewHandler: missing camera indices");
                        return false;
                    }

                    // Validate all of the indices
                    foreach (byte cameraIdx in cameraIdxData)
                    {
                        if (cameraIdx < 0 || cameraIdx >= allMjpegData.Length)
                        {
                            Debug.LogWarning($"ViewHandler: invalid requested camera idx ({ cameraIdx })");
                            return false;
                        }
                    }

                    // Send the image data back
                    Send2DImageDataToViewer(client.Id, clientStream, packetType, cameraIdxData.Select(n => (int)n).ToList());
                }
                else if (packetType == PacketType.Fusion_ToggleQuality)
                {
                    Debug.Log($"Toggle quality received from client: {client.Id}");
                    //byte[] boolData = new byte[1];
                    //received = socket.TryReadFrameBytes(TimeSpan.FromSeconds(cNetworkTimeoutSeconds), out boolData);
                    //if (!received)
                    //{
                    //    Debug.LogWarning("ViewHandler: missing frame");
                    //    return false;
                    //}
                    //holoCPC.SendToggleFusionQuality(BitConverter.ToBoolean(boolData, 0));
                }
                else if (packetType == PacketType.Heartbeat)
                {
                    Debug.Log($"Heartbeat received from client: {client.Id}");
                    SendHeartbeatACKToViewer(client.Id);
                }
                else
                {
                    Debug.LogWarning($"Unknown packet type: {packetType}");
                    return false;
                }
            }
            else
            {
                Debug.LogWarning($"Missing packet type header");
                return false;
            }
        }

        return true;

    }

    /// <summary>
    /// Returns true if the given exception is an exception that indicates a diconnection from the server
    /// </summary>
    /// <param name="e"></param>
    /// <returns></returns>
    private static bool IsServerDisconnectException(Exception e)
    {
        SocketException se = e as SocketException;
        if (se == null)
        {
            // Sometimes it is on the InnerException prop
            se = e.InnerException as SocketException;
        }

        return se != null &&
            (se.SocketErrorCode == SocketError.ConnectionAborted ||
             se.SocketErrorCode == SocketError.ConnectionRefused ||
             se.SocketErrorCode == SocketError.ConnectionReset);
    }

    /// <summary>
    /// Sends the heartbeat acknowledgement to the given client
    /// </summary>
    /// <param name="client">The client to send the acknowledgement to</param>
    private void SendHeartbeatACKToViewer(string client)
    {
        ViewerMessage msg = new ViewerMessage(PacketType.Heartbeat);

        // Send client ID first
        msg.TryWriteString(client);
        msg.TryWriteByte((byte)PacketType.Heartbeat);

        clients[client].Outbox.Enqueue(msg);
    }

    /// <summary>
    /// Sends the HQ holoportation data frame to clients
    /// </summary>
    /// <param name="structToSend">The holoportation data</param>
    public void Send3DMeshDataToViewer(HoloportObjectStruct structToSend)
    {
        List<string> clientList = GetActiveClients().ToList();

        // No need to do any of this if there is no client
        if (clientList.Count > 0)
        {
            // Copy what we need out of the struct
            int vertexByteCount = structToSend.vertexCount * 3 * sizeof(ushort); // x y z = 3 components
            if (rawVertexData == null || vertexByteCount > rawVertexData.Length)
            {
                rawVertexData = new byte[vertexByteCount];
            }
            Buffer.BlockCopy(structToSend.positionXYZ_ushort, 0, rawVertexData, 0, rawVertexData.Length);

            int indexByteCount = structToSend.indexCount * sizeof(int);
            if (rawIndexData == null || indexByteCount > rawIndexData.Length)
            {
                rawIndexData = new byte[indexByteCount];
            }
            Buffer.BlockCopy(structToSend.indexData, 0, rawIndexData, 0, rawIndexData.Length);

            int vertexCount = structToSend.vertexCount;
            int indexCount = structToSend.indexCount;

            int textureDim = structToSend.textureDim;
            int perTriangleTextureDim = structToSend.perTriangleTextureDim;
            byte[] preAllocatedRenderTextureFetchArray_BYTE = structToSend.preAllocatedRenderTextureFetchArray_BYTE;

            // Stuff is allocated relative to the expected index count
            float usedTexturePercentage = (float)indexCount / SettingsManager.Instance.ExpectedIndexCountHQ;
            int firstNonBlankColorByte =
                preAllocatedRenderTextureFetchArray_BYTE.Length -
                (int)(usedTexturePercentage * preAllocatedRenderTextureFetchArray_BYTE.Length);

            MemoryStream baseMessage = new MemoryStream();

            baseMessage.TryWriteByte((byte)PacketType.Fusion_ToggleQuality);

            baseMessage.TryWriteInt(vertexCount);
            baseMessage.TryWriteInt(indexCount);

            baseMessage.TryWriteArrayWithLength(rawVertexData, vertexByteCount);
            baseMessage.TryWriteArrayWithLength(rawIndexData, indexByteCount);

            baseMessage.TryWriteInt(textureDim);
            baseMessage.TryWriteInt(perTriangleTextureDim);

            baseMessage.TryWriteArrayWithLength(preAllocatedRenderTextureFetchArray_BYTE, preAllocatedRenderTextureFetchArray_BYTE.Length - firstNonBlankColorByte, firstNonBlankColorByte);

            foreach (string client in clientList)
            {
                ViewerMessage msg = new ViewerMessage(PacketType.Fusion_ToggleQuality);

                msg.TryWriteString(client);

                baseMessage.Position = 0;
                baseMessage.CopyTo(msg);

                clients[client].Outbox.Enqueue(msg);
            }
        }
    }

    /// <summary>
    /// Sends Color2D image data back to the viewer
    /// </summary>
    /// <param name="client">The client to send the message to</param>
    /// <param name="packetType">The packet type to use for the message</param>
    /// <param name="cameraIndices">The indices of the cameras to send back</param>
    private void Send2DImageDataToViewer(string client, NetworkStream stream, PacketType packetType, IList<int> cameraIndices)
    {
        // ZMQ header: [sender ID] [EMPTY frame]
        /* Our Data: 
            [Packet Type] 
            [List of camera indices(byte[])] 
            
            For each camera idx
            [image data length in bytes (int)] 
            [full color bytes (byte[])]
        */

        ViewerMessage msg = new ViewerMessage(packetType);

        msg.TryWriteString(client);
        msg.TryWriteByte((byte)packetType);

        // Append the list of camera indices
        msg.TryWriteArrayWithLength(cameraIndices.Select(n => (byte)n).ToArray());

        // Add each of the camera data, in the order it was requested
        foreach (int cameraIdx in cameraIndices)
        {
            // Lock the camera data as we write it to the stream
            lock (allMjpegData[cameraIdx])
            {
                msg.TryWriteArrayWithLength(allMjpegData[cameraIdx].mjpegData, allMjpegData[cameraIdx].mjpegColorSize);
            }
        }

        clients[client].Outbox.Enqueue(msg);
    }

    /// <summary>
    /// Gets the list of active clients
    /// </summary>
    /// <returns></returns>
    private IEnumerable<string> GetActiveClients()
    {
        DateTimeOffset current = DateTimeOffset.Now;
        KeyValuePair<string, ViewerClient>[] currArray = clients.ToArray();
        foreach (KeyValuePair<string, ViewerClient> clientKVP in currArray)
        {
            // We haven't heard from this client recently
            if ((current - clientKVP.Value.LastMessageTimestamp).TotalSeconds > clientOfflineTimeoutSeconds)
            {
                // Remove it from the list of clients
                clients.Remove(clientKVP.Key);
            }
        }

        return clients.Keys;
    }

    /// <summary>
    /// Represents a message for the viewer
    /// </summary>
    private class ViewerMessage : MemoryStream
    {
        /// <summary>
        /// Constructor for ViewerMessage
        /// </summary>
        /// <param name="type"></param>
        public ViewerMessage(PacketType type)
        {
            Type = type;
        }

        /// <summary>
        /// The type of message
        /// </summary>
        public PacketType Type { get; set; }
    }

    private class ViewerClient
    {
        /// <summary>
        /// The id of the client
        /// </summary>
        public string Id { get; set; }

        /// <summary>
        /// The TCP client for the viewer client
        /// </summary>
        public TcpClient Tcp { get; set; }

        /// <summary>
        /// The data stream for the client
        /// </summary>
        public NetworkStream Stream { get; set; }

        /// <summary>
        /// The last time we heard from the client
        /// </summary>
        public DateTimeOffset LastMessageTimestamp { get; set; }

        /// <summary>
        /// The queue of messages to send to the client
        /// </summary>
        public ConcurrentQueue<ViewerMessage> Outbox { get; set; }
    }
}

