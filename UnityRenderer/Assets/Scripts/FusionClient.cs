using System;
using System.Collections.Concurrent;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    public class FusionClient : IFusionClient
    {
        const string RenderingClientImport = "rendering_client";
        [DllImport(RenderingClientImport)]
        private unsafe static extern int get_max_vertex_size();
        [DllImport(RenderingClientImport)]
        private unsafe static extern int get_max_index_size();
        [DllImport(RenderingClientImport)]
        private unsafe static extern IntPtr create_rendering_client(string host, string port);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void destroy_rendering_client(IntPtr pointer);
        [DllImport(RenderingClientImport)]
        private unsafe static extern bool client_get_counts(IntPtr pointer, int* nvertices, int* ntriangles);
        [DllImport(RenderingClientImport)]
        private unsafe static extern bool client_get_data(IntPtr pointer, int* vertex_data, int* index_data, char* color_images, char* audio_data, char* compressedColorData);
        [DllImport(RenderingClientImport)]
        private unsafe static extern int client_get_color_size(IntPtr pointer, int podID);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_set_resolution(IntPtr pointer, int width, int height, int colorPixelBytes);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_set_pod_count(IntPtr pointer, int count);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_get_simplified_mesh(IntPtr pointer, int* vertex_data, int* index_data, int* nVertices, int* nTriangles);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_set_should_simplify(IntPtr pointer, bool should);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_set_simplify_parameters(IntPtr pointer, double aggr, int targetTriCount);

        [DllImport(RenderingClientImport)]
        private unsafe static extern void client_set_audio_parameters(IntPtr pointer, int sampleRate, int sampleSize);
        [DllImport(RenderingClientImport)]
        private unsafe static extern int client_get_audio_stream_count(IntPtr pointer);
        [DllImport(RenderingClientImport)]
        private unsafe static extern int client_get_audio_stream_size(IntPtr pointer, int streamID);
        [DllImport(RenderingClientImport)]
        private unsafe static extern void WriteWavData(char* audioData, int audioDataSize);
        [DllImport(RenderingClientImport)]
        private unsafe static extern bool client_get_is_HQ_Frame(IntPtr pointer);

        /// <summary>
        /// Whether or not to write the incoming fusion packets
        /// </summary>
        private bool writeFusionPackets = false;

        /// <summary>
        /// The delay between writing frames to disc
        /// </summary>
        private int writeFrameSleepMS = 10;

        /// <summary>
        /// The queue used to write the raw frames to disk
        /// </summary>
        private ConcurrentQueue<HoloportObjectStruct> writeQueue = new ConcurrentQueue<HoloportObjectStruct>();

        /// <summary>
        /// Triggered when a new holoportation frame is ready
        /// </summary>
        public event FrameReadyHandler FrameReady;

        /// <summary>
        /// The token source used for stopping the background threads
        /// </summary>
        private CancellationTokenSource threadTokenSource;

        /// <summary>
        /// Pool of available structs to use when processing the data from fusion
        /// </summary>
        private ConcurrentQueue<HoloportObjectStruct> objPool;

        /// <summary>
        /// The number of indices in a triangle
        /// </summary>
        private const int cIndicesInTriangle = 3;

        /// <summary>
        /// Network Client Pointer
        /// </summary>
        private IntPtr networkClientPointer = IntPtr.Zero;

        /// <summary>
        /// The number of pods in the system
        /// </summary>
        private int numPods;

        /// <summary>
        /// The target number of fps to retreive from the server
        /// </summary>
        private float targetFPS;
        private string host;
        private string port;
        private int imageWidth;
        private int imageHeight;
        private int imageByteCount;
        private int audioSampleRate;
        private int audioSampleSize;

        /// <summary>
        /// The output folder for the frames being saved.
        /// </summary>
        private string outputFolder;

        /// <summary>
        /// A backup struct to use when processing network requests and there is no frame in the pool
        /// </summary>
        private HoloportObjectStruct backup;

        /// <summary>
        /// Creates a client with the given settings
        /// </summary>
        /// <param name="targetFPS"></param>
        /// <param name="settings"></param>
        public static FusionClient CreateWithSettings(SettingsManager settings, ConcurrentQueue<HoloportObjectStruct> pool)
        {
            HoloportObjectStruct backup = HoloportObjectStruct.CreateWithSettings(-1, SettingsManager.Instance);
            return new FusionClient(
                settings.FusionIP,
                settings.FusionPort,
                settings.NumPods,
                settings.FusionNetworkTargetFPS,
                settings.ColorImageWidth,
                settings.ColorImageHeight,
                settings.ColorPixelBytes,
                settings.AudioSampleRate,
                settings.AudioSampleSize,
                pool,
                backup,
                settings.WriteFusionFramesToDisk,
                settings.WriteFusionFramesToDiskFolder
            );
        }

        /// <summary>
        /// Constructs a new fusion client
        /// </summary>
        private FusionClient(
            string host,
            string port,
            int numPods,
            float targetFPS,
            int imageWidth,
            int imageHeight,
            int imageByteCount,
            int audioSampleRate,
            int audioSampleSize,
            ConcurrentQueue<HoloportObjectStruct> objPool,
            HoloportObjectStruct backup,
            bool writeFusionPackets,
            string outputFolder
        )
        {
            this.numPods = numPods;
            this.targetFPS = targetFPS;
            this.objPool = objPool;
            this.host = host;
            this.port = port;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.imageByteCount = imageByteCount;
            this.audioSampleRate = audioSampleRate;
            this.audioSampleSize = audioSampleSize;
            this.backup = backup;
            this.writeFusionPackets = writeFusionPackets;
            this.outputFolder = outputFolder;
        }

        /// <summary>
        /// Gets/sets if the client is currently paused
        /// </summary>
        public bool Paused
        {
            get;
            set;
        }

        /// <summary>
        /// Gets the current count of skipped frames
        /// </summary>
        public int CountSkippedFrames
        {
            get;
            private set;
        } = 0;

        /// <summary>
        /// Gets the speed of the network processing
        /// </summary>
        public float FPS
        {
            get;
            private set;
        } = 0;

        /// <summary>
        /// Starts the fusion client
        /// </summary>
        public void Start()
        {
            Stop();

            threadTokenSource = new CancellationTokenSource();

            Task.Run(() => NetworkThread(threadTokenSource.Token));

            if (writeFusionPackets)
            {
                Task.Run(() => RecordDataThread(threadTokenSource.Token, outputFolder));
            }
        }

        /// <summary>
        /// Pauses the output of the client
        /// </summary>
        public void Pause()
        {
            Paused = true;
        }

        /// <summary>
        /// Stops the fusion client
        /// </summary>
        public void Stop()
        {
            if (threadTokenSource != null)
            {
                threadTokenSource.Cancel();
                threadTokenSource = null;
            }
        }

        /// <summary>
        /// A thread for receiving the data from the networking sockets of the mesh recorder
        /// </summary>
        unsafe private void NetworkThread(object param)
        {
            networkClientPointer = create_rendering_client(host, port);
            client_set_pod_count(networkClientPointer, numPods);
            client_set_resolution(networkClientPointer, imageWidth, imageHeight, imageByteCount);

            // This is required for stuff to work right
            client_set_audio_parameters(networkClientPointer, audioSampleRate, audioSampleSize);

            CancellationToken token = (CancellationToken)param;
            int count = 0;
            DateTime prevTime = DateTime.Now;

            while (!token.IsCancellationRequested)
            {
                if (Paused)
                {
                    Thread.Sleep(200);
                    continue;
                }

                try
                {
                    int numVertices = 0;
                    int numTriangles = 0;

                    float newFPS = Utils.ComputeFPS(prevTime, DateTime.Now, 1, FPS, 0.2f);
                    if (newFPS <= targetFPS)
                    {
                        // check how many vertices and triangles are incoming
                        if (client_get_counts(networkClientPointer, &numVertices, &numTriangles))
                        {
                            if (SettingsManager.Instance.Verbosity() > 1)
                            {
                                Debug.Log(numVertices + " " + numTriangles);
                            }

                            if (client_get_is_HQ_Frame(networkClientPointer))
                            {
                                // read buffers from network to CPU memory
                                float timeDiff = (float)(DateTime.Now - prevTime).TotalMilliseconds;
                                HoloportObjectStruct currObj = GetObjectStructFromPool();

                                int newIndexCount = numTriangles * cIndicesInTriangle;
                                int newVertexCount = numVertices;
                                if (newIndexCount > currObj.indexData.Length)
                                {
                                    Debug.LogError("ERROR - Received more indices than supported!");
                                }
                                else if ((newVertexCount * cIndicesInTriangle) > currObj.vertexData.Length)
                                {
                                    Debug.LogError("ERROR - Received more vertices than supported!");
                                }
                                else
                                {
                                    bool received = TryReadHoloportationFrame(currObj);
                                    if (received)
                                    {
                                        //update these after we do indexCount check (since the orignal indexCount indicates allocated size)
                                        count++;

                                        currObj.indexCount = newIndexCount;
                                        currObj.vertexCount = newVertexCount;

                                        int audioStreamCount = client_get_audio_stream_count(networkClientPointer);
                                        for (int i = 0; i < audioStreamCount; ++i)
                                        {
                                            int clientAudioStreamSize = audioSampleRate * audioSampleSize;
                                            currObj.audioDataSize[i] = clientAudioStreamSize;
                                        }

                                        FPS = Utils.ComputeFPS(prevTime, DateTime.Now, 1, FPS, 0.2f);
                                        prevTime = DateTime.Now;

                                        // The frame isn't ready if we're writing the fusion packets
                                        if (writeFusionPackets)
                                        {
                                            writeQueue.Enqueue(currObj);
                                        }
                                        else if (currObj != backup)
                                        {
                                            FrameReady?.Invoke(this, currObj);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    Thread.Sleep(16);
                }
                catch (Exception e)
                {
                    Debug.LogError("HoloPortScript NetworkThread error, stopping");
                    Debug.LogException(e);
                    return;
                }
            }


            if (networkClientPointer != IntPtr.Zero)
            {
                destroy_rendering_client(networkClientPointer);
            }
        }

        /// <summary>
        /// Gets the next available object struct from the pool, otherwise falls back to the backup struct
        /// </summary>
        /// <returns></returns>
        private HoloportObjectStruct GetObjectStructFromPool()
        {
            // Use the backup struct if there are no structs available in the pool
            HoloportObjectStruct currObj = backup;
            if (objPool.Count == 0 || !objPool.TryDequeue(out currObj))
            {
                currObj = backup;
                CountSkippedFrames++;
            }

            return currObj;
        }

        /// <summary>
        /// Reads the holoportation frame from the server
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="currObj"></param>
        /// <returns></returns>
        private unsafe bool TryReadHoloportationFrame(HoloportObjectStruct currObj)
        {
            bool receivedData = false;

            try
            {
                fixed (void* vertexDataPtr = currObj.vertexData)
                fixed (int* indexDataPtr = currObj.indexData)
                fixed (void* colorImagesPtr = currObj.colorData)
                fixed (void* audioDataPtr = currObj.audioData)
                fixed (void* compressedColorPtr = currObj.rawMJPEGData)
                {
                    if (client_get_data(networkClientPointer, (int*)vertexDataPtr, indexDataPtr, (char*)colorImagesPtr, (char*)audioDataPtr, (char*)compressedColorPtr))
                    {
                        if (SettingsManager.Instance.Verbosity() > 1)
                        {
                            Debug.Log($"Client got data from server.");
                        }
                        for (int i = 0; i < this.numPods; ++i)
                        {
                            currObj.MJPEGDataSize[i] = client_get_color_size(networkClientPointer, i);
                            if (SettingsManager.Instance.Verbosity() > 1)
                            {
                                Debug.Log($"Pod {i} MJPEG data size is {currObj.MJPEGDataSize[i]}.");
                            }
                        }

                        receivedData = true;
                    }
                }
            } 
            catch (Exception e)
            {
                Debug.LogException(e);
            }

            return receivedData;
        }

        /// <summary>
        /// A thread used to record fusion data
        /// </summary>
        private void RecordDataThread(CancellationToken token, string outputFolder)
        {
            try
            {
                Directory.CreateDirectory(outputFolder);

                int frameNum = 0;
                while (!token.IsCancellationRequested)
                {
                    HoloportObjectStruct obj;
                    if (writeQueue.Count > 0 && writeQueue.TryDequeue(out obj))
                    {
                        string filePath =
                            Path.Combine($"{outputFolder}/", $"RendererHoloportationFrame{frameNum.ToString("00000")}.bin");
                        File.WriteAllBytes(filePath, obj.Serialize());
                        frameNum++;

                        if (obj != backup)
                        {
                            FrameReady?.Invoke(this, obj);
                        }
                    }
                    Thread.Sleep(writeFrameSleepMS);
                }
            }
            catch (Exception e)
            {
                Debug.LogError("RecordDataThread error, stopping");
                Debug.LogException(e);
            }
        }

        /// <summary>
        /// Disposes the fusion client
        /// </summary>
        public void Dispose()
        {
            Stop();
        }
    }
}
