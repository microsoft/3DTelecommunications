using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using System.Linq;

namespace Assets.Scripts
{
    /// <summary>
    /// A fusion client which loads data from disk
    /// </summary>
    public class LocalFusionClient : IFusionClient
    {
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
        /// The input folder for the frames being read.
        /// </summary>
        private string inputFolder;

        /// <summary>
        /// A backup struct to use when processing network requests and there is no frame in the pool
        /// </summary>
        private HoloportObjectStruct backup;

        /// <summary>
        /// The sleep time between frame reads
        /// </summary>
        private int readFrameSleepMS = 10;

        /// <summary>
        /// Creates a client with the given settings
        /// </summary>
        /// <param name="targetFPS"></param>
        /// <param name="settings"></param>
        public static LocalFusionClient CreateWithSettings(SettingsManager settings, ConcurrentQueue<HoloportObjectStruct> pool)
        {
            HoloportObjectStruct backup = HoloportObjectStruct.CreateWithSettings(-1, SettingsManager.Instance);
            return new LocalFusionClient(
                settings.FusionNetworkTargetFPS,
                pool,
                backup,
                settings.ReadFusionFramesFromDiskFolder
            );
        }

        /// <summary>
        /// Constructs a new fusion client
        /// </summary>
        private LocalFusionClient(
            float targetFPS,
            ConcurrentQueue<HoloportObjectStruct> objPool,
            HoloportObjectStruct backup,
            string inputFolder
        )
        {
            this.objPool = objPool;
            this.backup = backup;
            this.inputFolder = inputFolder;
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
            Task.Run(() => PlaybackDataThread(threadTokenSource.Token, inputFolder));
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
        /// A thread used to record fusion data
        /// </summary>
        private void PlaybackDataThread(CancellationToken token, string inputFolder)
        {
            try
            {
                Regex regex = new Regex("RendererHoloportationFrame(\\d\\d\\d\\d\\d).bin");
                string[] filePaths =
                    Directory.GetFiles(inputFolder)
                        .Where(n => regex.IsMatch(n))
                        .Select(n => Path.Combine($"{inputFolder}/", n))
                        .ToArray();
                Dictionary<string, byte[]> cache = new Dictionary<string, byte[]>();
                if (filePaths.Count() > 0)
                {
                    int idx = 0;
                    while (!token.IsCancellationRequested)
                    {
                        if (!Paused)
                        {
                            string filePath = filePaths[idx];
                            byte[] content;
                            if (!cache.TryGetValue(filePath, out content))
                            {
                                content = File.ReadAllBytes(filePath);
                                cache.Add(filePath, content);
                            }

                            HoloportObjectStruct currObj = GetObjectStructFromPool();
                            if (currObj != backup)
                            {
                                currObj.Deserialize(content);
                                FrameReady?.Invoke(this, currObj);
                            }
                            idx = (idx + 1) % filePaths.Length;
                        }

                        Thread.Sleep(readFrameSleepMS);
                    }
                }
                else
                {
                    Debug.Log($"No playback data found: {inputFolder}");
                }
            }
            catch (Exception e)
            {
                Debug.LogError("PlaybackDataThread error, stopping");
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
