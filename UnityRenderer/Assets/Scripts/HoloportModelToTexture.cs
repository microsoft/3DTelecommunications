using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System.Text;
using UnityEngine.UI;
//for async fetching
using System.Threading;
using Assets.Scripts;
using System.Collections.Concurrent;
#if UNITY_2018_2_OR_NEWER
using UnityEngine.Rendering;
#else
using UnityEngine.Experimental.Rendering;
#endif
[RequireComponent(typeof(HoloPortScript))]
public class HoloportModelToTexture : MonoBehaviour
{
    public ViewerHandler associatedViewHandler; //handler that sends 2D streams + HQ 3D streams

    public HoloPortScript localHoloportScript;
    public string serverFolderBasePath;

    public bool debugTestMesh;
    private DateTimeOffset lastUpdate;
    public MeshFilter testMeshFilter;
    //Vector3[] meshVertices;
    //Vector3[] meshNormals;
    //Vector2[] meshUVs;
    [Header("[GENUV Param]")]

    private int mTextureDimension = 2048;

    public int frameUVBufferSize = 10; // change base on frame rate (?)
    public ConcurrentQueue<HoloportObjectStruct> readyToSaveHOSFrames = new ConcurrentQueue<HoloportObjectStruct>();

    public int expectedVertexCount;
    public int expectedIndexCount;
    public bool useSimplifiedData = false;

    public Text fpsText;
    public Text MeshInfoText;
    public HoloportObjectStruct lastProcessedStruct;
    [Header("[ComputeShader Param]")]
    public Camera TextureCam;

    //high quality data buffer
    public RenderTexture mHoloportResultTexture_HQ;

    public bool exiting = false;
    public int numberOfMeshProcessingThreads = 1;
    CancellationTokenSource[] processingThreads;
    ConcurrentDictionary<AsyncGPUReadbackRequest, HoloportObjectStruct> asyncRequests = 
        new ConcurrentDictionary<AsyncGPUReadbackRequest, HoloportObjectStruct>();
    ConcurrentDictionary<HoloportObjectStruct, System.Diagnostics.Stopwatch> stopWatches = 
        new ConcurrentDictionary<HoloportObjectStruct, System.Diagnostics.Stopwatch>();

    public int readbackCallbackCount = 0;
    public int outputCount = 0;
    public int numProcessedSinceUpdate = 0;

    [Header("HoloObjStruct Write Format")]
    public bool writeOneHolobin = false;
    public bool writeSeparateHolobin = false;
    public bool writeOBJ = false;
    public bool writeTexturePNG = false;
    public bool writeTextureBinary = false;

    public bool embedUVsInOBJ = false;
    public bool useCorrectionTransform = false;
    public Transform correctionTransform;
    public Quaternion correctionQ;
    public Vector3 correctionP;

    private Queue<float> timeDurations;
    private const int durationWindow = 30;
    private float runningTotal = 0.0f;

    private int saveThreadLoopSleepTimeMS = 33;

    private Win32FileIO.WinFileIO quickFileIO;
    private HashSet<string> objectDirectorySet = new HashSet<string>();
    private byte[] pinnedTextureMemory;
    private Matrix4x4 rotationFix = Matrix4x4.identity;
    
    // controls output to obj
    private bool writeObjFromSettings = false;
    private string objOutputPathFromSettings = null;

    /// <summary>
    /// Event handler for when a frame is processed
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="frame"></param>
    public delegate void FrameProcessedHandler(object sender, HoloportObjectStruct frame);

    /// <summary>
    /// Triggered when a new holoportation frame is processed
    /// </summary>
    public event FrameProcessedHandler FrameProcessed;

    public Material holoportObjectMaterial;

    public void AwakeWithConfig()
    {
        //Check section exists
        mTextureDimension = SettingsManager.Instance.TextureDimensionHQ;
        useCorrectionTransform = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "UseCorrection", useCorrectionTransform);
        if (useCorrectionTransform)
        {
            float x = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionPositionX", 0.0f);
            float y = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionPositionY", 0.0f);
            float z = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionPositionZ", 0.0f);
            correctionP = new Vector3(x, y, z);

            float xQ = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionRotationX", 0.0f);
            float yQ = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionRotationY", 0.0f);
            float zQ = SettingsManager.Instance.GetValueWithDefault("TransformCorrection", "CorrectionRotationZ", 0.0f);
            correctionQ = Quaternion.Euler(xQ, yQ, zQ);

        }
        else
        {
            if (correctionTransform != null)
            {
                correctionQ = correctionTransform.rotation;
                correctionP = correctionTransform.position;
            }
        }
        AppLogger.Trace("Using Correction Transform" + correctionP + " " + correctionQ);

        writeObjFromSettings = SettingsManager.Instance.GetValueWithDefault("Renderer", "WriteOBJStream", false);
        objOutputPathFromSettings = SettingsManager.Instance.GetValueWithDefault<string>("Renderer", "WriteOBJStreamPath", null);
    }

    // Use this for initialization
    void Awake()
    {
        AwakeWithConfig();
        lastProcessedStruct = HoloportObjectStruct.CreateEmpty();

        mHoloportResultTexture_HQ = new RenderTexture(mTextureDimension, mTextureDimension, 0, RenderTextureFormat.ARGB32);
        mHoloportResultTexture_HQ.enableRandomWrite = true;
        mHoloportResultTexture_HQ.useMipMap = false;
        mHoloportResultTexture_HQ.Create();

        localHoloportScript = GetComponent<HoloPortScript>();

        RenderTexture prevActive = RenderTexture.active;
        RenderTexture.active = mHoloportResultTexture_HQ;
        GL.Clear(true, true, Color.blue);
        RenderTexture.active = prevActive;

        TextureCam.aspect = 1;
        TextureCam.enabled = false;

        StartProcessingThreads();
    }

    /// <summary>
    /// Gets the current FPS of the processing
    /// </summary>
    public float CurrentFPS
    {
        get;
        private set;
    }

    /// <summary>
    /// Gets the last processed struct
    /// </summary>
    public HoloportObjectStruct LastProcessedStruct
    {
        get
        {
            return lastProcessedStruct;
        }
    }

    /// <summary>
    /// Called when the behavior is enabled
    /// </summary>
    private void OnEnable()
    {
        // TODO: Check if this matters
        // BUG: https://dev.azure.com/msrp/PeabodyMain/_workitems/edit/9639
        rotationFix = transform.localToWorldMatrix;
        if (associatedViewHandler != null)
        {
            associatedViewHandler.UnusedBackgroundColor = TextureCam.backgroundColor;
        }
    }

    /// <summary>
    /// Called when the behavior is disabled
    /// </summary>
    private void OnDisable()
    {
        StopProcessingThreads();
    }

    /// <summary>
    /// Handles
    /// </summary>
    /// <param name="tokenObj"></param>
    private void HandleFileToSaveThread(object tokenObj)
    {
        CancellationToken token = (CancellationToken)tokenObj;
        while (!token.IsCancellationRequested)
        {
            Thread.Sleep(saveThreadLoopSleepTimeMS);
            try
            {
                ProcessNextMesh();
            }
            catch (Exception e)
            {
                Debug.LogError("HandleFileThread exception ");
                Debug.LogException(e);
            }
        }
    }

    /// <summary>
    /// Processes the given frame
    /// </summary>
    /// <param name="frame"></param>
    public void ProcessFrame(HoloportObjectStruct frame)
    {
        RenderHoloportTexture();
        HandleRenderedHoloportTexture(frame);
    }

    private void RenderHoloportTexture()
    {
        TextureCam.targetTexture = mHoloportResultTexture_HQ;
        TextureCam.Render();
    }

    private void HandleRenderedHoloportTexture(HoloportObjectStruct currStruct)
    {
        //Async render --> gpu resourceFetch


        //ASYNC WAY OF DOING IT (doesnt block, but introduces few frames of latency!)
        var request = AsyncGPUReadback.Request(mHoloportResultTexture_HQ, 0, ReadbackCallback);
        asyncRequests[request] = currStruct;
        stopWatches[currStruct] = System.Diagnostics.Stopwatch.StartNew();


        //SYNCHRONOUS WAY OF DOING IT
        // //render to correct RT
        //RenderTexture.active = mHoloportResultTexture;
        //currStruct.destinationTexture.ReadPixels(new Rect(0, 0, mTextureDimension, mTextureDimension), 0, 0);

        //currStruct.destinationTexture.GetRawTextureData<byte>().CopyTo(currStruct.preAllocatedRenderTextureFetchArray_BYTE);


        if (writeTexturePNG || writeObjFromSettings)
        {

            DumpRGBA32ToBitmap(currStruct.preAllocatedRenderTextureFetchArray_BYTE, currStruct.destinationTextureBitmap);
        }

        //readyToSaveHOSFrames.Enqueue(currStruct);
    }

    static void DumpRGBA32ToBitmap(byte[] rgba, System.Drawing.Bitmap bmp)
    {
        System.Drawing.Imaging.BitmapData bmpData = bmp.LockBits(new System.Drawing.Rectangle(0, 0, bmp.Width, bmp.Height), System.Drawing.Imaging.ImageLockMode.WriteOnly, bmp.PixelFormat);

        System.Runtime.InteropServices.Marshal.Copy(rgba, 0, bmpData.Scan0, rgba.Length);

        bmp.UnlockBits(bmpData);
    }

    void ReadbackCallback(AsyncGPUReadbackRequest currRequest)
    {
        HoloportObjectStruct currObj;
        if (currRequest.hasError)
        {
            Debug.LogError("GPU readback error detected.");
            if (!asyncRequests.TryRemove(currRequest, out currObj))
            {
                Debug.LogError("Could not remove HoloportObjectStruct from asyncRequests map");
            }
        }
        else if (currRequest.done)
        {
            if (asyncRequests.TryRemove(currRequest, out currObj))
            {
                System.Diagnostics.Stopwatch currWatch;
                if (stopWatches.TryRemove(currObj, out currWatch))
                { 
                    AppLogger.Trace(readbackCallbackCount + " gpu Fetch took " + currWatch.ElapsedMilliseconds + "ms");
                } 
                else
                {
                    Debug.LogError("Could not get stop watch for HoloportObjectStruct");
                }

                var wa = System.Diagnostics.Stopwatch.StartNew();
                currRequest.GetData<byte>().CopyTo(currObj.preAllocatedRenderTextureFetchArray_BYTE);
                wa.Stop();
                AppLogger.Trace(readbackCallbackCount + " texture copy byte took " + wa.ElapsedMilliseconds + "ms");

                if (writeTexturePNG || writeObjFromSettings)
                {
                    wa = System.Diagnostics.Stopwatch.StartNew();
                    DumpRGBA32ToBitmap(currObj.preAllocatedRenderTextureFetchArray_BYTE, currObj.destinationTextureBitmap);
                    wa.Stop();
                    AppLogger.Trace(readbackCallbackCount + " png conversion copy took " + wa.ElapsedMilliseconds + "ms");
                }
                readbackCallbackCount++;
                readyToSaveHOSFrames.Enqueue(currObj);
            } 
            else 
            { 
                Debug.LogError("Could not remove HoloportObjectStruct from asyncRequests map");
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        //  DispatchTextureUVCompute();

        if (numberOfMeshProcessingThreads == 0)
        {
            ProcessNextMesh();
        }

        if (Input.GetKeyUp(KeyCode.Tab))
        {
            ProcessNextMesh();
        }

        if (Input.GetKeyUp(KeyCode.V))
        {
            objectDirectorySet.Clear();
        }

        if (numProcessedSinceUpdate > 0)
        {
            CurrentFPS = Utils.ComputeFPS(lastUpdate, DateTimeOffset.Now, numProcessedSinceUpdate, CurrentFPS, 0.01f);
            lastUpdate = DateTimeOffset.Now;
            numProcessedSinceUpdate = 0;
        }
    }

    /// <summary>
    /// Processes the next available mesh
    /// </summary>
    private void ProcessNextMesh()
    {
        HoloportObjectStruct currObj;
        if (readyToSaveHOSFrames.TryDequeue(out currObj))
        {
            // hex equivalent of NetworkHash128 string format
            bool useHex = false;
            string nameInString = outputCount.ToString("0000");
            if (useHex)
            {
                nameInString = outputCount.ToString("X16");
            }
            ++outputCount;
            ++numProcessedSinceUpdate;

            var wa = System.Diagnostics.Stopwatch.StartNew();

            ////PREPROCESS
            ConvertHoloPortStructToUShort(currObj);

            wa.Stop();

            string outputDir =
                CreateOutputDir(objOutputPathFromSettings != null ? objOutputPathFromSettings : serverFolderBasePath);

            WriteHolobin(currObj);
            WriteSeparateHoloBins(currObj, nameInString, outputDir);
            WriteTexturePNG(currObj, nameInString, outputDir);
            WriteModelOBJ(currObj, nameInString, outputDir);

            AppLogger.Trace("Convert " + wa.ElapsedMilliseconds + " ms");

            if (associatedViewHandler == null)
            {
                Debug.LogError("ViewHandler missing for sending HQ frame!");
            }
            else
            {
                associatedViewHandler.Send3DMeshDataToViewer(currObj);
            }

            FrameProcessed?.Invoke(this, currObj);
        }

    }

    /// <summary>
    /// Writes out a holoportation bin file
    /// </summary>
    /// <param name="holoportObjStruct"></param>
    private void WriteHolobin(HoloportObjectStruct holoportObjStruct)
    {
        if (writeOneHolobin)
        {
            System.Diagnostics.Stopwatch wa = System.Diagnostics.Stopwatch.StartNew();
            if (quickFileIO == null)
            {
                //lazy init
                quickFileIO = new Win32FileIO.WinFileIO();
                quickFileIO.OpenForWriting(serverFolderBasePath + "\\recording.holobin");
                pinnedTextureMemory = new byte[mTextureDimension * mTextureDimension * 4];
                quickFileIO.PinBuffer(pinnedTextureMemory);
            }

            //write binary textuer
            if (writeTextureBinary)
            {
                quickFileIO.Write(holoportObjStruct.preAllocatedRenderTextureFetchArray_BYTE.Length);
            }

            wa.Stop();
            AppLogger.Trace("binary holobin write time " + wa.ElapsedMilliseconds + " ms");
        }
    }

    private void WriteModelOBJ(HoloportObjectStruct holoportObjStruct, string fileName, string newFolderPath)
    {
        //write OBJ (NOTE:EXPENSIVE CALL! since this mass converts half -> floats then floats -> strings)
        if (writeOBJ || writeObjFromSettings)
        {
            System.Diagnostics.Stopwatch wa = System.Diagnostics.Stopwatch.StartNew();
            //need to first convert it from packed half's to floats
            HoloportObjectStructUtils.DecodeHalfDataIntoFloatData(
                holoportObjStruct.vertexData, holoportObjStruct.meshVertices,
                holoportObjStruct.meshNormals, holoportObjStruct.vertexCount);
            //debug obj
            string filePath2 = newFolderPath + "\\" + fileName + "_debug.obj";
            if (useCorrectionTransform)
            {
                System.IO.File.WriteAllText(filePath2, holoportObjStruct.ToOBJString(correctionQ, correctionP, embedUVsInOBJ || writeObjFromSettings, fileName));
            }
            else
            {
                System.IO.File.WriteAllText(filePath2, holoportObjStruct.ToOBJString(embedUVsInOBJ || writeObjFromSettings, fileName));
            }


            using (StreamWriter sw = new StreamWriter(Path.Combine(newFolderPath, fileName + ".mtl")))
            {
                sw.Write("\n");
                sw.Write("newmtl {0}\n", fileName);
                sw.Write("Ka  0.0 0.0 0.0\n");
                sw.Write("Kd  0.0 0.0 0.0\n");
                sw.Write("Ks  0.0 0.0 0.0\n");
                sw.Write("d  1.0\n");
                sw.Write("Ns  0.0\n");
                sw.Write("illum 2\n");
                sw.Write("map_Kd {0}\n", fileName + "_texture.png");
            }

            wa.Stop();
            AppLogger.Trace("Writing obj took " + wa.ElapsedMilliseconds + "ms");
        }
    }

    private void WriteTexturePNG(HoloportObjectStruct holoportObjStruct, string fileName, string newFolderPath)
    {
        //write binary textuer
        //if (writeTextureBinary)
        //{
        //    wa = System.Diagnostics.Stopwatch.StartNew();
        //    BinaryWriter bw = new BinaryWriter(File.Open(newFolderPath + "\\" + fileName + "_texture.holobinTexture", FileMode.Create));
        //    bw.Write(holoportObjStruct.preAllocatedRenderTextureFetchArray_BYTE); //the raw data that was in destinationTextureBitmap
        //    bw.Close();

        //    wa.Stop();
        //    AppLogger.Trace("Writing texture binary took " + wa.ElapsedMilliseconds + "ms");
        //}

        //write texture
        if (writeTexturePNG || writeObjFromSettings)
        {
            System.Diagnostics.Stopwatch wa = System.Diagnostics.Stopwatch.StartNew();
            string filePath = newFolderPath + "\\" + fileName + "_texture.png";
            if (holoportObjStruct.destinationTextureBitmap != null)
            {
                holoportObjStruct.destinationTextureBitmap.Save(filePath);
            }
            wa.Stop();
            AppLogger.Trace("Writing texture took " + wa.ElapsedMilliseconds + "ms");
        }
    }

    private void WriteSeparateHoloBins(HoloportObjectStruct holoportObjStruct, string fileName, string newFolderPath)
    {
        if (writeSeparateHolobin)
        {
            System.Diagnostics.Stopwatch wa = System.Diagnostics.Stopwatch.StartNew();

            FileStream fs = File.Open(newFolderPath + "\\" + fileName + ".holobinMesh", FileMode.Create);
            BinaryWriter bw = new BinaryWriter(fs);

            bw.Write(holoportObjStruct.vertexCount); //vertex count
            bw.Write(holoportObjStruct.indexCount); //index count

            //TODO: need faster write here
            //write vertices
            for (int i = 0; i < holoportObjStruct.vertexCount * 3; ++i)
            {
                bw.Write(holoportObjStruct.positionXYZ_ushort[i]);
            }
            //write indices
            for (int i = 0; i < holoportObjStruct.indexCount; ++i)
            {
                bw.Write(holoportObjStruct.indices_ushort[i]);
            }

            ////Write UVS (? do we need to?)
            var byteUVBuf = Utils.ConvertToBytes(holoportObjStruct.uvData, 170000 * 2);
            //bw.Write(byteUVBuf);  
            FileStream fs3 = File.Open(newFolderPath + "\\" + fileName + ".holobinUV", FileMode.Create);
            BinaryWriter bw3 = new BinaryWriter(fs3);
            bw3.Write(byteUVBuf); //the raw data that was in destinationTextureBitmap
            bw3.Close();

            //write binary textuer
            if (writeTextureBinary)
            {
                FileStream fs2 = File.Open(newFolderPath + "\\" + fileName + ".holobinTexture", FileMode.Create);
                BinaryWriter bw2 = new BinaryWriter(fs2);
                bw2.Write(holoportObjStruct.preAllocatedRenderTextureFetchArray_BYTE); //the raw data that was in destinationTextureBitmap
                bw2.Close();
            }
            bw.Close();

            wa.Stop();
            AppLogger.Trace("binary holobin write time " + wa.ElapsedMilliseconds + " ms");
        }
    }

    /// <summary>
    /// Converts the HoloPortObjectStruct to use ushort data types
    /// </summary>
    /// <param name="currObj"></param>
    private void ConvertHoloPortStructToUShort(HoloportObjectStruct currObj)
    {
        Vector3 tempVect = Vector3.zero;
        for (int i = 0; i < currObj.vertexCount; ++i)
        {
            var packed1 = currObj.vertexData[3 * i];
            var packed2 = currObj.vertexData[3 * i + 1];

            var x = (ushort)(packed1 & 0xFFFF);
            var y = (ushort)((packed1 >> 16) & 0xFFFF);
            var z = (ushort)(packed2 & 0xFFFF);

            ref Vector3 v = ref tempVect;
            v.x = HalfHelper.HalfToSingle(Half.ToHalf(x));
            v.y = HalfHelper.HalfToSingle(Half.ToHalf(y));
            v.z = HalfHelper.HalfToSingle(Half.ToHalf(z));
            v = (correctionP * 100 + correctionQ * v) * 0.01f; // cm to M 
            v = rotationFix * v;

            currObj.positionXYZ_ushort[3 * i] = HalfHelper.SingleToHalf(v.x).value;
            currObj.positionXYZ_ushort[3 * i + 1] = HalfHelper.SingleToHalf(v.y).value;
            currObj.positionXYZ_ushort[3 * i + 2] = HalfHelper.SingleToHalf(v.z).value;

        }


        // reverse winding order of triangles due to LHS->RHS conversion
        for (int i = 0; i < currObj.indexCount; i += 3)
        {
            //change winding order since we just flipped LHS->RHS
            int origIValue = currObj.indexData[i];
            currObj.indexData[i] = currObj.indexData[i + 2];
            currObj.indexData[i + 2] = origIValue;
        }

        //convert to ushort

        for (int i = 0; i < currObj.indexCount; ++i)
        {
            currObj.indices_ushort[i] = (ushort)currObj.indexData[i];
        }
    }

    /// <summary>
    /// Starts the save file threads
    /// </summary>
    private void StartProcessingThreads()
    {
        processingThreads = new CancellationTokenSource[numberOfMeshProcessingThreads];
        for (int i = 0; i < numberOfMeshProcessingThreads; ++i)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            processingThreads[i] = cts;

            Thread t = new Thread(new ParameterizedThreadStart(HandleFileToSaveThread));
            t.Start(cts.Token);
        }
    }

    /// <summary>
    /// Stops the running save file threads
    /// </summary>
    private void StopProcessingThreads()
    {
        if (processingThreads != null)
        {
            //check the state of the threads and start it again if they died
            for (int i = 0; i < processingThreads.Length; ++i)
            {
                // Stop the processing thread
                if (!processingThreads[i].IsCancellationRequested)
                {
                    processingThreads[i].Cancel();
                }
            }
        }
        processingThreads = null;
    }

    /// <summary>
    /// Creates an output directory
    /// </summary>
    /// <param name="baseFolder">The base folder</param>
    /// <param name="baseFile">The base file</param>
    /// <returns></returns>
    private static string CreateOutputDir(string baseFolder)
    {
        Directory.CreateDirectory(baseFolder); // create if doesnt exist
                                                  //TODO: add compression? 
        return baseFolder;
    }
}
