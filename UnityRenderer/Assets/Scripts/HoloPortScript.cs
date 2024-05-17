#undef OBJ_CAPTURE_RIG

#define USE_OCCLUSION_SEAMS
#define M4D_USE_RENDERER_IN_HOLOPORT_SCRIPT
#define M4D_EFFICIENTY_CODE_PRONE
using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using NAudio.Wave;
using Assets.Scripts;
using System.Collections.Concurrent;

/// <summary>
/// Render Effect States
/// </summary>
public enum RenderEffectState
{
    Default,
    Montage4D,
    Intrinsic4D,
    Normal,
    ShinyGeometry,
    HolographicEffect,
    Wireframe,
    Art,
    Occlusion,
    RenderEffectStateCount
}

/// <summary>
/// The main renderer for the Peabody / Holoportation Project
/// </summary>
public class HoloPortScript : MonoBehaviour
{
    public Configuration RigConfiguration;
    [Header("[Network]")]
    public HoloportControlPanelConnector hcpc;

    public float[] UseCameraTexture;

    public const int cIndicesInTriangle = 3;

    public int MaxIndices = 2100000;
    public int MaxVertices = 250000;

    private bool useDataUploadInterleaving = false;
    private int dataUploadFramesStride = 2;

    public bool PauseDrawing = false;
    public bool PauseNetworkUpdate = false;
    float explosionFPS = 90;
    float normalFPS = 50;
    int holoportID;
    static int holoportsCount = 0;
    static int activeHoloportsCount = 0;

    /// Input for the special shader code that displaces vertices in the Graphics.DrawProcedural code - this overcomes the 65k vertices limit.
    ComputeBuffer vertexHalfBuffer; // int
    ComputeBuffer indexBuffer; // int
    private float lastTimeStatsWereLogged;

    // Keep track of the created buffers so we can clean them all up
    private List<ComputeBuffer> bufferList = new List<ComputeBuffer>();

    /// <summary>
    /// Whether the compute shader has processed this frame. Since the compute shader is asynchronous with the rendering pipeline, 
    /// we don't want to process the mesh again if the Fusion has slower frame rate than the renderer
    /// </summary>
    private bool HasRunThisFrame = false;
    // Montage4D buffers and shaders

    /// <summary>
    /// Buffer that stores the texture fields per vertex
    /// </summary>
    ComputeBuffer vertexTextureFieldsBuffer;        // int

    /// <summary>
    /// Buffer that stores the occlusion seams from 0 to 1 per vertex
    /// </summary>
    ComputeBuffer vertexOcculusionSeamsBuffer;      // int

    /// <summary>
    /// Buffer that stores the geodesic distance to the seams per vertex
    /// </summary>
    ComputeBuffer vertexGeodesicWeightsBuffer;      // int

    /// <summary>
    /// Buffer that counts the rankings of the visibilities
    /// </summary>
    //ComputeBuffer visibilityRankingBuffer; // int
    public ComputeShader ComputeVertexWeightsShader;

    // pointer to the compute buffers used in Montage4D
    int initVertexWeightsKernelIndex;
    int occlusionSeamIdenficationKernelIndex;
    int occlusionSeamIterationKernelIndex;
    int vertexWeightsKernelIndex;
    int initTriangleSeamsKernelIndex;

    // matrices used in Montage4D
    float[] textureCameraMatrices;
    float[] textureMatrices;
    float[] invTextureCameraMatrices;
    float[] modelTransformMatrices;
    // float[] cameraMatrixBuffer;
    float[] unity_object2WorldMatrix;
    // float[] currentCamCenterBuffer;
    float[][][] vertexWeightsData;
    float[][][] vertexTextureFieldsData;
    int[][][] vertexTextureRankingData;    // TODO: convert to byte 
    int[][][] vertexSortedRankingData;     // TODO: convert to byte
    float[][][] vtOcclusionSeamsData;      // TODO: convert to byte
    float[][][] vtGeodesicData;            // TODO: convert to byte
    int[] visibilitiesData;                // TODO: convert to byte
    int[] visibilityRankings;              // TODO: convert to byte

    // debuggers
    int debugColorLabeledFields = 0;
    int showTexWeights = 0;
    int debugSingleView = 10;
    int debugDiscontinuity = 0;
    int debugNormalWeightedField = 0;
    int debugGeodesicFields = 0;
    int debugSeams = 0;
    int debugNormalWeightedBlendingColor = 0;
    int debugTextureWeights = 0;
    int debugTemp = 0;
    bool enableMontage4DSlowVisibilitiesTesting = false;
#if !M4D_USE_RENDERER_IN_HOLOPORT_SCRIPT
    bool d3d = true;
#endif
    bool firstUpdate = true;
    int maxIndexCount = 0;
    int maxVertexCount = 0;

    /// <summary>
    /// For outputing the Montage4D snapshots in quantitative experiments
    /// </summary>
    private int debugSnapshotID = 0;
    // Monage4D buffers and shaders end
    // sizes for pre-allocated mesh buffers - use the closest buffer each frame to reduce CPU and GPU copy
    // Ruofei: the more buffers the faster the algorithm runs, but more buffer may induce more memory consumption
    /// <summary>
    /// Estimated memory cost: 4 bytes * average vertex size * array length, is smaller than 100 MB
    /// </summary>
    private List<int> vertexDataSizes = new List<int> { 100000, 150000, 200000, 250000, 260000, 270000, 280000, 290000, 300000, 310000, 320000, 330000, 340000, 350000,
        360000, 370000, 380000, 390000, 400000, 500000, 550000, 600000, 700000 };
    /// <summary>
    /// 4 bytes * 1800000 * array length, is smaller than 100MB
    /// </summary>
    private List<int> indexDataSizes = new List<int> { 300000, 700000, 100000, 110000, 1200000,
        1300000, 1500000, 1600000, 1700000, 1800000 };

    public delegate void OnDataReadyCallback(byte[] colorData, int[] colorSizes);
    public OnDataReadyCallback onDataReadyCallback;

    /// <summary>
    /// The total number of vertices to be drawn
    /// </summary>
    private int stageIndexCount = 0;
    private int stageVertexCount = 0;
    /// <summary>
    /// Double buffered (+ GPU) : working buffer, stage buffer, draw buffer on GPU
    /// </summary>
    private int bufferCount = 2;

    private DateTimeOffset lastUpdate;

    /// <summary>
    /// preloaded meshes to simulate data transmitted via the network - for testing.
    /// </summary>
    // PlyReader preloadedMeshes;
    // int currentMeshID;

    // flags to control network threading
    private readonly object sync = new object();
    private bool isInitialized = false;

    private Timer ExplosionAnimationTimer;
    private ReadAndProcessRawImage mLocalImageComputeScript;
    private ProjectiveTextures mLocalProjectiveTextureScript;

    [Header("[Shader Effects Control]")]
    private bool useHotKey = true;
    // This material should be linked with a special shader code that displaces vertices based on a ComputeBuffer.
    [Range(0, 1)]
    public float GlobalAlpha = 1.0f;

    public Material[] HoloPortMaterial; // [bool GEOMETRY shader] -> objectcaptureShader

    public RenderEffectState CurrentRenderEffectState;
    [Range(0, 0.2f)]
    public float EffectChangeRate = 0.04f;
    private RenderEffectState PrevRenderEffectState;
    IEnumerator renderEffectStateChangeCoroutine;
    [Range(0, 1)]
    public float[] EffectWeightArray;
    public bool ApplyLightingToNormalMap = true;

    private Queue<HoloportObjectStruct> uvGeneratedList = new Queue<HoloportObjectStruct>();
    private Matrix4x4 cameraMatrix;
    /// <summary>
    /// Impact the speed a lot
    /// </summary>

    static public string DebugPath = "\\Debug\\";

    // Montage4D declaration begins
    [Header("[Montage4D Renderer]")]
    /// <summary>
    /// Iterations for the spatial diffusion over the triangular mesh
    /// 20 is good, the more, the better
    /// </summary>
    [Range(1, 40)]
    public int SpatialDiffusionIterations = 20;
    /// <summary>
    /// The threshold to determine seams using normal-based weights
    /// If just human, use 0.01, if with table which basically no camera is looking at it directly, use ~0.2
    /// </summary>
    [Range(0, 1.0f)]
    public float NormalThresholdForSeams = 0.01f;
    /// <summary>
    /// The percentage to blend normal weighted blending into the final results
    /// </summary>
    [Range(0, 1.5f)]
    public float NormalWeightedPercentage = 0.2f;
    /// <summary>
    /// The scale of the geodesic distance
    /// </summary>
    [Range(-8, 8)]
    public double GeodesicScale = 4.2;
    /// <summary>
    /// The opacity of the untrusted normal weighted blending, adjust this, the results are fun and beautiful!
    /// </summary>
    [Range(0, 1.0f)]
    public float NormalWeightedBlendingOpacity = 1.0f;

    /// <summary>
    /// Speed of Temporal Transition
    /// <para>10 means 10 frames for transition</para> 
    /// </summary>
    [Range(1, 1000)]
    public float TemporalTransitionSpeed = 300.0f;

    /// <summary>
    /// The value to determine the discontinuity threshold
    /// TODO: currently I broke it, it worked for the SIGGRAPH paper though
    /// </summary>
    [Range(0, 1.0f)]
    public float DiscontinuityThreshold = 0.2f;

    [Range(0, 1.0f)]
    public float DotProductNormalWeightThreshold = 0.707f; // 45 degree unit vector dot product

    [Range(0, 0.5f)]
    public float ColorWeightThreshold = 0.0f;

    /// <summary>
    /// The trigger for view dependent temporal texture fields
    /// </summary>
    private bool EnableViewDependentTemporalTextureFields = true;
    /// <summary>
    /// The trigger to output th index and vertex cound, 
    /// if the program exits accidentally, please check whether there are too many vertices
    /// </summary>
    public bool _DebugIndexAndVertexCount = false;
    /// <summary>
    /// Steps to transit the view dependent temporal texture fields
    /// </summary>
    [Range(0, 1)]
    public float ViewDependentTemporalTextureFieldStep = 0.1f;
    /// <summary>
    /// Whether or not to remove the black pixels in the last pass
    /// </summary>
    public bool RemoveBackgroundPixels = false;

    const float MAX_TTF_RANGE = 5.0f;
    /// <summary>
    /// Temporal Texture Weight is determined by the current view directions
    /// <para>In order to prevent sudden jump from one view to another, we use this array to smoothly transit view interpoloations</para> 
    /// </summary>
    [Range(0, MAX_TTF_RANGE)]
    public float[] TemporalTextureFields;

    /// <summary>
    /// The Target Texture Fields should be determined by view directions (# of vertices rendered from each view), currently it's the distance between main camera and all other cameras.
    /// <para>TTF is short for Temporal Texture Weight</para> 
    /// </summary>
    [Range(0, MAX_TTF_RANGE)]
    public float[] TargetTextureFields;

    /// <summary>
    /// ViewPreference provides a flexible interface to adjust view preference
    /// <para>For instance, if you find the rendering from one camera is not satisfactory, just use view preference to set the value to -1</para> 
    /// </summary>
    [Range(-1, 1)]
    public float[] ViewPreference;
    // Montage4D declaration ends

    [Header("[Voxelize Effect]")]
    public bool Voxelize = false;
    [Range(0.0f, 0.1f)]
    public float VoxelSize = 0.01f;
    [Range(0.0f, 0.1f)]
    public float VoxelNoSkipSize = 0.009f;
    [Header("[Explosion Effect]")]

    public RenderEffectState targetExplosionMode = RenderEffectState.Default;
    public bool Materialize = false;
    bool pastMaterializedValue = false;
    [Range(0, 1)]
    public float ExplosionAmount = 1;
    [Range(0, 2)]
    public float ExplosionTimeSec = 1f;
    [Header("[Pulse Effect]")]
    [Range(-0.5f, 2)]
    public float NormalBandCenter = -1;
    public bool isDecreasingNormalBandCenter = true;
    public bool useNormalBands = true;
    [Header("[HolographicEffect Parameter]")]
    // holographic edge effect parameters
    public Color HighlightColor = new Color(0.149f, 0.541f, 1f, 1f); // Highlight color(RGB)
    public Color HologramTint = new Color(1, 1, 1, 1);
    [Range(0, 16)]
    public float RimPower = 1.5f; // Edge sharpness
    [Range(0, 1)]
    public float RimVis = 1f; // Edge visibility
    [Range(0, 1)]
    public float ModelOpacity = 0.7f;
    [Header("[Culling]")]
    public Transform cullingBox1;
    public Transform cullingBox2;

    [Header("[Depth Tuning")]
    public DepthDilateProfile depthDilateProfile;
    ComputeBuffer depthSearchRadiusBuffer;
    ComputeBuffer depthSearchStepBuffer;

    [Header("Audio")]
    public bool useAudio = false;
    public bool playbackAudio = false;
    public bool constantAudioFrameSize = true;

    public AudioSource playbackSource;
    public AudioClip currClip;

    public int samplesToRead = 0;

    public int audioSampleRate = 48000;
    public int audioSampleSize = 2; // 16-bit = 2; 32-bit = 4
    BufferedWaveProvider waveProvider;
    WaveOutEvent waveOut;

    [Header("[Debug Param]")]
    public bool renderUVSpace = true;
    public double aggresivenexx = 7.0;
    public int targetPolyCount = 21000;

    public bool configInitialized = false;

    Stopwatch stopWatch;

    ComputeBuffer visibilityBuffer;
    ComputeBuffer textureUVBuffer;
    HoloportModelToTexture mLocalHoloportModelToTextureScript;

    private IFusionClient client;
    private ConcurrentQueue<HoloportObjectStruct> holoObjectProcessingPool;
    private int holoObjectProcessingPoolSize;

    public enum HoloportMaterialType
    {
        Regular,
        Voxelization,
        ObjectCapture,
        Count
    }

    private KeyCode[] keyCodes = {
         KeyCode.Alpha1,
         KeyCode.Alpha2,
         KeyCode.Alpha3,
         KeyCode.Alpha4,
         KeyCode.Alpha5,
         KeyCode.Alpha6,
         KeyCode.Alpha7,
         KeyCode.Alpha8,
         KeyCode.Alpha9,
     };


    public void AwakeWithConfig()
    {
        // Log warnings & Exceptions in prod
#if !(DEVELOPMENT_BUILD || UNITY_EDITOR)
            Debug.unityLogger.filterLogType = LogType.Exception | LogType.Warning;
#endif

        //RigConfiguration check
        if (SettingsManager.Instance.config.Contains("Renderer", "UseBuiltInConfig"))
        {
            if (!SettingsManager.Instance.config["Renderer"]["UseBuiltInConfig"].BoolValue)
            {
                Debug.Log($"Using {SettingsManager.Instance.fileName} Configuration for HoloportScript");
                if (SettingsManager.Instance.config.Contains("DepthGeneration", "DepthCameraCount"))
                {
                    RigConfiguration.NumPods = SettingsManager.Instance.config["DepthGeneration"]["DepthCameraCount"].IntValue;
                }
                if (SettingsManager.Instance.config.Contains("ColorProcessing", "ColorImageWidth"))
                {
                    RigConfiguration.ColorWidth = SettingsManager.Instance.config["ColorProcessing"]["ColorImageWidth"].IntValue;
                }
                if (SettingsManager.Instance.config.Contains("ColorProcessing", "ColorImageHeight"))
                {
                    RigConfiguration.ColorHeight = SettingsManager.Instance.config["ColorProcessing"]["ColorImageHeight"].IntValue;
                }
                if (SettingsManager.Instance.config.Contains("ColorProcessing", "ColorPixelBytes"))
                {
                    RigConfiguration.ColorPixelBytes = SettingsManager.Instance.config["ColorProcessing"]["ColorPixelBytes"].IntValue;
                    if (RigConfiguration.ColorPixelBytes == 1) // 1 byte per channel = bayered 
                    {
                        RigConfiguration.TextureFormat = TextureFormat.Alpha8;
                    }
                    else if (RigConfiguration.ColorPixelBytes == 3) // 3 Bytes per channel = RGB
                    {
                        RigConfiguration.TextureFormat = TextureFormat.RGB24;
                    }
                    else if (RigConfiguration.ColorPixelBytes == 4) //4 bytes per channel = ARGB
                    {
                        RigConfiguration.TextureFormat = TextureFormat.ARGB32;
                    }
                    else
                    {
                        Debug.LogError("Rig Config error: unsupported bytes per pixel =" + RigConfiguration.ColorPixelBytes);
                    }
                }
                if (SettingsManager.Instance.config.Contains("Network", "FusionIPAddress"))
                {
                    RigConfiguration.NetworkHostIP = SettingsManager.Instance.config["Network"]["FusionIPAddress"].StringValue;
                }
                if (SettingsManager.Instance.config.Contains("Ports", "DataStreamPort"))
                {
                    RigConfiguration.NetworkHostPort = SettingsManager.Instance.config["Ports"]["DataStreamPort"].StringValue;
                }
                Debug.Log($"{RigConfiguration.ToString()}");

                //AUDIO
                if (SettingsManager.Instance.config.Contains("Audio", "UseAudio"))
                {
                    useAudio = SettingsManager.Instance.config["Audio"]["UseAudio"].BoolValue;
                }
                if (SettingsManager.Instance.config.Contains("Audio", "AudioSampleRate"))
                {
                    audioSampleRate = SettingsManager.Instance.config["Audio"]["AudioSampleRate"].IntValue;
                }
                if (SettingsManager.Instance.config.Contains("Audio", "AudioSampleSize"))
                {
                    audioSampleSize = SettingsManager.Instance.config["Audio"]["AudioSampleSize"].IntValue;
                }
                Debug.Log($"Audio Enabled: {useAudio}\nSample Rate: {audioSampleRate}\nSample Size: {audioSampleSize}");

                //DepthDilation settings check
                if (depthDilateProfile == null)
                {
                    Debug.Log("creating new DepthDilateProfile");
                    //initialize 
                    depthDilateProfile = ScriptableObject.CreateInstance<DepthDilateProfile>();
                    if (depthDilateProfile.DepthSearchRadius == null)
                    {
                        depthDilateProfile.HoloportationConfig = RigConfiguration;

                        depthDilateProfile.DepthSearchRadius = new int[RigConfiguration.NumPods];
                        depthDilateProfile.DepthSearchStep = new int[RigConfiguration.NumPods];
                        depthDilateProfile.DilateStepSizes = new int[RigConfiguration.NumPods];
                        for (int i = 0; i < RigConfiguration.NumPods; ++i)
                        {
                            depthDilateProfile.DepthSearchRadius[i] = 1;
                            depthDilateProfile.DepthSearchStep[i] = 1;
                            depthDilateProfile.DilateStepSizes[i] = 1;
                        }
                        depthDilateProfile.DilateSteps = 0;// dont dilate by default
                    }
                }
                depthDilateProfile.DilateSteps = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DilateSteps", 0);

                int defaultDepthSearchRadius = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DefaultDepthSearchRadius", 1);
                int defaultSearchStep = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DefaultDepthSearchStep", 1);
                int defaultDilateStepSize = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DefaultDepthDilateStepSize", 1);

                for (int i = 0; i < RigConfiguration.NumPods; ++i)
                {
                    //check for specific overwrite
                    depthDilateProfile.DepthSearchRadius[i] = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DepthSearchRadius_Pod" + i, defaultDepthSearchRadius);
                    depthDilateProfile.DepthSearchStep[i] = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DepthSearchStep_Pod" + i, defaultSearchStep);
                    depthDilateProfile.DilateStepSizes[i] = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DepthDilateStepSize_Pod" + i, defaultDilateStepSize);

                }

                //Color Projection quality settings 
                DiscontinuityThreshold = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DiscontinuityThreshold", DiscontinuityThreshold);
                DotProductNormalWeightThreshold = SettingsManager.Instance.GetValueWithDefault("DepthDilationProfile", "DotProductNormalWeightThreshold", DotProductNormalWeightThreshold);
            }
            else
            {
                Debug.Log("Using Built int Configuration for HoloportScript");
            }
        }
        UseCameraTexture = new float[RigConfiguration.NumPods];
        int[] emptyList = new int[0];
        int[] disabledCameraArray = SettingsManager.Instance.GetValueArrayWithDefault<int>("Renderer", "DisabledCameraList", emptyList, true);
        if (disabledCameraArray == null)
        {
            Debug.Log("ERROR - [Renderer][DisabledCameraList] is not a valid array! Must be in form {x,y,z,...}");
            disabledCameraArray = emptyList;
        }
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
        {
            UseCameraTexture[i] = Array.Exists(disabledCameraArray, ele => ele == i + 1) ? 0.0f : 1.0f;
        }
        configInitialized = true;
    }
    /// <summary>
    /// Lock the frame rate
    /// Note that setting targetFrameRate does not guarantee that frame rate. There can be fluctuations due to platform specifics, or the game might not achieve the frame rate because the application is using too much processing power.
    /// </summary>
    void Awake()
    {
        if (!configInitialized)
            AwakeWithConfig();

        holoObjectProcessingPool = new ConcurrentQueue<HoloportObjectStruct>();
        holoObjectProcessingPoolSize = SettingsManager.Instance.FusionNetworkProcessingPoolSize;
        for (int i = 0; i < holoObjectProcessingPoolSize; i++)
        {
            holoObjectProcessingPool.Enqueue(HoloportObjectStruct.CreateWithSettings(i, SettingsManager.Instance));
        }

        for (int i = 0; i < HoloPortMaterial.Length; ++i)
        {
            HoloPortMaterial[i] = Instantiate(HoloPortMaterial[i]);
        }

        // Application.targetFrameRate = 60;
        samplesToRead = 0;

        if (useAudio && audioSampleRate > 0)
        {
            waveProvider = new BufferedWaveProvider(new WaveFormat(audioSampleRate, 16, 1)) { BufferDuration = TimeSpan.FromMilliseconds(500), DiscardOnBufferOverflow = true };
            waveOut = new WaveOutEvent() { DesiredLatency = 200 };
            waveOut.Init(waveProvider);
            waveOut.Play();
        }
    }

    unsafe void Start()
    {
        holoportID = holoportsCount;
        holoportsCount++;
    }

    void OnEnable()
    {
        InitializeResources();
    }

    /// <summary>
    /// Initialization of all the resources (buffers)
    /// </summary>
    unsafe void InitializeResources()
    {
        if (isInitialized) return;
        activeHoloportsCount++;

        stageIndexCount = 0;

        // instantiate network client plugin
        client = 
            SettingsManager.Instance.ReadFusionFramesFromDisk ?
                (IFusionClient)LocalFusionClient.CreateWithSettings(SettingsManager.Instance, holoObjectProcessingPool) :
                (IFusionClient)FusionClient.CreateWithSettings(SettingsManager.Instance, holoObjectProcessingPool);
        client.FrameReady += OnFusionFrameReady;

        // add the max buffer size to the buffer sizes list
        vertexDataSizes.Add(MaxVertices);
        indexDataSizes.Add(MaxIndices);

        // sort the sizes list in descending order - will be used to choose the buffer with the appropriate size later
        vertexDataSizes.Sort();
        vertexDataSizes.Reverse();
        indexDataSizes.Sort();
        indexDataSizes.Reverse();

        // pre-allocated buffers based on the sizes lust 
        vtOcclusionSeamsData = new float[bufferCount][][];
        vtGeodesicData = new float[bufferCount][][];
        vertexTextureFieldsData = new float[bufferCount][][];

        for (int b = 0; b < bufferCount; b++)
        {
            vertexTextureFieldsData[b] = new float[vertexDataSizes.Count][];
            vtOcclusionSeamsData[b] = new float[vertexDataSizes.Count][];
            vtGeodesicData[b] = new float[vertexDataSizes.Count][];

            for (int i = 0; i < vertexDataSizes.Count; i++)
            {
                vertexTextureFieldsData[b][i] = new float[vertexDataSizes[i] * RigConfiguration.NumPods];
                vtOcclusionSeamsData[b][i] = new float[vertexDataSizes[i] * RigConfiguration.NumPods];
                vtGeodesicData[b][i] = new float[vertexDataSizes[i] * RigConfiguration.NumPods];
            }
        }


        if (useNormalBands)
            StartCoroutine(NormalBandSlide());
        if (mLocalImageComputeScript == null)
            mLocalImageComputeScript = GetComponent<ReadAndProcessRawImage>();
        if (mLocalProjectiveTextureScript == null)
            mLocalProjectiveTextureScript = GetComponent<ProjectiveTextures>();

        if (mLocalHoloportModelToTextureScript == null)
        {
            mLocalHoloportModelToTextureScript = GetComponent<HoloportModelToTexture>();
        }

        mLocalHoloportModelToTextureScript.FrameProcessed += OnModelToTextureFrameProcessed;

        EffectWeightArray = new float[(int)RenderEffectState.RenderEffectStateCount];
        EffectWeightArray[(int)targetExplosionMode] = 1;
        PrevRenderEffectState = CurrentRenderEffectState;

        // Montage4D begins
#if !M4D_USE_RENDERER_IN_HOLOPORT_SCRIPT
        d3d = SystemInfo.graphicsDeviceVersion.IndexOf("Direct3D") > -1;
#endif
        PrevRenderEffectState = CurrentRenderEffectState;
        TemporalTextureFields = new float[RigConfiguration.NumPods];
        TargetTextureFields = new float[RigConfiguration.NumPods];
        ViewPreference = new float[RigConfiguration.NumPods];
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
        {
            TemporalTextureFields[i] = 1.0f;
            TargetTextureFields[i] = 1.0f;
            ViewPreference[i] = 0.0f;
        }

        // init compute shader
        initVertexWeightsKernelIndex = ComputeVertexWeightsShader.FindKernel("InitVertexWeightsKernel");
        if (initVertexWeightsKernelIndex < 0)
            Debug.LogError("[HoloPortScript] Warning, can't find kernel initVertexWeightsKernelIndex. Reimport the compute shader");
        vertexWeightsKernelIndex = ComputeVertexWeightsShader.FindKernel("ComputeVertexWeightsKernel");
        if (vertexWeightsKernelIndex < 0)
            Debug.LogError("[HoloPortScript] Warning, can't find kernel vertexWeightsKernelIndex. Reimport the compute shader");
        initTriangleSeamsKernelIndex = ComputeVertexWeightsShader.FindKernel("InitTriangleSeamsKernel");
        if (initTriangleSeamsKernelIndex < 0)
            Debug.LogError("[HoloPortScript] Warning, can't find kernel initTriangleSeamsKernelIndex. Reimport the compute shader");
        // Montage4D ends

        // create structured buffers for compute shader with enough memory
        Debug.Log("[Holoport Script][Start] Initialize buffers with max indices " + MaxIndices + " and vertices " + MaxVertices + ", please be aware of the sizes of the incoming mesh.\n");
        indexBuffer = AddComputeBuffer(MaxIndices * 3, sizeof(int), ComputeBufferType.Default);
        vertexHalfBuffer = AddComputeBuffer(MaxVertices * 3, sizeof(int), ComputeBufferType.Default);
        // TODO: size incorrect
        vertexTextureFieldsBuffer = AddComputeBuffer(MaxVertices * RigConfiguration.NumPods, sizeof(float), ComputeBufferType.Default); ; // float
        vertexOcculusionSeamsBuffer = AddComputeBuffer(MaxVertices * RigConfiguration.NumPods, sizeof(float), ComputeBufferType.Default); // float
        vertexGeodesicWeightsBuffer = AddComputeBuffer(MaxVertices * RigConfiguration.NumPods, sizeof(float), ComputeBufferType.Default); // float
        visibilityBuffer = AddComputeBuffer(RigConfiguration.NumPods, sizeof(int), ComputeBufferType.Default);
        textureUVBuffer = AddComputeBuffer(MaxIndices * 2, sizeof(float), ComputeBufferType.Default);
        depthSearchRadiusBuffer = AddComputeBuffer(RigConfiguration.NumPods, sizeof(int), ComputeBufferType.Default);
        depthSearchStepBuffer = AddComputeBuffer(RigConfiguration.NumPods, sizeof(int), ComputeBufferType.Default);
        if (depthDilateProfile == null)
        {
            depthDilateProfile = new DepthDilateProfile();
        }
        if (depthDilateProfile.DepthSearchRadius == null)
        {
            depthDilateProfile.HoloportationConfig = RigConfiguration;

            depthDilateProfile.DepthSearchRadius = new int[RigConfiguration.NumPods];
            depthDilateProfile.DepthSearchStep = new int[RigConfiguration.NumPods];
            for (int i = 0; i < RigConfiguration.NumPods; ++i)
            {
                depthDilateProfile.DepthSearchRadius[i] = 1;
                depthDilateProfile.DepthSearchStep[i] = 1;
            }
        }

        if (hcpc == null)
        {
            hcpc = GetComponent<HoloportControlPanelConnector>();
        }

        client.Start();
        hcpc.SendReadyStatus();
        isInitialized = true;
    }

    /// <summary>
    /// Gets the rate at which computation is happening
    /// </summary>
    public float ComputeFPS
    {
        get;
        private set;
    } = 0;

    /// <summary>
    /// Gets the number of available structs in the processing pool
    /// </summary>
    public int ProcessingPoolCount
    {
        get
        {
            return holoObjectProcessingPool.Count;
        }
    }

    /// <summary>
    /// Gets the rate at which network frames are being processed
    /// </summary>
    public float NetworkFPS
    {
        get
        {
            return client != null ? client.FPS : 0;
        }
    }

    /// <summary>
    /// Gets the number of skipped frames from the network due to slow processing
    /// </summary>
    public int NetworkSkippedFrames
    {
        get
        {
            return client != null ? client.CountSkippedFrames : 0;
        }
    }
    
    /// <summary>
    /// Event listener for when a frame is processed through the modelToTexture step
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="frame"></param>
    private void OnModelToTextureFrameProcessed(object sender, HoloportObjectStruct frame)
    {
        if (frame.ID < holoObjectProcessingPoolSize)
        {
            // We're done with this frame, add it back to the pool
            holoObjectProcessingPool.Enqueue(frame);
        }
    }

    /// <summary>
    /// Fires when a frame is ready from fusion
    /// </summary>
    /// <param name="sender"></param>
    /// <param name="frame"></param>
    private void OnFusionFrameReady(object sender, HoloportObjectStruct frame)
    {
        frame.processStartTime = DateTimeOffset.Now;

        // We've received a frame from fusion, start computing on it.
        uvGeneratedList.Enqueue(frame);
    }

    /// <summary>
    /// Check if Montage4D is activated
    /// </summary>
    /// <returns></returns>
    bool isRenderingMontage4D()
    {
        return EffectWeightArray[(int)RenderEffectState.Montage4D] > 1e-4;
    }

    /// <summary>
    /// Check if the Holographic Effect is activated
    /// </summary>
    /// <returns></returns>
    bool isRenderingHolographicEffect()
    {
        return EffectWeightArray[(int)RenderEffectState.Montage4D] > 1e-4;
    }

    /// <summary>
    /// Every 4 seconds, slide a normal band
    /// </summary>
    /// <returns></returns>
    IEnumerator NormalBandSlide()
    {
        yield return new WaitForSeconds(4f);
        while (true)
        {
            if (isDecreasingNormalBandCenter)
                if (isDecreasingNormalBandCenter)
                {
                    if (NormalBandCenter > -0.6f)
                    {
                        NormalBandCenter -= 0.008f;
                    }
                    else
                    {
                        NormalBandCenter = 1.0f;
                        yield return new WaitForSeconds(UnityEngine.Random.Range(4.0f, 6.0f));
                        //isDecreasingNormalBandCenter = false;
                    }

                }
                else
                {
                    if (NormalBandCenter < 1.0f)
                    {
                        NormalBandCenter += 0.02f;
                    }
                    else
                    {
                        NormalBandCenter = -0.4f;
                        yield return new WaitForSeconds(UnityEngine.Random.Range(4.0f, 6.0f));
                        //isDecreasingNormalBandCenter = true;
                    }
                }

            yield return new WaitForSeconds(.02f);
        }
    }

    /// <summary>
    /// Smooth transition among the rendering effects. Called on Update()
    /// </summary>
    void OnRenderEffectStateChange()
    {
        if (PrevRenderEffectState == CurrentRenderEffectState) return;
        if (renderEffectStateChangeCoroutine != null)
            StopCoroutine(renderEffectStateChangeCoroutine);
        renderEffectStateChangeCoroutine = ChangeRenderEffect((int)CurrentRenderEffectState);
        StartCoroutine(renderEffectStateChangeCoroutine);
        PrevRenderEffectState = CurrentRenderEffectState;
    }

    /// <summary>
    /// Change the current rendering effect. Called on KeyPress events 1-9, and ,.
    /// </summary>
    /// <param name="targetRenderState"></param>
    /// <returns></returns>
    IEnumerator ChangeRenderEffect(int targetRenderState)
    {
        int renderStateCount = (int)RenderEffectState.RenderEffectStateCount;
        while (EffectWeightArray[targetRenderState] < 1)
        {
            EffectWeightArray[targetRenderState] += EffectChangeRate;
            for (int i = 0; i < renderStateCount; i++)
            {
                if (i != targetRenderState && EffectWeightArray[i] > 0)
                {
                    EffectWeightArray[i] -= EffectChangeRate;
                }

            }
            yield return new WaitForSeconds(0.02f);
        }

        // Clear other effects
        for (int i = 0; i < renderStateCount; i++)
            if (i != targetRenderState)
                EffectWeightArray[i] = 0;
        EffectWeightArray[targetRenderState] = 1;
    }

    /// <summary>
    /// Late Update is called before scene rendering but after updating
    /// </summary>
    void LateUpdate()
    {
        if (Input.GetKeyDown(KeyCode.F9))
        {
            int currentEffectID = (int)CurrentRenderEffectState;
            string filename = string.Format("Screen_{0}_{1}_{2}.png", debugSnapshotID, currentEffectID, System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss"));
            string path = Environment.CurrentDirectory + DebugPath + filename;
            ScreenCapture.CaptureScreenshot(path);
        }
    }

    /// <summary>
    /// initialize the camera parameters in the vertex compute shader Montage4D, run once only
    /// </summary>
    private void initCameraParametersInVertexComputeShader()
    {
        visibilitiesData = new int[RigConfiguration.NumPods];
        visibilityRankings = new int[RigConfiguration.NumPods];
        textureCameraMatrices = new float[4 * RigConfiguration.NumPods];
        //currentCamCenterBuffer = new float[3];

        for (int i = 0; i < RigConfiguration.NumPods; ++i)
            for (int j = 0; j < 4; ++j)
                textureCameraMatrices[i * 4 + j] = mLocalImageComputeScript.mCameraCalibrationArray[i].projectiveTextureCenter[j];
        ComputeVertexWeightsShader.SetFloats("texCamCenters", textureCameraMatrices);
        textureMatrices = new float[16 * RigConfiguration.NumPods];
        invTextureCameraMatrices = new float[16 * RigConfiguration.NumPods];
        modelTransformMatrices = new float[16];
        // cameraMatrixBuffer = new float[16];
        unity_object2WorldMatrix = new float[16];

        // as for unity 5.4, there is no set matrices method for computer shaders
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
            for (int j = 0; j < 16; ++j)
            {
                textureMatrices[i * 16 + j] = mLocalProjectiveTextureScript.textureMatrix[i][j];
                invTextureCameraMatrices[i * 16 + j] = mLocalProjectiveTextureScript.invTextureMatrix[i][j];
            }
        ComputeVertexWeightsShader.SetFloats("gl_TextureMatrices", textureMatrices);
        ComputeVertexWeightsShader.SetFloats("gl_TextureInvMatrices", invTextureCameraMatrices);
        Debug.Log("[HoloPortScript][initCameraParametersInVertexComputeShader] Complete. \n");
    }

    /// <summary>
    /// Update the Temporal Texture Fields (TTF)
    /// TODO: Use the visibility
    /// </summary>
    void UpdateViewDependentTemporalTextureWeights()
    {
        if (!isRenderingMontage4D()) return;
        if (!EnableViewDependentTemporalTextureFields) return;

        // calculate the min and max target texture fields for normalization
        float maxTTF = 0.0f, minTTF = 1000.0f;

        Vector3 capCenter = Vector3.zero;
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
        {
            Vector3 temp = mLocalProjectiveTextureScript.projectiveTextureCameras[i].transform.position;
            capCenter += new Vector3(temp.x, temp.y, temp.z);
        }
        capCenter = capCenter / ((float)RigConfiguration.NumPods);

        for (int i = 0; i < RigConfiguration.NumPods; ++i)
        {
            Vector3 targetTextureCenter = mLocalProjectiveTextureScript.projectiveTextureCameras[i].transform.position;

            // Suppose Y is up
            //Vector2 a = new Vector2(tc.x, tc.z);
            //Vector2 b = new Vector2(Camera.main.transform.position.x, Camera.main.transform.position.z);
            //TargetTTW[i] = Math.Max(0.0001f, Vector2.Distance(a, b) - 1 * ViewPreference[i]);

            // This is a naive approach
            // calculate the distance in the world matrix, a better approach is to calculate the visibilites in pixel level in the compute shader
            Vector3 a = targetTextureCenter;
            Vector3 b = Camera.main.transform.position;
            TargetTextureFields[i] = -Vector3.Dot(b - capCenter, a - capCenter);

            /// OLD WAY TO CALCULATE WEIGHT
            //Math.Max(0.0001f, Vector3.Distance(a, b) - 1 * ViewPreference[i]);


            // TargetTextureFields[i] = Vector3.Distance(Camera.current.transform.position, tt); 

            maxTTF = Math.Max(maxTTF, TargetTextureFields[i]);
            minTTF = Math.Min(minTTF, TargetTextureFields[i]);
        }

        int goodViews = 0;
        // update the temporal texture fields smoothly over the time
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
        {
            TargetTextureFields[i] = Math.Min(1.0f, 1.0f - (TargetTextureFields[i] - minTTF) / (maxTTF - minTTF));
            TargetTextureFields[i] = TargetTextureFields[i] * TargetTextureFields[i] * MAX_TTF_RANGE;
            if (TargetTextureFields[i] < MAX_TTF_RANGE - 1.0f) TargetTextureFields[i] /= MAX_TTF_RANGE; else ++goodViews;
            if (TargetTextureFields[i] < MAX_TTF_RANGE - 0.1f) TargetTextureFields[i] /= 1.3f;
            float difference = TargetTextureFields[i] - TemporalTextureFields[i];
            TemporalTextureFields[i] += difference / TemporalTransitionSpeed;
        }
    }

    /// <summary>
    /// The visibilities are used to determine the view-dependent global weights for each view.
    /// This may slow down the rendering process a lot, closed for now
    /// </summary>
    void UpdateMontage4DVisibilities()
    {
        if (!isRenderingMontage4D()) return;
        if (!enableMontage4DSlowVisibilitiesTesting) return;
        for (int i = 0; i < RigConfiguration.NumPods; ++i) visibilityRankings[i] = i;
        visibilityBuffer.GetData(visibilitiesData);
        for (int j = RigConfiguration.NumPods; j > 0; --j)
        {
            for (int i = 0; i < j - 1; ++i)
            {
                if (visibilitiesData[i] < visibilitiesData[i + 1])
                {
                    int t = visibilitiesData[i];
                    visibilitiesData[i] = visibilitiesData[i + 1];
                    visibilitiesData[i + 1] = t;

                    t = visibilityRankings[i];
                    visibilityRankings[i] = visibilityRankings[i + 1];
                    visibilityRankings[i + 1] = t;
                }
            }
        }
        Debug.Log(visibilityRankings[0] + ": " + visibilitiesData[0] + "\n"
                + visibilityRankings[1] + ": " + visibilitiesData[1] + "\n"
                + visibilityRankings[2] + ": " + visibilitiesData[2]);
        // visibilityRankingBuffer.SetData(visibilityRankings); 
        // materialToUse.SetBuffer("visibilities", visibilityRankingBuffer);
    }

    /// <summary>
    /// Update the uniforms and buffers in Montage4D shaders
    /// </summary>
    void UpdateMontage4DUniformsAndBuffers(Material materialToUse)
    {
        materialToUse.SetInt("debugSingleView", debugSingleView);
        materialToUse.SetInt("debugDiscontinuity", debugDiscontinuity);
        if (!isRenderingMontage4D()) return;
        materialToUse.SetBuffer("vtGeodesicWeights", vertexGeodesicWeightsBuffer);

        materialToUse.SetInt("RemoveBackgroundPixels", RemoveBackgroundPixels ? 1 : 0);
        materialToUse.SetInt("debugColorLabeledFields", debugColorLabeledFields);
        materialToUse.SetInt("showTexWeights", showTexWeights);
        materialToUse.SetInt("debugSingleView", debugSingleView);
        materialToUse.SetInt("debugDiscontinuity", debugDiscontinuity);
        materialToUse.SetInt("debugNormalWeightedField", debugNormalWeightedField);
        materialToUse.SetInt("debugGeodesicFields", debugGeodesicFields);
        materialToUse.SetInt("debugSeams", debugSeams);
        materialToUse.SetInt("debugNormalWeightedBlendingColor", debugNormalWeightedBlendingColor);
        materialToUse.SetInt("debugTextureWeights", debugTextureWeights);
        materialToUse.SetInt("debugTemp", debugTemp);
        materialToUse.SetMatrix("modelTransform", transform.localToWorldMatrix); // localToWorldMatrix is I with inversed y.
        materialToUse.SetFloat("ViewDependentTTWStep", ViewDependentTemporalTextureFieldStep);
        materialToUse.SetFloat("NormalBaseWeight", NormalWeightedPercentage);
        materialToUse.SetFloat("NormalWeightsThreshold", NormalThresholdForSeams);
        materialToUse.SetFloat("NormalWeightedBlendingOpacity", NormalWeightedBlendingOpacity);
        materialToUse.SetFloatArray("TemporalTextureFields", TemporalTextureFields);
        materialToUse.SetFloat("_DiscontinuityThreshold", DiscontinuityThreshold);
    }

    /// <summary>
    /// In Montage4D, we need to pass through buffers and textures to the compute shaders
    /// </summary>
    public void UpdateAndDispatchComputeShaders()
    {
        if (!isRenderingMontage4D()) return;
        if (HasRunThisFrame) return;

        HasRunThisFrame = true;

        ComputeVertexWeightsShader.SetInt("m", stageIndexCount);
        ComputeVertexWeightsShader.SetInt("n", stageVertexCount);
        ComputeVertexWeightsShader.SetFloat("GeodesicScale", (float)Math.Pow(2, GeodesicScale));
        ComputeVertexWeightsShader.SetFloat("NormalWeightsThreshold", NormalThresholdForSeams);

        // the modelTransform matrix and unity_ObjectToWorld matrix are used to determine the texture coordinates
        for (int i = 0; i < 16; ++i) modelTransformMatrices[i] = transform.localToWorldMatrix[i];
        Matrix4x4 camMatrix;
#if !M4D_USE_RENDERER_IN_HOLOPORT_SCRIPT
        Matrix4x4 V = Camera.main.worldToCameraMatrix;
        Matrix4x4 P = Camera.main.projectionMatrix;
        if (d3d)
        {
            // Invert Y for rendering to a render texture
            for (int i = 0; i < 4; i++)
            {
                P[1, i] = -P[1, i];
            }
            // Scale and bias from OpenGL -> D3D depth range
            for (int i = 0; i < 4; i++)
            {
                P[2, i] = P[2, i] * 0.5f + P[3, i] * 0.5f;
            }
        }
        camMatrix = P * V;
#else
        camMatrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        for (int i = 0; i < 16; ++i) unity_object2WorldMatrix[i] = camMatrix[i];
#endif
        ComputeVertexWeightsShader.SetFloats("modelTransform", modelTransformMatrices);
        ComputeVertexWeightsShader.SetFloats("unity_ObjectToWorld", unity_object2WorldMatrix);

        // as for unity 5.4, there is no set matrices method for computer shaders
        for (int i = 0; i < RigConfiguration.NumPods * 16; ++i) textureMatrices[i] = mLocalProjectiveTextureScript.textureMatrix[i / 16][i % 16];
        ComputeVertexWeightsShader.SetFloats("gl_TextureMatrices", textureMatrices);

        // deprecated for efficienty, invert texture matrix is disabled, may be used for accounting the distance in the depth discontinuities algorithm. 
#if !M4D_EFFICIENTY_CODE_PRONE
        for (int i = 0; i < RigConfiguration.NumPods; ++i)
            for (int j = 0; j < 16; ++j)
                invTexMatrixBuffer[i * 16 + j] = mLocalProjectiveTextureScript.invTextureMatrix[i][j];
        ComputeVertexWeightsShader.SetFloats("gl_TextureInvMatrices", invTexMatrixBuffer);
#endif
        // textures
        ComputeVertexWeightsShader.SetTexture(initVertexWeightsKernelIndex, "AtlasColorMap", mLocalImageComputeScript.renderTextureAtlas);
        ComputeVertexWeightsShader.SetTexture(initVertexWeightsKernelIndex, "AtlasDepthMap", mLocalProjectiveTextureScript.atlasDepthMap);
        ComputeVertexWeightsShader.SetBuffer(initVertexWeightsKernelIndex, "verticesHalf", vertexHalfBuffer);
        ComputeVertexWeightsShader.SetBuffer(initVertexWeightsKernelIndex, "vtTexWeights", vertexTextureFieldsBuffer);

        if (Camera.main != null) ComputeVertexWeightsShader.SetVector("currentCamCenter", Camera.main.transform.position);
        ComputeVertexWeightsShader.SetBuffer(initVertexWeightsKernelIndex, "vtGeodesicWeights", vertexGeodesicWeightsBuffer);
        ComputeVertexWeightsShader.SetBuffer(initTriangleSeamsKernelIndex, "indices", indexBuffer);
        ComputeVertexWeightsShader.SetBuffer(initTriangleSeamsKernelIndex, "vtTexWeights", vertexTextureFieldsBuffer);
#if USE_OCCLUSION_SEAMS
        ComputeVertexWeightsShader.SetBuffer(initTriangleSeamsKernelIndex, "vtOcculusionSeams", vertexOcculusionSeamsBuffer);
        ComputeVertexWeightsShader.SetBuffer(initVertexWeightsKernelIndex, "vtOcculusionSeams", vertexOcculusionSeamsBuffer);
        ComputeVertexWeightsShader.SetBuffer(vertexWeightsKernelIndex, "vtOcculusionSeams", vertexOcculusionSeamsBuffer);
#endif
        //
        ComputeVertexWeightsShader.SetBuffer(initTriangleSeamsKernelIndex, "vtGeodesicWeights", vertexGeodesicWeightsBuffer);
        ComputeVertexWeightsShader.SetBuffer(vertexWeightsKernelIndex, "indices", indexBuffer);
        ComputeVertexWeightsShader.SetBuffer(vertexWeightsKernelIndex, "verticesHalf", vertexHalfBuffer);
        ComputeVertexWeightsShader.SetBuffer(vertexWeightsKernelIndex, "vtGeodesicWeights", vertexGeodesicWeightsBuffer);
        const int VERTEX_BLOCK_STRIDE = 32;
        const int TRI_BLOCK_STRIDE = 32;
        //Debug.Log("Dispatch\n");
        ComputeVertexWeightsShader.Dispatch(initVertexWeightsKernelIndex, VERTEX_BLOCK_STRIDE, VERTEX_BLOCK_STRIDE, 1);   // support 262144 vertices at most
        ComputeVertexWeightsShader.Dispatch(initTriangleSeamsKernelIndex, TRI_BLOCK_STRIDE, TRI_BLOCK_STRIDE, 1);         // support 3145728 indices at most
        ComputeVertexWeightsShader.SetFloat("GeodesicScale", (float)Math.Pow(2, GeodesicScale));

        const float DIFFUSION_STEP_SIZE = 20.0f;
        const float DELTA = 1.0f / DIFFUSION_STEP_SIZE;
        for (int i = 0; i < SpatialDiffusionIterations; ++i)
        {
            ComputeVertexWeightsShader.SetFloat("itLimit", 0.99f - i * DELTA);
            ComputeVertexWeightsShader.SetInt("prev", i % 2);
            ComputeVertexWeightsShader.SetInt("curr", (i + 1) % 2);
            ComputeVertexWeightsShader.Dispatch(vertexWeightsKernelIndex, TRI_BLOCK_STRIDE, TRI_BLOCK_STRIDE, 1);         // support 3145728 indices at most
        }
    }

    /// <summary>
    /// Keyboard events are called per frame udpate
    /// </summary>
    unsafe void UpdateKeyboardEvents()
    {
        if (!useHotKey) return;
        int renderStateCount = (int)RenderEffectState.RenderEffectStateCount;

        // use comma to decrease the rendering effects, period to increase the rendering effects
        if (Input.GetKeyUp(KeyCode.Comma))
        {
            CurrentRenderEffectState = (RenderEffectState)(((int)(CurrentRenderEffectState - 1 + renderStateCount)) % (renderStateCount));
        }
        else if (Input.GetKeyUp(KeyCode.Period))
        {
            CurrentRenderEffectState = (RenderEffectState)(((int)(CurrentRenderEffectState + 1)) % (renderStateCount));
        }

        // press 1 for the original rendering algorithm, and press 2 for the Montage4D rendering algorithm
        if (!Input.GetKey(KeyCode.LeftAlt) && !Input.GetKey(KeyCode.LeftControl) && !Input.GetKey(KeyCode.LeftShift))
        {
            // use the numeric keys (main pad) [1, renderStateCount] to change the rendering effects directly
            for (int i = 0; i < renderStateCount; ++i)
            {
                if (Input.GetKeyDown(keyCodes[i]))
                {
                    CurrentRenderEffectState = (RenderEffectState)(((int)i) % (renderStateCount));
                }
            }

            // use the numeric keypads [0, RigConfiguration.NumPods) to observe the back-projection from a specific view
            for (int i = 0; i < RigConfiguration.NumPods; ++i)
            {
                if (Input.GetKeyUp(KeyCode.Keypad0 + i))
                {
                    debugSingleView = i;
                }
            }
            // use the numeric keypad 8 to enable or disable
            if (Input.GetKeyUp(KeyCode.Keypad0 + RigConfiguration.NumPods))
            {
                debugSingleView += RigConfiguration.NumPods * ((debugSingleView < RigConfiguration.NumPods) ? 1 : -1);
            }

            // use number pad 9 to toggle the discontinuity
            if (Input.GetKeyUp(KeyCode.Keypad0 + RigConfiguration.NumPods + 1))
            {
                debugDiscontinuity = 1 - debugDiscontinuity;
            }
        }

        // use alt and numeric keys for demoting a view globally
        if (Input.GetKey(KeyCode.LeftAlt))
        {
            for (int i = 0; i < renderStateCount; ++i)
            {
                if (Input.GetKeyUp(KeyCode.Keypad0 + i))
                {
                    ViewPreference[i] -= 0.2f;
                }
            }
        }

        // use ctrl and numeric keys for promoting a view globally
        if (Input.GetKey(KeyCode.LeftControl))
        {
            for (int i = 0; i < renderStateCount; ++i)
            {
                if (Input.GetKeyUp(KeyCode.Keypad0 + i))
                {
                    ViewPreference[i] += 0.2f;
                }
            }
        }

        /**
         * Function keys
         * 
         * F1 - Show the color labels for the texture fields
         * F2 - Show the Geodesic Fields by the compute shaders
         * F3 - debugNormalWeightedField
         * F4 - debugSeams
         * F5 - showTexWeights
         * F6 - debugNormalWeightedBlendingColor
         * F7 - debugTextureWeights
         * F9 - take a screen shot to the debug folder
         * F10 - run the Spline-curved camera path for capturing
         * Enter - temporal debug
         *
         */
        if (Input.GetKeyUp(KeyCode.F1))
        {
            debugColorLabeledFields = 1 - debugColorLabeledFields;
            if (debugNormalWeightedField == 1)
            {
                debugGeodesicFields = 0;
                debugNormalWeightedBlendingColor = 0;
                debugNormalWeightedField = 0;
                debugSeams = 0;
                showTexWeights = 0;
            }
            Debug.Log("Toggle Visualization of the color-labeled texture fields " + debugColorLabeledFields + ".\n");
        };
        if (Input.GetKeyUp(KeyCode.F2))
        {
            debugGeodesicFields = 1 - debugGeodesicFields;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugNormalWeightedBlendingColor = 0;
                debugNormalWeightedField = 0;
                debugSeams = 0;
                showTexWeights = 0;
            }
            Debug.Log("Toggle Visualization of the GeodesicFields " + debugGeodesicFields + ".\n");
        };

        if (Input.GetKeyUp(KeyCode.F3))
        {
            debugNormalWeightedField = 1 - debugNormalWeightedField;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugGeodesicFields = 0;
                debugSeams = 0;
                showTexWeights = 0;
                debugNormalWeightedBlendingColor = 0;
            }
            Debug.Log("Toggle Visualization of the NormalWeightsField " + debugGeodesicFields + ".\n");
        };

        if (Input.GetKeyUp(KeyCode.F4))
        {
            debugSeams = 1 - debugSeams;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugGeodesicFields = 0;
                debugNormalWeightedField = 0;
                showTexWeights = 0;
                debugNormalWeightedBlendingColor = 0;
            }
            Debug.Log("Toggle Visualization of the Seams " + debugGeodesicFields + ".\n");
        };

        if (Input.GetKeyUp(KeyCode.F5))
        {
            showTexWeights = 1 - showTexWeights;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugGeodesicFields = 0;
                debugNormalWeightedField = 0;
                debugSeams = 0;
                debugNormalWeightedBlendingColor = 0;
            }
            Debug.Log("Toggle Visualization of the texture weights " + debugGeodesicFields + ".\n");
        }

        if (Input.GetKeyUp(KeyCode.F6))
        {
            debugNormalWeightedBlendingColor = 1 - debugNormalWeightedBlendingColor;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugGeodesicFields = 0;
                debugNormalWeightedField = 0;
                debugSeams = 0;
                showTexWeights = 0;
            }
            Debug.Log("Toggle Visualization of the normal weighted blending results " + debugNormalWeightedBlendingColor + ".\n");
        }

        if (Input.GetKeyUp(KeyCode.F7))
        {
            debugTextureWeights = 1 - debugTextureWeights;
            if (debugNormalWeightedField == 1)
            {
                debugColorLabeledFields = 0;
                debugGeodesicFields = 0;
                debugNormalWeightedField = 0;
                debugSeams = 0;
                showTexWeights = 0;
            }
            Debug.Log("Toggle Visualization of the texture weights " + debugNormalWeightedBlendingColor + ".\n");
        }

        // for debugging only
        if (Input.GetKeyUp(KeyCode.KeypadEnter))
        {
            //debugOcculusionSeams = 1;
            //debugSingleView = 7;
            //CurrentRenderEffectState = RenderEffectState.Montage4D; 
            ViewPreference[0] = -0.7f;
            ViewPreference[1] = -0.4f;
            ViewPreference[7] = 1.0f;
        }

        if (Input.GetKeyUp(KeyCode.X))
        {
            Debug.Log("Simplify took: " + stopWatch.ElapsedMilliseconds + "ms");
        }
    }

    System.Diagnostics.Stopwatch watch = System.Diagnostics.Stopwatch.StartNew();
    /// <summary>
    /// Update is called once per frame
    /// </summary>
    unsafe void Update()
    {
        UpdateKeyboardEvents();

        if (PrevRenderEffectState != CurrentRenderEffectState)
        {
            OnRenderEffectStateChange();
        }

        //Debug.Log("Frame:" + Time.frameCount + " Update --- object: " + name);

        if (Materialize && !pastMaterializedValue)
        {
            pastMaterializedValue = true;
            ExplosionAmount = 1;

            if (ExplosionAnimationTimer != null)
            {
                ExplosionAnimationTimer.Dispose();
                ExplosionAnimationTimer = null;
            }

            ExplosionAnimationTimer = new Timer((object state) =>
            {
                if (ExplosionAmount < 0.001)
                {
                    //Stop explosion and start fading normal
                    ExplosionAmount = 0;
                    ExplosionAnimationTimer.Dispose();
                    /* ExplosionAnimationTimer = new Timer((object state2) =>
                     {
                         if(EffectWeightArray [(int)targetExplosionMode] < 0.001)
                         {
                             EffectWeightArray [(int)targetExplosionMode] = 0;
                             ExplosionAnimationTimer.Dispose();
                             ExplosionAnimationTimer = null;

                         }
                         else
                         {
                             EffectWeightArray [(int)targetExplosionMode] -= 1 / (normalFPS * ExplosionTimeSec);
                         }

                     }, null, TimeSpan.FromSeconds(0), TimeSpan.FromSeconds(1 / explosionFPS));*/

                }
                else
                {
                    ExplosionAmount -= 1 / (explosionFPS * ExplosionTimeSec);
                }
            }, null, TimeSpan.FromSeconds(0), TimeSpan.FromSeconds(1 / explosionFPS));
        }

        if (!Materialize && pastMaterializedValue)
        {
            pastMaterializedValue = false;
            ExplosionAmount = 0;

            if (ExplosionAnimationTimer != null)
            {
                ExplosionAnimationTimer.Dispose();
                ExplosionAnimationTimer = null;
            }

            ExplosionAnimationTimer = new Timer((object state2) =>
            {
                if (EffectWeightArray[(int)targetExplosionMode] > 0.999)
                {
                    EffectWeightArray[(int)targetExplosionMode] = 1;
                    ExplosionAnimationTimer.Dispose();
                    ExplosionAnimationTimer = new Timer((object state) =>
                    {
                        if (ExplosionAmount > 0.999)
                        {
                            ExplosionAmount = 1;
                            ExplosionAnimationTimer.Dispose();
                            ExplosionAnimationTimer = null;
                        }
                        else
                        {
                            ExplosionAmount += 1 / (explosionFPS * ExplosionTimeSec);
                        }
                    }, null, TimeSpan.FromSeconds(0), TimeSpan.FromSeconds(1 / explosionFPS));

                }
                else
                {
                    EffectWeightArray[(int)targetExplosionMode] += 1 / (normalFPS * ExplosionTimeSec);
                }

            }, null, TimeSpan.FromSeconds(0), TimeSpan.FromSeconds(1 / explosionFPS));
        }

        bool allowedToUpload = (activeHoloportsCount == 1 || !useDataUploadInterleaving) ? true : Time.frameCount % (dataUploadFramesStride * holoportsCount) == dataUploadFramesStride * holoportID;

        /// ACTUAL DATA UPDATE
        if (uvGeneratedList.Count > 0 && allowedToUpload)
        {
            watch.Stop();
            //  Debug.Log("Time between frames = " + watch.ElapsedMilliseconds + " ");
            watch = System.Diagnostics.Stopwatch.StartNew();
            //var lockStartTime = Stopwatch.GetTimestamp();

            //var lockEndTime = Stopwatch.GetTimestamp();
            //var lockTimeMS = (float)(lockEndTime - lockStartTime) / (Stopwatch.Frequency / 1000);

            bool shouldDebayerColor = mLocalImageComputeScript != null && EffectWeightArray[(int)RenderEffectState.Occlusion] < 1.0f && stageIndexCount > 0;

            HoloportObjectStruct curr = null;

            if (uvGeneratedList.Count <= 0)
            {
                Debug.LogError("trying to upload data before its ready. How?");
            }
            curr = uvGeneratedList.Dequeue();
            textureUVBuffer.SetData(curr.uvData);

            mLocalHoloportModelToTextureScript.lastProcessedStruct.CopyInfo(curr);

            if (shouldDebayerColor)
                mLocalImageComputeScript.LoadAllColorFiles(curr.colorData);


                
            vertexHalfBuffer.SetData(curr.vertexData);
            indexBuffer.SetData(curr.indexData);
            stageIndexCount = curr.indexCount;
            stageVertexCount = curr.vertexCount;

            //right before rendering
            onDataReadyCallback?.Invoke(curr.rawMJPEGData, curr.MJPEGDataSize);

            //var uploadTimeMS = (float)(Stopwatch.GetTimestamp() - lockEndTime) / (Stopwatch.Frequency / 1000);
            if (Time.realtimeSinceStartup - lastTimeStatsWereLogged > 5)
            {
                lastTimeStatsWereLogged = Time.realtimeSinceStartup;
                // Debug.Log(string.Format("{0}: VertexBuffer.length={1}K, IndexBuffer.length={1}K, lockTime={3}ms, uploadTime={4}ms\n", name, vertexBufferData.Length / 1000, indexBufferData.Length / 1000, lockTimeMS, uploadTimeMS));
            }

            if (firstUpdate)
            {
                firstUpdate = false;
                initCameraParametersInVertexComputeShader();
            }
            ComputeVertexWeightsShader.SetInt("m", stageIndexCount);
            ComputeVertexWeightsShader.SetInt("n", stageVertexCount);

            HasRunThisFrame = false;

            mLocalProjectiveTextureScript.RenderAllProjectiveCameras();

            ComputeFPS = Utils.ComputeFPS(lastUpdate, DateTimeOffset.Now, 1, ComputeFPS);

            lastUpdate = DateTimeOffset.Now;

            if (curr.ID < holoObjectProcessingPoolSize)
            {
                if (renderUVSpace)
                {
                    mLocalHoloportModelToTextureScript.ProcessFrame(curr);
                }
                else
                {
                    // We're done with this frame, add it back to the pool
                    holoObjectProcessingPool.Enqueue(curr);
                }
            }
        }
    }

    /// <summary>
    /// Sets the processing pool size
    /// </summary>
    /// <param name="size">The new size</param>
    public void SetProcessingPoolSize(int size)
    {
        int newCount = size - holoObjectProcessingPoolSize;
        for (int i = 0; i < newCount; i++)
        {
            holoObjectProcessingPool.Enqueue(HoloportObjectStruct.CreateWithSettings(holoObjectProcessingPoolSize + i, SettingsManager.Instance));
        }
        holoObjectProcessingPoolSize = size;
    }

    /// <summary>
    /// OnRenderObject is called after camera has rendered the scene.
    /// </summary>
    void OnRenderObject()
    {
        if (isInitialized)
        {
            // for multiple cameras and not all of them requires color projection
            if ((Camera.current.cullingMask & (1 << LayerMask.NameToLayer(name))) == 0) return;
            //Debug.Log("Frame: " + Time.frameCount + " OnRenderObject --- object: " + name + " camera: " + Camera.current.name);

            if (PauseDrawing) return;

            bool notRigOcclusionCamera = !Camera.current.name.Contains("Projective");
            bool useVoxelizationForThisCamera = Voxelize && notRigOcclusionCamera;
            useVoxelizationForThisCamera = useVoxelizationForThisCamera && VoxelSize >= 0.0099;
            Material materialToUse = HoloPortMaterial[Convert.ToInt32(useVoxelizationForThisCamera)];

            if (renderUVSpace && notRigOcclusionCamera)
            {
                materialToUse = HoloPortMaterial[(int)HoloportMaterialType.ObjectCapture]; ;
                materialToUse.SetBuffer("textureUVs", textureUVBuffer);
            }

            materialToUse.SetInt("_NumPods", RigConfiguration.NumPods);

            materialToUse.SetFloatArray("UseCameraTexture", UseCameraTexture);
            // set buffers and parameters to the shader
            materialToUse.SetBuffer("indices", indexBuffer);
            materialToUse.SetBuffer("verticesHalf", vertexHalfBuffer);

            materialToUse.SetInt("colorImageWidth", RigConfiguration.ColorWidth);
            materialToUse.SetInt("ColorImageHeight", RigConfiguration.ColorHeight);

            //depth check parameters
            materialToUse.SetFloat("_DiscontinuityThreshold", DiscontinuityThreshold);
            materialToUse.SetFloat("_DotProductNormalWeightThreshold", DotProductNormalWeightThreshold);
            materialToUse.SetFloat("_ColorWeightThreshold", ColorWeightThreshold);

            depthSearchRadiusBuffer.SetData(depthDilateProfile.DepthSearchRadius);
            depthSearchStepBuffer.SetData(depthDilateProfile.DepthSearchStep);
            materialToUse.SetBuffer("_DepthSearchRadius", depthSearchRadiusBuffer);
            materialToUse.SetBuffer("_DepthSearchStep", depthSearchStepBuffer);


            // update Montage4D uniforms and buffers 
            UpdateMontage4DVisibilities();
            UpdateViewDependentTemporalTextureWeights();
            UpdateMontage4DUniformsAndBuffers(materialToUse);

            // for debug only to observe if the reconstructed scene has too many vertices for the compute shaders 
            if (stageIndexCount > maxIndexCount) maxIndexCount = stageIndexCount;
            if (stageVertexCount > maxVertexCount) maxVertexCount = stageVertexCount;
            if (_DebugIndexAndVertexCount) Debug.Log(maxIndexCount + "\t" + maxVertexCount);

            // rendering effects, normals, explosions, and holographic effects
            materialToUse.SetFloat("_NormalBand", NormalBandCenter);
            materialToUse.SetFloat("ExplosionAmount", notRigOcclusionCamera ? ExplosionAmount : 0);

            materialToUse.SetMatrix("modelTransform", transform.localToWorldMatrix);
            //Debug.Log(transform.localToWorldMatrix + "\n");

            materialToUse.SetMatrix("inverseModelTransform", transform.localToWorldMatrix.inverse);


            materialToUse.SetColor("_HighlightColor", HighlightColor);
            materialToUse.SetColor("_HologramTint", HologramTint);
            materialToUse.SetFloat("_RimPower", RimPower);
            materialToUse.SetFloat("_RimVis", RimVis);
            materialToUse.SetFloat("_ModelOpacity", ModelOpacity);

            materialToUse.SetInt("ApplyLightingToNormalMap", ApplyLightingToNormalMap ? 1 : 0);

            materialToUse.SetFloat("Weight_Color", EffectWeightArray[(int)RenderEffectState.Default]);
            materialToUse.SetFloat("Weight_Normal", EffectWeightArray[(int)RenderEffectState.Normal]);
            materialToUse.SetFloat("Weight_ShinyGeometry", EffectWeightArray[(int)RenderEffectState.ShinyGeometry]);
            materialToUse.SetFloat("Weight_HolographicEffect", EffectWeightArray[(int)RenderEffectState.HolographicEffect]);
            materialToUse.SetFloat("Weight_Occlusion", EffectWeightArray[(int)RenderEffectState.Occlusion]);
            if (!notRigOcclusionCamera)
            {
                materialToUse.SetFloat("Weight_Occlusion", 2.0f);
            }
            materialToUse.SetFloat("Weight_Wireframe", EffectWeightArray[(int)RenderEffectState.Wireframe]);
            materialToUse.SetFloat("Weight_Montage4D", EffectWeightArray[(int)RenderEffectState.Montage4D]);
            materialToUse.SetFloat("Weight_Intrinsic4D", EffectWeightArray[(int)RenderEffectState.Intrinsic4D]);
            materialToUse.SetFloat("Weight_Art", EffectWeightArray[(int)RenderEffectState.Art]);

            if (Voxelize)
            {
                materialToUse.SetFloat("voxel_size", VoxelSize);
                materialToUse.SetFloat("voxel_no_skip_size", VoxelNoSkipSize);
            }

            if (cullingBox1 != null && notRigOcclusionCamera)
            {
                materialToUse.SetVector("cullingBbox1Center", cullingBox1.position);
                materialToUse.SetVector("cullingBbox1HalfSize", cullingBox1.localScale * 0.5f);
            }
            else
            {
                materialToUse.SetVector("cullingBbox1HalfSize", Vector4.zero);
            }

            if (cullingBox2 != null && notRigOcclusionCamera)
            {
                materialToUse.SetVector("cullingBbox2Center", cullingBox2.position);
                materialToUse.SetVector("cullingBbox2HalfSize", cullingBox2.localScale * 0.5f);
            }
            else
            {
                materialToUse.SetVector("cullingBbox2HalfSize", Vector4.zero);
            }

            materialToUse.SetFloat("GlobalAlphaMultiplier", GlobalAlpha);

            materialToUse.SetPass(0);

            if (stageIndexCount == 0)
            {
                materialToUse.SetInt("_UsePointCloud", 1);
                Graphics.DrawProceduralNow(MeshTopology.Points, stageVertexCount);
            }
            else
            {
                materialToUse.SetInt("_UsePointCloud", 0);
                Graphics.DrawProceduralNow(useVoxelizationForThisCamera ? MeshTopology.Points : MeshTopology.Triangles, stageIndexCount);
            }
        }
    }

    void OnDestroy()
    {
        ReleaseResources();
    }

    void OnDisable()
    {
        ReleaseResources();
    }

    void ReleaseResources()
    {
        if (!isInitialized) return;
        isInitialized = false;
        activeHoloportsCount--;

        if (mLocalHoloportModelToTextureScript)
        {
            mLocalHoloportModelToTextureScript.FrameProcessed -= OnModelToTextureFrameProcessed;
        }

        foreach (ComputeBuffer buffer in this.bufferList)
        {
            buffer.Release();
        }
        this.bufferList.Clear();

        try
        {
            client.Dispose();
            client = null;
        }
        catch (Exception e)
        {
            Debug.LogError("Error releasing resources");
            Debug.LogException(e);
        }

        Debug.Log("[HoloPortScript] Successfully exit.\n");
    }

    /// <summary>
    /// Creates a Compute Buffer with the given params, and adds it to the list of tracked buffers
    /// </summary>
    /// <param name="count"></param>
    /// <param name="stride"></param>
    /// <param name="type"></param>
    /// <returns></returns>
    private ComputeBuffer AddComputeBuffer(int count, int stride, ComputeBufferType type)
    {
        ComputeBuffer buffer = new ComputeBuffer(count, stride, type);
        bufferList.Add(buffer);
        return buffer;
    }
}
