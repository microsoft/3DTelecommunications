using UnityEngine;
using System.Collections;
using System.IO;
using System;
using SharpDX.Direct3D9;
using UnityEngine.Rendering;
using Newtonsoft.Json.Linq;

public class CameraCalibration
{
	public string name;
	public int w;
	public int h;
	public float fx;
	public float fy;
	public float s;
	public float px;
	public float py;
	public float[] kappa = new float[8];
	public bool isR6kt = false;
	public Quaternion rotation;
	public Vector3 translation;
	public Vector3 colorScale = new Vector3();
	public Vector3 colorBias = new Vector3();

	public float invFx;
	public float invFy;
	public float invPx;
	public float invPy;

	public Matrix4x4 cameraIntrinsics;
	public Matrix4x4 rotationMatrix;
//	public Vector3 translation;					//DUPE
//	public float[] kappa_radial = new float[5];  //DUPE

	public Matrix4x4 perProjectionMatrix;  // perspective projection matrix
	public Matrix4x4 projectionMatrix;
	public Matrix4x4 modelViewMatrix = Matrix4x4.zero;

	public Vector4 projectiveTextureCenter;
	
}

struct CameraParameters
{
	public Vector4 g_F;
	public Vector4 g_P;
	public Vector4 g_Kappa;
    public Vector4 g_Tangential;
    public Vector4 g_Scale;
	public Vector4 g_Bias;
    public int     g_IsR6kt;
}

[RequireComponent(typeof(HoloPortScript))]
public class ReadAndProcessRawImage : MonoBehaviour {
    //The path is based off of CurrWorking Directory
    [Space(10)]
    [Header("[Calibration Files Path]")]
    // path to folder with ba/ calibcolorcam.txts ; can be absolute path or just a relative path from DummyData
    public string CalibFolderLocation;
    public string LocalCalibDirectoryName = "DummyData";
    // the general calibration path can start from both X:\ and \\DummyData, it's usually named calibColorCams.txt
    public const string cColorCalibFileName = "calibColorCams.txt";
    // the depth bundle adjustment path can start from both X:\ and \\DummyData, the suffix ends with _ba.txt
    public const string cDepthCalibPrefixFileName = "calib_Depth";
    public const string cDepthCalibSuffix = "_ba.txt";
    public const string c3DTMCalibFormatFilename = "calibCameras3DTM.json";

	public bool EnableAutoCalibration = true;
	[Range(1, 100f)]
	public float AutoCalibrationInverval = 10; 
	private bool firstTimeCalibration = true; 
	private CameraParameters[] cameraParametersArray;
	public bool Force3kt = false;

    public bool EnableSharpening = true;

    byte[][] camImageByteArrays;
	byte[][] camImageByteArrays2;
    //public RenderTexture[] renderTextureArray;
    [Space(10)]
    [Header("[Render Textures]")]
    public Texture2DArray inputTextureArray;
    public RenderTexture processedColorAtlas;
    public RenderTexture sharpenedColorAtlas;    
    public RenderTexture renderTextureAtlas;

    public CameraCalibration[] mCameraCalibrationArray;

	int processRawImageKernelIndex;
    int unsharpMaskKernelIndex;    
    int undistortImageKernelIndex;
    public ComputeShader processRawImageShader;

    Configuration rigConfiguration;
	ComputeBuffer cameraParameterBuffer;
    ComputeBuffer sharpMaskBuffer;    

    public float maskScale = 1.0f;

    public float[] sharpMask = {
     0.0f, -1.0f,  0.0f,
     -1.0f, 5.0f, -1.0f,
     0.0f, -1.0f,  0.0f
    };

    //public float[] sharpMask = {
    // -0.5f, -1.0f,  -0.5f,
    // -1.0f, 7.0f, -1.0f,
    // -0.5f, -1.0f,  -0.5f
    //};

    //private float[] sharpMask = {
    // 0.0f, -1.0f,  0.0f,
    // -1.0f, 5.0f, -1.0f,
    // 0.0f, -1.0f,  0.0f
    //};

    [Header("Color Grading LUT")]
    public bool useLUT = false;
    public Texture3D LUTTableTexture;

    private void Awake()
    {
        if(!GetComponent<HoloPortScript>().configInitialized)
        {
            GetComponent<HoloPortScript>().AwakeWithConfig();
        }
        //RigConfiguration check
        if (SettingsManager.Instance.config.Contains("Renderer", "UseBuiltInConfig"))
        {
            if (!SettingsManager.Instance.config["Renderer"]["UseBuiltInConfig"].BoolValue)
            {
                Debug.Log("ReadAndProcessRawImage Using " + SettingsManager.Instance.fileName + " Configuration for HoloportScript");

                if (SettingsManager.Instance.config.Contains("Renderer", "CalibrationDirectory"))
                {
                    CalibFolderLocation = SettingsManager.Instance.config["Renderer"]["CalibrationDirectory"].StringValue;
                }
                if (SettingsManager.Instance.config.Contains("Renderer", "LocalCalibDirectoryName"))
                {
                    LocalCalibDirectoryName = SettingsManager.Instance.config["Renderer"]["LocalCalibDirectoryName"].StringValue;
                }

                useLUT = SettingsManager.Instance.GetValueWithDefault("Renderer", "UseColorCorrectionLUT", false);
            }
        }
    }

    // Use this for allocating memories
    void InitializeResources()
    {
        rigConfiguration = GetComponent<HoloPortScript>().RigConfiguration;
        mCameraCalibrationArray = new CameraCalibration[rigConfiguration.NumPods];

        Debug.Log("[ReadAndProcessRawImages] Intializing resources.");
        firstTimeCalibration = true;

        processRawImageKernelIndex = processRawImageShader.FindKernel("ProcessRawImages");
        if (processRawImageKernelIndex < 0)
            Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel ProcessRawImages. Reimport the compute shader.\n");

        unsharpMaskKernelIndex = processRawImageShader.FindKernel("UnsharpMask");
        if (unsharpMaskKernelIndex < 0)
            Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel UnsharpMask. Reimport the compute shader.\n");
        
        undistortImageKernelIndex = processRawImageShader.FindKernel("UndistortImages");
        if (undistortImageKernelIndex < 0)
            Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel UndistortImages. Reimport the compute shader.\n");

        // auto calibration, load depth bundle adjustment, and color calibration files
        updateAutomaticCalibration();
        if (EnableAutoCalibration)
            InvokeRepeating("updateAutomaticCalibration", 0, AutoCalibrationInverval);

        sharpMaskBuffer = new ComputeBuffer(sharpMask.Length, sizeof(float));
        sharpMaskBuffer.SetData(sharpMask);

        InitializeRenderTextures();
    }


    #region LUT
    public void LoadLUTFiles()
    {
        int LUTTableSize = 1;

        string lutFilePath = CalibFolderLocation.Trim();
        // use the absolute path if the path starts with X:\, otherwise, use the relative path starting with DummyData
        if (lutFilePath.Contains(":\\"))
        {
            lutFilePath = lutFilePath + "\\";
        }
        else
        {
            lutFilePath = Environment.CurrentDirectory + "\\" + LocalCalibDirectoryName + "\\" + lutFilePath + "\\";
        }

        const int cRGBAChannels = 4;
        float[] rawLUTData = null;
        for (int fileNum = 0; fileNum < rigConfiguration.NumPods; fileNum++)
        {
            // fileNum + 1 --> to make it one-based indexing for the color correction files
            string filePath = lutFilePath + "cam" + (fileNum + 1).ToString() + ".cube";
            if (!File.Exists(filePath))
            {
                Debug.LogWarning("Warning: Missing Color calibration LUT files!");
                return;
            }
            string[] allLines = File.ReadAllLines(filePath);
            string[] lineSplit;
            //Parse the start of file to find size of LUT
            bool establishedSize = false;
            int lineToRead = 0;
            while(!establishedSize && lineToRead < allLines.Length)
            {
                
                lineSplit = allLines[lineToRead].Split(' ');

                ++lineToRead;
                if (lineSplit[0][0] == '#')
                {
                    //comment, so ignore 
                    continue;
                }
                else if(lineSplit[0].Equals("LUT_3D_SIZE"))
                {
                    //found hte size
                    LUTTableSize = int.Parse(lineSplit[1]);
                    establishedSize = true;
                }
            }

            if(!establishedSize)
            {
                Debug.LogError("Invalid LUT .cube file!");
                useLUT = false;
                return;
            }

            //there is exra line between size and actual data, offset it ;
            ++lineToRead;
            //found lines, now just parse the data
            if(rawLUTData == null)
            {
                LUTTableTexture = new Texture3D(LUTTableSize, LUTTableSize, LUTTableSize * rigConfiguration.NumPods, TextureFormat.RGBAFloat, false);
                
                rawLUTData = new float[cRGBAChannels * LUTTableSize * LUTTableSize * LUTTableSize * rigConfiguration.NumPods];

            }

            int rigLevelOffset = LUTTableSize * LUTTableSize * LUTTableSize * cRGBAChannels * fileNum;
            Debug.Log("Reading LUT:" + fileNum);
            for (int z = 0; z < LUTTableSize; ++ z)
            {
                int zLevelOffset = LUTTableSize * LUTTableSize * z;
                for(int y= 0; y < LUTTableSize; ++y)
                {                    
                    int yLevelOffset = LUTTableSize * y;
                    for(int x =0; x <LUTTableSize; ++x)
                    {
                        lineSplit = allLines[lineToRead].Split(' ');
                        int currValueIndex = rigLevelOffset +  cRGBAChannels * (zLevelOffset + yLevelOffset + x);

                        rawLUTData[currValueIndex + 0] =  float.Parse(lineSplit[0]);
                        rawLUTData[currValueIndex + 1] =  float.Parse(lineSplit[1]);
                        rawLUTData[currValueIndex + 2] =  float.Parse(lineSplit[2]);
                        rawLUTData[currValueIndex + 3] = 0.0f; // empty 0 Alpha
                        ++lineToRead;
                    }
                }
            }            
        } // end for all pods
        LUTTableTexture.SetPixelData(rawLUTData, 0);
        LUTTableTexture.wrapMode = TextureWrapMode.Clamp;
        LUTTableTexture.filterMode = FilterMode.Trilinear;
        LUTTableTexture.Apply();

        processRawImageShader.SetBool("UseLUT", useLUT);
        processRawImageShader.SetTexture(processRawImageKernelIndex, "LutTable", LUTTableTexture);
    }
    #endregion


    /// <summary>
    /// Late Update is called before scene rendering but after updating
    /// </summary>
    void LateUpdate()
    {
        if (Input.GetKeyUp(KeyCode.C)) ExportToPNG();
		if (Input.GetKeyDown (KeyCode.F12)) updateAutomaticCalibration(); 
    }

    void ExportToPNG()
    {
        Debug.Log("[ReadAndProcessRawImage] ExportToPNG\n");

        if(renderTextureAtlas.dimension == TextureDimension.Tex2DArray)
        {
            SaveRenderTextureArrayAsyncGPU();
        }
        else
        {
            for (int i = 0; i < rigConfiguration.NumPods; ++i) SaveRenderTexture(i);
        }        
    }

    void SaveRenderTextureArrayAsyncGPU()
    {
        AsyncGPUReadback.Request(renderTextureAtlas, 0, 0, renderTextureAtlas.width, 0, renderTextureAtlas.height, 0, renderTextureAtlas.volumeDepth, new Action<AsyncGPUReadbackRequest>
        (
            (AsyncGPUReadbackRequest request) =>
            {
                if (!request.hasError)
                {
                    // Create CPU texture array
                    Texture2DArray Destination = new Texture2DArray(renderTextureAtlas.width, renderTextureAtlas.height, request.layerCount, TextureFormat.ARGB32, false);

                    // Copy the data
                    for (var i = 0; i < request.layerCount; i++)
                    {
                        string fileName = Environment.CurrentDirectory + HoloPortScript.DebugPath + i + ".png";

                        Texture2D newTexture = new Texture2D(renderTextureAtlas.width, renderTextureAtlas.height, TextureFormat.ARGB32, false);
                        newTexture.SetPixels32(request.GetData<Color32>(i).ToArray(), 0);
                        newTexture.Apply();
                        File.WriteAllBytes(fileName, newTexture.EncodeToPNG());
                    }
                }
            }
        ));
    }

    void SaveRenderTexture(int i)
    {
        RenderTexture rt = renderTextureAtlas; //renderTextureArray[i] as RenderTexture;
        string fileName = Environment.CurrentDirectory + HoloPortScript.DebugPath + i + ".png";
        if (!string.IsNullOrEmpty(fileName))
        {
            var oldRT = RenderTexture.active;
            RenderTexture.active = rt;
            int w = rt.width;
            int h = rt.height;
            
            Texture2D newTexture = new Texture2D(w, h, TextureFormat.ARGB32, false);
            newTexture.ReadPixels(new Rect(0.0f, h * i, rt.width, h), 0, 0, false);
            newTexture.Apply(false, false);
            File.WriteAllBytes(fileName, newTexture.EncodeToPNG());

            RenderTexture.active = oldRT;
        }
    }

	public void InvokeComputeShader()
	{
        if (processRawImageKernelIndex < 0)
            Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel ProcessRawImages. Reimport the compute shader");

        int threadGroupsX = rigConfiguration.ColorWidth / 32 + (rigConfiguration.ColorWidth % 32 > 0 ? 1 : 0);
        int threadGroupsY = rigConfiguration.ColorHeight / 32 + (rigConfiguration.ColorHeight % 32 > 0 ? 1 : 0);
        // color processing
        ////processRawImageShader.SetFloat("cameraIDFloat", (float)i);
        //processRawImageShader.SetTexture(processRawImageKernelIndex, "input4", debayeredInputTextureAtlas);
        processRawImageShader.SetBuffer(processRawImageKernelIndex, "cameraParameterBuffer", cameraParameterBuffer);
        processRawImageShader.SetTexture(processRawImageKernelIndex, "inputArray", inputTextureArray);
        processRawImageShader.SetTexture(processRawImageKernelIndex, "processedColor4", processedColorAtlas);
        processRawImageShader.SetInt("numPods", rigConfiguration.NumPods);
        processRawImageShader.SetInt("colorWidth", rigConfiguration.ColorWidth);
        processRawImageShader.SetInt("colorHeight", rigConfiguration.ColorHeight);
        //LUT related
        processRawImageShader.SetBool("UseLUT", useLUT);
        processRawImageShader.SetTexture(processRawImageKernelIndex, "LutTable", LUTTableTexture);
        processRawImageShader.Dispatch(processRawImageKernelIndex, threadGroupsX, threadGroupsY, rigConfiguration.NumPods);  //???? 16 or not 16 1024/16 or 2048/16? //TODO: double check correct dispatch size for GPU cards

        if (EnableSharpening)
        {
            // sharpen
            if (unsharpMaskKernelIndex < 0)
                Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel UnsharpMask. Reimport the compute shader");
            processRawImageShader.SetTexture(unsharpMaskKernelIndex, "processedColor4", processedColorAtlas);
            processRawImageShader.SetTexture(unsharpMaskKernelIndex, "sharpenedColor4", sharpenedColorAtlas);            
            sharpMaskBuffer.SetData(sharpMask);
            processRawImageShader.SetBuffer(unsharpMaskKernelIndex, "mask", sharpMaskBuffer);
            processRawImageShader.SetFloat("maskScale", maskScale);            
            processRawImageShader.Dispatch(unsharpMaskKernelIndex , threadGroupsX, threadGroupsY, rigConfiguration.NumPods);  //???? 16 or not 16 1024/16 or 2048/16? //TODO: double check correct dispatch size for GPU cards
        }

        // undistort
        if (undistortImageKernelIndex < 0)
            Debug.LogError("[ReadAndProcessRawImage] Warning, can't find kernel UndistortImages. Reimport the compute shader");

        processRawImageShader.SetBuffer(undistortImageKernelIndex, "cameraParameterBuffer", cameraParameterBuffer);
        processRawImageShader.SetTexture(undistortImageKernelIndex, "processedColor4", EnableSharpening ? sharpenedColorAtlas : processedColorAtlas);
        processRawImageShader.SetTexture(undistortImageKernelIndex, "output4", renderTextureAtlas);
        processRawImageShader.Dispatch(undistortImageKernelIndex, threadGroupsX, threadGroupsY, rigConfiguration.NumPods);  //???? 16 or not 16 1024/16 or 2048/16? //TODO: double check correct dispatch size for GPU cards
    }

    void InitializeRenderTextures()
	{
        Debug.Log("Max Texture size is : " + SystemInfo.maxTextureSize);

        inputTextureArray = new Texture2DArray(rigConfiguration.ColorWidth, rigConfiguration.ColorHeight, rigConfiguration.NumPods, rigConfiguration.TextureFormat, false);

        processedColorAtlas = new RenderTexture(rigConfiguration.ColorWidth, rigConfiguration.ColorHeight, 0, RenderTextureFormat.ARGB32);
        processedColorAtlas.dimension = UnityEngine.Rendering.TextureDimension.Tex2DArray;
        processedColorAtlas.volumeDepth = rigConfiguration.NumPods;
        processedColorAtlas.enableRandomWrite = true;
        processedColorAtlas.Create();

        sharpenedColorAtlas = new RenderTexture(rigConfiguration.ColorWidth, rigConfiguration.ColorHeight, 0, RenderTextureFormat.ARGB32);
        sharpenedColorAtlas.dimension = UnityEngine.Rendering.TextureDimension.Tex2DArray;
        sharpenedColorAtlas.volumeDepth = rigConfiguration.NumPods;
        sharpenedColorAtlas.enableRandomWrite = true;
        sharpenedColorAtlas.Create();    

        renderTextureAtlas = new RenderTexture(rigConfiguration.ColorWidth, rigConfiguration.ColorHeight, 0, RenderTextureFormat.ARGB32);
        renderTextureAtlas.dimension = UnityEngine.Rendering.TextureDimension.Tex2DArray;
        renderTextureAtlas.volumeDepth = rigConfiguration.NumPods;
        renderTextureAtlas.enableRandomWrite = true;
		renderTextureAtlas.Create();

        if(LUTTableTexture == null)
            LUTTableTexture = new Texture3D(1, 1, rigConfiguration.NumPods, TextureFormat.RGBAFloat, false);

    }

    public unsafe void LoadAllColorFiles(byte[] colorData)
	{
        for(int i = 0; i < rigConfiguration.NumPods; ++i)
        {
            inputTextureArray.SetPixelData(colorData, 0, i, rigConfiguration.ColorWidth * rigConfiguration.ColorHeight * rigConfiguration.ColorPixelBytes * i);
        }
        inputTextureArray.Apply();
        InvokeComputeShader();
	}

	void LoadCalibrationDepthFiles()
	{
        // use the absolute path if the path starts with X:\, otherwise, use the relative path starting with DummyData
        string depthCalibrationPath = CalibFolderLocation.Trim();
        //i=if it's absolute path
        if (depthCalibrationPath.Contains(":\\"))
        {
            depthCalibrationPath = depthCalibrationPath + "\\" + cDepthCalibPrefixFileName;
        }
        else
        {
            depthCalibrationPath = Environment.CurrentDirectory + "\\" + LocalCalibDirectoryName + "\\" + depthCalibrationPath + "\\" + cDepthCalibPrefixFileName;
        }


        for (int fileNum = 0; fileNum < rigConfiguration.NumPods; fileNum++) 
		{
			string filePath = depthCalibrationPath + fileNum.ToString() + cDepthCalibSuffix;
		    Debug.Log("[ReadAndProcessRawImages][LoadCalibrationDepthFiles] Read from " + filePath + "\n");

            if (!File.Exists(filePath))
            {
                Debug.LogError("Critical Error: Missing Depth BA Calib file.");
                return;
            }
            string[] allLines = File.ReadAllLines (filePath);
			
			string[] lineSplit;
			Matrix4x4 cameraIntrinsics = new Matrix4x4();
			Matrix4x4 rotationMatrix = new Matrix4x4();
			Vector3 translation;
			float[] kappa_radial = new float[8];
			//calculated
			Matrix4x4 perProjectionMatrix = new Matrix4x4();  // perspective projection matrix
			Matrix4x4 projectionMatrix = new Matrix4x4();
			Matrix4x4 modelViewMatrix = Matrix4x4.zero;

            if (mCameraCalibrationArray[fileNum] == null)
                mCameraCalibrationArray[fileNum] = new CameraCalibration();

            lineSplit = allLines [0].Split (' ');
            int calibration_width = int.Parse(lineSplit[0]);
            int calibration_height = int.Parse(lineSplit[1]);
            float scale_w = rigConfiguration.ColorWidth / (float)calibration_width;
            float scale_h = rigConfiguration.ColorHeight / (float)calibration_height;

            mCameraCalibrationArray[fileNum].w = rigConfiguration.ColorWidth;
            mCameraCalibrationArray[fileNum].h = rigConfiguration.ColorHeight;

            lineSplit = allLines [1].Split (' ');
			cameraIntrinsics [0, 0] = float.Parse (lineSplit [0]) * scale_w;
			cameraIntrinsics [0, 1] = float.Parse (lineSplit [1]);
			cameraIntrinsics [0, 2] = float.Parse (lineSplit [2]) * scale_w;
			lineSplit = allLines [2].Split (' ');
			cameraIntrinsics [1, 0] = float.Parse (lineSplit [0]);
			cameraIntrinsics [1, 1] = float.Parse (lineSplit [1]) * scale_h;
			cameraIntrinsics [1, 2] = float.Parse (lineSplit [2]) * scale_h;
			lineSplit = allLines [3].Split (' ');
			cameraIntrinsics [2, 0] = float.Parse (lineSplit [0]);
			cameraIntrinsics [2, 1] = float.Parse (lineSplit [1]);
			cameraIntrinsics [2, 2] = float.Parse (lineSplit [2]);
			
			lineSplit = allLines [4].Split (' ');
			rotationMatrix [0, 0] = float.Parse (lineSplit [0]);
			rotationMatrix [0, 1] = float.Parse (lineSplit [1]);
			rotationMatrix [0, 2] = float.Parse (lineSplit [2]);
			lineSplit = allLines [5].Split (' ');
			rotationMatrix [1, 0] = float.Parse (lineSplit [0]);
			rotationMatrix [1, 1] = float.Parse (lineSplit [1]);
			rotationMatrix [1, 2] = float.Parse (lineSplit [2]);
			lineSplit = allLines [6].Split (' ');
			rotationMatrix [2, 0] = float.Parse (lineSplit [0]);
			rotationMatrix [2, 1] = float.Parse (lineSplit [1]);
			rotationMatrix [2, 2] = float.Parse (lineSplit [2]);
			
			translation = new Vector3 (float.Parse (allLines [7]),
			                           float.Parse (allLines [8]),
			                           float.Parse (allLines [9]))/100;

            // K = 5: default r3kt (3 radial, and 2 tangential distortion parameters)
            int numK = 5;
			kappa_radial [0] = float.Parse (allLines [10]);
			kappa_radial [1] = float.Parse (allLines [11]);
			kappa_radial [2] = float.Parse (allLines [12]);
			kappa_radial [3] = float.Parse (allLines [13]);
			kappa_radial [4] = float.Parse (allLines [14]);

            // K = 8: r6kt (6 radial, and 2 tangential distortion parameters)
            if (allLines.Length >= 18)
            {
                kappa_radial[5] = float.Parse(allLines[15]);
                kappa_radial[6] = float.Parse(allLines[16]);
                kappa_radial[7] = float.Parse(allLines[17]);
                numK = 8;
            }

            mCameraCalibrationArray[fileNum].fx = cameraIntrinsics[0, 0];
            mCameraCalibrationArray[fileNum].fy = cameraIntrinsics[1, 1];
            mCameraCalibrationArray[fileNum].px = cameraIntrinsics[0, 2];
            mCameraCalibrationArray[fileNum].py = cameraIntrinsics[1, 2];
            for (int i = 0; i < numK; ++i)
            {
                mCameraCalibrationArray[fileNum].kappa[i] = kappa_radial[i];
            }
            mCameraCalibrationArray[fileNum].isR6kt = numK == 8;
            mCameraCalibrationArray[fileNum].invFx = 1.0f / mCameraCalibrationArray[fileNum].fx;
            mCameraCalibrationArray[fileNum].invFy = 1.0f / mCameraCalibrationArray[fileNum].fy;
            mCameraCalibrationArray[fileNum].invPx = -mCameraCalibrationArray[fileNum].px / mCameraCalibrationArray[fileNum].fx;
            mCameraCalibrationArray[fileNum].invPy = -mCameraCalibrationArray[fileNum].py / mCameraCalibrationArray[fileNum].fy;
            //flip axis
            /*GameObject par = new GameObject();
			GameObject go = new GameObject();
			go.transform.parent = par.transform;
			go.transform.position = translation;
			go.transform.rotation = ProjectiveTextures.QuaternionFromMatrix(rotationMatrix);
			par.transform.RotateAround(-translation, go.transform.up, 0);
			rotationMatrix = Matrix4x4.TRS(Vector3.zero, go.transform.rotation, new Vector3(1,1,1));*/

            if (mCameraCalibrationArray[fileNum] != null)
			{
				/*mCameraCalibrationArray[fileNum].w = width;
				mCameraCalibrationArray[fileNum].h = height;*/
				mCameraCalibrationArray[fileNum].cameraIntrinsics = cameraIntrinsics;
				mCameraCalibrationArray[fileNum].rotationMatrix = rotationMatrix;
				mCameraCalibrationArray[fileNum].perProjectionMatrix = perProjectionMatrix;
				mCameraCalibrationArray[fileNum].projectionMatrix = projectionMatrix;
				mCameraCalibrationArray[fileNum].modelViewMatrix = modelViewMatrix;

				mCameraCalibrationArray[fileNum].translation = translation;
			}

			BuildProjectionMatrix(fileNum);
		}
	}

    void Load3DTMCalibrationFile()
    {
        // use the absolute path if the path starts with X:\, otherwise, use the relative path starting with DummyData
        string calibCameras3DTMPath = CalibFolderLocation.Trim();
        // Reading RGB calibration information from file
        string cam_type = "rgb";
        //i=if it's absolute path
        if (calibCameras3DTMPath.Contains(":\\"))
        {
            calibCameras3DTMPath = calibCameras3DTMPath + "\\" + c3DTMCalibFormatFilename;
        }
        else
        {
            calibCameras3DTMPath = Environment.CurrentDirectory + "\\" + c3DTMCalibFormatFilename;
        }

        if (!File.Exists(calibCameras3DTMPath))
        {
            Debug.LogError("Critical Error: Missing 3DTM calibration file.");
            return;
        }

        Debug.Log("[ReadAndProcessRawImages][Load3DTMCalibrationFile] Read from " + calibCameras3DTMPath + "\n");

        JObject root = JObject.Parse(File.ReadAllText(calibCameras3DTMPath));

        for (int cam_idx = 0; cam_idx < rigConfiguration.NumPods; cam_idx++)
        {
            Matrix4x4 cameraIntrinsics = new Matrix4x4();
            Matrix4x4 rotationMatrix = new Matrix4x4();
            Vector3 translation;
            float[] kappa_radial = new float[8];
            //calculated
            Matrix4x4 perProjectionMatrix = new Matrix4x4();  // perspective projection matrix
            Matrix4x4 projectionMatrix = new Matrix4x4();
            Matrix4x4 modelViewMatrix = Matrix4x4.zero;
            float scale_w = rigConfiguration.ColorWidth / root["inputCameras"][cam_idx][cam_type]["intrinsics"]["resolution"]["width"].Value<float>();
            float scale_h = rigConfiguration.ColorHeight / root["inputCameras"][cam_idx][cam_type]["intrinsics"]["resolution"]["height"].Value<float>();

            if (mCameraCalibrationArray[cam_idx] == null)
                mCameraCalibrationArray[cam_idx] = new CameraCalibration();

            mCameraCalibrationArray[cam_idx].w = rigConfiguration.ColorWidth;
            mCameraCalibrationArray[cam_idx].h = rigConfiguration.ColorHeight;

            cameraIntrinsics[0, 0] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["fx"].Value<float>() * scale_w;
            cameraIntrinsics[0, 1] = 0.0f;
            cameraIntrinsics[0, 2] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["cx"].Value<float>() * scale_w;
            cameraIntrinsics[1, 0] = 0.0f;
            cameraIntrinsics[1, 1] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["fy"].Value<float>() * scale_h;
            cameraIntrinsics[1, 2] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["cy"].Value<float>() * scale_h;
            cameraIntrinsics[2, 0] = 0.0f;
            cameraIntrinsics[2, 1] = 0.0f;
            cameraIntrinsics[2, 2] = 1.0f;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rotationMatrix[i, j] = root["inputCameras"][cam_idx][cam_type]["extrinsics"]["rotation"][i][j].Value<float>();
                }
            }

            translation = new Vector3(root["inputCameras"][cam_idx][cam_type]["extrinsics"]["translation"][0].Value<float>(),
                                       root["inputCameras"][cam_idx][cam_type]["extrinsics"]["translation"][1].Value<float>(),
                                       root["inputCameras"][cam_idx][cam_type]["extrinsics"]["translation"][2].Value<float>()) / 1000; // Converting to meters

            // K = 5: default r3kt (3 radial, and 2 tangential distortion parameters)
            int numK = 5;
            kappa_radial[0] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k1"].Value<float>();
            kappa_radial[1] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k2"].Value<float>();
            kappa_radial[2] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["p1"].Value<float>();
            kappa_radial[3] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["p2"].Value<float>();
            kappa_radial[4] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k3"].Value<float>();

            // K = 8: r6kt (6 radial, and 2 tangential distortion parameters)
            kappa_radial[5] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k4"].Value<float>();
            kappa_radial[6] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k5"].Value<float>();
            kappa_radial[7] = root["inputCameras"][cam_idx][cam_type]["intrinsics"]["k6"].Value<float>();
            numK = 8;

            mCameraCalibrationArray[cam_idx].fx = cameraIntrinsics[0, 0];
            mCameraCalibrationArray[cam_idx].fy = cameraIntrinsics[1, 1];
            mCameraCalibrationArray[cam_idx].px = cameraIntrinsics[0, 2];
            mCameraCalibrationArray[cam_idx].py = cameraIntrinsics[1, 2];
            for (int i = 0; i < numK; ++i)
            {
                mCameraCalibrationArray[cam_idx].kappa[i] = kappa_radial[i];
            }
            mCameraCalibrationArray[cam_idx].isR6kt = numK == 8;
            mCameraCalibrationArray[cam_idx].invFx = 1.0f / mCameraCalibrationArray[cam_idx].fx;
            mCameraCalibrationArray[cam_idx].invFy = 1.0f / mCameraCalibrationArray[cam_idx].fy;
            mCameraCalibrationArray[cam_idx].invPx = -mCameraCalibrationArray[cam_idx].px / mCameraCalibrationArray[cam_idx].fx;
            mCameraCalibrationArray[cam_idx].invPy = -mCameraCalibrationArray[cam_idx].py / mCameraCalibrationArray[cam_idx].fy;

            // Copying color calibration information
            mCameraCalibrationArray[cam_idx].colorScale = new Vector3(root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorScale"][0].Value<float>(),
                                                                        root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorScale"][1].Value<float>(),
                                                                        root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorScale"][2].Value<float>());
            mCameraCalibrationArray[cam_idx].colorBias = new Vector3(root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorBias"][0].Value<float>(),
                                                                        root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorBias"][1].Value<float>(),
                                                                        root["inputCameras"][cam_idx][cam_type]["colorCalibration"]["colorBias"][2].Value<float>());
            if (mCameraCalibrationArray[cam_idx] != null)
            {
                mCameraCalibrationArray[cam_idx].cameraIntrinsics = cameraIntrinsics;
                mCameraCalibrationArray[cam_idx].rotationMatrix = rotationMatrix;
                mCameraCalibrationArray[cam_idx].perProjectionMatrix = perProjectionMatrix;
                mCameraCalibrationArray[cam_idx].projectionMatrix = projectionMatrix;
                mCameraCalibrationArray[cam_idx].modelViewMatrix = modelViewMatrix;

                mCameraCalibrationArray[cam_idx].translation = translation;
            }

            BuildProjectionMatrix(cam_idx);
        }
    }

    public void BuildProjectionMatrix(int index )
	{
		Matrix4x4 perProjectionMatrix = mCameraCalibrationArray [index].perProjectionMatrix;
		Matrix4x4 rotationMatrix = mCameraCalibrationArray [index].rotationMatrix;
		Matrix4x4 cameraIntrinsics = mCameraCalibrationArray [index].cameraIntrinsics;
		Vector3 translation = mCameraCalibrationArray [index].translation;
        //int width = mCameraCalibrationArray[index].w;
        //int height = mCameraCalibrationArray[index].h;
        Matrix4x4 modelViewMatrix = mCameraCalibrationArray [index].modelViewMatrix;

        // model view matrix, row i, column j
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                modelViewMatrix[i, j] = rotationMatrix[i, j];
            }
        }
        for (int i = 0; i < 3; i++)
        {
            modelViewMatrix[i, 3] = translation[i];
        }
        modelViewMatrix [3, 3] = 1.0f;


        // projMat = K * [R | T]
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                perProjectionMatrix[i,j] = cameraIntrinsics[i,0]*rotationMatrix[0,j] + cameraIntrinsics[i,1]*rotationMatrix[1,j] + cameraIntrinsics[i,2]*rotationMatrix[2,j];
				//this->projMat[i][j] = K[i][0]*R[0][j] + K[i][1]*R[1][j] + K[i][2]*R[2][j];
			}
			//this->projMat[i][3] = K[i][0]*T[0] + K[i][1]*T[1] + K[i][2]*T[2];
			perProjectionMatrix[i,3] = cameraIntrinsics[i,0]*translation.x+ cameraIntrinsics[i,1]*translation.y + cameraIntrinsics[i,2]*translation.z;
		}
		
		float n = 0.1f;
		float f = 2000.0f;
		Matrix4x4 viewport_proj_mat = new Matrix4x4();
		Matrix4x4 viewport_mat = Matrix4x4.zero;
		Matrix4x4 viewport_mat_inv = new Matrix4x4();
		
		viewport_proj_mat[0, 0] = cameraIntrinsics[0, 0];
		viewport_proj_mat[0, 1] = cameraIntrinsics[0, 1];
		viewport_proj_mat[0, 2] = cameraIntrinsics[0, 2];
		viewport_proj_mat[0, 3] = 0.0f;
		viewport_proj_mat[1, 0] = cameraIntrinsics[1, 0];
		viewport_proj_mat[1, 1] = cameraIntrinsics[1, 1];
		viewport_proj_mat[1, 2] = cameraIntrinsics[1, 2];
		viewport_proj_mat[1, 3] = 0.0f;
		viewport_proj_mat[2, 0] = 0.0f;
		viewport_proj_mat[2, 1] = 0.0f;
		viewport_proj_mat[2, 2] = (f+n)/(f-n);
		viewport_proj_mat[2, 3] = -2*f*n/(f-n);
		viewport_proj_mat[3, 0] = 0.0f;
		viewport_proj_mat[3, 1] = 0.0f;
		viewport_proj_mat[3, 2] = 1.0f;
		viewport_proj_mat[3, 3] = 0.0f;
		
		//	printf("=======viewport_proj_mat Matrix=====\n");
		//	printMat(viewport_proj_mat);
		viewport_mat[0, 0]=  rigConfiguration.ColorWidth/2.0f; // TODO:PEABODY
		viewport_mat[0, 3]= (rigConfiguration.ColorWidth - 1)/2.0f;
		viewport_mat[1, 1]=  rigConfiguration.ColorHeight /2.0f;
		viewport_mat[1, 3]= (rigConfiguration.ColorHeight - 1)/2.0f;
		viewport_mat[2, 2]= 1.0f;
		viewport_mat[3, 3]= 1.0f;
		
		//	printf("=======viewport Matrix=====\n");
		//	printMat(viewport_mat);
		
		viewport_mat_inv = viewport_mat.inverse;
		//	printf("=======viewport Matrix Invert=====\n");
		//	printMat(viewport_mat_inv);

		Vector4 projectiveTextureCenter = Vector4.zero;
		//c[i] = -( this->R[0][i]*this->T[0] + this->R[1][i]*this->T[1] + this->R[2][i]*this->T[2]);
		projectiveTextureCenter.x =  -(rotationMatrix[0,0]*translation.x + rotationMatrix[1,0]*translation.y + rotationMatrix[2,0]*translation.z);
		projectiveTextureCenter.y =  -(rotationMatrix[0,1]*translation.x + rotationMatrix[1,1]*translation.y + rotationMatrix[2,1]*translation.z);
		projectiveTextureCenter.z =  -(rotationMatrix[0,2]*translation.x + rotationMatrix[1,2]*translation.y + rotationMatrix[2,2]*translation.z);
		//projectiveTextureCenter = modelViewMatrix.inverse * (new Vector4 (0, 0, 0, 1));

		Matrix4x4 scale = Matrix4x4.Scale(new Vector3(1,1,1));
		Matrix4x4 inverse_y = Matrix4x4.identity;
		inverse_y.m11 = -1.0f;
        mCameraCalibrationArray[index].projectionMatrix = inverse_y * viewport_mat_inv * viewport_proj_mat * scale;
		mCameraCalibrationArray [index].modelViewMatrix = modelViewMatrix * scale;
		mCameraCalibrationArray [index].projectiveTextureCenter = projectiveTextureCenter;
	}


	void LoadCalibrationFile()
	{
        string generalCalibrationPath = CalibFolderLocation.Trim();
        // use the absolute path if the path starts with X:\, otherwise, use the relative path starting with DummyData
        if (generalCalibrationPath.Contains(":\\"))
        {
            generalCalibrationPath = generalCalibrationPath + "\\"+  cColorCalibFileName;
        }
        else
        {
            generalCalibrationPath = Environment.CurrentDirectory + "\\" + LocalCalibDirectoryName + "\\" + generalCalibrationPath + "\\" + cColorCalibFileName;
        }

        if(!File.Exists(generalCalibrationPath))
        {
            Debug.LogWarning("Missing Color file. Ok, but no color bias or scale applied!");
            for(int i = 0; i < rigConfiguration.NumPods; ++i )
            {
                if (mCameraCalibrationArray[i] == null)
                {
                    mCameraCalibrationArray[i] = new CameraCalibration();
                }
                mCameraCalibrationArray[i].colorScale = new Vector3(1, 1, 1);
                mCameraCalibrationArray[i].colorBias = new Vector3(0, 0, 0) / 255.0f;
                
            }
            

            return;
        }
#if UNITY_WSA
        string[] allLines = File.ReadAllLines(generalCalibrationPath);

#else
        string[] allLines = File.ReadAllLines(generalCalibrationPath);
#endif
        Debug.Log($"[ReadAndProcessRawImages][LoadCalibrationFile] Read files from {generalCalibrationPath}. {allLines.Length} lines\n");
        //string[] firstLine = allLines[0].Split(' ');
        //int NumCams = int.Parse(firstLine [0]);
        int camIndex = 0;

		for (int i = 1; i < allLines.Length; ) {
            string[] lineSplit = allLines[i].Split(' ');
            if(SettingsManager.Instance.Verbosity() > 0)
            {
                Debug.Log($"[ReadAndProcessRawImages][LoadCalibrationFile] Reading color data for camera {camIndex}");
            }
            mCameraCalibrationArray[camIndex].name = lineSplit[0];
            //only used to load color bias and scale now
            lineSplit = allLines[i+3].Split(' ');
            mCameraCalibrationArray[camIndex].colorScale = new Vector3(float.Parse(lineSplit[0]), float.Parse(lineSplit[1]), float.Parse(lineSplit[2]));
            mCameraCalibrationArray[camIndex].colorBias = new Vector3(float.Parse(lineSplit[3]), float.Parse(lineSplit[4]), float.Parse(lineSplit[5]));

            camIndex++;
            i = i + 4;

            if(camIndex >= mCameraCalibrationArray.Length && i < allLines.Length)
            {
                // config file has more cameras than I need, skip the rest
                Debug.Log("WARNING:  calibColorCams.txt contains more cameras than the config file asked me to render.  Skipping remaining configurations");
                break;
            }
        }
	}

    /// <summary>
    /// Read the calibration files and update the camera parameters in the compute shader.
    /// </summary>
	void updateAutomaticCalibration()
	{
        // allocating memory for dataArray, textureAtlas, and shaders
        if (firstTimeCalibration) 
        {
            cameraParametersArray = new CameraParameters[rigConfiguration.NumPods];
        }
        Load3DTMCalibrationFile();

        for (int i = 0; i < rigConfiguration.NumPods; i++)
		{
			//vector4 arrangement? zwxy  yxwz  1032
			CameraParameters currCamParam = new CameraParameters();
			currCamParam.g_F = new Vector4(mCameraCalibrationArray[i].fx, mCameraCalibrationArray[i].fy, mCameraCalibrationArray[i].invFx, mCameraCalibrationArray[i].invFy);
            
			currCamParam.g_P = new Vector4(mCameraCalibrationArray[i].px, mCameraCalibrationArray[i].py, mCameraCalibrationArray[i].invPx, mCameraCalibrationArray[i].invPy);

			currCamParam.g_Kappa = new Vector4(mCameraCalibrationArray[i].kappa[0], mCameraCalibrationArray[i].kappa[1],  mCameraCalibrationArray[i].kappa[4], mCameraCalibrationArray[i].kappa[5]);

            // kappa[6] and kappa[7] are radial parameters --> g_Tangential[2], [3]
            currCamParam.g_Tangential = new Vector4(mCameraCalibrationArray[i].kappa[2], mCameraCalibrationArray[i].kappa[3], mCameraCalibrationArray[i].kappa[6], mCameraCalibrationArray[i].kappa[7]);

            currCamParam.g_Scale = new Vector4 (mCameraCalibrationArray [i].colorScale.x, mCameraCalibrationArray [i].colorScale.y, mCameraCalibrationArray [i].colorScale.z, 0);
			currCamParam.g_Bias  = new Vector4 (mCameraCalibrationArray [i].colorBias.x, mCameraCalibrationArray [i].colorBias.y, mCameraCalibrationArray [i].colorBias.z, 0);

            currCamParam.g_IsR6kt = (mCameraCalibrationArray[i].isR6kt && !Force3kt) ? 1 : 0;
			cameraParametersArray[i] = currCamParam;
		}
        if (firstTimeCalibration)
            cameraParameterBuffer = new ComputeBuffer(cameraParametersArray.Length, 100);
        cameraParameterBuffer.SetData (cameraParametersArray);
		processRawImageShader.SetBuffer(processRawImageKernelIndex, "cameraParameterBuffer", cameraParameterBuffer);
		firstTimeCalibration = false;

        //Load LUT files
        LoadLUTFiles();
    }
    void OnEnable()
    {
        InitializeResources();
    }

    void OnDisable()
    {
        ReleaseResources();
    }

    void ReleaseResources()
    {
        if (EnableAutoCalibration)
            CancelInvoke("updateAutomaticCalibration");
        if (cameraParameterBuffer != null)
        {
            cameraParameterBuffer.Release();
            cameraParameterBuffer = null;
        }
        if (sharpMaskBuffer != null)
        {
            sharpMaskBuffer.Release();
            sharpMaskBuffer = null;
        }
        Debug.Log("[ReadAndProcessRawImages] Successfully exit.\n");
    }
}
