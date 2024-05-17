using UnityEngine;
using System.Collections;
using System;

[RequireComponent(typeof(HoloPortScript))]
public class ProjectiveTextures : MonoBehaviour {
	public bool useRenderTexture = true;
	public bool useMockSetup = false;
	public ReadAndProcessRawImage imageProcessScript;
    public Transform ModelTransform;
	GameObject[] projectiveTextureCameraGameObjects;
	public Camera[] projectiveTextureCameras;
	Material[] projectiveMaterial;
	GameObject target = null;
	HoloPortScript holoport;
    Configuration rigConfiguration;

	public RenderTexture atlasDepthMap;
    public RenderTexture[] cameraDepthMaps; 
	//RenderTexture[] cameraDepthMaps;
	Matrix4x4 bias = Matrix4x4.zero;
    private Vector4[] m_camCenters;
    public Matrix4x4[] textureMatrix;
    public Matrix4x4[] invTextureMatrix;

    [Header("DepthDilate")]
    DepthDilateProfile depthDilateProfile;
    public ComputeShader depthDilate;
    public RenderTexture processedDepthMap;
    public int depthDilateKernelIndex;
    ComputeBuffer stepSizeBuffer;

    public int depthScaleReduction = 1;
    // Use this for initialization


    void Start() {

    

        holoport = GetComponent<HoloPortScript>();
        rigConfiguration = holoport.RigConfiguration;
        m_camCenters = new Vector4[rigConfiguration.NumPods]; // TODO, did M4D requries camcenters?
        textureMatrix = new Matrix4x4[rigConfiguration.NumPods];
        invTextureMatrix = new Matrix4x4[rigConfiguration.NumPods];

		projectiveMaterial = new Material[holoport.HoloPortMaterial.Length];
        for (int i = 0; i < holoport.HoloPortMaterial.Length; ++i)
        {
            projectiveMaterial[i] = holoport.HoloPortMaterial[i];
        }

        bias.m00 = 0.5f;
        bias.m11 = 0.5f;
        bias.m22 = 1.0f;
        bias.m30 = 0.5f;
        bias.m31 = 0.5f;
        bias.m32 = 0.0f;
        bias.m33 = 1.0f;

        bias = bias.transpose;
		SetupCameras();

        //SetupRenderTestCubesToDebugTextureAtlas();
        if (depthDilateProfile == null)
        {            
            depthDilateProfile = holoport.depthDilateProfile;
        }
        if (depthDilateProfile.DilateStepSizes == null )
        {
            Debug.LogWarning("Missing depth dilation step settings! Defaulting back to no depth dilation");
            depthDilateProfile.DilateSteps = 0;
            depthDilateProfile.DilateStepSizes = new int[rigConfiguration.NumPods];
            for(int i = 0; i < rigConfiguration.NumPods; ++ i)
            {
                depthDilateProfile.DilateStepSizes[i] = 0;
            }
        }

        depthDilateKernelIndex = depthDilate.FindKernel("DepthDilationKernel");
        stepSizeBuffer = new ComputeBuffer(rigConfiguration.NumPods, sizeof(int));
    }

    void SetupRenderTestCubesToDebugTextureAtlas()
	{
        for (int i = 0; i < rigConfiguration.NumPods; i++)
		{
			GameObject currCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
			currCube.transform.position = new Vector3(i, 0, 0);
			MeshRenderer mr = currCube.GetComponent<MeshRenderer>();
			//mr.material.mainTexture = imageProcessScript.renderTextureArray[i];
			//mr.material.mainTexture = imageProcessScript.renderTextureAtlas;
            mr.material.mainTexture = atlasDepthMap;
        }
	}

	void SetupCameras()
	{
        projectiveTextureCameraGameObjects = new GameObject[rigConfiguration.NumPods];
        projectiveTextureCameras = new Camera[rigConfiguration.NumPods];
        // the depth map is 4096 * 2048 pixels
		atlasDepthMap = new RenderTexture(rigConfiguration.ColorWidth * 4 / depthScaleReduction, rigConfiguration.ColorHeight * 2 / depthScaleReduction, 24, RenderTextureFormat.Depth); // TODO: think about number of cameras & depth resolution
		atlasDepthMap.filterMode = FilterMode.Point;
		atlasDepthMap.antiAliasing = 1;

        processedDepthMap = new RenderTexture(rigConfiguration.ColorWidth * 4 / depthScaleReduction, rigConfiguration.ColorHeight * 2/ depthScaleReduction, 24, RenderTextureFormat.RFloat); // TODO: think about number of cameras & depth resolution
        processedDepthMap.filterMode = FilterMode.Point;
        processedDepthMap.antiAliasing = 1;
        processedDepthMap.enableRandomWrite = true;
        processedDepthMap.Create();

        //  cameraDepthMaps = new RenderTexture[rigConfiguration.NumPods];

        int[] xOffset = { -1, 0, 1, 1, 1,  0, -1, -1 };
        int[] zOffset = {  1, 1, 1, 0, -1, -1, -1, 0 };
		float scale = 20.0f;
        for (int i = 0; i < rigConfiguration.NumPods; i++) 
		{
            //cameraDepthMaps[i] = new RenderTexture(rigConfiguration.ColorWidth, rigConfiguration.ColorHeight, 24, RenderTextureFormat.Depth);
            projectiveTextureCameraGameObjects[i] = new GameObject("ProjectiveCam " + i);
            projectiveTextureCameraGameObjects[i].transform.parent = gameObject.transform;
			projectiveTextureCameras[i] = projectiveTextureCameraGameObjects[i].AddComponent<Camera>();
			projectiveTextureCameras[i].aspect = 1;
            projectiveTextureCameras[i].enabled = false;
            projectiveTextureCameras[i].renderingPath = RenderingPath.VertexLit;
            projectiveTextureCameras[i].useOcclusionCulling = false;
            projectiveTextureCameras[i].allowMSAA = false;

            if (useMockSetup) {
				projectiveTextureCameras[i].transform.position = scale * new Vector3(xOffset[i], 0, zOffset[i]);
				projectiveTextureCameras[i].transform.LookAt(target.transform.position);
			}
			else
			{
                
                projectiveTextureCameras[i].transform.position = (Matrix4x4.Transpose(imageProcessScript.mCameraCalibrationArray[i].rotationMatrix) * -imageProcessScript.mCameraCalibrationArray[i].translation);

                // projectiveTextureCameras[i].transform.rotation = QuaternionFromMatrix(Matrix4x4.Transpose(imageProcessScript.mCameraCalibrationArray[i].rotationMatrix));



                projectiveTextureCameras[i].projectionMatrix = imageProcessScript.mCameraCalibrationArray[i].projectionMatrix;



                Matrix4x4 test = (Matrix4x4.TRS(projectiveTextureCameras[i].transform.position, projectiveTextureCameras[i].transform.rotation, Vector3.one));

                test = test.inverse; // localtoworld -> worldtolocal = worldtoCamera



                projectiveTextureCameras[i].worldToCameraMatrix = test * ModelTransform.localToWorldMatrix;

                projectiveTextureCameras[i].cullingMask = 1 << LayerMask.NameToLayer(holoport.name);

                projectiveTextureCameras[i].clearFlags = CameraClearFlags.Nothing;

                projectiveTextureCameras[i].transform.position = ModelTransform.localToWorldMatrix * (Matrix4x4.Transpose(imageProcessScript.mCameraCalibrationArray[i].rotationMatrix) * -imageProcessScript.mCameraCalibrationArray[i].translation);

             //   projectiveTextureCameras[i].renderingPath = RenderingPath.DeferredShading; // FIX so Vive frame grab doesnt mess with our Z-buffers
            }
            projectiveTextureCameras[i].depthTextureMode = DepthTextureMode.Depth;
            projectiveTextureCameras[i].targetTexture = atlasDepthMap;
        }

        // Montage4D refraction begins
        // first, lay out the top 4 camera on the first row, and the rest on the bottom row
        for (int i = 0; i < rigConfiguration.NumPods; ++i)
        {
            projectiveTextureCameras[i].rect = new Rect(0.25f * (i % 4), 0.5f * ((i < 4) ? 1 : -1), 0.25f, 1f);
        }

        if (useRenderTexture)
        {
            for (int i = 0; i < projectiveMaterial.Length; ++i)
                projectiveMaterial[i].SetTexture("_CamImg0", imageProcessScript.renderTextureAtlas);

            for (int i = 0; i < projectiveMaterial.Length; ++i)
            {
                projectiveMaterial[i].SetTexture("_DepthMap0", processedDepthMap);
                //for (int c = 0; c < rigConfiguration.NumPods; ++c)
                //{
                //    projectiveMaterial[i].SetTexture("_DepthMapArr[" + c + "]", cameraDepthMaps[c]);
                //}
            }
        }
        Debug.Log("[Montage4D] Projective textures and matrices are successfully set.\n");
    }

    public void RenderAllProjectiveCameras()
    {
        RenderTexture temp = RenderTexture.active;
        RenderTexture.active = atlasDepthMap;
        GL.Clear(true, false, Color.green);
        
        for (int i = 0; i < rigConfiguration.NumPods; ++i)
        {
            projectiveTextureCameras[i].Render();
        }
        RenderTexture.active = temp;
        InvokeDepthDilation();
    }

    void InvokeDepthDilation()
    {
        if (depthDilateKernelIndex >= 0)
        {
            stepSizeBuffer.SetData(depthDilateProfile.DilateStepSizes);
            depthDilate.SetBuffer(depthDilateKernelIndex, "stepSizes", stepSizeBuffer);
            depthDilate.SetTexture(depthDilateKernelIndex, "inputDepth", atlasDepthMap);
            depthDilate.SetTexture(depthDilateKernelIndex, "Result", processedDepthMap);
            depthDilate.SetInt("steps", depthDilateProfile.DilateSteps);
            depthDilate.SetInt("resolutionWidth", rigConfiguration.ColorWidth / depthScaleReduction);
            depthDilate.SetInt("resolutionHeight", rigConfiguration.ColorHeight / depthScaleReduction);
            depthDilate.Dispatch(depthDilateKernelIndex, (rigConfiguration.ColorWidth / 32 / depthScaleReduction + 1) * 4, (rigConfiguration.ColorHeight / 32/ depthScaleReduction + 1) * 2, 1);  //???? 16 or not 16 1024/16 or 2048/16? //TODO: double check correct dispatch size for GPU cards
        }
    }

    // Update is called once per frame
    void Update ()
    {
        // Set projectiveTextureCameras, projectiveMaterials, textureMatrix, invTextureMatrix
        for (int i = 0; i < rigConfiguration.NumPods; ++i)
        {
            projectiveTextureCameras[i].worldToCameraMatrix = imageProcessScript.mCameraCalibrationArray[i].modelViewMatrix * ModelTransform.localToWorldMatrix.inverse;
            m_camCenters[i] = imageProcessScript.mCameraCalibrationArray[i].projectiveTextureCenter;
            textureMatrix[i] = bias * projectiveTextureCameras[i].projectionMatrix * projectiveTextureCameras[i].worldToCameraMatrix;
            invTextureMatrix[i] = textureMatrix[i].inverse;
        }

        for (int i = 0; i < projectiveMaterial.Length; ++i)
        {
            projectiveMaterial[i].SetVectorArray("_texCamCenters", m_camCenters);
            projectiveMaterial[i].SetMatrixArray("gl_TextureMatrices", textureMatrix);
            projectiveMaterial[i].SetMatrixArray("gl_TextureInvMatrices", invTextureMatrix);
        }
    }

    private void OnDestroy()
    {
        if (stepSizeBuffer != null) {
            stepSizeBuffer.Release();
            stepSizeBuffer = null;
        }
    }

    public static Quaternion QuaternionFromMatrix(Matrix4x4 m) {
		// Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
		Quaternion q = new Quaternion();
		q.w = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0,0] + m[1,1] + m[2,2] ) ) / 2; 
		q.x = Mathf.Sqrt( Mathf.Max( 0, 1 + m[0,0] - m[1,1] - m[2,2] ) ) / 2; 
		q.y = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0,0] + m[1,1] - m[2,2] ) ) / 2; 
		q.z = Mathf.Sqrt( Mathf.Max( 0, 1 - m[0,0] - m[1,1] + m[2,2] ) ) / 2; 
		q.x *= Mathf.Sign( q.x * ( m[2,1] - m[1,2] ) );
		q.y *= Mathf.Sign( q.y * ( m[0,2] - m[2,0] ) );
		q.z *= Mathf.Sign( q.z * ( m[1,0] - m[0,1] ) );
		return q;
	}
}
