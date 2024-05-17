using UnityEngine;
using System.Collections;

public class ShadowScript : MonoBehaviour {

    public Light directionalLight;
    Camera lightcamera;
    public RenderTexture shadowMap;

    Camera mainCamera;
    Camera mainCameraDoppel;
    private Material shaderMaterial;
    private Material blurShaderMaterial;
    Matrix4x4 bias = Matrix4x4.zero;

    public int shadowRes = 2048;
    public bool hologramReceiveShadow = false; 

    [Range(0, 1)] public float ShadowIntensity = 0.3f;
    [Range(-0.1f, 0.1f)] public float ShadowBias = 0.003f;

    RenderTexture createMaskTexture( int width, int height )
    {
        RenderTexture res = new RenderTexture(width, height, 0, RenderTextureFormat.Default);
        res.enableRandomWrite = true;
        return res;
    }

    RenderTexture createDepthTexture( int width, int height )
    {
        RenderTexture res = new RenderTexture(width, height, 24, RenderTextureFormat.Depth);
        res.filterMode = FilterMode.Point;
        return res;
    }

    void enableShadowObjects()
    {
        mainCameraDoppel.enabled = enabled;
        lightcamera.enabled = enabled;
        GameObject shadowGroundPlane = GameObject.Find("TransparentShadowReceiverPlane");
        if (shadowGroundPlane != null)
            shadowGroundPlane.GetComponent<MeshRenderer>().enabled = enabled;

    }    

    public RenderTexture shadowMask;
	// Use this for initialization
	void Awake(){
        int screenWidth = Screen.width;
        int screenHeight = Screen.height;

        lightcamera = directionalLight.gameObject.AddComponent<Camera>();
        shadowMask = createMaskTexture(screenWidth, screenHeight);
        shadowMap = createDepthTexture(shadowRes, shadowRes);
        lightcamera.renderingPath = RenderingPath.Forward;
        lightcamera.targetTexture = shadowMap;
        lightcamera.clearFlags = CameraClearFlags.Depth;
        // TODO: only one renderer (e.g., need to distinguish between local and remote)
        lightcamera.cullingMask = (1 << LayerMask.NameToLayer("HoloPort Renderer")) | (1 << LayerMask.NameToLayer("HoloPort Renderer (1)"));

        
        //mainCameraDoppel = new Camera();
        mainCamera = Camera.main;

        GameObject doppelCameraGO = new GameObject("cameraDopple");
        doppelCameraGO.transform.position = mainCamera.transform.position;
        doppelCameraGO.transform.rotation = mainCamera.transform.rotation;
        doppelCameraGO.transform.parent = mainCamera.transform;
        mainCameraDoppel = doppelCameraGO.AddComponent<Camera>();
        mainCameraDoppel.renderingPath = RenderingPath.Forward;
        mainCameraDoppel.clearFlags = CameraClearFlags.Depth;
        mainCameraDoppel.targetTexture = createDepthTexture(screenWidth, screenHeight);
        mainCameraDoppel.cullingMask = (1 << LayerMask.NameToLayer("HoloPort Renderer")) | (1 << LayerMask.NameToLayer("HoloPort Renderer (1)")) | (1 << LayerMask.NameToLayer("CameraDoppelDepth"));

        enableShadowObjects();

        // TODO: deal with SSAO
        //if (GetComponent<ScreenSpaceAmbientObscurance>() != null)
        //   mainCameraDoppel.enabled |=GetComponent<ScreenSpaceAmbientObscurance>().enabled;

        bias.m00 = 0.5f;
        bias.m11 = 0.5f;
        bias.m22 = 0.5f;
        bias.m30 = 0.5f;
        bias.m31 = 0.5f;
        bias.m32 = 0.5f;
        bias.m33 = 1.0f;

        /*	bias.m00 = 1f;
            bias.m11 = 1f;
            bias.m22 = 1;
            bias.m30 = 1f;
            bias.m31 = 1f;
            bias.m32 = 0;
            bias.m33 = 1.0f;*/
        bias = bias.transpose;

        shaderMaterial = new Material(Shader.Find("Custom/ShadowShader"));
        blurShaderMaterial = new Material(Shader.Find("Custom/BlurShadowShader"));
        mainCamera.depthTextureMode = DepthTextureMode.DepthNormals;
	}


    void OnEnable()
    {
        enableShadowObjects();
    }    

    void OnDisable()
    {
        enableShadowObjects();
    }        
	
	// Update is called once per frame
	void Update () {


        if (shadowMask.width != Screen.width || shadowMask.height != Screen.height)
        {
            Debug.Log("===============Screen Texture Resize===============");
            shadowMask = createMaskTexture(Screen.width, Screen.height);
            mainCameraDoppel.targetTexture = createDepthTexture(Screen.width, Screen.height);
        }

        if (lightcamera.targetTexture.width != shadowRes)
        {
            Debug.Log("===============Shadow Map Resize===============");
            lightcamera.targetTexture = createDepthTexture(shadowRes, shadowRes);
        }

        shaderMaterial.SetTexture("_ShadowTex", shadowMap);
        shaderMaterial.SetTexture("_MainCamShadowTex", mainCameraDoppel.targetTexture);
        shaderMaterial.SetMatrix("_ShadowMatrix", bias * lightcamera.projectionMatrix * lightcamera.worldToCameraMatrix);
        shaderMaterial.SetMatrix("_MainTexToWorld", (bias * mainCamera.projectionMatrix * mainCamera.worldToCameraMatrix).inverse);
        blurShaderMaterial.SetMatrix("_MainTexToView", (bias * mainCamera.projectionMatrix).inverse);
        shaderMaterial.SetFloat("_ShadowFactor", ShadowIntensity);
        shaderMaterial.SetFloat("_ShadowBias", ShadowBias);
        shaderMaterial.SetInt("hologramReceiveShadow", hologramReceiveShadow ? 1 : 0);
        
	}


    void OnRenderImage(RenderTexture src, RenderTexture dest)
    {
       // Graphics.Blit(src, shadowMask, shaderMaterial);


        Graphics.Blit(src, shadowMask, shaderMaterial);
        blurShaderMaterial.SetTexture("_ShadowMask", shadowMask);
        blurShaderMaterial.SetTexture("_ScreenDepth", mainCameraDoppel.targetTexture);
        blurShaderMaterial.SetFloat("_ScreenWidth", Screen.width);
        blurShaderMaterial.SetFloat("_ScreenHeight", Screen.height);
        Graphics.Blit(src, dest, blurShaderMaterial);
        
       // Graphics.Blit(shadowMask, dest, blurShaderMaterial);
    }
}
