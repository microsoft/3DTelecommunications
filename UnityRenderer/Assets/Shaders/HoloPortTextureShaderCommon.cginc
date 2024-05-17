// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'

#define OPT_UNROLL unroll // for release
//#define OPT_UNROLL loop // for quicker compilation and debugging          


float voxel_size;
float voxel_no_skip_size;
#define VOXEL_SNAP_IN_VERTEX_SHADER 1

// Textures 
UNITY_DECLARE_TEX2DARRAY(_CamImg0); 
sampler2D _DepthMap0;
sampler2D _Gradient;
sampler2D _DepthMapArr[NUM_PODS];      

int _NumPods;  
int _UsePointCloud; 

// Montage4D Requires
uniform float4x4 gl_TextureMatrices[NUM_PODS]; 
uniform float4x4 gl_TextureInvMatrices[NUM_PODS];
float4 _texCamCenters[NUM_PODS];
float TemporalTextureFields[NUM_PODS]; 
float UseCameraTexture[NUM_PODS];
float GlobalAlphaMultiplier;
float _ColorWeightThreshold;
float _DiscontinuityThreshold;
float _DotProductNormalWeightThreshold;
uniform StructuredBuffer<int> _DepthSearchStep;
uniform StructuredBuffer<int> _DepthSearchRadius;
// buffers  
uniform StructuredBuffer<int> verticesHalf;
// removed for performance issues 
//uniform StructuredBuffer<float> verticesWeights;
//uniform StructuredBuffer<float> verticesTexWeights;
uniform StructuredBuffer<float> vtGeodesicWeights;
//uniform StructuredBuffer<int> verticesGeodesic;
uniform StructuredBuffer<int> indices;

uniform StructuredBuffer<float> textureUVs; 
//uniform StructuredBuffer<int> colorImages;
//uniform StructuredBuffer<int> visibilities;
float NormalBaseWeight;
float NormalWeightsThreshold;
float NormalWeightedBlendingOpacity;
float ViewDependentTTWStep;
// Montage4D Debug
int		debugColorLabeledFields;
int		debugTextureWeights; 
int		debugSingleView;
int		debugDiscontinuity;
int		debugNormalWeightedField;
int		debugGeodesicFields;
int		debugSeams;
int		debugNormalWeightedBlendingColor; 
int		debugC0;
int		debugTemp;
bool	RemoveBackgroundPixels;
// Montage4D Requires End

float4x4 modelTransform;
float4x4 inverseModelTransform;
int indexCount;

int screenWidth;
int screenHeight;

float3 cullingBbox1Center;
float3 cullingBbox1HalfSize;

float3 cullingBbox2Center;
float3 cullingBbox2HalfSize;
//normal effect parameters
int colorImageWidth;
float _NormalWeight;
float _NormalBand;

//holographic effect parameters
fixed4 _HighlightColor;
half _RimPower;
fixed _RimVis;
fixed _ModelOpacity; 
fixed4 _HologramTint;

uniform fixed4     _SpecColor;
uniform float     _Shininess;
uniform float	ExplosionAmount;
uniform float _MaxExplosionAmount;
uniform fixed4     _LightColor0;

uniform float Weight_Color;
uniform float Weight_Normal;
uniform float Weight_ShinyGeometry;
uniform float Weight_HolographicEffect;
uniform float Weight_Occlusion;
uniform float Weight_Wireframe;
uniform float Weight_Montage4D;
uniform float Weight_Intrinsic4D;
uniform float Weight_Art;
uniform float ApplyLightingToNormalMap;

# include "Utilities.cginc"  
# include "HoloPortVertexShader.cginc"
# include "HoloPortGeometryShader.cginc"
# include "RenderingAlgorithms/RenderColor.cginc"
# include "RenderingAlgorithms/RenderExplosion.cginc" 
# include "RenderingAlgorithms/RenderHolographicEffect.cginc"
# include "RenderingAlgorithms/RenderNormals.cginc"
# include "RenderingAlgorithms/RenderShinyGeometry.cginc"
# include "RenderingAlgorithms/RenderWireframes.cginc"
# include "RenderingAlgorithms/RenderMontage4D.cginc"
//# include "RenderingAlgorithms/RenderPBR.cginc"
# include "RenderingAlgorithms/RenderOcclusion.cginc" 

//color space conversion if unity's using linear space 
float lin2s(float x) 
{ 
    float a = 0.055;
    if(x <= 0.0031308f) 
    {
        return x * 12.92f;
    }
    else
    {
        return (1 + a) * pow(x, 1 / 2.4) - a; 
    } 
}
   
// fragment shader
fixed4 frag(v2f input) : SV_Target
{ 
	if (Weight_Occlusion >= 1.0f)    
		return RenderOcclusion(float4(0, 0, 0, 0));   
#if RENDER_UV_SPACE  
    input.worldPosition= mul(unity_ObjectToWorld, mul(modelTransform, input.vertexPosition)); // recover world position after fooling it in between vert->frag
#endif 

    float4 resultColor = float4(0, 0, 0, 0);

	// swap the following evert two lines to enable the holographic effects to be applid on the original Holoportation rendering algorithm
	// resultColor += RenderColor(input) * (Weight_Color + Weight_HolographicEffect);
	resultColor += RenderColor(input) * (Weight_Color + Weight_HolographicEffect);   
	//resultColor += RenderMontage4D(input) * (Weight_Montage4D + Weight_Intrinsic4D);   
	resultColor += RenderMontage4D(input, Weight_Montage4D) * (Weight_Montage4D + Weight_Intrinsic4D);

	// this is a simple cartoon poster effect 
	if (Weight_Intrinsic4D > EPSILON) {
		float3 yuv = RGBtoYUV(resultColor.rgb);
		resultColor = float4(YUV2RGB(float3(floor(yuv.x * 3.0) / 3.0, yuv.yz)), 1.0);
		//resultColor = float4(YUV2RGB(float3(0.5, yuv.yz)), 1.0); 
	} 
	 
	resultColor += RenderShinyGeometry(input) * Weight_ShinyGeometry;
	resultColor += RenderWireframes(input) * Weight_Wireframe;
//	resultColor += RenderPBR(input, Weight_Art) * Weight_Art;  

	// add posterior effects
	resultColor = RenderNormals(input, resultColor);
	resultColor = RenderHolographicEffect(input, resultColor);
	resultColor = RenderExplosion(resultColor);

	resultColor = RenderOcclusion(resultColor);

//#if RENDER_UV_SPACE
//    resultColor = float4(lin2s(resultColor.r), lin2s(resultColor.g), lin2s(resultColor.b), 1); 
//#endif
      
    resultColor.a *= GlobalAlphaMultiplier;
	resultColor.rgb *= resultColor.a;

    #if RENDER_UV_SPACE 
    //this is here so when we write the PNG to disk using bmp writer, the channels are in the right place
        resultColor = float4(resultColor.b, resultColor.g, resultColor.r, resultColor.a);
    #endif
		  
	return resultColor;   
}  
    