// Upgrade NOTE: replaced '_Object2World' with 'unity_ObjectToWorld'
// Upgrade NOTE: replaced '_World2Object' with 'unity_WorldToObject'

// Unity built-in shader source. Copyright (c) 2016 Unity Technologies. MIT license (see license.txt)

// Unlit shader. Simplest possible textured shader.
// - no lighting
// - no lightmap support
// - no per-material color

Shader "Unlit/HoloBin" {
Properties {
    _MainTex("Base (RGB)", 2D) = "white" {}
    _HighlightColor("_HighlightColor", Color) = (0.0,1.0,0.0,1.0)
    _HologramTint("_HologramTint", Color) = (0.0,1.0,0.0,1.0)
    _RimPower("_RimPower", Range(0.0,2.0) ) = 0.0
    _RimVis("_RimVis", Range(0.0,5.0) ) = 1.2
_ModelOpacity("_ModelOpacity", Range(0.0, 1.0)) = 0.5
}

SubShader {
    Tags { "RenderType"="Opaque" }
    LOD 100

    Pass {
        CGPROGRAM
#pragma vertex vert
#pragma fragment frag
#pragma target 2.0
#pragma multi_compile_fog

# include "UnityCG.cginc"

//     struct appdata_base
//{
//    float4 vertex   : POSITION;  // The vertex position in model space.
//     float3 normal   : NORMAL;    // The vertex normal in model space.
//     float4 texcoord : TEXCOORD0; // The first UV coordinate.
// };

//holographic effect parameters
fixed4 _HighlightColor;
half _RimPower;
fixed _RimVis;
fixed _ModelOpacity;
fixed4 _HologramTint;

struct v2f
{
    float4 vertex : SV_POSITION;
    float2 texcoord : TEXCOORD0;
    float3 viewVec :	TEXCOORD2;
        float3 localNormal : TEXCOORD3;
    UNITY_VERTEX_OUTPUT_STEREO
};

sampler2D _MainTex;
float4 _MainTex_ST;

v2f vert(appdata_base v)
{
    v2f o;
    UNITY_SETUP_INSTANCE_ID(v);
    UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
    o.vertex = UnityObjectToClipPos(v.vertex);
    o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);
    o.viewVec = _WorldSpaceCameraPos - mul(unity_ObjectToWorld, o.vertex);

    float4x4 modelMatrixInverse = unity_WorldToObject;
    o.localNormal = normalize(mul(float4(v.normal, 0.0), modelMatrixInverse).xyz);

    return o;
}

fixed4 frag(v2f input) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, input.texcoord);

                // Rim highlighting 
                fixed rim = 1 - saturate(dot(normalize(input.viewVec), input.localNormal));
                col.rgb *= _HologramTint.rgb;
	            col.rgb += _HighlightColor.rgb* pow(rim, _RimPower) * _RimVis;
                col.a = _ModelOpacity + pow(rim, _RimPower) * _RimVis;

                return col;
            }
        ENDCG
    }
}

}
