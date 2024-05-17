// Unlit shader. Simplest possible textured shader.
// - no lighting
// - no lightmap support
// - no per-material color

Shader "Unlit/AnnotatedTexture" 
{
    Properties
    {
        _MainTex("Base (RGB)", 2D) = "white" {}
        _AnnotatedTexture("PaintedTexture", 2D) = "black" {}
        
    }

        SubShader{
            Tags { "RenderType" = "Opaque" }
            LOD 100

            Pass {
                CGPROGRAM
                    #pragma vertex vert
                    #pragma fragment frag
                    #pragma target 2.0
                    #pragma multi_compile_fog

                    #include "UnityCG.cginc"

                    struct appdata_t {
                        float4 vertex : POSITION;
                        float2 texcoord : TEXCOORD0;
                        UNITY_VERTEX_INPUT_INSTANCE_ID
                    };

                    struct v2f {
                        float4 vertex : SV_POSITION;
                        float2 texcoord : TEXCOORD0;
                        UNITY_FOG_COORDS(1)
                        UNITY_VERTEX_OUTPUT_STEREO
                    };

                    sampler2D _MainTex;
                    sampler2D _AnnotatedTexture;
                    float4 _MainTex_ST;

                    v2f vert(appdata_t v)
                    {
                        v2f o;
                        UNITY_SETUP_INSTANCE_ID(v);
                        UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                        o.vertex = UnityObjectToClipPos(v.vertex);
                        o.texcoord = TRANSFORM_TEX(v.texcoord, _MainTex);
                        UNITY_TRANSFER_FOG(o,o.vertex);
                        return o;
                    }

                    fixed4 frag(v2f i) : SV_Target
                    {
                        fixed4 baseColor = tex2D(_MainTex, i.texcoord);
                            
                        fixed4 paintedColor = tex2D(_AnnotatedTexture, i.texcoord);
                        float paintedColorWeight = 0.5f;
                        fixed4 col = paintedColorWeight * paintedColor + baseColor; //interpolate two textures
                        UNITY_APPLY_FOG(i.fogCoord, col);
                        UNITY_OPAQUE_ALPHA(col.a);
                        return col;
                    }
                ENDCG
            }
    }

}