// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/Spotlight"
{
    Properties{
        //_MainTex("Base (RGB)", 2D) = "white" {}
        _Color("Main Color", Color) = (0,0,0,1)
        _Size("Spotlight Size", Range(0.0, 1.0)) = 0.1
        _ShadedOpacity("Shaded Opacity", Range(0.0, 1.0)) = 0.0
    }

        SubShader{
            Tags { "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" }
            Blend SrcAlpha OneMinusSrcAlpha
            ColorMask RGB
            ZWrite Off
            Cull Off

            Pass {
                CGPROGRAM
                    #pragma vertex vert
                    #pragma fragment frag
                    //#pragma target 2.0

                    #include "UnityCG.cginc"

                    struct appdata_t {
                        float4 vertex : POSITION;
                        float2 uv : TEXCOORD0;
                        UNITY_VERTEX_INPUT_INSTANCE_ID
                    };

                    struct v2f {
                        float4 vertex : SV_POSITION;
                        float2 uv : TEXCOORD0;
                        float2 mouseOffset : TEXCOORD1;
                        float2 aspect : TEXCOORD2;
                        UNITY_VERTEX_OUTPUT_STEREO
                    };

                    float4 _Mouse;
                    float4 _Dimensions;
                    fixed4 _Color;
                    float _Size;
                    float _ShadedOpacity;

                    v2f vert(appdata_t v)
                    {
                        v2f o;
                        UNITY_SETUP_INSTANCE_ID(v);
                        UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                        o.vertex = UnityObjectToClipPos(v.vertex);
                        o.uv = v.uv;
                        float2 aspect = float2(_Dimensions.x / _Dimensions.y, 1.0);
                        o.mouseOffset = (_Mouse.xy / _Dimensions.xy) * aspect;
                        o.aspect = aspect;
                        return o;
                    }

                    fixed4 frag(v2f i) : COLOR
                    {
                        float dist = distance(i.mouseOffset, i.uv.xy * i.aspect) / _Size;
                        float pwidth = length(float2(ddx(dist), ddy(dist)));
                        _Color.a = clamp(_ShadedOpacity - smoothstep(0.5, 0.5 - pwidth, dist), 0.0, 1.0);
                        return _Color;
                        //return fixed4(i.uv.x, i.uv.y, 0.0, 1.0);
                    }
                ENDCG
            }
    }
}