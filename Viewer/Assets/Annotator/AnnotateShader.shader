Shader "Custom/AnnotateShader"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _MainTex ("Albedo (RGB)", 2D) = "white" {}
    }
	SubShader
	{
		// =====================================================================================================================
		// TAGS AND SETUP ------------------------------------------
		Tags { "RenderType"="Opaque" }
		LOD	   100
		ZTest  Off
		ZWrite Off
		Cull   Off

		Pass
		{
			CGPROGRAM
			// =====================================================================================================================
			// DEFINE AND INCLUDE ----------------------------------
			#pragma vertex   vert
			#pragma fragment frag

			
			#include "UnityCG.cginc"

			// =====================================================================================================================
			// DECLERANTIONS ----------------------------------
			struct appdata
			{
				float4 vertex   : POSITION;
				float2 uv	    : TEXCOORD0;
				float2 uv2      : TEXCOORD1;
				uint id         : SV_VertexID;
			};

			struct v2f
			{
				float4 vertex   : SV_POSITION;
				float3 worldPos : TEXCOORD0;
				float2 uv       : TEXCOORD1;
			};

			float4    _Mouse;
			float4x4  mesh_Object2World;
			sampler2D _MainTex;
			float4	  _BrushColor;
			float	  _BrushOpacity;
			float	  _BrushHardness;
			float	  _BrushSize;

			// Is the user interacting
			float    _Active;

			// =====================================================================================================================
			// VERTEX FRAGMENT ----------------------------------

			v2f vert (appdata v)
			{
				v2f o;

				// coordinate transform from uvspace --> screen space
				float2 uvRemapped   = v.uv2.xy;

				uvRemapped.y = 1.0f - uvRemapped.y;
				uvRemapped   = 2 * uvRemapped - 1.0f;

				

				o.vertex     = float4(uvRemapped.xy, 0, 1.0f);
				o.worldPos   = mul(mesh_Object2World, v.vertex);
				o.uv         = v.uv;

				return o;
			}
			
			fixed4 frag(v2f i) : SV_Target
			{
				float4 col = tex2D(_MainTex, i.uv);

				// Hide the brush if the mouse is off the screen
				float  size = _Active * _BrushSize;

				float  soft = _BrushHardness;
				float  f	= distance(_Mouse.xyz, i.worldPos);
				f    = 1.0f-smoothstep(size*soft, size, f);
				
				col  = lerp(col, _BrushColor, f * _BrushOpacity);
				col.w = f * _BrushOpacity;
				col  = saturate(col);
				return col;
			}
			ENDCG
		}

	}
}
