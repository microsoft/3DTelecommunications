// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/BlurShadowShader"
{
	Properties
	{
		_MainTex("Texture", 2D) = "white" {}
		_ShadowMask("ShadowMask", 2D) = "white" {}
		_ScreenDepth("ScreenDepth", 2D) = "white" {}
	}
		SubShader
	{
		// No culling or depth
		Cull Off ZWrite Off ZTest Always

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"

			uniform sampler2D _ShadowMask;
			uniform sampler2D _ScreenDepth;
			uniform sampler2D _MainTex;
			uniform float _ScreenWidth;
			uniform float _ScreenHeight;
			float4x4 _MainTexToView;
			struct appdata
			{
				float4 vertex : POSITION;
				float2 uv : TEXCOORD0;
			};

			struct v2f
			{
				float2 uv : TEXCOORD0;
				float4 vertex : SV_POSITION;
			};

			v2f vert(appdata v)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(v.vertex);
				o.uv = v.uv;
				return o;
			}


			fixed4 frag(v2f input) : SV_Target
			{
				float blurred_mask = 0.0f;

				int kernel_radius = 3;

				// Uniforms
				float pixelSizeX = 1.f / _ScreenWidth;
				float pixelSizeY = 1.f / _ScreenHeight;

				float blur_scale = 6.f * (_ScreenWidth / 1920.f);

				float center_depth = tex2Dlod(_ScreenDepth, float4(input.uv, 0, 0)).r;
				float4 h_pos_view = mul(_MainTexToView, float4(input.uv, center_depth, 1));
				float center_view_z = -h_pos_view.z / h_pos_view.w;

				blur_scale *= max(0.4f, 1.f - saturate(center_view_z / 5.f));
				//return pos_view_z / 10.f;

				float w_total = 0.0;
				for (int x = -kernel_radius; x <= kernel_radius; ++x)
				for (int y = -kernel_radius; y <= kernel_radius; ++y)
				{
					float xy_depth = tex2Dlod(_ScreenDepth, float4(input.uv + blur_scale * float2(x * pixelSizeX, y * pixelSizeY), 0, 0)).r;

					float4 h_pos_view = mul(_MainTexToView, float4(input.uv, xy_depth, 1));
					float this_view_z = -h_pos_view.z / h_pos_view.w;


					//float w = max(1.f - 300.f * abs(xy_depth - center_depth), 0.f);
					float w = max(0.1f - abs(this_view_z - center_view_z), 0.f);
					float shadow_mask = tex2D(_ShadowMask, input.uv + blur_scale * float2(x * pixelSizeX, y * pixelSizeY)).r;

					// TODO: bilateral blur
					blurred_mask += shadow_mask * w;
					//blurred_mask += shadow_mask;
					w_total += w;
				}
				//blurred_mask /= (2 * kernel_radius + 1) * (2 * kernel_radius + 1);
				blurred_mask /= w_total;

				float4 mainColor = tex2D(_MainTex, input.uv);

				return float4(blurred_mask * mainColor.rgb, lerp(1, mainColor.a, blurred_mask));
				//return center_depth;
				//return mainColor;
return 0.1*mainColor;
			}
			ENDCG
		}
	}
}
