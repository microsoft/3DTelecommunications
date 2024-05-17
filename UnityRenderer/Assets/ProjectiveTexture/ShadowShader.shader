// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/ShadowShader" {
	Properties{
	_MainTex("Base (RGB)", 2D) = "white" {}
	_bwBlend("Black & White blend", Range(0, 1)) = 0
	_ShadowTex("ShadowTex", 2D) = "white" {}
	
	}
	SubShader{
ZTest Always Cull Off ZWrite Off
Fog { Mode off }
		Pass{
			CGPROGRAM

			#pragma vertex vert
			#pragma fragment frag

			#include "UnityCG.cginc"

#define SHADOW_DISTANCE_FADE 0

				float _ShadowFactor;
				float _ShadowBias;

				int hologramReceiveShadow;

				float4x4 _ShadowMatrix;
				float4x4 _MainTexToWorld;
                sampler2D _CameraDepthTexture;
				uniform sampler2D _MainTex;
				uniform sampler2D _ShadowTex;
				uniform sampler2D _MainCamShadowTex;
				uniform float _bwBlend;

				struct appdata
				{
					float4 vertex : POSITION;
					float2 uv : TEXCOORD0;
				};

				struct v2f_Shadow
				{					
					float4 vertex : SV_POSITION;
					float2 uv : TEXCOORD0;
					float4 shadowCoord : TEXCOORD1;

					float4 scrPos : TEXCOORD2;
				};

				v2f_Shadow vert(appdata v)
				{
					v2f_Shadow o;
					o.vertex = UnityObjectToClipPos(v.vertex);
					o.uv = v.uv;
					//o.shadowCoord = mul(_ShadowMatrix, mul(_Object2World, v.vertex));
					//o.scrPos = ComputeScreenPos(o.vertex);
					return o;
				}

				//v2f_image
				float4 frag(v2f_Shadow i) : COLOR{

					//float depthValue = tex2Dproj(_CameraDepthTexture, UNITY_PROJ_COORD(i.scrPos)).r;

 					float mainCameraDepthValue = tex2D(_MainCamShadowTex, i.uv).r;

 					if (!hologramReceiveShadow)
 					{
						float4 h_pos_world = mul(_MainTexToWorld, float4(i.uv, mainCameraDepthValue, 1));
						float3 pos_world = h_pos_world.xyz / h_pos_world.w;
						if (pos_world.y > 0.01)
							return float4(1, 0, 0, 1);
 					}

					float depthValue = min(tex2D(_CameraDepthTexture, i.uv).r, mainCameraDepthValue);

					//float depthValue = mainCameraDepthValue;
					//float depthValue = tex2D(_MainCamShadowTex, i.uv).r;
					float4 h_pos_world = mul(_MainTexToWorld, float4(i.uv, depthValue, 1));
					float3 pos_world = h_pos_world.xyz / h_pos_world.w;

					float4 shadowCoord = mul(_ShadowMatrix, float4(pos_world, 1));
					//return half4(pos_world.x, pos_world.y, pos_world.z, 1);
					//return half4(depthValue, depthValue, depthValue, 1);
					//return float4(i.vertex.yyy * 0.001, 1);
				//	float4 c = tex2D(_MainTex, i.uv);
					
						//SHADOW MAPPING PASS

						//float4 shadowCoord = _ShadowMatrix * i.
					float visibility = 1.0;
					float3 shadowIndex = (shadowCoord.xyz / shadowCoord.w);
					//return c;

					
					if (shadowCoord.w > 0 && shadowIndex.x >= 0.0 && shadowIndex.x <= 1.0 && shadowIndex.y >= 0.0 && shadowIndex.y <= 1.0 && shadowIndex.z < 0.999f)
					{

					//	float epsilon = 0.005*tan(acos(clamp(dotNL, 0, 1))); // cosTheta is dot( n,l ), clamped between 0 and 1
					//	epsilon = clamp(epsilon, 0, 0.005);


						float lightDepth = tex2D(_ShadowTex, shadowIndex.xy).r;
						float pixelDepth = shadowIndex.z;

						//means we're in shadow
						if (lightDepth <= (pixelDepth - _ShadowBias))//epsilon))
						{
						#if SHADOW_DISTANCE_FADE
							float fade_in_distance = 10.f;
							float fade_out_distance = 100.f;
							float distance_lerp = saturate( (length(pos_world) - fade_in_distance) / (fade_out_distance - fade_in_distance) );						
							visibility = lerp(_ShadowFactor, 1, distance_lerp);
						#else
							visibility = _ShadowFactor;
						#endif
						}
					}
					//else
					{
						//visibility = 0.1;
					}
					//return c + float4(shadowIndex.zzz, 1);
					//return float4(i.shadowCoord.b * 0.1, 0, 0, 1);
					//return c + 2*tex2D(_ShadowTex, shadowIndex).rrrr  - 1;
				//	return float4(visibility, 0, 0, 1);
					//return c * visibility;
					return float4(visibility, 0, 0, 1);
				}
				ENDCG
			}
	}
}
