Shader "Custom/HoloPortTextureShader" {
	Properties {
		ExplosionAmount("Explosion Amount", Range(0, 1.0))			=	0
		_NormalBand("Normal Band Center", Range(-0.3, 1.5))			=	0
		_MaxExplosionAmount("Max Explosion Amount", Range(0, 1.0))	=	0.3
		_SpecColor("Specular Color", Color)							=	(1, 1, 1, 1)
		_Shininess("Shininess", Float)								=	10
		_Roughness("Roughness", Float)								=	0.5
		_FresnelReflectance("Fresnel Reflectance", Float)			=	0.2

		_Gradient("Gradient", 2D)									=	"white" {}
		_CamImg0("CamImg 0", 2DArray)									=	"black" {}
		_DepthMap0("DepthMap 0", 2D)								=	"black" {}
		//_Cube("Cubemap", Cube)										=	"" {}
	}

	SubShader 
	{
		Tags { "RenderType" = "Opaque" }
		Tags { "LightMode" = "ForwardBase" }
		Blend SrcAlpha OneMinusSrcAlpha
		//Blend OneMinusDstColor One
		Lighting Off
		ZWrite On

		Pass
		{
			Cull Front
			Name "EssentialPass"
			CGPROGRAM
				#include "UnityCG.cginc"
				#pragma target 5.0
				#pragma vertex vert
				#pragma fragment frag
				#pragma require 2darray

				#define VOXELIZATION 0
                #define RENDER_UV_SPACE 0
				#include "Constants.cginc"
				#include "HoloPortTextureShaderCommon.cginc"			
			ENDCG
		} //pass
	} //subshader
} //shader
