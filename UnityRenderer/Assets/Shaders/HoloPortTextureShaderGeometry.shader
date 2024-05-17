Shader "Custom/HoloPortTextureShaderGeometry" {
	Properties {
		ExplosionAmount("Explosion Amount", Range(0, 1.0)) = 0
		_NormalBand("Normal Band Center", Range(-0.3, 1.5)) = 0
		_MaxExplosionAmount("Max Explosion Amount", Range(0, 1.0)) = 0.3
		_SpecColor("Specular Color", Color) = (1, 1, 1, 1)
		_Shininess("Shininess", Float) = 10
		_Roughness("Roughness", Float)						= 0.5
		_FresnelReflectance("Fresnel Reflectance", Float)	= 0.2

		_Gradient("Gradient", 2D) = "white" {}
		_CamImg0("CamImg 0", 2DArray) = "black" {}
		_DepthMap0("DepthMap 0", 2D) = "black" {}
	}

	SubShader
	{
		Tags{ "RenderType" = "Opaque" }
		Tags{ "LightMode" = "ForwardBase" }
		Tags{ "Queue" = "Transparent" "IgnoreProjector" = "True" "RenderType" = "Transparent" }
		Blend SrcAlpha OneMinusSrcAlpha
		Lighting On

		Pass
		{
			Cull Front
			Name "EssentialPass"
			CGPROGRAM
				#include "UnityCG.cginc"
				#pragma target 5.0
				#pragma vertex vert
				#pragma geometry geom
				#pragma fragment frag
			#pragma require 2darray

				#define VOXELIZATION 1
				#include "Constants.cginc"
				#include "HoloPortTextureShaderCommon.cginc"			
			ENDCG
		} //pass
	} //subshader
} //shader
