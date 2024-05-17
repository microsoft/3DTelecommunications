Shader "Custom/HoloPortObjectUVShader" {
	Properties {
		ExplosionAmount("Explosion Amount", Range(0, 1.0))			=	0
		_NormalBand("Normal Band Center", Range(-0.3, 1.5))			=	0
		_MaxExplosionAmount("Max Explosion Amount", Range(0, 1.0))	=	0.3
		_SpecColor("Specular Color", Color)							=	(1, 1, 1, 1)
		_Shininess("Shininess", Float)								=	10

		_Gradient("Gradient", 2D)									=	"white" {}
		_CamImg0("CamImg 0", 2DArray)									=	"black" {}
		_DepthMap0("DepthMap 0", 2D)								=	"black" {}
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
			Cull Back
			Name "EssentialPass"
			CGPROGRAM
            
            # include "UnityCG.cginc"
            #pragma target 5.0
            #pragma vertex vert
            #pragma fragment frag
            #define VOXELIZATION 0
            #define RENDER_UV_SPACE 1
			#include "Constants.cginc"
            # include "HoloPortTextureShaderCommon.cginc"			
            ENDCG
		} //pass
	} //subshader
} //shader
