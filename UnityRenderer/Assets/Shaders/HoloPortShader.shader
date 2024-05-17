// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/HoloPortShader" {
	Properties{
		_Color("Color", Color) = (1, 0.3, 0.3, 1)
	}

	SubShader
	{
		Pass
		{
			Cull off

			CGPROGRAM
				#pragma target 5.0
				#pragma vertex vert
				#pragma fragment frag
				#include "UnityCG.cginc"

				uniform StructuredBuffer<float3> vertices;
				uniform StructuredBuffer<int> verticesHalf;
				uniform StructuredBuffer<int> indices;

				float4x4 modelTransform;
				int indexCount;
				int colorImageWidth;
				int useHalf;
				fixed4 _Color;

				struct v2f
				{
					float4 pos	:	SV_POSITION;
					float3 col	:	Color;
				};

				v2f vert(uint id : SV_VertexID)
				{
					int idx = indices[id];

					float3 position;
					float3 normal;

					if (useHalf > 0) {
						int newIndex = idx * 3;
						int packedHalf = verticesHalf[newIndex++];
						position.x = f16tof32(packedHalf & 0xFFFF);
						position.y = f16tof32((packedHalf >> 16) & 0xFFFF);

						packedHalf = verticesHalf[newIndex++];
						position.z = f16tof32(packedHalf & 0xFFFF);
						normal.x = f16tof32((packedHalf >> 16) & 0xFFFF);

						packedHalf = verticesHalf[newIndex];
						normal.y = f16tof32(packedHalf & 0xFFFF);
						normal.z = f16tof32((packedHalf >> 16) & 0xFFFF);
					}
					else {
						int index = idx * 2;
						position = vertices[index];
						normal = vertices[index + 1];
					}

					normal = normalize(mul(modelTransform, float4(normal, 0.0)).xyz);
					v2f OUT;
					OUT.pos = UnityObjectToClipPos(mul(modelTransform, float4(position, 1)));
					OUT.col = dot(float3(0,1,0), normal) * 0.5 + 0.5;

					return OUT;
				}

				float4 frag(v2f IN) : COLOR {
					return float4(IN.col.x * _Color.x, IN.col.y * _Color.y, IN.col.z * _Color.z, 1);
				}
			ENDCG
		}
	}
}
