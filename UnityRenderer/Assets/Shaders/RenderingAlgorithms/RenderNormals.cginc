#define NORMAL_BAND_RADIUS 0.1f
#define NORMAL_BAND_INC

float4 RenderNormals(in v2f input, in float4 originalColor)
{
	float4 resultColor = BLANK_COLOR;
	float modifiedNormalWeight = Weight_Normal;

	/*if (modifiedNormalWeight <= 0.0f)
	{
		float currY = -input.vertexPosition.zzzz * 0.7;
		float maxY = _NormalBand + NORMAL_BAND_RADIUS;
		float minY = _NormalBand - NORMAL_BAND_RADIUS;
		float rangeY = 2 * NORMAL_BAND_RADIUS;
		if (currY < (maxY) && (minY) < currY)
		{
			float newY = (currY - minY) / rangeY;
			modifiedNormalWeight = (1 - tex2D(_Gradient, float2(0, newY))) *(1 - tex2D(_Gradient, float2(0, newY))).r;
		}
	}*/

	float3 normal_for_color = -input.normal;

	resultColor.rgb = normal_for_color * 0.5 + 0.5;

	if (ApplyLightingToNormalMap)
	{
		float ambient_light_intensity = 0.5f;
		float diffuse_light_intensity = 0.5f;
		float specular_light_intensity = 0.45f;
		float shininess = 150;

		float3 V = normalize(input.viewVec);
		float3 L = normalize(float3(0, 1, 0) + normalize(input.viewVec));
		float3 N = normalize(input.localNormal);
		float3 H = normalize(V + N);

		float3 ambient = ambient_light_intensity.xxx;
		float3 diffuse = diffuse_light_intensity * saturate(dot(N, L)).xxx;
		float3 specular = specular_light_intensity * pow(saturate(dot(N, H)), shininess).xxx;
		resultColor.rgb = resultColor.rgb * (ambient + diffuse) + specular;
	}

	resultColor.a = 1;

	return lerp(originalColor, resultColor, modifiedNormalWeight);
}
