float4 RenderOcclusion(float4 resultColor) {
	if (Weight_Occlusion < EPSILON) return resultColor;
	return lerp(float4(0, 0, 0, 0), resultColor, Weight_Occlusion);
}
