float4 RenderHolographicEffect(v2f input, float4 originalColor)
{
	if (Weight_HolographicEffect < EPSILON) return originalColor;
	float4 resultColor = originalColor;

	// Rim highlighting  
	fixed rim = 1 - saturate(dot(normalize(input.viewVec), input.localNormal));
	resultColor.rgb *= _HologramTint.rgb;
	resultColor.rgb += _HighlightColor.rgb * pow(rim, _RimPower) * _RimVis;
	resultColor.a = _ModelOpacity + pow(rim, _RimPower) * _RimVis;

	return lerp(originalColor, resultColor, Weight_HolographicEffect);
}