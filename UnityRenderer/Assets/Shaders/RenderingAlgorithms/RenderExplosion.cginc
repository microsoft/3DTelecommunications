float4 RenderExplosionAlpha(float4 resultColor)
{
	if (ExplosionAmount < EPSILON) return resultColor;
	resultColor.a *= 1 - ExplosionAmount;
	return resultColor;
}

float4 RenderExplosionRGB(float4 resultColor)
{
	if (ExplosionAmount < EPSILON) return resultColor; 
	if (ExplosionAmount > 1 - EPSILON) clip(-1);
	resultColor.rgb += float3(ExplosionAmount * 0.6, ExplosionAmount * 0.6, ExplosionAmount * 1.1);
	return resultColor;
}


float4 RenderExplosion(float4 resultColor)
{
	if (ExplosionAmount < EPSILON) return resultColor;
	if (ExplosionAmount > 1 - EPSILON) clip(-1);
	resultColor.rgb += float3(ExplosionAmount * 0.6, ExplosionAmount * 0.6, ExplosionAmount * 1.1);
	resultColor.a *= 1 - ExplosionAmount;
	return resultColor;
}