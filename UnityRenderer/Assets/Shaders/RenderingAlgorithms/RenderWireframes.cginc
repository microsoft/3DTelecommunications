// Author: Ruofei Du. Microsoft Research, Redmond. August 2016.
// Fragment shader for rendering wireframes for the Mobile Holoportation Project
float4 RenderWireframes(v2f input)
{
	if (Weight_Wireframe < EPSILON) return BLANK_COLOR;
	const float4 WIREFRAME_COLOR = float4(0.0f, 1.0f, 0.0f, 1.0f);
	const float4 INNER_COLOR = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float3 d = fwidth(input.col.xyz);
	// anti-aliasing the wireframe
	float3 tdist = smoothstep(float3(0.0f, 0.0f, 0.0f), d * 1.0, input.col.xyz);
	return lerp(WIREFRAME_COLOR, INNER_COLOR, min(min(tdist.x, tdist.y), tdist.z));
}