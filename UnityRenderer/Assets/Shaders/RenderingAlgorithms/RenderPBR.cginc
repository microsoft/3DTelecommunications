// Author: Ruofei Du. Microsoft Research, Redmond. August 2016.
// Fragment shader for rendering art for the Mobile Holoportation Project
// changed to glass effects?
// I was thinking to use reflection cube to write a glass shader, 
// however, the SpecCube does not work
// so, I have changed it to a PBR (Physcially-based rendering shader)
// the environmental reflection is missing from this PBR now since 
// I don't know why the SpecCube0 is not working. It works if I create a new scene and a new object.
// press 2 and then press 8 for this PBR, otherwise diffuse map is not loaded correctly

// SIGGRAPH course: http://renderwonk.com/publications/s2010-shading-course/
// Other intro: https://www.marmoset.co/posts/basic-theory-of-physically-based-rendering/
// Fresnel refers to differing reflectivity that occurs at different angles. 
// Specifically, light that lands on a surface at a grazing angle will be much 
// more likely to reflect than that which hits a surface dead-on.
float Flesnel(float3 V, float3 H) {
	float VdotH = saturate(dot(V, H));
	float F0 = saturate(_FresnelReflectance);
	float F = pow(1.0 - VdotH, 5.0);
	F *= (1.0 - F0);
	F += F0;
	return F;
}

float D_GGX(float3 H, float3 N) {
	const float PI = 3.1415926535897;
	float NdotH = saturate(dot(H, N));
	float roughness = saturate(_Roughness);
	float alpha = roughness * roughness;
	float alpha2 = alpha * alpha;
	float t = ((NdotH * NdotH) * (alpha2 - 1.0) + 1.0);
	return alpha2 / (PI * t * t);
}

float G_CookTorrance(float3 L, float3 V, float3 H, float3 N) {
	float NdotH = saturate(dot(N, H));
	float NdotL = saturate(dot(N, L));
	float NdotV = saturate(dot(N, V));
	float VdotH = saturate(dot(V, H));

	float NH2 = 2.0 * NdotH;
	float g1 = (NH2 * NdotV) / VdotH;
	float g2 = (NH2 * NdotL) / VdotH;
	float G = min(1.0, min(g1, g2));
	return G;
}

float4 RenderPBR(in v2f input, in float4 originalColor)
{
	if (Weight_Art < EPSILON) return originalColor;
	float4 resultColor = BLANK_COLOR;

	// normal color
	//float3 normal_for_color = -input.normal;
	//resultColor.rgb = normal_for_color * 0.5 + 0.5; 
	
	// reflection effects
	//float3 ref = reflect(-input.viewVec, input.normal);
	//float4 skyData = UNITY_SAMPLE_TEXCUBE(unity_SpecCube0, input.worldRefl);
	//float4 skyData = UNITY_SAMPLE_TEXCUBE(unity_SpecCube0, float3(0.5, 0.5, 0.5));
	//float3 skyColor = DecodeHDR(skyData, unity_SpecCube0_HDR);
	//resultColor.rgb = input.worldRefl;
	//resultColor.a = 1.0; 
	//resultColor.rgb = skyColor;


	// PBR 
	// originalColor is the albedo / diffuse map.
	float3 ambientLight = unity_AmbientEquator.xyz * originalColor.rgb;
	float3 lightDirectionNormal = normalize(_WorldSpaceLightPos0.xyz);
	float NdotL = saturate(dot(input.normal, lightDirectionNormal));
	float3 viewDirectionNormal = normalize((float4(_WorldSpaceCameraPos, 1.0) - input.position).xyz);
	float NdotV = saturate(dot(input.normal, viewDirectionNormal));
	float3 halfVector = normalize(lightDirectionNormal + viewDirectionNormal);
	float D = D_GGX(halfVector, input.normal);
	float F = Flesnel(viewDirectionNormal, halfVector);
	float G = G_CookTorrance(lightDirectionNormal, viewDirectionNormal, halfVector, input.normal);
	float specularReflection = (D * F * G) / (4.0 * NdotV * NdotL + 0.000001);
	float3 diffuseReflection = _LightColor0.xyz * originalColor.xyz * NdotL;
	resultColor.rgb = ambientLight + diffuseReflection + specularReflection; 
	resultColor.a = 1.0; 

	return lerp(originalColor, resultColor, Weight_Art);
}