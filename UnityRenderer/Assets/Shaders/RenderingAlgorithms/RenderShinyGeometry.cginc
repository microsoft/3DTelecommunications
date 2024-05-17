float4 RenderShinyGeometry(v2f input)
{
	if (Weight_ShinyGeometry < EPSILON) return BLANK_COLOR;

	float4 resultColor = BLANK_COLOR;

	float ambient_light_intensity = 0.1f;
	float diffuse_light_intensity = 0.45f;
	float specular_light_intensity = 0.45f;
	float shininess = 100;

	float3 V = normalize(input.viewVec);
	float3 L = V;
	float3 N = normalize(input.localNormal);
	float3 H = normalize(V + N);

	float3 ambient = ambient_light_intensity.xxx;
	float3 diffuse = diffuse_light_intensity * saturate(dot(N, L)).xxx;
	float3 specular = specular_light_intensity * pow(saturate(dot(N, H)), shininess).xxx;
	resultColor.rgb = ambient + diffuse + specular;

	resultColor.a = 1;

	// Montage4D debugger 
	//float occulusionSeamFactors[NUM_PODS];
	//occulusionSeamFactors[0] = input.occulusionSeams0.x;	occulusionSeamFactors[1] = input.occulusionSeams0.y;	occulusionSeamFactors[2] = input.occulusionSeams0.z;	occulusionSeamFactors[3] = input.occulusionSeams0.w;
	//occulusionSeamFactors[4] = input.occulusionSeams1.x;	occulusionSeamFactors[5] = input.occulusionSeams1.y;	occulusionSeamFactors[6] = input.occulusionSeams1.z;	occulusionSeamFactors[7] = input.occulusionSeams1.w;

	//if (debugOcculusionSeams > 0) {
	//	for (int i = 3; i < 4; ++i) {
	//		if (occulusionSeamFactors[i] < 0.99 && occulusionSeamFactors[i] > 0.95) resultColor = RGBLabel(i);
	//	}
	//}

	return resultColor * Weight_ShinyGeometry;
}