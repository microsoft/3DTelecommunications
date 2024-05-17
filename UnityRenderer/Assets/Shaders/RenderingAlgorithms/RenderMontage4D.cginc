// enable the DEBUG flag to enable 0-9, F1-F7 debug views.
#define ENABLE_MONTAGE4D_DEBUG
//#define ENABLE_VOTE

// This is a variant, more real-time version of the Montage4D rendering algorithm
// See the submitted paper, "Montage4D: Interactive Seamless Fusion of Multiview Video Textures"
// Project Peabody. Ruofei Du and Wayne Chang.
float4 RenderMontage4D(in v2f input, in float SumWeight_Montage4D)
{
	// ealry exit
	if (SumWeight_Montage4D < EPSILON) return BLANK_COLOR;

	// depth texture coordinates per texture camera
	float3 mDepthTexCoord[NUM_PODS];

	// depth texture coordinates per texture camera
	float2 mColorTexCoord[NUM_PODS];

	// weights based on normals
	float normalWeights[NUM_PODS];
	//float3 vt_to_cam[NUM_PODS];		// angles between vertex to camera 

	uint i;
	// First, calculate the corresponding uv coordinates in the texture map and depth map
	[OPT_UNROLL] for (i = 0; i < NUM_PODS; ++i) {
		float4 t = mul(gl_TextureMatrices[i], input.worldPosition);
		// Color texture is arranged in 1xN layout
		mColorTexCoord[i] = float2(t.x / t.w, t.y / t.w * COLOR_UVCOORD_Y + COLOR_UVCOORD_Y * i);
		// Depth texture is arranged in 2x4 layout
		mDepthTexCoord[i] = float3(t.x / t.w * DEPTH_UVCOORD_X + DEPTH_UVCOORD_X * float(i % 4), t.y / t.w * DEPTH_UVCOORD_Y + DEPTH_UVCOORD_Y * float(i < 4), t.z / t.w);
		// Normal weight is determined by the normals, and the vector from camera center to the vertex, in the object space
		// normalWeights[i] = pow(max(0.0f, normDot(_texCamCenters[i].xyz - input.vertexPosObjectSpace, input.normal)), SMOOTH_NORMALS_POWER); // normDot eliminates the y-axis
		// a better approach is to use normDot, if we know the up vector is the y axis, so that we can eliminate the impact from the y axis.
		float dotProd = max(0.0f, dot(normalize(_texCamCenters[i].xyz - input.vertexPosObjectSpace), normalize(input.normal)));
		if (dotProd >= _DotProductNormalWeightThreshold)
		{
			normalWeights[i] = pow(dotProd, SMOOTH_NORMALS_POWER);
		}
		else
		{
			normalWeights[i] = 0.01;
		}

	}
	
	float w[NUM_PODS], geodesicFields[NUM_PODS];
	//float badDiffusion[NUM_PODS];
	
	//float3 flippedY_WorldSpaceCameraPos = mul(_WorldSpaceCameraPos, modelTransform);
	//float3 currentViewVector = flippedY_WorldSpaceCameraPos.xyz - input.worldPosition;

	// Calculate the normalWeightedBlendedColor for comparison, the result is exactly the same as the original approach
	const float COLOR_TINY_EPS = 0.0001f;	// < 1 in RGB values
	float3 normalWeightedBlendedColor = BLANK_VEC3;
	float3 normalWeightedBlendedLabel = BLANK_VEC3;
	{
		float weights_total = 0.0, weights_wider = 0.0;
		float3 wider_color = BLANK_VEC3;
		float3 wider_label = BLANK_VEC3;

		const float search_radius = 4;
		const float search_step = 2;
		const float depth_tolerance = 5. / 100.;
		const float depth_tolerance2 = depth_tolerance * depth_tolerance;
		const float discontinuity_total_w = 25.0f;

		float3 depthTexCoord = BLANK_VEC3;
		float2 colorTexCoord = BLANK_VEC2;

		[unroll] for (int i = 0; i < NUM_PODS; i++) {
			depthTexCoord = mDepthTexCoord[i];
			colorTexCoord = mColorTexCoord[i];
			float discontinuity_factor = 0.f;

			// 1) depth test 
			float d_center = sampleDepthMap(depthTexCoord.xy, i);
			if (depthTexCoord.z > d_center + 0.008f) continue;

			// 2) foreground test
			float4 curColor = sampleColor(colorTexCoord.xy, i);
	
			if (curColor.r < COLOR_TINY_EPS && curColor.g < COLOR_TINY_EPS && curColor.b < COLOR_TINY_EPS) continue;

			// 3) depth discontinuousity test
			[loop] for (float x = -search_radius; x <= search_radius + 0.1; x += search_step)
				[loop] for (float y = -search_radius; y <= search_radius + 0.1; y += search_step) {
				float depth_x = depthTexCoord.x + 0.25f * x * PIXEL_SIZE_X_DEPTH;
				float dpeth_y = depthTexCoord.y + 0.5f * y * PIXEL_SIZE_Y_DEPTH;
				float w = 1.f;
				float di = sampleDepthMap(depth_x, dpeth_y, i);
				float distance2 = dist2(i, depthTexCoord.z, di);
				if (distance2 > depth_tolerance2) discontinuity_factor += w;
			}

			discontinuity_factor /= discontinuity_total_w * 0.4;
			discontinuity_factor = 1.f - saturate(discontinuity_factor);
			float w = discontinuity_factor * normalWeights[i];
			normalWeightedBlendedColor += curColor.rgb * w;
			if (debugColorLabeledFields) normalWeightedBlendedLabel += RGBLabel(i).rgb * w;
			weights_total += w;

			wider_color += curColor.rgb * w;
			wider_label += RGBLabel(i).rgb * w;

			normalWeights[i] = discontinuity_factor * normalWeights[i];
			weights_wider += normalWeights[i];
		}

		if (weights_total > 0) 
			normalWeightedBlendedColor /= weights_total; 
		else
			if (weights_wider > 0) 
				normalWeightedBlendedColor = wider_color / weights_wider;

#ifdef ENABLE_MONTAGE4D_DEBUG
		if (debugColorLabeledFields) {
			if (weights_total > 0) 
				normalWeightedBlendedLabel /= weights_total; 
			else
				if (weights_wider > 0) 
					normalWeightedBlendedLabel = wider_label / weights_wider;
		}
#endif
	} // End of normalWeightedBlendedColor

	// Calculate the geodesicFields weights from the compute shaders
	geodesicFields[0] = input.geodesics0.x;	
	#if NUM_PODS > 1
		geodesicFields[1] = input.geodesics0.y;	
	#endif
	#if NUM_PODS > 2
		geodesicFields[2] = input.geodesics0.z;	
	#endif
	#if NUM_PODS > 3
		geodesicFields[3] = input.geodesics0.w;	
	#endif
	#if NUM_PODS > 4
		geodesicFields[4] = input.geodesics1.x;
	#endif
    #if NUM_PODS > 5
    	geodesicFields[5] = input.geodesics1.y;	
    #endif	
	#if NUM_PODS > 6
			geodesicFields[6] = input.geodesics1.y;	
	#endif	
	#if NUM_PODS > 7
			geodesicFields[7] = input.geodesics1.w;
	#endif
	#if NUM_PODS > 8
			geodesicFields[8] = input.geodesics2.x;
	#endif
	#if NUM_PODS > 9
			geodesicFields[9] = input.geodesics2.y;
	#endif	
	#if NUM_PODS > 10
			geodesicFields[10] = input.geodesics2.y;
	#endif	
	#if NUM_PODS > 11
			geodesicFields[11] = input.geodesics2.w;
	#endif	
	[OPT_UNROLL] for (i = 0; i < NUM_PODS; ++i) {
		w[i] = normalWeights[i];
		//w[i] = normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition); // +emphasis[i]; // angles between view to current camera
		//w[i] = normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition) * normDot(vt_to_cam[i], _texCamCenters[i].xyz - input.worldPosition);
		//w[i] = max(normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition), normDot(vt_to_cam[i], _texCamCenters[i].xyz - input.worldPosition)); 
	}
	// Calculate candidate colors and discontinuity factors from each view
	float4 colorCandidates[NUM_PODS]; // .rgb - color with depth test .w - blurred edge detection mask
	 float search_radius = 4; // 3
	 float search_step = 2; // 1
	//const float depth_tolerance = 20.0 / 100; // 5.0 / 100
	float depth_tolerance = _DiscontinuityThreshold;
	const float depth_tolerance2 = depth_tolerance * depth_tolerance;
	const float depth_tolerance_sil = 50000.0 / 100;
	const float depth_tolerance_sil2 = depth_tolerance_sil * depth_tolerance_sil;
	const float NINF = -100.0f; 
	const float EPS = 0.0001f;
	const float COLOR_EPS = 0.01f;	// < 2.55 in RGB values

	int candidates_num = 0;
	
	float3 depthTexCoord = float3(0, 0, 0);
	float2 colorTexCoord = float2(0, 0); 

	[unroll] for (i = 0; i < NUM_PODS; i++) {

		search_radius = _DepthSearchRadius[i];
		search_step = _DepthSearchStep[i];
		depthTexCoord = mDepthTexCoord[i];
		colorTexCoord = mColorTexCoord[i];
		float d0 = sampleDepthMap(depthTexCoord.xy, i);
		colorCandidates[i] = float4(sampleColor(colorTexCoord.xy, i).rgb, 0.0f);

		// a) discard the candidates whose depth are out of boundary or directed occulded 
		if (d0 >= 1.0f || depthTexCoord.z > d0 + 0.008f) {
			w[i] = NINF;
			continue;
		}
		
		// b) check if this is background
		/*if (colorCandidates[i].r < COLOR_EPS && colorCandidates[i].g < COLOR_EPS && colorCandidates[i].b < COLOR_EPS) {
			w[i] = NINF;
			continue;
		}*/
		colorCandidates[i].w = 0.02f;
		float discontinuity_factor = 0.f;
		float discontinuity_total_w = 0.000001f;

		[loop] for (float x = -search_radius; x <= search_radius + 0.1; x += search_step) {
			[loop] for (float y = -search_radius; y <= search_radius + 0.1; y += search_step) {
				float depth_x = depthTexCoord.x + 0.25f * x * PIXEL_SIZE_X_DEPTH;
				float depth_y = depthTexCoord.y + 0.5f * y * PIXEL_SIZE_Y_DEPTH;
				float dw = 1.f;
				discontinuity_total_w += dw;
				float di = sampleDepthMap(depth_x, depth_y,i);
				float distance2 = dist2(i, depthTexCoord.z, di);
				if (distance2 > depth_tolerance_sil2) { 
					w[i] = NINF; 
				}
				if (distance2 > depth_tolerance2) discontinuity_factor += dw;
				//if (distance2 > depth_tolerance2) w[i] = NINF; 
			}
		}
		if (w[i] < 0) continue;

		discontinuity_factor /= discontinuity_total_w * 0.4;
		discontinuity_factor = 1.0f - saturate(discontinuity_factor);
		colorCandidates[i].w = discontinuity_factor *normalWeights[i];
		w[i] = w[i] * discontinuity_factor;

		if (colorCandidates[i].w > 0.1) {
			candidates_num += 1.f;
		}
		// Experiment: Apply a hard threshold? 
		// if (normalWeights[i] < NormalWeightsThreshold) normalWeights[i] = 0.0; 
	}

#ifdef ENABLE_VOTE
	// votes for color agreements
	int votes[NUM_PODS];
	[OPT_UNROLL] for (int j = 0; j < NUM_PODS; j++) {
		votes[j] = 0;
		[OPT_UNROLL] for (int k = 0; k < NUM_PODS; k++) {
			votes[j] += (k != j && colorCandidates[j].r >= 0 && colorCandidates[k].r >= 0 && distanceSqr(colorCandidates[k], colorCandidates[j]) < 0.15);
		}
	}
#endif

	// Debug colors from single view  
#ifdef ENABLE_MONTAGE4D_DEBUG
	if (debugSingleView < NUM_PODS) {
		float4 resultColor = BLANK_COLOR;
		// F2
		if (debugGeodesicFields > 0) {
			float z = 0.0;
			z = geodesicFields[debugSingleView];
			// z = vertexComputedWeights[debugSingleView];
			//return float4(z, z, z, 1.0);
			//z = (normalWeights[debugSingleView] + NormalBaseWeight) * geodesicFields[debugSingleView] * TemporalTextureFields[debugSingleView];
			//if (z > geodesicFields[debugSingleView]) z = geodesicFields[debugSingleView];
			//z = 1.0 - pow(1.0 - geodesicFields[debugSingleView], 2.2);
			//if (debugNormalWeightedField) z = 1.0 - pow(1.0 - badDiffusion[debugSingleView], 2.2);
			//if (debugNormalWeightedField) z = 0.5; 
			return float4(z, z, z, 1.0);
		}

		// F3
		if (debugNormalWeightedField > 0) {
			float x = clamp(normalWeights[debugSingleView], 0.0, 1.0);
			// x = colorCandidates[debugSingleView].w;
			return float4(x, x, x, 1);
		}

		// F7
		if (debugC0) {
			i = debugSingleView;
			float x = 0;  
			if (normalWeights[i] > NormalWeightsThreshold) {
				x = clamp((normalWeights[i] * 3.0 + NormalBaseWeight) * geodesicFields[i], 0, geodesicFields[i]);
			}
			return float4(x, x, x, 1);
		}

		if (debugDiscontinuity > 0) {
			//vertexComputedWeights  
			if (debugNormalWeightedField) {
				resultColor = (colorCandidates[debugSingleView].w > 0.9) ? float4(colorCandidates[debugSingleView].rgb, 1.0) : RGBLabel(0);
			}
			//else 
			//if (debugC0) {
			//	//resultColor = (votes[debugSingleView] > 3 || colorCandidates[debugSingleView].w > 0.9) ? float4(colorCandidates[debugSingleView].rgb, 1.0) : RGBLabel(0);
			//	//if (colorCandidates[debugSingleView].w < 0.2) resultColor = RGBLabel(0);
			//	resultColor = float4(colorCandidates[debugSingleView].rgb, 1.0);
			//	if (colorCandidates[debugSingleView].r < 0.1 && colorCandidates[debugSingleView].g < 0.1 &&
			//		colorCandidates[debugSingleView].b < 0.1) resultColor = RGBLabel(0);
			//}
			else
			{ 
				resultColor = (colorCandidates[debugSingleView].w > 0.05) ? float4(colorCandidates[debugSingleView].rgb, 1.0) : RGBLabel(0);
				//resultColor = float4(colorCandidates[debugSingleView].rgb, 1.0 - geodesicFields[debugSingleView]);  
			}
		} else {
			resultColor = float4( colorCandidates[debugSingleView].rgb, 1.0 ) ;
		}

		float3 d = fwidth(input.col.xyz);
		float3 tdist = smoothstep(float3(0.0, 0.0, 0.0), d * 1.0, input.col.xyz);

		if (debugSeams)
			return lerp(float4(0.0, 1.0, 0.0, 1.0), resultColor, min(min(tdist.x, tdist.y), tdist.z));
		else
			return resultColor; 
	}
	// normalWeightedBlendedColor = float3(0.0, 0.0, 0.0); 
	if (debugNormalWeightedBlendingColor) {
		return float4(normalWeightedBlendedColor, NormalWeightedBlendingOpacity);
	}
#endif

	// final pass to mix the texture fields
	float3 resultColor = float3(0.0, 0.0, 0.0);
	float resultWeights = 0.0;
	[OPT_UNROLL] for (i = 0; i < NUM_PODS; i++) {
		float w = clamp( (normalWeights[i] + NormalBaseWeight) * geodesicFields[i] * TemporalTextureFields[i], 0, geodesicFields[i]);
		resultColor += colorCandidates[i].rgb * w;
		resultWeights += w;
	}

	if (resultWeights <= 0.0) return float4(normalWeightedBlendedColor * NormalWeightedBlendingOpacity, 1.0);
	resultColor = lerp(resultColor + (1.0f - saturate(resultWeights)) * normalWeightedBlendedColor * NormalWeightedBlendingOpacity,
		resultColor / resultWeights, step(1.0, resultWeights));
	// more shiny rendering effect will occur, if you change the last step(1.0, resultWeights) to step(resultWeights, 1.0)

	// Debug: F1, Color Label, Start
#ifdef ENABLE_MONTAGE4D_DEBUG
	if (debugColorLabeledFields) {
		resultColor = float3(0.0, 0.0, 0.0);
		float resultWeights = 0.0;
		for (i = 0; i < NUM_PODS; i++) {
			float w = (normalWeights[i] + NormalBaseWeight) * geodesicFields[i] * TemporalTextureFields[i];
			if (w > geodesicFields[i]) w = geodesicFields[i];

			if (w <= 0.0f) continue;
			resultColor += RGBLabel(i).rgb * w;
			resultWeights += w;
		}

		// if the result weights are not enough, use normal weighted blending to compensate
		if (resultWeights <= 0.0) return float4(normalWeightedBlendedLabel * NormalWeightedBlendingOpacity, 1.0);
		resultColor = lerp(resultColor + (1.0f - clamp(resultWeights, 0.0, 1.0)) * normalWeightedBlendedLabel * NormalWeightedBlendingOpacity,
			resultColor / resultWeights, step(1.0, resultWeights));
	}
#endif
	//if (RemoveBackgroundPixels && !any(resultColor)) return BLANK_COLOR;
	return float4(resultColor, 1.0);
}
