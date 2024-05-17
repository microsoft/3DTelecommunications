#define NEED_VOTING 0

#define CAM_PICK_METHOD_CURRENT_VIEW 0
#define CAM_PICK_METHOD_SMOOTH_NORMALS 1

//#define CAM_PICK_METHOD CAM_PICK_METHOD_CURRENT_VIEW            
#define CAM_PICK_METHOD CAM_PICK_METHOD_SMOOTH_NORMALS

// only valid for METHOD_SMOOTH_NORMALS
//#define MULTIPASS_USE_SHARPENING 0 // slower, better quality
#define MULTIPASS_DRAW_FIRST_PASS_ONLY 0
#define MULTIPASS_DRAW_SECOND_PASS_ONLY 0
#define MULTIPASS_USE_MEAN_TEST 0
#define MULTIPASS_USE_VOTING NEED_VOTING
#define MULTIPASS_VOTING_IN_FIRST_PASS 0
#define MULTIPASS_USE_COLOR_NEIGHBOR NEED_VOTING
#define MULTIPASS_USE_MEAN_TEST_NO_DISCONTINUITY 0

#define SMOOTH_NORMALS_POWER (NEED_VOTING ? 16.f : 4.f) // use 3 for blurry, but less-noisy results 

// original version of the Holoportation Renderer
// revised by Ruofei to use arrays instead of the original verbose code
float4 RenderColor(v2f input)
{
//	if (Weight_Color < EPSILON) return BLANK_COLOR;

	uint i = 0, t = 0;
	float3 mDepthTexCoord[NUM_PODS];
	float2 mColorTexCoord[NUM_PODS];
	[OPT_UNROLL] for (i = 0; i < NUM_PODS; ++i) {
		// Color texture is arranged in 1xN layout
		float4 t = mul(gl_TextureMatrices[i], input.worldPosition);
		mColorTexCoord[i] = float2(t.x / t.w, t.y / t.w);
		// Depth texture is arranged in 2x4 layout
		mDepthTexCoord[i] = float3(t.x / t.w * DEPTH_UVCOORD_X + DEPTH_UVCOORD_X * float(i % 4), t.y / t.w * DEPTH_UVCOORD_Y + DEPTH_UVCOORD_Y * float(i < 4), t.z / t.w);

	}


	float3 normal = input.normal;
	float3 mColor = float3(0.0, 0.0, 0.0);

	// calculate the weights between normals and each camera's center  
#if CAM_PICK_METHOD == CAM_PICK_METHOD_SMOOTH_NORMALS
	float3 vt_to_cam[NUM_PODS];
	float normalWeights[NUM_PODS];
	float3 smooth_normal = normal; 

	[OPT_UNROLL] for (i = 0; i < NUM_PODS; i++) {
		vt_to_cam[i] = normalize(_texCamCenters[i].xyz - input.vertexPosObjectSpace);
		float w = max(0.0, dot(normalize(vt_to_cam[i]), normalize(smooth_normal)));// * 100.0f/dist;
		if (w > _DotProductNormalWeightThreshold)
		{
			normalWeights[i] = pow(w, SMOOTH_NORMALS_POWER); 
		}
		else
		{
			normalWeights[i] = 0.00;
		}

	}
	float w[NUM_PODS];
	[OPT_UNROLL] for (i = 0; i < NUM_PODS; ++i) {
		w[i] = normalWeights[i];
		//w[i] = normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition); // +emphasis[i]; // angles between view to current camera
		//w[i] = normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition) * normDot(vt_to_cam[i], _texCamCenters[i].xyz - input.worldPosition);
		//w[i] = max(normDot(currentViewVector, _texCamCenters[i].xyz - input.worldPosition), normDot(vt_to_cam[i], _texCamCenters[i].xyz - input.worldPosition)); 
	}
	{
		float4 color_candidates[NUM_PODS]; // .rgb - color with depth test .w - blurred edge detection mask
		 float search_radius = 6; // 3
		 float search_step = 2; // 1
		float depth_tolerance = _DiscontinuityThreshold;
		const float depth_tolerance2 = depth_tolerance * depth_tolerance;

		const float depth_tolerance_sil = 50000.0 / 100;
		const float depth_tolerance_sil2 = depth_tolerance_sil * depth_tolerance_sil;

		const float NINF = -100.0f;
		const float COLOR_EPS = 0.01f;	// < 2.55 in RGB values

		[OPT_UNROLL] for (int i = 0; i < NUM_PODS; i++) {
			
			search_radius = _DepthSearchRadius[i];
			search_step = _DepthSearchStep[i];

				//float4(-1.0f, -1.0f, -1.0f, 0.02f);
			float3 depthTexCoord = mDepthTexCoord[i];
			float2 colorTexCoord = mColorTexCoord[i];

			float discontinuity_factor = 0.f;
			float discontinuity_total_w = 0.000001f;

			float d_center = sampleDepthMap(depthTexCoord.xy, i);

			float4 sampledColor4 = sampleColor(colorTexCoord.xy, i);
			color_candidates[i] = float4(sampledColor4.rgb, -1.0f);

			// a) discard the candidates whose depth are out of boundary or directed occulded 
			if (d_center >= 1.0f || depthTexCoord.z > d_center + 0.008f) {
				continue;
			}

			//undefined pixel check
			if (sampledColor4.w <= 0)
			{
				continue;
			}

			// if it's too small, will infinite loop 
			if (search_step <= COLOR_EPS)
			{
				continue;
			}

			[loop] for (float x = -search_radius; x <= search_radius + 0.1; x += search_step) {
				[loop] for (float y = -search_radius; y <= search_radius + 0.1; y += search_step) {
					float depth_x = depthTexCoord.x + 0.25f * x * PIXEL_SIZE_X_DEPTH;
					float depth_y = depthTexCoord.y + 0.5f * y * PIXEL_SIZE_Y_DEPTH;
					float dw = 1.f;
					discontinuity_total_w += dw;
					float di = sampleDepthMap(depth_x, depth_y, i);
					float distance2 = dist2(i, depthTexCoord.z, di);

					//only care if we're being occluded, not when we are occluding someone else
					if (depthTexCoord.z >= di)
					{
						if (distance2 > depth_tolerance_sil2) {
							w[i] = NINF;
							discontinuity_factor += dw;
						}
						if (distance2 > depth_tolerance2) discontinuity_factor += dw;
					}
					
					//if (distance2 > depth_tolerance2) w[i] = NINF; 
				}
			}
			discontinuity_factor /= discontinuity_total_w * 0.4;
			discontinuity_factor = 1.0f - saturate(discontinuity_factor);
			color_candidates[i].w = discontinuity_factor * normalWeights[i];
		}

#define n_trials 1
		float tolerances[n_trials] = { 0.1f };

		float3 filteredColor = float3(0.f, 0.f, 0.f);
		float filtered_weights_total = 0.f;



		int votes[NUM_PODS];
		int max_votes = -1;

		// NEED_VOTING


		// FALSE

		int min_variance_arg = -1;
		// NEED_VOTING
		if (filtered_weights_total > 0) // NaN otherwise
			filteredColor = filteredColor / filtered_weights_total;


		//return float4(filteredColor <= 0, 1);

		float weights_total = 0.f;

		//return float4(color_candidates[1].rgb, 1.f);

		// Majority Voting Algorithm ?
		if (min_variance_arg != -1)
		{
			mColor += color_candidates[min_variance_arg];
			weights_total = 1.f;
		}
		else
		{
			[OPT_UNROLL] for (int i = 0; i < NUM_PODS; i++) //MULTIPASS_DRAW_FIRST_PASS_ONLY
			{
				if (color_candidates[i].x < 0
#if MULTIPASS_VOTING_IN_FIRST_PASS == 0 && !MULTIPASS_DRAW_SECOND_PASS_ONLY
					|| (max_votes > 0 && votes[i] == 0)
#endif
					)
					continue;
				float w = color_candidates[i].w * normalWeights[i] *UseCameraTexture[i];
				if (color_candidates[i].w < _ColorWeightThreshold)
				{
					w = 0.f;
				}
				mColor += color_candidates[i].rgb * w;
				weights_total += w;
			}


		}

		if (weights_total > 0) mColor /= weights_total;


		if (filteredColor.x != 0.f)
		{
			float lerp_factor_rel = lerp_clamp(colorDistanceRel(mColor, filteredColor), 0.3, 0.5, 0, 1);
			float lerp_factor_abs = lerp_clamp(colorDistanceAbs(mColor, filteredColor), 0.05, 0.1, 0, 1);
			float lerp_factor = min(lerp_factor_rel, lerp_factor_abs);
			mColor = lerp(mColor, filteredColor, lerp_factor);
		}


		if (debugSingleView < NUM_PODS) 
		{
			float4 resultColor = BLANK_COLOR;

			float w = color_candidates[debugSingleView].w * normalWeights[debugSingleView];
			if (color_candidates[debugSingleView].w < _ColorWeightThreshold)
			{
				w = 0.f;
			}
			if (debugDiscontinuity > 0) 
			{	
				resultColor = (color_candidates[debugSingleView].w > _ColorWeightThreshold && normalWeights[debugSingleView] > 0.0f) ? float4(color_candidates[debugSingleView].rgb, 1.0) : RGBLabel(0);				
			}
			else 
			{
				resultColor = float4(color_candidates[debugSingleView].rgb * w, 1.0);
			}

			mColor = resultColor;
		}
	}
#endif // CAM_PICK_METHOD_SMOOTH_NORMALS



	

	return float4(mColor, 1.0);
}
