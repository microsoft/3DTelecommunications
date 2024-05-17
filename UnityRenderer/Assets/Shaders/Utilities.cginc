float3 RGBtoHCV(in float3 RGB)
{
	float Epsilon = 1e-10;
	float4 P = (RGB.g < RGB.b) ? float4(RGB.bg, -1.0, 2.0 / 3.0) : float4(RGB.gb, 0.0, -1.0 / 3.0);
	float4 Q = (RGB.r < P.x) ? float4(P.xyw, RGB.r) : float4(RGB.r, P.yzx);
	float C = Q.x - min(Q.w, Q.y);
	float H = abs((Q.w - Q.y) / (6 * C + Epsilon) + Q.z);
	return float3(H, C, Q.x);
}

float3 RGBtoHSV(in float3 RGB)
{
	float Epsilon = 1e-10;
	float3 HCV = RGBtoHCV(RGB);
	float S = HCV.y / (HCV.z + Epsilon);
	return float3(HCV.x, S, HCV.z);
}

inline float3 RGBtoYUV(in float3 RGB)
{
	const float3x3 RGB2YUV = float3x3(0.2126, -0.09991, 0.615, 0.7152, -0.33609, -0.55861, 0.0722, 0.436, -0.05639);
	return mul(RGB, RGB2YUV);
}

inline float3 YUV2RGB(in float3 YUV)
{
	const float3x3 YUV2RGB = float3x3(1.0, 1.0, 1.0, 0.0, -0.21482, 2.12798, 1.28033, -0.38059, 0.0);
	return mul(YUV, YUV2RGB); 
}

float4 RGBColor(float r, float g, float b) {
	return float4(r / 255, g / 255, b / 255, 1.0); 
}

// for debugging purpose, return a color-blind-friendly and print-friendly color in RGB space
// for visualization, please refer to Ruofei's shadertoy: https://www.shadertoy.com/view/4ltSWN
float4 RGBLabel(int i) {
	if (i == 0) return RGBColor(127, 201, 127);
	if (i == 1) return RGBColor(190, 174, 212);
	if (i == 2) return RGBColor(253, 192, 134);
	if (i == 3) return RGBColor(255, 255, 153);
	if (i == 4) return RGBColor(56, 108, 176);
	if (i == 5) return RGBColor(240, 2, 127);
	if (i == 6) return RGBColor(191, 91, 23);
	if (i == 7) return RGBColor(102, 102, 102);
	return RGBColor(0, 0, 0);
}

float lerp(float x, float x1, float x2, float y1, float y2)
{
	return lerp(y1, y2, (x - x1) / (x2 - x1));
}

float lerp_clamp(float x, float x1, float x2, float y1, float y2)
{
	return clamp(lerp(x, x1, x2, y1, y2), min(y1, y2), max(y1, y2));
}

float3 project(float4x4 mat, float3 p)
{
	float4 h = mul(mat, float4(p, 1));
	return h.xyz / h.w;
}

float distanceSqr(float3 a, float3 b)
{
	float3 diff = a - b;
	return dot(diff, diff);
}

float colorDistanceAbs(float3 a, float3 b)
{
	return distance(a, b);
}

float distanceRel(float a, float b)
{
	return max(a, b) / min(a, b) - 1.f;
}

float colorDistanceRel(float3 a, float3 b)
{
	float res = max(distanceRel(a.r, b.r), distanceRel(a.g, b.g));
	return max(res, distanceRel(a.b, b.b));
}

// Note, this normalized dot product removes the elements on the y axis, which produces more accurate angle judgement
float normDot(float3 a, float3 b)
{
	a.y = 0;
	b.y = 0;
	a = normalize(a);
	b = normalize(b);
	return dot(a, b);
}

float dist2(int i, float depth1, float depth2)
{
	float4x4 matToUse = gl_TextureInvMatrices[i];
	float3 p1 = project(matToUse, float3(0.5, 0.5, 0.5 * depth1 + 0.5));
	float3 p2 = project(matToUse, float3(0.5, 0.5, 0.5 * depth2 + 0.5));
	return distanceSqr(p1, p2);
}

float4 sampleColor(float2 uv, int i) {
	if (uv.x < 0.0 || uv.y < 0.0 || uv.y > 1.0 || uv.y > 1.0) 
		return float4(0.0, 0.0, 1.0, 0.0);
	return UNITY_SAMPLE_TEX2DARRAY(_CamImg0, float4(uv, i, 0));
}

float4 sampleTexture(float2 uv, int i) {
	if (uv.x < 0.0 || uv.y < 0.0 || uv.y > 1.0 || uv.y > 1.0) 
		return float4(0.0, 0.0, 0.0, 0.0);
	return UNITY_SAMPLE_TEX2DARRAY(_CamImg0, float4(uv, i, 0));
}

float4 sampleTexture(float u, float v, int i) { 
	return sampleTexture(float2(u, v), i); 
}

float sampleDepthMap(float2 uv, int podNum) {
	if (uv.x < 0.0 || uv.y < 0.0 || uv.y > 1.0 || uv.y > 1.0) 
		return 1e6;
    return (1 - tex2Dlod(_DepthMap0, float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 0) return (1 - tex2Dlod(_DepthMapArr[0], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 1) return (1 - tex2Dlod(_DepthMapArr[1], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 2) return (1 - tex2Dlod(_DepthMapArr[2], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 3) return (1 - tex2Dlod(_DepthMapArr[3], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 4) return (1 - tex2Dlod(_DepthMapArr[4], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 5) return (1 - tex2Dlod(_DepthMapArr[5], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    if (podNum == 6) return (1 - tex2Dlod(_DepthMapArr[6], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
    return (1 - tex2Dlod(_DepthMapArr[0], float4(uv, 0, 0)).x) * 2.0 - 1.0;
    //return (1 - tex2Dlod(_DepthMapArr[podNum], float4(uv, 0, 0)).x) * 2.0 - 1.0; 
}
float  sampleDepthMap(float u, float v, int podNum) { 
	return sampleDepthMap(float2(u, v) , podNum); 
}

float3 sampleColorSharpened(float2 texCoord, int i) {
	float filter[5][5] = {
		-1, -1, -1, -1, -1,
		-1,  2,  2,  2, -1,
		-1,  2,  8,  2, -1,
		-1,  2,  2,  2, -1,
		-1, -1, -1, -1, -1,
	};

	float weight_sum = 0.0f;
	float3 color_sum = float3(0, 0, 0);
	for (int x = -2; x <= 2; ++x)
		for (int y = -2; y <= 2; ++y) {
			float w = filter[x + 2][y + 2];
			color_sum += w * UNITY_SAMPLE_TEX2DARRAY(_CamImg0, float4(texCoord.x + x * PIXEL_SIZE_X_COLOR, texCoord.y + y * PIXEL_SIZE_Y_COLOR * 0.125f, i, 0)).rgb;
			weight_sum += w;
		}
	return color_sum / weight_sum;
}
