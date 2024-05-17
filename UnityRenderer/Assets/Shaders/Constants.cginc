// Please modify these parameters when changing the pods settings
#undef RGBA_COLOR
#undef OBJ_CAPTURE_RIG
#define EDEN_CAPTURE
#define NUM_PODS 10

#define IMAGE_WIDTH_COLOR 2048.0f
#define IMAGE_HEIGHT_COLOR 2048.0f

# ifdef  RGBA_COLOR
#define NUM_PODS 2
#define IMAGE_WIDTH_COLOR 1920.0f
#define IMAGE_HEIGHT_COLOR 1080.0f
#endif

# ifdef OBJ_CAPTURE_RIG
#define NUM_PODS 5
#endif

# ifdef EDEN_CAPTURE
#define NUM_PODS 10
#define IMAGE_WIDTH_COLOR 3840.0f
#define IMAGE_HEIGHT_COLOR 2160.0f

#endif

#define UNITYDEPTH_DIM_X 4
#define UNITYDEPTH_DIM_Y 2

#define DEPTH_UVCOORD_X 1.0f / UNITYDEPTH_DIM_X
#define DEPTH_UVCOORD_Y 1.0f / UNITYDEPTH_DIM_Y

#define COLOR_UVCOORD_Y 1.0f / NUM_PODS

#define DEPTH_REDUCTION 1.f		
#define IMAGE_WIDTH_DEPTH (IMAGE_WIDTH_COLOR / DEPTH_REDUCTION)
#define IMAGE_HEIGHT_DEPTH (IMAGE_HEIGHT_COLOR / DEPTH_REDUCTION)

#define PIXEL_SIZE_X_DEPTH (1.0 / IMAGE_WIDTH_DEPTH)
#define PIXEL_SIZE_Y_DEPTH (1.0 / IMAGE_HEIGHT_DEPTH)
#define PIXEL_SIZE_X_COLOR (1.0 / IMAGE_WIDTH_COLOR)
#define PIXEL_SIZE_Y_COLOR (1.0 / IMAGE_HEIGHT_COLOR)

#define EPSILON 0.000001
#define BLANK_VEC4 float4(0.0, 0.0, 0.0, 0.0)
#define BLANK_VEC3 float3(0.0, 0.0, 0.0)
#define BLANK_VEC2 float2(0.0, 0.0)
#define BLANK_COLOR BLANK_VEC4

const float4 COLOR_RED = float4(1.0, 0.0, 0.0, 1.0);
const float4 COLOR_GREEN = float4(0.0, 1.0, 0.0, 1.0);
const float4 COLOR_BLUE = float4(0.0, 0.0, 1.0, 1.0);
