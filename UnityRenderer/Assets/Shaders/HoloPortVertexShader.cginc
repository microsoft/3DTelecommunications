// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

// vertex to fragment shader output 
struct v2f {
	float4 position				:	SV_POSITION;
	float3 normal				:	NORMAL;
	float4 worldPosition		:	WORLDPOSITION;
	float4 col					:	Color;
	float4 vertexPosition		:	TEXCOORD0;
	float3 vertexPosObjectSpace :	TEXCOORD1;
	float3 viewVec				:	TEXCOORD2;
	float3 localNormal			:	TEXCOORD3;
	float4 geodesics0			:	TEXCOORD4;
	float4 geodesics1			:	TEXCOORD5;
	float4 geodesics2			:	TEXCOORD6;
    float2 uvCoord              :   Color1;
	//float4 weights0			:	TEXCOORD6;
	//float4 weights1			:	TEXCOORD7;
	//float4 compare0			:	TEXCOORD8;
	//float4 compare1			:	TEXCOORD9;
};

void decodePositionNormal(int idx, out float3 position, out float3 normal)
{
	int newIndex = idx * 3;
	int packedHalf = verticesHalf[newIndex++];
	position.x = f16tof32(packedHalf & 0xFFFF);
	position.y = f16tof32((packedHalf >> 16) & 0xFFFF);

	packedHalf = verticesHalf[newIndex++];
	position.z = f16tof32(packedHalf & 0xFFFF);
	normal.x = f16tof32((packedHalf >> 16) & 0xFFFF);

	packedHalf = verticesHalf[newIndex];
	normal.y = f16tof32(packedHalf & 0xFFFF);
	normal.z = f16tof32((packedHalf >> 16) & 0xFFFF);

	position *= 0.01f;
}

// vertex shader, id is index for triangle's node, idx is the true vertex id              
v2f vert(uint id : SV_VertexID) {

    int mult = id / 3;
    int rem = id % 3;
    uint altID = 3 * mult + (2 - rem);

	int idx = _UsePointCloud ? id : indices[id];

   


    float3 normalFirstVertex = float3(1, 0, 0);
	float3 position, normal;

	decodePositionNormal(idx, position, normal);
	v2f o;
	UNITY_INITIALIZE_OUTPUT(v2f, o);

	// col is used to label the three vertices around each triangle. TODO: remove it for release                        
	o.col = float4((id % 3 == 0) ? 1.0 : 0.0, (id % 3 == 1) ? 1.0 : 0.0, (id % 3 == 2) ? 1.0 : 0.0, 1.0);
	
	o.geodesics0 = float4(1.0, 1.0, 1.0, 1.0) - float4(vtGeodesicWeights[idx * NUM_PODS + 0], vtGeodesicWeights[idx * NUM_PODS + 1], vtGeodesicWeights[idx * NUM_PODS + 2], vtGeodesicWeights[idx * NUM_PODS + 3]);
	
	#if NUM_PODS > 4
	o.geodesics1 = float4(1.0, 1.0, 1.0, 1.0) - float4(vtGeodesicWeights[idx * NUM_PODS + 4], vtGeodesicWeights[idx * NUM_PODS + 5], vtGeodesicWeights[idx * NUM_PODS + 6], vtGeodesicWeights[idx * NUM_PODS + 7]);
	#endif
	
	#if NUM_PODS > 8
	o.geodesics2 = float4(1.0, 1.0, 1.0, 1.0) - float4(vtGeodesicWeights[idx * NUM_PODS + 8], vtGeodesicWeights[idx * NUM_PODS + 9], vtGeodesicWeights[idx * NUM_PODS + 10], vtGeodesicWeights[idx * NUM_PODS + 11]);
	#endif
	//o.weights0 = float4(verticesGeodesic[idx * 8 + 0], verticesGeodesic[idx * 8 + 1], verticesGeodesic[idx * 8 + 2], verticesGeodesic[idx * 8 + 3]);
	//o.weights1 = float4(verticesGeodesic[idx * 8 + 4], verticesGeodesic[idx * 8 + 5], verticesGeodesic[idx * 8 + 6], verticesGeodesic[idx * 8 + 7]);
	//o.compare0 = float4(verticesGeodesic[idx * 8 + 0], verticesGeodesic[idx * 8 + 1], verticesGeodesic[idx * 8 + 2], verticesGeodesic[idx * 8 + 3]);
	//o.compare1 = float4(verticesGeodesic[idx * 8 + 4], verticesGeodesic[idx * 8 + 5], verticesGeodesic[idx * 8 + 6], verticesGeodesic[idx * 8 + 7]);

	if (cullingBbox1HalfSize.x > 0 && all(abs(mul(modelTransform, float4(position, 1)).xyz - cullingBbox1Center) <= cullingBbox1HalfSize)) {
		decodePositionNormal(indices[id / 3 * 3], position, normal);
	}

	if (cullingBbox2HalfSize.x > 0 && all(abs(mul(modelTransform, float4(position, 1)).xyz - cullingBbox2Center) <= cullingBbox2HalfSize)) {
		decodePositionNormal(indices[id / 3 * 3], position, normal);
	}

#if VOXELIZATION && VOXEL_SNAP_IN_VERTEX_SHADER
	position += 0.0001;
	position = float4(round(position.xyz / voxel_size) * voxel_size, 1);
#endif

#if VOXELIZATION
	if (id % max(1, int(voxel_size / voxel_no_skip_size) * int(voxel_size / voxel_no_skip_size)) != 0)
		position.y += 10000.f;
#endif
	o.position = float4(position, 1);
    normal = -normal;

    o.vertexPosObjectSpace = o.position.xyz;
#if RENDER_UV_SPACE
    o.uvCoord.x = textureUVs[(id) * 2];
    o.uvCoord.y = 1-textureUVs[(id) * 2 + 1];     //"1-y" is here so when we write the PNG to disk using bmp writer, the image is right sideu p
    o.position = float4(o.uvCoord.xy, 1, 1);
    normal = normal;  // flip it back
#endif



    o.normal = normal;

	float4 transformedPos = mul(modelTransform, o.position);  

	// holographic effect
	float4x4 modelMatrixInverse = inverseModelTransform;
	o.localNormal = normalize(mul(float4(normal, 0.0), modelMatrixInverse).xyz);
	o.viewVec = _WorldSpaceCameraPos - transformedPos.xyz;

	// explosion effect
	if (ExplosionAmount > 0.005) {
		idx = indices[id / 3 * 3];
		int newIndex = idx * 3 + 1;
		int packedHalf = verticesHalf[newIndex++];
		normalFirstVertex.x = f16tof32((packedHalf >> 16) & 0xFFFF);
		packedHalf = verticesHalf[newIndex];
		normalFirstVertex.y = f16tof32(packedHalf & 0xFFFF);
		normalFirstVertex.z = f16tof32((packedHalf >> 16) & 0xFFFF);

		normalFirstVertex = normalize(mul(modelTransform, float4(normalFirstVertex, 0.0)).xyz);
		normalFirstVertex *= -1;

		transformedPos.xyz += normalFirstVertex * ExplosionAmount * _MaxExplosionAmount;
	}
    
    o.worldPosition = mul(unity_ObjectToWorld, transformedPos);
#if !VOXELIZATION
	o.position = UnityObjectToClipPos(transformedPos);
#else
	o.position = transformedPos;
#endif
	o.vertexPosition = float4(position, 1);
    return o;
}
