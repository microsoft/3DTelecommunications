// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

#if VOXELIZATION
#define total_verts_count 14
[maxvertexcount(total_verts_count)]
void geom(point v2f p[1], inout TriangleStream<v2f> triStream) {
	v2f outputG2F;
	UNITY_INITIALIZE_OUTPUT(v2f, outputG2F);

	float3 cube_vertices[8] = {
		float3(-1,  1, -1),
		float3(1,  1, -1),
		float3(-1,  1,  1),
		float3(1,  1,  1),
		float3(-1, -1, -1),
		float3(1, -1, -1),
		float3(1, -1,  1),
		float3(-1, -1,  1)
	};

	int indices[total_verts_count] = {
		4, 3, 7, 8, 5, 3, 1, 4, 2, 7, 6, 5, 2, 1
	};

	outputG2F = p[0];

#if !VOXEL_SNAP_IN_VERTEX_SHADER
	float3 position = p[0].position.xyz;
	position += 0.0001;
	float4 snapped_position = float4(round(position / voxel_size) * voxel_size, 1);
#else
	float4 snapped_position = p[0].position;
#endif

	for (int i = 0; i < total_verts_count; ++i) {
		//outputG2F = p[0]; 
		outputG2F.position = UnityObjectToClipPos(snapped_position + float4(0.5f * voxel_size * cube_vertices[indices[i] - 1], 0));
		triStream.Append(outputG2F);
	}
	triStream.RestartStrip();
}
#endif	