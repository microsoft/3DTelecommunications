namespace McMeshProcessingInternals
{
	//
	// Basic math.
	//
#if !defined(__CUDACC__)
	__forceinline float rsqrtf(float x) { return 1.0f / sqrtf(x); }
#endif

	template <typename T> __forceinline__ __host__ __device__ T min(T a, T b) { return a < b ? a : b; }
	template <typename T> __forceinline__ __host__ __device__ T max(T a, T b) { return a > b ? a : b; }
	template <typename T> __forceinline__ __host__ __device__ T clamp(T x, T mi, T mx) { return min(max(x, mi), mx); }
	__forceinline__ __host__ __device__ float3 min(float3 a, float3 b) { return make_float3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)); }
	__forceinline__ __host__ __device__ float3 max(float3 a, float3 b) { return make_float3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)); }
	__forceinline__ __host__ __device__ float3 cross(float3 a, float3 b) { return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); }
	__forceinline__ __host__ __device__ float dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	__forceinline__ __host__ __device__ float lengthSquared(float3 a) { return dot(a, a); }
	__forceinline__ __host__ __device__ float3 add(float3 a, float3 b) { return make_float3(a.x + b.x, a.y + b.y, a.z + b.z); }
	__forceinline__ __host__ __device__ float3 sub(float3 a, float3 b) { return make_float3(a.x - b.x, a.y - b.y, a.z - b.z); }
	__forceinline__ __host__ __device__ float3 mul(float3 a, float3 b) { return make_float3(a.x * b.x, a.y * b.y, a.z * b.z); }
	__forceinline__ __host__ __device__ float3 mul(float3 a, float b) { return make_float3(a.x * b, a.y * b, a.z * b); }
	__forceinline__ __host__ __device__ bool isValid(float v) { return !(isnan(v) || isinf(v)); }
	__forceinline__ __host__ __device__ bool isValid(float3 v) { return isValid(v.x) && isValid(v.y) && isValid(v.z); }
	__forceinline__ __host__ __device__ uint16_t floatToHalf(float x) { union { float f; int i; } u; u.f = x * (1.0f + 1.0f / 256.f); return uint16_t(u.i >> 16); }
	__forceinline__ __host__ __device__ float halfToFloat(uint32_t x) { union { int i; float f; } u; u.i = int(x << 16); return u.f; }

	//
	// CPU Parallel.
	//
	struct Parallel
	{
		enum { MAX_THREADS = 6, MAX_THREADS_M1 = MAX_THREADS - 1 };

		struct Range
		{
			__forceinline__ Range(int start, int end) : m_start(start), m_end(end) {}
			__forceinline__ Range begin() const { return *this; }
			__forceinline__ Range end() const { return Range(m_end, m_end); }
			__forceinline__ Range& operator++() { m_start++; return *this; }
			__forceinline__ bool operator !=(const Range& other) const { return m_start != other.m_start || m_end != other.m_end; }
			__forceinline__ int operator*() const { return m_start; }
			int m_start, m_end;
		};
		
		template <typename FUNCTOR>
		static void forEach(int numItems, const FUNCTOR& functor)
		{
			const int blockSize = numItems / MAX_THREADS;
			if (blockSize > 1)
			{
				std::future<void> res[MAX_THREADS_M1];
				for (int i = 0; i < MAX_THREADS_M1; ++i)
				{
					res[i] = std::async(std::launch::async, [&](int block) { const int blockStart = blockSize * block; functor(Range(blockStart, blockStart + blockSize)); }, i);
				}

				functor(Range(blockSize*MAX_THREADS_M1, numItems));

				for (auto& r : res) r.get();
			}
			else
			{
				functor(Range(0, numItems));
			}
		}
	};

	//
	// Functions.
	//

	// Compute collapse data from triangle and returns its error.
	__forceinline __host__ __device__ uint16_t computeCollapseData(
		const float3 positionA, const float3 normalA,
		const float3 positionB, const float3 normalB,
		const float3 positionC, const float3 normalC,
		McMeshProcessing::TriangleCollapse& output)
	{
		const float3 triangleCentroid = mul(add(add(positionA, positionB), positionC), 1 / 3.0f);
		const float offsetA = dot(normalA, positionA);
		const float offsetB = dot(normalB, positionB);
		const float offsetC = dot(normalC, positionC);
		const float3 projectionA = sub(triangleCentroid, mul(normalA, dot(triangleCentroid, normalA) - offsetA));
		const float3 projectionB = sub(triangleCentroid, mul(normalB, dot(triangleCentroid, normalB) - offsetB));
		const float3 projectionC = sub(triangleCentroid, mul(normalC, dot(triangleCentroid, normalC) - offsetC));
		const float3 averageProjection = mul(add(add(projectionA, projectionB), projectionC), 1 / 3.0f);
		const float error = dot(cross(sub(positionB, positionA), sub(positionC, positionA)), sub(averageProjection, positionA));

		output.m_position = averageProjection;
		output.m_normal = add(add(normalA, normalB), normalC);
		return floatToHalf(error * error);
	}

	// Traverse cluster linked-list to root and returns the cluster id.	
	__forceinline __host__ int getCluster(McMeshProcessing::Cluster* __restrict clusters, int cluster)
	{
		const int baseCluster = cluster;
		while (clusters[cluster].m_index != cluster) cluster = clusters[cluster].m_index;
		clusters[baseCluster].m_index = cluster;
		return cluster;
	}

	// https://devblogs.nvidia.com/thinking-parallel-part-iii-tree-construction-gpu/
	// Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
	__forceinline __host__ unsigned int expandBits(unsigned int v)
	{
		v = clamp<unsigned int>(v, 0, 1023);
		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	// Radix sort of elements.
	template <int BITS, typename T, typename GET_RADIX>
	__forceinline __host__ void   radixSort(
		const T* __restrict const itemsIn, T* __restrict const itemsOut,
		int count, const GET_RADIX& getRadix)
	{
		const int n = 1 << BITS;
		unsigned h[n]; memset(h, 0, sizeof(h));
		for (int i = 0; i < count; ++i)		++h[getRadix(itemsIn[i])];
		for (int i = 0, j = 0; i < n; ++i)	{ const unsigned c = h[i] + j; h[i] = j; j = c; }
		for (int i = 0; i < count; ++i)		itemsOut[h[getRadix(itemsIn[i])]++] = itemsIn[i];
	}
	
	//
	// Kernels.
	//

	// Used by simplify.
	__global__ void computePerTriangleErrorKernel(
		int numTriangles,
		McMeshProcessing::TriangleError* __restrict trianglesErrorOut,
		McMeshProcessing::TriangleCollapse* __restrict collapseOut,
		const McMeshProcessing::StridedPointer<float3> positions,
		const McMeshProcessing::StridedPointer<float3> normals,
		const McMeshProcessing::StridedPointer<int3> triangles)
	{
		const int i = blockIdx.x * blockDim.x + threadIdx.x;
		const int3 indices = triangles[i];

		trianglesErrorOut[i].m_error = computeCollapseData(
			positions[indices.x], normals[indices.x],
			positions[indices.y], normals[indices.y],
			positions[indices.z], normals[indices.z],
			collapseOut[i]);
		trianglesErrorOut[i].m_value = i;
	}
};

using namespace McMeshProcessingInternals;

__host__ void McMeshProcessing::initialize(MemoryModel memoryModel, int maxTriangles, int maxVertices)
{
	assert(memoryModel == MEMORY_MODEL_HOST || memoryModel == MEMORY_MODEL_DEVICE_MANAGED);
	PERF_SCOPE("McMeshProcessing::initialize");
	m_memoryModel = memoryModel;
	m_maxTriangles = maxTriangles;
	m_maxVertices = maxVertices;

	int max_vertex_size_needed = maxVertices * 6 * sizeof(short) * sizeof(float);
	int max_index_size_needed = maxTriangles * 3 * sizeof(int);

	switch (m_memoryModel)
	{
	case MEMORY_MODEL_HOST:
		m_triangleErrors = new TriangleError[maxTriangles * 2];
		m_triangleCollapse = new TriangleCollapse[maxTriangles];

		//host model needs some place for gpu->cpu copy

		cudaMallocHost(&mPinnedVerts, max_vertex_size_needed);
		cudaMallocHost(&mPinnedInds, max_index_size_needed);

		break;

	case MEMORY_MODEL_DEVICE_MANAGED:
		checkCudaErrors(cudaMallocManaged(&m_triangleErrors, 2 * maxTriangles * sizeof(*m_triangleErrors)));
		checkCudaErrors(cudaMallocManaged(&m_triangleCollapse, maxTriangles * sizeof(*m_triangleCollapse)));
		break;

	default:
		assert(false);
	}
	cudaDeviceSynchronize();

	// Common.
	m_vertexMap = new int[maxTriangles * 3];
	m_locks = new bool[maxTriangles * 3];
	m_halfEdgeKeys = new uint64_t[maxTriangles * 3];
	m_halfEdgeValues = new int[maxTriangles * 3];
}

__host__ void McMeshProcessing::release()
{
	switch (m_memoryModel)
	{
	case MEMORY_MODEL_HOST:
		delete[] m_triangleErrors;
		delete[] m_triangleCollapse;
		break;

	case MEMORY_MODEL_DEVICE_MANAGED:
		cudaFree(m_triangleErrors);
		cudaFree(m_triangleCollapse);
		break;
	}

	// Common.
	delete[] m_vertexMap;
	delete[] m_locks;
	delete[] m_halfEdgeKeys;
	delete[] m_halfEdgeValues;
}

// Simplify mesh using triangle collapse.
// - Compute per-triangle error.
// - Sort by increasing error.
// - Traverse the list and collapse each triangle who's vertices was not locked by a previous collpase.
__host__ int McMeshProcessing::simplify(
	float maxError,
	int numVertices,
	int numTriangles,
	StridedPointer<float3> positions,
	StridedPointer<float3> normals,
	StridedPointer<int3> triangles,
	int minTriangles)
{
	PERF_SCOPE("simplify");
	assert(numTriangles <= m_maxTriangles);
	assert(triangles.m_stride == sizeof(int3)); // Custom triangle not implemented.

	// Early exit.
	if (numTriangles <= minTriangles) return numTriangles;

	const bool useCuda = m_memoryModel == MEMORY_MODEL_DEVICE_MANAGED;

	// Initialize vertex map to identity and vertex unlock.
	// Done in the background while processing triangles.
	auto initializeAsync = std::async([&]()
	{
		for (int i = 0; i < numVertices; ++i)
		{
			m_vertexMap[i] = i;
			m_locks[i] = false;
		}
	});

	// Compute triangles error and collapse data.	
	{
		PERF_SCOPE("buildError");
		if (useCuda)
		{
			const int blockSize = 1024;
			const int numBlocks = (numTriangles + blockSize - 1) / blockSize;
			computePerTriangleErrorKernel << < numBlocks, blockSize >> >(
				numTriangles,
				m_triangleErrors,
				m_triangleCollapse,
				positions,
				normals,
				triangles);
			cudaDeviceSynchronize();
			checkCudaErrors(cudaGetLastError());
		}
		else
		{
			Parallel::forEach(numTriangles, [&](Parallel::Range range)
			{
				for (int index : range)
				{
					const int3 triangle = triangles[index];
					m_triangleErrors[index].m_value = index;
					m_triangleErrors[index].m_error = computeCollapseData(
						positions[triangle.x], normals[triangle.x],
						positions[triangle.y], normals[triangle.y],
						positions[triangle.z], normals[triangle.z],
						m_triangleCollapse[index]);
				}
			});
		}
	}

	// Sort triangle errors.
	{
		PERF_SCOPE("sort");
		radixSort<8>(m_triangleErrors, m_triangleErrors + numTriangles, numTriangles, [&](const TriangleError& e) -> unsigned { return e.m_error & 0xff; });
		radixSort<8>(m_triangleErrors + numTriangles, m_triangleErrors, numTriangles, [&](const TriangleError& e)-> unsigned { return (e.m_error >> 8) & 0xff; });
	}

	// Wait for vertex initialization to complete.
	initializeAsync.wait();

	// Collapse triangles.		
	{
		PERF_SCOPE("collapse");
		int trianglesLeft = numTriangles;
		const uint16_t maxError16 = floatToHalf(maxError);
		for (int i = 0; i < numTriangles; ++i)
		{
			if (m_triangleErrors[i].m_error > maxError16) break;
			const int index = m_triangleErrors[i].m_value;
			const int3 ti = triangles[index];
			if (!(m_locks[ti.x] || m_locks[ti.y] || m_locks[ti.z]))
			{
				const TriangleCollapse& tc = m_triangleCollapse[index];

				// Lock vertices.
				m_locks[ti.x] = true;
				m_locks[ti.y] = true;
				m_locks[ti.z] = true;

				// Update vertex map so that vertices ti[x,y,z] all point to the same index (ti.x).
				m_vertexMap[ti.y] = ti.x;
				m_vertexMap[ti.z] = ti.x;

				// Set new position and normal.
				positions[ti.x] = tc.m_position;
				normals[ti.x] = mul(tc.m_normal, rsqrtf(lengthSquared(tc.m_normal)));

				// Triangle collapse can removed up to 4 triangles, but collapsing a triangle on the boundary would remove only 3 triangles,
				// Here we assume 3 triangles removed for each collapse so as to make 'minTriangles' concervative.
				trianglesLeft -= 3;
				if (trianglesLeft <= minTriangles) break;
			}
		}
	}

	{
		// Apply mapping and compact triangles.
		PERF_SCOPE("remap");
		int numActualTriangles = 0;
		for (int i = 0; i < numTriangles; ++i)
		{
			int3 indices = triangles[i];
			indices.x = m_vertexMap[indices.x];
			indices.y = m_vertexMap[indices.y];
			indices.z = m_vertexMap[indices.z];
			if (indices.x != indices.y && indices.y != indices.z && indices.z != indices.x)
			{
				triangles[numActualTriangles++] = indices;
			}
		}
		numTriangles = numActualTriangles;
	}
	return numTriangles;
}

__host__ int McMeshProcessing::trianglesToQuads(int numTriangles, const StridedPointer<int3> triangles, const StridedPointer<float3> positions, StridedPointer<int4> quadsOut, int minTriangles)
{
	PERF_SCOPE("trianglesToQuads");
	assert(numTriangles <= m_maxTriangles);

	const int bitsPerIndex = 24;

	// Build all half-edges and reset locks.
	{
		PERF_SCOPE("buildEdges");
		const auto buildHalfEdgeKey = [&](int a, int b) -> uint64_t
		{
			// Make sure half edge [A-B] has exactly the same key as half edge [B-A].
			if (a > b) { const int t = a; a = b; b = t; }

			// Initialize the most signification part of the key with edge length^2 as half, subtracted from 0xffff to allow descending order sorting.
			const float lengthSq = lengthSquared(sub(positions[a], positions[b]));
			uint64_t key = 0xffff - floatToHalf(lengthSq);
			key = key << (bitsPerIndex * 2);

			// Set the least significant part of the key to vertex sorted indices.
			key |= (uint64_t(a) << bitsPerIndex) | b;

			return key;
		};
		Parallel::forEach(numTriangles, [&](Parallel::Range range)
		{
			for (int i : range)
			{
				const int j = i << 2;
				const int k = i * 3;
				const int3 indices = triangles[i];
				m_halfEdgeKeys[k + 0] = buildHalfEdgeKey(indices.x, indices.y); m_halfEdgeValues[k + 0] = j | 0;
				m_halfEdgeKeys[k + 1] = buildHalfEdgeKey(indices.y, indices.z); m_halfEdgeValues[k + 1] = j | 1;
				m_halfEdgeKeys[k + 2] = buildHalfEdgeKey(indices.z, indices.x); m_halfEdgeValues[k + 2] = j | 2;
				m_locks[i] = false;
			}
		});
	}

	// Sort half edges.
	{
		PERF_SCOPE("sort");
		thrust::sort_by_key(thrust::cpp::par, m_halfEdgeKeys, m_halfEdgeKeys + numTriangles * 3, m_halfEdgeValues);
	}

	// Handle connected surface patches if minTriangles is greater than zero.
	if (minTriangles > 0)
	{
		PERF_SCOPE("cluster");
		std::vector<Cluster> clusters(numTriangles);

		// Initialize clusters.
		for (int i = 0; i < numTriangles; ++i)
		{
			clusters[i].m_index = i;
			clusters[i].m_count = 0;
		}

		// Build clusters.
		for (int i = 0, n = numTriangles * 3 - 1; i < n; ++i)
		{
			// Matching edge?
			if (m_halfEdgeKeys[i] == m_halfEdgeKeys[i + 1])
			{
				const int t0 = m_halfEdgeValues[i] >> 2;
				const int t1 = m_halfEdgeValues[i + 1] >> 2;
				// Different triangles?
				if (t0 != t1)
				{
					// Merge clusters.
					clusters[getCluster(clusters.data(), t0)].m_index = getCluster(clusters.data(), t1);
				}
			}
		}
		
		// Compute clusters number of triangles.
		for (int i = 0; i < numTriangles; ++i)
		{
			clusters[getCluster(clusters.data(), i)].m_count++;
		}
		
		// Disable triangles that belong to clusters made of less than 'minTriangles' triangles.
		for (int i = 0; i < numTriangles; ++i)
		{
			const int cluster = getCluster(clusters.data(), i);
			if (clusters[cluster].m_count < minTriangles) m_locks[i] = true;
		}
	}

	// Create quads.
	int  numQuads = 0;
	{
		PERF_SCOPE("createQuads");
		const int numHalfEdges = numTriangles * 3;
		for (int i = 0, n = numHalfEdges - 1; i < n; ++i)
		{
			// Edge vertices matches?
			if (m_halfEdgeKeys[i] == m_halfEdgeKeys[i + 1])
			{
				const int vI = m_halfEdgeValues[i];
				const int vJ = m_halfEdgeValues[i + 1];
				const int tI = vI >> 2;
				const int tJ = vJ >> 2;
				// Triangles are unlocked and different ?
				if (!m_locks[tI] && !m_locks[tJ] && tI != tJ)
				{
					const int* iI = &triangles[tI].x;
					const int* iJ = &triangles[tJ].x;
					const int sI = vI & 3;
					const int sJ = vJ & 3;
					// Half edges are opposite from each other ?
					if (iI[sI] != iJ[sJ])
					{
						quadsOut[numQuads++] = make_int4(iJ[sJ], iI[(sI + 2) % 3], iI[sI], iJ[(sJ + 2) % 3]);
						m_locks[tI] = true;
						m_locks[tJ] = true;
						i++;
					}
				}
			}
		}

		// Left over triangles as degenerated quads.
		for (int i = 0; i < numTriangles; ++i)
		{
			if (m_locks[i]) continue;
			const int3 I = triangles[i];
			quadsOut[numQuads++] = make_int4(I.x, I.y, I.z, I.z);
		}
	}

	return numQuads;
}

__host__ int McMeshProcessing::quadsToTriangles(int numQuads, const StridedPointer<int4> quadsIn, StridedPointer<int3> trianglesOut)
{
	const int numTriangles = numQuads * 2;
	thrust::for_each_n(thrust::cpp::par, thrust::counting_iterator<int>(0), numQuads, [&](int i)
	{
		const int4 quad = quadsIn[i];
		trianglesOut[i * 2 + 0] = make_int3(quad.x, quad.y, quad.z);
		trianglesOut[i * 2 + 1] = make_int3(quad.x, quad.z, quad.w);
	});

	return numTriangles;
}

__host__ int McMeshProcessing::removeUnreferencedVertices(int numTriangles, int numVertices, StridedPointer<int3> triangles, StridedPointer<uchar1> vertices)
{
	PERF_SCOPE("removeUnreferencedVertices");
	std::vector<int> indices(numVertices, 0);
	// Mark used vertices.
	{
		PERF_SCOPE("mark");
		thrust::for_each_n(thrust::cpp::par, thrust::counting_iterator<int>(0), numTriangles, [&](int i)
		{
			const auto triangle = triangles[i];
			indices[triangle.x] = 1;
			indices[triangle.y] = 1;
			indices[triangle.z] = 1;
		});
	}

	// Compact and assign indexes.
	int nextVertex = 0;
	{
		PERF_SCOPE("compact");
		for (int i = 0; i < numVertices; ++i)
		{
			if (indices[i])
			{
				const int index = nextVertex++;
				memcpy(&vertices[index], &vertices[i], vertices.m_stride);
				indices[i] = index;
			}
		}
	}

	// Apply mapping to triangles.
	{
		PERF_SCOPE("remap");
		thrust::for_each_n(thrust::cpp::par, thrust::counting_iterator<int>(0), numTriangles, [&](int i)
		{
			auto& triangle = triangles[i];
			triangle.x = indices[triangle.x];
			triangle.y = indices[triangle.y];
			triangle.z = indices[triangle.z];
		});
	}

	return nextVertex;
}

__host__ void McMeshProcessing::mortonSortTriangles(int numVertices, int numTriangles, StridedPointer<int3> triangles, StridedPointer<float3> positions)
{
	PERF_SCOPE("mortonSortTriangles");

	assert(triangles.m_stride == sizeof(int3)); // Custom triangle not implemented.

	// Compute bounds.
	float3 mins = positions[0], maxs = positions[0];
	for (int i = 1; i < numVertices; ++i)
	{
		const float3 p = positions[i];
		mins = min(mins, p);
		maxs = max(maxs, p);
	}

	const float3 extents = sub(maxs, mins);
	const float3 scale = make_float3(1023 / extents.x, 1023 / extents.y, 1023 / extents.z);

	// Compute keys (borrow m_halfEdgeKeys for key storage).
	uint32_t* __restrict keys = reinterpret_cast<uint32_t*>(m_halfEdgeKeys);
	Parallel::forEach(numTriangles, [&](Parallel::Range range)
	{
		for (int i : range)
		{
			const int3 triangle = triangles[i];
			const float3 center = mul(add(add(positions[triangle.x], positions[triangle.y]), positions[triangle.z]), 1 / 3.0f);
			const float3 scaledCenter = mul(sub(center, mins), scale);
			keys[i] =
				expandBits((unsigned int)(scaledCenter.x + 0.5f)) * 4 +
				expandBits((unsigned int)(scaledCenter.y + 0.5f)) * 2 +
				expandBits((unsigned int)(scaledCenter.z + 0.5f));
		}
	});

	// Sort.
	thrust::sort_by_key(thrust::cpp::par, keys, keys + numTriangles, triangles.begin());
}

__host__ void McMeshProcessing::mortonSortQuads(int numVertices, int numQuads, StridedPointer<int4> quads, StridedPointer<float3> positions)
{
	PERF_SCOPE("mortonSortQuads");

	assert(quads.m_stride == sizeof(int4)); // Custom quad not implemented.

	// Compute bounds.
	float3 mins = positions[0], maxs = positions[0];
	{
		PERF_SCOPE("bounds");
		for (int i = 1; i < numVertices; ++i)
		{
			const float3 p = positions[i];
			mins = min(mins, p);
			maxs = max(maxs, p);
		}
	}

	const float3 extents = sub(maxs, mins);
	const float3 scale = make_float3(1023 / extents.x, 1023 / extents.y, 1023 / extents.z);

	// Compute keys (borrow m_halfEdgeKeys for key storage).
	uint32_t* __restrict keys = reinterpret_cast<uint32_t*>(m_halfEdgeKeys);
	{
		PERF_SCOPE("keys");
		Parallel::forEach(numQuads, [&](Parallel::Range range)
		{
			for (int i : range)
			{
				const int4 quad = quads[i];
				const float3 center = mul(add(add(positions[quad.x], positions[quad.y]), add(positions[quad.z], positions[quad.w])), 1 / 4.0f);
				const float3 scaledCenter = mul(sub(center, mins), scale);
				keys[i] =
					expandBits((unsigned int)(scaledCenter.x + 0.5f)) * 4 +
					expandBits((unsigned int)(scaledCenter.y + 0.5f)) * 2 +
					expandBits((unsigned int)(scaledCenter.z + 0.5f));
			}
		});
	}

	// Sort.
	{
		PERF_SCOPE("sort");
		thrust::sort_by_key(thrust::cpp::par, keys, keys + numQuads, quads.begin());
	}
}

__host__ bool McMeshProcessing::checkValid(int numVertices, int numTriangles, StridedPointer<int3> triangles, StridedPointer<float3> positions)
{
	bool valid = true;
	for (int i = 0; i < numVertices; ++i)
	{
		const float3 position = positions[i];
		if (!isValid(position))
		{
			valid = false;
			printf("Invalid vertex position: %d\r\n", i);
		}
	}

	for (int i = 0; i < numTriangles; ++i)
	{
		const int3 indices = triangles[i];
		if (indices.x < 0 || indices.x >= numVertices || indices.y < 0 || indices.y >= numVertices || indices.z < 0 || indices.z >= numVertices)
		{
			valid = false;
			printf("Invalid triangle: %d\r\n", i);
		}
	}
	return valid;
}
