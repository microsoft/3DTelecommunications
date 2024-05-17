// Set to 1 to enable profiling instrumentation (the PERF_SCOPE macro).

//
// Marching cube mesh processing utilities.
//
struct McMeshProcessing
{
	//
	// Instrumentation.
	//

#define PERF_SCOPE(_name_)

	/// Memory model.
	enum MemoryModel
	{
		MEMORY_MODEL_UNDEFINED,
	
		// Use host memory and CPU only implementation.
		MEMORY_MODEL_HOST,

		// Use managed memory and CUDA implementation where possible (experimental).
		MEMORY_MODEL_DEVICE_MANAGED
	};

	/// Strided pointer.
	template <typename T>
	struct StridedPointer
	{
		__forceinline StridedPointer() :m_firstElement(nullptr), m_stride(0) {}
		__forceinline StridedPointer(void* firstElement, int stride) : m_firstElement((uchar1*)firstElement), m_stride(stride) {}
		__forceinline __host__ __device__ const T& operator[](int i) const { return *reinterpret_cast<const T*>(m_firstElement + m_stride * i); }
		__forceinline __host__ __device__ T& operator[](int i) { return *reinterpret_cast<T*>(m_firstElement + m_stride * i); }
		__forceinline __host__ __device__ const T* begin() const { return reinterpret_cast<const T*>(m_firstElement); }
		__forceinline __host__ __device__ T* begin() { return reinterpret_cast<T*>(m_firstElement); }
		uchar1* m_firstElement;
		const int m_stride;
	};

	/// Triangle error.
	struct TriangleError { uint16_t m_error; int m_value; };

	/// Triangle collapse data.
	struct TriangleCollapse { float3 m_position, m_normal; };
	
	/// Cluster data.
	struct Cluster { int m_index, m_count; };

	/// Initialize an instance of McMeshProcessing.
	__host__ void initialize(MemoryModel memoryModel, int maxTriangles, int maxVertices);

	/// Release an instance of McMeshProcessing.
	__host__ void release();

	/// In-place simplification of a mesh given its vertices position and normal, returns the number of triangle of the simplified mesh.
	/// Decimates about 50% of triangles per-call (assuming maxError = FLT_MAX).
	/// Notes:
	// - Unreferenced vertices are not removed by this method.
	// - Vertices normals are updated and do not need to be recomputed.	
	// - MinTriangles (default 0) can be used to stop the simplification when the number of triangles left falls below that value.
	__host__ int simplify(float maxError, int numVertices, int numTriangles, StridedPointer<float3> positions, StridedPointer<float3> normals, StridedPointer<int3> triangles, int minTriangles = 0);
	
	/// Create quads from a triangular mesh, returns the number of quads found.
	/// Notes:
	/// - quads out must be large enought to accomodate number of triangles quads (the worth case).	
	/// - if minTriangles is greater than zero, only convert connected surface patches made of more than minTriangles will be included.
	__host__ int trianglesToQuads(int numTriangles, const StridedPointer<int3> triangles, const StridedPointer<float3> positions, StridedPointer<int4> quadsOut, int minTriangles = 0);
	
	/// Create triangles from quads, returns the number of triangles (always numTriangles * 2).
	__host__ int quadsToTriangles(int numQuads, const StridedPointer<int4> quadsIn, StridedPointer<int3> trianglesOut);

	/// Remove unreferenced vertices.
	/// Returns the number of vertices left after removal.
	__host__ int removeUnreferencedVertices(int numVertices, int numTriangles, StridedPointer<int3> triangles, StridedPointer<uchar1> vertices);

	/// Sort triangles along a zcurve.
	__host__ void mortonSortTriangles(int numVertices, int numTriangles, StridedPointer<int3> triangles, StridedPointer<float3> positions);

	/// Sort quads along a zcurve.
	__host__ void mortonSortQuads(int numVertices, int numQuads, StridedPointer<int4> quads, StridedPointer<float3> positions);

	/// Check if a mesh is valid.
	__host__ bool checkValid(int numVertices, int numTriangles, StridedPointer<int3> triangles, StridedPointer<float3> positions);

	MemoryModel m_memoryModel = MEMORY_MODEL_UNDEFINED;
	int m_maxTriangles = 0;
	int m_maxVertices = 0;

	TriangleError* m_triangleErrors = nullptr;
	TriangleCollapse* m_triangleCollapse = nullptr;

	int* m_vertexMap = nullptr;
	bool* m_locks = nullptr;
	uint64_t* m_halfEdgeKeys = nullptr;
	int* m_halfEdgeValues = nullptr;


	//host mem
	char* mPinnedInds;
	char* mPinnedVerts;
};