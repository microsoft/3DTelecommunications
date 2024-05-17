#pragma once

#include <vector>
#include <string>
#include <asio.hpp>
#include "turbojpeg.h"
#include "data_frame.h"

#define HALF_FLOATS
// The following define is used to process YUV color images.  #undef this to process raw bayer RGB instead.
#define WITH_MOCKING

#ifdef HALF_FLOATS
typedef short vertex_type;
#else
typedef float vertex_type;
#endif

class rendering_client
{
public:
	rendering_client(std::string host, std::string port);
	~rendering_client();

	bool get_counts(int* nvertices, int* ntriangles);
	bool get_data(vertex_type* vertex_data, int* index_data, char* color_images, char* audioData, char* compressedColorData = NULL);
	bool get_is_MJPEG();
	bool get_is_HQ_Frame();
	int get_color_size(int podID);

#ifdef WITH_MOCKING
	bool mock_get_counts(int* nvertices, int* ntriangles);
	bool mock_get_data(vertex_type* vertex_data, int* index_data, char* color_images);
#endif

	void set_podCount(int newPodCount);
	void set_resolution(int width, int height, int colorPixelBytes);

	void set_audio_parameters(int sampleRate, int sampleSize);
	int get_audio_stream_count();
	int get_audio_stream_size(int streamID);

	void get_simplified_mesh(vertex_type *vertex_data, int * index_data, int* nVertices, int * nTriangles);
	void set_should_simplify(bool should);

	static const int MAX_VERTEX_SIZE = 1000 * 1000 * 9 * sizeof(float);
	static const int MAX_INDEX_SIZE = 2000 * 1000 * 3 * sizeof(int);

	const static int MAX_PODS = 16;

	static const int cResolutionWidth = 2048;
	static const int cResolutionHeight = 2048;


	int NPODS;
	int WIDTH;
	int HEIGHT;
    int COLOR_PIXEL_SIZE;
	int audioSampleRate;
	int audioSampleSize;
	int audioStreamCount;

	bool should_simplify;
	int headerVersion;

private:
	boost::asio::io_service io_service_;
	boost::asio::ip::tcp::socket* socket_;

	MeshHeaderVersion header_;

	int color_size[MAX_PODS];

	int audio_size[MAX_PODS];
	int header_frame_ = 0;
	int data_frame_ = 0;

	tjhandle _jpegDecompressor;

	char* compressed_color_;
	float* compressed_vertex_;
	int* compressed_index_;
	char* audioData;
};

#define DLLEXPORT __declspec( dllexport )

extern "C"
{
	DLLEXPORT int get_max_vertex_size();
	DLLEXPORT int get_max_index_size();

	DLLEXPORT void* create_rendering_client(const char* host, const char* port);
	DLLEXPORT void destroy_rendering_client(void* pointer);

	DLLEXPORT bool client_get_counts(void* pointer, int* nvertices, int* ntriangles);
	DLLEXPORT bool client_get_data(void* pointer, vertex_type* vertex_data, int* index_data, char* color_images, char* audio_data, char* compressedColorData = NULL);

	DLLEXPORT bool client_mock_get_counts(void* pointer, int* nvertices, int* ntriangles);
	DLLEXPORT bool client_mock_get_data(void* pointer, vertex_type* vertex_data, int* index_data, char* color_images);

	DLLEXPORT void client_set_pod_count(void* pointer, int podCount);
	DLLEXPORT void client_set_resolution(void* pointer, int width, int height, int colorPixelBytes);

	DLLEXPORT void client_get_simplified_mesh(void *pointer, vertex_type *vertex_data, int * index_data, int* nVertices, int * nTriangles);
	DLLEXPORT void client_set_should_simplify(void *pointer, bool should);

	DLLEXPORT void client_set_audio_parameters(void *pointer, int sampleRate, int sampleSize);
	DLLEXPORT int client_get_audio_stream_count(void *pointer);
	DLLEXPORT int client_get_audio_stream_size(void * pointer, int streamID);

	DLLEXPORT bool client_get_is_HQ_Frame(void* pointer);
	DLLEXPORT bool client_get_is_MJPEG(void* pointer);
	DLLEXPORT int client_get_color_size(void* pointer, int podID);
}
