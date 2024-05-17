#pragma once

#include <vector>
#include <string>
#include <asio.hpp>
#include "data_frame.h"

#define HALF_FLOATS
#define WITH_MOCKING

#ifdef HALF_FLOATS
typedef short vertex_type;
#else
typedef float vertex_type;
#endif

class rendering_client_async
{
public:
	rendering_client_async(std::string host, std::string port);
	~rendering_client_async();

	bool get_counts(int* nvertices, int* ntriangles);
	bool get_data(vertex_type* vertex_data, int* index_data, char* color_images);

#ifdef WITH_MOCKING
	bool mock_get_counts(int* nvertices, int* ntriangles);
	bool mock_get_data(vertex_type* vertex_data, int* index_data, char* color_images);
#endif

	static const int MAX_VERTEX_SIZE = 1000 * 1000 * 9 * sizeof(float);
	static const int MAX_INDEX_SIZE = 2000 * 1000 * 3 * sizeof(int);

private:
	boost::asio::io_service io_service_;
	boost::asio::ip::tcp::socket* socket_;

	static const int NPODS = 8;
	static const int WIDTH = 1024;
	static const int HEIGHT = 1024;

	TofinoMeshHeaderV1 header_;
	int color_size[NPODS];
	int header_frame_ = 0;
	int data_frame_ = 0;

	char* compressed_color_;
	float* compressed_vertex_;
	int* compressed_index_;
};
