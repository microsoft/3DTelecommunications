#include "rendering_client_async.h"
#include <iostream>
#include <fstream>
#include <lz4.h>

template <typename T>
int memory_read(T* data, size_t count, const std::string& filename)
{
	std::ifstream file(filename, std::ios::binary);
	file.read(reinterpret_cast<char*>(data), count * sizeof(T));
	int read_count = file.gcount();
	file.close();
	return read_count;
}

rendering_client_async::rendering_client_async(std::string host, std::string port)
{
	boost::asio::ip::tcp::resolver resolver(io_service_);
	boost::system::error_code ec;

	auto endpoint = resolver.resolve({ host, port }, ec);
	if (ec) std::cout << ec.value() << " " << ec.message() << std::endl;

	socket_ = new boost::asio::ip::tcp::socket(io_service_);
	boost::asio::connect(*socket_, endpoint, ec);
	if (ec)
		std::cout << ec.value() << " " << ec.message() << std::endl;
	else
		std::cout << "connected to " << socket_->remote_endpoint().address() << ":" << socket_->remote_endpoint().port() << std::endl;

	compressed_color_ = new char[WIDTH * HEIGHT];
	compressed_vertex_ = new float[MAX_VERTEX_SIZE];
	compressed_index_ = new int[MAX_INDEX_SIZE];
}

rendering_client_async::~rendering_client_async()
{
	delete socket_;
	delete[] compressed_color_;
	delete[] compressed_vertex_;
	delete[] compressed_index_;
}

#ifdef WITH_MOCKING
bool client_mock_get_counts(void* pointer, int* nvertices, int* ntriangles)
{
	auto client = reinterpret_cast<rendering_client_async*>(pointer);
	return client->mock_get_counts(nvertices, ntriangles);
}

bool client_mock_get_data(void* pointer, vertex_type* vertex_data, int* index_data, char* color_images)
{
	auto client = reinterpret_cast<rendering_client_async*>(pointer);
	return client->mock_get_data(vertex_data, index_data, color_images);
}

bool rendering_client_async::mock_get_counts(int* nvertices, int* ntriangles)
{
	header_.vertex_count = 308547;
	header_.tri_count = 102849;

	*nvertices = header_.vertex_count;
	*ntriangles = header_.tri_count;
	return true;
}

bool rendering_client_async::mock_get_data(vertex_type* vertex_data, int* index_data, char* color_images)
{
	std::string directory = "C:\\Users\\samehk\\Downloads\\mock_frame\\";

	memory_read(vertex_data, 6 * header_.vertex_count, directory + "data.bin");

	for (int i = 0; i < header_.tri_count * 3; i++)
		index_data[i] = i;

	for (int c = 0; c < NPODS; c++)
		memory_read(&color_images[WIDTH * HEIGHT * c], WIDTH * HEIGHT, directory + "bayer_color_" + std::to_string(c) + ".bin");

	return true;
}
#endif

bool rendering_client_async::get_counts(int* nvertices, int* ntriangles)
{
	if (header_frame_ != data_frame_)
		return false;

	size_t len;
	boost::system::error_code ec;

	int total_size;

	len = boost::asio::write(*socket_, boost::asio::buffer(&header_frame_, sizeof(int)), boost::asio::transfer_at_least(sizeof(int)), ec);

	len = boost::asio::read(*socket_, boost::asio::buffer(&total_size, sizeof(int)), boost::asio::transfer_exactly(sizeof(int)), ec);

	if (total_size == 0)
	{
		header_frame_++;
		return false;
	}

	len = boost::asio::read(*socket_, boost::asio::buffer(&header_, sizeof(TofinoMeshHeaderV1)), boost::asio::transfer_exactly(sizeof(TofinoMeshHeaderV1)), ec);
	len = boost::asio::read(*socket_, boost::asio::buffer(color_size, NPODS * sizeof(int)), boost::asio::transfer_exactly(NPODS * sizeof(int)), ec);

	*nvertices = header_.vertex_count;
	*ntriangles = header_.tri_count;

	header_frame_++;
	return true;
}


bool rendering_client_async::get_data(vertex_type* vertex_data, int* index_data, char* color_images)
{
	if (header_frame_ != data_frame_ + 1)
		return false;

	size_t len;
	boost::system::error_code ec;

	for (int c = 0; c < NPODS; c++)
	{
		if (header_.color_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_color_, WIDTH * HEIGHT * sizeof(char)), boost::asio::transfer_exactly(color_size[c]), ec);
			LZ4_decompress_fast(compressed_color_, &color_images[WIDTH * HEIGHT * c], WIDTH * HEIGHT * sizeof(char)); // Decompress but leave Bayered
		}
		else
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(&color_images[WIDTH * HEIGHT * c], WIDTH * HEIGHT * sizeof(char)), boost::asio::transfer_exactly(color_size[c]), ec);
		}
	}

	if (header_.vertex_data_size > 0)
	{
		if (header_.vertex_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_vertex_, header_.vertex_data_size), boost::asio::transfer_exactly(header_.vertex_data_size), ec);
			LZ4_decompress_fast(reinterpret_cast<char*>(compressed_vertex_), reinterpret_cast<char*>(vertex_data), header_.vertex_full_size_bytes); // Decompress
		}
		else
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(vertex_data, header_.vertex_data_size), boost::asio::transfer_exactly(header_.vertex_data_size), ec);
		}
	}

	if (header_.tri_data_size > 0)
	{
		if (header_.tri_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_index_, header_.tri_data_size), boost::asio::transfer_exactly(header_.tri_data_size), ec);
			LZ4_decompress_fast(reinterpret_cast<char*>(compressed_index_), reinterpret_cast<char*>(index_data), header_.tri_full_size_bytes); // Decompress
		}
		else
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(index_data, header_.tri_data_size), boost::asio::transfer_exactly(header_.tri_data_size), ec);
		}
	}

	data_frame_++;
	return true;
}
