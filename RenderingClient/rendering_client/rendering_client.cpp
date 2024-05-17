#include "rendering_client.h"
#include "Simplify.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <lz4.h>

#undef DEBUG_SIMPLIFY



int get_max_vertex_size()
{
	return rendering_client::MAX_VERTEX_SIZE;
}

int get_max_index_size()
{
	return rendering_client::MAX_INDEX_SIZE;
}

void* create_rendering_client(const char* host, const char* port)
{
	rendering_client* client = new rendering_client(std::string(host), std::string(port));
	return client;
}

void destroy_rendering_client(void* pointer)
{
	rendering_client* client = reinterpret_cast<rendering_client*>(pointer);
	delete client;
}

bool client_get_counts(void* pointer, int* nvertices, int* ntriangles)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_counts(nvertices, ntriangles);
}

bool client_get_data(void* pointer, vertex_type* vertex_data, int* index_data, char* color_images, char* audio_data, char* compressedColorData)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_data(vertex_data, index_data, color_images, audio_data, compressedColorData);
}

int client_get_audio_stream_count(void *pointer)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_audio_stream_count();
}

int client_get_audio_stream_size(void * pointer, int streamID)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_audio_stream_size(streamID);
}

bool client_get_is_HQ_Frame(void* pointer)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_is_HQ_Frame();
}

bool client_get_is_MJPEG(void* pointer)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_is_MJPEG();
}

int client_get_color_size(void* pointer, int podID)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->get_color_size(podID);
}


template <typename T>
int memory_read(T* data, size_t count, const std::string& filename)
{
	std::ifstream file(filename, std::ios::binary);
	file.read(reinterpret_cast<char*>(data), count * sizeof(T));
	int read_count = file.gcount();
	file.close();
	return read_count;
}

void client_set_pod_count(void* pointer, int podCount)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	client->set_podCount(podCount);
}

void client_set_resolution(void* pointer, int width, int height, int colorPixelBytes)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
    client->set_resolution(width, height, colorPixelBytes);
}



void client_get_simplified_mesh(void *pointer, vertex_type *vertex_data, int * index_data, int* nVertices, int * nTriangles)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	client->get_simplified_mesh(vertex_data, index_data, nVertices, nTriangles);
}

void client_set_should_simplify(void *pointer, bool should)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	client->set_should_simplify(should);
}

void client_set_audio_parameters(void *pointer, int sampleRate, int sampleSize)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	client->set_audio_parameters(sampleRate, sampleSize);
}


size_t WriteWavHeader(std::ostream& outs, size_t channels, size_t bitsPerChannel, size_t sampleRate)
{
	outs.write("RIFF", 4);
	outs.write((const char*)& ((const int&)0), sizeof(int)); // empty size for now
	outs.write("WAVE", 4);
	outs.write("fmt ", 4);

	outs.write((const char*)&((const int&)bitsPerChannel), sizeof(int)); // 16 bits per channel

	outs.write((const char*)&((const int&)1), sizeof(short)); // PCM - integer samples

	outs.write((const char*)&((const int&)channels), sizeof(short)); // PCM - integer samples

	outs.write((const char*)&((const int&)sampleRate), sizeof(int)); // samples per second (Hz)

	outs.write((const char*)&((const int&)(sampleRate*channels*(bitsPerChannel / 8))), sizeof(int)); // (Sample Rate * BitsPerSample * Channels) / 8

	outs.write((const char*)&((const int&)(channels*(bitsPerChannel / 8))), sizeof(short)); // data block size (size of 16-bit samples, one for each channel, in bytes)

	outs.write((const char*)&((const int&)bitsPerChannel), sizeof(short)); // number of bits per sample (use a multiple of 8)

	// Write the data chunk header
	size_t data_chunk_pos = outs.tellp();
	outs.write("data", 4);  // (chunk size to be filled in later)
	size_t bufSize = 0;
	char* buffer[1];
	outs.write((const char*)&bufSize, 0);
	outs.write((const char*)buffer, 0);
	return data_chunk_pos;
}



rendering_client::rendering_client(std::string host, std::string port) :
WIDTH(0),
HEIGHT(0),
COLOR_PIXEL_SIZE(0),
NPODS(MAX_PODS),
should_simplify(false),
compressed_color_(nullptr),
headerVersion(1)
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
	
	compressed_vertex_ = new float[MAX_VERTEX_SIZE];
	compressed_index_ = new int[MAX_INDEX_SIZE];

	_jpegDecompressor = tjInitDecompress();

    set_resolution(cResolutionWidth, cResolutionHeight, sizeof(char)); // char --> defaulting support for YUV
}

rendering_client::~rendering_client()
{
	delete socket_;
	tjDestroy(_jpegDecompressor);
	delete[] compressed_color_;
	delete[] compressed_vertex_;
	delete[] compressed_index_;
}

#ifdef WITH_MOCKING
bool client_mock_get_counts(void* pointer, int* nvertices, int* ntriangles)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->mock_get_counts(nvertices, ntriangles);
}

bool client_mock_get_data(void* pointer, vertex_type* vertex_data, int* index_data, char* color_images)
{
	auto client = reinterpret_cast<rendering_client*>(pointer);
	return client->mock_get_data(vertex_data, index_data, color_images);
}

bool rendering_client::mock_get_counts(int* nvertices, int* ntriangles)
{
	header_.vertex_count = 308547;
	header_.tri_count = 102849;

	*nvertices = header_.vertex_count;
	*ntriangles = header_.tri_count;
	return true;
}

bool rendering_client::mock_get_data(vertex_type* vertex_data, int* index_data, char* color_images)
{
	std::string directory = "C:\\Users\\samehk\\Downloads\\mock_frame\\";

	memory_read(vertex_data, 6 * header_.vertex_count, directory + "data.bin");
	
	for (int i = 0; i < header_.tri_count *3; i++)
		index_data[i] = i;

	for (int c = 0; c < NPODS; c++)
		memory_read(&color_images[WIDTH * HEIGHT * c], WIDTH * HEIGHT, directory + "bayer_color_" + std::to_string(c) + ".bin");

	return true;
}
#endif

void rendering_client::set_audio_parameters(int sampleRate, int sampleSize)
{
	audioSampleSize = sampleSize;
	audioSampleRate = sampleRate;

	//we are using audio data, so we need V2
	headerVersion = 2;
}

int rendering_client::get_audio_stream_count()
{
	return header_.audio_data_count;
}

int rendering_client::get_audio_stream_size(int streamID)
{
	return audio_size[streamID];
}

void rendering_client::set_podCount(int newPodcount)
{
	NPODS = newPodcount;
}

void rendering_client::set_resolution(int width, int height, int colorPixelBytes)
{
	WIDTH = width;
	HEIGHT = height;
    COLOR_PIXEL_SIZE = colorPixelBytes;

    if (compressed_color_ != nullptr)
        delete[] compressed_color_;

    compressed_color_ = new char[WIDTH * HEIGHT * COLOR_PIXEL_SIZE];
}

bool rendering_client::get_counts(int* nvertices, int* ntriangles)
{
	if (header_frame_ != data_frame_)
		return false;

	size_t len;
	boost::system::error_code ec;

	int total_size;

	len = boost::asio::write(*socket_, boost::asio::buffer(&header_frame_, sizeof(int)), boost::asio::transfer_at_least(sizeof(int)), ec);
	if (len == 0)
		return false;

	len = boost::asio::read(*socket_, boost::asio::buffer(&total_size, sizeof(int)), boost::asio::transfer_exactly(sizeof(int)), ec);
	if (len == 0)
		return false;

	if (total_size == 0 || total_size == sizeof(int))
		return false;

	if (headerVersion == 2)
	{ 
		len = boost::asio::read(*socket_, boost::asio::buffer(&header_, sizeof(TofinoMeshHeaderV2)), boost::asio::transfer_exactly(sizeof(TofinoMeshHeaderV2)), ec);
	}
	else
	{
		len = boost::asio::read(*socket_, boost::asio::buffer(&header_, sizeof(TofinoMeshHeaderV1)), boost::asio::transfer_exactly(sizeof(TofinoMeshHeaderV1)), ec);
	}
	
	len = boost::asio::read(*socket_, boost::asio::buffer(color_size, NPODS * sizeof(int)), boost::asio::transfer_exactly(NPODS * sizeof(int)), ec);

	*nvertices = header_.vertex_count;
	*ntriangles = header_.tri_count;

	header_frame_ = header_.packet_header.frame_number; 
	return true;
}

bool rendering_client::get_is_HQ_Frame()
{
	return header_.vertex_data_format & TofinoCompress::HIGHQUALITY;
}

bool rendering_client::get_is_MJPEG()
{
	return header_.color_data_format & TofinoCompress::MJPEG;
}

int rendering_client::get_color_size(int podID)
{
	return color_size[podID];
}

int frameNum = 0;
int frameWriteInterval = 30;
size_t chunkPos = 0;

//HACK for READY demo: to always read the 4th stream (index = 3)  at the start
//TODO: smartly blend/pick audio from different channels
const int cStreamIndexToUseAtStart = 3;
bool rendering_client::get_data(vertex_type* vertex_data, int* index_data, char* color_images, char* audioData, char* rawColorData)
{
	if (header_frame_ == data_frame_)
		return false;

	size_t len;
	boost::system::error_code ec;

	for (int c = 0; c < NPODS; c++)
	{
		if (header_.color_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_color_, WIDTH * HEIGHT * COLOR_PIXEL_SIZE), boost::asio::transfer_exactly(color_size[c]), ec);
			if (len < color_size[c])
				return false;

			//fast method
            len = LZ4_decompress_fast(compressed_color_, &color_images[WIDTH * HEIGHT * c * COLOR_PIXEL_SIZE], WIDTH * HEIGHT * COLOR_PIXEL_SIZE); // Decompress but leave Bayered
		}  

		else if (header_.color_data_format & TofinoCompress::MJPEG)
		{
			//if user passed in a pointer to be filled out, we fill decode off of that. Otherwise, use internal array.
			unsigned char* encodedDataPointer = reinterpret_cast<unsigned char*>(compressed_color_);
			if (rawColorData != NULL)
			{
				encodedDataPointer = reinterpret_cast<unsigned char*>(rawColorData + WIDTH * HEIGHT * COLOR_PIXEL_SIZE * c);
			}
			len = boost::asio::read(*socket_, boost::asio::buffer(encodedDataPointer, WIDTH * HEIGHT * COLOR_PIXEL_SIZE), boost::asio::transfer_exactly(color_size[c]), ec);
			if (len < color_size[c])
				return false;

			int jpegSubsamp, width, height;			

			tjDecompressHeader2(_jpegDecompressor, encodedDataPointer, color_size[c], &width, &height, &jpegSubsamp);

			tjDecompress2(_jpegDecompressor, encodedDataPointer, color_size[c],
				reinterpret_cast<unsigned char*> (color_images) + c * WIDTH * HEIGHT * COLOR_PIXEL_SIZE, width,
				0/*pitch*/, height, TJPF_RGB, TJFLAG_FASTDCT);

		}
		else
		{

            len = boost::asio::read(*socket_, boost::asio::buffer(&color_images[WIDTH * HEIGHT * c * COLOR_PIXEL_SIZE], WIDTH * HEIGHT * COLOR_PIXEL_SIZE), boost::asio::transfer_exactly(color_size[c]), ec);
			if (len < color_size[c])
				return false;
		}
	}

	//audio read
	if (headerVersion == 2)
	{
		//read audio sizees
		len = boost::asio::read(*socket_, boost::asio::buffer(audio_size, header_.audio_data_count * sizeof(int)), boost::asio::transfer_exactly(header_.audio_data_count * sizeof(int)), ec);
		if (len < sizeof(int) * header_.audio_data_count)
		{
			return false;
		}

		//read actual audio data
		int audioOffset = 0;
		int fullAudioPacketSize = audioSampleRate * audioSampleSize;

		for (int i = 0; i < header_.audio_data_count; ++i)
		{
			//HACK for READY demo: to always read the 4th stream (index = 3)  at the start
			//TODO: smartly blend/pick audio from different channels
			if (i == cStreamIndexToUseAtStart)
			{
				audioOffset = 0;				
			}
			len = boost::asio::read(*socket_, boost::asio::buffer(audioData + audioOffset, fullAudioPacketSize), boost::asio::transfer_exactly(fullAudioPacketSize), ec);
			audioOffset += fullAudioPacketSize;
			if (len < fullAudioPacketSize)
				return false;
		}
	}
	

	if (header_.vertex_data_size > 0)
	{
		if (header_.vertex_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_vertex_, header_.vertex_data_size), boost::asio::transfer_exactly(header_.vertex_data_size), ec);
			if (len < header_.vertex_data_size)
				return false;
			len = LZ4_decompress_fast(reinterpret_cast<char*>(compressed_vertex_), reinterpret_cast<char*>(vertex_data), header_.vertex_full_size_bytes); // Decompress
		}
		else
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(vertex_data, header_.vertex_data_size), boost::asio::transfer_exactly(header_.vertex_data_size), ec);
			if (len < header_.vertex_data_size)
				return false;
		}
	}

	if (header_.tri_data_size > 0)
	{
		if (header_.tri_data_format & TofinoCompress::LZ4)
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(compressed_index_, header_.tri_data_size), boost::asio::transfer_exactly(header_.tri_data_size), ec);
			if (len < header_.tri_data_size)
				return false;
			len = LZ4_decompress_fast(reinterpret_cast<char*>(compressed_index_), reinterpret_cast<char*>(index_data), header_.tri_full_size_bytes); // Decompress
		}
		else
		{
			len = boost::asio::read(*socket_, boost::asio::buffer(index_data, header_.tri_data_size), boost::asio::transfer_exactly(header_.tri_data_size), ec);
			if (len < header_.tri_data_size)
				return false;
		}
	}

	data_frame_ = header_frame_;


	//mesh simplification	
	if (should_simplify)
	{ 
		//std::ofstream myfile("C:\\DebugPrintSimplify\\meshSimplification.txt");
		//myfile << "Mesh simplified Start" << std::endl;
		Simplify::load_from_rendering_client(vertex_data, index_data, header_.vertex_count, header_.tri_count);
		//myfile << "Mesh loaded" << std::endl;
		static const double agressiveness = 7.0;
		static const int target_count = 21000; //21k  = 63000 max vertices; Unity has 65k max vertex/index count
		Simplify::simplify_mesh(target_count, agressiveness, true);
		//myfile << "Mesh simplified End" << std::endl;
		//Simplify::write_obj("D:\\reduced.obj"); 
	}	
	return true;
}

void rendering_client::get_simplified_mesh(vertex_type *vertex_data, int * index_data, int* nVertices, int * nTriangles)
{
	Simplify::fetch_mesh(vertex_data, index_data, nVertices, nTriangles);
	//std::ofstream myfile("C:\\DebugPrintSimplify\\meshSimplification.txt", std::ofstream::out | std::ofstream::app);
	//myfile << "vCount: " << *nVertices << std::endl;
	//myfile << "tCount: " << *nTriangles << std::endl;
}

void rendering_client::set_should_simplify(bool should)
{
	should_simplify = should;
}