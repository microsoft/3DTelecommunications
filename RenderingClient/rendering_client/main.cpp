#include "rendering_client.h"
#include "rendering_server.h"
#include <iostream>
#include <fstream>

template <typename T>
void memory_dump(const T* data, size_t count, const std::string& filename)
{
	std::ofstream file(filename, std::ios::binary);
	file.write(reinterpret_cast<const char*>(data), count * sizeof(T));
	file.close();
}

int main(int argc, char* argv[])
{
	vertex_type* vertex;
	int* index;
	char* color;
	vertex = new vertex_type[get_max_vertex_size()];
	index = new int[get_max_index_size()];
	color = new char[rendering_client::NPODS * rendering_client::WIDTH * rendering_client::HEIGHT];
	int nvertices, ntriangles;

	//rendering_server server("68999");
	//server.start();

	auto host = "10.137.62.165"; // Phil
	//auto host = "10.137.62.142"; // Sergio
	//auto host = "localhost";
	auto client = create_rendering_client(host, "62015");

	for (int i = 0; i < 1000; i++)
	{
		if (client_get_counts(client, &nvertices, &ntriangles))
		{
			if (client_get_data(client, vertex, index, color))
			{
				memory_dump(vertex, nvertices * 6, "C:\\users\\samehk\\Downloads\\dump\\vertices" + std::to_string(i) + ".txt");
				memory_dump(index, ntriangles * 3, "C:\\users\\samehk\\Downloads\\dump\\faces" + std::to_string(i) + ".txt");

				for (int c = 0; c < rendering_client::NPODS; c++)
					memory_dump(&color[rendering_client::WIDTH * rendering_client::HEIGHT * c], rendering_client::WIDTH * rendering_client::HEIGHT,
					"C:\\users\\samehk\\Downloads\\dump\\color" + std::to_string(i) + "_" + std::to_string(c) + ".txt");
			}
		}
	}

	destroy_rendering_client(client);

	//server.stop();


	delete[] vertex;
	delete[] index;
	delete[] color;
	return 0;
}
