#include "rendering_server.h"
#include <iostream>

rendering_server::rendering_server(std::string port)
{
	int port_int = std::atoi(port.c_str());
	try
	{
		acceptor_ = new asio::ip::tcp::acceptor(io_service_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port_int));
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	// TODO double buffer mesh and color?
}

rendering_server::~rendering_server()
{
	delete acceptor_;
}

void rendering_server::start()
{
	exit_ = false;
	worker_ = std::thread(&rendering_server::thread_start, &io_service_, acceptor_, exit_);
}

void rendering_server::stop()
{
	exit_ = true;
}

void rendering_server::thread_start(asio::io_service* io_service, asio::ip::tcp::acceptor* acceptor, const bool& exit)
{
	int frame;
	while (!exit)
	{
		size_t len, transfer_size;
		asio::error_code ec;

		try
		{
			asio::ip::tcp::socket socket(*io_service);
			acceptor->accept(socket);

			len = asio::read(socket, asio::buffer(&frame, sizeof(int)), asio::transfer_exactly(sizeof(int)), ec);

			len = asio::write(socket, asio::buffer(&frame, sizeof(int)), asio::transfer_at_least(sizeof(int)), ec);
			// send total_size first
			// send mesh_size
			// send mesh data into mesh
			// send color_size
			// send color data into color
		}
		catch (std::exception& e)
		{
			std::cerr << e.what() << std::endl;
		}
	}
}
