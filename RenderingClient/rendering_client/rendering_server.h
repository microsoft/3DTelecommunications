#pragma once

#include <string>
#include <asio.hpp>
#include <thread>

class rendering_server
{
public:
	rendering_server(std::string port);
	~rendering_server();

	void start();
	void stop();

private:
	static void thread_start(asio::io_service* io_service, asio::ip::tcp::acceptor* acceptor, const bool& exit);

	volatile bool exit_;
	asio::io_service io_service_;
	asio::ip::tcp::acceptor* acceptor_;
	std::thread worker_;
};
