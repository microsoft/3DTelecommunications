#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <ncurses.h>
#include "ICompressor.h"

using boost::asio::ip::tcp;

class TCPServer
{
    public:
    	interface ICompressor* m_compressor;

      TCPServer(boost::asio::io_service& io_service, short port)
      : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
          socket_(io_service)
      {
        m_is_running = true;
        m_zero_buffer = (BYTE*)calloc(12, sizeof(BYTE));
        memset(m_zero_buffer, 0, 12);

        do_accept();                
      }
      ~TCPServer()
      {
        m_is_running = false;
        delete m_zero_buffer;
      }

      void SetInput(interface ICompressor* compressor);
      void Shutdown();
      DWORD ProcessRequest(DWORD requestedFrame, const BYTE** pData, DWORD* cbData);
      void SendComplete();
    private:
     tcp::acceptor acceptor_;
    public:
      tcp::socket socket_;
      DWORD m_lastFrameSent;

    protected:
      enum { eStopped, eStarted, eStopping } m_eState;
    private:
      void do_accept();

      BYTE* m_zero_buffer;
      DWORD m_lastFrameProcessed;

      DWORD m_swapBufferIndex;
      bool m_is_running; // this is used to indicate that the system is shutting down, so internal loops waiting for data should stop waiting
};

class TCPConnection
{
  public:
    TCPConnection(boost::asio::io_service& service);
    bool IsRunning();
    void Shutdown();
    void WorkerThread(boost::asio::io_service& service);
  protected:
    void StartThread();
    void StopThread();
    bool ThreadRunning;
    std::mutex ThreadMutex;
    std::thread processThread;
    boost::asio::io_service& _service;
};

class session
  : public std::enable_shared_from_this<session>
{
public:
  session(TCPServer* server)
    : socket_(std::move(server->socket_))
  {    
    server_ = server;
  }

  void start()
  {
    do_read();
  }

private:
  void do_read()
  {
    auto self(shared_from_this());
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        [this, self](boost::system::error_code ec, std::size_t length)
        {
          if (!ec)
          {
              // Here is where I would request that the decompressor give me the latest packet
              // and do a send
              int requestedFrame = ((int*)data_)[0];
              // pointers to get the data
              const BYTE* pData = NULL;
              DWORD cbData = 0;
              server_->ProcessRequest(requestedFrame, &pData, &cbData);
              DWORD actualSize = ((DWORD*)pData)[0];
              if(actualSize != cbData)
              {
                std::cout << "WARNING: Sizes are NOT EQUAL.  " << actualSize << " " << cbData << " this implies something is overwriting the packet you are about to send!  I will reset the size to be correct, but data loss may occur!" << std::endl;
                cbData = actualSize;
              }
              do_write(&pData, cbData);              
          }
        });
  }

  void do_write(const BYTE** data, size_t length)
  {
    auto self(shared_from_this());
    //std::cout << "[TCP]XMIT [" << ((int*)*data)[1] << "] [" << ((int*)*data)[0] << "] START" << std::endl;
    boost::asio::async_write(socket_, boost::asio::buffer(*data, length),
        [this, self](boost::system::error_code ec, std::size_t length)
        {
          //std::cout << "[TCP]XMIT [" << length << "] END" << std::endl;
          server_->SendComplete(); // releases the buffers
          if (!ec)
          {
            do_read();
          }
          else 
          {
            std::cout << "Send error\r\n";
          }
        });
  }

  TCPServer* server_;
  tcp::socket socket_;
  enum { max_length = 1024 };
  char data_[max_length];
};

