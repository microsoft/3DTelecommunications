//
// async_tcp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2017 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include "TCPServer.h"

void TCPServer::SetInput(interface ICompressor* compressor)
{
    std::cout << "[TCPServer] Compressor set" << std::endl;
    m_compressor = compressor;
}

void TCPServer::do_accept()
{
  acceptor_.async_accept(socket_,
      [this](boost::system::error_code ec)
      {
        if (!ec)
        {
            std::cout << "Accepted a new client on the socket." << std::endl;
          std::make_shared<session>(std::move(this))->start();
        }

        do_accept();
      });
}

void TCPServer::Shutdown()
{
  std::cout << "TCP Server shutdown.  Deleting memory...";
  m_is_running = false;
  delete m_zero_buffer;
  std::cout << "Done.  Closing acceptor...";
  acceptor_.close();  
  std::cout << "Done." << std::endl;
}

DWORD TCPServer::ProcessRequest(DWORD requestedFrame, const BYTE** pData, DWORD* cbData)
{
  //std::cout << "[" << std::this_thread::get_id() << "]PR" << std::endl;
  DWORD dwFrameNumber = m_lastFrameSent;
  int idx = -1;
  while(dwFrameNumber == m_lastFrameSent && m_is_running)
  {
    idx = m_compressor->GetLatestPacket(pData, cbData, &dwFrameNumber);
    if(dwFrameNumber == m_lastFrameSent && idx != -1)
    {
      // I got the same frame that I've already transmitted,
      // so mark the packet as released and continue to poll for a fresh frame
      m_compressor->ReleaseLastFetchedPacket(idx);
    }
  }
  if( idx < 0)
  {
    std::cout << "GetLatestPacket returned FAIL with error " << m_swapBufferIndex << "\r\n";
    goto bailout;
  }

  m_swapBufferIndex = idx;
  // Requested frame is greater than our most recently captured frame
  if(dwFrameNumber < requestedFrame)
  {
    std::cout << "Sending a zero frame because latest frame number [" << dwFrameNumber <<"] < [" << requestedFrame << "]\r\n";
    // Send a zero to tell the system that there is no new frame available
    pData = (const BYTE**)&m_zero_buffer;
    *cbData = 12;
  }
  else
  {    
    m_lastFrameProcessed = dwFrameNumber;
    //pData and cbData should be correct already thanks to the compressor
  }

  //std::cout << "[ProcessRequest] Packet[0] = " << ((int*)*pData)[0] << " cbData = " << *cbData << std::endl;

bailout:
  return dwFrameNumber;
}

void TCPServer::SendComplete()
{
  m_lastFrameSent = m_lastFrameProcessed;  //update the latest sent frame
  m_compressor->ReleaseLastFetchedPacket(m_swapBufferIndex);
}

TCPConnection::TCPConnection(boost::asio::io_service& service):_service(service)
{
  StartThread();
  processThread = std::thread(&TCPConnection::WorkerThread, this, std::ref(service));
}

bool TCPConnection::IsRunning()
{
    const std::lock_guard<std::mutex> lock(ThreadMutex);
    return ThreadRunning;
}
void TCPConnection::StartThread()
{
    const std::lock_guard<std::mutex> lock(ThreadMutex);
    ThreadRunning = true;
}

void TCPConnection::Shutdown()
{  
  std::cout << "TCPConnector calling stopthread...";
  StopThread();
  std::cout << "Done.  Stopping service...";
  _service.stop();  // We need to properly stop the service before trying to join threads, because it might be inside a poll 
  std::cout << "Done.  Joining threads....";
  if(processThread.joinable())
    processThread.join();
  else
    std::cout << "[TCP] thread was not joinable during shutdown." << std::endl;
  std::cout << "Done." << std::endl;
}

void TCPConnection::StopThread()
{
    const std::lock_guard<std::mutex> lock(ThreadMutex);
    ThreadRunning = false;
}

void TCPConnection::WorkerThread(boost::asio::io_service& service)
{
  boost::asio::io_service::work work(service);
  service.run();
  std::cout << "TCPConnection WorkerThread exiting." << std::endl;
}


#undef STANDALONE
#ifdef STANDALONE
  
int main(int argc, char* argv[])
{
    // ncurses setup
    initscr();
    cbreak();
    noecho();
    scrollok(stdscr, TRUE);
    nodelay(stdscr, TRUE);

  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: async_tcp_echo_server <port>\n";
      return 1;
      //endwin();
    }

    boost::asio::io_service io_service;

    TCPServer s(io_service, std::atoi(argv[1]));

    StartThread();
    std::thread worker = std::thread(WorkerThread, std::ref(io_service));
    
    bool running = true;
    while (running)
    {
        char c = getch();
            switch (c)
            {
            case 'q':
                std::cout << "Quitting." << std::endl;
                running = false;
                break;
            }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    //endwin();

    StopThread();
    worker.join();

  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
#endif