#include <winsock2.h>
#include <ws2tcpip.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")
#pragma once
#define DEFAULT_BUFLEN 512
#define IPV4_ADDRESS_LENGTH 15
#define PRINT_BITRATE
class TCPserver
{
public:
	TCPserver(std::string serverport);
	~TCPserver();
	int Initialize();
	int SendData(const char * sendBuffer, int sendBufferLength);
	void Uninitialize();
	int GetConnectedClients();

protected:
	bool m_IsServerLive;
	std::string m_ServerPort;

	SOCKET ListenSocket = INVALID_SOCKET;
	std::vector<SOCKET> m_ClientSockets;


	std::thread m_AcceptThread;

	int m_TemporarySentAmount;
	std::chrono::time_point<std::chrono::system_clock> m_StartTime;
};