#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <vector>
#include <mutex>
#pragma comment(lib, "Ws2_32.lib")
//max possible size from a 4k x 4k stream
#define DEFAULT_BUFLEN 4 * 4 * 1024 * 1024 
#define RING_BUFFER_COUNT 100
class TCPclient
{
public:
	TCPclient(std::string serverAddress, std::string serverport);
	~TCPclient();
	int Initialize();
	void Uninitialize();
	bool HasNewData();
	const char* GetNewData();
	unsigned int GetNewDataSize();
	void FinishedNewData();

protected:
	bool m_Running;
	bool m_IsConnectedToServer;
	std::string m_ServerAddress;
	std::string m_ServerPort;

	SOCKET ConnectSocket = INVALID_SOCKET;
	std::thread m_ReceiveThread;

	char m_ReceiveBuffer[RING_BUFFER_COUNT][DEFAULT_BUFLEN];
	int m_ReceiveBufferSize[RING_BUFFER_COUNT];

	volatile unsigned int m_ReceivedIndex;
	volatile unsigned int m_ParsedIndex;

	int Connect();

};