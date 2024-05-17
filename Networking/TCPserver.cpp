#include "TCPServer.h"

TCPserver::TCPserver(std::string serverPort) :m_TemporarySentAmount(0)
{
	m_ServerPort = serverPort;
}

TCPserver::~TCPserver()
{

}

//1 if failed
int TCPserver::Initialize()
{
	HRESULT hr = S_OK;
	WSADATA wsaData;
	int iResult;

	struct addrinfo *result = NULL;
	struct addrinfo hints;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return E_FAIL;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	iResult = getaddrinfo(NULL, m_ServerPort.c_str(), &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return E_FAIL;
	}

	// Create a SOCKET for connecting to server
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return E_FAIL;
	}
	// Setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return E_FAIL;
	}
	freeaddrinfo(result);
	
	m_IsServerLive = true;

	m_AcceptThread = std::thread([this]()
	{
		while (m_IsServerLive)
		{

			// blocking call
			int iResult = listen(ListenSocket, SOMAXCONN);
			if (iResult == SOCKET_ERROR) {
				printf("listen failed with error: %d\n", WSAGetLastError());
				closesocket(ListenSocket);
				WSACleanup();
				break;
			}

			// Accept a client socket
			SOCKADDR_IN client_info = { 0 };
			int addrsize = sizeof(client_info);
			SOCKET ClientSocket = accept(ListenSocket, (struct sockaddr*)&client_info, &addrsize);
			if (ClientSocket == INVALID_SOCKET) {
				printf("accept failed with error: %d\n", WSAGetLastError());
				closesocket(ListenSocket);
				WSACleanup();
				break;
			}
			else
			{
				char ip[15];
				inet_ntop(AF_INET, &(client_info.sin_addr), ip, IPV4_ADDRESS_LENGTH);
				printf("Accepted client %s\n", ip);
				m_ClientSockets.push_back(ClientSocket);
			}

		}
	});
	m_StartTime = std::chrono::system_clock::now();
	return hr;
}

void TCPserver::Uninitialize()
{
	m_IsServerLive = false;
	closesocket(ListenSocket);
	int count = m_ClientSockets.size();
	for (int i = 0; i < count; i++)
	{
		int result = shutdown(m_ClientSockets[i], SD_SEND);
		if (result == SOCKET_ERROR) {
			printf("shutdown failed with error: %d\n", WSAGetLastError());
			closesocket(m_ClientSockets[i]);
			WSACleanup();
		}
		closesocket(m_ClientSockets[i]);
	}
	m_ClientSockets.clear();

	WSACleanup();

	m_AcceptThread.join();
}

int TCPserver::SendData(const char * sendBuffer, int sendBufferLength)
{
	//printf("sendBufferLength %d", sendBufferLength);
	int socketLength = m_ClientSockets.size();
	for (int i = 0; i < socketLength; i++)
	{
		//send size then payload
		int convInt = htonl(sendBufferLength);
		int sendResult = send(m_ClientSockets[i], (const char*)&convInt, sizeof(int), 0);
		if (sendResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(m_ClientSockets[i]);
			WSACleanup();
			return 1;
		}
		int totalSent = 0;
		do{
			sendResult = send(m_ClientSockets[i], sendBuffer + totalSent, sendBufferLength - totalSent, 0);
			if (sendResult == SOCKET_ERROR) {
				printf("send failed with error: %d\n", WSAGetLastError());
				closesocket(m_ClientSockets[i]);
				WSACleanup();
				return 1;
			}
			totalSent += sendResult;
		} while (totalSent < sendBufferLength);
	}

#ifdef PRINT_BITRATE
	//calculating send bitrate
	std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - m_StartTime;
	if (elapsed_seconds.count() >=3.0)
	{
		printf("byte/sec: %f \n", m_TemporarySentAmount / elapsed_seconds.count());
		m_StartTime = std::chrono::system_clock::now();
		m_TemporarySentAmount = 0;
	}
	m_TemporarySentAmount += sendBufferLength;
#endif
	return 0;
}

int TCPserver::GetConnectedClients()
{
	return m_ClientSockets.size();
}
