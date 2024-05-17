#include "TCPclient.h"

TCPclient::TCPclient(std::string serverAddress, std::string serverport) :
m_IsConnectedToServer(false),
m_ReceivedIndex(0),
m_ParsedIndex(0)
{
	m_ServerAddress = serverAddress;
	m_ServerPort = serverport;
	for (int i = 0; i < RING_BUFFER_COUNT; i++)
	{
		m_ReceiveBufferSize[i] = 0;
	}
	m_Running = false;
}

TCPclient::~TCPclient()
{

}

bool TCPclient::HasNewData()
{
	return m_ParsedIndex != m_ReceivedIndex;
}

const char* TCPclient::GetNewData()
{
	const char* temp = &m_ReceiveBuffer[m_ParsedIndex][0];
	return temp;
}

void TCPclient::FinishedNewData()
{
	m_ReceiveBufferSize[m_ParsedIndex] = 0;
	m_ParsedIndex = (m_ParsedIndex + 1) % RING_BUFFER_COUNT;
}

unsigned int TCPclient::GetNewDataSize()
{
	return m_ReceiveBufferSize[m_ParsedIndex];
}

void TCPclient::Uninitialize()
{
	m_IsConnectedToServer = FALSE;
	m_Running = FALSE;
	WSACleanup();
	// don't join, the recv call is blocking and we'll get stuck here 
	m_ReceiveThread.join();
}

int TCPclient::Connect()
{
	WSADATA wsaData;
	int iResult;

	struct addrinfo *result = NULL;
	struct addrinfo *ptr = NULL;
	struct addrinfo hints;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return E_FAIL;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(m_ServerAddress.c_str(), m_ServerPort.c_str(), &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return E_FAIL;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return E_FAIL;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}
	freeaddrinfo(result);
	if (ConnectSocket != INVALID_SOCKET)
	{
		m_IsConnectedToServer = true;
	}
	else
	{
		m_IsConnectedToServer = false;
		return E_FAIL;
	}
	return 0;
}

int TCPclient::Initialize()
{
	int retval = Connect();
	m_Running = true;
	//start thread just receiving 
	m_ReceiveThread = std::thread([this]()
	{
		while (m_Running)
		{
			while (m_IsConnectedToServer)
			{
				int sentSize = 0;
				int actualReceivedSize = recv(ConnectSocket, (char *)&sentSize, sizeof(int), 0);
				if (actualReceivedSize != sizeof(int))
				{
					//!error
					printf("recv failed with error: %d\n", WSAGetLastError());
					m_IsConnectedToServer = false;  //probably a disconnect.  Re-connect to the encoder
				}
				sentSize = ntohl(sentSize);


				int totalReceivedLength = 0;
			
				if (m_ReceiveBufferSize[m_ReceivedIndex] != 0)
				{
					printf("Buffer overun %d %d \n", m_ReceivedIndex, m_ParsedIndex);
				}
				int currReceiveBufferLength = -1;
			//receive until we have the full packet
			do
			{								
					currReceiveBufferLength = recv(ConnectSocket, &m_ReceiveBuffer[m_ReceivedIndex][0] + totalReceivedLength, sentSize - totalReceivedLength, 0);
					totalReceivedLength += currReceiveBufferLength;
			} 
			while (sentSize > totalReceivedLength && currReceiveBufferLength >= 0);
			
				m_ReceiveBufferSize[m_ReceivedIndex] = totalReceivedLength;
				m_ReceivedIndex = (m_ReceivedIndex + 1) % RING_BUFFER_COUNT;
			if (totalReceivedLength != sentSize || totalReceivedLength < 0)
				{
					//!error
					printf("recv failed with error: %d\n", WSAGetLastError());
				}

			}
			if (!m_IsConnectedToServer && m_Running)
			{
				// Attempt to reconnect
				Connect();
			}
		}
	});
	return retval;
}



