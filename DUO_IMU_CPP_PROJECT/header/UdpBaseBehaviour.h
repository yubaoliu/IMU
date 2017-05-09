#ifndef _UDP_BASE_HEADER_H_
#define _UDP_BASE_HEADER_H_
#ifndef UNICODE
#define UNICODE
#endif

#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>

#pragma comment(lib, "Ws2_32.lib")
class Udp_base_behaviour {
private:
	SOCKET localSocket = INVALID_SOCKET;
	SOCKET remoteSocket = INVALID_SOCKET;
	sockaddr_in remoteAddr;
	sockaddr_in localAddr;
	unsigned short localPort = 8888;
	unsigned short remotePort = 6666;

public:
	bool Winsock_init()
	{
#ifdef _DEBUG
		wprintf(L"InitSocket fun must to be called firstly\n");
#endif
		WSADATA wsaData;
		int iResult;
		// Initialize Winsock

		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != NO_ERROR) {
			wprintf(L"WSAStartup failed with error: %d\n", iResult);
			return true;
		}
		else {
			return false;
		}
	}
	bool InitRemoteSocket_Addr_Port(const char* ipDotAllignedAddr, unsigned short port)
	{
		// Create a socket for sending data
		this->remoteSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (this->remoteSocket == INVALID_SOCKET) {
			wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return false;
		}
		this->remoteAddr.sin_family = AF_INET;
		this->remoteAddr.sin_port = htons(port);
		this->remoteAddr.sin_addr.s_addr = inet_addr(ipDotAllignedAddr);
		this->remotePort = port;
		return true;
	}
	bool InitLocalSocket_LocalAddr_Port(const char* ipDotAllignedAddr, unsigned short port)
	{
		this->localSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (this->localSocket == INVALID_SOCKET) {
			wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return false;
		}
		this->localAddr.sin_family = AF_INET;
		this->localAddr.sin_port = htons(port);
		this->localAddr.sin_addr.s_addr = inet_addr(ipDotAllignedAddr);
		this->localPort = port;
		return true;
	}


	int SendUdpPacket(const char* buf, int buf_len)
	{
		int numOfSent;
		// Send a datagram to the receiver
		wprintf(L"Sending a datagram to the receiver...\n");
		numOfSent = sendto(this->remoteSocket,
			buf, buf_len, 0, (SOCKADDR *)&this->remoteAddr, sizeof(this->remoteAddr));
		if (numOfSent == SOCKET_ERROR) {
			wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
			closesocket(this->remoteSocket);
			WSACleanup();
			return 0;
		}
		else {
			return numOfSent;
		}
	}
	~Udp_base_behaviour()
	{
		// When the application is finished sending, close the socket.
		wprintf(L"Finished sending. Closing socket.\n");
		if (localSocket != INVALID_SOCKET)
		{
			closesocket(localSocket);
		}

		if (remoteSocket != INVALID_SOCKET)
		{
			closesocket(remoteSocket);
		}

		WSACleanup();
	}

};
//main example
/*
int main()
{
	Udp_base_behaviour *udpClient = new Udp_base_behaviour();
	udpClient->Winsock_init();
	udpClient->InitRemoteSocket_Addr_Port("127.0.0.1", 27015);

	char SendBuf[1024] = "I am Client!";
	int BufLen = 1024;
	udpClient->SendUdpPacket(SendBuf, BufLen);

	return 0;
}
*/
#endif
