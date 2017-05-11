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
	SOCKET s = INVALID_SOCKET;

	sockaddr_in remoteAddr;
	int remoteAddr_size;
	sockaddr_in localAddr;
	int localAddr_size;
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
			return false;
		}
		this->s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (this->s == INVALID_SOCKET) {
			wprintf(L"socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return false;
		}
		
	}
	bool InitRemoteSocket_Addr_Port(const char* ipDotAllignedAddr, unsigned short port)
	{
		this->remoteAddr.sin_family = AF_INET;
		this->remoteAddr.sin_port = htons(port);
		this->remoteAddr.sin_addr.s_addr = inet_addr(ipDotAllignedAddr);
		this->remotePort = port;
		this->remoteAddr_size = sizeof(this->remoteAddr);
		return true;
	}
	bool InitLocalSocket_LocalAddr_Port(const char* ipDotAllignedAddr, unsigned short port)
	{
		this->localAddr.sin_family = AF_INET;
		this->localAddr.sin_port = htons(port);
		this->localAddr.sin_addr.s_addr = inet_addr(ipDotAllignedAddr);
		this->localPort = port;
		this->localAddr_size = sizeof(this->localAddr);

		int iResult = bind(this->s, (SOCKADDR *)& this->localAddr, sizeof(this->localAddr_size));
		if (iResult != 0) {
			wprintf(L"bind failed with error %d\n", WSAGetLastError());
			return false;
		}
		else {
			return true;
		}
	}


	int SendUdpPacket(const char* buf, int buf_len)
	{
		int numOfSent;
		// Send a datagram to the receiver
	//	wprintf(L"Sending a datagram to the receiver...\n");
		numOfSent = sendto(this->s,
			buf, buf_len, 0, (SOCKADDR *)&this->remoteAddr, sizeof(this->remoteAddr));
		if (numOfSent == SOCKET_ERROR) {
			wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
			closesocket(this->s);
			WSACleanup();
			return 0;
		}
		else {
			return numOfSent;
		}
	}
	int ReceivePacket(char *RecvBuf, int BufLen)
	{
		int iResult;
		//iResult=recv()
		iResult = recvfrom(s,RecvBuf, BufLen, 0, (SOCKADDR *)& this->remoteAddr, &this->remoteAddr_size);
		if (iResult == SOCKET_ERROR) {
			wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
			return -1;
		}
		return iResult;
	}
	~Udp_base_behaviour()
	{
		// When the application is finished sending, close the socket.
		wprintf(L"Finished sending. Closing socket.\n");
		if (s != INVALID_SOCKET)
		{
			closesocket(s);
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
