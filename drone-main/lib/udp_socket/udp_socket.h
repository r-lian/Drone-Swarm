#ifndef UDP_SOCKET_H

#define UDP_SOCKET_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT    8080
#define MAXLINE 1024


// Driver code
class UDP_Position_Socket {
private:
	char *_ip_address;
	char *_port;
	float *buffer;
	int sockfd;
public:
	UDP_Socket();
	UDP_Socket(string ip_address, string port);

	virtual send(float x, float y, float z, float angle);
	virtual receive();
}
#endif
