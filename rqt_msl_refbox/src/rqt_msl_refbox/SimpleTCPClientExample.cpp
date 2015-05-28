#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

int main(void)
{
	struct sockaddr_in stSockAddr;
	int Res;
	int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (-1 == SocketFD)
	{
		perror("cannot create socket");
		exit(EXIT_FAILURE);
	}

	memset(&stSockAddr, 0, sizeof(stSockAddr));

	stSockAddr.sin_family = AF_INET;
	stSockAddr.sin_port = htons(28097);
	Res = inet_pton(AF_INET, "127.0.0.1", &stSockAddr.sin_addr);

	if (0 > Res)
	{
		perror("error: first parameter is not a valid address family");
		close(SocketFD);
		exit(EXIT_FAILURE);
	}
	else if (0 == Res)
	{
		perror("char string (second parameter does not contain valid ipaddress)");
		close(SocketFD);
		exit(EXIT_FAILURE);
	}

	if (-1 == connect(SocketFD, (struct sockaddr *)&stSockAddr, sizeof(stSockAddr)))
	{
		perror("connect failed");
		close(SocketFD);
		exit(EXIT_FAILURE);
	}

	/* perform read write operations ... */
	char buffer[1024];
	while (true)
	{
		int received = read(SocketFD, (void *)buffer, 1024);
		if (received < 0)
		{
			strerror (errno);
			exit(EXIT_FAILURE);
		}
		printf("recsize: %d\n ", received);
		sleep(1);
		printf("datagram: %.*s\n", (int)received, buffer);
	}

	(void)shutdown(SocketFD, SHUT_RDWR);

	close(SocketFD);
	return EXIT_SUCCESS;
}
