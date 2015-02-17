#ifndef UDP_H

#define UDP_H 1


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <errno.h>

#include <linux/cpc.h>

#define UDP_RECEIVE_BUFFER_SIZE 1024
unsigned char udp_in_buffer[UDP_RECEIVE_BUFFER_SIZE];

/* tcp buffer size */
#define MAX_CONV_MSG_BUF_SIZE      56
#define MAX_RECV_TCPFRAME_BUF_SIZE 1024

/* buffer size for converted UDP-Frames */
#define CPC_BUF_MAX 1500

/* buffer holding converted UDP-Frames */
typedef struct _CPC_BUF{
	unsigned int iidx; // input pointer
	unsigned int oidx; // output pointer
	unsigned char WnR; // Write not Read
	unsigned char oob; // out of band flag
	CPC_MSG_T *cpcmsg; // pointer to buffer
}CPC_BUF;


int setupUdpSockets(int conf_port, char* ipstring);
int sendUdp(unsigned char* message, int len);
//int receiveUdp();
void processReadUdp();
int convertAndStore_UdpFrame(unsigned char *buf, int len);
void processWriteCpc(int can);


#endif
