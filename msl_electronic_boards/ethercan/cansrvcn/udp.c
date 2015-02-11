#include "udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


#include <errno.h>

#include <linux/cpc.h>
#include <fcntl.h>

#include "utils.h"

int sendSocket;
CPC_BUF cpcBuffer;


struct sockaddr_in serverAddr;
struct sockaddr_in clientAddr;
struct sockaddr *serverAddrCast  = (struct sockaddr *) &serverAddr;
struct sockaddr *clientAddrCast = (struct sockaddr *) &clientAddr;

int setupUdpSockets(int conf_port, char* ipstring) {
	struct hostent *hp = gethostbyname(ipstring);

	memcpy((char*)&clientAddr.sin_addr, (char*)hp->h_addr, hp->h_length);

	sendSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (sendSocket < 0) return -1;

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(conf_port);
	serverAddr.sin_addr.s_addr = INADDR_ANY; 

	clientAddr.sin_family = AF_INET;
	clientAddr.sin_port = htons(conf_port);

	cpcBuffer.cpcmsg = (CPC_MSG_T *)malloc(sizeof(CPC_MSG_T)*CPC_BUF_MAX);
	memset(cpcBuffer.cpcmsg, 0, sizeof(CPC_MSG_T)*CPC_BUF_MAX);
	cpcBuffer.iidx = 0;
	cpcBuffer.oidx = 0;
	cpcBuffer.WnR  = 1;	


	bind(sendSocket, serverAddrCast, sizeof(serverAddr));

	setFdNonBlocking(sendSocket);
	return sendSocket;
}

int sendUdp(unsigned char* message, int len){
	return sendto(sendSocket, message, len, 0, clientAddrCast, sizeof(clientAddr));
}
/*
int receiveUdp() {	
	return recv(sendSocket,udp_in_buffer,UDP_RECEIVE_BUFFER_SIZE,0);
}
*/
void processReadUdp() {
	static int cntme = 0;
	int result = -1;

	for(cntme = 0; cntme < 25; cntme++){
		result = recv(sendSocket,udp_in_buffer,UDP_RECEIVE_BUFFER_SIZE,0);
		if (result < 1) {
			break;
		}
		//printf("Read %d bytes\n",result);
		convertAndStore_UdpFrame(udp_in_buffer, result);
	}
	return;
}

int convertAndStore_UdpFrame(unsigned char *buf, int len)
{

	CPC_MSG_T *udp_to_cpc_msg = &cpcBuffer.cpcmsg[cpcBuffer.iidx];
	
	if(IsBufferFull(cpcBuffer)) {
		return 0;
	}

	if (len < 5) {
		return 0;
	}
	//printf("Valid\n");
	
	/* CPC_MSG type */
	if (buf[1] & (1<<3)) { //extended
		if(buf[4] & 0x01) { //RTR
			udp_to_cpc_msg->type = CPC_CMD_T_XRTR;
		} else {
			udp_to_cpc_msg->type = CPC_CMD_T_XCAN;
		}
		udp_to_cpc_msg->msg.canmsg.id = (((unsigned short)buf[2])<<8) | buf[3];
		udp_to_cpc_msg->msg.canmsg.id |= ((buf[0] >> 3) << 24) | ((buf[1] & 0x03) << 16) | 
					(((buf[1] & 0xE0) >> 3) << 16) | (((buf[0] & 0x07) << 5) << 16);
		
	
	} else { //not extended
		udp_to_cpc_msg->msg.canmsg.id = (((unsigned short)buf[2])<<8) | buf[3];


		if(buf[4] & 0x01) { //RTR
			udp_to_cpc_msg->type = CPC_CMD_T_RTR;
		} else {
			udp_to_cpc_msg->type = CPC_CMD_T_CAN;
		}
	}
	
	
	/* CPC_CAN_MSG length */	
	udp_to_cpc_msg->msg.canmsg.length = buf[4] >> 1;
	
	//DATA
	memcpy(udp_to_cpc_msg->msg.canmsg.msg, buf+5, udp_to_cpc_msg->msg.canmsg.length);
	


	udp_to_cpc_msg->length = udp_to_cpc_msg->msg.canmsg.length+5;
	
	//printf("id: %d length: %d type: %d\n",udp_to_cpc_msg->msg.canmsg.id,udp_to_cpc_msg->msg.canmsg.length,udp_to_cpc_msg->type);	

	/* CPC_MSG ts_sec */	
	/*
	tcp_to_cpc_msg->ts_sec = hex_to_u8(&tmpbufptr);
	tcp_to_cpc_msg->ts_sec |= hex_to_u8(&tmpbufptr)<<8;
	tcp_to_cpc_msg->ts_sec |= hex_to_u8(&tmpbufptr)<<16;
	tcp_to_cpc_msg->ts_sec |= hex_to_u8(&tmpbufptr)<<24;
	*/
	/* CPC_MSG ts_nsec */		
	/*
	tcp_to_cpc_msg->ts_nsec = hex_to_u8(&tmpbufptr);
	tcp_to_cpc_msg->ts_nsec |= hex_to_u8(&tmpbufptr)<<8;
	tcp_to_cpc_msg->ts_nsec |= hex_to_u8(&tmpbufptr)<<16;
	tcp_to_cpc_msg->ts_nsec |= hex_to_u8(&tmpbufptr)<<24;
	*/
	/* CPC_MSG data */	


	/* increment iidx */	
	cpcBuffer.iidx = (cpcBuffer.iidx +1) % CPC_BUF_MAX;
	cpcBuffer.WnR  = 0;

	return 1;
}


void processWriteCpc(int can)
{
	int ret;
//printf("pwc: i %d o %d wnr %d\n",cpcBuffer.iidx,cpcBuffer.oidx,cpcBuffer.WnR);
	while (IsBufferNotEmpty(cpcBuffer)){
		//printf("Buffer not empty\n");
		if((ret=write(can, &cpcBuffer.cpcmsg[cpcBuffer.oidx], sizeof(CPC_MSG_T))) >= 0) {
			
			//printCpcMessage(&cpcBuffer.cpcmsg[cpcBuffer.oidx], 0);
	
			cpcBuffer.oidx = (cpcBuffer.oidx+1) % CPC_BUF_MAX;
			cpcBuffer.WnR  = 1;
			cpcBuffer.oob  = 0;
		}
		//printf("Sent %d bytes\n",ret);
	}
	
	return;
}



