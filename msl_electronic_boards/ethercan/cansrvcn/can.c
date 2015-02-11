/*     I N C L U D E S
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/socket.h>

#include <errno.h>

#include <linux/cpc.h>

#include "utils.h"
#include "can.h"
#include "udp.h"
#include "main.h"
CPC_MSG_T lastWrittenParams;

UDP_FRAME udpBuffer;

int cpcCounter = 0;

extern int recvCpcCnt;
extern int sendUdpCnt;


/******************************************************************************/
/*     F U N C T I O N S
*/


/*******************************************************************************
* Function:			 initCan
*
* Task:          Open device and handle errors
*
* Parameters:    char *dev: device path
*
* Return values: int: file descriptor
*******************************************************************************/
int initCan(const char *dev)
{
	int fd = open(dev, O_RDWR);
	
	if(fd > 0){
		if(udpBuffer.udpFrames == NULL)
			udpBuffer.udpFrames = (UDP_FRAMES *)malloc(sizeof(UDP_FRAMES)*UDP_BUF_MAX);
		memset(udpBuffer.udpFrames, 0, sizeof(UDP_FRAMES)*UDP_BUF_MAX);
		setFdNonBlocking(fd);
		udpBuffer.iidx = 0;
		udpBuffer.oidx = 0;
		udpBuffer.WnR  = 1;
		return fd;
	}
	else{
		perror("ERROR: Could not open CAN device");
		return -1;
	}
		
	return fd;
}

/*******************************************************************************
* Function:			 processReadCpc
*
* Task:          Check for CAN-Message and call "convert and store"
*
* Parameters:    int can_handle
*
* Return values: none
*******************************************************************************/
void processReadCpc(int can_handle)
{
	int result = -1;
	int readcnt, i = 0;
	CPC_MSG_T received_message;

	do{
		if(IsBufferFull(udpBuffer)) {
			break;
		}
		readcnt = read(can_handle, &received_message, sizeof(CPC_MSG_T)); /* show if a message is in the buffer */

		if(readcnt == 0){ /* if no message is in the buffer return */
			return;
		}
		/*
		if(received_message.type == CPC_MSG_T_CAN || received_message.type == CPC_MSG_T_RTR) {
			id = cfg.data.s.id[received_message.msg.canmsg.id];
			if(id & 0x800){
				continue;
			}
		}
		*/
		if(received_message.type == CPC_MSG_T_CANSTATE) {
			if(received_message.msg.canstate & 0x80) {
				isBusOff = 1;
			}
		}
		
		recvCpcCnt++;
		//if(showCPCMessages)
		//printCpcMessage(&received_message, 1);
		result = convertAndStore_CpcMessage(&received_message,readcnt); /* Call ConvertMessageTo... */
	} while(++i < 50);

	return;
}

/*******************************************************************************
* Function:			 printCpcMessage
*
* Task:          prints a cpc message
*
* Parameters:    CPC_MSG_T *srcBuf: the CAN-Message, unsigned char from: incoming or outgoing
*
* Return values: none
*******************************************************************************/
void printCpcMessage(CPC_MSG_T *srcBuf, unsigned char from)
{
	int i;
	
	if(from)
		printf("--> Incoming CPC-Message ");
	else
		printf("--> Outgoing CPC-Message ");

	printf("%d ", srcBuf->type);
	
	for(i= 0; i < srcBuf->length; i++) {
		printf("%2.2X ", srcBuf->msg.generic[i]);
	}
	printf("\n");
}

/*******************************************************************************
* Function:			 convertAndStore_CpcMessage
*
* Task:          Converts a CAN-Message to and UDP-Frame
*
* Parameters:    CPC_MSG_T *srcBuf: the CAN-Message
*
* Return values: int: 0 success, -1 send error
*******************************************************************************/
int convertAndStore_CpcMessage(const CPC_MSG_T *srcBuf, int len)
{
	unsigned char *udpbufptr;

	if(IsBufferFull(udpBuffer)) {
		perror("UDP Buffer is Full!");
		return 1;
	}

	cpcCounter++;

	udpbufptr = &udpBuffer.udpFrames[udpBuffer.iidx].data[0];
	//memcpy(udpbufptr,srcBuf,len);

	switch(srcBuf->type){
		case CPC_MSG_T_CAN:
			*udpbufptr++ = 0x0;
			*udpbufptr++ = 0x0;
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 8) & 0x7;
			*udpbufptr++ = (char)srcBuf->msg.canmsg.id & 0xFF;
			*udpbufptr++ = (srcBuf->msg.canmsg.length << 1) & 0xFE;
			memcpy(udpbufptr,srcBuf->msg.canmsg.msg,srcBuf->msg.canmsg.length);
			break;
		case CPC_MSG_T_XCAN:
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 21) & 0xFF;
			*udpbufptr++ = ((char)(srcBuf->msg.canmsg.id >> 16) & 0x03) | 0x0C |
					(((char)(srcBuf->msg.canmsg.id >> 16) & 0xFC) << 3);
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 8) & 0xFF;
			*udpbufptr++ = (char)srcBuf->msg.canmsg.id & 0xFF;	
			*udpbufptr++ = (srcBuf->msg.canmsg.length << 1) & 0xFE;
			memcpy(udpbufptr,srcBuf->msg.canmsg.msg,srcBuf->msg.canmsg.length);
			break;
		case CPC_MSG_T_RTR:
			*udpbufptr++ = 0x0;
			*udpbufptr++ = 0x0;
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 8) & 0x7;
			*udpbufptr++ = (char)srcBuf->msg.canmsg.id & 0xFF;
			*udpbufptr++ = ((srcBuf->msg.canmsg.length << 1) & 0xFE) | 0x01;
			memcpy(udpbufptr,srcBuf->msg.canmsg.msg,srcBuf->msg.canmsg.length);
			break;
		case CPC_MSG_T_XRTR:
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 21) & 0xFF;
			*udpbufptr++ = ((char)(srcBuf->msg.canmsg.id >> 16) & 0x03) | 0x0C |
					(((char)(srcBuf->msg.canmsg.id >> 16) & 0xFC) << 3);
			*udpbufptr++ = (char)(srcBuf->msg.canmsg.id >> 8) & 0xFF;
			*udpbufptr++ = (char)srcBuf->msg.canmsg.id & 0xFF;
			*udpbufptr++ = ((srcBuf->msg.canmsg.length << 1) & 0xFE) | 0x01;
			memcpy(udpbufptr,srcBuf->msg.canmsg.msg,srcBuf->msg.canmsg.length);
			break;		
		default:
			cpcCounter--;
			return 0;

	}
	udpBuffer.udpFrames[udpBuffer.iidx].length = srcBuf->msg.canmsg.length + 5;
	//udpBuffer.udpFrames[udpBuffer.iidx].length = strlen((char *)udpBuffer.udpFrames[udpBuffer.iidx].data);
	/*
	*udpbufptr++ = 'S';
	
	
	udpbufptr = u8_to_hex(udpbufptr,srcBuf->type);
	
	udpbufptr = u8_to_hex(udpbufptr,srcBuf->length);
	udpbufptr = u8_to_hex(udpbufptr,srcBuf->msgid);
	
	udpbufptr = u8_to_hex(udpbufptr,srcBuf->ts_sec);
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_sec >> 8));
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_sec >> 16));
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_sec >> 24));
	
	udpbufptr = u8_to_hex(udpbufptr,srcBuf->ts_nsec);
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_nsec >> 8));
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_nsec >> 16));
	udpbufptr = u8_to_hex(udpbufptr,(srcBuf->ts_nsec >> 24));
	*/
	/* Convert CAN parameters by hand to avoid alignment problems */
	/*if(srcBuf->type == CPC_MSG_T_CAN_PRMS) {
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_type);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.mode);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_code0);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_code1);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_code2);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_code3);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_mask0);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_mask1);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_mask2);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.acc_mask3);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.btr0);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.btr1);
		udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.canparams.cc_params.sja1000.outp_contr);
	} else {
		for(i= 0; i < srcBuf->length; i++)
			udpbufptr = u8_to_hex(udpbufptr,srcBuf->msg.generic[i]);
	}
	*/
	//*udpbufptr++ = 'T'; /* we sign the end of the udp frame */
	//*udpbufptr++ = '\0';
	
	

	udpBuffer.iidx = (udpBuffer.iidx +1) % UDP_BUF_MAX;
	udpBuffer.WnR  = 0;
	//printf("Put stuff into buffer iidx: %d\n",udpBuffer.iidx);
	return 0;
}

/*******************************************************************************
* Function:			 processWriteUdp
*
* Task:          try to discharge the buffer
*
* Parameters:    int remote_socket
*
* Return values: none
*******************************************************************************/
void processWriteUdp()
{
  static unsigned bytesSent = 0;
  signed int      retSend, i;
  
//printf("sending iidx: %d oidx: %d wnr: %d\n",udpBuffer.iidx, udpBuffer.oidx,udpBuffer.WnR); 
  for(i = 0; i < 60; i++) {
		if((udpBuffer.WnR) && (udpBuffer.iidx == udpBuffer.oidx)){
			//printf("Buffer is empty\n");	
			break;
		}
		errno = 0;
		retSend = sendUdp(&udpBuffer.udpFrames[udpBuffer.oidx].data[bytesSent], udpBuffer.udpFrames[udpBuffer.oidx].length-bytesSent);
		//retSend = send(remote_socket, &udpBuffer.udpFrames[udpBuffer.oidx].data[bytesSent], udpBuffer.udpFrames[udpBuffer.oidx].length-bytesSent, MSG_DONTWAIT);
		//printf("sent %d of %d bytes\n",retSend,udpBuffer.udpFrames[udpBuffer.oidx].length);
		if(retSend >= 0){
			bytesSent += retSend;
			if(bytesSent == udpBuffer.udpFrames[udpBuffer.oidx].length){
				bytesSent = 0;
				cpcCounter--;
				sendUdpCnt++;
			
				udpBuffer.oidx = (udpBuffer.oidx+1) % UDP_BUF_MAX;
				udpBuffer.WnR  = 1;
			}
		} else {
			perror("Error while sending:");
			break;
		}
	}
	return;
}
