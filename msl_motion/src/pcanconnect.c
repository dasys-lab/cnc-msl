#include "pcanconnect.h"


#define DEFAULT_NODE "/dev/pcan40"

#define QUEUESIZE 255 //whatever, whichever


HANDLE can_handle;

TPCANRdMsg explicitAnswer;
TPCANRdMsg terminalQueue[QUEUESIZE];
TPCANRdMsg traceQueue[QUEUESIZE];
TPCANRdMsg incoming;
unsigned int waitOnId=0;
unsigned int tracedId=0;
unsigned int traceWritePtr=0;
unsigned int traceReadPtr=0;
unsigned int terminalWritePtr=0;
unsigned int terminalReadPtr=0;

//void *receiveCaEvent(*TPCANRdMsg,int)
void (*receiveCanEvent)(TPCANRdMsg*) = NULL;
void (*receiveTraceCanEvent)(TPCANRdMsg*) = NULL;

void enableCanEvent(void (*callback)(TPCANRdMsg*)) {
	receiveCanEvent=callback;
}
void disableCanEvent() {
	receiveCanEvent = NULL;
}
void enableTraceCanEvent(void (*callback)(TPCANRdMsg*), int traceId) {
	receiveTraceCanEvent=callback;
	tracedId = traceId;
}
void disableTraceCanEvent() {
	receiveTraceCanEvent = NULL;
}


void printMessage(TPCANMsg *m)
{
  int i;

  // print RTR, 11 or 29, CAN-Id and datalength
  printf("%c %c 0x%08x %1d  ",
      (m->MSGTYPE & MSGTYPE_RTR)      ? 'r' : 'm',
      (m->MSGTYPE & MSGTYPE_EXTENDED) ? 'e' : 's',
       m->ID,
       m->LEN);

	// don't print any telegram contents for remote frames
  if (!(m->MSGTYPE & MSGTYPE_RTR))
  	for (i = 0; i < m->LEN; i++)
    	printf("0x%02x ", m->DATA[i]);

  printf("\n");
}
int can_init() {
	const char* devnode = DEFAULT_NODE;
	can_handle = LINUX_CAN_Open(devnode,1);
	if (can_handle == NULL) {
		printf("Cannot open can!\n");
		return 0;
	}
	return (0==CAN_Init(can_handle,CAN_BAUD_1M,CAN_INIT_TYPE_ST));

}
void can_startListener() {
	pthread_t listenerThread;
	int ret = pthread_create(&listenerThread,NULL,readLoop, NULL);
	if (ret!=0) {
		printf("Thread creation failed\n");
	}
	printf("Started");
}

void* readLoop(void* stuff) {
	TPCANRdMsg msgBuf;
	DWORD ret;
	while(1) {
		ret = LINUX_CAN_Read(can_handle,&msgBuf);
//		printf("Read Status: %d\n",ret);
		if (ret==0) {
//            printf("rcv: ");
//            printMessage(&(msgBuf.Msg));
            if (waitOnId && waitOnId == msgBuf.Msg.ID) {
                //printf("waited for\n");
                explicitAnswer = msgBuf;
                waitOnId = 0;
            }
            else if (receiveTraceCanEvent != NULL &&  tracedId == (msgBuf.Msg.ID & CAN_MSG_ID_MASK)) {
                //printf("trace!\n");
                (*receiveTraceCanEvent)(&msgBuf);
            } else if (receiveCanEvent != NULL) {
                //printf("terminal\n");
                (*receiveCanEvent)(&msgBuf);
            }
		}
	}
	return (void*)NULL;
}

int writeCanMsg(unsigned int msgid, unsigned char receiver, unsigned char* buffer, int len) {
    TPCANMsg msgbuf;
    msgbuf.ID = msgid+receiver;
    if (len == 0xFF) {
        msgbuf.MSGTYPE = MSGTYPE_RTR;
        msgbuf.LEN = 0;
    }
    else {
        msgbuf.MSGTYPE = MSGTYPE_STANDARD;
        unsigned char i;
        for (i =0; i < len ; i++) {
            msgbuf.DATA[i] = buffer[i];
        }
        msgbuf.LEN = len;
    }
//    printf("Sending: ");
//    printMessage(&msgbuf);
    DWORD ret = LINUX_CAN_Write_Timeout(can_handle, &msgbuf, -1);
 //   printf("Send status: %d\n",ret);
    if (ret != 0) {
        printf("CAN FAILURE\n");
        return 0;
    }
    return 1;
}

void can_close() {
	CAN_Close(can_handle);
}

int waitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout) {
    waitOnId = msgid;
    int i=0;
    while(waitOnId && i++ < timeout) {
        usleep(10);
    }
    if (waitOnId) {
        waitOnId = 0;
        printf("timeout during wait!\n");
        return 0;
    }
    for (i=0; i < explicitAnswer.Msg.LEN; i++) {
        buffer[i] = explicitAnswer.Msg.DATA[i];
    }
    (*len)=explicitAnswer.Msg.LEN;
    return 1;
}
