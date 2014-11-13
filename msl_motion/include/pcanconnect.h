#ifndef pcanconnect
#define pcanconnect 1
#include <libpcan.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "MCDCProtocol.h"

#ifndef CAN_TIMEOUT
#define CAN_TIMEOUT 500
#endif

void printMessage(TPCANMsg *m);
int can_init();
void can_close();
void* readLoop(void* stuff);
void can_startListener();
void enableCanEvent(void (*callback)(TPCANRdMsg*));
void disableCanEvent();
void enableTraceCanEvent(void (*callback)(TPCANRdMsg*), int traceId);
void disableTraceCanEvent();
int writeCanMsg(unsigned int msgid, unsigned char receiver, unsigned char* buffer, int len);


int waitForCanMsg(unsigned int msgid, unsigned char* buffer, int* len, int timeout);

#endif
