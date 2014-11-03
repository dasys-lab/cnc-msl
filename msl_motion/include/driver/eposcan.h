#ifndef EPOSCAN_H
#define EPOSCAN_H 1
#include "controller.h"
#include <canlistener.h>
#include "canconnection.h"
#include "eposprotocol.h"
#include <stdint.h>  /* int types with given size */
#include <stdlib.h>
#include <sys/time.h>
#include "../util.h"



namespace Controlling {

typedef uint16_t WORD;
typedef uint32_t DWORD;


//typedef enum NetStatus { net_off,net_on,preOperational } NetStatus;
//typedef enum OpModus {op_velocity, op_unknown} OpModus;
typedef enum Status {running,preinit,initing,failure,recover} Status;

typedef enum EposNMTStatus {unknown=0,stopped=1,operational=2,preoperational=3, booting=4} EposNMTStatus;

typedef struct {
//  NetStatus network;
//  OpModus opMode;
	EposNMTStatus nmtStatus;

 // int nmtstatus;
  short statusword;

  int actualRPM;
  int actualCurrent; //mA

  int demandRPM;
  timeval lastReceivedUpdate;


} EposStatus;



class EposCan : public Controller, public CanListener {
	public:
		EposCan(int count, CanConnection* con) : Controller(count), can(con) {
			epos=(EposStatus*)malloc(count*sizeof(EposStatus));
			status = preinit;
			ResetData();
			con->SetReceiver(this);
			gettimeofday(&last_triggertime,NULL);
			gettimeofday(&cur_triggertime,NULL);
		}

		int InitAllNodes();

		int InitNode(int nodeid);

		int DisableAllNodes();
		int EnableAllNodes(); 
		int EnableNode(int which);
		int DisableNode(int which);

		//int SendCommand(unsigned char op,unsigned char* data, int datalen);
		//int QueryObject(int nodeid,unsigned char* index);
		//int Listen();
		void SendLifeGuard();
		
		//void SendCanFrame(int id,int nodeid,unsigned char* data, int len);
		void Receive(unsigned int canid, unsigned char* data, int len);
		
		int IsFunctional();
		int ActualRPM(int which);
		int DemandRPM(int which);
		int ActualCurrent(int which);
		void SetDemandRPM(int which, int rpm);
		void PrintStatus();
		void SetVelocityDirect(int which, int rpm);
		void EmergencyShutdown();
		void Trigger(int enabled);
		void ReadErrorRegister();
		int ResetFaults();		
	protected:
		//WORD CalcFieldCRC(WORD oplen, WORD* pDataArray, WORD numberOfWords);
		//void SwitchByteOrder(unsigned char* buf, int len);
		//void ResetCommunication();
		void ProcessNMTResponse(unsigned char nodeid,unsigned char* buffer, int len);
		void ProcessSDOResponse(unsigned char nodeid,unsigned char* buffer, int len);
		void ProcessOdometry(unsigned char nodeid,unsigned char* data,int len);
		int SdoDownloadConfirmation(unsigned char nodeid, unsigned char* toconfirm);
		void HandleEposError(unsigned char nodeid,unsigned int errorCode);
		void SendVelocity();
		int IsEnabled(int node);
		int HasStatusError(int node);

		void ResetCommunication();

		void ResetAllNodes();
		
		void ResetData();
		void SendSynch();
		Status status;
		unsigned char txbuf[64];
		int txbuflen;
		//unsigned char rxbuf[64];
		EposStatus* epos;
		CanConnection* can;

		long lifeguardcounter;
		struct timeval cur_triggertime;
		struct timeval last_triggertime;
		

};


}

#endif

