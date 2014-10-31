#ifndef EPOSUSB_H
#define EPOSUSB_H 1
#include "controller.h"
#include <ftdi.h>
#include <stdint.h>  /* int types with given size */


//USB CMDS:
#define EPOS_USB_READCMD 	0x10
#define EPOS_USB_RTR		0x21
#define EPOS_USB_SENDCAN	0x20
#define EPOS_USB_NMT		0x0E

///CAN CMDS:

#define EPOS_SET_PROFILE 0x6060

#define EPOS_MAX_VELO	0x607F
#define EPOS_MAX_ACCEL 	0x6083
#define EPOS_MAX_DECCEL	0x6084
#define EPOS_SET_VELO	0x60FF

#define EPOS_SET_CONTROL 0x6040

//GETTERS:

#define EPOS_GET_STATUS		0x6041
#define EPOS_GET_VELO		0x2028
#define EPOS_GET_CURRENT	0x6078



//VALUES:
#define EPOS_PROFILE_VELOCITY 0x03

#define EPOS_MODE_QUICKSTOP 	0x000B
#define EPOS_MODE_SHUTDOWN 		0x0006
#define EPOS_MODE_SWITCHON 		0x000F
#define EPOS_MODE_RESET_FAULTS 	0x0080
//NMT
#define EPOS_USB_SENDNMT	0x0E


#define EPOS_NMT_STATUS_START 		0x01
#define EPOS_NMT_STATUS_STOP		0x02
#define EPOS_NMT_STATUS_PREOP		0x80
#define EPOS_NMT_STATUS_RESET		0x81
#define EPOS_NMT_STATUS_RESETCOM	0x82


namespace Controlling {

typedef uint16_t WORD;
typedef uint32_t DWORD;


typedef enum NetStatus { net_off,net_on,preOperational } NetStatus;
typedef enum OpModus {op_velocity, op_unknown} OpModus;
typedef enum Status {running,preinit,initing,failure,recover} Status;

typedef struct {
  NetStatus network;
  OpModus opMode;

  int nmtstatus;
  int statusword;

  int actualRPM;
  int actualCurrent; //mA


} EposStatus;



class EposUSB : public Controller {
	public:
		EposUSB(int count) : Controller(count) {
			epos=(EposStatus*)malloc(4*sizeof(EposStatus));
			status = preinit;
		}

		int InitConnection();
		int SendCommand(unsigned char op,unsigned char* data, int datalen);
		int QueryObject(int nodeid,unsigned char* index);
		int Listen();
		void SendLifeGuard();
		void SendCanFrame(int id,int nodeid,unsigned char* data, int len);
	protected:
		WORD CalcFieldCRC(WORD oplen, WORD* pDataArray, WORD numberOfWords);
		void SwitchByteOrder(unsigned char* buf, int len);

		struct ftdi_context ftdic;
		int connected;
		Status status;
		unsigned char txbuf[64];
		unsigned char rxbuf[64];
		EposStatus* epos;

};


}

#endif

