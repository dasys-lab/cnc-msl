#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ftdi.h>
#include "eposusb.h"
#include <errno.h>


#define DEBUG_RCV 1
#define DEBUG_SEND 1

using namespace Controlling;

#define VENDORID 0x403
#define PRODUCTID 0xa8b0

#define DLE 0x90
#define STX 0x02

/*
EposUSB::EposUSB(int count) {
	
	};
*/



 
int EposUSB::InitConnection() {
	//! FTDI calls return code
	int ret;
	connected = 0;
	// Init
    if (ftdi_init(&ftdic) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return -1;
    }

    // Select first interface
    ftdi_set_interface(&ftdic, INTERFACE_ANY);


	if ((ret = ftdi_usb_open(&ftdic, VENDORID, PRODUCTID)) < 0) {
		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}

	//! reset FTDI device
	if ((ret = ftdi_usb_reset(&ftdic)) < 0) {
		fprintf(stderr, "unable to reset ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}

	if ((ret = ftdi_set_line_property(&ftdic, BITS_8, STOP_BIT_1, NONE)) < 0) {
		fprintf(stderr, "unable to set ftdi line property: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
    }
	//! set flow control
	if ((ret = ftdi_setflowctrl(&ftdic, SIO_DISABLE_FLOW_CTRL)) < 0) {
	//if ((ret = ftdi_setflowctrl(&ftdic, SIO_RTS_CTS_HS)) < 0) {
		fprintf(stderr, "unable to set ftdi flow control: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
    }
	//! set latency timer
	if ((ret = ftdi_set_latency_timer(&ftdic, 1)) < 0) {
		fprintf(stderr, "unable to set ftdi latency timer: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}
	//! set baud rate
	if ((ret = ftdi_set_baudrate(&ftdic, 1000000)) < 0) {
		fprintf(stderr, "unable to set ftdi baudrate: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}
#if 0
	//! set write data chunk size
	if ((ret = ftdi_write_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi write chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}

	//! set read data chunk size
	if ((ret = ftdi_read_data_set_chunksize(&ftdic, 512)) < 0) {
		fprintf(stderr, "unable to set ftdi read chunksize: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
		return -1;
	}
#endif
	connected = 1;
	return 1;
}
	void EposUSB::SendLifeGuard() {
		WORD cmd[2];
		for(int i=1; i<controllerCount; i++) {
			cmd[0] = 0x700+(i+1);
			cmd[1] = 0;
			//SwitchByteOrder((unsigned char*)cmd,2);
			SendCommand(EPOS_USB_RTR,(unsigned char*)cmd,4);
		}
		cmd[0] = 0x01;
		cmd[1] = 0;
		SendCommand(EPOS_USB_NMT,(unsigned char*)cmd,4);
	}
	void EposUSB::SendCanFrame(int id,int nodeid,unsigned char* data, int len) {
		WORD buf[6];
		buf[0] = id | nodeid;
		buf[1] = len;
		memset(&buf[2],0,8);
		memcpy(&buf[2],data,len);
		SendCommand(EPOS_USB_SENDCAN,(unsigned char*)buf,12);
 	}
	int EposUSB::QueryObject(int nodeid,unsigned char* index) {
		unsigned char buf[4];
		memcpy(buf,index,3);
		buf[3] = nodeid;
		return SendCommand(EPOS_USB_READCMD,buf,4);

	}
	int EposUSB::SendCommand(unsigned char op,unsigned char* data, int datalen) {//note: data len is in chars
		int i=0;
		int curpos = 2;
		unsigned char wordcount;
		WORD crc;
		if (datalen < 0 || datalen > 255)  {
			printf("Command Data length malformed: %d\n",datalen);
			return -1;
		}
		wordcount = (unsigned char)(datalen/2);
		txbuf[0] = DLE;
		txbuf[1] = STX;

		//stuffing:
		if(op == DLE) {
			txbuf[2] = DLE;
			txbuf[3] = DLE;
			curpos +=2;
		} else {
			txbuf[2] = op;
			curpos++;
		}
		if (wordcount==DLE) {
			txbuf[curpos++] = DLE;
			txbuf[curpos++] = DLE;
		} else {
			txbuf[curpos++] = wordcount;
		}
		while(i<datalen) {
			if(data[i] ==DLE) {
				txbuf[curpos++] = DLE;
				txbuf[curpos++] = DLE;
			} else {
				txbuf[curpos++] = data[i++];
			}
		}
		crc = CalcFieldCRC(((WORD)wordcount<<8)|op,(WORD*)data,wordcount+1);
		//(WORD*)(txbuf[curpos]) = crc;
		txbuf[curpos++] = crc & 0xFF;
		txbuf[curpos++] =((unsigned char)(crc>>8)) & 0xFF;
		//curpos+=2;
//CalcFieldCRC({0x10,0x02,0x81,0x20,0x00,0x01}, 3)

	#if DEBUG_SEND
		printf(">> ");
		for(int k=0; k<curpos; k++) {
			printf("0x%02x,", txbuf[k]);
		}
		printf("\n");
	#endif

		i = ftdi_write_data(&ftdic, txbuf, curpos);

		if (i != curpos) {
			fprintf(stderr, "ftdi device write failed (%d/%d chars written): (%s)\n", i, curpos, ftdi_get_error_string(&ftdic));
			perror("");
			return -1;
		}

		return 1;
	}
int EposUSB::Listen() {

		int ret = ftdi_read_data(&ftdic, rxbuf, sizeof(rxbuf));
		if (ret < 0) {
			fprintf(stderr, "ftdi device read failed (%d): (%s)\n", ret, ftdi_get_error_string(&ftdic));
			return -1;
		} 

	#if DEBUG_RCV
		printf("<< ");
		for(int i=0; i<ret; i++) {
			printf("0x%02x,", rxbuf[i]);
		}
		printf("\n");
	#endif
		return 1;

}
void EposUSB::SwitchByteOrder(unsigned char* buf, int len) {
	char temp;
	for(int i=0; i<len; i+=2) {
		temp = buf[i];
		buf[i] = buf[i+1];
		buf[i+1] = temp;
	}
}
WORD EposUSB::CalcFieldCRC(WORD oplen, WORD* pDataArray, WORD numberOfWords) {
	WORD shifter,c;
	WORD carry;
	WORD CRC=0;
	int first = 1;
	numberOfWords++;
	while(numberOfWords--) {
		shifter = 0x8000;
		if (first) {
			c = oplen;
		}
		else if(numberOfWords==0) {
			c = 0x00;
		} else {
			c=*pDataArray++;
		}

		first = 0;
		do {
			carry = CRC & 0x8000;
			CRC <<= 1;
			if(c & shifter) CRC++;
			if(carry) CRC ^= 0x1021;
			shifter >>=1;			
		} while(shifter);
	}
	//printf("CRC: %#x\n",CRC);
	return CRC;
}
