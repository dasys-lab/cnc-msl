#ifndef MCDC
#define MCDC
#include "MCDCProtocol.h"
#include "pcanconnect.h"
#include <sys/time.h>
#include "settings.h"

#define CONTROLLER_COUNT 4

#define MCDC_ERROR_FLAGS1 0xFF
#define MCDC_ERROR_FLAGS2 (0x01 | 0x02 | 0x10) // Life Guard, Bus Error and Software overflow
#define MCDC_CAN_TIMEOUT 10000 //1000   //16bit! in milliseconds
#define MCDC_CAN_TIMEOUT_MULTIPLY 1 //8bit multiplier
#define MCDC_GUARD_SEND_INTERVAL 600

#define MCDC_TRACE_PWM 2
#define MCDC_TRACE_TARGET_RPM 1
#define MCDC_TRACE_CURRENT 4
#define MCDC_TRACE_HOUSE_TEMP 44
#define MCDC_TRACE_COIL_TEMP 46
#define MCDC_TRACE_POSITION 200
#define MCDC_TRACE_TARGET_POSITION 201

#define MCDC_TRACE_NONE 255

typedef enum NetStatus { net_off,net_on,preOperational } NetStatus;
typedef enum OpModus {op_position, op_faulhaber, op_velocity, op_homing} OpModus;
typedef enum Amplifier {amp_off,amp_on} Amplifier;

extern  controller_settings default_settings;
extern  controller_settings current_settings;


typedef struct {
	unsigned char nodeid;
} MCDCBasicConfig;


typedef struct {
  NetStatus network;
  OpModus opMode;
  Amplifier amplifier;

  int nmtstatus;
  int statusword;

  int fault;
  int errorword;

  int tracecounter;

  int actualRPM;
  int actualPWM;
  int actualCurrent; //mA

  int housingTemp;


  struct timeval lastTraceTime;
  int communicating;

} MCDCStatus;

typedef struct  {
	MCDCBasicConfig baseConfig;
	MCDCStatus status;
} MCDCController;

#define MCDC_ACTIVE(x) ((x).status.amplifier == amp_on && (x).status.network==net_on && (x).status.opMode==op_faulhaber && (x).status.fault==0)

#define INT2BYTEPOS(i,b,pos) { (b)[(pos)] = (char)(i); (b)[(pos)+1] = (char)((i)>>8); (b)[(pos)+2] = (char)((i)>>16); (b)[(pos)+3] = (char)((i)>>24);}




void mcdc_load();
int mcdc_init_controllers();
int mcdc_init_controller(unsigned char i);
int mcdc_set_fault_configuration(unsigned char nodeid);
void mcdc_send_life_guard();
void mcdc_query_nmt_status(unsigned char nodeid);
void mcdc_query_status(unsigned char nodeid);
void mcdc_query_status_all();
void mcdc_save_all();


void mcdc_processSDOResponse(unsigned char nodeid,unsigned char* buffer, unsigned char len);
void mcdc_processNMTResponse(unsigned char nodeid,unsigned char* buffer,unsigned char len);
int mcdc_processPDO2Response(unsigned char nodeid,unsigned char* buffer,unsigned char len);
void mcdc_processTraceData(unsigned char nodeid,unsigned char* buffer,unsigned char len);
int mcdc_pdo2Confirmation(unsigned char nodeid, unsigned char* toconfirm);
int mcdc_sdoDownloadConfirmation(unsigned char nodeid, unsigned char* toconfirm);

void mcdc_object_info(unsigned char index, int key, int value);
void mcdc_update_statusword(unsigned char index, int status);
int mcdc_enable_all();
int mcdc_disable_all();
int mcdc_reset_all_nodes();

void mcdc_set_velocity_direct(unsigned char nodeid, int value);

//void mcdc_query_voltage(unsigned char nodeid);
void mcdc_query_housing_temp(unsigned char nodeid);
void mcdc_query_infos();

int mcdc_enable(unsigned char nodeid);
int mcdc_disable(unsigned char nodeid);
//void mcdc_achieve_active(unsigned char nodeid);


int mcdc_wait_for_nmt_status_update(unsigned char nodeid);
int mcdc_set_network_on(unsigned char nodeid);

void mcdc_check_status();
void mcdc_print_node_status();
#endif
