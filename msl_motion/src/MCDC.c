#include "MCDC.h"
#include "gonzales.h"
#include "SpicaHelper.h"
MCDCController  mcdc[ CONTROLLER_COUNT];




unsigned char mcdc_buffer[8];
int mcdc_bufferlen=0;

//struct timeval mcdc_timestamp;

void mcdc_load() {
    unsigned char i;
    for (i=0; i < CONTROLLER_COUNT; i++){
        mcdc[i].baseConfig.nodeid = i;
        mcdc[i].status.amplifier = amp_off;
        mcdc[i].status.network = net_off;
        mcdc[i].status.opMode = op_position;
        mcdc[i].status.fault = 0;
        mcdc[i].status.statusword = 0;
        mcdc[i].status.errorword = 0;
        mcdc[i].status.nmtstatus = 0;
        mcdc[i].status.tracecounter = 0;

        mcdc[i].status.actualPWM = 0;
        mcdc[i].status.actualRPM = 0;
        mcdc[i].status.actualCurrent = 0;

        mcdc[i].status.housingTemp = 0;


        gettimeofday(&mcdc[i].status.lastTraceTime,NULL);
        mcdc[i].status.communicating = 1;

    }
}
int mcdc_init_controller(unsigned char i) {
    unsigned char nodeid;
    int ret=0;
    unsigned char buffer[5];
    printf("Initialising Controller %d\n",i);
    nodeid = i+1;
    if (!mcdc_set_fault_configuration(nodeid)) {
        printf("Cannot set Fault configuration for node %d\n",nodeid);
        return 0;
    }

    //Set the special FAULHABER protection bit:
    buffer[0] = CAN_CMD_DOWNLOAD_REQUEST | 0x0B;
    buffer[1] = 0x01; buffer[2] = 0x24; buffer[3] = 0x00; buffer[4] = 0x01; buffer[5] = 0x00; buffer[6] = 0x00; buffer[7] = 0x00;
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,buffer,8);
    if (!mcdc_sdoDownloadConfirmation(nodeid, buffer)) {
        printf("Could not set special Faulhaber bit!\n"); return 0;
    }
    //End of special faulhaber mode

    ret=mcdc_set_network_on(nodeid); //switch to OPERATIONAL
    if (!ret) {
        printf("No Answer from node %d\n",nodeid);
        return 0;
    }

    //SWITCH TO FAULHABER MODE
    buffer[0] = CAN_CMD_SET_OPMODE;
    buffer[1] = 0xFF; buffer[2] = 0xFF; buffer[3] = 0xFF; buffer[4] = 0xFF;
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    buffer[0] = CAN_CMD_GET_OPMODE;
    buffer[1]=0; buffer[2]=0; buffer[3]=0; buffer[4]=0;
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    writeCanMsg(CAN_ID_PDO2_RESPONSE,nodeid,NULL,0xFF);
    ret = waitForCanMsg(CAN_ID_PDO2_RESPONSE | nodeid,mcdc_buffer,&mcdc_bufferlen,CAN_TIMEOUT);
    if (!ret) {
        printf("Cannot set mode!\n");
        return 0;
    }
    if (ret) {
        ret =  mcdc_processPDO2Response(nodeid,mcdc_buffer,mcdc_bufferlen);
        if (!ret) return 0;
    }
    if (mcdc[i].status.opMode!=op_faulhaber) {
        printf("Faulhaber Mode not active!\n");
        return 0;
    }
    //Disable Drive:
    buffer[0] = CAN_CMD_DISABLE;
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot disable drive!\n");
        return 0;
    }
    //Set Up Trace Configuration:

    buffer[0] = 0; //velocity
    buffer[1] = current_settings.additionalTraceData; //pwm
    buffer[2] = 1; //timecode
    buffer[3] = current_settings.traceCount; //num of packets
    buffer[4] = current_settings.traceInterval; //interval in ms
    writeCanMsg(CAN_ID_SET_TRACE,nodeid,buffer,5);
    //writeCanMsg(CAN_ID_SYNC,0,NULL,0);

    printf("End Setting Trace\n");
    ///Set Controller Mode:
    buffer[0] = CAN_CMD_SET_CONT_MODE;
    buffer[1]=0; buffer[2]=0; buffer[3]=0; buffer[4]=0;
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set controller mode!\n");
        return 0;
    }
    //Set Config:
    buffer[0] = CAN_CMD_SET_CURRENT_INTEGRAL;
    INT2BYTEPOS(current_settings.amp_control_I,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set current integral!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_ENCODER_RESOLUTION;
    INT2BYTEPOS(current_settings.encoder_resolution,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set encoder resolution!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_INTEGRAL;
    INT2BYTEPOS(current_settings.I,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set integral!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_ACCEL;
    INT2BYTEPOS(current_settings.max_accel,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max acceleration!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_CONT_CURRENT;
    INT2BYTEPOS(current_settings.max_continous_amp,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max continous current!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_CURRENT;
    INT2BYTEPOS(current_settings.max_amp,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max peak current!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_DECCEL;
    INT2BYTEPOS(current_settings.max_deccel,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max decceleration!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_SPEED;
    INT2BYTEPOS(current_settings.maxRPM,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max speed!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MAX_VELO_DEVIATION;
    INT2BYTEPOS(current_settings.max_deviation,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set max deviation!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_MOTOR_RESISTANCE;
    INT2BYTEPOS(current_settings.motorResistance,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set motor resistance!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_PROPORTIONAL;
    INT2BYTEPOS(current_settings.P,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set proportional gain!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_SAMPLE_RATE;
    INT2BYTEPOS(current_settings.sampling_rate,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set sampling rate!\n");
        return 0;
    }
    buffer[0] = CAN_CMD_SET_SPEED_CONSTANT;
    INT2BYTEPOS(current_settings.speedConstant,buffer,1);
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
        printf("Cannot set speed constant!\n");
        return 0;
    }
    //trigger trace
    mcdc[i].status.tracecounter=0;
    writeCanMsg(CAN_ID_GET_TRACE,nodeid,NULL,0xFF);
	return 1;
}
int mcdc_init_controllers() {
    unsigned char i;
    int ret=0;
    printf("Initialising...\n");
    for (i=0; i<CONTROLLER_COUNT; i++) {
		ret = mcdc_init_controller(i);
		if (gonz_get_mode()==GONZ_MODE_NORMAL) {
				if (!ret) return ret;
		} else {
			if (!ret) printf("Unable to initialise controller %d\n",i);
		}
	}

    //ALL GOOD and inited: enable drive!
	if(gonz_get_mode()==GONZ_MODE_NORMAL) {
	    return mcdc_enable_all();
	}
	else return 1;
}
int mcdc_disable_all() { //called event based upon receiving emergency signals, hence not sychronised to a confirmation
    unsigned char nodeid;
    unsigned char buffer[5] = {CAN_CMD_DISABLE,0,0,0,0};
    for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    }
    /*int ret=1;
    for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
                printf("Cannot Disable Drive %d!\n",nodeid);
                ret = 0;
            }
    }*/
    return 1;
}
int mcdc_enable_all() {
    unsigned char nodeid;
    unsigned char buffer[5] = {CAN_CMD_ENABLE,0,0,0,0};
    for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
            if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
                printf("Cannot Enable Drive %d!\n",nodeid);
                mcdc_disable_all();
                return 0;
            }
    }
    return 1;
}
int mcdc_enable(unsigned char nodeid) {
    unsigned char buffer[5] = {CAN_CMD_ENABLE,0,0,0,0};
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    if (!mcdc_pdo2Confirmation(nodeid,buffer)) {
           printf("Cannot Enable Drive %d!\n",nodeid);
           mcdc_disable(nodeid);
           return 0;
    }
    return 1;
}
void mcdc_save_all() {
	unsigned char buffer[8] = {0x23,0x10,0x10,0x01,0x73,0x61,0x76,0x65};
	int i=0;
	for(;i<CONTROLLER_COUNT;i++) {		
		writeCanMsg(CAN_ID_SDO_REQUEST,i+1,buffer,8);
	}
}

int mcdc_disable(unsigned char nodeid) { //called event based upon receiving emergency signals, hence not sychronised to a confirmation
    unsigned char buffer[5] = {CAN_CMD_DISABLE,0,0,0,0};
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    return 1;
}


int mcdc_sdoDownloadConfirmation(unsigned char nodeid, unsigned char* toconfirm) {
    int ret;
    ret = waitForCanMsg(CAN_ID_SDO_RESPONSE | nodeid,mcdc_buffer,&mcdc_bufferlen,CAN_TIMEOUT);
    if (!ret) return 0;
    if (mcdc_bufferlen!=8) return 0;
    if (mcdc_buffer[0]!= CAN_CMD_DOWNLOAD_RESPONSE) return 0;
    unsigned char i;
    for (i=1; i < 4; i++) {
        if (mcdc_buffer[i]!=toconfirm[i]) return 0;
    }
    return 1;
}
int mcdc_pdo2Confirmation(unsigned char nodeid, unsigned char* toconfirm) {
    int ret;
    writeCanMsg(CAN_ID_PDO2_RESPONSE,nodeid,NULL,0xFF);
    ret=waitForCanMsg(CAN_ID_PDO2_RESPONSE | nodeid,mcdc_buffer,&mcdc_bufferlen,CAN_TIMEOUT);
    if (!ret) return 0;
    if (mcdc_bufferlen!=6) return 0;
    if (mcdc_buffer[5]!= 1) return 0;
    if (mcdc_buffer[0]!=toconfirm[0]) return 0;
    /*unsigned char i;
    for (i=0; i<5; i++) {
        if (toconfirm[i] != mcdc_buffer[i]) return 0;
    }*/
    return 1;
}

//void mcdc_achieve_active(unsigned char nodeid) {
    /*MCDCController* m = &mcdc[nodeid];
    if (m->status.want_active==0 && MCDC_ACTIVE(*m)) {
        //switch off
printf("NOT IMPLEMENTED\n");
    } else if (!MCDC_ACTIVE(*m)) {
        //switch on
printf("NOT IMPLEMENTED\n");
    }*/
//}

void mcdc_update_statusword(unsigned char index, int status) {
    int smstatus = status & 0xFF;
    if (smstatus == 0x23 || smstatus == 0x27 || smstatus == 0x07) {
        mcdc[index].status.amplifier = amp_on;
    } else {
        mcdc[index].status.amplifier = amp_off;
    }
    if (status & 0x08) {
        mcdc[index].status.fault = 1;
    } else {
        mcdc[index].status.fault = 0;
    }
    if (mcdc[index].status.statusword != status) {
        printf("New Status on node %d\n",(index+1));
        mcdc[index].status.statusword = status;
    }


}
void mcdc_processTraceData(unsigned char nodeid,unsigned char* buffer,unsigned char len) {
    unsigned char i = nodeid-1;

    mcdc[i].status.actualRPM = (short) (buffer[0]+(buffer[1]<<8));
    if (current_settings.additionalTraceData==MCDC_TRACE_PWM) {
        mcdc[i].status.actualPWM = (short)( buffer[2]+(buffer[3]<<8));
    } else if (current_settings.additionalTraceData==MCDC_TRACE_CURRENT) {
         mcdc[i].status.actualCurrent = (short)( buffer[2]+(buffer[3]<<8));
        //printf("mA:\t%d\n",(short)( buffer[2]+(buffer[3]<<8)));
    }

    //printf("trace:\t%d\t%d\t%d\t%d\n",mcdc[i].status.tracecounter,mcdc[i].status.actualRPM,mcdc[i].status.actualPWM,buffer[4]);

    if(++mcdc[i].status.tracecounter >= current_settings.traceCount) {
        mcdc[i].status.tracecounter =0;
        writeCanMsg(CAN_ID_GET_TRACE,nodeid,NULL,0xFF);
    }

    gettimeofday(&mcdc[i].status.lastTraceTime,NULL);
}
void mcdc_processNMTResponse(unsigned char nodeid,unsigned char* buffer,unsigned char len) {
    unsigned char status = 0x7F & buffer[0];
    unsigned char index = nodeid-1;
    mcdc[index].status.nmtstatus = status;
    switch (status) {
        case 0x00:
            mcdc[index].status.network = preOperational;
            break;
        case 0x04:
            mcdc[index].status.network = net_off;
            break;
        case 0x05:
            mcdc[index].status.network = net_on;
            break;
        case 0x7F:
            mcdc[index].status.network = preOperational;
            break;
        default:
            printf("UNKNOWN NMT STATUS\n");
    }
}
void mcdc_object_info(unsigned char index, int key, int value) {
    switch(key) {
        case CAN_SDO_STATUSWORD: //Statusword
            mcdc_update_statusword(index,value);
            printf("status is: 0x%x\n",value);
            break;
        case CAN_SDO_GET_OPMODE: //Operation Mode
        //printf("Value: %d\n",value);
            if (value== 1) {
                mcdc[index].status.opMode = op_position;
                printf("Positioning\n");
            }
            else if (value == 3) {
                mcdc[index].status.opMode = op_velocity;
                printf("Velocity\n");
            }
            else if (value == 6) {
                mcdc[index].status.opMode = op_homing;
                printf("Homing\n");
            }
            else if (value == 0xFF) {
                mcdc[index].status.opMode = op_faulhaber;
                printf("Faulhaber\n");
            }
            break;
        case CAN_SDO_ERRORWORD:
        //printf("Error Word: 0x%x",value);
            mcdc[index].status.errorword = value;
            break;
        default:
            printf("Unknown status key: 0x%x from node: %d\n",key,index+1);
    }
}
int mcdc_set_fault_configuration(unsigned char nodeid) {
    //int ret;
    //Config message on failure register:
    unsigned char msg[8] = { CAN_CMD_DOWNLOAD_REQUEST | 0x0B ,0x20, 0x23, 2,MCDC_ERROR_FLAGS1, MCDC_ERROR_FLAGS2,0,0};
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8);
    if (!mcdc_sdoDownloadConfirmation(nodeid,msg)) return 0;
    //Config Fault state on failure register:
    msg[3]=3;
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8);
    if (!mcdc_sdoDownloadConfirmation(nodeid,msg)) return 0;
    //Config Life Guard Time:
    msg[1] = 0x0C; msg[2] = 0x10; msg[3] = 0;
    INT2BYTEPOS(MCDC_CAN_TIMEOUT,msg,4);
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8);
    if (!mcdc_sdoDownloadConfirmation(nodeid,msg)) return 0;

    msg[0] = CAN_CMD_DOWNLOAD_REQUEST| 0x0F; msg[1] = 0x0D;
    INT2BYTEPOS(MCDC_CAN_TIMEOUT_MULTIPLY,msg,4);
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8);
    if (!mcdc_sdoDownloadConfirmation(nodeid,msg)) return 0;

    /*unsigned char msg2[8] = { CAN_CMD_UPLOAD_REQUEST, 0x20,0x23,2,0,0,0,0};
    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg2,8);*/
    return 1;
}
void mcdc_send_life_guard() {
    unsigned char nodeid;
    for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
        writeCanMsg(CAN_ID_NMT_STATUS,nodeid,NULL,0xFF); //RTR
    }
}
void mcdc_query_nmt_status(unsigned char nodeid) {
    writeCanMsg(CAN_ID_NMT_STATUS,nodeid,NULL,0xFF); //RTR
}

void mcdc_query_status(unsigned char nodeid) {
    //writeCanMsg(CAN_ID_NMT_STATUS,nodeid,NULL,0xFF); //RTR
    writeCanMsg(CAN_ID_STATUS,nodeid,NULL,0xFF);
    //unsigned char errmsg[8] = {CAN_CMD_UPLOAD_REQUEST,0x20,0x23,1,0,0,0,0};
    //writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,errmsg,8);
    //unsigned char msg[8]= { CAN_CMD_UPLOAD_REQUEST,0x41,0x60,0,0,0,0,0};
    //writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8); //statusword
    //msg[1] = 0x61;
    //writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,msg,8); //mode
}
void mcdc_query_status_all() {
    unsigned char nodeid;
    //unsigned char errmsg[8] = {CAN_CMD_UPLOAD_REQUEST,0x20,0x23,1,0,0,0,0};
    for(nodeid=CONTROLLER_COUNT; nodeid>0; nodeid--) {
        writeCanMsg(CAN_ID_STATUS,nodeid,NULL,0xFF);
    //    writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,errmsg,8);
    }
}
void mcdc_set_velocity_direct(unsigned char nodeid, int value) {
    unsigned char buffer[5];
    buffer[0] = CAN_CMD_SET_VELOCITY;
     INT2BYTEPOS(value,buffer,1);
     writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
}
/*void mcdc_query_voltage(unsigned char nodeid) {
    unsigned char buffer[5] = {CAN_CMD_GET_VOLTAGE, 3,0,0,0};
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    writeCanMsg(CAN_ID_PDO2_RESPONSE,nodeid,NULL,0xFF);
}*/
void mcdc_query_housing_temp(unsigned char nodeid) {
    unsigned char buffer[5] = {CAN_CMD_GET_TEMP, 0,0,0,0};
    writeCanMsg(CAN_ID_PDO2_CMD,nodeid,buffer,5);
    writeCanMsg(CAN_ID_PDO2_RESPONSE,nodeid,NULL,0xFF);
}
void mcdc_query_infos() {
    unsigned char nodeid;
    for(nodeid=CONTROLLER_COUNT; nodeid>0; nodeid--) {
        //mcdc_query_voltage(nodeid);
        mcdc_query_housing_temp(nodeid);
    }
}

int mcdc_wait_for_nmt_status_update(unsigned char nodeid) {
    int ret = waitForCanMsg(CAN_ID_NMT_STATUS | nodeid, mcdc_buffer,&mcdc_bufferlen,CAN_TIMEOUT);
    if (ret) {
        mcdc_processNMTResponse(nodeid,mcdc_buffer,mcdc_bufferlen);
        mcdc_bufferlen=0;
        return 1;
    }
    return 0;
}

int mcdc_set_network_on(unsigned char nodeid) {
    unsigned char msg[2] = {CAN_NMT_SET_OPERATIONAL,nodeid};
    int ret=writeCanMsg(CAN_ID_NMT,0,msg,2);
    mcdc_query_nmt_status(nodeid);
    ret &= mcdc_wait_for_nmt_status_update(nodeid);
    return (mcdc[nodeid-1].status.network == net_on);
}
int mcdc_processPDO2Response(unsigned char nodeid,unsigned char* buffer, unsigned char len) {
    unsigned char index = nodeid-1;
    int value;
    if (len!=6) {
        printf("PDO2 Packet Malformed!\n");
        return 0;
    }
    switch (buffer[5]) { //Error code
        case 1: //all normal
            break;
        case (unsigned char)-2:
            printf("EEPROM writing done\n"); //not an error(?)
            break;
        case (unsigned char)-4:
            printf("Overtemperature on drive %d!\n",nodeid);
            return 0;
        case (unsigned char)-5:
            printf("Invalid Parameter on drive %d!\n",nodeid);
            return 0;
        case (unsigned char)-7:
            printf("Unknown Command on drive %d!\n",nodeid);
            return 0;
        case (unsigned char)-8:
            printf("Command not available on drive %d!\n",nodeid);
            return 0;
        case (unsigned char)-13:
            printf("Flash Defect on drive %d!\n",nodeid);
            return 0;
        default:
            printf("Unknown Error, drive %d!\n",nodeid);
            return 0;
    }
    switch(buffer[0]) {
        case CAN_CMD_GET_OPMODE:
            if (buffer[1]== 1) {
                mcdc[index].status.opMode = op_position;
                printf("Positioning\n");
            }
            else if (buffer[1] == 3) {
                mcdc[index].status.opMode = op_velocity;
                printf("Velocity\n");
            }
            else if (buffer[1] == 6) {
                mcdc[index].status.opMode = op_homing;
                printf("Homing\n");
            }
            else if (buffer[1] == 0xFF) {
                mcdc[index].status.opMode = op_faulhaber;
                printf("Faulhaber\n");
            }
            break;
        case CAN_CMD_GET_TEMP:
            value = (int)(buffer[1]+(buffer[2]<<8)+(buffer[3]<<16)+(buffer[4]<<24));
            printf("Node %d Temp: %d\n",nodeid,value);
            mcdc[index].status.housingTemp = value;
            break;
        case CAN_CMD_GET_VOLTAGE:
            value = (int)(buffer[1]+(buffer[2]<<8)+(buffer[3]<<16)+(buffer[4]<<24));
            printf("Voltage: %d\n",value);
            break;
        default:
            printf("Unknown PDO2 Response!\n");
            return 0;
    }
    return 1;
}
void mcdc_processSDOResponse(unsigned char nodeid,unsigned char* buffer, unsigned char len) {
    if(buffer[0] & CAN_CMD_UPLOAD_RESPONSE) {
        int key = buffer[1] | buffer[2]<<8;
        int value = buffer[4];
        unsigned char byteCount = buffer[0] & 0x0F;
        if ((byteCount) < 0x0C) {
            value += buffer[5] << 8;
            if (byteCount < 0x08) {
                value += buffer[6] << 16;
                if (byteCount < 0x04) {
                    value += buffer[7] << 24;
                }
            }
        }
        mcdc_object_info(nodeid-1,key,value);
    }
    else if (buffer[0] & CAN_CMD_SDO_TERMINATION) {
        printf("Error: SDO Termination!\n");
    }
    else printf("Unknown SDO Response: 0x%x\n",buffer[0]);

}

//regardless of error, set all nodes back
int mcdc_reset_all_nodes() {
    int allShutdown;
    unsigned char msg[2] = {0x80,0}; //Fault Reset
    unsigned char errmsg[8] = {CAN_CMD_UPLOAD_REQUEST,0x20,0x23,1,0,0,0,0};
    unsigned char nodeid;
    do {
        msg[0] = 0x80; msg[1] = 0; //reset fault
        for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            writeCanMsg(CAN_ID_CONTROL,nodeid,msg,2);
            //writeCanMsg(CAN_ID_STATUS,nodeid,NULL,0xFF);
        }
        msg[0] = CAN_NMT_SET_OPERATIONAL; //set operational in case node died
        for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            msg[1]=nodeid;
            writeCanMsg(CAN_ID_NMT,0,msg,2);
        }
        msg[0] = 0x06; msg[1] = 0; //Shutdown
        for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            writeCanMsg(CAN_ID_CONTROL,nodeid,msg,2); //Fault Reset
            //writeCanMsg(CAN_ID_STATUS,nodeid,NULL,0xFF);
        }

        //Query Status and error register before continuing:
        allShutdown = 1;
        for (nodeid=CONTROLLER_COUNT; nodeid > 0; nodeid--) {
            mcdc_query_status(nodeid);

            writeCanMsg(CAN_ID_SDO_REQUEST,nodeid,errmsg,8);
            waitForCanMsg(CAN_ID_SDO_RESPONSE|nodeid,mcdc_buffer,&mcdc_bufferlen,CAN_TIMEOUT);
            mcdc_processSDOResponse(nodeid,mcdc_buffer,mcdc_bufferlen);

            allShutdown &= ((mcdc[nodeid-1].status.statusword&0xFF) == 0x21);
            allShutdown &= ((mcdc[nodeid-1].status.errorword & (MCDC_ERROR_FLAGS1 | (MCDC_ERROR_FLAGS2 <<8))) == 0);

        }
        mcdc_send_life_guard();
        if (!allShutdown) {
            printf("retry...\n");
            can_close(); //orkaround for driver acting up if cable is disconnected
            usleep(1000);
            can_init();
            usleep(100000);

        }
    } while(!allShutdown);

    return mcdc_init_controllers();

}

void print_node_status() {
    unsigned char index;
    for(index=0; index < CONTROLLER_COUNT; index++) {

        printf("Node: %d\t Fault: %d\n",index+1,mcdc[index].status.fault);
        switch(mcdc[index].status.amplifier) {
            case amp_on: printf("\tAmp On\t"); break;
            case amp_off: printf("\tAmp Off\t"); break;
            default: printf("\tAmp unknown\t");
        }
        switch(mcdc[index].status.network) {
            case net_on: printf("Net On\n"); break;
            case net_off: printf("Net Off\n"); break;
            case preOperational: printf("PreOp\n"); break;
            default: printf("Net unknown\n");
        }
        printf("\tStatus: %x\n",mcdc[index].status.statusword);
        printf("\tError: %x\n",mcdc[index].status.errorword);

    }

}
