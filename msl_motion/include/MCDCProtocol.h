#ifndef MCDCProtocol
#define MCDCProtocol



#define CAN_MSG_ID_MASK 		0x780
#define CAN_NODE_ID_MASK 		0x7F




#define CAN_ID_NMT 			0x000

#define CAN_NMT_SET_OPERATIONAL 0x01

//NMT STATUS stuff
#define CAN_ID_NMT_STATUS 0x700

//PDO1 Communication

#define CAN_ID_CONTROL 0x200
#define CAN_ID_STATUS 0x180


#define CAN_ID_ERROR 			0x80

//----------SDO Communication:
#define CAN_ID_SDO_REQUEST 		0x600
#define CAN_ID_SDO_RESPONSE 		0x580

#define CAN_CMD_UPLOAD_REQUEST 		0x40
#define CAN_CMD_UPLOAD_RESPONSE 	0x40
#define CAN_CMD_DOWNLOAD_REQUEST 	0x20
#define CAN_CMD_DOWNLOAD_RESPONSE 	0x60
#define CAN_CMD_SDO_TERMINATION 	0x80

#define CAN_SDO_CONTROLWORD 0x6040
#define CAN_SDO_STATUSWORD 0x6041
#define CAN_SDO_SET_OPMODE 0x6060
#define CAN_SDO_GET_OPMODE 0x6061
#define CAN_SDO_ERRORWORD 0x2320


//---------PDO2 Communication:
//FAULHABER SPECIFIC COMMANDS:
#define CAN_ID_PDO2_CMD                 0x300
#define CAN_ID_PDO2_RESPONSE            0x280


//configuration
#define CAN_CMD_SET_OPMODE              0xFD  // Faulhaber
#define CAN_CMD_SET_VELOCITY_SOURCE     0x8E //<- keep at default
#define CAN_CMD_SET_CONT_MODE           0x06 // use this
#define CAN_CMD_SET_STEP_MODE           0x46
#define CAN_CMD_SET_APCMODE             0x02
#define CAN_CMD_SET_GEARMODE            0x1D
#define CAN_CMD_SET_VOLTMODE            0x49 //or this
#define CAN_CMD_SET_IXRMODE             0x50

#define CAN_CMD_SET_ENCODER_RESOLUTION  0x70
#define CAN_CMD_SET_SPEED_CONSTANT      0x9E
#define CAN_CMD_SET_MOTOR_RESISTANCE    0x9F

#define CAN_CMD_SET_MAX_SPEED           0x8F
#define CAN_CMD_SET_MAX_ACCEL           0x65
#define CAN_CMD_SET_MAX_DECCEL          0x6D
#define CAN_CMD_SET_SAMPLE_RATE         0xA4
#define CAN_CMD_SET_PROPORTIONAL        0x89
#define CAN_CMD_SET_INTEGRAL            0x9B
#define CAN_CMD_SET_CURRENT_INTEGRAL    0xA2
#define CAN_CMD_SET_MAX_CURRENT         0x81
#define CAN_CMD_SET_MAX_CONT_CURRENT    0x80
#define CAN_CMD_SET_MAX_VELO_DEVIATION  0x6F
//getters:

#define CAN_CMD_GET_OPMODE              0xFE

#define CAN_CMD_GET_CURRENT_INTEGRAL    0x63

#define CAN_CMD_GET_VOLTAGE             0xB2
#define CAN_CMD_GET_TEMP                0x47

//commands:
#define CAN_CMD_SAVE    0x53
#define CAN_CMD_RESET   0x59

#define CAN_CMD_DISABLE         0x08
#define CAN_CMD_ENABLE          0x0F
#define CAN_CMD_MOTION          0x3C
#define CAN_CMD_SET_VELOCITY    0x93
#define CAN_CMD_SET_PWM         0x92
#define CAN_CMD_SET_REL_POSITION 0xB6
#define CAN_CMD_SET_ABS_POSITION 0xB4


//PDO3 - Trace
#define CAN_ID_SET_TRACE 0x400
#define CAN_ID_GET_TRACE 0x380
//Other
#define CAN_ID_SYNC 0x80


#endif
