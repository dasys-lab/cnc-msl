/** @mainpage MonitoringBoard
 * <br></br>
 *  <b>User Guide:</b>
 *  <br></br>
 *  The ISP connector marks left side, for easier descriptions
 *  <br></br>
 * <b>Voltages:</b>
 * <br></br>
 *  Jumper in row: 5V/26V, Jumper front: enable 26V
 *  <br></br>
 *  5V  -> Jumper 5V/26V left
 *  <br></br>
 *  26V -> Jumper 5V/26V right + enable Jumper
 *<br></br>
 *  Use the Poti to decrease the heartbeat of the main loop
 *  <br></br>
 *  <b> LED's:</b>
 *  left: show Vcc on/off
 *  middle: show activity
 *  red: show error
 *  Relais: show status -> LED on -> controlled device is on
 *
**/
/**
 * @file mBoard.h The main loop
 * @defgroup main Main Control Loop
 *
 * @brief This is the main file of the firmware
 * Here the necessary Functions to measure and scale Values, communicate and react are
 * implemented
 *
 * @author christian schaub
 */
/**@{*/

#ifndef MBOARD_H_
#define MBOARD_H_

/** @brief led test counter */
#define TIMERCOUNTER TCNT0
/** @brief prescaler led counter*/
#define TIMERCONTROL_B TCCR0B
/** @brief ports are ranged from 0 to 7 --- 10 for ACK */
#define ACK_CAN 10

#define LED_ALERT 14

#define RELAISSWITCHED 15

#define NOMEASUREBOARD 13

/** @brief when this flag is set we send */
uint8_t sendFlag = 55;
/** @brief sending intervals for each unit */
uint32_t NOISE_SEND = 60;
uint32_t VOL_SEND = 100;
uint32_t DISTANCE_SEND = 150;
uint32_t LIGHT_SEND  =170;
uint32_t TEMP_SEND  =200;



/** @brief the scalings (1,5,26,111 to calc the voltage, 0 means no measurement. 400 -> 111 as voltagefaktor */
uint8_t measuringVoltageFaktor[8];
/** @brief char buffer for measure conversion in string*/
char voltageValueBuffer [7];
/** @brief the measurement */
double measurement;
/** @brief led test last measure */
double ledLastMeasure;





uint8_t comboReactCheck1 = 0;
uint8_t comboReactCheck2 = 0;
uint8_t ledTick = 150;

/** @brief en-/disable serial line */
uint8_t serialCommunikation = 1;
/** @brief en-/disable can */
uint8_t canCommunikation = 1;

/** @brief en-/disable serial line kommunikation*/
uint8_t serialCommunikationK = 0;
/** @brief en-/disable can kommunikation*/
uint8_t canCommunikationK = 0;

/** @brief en-/disable reactivity*/
uint8_t reactions = 1;


uint8_t portToRelais [8]; /** @brief which port korresponds to which relais to react */

uint8_t reactionTriggerMode [8]; /** @brief should be switched when measure is higher or lower val */

uint32_t reactionValues [8]; /** @brief values interval when reaction is triggerd, without interval the relais jitters */

enum portMeasureModes {NOT_USED ,Temp, Light, Vol, Noise, Distance, Led}; /** @brief modes how we can measure */
//								0	1	  2	 	  3	 4	  5  	  6   7
uint8_t portModeMeasure [] = {Temp, /** @brief should be switched when measure is higher or lower val */
		Vol,
		Distance,
		Noise,
		0,
		0,
		Vol,
		0}; /** @brief which measure mode on specific port */

double intervalValues[7]; /** @brief the specific relais-switch interval to handle relais jittering: Temp 1C, Light 7%, Vol 0,5, Distance 5cm */

double distanceScale = 250; /**@brief  Vcc/512 * 2,54 (inch -> cm) */

double heartBeat = 50.0; /** @brief tick when the loop1...n ints are incremented 10 ms tune up xxx with poti */

double mea [200];

int loopCounter = 0; /** @brief the counter for the inner measure loop */

char te [33];

//-------------------------------------------------------------------------
/**
 * @brief sleep methode
 * self-made sleep method for better timing
 * @param the val in ms of sleeping
 */
extern void sleep(uint32_t val);

/**
 * @brief init the ports used as in or output and init the ADC with 125 kHz
 */
extern void initPorts(void);
/**
 * @brief init the multiplicator for the voltage calc 1, 5,24, or 111 for 400 V
 */
extern void initVoltageMultiplikator(void);

/**
 * @brief get the temperature measured with the temp-sensor
 */
extern double getTemp (void) ;

/**
 * @brief get the light intensity with a level of 0-100, 0 means its dark and 100 is very bright
 */
extern double getLightIntensity (void) ;

/**
 * @brief get noise level with the mic sensor, with level 0-100  100 is very loud
 */
extern double getNoise (void) ;

/**
 * @brief get the distance to the next object near to the ultrasonic sensor
 */
extern double getDistance (void) ;

/**
 * @brief measure voltage on port x
 */
extern uint16_t startMeasure(unsigned int X);

/**
 * @brief use startMeasure to measure and scale with given scalingfactor
 */
extern void measureAndScale (uint8_t port);

/**
 * @brief calculate the value of the real unit e.g. Distance in cm from measure
 * @param the port which was measured
 * @retval the recalculated measurement
 */

extern double getUnit(uint8_t port) ;

/**
 * @brief send a msg over CAN that the system has reacted by changing relais state
 * format: "re:r;port,state"
 * @param the port, high or low (1,0)
 */
extern void sendReactCAN(uint8_t port, uint8_t HoL) ;

/**
 * @brief reacts on measured values e.g. by setting a port High or Low to switch
 * the relais --- calls sendReactCan
 */
extern void reactOnMeasure (uint8_t port) ;

/**
 * @brief send measured data by uart
 * Format: P:n;x y \n --- n = portnumber, x = value, y = unit
 * @param port to measure
 */
extern void sendMeasureUART (uint8_t port);

/**
 * @brief generates a can message
 * Format: <br></br>
 * Relais ACK: "ok:port "<br></br>
 * General ACK and ping answer: "ackR|r :R|r " <br></br>
 * LED Alert: "led"<br></br>
 * physical Val: bits [0-5] value, bit[6] unit, bit [7] port
 * @param pointer to the message, modes: 11,12 relais 1,2 ; 0-9 measured port; 10 ACK
 */
extern void generateCANMsg(tExtendedCAN *M, uint8_t mode);

/*
 * @brief read a new config send by pc
 * format: A:n:m;n1:m1;M:n:m;n1;m1;T:n;C:x;y;z;E
 * @param the read char from line
 */
extern uint8_t readUARTConfig(unsigned char conf);

/*
 * @brief read new config modes from pc e.g. reactionvalues
 * format: H0:1;1:0;...;V0:20;1:300;E
 * @param the read char from line
 */
extern uint8_t readUARTConfModes(unsigned char conf);

/**
 * @brief reaction for given commands
 * @param the command given by uart
 */

extern uint8_t reactOnUARTCommand (unsigned char c);

/**
 * @brief reaction for given commands
 * @param the command given by can
 */
extern uint8_t reactOnCANCommand (char *data) ;

/**
 * @brief
 * initialize the can driver
 */
extern void can_init(void);

/**
 * @brief the main method with statemachine loop as follows:
 * statemachine ----!!!
 */
extern int main(void);

#endif /* MBOARD_H_ */
