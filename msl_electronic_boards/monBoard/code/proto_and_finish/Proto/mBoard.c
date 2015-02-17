#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

#include "defaults.h"
#include "mcp2515.h"
#include "uart.h"
#include "mBoard.h"

//----------------------------------------------------------------------------------


void sleep ( uint32_t ms ) {
	for(; ms > 0; ms--) _delay_ms(1);
}



void initPorts(void) {


	DDRC |= (1 << showON); // PC0 als Ausgang festlegen
	DDRC |= (1 << lastPin1); // PC1 als Ausgang festlegen
	DDRC |= (1 << lastPin2); // PC2 als Ausgang festlegen
	DDRC |= (1 << lastPin3); // PC3 als Ausgang festlegen

	DDRC |= (1 << errorLed);

	DDRB &= ~(1 << isON); // PB0 als Eingang festlegen
	PORTB |= (1 << isON); // Pullup für PB0 aktivieren

	//	PORTC &= ~(1 << lastPin1);

	// ADC anschalten mit vorteiler(frequenz) 128 -> 125 kHz -> 8µs
	// A normal conversion takes 13 ADC clock cycles -> 104µs

	ADCSRA  =  (1<<ADEN)  | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);

}


void initVoltageMultiplikator(void) {
	measuringVoltageFaktor[0]=0;
	measuringVoltageFaktor[1]=5;
	measuringVoltageFaktor[2]=0;
	measuringVoltageFaktor[3]=0;

	measuringVoltageFaktor[4]=0;
	measuringVoltageFaktor[5]=0; // gained
	measuringVoltageFaktor[CLOCK_REGULATOR]=0;
	measuringVoltageFaktor[7]=0;
	for(int i = 0; i < 8; i++) {
		portToRelais[i] = NO_RELAIS;
	}
	//portToRelais[0] = 0;
	//portToRelais[1] = 1;

	// in V * 10 -> 3.0 V = 30
	reactionValues[0] = DONT_REAC; //temp
	reactionValues[1] = DONT_REAC; //light
	reactionValues[2] = DONT_REAC;
	reactionValues[3] = DONT_REAC;
	reactionValues[4] = DONT_REAC; //distance cm
	reactionValues[5] = DONT_REAC; // noise
	reactionValues[6] = DONT_REAC; // never triggerd ... clock generator -- poti
	reactionValues[7] = DONT_REAC; // 400v

	// set 2 for no reaction
	reactionTriggerMode[0] = 2;
	reactionTriggerMode[1] = 2;
	reactionTriggerMode[2] = 2;
	reactionTriggerMode[3] = 2;
	reactionTriggerMode[4] = 2;
	reactionTriggerMode[5] = 2;
	reactionTriggerMode[6] = 2;
	reactionTriggerMode[7] = 2;

	intervalValues[Temp] = 1;
	intervalValues[Vol] = 0.5;
	intervalValues[Light] = 7;
	intervalValues[Distance] = 5;

}


double getTemp (void) {
	double tmp = 5.0;
	tmp = (measurement - tmp) * (-1);
	tmp /= 0.05; // steigung 0.05
	return tmp;
}


double getLightIntensity (void) {
	double tmp = (measurement/4.3)*100.0;
	tmp = 100.0 - tmp;
	return abs(tmp);
}


double getNoise (void) {
	return (measurement/5.0)*100.0;
}



double getDistance (void) {
	//dtostrf(measurement ,7,3, voltageValueBuffer);
	return (measurement *  distanceScale);
}



uint16_t startMeasure(unsigned int X) {
	uint16_t val = 0;
	ADMUX   =  X;
	// Referenz AVCC --> Vorsicht bei leerer Batterie !!!
	ADMUX  |=  (1<<REFS0);

	//Dummy lesen
	// wandlung an
	ADCSRA |=  (1<<ADSC);
	// auswerten
	while(ADCSRA & (1<<ADSC));
	val = ADCW;

	//echte messung
	val = 0;
	// wandlung an
	ADCSRA |=  (1<<ADSC);

	// auswerten
	while(ADCSRA & (1<<ADSC));
	val = ADCW;

	return val;
}

/**
 * @brief use startMeasure to measure and scale with given scalingfactor
 */
void measureAndScale (uint8_t port) {
	measurement = (double) startMeasure(port);
	if (measuringVoltageFaktor[port] == 1) {
		measurement = measurement * 0.00097;
	}
	if (measuringVoltageFaktor[port] == 5) {
		measurement = measurement * 0.0048 ;
	} else if (measuringVoltageFaktor[port]== 26) {
		measurement = measurement * 0.028;
	} else if (measuringVoltageFaktor[port]== 111) {
		measurement = measurement * 0.3906;
	}
}


double getUnit(uint8_t port) {
	switch (portModeMeasure[port]) {
	case Vol : return measurement; break;
	case Light : return getLightIntensity(); break;
	case Temp : return getTemp(); break;
	case Noise : return getNoise(); break;
	case Distance : return getDistance(); break;
	default : return -1;
	}
}


void sendReactCAN(uint8_t port, char HoL) {
	tExtendedCAN M;
	M.id[0] = 0x00;
	M.id[1] = PRIORITY_NORM;
	M.id[2] = 0x50;
	M.id[3] = 0x00;
	M.header.rtr = 0;
	M.header.length = 2;
	M.data[0] = HoL;
	M.data[1] = port;
}

void reactOnMeasure (uint8_t port) {
	double measureUnit = getUnit(port);

	switch (portToRelais[port]) {
	case NO_RELAIS: break;
	case 0:
		if (reactionTriggerMode[port] == HIGHER) { // measurement must be higher then value
			if (measureUnit < reactionValues[port] + intervalValues[portModeMeasure[port]] &&
					!(PINC & (1 << lastPin1))) {
				// deactivate
				PORTC |= (1 << lastPin1);
				if (serialCommunikation) {
					uart_puts("H0:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'H');
				}
			}
			if (measureUnit > reactionValues[port] &&
					(PINC & (1 << lastPin1))) {
				// deactivate
				PORTC &= ~(1 << lastPin1);
				if (serialCommunikation) {
					uart_puts("L0:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'L');
				}
			}
		}
		if (reactionTriggerMode[port] == LOWER) {
			if (measureUnit > reactionValues[port] &&
					!(PINC & (1 << lastPin1))) {
				// deactivate
				PORTC |= (1 << lastPin1);
				if (serialCommunikation) {
					uart_puts("H0:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'H');
				}
			}
			if (measureUnit < reactionValues[port] - intervalValues[portModeMeasure[port]] &&
					(PINC & (1 << lastPin1))) {
				// reactivate
				PORTC &= ~(1 << lastPin1);
				uart_puts("L0:");
				uart_putc((char)port +48);
				uart_putc('\n');
			}
		}
		break;
	case 1:
		if (reactionTriggerMode[port] == HIGHER) { // measurement must be higher then value
			if (measureUnit < reactionValues[port] + intervalValues[portModeMeasure[port]]  &&
					!(PINC & (1 << lastPin2))) {
				// deactivate
				PORTC |= (1 << lastPin2);
				if (serialCommunikation) {
					uart_puts("H1:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'H');
				}
			}
			if (measureUnit > reactionValues[port] &&
					(PINC & (1 << lastPin2))) {
				// deactivate
				PORTC &= ~(1 << lastPin2);
				if (serialCommunikation) {
					uart_puts("L1:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'L');
				}
			}
		}
		if (reactionTriggerMode[port] == LOWER) {
			if (measureUnit > reactionValues[port] - intervalValues[portModeMeasure[port]] &&
					!(PINC & (1 << lastPin2))) {
				PORTC |= (1 << lastPin2);
				if (serialCommunikation) {
					uart_puts("H1:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'H');
				}		//@TODO hier fehlt noch was ....s.o.

			}
			if (measureUnit < reactionValues[port] -5 &&
					(PINC & (1 << lastPin2))){
				// reactivate
				PORTC &= ~(1 << lastPin2);
				if (serialCommunikation) {
					uart_puts("L1:");
					uart_putc((char)port +48);
					uart_putc('\n');
				}
				if (canCommunikation) {
					sendReactCAN(port, 'L');
				}
			}
		}
		break;
		/*case 2:
		if (reactionValueMode[port] == HIGHER) { // measurement must be higher then value
			if (measureUnit > reactionValues[port]  &&
					!(PINC & (1 << lastPin3))) {
				// deactivate
				PORTC |= (1 << lastPin3);
				uart_puts("H:");
				uart_putc((char)port +48);
				uart_putc('\n');
			}
		}
		if (reactionValueMode[port] == LOWER) {
			if (measureUnit < reactionValues[port]) {
				// reactivate
				PORTC &= ~(1 << lastPin3);
				uart_puts("L:");
				uart_putc((char)port +48);
				uart_putc('\n');
			}
		}
		break;*/
	}
}



void sendMeasureUART (uint8_t port) {
	double reCalc = 0.0;
	switch (portModeMeasure[port]) {
	case Vol :
		reCalc = measurement;
		dtostrf(reCalc ,7,3, voltageValueBuffer);
		break;
	case Light :
		reCalc = getLightIntensity();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Temp :
		reCalc = getTemp();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Noise :
		reCalc = getNoise();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Distance :
		reCalc = getDistance();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Led :
		reCalc = measurement;
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	}
	uart_puts("P:");

	uart_putc((char) port +48);
	uart_putc(';');
	// +1 wegen leerzeichen ... -- vorzeichen platzhalter
	uart_puts(voltageValueBuffer+1);

	switch (portModeMeasure[port]) {
	case Vol : uart_puts(" V"); break;
	case Light : uart_puts(" L"); break;
	case Temp : uart_puts(" C"); break;
	case Noise : uart_puts(" S"); break;
	case Distance : uart_puts(" cm"); break;
	case Led : uart_puts(" Led"); break;
	}

	uart_putc('\n');
}


void generateCANMsg(tExtendedCAN *M, uint8_t port) {
	// 0x00, priority, sender, receiver
	uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, COMPASS_ID};
	double reCalc = 0.0;
	if (port > 10) { // relais ack 11,12 .. -> 1,2,...

		M->id[0] = 0x00;
		M->id[1] = 0x90;
		M->id[2] = M_BOARD_ID2;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 3;
		M->data[0] = (uint8_t) 'o';
		M->data[1] = (uint8_t) 'k';
		M->data[2] = (port -10);
	}
	if (port == 10) { // ACK
		M->id[0] = 0x00;
		M->id[1] = 0x90;
		M->id[2] = M_BOARD_ID2;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 2;
		M->data[0] = (uint8_t) 'o';
		M->data[1] = (uint8_t) 'k';
	} else {
		M->id[0] = 0x00;
		M->id[1] = 0x90;
		M->id[2] = M_BOARD_ID2; // sender
		M->id[3] = 0x00; // receiver
		M->header.rtr = 0;
		M->header.length = 8;
		switch (portModeMeasure[port]) {
		case Vol : reCalc = measurement; break;
		case Light : reCalc = getLightIntensity(); break;
		case Temp : reCalc = getTemp(); break;
		case Noise : reCalc = getNoise(); break;
		case Distance :	reCalc = getDistance(); break;
		}
		switch (portModeMeasure[port]) {
		case Vol :
			dtostrf(reCalc ,5,3, M->data);
			M->data[6] = 'V';
			break;
		case Light :
			dtostrf(reCalc ,5,1, M->data);
			M->data[6] = 'L';
			break;
		case Temp :
			dtostrf(reCalc ,5,1, M->data);
			M->data[6] = 'C';
			break;
		case Noise :
			dtostrf(reCalc ,5,1, M->data);
			M->data[6] = 'S';
			break;
		case Distance :
			dtostrf(reCalc ,5,1, M->data);
			M->data[6] = 'D';
			break;
		}
		M->data[5] = ' ';
		M->data[7] = port ;
	}
}

uint8_t readUARTConfig(unsigned char conf) {
	//uint8_t* com;
	uint8_t val = 0;
	uint8_t count = 0;

	if (conf == 'A') {// ADC Channel
		for (uint8_t i = 0;i < 8;i++) {
			count = uart_getc();

			unsigned char a = uart_getc();

			if(a != ':') return 0;
			a = uart_getc();
			if (a == '2') {
				measuringVoltageFaktor[count - '0'] = 26;
				uart_puts("26 set \n");
			}
			if (a == '5') {
				measuringVoltageFaktor[count - '0'] = 5;
				uart_puts("5 set \n");
			}
			if (a == '4') {
				measuringVoltageFaktor[count -'0'] = 111;
				uart_puts("400 set \n");
			}
			a = uart_getc();
			if(a != ';') return 0;
		}
	}

	val = uart_getc();
	if (val == 'E') {
		uart_puts("\n\n ---  new config set !!! \n\n");
		return 1;
	}

	if (val == 'M') { // modes M0:1;1:2;
		for (uint8_t i = 0;i < 8;i++) {
			count = uart_getc();

			unsigned char a = uart_getc();

			if(a != ':') return 0;
			a = uart_getc();
			if (a == '1') {
				portModeMeasure[count - '0'] = Temp;
				uart_puts("temp set \n");
			}
			if (a == '2') {
				portModeMeasure[count - '0'] = Light;
				uart_puts("light set \n");
			}
			if (a == '3') {
				portModeMeasure[count - '0'] = Vol;
				uart_puts("vol set \n");
			}
			if (a == '4') {
				portModeMeasure[count - '0'] = Noise;
				uart_puts("noise set \n");
			}
			if (a == '5') {
				portModeMeasure[count - '0'] = Distance;
				uart_puts("distance set \n");
			}

			a = uart_getc();
			if(a != ';') return 0;
		}
	}

	val = uart_getc();

	if (val == 'E') {
			uart_puts("\n\n ---  new config set !!! \n\n");
			return 1;
	}

	if (val == 'T') {
		val = uart_getc();
		//	tmp = tmp -'0';
		heartBeat = (uint16_t) (100 * (val- '0'));
		if(uart_getc() != ';') return 0;
	}
	val = uart_getc();
	if (val == 'E') {
			uart_puts("\n\n ---  new config set !!! \n\n");
			return 1;
		}
	if (val == 'C') {
		val = uart_getc();
		if (val == 'S')  serialCommunikation = 1;// serial
		if (val == 's')  serialCommunikation = 0;// serial
		val = uart_getc();
		if (val == 'C')  canCommunikation = 1;//can
		if (val == 'c')  canCommunikation = 0;//can
		val = uart_getc();
		if (val == 'N')  ethCommunikation = 1; // eth
		if (val == 'n')  ethCommunikation = 0; // eth
	}
	val = uart_getc();

	if (val == 'E') {
		uart_puts("\n\n ---  new config set !!! \n\n");
		return 1;

	}
	return 0;
}


uint8_t readUARTConfModes(unsigned char conf) {
	//uint8_t* com;
	uint8_t val = 0;
	uint8_t count = 0;

	if (conf == 'H') {// higher or lower mode
		for (uint8_t i = 0;i < 8;i++) {
			count = uart_getc();

			unsigned char a = uart_getc();

			if(a != ':') return 0;
			a = uart_getc();
			if (a == '1') {
				reactionTriggerMode[count - '0'] = HIGHER;
				uart_puts("higher set \n");
			}
			if (a == '0') {
				reactionTriggerMode[count - '0'] = LOWER;
				uart_puts("lower set \n");
			}
			a = uart_getc();
			if(a != ';') return 0;
		}
	}

	val = uart_getc();

	if (val == 'V') { // values but how ???
		for (uint8_t i = 0;i < 8;i++) {
			count = uart_getc();

			unsigned char a = uart_getc();
			char v[2];
			if(a != ':') return 0;
			v[0] = uart_getc();
			v[1] = uart_getc();
			//	int x = atoi[v];
			//	reactionValues[count - '0'] = ((double) x /10);
			a = uart_getc();
			if(a != ';') return 0;
		}
	}

	val = uart_getc();

	if (val == 'E') {
		uart_puts("\n\n ---  new config set !!! \n\n");
		return 1;

	}
	return 0;
}

void printVreal(uint8_t port) {
	static int x = 0;
	char buf[33];
	itoa(x, buf, 10);
	double reCalc = 0.0;
	switch (portModeMeasure[port]) {
	case Vol :
		reCalc = measurement;
		dtostrf(reCalc ,7,3, voltageValueBuffer);
		break;
	case Light :
		reCalc = getLightIntensity();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Temp :
		reCalc = getTemp();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Noise :
		reCalc = getNoise();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	case Distance :
		reCalc = getDistance();
		dtostrf(reCalc ,7,1, voltageValueBuffer);
		break;
	}
	uart_puts(buf);
	uart_putc('\t');
	uart_puts(voltageValueBuffer+1);
	uart_putc('\n');
	x++;
}

void printVmeasure(uint8_t port) {
	static int x = 0;
	char buf[33];
	itoa(x, buf, 10);
	double reCalc = 0.0;

	reCalc = measurement;
	dtostrf(reCalc ,7,3, voltageValueBuffer);

	uart_puts(buf);
	uart_putc('\t');
	uart_puts(voltageValueBuffer+1);
	uart_putc('\t');
	uart_putc((char) port +48);
	uart_putc('\n');
	x++;
}


uint8_t reactOnUARTCommand (unsigned char c) {
	unsigned char port = 0;
	unsigned char com = 0;
	unsigned char isRead = 0;
	unsigned char * value = 0;
	uart_puts("react \n");
	switch (c) {
	case 's':
		set_sleep_mode(SLEEP_MODE_IDLE);
		com = 1;
		sleep_mode();                   // in den Schlafmodus wechseln

		break;
	case 'p' : // Ping
		uart_puts("p:ok      \n");
		com=1;
		break;
	case 'R' : //Relais anschalten
		port = uart_getc();

		switch(port - '0') {
		case 1 : PORTC |= (1 << lastPin1); com=1;break;
		case 2 : PORTC |= (1 << lastPin2); com=1;break;
		case 3 : PORTC |= (1 << lastPin3); com=1;break;
		}

		if (com) uart_puts("R:ok      \n"); // TODO   anschauen wegen dem ringbuffer --
		break;
		case 'r' : //Relais ausschalten
			port = uart_getc();

			switch(port - '0') {
			case 1 : PORTC &= ~(1 << lastPin1); com=1;break;
			case 2 : PORTC &= ~(1 << lastPin2); com=1;break;
			case 3 : PORTC &= ~(1 << lastPin3); com=1;break;
			}
			if (com) uart_puts("r:ok      \n");
			break;
			case 'c' : // config board
				isRead = readUARTConfig(uart_getc());
				break;
	}
	// read to end, or no command set
	if ((com==1 && uart_getc() == 'E') || isRead ==1) return 1;
	return 0;
}


uint8_t reactOnCANCommand (char *data) {
	unsigned char port = data[1];
	unsigned char com = data[0];
	unsigned char isRead = 0;
	unsigned char * value = 0;
	tExtendedCAN M;

	switch (com) {
	case 's':
		set_sleep_mode(SLEEP_MODE_IDLE);
		isRead = 1;
		sleep_mode();                   // in den Schlafmodus wechseln

		break;
	case 'p' : // Ping
		generateCANMsg(&M, 10);
		PORTC |= (1 << errorLed);
		uart_puts("ping by can \n");
		mcp2515_send_extmessage(&M);
		M.data[0] = 0;
		isRead = 1;
		break;
	case 'R' : //Relais anschalten
		switch(port - '0') {
		case 1 : PORTC |= (1 << lastPin1);
		generateCANMsg(&M, 11);
		mcp2515_send_extmessage(&M);
		break;
		case 2 : PORTC |= (1 << lastPin2);
		generateCANMsg(&M, 12);
		mcp2515_send_extmessage(&M);
		break;
		case 3 : PORTC |= (1 << lastPin3);
		generateCANMsg(&M, 13);
		mcp2515_send_extmessage(&M);
		break;
		default : break;
		}
		uart_puts("relais by can \n");
		M.data[0] = 0;
		isRead=1;
		break;
		case 'r' : //Relais ausschalten
			switch(port - '0') {
			case 1 :
				PORTC &= ~(1 << lastPin1);

				generateCANMsg(&M, 10);
				mcp2515_send_extmessage(&M);
				break;
			case 2 :
				PORTC &= ~(1 << lastPin2);

				generateCANMsg(&M, 10);
				mcp2515_send_extmessage(&M);
				break;
			case 3 :
				PORTC &= ~(1 << lastPin3);

				generateCANMsg(&M, 10);
				mcp2515_send_extmessage(&M);
				break;
			default : break;

			}
			M.data[0] = 0;
			isRead=1;
			break;
	}

	// read to end, or no command set
	if ( isRead ==1) return 1;
	return 0;
}


void can_init(void) {
	if (!mcp2515_init()) {
		uart_puts("Fehler: kann MCP2515 nicht ansprechen!\n\n");
	}
	else {
		uart_puts("MCP2515 is aktiv\n\n");
	}
}




void startTimerLed(void) {
	TIMERCONTROL_B |= (1 << CS00) | (1 << CS01); // 64 --> 32 ms f = clock/ 2*pre*(1+OCR) hier OCR 255
}


uint8_t  checkLedTime(void) {
	uint8_t a = 0;

	return a;
}

int main(void) {

	sei(); /** enable interrupts */
	initPorts(); /** init ports*/
	initVoltageMultiplikator();/** init scalings */

	uart_init(UART_BAUD_SELECT(9600UL, 16000000UL));/** init uart */

	if (LOGGING) { /** welcome message uart*/
		uart_puts("Hi have fun using the System  !! \n");
		uart_puts("Systems running in general mode  !! \n");
	}

	can_init(); /** init can */


	PORTC |= (1 << showON); // PC0 aktivieren
	PORTC |= (1 << errorLed);
	PORTC &= ~(1 << errorLed);
	uint8_t portCounter = 0;
	uint8_t portCounter1 = 0;

	uint8_t isMeasured = 0;

	TCNT1 = 0;
	TCCR1B |= (1 << CS12) | (1 << CS10); /** CS12 1 CS11 0 CS10 1 clkI/O/1024 (From prescaler)*/
	int ledTicker = 0;
	heartBeat = 500;
	while ( 1 ) {/** main loop */
		// Schaltung aktiv ? Schalter muss auf Masse ziehen
		//	if (!(PINB & (1 << isON))) {
		PORTC ^= (1 << showON);/** toggle status led */
		if (portCounter == 8) {
			portCounter =0;
		}
		loopCounter = 0.0;
		while ( 1 ) { /** the measure loop  */
			if (loopCounter > heartBeat) {
				break;
			}
			if (portCounter1 == 8) {
				portCounter1=0;
			}
			if (measuringVoltageFaktor[portCounter1] != 0) {/** measure required ??*/
				measureAndScale(portCounter1);
				if (reactionValues[portCounter1] != DONT_REAC) {/** reaction required ??*/
					reactOnMeasure(portCounter1);
				}
				if (portModeMeasure[portCounter1] == Led) { /** led check required ?? */
					if (ledLastMeasure == -1) { /** -1 is the initial value */
						ledLastMeasure = measurement;
					} else {/** check the status of the led with the light sensor*/
						uint32_t last = (int) (ledLastMeasure * 100);
						uint32_t current = (int) (measurement * 100);
						if (abs(current -last) > 4 ) {
							uart_puts("L:ok \n");
							ledTicker =0;
						/*	if (!(PINC & (1 << lastPin1))) {
								PORTC |= (1<< lastPin1);
							} else {
								PORTC &= ~(1 <<lastPin1);
							}*/
						//	dtostrf(current ,7,3, voltageValueBuffer);
						//	uart_puts(voltageValueBuffer);
						//	dtostrf(last ,7,3, voltageValueBuffer);
						//	uart_puts(voltageValueBuffer);
						//	uart_puts("\n");
						} else {
							if (ledTicker > 1800) {
								uart_puts("L:alert\n");
								ledTicker=0;
							}
							ledTicker++;
						}
						ledLastMeasure = measurement;
					}
				}
			}
			portCounter1++;
			loopCounter++;
		}
		uint8_t ledMode = 0;
		if (serialCommunikation) {/** check uart commands and send data by uart when activated */
			char c = uart_getc();
			if (c == 'c' || c == 'p' || c == 's' || c == 'R' || c == 'r') {
				if (!reactOnUARTCommand(c)) {/** reaction required ??*/
					uart_puts(" command UART ");
					uart_putc(c);
					uart_puts(" read incomplete !!!\n");
					PORTC |= (1 << errorLed);
				}
			}
			if (measuringVoltageFaktor[portCounter] != 0) {/** measure required ??*/
				// messung
				measureAndScale(portCounter); // 2 messungen
				// Reaktionen noetig ?
				if (reactionTriggerMode[portCounter] != DONT_REAC) {
					reactOnMeasure(portCounter);
				}
				// Messung abschicken - UART
				sendMeasureUART(portCounter);
			}
		}
		if (canCommunikation) {/** check can commands and send data when activated */
			//	testCan();
			tExtendedCAN M;
			tExtendedCAN m1;
			// CAN message auf bus ?
			if (mcp2515_get_extmessage(&m1) != 0) {
				if (m1.data[2] - '0' == 4) {
					// command von CAN ?
					if (m1.data[0] == 'p' || m1.data[0] == 'c' || m1.data[0] == 's' || m1.data[0] == 'R' || m1.data[0] == 'r') {
						if (reactOnCANCommand(m1.data) == 0) {
							uart_puts(" command CAN ");
							uart_putc(m1.data[0]);
							uart_puts(" read incomplete !!!\n");
						}
					}
				}

			}
			if (measuringVoltageFaktor[portCounter] != 0) {
				// messung
				measureAndScale(portCounter); // 2 messungen
				// Reaktionen noetig ?
				if (reactionTriggerMode[portCounter] != DONT_REAC) {
					reactOnMeasure(portCounter);
				}
				// messung schicken per CAN
				generateCANMsg(&M, portCounter);
				mcp2515_send_extmessage(&M);
			}
		}
		if (portCounter == CLOCK_REGULATOR) {/** recalculate the heartBeat if the poti port is measured */
			//heartBeat = 180.0;
			// get a value between 300 and 1500 ms
		//	heartBeat =  heartBeat * ((measurement < 1.0) ? 1.0 : measurement*10.0);
		}
		portCounter++;
		PORTC &= ~(1 << errorLed);
//		uart_puts("\n\n");
		//	sleep(heartBeat);
	}
	PORTC &= ~(1 << showON);
}




