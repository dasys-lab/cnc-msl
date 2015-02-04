#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

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
	DDRC |= (1 << showCommand); // PC5 als Ausgang festlegen
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
	measuringVoltageFaktor[0]=5;
	measuringVoltageFaktor[1]=5;
	measuringVoltageFaktor[2]=5;
	measuringVoltageFaktor[3]=5;

	measuringVoltageFaktor[4]=0;// gained
	measuringVoltageFaktor[5]=0;
	measuringVoltageFaktor[CLOCK_REGULATOR]=5;
	measuringVoltageFaktor[7]=0;
	for(int i = 0; i < 8; i++) {
		portToRelais[i] = NO_RELAIS;
	}
	// measurement on port 0 -> relais 1
	portToRelais[0] = 1;
	//	portToRelais[1] = 1;

	// in V * 10 -> 3.0 V = 30
	reactionValues[0] = 22; //temp
	reactionValues[1] = DONT_REAC; //voltage
	reactionValues[2] = DONT_REAC; //distance 50 cm
	reactionValues[3] = DONT_REAC;
	reactionValues[4] = DONT_REAC;
	reactionValues[5] = DONT_REAC; // noise + distance -> combo
	reactionValues[6] = DONT_REAC; // never triggerd ... clock generator -- poti
	reactionValues[7] = DONT_REAC; // 400v

	// set 2 for no reaction
	reactionTriggerMode[0] = LOWER;
	reactionTriggerMode[1] = 2;
	reactionTriggerMode[2] = 2;
	reactionTriggerMode[3] = 2;
	reactionTriggerMode[4] = 2;
	reactionTriggerMode[5] = 2;
	reactionTriggerMode[6] = 2;
	reactionTriggerMode[7] = 2;

	intervalValues[Temp] = 0.5;
	intervalValues[Vol] = 0.5;
	intervalValues[Light] = 15;
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
	return measurement;
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


void sendReactCAN(uint8_t port, uint8_t HoL) {
	tExtendedCAN *M;

			uint8_t porttmp = port -RELAISSWITCHED;
			M->id[0] = 0x00;
			M->id[1] = PRIORITY_NORM;
			M->id[2] = 0x50;
			M->id[3] = 0x00;
			M->header.rtr = 0;
			M->header.length = 7;
			M->data[0] = (uint8_t) 'e';
			M->data[1] = (uint8_t) 'x';
			M->data[2] = (HoL) ? (uint8_t) 'R' : (uint8_t) 'r';
			M->data[3] = (uint8_t) ' ';
			M->data[4] = (uint8_t) ':';
			M->data[5] = port;
			M->data[6] = ' ';
			mcp2515_send_extmessage(M);
}

void reactOnMeasure (uint8_t port) {
	double measureUnit = getUnit(port);

	switch (portToRelais[port]) {
	case NO_RELAIS: break;
	case 0: // trigerd by pc
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
				// reactivate
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
			if (measureUnit < reactionValues[port] - intervalValues[portModeMeasure[port]]  &&
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
			if (measureUnit > reactionValues[port] + intervalValues[portModeMeasure[port]]&&
					(PINC & (1 << lastPin2))) {
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
		if (reactionTriggerMode[port] == LOWER) {
			if (measureUnit > reactionValues[port] + intervalValues[portModeMeasure[port]] &&
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
			if (measureUnit < reactionValues[port] - intervalValues[portModeMeasure[port]] &&
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
		dtostrf(reCalc ,7,3, voltageValueBuffer);
		break;
	case Distance :
		reCalc = getDistance();
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
	}

	uart_putc('\n');
}


void generateCANMsg(tExtendedCAN *M, uint8_t port) {
	// 0x00, priority, sender, receiver
	uint8_t id[4] = {0x00, PRIORITY_NORM, REKICK_ID, COMPASS_ID};
	char buff[5];
	double reCalc = 0.0;
	if (port == 11 || port == 12) { // relais (triggerd by sys) ACK 11,12 .. -> 1,2,...
		/*		M->id[1] = 0x90;
		M->id[2] = M_BOARD_ID2;*/
		M->id[0] = 0x00;
		M->id[1] = PRIORITY_NORM;
		M->id[2] = 0x50;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 5;
		M->data[0] = (uint8_t) 'o';
		M->data[1] = (uint8_t) 'k';
		M->data[2] = (uint8_t) ':';
		M->data[3] = (port -10);
		M->data[4] = (uint8_t) ' ';
	}
	if (port == NOMEASUREBOARD) {
		M->id[0] = 0x00;
		M->id[1] = PRIORITY_NORM;
		M->id[2] = 0x50;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 6;
		M->data[0] = (uint8_t) 'n';
		M->data[1] = (uint8_t) 'o';
		M->data[2] = (uint8_t) 'b';
		M->data[3] = (uint8_t) 'o';
		M->data[4] = (uint8_t) 'a';
		M->data[5] = (uint8_t) 'r';
	}
	if (port == LED_ALERT) {
		M->id[0] = 0x00;
		M->id[1] = PRIORITY_NORM;
		M->id[2] = 0x50;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 3;
		M->data[0] = (uint8_t) 'l';
		M->data[1] = (uint8_t) 'e';
		M->data[2] = (uint8_t) 'd';
	}
	if (port == ACK_CAN) { // ACK
		M->id[0] = 0x00;
		M->id[1] = PRIORITY_NORM;
		M->id[2] = 0x50;
		M->id[3] = 0x00;
		M->header.rtr = 0;
		M->header.length = 7;
		M->data[0] = (uint8_t) 'a';
		M->data[1] = (uint8_t) 'c';
		M->data[2] = (uint8_t) !(PINC & (1<< lastPin1)) ? 'R' : 'r';
		M->data[3] = (uint8_t) ' ';
		M->data[4] = (uint8_t) ':';
		M->data[5] = (uint8_t) !(PINC & (1<< lastPin2)) ? 'R' : 'r';
		M->data[6] = (uint8_t) ' ';
	} else {
		M->id[0] = 0x00;
		M->id[1] = PRIORITY_NORM;
		M->id[2] = 0x50; // sender
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
			dtostrf(reCalc ,5,1, buff);
			M->data[6] = 'V';
			break;
		case Light :
			dtostrf(reCalc, 5,2, buff);
			M->data[4] = ' ';
			M->data[5] = ' ';
			M->data[6] = 'L';
			break;
		case Temp :
			dtostrf(reCalc ,5,2, buff);
			M->data[5] = ' ';
			M->data[6] = 'C';
			break;
		case Noise :
			dtostrf(reCalc ,5,1, buff);
			M->data[6] = 'S';
			break;
		case Distance :
			dtostrf(reCalc ,5,1, buff);
			M->data[4] = ' ';
			M->data[5] = ' ';
			M->data[6] = 'D';
			break;
		}
		memcpy((void *)M->data, (const void *)buff, 5);
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
		if (val == 'N')  /*fr*/;
		if (val == 'n') /*fr*/;
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

	case 'p' : // Ping
		serialCommunikationK=1;
		if (uart_getc() == 'E') {
			uart_puts("ok      \n");
			com=1;
		}
		break;
	case 'R' : //Relais anschalten
		port = uart_getc();

		switch(port - '0') {
		case 1 : PORTC |= (1 << lastPin1); com=1;break;
		case 2 : PORTC |= (1 << lastPin2); com=1;break;
		case 3 : PORTC |= (1 << lastPin3); com=1;break;
		}

		if (com && uart_getc() == 'E') uart_puts("ok      \n"); // TODO   anschauen wegen dem ringbuffer --
		break;
		case 'r' : //Relais ausschalten
			port = uart_getc();

			switch(port - '0') {
			case 1 : PORTC &= ~(1 << lastPin1); com=1;break;
			case 2 : PORTC &= ~(1 << lastPin2); com=1;break;
			case 3 : PORTC &= ~(1 << lastPin3); com=1;break;
			}
			if (com && uart_getc() == 'E') uart_puts("ok      \n");
			break;
			case 'c' : // config board
				isRead = readUARTConfig(uart_getc());
				break;
	}
	// read to end, or no command set
	if (com==1  || isRead ==1) {
		PORTC |= (1 << showCommand);
		return 1;
	}
	return 0;
}


uint8_t reactOnCANCommand (char *data) {
	unsigned char port = data[1];
	unsigned char com = data[0];
	unsigned char isRead = 0;
	unsigned char * value = 0;
	uint8_t val = 0;
	uint8_t unit = 0;
	tExtendedCAN M;

	switch (com) {
	case 'p' : // Ping
		canCommunikationK = 1;
		generateCANMsg(&M, 10);
		PORTC |= (1 << showCommand);
		//		uart_puts("ping by can \n");
		mcp2515_send_extmessage(&M);
		M.data[0] = 0;
		isRead = 1;
		break;
	case 'c' : // config   "c n m" n val , m unit 1-5
		val = (uint8_t)data[1];
		val*= 10;
		unit = data[2] -'0';
		switch(unit) {
		case 0 : TEMP_SEND = val;break;

		}
		generateCANMsg(&M, 10);
		mcp2515_send_extmessage(&M);
		PORTC |= (1 << showCommand);
		M.data[0] = 0;
		isRead = 1;
		break;
	case 'R' : //Relais anschalten
			switch(port - '0') {
			case 1 : PORTC |= (1 << lastPin1);

			generateCANMsg(&M, 11);
			mcp2515_send_extmessage(&M);
			uart_puts(data);

			uart_puts("relais1 on by can \n");

			break;
			case 2 : PORTC |= (1 << lastPin2);

			generateCANMsg(&M, 12);
			mcp2515_send_extmessage(&M);
			uart_puts(data);
			uart_puts("relais2 on by can \n");

			break;
			case 3 : PORTC |= (1 << lastPin3);
			generateCANMsg(&M, 13);
			mcp2515_send_extmessage(&M);
			break;
			default : break;
			}
			PORTC |= (1 << showCommand);
			M.data[0] = 0;
			isRead=1;
			break;
	case 'r' : //Relais ausschalten
				switch(port - '0') {
				case 1 :
					PORTC &= ~(1 << lastPin1);

					generateCANMsg(&M, 10);
					mcp2515_send_extmessage(&M);
					uart_puts(data);
					uart_puts("relais1 off by can \n");

					break;
				case 2 :
					PORTC &= ~(1 << lastPin2);

					generateCANMsg(&M, 10);
					mcp2515_send_extmessage(&M);
					uart_puts(data);
					uart_puts("relais2 off by can \n");

					break;
				case 3 :
					PORTC &= ~(1 << lastPin3);

					generateCANMsg(&M, 10);
					mcp2515_send_extmessage(&M);
					break;
				default : break;

				}

				PORTC |= (1 << showCommand);
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

	uint8_t portCounter = 0;
	uint8_t portCounter1 = 0;
	uint32_t errorCounter =0;
	uint8_t errorLedResetCounter=0;

	uint32_t loop1 = 0;
	uint32_t loop2 = 0;
	uint32_t loop3 = 0;
	uint32_t loop4 = 0;
	uint32_t loop5 = 0;

	uint32_t loop6 =0;


	uint8_t isMeasured = 0;

	TCNT1 = 0;
	TCCR1B |= (1 << CS12) | (1 << CS10); /** CS12 1 CS11 0 CS10 1 clkI/O/1024 (From prescaler)*/

	/**
	 * @brief Main loop
	 * The loop Timer increments a counter each time it reaches its max. 65536
	 *
	 */
	while ( 1 ) {/** main loop */
		// Schaltung aktiv ? Schalter muss auf Masse ziehen
		if ((PINB & (1 << isON))) {
			if (portCounter == 8) {
				portCounter =0;
			}
			//	loopCounter = 0;
			while ( 1 ) { /** the measure loop  */
				/*	if (loopCounter > heartBeat) {
				break;
			}*/// heartBeat version
				if (TCNT1 > (int)heartBeat) { // timer reached 65536 (4 s) old ver., increment counter
					loop1++; // new ver. 10 ms
					loop2++;
					loop3++;
					loop4++;
					loop5++;
					loop6++;
					//TIFR1 |= (1<<TOV1);
					TCNT1 = 0;
				}
				if (loop1 > TEMP_SEND ) {
					sendFlag = 0;
					loop1 = 0;//PORTC |= (1 << errorLed);
					break;
				}

				if (loop2 > VOL_SEND ) {
					sendFlag = 1;
					loop2 = 0;//PORTC |= (1 << errorLed);
					break;
				}

				if (loop4 > DISTANCE_SEND ) {
					sendFlag = 2;
					loop4=0;
					break;
				}


				if (loop5 > NOISE_SEND) {
					sendFlag = 3;
					loop5=0;
					break;
				}

				if (portCounter1 == 8) {
					portCounter1=0;
				}
				if (measuringVoltageFaktor[portCounter1] != 0) {/** measure required ??*/
					measureAndScale(portCounter1);
					if (reactionValues[portCounter1] != DONT_REAC) {/** reaction required ??*/
						if (reactionValues[portCounter1] == DONT_REAC+1) {
							//	reactOnCombo(portCounter1);
						} else {
							reactOnMeasure(portCounter1);
						}
					}
				}
				if (portCounter1 == CLOCK_REGULATOR && loop6 > 20) {/** recalculate the heartBeat if the poti port is measured */
					heartBeat = 50.0;
					// get a value between xxxx ms
					//	measureAndScale(6);
					//	sendMeasureUART(6);
					heartBeat =  heartBeat * ((measurement < 1.0) ? 1.0 : measurement);
					loop6 = 0;
				}
				portCounter1++;
				//	loopCounter++;
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
				if (measuringVoltageFaktor[sendFlag] != 0) {/** measure required ??*/
					// messung
					measureAndScale(sendFlag); // 2 messungen
					// Reaktionen noetig ?
					if (reactionTriggerMode[sendFlag] != DONT_REAC) {
						reactOnMeasure(sendFlag);
					}
					// Messung abschicken - UART
					if (serialCommunikationK) {
						sendMeasureUART(sendFlag);
						PORTC ^= (1 << showON);/** toggle status led */
					}
				}
			}
			if (canCommunikation) {/** check can commands and send data when activated */
				//	testCan();
				tExtendedCAN M;
				tExtendedCAN m1;
				// CAN message auf bus ?
				if (mcp2515_get_extmessage(&m1) != 0) {
					if (m1.data[2]-'0'  == 2) {
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
				if (measuringVoltageFaktor[sendFlag] != 0) {
					// messung
					measureAndScale(sendFlag); // 2 messungen
					// Reaktionen noetig ?
					if (reactionTriggerMode[sendFlag] != DONT_REAC) {
						reactOnMeasure(sendFlag);
					}
					// messung schicken per CAN
					if (canCommunikationK) {
						generateCANMsg(&M, sendFlag);
						mcp2515_send_extmessage(&M);
						PORTC ^= (1 << showON);/** toggle status led */

					}
				}
			}

			sendFlag = 55;

			portCounter++;
			if (errorLedResetCounter == 3) {
				PORTC &= ~(1 << errorLed);
				errorLedResetCounter = 0;
			}
			if (PORTC & (1 << errorLed)) errorLedResetCounter++;

			//	sleep(heartBeat);
		} else {
			tExtendedCAN M;
			generateCANMsg(&M, 116);
		}
	}
}




