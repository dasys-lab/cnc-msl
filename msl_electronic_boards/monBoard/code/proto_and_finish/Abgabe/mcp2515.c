/*// Neue Nachricht erzeugen
CANMessage message;

// Daten eintragen
message.id = 0x0123;
message.rtr = 0;
message.length = 2;
message.data[0] = 0x04;
message.data[1] = 0xf3;*/
// ----------------------------------------------------------------------------

/**
 * @file mcp2515.c
 * @brief
 * the functions to interact with the can bus controller and the spi bus
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "mcp2515.h"

#include "defaults.h"
#include "uart.h"


// -------------------------------------------------------------------------
// Schreibt/liest ein Byte ueber den Hardware SPI Bus

uint8_t spi_putc( uint8_t data )
{
	// put byte in send-buffer
	SPDR = data;
	
	// wait until byte was send
	while( !( SPSR & (1<<SPIF) ) )
		;
	
	return SPDR;
}

// -------------------------------------------------------------------------
void spi_init(void)
{
    // Aktivieren der Pins für das SPI Interface
	 PORTB &= ~((1<<P_SCK)|(1<<P_MOSI)|(1<<P_MISO));
    DDRB  |= (1<<P_SCK)|(1<<P_MOSI);
    DDRB &= ~(1<<P_MISO);

    DDRB  |= (1<<P_CS);
    PORTB  |= (1<<P_CS);

    // Aktivieren des SPI Master Interfaces, fosc = fclk / 2
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = /*(1<<SPI2X)*/0;
}

//-------------------------------------------------------------------------
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
    // /CS des MCP2515 auf Low ziehen
    PORTB &= ~(1<<P_CS);

    spi_putc(SPI_BIT_MODIFY);
    spi_putc(adress);
    spi_putc(mask);
    spi_putc(data);

    // /CS Leitung wieder freigeben
    PORTB |= (1<<P_CS);
}

//---------------------------------------------------

void testCan(void) {
	uart_puts("Erzeuge Nachricht\n");
	tCAN message;
	tCAN message2;

	// einige Testwerte
	message.id = 1;
	message.header.rtr = 0;
	message.header.length = 2;
	message.data[0] = 0x33;
	message.data[1] = 0x11;

	print_can_message(&message);

	uart_puts("\nwechsle zum Loopback-Modus\n\n");
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));

	// Sende eine Nachricht
	if (can_send_message(&message)) {
		uart_puts("Nachricht wurde in die Puffer geschrieben\n");
	}
	else {
		uart_puts("Fehler: konnte die Nachricht nicht senden\n");
	}

	// warte ein bisschen
	_delay_ms(100);

	//	if (check_message()) {
	//	uart_puts("Nachricht empfangen!\n");

	// read the message from the buffers
	if (can_get_message(&message2)) {
		uart_puts("Nachricht empfangen!\n");
		print_can_message(&message2);
		uart_puts("\n");
	}
	else {
		uart_puts("Fehler: konnte die Nachricht nicht auslesen\n\n");
	}
}


//---------------------------------------------------

void print_can_message(tCAN *message)
{
	uint8_t length = message->header.length;
	uint8_t rtr = message->header.rtr;

	uart_puts("id:    ");
	uart_putc( (char) message->id+48 );
	uart_putc('\n');
	uart_puts("laenge: ");
	uart_putc((char) length+48);
	uart_putc('\n');
	uart_puts("rtr:    ");
	uart_putc((char)rtr+48 );
	uart_putc('\n');

	if (!rtr) {
		uart_puts("daten:  ");

		for (uint8_t i = 0; i < length; i++) {
			uart_putc((char)message->data[i]+48);
			uart_puts(" ; ");
		}
		uart_puts("\n");
	}
}

//--------------------------------------------------------------------------
uint8_t mcp2515_init(void)
{
	uint8_t check = 0;
    // SPI Interface initialisieren
	PORTB |= (1<<P_CS);
	spi_init();

    // MCP2515 per Software Reset zuruecksetzten,
    // danach ist der MCP2515 im Configuration Mode
    PORTB &= ~(1<<P_CS);
    spi_putc( SPI_RESET );
    _delay_ms(5);
    PORTB |= (1<<P_CS);

    // etwas warten bis sich der MCP2515 zurueckgesetzt hat
    _delay_ms(10);

    /*
     *  Einstellen des Bit Timings
     *
     *  Fosc       = 16MHz
     *  BRP        = 7                (teilen durch 8)
     *  TQ = 2 * (BRP + 1) / Fosc  (=> 1 uS)
     *
     *  Sync Seg   = 1TQ
     *  Prop Seg   = (PRSEG + 1) * TQ  = 1 TQ
     *  Phase Seg1 = (PHSEG1 + 1) * TQ = 3 TQ
     *  Phase Seg2 = (PHSEG2 + 1) * TQ = 3 TQ
     *
     *  Bus speed  = 1 / (Total # of TQ) * TQ
     *             = 1 / 8 * TQ = 125 kHz
     */

    PORTB &= ~(1<<P_CS);
    	spi_putc(SPI_WRITE);
    	spi_putc(CNF3);

    	spi_putc((1<<PHSEG21));		// Bitrate 125 kbps at 16 MHz
    	spi_putc((1<<BTLMODE)|(1<<PHSEG11));

    	//ENDY: Hier kann man Bitrate ändern -> einfach ersten 2 Klammren weg
    	spi_putc(0);

    	// activate interrupts
    	//spi_putc((1<<RX1IE)|(1<<RX0IE));
    	PORTB |= (1<<P_CS);

    	// test if we could read back the value => is the chip accessible?
    	if (mcp2515_read_register(CNF1) != 0) {
    		return 0;
    	}


    // deaktivate the RXnBF Pins (High Impedance State)
    	mcp2515_write_register(BFPCTRL, 1);

    	// set TXnRTS as inputs
    	mcp2515_write_register(TXRTSCTRL, 0);

    	// turn off filters => receive any message
    	mcp2515_write_register(RXB0CTRL, (1<<RXM1)|(1<<RXM0));
    	mcp2515_write_register(RXB1CTRL, (1<<RXM1)|(1<<RXM0));

    	// reset device to normal mode
    	// mcp2515_write_register(CANCTRL, 0);
    	// Takt wird 1/1 an atmega weitergegeben
    	mcp2515_write_register(CANCTRL, 0x04);
    	// Takt wird 1/2 an atmega weitergegeben
    	//mcp2515_write_register(CANCTRL, 0x05);

    return 1;
}


// -------------------------------------------------------------------------
void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	PORTB &= ~(1<<P_CS);
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	PORTB |= (1 <<P_CS);
}

//--------------------------------------------------------------------------
uint8_t mcp2515_read_register(uint8_t adress)
{
    uint8_t data;

    // /CS des MCP2515 auf Low ziehen
    PORTB &= ~(1<<P_CS);

    spi_putc(SPI_READ);
    spi_putc(adress);

    data = spi_putc(0xff);

    // /CS Leitung wieder freigeben
    PORTB |= (1<<P_CS);

    return data;
}

//---------------------------------------------------------------------

uint8_t mcp2515_read_rx_status(void)
{
    uint8_t data;

    // /CS des MCP2515 auf Low ziehen
    PORTB &= ~(1<<P_CS);

    spi_putc(SPI_RX_STATUS);
    data = spi_putc(0xff);

    // Die Daten werden noch einmal wiederholt gesendet,
    // man braucht also nur eins der beiden Bytes auswerten.
    spi_putc(0xff);

    // /CS Leitung wieder freigeben
    PORTB |= (1<<P_CS);

    return data;
}

//----------------------------------------------------------------------------

uint8_t can_get_message(tCAN *p_message)
{
    // Status auslesen
    uint8_t status = mcp2515_read_rx_status();


        // Nachricht in Puffer 0

        PORTB &= ~(1<<P_CS);    // CS Low
        spi_putc(SPI_READ_RX);

  /*      // Nachricht in Puffer 1

        PORT_CS &= ~(1<<P_CS);    // CS Low
        spi_putc(SPI_READ_RX | 0x04); */


    // Standard ID auslesen
    p_message->id =  (uint16_t) spi_putc(0xff) << 3;
    p_message->id |= (uint16_t) spi_putc(0xff) >> 5;

    spi_putc(0xff);
    spi_putc(0xff);

    // Laenge auslesen
    uint8_t length = spi_putc(0xff) & 0x0f;
    p_message->header.length = length;

    // Daten auslesen
    for (uint8_t i=0;i<length;i++) {
        p_message->data[i] = spi_putc(0xff);
    }

    PORTB |= (1<<P_CS);

    if (bit_is_set(status,3)) {
    	p_message->header.rtr = 1;
    } else {
    	p_message->header.rtr = 0;
    }

    return (/*status & 0x07*/1);
}

uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;

	PORTB &= ~(1<<P_CS);

	spi_putc(type);
	data = spi_putc(0xff);

	PORTB |= (1<<P_CS);

	return data;
}

//-----------------------------------------------------------------------------

uint8_t can_send_message(tCAN *p_message)
{
    uint8_t length = p_message->header.length;

    // ID einstellen
    mcp2515_write_register(TXB0SIDH, (uint8_t) (p_message->id>>3));
    mcp2515_write_register(TXB0SIDL, (uint8_t) (p_message->id<<5));

    // Ist die Nachricht ein "Remote Transmit Request"
    if (p_message->header.rtr)
    {
        /* Eine RTR Nachricht hat zwar eine Laenge,
           aber keine Daten */

        // Nachrichten Laenge + RTR einstellen
        mcp2515_write_register(TXB0DLC, (1<<RTR) | length);
    }
    else
    {
        // Nachrichten Laenge einstellen
        mcp2515_write_register(TXB0DLC, length);

        // Daten
        for (uint8_t i=0;i<length;i++) {
            mcp2515_write_register(TXB0D0 + i, p_message->data[i]);
        }
    }

    // CAN Nachricht verschicken
    PORTB &= ~(1<<P_CS);
    spi_putc(SPI_RTS | 0x01);
    PORTB |= (1<<P_CS);
    return 1;
}

uint8_t mcp2515_get_extmessage(tExtendedCAN *message)
{
	// read status
	uint8_t status = mcp2515_read_status(SPI_RX_STATUS);
	uint8_t addr=SPI_READ_RX;

	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	else {
		// Error: no message available
		return 0;
	}

	PORTB &= ~(1<<P_CS);
	spi_putc(addr);

	// read id
	message->id[0]  = (char) spi_putc(0xff);
	message->id[1]  = (char) spi_putc(0xff);

	message->id[2] = spi_putc(0xff);
	message->id[3] = spi_putc(0xff);

	// read DLC
	uint8_t length = spi_putc(0xff) & 0x0f;

	message->header.length = length;
	message->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;

	// read data
	for (uint8_t i=0;i<length;i++) {
		message->data[i] = spi_putc(0xff);
	}
	 PORTB |= (1<<P_CS);

	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);
	}
	else {
		mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);
	}

	return (status & 0x07) + 1;
}
uint8_t mcp2515_send_extmessage(tExtendedCAN *message)
{
	uint8_t status = mcp2515_read_status(SPI_READ_STATUS);

	// Statusbyte:
	// *
	// * Bit	Function
	// *  2	TXB0CNTRL.TXREQ
	// *  4	TXB1CNTRL.TXREQ
	// *  6	TXB2CNTRL.TXREQ
	// *
	uint8_t address=0x00;
	if (bit_is_clear(status, 2)) {
		address = 0x00;
	}
	else if (bit_is_clear(status, 4)) {
		address = 0x02;
	}
	else if (bit_is_clear(status, 6)) {
		address = 0x04;
	}
	else {
		// all buffer used => could not send message
		return 0;
	}

	PORTB &= ~(1<<P_CS);
	spi_putc(SPI_WRITE_TX | address);

	spi_putc(message->id[0]);
	spi_putc(message->id[1]);

	spi_putc(message->id[2]);
	spi_putc(message->id[3]);

	uint8_t length = message->header.length & 0x0f;

	if (message->header.rtr) {
		// a rtr-frame has a length, but contains no data
		spi_putc((1<<RTR) | length);
	}
	else {
		// set message length
		spi_putc(length);

		// data
		for (uint8_t i=0;i<length;i++) {
			spi_putc(message->data[i]);
		}
	}
	 PORTB |= (1<<P_CS);

	_delay_us(1);

	// send message
	PORTB &= ~(1<<P_CS);
	address = (address == 0) ? 1 : address;
	spi_putc(SPI_RTS | address);
	 PORTB |= (1<<P_CS);

	return address;
}

