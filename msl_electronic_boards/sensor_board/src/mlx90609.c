/*
File:
    mlx90609.c

Description:
    Routines to access the Melexis MLX90609 Angular-rate sensor

External References:
    Requires a function called "SPI_sendrec()" that takes a uint8_t
    and returns a uint8_t. This function sends and receives a single
    byte over the SPI interface. The SPI interface should be set up
    before calling any functions here!

    Requires two functions/macros called "SS_low()" and "SS_high()"
    that set the SS line to the MLX device in the low and high state

    Requires a "delay_uS(x)" function or macro that will take a value
    between 0 to 255. The function can take LONGER then specified to
    complete, but must wait a least 'x' uS

Author:
    Colin O'Flynn 

    For technical support with the Melexis devices, please use the
        web-forum at www.melexis.com

Copyright notice:

    Copyright Colin O'Flynn, 2006. This code can be freely used for any
    applications with a Melexis rate sensor.

    This copyright notice may not be removed when the source is redistributed.
    You may however extend this code, and add your name to the copyright notice.

    THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE
    LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR
    OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE 
    ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.
    SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY 
    SERVICING, REPAIR OR CORRECTION.
    
    IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL
    ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR REDISTRIBUTE
    THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY
    GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE
    OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR
    DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR
    A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER PROGRAMS), EVEN IF SUCH
    HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. 


Changelog:
    July 20, 2006   CO      Fixed errors
    July 9, 2006    CO      Initial file creation
*/


#include <stdint.h>
#include "mlx90609.h"
#include "spi.h"


/************* Compiler / Hardware specific options **********/
#include <util/delay.h>

#include <avr/io.h>

//provides the delay
#define delay_uS(x) _delay_us(x)


/**************** mlx90609_error Variable *********************************/
/*This variable holds the value 0 if there are no errors. It is 
  updated after every function that access the rate gyro,
  and must be reset by your program.  Once the mlx90609_error variable is
  non-zero, it will remain non-zero after all following function
  calls even if errors do not occur in them.

  This variable is NOT used by the EEPROM functions */
char mlx90609_error = 0x00;

/*
Routine Name:
    mlx90609_status

Routine Description:
    Reads status byte from MLX device

Arguments:
    none

Return Value:
    uint16_t which is the response from MLX device

*/

uint16_t mlx90609_status(void)
{
    uint16_t response;

    NSS_Low();

    //Send ADC Channel, init conversion
    SPI_EXCH(ML_ADCC | 1<<ML_ADEN);

    response = SPI_EXCH(0x00) << 8;
    response |= SPI_EXCH(0x00);

    NSS_High();

    //Check for refusal answer
    if ((response & (1<<15)) != 0)
    {
    	mlx90609_error |= 1<<7 | (uint8_t)response;
    }

    return response;    
}


/*
Routine Name:
    mlx90609_init

Routine Description:
    Inits the MLX device by setting the ADC online

Arguments:
    none

Return Value:
    none

*/

void mlx90609_init(void)
{
    uint16_t response;

    NSS_Low();

    //Send ADC Channel, init conversion
    SPI_EXCH(ML_ADCC | 1<<ML_ADEN);

    response = SPI_EXCH(0x00);
    response = response << 8;
    response |= SPI_EXCH(0x00);

    NSS_High();

    //Check for refusal answer
    if ((response & (1<<15)) != 0)
    {
        mlx90609_error |= 1<<7 | (uint8_t)response;
    }

    //Check for an mlx90609_error
    else if (response & (1<<ML_ERR))
    {
        mlx90609_error |= 1<<7;
    }

    //Just so we don't try right away to read ADC
    delay_uS(200);

    return;    
}


/*
Routine Name:
    mlx90609_selftest

Routine Description:
    Sets up the selftest mode

Arguments:
    unsigned char test: '0' sets mode to normal
                        '1' sets mode to positive rate excitation
                        '2' sets mode to negative rate excitation
                        '3' sets mode to EEPROM mode
Return Value:
    none
*/

void mlx90609_mode(unsigned char test)
{
    uint16_t response;

    NSS_Low();

    //Send MODEW
    SPI_EXCH(ML_MODEW | (test << ML_SFT0));

    response = SPI_EXCH(0x00) << 8;
    response |= SPI_EXCH(0x00);

    NSS_High();

    //Check for refusal answer
    if ((response & (1<<15)) != 0)
    {
        mlx90609_error |= 1<<7 | (uint8_t)response;
    }

    //Check for an mlx90609_error
    else if (response & (1<<ML_ERR))
    {
        mlx90609_error |= 1<<7;
    }
    
    //Check the requested test mode meets what we wanted
    else if ((((uint8_t)response >> ML_SF0) & 0x03) != test)
    {
        mlx90609_error |= 1<<6;
    }

    return;    
}

/*
Routine Name:
    mlx90609_adcstart

Routine Description:
    Starts a conversion on the ADC, reading either the temp or gyro

Arguments:
    unsigned char channel: specifies to use either the gyro or temperature
                           sensor as the analog source. Use the predefined
                           values of CH_GYRO or CH_TEMP to specificy.

Return Value:
    none
*/

void mlx90609_adcstart(unsigned char channel)
{
    uint16_t response;

    NSS_Low();

    //Send ADC Channel, init conversion
    SPI_EXCH(ML_ADCC | 1<<ML_ADEN | channel);

    response = SPI_EXCH(0x00) << 8;
    response |= SPI_EXCH(0x00);

    NSS_High();

    //Check for refusal answer
    if ((response & (1<<15)) != 0)
    {
        mlx90609_error |= 1<<7 | (uint8_t)response;
    }

    //Check for an mlx90609_error
    else if (response & (1<<ML_ERR))
    {
        mlx90609_error |= 1<<7;
    }
    
    return;
}


/*
Routine Name:
    mlx90609_adcread

Routine Description:
    Waits for the conversion to finish if it isn't done, and reads the value

Arguments:
    none

Return Value:
    unsigned int that is an 11-bit value corresponding to the ADC reading.
    Note there is no sign here, it will be biased to a zero-rate value of
    approximately 1024 for the angular rate
*/


unsigned int mlx90609_adcread(void)
{
    unsigned int adc;

    /*Until the EOC bit is set, we keep reading the ADC */
    do
    {    
        NSS_Low();

        //Read ADC
        SPI_EXCH(ML_ADCR);
    
        adc = SPI_EXCH(0x00) << 8;
        adc |= SPI_EXCH(0x00);
    
        NSS_High();

        //Check for refusal answer
        if ((adc & (1<<15)) != 0)
        {
            mlx90609_error |= 1<<7 | (uint8_t)adc;
            return 0;
        }
    
        //Check for an mlx90609_error
        else if (adc & (1<<ML_ERR))
        {
            mlx90609_error |= 1<<7;
            return 0;
        }

    }while ((adc & (1<<ML_EOC)) == 0);

    uart1_puts("nach lese\n");


    /* Mask off unused bits, shift left 1 */
    adc = 0x7FF & (adc>>1);

    return adc;
}

/*
Routine Name:
    mlx90609_erase

Routine Description:
    Erases the EEPROM bank

Arguments:
    unsigned char bank, either '0' or '1'

Return Value:
    0 if OK, non-zero if errors occured
*/

char mlx90609_erase(unsigned char bank)
{
    uint16_t response;
    uint8_t smallresp;

    if (bank > 1)
        return -1;


    /***** Set Bank ******/

    NSS_Low();
    
    SPI_EXCH(ML_EER | bank << 3);
    
    smallresp = SPI_EXCH(0x00);

    NSS_High();
    
    if ((smallresp & (1<<7)) != 0x00)    
        return -2;



    /***** Erase Instruction ******/        
    NSS_Low();

    SPI_EXCH(ML_EEE);

    //Check for now refusal answer
    smallresp = SPI_EXCH(0x00);

    NSS_High();
    
    if ((smallresp & (1<<7)) != 0x00)    
        return -2;


    /*Until the EEB bit is clear, we wait */
    do
    {    
        NSS_Low();

        //Read status
        SPI_EXCH(ML_STATR);
    
        response = SPI_EXCH(0x00) << 8;
        response |= SPI_EXCH(0x00);
    
        NSS_High();

        //Check for refusal answer
        if ((response & (1<<15)) != 0)
        {
            return -2;
        }

    }while ((response & (1<<ML_EEB)) != 0x00);


    return 0;
}


/*
Routine Name:
    mlx90609_write

Routine Description:
    Writes 8 bytes to the EEPROM bank

Arguments:
    unsigned char bank  :either '0' or '1'
    uint8_t * data     :pointer to 8 bytes

Return Value:
    0 if OK, non-zero if errors occured
*/
char mlx90609_write(unsigned char bank, uint8_t * data)
{
    uint16_t response;
    uint8_t smallresp;
    unsigned char count;


    if (bank > 1)
        return -1;

    /***** Set Bank ******/
    NSS_Low();
    
    SPI_EXCH(ML_EER | bank << 3);

    smallresp = SPI_EXCH(0x00);

    NSS_High();
    
    if ((smallresp & (1<<7)) != 0x00)    
        return -2;



    /***** Load Data ******/        
    count = 0;

    while(count < 4)
    {
        NSS_Low();
    
        SPI_EXCH(ML_EED | count);
    
        //Get response and, send data bytes
        response |= SPI_EXCH((uint8_t) *(data + count*2 + 1));
        response = SPI_EXCH((uint8_t) *(data + count*2)) << 8;
        
        NSS_High();

        //Check for refusal answer
        if ((response & (1<<15)) != 0)
        {
            return -2;
        }

        count++;
    }

    /* Start the write! */
    NSS_Low();

    SPI_EXCH(ML_EEW);

    smallresp = SPI_EXCH(0x00);

    NSS_High();
    
    if ((smallresp & (1<<7)) != 0x00)    
        return -2;

    /*Until the EEB bit is clear, we wait */
    do
    {    
        NSS_Low();

        //Read status
        SPI_EXCH(ML_STATR);
    
        response = SPI_EXCH(0x00) << 8;
        response |= SPI_EXCH(0x00);
    
        NSS_High();

        //Check for refusal answer
        if ((response & (1<<15)) != 0)
        {
            return -2;
        }

    }while ((response & (1<<ML_EEB)) != 0x00);

    return 0;
}

/*
Routine Name:
    mlx90609_read

Routine Description:
    Reads one byte

Arguments:
    unsigned char bank : either '0' or '1'
    uint8_t * data     : pointer to where to store byte

Return Value:
    0 if OK, non-zero if errors occured
*/
char mlx90609_read(unsigned char address, uint8_t * data)
{
    uint8_t smallresp;

    NSS_Low();
    
    SPI_EXCH(ML_EER | address);
    
    smallresp = SPI_EXCH(0x00);
 
    delay_uS(60);

    *data = SPI_EXCH(0x00);

    NSS_High();

    if ((smallresp & (1<<7)) != 0x00)   
        return -2;

    return 0;

}

