/*
File:
    mlx90609.h

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
    complete, but must wait at least 'x' uS

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
    July 23, 2006   CO      Moved bit def's to this file
    July 9, 2006    CO      Initial file creation
*/

/************** Commands **************/

/* Gyro Commands */

#define ML_STATR   0x88
#define ML_MODEW   0xA0
#define ML_ADCC    0x90
#define ML_ADCR    0x80

/* EEPROM Commands */
#define ML_EED     0x84
#define ML_EEW     0x83
#define ML_EEC     0x81
#define ML_EEE     0x82
//Nothing for reading... it's all a command except bit 7
#define ML_EER     0x00

/********** Bit Definitions **********/

#define ML_EOC     13
#define ML_MLB     12
#define ML_EEB     11

#define ML_ERR     14
#define ML_BN      10
#define ML_EERP    7
#define ML_EERS    6
#define ML_SF1     5
#define ML_SF0     4
#define ML_CHAN    3
#define ML_ADEN    2
#define ML_SFT0    0
#define ML_SFT1    1

/* Refusal Answer */

#define ML_OPC     14
#define ML_BUSY    10
#define ML_EEDIS   9
#define ML_ECRC    6
#define ML_EPLL    5
#define ML_EDRV    4


/**************** Options to pass to functions ********************/

/* Self-test modes */
#define ST_NONE     0
#define ST_POS      1
#define ST_NEG      2
#define ST_EEPROM   3

/* ADC Channels */
#define CH_GYRO     (0)
#define CH_TEMP     (1<<ML_CHAN)

/**************** Error Variable *********************************/
/*This variable holds the value 0 if there are no errors. It is 
  updated after every function that access the rate gyro,
  and must be reset by your program.  Once the error variable is
  non-zero, it will remain non-zero after all following function
  calls even if errors do not occur in them.

  This variable is NOT used by the EEPROM functions */
extern char mlx90609_error;


/**************** Rate Gyro Functions ***************************/
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
uint16_t mlx90609_status(void);

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
void mlx90609_init(void);

/*
Routine Name:
    mlx90609_mode

Routine Description:
    Sets up the selftest mode

Arguments:
    unsigned char test: '0' sets mode to normal
                        '1' sets mode to positive rate excitation
                        '2' sets mode to negative rate excitation
                        '3' sets mode to EEPROM
Return Value:
    none
*/
void mlx90609_mode(unsigned char test);

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
void mlx90609_adcstart(unsigned char channel);


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
unsigned int mlx90609_adcread(void);


/********************* EEPROM Functions *********************************/

/*NOTE: Make sure that you set the high-voltage to be used by EEPROM for these */


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

char mlx90609_erase(unsigned char bank);

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
char mlx90609_write(unsigned char bank, uint8_t * data);

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
char mlx90609_read(unsigned char address, uint8_t * data);