//****************************************************************************
// @Module        Asynchronous/Synchronous Serial Interface (ASC0)
// @Filename      ASC0.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the ASC0 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (ASC0_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (ASC0_General,2)

#include "cn_serial.h"

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (ASC0_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (ASC0_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (ASC0_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (ASC0_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (ASC0_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (ASC0_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (ASC0_General,9)

// USER CODE END


//****************************************************************************
// @Function      void ASC0_Init(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the ASC0 function 
//                library. It is assumed that the SFRs used by this library 
//                are in its reset state. 
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (Init,1)

// USER CODE END

void ASC0_Init(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  -----------------------------------------------------------------------
  ///  Configuration of the ASC0 Baud Rate Generator:
  ///  -----------------------------------------------------------------------
  ///  - the ASC0 module clock is 20 MHz
  ///  - required baud rate = 57,600 kbaud
  ///  - real baud rate     = 59,524 kbaud
  ///  - deviation          = 3,340 %

  S0BG           =  0x0006;      // load ASC0 baud rate time reload register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the ASC0 Operation Mode:
  ///  -----------------------------------------------------------------------
  ///  - 8-bit data asychronous operation width one stop bit
  ///  - receiver is enabled

  S0CON          =  0x2011;      // load ASC0 control register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used ASC0 Port Pins:
  ///  -----------------------------------------------------------------------
  ///  - P3.10 is used for ASC0 Transmit Data Output (TxD0)
  ///  - P3.11 is used for ASC0 Receive data Input (RxD0)

  P3   = (P3   & ~(uword)0x0400) | 0x0400;    //set data register
  DP3  = (DP3  & ~(uword)0x0400) | 0x0400;    //set direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used ASC0 Interrupts:
  ///  -----------------------------------------------------------------------
  ///  - transmit buffer service request node configuration:
  ///  - transmit buffer interrupt priority level (ILVL) = 13
  ///  - transmit buffer interrupt group level (GLVL) = 2

  S0TBIC         =  0x0076;     

  ///  - receive service request node configuration:
  ///  - receive interrupt priority level (ILVL) = 12
  ///  - receive interrupt group level (GLVL) = 2

  S0RIC          =  0x0072;     


  //   -----------------------------------------------------------------------
  //   Default Settings for Service Request Flags:
  //   -----------------------------------------------------------------------
  S0TIC_S0TIR    =  1;           // indicates that the transmit register is 
                                 // empty

  // USER CODE BEGIN (ASC0_Function,3)

  // USER CODE END

  S0CON         |=  0x8000;      // enable baud rate generator


} //  End of function ASC0_Init


//****************************************************************************
// @Function      void ASC0_TransmitData(uword uwData) 
//
//----------------------------------------------------------------------------
// @Description   This function writes a send data initialization word into 
//                the S0TBUF register.
//                
//                Note: 
//                In a multiprocessor system the master with this function 
//                has the possibility to send data to the selected slave. To 
//                achieve this, the 9th bit must set on zero.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    uwData: 
//                Data to be send
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (SendData,1)

// USER CODE END

void ASC0_TransmitData(uword uwData)
{
  S0TBIC_S0TBIR = 0;         //  reset transmit buffer interrupt request flag
  S0TBUF  = uwData;   //  load transmit buffer register

} //  End of function ASC0_TransmitData


//****************************************************************************
// @Function      uword ASC0_GetData(void) 
//
//----------------------------------------------------------------------------
// @Description   This function reads out the content of the S0RBUF register 
//                which contains a received data byte.
//
//----------------------------------------------------------------------------
// @Returnvalue   Data that has been received
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (GetData,1)

// USER CODE END

uword ASC0_GetData(void)
{
  return(S0RBUF);     // return receive buffer register

} //  End of function ASC0_GetData


//****************************************************************************
// @Function      void ASC0_ISR_Rx(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the receive interrupt service routine for the ASC0. 
//                It is called if a byte has been received via ASC0 (S0RIR is 
//                set). 
//                Please note that you have to add application specific code 
//                to this function.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (Rx,1)

// USER CODE END

void ASC0_ISR_Rx(void) interrupt S0RINT
{

  // USER CODE BEGIN (Rx,2)

  cn_serial_rx_interrupt();

  // USER CODE END

} //  End of function ASC0_ISR_Rx


//****************************************************************************
// @Function      void ASC0_viTxBuf(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the transmit buffer interrupt service routine for 
//                the ASC0. It is called if the content of the TX-buffer has 
//                been loaded into the TX-shift register.
//                Please note that you have to add application specific code 
//                to this function.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (TxBuf,1)

// USER CODE END

void ASC0_viTxBuf(void) interrupt S0TBINT
{

  // USER CODE BEGIN (TxBuf,2)
  cn_serial_tx_interrupt();
  // USER CODE END

} //  End of function ASC0_viTxBuf


//****************************************************************************
// @Function      ubyte ASC0_IsTransmitDone(void) 
//
//----------------------------------------------------------------------------
// @Description   This function can be used for checking up the status of the 
//                ASC0 transmitter interrupt flags (S0TIR). This shows when 
//                the sending of a byte has terminated. By continuously 
//                polling the S0TIR bit after the function ASC0_TransmitData 
//                has been called, it is possible to establish when the ASC0 
//                has terminated its task.
//
//----------------------------------------------------------------------------
// @Returnvalue   0 if transmitter is busy, else 1
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (TxDataReady,1)

// USER CODE END

ubyte ASC0_IsTransmitDone(void)
{
  ubyte ubReturnValue;

  ubReturnValue = 0;

  if(S0TIC_S0TIR)         // if sending of data is terminated
  {
    ubReturnValue = 1;
    S0TIC_S0TIR = 0;
  }
  return(ubReturnValue);         // return receive buffer register

} //  End of function ASC0_IsTransmitDone




// USER CODE BEGIN (ASC0_General,10)

// USER CODE END

