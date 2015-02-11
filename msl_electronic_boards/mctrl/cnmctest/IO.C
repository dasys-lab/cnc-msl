//****************************************************************************
// @Module        Parallel Ports
// @Filename      IO.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the IO module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:16
//
//****************************************************************************

// USER CODE BEGIN (IO_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (IO_General,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (IO_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (IO_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (IO_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (IO_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (IO_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (IO_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (IO_General,9)

// USER CODE END


//****************************************************************************
// @Function      void IO_Init(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the IO function 
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

// USER CODE BEGIN (IO_Function,1)

// USER CODE END

void IO_Init(void)
{
  // USER CODE BEGIN (IO_Function,2)

  // USER CODE END

  ///  -----------------------------------------------------------------------
  ///  Bus Interface Pins Edge Characteristic:
  ///  -----------------------------------------------------------------------
  ///  - Fast edge mode, rise/fall times depend on the driver’s dimensioning

  ///  -----------------------------------------------------------------------
  ///  Non-Bus Pins Edge Characteristic:
  ///  -----------------------------------------------------------------------
  ///  - Fast edge mode, rise/fall times depend on the driver’s dimensioning

  PDCR           =  0x0000;      // load port driver control register

  ///  -----------------------------------------------------------------------
  ///  General Port Settings:
  ///  -----------------------------------------------------------------------
  PICON          =  0x0000;      // load input configuration register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P0H:
  ///  -----------------------------------------------------------------------
  ///  P0H.0 is used as alternate output for the Port Pin (AD8)
  ///  P0H.1 is used as alternate output for the Port Pin (AD9)
  ///  P0H.2 is used as alternate output for the Port Pin (AD10)
  ///  P0H.3 is used as alternate output for the Port Pin (AD11)
  ///  P0H.4 is used as alternate output for the Port Pin (AD12)
  ///  P0H.5 is used as alternate output for the Port Pin (AD13)
  ///  P0H.6 is used as alternate output for the Port Pin (AD14)
  ///  P0H.7 is used as alternate output for the Port Pin (AD15)

  P0H            =  0x0000;      // load data register
  DP0H           =  0x0000;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P0L:
  ///  -----------------------------------------------------------------------
  ///  P0L.0 is used as alternate output for the Port Pin (AD0)
  ///  P0L.1 is used as alternate output for the Port Pin (AD1)
  ///  P0L.2 is used as alternate output for the Port Pin (AD2)
  ///  P0L.3 is used as alternate output for the Port Pin (AD3)
  ///  P0L.4 is used as alternate output for the Port Pin (AD4)
  ///  P0L.5 is used as alternate output for the Port Pin (AD5)
  ///  P0L.6 is used as alternate output for the Port Pin (AD6)
  ///  P0L.7 is used as alternate output for the Port Pin (AD7)

  P0L            =  0x0000;      // load data register
  DP0L           =  0x0000;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P1H:
  ///  -----------------------------------------------------------------------
  ///  - no pin of port P1H is used

  P1H            =  0x0000;      // load data register
  DP1H           =  0x0000;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P1L:
  ///  -----------------------------------------------------------------------
  ///  P1L.0 is used as general purpose output
  ///  - the pin status is low level
  ///  P1L.1 is used as alternate output for the CAPCOM6 Compare Output 0 
  ///  (COUT60)
  ///  P1L.2 is used as general purpose output
  ///  - the pin status is low level
  ///  P1L.3 is used as alternate output for the CAPCOM6 Compare Output 1 
  ///  (COUT61)
  ///  P1L.4 is used as general purpose output
  ///  - the pin status is low level
  ///  P1L.5 is used as alternate output for the CAPCOM6 Compare Output 2 
  ///  (COUT62)
  ///  P1L.6 is used as general purpose output
  ///  - the pin status is low level
  ///  P1L.7 is used as general purpose output
  ///  - the pin status is low level

  P1L            =  0x0000;      // load data register
  DP1L           =  0x00D5;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P3:
  ///  -----------------------------------------------------------------------
  ///  P3.4 is used as alternate input for the Timer 3 ext. Up/Down Input 
  ///  (T3EUD)
  ///  P3.6 is used as alternate input for the Timer 3 Count Input (T3IN)
  ///  P3.9 is used as general purpose output
  ///  - push/pull output is selected
  ///  - the pin status is low level
  ///  P3.10 is used as alternate output for the ASC0 Transmit Data Output 
  ///  (TxD0)
  ///  - push/pull output is selected
  ///  P3.11 is used as alternate input for the ASC0 Receive data Input (RxD0)
  ///  P3.12 is used as alternate output for the Byte High Enable Output 
  ///  (BHE_n)

  ///  P3.0 - P3.7 threshold type: TTL input
  ///  P3.8 - P3.15 threshold type: TTL input

  ODP3           =  0x0000;      // load open-drain register
  P3             =  0x0000;      // load data register
  DP3            =  0x0200;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P4:
  ///  -----------------------------------------------------------------------
  ///  P4.0 is used as alternate input for the Port Pin (A16)
  ///  P4.1 is used as alternate input for the Port Pin (A17)
  ///  P4.5 is used as alternate input for the CAN1 Interface Input (CAN1_RxD)
  ///  P4.6 is used as alternate output for the CAN1 Interface Output 
  ///  (CAN1_TxD)
  ///  - push/pull output is selected

  ///  P4.0 - P4.7 threshold type: TTL input

  ODP4           =  0x0000;      // load open-drain register
  P4             =  0x0000;      // load data register
  DP4            =  0x0000;      // load direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P5:
  ///  -----------------------------------------------------------------------
  ///  P5.0 is used as alternate input for the Analog Input 0 (AN0)
  ///  P5.1 is used as alternate input for the Analog Input 1 (AN1)
  ///  P5.2 is used as alternate input for the Analog Input 2 (AN2)
  ///  P5.3 is used as alternate input for the Analog Input 3 (AN3)
  ///  P5.4 is used as alternate input for the Timer 2 ext. Up/Down Input 
  ///  (T2EUD)
  ///  P5.5 is used as alternate input for the Timer 4 ext. Up/Down Input 
  ///  (T4EUD)
  ///  P5.6 is used as alternate input for the Timer 2 Count Input (T2IN)
  ///  P5.7 is used as alternate input for the Timer 4 Count Input (T4IN)

  P5             =  0x0000;      // load data register

  ///  -----------------------------------------------------------------------
  ///  Configuration of Port P8:
  ///  -----------------------------------------------------------------------
  ///  - no pin of port P8 is used

  ODP8           =  0x0000;      // load open-drain register
  P8             =  0x0000;      // load data register
  DP8            =  0x0000;      // load direction register


  // USER CODE BEGIN (IO_Function,3)

  // USER CODE END

} //  End of function IO_Init




// USER CODE BEGIN (IO_General,10)

// USER CODE END

