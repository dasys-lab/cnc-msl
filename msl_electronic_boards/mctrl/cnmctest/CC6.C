//****************************************************************************
// @Module        Capture / Compare Unit 6 (CAPCOM6)
// @Filename      CC6.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the CC6 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (CC6_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (CC6_General,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (CC6_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (CC6_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (CC6_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (CC6_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (CC6_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (CC6_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (CC6_General,9)

// USER CODE END


//****************************************************************************
// @Function      void CC6_Init(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the CC6 function 
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

void CC6_Init(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 module clock:
  ///  -----------------------------------------------------------------------
  ///  - the CC6 module clock is 20 MHz

  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 multi channels mode:
  ///  -----------------------------------------------------------------------
  ///  - multi channel mode is disabled
  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 timer T12:
  ///  -----------------------------------------------------------------------
  ///  - clock prescaler of timer T12 is 1
  ///  - timer T12 is started after initialization
  ///  - timer T12 works in edge aligned mode (cout up)
  ///  - the interrupt, when T12 matches the period value, is disabled
  ///  - timer T12 is cleared when it is stopped and the compare outputs are 
  ///    set to their defined passive state

  T12P           =  0x03E8;      // load CC6 timer T12 period register


  T12OF          =  0x0000;      // load CC6 timer T12 offset register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 timer T13:
  ///  -----------------------------------------------------------------------
  ///  - clock prescaler of timer T13 is 1
  ///  - timer T13 is not started after initialization
  ///  - no effect on timer T13 when it is stopped
  ///  - T13's output signal is used to modulate the compare outputs COUT6n 
  ///    in burst or multi-channel mode (n=0..2)

  T13P           =  0x007C;      // load CC6 timer T13 period register

  CTCON          =  0x1010;      // load CC6 compare timer control register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 channel 0:
  ///  -----------------------------------------------------------------------
  ///  - compare output on pin COUT60 (P1L.1)
  ///  - generation of interrupt when T12 matches compare register CC60 while 
  ///    counting up is disabled
  ///  - the passive output level on COUT60 is low
  ///  - output COUT60 is not modulated during its active phase
  ///  - requiered duty cycle is 0,00 %
  ///  - real duty cycle is 0,00 %


  CC60           =  0x03E9;      // load CC6 compare register 0


  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 channel 1:
  ///  -----------------------------------------------------------------------
  ///  - compare output on pin COUT61 (P1L.3)
  ///  - generation of interrupt when T12 matches compare register CC61 while 
  ///    counting up is disabled
  ///  - the passive output level on COUT61 is low
  ///  - output COUT61 is not modulated during its active phase
  ///  - requiered duty cycle is 0,00 %
  ///  - real duty cycle is 0,00 %


  CC61           =  0x03E9;      // load CC6 compare register 1


  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 channel 2:
  ///  -----------------------------------------------------------------------
  ///  - compare output on pin COUT62 (P1L.5)
  ///  - generation of interrupt when T12 matches compare register CC62 while 
  ///    counting up is disabled
  ///  - the passive output level on COUT62 is low
  ///  - output COUT62 is not modulated during its active phase
  ///  - requiered duty cycle is 0,00 %
  ///  - real duty cycle is 0,00 %


  CC62           =  0x03E9;      // load CC6 compare register 2


  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 channel 3:
  ///  -----------------------------------------------------------------------
  ///  - timer T13 output signal COUT63 (P1L.6) is disabled
  ///  - the passive output level on COUT63 is low
  ///  - requiered duty cycle is 0,00 %
  ///  - real duty cycle is 0,00 %


  CMP13          =  0x007D;      // load CC6 compare register 3


  CC6MCON        =  0x0000;      // load CC6 mode control register

  CC6MSEL        =  0x0222;      // load CC6 mode select register

  CC6MIC         =  0x0000;      // load CC6 interrupt control register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the CC6 trap function:
  ///  -----------------------------------------------------------------------
  ///  - external trap input CTRAP (P1L.7) is disabled

  TRCON          =  0x0000;      // load CC6 trap enable register


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CC6 Port Pins:
  ///  -----------------------------------------------------------------------
  ///  - P1L.1 is used for CAPCOM6 Compare Output 0 (COUT60)
  ///  - P1L.3 is used for CAPCOM6 Compare Output 1 (COUT61)
  ///  - P1L.5 is used for CAPCOM6 Compare Output 2 (COUT62)

  P1L   = (P1L   & ~(uword)0x002A) | 0x002A;    //set data register
  DP1L  = (DP1L  & ~(uword)0x002A) | 0x002A;    //set direction register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CC6 Interrupts:
  ///  -----------------------------------------------------------------------

  // USER CODE BEGIN (CC6_Function,3)

  // USER CODE END

  CTCON_STE12    =  1;           // timer 12 transfer shadow latch

  CTCON_STE13    =  1;           // timer 13 transfer shadow latch

  CTCON_CT12R    =  1;           // timer 12 run bit is set

} //  End of function CC6_Init




// USER CODE BEGIN (CC6_General,10)

// USER CODE END

