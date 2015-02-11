//****************************************************************************
// @Module        Capture / Compare Unit 2 (CAPCOM2)
// @Filename      CC2.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the CC2 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (CC2_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (CC2_General,2)

#include "cn_controller.h"

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (CC2_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (CC2_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (CC2_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (CC2_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (CC2_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (CC2_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (CC2_General,9)

// USER CODE END


//****************************************************************************
// @Function      void CC2_Init(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the CC2 function 
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

void CC2_Init(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  -----------------------------------------------------------------------
  ///  Configuration of CC2 timer 7:
  ///  -----------------------------------------------------------------------
  ///  - timer 7 works in timer mode
  ///  - prescaler factor is 8
  ///  - timer 7 run bit is reset

  ///  -----------------------------------------------------------------------
  ///  Configuration of CC2 timer 8:
  ///  -----------------------------------------------------------------------
  ///  - timer 8 works in timer mode
  ///  - prescaler factor is 1024
  ///  - timer 8 run bit is reset


  T78CON         =  0x0700;      // load CAPCOM2 timer 7 and timer 8 control 
                                 // register

  T7             =  0x0000;      // load CAPCOM2 timer 7 register

  T7REL          =  0x0000;      // load CAPCOM2 timer 7 reload register

  T8             =  0xD9DA;      // load CAPCOM2 timer 8 register

  T8REL          =  0xD9DA;      // load CAPCOM2 timer 8 reload register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CC2 Timer Port Pins:
  ///  -----------------------------------------------------------------------


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 timer Interrupts:
  ///  -----------------------------------------------------------------------
  ///  - Tmr7 service request node configuration:
  //// - Tmr7 interrupt is enabled but NO INTERRUPT WILL BE GENERATED because 
  ////   priority level is 0

  T7IC           =  0x0040;     


  ///  - Tmr8 service request node configuration:
  ///  - Tmr8 interrupt priority level (ILVL) = 13
  ///  - Tmr8 interrupt group level (GLVL) = 3

  T8IC           =  0x0077;     


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 16:
  ///  -----------------------------------------------------------------------
  ///  - channel 16 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 17:
  ///  -----------------------------------------------------------------------
  ///  - channel 17 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 18:
  ///  -----------------------------------------------------------------------
  ///  - channel 18 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 19:
  ///  -----------------------------------------------------------------------
  ///  - channel 19 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 24:
  ///  -----------------------------------------------------------------------
  ///  - channel 24 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 25:
  ///  -----------------------------------------------------------------------
  ///  - channel 25 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 26:
  ///  -----------------------------------------------------------------------
  ///  - channel 26 is disabled

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channel 27:
  ///  -----------------------------------------------------------------------
  ///  - channel 27 is disabled

  CCM4           =  0x0000;      // load CAPCOM2 mode register 4
  CCM5           =  0x0000;      // load CAPCOM2 mode register 5
  CCM6           =  0x0000;      // load CAPCOM2 mode register 6
  CCM7           =  0x0000;      // load CAPCOM2 mode register 7

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CC2 Channel Port Pins:
  ///  -----------------------------------------------------------------------


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used CAPCOM2 channels Interrupts:
  ///  -----------------------------------------------------------------------

  // USER CODE BEGIN (Init,3)

  // USER CODE END

} //  End of function CC2_Init


//****************************************************************************
// @Function      void CC2_Timer8_ISR(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the interrupt service routine for the CAPCOM2 timer 
//                8. It is called when overflow of the timer 8 register 
//                occurs.
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

// USER CODE BEGIN (Tmr8,1)

// USER CODE END

void CC2_Timer8_ISR(void) interrupt T8INT
{
  // USER CODE BEGIN (Tmr8,2)

   cn_controller_emergency_stop();

  // USER CODE END

} //  End of function CC2_Timer8_ISR




// USER CODE BEGIN (CC2_General,10)

// USER CODE END

