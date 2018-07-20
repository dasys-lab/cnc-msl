//****************************************************************************
// @Module        General Purpose Timer Unit (GPT1)
// @Filename      GPT1.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the GPT1 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (GPT1_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (GPT1_General,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (GPT1_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (GPT1_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (GPT1_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (GPT1_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (GPT1_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (GPT1_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (GPT1_General,9)

// USER CODE END


//****************************************************************************
// @Function      void GPT1_vInit(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the GPT1 function 
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

void GPT1_vInit(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  -----------------------------------------------------------------------
  ///  Configuration of the GPT1 Core Timer 3:
  ///  -----------------------------------------------------------------------
  ///  - timer 3 works in incremental interface mode
  ///  - maximum input frequency for timer 3 is 1,25 MHz
  ///  - counting by any transition on T3IN (P3.6)
  ///  - up/down control bit is reset

  T3CON          =  0x0131;      // load timer 3 control register
  T3             =  0x7FFF;      // load timer 3 register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the GPT1 Core Timer 2:
  ///  -----------------------------------------------------------------------
  ///  - timer 2 works in incremental interface mode
  ///  - maximum input frequency for timer 2 is 1,25 MHz
  ///  - counting by any transition on T2IN (P5.6)
  ///  - up/down control bit is reset

  T2CON          =  0x0131;      // load timer 2 control register
  T2             =  0x7FFF;      // load timer 2 register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the GPT1 Core Timer 4:
  ///  -----------------------------------------------------------------------
  ///  - timer 4 works in incremental interface mode
  ///  - maximum input frequency for timer 4 is 1,25 MHz
  ///  - counting by any transition on T4IN (P5.7)
  ///  - up/down control bit is reset

  T4CON          =  0x0131;      // load timer 4 control register
  T4             =  0x7FFF;      // load timer 4 register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used GPT1 Port Pins:
  ///  -----------------------------------------------------------------------
  ///  - P5.7 is used for Timer 4 Count Input (T4IN)
  ///  - P5.4 is used for Timer 2 ext. Up/Down Input (T2EUD)
  ///  - P3.4 is used for Timer 3 ext. Up/Down Input (T3EUD)
  ///  - P3.6 is used for Timer 3 Count Input (T3IN)
  ///  - P5.6 is used for Timer 2 Count Input (T2IN)
  ///  - P5.5 is used for Timer 4 ext. Up/Down Input (T4EUD)


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used GPT1 Interrupts:
  ///  -----------------------------------------------------------------------

  // USER CODE BEGIN (GPT1_Function,3)

  // USER CODE END

  T2CON_T2R      =  1;           // timer 2 run bit is set

  T4CON_T4R      =  1;           // timer 4 run bit is set

  T3CON_T3R      =  1;           // timer 3 run bit is set

} //  End of function GPT1_vInit




// USER CODE BEGIN (GPT1_General,10)

// USER CODE END

