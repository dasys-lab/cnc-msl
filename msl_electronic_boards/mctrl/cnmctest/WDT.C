//****************************************************************************
// @Module        Watchdog Timer (WDT)
// @Filename      WDT.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the WDT module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (WDT_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (WDT_General,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (WDT_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (WDT_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (WDT_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (WDT_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (WDT_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (WDT_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (WDT_General,9)

// USER CODE END


//****************************************************************************
// @Function      void WDT_Init(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the WDT function 
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

void WDT_Init(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  - watchdog timer input frequency is Fcpu / 128
  ///  - watchdog timer reload value is 0x7F
  ///  - watchdog timer overflow periode is 211,354 ms

  WDTCON         =  0x7F01;      // load WDT control register


  // USER CODE BEGIN (Init,3)

  // USER CODE END

} //  End of function WDT_Init




// USER CODE BEGIN (WDT_General,10)

// USER CODE END

