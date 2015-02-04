//****************************************************************************
// @Module        Project Settings
// @Filename      MAIN.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains the project initialization function.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:16
//
//****************************************************************************

// USER CODE BEGIN (MAIN_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (MAIN_General,2)

#include "cn_main.h"

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (MAIN_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (MAIN_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (MAIN_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (MAIN_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (MAIN_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (MAIN_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (MAIN_General,9)

// USER CODE END


//****************************************************************************
// @Function      void MAIN_vInit(void) 
//
//----------------------------------------------------------------------------
// @Description   This function initializes the microcontroller.
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

void MAIN_vInit(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  //// -----------------------------------------------------------------------
  //// Begin of Important Settings for the Start-Up File
  //// -----------------------------------------------------------------------
  ///  All following settings must be set in the start-up file. You can use 
  ///  DAvE's project file (*.dpt) to include this register values into your 
  ///  compiler EDE.

    ///  ---------------------------------------------------------------------
    ///  Initialization of the SYSCON Register:
    ///  ---------------------------------------------------------------------
    ///  - 256 words system stack
    ///  - Internal ROM area mapped to segment 1
    ///  - the segmentation is enabled (CSP is saved/restored during 
    ///  interrupt entry/exit)
    ///  - Internal ROM disabled
    ///  - the pin #BHE is enabled
    ///  - the pins #WR and #BHE retain their normal functions
    ///  - system clock output CLKOUT is disabled
    ///  - latched #CS mode
    ///  - pin #RSTIN is pulled low during the internal reset sequence
    ///  - the on-chip X-Peripherals are enabled and can be accessed
    ///  - accesses to the XBUS peripherals are done internally

    //// this register must be set in the start-up file
    //// SYSCON  =  0x101C

    ///  ---------------------------------------------------------------------
    ///  Initialization of the SYSCON1 Register:
    ///  ---------------------------------------------------------------------

    //// this register must be set in the start-up file
    //// SYSCON1  =  0x0000

    ///  ---------------------------------------------------------------------
    ///  Initialization of the SYSCON2 Register:
    ///  ---------------------------------------------------------------------

    //// this register must be set in the start-up file
    //// SYSCON2  =  0x0000

    ///  ---------------------------------------------------------------------
    ///  Initialization of the SYSCON3 Register:
    ///  ---------------------------------------------------------------------

    //// this register must be set in the start-up file
    //// SYSCON3  =  0x0000

    ///  ---------------------------------------------------------------------
    ///  --- initialization of the BUSCON 0-4 and ADRRSEL Registers 1-4 ---
    ///  ---------------------------------------------------------------------


    ///  ---------------------------------------------------------------------
    ///  ---------- external bus 0 is enabled ----------
    ///  ---------------------------------------------------------------------
    ///  - 16-bit Multiplexed Bus
    ///  - memory cycle time control: 15 waitstates
    ///  - With read/write delay: activate command 1 TCL after falling edge 
    ///  of ALE
    ///  - chip select mode: address chip select
    ///  - memory tristate control: 1 waitstate
    ///  - ALE lengthening control: lengthened ALE signal
    ///  - Address windows are switched immediately
    ///  - Normal #WR signal

    //// this register must be set in the start-up file
    //// BUSCON0  =  0x06C0




  //// -----------------------------------------------------------------------
  //// End of Important Settings for the Start-Up File
  //// -----------------------------------------------------------------------




  //   -----------------------------------------------------------------------
  //   Initialization of the Peripherals:
  //   -----------------------------------------------------------------------

  //   initializes the Parallel Ports
  IO_Init();

  //   initializes the Asynchronous/Synchronous Serial Interface (ASC0)
  ASC0_Init();

  //   initializes the General Purpose Timer Unit (GPT1)
  GPT1_vInit();

  //   initializes the Watchdog Timer (WDT)
  WDT_Init();

  //   initializes the Real Timer Clock (RTC)
  RTC_Init();

  //   initializes the Analog / Digital Converter (ADC)
  ADC_vInit();

  //   initializes the Capture / Compare Unit 2 (CAPCOM2)
  CC2_Init();

  //   initializes the Capture / Compare Unit 6 (CAPCOM6)
  CC6_Init();

  //   initializes the On-Chip CAN Interface 1 (CAN1)
  CAN1_Init();

  // USER CODE BEGIN (Init,3)

  // USER CODE END

  //   globally enable interrupts
  PSW_IEN        =  1;          

} //  End of function MAIN_vInit


//****************************************************************************
// @Function      void main(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the main function.
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

// USER CODE BEGIN (Main,1)

// USER CODE END

void main(void)
{
  // USER CODE BEGIN (Main,2)

  // USER CODE END

  MAIN_vInit();

  // USER CODE BEGIN (Main,4)

  cn_main();

  // USER CODE END

} //  End of function main



// USER CODE BEGIN (MAIN_General,10)

// USER CODE END

