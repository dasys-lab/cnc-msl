//****************************************************************************
// @Module        Analog / Digital Converter (ADC)
// @Filename      ADC.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains functions that use the ADC module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (ADC_General,1)

// USER CODE END



//****************************************************************************
// @Project Includes
//****************************************************************************

#include "MAIN.H"

// USER CODE BEGIN (ADC_General,2)

#include "cn_adc.h"

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (ADC_General,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (ADC_General,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (ADC_General,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (ADC_General,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (ADC_General,7)

// USER CODE END


//****************************************************************************
// @External Prototypes
//****************************************************************************

// USER CODE BEGIN (ADC_General,8)

// USER CODE END


//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

// USER CODE BEGIN (ADC_General,9)

// USER CODE END


//****************************************************************************
// @Function      void ADC_vInit(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the initialization function of the ADC function 
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

void ADC_vInit(void)
{
  // USER CODE BEGIN (Init,2)

  // USER CODE END

  ///  - fixed channel single conversion mode is selected
  ///  - repeatedly converts channel 0
  ///  - ADC start bit is reset
  ///  - 'wait for ADDAT read mode' is disabled
  ///  - converter basic clock tbc is fcpu / 16
  ///  - sample time tsc is tbc * 64


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used ADC Interrupts:
  ///  -----------------------------------------------------------------------
  ///  - Conv service request node configuration:
  ///  - Conv interrupt priority level (ILVL) = 9
  ///  - Conv interrupt group level (GLVL) = 0

  ADCIC          =  0x0064;     


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used ADC Port Pins:
  ///  -----------------------------------------------------------------------
  ///  - P5.0 is used for Analog Input 0 (AN0)
  ///  - P5.1 is used for Analog Input 1 (AN1)
  ///  - P5.2 is used for Analog Input 2 (AN2)
  ///  - P5.3 is used for Analog Input 3 (AN3)


  ///  - digital input stage is disconnected from port line P5.0
  ///  - digital input stage is disconnected from port line P5.1
  ///  - digital input stage is disconnected from port line P5.2
  ///  - digital input stage is disconnected from port line P5.3

  P5DIDIS        =  0x000F;      // load Port 5 digital input disable register


  ADDAT2         =  0x0000;      // load A/D converter 2 result register

  ADCON          =  0xB000;      // load ADC control register


  // USER CODE BEGIN (Init,3)

  // USER CODE END

} //  End of function ADC_vInit


//****************************************************************************
// @Function      void ADC_SetConvMode(ubyte ubMode, ubyte ubChannel) 
//
//----------------------------------------------------------------------------
// @Description   This function configures a new A/D conversion. In fixed 
//                channel single conversion mode (ADC_FIXED) the forwarded 
//                channel is converted once. In fixed channel continuous 
//                conversion mode (ADC_FIXED_CONTI) the forwarded channel is 
//                converted continuously. In auto scan single conversion mode 
//                (ADC_SCAN) a sequence beginning with the forwarded channel 
//                and ending with channel 0 is converted. In auto scan 
//                continuous conversion mode (ADC_SCAN_CONTI) the sequence is 
//                converted continuously. To start the conversion it is 
//                necessary to call ADC_StartConv.
//                Note: 
//                While a conversion is in progress, the mode and the channel 
//                may be changed. The new mode will be evaluated after the 
//                current conversion.The new channel will be evaluated after 
//                the current conversion (fixed channel modes) or after the 
//                current conversion sequence (auto scan modes).  The 
//                following constants are available for ubMode: 
//                ADC_FIXED 
//                ADC_FIXED_CONTI 
//                ADC_SCAN 
//                ADC_SCAN_CONTI 
//                 
//                The following constants are available for ubChannel: 
//                ADC_ANA_0 .. ADC_ANA_7 (see @Defines in header file) 
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    ubMode: 
//                Conversion mode
// @Parameters    ubChannel: 
//                Channel number
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (ConfConv,1)

// USER CODE END

void ADC_SetConvMode(ubyte ubMode, ubyte ubChannel)
{

  ADCON = (ADCON & 0xFF00)| (ubMode << 4) | ubChannel;

} //  End of function ADC_SetConvMode


//****************************************************************************
// @Function      uword ADC_ReadConvResult(void) 
//
//----------------------------------------------------------------------------
// @Description   This function returns the result of a conversion. This 
//                function must be called after each conversion. The lower 10 
//                bits contain the conversion result while the upper 4 bits 
//                identify the converted analog channel. 
//
//----------------------------------------------------------------------------
// @Returnvalue   Conversion result
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

// USER CODE BEGIN (ReadConv,1)

// USER CODE END

uword ADC_ReadConvResult(void)
{

  return(ADDAT);

} //  End of function ADC_ReadConvResult


//****************************************************************************
// @Function      void ADC_ISR(void) 
//
//----------------------------------------------------------------------------
// @Description   This is the interrupt service routine for the ADC. It is 
//                called at the end of each conversion. The user obtains the 
//                conversion result by calling the function 
//                ADC_ReadConvResult.
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

// USER CODE BEGIN (Conv,1)

// USER CODE END

void ADC_ISR(void) interrupt ADCINT
{
  // USER CODE BEGIN (Conv,2)

	cn_adc_isr();

  // USER CODE END

} //  End of function ADC_ISR




// USER CODE BEGIN (ADC_General,10)

// USER CODE END

