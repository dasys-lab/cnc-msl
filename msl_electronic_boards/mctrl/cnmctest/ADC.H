//****************************************************************************
// @Module        Analog / Digital Converter (ADC)
// @Filename      ADC.H
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the ADC module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (ADC_Header,1)

// USER CODE END



#ifndef _ADC_H_
#define _ADC_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (ADC_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (ADC_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

//   This parameter identifies ADC fixed channel single conversion mode
#define ADC_FIXED 0

//   This parameter identifies ADC fixed channel continuous conversion mode
#define ADC_FIXED_CONTI 1

//   This parameter identifies ADC auto scan single conversion mode
#define ADC_SCAN 2

//   This parameter identifies ADC auto scan continuous conversion mode
#define ADC_SCAN_CONTI 3

//   This parameter identifies ADC analog channel 0
#define ADC_ANA_0 0

//   This parameter identifies ADC analog channel 1
#define ADC_ANA_1 1

//   This parameter identifies ADC analog channel 2
#define ADC_ANA_2 2

//   This parameter identifies ADC analog channel 3
#define ADC_ANA_3 3

//   This parameter identifies ADC analog channel 4
#define ADC_ANA_4 4

//   This parameter identifies ADC analog channel 5
#define ADC_ANA_5 5

//   This parameter identifies ADC analog channel 6
#define ADC_ANA_6 6

//   This parameter identifies ADC analog channel 7
#define ADC_ANA_7 7


// USER CODE BEGIN (ADC_Header,4)

#define ADC_ANA_8 8
#define ADC_ANA_9 9

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (ADC_Header,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (ADC_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (ADC_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************

void ADC_vInit(void);
void ADC_SetConvMode(ubyte ubMode, ubyte ubChannel);
uword ADC_ReadConvResult(void);

//****************************************************************************
// @Macro         ADC_StartConv() 
//
//----------------------------------------------------------------------------
// @Description   This macro starts the A/D conversion.
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

#define ADC_StartConv() ADCON_ADST = 1


//****************************************************************************
// @Macro         ADC_StopConv() 
//
//----------------------------------------------------------------------------
// @Description   This macro stops a continuous A/D conversion. The last 
//                conversion or sequence is terminated correctly.
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

#define ADC_StopConv() ADCON_ADST = 0


//****************************************************************************
// @Macro         ADC_IsBusy() 
//
//----------------------------------------------------------------------------
// @Description   This macro checks if an A/D conversion is active or not.  
//                If a task has to wait for the result of an A/D conversion 
//                (result is read in the ISR), it can find with this macro, 
//                when the ADC is ready. 
//
//----------------------------------------------------------------------------
// @Returnvalue   Returns a 1 if AD converter is active, else 0
//
//----------------------------------------------------------------------------
// @Parameters    None
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define ADC_IsBusy() ADCON_ADBSY


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************

#define ADCINT    0x28


// USER CODE BEGIN (ADC_Header,8)

// USER CODE END


#endif  // ifndef _ADC_H_
