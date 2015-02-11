//****************************************************************************
// @Module        Capture / Compare Unit 6 (CAPCOM6)
// @Filename      CC6.H
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the CC6 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (CC6_Header,1)

// USER CODE END



#ifndef _CC6_H_
#define _CC6_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (CC6_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

//   This parameter identifies CC6 timer T12
#define CC6_TIMER_12 T12

#define T12 0x0008
//   This parameter identifies CC6 timer T13
#define CC6_TIMER_13 T13

#define T13 0x0800
//   This parameter identifies CC6 channel 0
#define CC6_CC_0 CC0

//   This parameter identifies CC6 channel 1
#define CC6_CC_1 CC1

//   This parameter identifies CC6 channel 2
#define CC6_CC_2 CC2

//   This parameter identifies CC6 channel 3
#define CC6_CC_3 CC3

//   This parameter identifies CC6 multi channel PWM mode IDLE
#define CC6_IDLE 0x0000

//   This parameter identifies CC6 multi channel PWM mode RIGHT
#define CC6_RIGHT 0x0100

//   This parameter identifies CC6 multi channel PWM mode LEFT
#define CC6_LEFT 0x0200

//   This parameter identifies CC6 multi channel PWM mode DOWN
#define CC6_DOWN 0x0300

// USER CODE BEGIN (CC6_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

// USER CODE BEGIN (CC6_Header,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (CC6_Header,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (CC6_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (CC6_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************

void CC6_Init(void);


// USER CODE BEGIN (CC6_Header,8)

// USER CODE END



//****************************************************************************
// @Macro         CC6_SetCCxReg(RegName, Value) 
//
//----------------------------------------------------------------------------
// @Description   This macro loads the selected capture/compare latch 
//                register with the parameter Value. The transfer from the 
//                latch register to the capture/compare register is enabled 
//                by calling the macro CC6_CommitValue(TimerNr). 
//                The following definitions are available for RegName (only 
//                for enabled CAPCOM6 channels): 
//                CC6_CC_0
//                CC6_CC_1
//                CC6_CC_2
//                CC6_CC_3
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    RegName: 
//                Name of the CAPCOM6 channel to be used
// @Parameters    Value: 
//                16-bit or 10-bit (channel 3 only) value for the capture 
//                register
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC6_SetCCxReg(RegName, Value) CC6_SetCCxReg_##RegName(Value)
#define CC6_SetCCxReg_CC6_CC_0(Value) CC60  = (Value)
#define CC6_SetCCxReg_CC6_CC_1(Value) CC61  = (Value)
#define CC6_SetCCxReg_CC6_CC_2(Value) CC62  = (Value)
#define CC6_SetCCxReg_CC6_CC_3(Value) CMP13 = (Value)


//****************************************************************************
// @Macro         CC6_GetCCxReg(RegName) 
//
//----------------------------------------------------------------------------
// @Description   This macro returns the contents of the selected 
//                capture/compare register. 
//                The following definitions are available for RegName (only 
//                for enabled CAPCOM6 channels): 
//                CC6_CC_0
//                CC6_CC_1
//                CC6_CC_2
//                CC6_CC_3
//
//----------------------------------------------------------------------------
// @Returnvalue   16-bit or 10-bit (channel 3 only) value of the capcom 
//                register
//
//----------------------------------------------------------------------------
// @Parameters    RegName: 
//                Name of the CAPCOM6 channel to be used
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC6_GetCCxReg(RegName) CC6_GetCCxReg_##RegName()
#define CC6_GetCCxReg_CC6_CC_0() CC60
#define CC6_GetCCxReg_CC6_CC_1() CC61
#define CC6_GetCCxReg_CC6_CC_2() CC62
#define CC6_GetCCxReg_CC6_CC_3() CMP13


//****************************************************************************
// @Macro         CC6_CommitValue(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   The timer period, offset and compare values are written to 
//                shadow latches rather than the actual register. The 
//                transfer from the shadow latches to the registers is 
//                enabled by calling this macro. If the transfer is enabled 
//                the shadow latches are copied to the respective register as 
//                soon as the associated timer reaches the value zero the 
//                next time. When timer T12 is operating in center aligned 
//                mode it will also copy the latches if it reaches the 
//                currently programmed period value (counting up).  
//                The following definitions for TimerNr are available: 
//                CC6_TIMER_12
//                CC6_TIMER_13
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                CC6 timer to be used
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC6_CommitValue(TimerNr) CC6_CommitValue_##TimerNr()
#define CC6_CommitValue_CC6_TIMER_12() CTCON_STE12 = 1
#define CC6_CommitValue_CC6_TIMER_13() CTCON_STE13 = 1


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************


// USER CODE BEGIN (CC6_Header,9)

// USER CODE END


#endif  // ifndef _CC6_H_
