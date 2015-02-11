//****************************************************************************
// @Module        Capture / Compare Unit 2 (CAPCOM2)
// @Filename      CC2.H
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the CC2 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (CC2_Header,1)

// USER CODE END



#ifndef _CC2_H_
#define _CC2_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (CC2_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (CC2_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

//   This parameter identifies CAPCOM2 timer 7
#define CC2_TIMER_7 T7

//   This parameter identifies CAPCOM2 timer 8
#define CC2_TIMER_8 T8


// USER CODE BEGIN (CC2_Header,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (CC2_Header,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (CC2_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (CC2_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************

void CC2_Init(void);

//****************************************************************************
// @Macro         CC2_StartTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro starts the selected CAPCOM2 timer. The timer 
//                continues counting from where it stopped. 
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be started
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_StartTimer(TimerNr) CC2_StartTimer_##TimerNr()
#define CC2_StartTimer_CC2_TIMER_7() T78CON_T7R = 1
#define CC2_StartTimer_CC2_TIMER_8() T78CON_T8R = 1


//****************************************************************************
// @Macro         CC2_StopTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro stops the selected CAPCOM2 timer. The content of 
//                the timer register remains unchanged. 
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be stopped
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_StopTimer(TimerNr) CC2_StopTimer_##TimerNr()
#define CC2_StopTimer_CC2_TIMER_7() T78CON_T7R = 0
#define CC2_StopTimer_CC2_TIMER_8() T78CON_T8R = 0


//****************************************************************************
// @Macro         CC2_ClearTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro stops the selected CAPCOM2 timer and sets the 
//                timer register to 0. 
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be cleared
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_ClearTimer(TimerNr) CC2_ClearTimer_##TimerNr()
#define CC2_ClearTimer_CC2_TIMER_7() T78CON_T7R = 0; T7 = 0x0000
#define CC2_ClearTimer_CC2_TIMER_8() T78CON_T8R = 0; T8 = 0x0000


//****************************************************************************
// @Macro         CC2_ReadTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro returns the content of the CAPCOM2 timer. The 
//                timer is not stopped. CPU access to the timer register has 
//                a higher priority than a timer increment or a reload. 
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   Current 16-bit value of the timer register
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be readed
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_ReadTimer(TimerNr) TimerNr


//****************************************************************************
// @Macro         CC2_LoadTimer(TimerNr, Value) 
//
//----------------------------------------------------------------------------
// @Description   This macro loads the timer register of the selected CAPCOM2 
//                timer with the requested value. The timer need not be be 
//                stopped for this happen as a CPU access to the timer 
//                register has a higher priority than a timer increment or a 
//                reload. 
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be loaded
// @Parameters    Value: 
//                16-bit Value with which the CAPCOM2 timer is to be loaded
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_LoadTimer(TimerNr, Value) TimerNr = (Value)


//****************************************************************************
// @Macro         CC2_SetReload(TimerNr, Value) 
//
//----------------------------------------------------------------------------
// @Description   This macro loads the reload register of the selected 
//                CAPCOM2 timer.
//                
//                The following definitions for TimerNr are available: 
//                CC2_TIMER_7
//                CC2_TIMER_8
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the CAPCOM2 timer to be used
// @Parameters    Value: 
//                16-bit Value with which the CAPCOM2 timer reload register 
//                is to be loaded
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_SetReload(TimerNr, Value) CC2_SetReload_##TimerNr(Value)
#define CC2_SetReload_CC2_TIMER_7(Value) T7REL = (Value)
#define CC2_SetReload_CC2_TIMER_8(Value) T8REL = (Value)


//****************************************************************************
// @Macro         CC2_SetCCxReg(RegName, Value) 
//
//----------------------------------------------------------------------------
// @Description   This macro loads the selected capture/compare register with 
//                the parameter Value. 
//                
//                The following definitions are available for RegName (only 
//                for enabled CAPCOM channels): 
//                CC2_CC_16 .. CC2_CC_27 (see @Defines in header file)
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    RegName: 
//                Name of the capture/compare channel to be used
// @Parameters    Value: 
//                16-bit Value with which the capture/compare register is to 
//                be loaded
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_SetCCxReg(RegName, Value) RegName = (Value)


//****************************************************************************
// @Macro         CC2_GetCCxReg(RegName) 
//
//----------------------------------------------------------------------------
// @Description   This macro returns the content of the selected 
//                capture/compare register.
//                The following definitions are available for RegName (only 
//                for enabled CAPCOM2 channels): 
//                CC2_CC_16 .. CC2_CC_27 (see @Defines in header file)
//
//----------------------------------------------------------------------------
// @Returnvalue   Current 16-bit value of the capture/compare register
//
//----------------------------------------------------------------------------
// @Parameters    RegName: 
//                Name of the CAPCOM2 channel to be readed
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define CC2_GetCCxReg(RegName) RegName


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************

#define T8INT    0x3E


// USER CODE BEGIN (CC2_Header,8)

// USER CODE END


#endif  // ifndef _CC2_H_
