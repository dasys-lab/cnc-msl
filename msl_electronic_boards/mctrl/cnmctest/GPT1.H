//****************************************************************************
// @Module        General Purpose Timer Unit (GPT1)
// @Filename      GPT1.H
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the GPT1 module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:17
//
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,1)

// USER CODE END



#ifndef _GPT1_H_
#define _GPT1_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

//   This parameter identifies GPT1 timer 2
#define GPT1_TIMER_2 T2

//   This parameter identifies GPT1 timer 3
#define GPT1_TIMER_3 T3

//   This parameter identifies GPT1 timer 4
#define GPT1_TIMER_4 T4

// USER CODE BEGIN (GPT1_Header,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (GPT1_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************


void GPT1_vInit(void);


// USER CODE BEGIN (GPT1_Header,8)

// USER CODE END

//****************************************************************************
// @Macro         GPT1_StartTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro starts the selected GPT1 timer. The timer 
//                continues to count from where it had stopped. The following 
//                definitions are available for TimerNr:
//                GPT1_TIMER_2
//                GPT1_TIMER_3
//                GPT1_TIMER_4
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the timer to be started
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define GPT1_StartTimer(TimerNr) GPT1_StartTimer_##TimerNr()
#define GPT1_StartTimer_GPT1_TIMER_2() T2CON_T2R = 1
#define GPT1_StartTimer_GPT1_TIMER_3() T3CON_T3R = 1
#define GPT1_StartTimer_GPT1_TIMER_4() T4CON_T4R = 1


//****************************************************************************
// @Macro         GPT1_StopTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro stops the selected GPT1 timer. The contents of 
//                the timer register remain unchanged. The following 
//                definitions are available for TimerNr:
//                GPT1_TIMER_2
//                GPT1_TIMER_3
//                GPT1_TIMER_4
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the timer to be stopped
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define GPT1_StopTimer(TimerNr) GPT1_StopTimer_##TimerNr()
#define GPT1_StopTimer_GPT1_TIMER_2() T2CON_T2R = 0
#define GPT1_StopTimer_GPT1_TIMER_3() T3CON_T3R = 0
#define GPT1_StopTimer_GPT1_TIMER_4() T4CON_T4R = 0


//****************************************************************************
// @Macro         GPT1_ReadTimer(TimerNr) 
//
//----------------------------------------------------------------------------
// @Description   This macro returns the contents of the selected GPT1 timer. 
//                The timer is not stopped. Access of the CPU to the 
//                registers of the timer is of higher priority than a timer 
//                increment, a reload or a capture event. The following 
//                definitions are available for TimerNr:
//                GPT1_TIMER_2
//                GPT1_TIMER_3
//                GPT1_TIMER_4
//
//----------------------------------------------------------------------------
// @Returnvalue   current 16-bit value of the timer
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the timer to be readed
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define GPT1_ReadTimer(TimerNr) TimerNr


//****************************************************************************
// @Macro         GPT1_LoadTimer(TimerNr, Value) 
//
//----------------------------------------------------------------------------
// @Description   This macro loads the selected GPT1 timer with the forwarded 
//                value. The timer is not stopped. The access of the CPU to 
//                the registers of the timers is of higher priority than a 
//                timer increment, a reload or a capture event. The following 
//                definitions are available for TimerNr:
//                GPT1_TIMER_2
//                GPT1_TIMER_3
//                GPT1_TIMER_4
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    TimerNr: 
//                Name of the timer to be loaded
// @Parameters    Value: 
//                16-bit value to be loaded
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define GPT1_LoadTimer(TimerNr, Value) TimerNr = Value


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************


// USER CODE BEGIN (GPT1_Header,9)

// USER CODE END


#endif  // ifndef _GPT1_H_
