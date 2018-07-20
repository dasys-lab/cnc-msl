//****************************************************************************
// @Module        Parallel Ports
// @Filename      IO.C
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the IO module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:16
//
//****************************************************************************

// USER CODE BEGIN (IO_Header,1)

// USER CODE END



#ifndef _IO_H_
#define _IO_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (IO_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (IO_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************

//----------------------------------------------------------------------------
// Defines for the parameter PinName
//----------------------------------------------------------------------------

//   
#define IO_P0H_0 P0H_P0H_0

//   
#define IO_P0H_1 P0H_P0H_1

//   
#define IO_P0H_2 P0H_P0H_2

//   
#define IO_P0H_3 P0H_P0H_3

//   
#define IO_P0H_4 P0H_P0H_4

//   
#define IO_P0H_5 P0H_P0H_5

//   
#define IO_P0H_6 P0H_P0H_6

//   
#define IO_P0H_7 P0H_P0H_7


//   
#define IO_P0L_0 P0L_P0L_0

//   
#define IO_P0L_1 P0L_P0L_1

//   
#define IO_P0L_2 P0L_P0L_2

//   
#define IO_P0L_3 P0L_P0L_3

//   
#define IO_P0L_4 P0L_P0L_4

//   
#define IO_P0L_5 P0L_P0L_5

//   
#define IO_P0L_6 P0L_P0L_6

//   
#define IO_P0L_7 P0L_P0L_7


//   
#define IO_P1H_0 P1H_P1H_0

//   
#define IO_P1H_1 P1H_P1H_1

//   
#define IO_P1H_2 P1H_P1H_2

//   
#define IO_P1H_3 P1H_P1H_3

//   
#define IO_P1H_4 P1H_P1H_4

//   
#define IO_P1H_5 P1H_P1H_5

//   
#define IO_P1H_6 P1H_P1H_6

//   
#define IO_P1H_7 P1H_P1H_7


//   
#define IO_P1L_0 P1L_P1L_0

//   
#define IO_P1L_1 P1L_P1L_1

//   
#define IO_P1L_2 P1L_P1L_2

//   
#define IO_P1L_3 P1L_P1L_3

//   
#define IO_P1L_4 P1L_P1L_4

//   
#define IO_P1L_5 P1L_P1L_5

//   
#define IO_P1L_6 P1L_P1L_6

//   
#define IO_P1L_7 P1L_P1L_7


//   
#define IO_P3_4 P3_P3_4

//   
#define IO_P3_6 P3_P3_6

//   
#define IO_P3_8 P3_P3_8

//   
#define IO_P3_9 P3_P3_9

//   
#define IO_P3_10 P3_P3_10

//   
#define IO_P3_11 P3_P3_11

//   
#define IO_P3_12 P3_P3_12

//   
#define IO_P3_13 P3_P3_13

//   
#define IO_P3_15 P3_P3_15


//   
#define IO_P4_0 P4_P4_0

//   
#define IO_P4_1 P4_P4_1

//   
#define IO_P4_2 P4_P4_2

//   
#define IO_P4_3 P4_P4_3

//   
#define IO_P4_5 P4_P4_5

//   
#define IO_P4_6 P4_P4_6


//   
#define IO_P5_0 P5_P5_0

//   
#define IO_P5_1 P5_P5_1

//   
#define IO_P5_2 P5_P5_2

//   
#define IO_P5_3 P5_P5_3

//   
#define IO_P5_4 P5_P5_4

//   
#define IO_P5_5 P5_P5_5

//   
#define IO_P5_6 P5_P5_6

//   
#define IO_P5_7 P5_P5_7


//   
#define IO_P8_0 P8_P8_0

//   
#define IO_P8_1 P8_P8_1

//   
#define IO_P8_2 P8_P8_2

//   
#define IO_P8_3 P8_P8_3



//----------------------------------------------------------------------------
// Defines used by DAvE
//----------------------------------------------------------------------------

#define INIO_P0H_0       P0H_P0H_0
#define INIO_P0H_1       P0H_P0H_1
#define INIO_P0H_2       P0H_P0H_2
#define INIO_P0H_3       P0H_P0H_3
#define INIO_P0H_4       P0H_P0H_4
#define INIO_P0H_5       P0H_P0H_5
#define INIO_P0H_6       P0H_P0H_6
#define INIO_P0H_7       P0H_P0H_7

#define DIO_P0H_0       DP0H_DP0H_0
#define DIO_P0H_1       DP0H_DP0H_1
#define DIO_P0H_2       DP0H_DP0H_2
#define DIO_P0H_3       DP0H_DP0H_3
#define DIO_P0H_4       DP0H_DP0H_4
#define DIO_P0H_5       DP0H_DP0H_5
#define DIO_P0H_6       DP0H_DP0H_6
#define DIO_P0H_7       DP0H_DP0H_7

#define INIO_P0L_0       P0L_P0L_0
#define INIO_P0L_1       P0L_P0L_1
#define INIO_P0L_2       P0L_P0L_2
#define INIO_P0L_3       P0L_P0L_3
#define INIO_P0L_4       P0L_P0L_4
#define INIO_P0L_5       P0L_P0L_5
#define INIO_P0L_6       P0L_P0L_6
#define INIO_P0L_7       P0L_P0L_7

#define DIO_P0L_0       DP0L_DP0L_0
#define DIO_P0L_1       DP0L_DP0L_1
#define DIO_P0L_2       DP0L_DP0L_2
#define DIO_P0L_3       DP0L_DP0L_3
#define DIO_P0L_4       DP0L_DP0L_4
#define DIO_P0L_5       DP0L_DP0L_5
#define DIO_P0L_6       DP0L_DP0L_6
#define DIO_P0L_7       DP0L_DP0L_7

#define INIO_P1H_0       P1H_P1H_0
#define INIO_P1H_1       P1H_P1H_1
#define INIO_P1H_2       P1H_P1H_2
#define INIO_P1H_3       P1H_P1H_3
#define INIO_P1H_4       P1H_P1H_4
#define INIO_P1H_5       P1H_P1H_5
#define INIO_P1H_6       P1H_P1H_6
#define INIO_P1H_7       P1H_P1H_7

#define DIO_P1H_0       DP1H_DP1H_0
#define DIO_P1H_1       DP1H_DP1H_1
#define DIO_P1H_2       DP1H_DP1H_2
#define DIO_P1H_3       DP1H_DP1H_3
#define DIO_P1H_4       DP1H_DP1H_4
#define DIO_P1H_5       DP1H_DP1H_5
#define DIO_P1H_6       DP1H_DP1H_6
#define DIO_P1H_7       DP1H_DP1H_7

#define INIO_P1L_0       P1L_P1L_0
#define INIO_P1L_1       P1L_P1L_1
#define INIO_P1L_2       P1L_P1L_2
#define INIO_P1L_3       P1L_P1L_3
#define INIO_P1L_4       P1L_P1L_4
#define INIO_P1L_5       P1L_P1L_5
#define INIO_P1L_6       P1L_P1L_6
#define INIO_P1L_7       P1L_P1L_7

#define DIO_P1L_0       DP1L_DP1L_0
#define DIO_P1L_1       DP1L_DP1L_1
#define DIO_P1L_2       DP1L_DP1L_2
#define DIO_P1L_3       DP1L_DP1L_3
#define DIO_P1L_4       DP1L_DP1L_4
#define DIO_P1L_5       DP1L_DP1L_5
#define DIO_P1L_6       DP1L_DP1L_6
#define DIO_P1L_7       DP1L_DP1L_7

#define INIO_P3_4       P3_P3_4
#define INIO_P3_6       P3_P3_6
#define INIO_P3_8       P3_P3_8
#define INIO_P3_9       P3_P3_9
#define INIO_P3_10       P3_P3_10
#define INIO_P3_11       P3_P3_11
#define INIO_P3_12       P3_P3_12
#define INIO_P3_13       P3_P3_13
#define INIO_P3_15       P3_P3_15

#define DIO_P3_4       DP3_DP3_4
#define DIO_P3_6       DP3_DP3_6
#define DIO_P3_8       DP3_DP3_8
#define DIO_P3_9       DP3_DP3_9
#define DIO_P3_10       DP3_DP3_10
#define DIO_P3_11       DP3_DP3_11
#define DIO_P3_12       DP3_DP3_12
#define DIO_P3_13       DP3_DP3_13
#define DIO_P3_15       DP3_DP3_15

#define INIO_P4_0       P4_P4_0
#define INIO_P4_1       P4_P4_1
#define INIO_P4_2       P4_P4_2
#define INIO_P4_3       P4_P4_3
#define INIO_P4_5       P4_P4_5
#define INIO_P4_6       P4_P4_6

#define DIO_P4_0       DP4_DP4_0
#define DIO_P4_1       DP4_DP4_1
#define DIO_P4_2       DP4_DP4_2
#define DIO_P4_3       DP4_DP4_3
#define DIO_P4_5       DP4_DP4_5
#define DIO_P4_6       DP4_DP4_6

#define INIO_P5_0       P5_P5_0
#define INIO_P5_1       P5_P5_1
#define INIO_P5_2       P5_P5_2
#define INIO_P5_3       P5_P5_3
#define INIO_P5_4       P5_P5_4
#define INIO_P5_5       P5_P5_5
#define INIO_P5_6       P5_P5_6
#define INIO_P5_7       P5_P5_7


#define INIO_P8_0       P8_P8_0
#define INIO_P8_1       P8_P8_1
#define INIO_P8_2       P8_P8_2
#define INIO_P8_3       P8_P8_3

#define DIO_P8_0       DP8_DP8_0
#define DIO_P8_1       DP8_DP8_1
#define DIO_P8_2       DP8_DP8_2
#define DIO_P8_3       DP8_DP8_3

// USER CODE BEGIN (IO_Header,4)

// USER CODE END


//****************************************************************************
// @Typedefs
//****************************************************************************

// USER CODE BEGIN (IO_Header,5)

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (IO_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (IO_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************

void IO_Init(void);


// USER CODE BEGIN (IO_Header,8)

// USER CODE END


//****************************************************************************
// @Macro         IO_ReadPin(PinName) 
//
//----------------------------------------------------------------------------
// @Description   This macro returns the status of the chosen portpin.
//                Note:
//                See the 'Defines for the parameter PinName' section in this 
//                header file for the available definitions for the parameter 
//                PinName.
//
//----------------------------------------------------------------------------
// @Returnvalue   Status of the chosen portpin
//
//----------------------------------------------------------------------------
// @Parameters    PinName: 
//                Pin to be read
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define IO_ReadPin(PinName) PinName


//****************************************************************************
// @Macro         IO_SetPin(PinName) 
//
//----------------------------------------------------------------------------
// @Description   This macro sets the chosen portpin to '1'.
//                Note:
//                See the 'Defines for the parameter PinName' section in this 
//                header file for the available definitions for the parameter 
//                PinName.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    PinName: 
//                Pin to be set to '1'
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define IO_SetPin(PinName) PinName = 1


//****************************************************************************
// @Macro         IO_ResetPin(PinName) 
//
//----------------------------------------------------------------------------
// @Description   This macro sets the chosen portpin to '0'. 
//                Note: 
//                See the 'Defines for the parameter PinName' section in this 
//                header file for the available definitions for the parameter 
//                PinName.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    PinName: 
//                Pin to be set to '0'
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define IO_ResetPin(PinName) PinName = 0


//****************************************************************************
// @Macro         IO_TogglePin(PinName) 
//
//----------------------------------------------------------------------------
// @Description   This marco toggles the chosen portpin. 
//                Note:
//                See the 'Defines for the parameter PinName' section in this 
//                header file for the available definitions for the parameter 
//                PinName.
//
//----------------------------------------------------------------------------
// @Returnvalue   None
//
//----------------------------------------------------------------------------
// @Parameters    PinName: 
//                Pin to be toggled
//
//----------------------------------------------------------------------------
// @Date          02.01.2009
//
//****************************************************************************

#define IO_TogglePin(PinName) PinName = !PinName


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************

// USER CODE BEGIN (IO_Header,9)

// USER CODE END


#endif  // ifndef _IO_H_
