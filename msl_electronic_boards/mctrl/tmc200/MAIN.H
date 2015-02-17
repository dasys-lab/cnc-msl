//****************************************************************************
// @Module        Project Settings
// @Filename      MAIN.H
// @Project       cn_tmc200.dav
//----------------------------------------------------------------------------
// @Controller    Infineon C164CI-L
//
// @Compiler      Keil
//
// @Codegenerator 2.1
//
// @Description   This file contains all function prototypes and macros for 
//                the MAIN module.
//
//----------------------------------------------------------------------------
// @Date          02.01.2009 15:59:16
//
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,1)

// USER CODE END



#ifndef _MAIN_H_
#define _MAIN_H_

//****************************************************************************
// @Project Includes
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,2)

// USER CODE END


//****************************************************************************
// @Macros
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,3)

// USER CODE END


//****************************************************************************
// @Defines
//****************************************************************************
#define KEIL
#define SRVWDT _srvwdt_()

// USER CODE BEGIN (MAIN_Header,4)

// USER CODE END


//****************************************************************************
// @Declaration of SFRs
//****************************************************************************


//The source and destination pointers SRCPx and DSTPx
#define SRCP0                  (*((uword volatile *) 0xFCE0))
#define DSTP0                  (*((uword volatile *) 0xFCE2))
#define SRCP1                  (*((uword volatile *) 0xFCE4))
#define DSTP1                  (*((uword volatile *) 0xFCE6))
#define SRCP2                  (*((uword volatile *) 0xFCE8))
#define DSTP2                  (*((uword volatile *) 0xFCEA))
#define SRCP3                  (*((uword volatile *) 0xFCEC))
#define DSTP3                  (*((uword volatile *) 0xFCEE))
#define SRCP4                  (*((uword volatile *) 0xFCF0))
#define DSTP4                  (*((uword volatile *) 0xFCF2))
#define SRCP5                  (*((uword volatile *) 0xFCF4))
#define DSTP5                  (*((uword volatile *) 0xFCF6))
#define SRCP6                  (*((uword volatile *) 0xFCF8))
#define DSTP6                  (*((uword volatile *) 0xFCFA))
#define SRCP7                  (*((uword volatile *) 0xFCFC))
#define DSTP7                  (*((uword volatile *) 0xFCFE))

// A/D Converter End of Conversion Interrupt Control Register
#define ADCIC                  (*((uword volatile *) 0xFF98))
#define ADCIC_ADCIE                    ((T_Reg16 *) 0xFF98)->bit6
#define ADCIC_ADCIR                    ((T_Reg16 *) 0xFF98)->bit7

// A/D Converter Control Register
#define ADCON                  (*((uword volatile *) 0xFFA0))
#define ADCON_ADBSY                    ((T_Reg16 *) 0xFFA0)->bit8
#define ADCON_ADCIN                    ((T_Reg16 *) 0xFFA0)->bit10
#define ADCON_ADCRQ                    ((T_Reg16 *) 0xFFA0)->bit11
#define ADCON_ADST                     ((T_Reg16 *) 0xFFA0)->bit7
#define ADCON_ADWR                     ((T_Reg16 *) 0xFFA0)->bit9
#define ADCON_ADX                      ((T_Reg16 *) 0xFFA0)->bit6

// A/D Converter Result Register
#define ADDAT                  (*((uword volatile *) 0xFEA0))

// A/D Converter 2 Result Register
#define ADDAT2                 (*((uword volatile *) 0xF0A0))
#define ADDAT2_CHX                      ((T_Reg16 *) 0xF0A0)->bit11

// Address Select Register 1
#define ADDRSEL1               (*((uword volatile *) 0xFE18))

// Address Select Register 2
#define ADDRSEL2               (*((uword volatile *) 0xFE1A))

// Address Select Register 3
#define ADDRSEL3               (*((uword volatile *) 0xFE1C))

// Address Select Register 4
#define ADDRSEL4               (*((uword volatile *) 0xFE1E))

// A/D Converter Overrun Error Interrupt Control Register
#define ADEIC                  (*((uword volatile *) 0xFF9A))
#define ADEIC_ADEIE                    ((T_Reg16 *) 0xFF9A)->bit6
#define ADEIC_ADEIR                    ((T_Reg16 *) 0xFF9A)->bit7

// Bus Configuration Register 0
#define BUSCON0                (*((uword volatile *) 0xFF0C))
#define BUSCON0_ALECTL0                  ((T_Reg16 *) 0xFF0C)->bit9
#define BUSCON0_BSWC0                    ((T_Reg16 *) 0xFF0C)->bit11
#define BUSCON0_BUSACT0                  ((T_Reg16 *) 0xFF0C)->bit10
#define BUSCON0_CSREN0                   ((T_Reg16 *) 0xFF0C)->bit14
#define BUSCON0_CSWEN0                   ((T_Reg16 *) 0xFF0C)->bit15
#define BUSCON0_EWEN0                    ((T_Reg16 *) 0xFF0C)->bit8
#define BUSCON0_MTTC0                    ((T_Reg16 *) 0xFF0C)->bit5
#define BUSCON0_RWDC0                    ((T_Reg16 *) 0xFF0C)->bit4

// Bus Configuration Register 1
#define BUSCON1                (*((uword volatile *) 0xFF14))
#define BUSCON1_ALECTL1                  ((T_Reg16 *) 0xFF14)->bit9
#define BUSCON1_BSWC1                    ((T_Reg16 *) 0xFF14)->bit11
#define BUSCON1_BUSACT1                  ((T_Reg16 *) 0xFF14)->bit10
#define BUSCON1_CSREN1                   ((T_Reg16 *) 0xFF14)->bit14
#define BUSCON1_CSWEN1                   ((T_Reg16 *) 0xFF14)->bit15
#define BUSCON1_EWEN1                    ((T_Reg16 *) 0xFF14)->bit8
#define BUSCON1_MTTC1                    ((T_Reg16 *) 0xFF14)->bit5
#define BUSCON1_RWDC1                    ((T_Reg16 *) 0xFF14)->bit4

// Bus Configuration Register 2
#define BUSCON2                (*((uword volatile *) 0xFF16))
#define BUSCON2_ALECTL2                  ((T_Reg16 *) 0xFF16)->bit9
#define BUSCON2_BSWC2                    ((T_Reg16 *) 0xFF16)->bit11
#define BUSCON2_BUSACT2                  ((T_Reg16 *) 0xFF16)->bit10
#define BUSCON2_CSREN2                   ((T_Reg16 *) 0xFF16)->bit14
#define BUSCON2_CSWEN2                   ((T_Reg16 *) 0xFF16)->bit15
#define BUSCON2_EWEN2                    ((T_Reg16 *) 0xFF16)->bit8
#define BUSCON2_MTTC2                    ((T_Reg16 *) 0xFF16)->bit5
#define BUSCON2_RWDC2                    ((T_Reg16 *) 0xFF16)->bit4

// Bus Configuration Register 3
#define BUSCON3                (*((uword volatile *) 0xFF18))
#define BUSCON3_ALECTL3                  ((T_Reg16 *) 0xFF18)->bit9
#define BUSCON3_BSWC3                    ((T_Reg16 *) 0xFF18)->bit11
#define BUSCON3_BUSACT3                  ((T_Reg16 *) 0xFF18)->bit10
#define BUSCON3_CSREN3                   ((T_Reg16 *) 0xFF18)->bit14
#define BUSCON3_CSWEN3                   ((T_Reg16 *) 0xFF18)->bit15
#define BUSCON3_EWEN3                    ((T_Reg16 *) 0xFF18)->bit8
#define BUSCON3_MTTC3                    ((T_Reg16 *) 0xFF18)->bit5
#define BUSCON3_RWDC3                    ((T_Reg16 *) 0xFF18)->bit4

// Bus Configuration Register 4
#define BUSCON4                (*((uword volatile *) 0xFF1A))
#define BUSCON4_ALECTL4                  ((T_Reg16 *) 0xFF1A)->bit9
#define BUSCON4_BSWC4                    ((T_Reg16 *) 0xFF1A)->bit11
#define BUSCON4_BUSACT4                  ((T_Reg16 *) 0xFF1A)->bit10
#define BUSCON4_CSREN4                   ((T_Reg16 *) 0xFF1A)->bit14
#define BUSCON4_CSWEN4                   ((T_Reg16 *) 0xFF1A)->bit15
#define BUSCON4_EWEN4                    ((T_Reg16 *) 0xFF1A)->bit8
#define BUSCON4_MTTC4                    ((T_Reg16 *) 0xFF1A)->bit5
#define BUSCON4_RWDC4                    ((T_Reg16 *) 0xFF1A)->bit4

// CAN1 Bit Timing Register
#define C1BTR                  (*((uword volatile *) 0xEF04))

// CAN1 Control / Status Register
#define C1CSR                  (*((uword volatile *) 0xEF00))

// CAN1 Global Mask Short
#define C1GMS                  (*((uword volatile *) 0xEF06))

// CAN1 Lower Arbitration Register 1
#define C1LAR1                 (*((uword volatile *) 0xEF14))

// CAN1 Lower Arbitration Register 10
#define C1LAR10                (*((uword volatile *) 0xEFA4))

// CAN1 Lower Arbitration Register 11
#define C1LAR11                (*((uword volatile *) 0xEFB4))

// CAN1 Lower Arbitration Register 12
#define C1LAR12                (*((uword volatile *) 0xEFC4))

// CAN1 Lower Arbitration Register 13
#define C1LAR13                (*((uword volatile *) 0xEFD4))

// CAN1 Lower Arbitration Register 14
#define C1LAR14                (*((uword volatile *) 0xEFE4))

// CAN1 Lower Arbitration Register 15
#define C1LAR15                (*((uword volatile *) 0xEFF4))

// CAN1 Lower Arbitration Register 2
#define C1LAR2                 (*((uword volatile *) 0xEF24))

// CAN1 Lower Arbitration Register 3
#define C1LAR3                 (*((uword volatile *) 0xEF34))

// CAN1 Lower Arbitration Register 4
#define C1LAR4                 (*((uword volatile *) 0xEF44))

// CAN1 Lower Arbitration Register 5
#define C1LAR5                 (*((uword volatile *) 0xEF54))

// CAN1 Lower Arbitration Register 6
#define C1LAR6                 (*((uword volatile *) 0xEF64))

// CAN1 Lower Arbitration Register 7
#define C1LAR7                 (*((uword volatile *) 0xEF74))

// CAN1 Lower Arbitration Register 8
#define C1LAR8                 (*((uword volatile *) 0xEF84))

// CAN1 Lower Arbitration Register 9
#define C1LAR9                 (*((uword volatile *) 0xEF94))

// CAN1 Lower Global Mask Long
#define C1LGML                 (*((uword volatile *) 0xEF0A))

// CAN1 Lower Mask of Last Message
#define C1LMLM                 (*((uword volatile *) 0xEF0E))

// CAN1 Message Configuratiom Register 1
#define C1MCFG1                (*((uword volatile *) 0xEF16))

// CAN1 Message Configuratiom Register 10
#define C1MCFG10               (*((uword volatile *) 0xEFA6))

// CAN1 Message Configuratiom Register 11
#define C1MCFG11               (*((uword volatile *) 0xEFB6))

// CAN1 Message Configuratiom Register 12
#define C1MCFG12               (*((uword volatile *) 0xEFC6))

// CAN1 Message Configuratiom Register 13
#define C1MCFG13               (*((uword volatile *) 0xEFD6))

// CAN1 Message Configuratiom Register 14
#define C1MCFG14               (*((uword volatile *) 0xEFE6))

// CAN1 Message Configuratiom Register 15
#define C1MCFG15               (*((uword volatile *) 0xEFF6))

// CAN1 Message Configuratiom Register 2
#define C1MCFG2                (*((uword volatile *) 0xEF26))

// CAN1 Message Configuratiom Register 3
#define C1MCFG3                (*((uword volatile *) 0xEF36))

// CAN1 Message Configuratiom Register 4
#define C1MCFG4                (*((uword volatile *) 0xEF46))

// CAN1 Message Configuratiom Register 5
#define C1MCFG5                (*((uword volatile *) 0xEF56))

// CAN1 Message Configuratiom Register 6
#define C1MCFG6                (*((uword volatile *) 0xEF66))

// CAN1 Message Configuratiom Register 7
#define C1MCFG7                (*((uword volatile *) 0xEF76))

// CAN1 Message Configuratiom Register 8
#define C1MCFG8                (*((uword volatile *) 0xEF86))

// CAN1 Message Configuratiom Register 9
#define C1MCFG9                (*((uword volatile *) 0xEF96))

// CAN1 Message Control Register 1
#define C1MCR1                 (*((uword volatile *) 0xEF10))

// CAN1 Message Control Register 10
#define C1MCR10                (*((uword volatile *) 0xEFA0))

// CAN1 Message Control Register 11
#define C1MCR11                (*((uword volatile *) 0xEFB0))

// CAN1 Message Control Register 12
#define C1MCR12                (*((uword volatile *) 0xEFC0))

// CAN1 Message Control Register 13
#define C1MCR13                (*((uword volatile *) 0xEFD0))

// CAN1 Message Control Register 14
#define C1MCR14                (*((uword volatile *) 0xEFE0))

// CAN1 Message Control Register 15
#define C1MCR15                (*((uword volatile *) 0xEFF0))

// CAN1 Message Control Register 2
#define C1MCR2                 (*((uword volatile *) 0xEF20))

// CAN1 Message Control Register 3
#define C1MCR3                 (*((uword volatile *) 0xEF30))

// CAN1 Message Control Register 4
#define C1MCR4                 (*((uword volatile *) 0xEF40))

// CAN1 Message Control Register 5
#define C1MCR5                 (*((uword volatile *) 0xEF50))

// CAN1 Message Control Register 6
#define C1MCR6                 (*((uword volatile *) 0xEF60))

// CAN1 Message Control Register 7
#define C1MCR7                 (*((uword volatile *) 0xEF70))

// CAN1 Message Control Register 8
#define C1MCR8                 (*((uword volatile *) 0xEF80))

// CAN1 Message Control Register 9
#define C1MCR9                 (*((uword volatile *) 0xEF90))

// CAN1 Port Control and Interrupt Register
#define C1PCIR                 (*((uword volatile *) 0xEF02))

// CAN1 Upper Arbitration Register 1
#define C1UAR1                 (*((uword volatile *) 0xEF12))

// CAN1 Upper Arbitration Register 10
#define C1UAR10                (*((uword volatile *) 0xEFA2))

// CAN1 Upper Arbitration Register 11
#define C1UAR11                (*((uword volatile *) 0xEFB2))

// CAN1 Upper Arbitration Register 12
#define C1UAR12                (*((uword volatile *) 0xEFC2))

// CAN1 Upper Arbitration Register 13
#define C1UAR13                (*((uword volatile *) 0xEFD2))

// CAN1 Upper Arbitration Register 14
#define C1UAR14                (*((uword volatile *) 0xEFE2))

// CAN1 Upper Arbitration Register 15
#define C1UAR15                (*((uword volatile *) 0xEFF2))

// CAN1 Upper Arbitration Register 2
#define C1UAR2                 (*((uword volatile *) 0xEF22))

// CAN1 Upper Arbitration Register 3
#define C1UAR3                 (*((uword volatile *) 0xEF32))

// CAN1 Upper Arbitration Register 4
#define C1UAR4                 (*((uword volatile *) 0xEF42))

// CAN1 Upper Arbitration Register 5
#define C1UAR5                 (*((uword volatile *) 0xEF52))

// CAN1 Upper Arbitration Register 6
#define C1UAR6                 (*((uword volatile *) 0xEF62))

// CAN1 Upper Arbitration Register 7
#define C1UAR7                 (*((uword volatile *) 0xEF72))

// CAN1 Upper Arbitration Register 8
#define C1UAR8                 (*((uword volatile *) 0xEF82))

// CAN1 Upper Arbitration Register 9
#define C1UAR9                 (*((uword volatile *) 0xEF92))

// CAN1 Upper Global Mask Long
#define C1UGML                 (*((uword volatile *) 0xEF08))

// CAN1 Upper Mask of Last Message
#define C1UMLM                 (*((uword volatile *) 0xEF0C))

// External Interrupt 2 Control Register
#define CC10IC                 (*((uword volatile *) 0xFF8C))
#define CC10IC_CC10IE                   ((T_Reg16 *) 0xFF8C)->bit6
#define CC10IC_CC10IR                   ((T_Reg16 *) 0xFF8C)->bit7

// External Interrupt 3 Control Register
#define CC11IC                 (*((uword volatile *) 0xFF8E))
#define CC11IC_CC11IE                   ((T_Reg16 *) 0xFF8E)->bit6
#define CC11IC_CC11IR                   ((T_Reg16 *) 0xFF8E)->bit7

// CAPCOM Register 16
#define CC16                   (*((uword volatile *) 0xFE60))

// Capture/Compare Register 16 Interrupt Control Register
#define CC16IC                 (*((uword volatile *) 0xF160))
#define CC16IC_CC16IE                   ((T_Reg16 *) 0xF160)->bit6
#define CC16IC_CC16IR                   ((T_Reg16 *) 0xF160)->bit7

// CAPCOM Register 17
#define CC17                   (*((uword volatile *) 0xFE62))

// Capture/Compare Register 17 Interrupt Control Register
#define CC17IC                 (*((uword volatile *) 0xF162))
#define CC17IC_CC17IE                   ((T_Reg16 *) 0xF162)->bit6
#define CC17IC_CC17IR                   ((T_Reg16 *) 0xF162)->bit7

// CAPCOM Register 18
#define CC18                   (*((uword volatile *) 0xFE64))

// Capture/Compare Register 18 Interrupt Control Register
#define CC18IC                 (*((uword volatile *) 0xF164))
#define CC18IC_CC18IE                   ((T_Reg16 *) 0xF164)->bit6
#define CC18IC_CC18IR                   ((T_Reg16 *) 0xF164)->bit7

// CAPCOM Register 19
#define CC19                   (*((uword volatile *) 0xFE66))

// Capture/Compare Register 19 Interrupt Control Register
#define CC19IC                 (*((uword volatile *) 0xF166))
#define CC19IC_CC19IE                   ((T_Reg16 *) 0xF166)->bit6
#define CC19IC_CC19IR                   ((T_Reg16 *) 0xF166)->bit7

// CAPCOM Register 20
#define CC20                   (*((uword volatile *) 0xFE68))

// CAPCOM Register 21
#define CC21                   (*((uword volatile *) 0xFE6A))

// CAPCOM Register 22
#define CC22                   (*((uword volatile *) 0xFE6C))

// CAPCOM Register 23
#define CC23                   (*((uword volatile *) 0xFE6E))

// CAPCOM Register 24
#define CC24                   (*((uword volatile *) 0xFE70))

// Capture/Compare Register 24 Interrupt Control Register
#define CC24IC                 (*((uword volatile *) 0xF170))
#define CC24IC_CC24IE                   ((T_Reg16 *) 0xF170)->bit6
#define CC24IC_CC24IR                   ((T_Reg16 *) 0xF170)->bit7

// CAPCOM Register 25
#define CC25                   (*((uword volatile *) 0xFE72))

// Capture/Compare Register 25 Interrupt Control Register
#define CC25IC                 (*((uword volatile *) 0xF172))
#define CC25IC_CC25IE                   ((T_Reg16 *) 0xF172)->bit6
#define CC25IC_CC25IR                   ((T_Reg16 *) 0xF172)->bit7

// CAPCOM Register 26
#define CC26                   (*((uword volatile *) 0xFE74))

// Capture/Compare Register 26 Interrupt Control Register
#define CC26IC                 (*((uword volatile *) 0xF174))
#define CC26IC_CC26IE                   ((T_Reg16 *) 0xF174)->bit6
#define CC26IC_CC26IR                   ((T_Reg16 *) 0xF174)->bit7

// CAPCOM Register 27
#define CC27                   (*((uword volatile *) 0xFE76))

// Capture/Compare Register 27 Interrupt Control Register
#define CC27IC                 (*((uword volatile *) 0xF176))
#define CC27IC_CC27IE                   ((T_Reg16 *) 0xF176)->bit6
#define CC27IC_CC27IR                   ((T_Reg16 *) 0xF176)->bit7

// CAPCOM Register 28
#define CC28                   (*((uword volatile *) 0xFE78))

// CAPCOM Register 29
#define CC29                   (*((uword volatile *) 0xFE7A))

// CAPCOM Register 30
#define CC30                   (*((uword volatile *) 0xFE7C))

// CAPCOM Register 31
#define CC31                   (*((uword volatile *) 0xFE7E))

// CAPCOM 6 Register Channel 0
#define CC60                   (*((uword volatile *) 0xFE30))

// CAPCOM 6 Register Channel 1
#define CC61                   (*((uword volatile *) 0xFE32))

// CAPCOM 6 Register Channel 2
#define CC62                   (*((uword volatile *) 0xFE34))

// GPT1 Timer 2 Interrupt Control Register
#define CC6CIC                 (*((uword volatile *) 0xF17E))
#define CC6CIC_CC6IE                    ((T_Reg16 *) 0xF17E)->bit6
#define CC6CIC_CC6IR                    ((T_Reg16 *) 0xF17E)->bit7

// GPT1 Timer 2 Interrupt Control Register
#define CC6EIC                 (*((uword volatile *) 0xF188))
#define CC6EIC_CC6EIE                   ((T_Reg16 *) 0xF188)->bit6
#define CC6EIC_CC6EIR                   ((T_Reg16 *) 0xF188)->bit7

// CAPCOM 6 Mode Control Register
#define CC6MCON                (*((uword volatile *) 0xFF32))
#define CC6MCON_BCEN                     ((T_Reg16 *) 0xFF32)->bit10
#define CC6MCON_BCERR                    ((T_Reg16 *) 0xFF32)->bit11
#define CC6MCON_BCMP_BCEM                ((T_Reg16 *) 0xFF32)->bit15
#define CC6MCON_CC0I                     ((T_Reg16 *) 0xFF32)->bit0
#define CC6MCON_CC1I                     ((T_Reg16 *) 0xFF32)->bit2
#define CC6MCON_CC2I                     ((T_Reg16 *) 0xFF32)->bit4
#define CC6MCON_COUT0I                   ((T_Reg16 *) 0xFF32)->bit1
#define CC6MCON_COUT1I                   ((T_Reg16 *) 0xFF32)->bit3
#define CC6MCON_COUT2I                   ((T_Reg16 *) 0xFF32)->bit5
#define CC6MCON_COUT3I                   ((T_Reg16 *) 0xFF32)->bit7
#define CC6MCON_COUTXI                   ((T_Reg16 *) 0xFF32)->bit6
#define CC6MCON_EBCE                     ((T_Reg16 *) 0xFF32)->bit12

// CAPCOM 6 Mode Interrupt Control Register
#define CC6MIC                 (*((uword volatile *) 0xFF36))
#define CC6MIC_CC0F                     ((T_Reg16 *) 0xFF36)->bit9
#define CC6MIC_CC0FEN                   ((T_Reg16 *) 0xFF36)->bit1
#define CC6MIC_CC0R                     ((T_Reg16 *) 0xFF36)->bit8
#define CC6MIC_CC0REN                   ((T_Reg16 *) 0xFF36)->bit0
#define CC6MIC_CC1F                     ((T_Reg16 *) 0xFF36)->bit11
#define CC6MIC_CC1FEN                   ((T_Reg16 *) 0xFF36)->bit3
#define CC6MIC_CC1R                     ((T_Reg16 *) 0xFF36)->bit10
#define CC6MIC_CC1REN                   ((T_Reg16 *) 0xFF36)->bit2
#define CC6MIC_CC2F                     ((T_Reg16 *) 0xFF36)->bit13
#define CC6MIC_CC2FEN                   ((T_Reg16 *) 0xFF36)->bit5
#define CC6MIC_CC2R                     ((T_Reg16 *) 0xFF36)->bit12
#define CC6MIC_CC2REN                   ((T_Reg16 *) 0xFF36)->bit4
#define CC6MIC_CT12FC                   ((T_Reg16 *) 0xFF36)->bit14
#define CC6MIC_CT12FP                   ((T_Reg16 *) 0xFF36)->bit15
#define CC6MIC_ECTC                     ((T_Reg16 *) 0xFF36)->bit6
#define CC6MIC_ECTP                     ((T_Reg16 *) 0xFF36)->bit7

// CAPCOM 6 Mode Select Register
#define CC6MSEL                (*((uword volatile *) 0xF036))
#define CC6MSEL_CMSEL03                  ((T_Reg16 *) 0xF036)->bit3
#define CC6MSEL_CMSEL13                  ((T_Reg16 *) 0xF036)->bit7
#define CC6MSEL_CMSEL23                  ((T_Reg16 *) 0xF036)->bit11
#define CC6MSEL_ESMC                     ((T_Reg16 *) 0xF036)->bit15
#define CC6MSEL_NMCS                     ((T_Reg16 *) 0xF036)->bit14

// External Interrupt 0 Control Register
#define CC8IC                  (*((uword volatile *) 0xFF88))
#define CC8IC_CC8IE                    ((T_Reg16 *) 0xFF88)->bit6
#define CC8IC_CC8IR                    ((T_Reg16 *) 0xFF88)->bit7

// External Interrupt 1 Control Register
#define CC9IC                  (*((uword volatile *) 0xFF8A))
#define CC9IC_CC9IE                    ((T_Reg16 *) 0xFF8A)->bit6
#define CC9IC_CC9IR                    ((T_Reg16 *) 0xFF8A)->bit7

// CAPCOM Mode Control Register 4
#define CCM4                   (*((uword volatile *) 0xFF22))
#define CCM4_ACC16                    ((T_Reg16 *) 0xFF22)->bit3
#define CCM4_ACC17                    ((T_Reg16 *) 0xFF22)->bit7
#define CCM4_ACC18                    ((T_Reg16 *) 0xFF22)->bit11
#define CCM4_ACC19                    ((T_Reg16 *) 0xFF22)->bit15

// CAPCOM Mode Control Register 5
#define CCM5                   (*((uword volatile *) 0xFF24))
#define CCM5_ACC20                    ((T_Reg16 *) 0xFF24)->bit3
#define CCM5_ACC21                    ((T_Reg16 *) 0xFF24)->bit7
#define CCM5_ACC22                    ((T_Reg16 *) 0xFF24)->bit11
#define CCM5_ACC23                    ((T_Reg16 *) 0xFF24)->bit15

// CAPCOM Mode Control Register 6
#define CCM6                   (*((uword volatile *) 0xFF26))
#define CCM6_ACC24                    ((T_Reg16 *) 0xFF26)->bit3
#define CCM6_ACC25                    ((T_Reg16 *) 0xFF26)->bit7
#define CCM6_ACC26                    ((T_Reg16 *) 0xFF26)->bit11
#define CCM6_ACC27                    ((T_Reg16 *) 0xFF26)->bit15

// CAPCOM Mode Control Register 7
#define CCM7                   (*((uword volatile *) 0xFF28))
#define CCM7_ACC28                    ((T_Reg16 *) 0xFF28)->bit3
#define CCM7_ACC29                    ((T_Reg16 *) 0xFF28)->bit7
#define CCM7_ACC30                    ((T_Reg16 *) 0xFF28)->bit11
#define CCM7_ACC31                    ((T_Reg16 *) 0xFF28)->bit15

// CAPCOM 6 Timer 13 Compare Register
#define CMP13                  (*((uword volatile *) 0xFE36))

// CPU Contex Pointer Register
#define CP                     (*((uword volatile *) 0xFE10))

// CPU Code Segment Point Register
#define CSP                    (*((uword volatile *) 0xFE08))

// CAPCOM 6 Compare Timer Control Register
#define CTCON                  (*((uword volatile *) 0xFF30))
#define CTCON_CT12R                    ((T_Reg16 *) 0xFF30)->bit3
#define CTCON_CT12RES                  ((T_Reg16 *) 0xFF30)->bit4
#define CTCON_CT13P                    ((T_Reg16 *) 0xFF30)->bit15
#define CTCON_CT13R                    ((T_Reg16 *) 0xFF30)->bit11
#define CTCON_CT13RES                  ((T_Reg16 *) 0xFF30)->bit12
#define CTCON_CTM                      ((T_Reg16 *) 0xFF30)->bit7
#define CTCON_ECT13O                   ((T_Reg16 *) 0xFF30)->bit14
#define CTCON_ETRP                     ((T_Reg16 *) 0xFF30)->bit6
#define CTCON_STE12                    ((T_Reg16 *) 0xFF30)->bit5
#define CTCON_STE13                    ((T_Reg16 *) 0xFF30)->bit13

// Port 0 High Direction Control Register
#define DP0H                   (*((uword volatile *) 0xF102))
#define DP0H_DP0H_0                   ((T_Reg16 *) 0xF102)->bit0
#define DP0H_DP0H_1                   ((T_Reg16 *) 0xF102)->bit1
#define DP0H_DP0H_2                   ((T_Reg16 *) 0xF102)->bit2
#define DP0H_DP0H_3                   ((T_Reg16 *) 0xF102)->bit3
#define DP0H_DP0H_4                   ((T_Reg16 *) 0xF102)->bit4
#define DP0H_DP0H_5                   ((T_Reg16 *) 0xF102)->bit5
#define DP0H_DP0H_6                   ((T_Reg16 *) 0xF102)->bit6
#define DP0H_DP0H_7                   ((T_Reg16 *) 0xF102)->bit7

// Port 0 Low Direction Control Register
#define DP0L                   (*((uword volatile *) 0xF100))
#define DP0L_DP0L_0                   ((T_Reg16 *) 0xF100)->bit0
#define DP0L_DP0L_1                   ((T_Reg16 *) 0xF100)->bit1
#define DP0L_DP0L_2                   ((T_Reg16 *) 0xF100)->bit2
#define DP0L_DP0L_3                   ((T_Reg16 *) 0xF100)->bit3
#define DP0L_DP0L_4                   ((T_Reg16 *) 0xF100)->bit4
#define DP0L_DP0L_5                   ((T_Reg16 *) 0xF100)->bit5
#define DP0L_DP0L_6                   ((T_Reg16 *) 0xF100)->bit6
#define DP0L_DP0L_7                   ((T_Reg16 *) 0xF100)->bit7

// Port 1 High Direction Control Register
#define DP1H                   (*((uword volatile *) 0xF106))
#define DP1H_DP1H_0                   ((T_Reg16 *) 0xF106)->bit0
#define DP1H_DP1H_1                   ((T_Reg16 *) 0xF106)->bit1
#define DP1H_DP1H_2                   ((T_Reg16 *) 0xF106)->bit2
#define DP1H_DP1H_3                   ((T_Reg16 *) 0xF106)->bit3
#define DP1H_DP1H_4                   ((T_Reg16 *) 0xF106)->bit4
#define DP1H_DP1H_5                   ((T_Reg16 *) 0xF106)->bit5
#define DP1H_DP1H_6                   ((T_Reg16 *) 0xF106)->bit6
#define DP1H_DP1H_7                   ((T_Reg16 *) 0xF106)->bit7

// Port 1 Low Direction Control Register
#define DP1L                   (*((uword volatile *) 0xF104))
#define DP1L_DP1L_0                   ((T_Reg16 *) 0xF104)->bit0
#define DP1L_DP1L_1                   ((T_Reg16 *) 0xF104)->bit1
#define DP1L_DP1L_2                   ((T_Reg16 *) 0xF104)->bit2
#define DP1L_DP1L_3                   ((T_Reg16 *) 0xF104)->bit3
#define DP1L_DP1L_4                   ((T_Reg16 *) 0xF104)->bit4
#define DP1L_DP1L_5                   ((T_Reg16 *) 0xF104)->bit5
#define DP1L_DP1L_6                   ((T_Reg16 *) 0xF104)->bit6
#define DP1L_DP1L_7                   ((T_Reg16 *) 0xF104)->bit7

// Port 3 Direction Control Register
#define DP3                    (*((uword volatile *) 0xFFC6))
#define DP3_DP3_10                   ((T_Reg16 *) 0xFFC6)->bit10
#define DP3_DP3_11                   ((T_Reg16 *) 0xFFC6)->bit11
#define DP3_DP3_12                   ((T_Reg16 *) 0xFFC6)->bit12
#define DP3_DP3_13                   ((T_Reg16 *) 0xFFC6)->bit13
#define DP3_DP3_15                   ((T_Reg16 *) 0xFFC6)->bit15
#define DP3_DP3_4                    ((T_Reg16 *) 0xFFC6)->bit4
#define DP3_DP3_6                    ((T_Reg16 *) 0xFFC6)->bit6
#define DP3_DP3_8                    ((T_Reg16 *) 0xFFC6)->bit8
#define DP3_DP3_9                    ((T_Reg16 *) 0xFFC6)->bit9

// Port 4 Direction Control Register
#define DP4                    (*((uword volatile *) 0xFFCA))
#define DP4_DP4_0                    ((T_Reg16 *) 0xFFCA)->bit0
#define DP4_DP4_1                    ((T_Reg16 *) 0xFFCA)->bit1
#define DP4_DP4_2                    ((T_Reg16 *) 0xFFCA)->bit2
#define DP4_DP4_3                    ((T_Reg16 *) 0xFFCA)->bit3
#define DP4_DP4_5                    ((T_Reg16 *) 0xFFCA)->bit5
#define DP4_DP4_6                    ((T_Reg16 *) 0xFFCA)->bit6

// Port 8 Direction Control Register
#define DP8                    (*((uword volatile *) 0xFFD6))
#define DP8_DP8_0                    ((T_Reg16 *) 0xFFD6)->bit0
#define DP8_DP8_1                    ((T_Reg16 *) 0xFFD6)->bit1
#define DP8_DP8_2                    ((T_Reg16 *) 0xFFD6)->bit2
#define DP8_DP8_3                    ((T_Reg16 *) 0xFFD6)->bit3

// CPU Data Page Pointer 0 Register (10 bits)
#define DPP0                   (*((uword volatile *) 0xFE00))

// CPU Data Page Pointer 1 Register (10 bits)
#define DPP1                   (*((uword volatile *) 0xFE02))

// CPU Data Page Pointer 2 Register (10 bits)
#define DPP2                   (*((uword volatile *) 0xFE04))

// CPU Data Page Pointer 3 Register (10 bits)
#define DPP3                   (*((uword volatile *) 0xFE06))

// External Interrupt Control Register
#define EXICON                 (*((uword volatile *) 0xF1C0))

// External Interrupt Select Register
#define EXISEL                 (*((uword volatile *) 0xF1DA))

// Frequency Output Control Register
#define FOCON                  (*((uword volatile *) 0xFFAA))
#define FOCON_FOEN                     ((T_Reg16 *) 0xFFAA)->bit15
#define FOCON_FOSS                     ((T_Reg16 *) 0xFFAA)->bit14
#define FOCON_FOTL                     ((T_Reg16 *) 0xFFAA)->bit6

// Identifier
#define IDCHIP                 (*((uword volatile *) 0xF07C))

// Identifier
#define IDMANUF                (*((uword volatile *) 0xF07E))

// Identifier
#define IDMEM                  (*((uword volatile *) 0xF07A))

// Identifier
#define IDMEM2                 (*((uword volatile *) 0xF076))

// Identifier
#define IDPROG                 (*((uword volatile *) 0xF078))

// Interrupt Sub Node Control Register
#define ISNC                   (*((uword volatile *) 0xF1DE))
#define ISNC_PLLIE                    ((T_Reg16 *) 0xF1DE)->bit3
#define ISNC_PLLIR                    ((T_Reg16 *) 0xF1DE)->bit2
#define ISNC_RTCIE                    ((T_Reg16 *) 0xF1DE)->bit1
#define ISNC_RTCIR                    ((T_Reg16 *) 0xF1DE)->bit0

// CPU Multiply Divide Control Register
#define MDC                    (*((uword volatile *) 0xFF0E))
#define MDC_MDRIU                    ((T_Reg16 *) 0xFF0E)->bit4

// CPU Multiply Divide Register - High Word
#define MDH                    (*((uword volatile *) 0xFE0C))

// CPU Multiply Divide Register - Low Word
#define MDL                    (*((uword volatile *) 0xFE0E))

// Port 3 Open Drain Control Register
#define ODP3                   (*((uword volatile *) 0xF1C6))
#define ODP3_ODP3_10                  ((T_Reg16 *) 0xF1C6)->bit10
#define ODP3_ODP3_11                  ((T_Reg16 *) 0xF1C6)->bit11
#define ODP3_ODP3_13                  ((T_Reg16 *) 0xF1C6)->bit13
#define ODP3_ODP3_4                   ((T_Reg16 *) 0xF1C6)->bit4
#define ODP3_ODP3_6                   ((T_Reg16 *) 0xF1C6)->bit6
#define ODP3_ODP3_8                   ((T_Reg16 *) 0xF1C6)->bit8
#define ODP3_ODP3_9                   ((T_Reg16 *) 0xF1C6)->bit9

// Port 4 Open Drain Control Register
#define ODP4                   (*((uword volatile *) 0xF1CA))
#define ODP4_ODP4_0                   ((T_Reg16 *) 0xF1CA)->bit0
#define ODP4_ODP4_1                   ((T_Reg16 *) 0xF1CA)->bit1
#define ODP4_ODP4_2                   ((T_Reg16 *) 0xF1CA)->bit2
#define ODP4_ODP4_3                   ((T_Reg16 *) 0xF1CA)->bit3
#define ODP4_ODP4_5                   ((T_Reg16 *) 0xF1CA)->bit5
#define ODP4_ODP4_6                   ((T_Reg16 *) 0xF1CA)->bit6

// Port 8 Open Drain Control Register
#define ODP8                   (*((uword volatile *) 0xF1D6))
#define ODP8_ODP8_0                   ((T_Reg16 *) 0xF1D6)->bit0
#define ODP8_ODP8_1                   ((T_Reg16 *) 0xF1D6)->bit1
#define ODP8_ODP8_2                   ((T_Reg16 *) 0xF1D6)->bit2
#define ODP8_ODP8_3                   ((T_Reg16 *) 0xF1D6)->bit3

// Constant Value 1's Register
#define ONES                   (*((uword volatile *) 0xFF1E))

// Port 0 High Register (Upper half)
#define P0H                    (*((uword volatile *) 0xFF02))
#define P0H_P0H_0                    ((T_Reg16 *) 0xFF02)->bit0
#define P0H_P0H_1                    ((T_Reg16 *) 0xFF02)->bit1
#define P0H_P0H_2                    ((T_Reg16 *) 0xFF02)->bit2
#define P0H_P0H_3                    ((T_Reg16 *) 0xFF02)->bit3
#define P0H_P0H_4                    ((T_Reg16 *) 0xFF02)->bit4
#define P0H_P0H_5                    ((T_Reg16 *) 0xFF02)->bit5
#define P0H_P0H_6                    ((T_Reg16 *) 0xFF02)->bit6
#define P0H_P0H_7                    ((T_Reg16 *) 0xFF02)->bit7

// Port 0 Low Register (Lower half)
#define P0L                    (*((uword volatile *) 0xFF00))
#define P0L_P0L_0                    ((T_Reg16 *) 0xFF00)->bit0
#define P0L_P0L_1                    ((T_Reg16 *) 0xFF00)->bit1
#define P0L_P0L_2                    ((T_Reg16 *) 0xFF00)->bit2
#define P0L_P0L_3                    ((T_Reg16 *) 0xFF00)->bit3
#define P0L_P0L_4                    ((T_Reg16 *) 0xFF00)->bit4
#define P0L_P0L_5                    ((T_Reg16 *) 0xFF00)->bit5
#define P0L_P0L_6                    ((T_Reg16 *) 0xFF00)->bit6
#define P0L_P0L_7                    ((T_Reg16 *) 0xFF00)->bit7

// Port 1 High Register (Upper half)
#define P1H                    (*((uword volatile *) 0xFF06))
#define P1H_P1H_0                    ((T_Reg16 *) 0xFF06)->bit0
#define P1H_P1H_1                    ((T_Reg16 *) 0xFF06)->bit1
#define P1H_P1H_2                    ((T_Reg16 *) 0xFF06)->bit2
#define P1H_P1H_3                    ((T_Reg16 *) 0xFF06)->bit3
#define P1H_P1H_4                    ((T_Reg16 *) 0xFF06)->bit4
#define P1H_P1H_5                    ((T_Reg16 *) 0xFF06)->bit5
#define P1H_P1H_6                    ((T_Reg16 *) 0xFF06)->bit6
#define P1H_P1H_7                    ((T_Reg16 *) 0xFF06)->bit7

// Port 1 Low Register (Lower half)
#define P1L                    (*((uword volatile *) 0xFF04))
#define P1L_P1L_0                    ((T_Reg16 *) 0xFF04)->bit0
#define P1L_P1L_1                    ((T_Reg16 *) 0xFF04)->bit1
#define P1L_P1L_2                    ((T_Reg16 *) 0xFF04)->bit2
#define P1L_P1L_3                    ((T_Reg16 *) 0xFF04)->bit3
#define P1L_P1L_4                    ((T_Reg16 *) 0xFF04)->bit4
#define P1L_P1L_5                    ((T_Reg16 *) 0xFF04)->bit5
#define P1L_P1L_6                    ((T_Reg16 *) 0xFF04)->bit6
#define P1L_P1L_7                    ((T_Reg16 *) 0xFF04)->bit7

// Port 3 Register
#define P3                     (*((uword volatile *) 0xFFC4))
#define P3_P3_10                    ((T_Reg16 *) 0xFFC4)->bit10
#define P3_P3_11                    ((T_Reg16 *) 0xFFC4)->bit11
#define P3_P3_12                    ((T_Reg16 *) 0xFFC4)->bit12
#define P3_P3_13                    ((T_Reg16 *) 0xFFC4)->bit13
#define P3_P3_15                    ((T_Reg16 *) 0xFFC4)->bit15
#define P3_P3_4                     ((T_Reg16 *) 0xFFC4)->bit4
#define P3_P3_6                     ((T_Reg16 *) 0xFFC4)->bit6
#define P3_P3_8                     ((T_Reg16 *) 0xFFC4)->bit8
#define P3_P3_9                     ((T_Reg16 *) 0xFFC4)->bit9

// Port 4 Register (8 bits)
#define P4                     (*((uword volatile *) 0xFFC8))
#define P4_P4_0                     ((T_Reg16 *) 0xFFC8)->bit0
#define P4_P4_1                     ((T_Reg16 *) 0xFFC8)->bit1
#define P4_P4_2                     ((T_Reg16 *) 0xFFC8)->bit2
#define P4_P4_3                     ((T_Reg16 *) 0xFFC8)->bit3
#define P4_P4_5                     ((T_Reg16 *) 0xFFC8)->bit5
#define P4_P4_6                     ((T_Reg16 *) 0xFFC8)->bit6

// Port 5 Register (16 bits)
#define P5                     (*((uword volatile *) 0xFFA2))
#define P5_P5_0                     ((T_Reg16 *) 0xFFA2)->bit0
#define P5_P5_1                     ((T_Reg16 *) 0xFFA2)->bit1
#define P5_P5_2                     ((T_Reg16 *) 0xFFA2)->bit2
#define P5_P5_3                     ((T_Reg16 *) 0xFFA2)->bit3
#define P5_P5_4                     ((T_Reg16 *) 0xFFA2)->bit4
#define P5_P5_5                     ((T_Reg16 *) 0xFFA2)->bit5
#define P5_P5_6                     ((T_Reg16 *) 0xFFA2)->bit6
#define P5_P5_7                     ((T_Reg16 *) 0xFFA2)->bit7

// Port 5 Digital Input Disable Register
#define P5DIDIS                (*((uword volatile *) 0xFFA4))
#define P5DIDIS_P5D_0                    ((T_Reg16 *) 0xFFA4)->bit0
#define P5DIDIS_P5D_1                    ((T_Reg16 *) 0xFFA4)->bit1
#define P5DIDIS_P5D_2                    ((T_Reg16 *) 0xFFA4)->bit2
#define P5DIDIS_P5D_3                    ((T_Reg16 *) 0xFFA4)->bit3
#define P5DIDIS_P5D_4                    ((T_Reg16 *) 0xFFA4)->bit4
#define P5DIDIS_P5D_5                    ((T_Reg16 *) 0xFFA4)->bit5
#define P5DIDIS_P5D_6                    ((T_Reg16 *) 0xFFA4)->bit6
#define P5DIDIS_P5D_7                    ((T_Reg16 *) 0xFFA4)->bit7

// Port 8 Register (8 bits)
#define P8                     (*((uword volatile *) 0xFFD4))
#define P8_P8_0                     ((T_Reg16 *) 0xFFD4)->bit0
#define P8_P8_1                     ((T_Reg16 *) 0xFFD4)->bit1
#define P8_P8_2                     ((T_Reg16 *) 0xFFD4)->bit2
#define P8_P8_3                     ((T_Reg16 *) 0xFFD4)->bit3

// Port Driver Control Register
#define PDCR                   (*((uword volatile *) 0xF0AA))
#define PDCR_BIPEC                    ((T_Reg16 *) 0xF0AA)->bit0
#define PDCR_NBPEC                    ((T_Reg16 *) 0xF0AA)->bit4

// PEC Channel 0 Control Register
#define PECC0                  (*((uword volatile *) 0xFEC0))

// PEC Channel 1 Control Register
#define PECC1                  (*((uword volatile *) 0xFEC2))

// PEC Channel 2 Control Register
#define PECC2                  (*((uword volatile *) 0xFEC4))

// PEC Channel 3 Control Register
#define PECC3                  (*((uword volatile *) 0xFEC6))

// PEC Channel 4 Control Register
#define PECC4                  (*((uword volatile *) 0xFEC8))

// PEC Channel 5 Control Register
#define PECC5                  (*((uword volatile *) 0xFECA))

// PEC Channel 6 Control Register
#define PECC6                  (*((uword volatile *) 0xFECC))

// PEC Channel 7 Control Register
#define PECC7                  (*((uword volatile *) 0xFECE))

// Port Input Threshold Control Register
#define PICON                  (*((uword volatile *) 0xF1C4))
#define PICON_P3HIN                    ((T_Reg16 *) 0xF1C4)->bit3
#define PICON_P3LIN                    ((T_Reg16 *) 0xF1C4)->bit2
#define PICON_P4LIN                    ((T_Reg16 *) 0xF1C4)->bit4
#define PICON_P8LIN                    ((T_Reg16 *) 0xF1C4)->bit7

// Port P0H Output Control Register
#define POCON0H                (*((uword volatile *) 0xF082))

// Port P0L Output Control Register
#define POCON0L                (*((uword volatile *) 0xF080))

// Port P1H Output Control Register
#define POCON1H                (*((uword volatile *) 0xF086))

// Port P1L Output Control Register
#define POCON1L                (*((uword volatile *) 0xF084))

// Port P3 Output Control Register
#define POCON3                 (*((uword volatile *) 0xF08A))

// Port P4 Output Control Register
#define POCON4                 (*((uword volatile *) 0xF08C))

// Port P8 Output Control Register
#define POCON8                 (*((uword volatile *) 0xF092))

// CPU Program Status Word
#define PSW                    (*((uword volatile *) 0xFF10))
#define PSW_C                        ((T_Reg16 *) 0xFF10)->bit1
#define PSW_E                        ((T_Reg16 *) 0xFF10)->bit4
#define PSW_IEN                      ((T_Reg16 *) 0xFF10)->bit11
#define PSW_MULIP                    ((T_Reg16 *) 0xFF10)->bit5
#define PSW_N                        ((T_Reg16 *) 0xFF10)->bit0
#define PSW_USR0                     ((T_Reg16 *) 0xFF10)->bit6
#define PSW_V                        ((T_Reg16 *) 0xFF10)->bit2
#define PSW_Z                        ((T_Reg16 *) 0xFF10)->bit3

// Port Temperature Compensation Register
#define PTCR                   (*((uword volatile *) 0xF0AE))
#define PTCR_TCE                      ((T_Reg16 *) 0xF0AE)->bit3
#define PTCR_TCS                      ((T_Reg16 *) 0xF0AE)->bit7

// System Startup Configuration Register
#define RP0H                   (*((uword volatile *) 0xF108))
#define RP0H_WRC                      ((T_Reg16 *) 0xF108)->bit0

// Reset Control Register
#define RSTCON                 (*((uword volatile *) 0xF1E0))
#define RSTCON_SUE                      ((T_Reg16 *) 0xF1E0)->bit8

// RTC Timer Register High
#define RTCH                   (*((uword volatile *) 0xF0D6))

// RTC Timer Register Low
#define RTCL                   (*((uword volatile *) 0xF0D4))

// Serial Channel 0 Baud Rate Timer Reload Register
#define S0BG                   (*((uword volatile *) 0xFEB4))

// Serial Channel 0 Control Register
#define S0CON                  (*((uword volatile *) 0xFFB0))
#define S0CON_S0BRS                    ((T_Reg16 *) 0xFFB0)->bit13
#define S0CON_S0FE                     ((T_Reg16 *) 0xFFB0)->bit9
#define S0CON_S0FEN                    ((T_Reg16 *) 0xFFB0)->bit6
#define S0CON_S0LB                     ((T_Reg16 *) 0xFFB0)->bit14
#define S0CON_S0ODD                    ((T_Reg16 *) 0xFFB0)->bit12
#define S0CON_S0OE                     ((T_Reg16 *) 0xFFB0)->bit10
#define S0CON_S0OEN                    ((T_Reg16 *) 0xFFB0)->bit7
#define S0CON_S0PE                     ((T_Reg16 *) 0xFFB0)->bit8
#define S0CON_S0PEN                    ((T_Reg16 *) 0xFFB0)->bit5
#define S0CON_S0R                      ((T_Reg16 *) 0xFFB0)->bit15
#define S0CON_S0REN                    ((T_Reg16 *) 0xFFB0)->bit4
#define S0CON_S0STP                    ((T_Reg16 *) 0xFFB0)->bit3

// Serial Channel 0 Error Interrupt Control Register
#define S0EIC                  (*((uword volatile *) 0xFF70))
#define S0EIC_S0EIE                    ((T_Reg16 *) 0xFF70)->bit6
#define S0EIC_S0EIR                    ((T_Reg16 *) 0xFF70)->bit7

// Serial Channel 0 Receive Buffer Register
#define S0RBUF                 (*((uword volatile *) 0xFEB2))

// Serial Channel 0 Receive Interrupt Control Register
#define S0RIC                  (*((uword volatile *) 0xFF6E))
#define S0RIC_S0RIE                    ((T_Reg16 *) 0xFF6E)->bit6
#define S0RIC_S0RIR                    ((T_Reg16 *) 0xFF6E)->bit7

// Serial Channel 0 Transmit Buffer Interrupt Control Register
#define S0TBIC                 (*((uword volatile *) 0xF19C))
#define S0TBIC_S0TBIE                   ((T_Reg16 *) 0xF19C)->bit6
#define S0TBIC_S0TBIR                   ((T_Reg16 *) 0xF19C)->bit7

// Serial Channel 0 Transmit Buffer Register
#define S0TBUF                 (*((uword volatile *) 0xFEB0))

// Serial Channel 0 Transmit Interrupt Control Register
#define S0TIC                  (*((uword volatile *) 0xFF6C))
#define S0TIC_S0TIE                    ((T_Reg16 *) 0xFF6C)->bit6
#define S0TIC_S0TIR                    ((T_Reg16 *) 0xFF6C)->bit7

// CPU System Stack Pointer Register
#define SP                     (*((uword volatile *) 0xFE12))

// SSC Baudrate Register
#define SSCBR                  (*((uword volatile *) 0xF0B4))

// SSC Control Register
#define SSCCON                 (*((uword volatile *) 0xFFB2))
#define SSCCON_SSCAREN_BSY              ((T_Reg16 *) 0xFFB2)->bit12
#define SSCCON_SSCBEN_BE                ((T_Reg16 *) 0xFFB2)->bit11
#define SSCCON_SSCEN                    ((T_Reg16 *) 0xFFB2)->bit15
#define SSCCON_SSCHB                    ((T_Reg16 *) 0xFFB2)->bit4
#define SSCCON_SSCMS                    ((T_Reg16 *) 0xFFB2)->bit14
#define SSCCON_SSCPEN_PE                ((T_Reg16 *) 0xFFB2)->bit10
#define SSCCON_SSCPH                    ((T_Reg16 *) 0xFFB2)->bit5
#define SSCCON_SSCPO                    ((T_Reg16 *) 0xFFB2)->bit6
#define SSCCON_SSCREN_RE                ((T_Reg16 *) 0xFFB2)->bit9
#define SSCCON_SSCTEN_TE                ((T_Reg16 *) 0xFFB2)->bit8

// SSC Error Interrupt Control Register
#define SSCEIC                 (*((uword volatile *) 0xFF76))
#define SSCEIC_SSCEIE                   ((T_Reg16 *) 0xFF76)->bit6
#define SSCEIC_SSCEIR                   ((T_Reg16 *) 0xFF76)->bit7

// SSC Receive Buffer
#define SSCRB                  (*((uword volatile *) 0xF0B2))

// SSC Receive Interrupt Control Register
#define SSCRIC                 (*((uword volatile *) 0xFF74))
#define SSCRIC_SSCRIE                   ((T_Reg16 *) 0xFF74)->bit6
#define SSCRIC_SSCRIR                   ((T_Reg16 *) 0xFF74)->bit7

// SSC Transmit Buffer
#define SSCTB                  (*((uword volatile *) 0xF0B0))

// SSC Transmit Interrupt Control Register
#define SSCTIC                 (*((uword volatile *) 0xFF72))
#define SSCTIC_SSCTIE                   ((T_Reg16 *) 0xFF72)->bit6
#define SSCTIC_SSCTIR                   ((T_Reg16 *) 0xFF72)->bit7

// CPU Stack Overflow Pointer Register
#define STKOV                  (*((uword volatile *) 0xFE14))

// CPU Stack Underflow Pointer Register
#define STKUN                  (*((uword volatile *) 0xFE16))

// CPU System Configuration Register
#define SYSCON                 (*((uword volatile *) 0xFF12))
#define SYSCON_BDRSTEN                  ((T_Reg16 *) 0xFF12)->bit3
#define SYSCON_BYTDIS                   ((T_Reg16 *) 0xFF12)->bit9
#define SYSCON_CLKEN                    ((T_Reg16 *) 0xFF12)->bit8
#define SYSCON_CSCFG                    ((T_Reg16 *) 0xFF12)->bit6
#define SYSCON_OWDDIS                   ((T_Reg16 *) 0xFF12)->bit4
#define SYSCON_ROMEN                    ((T_Reg16 *) 0xFF12)->bit10
#define SYSCON_ROMS1                    ((T_Reg16 *) 0xFF12)->bit12
#define SYSCON_SGTDIS                   ((T_Reg16 *) 0xFF12)->bit11
#define SYSCON_VISIBLE                  ((T_Reg16 *) 0xFF12)->bit1
#define SYSCON_WRCFG                    ((T_Reg16 *) 0xFF12)->bit7
#define SYSCON_XPEN                     ((T_Reg16 *) 0xFF12)->bit2

// CPU System Configuration Register 1
#define SYSCON1                (*((uword volatile *) 0xF1DC))

// CPU System Configuration Register 2
#define SYSCON2                (*((uword volatile *) 0xF1D0))
#define SYSCON2_2                        ((T_Reg16 *) 0xF1D0)->bit2
#define SYSCON2_CLKLOCK                  ((T_Reg16 *) 0xF1D0)->bit15
#define SYSCON2_RCS                      ((T_Reg16 *) 0xF1D0)->bit6
#define SYSCON2_SCS                      ((T_Reg16 *) 0xF1D0)->bit7

// CPU System Configuration Register 3
#define SYSCON3                (*((uword volatile *) 0xF1D4))
#define SYSCON3_ADCDIS                   ((T_Reg16 *) 0xF1D4)->bit0
#define SYSCON3_ASC0DIS                  ((T_Reg16 *) 0xF1D4)->bit1
#define SYSCON3_CAN1DIS                  ((T_Reg16 *) 0xF1D4)->bit13
#define SYSCON3_CC2DIS                   ((T_Reg16 *) 0xF1D4)->bit7
#define SYSCON3_CC6DIS                   ((T_Reg16 *) 0xF1D4)->bit8
#define SYSCON3_DFMDIS                   ((T_Reg16 *) 0xF1D4)->bit4
#define SYSCON3_GPTDIS                   ((T_Reg16 *) 0xF1D4)->bit3
#define SYSCON3_PCDDIS                   ((T_Reg16 *) 0xF1D4)->bit15
#define SYSCON3_PFMDIS                   ((T_Reg16 *) 0xF1D4)->bit5
#define SYSCON3_SSCDIS                   ((T_Reg16 *) 0xF1D4)->bit2

// GPT1 Timer 2 Interrupt Control Register
#define T12IC                  (*((uword volatile *) 0xF190))
#define T12IC_T12IE                    ((T_Reg16 *) 0xF190)->bit6
#define T12IC_T12IR                    ((T_Reg16 *) 0xF190)->bit7

// CAPCOM 6 Timer 12 Offset Register
#define T12OF                  (*((uword volatile *) 0xF034))

// CAPCOM 6 Timer 12 Period Register
#define T12P                   (*((uword volatile *) 0xF030))

// GPT1 Timer 2 Interrupt Control Register
#define T13IC                  (*((uword volatile *) 0xF198))
#define T13IC_T13IE                    ((T_Reg16 *) 0xF198)->bit6
#define T13IC_T13IR                    ((T_Reg16 *) 0xF198)->bit7

// CAPCOM 6 Timer 13 Period Register
#define T13P                   (*((uword volatile *) 0xF032))

// RTC Timer 14 Register
#define T14                    (*((uword volatile *) 0xF0D2))

// RTC Timer 14 Reload Register
#define T14REL                 (*((uword volatile *) 0xF0D0))

// GPT1 Timer 2 Register
#define T2                     (*((uword volatile *) 0xFE40))

// GPT1 Timer 2 Control Register
#define T2CON                  (*((uword volatile *) 0xFF40))
#define T2CON_T2R                      ((T_Reg16 *) 0xFF40)->bit6
#define T2CON_T2UD                     ((T_Reg16 *) 0xFF40)->bit7
#define T2CON_T2UDE                    ((T_Reg16 *) 0xFF40)->bit8

// GPT1 Timer 2 Interrupt Control Register
#define T2IC                   (*((uword volatile *) 0xFF60))
#define T2IC_T2IE                     ((T_Reg16 *) 0xFF60)->bit6
#define T2IC_T2IR                     ((T_Reg16 *) 0xFF60)->bit7

// GPT1 Timer 3 Register
#define T3                     (*((uword volatile *) 0xFE42))

// GPT1 Timer 3 Control Register
#define T3CON                  (*((uword volatile *) 0xFF42))
#define T3CON_T3OTL                    ((T_Reg16 *) 0xFF42)->bit10
#define T3CON_T3R                      ((T_Reg16 *) 0xFF42)->bit6
#define T3CON_T3UD                     ((T_Reg16 *) 0xFF42)->bit7
#define T3CON_T3UDE                    ((T_Reg16 *) 0xFF42)->bit8

// GPT1 Timer 3 Interrupt Control Register
#define T3IC                   (*((uword volatile *) 0xFF62))
#define T3IC_T3IE                     ((T_Reg16 *) 0xFF62)->bit6
#define T3IC_T3IR                     ((T_Reg16 *) 0xFF62)->bit7

// GPT1 Timer 4 Register
#define T4                     (*((uword volatile *) 0xFE44))

// GPT1 Timer 4 Control Register
#define T4CON                  (*((uword volatile *) 0xFF44))
#define T4CON_T4R                      ((T_Reg16 *) 0xFF44)->bit6
#define T4CON_T4UD                     ((T_Reg16 *) 0xFF44)->bit7
#define T4CON_T4UDE                    ((T_Reg16 *) 0xFF44)->bit8

// GPT1 Timer 4 Interrupt Control Register
#define T4IC                   (*((uword volatile *) 0xFF64))
#define T4IC_T4IE                     ((T_Reg16 *) 0xFF64)->bit6
#define T4IC_T4IR                     ((T_Reg16 *) 0xFF64)->bit7

// CAPCOM Timer 7 Register
#define T7                     (*((uword volatile *) 0xF050))

// CAPCOM Timer 7 and Timer 8 Control Register
#define T78CON                 (*((uword volatile *) 0xFF20))
#define T78CON_T7M                      ((T_Reg16 *) 0xFF20)->bit3
#define T78CON_T7R                      ((T_Reg16 *) 0xFF20)->bit6
#define T78CON_T8M                      ((T_Reg16 *) 0xFF20)->bit11
#define T78CON_T8R                      ((T_Reg16 *) 0xFF20)->bit14

// CAPCOM Timer 7 Interrupt Control Register
#define T7IC                   (*((uword volatile *) 0xF17A))
#define T7IC_T7IE                     ((T_Reg16 *) 0xF17A)->bit6
#define T7IC_T7IR                     ((T_Reg16 *) 0xF17A)->bit7

// CAPCOM Timer 7 Reload Register
#define T7REL                  (*((uword volatile *) 0xF054))

// CAPCOM Timer 8 Register
#define T8                     (*((uword volatile *) 0xF052))

// CAPCOM Timer 8 Interrupt Control Register
#define T8IC                   (*((uword volatile *) 0xF17C))
#define T8IC_T8IE                     ((T_Reg16 *) 0xF17C)->bit6
#define T8IC_T8IR                     ((T_Reg16 *) 0xF17C)->bit7

// CAPCOM Timer 8 Reload Register
#define T8REL                  (*((uword volatile *) 0xF056))

// Trap Flag Register
#define TFR                    (*((uword volatile *) 0xFFAC))
#define TFR_ILLBUS                   ((T_Reg16 *) 0xFFAC)->bit0
#define TFR_ILLINA                   ((T_Reg16 *) 0xFFAC)->bit1
#define TFR_ILLOPA                   ((T_Reg16 *) 0xFFAC)->bit2
#define TFR_NMI                      ((T_Reg16 *) 0xFFAC)->bit15
#define TFR_PRTFLT                   ((T_Reg16 *) 0xFFAC)->bit3
#define TFR_STKOF                    ((T_Reg16 *) 0xFFAC)->bit14
#define TFR_STKUF                    ((T_Reg16 *) 0xFFAC)->bit13
#define TFR_UNDOPC                   ((T_Reg16 *) 0xFFAC)->bit7

// CAPCOM 6 Trap Enable Control Register
#define TRCON                  (*((uword volatile *) 0xFF34))
#define TRCON_TREN0                    ((T_Reg16 *) 0xFF34)->bit8
#define TRCON_TREN1                    ((T_Reg16 *) 0xFF34)->bit9
#define TRCON_TREN2                    ((T_Reg16 *) 0xFF34)->bit10
#define TRCON_TREN3                    ((T_Reg16 *) 0xFF34)->bit11
#define TRCON_TREN4                    ((T_Reg16 *) 0xFF34)->bit12
#define TRCON_TREN5                    ((T_Reg16 *) 0xFF34)->bit13
#define TRCON_TRF                      ((T_Reg16 *) 0xFF34)->bit14
#define TRCON_TRPEN                    ((T_Reg16 *) 0xFF34)->bit15
#define TRCON_TT12DIS                  ((T_Reg16 *) 0xFF34)->bit0
#define TRCON_TT13DIS                  ((T_Reg16 *) 0xFF34)->bit1

// Watchdog Timer Register
#define WDT                    (*((uword volatile *) 0xFEAE))

// Watchdog Timer Control Register
#define WDTCON                 (*((uword volatile *) 0xFFAE))
#define WDTCON_LHWR                     ((T_Reg16 *) 0xFFAE)->bit4
#define WDTCON_SHWR                     ((T_Reg16 *) 0xFFAE)->bit3
#define WDTCON_SWR                      ((T_Reg16 *) 0xFFAE)->bit2
#define WDTCON_WDTIN                    ((T_Reg16 *) 0xFFAE)->bit0
#define WDTCON_WDTPRE                   ((T_Reg16 *) 0xFFAE)->bit7
#define WDTCON_WDTR                     ((T_Reg16 *) 0xFFAE)->bit1

// CAN1 Module Interrupt Control Register
#define XP0IC                  (*((uword volatile *) 0xF186))
#define XP0IC_XP0IE                    ((T_Reg16 *) 0xF186)->bit6
#define XP0IC_XP0IR                    ((T_Reg16 *) 0xF186)->bit7

// Flash Termination Interrupt Control Register
#define XP1IC                  (*((uword volatile *) 0xF18E))
#define XP1IC_XP1IE                    ((T_Reg16 *) 0xF18E)->bit6
#define XP1IC_XP1IR                    ((T_Reg16 *) 0xF18E)->bit7

// RTC/PLL Interrupt Control Register
#define XP3IC                  (*((uword volatile *) 0xF19E))
#define XP3IC_XP3IE                    ((T_Reg16 *) 0xF19E)->bit6
#define XP3IC_XP3IR                    ((T_Reg16 *) 0xF19E)->bit7

// Constant Value 0's Register
#define ZEROS                  (*((uword volatile *) 0xFF1C))


//****************************************************************************
// @Typedefs
//****************************************************************************

typedef unsigned char  ubyte;    // 1 byte unsigned; prefix: ub 
typedef signed char    sbyte;    // 1 byte signed;   prefix: sb 
typedef unsigned int   uword;    // 2 byte unsigned; prefix: uw 
typedef signed int     sword;    // 2 byte signed;   prefix: sw 
typedef unsigned long  ulong;    // 4 byte unsigned; prefix: ul 
typedef signed long    slong;    // 4 byte signed;   prefix: sl 

typedef volatile struct
{
  unsigned int    bit0      : 1;
  unsigned int    bit1      : 1;
  unsigned int    bit2      : 1;
  unsigned int    bit3      : 1;
  unsigned int    bit4      : 1;
  unsigned int    bit5      : 1;
  unsigned int    bit6      : 1;
  unsigned int    bit7      : 1;
  unsigned int    bit8      : 1;
  unsigned int    bit9      : 1;
  unsigned int    bit10     : 1;
  unsigned int    bit11     : 1;
  unsigned int    bit12     : 1;
  unsigned int    bit13     : 1;
  unsigned int    bit14     : 1;
  unsigned int    bit15     : 1;
}  T_Reg16;

// USER CODE BEGIN (MAIN_Header,5)

typedef ubyte bool;	  // 1 byte unsigned
typedef ulong udword; // 4 byte unsigned
typedef slong sdword; // 4 byte   signed

typedef uword ureg;   // 2 byte	unsigned
typedef sword sreg;	  // 2 byte   signed

typedef unsigned char uchar; // 1 byte unsigned
typedef signed   char schar; // 1 byte   signed

#define MIN(a,b)	 ( ((a) < (b)) 	? (a) : (b) )
#define MAX(a,b)	 ( ((a) > (b)) 	? (a) : (b) )
#define ABS(a)		 ( ((a) < 0)	?  -1*(a) : (a) )
#define CLAMP(a,b,c) ( ((a) < (b)) 	? (b) : ((a) > (c) ? (c) : (a)) )
#define SIGN(a)		 (((bool)((a) >> ((sizeof(a)*8)-1)) & 0x01 ))

// USER CODE END


//****************************************************************************
// @Imported Global Variables
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,6)

// USER CODE END


//****************************************************************************
// @Global Variables
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,7)

// USER CODE END


//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************



// USER CODE BEGIN (MAIN_Header,8)

// USER CODE END


//****************************************************************************
// @Interrupt Vectors
//****************************************************************************

// USER CODE BEGIN (MAIN_Header,9)

// USER CODE END


//****************************************************************************
// @Project Includes
//****************************************************************************

#include <Intrins.h>

#include  "IO.H"
#include  "ASC0.H"
#include  "GPT1.H"
#include  "WDT.H"
#include  "RTC.H"
#include  "ADC.H"
#include  "CC2.H"
#include  "CC6.H"
#include  "CAN1.H"

// USER CODE BEGIN (MAIN_Header,10)

// USER CODE END


#endif  // ifndef _MAIN_H_
