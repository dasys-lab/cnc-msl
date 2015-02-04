#ifndef __CPLD_h_
#define __CPLD_h_ 1

/* CPLD.SPI_CS bits */
#define SPIP0	0
#define SPIP1	1
#define SPIP2	2
#define SPIP3	3
#define SPIP4	4
#define SPIP5	5
#define SPIP6	6
#define SPIP7	7

/* CPLD.Bank_Reg bits */
/* Bank_0 */
#define B0P1	0
#define B0P2	1
#define B0P3	2
#define B0P4	3
/* Bank_1 */
#define B1P1	4
#define B1P2	5
#define B1P3	6
#define B1P4	7

extern volatile unsigned char *pSPI_CS;
extern volatile unsigned char *pBank_Reg;

#endif /* __CPLD_h_ */
