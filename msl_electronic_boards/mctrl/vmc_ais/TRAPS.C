/***********************************************************************/
/*  This file is part of the C166 Compiler package                     */
/*  Copyright KEIL ELEKTRONIK GmbH 1996-1999                           */
/***********************************************************************/
/*                                                                     */
/*  TRAPS.C:  TRAP Handler for 166/167 hardware traps                  */
/*                                                                     */
/*  To translate this file use C166 with the following invocation:     */
/*                                                                     */
/*     C166 TRAPS.C                                                    */
/*                                                                     */
/*  To link the modified TRAPS.OBJ file to your application use the    */
/*  following L166 invocation:                                         */
/*                                                                     */
/*     L166 <your object file list>, TRAPS.OBJ <controls>              */
/*                                                                     */
/***********************************************************************/

#include <reg167.h>
#include "system/ais_utils.h" // LED control

#define MON 1

/*
 * Non-Maskable Interrupt
 */
#ifndef MON   // is used by Monitor-166
void NMI_trap (void) interrupt 0x02  {
  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  while (1);        // end-less loop */
}


/*
 * Stack Overflow Interrupt
 */
void STKOF_trap (void) interrupt 0x04  {
  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  while (1);        // end-less loop */
}


/*
 * Stack Underflow Interrupt
 */
void STKUF_trap (void) interrupt 0x06  {
  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  while (1);        // end-less loop 
}


/*
 * Class B Hardware Trap:
 *   ->  Undefined Opcode
 *   ->  Protected Instruction Fault
 *   ->  Illegal Word Operand Access
 *   ->  Illegal Instruction Access
 *   ->  Illegal External Bus Access
 */
void c_b_trap (void) interrupt 0x0A  {
  led_set_green(0); // Green led off
  led_set_red(1);   // Red led on
  /* add your code here */
  while (1);        // end-less loop 
}
#endif
