// IMPORTANT : The following code was done for the Roboduino board.
//The Roboduino is ATmega168 based so the hardware is the same. Only
//difference is that I used pin numbers instead of port numbers. To see
//what pin //number correlates to what port number use this chart -
//http://www.arduino.cc/en/Hacking/PinMapping168 .

// http://www.CuriousInventor.com/kits/roboduino

//For example  pwmInit56 is for pins 5 and 6 which translates into PD5 and PD6

//There are two channels of PWM per timer, in the comments I wrote down which pin is which

//thanks to mbateman for his PWM on ATmega640 code which I modified

//code by airman00 - Visit narobo.com

#define sbi(p,n)        (p) |= (1<<(n))
#define cbi(p,n)        (p) &= ~(1<<(n))
#define outb(p,n)       (p) = (n)
#define   inb(p)          (p)

void pwmInit56(void)  {    // Pin 6 is channel A and Pin 5 is channel B
   /***************************************************/
   /* Initialize timer0 for PWM  */
   /***************************************************/
   // 00   A off, toggle on compare
   // 00   B off, toggle on compare
   // 00   C channel does not exist
   // 11   Low part of WGM: PWM, Fast PWM
   TCCR0A=0b00000011;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 00   Fast PWM
   // 010   Prescale 8
   TCCR0B=0b00000010;              
   // Clear TCNT
   TCNT0=0x00;  

}

void pwmOff56(void) {
 /**********************************************************/
 /* Return timer and pins to normal use */
 /**********************************************************/
   // 00   A off, pin works as general I/O
   // 00   B off, pin works as general I/O
   // 00   C off, pin works as general I/O
   // 00   Low part of WGM: Normal
   TCCR0A=0x00;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 00   High part of WGM: Normal
   // 000   Prescale None
   TCCR0B=0x00;
   // Clear TCNT
   TCNT0=0x00;
}

void pwmInit311(void)  {  // Pin 11 is Channel A and pin 3 is channel B
   /***************************************************/
   /* Initialize timer0 for PWM  */ 
   /***************************************************/
   // 00   A off, pin works as general I/O
   // 00   B off, pin works as general I/O
   // 00   C channel does not exist
   // 11   Low part of WGM: Fast PWM
   TCCR2A=0b00000011;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 00   Fast PWM
   // 010   Prescale 8
   TCCR2B=0b00000010;              
   // Clear TCNT
   TCNT2=0x00;  
}

void pwmOff311(void) {
 /**********************************************************/
 /* Return timer and pins to normal use */
 /**********************************************************/
   // 00   A off, pin works as general I/O
   // 00   B off, pin works as general I/O
   // 00   C off,pin works as general I/O
   // 00   Low part of WGM: Normal
   TCCR2A=0x00;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 00   High part of WGM: Normal
   // 000   Prescale None
   TCCR2B=0x00;
   // Clear TCNT
   TCNT2=0x00;
}

void pwmInit910(void) {  // pin 9 is channel A and pin 10 is channel B
   /***************************************************/
   /* Initialize timer1 for PWM */
   /***************************************************/
   // 00   A off, pin works as general I/O
   // 00   B off,pin works as general I/O
   // 00   C off,pin works as general I/O
   // 00   Low part of WGM: PWM, phase & freq, ICRn
   TCCR1A=0x00;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 10   High part of WGM: PWM, phase & freq, ICRn
   // 010   Prescale 8
   TCCR1B=0b00010010;
   // Clear TCNT
   TCNT1=0b00000000;

ICR1 = 1000; // every 1000 uS or 1mS
}


void pwmOff910(void) {
   /**********************************************************/
   /* Return timer and pins to normal use */
   /**********************************************************/
   // 00   A off
   // 00   B off
   // 00   C off
   // 00   Low part of WGM: Normal
   TCCR1A=0x00;
   // 0   No input noise cancelling
   // 0   Input edge select does not matter
   // 0   Not used
   // 00   High part of WGM: Normal
   // 000   Prescale None
   TCCR1B=0x00;
   // Clear TCNT
   TCNT1=0x00;
}

void pwmOn9(void) {
   /****************************/
   /* Pin 9 on*/
   /****************************/

//9 A on - set at Bottom, clear at compare match
   sbi(TCCR1A,7);
  cbi(TCCR1A,6);

}

void pwmOff9(void) {
   /********************************/
   /* Pin 9 off */
   /********************************/
   // 00   A off , pin works as general I/O
   cbi(TCCR1A,7);
   cbi(TCCR1A,6);
}

void pwmOn10(void) {
   /****************************/
   /* Pin 10 on */
   /****************************/
   // 10   B on, set at Bottom, clear at compare match
   sbi(TCCR1A,5);
   cbi(TCCR1A,4);
}

void pwmOff10(void) {
   /********************************/
   /* Pin 10 off */
   /********************************/
   // 00   B off pin works as general I/O
   cbi(TCCR1A,5);
   cbi(TCCR1A,4);
}

void pwmOn6(void) {
   /****************************/
  /* Pin 6 on */
   /****************************/
   // 6  A on, set at Bottom, clear at compare match
   sbi(TCCR0A,7);
   cbi(TCCR0A,6);
}

void pwmOff6(void) {
   /********************************/
   /* Pin 6 off */
   /********************************/
   // 00   A off pin works as general I/O
   cbi(TCCR0A,7);
   cbi(TCCR0A,6);
}

void pwmOn5(void) {
   /****************************/
   /*pin 5 on */
   /****************************/
   // 5   B on, set at Bottom, clear at compare match
   sbi(TCCR0A,5);
   cbi(TCCR0A,4);
}

void pwmOff5(void) {
   /********************************/
   /* pin 5 off pin works as general I/O */
   /********************************/
   // 00   B off
   cbi(TCCR0A,5);
   cbi(TCCR0A,4);
}

void pwmOn11(void) {
   /****************************/
   /* Pin 11 on*/
   /****************************/
   // 11   A on, set at Bottom, clear at compare match
   sbi(TCCR2A,7);
   cbi(TCCR2A,6);
}

void pwmOff11(void) {
   /********************************/
   /* Pin 11 off */
   /********************************/
   // 00   A off ,pin works as general I/O
   cbi(TCCR2A,7);
   cbi(TCCR2A,6);
}

void pwmOn3(void) {
   /****************************/
   /*pin 3 on */
   /****************************/
   // 3   B on,set at Bottom, clear at compare match
   sbi(TCCR2A,5);
   cbi(TCCR2A,4);
}

void pwmOff3(void) {
   /********************************/
   /* pin 3 off , pin works as general I/O */
   /***************************/
   // 00   B off
   cbi(TCCR2A,5);
   cbi(TCCR2A,4);
}


//8 bit  val can be up to 255
#define pwmSet6(val)   OCR0A=val     
#define pwmSet5(val)   OCR0B=val     

//8 bit val can be up to 255
#define pwmSet11(val)   OCR2A=val     
#define pwmSet3(val)   OCR2B=val 

// 16 bit  val can be up to 65535
#define pwmSet9(ICR1 * val /255)   OCR1A=val
//void pwmSet9(int val) {
//	OCR1A = ICR1*val/255;
//}
#define pwmSet10(ICR1 * val /255)   OCR1B=val
//void pwmSet10(int val) {
//	OCR1B = ICR1*val/255;
//}
