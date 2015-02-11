#include "main.h"
#include "cn_pwm.h"

void cn_init_pwm(void) {

  // original TMC200 PWM code

  CTCON   = 0x1300;
  //T12P    = 0x3E7;
  T12P    = MAX_PWM;
  T12OF   = 0;
  T13P    = 0x7C;
  CC6MCON = 0x20FF;
  CC6MSEL = 0x222;
  CC60    = 0;
  CC61    = 0;
  CC62    = 0;
  CMP13   = 0;
  CC6MIC  = 0;
  CTCON  |= (1 << 5);
  CTCON	 |= (1 << 13);
  
  CTCON  |= 8;         // start T12
  //CTCON  |= 0x0800;  // start T13

  // END original TMC200 PWM code  

}