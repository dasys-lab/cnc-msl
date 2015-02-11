#include "main.h"
#include "io.h"
#include "cn_led.h"

void init_led(void) {

#ifdef VMC
	GREEN_LED_DIR = 1;
	RED_LED_DIR   = 1;
#endif

}

void greenLedOn(void) {
  IO_SetPin(GREEN_LED_PIN);  
}

void greenLedOff(void) {
  IO_ResetPin(GREEN_LED_PIN);
}

void greenLedToggle(void) {
  IO_TogglePin(GREEN_LED_PIN);
}

#ifdef VMC
void redLedOn(void) {
  IO_SetPin(RED_LED_PIN);  
}

void redLedOff(void) {
  IO_ResetPin(RED_LED_PIN);
}

void redLedToggle(void) {
  IO_TogglePin(RED_LED_PIN);
}
#endif