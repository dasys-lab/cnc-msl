#include "main.h"
#include "io.h"
#include "cn_led.h"

void ledOn(void) {
  IO_SetPin(LED_PIN);  
}

void ledOff(void) {
  IO_ResetPin(LED_PIN);
}

void ledToggle(void) {
  IO_TogglePin(LED_PIN);
}