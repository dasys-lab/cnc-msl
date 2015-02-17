#include "utils.h"
#include <fcntl.h>
#ifdef __uClinux__
#include <linux/led.h>
#endif


#ifdef __uClinux__
static int ledFd;

int init_LED( const char *ledDevice )
{
  ledFd = open( ledDevice, O_RDWR );
  
  if(ledFd < 0) return -1;
  
  SwitchLED (0, 1);
  SwitchLED (1, 1);
  SwitchLED (2, 1);

  usleep(100000);
  
  SwitchLED (0, 0);
  SwitchLED (1, 0);
  SwitchLED (2, 0);

  return 0;
}
inline void SwitchLED( unsigned int no, unsigned int state )
{
  LED_CMD_T led;
  
  if(ledFd < 0 || no > 2) return;
  
  led.cmd = LED_CMD_WRITE_SINGLE;
  led.reg = 0;
  led.mask = 0x01<<no;
  led.data = state<<no;

  write( ledFd, &led, sizeof(LED_CMD_T) );
}

void exit_LED(void)
{
  if(ledFd < 0) return;
  
  SwitchLED ( 0, 0 );
  SwitchLED ( 1, 0 );
  SwitchLED ( 2, 0 );
  
  close( ledFd );
  
  ledFd = -1;
}

#endif


void setFdNonBlocking(int sockfd)
{
	int prev_mode;

	if(( prev_mode = fcntl(sockfd, F_GETFL, 0)) != -1)
		fcntl(sockfd, F_SETFL, prev_mode | O_NONBLOCK);

//	printf("Set descriptor %d to nonblock\n", sockfd);

	return;
}
/*
inline unsigned char * u8_to_hex(unsigned char * dest, unsigned value)
{
	unsigned high_nib, low_nib;

	low_nib = (value & 0xf) + 0x30;

	if (low_nib > 0x39) low_nib += 0x27;

	high_nib = ((value >> 4) & 0xf) + 0x30;

	if (high_nib > 0x39) high_nib += 0x27;

	*dest++ = high_nib;
	*dest++ = low_nib;

	return dest;
}

inline unsigned hex_to_u8(unsigned char ** src)
{
	unsigned low_nib, high_nib;

	high_nib = *(*src)++ - 0x30;
	
	if (high_nib > 9){
		high_nib = high_nib | 0x20;
		high_nib -= 0x27;
	}

	low_nib = *(*src)++ - 0x30;

	if (low_nib > 9){
		low_nib = low_nib | 0x20;
		low_nib -= 0x27;
	}

	return (high_nib << 4) + low_nib;
}
*/
