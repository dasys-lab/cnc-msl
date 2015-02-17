#ifndef utils_h
#define utils_h 1


//LED Stuff
#ifdef __uClinux__
    int         init_LED ( const char *ledDevice );
    inline void SwitchLED( unsigned int no, unsigned int state );
    void        exit_LED (void);
#elif _DEBUG_LED_OUTPUT
    #define init_LED(x)    printf("INFO: Opening '%s' led device\n", x)
    #define SwitchLED(x, y) printf("INFO: Switching led no. %d %s\n", x, y?"on":"off")
    #define exit_LED()     printf("INFO: Restoring led settings\n")
#else
    #define init_LED(x)     
    #define SwitchLED(x, y) 
    #define exit_LED()      
#endif

#define LED_PROGRAM_RUNNING 0
#define LED_CONN_STATE      1
#define LED_BUS_OFF         2

#define LED_ON      1
#define LED_OFF     0


/* Macros help using of our buffers */
#define IsBufferFull(x)			(!x.WnR) && (x.iidx == x.oidx)
#define IsBufferEmpty(x)		(x.WnR) && (x.iidx == x.oidx)
#define IsBufferNotEmpty(x)		(!x.WnR) || (x.iidx != x.oidx)
#define IsOob(x)				(x.oob)

void setFdNonBlocking(int sockfd);
//inline unsigned hex_to_u8(unsigned char ** src);
//inline unsigned char * u8_to_hex(unsigned char * dest, unsigned value);

#endif
