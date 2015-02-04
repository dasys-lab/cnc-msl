#ifndef _APP_H_
#define _APP_H_

/*****************************************************************************
*  "A Very Simple Application" from the uIP 0.9 documentation
*****************************************************************************/

/* UIP_APPSTATE_SIZE: The size of the application-specific state
   stored in the uip_conn structure. (not used, but has to at least be one) */
#define UIP_APPSTATE_SIZE 1

#include "uip.h"

void connection_init(void);
void tcp_connection_app(void);
void udp_connection_app(void);

#define FS_STATISTICS 0

#define UIP_APPCALL     tcp_connection_app
//#define udp_appcall     udp_connection_app

struct uip_udp_conn *eth2can_status;
struct uip_udp_conn *eth2can_can;
struct uip_udp_conn *eth2can_uart0;
struct uip_udp_conn *eth2can_uart1;

char send_init;

void poll_can(void);
void poll_uart0(void);
void poll_uart1(void);
#endif
