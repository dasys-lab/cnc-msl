/***************************************************************************** 
*  "A Very Simple Application" from the uIP 0.9 documentation 
*****************************************************************************/ 

#include <stdlib.h> 
#include <stdio.h> 
#include <string.h> 

#include "main.h" 
#include "temperatur.h" 
#include "app.h" 
#include "uip.h" 

unsigned char prompt[80]; 
unsigned char *pDynamicSendBuffer; 

void welcomescreen(void); 
void parser(unsigned char *); 

void example1_init(void) 
{ 
    pDynamicSendBuffer = malloc(1024); 
    uip_listen(HTONS(3333)); 
} 


void example1_app(void) 
{ 
    if(uip_connected()) 
        welcomescreen(); 

        if(uip_newdata() || uip_rexmit()) 
        parser((unsigned char *) uip_appdata); 
} 

void 
parser(unsigned char *buffer) 
{ 
    switch(buffer[0]) 
        { 
    case 'u': 
    case 'U': 
        sprintf(pDynamicSendBuffer,"%c\r\nModule uptime: %d days %02d:%02d:%02d\r\n", buffer[0], iDays, iHour, iMin, iSec); 
        break; 
         
    case 't': 
    case 'T': 
        sprintf(pDynamicSendBuffer, "%c\r\n%c%d.%d°C\r\n", buffer[0], (subzero)?'-':'+', (int) cel, cel_frac_bits); 
        break; 
           
    case 's': 
        case 'S': 
                sprintf(pDynamicSendBuffer, 
                        "%c\r\n" 
                        "-----[ARTHERNET STATS]-----------------------------------------------------\r\n" 
                        "  == IP statistics: ==\r\n" 
                        "    * IP packets dropped:   %d\r\n" 
                        "    * IP packets received:  %d\r\n" 
                        "    * IP packets sent:      %d\r\n" 
                        "    * IP wrong version:     %d\r\n" 
                        "    * IP wrong low-len:     %d\r\n" 
                        "    * IP wrong hi-len:      %d\r\n" 
                        "    * IP fragmentation err: %d\r\n" 
                        "    * IP checksum err:      %d\r\n" 
                        "    * IP protocol err:      %d\r\n" 
                        "\r\n" 
                        "  == ICMP statistics: ==\r\n" 
                        "    * ICMP packets dropped:  %d\r\n" 
                        "    * ICMP packets received: %d\r\n" 
                        "    * ICMP packets sent:     %d\r\n" 
                        "    * ICMP type err:         %d\r\n" 
                        "\r\n" 
                        "  == TCP statistics: ==\r\n" 
                        "    * TCP packets dropped:  %d\r\n" 
                        "    * TCP packets received: %d\r\n" 
                        "    * TCP packets sent:     %d\r\n" 
                        "    * TCP checksum err:     %d\r\n" 
                        "    * TCP acknowledge err:  %d\r\n" 
                        "    * TCP resetpackets:     %d\r\n" 
                        "    * TCP retransmits:      %d\r\n" 
                        "    * TCP SYN-dropped:      %d\r\n" 
                        "    * TCP SYN-resets:       %d\r\n" 
                        "-----[END STATS]-----------------------------------------------------------\r\n" 
                        "\r\n", 
            buffer[0], 
                        uip_stat.ip.drop, 
                        uip_stat.ip.recv, 
                        uip_stat.ip.sent, 
                        uip_stat.ip.vhlerr, 
                        uip_stat.ip.hblenerr, 
                        uip_stat.ip.lblenerr, 
                        uip_stat.ip.fragerr, 
                        uip_stat.ip.chkerr, 
                        uip_stat.ip.protoerr, 
                        uip_stat.icmp.drop, 
                        uip_stat.icmp.recv, 
                        uip_stat.icmp.sent, 
                        uip_stat.icmp.typeerr, 
                        uip_stat.tcp.drop, 
                        uip_stat.tcp.recv, 
                        uip_stat.tcp.sent, 
                        uip_stat.tcp.chkerr, 
                        uip_stat.tcp.ackerr, 
                        uip_stat.tcp.rst, 
                        uip_stat.tcp.rexmit, 
                        uip_stat.tcp.syndrop, 
                        uip_stat.tcp.synrst); 
                break; 
                    
        case 'i': 
        case 'I': 
                sprintf(pDynamicSendBuffer, 
                        "%c\r\nConnection: IP = %d.%d.%d.%d; local port = %d; remote port = %d\r\n", 
            buffer[0], 
                        HTONS(uip_conn->ripaddr[0]) >> 8, 
                        HTONS(uip_conn->ripaddr[0]) & 0xFF, 
                        HTONS(uip_conn->ripaddr[1]) >> 8, 
                        HTONS(uip_conn->ripaddr[1]) & 0xFF, 
                        HTONS(uip_conn->lport), 
                        HTONS(uip_conn->rport)); 
                break;            

        case '\x1B': 
                uip_send("\r\nClosing connection.\r\nBye bye...\r\n\r\n", 37); 
                uip_close(); 
                break; 

    case '?': 
        sprintf(pDynamicSendBuffer, 
            "%c\r\n" 
            "-----[HELP]-------------------------\r\n" 
            " ?   - This help screen\r\n" 
            " i/I - Infoscreen\r\n" 
            " s/S - Network stats\r\n" 
            " t/T - DS18x20 temperature\r\n" 
            " u/U - Module's uptime\r\n" 
            " ESC - Close this connection\r\n" 
            "\r\n", buffer[0]); 
        break; 

    case '\xD': 
    case '\xA': 
        sprintf(pDynamicSendBuffer, "\r\n"); 
        break; 
         
        default: 
        sprintf(pDynamicSendBuffer, "%c\r\nunknown command: '%c'\r\n", buffer[0], buffer[0]); 
        break; 
        } 

    sprintf(prompt, "\r\narthernet [%02d:%02d:%02d] $ ", iHour, iMin, iSec); 
    strcat(pDynamicSendBuffer, prompt); 
    uip_send(pDynamicSendBuffer, strlen(pDynamicSendBuffer)); 
} 

void 
welcomescreen(void) 
{ 
    sprintf(pDynamicSendBuffer, 
        "\r\n" 
        "_______         _____ ______                              _____\r\n" 
        "___    |__________  /____  /_ _____ _______________ _____ __  /_\r\n" 
        "__  /| |__  ___/_  __/__  __ \\_  _ \\__  ___/__  __ \\_  _ \\_  __/\r\n" 
        "_  ___ |_  /    / /_  _  / / //  __/_  /    _  / / //  __// /_\r\n" 
        "/_/  |_|/_/     \\__/  /_/ /_/ \\___/ /_/     /_/ /_/ \\___/ \\__/\r\n" 
        "\r\n" 
        " Welcome!\r\n" 
        " ========\r\n" 
        "\r\n" 
        "  This is ARTHERNET v1 with a simple telnet demo-application!\r\n" 
        "  Have a lot of fun...\r\n\r\n%s", prompt); 

    uip_send(pDynamicSendBuffer, strlen(pDynamicSendBuffer)); 
} 
