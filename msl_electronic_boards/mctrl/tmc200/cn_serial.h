#ifndef CN_SERIAL
#define CN_SERIAL

#include "main.h"

// special characters
#define SERIAL_START        0x81
#define SERIAL_END          0x82
#define SERIAL_QUOTE        0x84
#define SERIAL_MAX_COUNTER  0x80

// size defines
#define SERIAL_BUFFER_SIZE  32
#define SERIAL_START_POS     0
//#define SERIAL_COUNT_POS     1
#define SERIAL_GROUP_POS     1
#define SERIAL_CMD_POS       2
#define SERIAL_DATA_POS      3

#define SERIAL_REQ_REL_POS  -2

// state defines
#define CN_STATE_WAIT    0
#define CN_STATE_START   1
#define CN_STATE_RECEIVE 2
#define CN_STATE_QUOTE   3

#define MAXPACKAGESIZE 64
#define SERIAL_BUFFER_COUNT 2

typedef struct cn_send_buffer {
 uchar tosend[MAXPACKAGESIZE];  
 uchar length;
 uchar send_counter;
 ubyte empty;  
}  send_buffer;

// START, COUNTER, GROUP, CMD, DATA (0-24), [REQ], CRC, END
// CRC = [COUNTER .. REQ] (CRC may be quoted but does not contain quote chars)

void cn_serial_init(void);
void cn_debug_serial(uword command);
void cn_serial_rx_interrupt(void);
void cn_serial_tx_interrupt(void);
void cn_serial_send_data(char* data, ubyte length);
uword cn_quote_data(ubyte* inbuffer,ubyte* outbuffer, uword length);
void cn_decode_command(ubyte* inbuffer, uword length);
void cn_write_response(ubyte group, ubyte cmd, ubyte* data, uword data_length);
__inline void cn_write_plain(ubyte c);
__inline void cn_write_quoted(ubyte* inbuffer, uword length);
__inline void cn_wait_tx_done();
__inline void cn_serial_store(uword c);
__inline void cn_serial_reset_buffer();

#endif