/*
 * rs485_packet.hpp
 *
 *  Created on: Oct 29, 2020
 *      Author: av
 */

#ifndef _RS485_PACKET_HPP_
#define _RS485_PACKET_HPP_

#include "defs.hpp"

#define MAX_DATA_LEN    20

enum {
  DATA_ID1 = 0xAA,
  DATA_ID2 = 0xC3
};

enum {
  PKT_CMD_READ_REG = 0,
  PKT_CMD_WRITE_REG,
  PKT_CMD_READ_CONF,
  PKT_CMD_WRITE_CONF,
  PKT_CMD_READ_1W,
  PKT_CMD_RESPONSE = 0x80,
  PKT_CMD_NAK = 0xFF
};

/*
  1 CMD
  1 LEN
  2 ADDR
  16 DATA[]
*/

#pragma pack(push, 1)
typedef struct
{
    u8  id[2];
    u8  from;
    u8  via;
    u8  to;
    u8  flags;
    u8  seq; // req/resp must match this field
    u8  len;
    u8  data[0];
} packet_t;
#pragma pack(pop)

void pkt_init();
int  pkt_pool();
void pkt_send(packet_t* pkt);

void pkt_process(packet_t* pkt);

#endif /* _RS485_PACKET_HPP_ */
