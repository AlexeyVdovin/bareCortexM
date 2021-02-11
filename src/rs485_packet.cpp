#include <memory.h>
#include "rs485.hpp"
#include "rs485_packet.hpp"

enum {
  PACKET_RX_TIMEOUT = 100
};

extern volatile s64 tick;

static inline s64 get_time() { return tick; }

/* Calculating XMODEM CRC-16 in 'C'
   ================================
   Reference model for the translated code */

#define poly 0x1021

/* On entry, addr=>start of data
             num = length of data
             crc = incoming CRC     */
static u16 crc16(u8 *addr, int num, u32 crc)
{
  int i;
  for(; num > 0; --num)                /* Step through bytes in memory */
  {
    crc = crc ^ (*addr++ << 8);        /* Fetch byte from memory, XOR into CRC top byte*/
    for(i = 0; i < 8; ++i)             /* Prepare to rotate 8 bits */
    {
      crc = crc << 1;                  /* rotate */
      if(crc & 0x10000)                /* bit 15 was set (now bit 16)... */
          crc = (crc ^ poly) & 0xFFFF; /* XOR with XMODEM polynomic */
                                       /* and ensure CRC remains 16-bit value */
    }                                  /* Loop for 8 bits */
  }                                    /* Loop until num=0 */
  return (u16)crc;                     /* Return updated CRC */
}

/* Calc packet CRC */
static u16 packet_crc(packet_t* pkt)
{
  u8* p = (u8*)pkt;
  u16 crc = crc16(p, sizeof(packet_t) + pkt->len, 0);
  return crc;
}

static packet_t* rx_packet()
{
  static u8 rx_pkt[sizeof(packet_t)+MAX_DATA_LEN+2/*crc*/];
  static u8 _rx_pos = 0;
  static s64 rx_time = 0;

  packet_t *pkt = (packet_t*)rx_pkt;
  register u8 rx_pos = _rx_pos;
  int c;

  // Check for timeout    
  if(rx_pos && rx_time && get_time() > rx_time) rx_pos = 0;

  while((c = rs485_read()) >= 0)
  {
    u8 u = (u8)(c & 0x00FF);
        
    if(rx_pos == 0 || (rx_pos == 1 && u == DATA_ID1))
    {
      rx_pos = 0;
      if(u != DATA_ID1) continue;
      else rx_time = get_time()+PACKET_RX_TIMEOUT;
    }
    if(rx_pos == 1 && u != DATA_ID2) { rx_pos = 0; continue; }
    if(rx_pos == sizeof(packet_t)-1 && pkt->len > MAX_DATA_LEN) { rx_pos = 0; continue; }
    if(rx_pos < sizeof(packet_t) || rx_pos < sizeof(packet_t) + pkt->len + 2)
    { 
      rx_pkt[rx_pos++] = u;
    }
    if(rx_pos >= sizeof(packet_t) + pkt->len + 2)
    {
      rx_pos = 0;
      u16 crc = ((u16)(pkt->data[pkt->len+1]) << 8)|(pkt->data[pkt->len]);
      if(pkt->len > 0 && crc == packet_crc(pkt)) { _rx_pos = rx_pos; return pkt; }
    }
  }
  _rx_pos = rx_pos;
  return 0;
}

static void tx_packet(packet_t* pkt)
{
  u8 n = pkt->len + sizeof(packet_t)+2, *c = (u8*)pkt;
  u16 crc = packet_crc(pkt);
  pkt->data[pkt->len] = (u8)(crc & 0x00FF);
  pkt->data[pkt->len+1] = (u8)((crc >> 8)& 0x00FF);

  rs485_write(c, n);
}

void pkt_init()
{

}

int  pkt_pool()
{
  packet_t* pkt = rx_packet();
  if(pkt != 0) pkt_process(pkt);
  return 0;
}

void pkt_send(packet_t* pkt)
{
  tx_packet(pkt);
}