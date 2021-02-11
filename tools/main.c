#include <stdio.h>

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <memory.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#define BAUD  9600
#define MAX_DATA_LEN    20
#define DATA_ID1        0xAA
#define DATA_ID2        0xC3

enum {
  PKT_CMD_READ_REG = 0,
  PKT_CMD_WRITE_REG,
  PKT_CMD_READ_CONF,
  PKT_CMD_WRITE_CONF,
  PKT_CMD_READ_1W,
  PKT_CMD_RESPONSE = 0x80
};


typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

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

static int port = -1;
static const char* devname = "/dev/ttyUSB1";

#define AFTERX(x) B ## x
#define XTTY_BAUD(x) AFTERX(x)
#define TTY_BAUD XTTY_BAUD(BAUD)

void sio_init()
{
    struct termios tty;
    port = open(devname, O_RDWR | O_NDELAY | O_NOCTTY );
    if(port < 0)
    {
        printf("Invalid COM port. Error: %d\n", errno);
        exit(1);
    }
    
	if(tcgetattr(port, &tty) < 0)
    {
        printf("tcgetattr() failed. Error: %d\n", errno);
        exit(1);
    }

    // Reset terminal to RAW mode
    cfmakeraw(&tty);

    // Set baud rate
    cfsetspeed(&tty, TTY_BAUD);

    if(tcsetattr(port, TCSAFLUSH, &tty) < 0)
    {
        printf("tcsetattr() failed. Error: %d\n", errno);
        exit(1);
    }
}

char sio_putchar(char c)
{
    return write(port, &c, 1);
}

int	sio_getchar()
{
    u8 c;
    if(read(port, &c, 1) <= 0) return -1;
    return (c & 0x00FF);
}

u8 sio_rxcount()
{
    int bytes_avail = 0;
    ioctl(port, FIONREAD, &bytes_avail);
    if(bytes_avail > 255) bytes_avail = 255;
    return (u8)bytes_avail;
}

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

static void tx_packet(packet_t* pkt)
{
  u8 n = pkt->len + sizeof(packet_t)+2, *c = (u8*)pkt;
  u16* crc = (u16*)(pkt->data + pkt->len);
  *crc = packet_crc(pkt);
  while(n--) sio_putchar(*c++);
}

int main(int argc, char* argv[])
{
	u8 data[32];
	packet_t* pkt = (packet_t*)data;
	
	if(argc == 2) devname = argv[1];
		
	memset(data, 0, sizeof(data));
	pkt->id[0] = DATA_ID1;
	pkt->id[1] = DATA_ID2;
	pkt->from = 1;
	pkt->to = 0x40;
	pkt->len = 4;
	pkt->data[0] = PKT_CMD_READ_REG;
	pkt->data[1] = 8;
	pkt->data[2] = 0; // Tick s64
	pkt->data[3] = 0;
	
	sio_init();
	
	tx_packet(pkt);
	
	close(port);

	return 0;
}
