#include <memory.h>

#include "interrupt.hpp"
#include "core/nvic.hpp"

#include "peripheral/gpio.hpp"
#include "peripheral/i2c.hpp"

#include "i2c_master.hpp"

enum {
  DS2482_CMD_TRIPLET          = 0x78,
  DS2482_CMD_SINGLEBIT        = 0x87,
  DS2482_CMD_READBYTE         = 0x96,
  DS2482_CMD_WRITEBYTE        = 0xA5,
  DS2482_CMD_RESETWIRE        = 0xB4,
  DS2482_CMD_WRITECONFIG      = 0xD2,
  DS2482_CMD_SET_POINTER      = 0xE1,
  DS2482_CMD_RESET            = 0xF0
};

enum {
  DS2482_CONFIG_REGISTER      = 0xC3,
  DS2482_DATA_REGISTER        = 0xE1,
  DS2482_STATUS_REGISTER      = 0xF0,
};

enum {
  OW_CMD_SKIP                 = 0xCC,
  OW_CMD_MATCH                = 0x55,
  OW_CMD_SEARCH               = 0xF0
};

enum {
  DS1820_CMD_CONVERT          = 0x44,
  DS1820_CMD_WR_SCRATCHPAD    = 0x4E,
  DS1820_CMD_RD_SCRATCHPAD    = 0xBE,
  DS1820_CMD_PPD              = 0xB4,
  DS1820_CMD_EE_WRITE         = 0x48,
  DS1820_CMD_EE_RECALL        = 0xB8
};

void delay(u32 us);

/*
void i2c_lock()
{
  while(I2C1::isTheBusBusy()) { };
  I2C1::maskInterrupts();
}

void i2c_unlock()
{
  I2C1::unmaskInterrupts();
}
*/
void ds2482_set_conf(u8 const dev, uint8_t conf)
{
  conf = ((~conf & 0x0F) << 4) | (conf & 0x0F);
  I2C1::writeSlaveRegister(dev, DS2482_CMD_WRITECONFIG, conf);
}

void ds2482_reset(u8 const dev)
{
  I2C1::writeSlaveByte(dev, DS2482_CMD_RESET);
  delay(1);
  ds2482_set_conf(dev, 0x00);
}

u8 ds2482_get_status(u8 const dev)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_SET_POINTER, DS2482_STATUS_REGISTER);
  return I2C1::readSlaveByte(dev);
}

u8 ds2482_get_data(u8 const dev)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_SET_POINTER, DS2482_DATA_REGISTER);
  return I2C1::readSlaveByte(dev);
}

u8 ds2482_get_conf(u8 const dev)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_SET_POINTER, DS2482_CONFIG_REGISTER);
  return I2C1::readSlaveByte(dev);
}

u8 ds2482_read(u8 const dev)
{
  return I2C1::readSlaveByte(dev);
}

int ds2482_pool(u8 const dev, u8 mask, u8 value, int count)
{
  int n = 0;
  int res = -1;
  u8 b;

  do {
    if(n++ > count) { res = -2; break; }
    res = I2C1::readSlaveByte(dev);
  } while((res & mask) != value);
  
  return res;
}

void ds2482_1w_reset(u8 const dev)
{
  I2C1::writeSlaveByte(dev, DS2482_CMD_RESETWIRE);
}

void ds2482_1w_1b(u8 const dev, char bit)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_SINGLEBIT, bit ? 0x80 : 0x00);
}

void ds2482_1w_wbyte(u8 const dev, u8 value)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_WRITEBYTE, value);
}

void ds2482_1w_rbyte(u8 const dev)
{
  I2C1::writeSlaveByte(dev, DS2482_CMD_READBYTE);
}

void ds2482_1w_triplet(u8 const dev, char bit)
{
  I2C1::writeSlaveRegister(dev, DS2482_CMD_TRIPLET, bit ? 0x80 : 0x00);
}

int ds2482_1w_search(u8 const dev, ow_serch_t* search)
{
  int res = -1;
  u8 id, dir, comp, status, i, zero = 0;

  if(search->next == 0xFF) return -2;

  do
  {
    ds2482_1w_wbyte(dev, OW_CMD_SEARCH);
    for(i = 0; i< 64; ++i)
    {
      int n = i/8;
      int b = 1 << (i%8);

      if(i < search->next) dir = search->addr[n] & b;
      else dir = i == search->next;

      res = ds2482_pool(dev, 0x01, 0x00, 2000);
      if(res < 0) break;

      ds2482_1w_triplet(dev, dir);

      res = ds2482_pool(dev, 0x01, 0x00, 2000);
      if(res < 0) break;
      status = res & 0x00FF;

      id = status & 0x20;
      comp = status & 0x40;
      dir = status & 0x80;

      if(id && comp) { res = -2; break; }
      else if(!id && !comp && !dir) zero = i;

      if(dir) search->addr[n] |= b;
      else search->addr[n] &= ~b;
    }

    if(zero == 0) search->next = 0xFF;
    else search->next = zero;

  } while(0);
  return res;
}

void ds2482_1w_skip(u8 const dev)
{
  ds2482_1w_wbyte(dev, OW_CMD_SKIP);
}

int ds2482_1w_match(u8 const dev, u8 addr[8])
{
  int i, res;
  u8 status;

  do
  {
    ds2482_1w_wbyte(dev, OW_CMD_MATCH);

    for(i = 0; i < 8; ++i)
    {
      res = ds2482_pool(dev, 0x01, 0x00, 2000);
      if(res < 0) break;

      ds2482_1w_wbyte(dev, addr[i]);
    }
  } while(0);
  return res;
}

void ds2482_ds18b20_convert(u8 const dev)
{
  ds2482_1w_wbyte(dev, DS1820_CMD_CONVERT);
}

int ds2482_ds18b20_read_power(u8 const dev)
{
  int res;
  do
  {
    ds2482_1w_wbyte(dev, DS1820_CMD_PPD);

    res = ds2482_pool(dev, 0x01, 0x00, 2000);

    ds2482_1w_1b(dev, 0x80);

    res = ds2482_pool(dev, 0x01, 0x00, 2000);

  } while(0);
  return res;
}

int ds2482_ds18b20_read_scratchpad(u8 const dev, u8* data)
{
  int i, res;

  do
  {
    ds2482_1w_wbyte(dev, DS1820_CMD_RD_SCRATCHPAD);

    res = ds2482_pool(dev, 0x01, 0x00, 2000);
    if(res < 0) break;

    for(i = 0; i < 9; ++i)
    {
      ds2482_1w_rbyte(dev);

      res = ds2482_pool(dev, 0x01, 0x00, 2000);

      data[i] = ds2482_get_data(dev);

      //printf("scr[%d]: 0x%02x\n", i, data[i]);
    }
    // TODO: Check CRC
  } while(0);
  return res;
}

void ds2482_ds18b20_read_eeprom(u8 const dev)
{
  ds2482_1w_wbyte(dev, DS1820_CMD_EE_RECALL);
}

int ds2482_ds18b20_write_configuration(u8 const dev, u8* conf)
{
  int i, res;

  do
  {
    ds2482_1w_wbyte(dev, DS1820_CMD_WR_SCRATCHPAD);

    for(i = 0; i < 3; ++i)
    {
      res = ds2482_pool(dev, 0x01, 0x00, 2000);
      if(res < 0) break;

      ds2482_1w_wbyte(dev, conf[i]);
    }
  } while(0);
  return res;
}

void ds2482_ds18b20_write_eeprom(u8 const dev)
{
  ds2482_1w_wbyte(dev, DS1820_CMD_EE_WRITE);
}

int ds2482_ds18b20_search(u8 const dev, u8 addr[8][8])
{
  int res, n = 0;
  ow_serch_t search;

  ds2482_reset(dev);

  res = ds2482_pool(dev, 0x10, 0x00, 4000);
  if(res < 0) return DS2482_ERROR_RESET_FAILED;

  memset(&search, 0, sizeof(search));

  do
  {
    ds2482_1w_reset(dev);

    res = ds2482_pool(dev, 0x01, 0x00, 3000);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_RESET_FAILED;
      break;
    }

    if(res & 0x04)
    {
      res = DS2482_ERROR_1W_SHORT_TO_GND;
      break;
    }

    if(res & 0x02 == 0)
    {
      // Error: 1W presence pulse is not detected
      res = DS2482_ERROR_1W_DISCONNECTED;
      break;
    }

    ds2482_set_conf(dev, 0x01); // Enable APU

    res = ds2482_1w_search(dev, &search);
    if(res < 0) { res = n; break; }

    // printf("Found 1W device [%d]: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n", n, search.addr[0], search.addr[1], search.addr[2], search.addr[3], search.addr[4], search.addr[5], search.addr[6], search.addr[7]);
    memcpy(addr[n], search.addr, sizeof(addr[0]));
    ++n;
    if(n > 7) { res = n-1; break; }
  } while(1);
  return res;
}

int ds2482_ds18b20_start(u8 const dev, u8* addr)
{
  int res;

  do
  {
    ds2482_1w_reset(dev);

    res = ds2482_pool(dev, 0x01, 0x00, 3000);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_RESET_FAILED;
      break;
    }

    if(res & 0x04)
    {
      res = DS2482_ERROR_1W_SHORT_TO_GND;
      break;
    }

    if(res & 0x02 == 0)
    {
      // Error: 1W presence pulse is not detected
      res = DS2482_ERROR_1W_DISCONNECTED;
      break;
    }

    ds2482_set_conf(dev, 0x01); // Enable APU

    if(addr == 0)
    {
      ds2482_1w_skip(dev);
    }
    else
    {
      res = ds2482_1w_match(dev, addr);
      if(res < 0)
      {
        res = DS2482_ERROR_1W_NOT_MATCH;
        break;
      }
    }
    res = ds2482_pool(dev, 0x01, 0x00, 2000);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_NOT_READY;
      break;
    }

    ds2482_ds18b20_convert(dev);
  } while(0);
  return res;
}

int ds2482_ds18b20_read(u8 const dev, u8* addr, u8* pad)
{
  int res;

  do
  {
    ds2482_1w_reset(dev);

    res = ds2482_pool(dev, 0x01, 0x00, 3000);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_RESET_FAILED;
      break;
    }

    if(res & 0x04)
    {
      res = DS2482_ERROR_1W_SHORT_TO_GND;
      break;
    }

    if(res & 0x02 == 0)
    {
      // Error: 1W presence pulse is not detected
      res = DS2482_ERROR_1W_DISCONNECTED;
      break;
    }

    ds2482_set_conf(dev, 0x01); // Enable APU

    res = ds2482_1w_match(dev, addr);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_NOT_MATCH;
      break;
    }

    res = ds2482_pool(dev, 0x01, 0x00, 2000);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_NOT_READY;
      break;
    }

    res = ds2482_ds18b20_read_scratchpad(dev, pad);
    if(res < 0)
    {
      res = DS2482_ERROR_1W_SCRATCHPAD;
      break;
    }
  } while(0);
  return res;
}
