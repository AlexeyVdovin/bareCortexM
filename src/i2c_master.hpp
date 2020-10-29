/*
 * i2c_master.hpp
 *
 *  Created on: Oct 26, 2020
 *      Author: av
 */

#ifndef _I2C_MASTER_HPP_
#define _I2C_MASTER_HPP_

#include "defs.hpp"

enum {
  DS2482_ERROR_RESET_FAILED = -3,
  DS2482_ERROR_1W_RESET_FAILED = -4,
  DS2482_ERROR_1W_SHORT_TO_GND = -5,
  DS2482_ERROR_1W_DISCONNECTED = -6,
  DS2482_ERROR_1W_NOT_MATCH = -7,
  DS2482_ERROR_1W_NOT_READY = -8,
  DS2482_ERROR_1W_SCRATCHPAD = -9,
};

typedef struct
{
  u8 addr[8];
  u8 next;
} ow_serch_t;

/* I2C gateway LL operations */
void ds2482_set_conf(u8 const dev, u8 conf);
void ds2482_reset(u8 const dev);
u8 ds2482_get_status(u8 const dev);
u8 ds2482_get_data(u8 const dev);
u8 ds2482_get_conf(u8 const dev);
u8 ds2482_read(u8 const dev);
int ds2482_pool(u8 const dev, u8 mask, u8 value, int count);

/* I2C gateway 1W operations */
void ds2482_1w_reset(u8 const dev);
void ds2482_1w_1b(u8 const dev, char bit);
void ds2482_1w_wbyte(u8 const dev, u8 value);
void ds2482_1w_rbyte(u8 const dev);
void ds2482_1w_triplet(u8 const dev, char bit);

/* I2C gateway 1W high level operations */
int ds2482_1w_search(u8 const dev, ow_serch_t* search);
void ds2482_1w_skip(u8 const dev);
int ds2482_1w_match(u8 const dev, u8 addr[8]);

/* I2C gateway DS18B20 operations */
void ds2482_ds18b20_convert(u8 const dev);
int ds2482_ds18b20_read_power(u8 const dev, u8* value);
int ds2482_ds18b20_read_scratchpad(u8 const dev, u8* data);
void ds2482_ds18b20_read_eeprom(u8 const dev);
int ds2482_ds18b20_write_configuration(u8 const dev, u8* conf);
void ds2482_ds18b20_write_eeprom(u8 const dev);

int ds2482_ds18b20_search(u8 const dev, u8 addr[8][8]);
int ds2482_ds18b20_start(u8 const dev, u8* addr);
int ds2482_ds18b20_read(u8 const dev, u8* addr, u8* pad);

#endif /* _I2C_MASTER_HPP_ */
