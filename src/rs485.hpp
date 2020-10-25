/*
 * rs485.hpp
 *
 *  Created on: Mar 26, 2016
 *      Author: av
 */

#ifndef _RS485_HPP_
#define _RS485_HPP_

#include "defs.hpp"

void rs485_init();
int  rs485_read();
int  rs485_write(u8* data, u8 count);

#endif /* _RS485_HPP_ */
