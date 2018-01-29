/*
 * fifo.hpp
 *
 *  Created on: Mar 21, 2016
 *      Author: av
 */

#ifndef FIFO_HPP_
#define FIFO_HPP_

#include "defs.hpp"

enum usart_fifo_error
{
    USART_FIFO_ERROR_FRAME = 1,
    USART_FIFO_ERROR_OVERRUN = 2,
    USART_FIFO_ERROR_NOISE = 4,
    USART_FIFO_ERROR_BUFFER_OVERFLOW = 8,
    USART_FIFO_NO_DATA = 16
};

template <typename USART, u8 rx_buff_size, u8 tx_buff_size> class usart_fifo
{
public:
	enum rx_buffer_size { RX_BUFFER_SIZE = rx_buff_size };
	enum tx_buffer_size { TX_BUFFER_SIZE = tx_buff_size };
	volatile static u8 rx_last_error;
	volatile static u8 rx_buffer[RX_BUFFER_SIZE];
	volatile static u8 rx_buffer_head;
	volatile static u8 rx_buffer_tail;
	volatile static u8 tx_buffer[TX_BUFFER_SIZE];
	volatile static u8 tx_buffer_head;
	volatile static u8 tx_buffer_tail;

	static inline int rx_getchar();
	static inline int tx_putchar(u8 c);
	static inline int rx_count();

	static inline int rx_put(u8 c);
	static inline int tx_char();

	static inline void init();
private:
	usart_fifo();
};

template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_last_error;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_buffer_head;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_buffer_tail;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_buffer[RX_BUFFER_SIZE];
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_buffer[TX_BUFFER_SIZE];
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_buffer_head;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_buffer_tail;

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::init()
{
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_getchar()
{
	u8 rx_head = rx_buffer_head;
	u8 rx_tail = rx_buffer_tail;

	if(rx_head == rx_tail) return -1;

	rx_tail = (u8)((rx_tail + 1)&(u8)(RX_BUFFER_SIZE - 1));

	rx_buffer_tail = rx_tail;

	return (rx_buffer[rx_tail] & 0x00FF);
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_putchar(u8 c)
{
	u8 tx_head = (u8)((tx_buffer_head + 1)&(u8)(TX_BUFFER_SIZE-1));

	if(tx_head == tx_buffer_tail) return -1;

	tx_buffer[tx_head] = c;
	tx_buffer_head = tx_head;
	return 0;
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_count()
{
	u8 rx_head = rx_buffer_head;
	u8 rx_tail = rx_buffer_tail;

	return (rx_head < rx_tail) ? (rx_head + RX_BUFFER_SIZE - rx_tail) : (rx_head - rx_tail);
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_put(u8 c)
{
	u8 rx_head = (u8)((rx_buffer_head + 1)&(u8)(RX_BUFFER_SIZE - 1));

	if(rx_head == rx_buffer_tail)
	{
		return -1;
	}
	else
	{
		rx_buffer[rx_head] = c;
		rx_buffer_head = rx_head;
	}
	return 0;
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_char()
{
	uint8_t tx_tail = tx_buffer_tail;
	uint8_t tx_head = tx_buffer_head;

	if(tx_head != tx_tail++)
	{
		tx_tail &= TX_BUFFER_SIZE - 1;

		tx_buffer_tail = tx_tail;
		return (tx_buffer[tx_tail] & 0x00FF);
	}
	return -1;
}

#endif /* FIFO_HPP_ */
