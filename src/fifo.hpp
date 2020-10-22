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
 	volatile static u8 rx_last_error;
	enum rx_buffer_size { RX_BUFFER_SIZE = rx_buff_size };
	enum tx_buffer_size { TX_BUFFER_SIZE = tx_buff_size };
	volatile static u8 rx_buffer[RX_BUFFER_SIZE];
	volatile static u8 rx_head;
	volatile static u8 rx_tail;
	volatile static bool rx_full;
	volatile static u8 tx_buffer[TX_BUFFER_SIZE];
	volatile static u8 tx_head;
	volatile static u8 tx_tail;
	volatile static bool tx_full;

    static inline void inc_rx_head();
	static inline void inc_rx_tail();
	static inline bool is_rx_empty();
	static inline void rx_put(u8 c);
	static inline int  rx_get();

    static inline void inc_tx_head();
	static inline void inc_tx_tail();
	static inline bool is_tx_empty();
	static inline void tx_put(u8 c);
	static inline int  tx_get();

	static inline void init();
private:
	usart_fifo();
};

template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_last_error;

template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_buffer[RX_BUFFER_SIZE];
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_head;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_tail;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile bool usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_full;

template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_buffer[TX_BUFFER_SIZE];
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_head;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile u8 usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_tail;
template <typename USART, u8 rx_buff_size, u8 tx_buff_size> volatile bool usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_full;

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::init()
{
		rx_head = 0;
		rx_tail = 0;
		rx_full = false;
		tx_head = 0;
		tx_tail = 0;
		tx_full = false;
}


template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::inc_rx_head()
{
	if(rx_full) rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
	rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
	rx_full = (rx_head == rx_tail);
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::inc_rx_tail()
{
	rx_full = false;
	rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
bool usart_fifo<USART, rx_buff_size, tx_buff_size>::is_rx_empty()
{
	return (!rx_full && (rx_head == rx_tail));
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_put(u8 c)
{
	rx_buffer[rx_head] = c;
	inc_rx_head();
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::rx_get()
{
	int r = -1;
	if(!is_rx_empty())
	{
		r = rx_buffer[rx_tail] & 0x00FF;
		inc_rx_tail();
	}
	return r;
}

// --------------------------------------------------------

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::inc_tx_head()
{
	if(tx_full) tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
	tx_head = (tx_head + 1) % TX_BUFFER_SIZE;
	tx_full = (tx_head == tx_tail);
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::inc_tx_tail()
{
	tx_full = false;
	tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
bool usart_fifo<USART, rx_buff_size, tx_buff_size>::is_tx_empty()
{
	return (!tx_full && (tx_head == tx_tail));
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
void usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_put(u8 c)
{
	tx_buffer[tx_head] = c;
	inc_tx_head();
}

template <typename USART, u8 rx_buff_size, u8 tx_buff_size>
int usart_fifo<USART, rx_buff_size, tx_buff_size>::tx_get()
{
	int r = -1;
	if(!is_tx_empty())
	{
		r = tx_buffer[tx_tail] & 0x00FF;
		inc_tx_tail();
	}
	return r;
}

// ---------------------------------------------------------
#endif /* FIFO_HPP_ */
