#include "interrupt.hpp"

#include "peripheral/gpio.hpp"
#include "peripheral/usart.hpp"
#include "core/nvic.hpp"

#include "fifo.hpp"
#include "rs485.hpp"

#define UART3_BAUD_RATE 9600 /* bps */

typedef PB10   U3TX;   // PP
typedef PB11   U3RX;   // IN
typedef PB14   U3DE;   // PP
typedef PB15   U3RE;   // PP


typedef usart_fifo<USART3, 128, 32> U3FIFO;

static inline void U3_RxEnable()
{
  U3RE::setLow();
}

static inline void U3_RxDisable()
{
  U3RE::setHigh();
}

static inline void U3_TxEnable()
{
  U3DE::setHigh();
}

static inline void U3_TxDisable()
{
  U3DE::setLow();
}

void initializeUsart3()
{
  USART3::enableClock();
  USART3::configure(
      usart::cr1::rwu::RECEIVER_IN_ACTIVE_MODE,
      usart::cr1::re::RECEIVER_ENABLED,
      usart::cr1::te::TRANSMITTER_ENABLED,
      usart::cr1::idleie::IDLE_INTERRUPT_DISABLED,
      usart::cr1::rxneie::RXNE_ORE_INTERRUPT_ENABLED,
      usart::cr1::tcie::TC_INTERRUPT_DISABLED,
      usart::cr1::txeie::TXEIE_INTERRUPT_DISABLED,
      usart::cr1::peie::PEIE_INTERRUPT_DISABLED,
      usart::cr1::ps::EVEN_PARITY,
      usart::cr1::pce::PARITY_CONTROL_DISABLED,
      usart::cr1::wake::WAKE_ON_IDLE_LINE,
      usart::cr1::m::START_8_DATA_N_STOP,
      usart::cr1::ue::USART_ENABLED,
      usart::cr1::over8::OVERSAMPLING_BY_16,
      usart::cr2::stop::_1_STOP_BIT,
      usart::cr3::eie::ERROR_INTERRUPT_DISABLED,
      usart::cr3::hdsel::FULL_DUPLEX,
      usart::cr3::dmar::RECEIVER_DMA_DISABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_DISABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);
  USART3::setBaudRate<UART3_BAUD_RATE>();

  U3RE::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  U3DE::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  U3TX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);
  U3RX::setMode(gpio::cr::INPUT_PULL_X);
  U3RX::pullUp();

  U3_TxDisable();
  U3_RxEnable();

  NVIC::enableIrq<nvic::irqn::USART3>();
}

void interrupt::USART3()
{
	u32 status = USART3::getStatus();

  if(USART3::isTCIEenabled() && (status & usart::sr::tc::TRANSMISSION_COMPLETED))
  {
    U3_TxDisable();
		USART3::disableTCIE();
  }

	if(status & usart::sr::rxne::DATA_RECEIVED)
	{
		u8 rx_data = USART3::getData();
		u8 rx_error = 0;

		if(status & usart::sr::fe::FRAMING_ERROR_DETECTED) rx_error |= USART_FIFO_ERROR_FRAME;
		if(status & usart::sr::ore::OVERRUN_ERROR_DETECTED) rx_error |= USART_FIFO_ERROR_OVERRUN;
		if(status & usart::sr::nf::NOISE_DETECTED) rx_error |= USART_FIFO_ERROR_NOISE;

    if(U3FIFO::rx_full) rx_error |= USART_FIFO_ERROR_BUFFER_OVERFLOW;
    else U3FIFO::rx_put(rx_data);

		U3FIFO::rx_last_error = rx_error;
	}
	if(USART3::isTXEIenabled() && (status & usart::sr::txe::DATA_TRANSFERED_TO_THE_SHIFT_REGISTER))
	{
		int c = U3FIFO::tx_get();

		if(c < 0)
		{
			USART3::disableTXEI();
		}
		else
		{
			USART3::sendData(c & 0x00FF);
		}
	}
}

void rs485_init()
{
  U3FIFO::init();
	initializeUsart3();
}

int rs485_read()
{
	return U3FIFO::rx_get();
}

int rs485_write(u8* data, u8 count)
{
  while(count)
  {
    // TODO: add Timeout
    if(USART3::canSendDataYet())
    {
      U3_TxEnable();
      USART3::sendData(*data);
	    USART3::enableTCIE();
      data++;
      count--;
    }
    else
    {
      if(U3FIFO::tx_full) continue;
      U3FIFO::tx_put(*data);
      USART3::enableTXEI();
      data++;
      count--;
    }
  }
  return 0;
}
