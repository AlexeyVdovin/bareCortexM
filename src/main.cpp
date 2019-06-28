/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 *
 * This file is part of libstm32pp.
 *
 * libstm32pp is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * libstm32pp is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libstm32pp. If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#include <stdio.h>

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/afio.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/usart.hpp"
#include "peripheral/adc.hpp"
#include "peripheral/dma.hpp"
#include "peripheral/tim.hpp"
#include "peripheral/i2c.hpp"

#define UART1_BAUD_RATE 115200
#define I2C_SLAVE_ADDR  0x50

typedef PA9  U1TX;
typedef PA10 U1RX;
typedef PB9  LED;

u64 tick = 0;

void initializeGpio()
{
  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();

  AFIO::enableClock();

  LED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
}

void initializeUsart1()
{
  USART1::enableClock();
  USART1::configure(
      usart::cr1::rwu::RECEIVER_IN_ACTIVE_MODE,
      usart::cr1::re::RECEIVER_ENABLED,
      usart::cr1::te::TRANSMITTER_ENABLED,
      usart::cr1::idleie::IDLE_INTERRUPT_DISABLED,
      usart::cr1::rxneie::RXNE_ORE_INTERRUPT_DISABLED,
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
  USART1::setBaudRate<UART1_BAUD_RATE /* bps */ >();
  U1TX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);
  U1RX::setMode(gpio::cr::INPUT_PULL_X);
  U1RX::pullUp();
}

void initializeTimer()
{
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt< 1000 /* Hz */ >();
}

void initializeI2c()
{
  I2C1::enableClock();
  I2C1::setSlaveAddr1(I2C_SLAVE_ADDR);
  I2C1::setSlaveAddr2(I2C_SLAVE_ADDR);
  I2C1::enablePeripheral();
}

/*
 ADC2  ADC1
------------
In1 V, A
In2 V, A
In3 V, A
In4 V, A
VIn V, Temp
Vcc/2, Vref
- accumulate sum of V*V and number of samples
- every second divide accumulated value by number of samples and get SQRT of it.
*/
void initializeAdc()
{
/*
  ADC1::enableClock();
  ADC2::enableClock();
  ADC1::configure(
    adc::cr1::awdch::SET_ANALOG_WATCHDOG_ON_CHANNEL0,
    adc::cr1::eocie::END_OF_CONVERSION_INTERRUPT_DISABLED,
    adc::cr1::awdie::ANALOG_WATCHDOG_INTERRUPT_DISABLED,
    adc::cr1::jeocie::END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED,
    adc::cr1::scan::SCAN_MODE_ENABLED,
    adc::cr1::awdsgl::ANALOG_WATCHDOG_ENABLED_ON_ALL_CHANNELS,
    adc::cr1::jauto::AUTOMATIC_INJECTED_CONVERSION_DISABLED,
    adc::cr1::discen::DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED,
    adc::cr1::jdiscen::DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED,
    adc::cr1::discnum::_1_CHANNEL_FOR_DISCONTINUOUS_MODE,
    adc::cr1::jawden::ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS,
    adc::cr1::awden::ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS,
    adc::cr1::res::_12_BITS_RESOLUTION,
    adc::cr1::ovrie::OVERRUN_INTERRUPT_DISABLED,
    adc::cr2::adon::ADC_ENABLED,
    adc::cr2::cont::CONTINUOUS_CONVERSION_MODE,
    adc::cr2::dma::DMA_MODE_ENABLED,
    adc::cr2::dds::NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER,
    adc::cr2::eocs::EOC_BIT_IS_SET_AFTER_A_SEQUENCE_OF_REGULAR_CONVERSIONS,
    adc::cr2::align::RIGTH_ALIGNED_DATA,
    adc::cr2::jextsel::INJECTED_GROUP_TRIGGERED_BY_TIMER1_CC4,
    adc::cr2::jexten::INJECTED_TRIGGER_DISABLED,
    adc::cr2::jswstart::INJECTED_CHANNELS_ON_RESET_STATE,
    adc::cr2::extsel::REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC1,
    adc::cr2::exten::REGULAR_TRIGGER_DISABLED,
    adc::cr2::swstart::REGULAR_CHANNELS_ON_RESET_STATE);
  ADC2::configure(
    adc::cr1::awdch::SET_ANALOG_WATCHDOG_ON_CHANNEL0,
    adc::cr1::eocie::END_OF_CONVERSION_INTERRUPT_DISABLED,
    adc::cr1::awdie::ANALOG_WATCHDOG_INTERRUPT_DISABLED,
    adc::cr1::jeocie::END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED,
    adc::cr1::scan::SCAN_MODE_DISABLED,
    adc::cr1::awdsgl::ANALOG_WATCHDOG_ENABLED_ON_ALL_CHANNELS,
    adc::cr1::jauto::AUTOMATIC_INJECTED_CONVERSION_DISABLED,
    adc::cr1::discen::DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED,
    adc::cr1::jdiscen::DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED,
    adc::cr1::discnum::_1_CHANNEL_FOR_DISCONTINUOUS_MODE,
    adc::cr1::jawden::ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS,
    adc::cr1::awden::ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS,
    adc::cr1::res::_12_BITS_RESOLUTION,
    adc::cr1::ovrie::OVERRUN_INTERRUPT_DISABLED,
    adc::cr2::adon::ADC_ENABLED,
    adc::cr2::cont::CONTINUOUS_CONVERSION_MODE,
    adc::cr2::dma::DMA_MODE_DISABLED,
    adc::cr2::dds::NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER,
    adc::cr2::eocs::EOC_BIT_IS_SET_AFTER_A_SEQUENCE_OF_REGULAR_CONVERSIONS,
    adc::cr2::align::RIGTH_ALIGNED_DATA,
    adc::cr2::jextsel::INJECTED_GROUP_TRIGGERED_BY_TIMER1_CC4,
    adc::cr2::jexten::INJECTED_TRIGGER_DISABLED,
    adc::cr2::jswstart::INJECTED_CHANNELS_ON_RESET_STATE,
    adc::cr2::extsel::REGULAR_GROUP_TRIGGERED_,
    adc::cr2::exten::REGULAR_TRIGGER_DISABLED,
    adc::cr2::swstart::REGULAR_CHANNELS_ON_RESET_STATE);
  ADC1::setRegularSequenceOrder<1, 0>();
  ADC1::setNumberOfRegularChannels<1>();
*/
}

void initializePeripherals()
{
  initializeGpio();
  initializeTimer();
  initializeUsart1();
  initializeI2c();

  TIM2::startCounter();
}

void loop()
{
  static u64 timer_t1 = 500;
  if(timer_t1 < tick)
  {
	  timer_t1 = tick + 500;
	  LED::setOutput(LED::isHigh() ? 0 : 1);
	  printf("Hello !!!\n");
  }
}

int main()
{
  clk::initialize();

  initializePeripherals();

  LED::setHigh();

  while (true) {
    loop();
  }
}

void interrupt::TIM2()
{
  TIM2::clearUpdateFlag();
  ++tick;
}

void interrupt::I2C1_EV()
{
	if(I2C1::isAddrMatched())
	{


	}
	else if(I2C1::isStopReceived())
	{


	}
	else if(I2C1::hasReceivedData())
	{


	}
	else if(I2C1::canSendData())
	{


	}


}

void interrupt::I2C1_ER()
{

}

void interrupt::DMA1_Channel1()
{

}
