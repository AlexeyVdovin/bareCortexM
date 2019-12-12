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
#include <string.h>

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/afio.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/usart.hpp"
#include "peripheral/adc.hpp"
#include "peripheral/dma.hpp"
#include "peripheral/tim.hpp"
#include "peripheral/i2c.hpp"
#include "core/stk.hpp"

#define UART1_BAUD_RATE 9600
#define I2C_SLAVE_ADDR  0x10

#define I2C1_REG reinterpret_cast<i2c::Registers*>(i2c::I2C1)

typedef PB8  LED_RED;
typedef PB9  LED_GREEN;

typedef PA0  AD_12V;  // IN0
typedef PA1  AD_BTT;  // IN1
typedef PA2  AD_5V0;  // IN2
typedef PA3  AD_3V3;  // IN3
typedef PB1  AD_VCC;  // IN9
//           AD_TEMP; // IN16
//           AD_REF;  // IN17

typedef DMA1_CHANNEL1 DMA_ADC1;

typedef PA9   U1TX;   // PP
typedef PA10  U1RX;   // IN

typedef PB6   SCL;    // OD
typedef PB7   SDA;    // OD

typedef PA15  OPT_V;  // IN
typedef PB13  PA07;   // OD
typedef PB3   INTB;   // IN
typedef PB4   EXT_RST;// OD
typedef PB5   OPT_EN; // PP
typedef PB15  PWR_EN; // OD

typedef struct
{
  u64 tick;     // 0x00
  u32 status;   // 0x08
  u16 control;  // 0x0a
  s16 adc_12v;  // 0x0c
  s16 adc_btt;  // 0x0e
  s16 adc_5v0;  // 0x10
  s16 adc_3v3;  // 0x12
  s16 adc_vcc;  // 0x14
  s16 adc_ref;  // 0x16
  s16 adc_temp; // 0x18
  u16 res1;     // 0x1a
  u16 res2;     // 0x1c
  u16 res3;     // 0x1e
  u16 nv_adr;   // 0x20
  u16 nv_cmd;   // 0x22
  u16 nv_data;  // 0x24
} REGS;

enum { NUM_DMA_ADC = 7 };

volatile u32 dma_count;
volatile u16 dma_adc_buff[NUM_DMA_ADC];

volatile u64 tick = 0;
volatile u64 i2c1_wd = 0;

volatile u32 dbg_i2c = 0;

volatile REGS regs;

void delay(u32 us)
{
  u32 i, t = us*(clk::AHB/1000/1000);
  for(i=0; i<t; ++i)
    __asm__("nop");
}

void initializeGpio()
{
  GPIOA::enableClock();
  GPIOB::enableClock();
  AFIO::enableClock();

  LED_RED::setHigh();
  LED_RED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  LED_GREEN::setHigh();
  LED_GREEN::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  OPT_EN::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  OPT_EN::setLow();

  PA07::setHigh();
  PA07::setMode(gpio::cr::GP_OPEN_DRAIN_2MHZ);

  EXT_RST::setHigh();
  EXT_RST::setMode(gpio::cr::GP_OPEN_DRAIN_2MHZ);

  PWR_EN::setHigh();
  PWR_EN::setMode(gpio::cr::GP_OPEN_DRAIN_2MHZ);

  INTB::setMode(gpio::cr::FLOATING_INPUT);
  OPT_V::setMode(gpio::cr::FLOATING_INPUT);
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
  // ADC/DMA periodic timer
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt<100 /*Hz*/>();

  // Sys Clock
  STK::configurePeriodicInterrupt(1000 /*Hz*/);
}

void initializeI2c()
{
  i2c1_wd = 0;
  I2C1::enableClock(); // Uses APB1 clock
  SCL::setMode(gpio::cr::AF_OPEN_DRAIN_2MHZ);
  SDA::setMode(gpio::cr::AF_OPEN_DRAIN_2MHZ);

  I2C1::configureI2c(
    i2c::cr1::pe::PERIPHERAL_ENABLED,
    i2c::cr1::engc::GENERAL_CALL_DISABLED,
    i2c::cr1::nostretch::CLOCK_STRETCHING_ENABLED,
    i2c::cr1::ack::ACKNOWLEDGE_ENABLED,
    i2c::cr2::iterren::ERROR_INTERRUPT_ENABLED,
    i2c::cr2::itevten::EVENT_INTERRUPT_ENABLED,
    i2c::cr2::itbufen::BUFFER_INTERRUPT_ENABLED,
    i2c::cr2::dmaen::DMA_REQUEST_DISABLED,
    i2c::cr2::last::NEXT_DMA_IS_NOT_THE_LAST_TRANSFER);
  I2C1::configureClock<i2c::ccr::f_s::STANDARD_MODE, i2c::ccr::duty::T_LOW_2_T_HIGH_1, 100000 /*Hz*/>();
  I2C1::setSlave7BitAddr1(I2C_SLAVE_ADDR);
  I2C1::enableACK();

  I2C1::unmaskInterrupts();
}

void initializeAdc()
{
  ADC1::enableClock();
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
    adc::cr1::dualmod::INDEPENDENT_MODE,
    adc::cr1::jawden::ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS,
    adc::cr1::awden::ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS,
    adc::cr2::adon::ADC_ENABLED,
    adc::cr2::cont::SINGLE_CONVERSION_MODE,
    adc::cr2::dma::DMA_MODE_ENABLED,
    adc::cr2::align::RIGTH_ALIGNED_DATA,
    adc::cr2::jextsel::INJECTED_GROUP_TRIGGERED_BY_TIMER1_TRGO,
    adc::cr2::jexten::INJECTED_TRIGGER_DISABLED,
    adc::cr2::jswstart::INJECTED_CHANNELS_ON_RESET_STATE,
    adc::cr2::extsel::REGULAR_GROUP_TRIGGERED_BY_SWSTART,
    adc::cr2::exten::REGULAR_TRIGGER_ENABLED,
    adc::cr2::swstart::START_CONVERSION_ON_REGULAR_CHANNELS,
    adc::cr2::tsvrefe::TEMPERATURE_SENSOR_ENABLED);

  ADC1::setConversionTime<0, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<1, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<2, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<3, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<4, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<5, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<6, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<7, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<8, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<9, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<10, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<11, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<12, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<13, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<14, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<15, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<16, adc::smp::SAMPLING_TIME_71_5_CYCLES>();
  ADC1::setConversionTime<17, adc::smp::SAMPLING_TIME_71_5_CYCLES>();

  ADC1::setRegularSequenceOrder<1, 0>();  // AD_12V
  ADC1::setRegularSequenceOrder<2, 1>();  // AD_BTT
  ADC1::setRegularSequenceOrder<3, 2>();  // AD_5V0
  ADC1::setRegularSequenceOrder<4, 3>();  // AD_3V3
  ADC1::setRegularSequenceOrder<5, 9>();  // AD_VCC
  ADC1::setRegularSequenceOrder<6, 16>(); // AD_TEMP
  ADC1::setRegularSequenceOrder<7, 17>(); // AD_REF

  ADC1::setNumberOfRegularChannels<NUM_DMA_ADC>();

  AD_12V::setMode(gpio::cr::ANALOG_INPUT);
  AD_BTT::setMode(gpio::cr::ANALOG_INPUT);
  AD_5V0::setMode(gpio::cr::ANALOG_INPUT);
  AD_3V3::setMode(gpio::cr::ANALOG_INPUT);
  AD_VCC::setMode(gpio::cr::ANALOG_INPUT);

  ADC1::disablePeripheral();
  delay(10);
  ADC1::enablePeripheral();
  ADC1::resetCalibration();
  while(!ADC1::hasCalibrationInitialized()) delay(10);
  ADC1::startCalibration();
  while(!ADC1::hasCalibrationEnded()) delay(10);
}

void initializeDma()
{
  dma_count = 0;
  memset((void*)dma_adc_buff, 0, sizeof(dma_adc_buff));

  DMA_ADC1::enableClock();
  DMA_ADC1::configure(
      dma::channel::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_ENABLED,
      dma::channel::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::channel::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::channel::cr::dir::READ_FROM_PERIPHERAL,
      dma::channel::cr::circ::CIRCULAR_MODE_DISABLED,
      dma::channel::cr::pinc::PERIPHERAL_INCREMENT_MODE_DISABLED,
      dma::channel::cr::minc::MEMORY_INCREMENT_MODE_ENABLED,
      dma::channel::cr::psize::PERIPHERAL_SIZE_16BITS,
      dma::channel::cr::msize::MEMORY_SIZE_16BITS,
      dma::channel::cr::pl::CHANNEL_PRIORITY_LEVEL_LOW,
      dma::channel::cr::mem2mem::MEMORY_TO_MEMORY_MODE_DISABLED);
  DMA_ADC1::setMemoryAddress((void*)dma_adc_buff);
  DMA_ADC1::setNumberOfTransactions(NUM_DMA_ADC);
  DMA_ADC1::setPeripheralAddress(&ADC1_REGS->DR);
  DMA_ADC1::enablePeripheral();
  DMA_ADC1::unmaskInterrupts();
}

void initializePeripherals()
{
  initializeGpio();
  initializeTimer();
  initializeI2c();
  initializeUsart1();
  initializeAdc();
  initializeDma();

  TIM2::startCounter();
}

#if 0
u16 sqrt32(u32 n)
{
  u16 c = 0x8000;
  u16 g = 0x8000;

  for(;;)
  {
    if(g*g > n) g ^= c;
    c >>= 1;
    if(c == 0) return g;
    g |= c;
  }
}
#endif

void loop()
{
  static u64 timer_t1 = 1000;
  if(tick > timer_t1)
  {
    timer_t1 = tick + 500;
    LED_RED::setOutput(LED_RED::isHigh() ? 0 : 1);

    printf("[%02x] SCL:%d, SDA:%d\n", dbg_i2c, SCL::getInput(), SDA::getInput());
  }
  // if(SCL::getInput() == 0 || SDA::getInput() == 0) printf("SCL:%d, SDA:%d\n", SCL::getInput(), SDA::getInput());
}

int main()
{
  clk::initialize();

  initializePeripherals();



  while (true) {
    loop();
    if(0 && i2c1_wd != 0 && i2c1_wd < tick)
    {
    	I2C1::disablePeripheral();
    	I2C1::reset();
    	initializeI2c();
    	printf("Reset I2C!\n");
    }
  }
}

/*
 * I2C interrupts should have the highest priority in the application
 * in order to make them uninterruptible.
 *
 * After checking the SR1 register content, the user should perform the complete clearing sequence
 * for each flag found set.
 * Thus, for ADDR and STOPF flags, the following sequence is required inside the I2C interrupt
 * routine:
 *   READ SR1
 *   if (ADDR == 1) {READ SR1; READ SR2}
 *   if (STOPF == 1) {READ SR1; WRITE CR1}
 *
 * The purpose is to make sure that both ADDR and STOPF flags are cleared if both are found set.
 */

enum
{
  I2C_IDLE = 0,
  I2C_SLAVE_WRITE_ADDR,
  I2C_SLAVE_WRITE_DATA,
  I2C_SLAVE_READ_DATA
};


void interrupt::I2C1_EV()
{
	static int mode = I2C_IDLE;
	static u8 addr = 0;
	u32 sr1, sr2, cr1;

	dbg_i2c = 0x22;

	if(I2C1::isAddrMatched())
	{
		dbg_i2c = 1;
		i2c1_wd = tick+3000;
		mode = I2C1::isSlaveTransmitting() ? I2C_SLAVE_READ_DATA : I2C_SLAVE_WRITE_ADDR;
		sr1 = I2C1_REG->SR1;
		sr2 = I2C1_REG->SR2;
	}
	else if(I2C1::isStopReceived())
	{
		dbg_i2c = 9;
		mode = I2C_IDLE;
		I2C1::enablePeripheral();
		cr1 = I2C1_REG->CR1;
		I2C1_REG->CR1 = cr1;
		i2c1_wd = 0;
	}
	else if(I2C1::hasReceivedData())
	{
		dbg_i2c = 3;
		if(mode == I2C_SLAVE_WRITE_ADDR)
		{
			addr = I2C1::getData();
			mode = I2C_SLAVE_WRITE_DATA;
		}
		else if(mode == I2C_SLAVE_WRITE_DATA)
		{
			I2C1::getData();
		}
		else
		{
			// Incorrect state, ignore data
			I2C1::getData();
		}
	}
	else if(I2C1::canSendData())
	{
		dbg_i2c = 5;
		if(mode == I2C_SLAVE_READ_DATA)
		{
			I2C1::sendData(0xac);
		}
		else
		{
			// Incorrect state, send bad data
			I2C1::sendData(0xbd);
		}
	}
	else if(I2C1::isNakReceived())
	{
		dbg_i2c = 0x11;
		I2C1::clearNAK();
	}


}

void interrupt::I2C1_ER()
{

}


void interrupt::TIM2()
{
  TIM2::clearUpdateFlag();
  // Start new conversion
  DMA_ADC1::enablePeripheral();
  ADC1::enablePeripheral();
}

void interrupt::DMA1_Channel1()
{
  ++dma_count;
  DMA_ADC1::clearGlobalFlag();

  DMA_ADC1::disablePeripheral();
  DMA_ADC1::setNumberOfTransactions(NUM_DMA_ADC);
}

extern "C" void SysTick()
{
  ++tick;
}
