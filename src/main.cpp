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

#define UART1_BAUD_RATE 115200
#define I2C_SLAVE_ADDR  0x50

typedef PA9  U1TX;
typedef PA10 U1RX;
typedef PB9  LED;


typedef PA0  IN4C;
typedef PA1  IN3C;
typedef PA2  IN2C;
typedef PA3  IN1C;
typedef PA4  IN4V;
typedef PA5  IN3V;
typedef PA6  IN2V;
typedef PA7  IN1V;

typedef PB0  VIN;
typedef PB1  VDIV;



typedef DMA1_CHANNEL1 DMA_ADC1;

volatile u64 tick = 0;

void initializeGpio()
{
  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();

  AFIO::enableClock();

  LED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  IN1C::setMode(gpio::cr::ANALOG_INPUT);
  IN1V::setMode(gpio::cr::ANALOG_INPUT);
  IN2C::setMode(gpio::cr::ANALOG_INPUT);
  IN2V::setMode(gpio::cr::ANALOG_INPUT);
  IN3C::setMode(gpio::cr::ANALOG_INPUT);
  IN3V::setMode(gpio::cr::ANALOG_INPUT);
  IN4C::setMode(gpio::cr::ANALOG_INPUT);
  IN4V::setMode(gpio::cr::ANALOG_INPUT);
  VIN::setMode(gpio::cr::ANALOG_INPUT);
  VDIV::setMode(gpio::cr::ANALOG_INPUT);
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
	// TODO: Switch to SysTimer
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt< 1000 /* Hz */ >();
}

void initializeI2c()
{
  I2C1::enableClock(); // Uses APB1 clock
  I2C1::configure(
    i2c::cr1::pe::PERIPHERAL_DISABLED,
    i2c::cr1::enpec::PACKET_ERROR_CHECKING_DISABLED,
    i2c::cr1::engc::GENERAL_CALL_DISABLED,
    i2c::cr1::nostretch::CLOCK_STRETCHING_ENABLED,
    i2c::cr2::iterren::ERROR_INTERRUPT_ENABLED,
    i2c::cr2::itevten::EVENT_INTERRUPT_ENABLED,
    i2c::cr2::itbufen::BUFFER_INTERRUPT_ENABLED,
    i2c::cr2::dmaen::DMA_REQUEST_DISABLED,
    i2c::cr2::last::NEXT_DMA_IS_NOT_THE_LAST_TRANSFER);
  I2C1::configureClock<
    i2c::ccr::f_s::STANDARD_MODE,
    i2c::ccr::duty::T_LOW_2_T_HIGH_1,
    100000 /* Hz */ >();
  I2C1::setSlaveAddr1(I2C_SLAVE_ADDR);
  I2C1::setSlaveAddr2(0);
  I2C1::enablePeripheral();

  I2C1::unmaskInterrupts();
}

/*
 ADC2  ADC1
------------
In1 V, A - A7, A3
In2 V, A - A6, A2
In3 V, A - A5, A1
In4 V, A - A4, A0
VIn V, Temp - A8, A16
Vcc/2, Vref - A9, A17

- accumulate sum of V*V and number of samples
- every second divide accumulated value by number of samples and get SQRT of it.
*/
void initializeAdc()
{
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
    adc::cr1::dualmod::DUALMODE_REGSIMULT_MODE,
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
  ADC2::configure(
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
    adc::cr2::dma::DMA_MODE_DISABLED,
    adc::cr2::align::RIGTH_ALIGNED_DATA,
    adc::cr2::jextsel::INJECTED_GROUP_TRIGGERED_BY_TIMER1_CC4,
    adc::cr2::jexten::INJECTED_TRIGGER_DISABLED,
    adc::cr2::jswstart::INJECTED_CHANNELS_ON_RESET_STATE,
    adc::cr2::extsel::REGULAR_GROUP_TRIGGERED_BY_SWSTART,
    adc::cr2::exten::REGULAR_TRIGGER_ENABLED,
    adc::cr2::swstart::START_CONVERSION_ON_REGULAR_CHANNELS,
    adc::cr2::tsvrefe::TEMPERATURE_SENSOR_ENABLED);

  ADC1::setConversionTime<0, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<1, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<2, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<3, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<4, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<5, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<6, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<7, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<8, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<9, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<10, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<11, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<12, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<13, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<14, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<15, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<16, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC1::setConversionTime<17, adc::smp::SAMPLING_TIME_13_5_CYCLES>();

  ADC2::setConversionTime<0, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<1, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<2, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<3, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<4, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<5, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<6, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<7, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<8, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<9, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<10, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<11, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<12, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<13, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<14, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<15, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<16, adc::smp::SAMPLING_TIME_13_5_CYCLES>();
  ADC2::setConversionTime<17, adc::smp::SAMPLING_TIME_13_5_CYCLES>();

  ADC1::setRegularSequenceOrder<1, 3>();
  ADC1::setRegularSequenceOrder<2, 2>();
  ADC1::setRegularSequenceOrder<3, 1>();
  ADC1::setRegularSequenceOrder<4, 0>();
  ADC1::setRegularSequenceOrder<5, 16>();
  ADC1::setRegularSequenceOrder<6, 17>();

  ADC1::setNumberOfRegularChannels<6>();

  ADC2::setRegularSequenceOrder<1, 7>();
  ADC2::setRegularSequenceOrder<2, 6>();
  ADC2::setRegularSequenceOrder<3, 5>();
  ADC2::setRegularSequenceOrder<4, 4>();
  ADC2::setRegularSequenceOrder<5, 9>();
  ADC2::setRegularSequenceOrder<6, 8>();

  ADC2::setNumberOfRegularChannels<6>();
  /*
   ADC2  ADC1
  ------------
  In1 V, A - A7, A3
  In2 V, A - A6, A2
  In3 V, A - A5, A1
  In4 V, A - A4, A0
  VIn V, Temp - A8, A16
  Vcc/2, Vref - A9, A17
*/
  ADC1::disablePeripheral();
  // TODO: replace to usleep()
  for(int i = 0; i < 100000; ++i) {}
  ADC1::enablePeripheral();
  ADC1::resetCalibration();
  // TODO: Add timeout ??
  while(!ADC1::hasCalibrationInitialized()) {}
  ADC1::startCalibration();
  // TODO: Add timeout ??
  while(!ADC1::hasCalibrationEnded()) {}

  ADC2::disablePeripheral();
  // TODO: replace to usleep()
  for(int i = 0; i < 100000; ++i) {}
  ADC2::enablePeripheral();
  ADC2::resetCalibration();
  // TODO: Add timeout ??
  while(!ADC2::hasCalibrationInitialized()) {}
  ADC2::startCalibration();
  // TODO: Add timeout ??
  while(!ADC2::hasCalibrationEnded()) {}
}

#define NUM_DMA_ADC 6
#define NUM_V_CH 5
#define NUM_P_CH 4

volatile u32 dma_count;
volatile u32 dma_adc_buff[NUM_DMA_ADC];
volatile s64 rms_ch[NUM_V_CH];
volatile s64 rms_pw[NUM_P_CH];
volatile s64 energy[NUM_P_CH];
volatile s16 power[NUM_P_CH];
volatile u16 voltage[NUM_V_CH];

enum {
	In1_C = 0,
	In1_V,
	In2_C,
	In2_V,
	In3_C,
	In3_V,
	In4_C,
	In4_V,
	Temp_C,
	Div_V,
	Int_V,
	In_V
};

float kv[] = { 1.0, 1.0, 1.0, 1.0, 1.0 };
float kp[] = { 1.0, 1.0, 1.0, 1.0 };

void initializeDma()
{
  dma_count = 0;
  memset((void*)dma_adc_buff, 0, sizeof(dma_adc_buff));
  memset((void*)rms_ch, 0, sizeof(rms_ch));

  DMA_ADC1::enableClock();
  DMA_ADC1::configure(
      dma::channel::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_ENABLED,
      dma::channel::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::channel::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::channel::cr::dir::READ_FROM_PERIPHERAL,
      dma::channel::cr::circ::CIRCULAR_MODE_DISABLED,
      dma::channel::cr::pinc::PERIPHERAL_INCREMENT_MODE_DISABLED,
      dma::channel::cr::minc::MEMORY_INCREMENT_MODE_ENABLED,
      dma::channel::cr::psize::PERIPHERAL_SIZE_32BITS,
      dma::channel::cr::msize::MEMORY_SIZE_32BITS,
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

void loop()
{
  static u64 timer_t1 = 1000;
  if(timer_t1 < tick)
  {
	s64 rms[NUM_V_CH];
	s64 pw[NUM_P_CH];

	u16 n = (u16)dma_count; dma_count = 0;
    memcpy(rms, (const void*)rms_ch, sizeof(rms));
    memcpy(pw, (const void*)rms_pw, sizeof(pw));
    memset((void*)rms_ch, 0, sizeof(rms_ch));
    memset((void*)rms_pw, 0, sizeof(rms_pw));

    if(dma_count != 0) printf("Error: DMA Overlap !!!\n");

    voltage[0] = (u16)(sqrt32(rms[0]/n) * kv[0]);
    voltage[1] = (u16)(sqrt32(rms[1]/n) * kv[1]);
    voltage[2] = (u16)(sqrt32(rms[2]/n) * kv[2]);
    voltage[3] = (u16)(sqrt32(rms[3]/n) * kv[3]);
    voltage[4] = (u16)(sqrt32(rms[4]/n) * kv[4]);
    power[0] = (s16)(pw[0]/n * kp[0]);
    power[1] = (s16)(pw[1]/n * kp[1]);
    power[2] = (s16)(pw[2]/n * kp[2]);
    power[3] = (s16)(pw[3]/n * kp[3]);
    energy[0] += power[0];
    energy[1] += power[1];
    energy[2] += power[2];
    energy[3] += power[3];

    printf("Vin: %dv\nV1: %dv\nV2: %dv\nV3: %dv\nV4: %dv\n", voltage[4],
    		voltage[0], voltage[1], voltage[2], voltage[3]);
    printf("P1: %dw\nP2: %dw\nP3: %dw\nP4: %dw\n",
    		power[0], power[1], power[2], power[3]);
    timer_t1 = tick + 500;
    LED::setOutput(LED::isHigh() ? 0 : 1);
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
  DMA_ADC1::enablePeripheral();
  ADC2::enablePeripheral(); // Start conversion
  ADC1::enablePeripheral(); // Start conversion
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
  u16* adcv = (u16*)dma_adc_buff;
  register s16 v, d;

  ++dma_count;
  DMA_ADC1::clearGlobalFlag();

  d = adcv[Div_V];

  v = adcv[In1_V]-d;
  rms_ch[0] += v*v;
  rms_pw[0] += v * (adcv[In1_C]-d);

  v = adcv[In2_V]-d;
  rms_ch[1] += v*v;
  rms_pw[1] += v * (adcv[In2_C]-d);

  v = adcv[In3_V]-d;
  rms_ch[2] += v*v;
  rms_pw[2] += v * (adcv[In3_C]-d);

  v = adcv[In4_V]-d;
  rms_ch[3] += v*v;
  rms_pw[3] += v * (adcv[In4_C]-d);

  v = adcv[In_V]-d;
  rms_ch[4] += v*v;

  DMA_ADC1::disablePeripheral();
  DMA_ADC1::setNumberOfTransactions(NUM_DMA_ADC);
}
