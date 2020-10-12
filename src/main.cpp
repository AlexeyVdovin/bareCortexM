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
#include "peripheral/crc.hpp"
#include "peripheral/flash.hpp"
#include "peripheral/usart.hpp"
#include "peripheral/adc.hpp"
#include "peripheral/dma.hpp"
#include "peripheral/tim.hpp"
#include "peripheral/i2c.hpp"
#include "core/stk.hpp"

#define UART1_BAUD_RATE 9600
#define I2C_SLAVE_ADDR  0x30


#define I2C1_REG reinterpret_cast<i2c::Registers*>(i2c::I2C1)

typedef PB9  LED_RED;
// typedef PB9  LED_GREEN;

typedef PA0  AD_AC1;  //
typedef PA1  AD_AC3;  //
typedef PA2  AD_AC7;  //
typedef PA3  AD_AC5;  //
typedef PA4  AD_MAIN; // AC_MAIN
typedef PA5  AD_AC8;  //
typedef PA6  AD_AC6;  //
typedef PA7  AD_AC4;  //
typedef PB0  AD_AC2;  //

typedef PB1  AD_12V;  // DC IN
//           AD_TEMP; // IN16
//           AD_REF;  // IN17

typedef DMA1_CHANNEL1 DMA_ADC1;

typedef PA9   U1TX;   // PP
typedef PA10  U1RX;   // IN

typedef PB6   SCL;    // OD
typedef PB7   SDA;    // OD

typedef PB13  ADR0;   // IN
typedef PB14  ADR1;   // IN
typedef PB15  ADR2;   // IN

typedef PB5   I2C_INT; // IN

enum {
	NV_CMD_READ  = 0x10000000,
	NV_CMD_WRITE = 0x20000000,
	NV_CMD_SAVE  = 0x40000000,
	NV_CMD_ADDR  = 0x0FFFFFFF
};

enum {
	CTRL_EXT_RST	= 0x8000,
	CTRL_OPT_EN		= 0x0100,
	CTRL_LED_ON		= 0x0001
};

typedef struct
{
  s64 tick;     // 0x00
  u32 status;   // 0x08
  u16 control;  // 0x0c
  s16 adc_12v;  // 0x0e
  s16 ac_main;  // 0x10
  s16 pwr[8];   // 0x12
  s32 enr[8];   // 0x22
  s16 adc_ref;  // 0x42
  s16 adc_temp; // 0x44
  u16 res1;     // 0x46
  u16 res2;     // 0x48
  u16 res3;     // 0x4a
  u16 res4;     // 0x4c
  u16 res5;     // 0x4e
  u32 nv_cmd;   // 0x50 NV_CMD_****
  u32 nv_data;  // 0x54
  u8 count[4];  // 0x58
} REGS;

typedef struct
{
	s16	k; // k in 1/8192
	s16 c; // value = k * RAW + c
} kv_t;

typedef struct
{
  u16 i2c_id;
  u16 node_id;
  kv_t kv_12v;
  kv_t kv_ac;
  kv_t kv_in[8];
  kv_t kv_ref;
  kv_t kv_temp;
  u32 crc;
} CONF;


enum { NUM_DMA_ADC = 6 };

extern const u32 __flash_start;
extern const u32 __flash_end;

const u32 flash_start = (u32)(&__flash_start);
const u32 flash_end = (u32)(&__flash_end);

volatile u32 dma_count;
volatile u16 dma_adc_buff[NUM_DMA_ADC*2];

volatile s64 tick = 0;
volatile s64 i2c1_wd = 0;

volatile REGS regs;
volatile CONF conf; // Copy from nv_conf
CONF* conf1;
CONF* conf2;
volatile u16 control;

volatile s32 ref_sum = 0;
volatile s16 ref_avg = 2048;

// Power metering
volatile s64 rms_main;   // MAIN RMS
volatile s64 rms_pwr[8]; // CH power RMS
volatile s32 energy[8];  // CH energy W/sec
volatile s16 power[8];   // CH power W
volatile s16 ac_main;    // AC V

inline void* align_ptr(void* adr, u32 align)
{
  return (void*)((1 + ((u32)adr - 1) / align) * align);
}

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
}

void conf_write(CONF* dst, volatile CONF* src)
{
  u32 c, i, *p;
  u16 *d, *s;

  p = (u32*)src;
  CRC::reset();
  for(i = 0; i < (sizeof(CONF)/sizeof(u32)-1); ++i)
    CRC::calc(*(p+i));
  src->crc = CRC::getCrc();

  do
  {
    s = (u16*)src;
    d = (u16*)dst;
    FLASH::unlock();
    FLASH::erasePage((u32)dst);
    for(c = 0; c < 100; c++)
    {
      if(!FLASH::isBusy()) { c = 0; break; }
      delay(100);
    }
    if(c > 0) { printf("Error: Flash erase 0x%08X!\n", dst); break; }
    for(i = 0; i < (sizeof(CONF)/sizeof(u16)); ++i)
    {
      FLASH::programm((u32)(d+i), *s+i);
      for(c = 0; c < 10; c++)
      {
  	    if(!FLASH::isBusy()) { c = 0; break; }
        delay(100);
      }
      if(c > 0) { printf("Error: Flash write 0x%08X!\n", d+i); break; }
    }
    if(c > 0) break;
    if(memcmp((void*)src, dst, sizeof(CONF)) != 0) { printf("Error: Flash verification failed 0x%08X!\n", dst); break; }
    printf("Config 0x%08X updated.\n", dst);
  } while(0);
  FLASH::lock();
}

void initializeConf()
{
  u32 i;
  u32* p;
  bool c1 = false;
  bool c2 = false;

  conf1 = (CONF*)align_ptr((void*)(flash_end-1024), 1024);
  conf2 = (CONF*)align_ptr((void*)(flash_end-2048), 1024);

  CRC::enableClock();

  p = (u32*)conf1;
  CRC::reset();
  for(i = 0; i < (sizeof(CONF)/sizeof(u32)-1); ++i)
	  CRC::calc(*(p+i));
  if(conf1->crc == CRC::getCrc())
  {
	memcpy((void*)&conf, conf1, sizeof(CONF));
	c1 = true;
	printf("Conf1 CRC match.\n");
  }

  p = (u32*)conf2;
  CRC::reset();
  for(i = 0; i < (sizeof(CONF)/sizeof(u32)-1); ++i)
	  CRC::calc(*(p+i));
  if(conf2->crc == CRC::getCrc())
  {
	if(c1 == false) memcpy((void*)&conf, conf2, sizeof(CONF));
	c2 = true;
	printf("Conf2 CRC match.\n");
  }

  if(c1 && (c2 == false || conf2->crc != conf1->crc))
  {
	// Save conf to conf2 Flash
	  conf_write(conf2, &conf);
  }
  if(c2 && c1 == false)
  {
	// Save conf to conf1 Flash
	  conf_write(conf1, &conf);
  }
  if(c1 == false && c2 == false)
  {
    printf("Initialize conf with default values.\n");
    conf.i2c_id = 0x10;
    conf.node_id = 0x3FFF;
    conf.kv_12v.k = 26460;
    conf.kv_12v.c = 0;
    conf.kv_ac.k = 256;
    conf.kv_ac.c = 0;
    conf.kv_in[0].k = 256;
    conf.kv_in[0].c = 0;
    conf.kv_in[1].k = 256;
    conf.kv_in[1].c = 0;
    conf.kv_in[2].k = 256;
    conf.kv_in[2].c = 0;
    conf.kv_in[3].k = 256;
    conf.kv_in[3].c = 0;
    conf.kv_in[4].k = 256;
    conf.kv_in[4].c = 0;
    conf.kv_in[5].k = 256;
    conf.kv_in[5].c = 0;
    conf.kv_in[6].k = 256;
    conf.kv_in[6].c = 0;
    conf.kv_in[7].k = 256;
    conf.kv_in[7].c = 0;
    conf.kv_ref.k = 8192;
    conf.kv_ref.c = 0;
    conf.kv_temp.k = 8192; // TODO: Set correct default Temperature conversion constants
    conf.kv_temp.c = 0;
  }

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
    i2c::cr2::iterren::ERROR_INTERRUPT_DISABLED,
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

  ADC1::setRegularSequenceOrder<1, 0>();
  ADC1::setRegularSequenceOrder<2, 2>();
  ADC1::setRegularSequenceOrder<3, 5>();
  ADC1::setRegularSequenceOrder<4, 7>();
  ADC1::setRegularSequenceOrder<5, 4>();  // AC MAIN
  ADC1::setRegularSequenceOrder<6, 16>(); // AD_TEMP

  ADC1::setNumberOfRegularChannels<NUM_DMA_ADC>();

  ADC2::setRegularSequenceOrder<1, 1>();
  ADC2::setRegularSequenceOrder<2, 3>();
  ADC2::setRegularSequenceOrder<3, 6>();
  ADC2::setRegularSequenceOrder<4, 8>();
  ADC2::setRegularSequenceOrder<5, 9>();  // DC 12V
  ADC2::setRegularSequenceOrder<6, 17>(); // AD_REF

  ADC2::setNumberOfRegularChannels<NUM_DMA_ADC>();

  AD_12V::setMode(gpio::cr::ANALOG_INPUT);
  AD_MAIN::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC1::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC2::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC3::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC4::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC5::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC6::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC7::setMode(gpio::cr::ANALOG_INPUT);
  AD_AC8::setMode(gpio::cr::ANALOG_INPUT);

  ADC1::disablePeripheral();
  delay(10);
  ADC1::enablePeripheral();
  ADC1::resetCalibration();
  while(!ADC1::hasCalibrationInitialized()) delay(10);
  ADC1::startCalibration();
  while(!ADC1::hasCalibrationEnded()) delay(10);

  ADC2::disablePeripheral();
  delay(10);
  ADC2::enablePeripheral();
  ADC2::resetCalibration();
  while(!ADC2::hasCalibrationInitialized()) delay(10);
  ADC2::startCalibration();
  while(!ADC2::hasCalibrationEnded()) delay(10);
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
  initializeUsart1();
  initializeConf();
  initializeTimer();
  initializeI2c();
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

/*
 * 	CTRL_EXT_RST	= 0x8000,
	CTRL_OPT_EN		= 0x0100,
	CTRL_LED_ON		= 0x0001
 *
 */

void loop()
{
  static s64 timer_t1 = 1000;
  s64 rms;
  s64 pw[8];
  s32 ref;
  int i;

  if(tick > timer_t1)
  {
    timer_t1 = tick + 500;
    LED_RED::setOutput(LED_RED::isHigh() ? 0 : 1);

	u16 n = (u16)dma_count; dma_count = 0;
    rms = rms_main; rms_main = 0;
    ref = ref_sum; ref_sum = 0;
    memcpy(pw, (const void*)rms_pwr, sizeof(pw));
    memset((void*)rms_pwr, 0, sizeof(rms_pwr));

    if(dma_count != 0) printf("Error: DMA Overlap !!!\n");

    ac_main = (s16)(sqrt32(rms/n) * conf.kv_ac.k/256 + conf.kv_ac.c);
    ref_avg = ref/n/8;
    for(i = 0; i < 8; ++i)
    {
    	power[i] = (s16)(pw[i]/n * conf.kv_in[i].k/256 + conf.kv_in[i].c);
    	energy[i] += power[i]/2; // loop every 500ms
    }
  }
}

int main()
{
  clk::initialize();

  memset((void*)&regs, 0, sizeof(regs));
  initializePeripherals();

  printf("conf1: 0x%08x\n", conf1);
  printf("conf2: 0x%08x\n", conf2);

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
	u8* data = (addr >= 0x80)?(u8*)&conf:(u8*)&regs;
	u8 a = (addr >= 0x80)?addr-0x80:addr;
	volatile u32 sr1, sr2, cr1;

	if(I2C1::isAddrMatched())
	{
		i2c1_wd = tick+3000;
		mode = I2C1::isSlaveTransmitting() ? I2C_SLAVE_READ_DATA : I2C_SLAVE_WRITE_ADDR;
		sr1 = I2C1_REG->SR1;
		sr2 = I2C1_REG->SR2;
	}
	else if(I2C1::isStopReceived())
	{
		mode = I2C_IDLE;
		I2C1::enablePeripheral();
		cr1 = I2C1_REG->CR1;
		I2C1_REG->CR1 = cr1;
		i2c1_wd = 0;
	}
	else if(I2C1::hasReceivedData())
	{
		if(mode == I2C_SLAVE_WRITE_ADDR)
		{
			addr = I2C1::getData();
			mode = I2C_SLAVE_WRITE_DATA;
		}
		else if(mode == I2C_SLAVE_WRITE_DATA && a < ((addr >= 0x80)?sizeof(conf):sizeof(regs)))
		{
			*(data+a) = I2C1::getData();
			addr++;
		}
		else
		{
			// Incorrect state, ignore data
			cr1 = I2C1::getData();
			(void)cr1;
		}
	}
	else if(I2C1::canSendData())
	{
		if(mode == I2C_SLAVE_READ_DATA && addr < a < ((addr >= 0x80)?sizeof(conf):sizeof(regs)))
		{
			if(addr == 0) regs.tick = tick;
 			I2C1::sendData(*(data+a));
			addr++;
		}
		else
		{
			// Incorrect state, send bad data
			I2C1::sendData(0xbd);
		}
	}
	else if(I2C1::isNakReceived())
	{
		I2C1::clearNAK();
	}
}

void interrupt::I2C1_ER()
{
  // TODO: Add error handling and recovery
}


void interrupt::TIM2()
{
  TIM2::clearUpdateFlag();
}

void interrupt::DMA1_Channel1()
{
  register s16 v, c;
  int i;

  ++dma_count;
  DMA_ADC1::clearGlobalFlag();

  v = dma_adc_buff[8] - ref_avg;
  rms_main += v * v;

  for(i = 0; i < 8; ++i)
  {
    c = dma_adc_buff[i] - ref_avg;
    rms_pwr[i] += v * c;
    ref_sum += c;
  }

  regs.adc_12v = (s16)((s32)conf.kv_12v.k * dma_adc_buff[0]/2048 + conf.kv_12v.c);
  regs.adc_temp = (s16)((s32)conf.kv_temp.k * dma_adc_buff[5]/8192 + conf.kv_temp.c);
  regs.adc_ref = (s16)((s32)conf.kv_ref.k * dma_adc_buff[6]/8192 + conf.kv_ref.c);

  DMA_ADC1::disablePeripheral();
  DMA_ADC1::setNumberOfTransactions(NUM_DMA_ADC);
}

extern "C" void SysTick()
{
  ++tick;
  // Start new conversion
  DMA_ADC1::enablePeripheral();
  ADC2::enablePeripheral();
  ADC1::enablePeripheral();
}
