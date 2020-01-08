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

enum {
	NV_CMD_READ  = 0x10000000,
	NV_CMD_WRITE = 0x20000000,
	NV_CMD_SAVE  = 0x40000000,
	NV_CMD_ADDR  = 0x0FFFFFFF
};

typedef struct
{
  u64 tick;     // 0x00
  u32 status;   // 0x08
  u16 control;  // 0x0c
  s16 adc_12v;  // 0x0e
  s16 adc_btt;  // 0x10
  s16 adc_5v0;  // 0x12
  s16 adc_3v3;  // 0x14
  s16 adc_vcc;  // 0x16
  s16 adc_ref;  // 0x18
  s16 adc_temp; // 0x1a
  u16 res1;     // 0x1c
  u16 res2;     // 0x1e
  u32 nv_cmd;   // 0x20 NV_CMD_****
  u32 nv_data;  // 0x24
} REGS;

typedef struct
{
  u16 i2c_id;
  u16 node_id;
  s16 k_12v;   // k in 1/8192
  s16 c_12v;   // value = k*dma + c
  s16 k_btt;
  s16 c_btt;
  s16 k_5v0;
  s16 c_5v0;
  s16 k_3v3;
  s16 c_3v3;
  s16 k_vcc;
  s16 c_vcc;
  s16 k_ref;
  s16 c_ref;
  s16 k_temp;
  s16 c_temp;
  s16 lo_btt;
  s16 reserved;
  u32 crc;
} CONF;


enum { NUM_DMA_ADC = 7 };

extern const u32 __flash_start;
extern const u32 __flash_end;

const u32 flash_start = (u32)(&__flash_start);
const u32 flash_end = (u32)(&__flash_end);

volatile u32 dma_count;
volatile u16 dma_adc_buff[NUM_DMA_ADC];

volatile u64 tick = 0;
volatile u64 i2c1_wd = 0;

volatile u32 dbg_i2c = 0;

volatile REGS regs;
volatile CONF conf; // Copy from nv_conf
CONF* conf1;
CONF* conf2;

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
    conf.k_12v = 8192;
    conf.c_12v = 0;
    conf.k_btt = 8192;
    conf.c_btt = 0;
    conf.k_5v0 = 8192;
    conf.c_5v0 = 0;
    conf.k_3v3 = 8192;
    conf.c_3v3 = 0;
    conf.k_vcc = 8192;
    conf.c_vcc = 0;
    conf.k_ref = 8192;
    conf.c_ref = 0;
    conf.k_temp = 8192; // TODO: Set correct default Temperature conversion constants
    conf.c_temp = 0;
    conf.lo_btt = 10800;
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
  initializeUsart1();
  initializeConf();
  initializeTimer();
  initializeI2c();
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

    // printf("[%02x] SCL:%d, SDA:%d\n", dbg_i2c, SCL::getInput(), SDA::getInput());
  }
  // if(SCL::getInput() == 0 || SDA::getInput() == 0) printf("SCL:%d, SDA:%d\n", SCL::getInput(), SDA::getInput());
}

int main()
{
  clk::initialize();

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
	u8* data = (u8*)&regs;
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
		else if(mode == I2C_SLAVE_WRITE_DATA && addr < sizeof(regs))
		{
			*(data+addr) = I2C1::getData();
			addr++;
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
		if(mode == I2C_SLAVE_READ_DATA && addr < sizeof(regs))
		{
			if(addr == 0) regs.tick = tick;
 			I2C1::sendData(*(data+addr));
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

  regs.adc_12v = (s16)((s32)conf.k_12v*dma_adc_buff[0]/8192 + conf.c_12v);
  regs.adc_btt = (s16)((s32)conf.k_btt*dma_adc_buff[1]/8192 + conf.c_btt);
  regs.adc_5v0 = (s16)((s32)conf.k_5v0*dma_adc_buff[2]/8192 + conf.c_5v0);
  regs.adc_3v3 = (s16)((s32)conf.k_3v3*dma_adc_buff[3]/8192 + conf.c_3v3);
  regs.adc_vcc = (s16)((s32)conf.k_vcc*dma_adc_buff[4]/8192 + conf.c_vcc);
  regs.adc_temp = (s16)((s32)conf.k_temp*dma_adc_buff[5]/8192 + conf.c_temp);
  regs.adc_ref = (s16)((s32)conf.k_ref*dma_adc_buff[6]/8192 + conf.c_ref);

  DMA_ADC1::disablePeripheral();
  DMA_ADC1::setNumberOfTransactions(NUM_DMA_ADC);
}

extern "C" void SysTick()
{
  ++tick;
}
