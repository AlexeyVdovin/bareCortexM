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
#include <stddef.h>

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

#include "rs485.hpp"
#include "rs485_packet.hpp"
#include "i2c_master.hpp"

#define UART1_BAUD_RATE 115200
#define I2C_SLAVE_ADDR  0x40


#define I2C1_REG reinterpret_cast<i2c::Registers*>(i2c::I2C1)

typedef PB12  LED_RED;

typedef PA0  AD_IN2;  
typedef PA1  AD_IN1;  
typedef PB0  AD_12V; 

typedef PA2  RELAY_X6;  
typedef PA3  RELAY_X5;  
typedef PA4  RELAY_X4;  
typedef PA5  RELAY_X3;  
typedef PA6  RELAY_X2;  
typedef PA7  RELAY_X1;  

typedef PA8  EN_1W1;
typedef PA11 EN_1W2;
typedef PA12 EN_1W3;
typedef PA15 EN_1W4;

typedef PC13 ADR0;     // IN
typedef PC14 ADR1;     // IN
typedef PC15 ADR2;     // IN

typedef PB1  VDD_OK;   // IN
typedef PB2  VDD_EN;   // PP

typedef PB3  EE_WP;    // PP Remap !!!

typedef PB9  PWM1;     // PP
typedef PB8  PWM2;     // PP

enum {
  PWM1_OC = 4,
  PWM2_OC = 3,
};

typedef PA9   U1TX;    // PP
typedef PA10  U1RX;    // IN

typedef PB6   SCL;     // OD
typedef PB7   SDA;     // OD

typedef DMA1_CHANNEL1 DMA_ADC1;

enum {
	NV_CMD_READ  = 0x10000000,
	NV_CMD_WRITE = 0x20000000,
	NV_CMD_SAVE  = 0x40000000,
	NV_CMD_ADDR  = 0x0FFFFFFF
};

enum {
	CTRL_1W1_EN		= 0x0001,
	CTRL_1W2_EN		= 0x0002,
	CTRL_1W3_EN		= 0x0004,
	CTRL_1W4_EN		= 0x0008,
	CTRL_VDD_EN		= 0x0010,
};

typedef struct
{
  s64 tick;      // 0x00 RO
  u32 status;    // 0x08 RO
  u16 control;   // 0x0c R/W
  s16 adc_12v;   // 0x0e RO
  s16 adc_in0;   // 0x10 RO
  s16 adc_in1;   // 0x12 RO
  s32 reserved;  // 0x14 --
  s16 adc_ref;   // 0x18 RO
  s16 adc_temp;  // 0x1a RO
  u16 pwm1;      // 0x1c RO
  u16 pwm2;      // 0x1e RO
  u8  ow_sid[8]; // 0x20 1W ROM sid
  u16 ow_type;   // 0x28 type
  s16 ow_temp;   // 0x2a 1W temp in 1/16C
  u8  ow_ch[4];  // 0x2c 1W sensors per channer
  u8  pwm1_tid;  // 0x30 1W sensor id for PWM1
  u8  pwm2_tid;  // 0x31 1W sensor id for PWM2
  s16 pwm1_set;  // 0x32 temp Set for PWM1
  s16 pwm2_set;  // 0x34 temp Set for PWM1
  u32 nv_cmd;    // 0x NV_CMD_****
  u32 nv_data;   // 0x
  u8  count[4];  // 0x
} REGS;

typedef struct
{
	s16	k; // k in 1/8192
	s16 c; // value = k * RAW + c
} kv_t;

typedef struct 
{
  u8  sid[8];
} ow_t;

typedef union 
{
  u32 dummy;
  struct
  {
    u16 i2c_id;
    u16 node_id;
    kv_t kv_12v;
    kv_t kv_in[2];
    kv_t kv_ref;
    kv_t kv_temp;
    ow_t sensor[16];
    u16  pwm_min;
    u16  pwm_max;
    u8   pwm1_tid; // 1W Device type for PWM1
    u8   pwm2_tid; // 1W Device type for PWM2
    u32 crc;    // Aligned ? !!!
  };
} CONF;

// 1W sensors
enum {
  OW_BOILER_TEMP1   =  0,
  OW_BOILER_TEMP2, //  1
  OW_BOILER_IN,    //  2
  OW_BOILER_OUT,   //  3
  OW_BOILER_RET,   //  4
  OW_FLOOR_IN,     //  5
  OW_FLOOR_OUT,    //  6
  OW_FLOOR_RET,    //  7
  OW_RADIATOR_OUT, //  8
  OW_RADIATOR_RET, //  9
  OW_HEAT_IN,      // 10
  OW_HEAT_OUT,     // 11
  OW_HOTW_IN,      // 12
  OW_HOTW_OUT,     // 13
  OW_AMBIENT,      // 14
  OW_RESERVED      // 15
}; 

typedef struct {
  int err;
  u8 addr[8]; // 1W HW addr
  s16 result; // Measured result
  u8 ch;      // Ch ID
  u8 type;    // 1W Device type
} ow_dev_t;

typedef struct {
  u8 dev[8]; // index in ow_dev
  int err;
  u8 n;      // devices on this channel
} ow_port_t;

u8 ow_dev_total = 0; // total 1W devices
ow_dev_t ow_dev[32];
ow_port_t ow[4];

enum { NUM_DMA_ADC = 5 };

extern const u32 __flash_start;
extern const u32 __flash_end;

const u32 flash_start = (u32)(&__flash_start);
const u32 flash_end = (u32)(&__flash_end);

volatile u32 dma_count;
volatile s16 dma_adc_buff[NUM_DMA_ADC];

volatile s64 tick = 0;
volatile s64 i2c1_wd = 0;

volatile REGS regs;
volatile CONF conf; // Copy from nv_conf
CONF* conf1;
CONF* conf2;
volatile u16 control;

static inline s64 get_time() { return tick; }

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

  RELAY_X1::setLow();
  RELAY_X1::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  RELAY_X2::setLow();
  RELAY_X2::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  RELAY_X3::setLow();
  RELAY_X3::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  RELAY_X4::setLow();
  RELAY_X4::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  RELAY_X5::setLow();
  RELAY_X5::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  RELAY_X6::setLow();
  RELAY_X6::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  EN_1W1::setHigh();
  EN_1W1::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  EN_1W2::setHigh();
  EN_1W2::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  EN_1W3::setHigh();
  EN_1W3::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  EN_1W4::setHigh();
  EN_1W4::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  ADR0::setMode(gpio::cr::FLOATING_INPUT);
  ADR1::setMode(gpio::cr::FLOATING_INPUT);
  ADR2::setMode(gpio::cr::FLOATING_INPUT);

  VDD_EN::setHigh();
  VDD_EN::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  VDD_OK::setMode(gpio::cr::FLOATING_INPUT);

  EE_WP::setHigh();
  EE_WP::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
  AFIO::configureSwj<afio::mapr::swj::JTAG_DP_DISABLED>();
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
    conf.i2c_id = I2C_SLAVE_ADDR;
    conf.node_id = 0x3FFF;
    conf.kv_12v.k = 26460;
    conf.kv_12v.c = 0;
    conf.kv_in[0].k = 147;
    conf.kv_in[0].c = 0;
    conf.kv_in[1].k = 256;
    conf.kv_in[1].c = 0;
    conf.kv_ref.k = 8192;
    conf.kv_ref.c = 0;
    conf.kv_temp.k = 8192; // TODO: Set correct default Temperature conversion constants
    conf.kv_temp.c = 0;
    conf.pwm_min = 0;
    conf.pwm_max = 999;
    memset((void*)conf.sensor, 0, sizeof(conf.sensor));
#if 0    
	  conf_write(conf1, &conf);
	  conf_write(conf2, &conf);
#endif
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

  // PWM timer
  TIM4::enableClock();
  TIM4::configurePwm<24000000, 1000>();

  TIM4::configurePwmOutput<PWM1_OC>(
      tim::ccer::cce::ENABLED,
      tim::ccer::ccp::RISING_EDGE_ACTIVE_HIGH,
      tim::occmr::ocm::PWM_MODE_1);
  TIM4::configurePwmOutput<PWM2_OC>(
      tim::ccer::cce::ENABLED,
      tim::ccer::ccp::RISING_EDGE_ACTIVE_HIGH,
      tim::occmr::ocm::PWM_MODE_1);

  TIM4::setPulse<PWM1_OC>(0);
  TIM4::setPulse<PWM2_OC>(0);

  PWM1::setLow();
  PWM1::setMode(gpio::cr::AF_PUSH_PULL_50MHZ);
  PWM2::setLow();
  PWM2::setMode(gpio::cr::AF_PUSH_PULL_50MHZ);

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
  I2C1::setSlave7BitAddr1(conf.i2c_id);
  I2C1::enableACK();

  // I2C1::unmaskInterrupts();
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

  ADC1::setConversionTime<0, adc::smp::SAMPLING_TIME_55_5_CYCLES>();
  ADC1::setConversionTime<1, adc::smp::SAMPLING_TIME_55_5_CYCLES>();
  ADC1::setConversionTime<8, adc::smp::SAMPLING_TIME_55_5_CYCLES>();
  ADC1::setConversionTime<16, adc::smp::SAMPLING_TIME_55_5_CYCLES>();
  ADC1::setConversionTime<17, adc::smp::SAMPLING_TIME_55_5_CYCLES>();

  ADC1::setRegularSequenceOrder<1, 1>();  // A_IN0
  ADC1::setRegularSequenceOrder<2, 0>();  // A_IN1
  ADC1::setRegularSequenceOrder<3, 8>();  // ADC_12V
  ADC1::setRegularSequenceOrder<4, 16>(); // AD_TEMP
  ADC1::setRegularSequenceOrder<5, 17>(); // AD_REF

  ADC1::setNumberOfRegularChannels<NUM_DMA_ADC>();

  AD_12V::setMode(gpio::cr::ANALOG_INPUT);
  AD_IN1::setMode(gpio::cr::ANALOG_INPUT);
  AD_IN2::setMode(gpio::cr::ANALOG_INPUT);

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

  rs485_init();

  TIM2::startCounter();
  TIM4::startCounter();
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
  return 0;
}

s16 get_temp(u8 id)
{

  return 0;
}

typedef struct {
  s16 p;   // Proportional gain
  s16 i;   // Integral gain
  s16 d;   // Derivative
  s16 t;   // previouse Temp
  s32 val; // control Value
  s32 integral;
} pd_t;

pd_t p1, p2;

/* p - pid*, t0 - target T, t - current T */
void update_pid(pd_t* p, s16 set, s16 t)
{
  s16 error = set - t;
  s16 delta = p->t - t;
  s32 out = p->p * error / 4096;
  out += p->d * delta / 4096;
  p->integral += p->i * error / 4096;
  p->t = t;
  out += p->integral;
  p->val = out;
}

static inline u16 min(s16 a, u16 b) { return (a < b) ? a : b; }
static inline u16 max(u16 a, s16 b) { return (a < b) ? b : a; }

void loop()
{
  static s64 timer_t1 = 1000;
  static u8 ow_n = 5;

  int i, res;
  u8 n, pad[9];

  if(tick >= timer_t1)
  {
    timer_t1 = tick + 1000;
    TIM4::setPulse<PWM1_OC>(regs.pwm1);
    TIM4::setPulse<PWM2_OC>(regs.pwm2);
    // LED_RED::setOutput(LED_RED::isHigh() ? 0 : 1);
#if 0
    // rs485_write((u8*)"Hello World !!!\n", 16);
    while((i = rs485_read()) >= 0)
    {
      putc(i & 0x00FF, stdout);
    }
    putc('\n', stdout);
#endif
#if 1
    if(ow_n < 5)
    {
      n = ow_n - 1;
      if(ow[n].err >= 0)
      {
        for(i = 0; i < ow[n].n; ++i)
        {
          res = ds2482_ds18b20_read(0x18, ow[n].addr[i], pad);
          if(res < 0 || ow[n].err == 0)
          {
            ow[n].result[i] = -200 + res;
          }
          else
          {
            s16 pwm = regs.pwm1;
            s16 temp = (pad[1] << 8) | pad[0];
            ow[n].result[i] = temp;
            if(memcmp((const void*)conf.sensor[regs.pwm1_tid].sid, ow[n].addr[i], 8) == 0)
            {
              if(p1.t < -999) p1.t = temp; // Set initial value
              update_pid(&p1, regs.pwm1_set, temp);
              pwm += p1.val;
              regs.pwm1 = max(conf.pwm_min, min(pwm, conf.pwm_max));
              printf("T1: %d.%02d Set1: %d.%02d PWM1: %d\n", 
                  temp/16, (temp - (temp/16)*16)*100/16, 
                  regs.pwm1_set/16, (regs.pwm1_set - (regs.pwm1_set/16)*16)*100/16, 
                  regs.pwm1);
            }
            if(memcmp((const void*)conf.sensor[regs.pwm2_tid].sid, ow[n].addr[i], 8) == 0)
            {
              if(p2.t < -999) p2.t = temp;  // Set initial value
              update_pid(&p2, regs.pwm2_set, temp);
              pwm += p2.val;
              regs.pwm2 = max(conf.pwm_min, min(pwm, conf.pwm_max));
              printf("T2: %d.%02d Set2: %d.%02d PWM2: %d\n", 
                  temp/16, (temp - (temp/16)*16)*100/16, 
                  regs.pwm2_set/16, ((regs.pwm2_set - (regs.pwm2_set/16)*16)*100)/16, 
                  regs.pwm2);
            }
          }
          printf("%d read %d: %d\n", n, i, ow[n].result[i]);
        }
      }
      if(ow_n > 3) ow_n = 0;
    }
    else ow_n = 0;
    switch(ow_n) {
    case 0:
      EN_1W1::setLow();
      EN_1W2::setHigh();
      EN_1W3::setHigh();
      EN_1W4::setHigh();
      break;
    case 1:
      EN_1W1::setHigh();
      EN_1W2::setLow();
      EN_1W3::setHigh();
      EN_1W4::setHigh();
      break;
    case 2:
      EN_1W1::setHigh();
      EN_1W2::setHigh();
      EN_1W3::setLow();
      EN_1W4::setHigh();
      break;
    case 3:
      EN_1W1::setHigh();
      EN_1W2::setHigh();
      EN_1W3::setHigh();
      EN_1W4::setLow();
      break;
    default: 
      EN_1W1::setHigh();
      EN_1W2::setHigh();
      EN_1W3::setHigh();
      EN_1W4::setHigh();
    }
    res = ds2482_ds18b20_search(0x18, ow[ow_n].addr);
    if(res < 0) {
      ow[ow_n].err = res;
      ow[ow_n].n = 0;
    } else {
      ow[ow_n].err = 0;
      ow[ow_n].n = res;
    }
    //printf("%d search [%d]: %d\n", ow_n, ow[ow_n].n, ow[ow_n].err);

    if(ow[ow_n].n)
    {
      res = ds2482_ds18b20_start(0x18, 0);
      ow[ow_n].err = res;
      //printf("%d start: %d\n", ow_n, ow[ow_n].err);
    }

    ++ow_n;
#endif    
  }
  pkt_pool();
}

int main()
{
  clk::initialize();

  p1.t = -1000;
  p2.t = -1000;

  memset((void*)&regs, 0, sizeof(regs));
  regs.pwm1_tid = OW_FLOOR_OUT;
  regs.pwm1_set = 28 << 4;  // 28C
  

  initializePeripherals();

//  printf("conf1: 0x%08x\n", conf1);
//  printf("conf2: 0x%08x\n", conf2);

  pkt_init();

  // rs485_write((u8*)"Hello World !!! ", 16);

  VDD_EN::setLow();
//  EE_WP::setLow();


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

int pkt_rw_reg(packet_t* pkt)
{
  u16 addr = (pkt->data[3]) << 8 | pkt->data[2];
  u8* p = (u8*)&regs;
  if(addr+pkt->data[1] > sizeof(regs) || pkt->data[1] > 16) return 0;
  if(addr == offsetof(REGS, tick)) regs.tick = tick;
  if(pkt->data[0] == PKT_CMD_READ_REG)
  {
    memcpy(pkt->data+4, p+addr, pkt->data[1]);
    pkt->len = 4 + pkt->data[1];
  }
  else
  {
    memcpy(p+addr, pkt->data+4, pkt->data[1]);
  }
  return 1;
}

int pkt_rw_conf(packet_t* pkt)
{
  u16 addr = (pkt->data[3]) << 8 | pkt->data[2];
  u8* p = (u8*)&conf;
  if(addr+pkt->data[1] > sizeof(conf) || pkt->data[1] > 16) return 0;
  if(pkt->data[0] == PKT_CMD_READ_CONF)
  {
    memcpy(&pkt->data[4], p+addr, pkt->data[1]);
    pkt->len = 4 + pkt->data[1];
  }
  else
  {
    memcpy(p+addr, &pkt->data[4], pkt->data[1]);
  }
  return 1;
}
/*
  u8 addr[8][8];
  s16 result[8];
  int err;
  u8 n;
*/
int pkt_read_1w(packet_t* pkt)
{
  u8 ch = pkt->data[3];
  u8 i = pkt->data[2];
  if(ch > 4 || i > 8) return 0;
  pkt->data[4] = ow[ch].n;
  memcpy(&pkt->data[5], &ow[ch].addr[i], 8);
  memcpy(&pkt->data[13], &ow[ch].result[i], sizeof(s16));
  pkt->data[1] = 11;
  pkt->len = 15;
  return 1;
}

void pkt_process(packet_t* pkt)
{
  int tx = 0;

  // printf("pkt: 0x%02x%02x %02x->%02x->%02x %02x %02x [%02x] %02x %02x %02x %02x\n", pkt->id[0], pkt->id[1], pkt->from, pkt->via, pkt->to, pkt->flags, pkt->seq, pkt->len, pkt->data[0], pkt->data[1], pkt->data[2], pkt->data[3]);
  if(pkt->from == conf.i2c_id) return;

  switch(pkt->data[0])
  {
    case PKT_CMD_READ_REG:
    case PKT_CMD_WRITE_REG:
      tx = pkt_rw_reg(pkt);
      break;
    case PKT_CMD_READ_CONF:
    case PKT_CMD_WRITE_CONF:
      tx = pkt_rw_conf(pkt);
      break;
    case PKT_CMD_READ_1W:
      tx = pkt_read_1w(pkt);
      break;
    default:
      pkt->len = 1;
      pkt->data[0] = PKT_CMD_NAK;
      tx = 1;
      break;
  }
  if(tx)
  {
    LED_RED::setOutput(LED_RED::isHigh() ? 0 : 1);
    pkt->to = pkt->from;
    pkt->from = conf.i2c_id;
    pkt->via = 0;
    // printf("TX: 0x%02x%02x %02x->%02x->%02x %02x %02x [%02x] %02x %02x %02x %02x %02x %02x %02x %02x\n", pkt->id[0], pkt->id[1], pkt->from, pkt->via, pkt->to, pkt->flags, pkt->seq, pkt->len, pkt->data[0], pkt->data[1], pkt->data[2], pkt->data[3], pkt->data[4], pkt->data[5], pkt->data[6], pkt->data[7]);
    pkt_send(pkt);
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

			if(addr == 0x0e) // Control word was written
			{
				data += a;

				if(*data & 0x08) // Status read
				{
					int n;
					*data &= ~0x08;
					n = *data & 0x07;
					//regs.pwr = power[n];
					//regs.energy = energy[n]/36000;
				}
			}
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
