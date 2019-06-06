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

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.

/* Reported GPS data:

Node ID
*GGA---
  UTC Time
  Latitude
  Longitude
  Position Fix Indicator
  Satellites Used
  HDOP
  MSL Altitude
  Geoid Separation
*RMC---
  Course over ground
  Speed over ground
  Magnetic variation
  Date
  Mode

 */
#include <stdio.h>

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/afio.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/usart.hpp"
#include "peripheral/can.hpp"

#include "peripheral/tim.hpp"

#define UART1_BAUD_RATE 115200

#define CAN_TX  USB_HP_CAN_TX
#define CAN_RX0 USB_LP_CAN_RX0
#define CAN_RX1 CAN1_RX1
#define CAN_SCE CAN1_SCE


typedef PB6 U1TX;
typedef PB7 U1RX;


typedef PC13 LED;
typedef PB8 CANRX;
typedef PB9 CANTX;

u64 tick = 0;
volatile u32 rx0i  = 0;
volatile u32 rx1i  = 0;
volatile u32 txi   = 0;
volatile u32 erri  = 0;

void clk::hseFailureHandler()
{
	while(1) {}
}

void initializeGpio()
{
  // GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();

  AFIO::enableClock();

  LED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
}

void initializeUsart1()
{
  AFIO::configureUsart1<afio::mapr::usart1::REMAP>();

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

void initializeCan()
{
  AFIO::configureCan<afio::mapr::can::PB8_PB9>();

  CAN::enableClocks();
  CANTX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);
  CANRX::setMode(gpio::cr::FLOATING_INPUT);

  CAN::masterReset();

  CAN::configEnter();
  CAN::setBaudrate(500000);
  CAN::configure(
		  can::mcr::txfp::TX_QUEUE_PRIORITY,
		  can::mcr::rflm::OVERRUN_NOT_LOCKED,
		  can::mcr::nart::NO_RETRANSMIT,
		  can::mcr::awum::AUTO_WAKEUP_ON,
		  can::mcr::abom::AUTO_BUSOFF,
		  can::mcr::ttcm::TIME_TRIGGERED_MODE_DISABLED,
		  can::mcr::dbf::NORMAL_RUN,
		  can::btr::lbkm::LOOPBACK_OFF, // LOOPBACK_ON / LOOPBACK_OFF
		  can::btr::silm::NORMAL_OPERATION // SILENT_MODE / NORMAL_OPERATION
		  );
  CAN::txEmptyInterrupt<can::interrupt::ENABLE>();
  CAN::rx0MsgInterrupt<can::interrupt::ENABLE>();
  CAN::rx1MsgInterrupt<can::interrupt::ENABLE>();

  CAN::unmaskInterrupts();

  // Configure filters //
  can::hdr ah, mh;

  ah.addr = 0x00;
  ah.ide = false; // std
  ah.rtr = false;

  mh.addr = 0; // 0x7ff;
  mh.ide = true;
  mh.rtr = true;

  CAN::setFilterMaskBank<0, 0>(&ah, &mh);
  CAN::setFilterActive<0>(true);

  ah.addr = 0x00;
  ah.ide = true; // ext
  ah.rtr = false;

  mh.addr = 0; // 0x1fffffff;
  mh.ide = true;
  mh.rtr = true;

  CAN::setFilterMaskBank<1, 1>(&ah, &mh);
  CAN::setFilterActive<1>(true);

  // u8 d = CAN::getMode();
  // printf("APB1 Frequency: %d\n", CAN::getFrequency());
  // printf("------------ Mode: %d\n", d);

  CAN::configExit();
}

void initializeTimer()
{
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt< 1000 /* Hz */ >();
}

void initializePeripherals()
{
  initializeGpio();
  initializeTimer();
  initializeUsart1();
  initializeCan();

  TIM2::startCounter();
}

can::msg m0, m1;
bool m0i = false;
bool m1i = false;

void loop()
{
  enum {
	  IDLE = 0,
	  KEY_LOCK,
	  KEY_UNLOCK,
	  KEY_RELEASE
  };
  static int s = IDLE, state = IDLE;
  static u64 t0 = 0;
  static u64 t1 = (u64)-1;
  can::msg m;
  u8 rc;

  if(m0i)
  {
	  m = m0;
	  m0i = false;
	  printf("rs%c %04x %01x %08x %08x\n", m.h.rtr?'r':'w', m.h.addr, m.len, m.u[0], m.u[1]);
	  if(1) {}
	  // 0180  00 00 00 80  00 00 00 00 - KEY lock button
	  else if(m.h.addr == 0x180 && m.len == 8 && m.u[0] == 0x80000000 && m.u[1] == 0x00000000) state = KEY_LOCK;
	  // 0180  00 00 00 40  00 00 00 00 - KEY unlock button
	  else if(m.h.addr == 0x180 && m.len == 8 && m.u[0] == 0x40000000 && m.u[1] == 0x00000000) state = KEY_UNLOCK;
	  else if(m.h.addr == 0x180 && m.len == 8 && m.u[0] == 0x00000000 && m.u[1] == 0x00000000)
	  {
		  s = state;
		  t1 = tick+200;
	  }
	  return;
  }
  if(m1i)
  {
	  m = m1;
	  m1i = false;
	  printf("rx%c %08x %01x %08x %08x\n", m1.h.rtr?'r':'w', m1.h.addr, m1.len, m1.u[0], m1.u[1]);
  }

  if(tick >= t0)
  {
    t0 = tick+1000;
    printf("%d: tx: %d, rx0: %d, rx1: %d, err: %d\n", (int)(tick/1000), txi, rx0i, rx1i, erri);
/*
    m.h.addr = 0x1674A123;
    m.h.ide = true;
    m.h.rtr = false;
    m.ts = 0;
    m.fmi = 0;
    m.len = 4;
    m.u[0] = 0x5555aaaa;
    m.u[1] = 0;

    rc = CAN::Send(&m);
    if(rc == 0xff)
    {
    	printf("Error can Tx!\n");
    }
    else
    {
    	u8 d = CAN::getMode();
    	printf("Mode: %d, Tx dispatched to: %d\n", d, rc);
    }
*/
    if(LED::isHigh()) { LED::setLow(); } else { LED::setHigh(); }
  }

  if(tick >= t1 && s != IDLE)
  {
    t1 = tick + 10;

    m.h.addr = 0x180;
    m.h.ide = false;
    m.h.rtr = false;
    m.ts = 0;
    m.fmi = 0;
    m.len = 8;

    // 0180  55 00 00 00  00 00 00 00 - Driver door lock button
    if(s == KEY_LOCK) m.u[0] = 0x00000055, m.u[1] = 0, s = KEY_RELEASE;
    // 0180  aa 02 00 00  00 88 02 00 - Driver door unlock button
    else if(s == KEY_UNLOCK)  m.u[0] = 0x000002aa, m.u[1] = 0x00028800, s = KEY_RELEASE;
    else if(s == KEY_RELEASE) m.u[0] = 0, m.u[1] = 0, s = IDLE;

    rc = CAN::Send(&m);
    if(rc == 0xff)
    {
    	printf("Error can Tx!\n");
    }
    else
    {
    	u8 d = CAN::getMode();
    	printf("M: %d, Tx %08x -> %d\n", d, m.u[0], rc);
    }
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

void interrupt::CAN_TX()
{
	++txi;

	can::mailbox::Complete m = CAN::getTxCompleteMbx();
	if(m & can::mailbox::MBX0)
	{
	  CAN::cleanTxComplete<0>();
	}
	if(m & can::mailbox::MBX1)
	{
	  CAN::cleanTxComplete<1>();
	}
	if(m & can::mailbox::MBX2)
	{
	  CAN::cleanTxComplete<2>();
	}
	// Look for some external FIFO ?
}

void interrupt::CAN_RX0()
{
  ++rx0i;

  while(CAN::msgRxPending<0>())
  {
    CAN::Receive<0>(&m0);
    m0i = true;
  }
  if(CAN::rxFifoOverrun<0>()) {}
  if(CAN::rxFifoFull<0>()) {}
}

void interrupt::CAN_RX1()
{
  ++rx1i;

  while(CAN::msgRxPending<1>())
  {
    CAN::Receive<1>(&m1);
    m1i = true;
  }
  if(CAN::rxFifoOverrun<1>()) {}
  if(CAN::rxFifoFull<1>()) {}
}

void interrupt::CAN_SCE()
{
  ++erri;
}

/*
void interrupt::USART1()
{ TODO: Buffered TX and RX

}
*/
