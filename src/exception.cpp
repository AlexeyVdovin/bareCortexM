/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 *
 * This file is part of bareCortexM.
 *
 * bareCortexM is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * bareCortexM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with bareCortexM. If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#include <stdint.h>
#include "exception.hpp"

extern char __StackTop;

typedef uint32_t u32;
typedef void (* pvf)();

extern "C" void resetHandler();

namespace exception {
  __attribute__ ((section(".exception_vector")))
    pvf exceptionVector[] = {
    pvf(&__StackTop),  // 0x0000
    resetHandler,// 0x0004
    NMI,// 0x0008
    HardFault,// 0x000C
    MemManage,// 0x0010
    BusFault,// 0x0014
    UsageFault,// 0x0018
    0,// 0x001C
    0,// 0x0020
    0,// 0x0024
    0,// 0x0028
    SVCall,// 0x002C
    DebugMon,// 0x0030
    0,// 0x0034
    PendSV,// 0x0038
    SysTick,// 0x003C
  };  // exceptionVector
}  // namespace exception

extern "C" void defaultExceptionHandler()
{
  while (true) {
  }
}

extern "C" void NMI_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void HardFault_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void MemManage_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void BusFault_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void UsageFault_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void SVCall_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void DebugMon_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void PendSV_ExceptionHandler()
{
  while (true) {
  }
}
extern "C" void SysTick_ExceptionHandler()
{
  while (true) {
  }
}

// Core exceptions
#pragma weak NMI        = NMI_ExceptionHandler
#pragma weak HardFault  = HardFault_ExceptionHandler
#pragma weak MemManage  = MemManage_ExceptionHandler
#pragma weak BusFault   = BusFault_ExceptionHandler
#pragma weak UsageFault = UsageFault_ExceptionHandler
#pragma weak SVCall     = SVCall_ExceptionHandler
#pragma weak DebugMon   = DebugMon_ExceptionHandler
#pragma weak PendSV     = PendSV_ExceptionHandler
#pragma weak SysTick    = SysTick_ExceptionHandler
