/*
 * Copyright (c) 2016  All rights reserved.
 * Author: Vitor Matos
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
// Bare minimum hardware resources allocated for rosserial tcp communication.
// * 1 heartbeat LED if desired
// * Systick Interrupt handler
//
//*****************************************************************************

#ifndef ROS_LIB_TIVAC_HARDWARE_FREERTOS_H
#define ROS_LIB_TIVAC_HARDWARE_FREERTOS_H

#include <stdbool.h>
#include <stdint.h>
extern "C"
{
  #include <inc/hw_types.h>
  #include <inc/hw_memmap.h>
  #include <inc/hw_ints.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/rom.h>
  #include <driverlib/rom_map.h>
  #include <driverlib/systick.h>
  #include <driverlib/pin_map.h>  
  #include <driverlib/uart.h>
  #include <utils/uartstdio.h>
  #include "FreeRTOS.h"
  #include "ethClient.h"
  #include "task.h"
}

extern volatile uint32_t g_ui32milliseconds;
extern volatile uint32_t g_ui32heartbeat;

class TivaCHardware
{
  public:
    TivaCHardware() {}

    void init(const char *portName)
    {
      this->ui32SysClkFreq = TM4C129FREQ;

      // Init ethernet tcp/ip 
      ethInit(this->ui32SysClkFreq, portName);

      // Enable processor interrupts.
      MAP_IntMasterEnable();
    }

    // read a byte -1 = failure
    int read()
    {
      return ethReadByte();
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
      ethWrite(data, length);
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
      return (uint32_t) xTaskGetTickCount();
    }

    // System frequency
    uint32_t ui32SysClkFreq;
    uint32_t getSysClkFreq(void)
    {
      return this->ui32SysClkFreq;
    }
};
#endif  // ROS_LIB_TIVAC_HARDWARE_FREERTOS_H
