#include <stdbool.h>
#include <stdint.h>
#include "ros_freertos.h"
extern "C"
{
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
}

#define tskSPIN_PRIORITY 1

// Should only be for TM4C129
#ifdef TM4C1294XL
#define LED1        GPIO_PIN_1  // D1 LED
#define LED_PORT    GPIO_PORTN_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPION
#ifndef TM4C129FREQ
#error "Must define system clock frequency on: TM4C129FREQ"
#endif
#else
#error "This package only works for Tiva C Launchpad Connected"
#endif

static ros::NodeHandle *nh_;

// ros spin() like task
static void spinTask(void *pvParameters)
{
  portTickType ui32WakeTime;
  // Get the current tick count.
  ui32WakeTime = xTaskGetTickCount();

  while (1)
  {
    // rosserial callback handling
    nh_->spinOnce();

    // Toggle for spin heartbeat
    MAP_GPIOPinWrite(LED_PORT, LED1, MAP_GPIOPinRead(LED_PORT, LED1)^LED1);

    vTaskDelayUntil(&ui32WakeTime, 100);
  }
}

// spin task initialization
uint32_t spinInitTask(ros::NodeHandle *nh)
{
  // Enable LED1 for spin heartbeat
#ifdef LED_HEARTBEAT
  MAP_SysCtlPeripheralEnable(LED_PERIPH);
  MAP_GPIOPinTypeGPIOOutput(LED_PORT, LED1);
#endif

  nh_ = nh;

  // Init spin task
  if (xTaskCreate(spinTask, (const portCHAR *)"spin", 120, NULL, tskIDLE_PRIORITY + tskSPIN_PRIORITY, NULL) != pdTRUE)
  {
    return 1;
  }
  return 0;
}
