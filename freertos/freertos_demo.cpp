#include <stdbool.h>
#include <stdint.h>
#include "ros_freertos.h"
extern "C"
{
  #include "FreeRTOSConfig.h"
  #include "FreeRTOS.h"
  #include "task.h"
}
#include "spin_task.h"
#include "publish_task.h"
#include "subscribe_task.h"

// ROS nodehandle
ros::NodeHandle nh;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
extern "C"
{
void __error__(char *pcFilename, uint32_t ui32Line)
{
  UARTprintf("Error at line: %d\n", ui32Line);
  UARTprintf(pcFilename);
}
}
#endif

extern "C" 
{
void vAssertCalled(unsigned long ulLine,  const char *pcFile)
{
volatile unsigned long ulSetTo1InDebuggerToExit = 0;

	taskENTER_CRITICAL();
	{
    UARTprintf("%s, %d\n", pcFile, ulLine);
		while( ulSetTo1InDebuggerToExit == 0 )
		{
			/* Nothing do do here.  Set the loop variable to a non zero value in
			the debugger to step out of this function to the point that caused
			the assertion. */
			( void ) pcFile;
			( void ) ulLine;
		}
	}
	taskEXIT_CRITICAL();
}

// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
  UARTprintf("Stack overflow: %s\n", pcTaskName);
  
  // Loop forever.  Interrupts are disabled on entry to this function,
  // so no processor interrupts will interrupt this loop.
  while (1);
}
}

void ConfigUART(uint32_t baud)
{
  // Enable the GPIO Peripheral used by the UART.
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  // Enable UART0
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  // Configure GPIO Pins for UART mode.
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  // Use the internal 16MHz oscillator as the UART clock source.
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, baud, 16000000);
}

// Initialize FreeRTOS and start the initial set of tasks.
int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  
  // Run from the PLL at 120 MHz.
  MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), TM4C129FREQ);
                          
  // Make sure the main oscillator is enabled because this is required by
  // the PHY.  The system must have a 25MHz crystal attached to the OSC
  // pins.  The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
  // frequency is 10MHz or higher.
  MAP_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
  
  MAP_IntEnable(FAULT_NMI);
  MAP_IntEnable(FAULT_MPU);
  MAP_IntEnable(FAULT_BUS);
  MAP_IntEnable(FAULT_USAGE);
  
  ConfigUART(115200);

  UARTprintf("\n\nWelcome to the EK-TM4C129XL FreeRTOS Demo!\n");
  
  // ROS nodehandle initialization and topic registration
  nh.initNode("192.168.1.135");

  // Start ROS spin task, responsible for handling callbacks and communications
  if (spinInitTask(&nh))
  {
    UARTprintf("Couldn't create ROS spin task.\n");
    while (1);
  }
  else 
  {
    UARTprintf("Created ROS spin task.\n");
  }
  
  // Register and init subscribe task
  if (subscribeInitTask(&nh))
  {
    UARTprintf("Couldn't create subscribe task.\n");
    while (1);
  }
  else 
  {
    UARTprintf("Created subscribe task.\n");
  }

  // Register and init publish task
  if (publishInitTask(&nh))
  {
    UARTprintf("Couldn't create publish task.\n");
    while (1);
  }
  else 
  {
    UARTprintf("Created publish task.\n");
  }
  
  // Start the scheduler.  This should not return.
  UARTprintf("Starting scheduller.\n");  
  vTaskStartScheduler();

  // In case the scheduler returns for some reason, print an error and loop forever.
  while (1)
  {
    UARTprintf("Scheduler returned!\n");
  }
}

#if configUSE_MALLOC_LOCK_UNLOCK == 1
extern "C"
{
/*-----------------------------------------------------------*/
void __malloc_lock(struct _reent* REENT)
{
	vTaskSuspendAll();
}

void __malloc_unlock(struct _reent* REENT)
{
	xTaskResumeAll();	
}
}
#endif

