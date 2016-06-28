/*********************************************************************
 *
 *  Copyright (c) 2016
 *  Author: Vitor Matos
 *
 *  rosserial_tivac_socket buttons subscriber tutorial
 *
 *  On this demo your TivaC Connected Launchpad will subscribe the user 
 * buttons state on the topic '/button_state'.
 *  This tutorial demonstrates the use of a custom message Buttons.msg
 *
 * Full guide: http://wiki.ros.org/rosserial_tivac_socket/Tutorials
 *
 *********************************************************************/

#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <drivers/pinout.h>
}
// ROS includes
#include <ros.h>
// Custom ROS message
#include "rosserial_tivac_socket_tutorials/Buttons.h"

// ROS nodehandle
ros::NodeHandle nh;

void button_cb(const rosserial_tivac_socket_tutorials::Buttons& msg)
{
  if (msg.sw1.data) {
    LEDWrite(CLP_D2, CLP_D2);
    UARTprintf("Turn on\n");
  }
  if (msg.sw2.data) {
    LEDWrite(CLP_D2, 0);
    UARTprintf("Turn off\n");
  }
}
ros::Subscriber<rosserial_tivac_socket_tutorials::Buttons> button_subscriber("button_state", &button_cb);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Run from the PLL at 120 MHz.
  MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), TM4C129FREQ);

  // Setup LED pins
  PinoutSet(1, 0);
  
  // ROS nodehandle initialization and topic registration
  nh.initNode((char *)"192.168.1.140");
  nh.subscribe(button_subscriber);

  while (1)
  {
    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
