/*********************************************************************
 *
 *  Copyright (c) 2016
 *  Author: Vitor Matos
 *
 *  rosserial_tivac_socket buttons tutorial
 *
 *  On this demo your TivaC Connected Launchpad will publish the user 
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
  #include "buttons.h"
}
// ROS includes
#include <ros.h>
// Custom ROS message
#include "rosserial_tivac_socket_tutorials/Buttons.h"

// ROS nodehandle
ros::NodeHandle nh;

rosserial_tivac_socket_tutorials::Buttons button_msg;
ros::Publisher button_publisher("button_state", &button_msg);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Run from the PLL at 120 MHz.
  MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), TM4C129FREQ);

  uint8_t button_debounced_delta;
  uint8_t button_raw_state;
  ButtonsInit();

  // ROS nodehandle initialization and topic registration
  nh.initNode("192.168.1.135");
  nh.advertise(button_publisher);

  while (1)
  {
    if (nh.connected())
    {
      uint8_t button_debounced_state = ButtonsPoll(&button_debounced_delta, &button_raw_state);
      // Publish message to be transmitted.
      button_msg.sw1.data = button_debounced_state & LEFT_BUTTON;
      button_msg.sw2.data = button_debounced_state & RIGHT_BUTTON;
      button_publisher.publish(&button_msg);
    }

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
