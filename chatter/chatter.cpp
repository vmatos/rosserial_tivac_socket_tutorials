/*********************************************************************
 *
 *  Copyright (c) 2016
 *  Author: Vitor Matos
 *
 *  rosserial_tivac_socket chatter tutorial
 *
 *  On this demo your TivaC Launchpad Connected will publish a string
 *  over the topic "/chatter".
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
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>

// ROS nodehandle
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "Hello world!";

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();

  // Run from the PLL at 120 MHz.
  MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), TM4C129FREQ);

  // ROS nodehandle initialization and topic registration
  // Nodehandle will connect to IP
  nh.initNode("192.168.1.135");
  nh.advertise(chatter);

  while (1)
  {
    // Publish message to be transmitted.
    if (nh.connected())
    {
      str_msg.data = hello;
      chatter.publish(&str_msg);
    }

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}
