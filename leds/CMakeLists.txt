cmake_minimum_required(VERSION 2.8.3)
project(leds)

# Device's static IP. Don't define if you want DHCP assigned address.
#add_definitions(-DROS_IP_ADDRESS="192.168.0.31")
#add_definitions(-DROS_NETMASK="255.255.255.0")
# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DTM4C129FREQ=120000000)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS 
    leds.cpp
    ${TIVA_WARE_PATH}/examples/boards/ek-tm4c1294xl/drivers/pinout.c
  INCS 
    . 
    ${TIVA_WARE_PATH}/examples/boards/ek-tm4c1294xl
)
