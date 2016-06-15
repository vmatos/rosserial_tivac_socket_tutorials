cmake_minimum_required(VERSION 2.8.3)
project(chatter)

# Include rosserial libraries for TivaC
include_directories(${ROS_LIB_DIR})

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DTM4C129FREQ=120000000)

# Generate target for TivaC
generate_tivac_firmware(
  SRCS chatter.cpp
)
