cmake_minimum_required(VERSION 2.8.3)
project(rosserial_tivac_socket_tutorials)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosserial_client
  rosserial_tivac_socket
  std_msgs
)

add_message_files(
  FILES Buttons.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_tivac_socket
  SCRIPT make_libraries_tiva
)
# Because we need to generate our custom messages before exporting our rosserial ros_lib
add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Chatter tutorial
rosserial_configure_client(
  DIRECTORY chatter
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(chatter chatter.axf ALL)
rosserial_add_client_target(chatter flash)
rosserial_add_client_target(chatter size)
rosserial_add_client_target(chatter dump)

# Custom message - Button publisher
rosserial_configure_client(
  DIRECTORY buttons
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(buttons buttons.axf ALL)
rosserial_add_client_target(buttons flash)
rosserial_add_client_target(buttons size)

# Custom message - Button subscriber
rosserial_configure_client(
  DIRECTORY leds
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(leds leds.axf ALL)
rosserial_add_client_target(leds flash)
rosserial_add_client_target(leds size)

# freeRTOS example
rosserial_configure_client(
  DIRECTORY freertos
  TOOLCHAIN_FILE ${ROSSERIAL_TIVAC_TOOLCHAIN}
)
rosserial_add_client_target(freertos freertos.axf ALL)
rosserial_add_client_target(freertos flash)
rosserial_add_client_target(freertos size)
rosserial_add_client_target(freertos dump)
