cmake_minimum_required(VERSION 2.8.3)
project(freertos)

# Per project based definitions and options
add_definitions(-DLED_HEARTBEAT)
add_definitions(-DTM4C129FREQ=120000000)
add_definitions(-DRX_BUFFER_SIZE=256)

# Generate target for TivaC
generate_tivac_firmware(
  STARTUP startup_gcc.c
  SRCS 
    freertos_demo.cpp
    spin_task.cpp
    publish_task.cpp
    subscribe_task.cpp
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/list.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/tasks.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/queue.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/MemMang/heap_3.c
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
  INCS 
    .
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/include 
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/MemMang/
    ${TIVA_WARE_PATH}/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)
