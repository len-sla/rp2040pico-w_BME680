# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set up the Pico SDK path
if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
else()
    set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/pico-sdk)
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Include the Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Link to the correct TF Lite library
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/pico-tflmicro/build)

project(7cpp C CXX ASM)

# Initialize the Pico SDK
pico_sdk_init()

add_executable(7cpp 
    7cpp.cpp
    bme68x.c
    mqtt.cpp
    uart_dma.cpp
    cloud_predictor.cpp
    model.cpp
)

pico_set_program_name(7cpp "7cpp")
pico_set_program_version(7cpp "0.1")

pico_enable_stdio_uart(7cpp 1)
pico_enable_stdio_usb(7cpp 0)

target_link_libraries(7cpp
    pico_stdlib
    hardware_i2c
    hardware_adc
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip
    pico_malloc
    pico_runtime
    hardware_rtc
    hardware_gpio
    pico_lwip_mqtt
    pico_util
    hardware_dma
    pico_multicore
    ${CMAKE_CURRENT_SOURCE_DIR}/pico-tflmicro/build/libpico-tflmicro.a
)

target_include_directories(7cpp PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PICO_SDK_PATH}/src/rp2_common/hardware_adc/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_rtc/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_gpio/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_dma/include
    ${PICO_SDK_PATH}/src/common/pico_stdlib/include
    ${PICO_SDK_PATH}/src/rp2_common/hardware_sync/include
    ${PICO_SDK_PATH}/src/common/pico_sync/include
    ${CMAKE_CURRENT_SOURCE_DIR}/pico-tflmicro/src
)

target_compile_definitions(7cpp PRIVATE
    PICO_I2C_DEBUG=1
)

pico_add_extra_outputs(7cpp)
