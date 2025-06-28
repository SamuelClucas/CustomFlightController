cmake_minimum_required(VERSION 3.13)

# Set path to your SDK (relative to this CMakeLists.txt)
set(PICO_SDK_PATH "../pico-sdk")
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(scout_firmware)
pico_sdk_init()

# Add main flight controller and Telemetry (test.cpp) source
add_executable(scout_firmware
    src/main.cpp
    src/telemetry.cpp
    src/radio_receiver.cpp
    src/act.cpp
    src/sense.cpp
)

pico_enable_stdio_uart(scout_firmware 0)
pico_enable_stdio_usb(scout_firmware 0)

# Include headers from src and include directories
target_include_directories(scout_firmware PRIVATE src include)

# Link required libraries
target_link_libraries(scout_firmware
    pico_stdlib
    hardware_adc
    hardware_i2c
    hardware_uart
    hardware_gpio
)

pico_add_extra_outputs(scout_firmware)
