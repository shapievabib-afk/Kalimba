cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(kalimba C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(kalimba
    src/main.c
)

target_link_libraries(kalimba
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_spi
    hardware_i2c
    hardware_pio
    hardware_clocks
)

pico_enable_stdio_usb(kalimba 1)
pico_enable_stdio_uart(kalimba 0)

pico_add_extra_outputs(kalimba)