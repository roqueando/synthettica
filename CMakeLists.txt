cmake_minimum_required(VERSION 3.13)

# Configure with -DPICO_SDK_PATH=/path/to/pico-sdk
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2040_sine_pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(sine_pwm
    src/main.c
    src/ssd1306.c
)
target_link_libraries(sine_pwm
    pico_stdlib
    hardware_pwm
    hardware_timer
    hardware_adc
    hardware_i2c
    m
)
pico_enable_stdio_usb(sine_pwm 1)
pico_enable_stdio_uart(sine_pwm 0)
pico_add_extra_outputs(sine_pwm)
