cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_BOARD pico_w)
set(PICO_CYW43_ARCH_POLL  1)  
set(PICO_LWIP_NO_SYS 1)

add_compile_definitions(
    PICO_CYW43_ARCH_ENABLE_LWIP_SYS=1
    PICO_LWIP_SKIP_NOSYS=1
    LWIP_TIMEVAL_PRIVATE=0
)

pico_sdk_init()

link_directories(libmicroros)

add_executable(main
    main.cpp
    wireless_communication/picow_udp_transports.c
    motor_control/motor.cpp
)

target_link_libraries(main
    pico_stdlib
    microros
    hardware_pwm
    pico_cyw43_arch_lwip_poll     
)

target_include_directories(main PUBLIC
    libmicroros/include
    ${CMAKE_CURRENT_LIST_DIR}/wireless_communication
    ${PICO_SDK_PATH}/lib/lwip/src/include
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)
target_compile_definitions(main PRIVATE LWIP_TIMEVAL_PRIVATE=0)
