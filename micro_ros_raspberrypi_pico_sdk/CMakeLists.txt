cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_BOARD pico_w)
set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)

pico_sdk_init()

link_directories(libmicroros)

add_executable(main
    main.cpp
    pico_uart_transport.c
    pico_wifi_transport.c
)

target_link_libraries(main
    pico_stdlib
    microros
    hardware_pwm
    pico_cyw43_arch_lwip_threadsafe_background
)

target_include_directories(main PUBLIC    
    libmicroros/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
    ${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/lib/lwip/src/include/compat
    ${PICO_SDK_PATH}/lib/lwip/src/include/compat/posix
    ${CMAKE_BINARY_DIR}/generated/pico_sdk/src/common/pico_lwip/include

)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

pico_add_extra_outputs(main)


add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(main)