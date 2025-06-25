cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
set(PICO_BOARD pico_w)
set(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND 1)


pico_sdk_init()
set(PICO_LWIP_NO_SYS 0)
add_compile_definitions(
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
    PICO_LWIP_NO_SYS=0
)

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
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/lwip_config
    ${CMAKE_CURRENT_SOURCE_DIR}/lwip_config/arch
    ${CMAKE_CURRENT_LIST_DIR}/lwip_config
    #${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/src/boards/include
    ${PICO_SDK_PATH}/src/common/pico_base/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include/arch/threadsafe_background
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include/arch/threadsafe_background
    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_rand/include
    ${PICO_SDK_PATH}/src/rp2_common/pico_async_context/include
    ${PICO_SDK_PATH}/lib/cyw43-driver/src
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(main)
