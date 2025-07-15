cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_multi_robot C CXX ASM)
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

# Create a executable for each robot
function(add_robot robot_id)
    set(exec_name "robot${robot_id}")
    add_executable(${exec_name}
        src/main.cpp
        src/motor_control/Motor.cpp
        src/line_follow/InfraredSensor.cpp
        src/obstacle_detection/UltrasonicSensor.cpp
        src/display/st7789.c
        src/wireless_communication/picow_udp_transports.c
    )
    # SPI PIO program
    pico_generate_pio_header(${exec_name} ${CMAKE_CURRENT_LIST_DIR}/src/line_follow/spi.pio)

    target_compile_definitions(${exec_name} PRIVATE
        LWIP_TIMEVAL_PRIVATE=0
        ROBOT_ID=${robot_id}
    )

    target_link_libraries(${exec_name}
        pico_stdlib
        microros
        hardware_pwm
        pico_cyw43_arch_lwip_poll
        hardware_spi
    )

    target_include_directories(${exec_name} PUBLIC
        libmicroros/include
        ${CMAKE_CURRENT_LIST_DIR}/src/wireless_communication
        ${PICO_SDK_PATH}/lib/lwip/src/include
    )

    pico_enable_stdio_usb(${exec_name} 1)
    pico_enable_stdio_uart(${exec_name} 0)
    pico_add_extra_outputs(${exec_name})
endfunction()

# Create the file for 2 robots
add_robot(1)
add_robot(2)