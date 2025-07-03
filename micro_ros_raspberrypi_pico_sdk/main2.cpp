#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include "hardware/pwm.h"
#include "pico/stdlib.h"
//#include "pico_uart_transports.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/async_context.h"
#include "wireless_communication/picow_udp_transports.h"

// Pines
const uint LED_PIN = 25;
const uint TRIG_PIN = 14;
const uint ECHO_PIN = 15;
const uint PWMA = 16;
const uint AIN1 = 18;
const uint AIN2 = 17;
const uint PWMB = 21;
const uint BIN1 = 19;
const uint BIN2 = 20;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 sub_msg;
absolute_time_t last_cmd_time; 

// Setup sensor ultrasónico
void setup_ultrasonic() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

// Medición de distancia
float read_distance_cm() {
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    absolute_time_t start = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 0) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000) return -1;
    }
    absolute_time_t signal_start = get_absolute_time();

    while (gpio_get(ECHO_PIN) == 1) {
        if (absolute_time_diff_us(signal_start, get_absolute_time()) > 30000) return -1;
    }
    absolute_time_t signal_end = get_absolute_time();

    int64_t duration = absolute_time_diff_us(signal_start, signal_end);
    return duration * 0.0343f / 2.0f;
}

// Configurar motores
void setup_motors() {
    gpio_init(AIN1); gpio_set_dir(AIN1, GPIO_OUT);
    gpio_init(AIN2); gpio_set_dir(AIN2, GPIO_OUT);
    gpio_init(BIN1); gpio_set_dir(BIN1, GPIO_OUT);
    gpio_init(BIN2); gpio_set_dir(BIN2, GPIO_OUT);

    gpio_set_function(PWMA, GPIO_FUNC_PWM);
    gpio_set_function(PWMB, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(PWMA);
    uint slice_b = pwm_gpio_to_slice_num(PWMB);
    pwm_set_wrap(slice_a, 65535);
    pwm_set_wrap(slice_b, 65535);
    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
}

// Control de velocidad PWM
void set_speed(uint pin, int speed_percent) {
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), speed_percent * 65535 / 100);
}

// Funciones de movimiento
void forward(int speed) {
    gpio_put(AIN1, 0); gpio_put(AIN2, 1);
    gpio_put(BIN1, 0); gpio_put(BIN2, 1);
    set_speed(PWMA, speed);
    set_speed(PWMB, speed);
}

void stop() {
    gpio_put(AIN1, 0); gpio_put(AIN2, 0);
    gpio_put(BIN1, 0); gpio_put(BIN2, 0);
    set_speed(PWMA, 0);
    set_speed(PWMB, 0);
}

// Callback de suscripción
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    last_cmd_time = get_absolute_time();
    if (msg->data == 1) {
        forward(15);
    } else {
        stop();
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer == NULL) return;

    float dist = read_distance_cm();
    printf("Distancia: %.2f cm\n", dist);

    if (dist > 5.0f && dist < 100.0f) {
        msg.data = 1;  // seguir
    } else {
        msg.data = 0;  // detener
    }

    rcl_publish(&publisher, &msg, NULL);
}


int main()
{

    stdio_init_all();
    ST_PICOW_TRANSPORT_PARAMS picow_params = {0};


    /*
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    */
    // VIA WIFI UDP

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

    rmw_uros_set_custom_transport(
        false,          
        &picow_params,
        picow_udp_transport_open,
        picow_udp_transport_close,
        picow_udp_transport_write,
        picow_udp_transport_read
    );

    // CONECTION COMPLETE

    setup_ultrasonic();
    setup_motors();

    // Esperar conexión con el agente
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) return 1;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // CREATE THE PUBLISHER
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "obstacle_distance"
    );

    // CREATE THE SUBSCRIBER
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "motor_control"
    );

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback);
    
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, subscription_callback, ON_NEW_DATA);

    msg.data = 0;
    last_cmd_time = get_absolute_time();
    while (true)
    {
        cyw43_arch_poll();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        // Timeout de seguridad: si no recibes comandos en 5 segundos, detener
        if (absolute_time_diff_us(last_cmd_time, get_absolute_time()) > 5e6) {
            stop();
        }
    }

    return 0;
}
