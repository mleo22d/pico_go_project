#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <string>  
#include <numeric>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32_multi_array.h>


// WIFI
#include "wireless_communication/picow_udp_transports.h"

// HARDWARE
#include <geometry_msgs/msg/twist.h>
#include "motor_control/Motor.h"
#include "line_follow/InfraredSensor.h"
#include "obstacle_detection/UltrasonicSensor.h"
#include "display/st7789.h"
#include <algorithm>


using namespace std;

#ifndef ROBOT_ID
#define ROBOT_ID 1  
#endif


// Pines
#define BUZZER_PIN 4
const uint LED_PIN = 25;

// Obstacle alert
rcl_publisher_t obstacle_alert_publisher;
std_msgs__msg__Bool obstacle_alert_msg;

// cmd_vel
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

// start line follow mode
rcl_subscription_t line_follow_subscriber;
std_msgs__msg__Bool line_follow_msg;
bool line_follow_mode = false;
absolute_time_t line_lost_start = nil_time;
bool line_lost = false;

// values publisher (DEBUG)
rcl_publisher_t ir_values_publisher;
std_msgs__msg__Int32MultiArray ir_values_msg;

// Components
Motor motors; 
InfraredSensor irSensor;
UltrasonicSensor utSensor;

// Navigation
enum State { FOLLOWING_LINE, INTERSECTION, DROPPING_PACKAGE };
State current_state = FOLLOWING_LINE;
static int route_step = 0;
static const char route[] = {'R','R','R','R'};  // example path

// pid controller
int16_t integral = 0;
int16_t proportional = 0;
int16_t derivative = 0;
int16_t last_proportional = 0;
int16_t power_diff = 0;
int16_t max_integral = 5000;
int maximum = 20;
float p = 0.05;
float i = 0.0;
//float i = 0.000637;0.2
float d = 0.15;
//float d = 5.38625;

static absolute_time_t last_time = get_absolute_time();
absolute_time_t last_cmd_time; 
ST_PICOW_TRANSPORT_PARAMS picow_params;

void buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(slice, 4.0f);  // reloj más rápido para audio
    pwm_set_wrap(slice, 62500);   // frecuencia base
    pwm_set_enabled(slice, true);
}

void buzzer_play_tone(int frequency, int duration_ms) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint16_t wrap = 125000000 / 4 / frequency;  // calcula el wrap para la frecuencia
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(BUZZER_PIN), wrap / 2);  // 50% duty cycle
    sleep_ms(duration_ms);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(BUZZER_PIN), 0);  // apaga el sonido
}

// Callback de suscripción
void cmd_vel_subscription_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
    int linear_vel = int((msg->linear.x)*100);
    int angular_vel = int((msg->angular.z)*100);

    if(linear_vel > 0) {
        //printf("Forward");
        motors.forward(linear_vel);
    } else if(linear_vel < 0) {
        //printf("Reverse");
        motors.reverse(-linear_vel);
    } else if(angular_vel < 0) {
        //printf("right");
        motors.right(-angular_vel);
    } else if(angular_vel > 0) {
        //printf("left");
        motors.left(angular_vel);
    } else {
        //printf("Stop");
        motors.stop();
    }
}

void line_follow_callback(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    line_follow_mode = msg->data;

    if (!line_follow_mode) {
        //printf("[PICO] Modo seguidor de línea DESACTIVADO\n");
        motors.stop();
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer == NULL) return;

    float dist = utSensor.read_distance_cm();
    //printf("Distancia: %.2f cm\n", dist);

    if (dist > 10.0f) {
        obstacle_alert_msg.data = false;
    } else {
        obstacle_alert_msg.data = true;  // stop
    }

    rcl_publish(&obstacle_alert_publisher, &obstacle_alert_msg, NULL);
}

void wifi_start() {
    
    // START WIFI CONNECTION

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        std::exit(0);
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        std::exit(0);
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
}

void line_follow() {


    uint16_t position;
    array<uint16_t, NUM_SENSORS> sensor_values;
    tie(position, sensor_values) = irSensor.read_line();

    //for (auto v : sensor_values) printf("%d ", v);
    //printf("\n");

    for (int i = 0; i < 5; ++i) {
        ir_values_msg.data.data[i] = sensor_values[i];
    }
    rcl_publish(&ir_values_publisher, &ir_values_msg, NULL);


    uint16_t sensor_sum = accumulate(sensor_values.begin(), sensor_values.end(), 0);

    // Check if all sensors see black (intersection)
    int black_count = count_if(sensor_values.begin(), sensor_values.end(), [](int v) { return v < 150; });
    bool all_black = black_count >= 4;

    // Check if center sensors detect black (delivery marker)
    bool center_bar = sensor_values[0] > 400 && sensor_values[4] > 400 &&
                      sensor_values[1] < 150 && sensor_values[2] < 150 && sensor_values[3] < 150;
    bool all_white = all_of(sensor_values.begin(), sensor_values.end(), [](int v) { return v > 400; });

    switch (current_state) {
        case FOLLOWING_LINE:
            if (all_white) {
                motors.stop();
                return;
            } else if (all_black) {
                motors.stop();
                current_state = INTERSECTION;
                return;
            } else if (center_bar) {
                motors.stop();
                current_state = DROPPING_PACKAGE;
                return;
            } else {
                proportional = position - 3000;

                int16_t derivative = proportional - last_proportional;
                integral += proportional;

                if (integral > max_integral) integral = max_integral;
                if (integral < -max_integral) integral = -max_integral;
        
                last_proportional = proportional;

                int power_diff = proportional * p + derivative * d + integral * i;

                if (power_diff > maximum) power_diff = maximum;
                if (power_diff < -maximum) power_diff = -maximum;
                
                if (power_diff > 0)
                    motors.set_motor(maximum - power_diff, maximum);
                else
                    motors.set_motor(maximum, maximum + power_diff);
            }
            break;

        case INTERSECTION:
            gpio_init(BUZZER_PIN);
            gpio_set_dir(BUZZER_PIN, GPIO_OUT);
            gpio_put(BUZZER_PIN, 1);
            sleep_ms(250);
            gpio_put(BUZZER_PIN, 0);

            if (route_step < sizeof(route)) {
                int turn_speed = 25;
                int time_sleep = 300;
                sleep_ms(250);  // tune
                switch (route[route_step]) {
                    case 'L':
                        motors.left(maximum);
                        sleep_ms(time_sleep);  // tune
                        break;   
                    case 'R':
                        motors.right(turn_speed);
                        sleep_ms(time_sleep);
                        while (true) {
                            auto [pos, sensors] = irSensor.read_line();
                            if (sensors[0] > 400 && 
                                (sensors[2] < 150 || sensors[1] < 150 || sensors[3] < 150) && 
                                sensors[4] > 400) {
                                break;  // aligned with the line
                            }
                        }
                        motors.stop();
                        break;
                    case 'S':
                        motors.forward(maximum);
                        sleep_ms(time_sleep);  // tune
                        break;
                }
                route_step++;
                if (route_step >= sizeof(route)) {
                    route_step = 0;  // Reset path to repeat
                }
            }

            current_state = FOLLOWING_LINE;
            break;

        case DROPPING_PACKAGE:
            motors.stop();
            // open_gate(); // add gate control logic here
            motors.stop();
            gpio_init(BUZZER_PIN);
            gpio_set_dir(BUZZER_PIN, GPIO_OUT);
            gpio_put(BUZZER_PIN, 1);
            sleep_ms(50);
            gpio_put(BUZZER_PIN, 0);
            sleep_ms(2000);
            gpio_init(BUZZER_PIN);
            gpio_set_dir(BUZZER_PIN, GPIO_OUT);
            gpio_put(BUZZER_PIN, 1);
            sleep_ms(50);
            current_state = FOLLOWING_LINE;
            break;
    }
}


/*
void line_follow() {

    // Read the position and the IR sensors
    uint16_t position;
    array<uint16_t, NUM_SENSORS> sensor_values;
    tie(position, sensor_values) = irSensor.read_line();

    //printf("[DEBUG] IR = ");
    for (auto v : sensor_values) printf("%d ", v);
    printf("\n");

    //printf("\n");

    auto calibrated = irSensor.read_calibrated();
    printf("Calibrated: ");
    for (auto v : calibrated) printf("%d ", v);
    printf("\n");


    uint16_t sensor_sum = accumulate(sensor_values.begin(), sensor_values.end(), 0);
    //printf("[DEBUG] position = %d\n", position);
    //printf("[DEBUG] sensor_sum = %d\n", sensor_sum);
    
    // Just for security, its supossed that sensor_sum should be maximun 5000
    if (sensor_sum < 200) {
        motors.stop();
        //Make a sound and decide
    }
    else if (sensor_sum < 2500) {
        motors.stop();
    }
    else if (sensor_sum < 5000) {
        // the proportional should be 0 if the line is in the center
        proportional = position - 3000;

        derivative = proportional - last_proportional;
        integral += proportional;

        const int16_t max_integral = 5000;
        if (integral > max_integral) integral = max_integral;
        if (integral < -max_integral) integral = -max_integral;

        last_proportional = proportional;

        power_diff = proportional * p + derivative * d + integral * i;
        //printf("[DEBUG] PD = %d, P = %d, D = %d, I = %d\n", power_diff, proportional, derivative, integral);

        if (power_diff > maximum) power_diff = maximum;
        if (power_diff < -maximum) power_diff = -maximum;

        if (power_diff > 0)
            motors.set_motor(maximum - power_diff, maximum);
        else
            motors.set_motor(maximum, maximum + power_diff);
        
    // Emergencia solo si sensor_sum muy alto
        if (sensor_sum > 3500) {
            if (!line_lost) {
                line_lost_start = get_absolute_time();  // solo una vez
                line_lost = true;
            } else if (absolute_time_diff_us(line_lost_start, get_absolute_time()) > 1e6) {
                motors.stop();  // lleva más de 2 segundos fuera
            }
        } else {
            line_lost = false;  // se corrigió a tiempo
        }
    }
    else {
        motors.stop();
    }
}
*/

int main()
{
   
    stdio_init_all();
     
  /*
    while (!stdio_usb_connected()) { 

        sleep_ms(100);
    }
*/
    //ST_PICOW_TRANSPORT_PARAMS picow_params = {0};
    memset(&picow_params, 0, sizeof(picow_params));
    
    wifi_start();

    irSensor.fixed_calibration();
    /* 
    
    irSensor.calibrate();
    auto min_vals = irSensor.get_calibrated_min();
    auto max_vals = irSensor.get_calibrated_max();

    
    printf("Valores de calibración:\n");

    printf("Min: {");
    for (int i = 0; i < NUM_SENSORS; ++i) {
        printf("%d%s", min_vals[i], (i < NUM_SENSORS - 1) ? ", " : "}\n");
    }

    printf("Max: {");
    for (int i = 0; i < NUM_SENSORS; ++i) {
        printf("%d%s", max_vals[i], (i < NUM_SENSORS - 1) ? ", " : "}\n");
    }
    */

    // WAIT CONNECTION WITH THE AGENT
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    //if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) return 1;

    while (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        //printf("Esperando al agente micro-ROS...\n");
        sleep_ms(1000);
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    string node_name = "pico_robot_" + to_string(ROBOT_ID);
    rclc_node_init_default(&node, node_name.c_str(), "", &support);
    //rclc_node_init_default(&node, "pico_node", "", &support);

    // Create the obstacle alert Publisher
    string obstacle_alert_topic = "/robot_" + to_string(ROBOT_ID) + "/obstacle_alert";
    rclc_publisher_init_default(
        &obstacle_alert_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        obstacle_alert_topic.c_str()
    );

    // Create de velocity control
    string sub_topic = "/robot_" + to_string(ROBOT_ID) + "/cmd_vel";
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        sub_topic.c_str()
    );

    // Create Line Follow Subscriber
    string line_follow_topic = "/robot_" + to_string(ROBOT_ID) + "/line_follow_start";
    rclc_subscription_init_default(
        &line_follow_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        line_follow_topic.c_str()
    );

    // Create the values publisher IR (DEBUG)
    string ir_values_topic = "/robot_" + to_string(ROBOT_ID) + "/ir_values";
    rclc_publisher_init_default(
        &ir_values_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        ir_values_topic.c_str()
    );

    // Inicializar layout y datos
    ir_values_msg.layout.dim.capacity = 1;
    ir_values_msg.layout.dim.size = 1;
    ir_values_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(sizeof(std_msgs__msg__MultiArrayDimension));
    ir_values_msg.layout.dim.data[0].label.data = (char *)"ir_array";
    ir_values_msg.layout.dim.data[0].label.size = 8;
    ir_values_msg.layout.dim.data[0].label.capacity = 8;
    ir_values_msg.layout.dim.data[0].size = 5;
    ir_values_msg.layout.dim.data[0].stride = 5;

    ir_values_msg.data.capacity = 5;
    ir_values_msg.data.size = 5;
    ir_values_msg.data.data = (int32_t*)malloc(5 * sizeof(int32_t));

    // Agregar

    rcl_timer_t timer;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback);
    
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    //rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, cmd_vel_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &line_follow_subscriber, &line_follow_msg, line_follow_callback, ON_NEW_DATA);

    last_cmd_time = get_absolute_time();

    while (true)
    {
        cyw43_arch_poll();
        rcl_ret_t exec_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

        // Timeout de seguridad: si no recibes comandos en 5 segundos, detener
        //if (absolute_time_diff_us(last_cmd_time, get_absolute_time()) > 5e6) {
        //   stop();  // SIN comunicación en 5 segundos → detiene
        //} 
        if (line_follow_mode) {
            line_follow();
        } else {
            motors.stop();  
        }
    }
}