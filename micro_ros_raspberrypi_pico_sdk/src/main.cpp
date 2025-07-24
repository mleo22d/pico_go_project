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
#include <std_msgs/msg/string.h>

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

#define MAX_ROUTE_LENGTH 100
static char route_data_buffer[MAX_ROUTE_LENGTH];  // buffer persistente

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
bool line_lost = false;

// Subscriber for navigation (route)
rcl_subscription_t route_subscriber;
//std_msgs__msg__Bool route_msg;
std_msgs__msg__String route_msg;
std::vector<char> route = {'R','R','R','R'};
size_t route_step = 0;

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
float d = 0.1   ;
//float d = 5.38625;

static absolute_time_t last_time = get_absolute_time();
absolute_time_t last_cmd_time; 
ST_PICOW_TRANSPORT_PARAMS picow_params;

void sleep_ms_non_blocking(uint32_t delay_ms) {
    absolute_time_t start = get_absolute_time();
    while (absolute_time_diff_us(start, get_absolute_time()) < delay_ms * 1000) {
        cyw43_arch_poll();
        tight_loop_contents(); 
    }
}

void line_follow() {

    printf("entro a la funcion\n");
    if (route_step >= route.size()) {
        line_follow_mode = false;  // Reset path to repeat
        motors.stop();
        return;
    }

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
            sleep_ms_non_blocking(100);
            gpio_put(BUZZER_PIN, 0);

            if (route_step < route.size()) {
                int turn_speed = 25;
                int time_sleep = 425;
                //sleep_ms_non_blocking(250);  // tune
                switch (route[route_step]) {
                    case 'L':
                        motors.left(maximum);
                        sleep_ms_non_blocking(time_sleep);
                        while (true) {
                            cyw43_arch_poll();
                            auto [pos, sensors] = irSensor.read_line();
                            if (sensors[0] > 400 && 
                                (sensors[2] < 150 || sensors[1] < 150 || sensors[3] < 150) && 
                                sensors[4] > 400) {
                                break;  // aligned with the line
                            }
                        }
                        //motors.stop();
                        break;   
                    case 'R':
                        motors.right(turn_speed);
                        sleep_ms_non_blocking(time_sleep);
                        while (true) {
                            cyw43_arch_poll();
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
                        sleep_ms_non_blocking(time_sleep*(1.0/2.0));  // tune
                        break;
                }
                route_step++;
            }

            current_state = FOLLOWING_LINE;
            break;

        case DROPPING_PACKAGE:
            // open_gate(); // add gate control logic here
            gpio_init(BUZZER_PIN);
            gpio_set_dir(BUZZER_PIN, GPIO_OUT);
            gpio_put(BUZZER_PIN, 1);
            sleep_ms_non_blocking(50);
            gpio_put(BUZZER_PIN, 0);
            current_state = FOLLOWING_LINE;
            break;
    }
}

// callback for controlling the pico with keyboard
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
    printf("CALLBACK RECIV LF \n");
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    //line_follow_mode = msg->data;
    line_follow_mode = true;
    gpio_init(BUZZER_PIN);    
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 1);
    sleep_ms_non_blocking(50);
    gpio_put(BUZZER_PIN, 0);
    /*
    if (!line_follow_mode) {
        //printf("[PICO] Modo seguidor de línea DESACTIVADO\n");
        motors.stop();
    }*/
}

void obstacle_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
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

void line_follow_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    
    (void) last_call_time;
    if (!line_follow_mode || timer == NULL) return;
    line_follow();
}

void route_callback(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
 
    route.clear();
    for (size_t i = 0; i < msg->data.size; ++i) {
        char c = msg->data.data[i];
        if (c == 'L' || c == 'R' || c == 'S') {
            route.push_back(c);
        }
    }
    route_step = 0;
    string route_str(route.begin(), route.end());
    printf("Ruta recibida: %s\n", route_str.c_str());
    current_state = FOLLOWING_LINE; 
    line_follow_mode = true;  
    route_step = 0;
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

int main()
{
    stdio_init_all();
     
  /*
    while (!stdio_usb_connected()) { 

        sleep_ms_non_blocking(100);
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

    while (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        printf("Esperando al agente micro-ROS...\n");
        sleep_ms_non_blocking(1000);
    }
    printf("Conectado con el agente.\n");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    string node_name = "pico_robot_" + to_string(ROBOT_ID);
    rclc_node_init_default(&node, node_name.c_str(), "", &support);

    // Create the obstacle alert Publisher
    string obstacle_alert_topic = "/robot_" + to_string(ROBOT_ID) + "/obstacle_alert";
    rclc_publisher_init_default(
        &obstacle_alert_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        obstacle_alert_topic.c_str()
    );

    // Create de velocity control subs for keyboard
    string sub_topic = "/robot_" + to_string(ROBOT_ID) + "/cmd_vel";
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        sub_topic.c_str()
    );

    // Create Line Follow Subscriber (to enable o disable the mode)
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
    
    std_msgs__msg__String__init(&route_msg);
    route_msg.data.data = route_data_buffer;
    route_msg.data.size = 0;
    route_msg.data.capacity = MAX_ROUTE_LENGTH;

    // Create subs for asign the route
    string route_topic = "/robot_" + to_string(ROBOT_ID) + "/route_assign";
    rclc_subscription_init_default(
    &route_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    route_topic.c_str()
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

    // timer for the obstacle detection
    rcl_timer_t obstacle_timer;
    rclc_timer_init_default(&obstacle_timer, &support, RCL_MS_TO_NS(10), obstacle_timer_callback);
    
    // timer for the line follow mode, its executing unless i send false in the line_folow_subs
    rcl_timer_t line_follow_timer;
    rclc_timer_init_default(&line_follow_timer, &support, RCL_MS_TO_NS(10), line_follow_timer_callback); // cada 10ms

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    //rclc_executor_add_timer(&executor, &obstacle_timer);
    rclc_executor_add_timer(&executor, &line_follow_timer);
    //rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, cmd_vel_subscription_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &route_subscriber, &route_msg, route_callback, ON_NEW_DATA);
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
    }
}