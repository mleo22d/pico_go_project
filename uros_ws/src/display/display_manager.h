#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "st7789.h"
#include "font.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include <cmath>
#include <cstdio>

class DisplayManager {
public:
    void startup_sequence(int robot_id);
    void show_calibration_status();
    void show_calibration_complete();
    void show_ros_connecting();
    void update_connection_progress(float progress);
    void show_connection_success();
    void show_connection_failed();
    void show_operational_status(bool line_follow_mode, int robot_id);
    void update_line_follow_active();

private:
    absolute_time_t last_blink = get_absolute_time();
    bool blink_state = false;
};

#endif