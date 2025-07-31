#include "display_manager.h"
#include <cstdio>


void DisplayManager::startup_sequence(int robot_id) {
    st7789_fill(BLACK);
    draw_text_rainbow(60, 30, "PICO ROBOT");
    sleep_ms(1000);

    char id_text[32];
    snprintf(id_text, sizeof(id_text), "Robot ID: %d", robot_id);
    draw_text(70, 50, id_text, WHITE);
    sleep_ms(500);

    for (int i = 0; i < 100; i += 10) {
        draw_progress_bar(50, 80, 140, 10, i / 100.0f, GRAY, GREEN);
        draw_text(95, 100, "Loading...", YELLOW);
        sleep_ms(100);
    }

    sleep_ms(500);
    st7789_fill(BLACK);
}

void DisplayManager::show_calibration_status() {
    st7789_fill(BLACK);
    draw_text(60, 55, "CALIBRATING", WHITE);
    draw_text(50, 70, "SENSORS...", WHITE);

    for (int i = 0; i < 8; i++) {
        int x = 120 + 30 * cos(i * M_PI / 4);
        int y = 140 + 30 * sin(i * M_PI / 4);
        draw_circle(x, y, 3, ORANGE);
        sleep_ms(100);
    }
}

void DisplayManager::show_calibration_complete() {
    st7789_fill(BLACK);
    draw_text(50, 55, "CALIBRATION", GREEN);
    draw_text(70, 70, "COMPLETE!", GREEN);

    for (int r = 5; r < 50; r += 5) {
        draw_circle(120, 120, r, GREEN);
        sleep_ms(50);
    }
    sleep_ms(1000);
}

void DisplayManager::show_ros_connecting() {
    st7789_fill(BLACK);
    draw_text(40, 55, "CONNECTING TO", WHITE);
    draw_text(60, 70, "ROS AGENT", WHITE);
}

void DisplayManager::update_connection_progress(float progress) {
    draw_progress_bar(50, 90, 140, 15, progress, GRAY, BLUE);

    char percent_text[16];
    snprintf(percent_text, sizeof(percent_text), "%d%%", (int)(progress * 100));
    draw_text(110, 110, percent_text, WHITE);
}

void DisplayManager::show_connection_success() {
    st7789_fill(BLACK);
    draw_text(50, 55, "CONNECTED!", GREEN);
    draw_text(80, 70, "READY", GREEN);
    sleep_ms(1500);
}

void DisplayManager::show_connection_failed() {
    st7789_fill(BLACK);
    draw_text(30, 55, "CONNECTION", RED);
    draw_text(60, 70, "FAILED!", RED);
    sleep_ms(2000);
}

void DisplayManager::show_operational_status(bool line_follow_mode, int robot_id) {
    st7789_fill(BLACK);

    if (line_follow_mode) {
        draw_text(60, 15, "LINE FOLLOWING", YELLOW);
    } else {
        draw_text(75, 15, "STANDBY", CYAN);
    }

    char id_char[2] = {static_cast<char>(robot_id + '0'), '\0'};

    int digit_width = 8 * 4;
    int digit_height = 8 * 4;
    int x_pos = (LCD_WIDTH - digit_width) / 2;
    int y_pos = 50;

    draw_large_digit(x_pos, y_pos, id_char[0], MAGENTA, 4);
    draw_rect(x_pos - 2, y_pos - 2, digit_width + 4, digit_height + 4, WHITE);
}

void DisplayManager::update_line_follow_active() {
    if (absolute_time_diff_us(last_blink, get_absolute_time()) > 500000) {
        blink_state = !blink_state;
        last_blink = get_absolute_time();

        uint16_t color = blink_state ? YELLOW : BLACK;
        draw_rect(180, 20, 50, 15, color);
        if (blink_state) {
            draw_text(185, 25, "ACTIVE", BLACK);
        }
    }
}
