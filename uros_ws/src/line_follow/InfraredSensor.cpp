#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "InfraredSensor.h"
#include "spi.pio.h"
#include <cstdio>


InfraredSensor::InfraredSensor(const uint buffer_size, const uint clk, const uint mosi, const uint miso, const uint cs)
    : buffer_size(buffer_size), clk_pin(clk), mosi_pin(mosi), miso_pin(miso), cs_pin(cs) {

    _pio = pio0;
    uint offset = pio_add_program(_pio, &spi_cpha0_program);
    _sm = pio_claim_unused_sm(_pio, true);

    pio_spi_init(_pio, _sm, offset,
                 12, 156.25f / 2.f, false, false,
                 clk_pin, mosi_pin, miso_pin);

    bi_decl(bi_4pins_with_names(miso_pin, "SPI RX", mosi_pin, "SPI TX", clk_pin, "SPI SCK", cs_pin, "SPI CS"));

    gpio_init(cs_pin);
    gpio_put(cs_pin, 1);
    gpio_set_dir(cs_pin, GPIO_OUT);
}

InfraredSensor::~InfraredSensor() {}

array<uint16_t, NUM_SENSORS> InfraredSensor::analog_read() {
    array<uint16_t, NUM_SENSORS + 1> raw{};
    for (int i = 0; i < NUM_SENSORS + 1; ++i) {
        gpio_put(cs_pin, 0);
        pio_sm_put_blocking(_pio, _sm, i << 28);
        raw[i] = pio_sm_get_blocking(_pio, _sm) & 0xfff;
        gpio_put(cs_pin, 1);
        raw[i] >>= 2;
        busy_wait_us(50);
    }
    array<uint16_t, NUM_SENSORS> result;
    copy(raw.begin() + 1, raw.end(), result.begin());
    return result;
}

const array<uint16_t, NUM_SENSORS>& InfraredSensor::get_calibrated_min() {
    return _calibrated_min;
}

const array<uint16_t, NUM_SENSORS>& InfraredSensor::get_calibrated_max() {
    return _calibrated_max;
}

void InfraredSensor::fixed_calibration() {
    //floor
     /*
    _calibrated_min = {79, 54, 112, 49, 97};
    _calibrated_max = {607, 670, 781, 489, 810};
    */
    //table
    _calibrated_min = {105, 102, 123, 98, 137};
    _calibrated_max = {772, 851, 937, 601, 938};
    /*
    _calibrated_min = {141, 154, 146, 144, 122};
    _calibrated_max = {949, 948, 935, 933, 829};*/
}

void InfraredSensor::calibrate() {
    _calibrated_min.fill(1023);  
    _calibrated_max.fill(0);

    absolute_time_t start = get_absolute_time();
    const int duration_ms = 10000;  //

    while (absolute_time_diff_us(start, get_absolute_time()) < duration_ms * 1000) {
        auto sensor_values = analog_read();
        for (size_t i = 0; i < NUM_SENSORS; ++i) {
            if (_calibrated_max[i] < sensor_values[i] && sensor_values[i] != 0)
                _calibrated_max[i] = sensor_values[i];
            if (_calibrated_min[i] > sensor_values[i] && sensor_values[i] != 0)
                _calibrated_min[i] = sensor_values[i];
        }

        printf("Raw: ");
        for (auto v : sensor_values) printf("%d ", v);
        printf("\n");

        sleep_ms(100);  // un poco de pausa
    }
   
}

const array<uint16_t, NUM_SENSORS> InfraredSensor::read_calibrated() {
    auto raw = analog_read();
    for (int i = 0; i < NUM_SENSORS; ++i) {
        int denom = _calibrated_max[i] - _calibrated_min[i];
        int value = 0;
        if (denom != 0)
            value = (raw[i] - _calibrated_min[i]) * 1000 / denom;
        
        if (value < 0) value = 0;
        if (value > 1000) value = 1000;
        raw[i] = static_cast<uint16_t>(value);
    }
    return raw;
}

tuple<uint16_t, array<uint16_t, NUM_SENSORS>> InfraredSensor::read_line(bool white_line) {
    auto sensor_values = read_calibrated();
    double avg = 0;
    double sum1 = 0;
    bool on_line = false;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        uint16_t value = sensor_values[i];
        if (white_line)
            value = 1000 - value;

        if (value < 800) on_line = true;
        if (value > 50) {
            avg += value * ((i + 1) * 1000);
            sum1 += value;
        }
    }

    if (on_line) {
        _successive_not_on_line = 0;
    } else {
        _successive_not_on_line++;
        if (_successive_not_on_line >= _max_fails)
            _successive_not_on_line = _max_fails;
    }

    if (_successive_not_on_line >= _max_fails) {
        _last_value = (_last_value < 3050) ? 2500 : 3500;
    }

    if (on_line && sum1 != 0) {
        _last_value = static_cast<uint16_t>(avg / sum1);
    }

    return std::make_tuple(_last_value, sensor_values);
}




