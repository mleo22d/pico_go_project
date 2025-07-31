#include <array>
#include <tuple>
#include "hardware/pio.h"

#define NUM_SENSORS 5

using namespace std;

class InfraredSensor {
private:
    array<uint16_t, NUM_SENSORS> _calibrated_min;
    array<uint16_t, NUM_SENSORS> _calibrated_max;
    PIO _pio;
    uint _sm;
    uint _successive_not_on_line = 0;
    uint _max_fails = 20;
    uint16_t _last_value = 0;

public:
    const uint buffer_size;
    const uint clk_pin;
    const uint mosi_pin;
    const uint miso_pin;
    const uint cs_pin;

    InfraredSensor(const uint buffer_size = 8,
              const uint clk_pin = 6,
              const uint mosi_pin = 7,
              const uint miso_pin = 27,
              const uint cs_pin = 28);

    ~InfraredSensor();

    array<uint16_t, NUM_SENSORS> analog_read();
    void calibrate();
    void fixed_calibration();

    const array<uint16_t, NUM_SENSORS>& get_calibrated_min();
    const array<uint16_t, NUM_SENSORS>& get_calibrated_max();
    const array<uint16_t, NUM_SENSORS> read_calibrated();
    
    tuple<uint16_t, array<uint16_t, NUM_SENSORS>> read_line(bool white_line = false);
};
